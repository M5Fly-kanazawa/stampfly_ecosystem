#!/usr/bin/env python3
"""Venue-disturbance (NT Kanazawa) validation of ti_hover shortening.

Re-uses the identification/replay method of
analysis/scripts/poshold_alt_loop_sim.py (kp/ti PI cascade, thrust actuation
lag TAU=0.11s, hover thrust HOVER=M*G*HOVER_CORR) but re-identifies the
vertical disturbance d_ext(t) from the two 2026-06-27 DAYTIME venue logs
(POS_HOLD, 3.5-3.8V) instead of the night low-battery log, and tests whether
shortening ti_hover (2.5 -> 1.5 -> 1.0) still reduces the altitude bob when the
disturbance is the much larger / broadband venue one.

CRITICAL DIFFERENCE FROM THE NIGHT LOG: the venue logs have heavy UDP
telemetry loss from WiFi congestion (up to ~25% of samples missing in one of
the two logs), with individual gaps up to ~0.5s (100+ gap events over ~140s).
Both the kinematic identification (numerical differentiation for az) and the
closed-loop PID replay (which carries integrator state) are run PER
CONTIGUOUS VALID SEGMENT ONLY -- never straight-line-interpolated or
integrated across a gap. See build_gap_mask()/find_valid_segments().
夜間ログとの決定的な違い: 会場ログはWiFi輻輳によるUDPテレメトリ欠損が大きく
(1本は最大で全体の約25%が欠損)、個々のギャップは最大0.5s程度・100件以上に及ぶ。
加速度の数値微分による同定と、積分状態(PID)を持つ閉ループ再生は、いずれもギャップを
またいで実行しない。有効な連続区間ごとに独立して行い、区間をまたぐ直線補間・積分は
一切行わない。

Usage: python3 alt_ti_venue_sim.py
"""
import sys
import numpy as np
from scipy.signal import welch

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
from yawlib import load_jsonl  # noqa: E402

# ---- plant / cascade parameters (identical to poshold_alt_loop_sim.py) ----
M, G, HOVER_CORR = 0.037, 9.80665, 1.12
HOVER = M * G * HOVER_CORR
TAU = 0.11              # actuation lag [s]
FS = 100.0               # resample grid [Hz]
GAP_THR = 0.05           # native-sample spacing beyond this = a telemetry gap [s]
MIN_SEG = 2.0            # discard valid segments shorter than this [s]
V_LIM = 0.15             # velocity-loop thrust-correction clamp (firmware default)

LOGS = [
    ("venue_165713", "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T165713.jsonl", (56.3, 196.5)),
    ("venue_164611", "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T164611.jsonl", (33.1, 199.6)),
]
# NOTE: poshold_alt_loop_sim.py's WIN=(108.0,178.0) is defined in a REBASED time
# frame (t0 = first posvel record's raw ts), NOT raw device-boot-time ts. The venue
# windows above ARE raw ts (verified against detect_flight_segments -- see analysis
# notes). To reproduce the reference numbers in STEP 0 we must rebase the same way.
# 注記: poshold_alt_loop_sim.py の WIN=(108.0,178.0) は「最初のposvelレコードのraw ts」
# を原点とした相対時間フレームであり、rawのデバイス起動基準tsではない。上記の会場ログの
# windowはraw tsそのもの(detect_flight_segmentsで裏取り済み)。STEP 0の基準値再現には
# 同じrebase規約を使う必要がある。
NIGHT_PATH = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T020137.jsonl"
NIGHT_WIN_REBASED = (108.0, 178.0)


# ---------------------------------------------------------------- utilities
def smooth(x, w):
    w = int(w)
    if w < 3:
        return x.copy()
    if w % 2 == 0:
        w += 1
    if w >= len(x):
        w = len(x) - 1 if len(x) % 2 == 0 else len(x)
        if w < 3:
            return x.copy()
    return np.convolve(x, np.ones(w) / w, mode='same')


def lpf(x, tau, fs):
    a = np.exp(-1 / (tau * fs))
    y = np.zeros_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = a * y[i - 1] + (1 - a) * x[i]
    return y


class PID:
    """pid.hpp-equivalent: standard form, incomplete derivative ON MEASUREMENT
    (Tustin, eta=0.125), trapezoidal integral with conditional anti-windup.
    td=0 -> pure PI (firmware altitude default). Verbatim from
    poshold_alt_loop_sim.py."""

    def __init__(s, kp, ti, lim, td=0.0, eta=0.125):
        s.kp, s.ti, s.td, s.lim, s.eta = kp, ti, td, lim, eta
        s.I = 0.0
        s.pe = 0.0
        s.df = 0.0
        s.pm = 0.0
        s.first = True

    def step(s, sp, meas, dt):
        e = sp - meas
        P = s.kp * e
        D = 0.0
        if s.td > 0:
            if s.first:
                s.pm = meas
            else:
                al = 2 * s.eta * s.td / dt
                a = (al - 1) / (al + 1)
                b = 2 * s.td / ((al + 1) * dt)
                s.df = a * s.df - b * (meas - s.pm)
                D = s.kp * s.df
        s.pm = meas
        s.first = False
        if s.ti >= 0.01:
            inext = s.I + (s.kp / s.ti) * (e + s.pe) * dt * 0.5
            ot = P + inext + D
            if not ((ot > s.lim and e > 0) or (ot < -s.lim and e < 0)):
                s.I = inext
            s.I = float(np.clip(s.I, -s.lim, s.lim))
        s.pe = e
        return float(np.clip(P + s.I + D, -s.lim, s.lim))


# ---------------------------------------------------------- gap-aware split
def build_gap_mask(win, time_arrays, gap_thr=GAP_THR, fs=FS):
    """tg: uniform grid over win. invalid[i] True if ANY of time_arrays has a
    native gap > gap_thr straddling tg[i], or tg[i] is outside that array's
    covered span."""
    tg = np.arange(win[0], win[1], 1 / fs)
    invalid = np.zeros(len(tg), dtype=bool)
    for t_native in time_arrays:
        tw = np.sort(t_native)
        m = (tw >= win[0] - 5) & (tw <= win[1] + 5)
        tw = tw[m]
        if len(tw) < 2:
            invalid[:] = True
            continue
        dt = np.diff(tw)
        idx = np.where(dt > gap_thr)[0]
        for i in idx:
            invalid |= (tg > tw[i]) & (tg < tw[i + 1])
        invalid |= (tg < tw[0]) | (tg > tw[-1])
    return tg, invalid


def find_valid_segments(tg, invalid, min_len=MIN_SEG, fs=FS):
    valid = ~invalid
    if not np.any(valid):
        return []
    idx = np.where(valid)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    segs = []
    min_n = int(min_len * fs)
    for s, e in zip(starts, ends):
        if e - s + 1 >= min_n:
            segs.append((s, e))
    return segs


# ------------------------------------------------------------- log loading
def load_log(path, win):
    d = load_jsonl(path)
    tpv = d['posvel']['ts']
    pos = d['posvel']['pos']
    vel = d['posvel']['vel']
    alt = -pos[:, 2]
    vz = -vel[:, 2]
    tcr = d['ctrl_ref']['ts']
    thr = d['ctrl_ref']['total_thrust']
    tim = d['imu']['ts']
    q = d['imu']['quat']
    cosT = 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)

    tg, invalid = build_gap_mask(win, [tpv, tcr, tim])
    segs = find_valid_segments(tg, invalid)

    altg = np.interp(tg, tpv, alt)
    vzg = np.interp(tg, tpv, vz)
    thrg = np.interp(tg, tcr, thr)
    cosg = np.interp(tg, tim, cosT)

    # global hold-altitude reference: median of altitude over ALL valid samples
    # (alt_sp is 0 in these logs -- telemetry not wired -- per task instructions)
    valid_mask = ~invalid
    sp = float(np.median(altg[valid_mask])) if np.any(valid_mask) else float(np.median(altg))

    return dict(tg=tg, invalid=invalid, segs=segs, altg=altg, vzg=vzg, thrg=thrg,
                cosg=cosg, sp=sp, win=win, n_total=len(tg))


# ------------------------------------------------------------- d_ext per-seg
def dext_for_segment(L, s, e):
    tg_seg = L['tg'][s:e + 1]
    vzs = L['vzg'][s:e + 1]
    thrs = L['thrg'][s:e + 1]
    coss = L['cosg'][s:e + 1]
    n = e - s + 1
    w = min(30, max(5, n // 8))
    az = smooth(np.gradient(smooth(vzs, w), 1 / FS), w)
    thr_lag = lpf(thrs, TAU, FS)
    d_ext = M * az - (thr_lag * coss - M * G)
    nw = max(3, int(FS / 1.5))
    noise = vzs - smooth(vzs, nw)
    return tg_seg, d_ext, noise


def identify_all(L):
    """Concatenate d_ext / noise over all valid segments (never across a gap)."""
    d_all, n_all, t_all, dur = [], [], [], 0.0
    per_seg = []
    for s, e in L['segs']:
        tg_seg, d_ext, noise = dext_for_segment(L, s, e)
        d_all.append(d_ext)
        n_all.append(noise)
        t_all.append(tg_seg)
        dur += tg_seg[-1] - tg_seg[0]
        per_seg.append((tg_seg, d_ext, noise))
    if not d_all:
        return None
    return dict(t=np.concatenate(t_all), d_ext=np.concatenate(d_all),
                noise=np.concatenate(n_all), coverage_s=dur, per_seg=per_seg)


def band_power_welch(per_seg, fs=FS, frac=0.8):
    """Duration-weighted-average PSD across per-segment Welch estimates, computed
    SEPARATELY per contiguous segment and never on the gap-concatenated series --
    concatenating discontinuous segments before an FFT/Welch would inject
    broadband spectral-leakage energy from the segment-boundary jumps, biasing
    the identified band toward higher frequency. Then report the peak-centered
    band containing `frac` of the AVERAGED spectrum's power (0.02-2Hz range).
    区間をまたいで連結してからWelchにかけると、区間境界の不連続がFFTに漏れ込み
    (スペクトルリーケージ)、同定される帯域が高域側に偏る。よって区間ごとに個別に
    PSDを計算し、区間長で重み付け平均してから帯域を求める。"""
    f_common = None
    Pxx_sum = None
    dur_sum = 0.0
    for tg_seg, d_ext, _noise in per_seg:
        n = len(d_ext)
        if n < 8:
            continue
        nper = min(n, max(8, int(fs * 10)))
        f, Pxx = welch(d_ext, fs=fs, nperseg=nper)
        dur = n / fs
        if f_common is None:
            f_common = f
        Pxx_i = np.interp(f_common, f, Pxx)
        Pxx_sum = Pxx_i * dur if Pxx_sum is None else Pxx_sum + Pxx_i * dur
        dur_sum += dur
    if Pxx_sum is None or dur_sum == 0:
        return None, None, (np.nan, np.nan)
    Pxx_avg = Pxx_sum / dur_sum
    band = (f_common >= 0.02) & (f_common <= 2.0)
    f, Pxx = f_common[band], Pxx_avg[band]
    if len(f) == 0 or Pxx.sum() == 0:
        return f, Pxx, (np.nan, np.nan)
    order = np.argsort(-Pxx)
    cum = np.cumsum(Pxx[order])
    k = np.searchsorted(cum, frac * Pxx.sum()) + 1
    sel = np.sort(order[:k])
    return f, Pxx, (f[sel[0]], f[sel[-1]])


# --------------------------------------------------------------- replay
def replay_segment(L, s, e, d_ext, akp, ati, vkp, vti, v_lim=V_LIM):
    """Closed-loop cascade replay over ONE valid segment. State (PID
    integrators, al/vc) is (re)initialized at the segment start from the
    MEASURED altitude/velocity -- no state carried across a gap."""
    tg_seg = L['tg'][s:e + 1]
    n = len(tg_seg)
    dt = 1 / FS
    a_ = PID(akp, ati, 0.5)
    v_ = PID(vkp, vti, v_lim)
    al = np.zeros(n)
    vc = np.zeros(n)
    corr_hist = np.zeros(n)
    al[0] = L['altg'][s]
    vc[0] = L['vzg'][s]
    thr_state = HOVER
    ac = np.exp(-1 / (TAU * FS))
    cosg_seg = L['cosg'][s:e + 1]
    # noise is recomputed locally to match dext_for_segment's own smoothing
    _, _, noise_seg = dext_for_segment(L, s, e)
    sp = L['sp']
    for i in range(1, n):
        vz_meas = vc[i - 1] + noise_seg[i]
        vsp = a_.step(sp, al[i - 1], dt)
        corr = v_.step(vsp, vz_meas, dt)
        corr_hist[i] = corr
        tcmd = float(np.clip(HOVER + corr, 0, 0.6))
        thr_state = ac * thr_state + (1 - ac) * tcmd
        azc = (thr_state * cosg_seg[i] - M * G + d_ext[i]) / M
        vc[i] = vc[i - 1] + azc * dt
        al[i] = al[i - 1] + vc[i] * dt
    return al, vc, corr_hist


def replay_all(L, akp, ati, vkp, vti, v_lim=V_LIM):
    al_all, corr_all, dur = [], [], 0.0
    for s, e in L['segs']:
        _, d_ext, _ = dext_for_segment(L, s, e)
        al, vc, corr = replay_segment(L, s, e, d_ext, akp, ati, vkp, vti, v_lim=v_lim)
        al_all.append(al)
        corr_all.append(corr)
        dur += (e - s) / FS
    al_all = np.concatenate(al_all)
    corr_all = np.concatenate(corr_all)
    return al_all, corr_all, dur


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def p2p(x):
    return float(np.max(x) - np.min(x))


# =================================================================== MAIN
def main():
    print("=" * 100)
    print("STEP 0: sanity check on the NIGHT log (should reproduce poshold_alt_loop_sim.py's own numbers)")
    print("=" * 100)
    d0 = load_jsonl(NIGHT_PATH)
    t0_night = d0['posvel']['ts'][0]
    night_win_raw = (NIGHT_WIN_REBASED[0] + t0_night, NIGHT_WIN_REBASED[1] + t0_night)
    print(f"night t0(rebase)={t0_night:.3f}s -> raw-ts window = ({night_win_raw[0]:.2f},{night_win_raw[1]:.2f})")
    Ln = load_log(NIGHT_PATH, night_win_raw)
    print(f"night log: n_segments={len(Ln['segs'])}, coverage={sum(e - s for s, e in Ln['segs']) / FS:.1f}s "
          f"/ window={night_win_raw[1] - night_win_raw[0]:.1f}s")
    idn = identify_all(Ln)
    f, Pxx, (blo, bhi) = band_power_welch(idn['per_seg'])
    print(f"night d_ext std = {np.std(idn['d_ext']) * 1000:.1f} mN  (reference: 4.5->8.5 mN range)")
    print(f"night d_ext dominant band (80% power) = {blo:.2f}-{bhi:.2f} Hz  (reference: 0.07-0.23 Hz)")
    al_cur, corr_cur, _ = replay_all(Ln, 0.45, 7.0, 0.1, 2.5)
    meas_alt = Ln['altg'][~Ln['invalid']]
    print(f"night measured alt std = {np.std(meas_alt) * 1000:.0f} mm   "
          f"replay(ti=2.5) std = {np.std(al_cur) * 1000:.0f} mm   "
          f"(reference: measured 87mm, replay 80mm)")

    print()
    print("=" * 100)
    print("STEP 1-5: VENUE logs")
    print("=" * 100)

    results = {}
    for name, path, win in LOGS:
        print(f"\n{'-'*100}\n{name}  window={win}\n{'-'*100}")
        L = load_log(path, win)
        total_valid_s = sum((e - s) / FS for s, e in L['segs'])
        window_s = win[1] - win[0]
        print(f"n_segments={len(L['segs'])}  valid coverage={total_valid_s:.1f}s / {window_s:.1f}s "
              f"({100*total_valid_s/window_s:.0f}%)  hold-alt sp={L['sp']:.3f} m")
        seg_lens = [(e - s) / FS for s, e in L['segs']]
        if seg_lens:
            print(f"  segment length: min={min(seg_lens):.1f}s max={max(seg_lens):.1f}s "
                  f"median={np.median(seg_lens):.1f}s")

        idr = identify_all(L)
        f, Pxx, (blo, bhi) = band_power_welch(idr['per_seg'])
        d_rms = rms(idr['d_ext'])
        print(f"  d_ext RMS = {d_rms*1000:.1f} mN   dominant band(80%) = {blo:.2f}-{bhi:.2f} Hz")

        # measured alt RMS/p2p over valid samples only
        meas_alt = L['altg'][~L['invalid']]
        meas_rms = rms(meas_alt - L['sp'])
        meas_p2p = p2p(meas_alt)
        print(f"  measured alt: RMS={meas_rms*1000:.1f} mm  p2p={meas_p2p*1000:.1f} mm")

        cases = {
            "ti=2.5 (current)": dict(akp=0.45, ati=7.0, vkp=0.1, vti=2.5, v_lim=V_LIM),
            "ti=1.5": dict(akp=0.45, ati=7.0, vkp=0.1, vti=1.5, v_lim=V_LIM),
            "ti=1.0": dict(akp=0.45, ati=7.0, vkp=0.1, vti=1.0, v_lim=V_LIM),
            "ti=1.0 + clamp0.20": dict(akp=0.45, ati=7.0, vkp=0.1, vti=1.0, v_lim=0.20),
            "ti=1.5 + clamp0.20": dict(akp=0.45, ati=7.0, vkp=0.1, vti=1.5, v_lim=0.20),
        }
        case_res = {}
        base_rms = None
        for cname, g in cases.items():
            al_r, corr_r, dur = replay_all(L, g['akp'], g['ati'], g['vkp'], g['vti'], v_lim=g['v_lim'])
            r_rms = rms(al_r - L['sp'])
            r_p2p = p2p(al_r)
            sat_frac = float(np.mean(np.abs(corr_r) >= 0.99 * g['v_lim']))
            if base_rms is None:
                base_rms = r_rms
            case_res[cname] = dict(rms_mm=r_rms * 1000, p2p_mm=r_p2p * 1000,
                                    sat_pct=sat_frac * 100, delta_pct=(r_rms / base_rms - 1) * 100)
        results[name] = dict(L=L, meas_rms_mm=meas_rms * 1000, meas_p2p_mm=meas_p2p * 1000,
                              d_ext_rms_mN=d_rms * 1000, band=(blo, bhi), cases=case_res,
                              coverage_pct=100 * total_valid_s / window_s)

        print(f"\n  {'case':24}{'RMS(mm)':>10}{'p2p(mm)':>10}{'clamp sat %':>13}{'vs ti2.5':>10}")
        for cname, r in case_res.items():
            print(f"  {cname:24}{r['rms_mm']:>10.1f}{r['p2p_mm']:>10.1f}{r['sat_pct']:>13.1f}"
                  f"{r['delta_pct']:>9.1f}%")
        print(f"  measured (for reference): RMS={meas_rms*1000:.1f}mm p2p={meas_p2p*1000:.1f}mm "
              f"-> replay(ti2.5)/measured RMS ratio = {case_res['ti=2.5 (current)']['rms_mm']/(meas_rms*1000):.2f}")

    return results, idn, Ln


if __name__ == '__main__':
    main()
