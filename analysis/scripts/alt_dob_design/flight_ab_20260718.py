#!/usr/bin/env python3
"""flight_ab_20260718.py — First-flight diagnostic for the altitude disturbance
observer (DOB), altitude.dob.fc=1.5, flown 2026-07-18.
高度外乱オブザーバ(DOB)初有効化フライトの診断（2026-07-18飛行、altitude.dob.fc=1.5）

Pilot report / パイロット所見
-----------------------------
"全般的にハイゲインな感じでカクカク動く（不自然）。しかし今までで最も良い。
ハイゲインで抑え込んでいるが時々入力が飽和しているのでは。" AC (air conditioner)
was switched off near t=70s (log-relative ts, same timebase as every other
channel here — NOT relative to takeoff).

Five questions this script answers (see module-level SECTION comments):
  1. Flight window / mode / hands-off-ness / voltage / AC-off changepoint
  2. d_hat reconstruction (bit-faithful port of computeDobCorrection(), see
     firmware/vehicle/components/sf_controller_pid/pid_controller.cpp) +
     saturation quantification (tests the pilot's "ときどき飽和" hypothesis)
  3. "カクカク" frequency identification: PSD overlays vs. the DOB-less
     baseline flight, d_hat/thrust coherence (is the DOB the driving source?),
     clamp-event <-> thrust-jerk timing coincidence (limit-cycle check)
  4. Performance quantification vs. baseline (164-171mm memory reference),
     split by AC on/off
  5. Improvement-candidate replay sweep (reuses step2_dob_design.py's
     closed-loop simulator, d_ext_acc reconstructed from THIS flight) —
     fc/clamp variants, alt std vs. thrust-jerkiness trade-off table

このスクリプトが答える5つの問い（モジュール内 SECTION コメント参照）:
  1. 飛行窓・モード・手放し度・電圧・エアコンOFF切替点
  2. d_hat のオフライン再構成（computeDobCorrection() のビット忠実移植）＋
     飽和の定量化（パイロットの「時々飽和」仮説の検証）
  3. 「カクカク」の周波数同定: DOBなしベースラインとのPSD重ね合わせ、
     d_hat/推力コヒーレンス（DOBが駆動源か）、クランプイベントと推力の
     カクカクの時間一致（リミットサイクル的挙動の確認）
  4. ベースライン（メモリ参照値164-171mm）比の性能定量、エアコンON/OFF別
  5. 改善candidateの再生シム（step2_dob_design.py の閉ループシミュレータを
     再利用、本フライトから再構成した d_ext_acc を注入）— fc/クランプの
     組み合わせで alt std とカクカク度のトレードオフ表

Usage / 使い方
--------------
    python3 analysis/scripts/alt_dob_design/flight_ab_20260718.py

No arguments. Outputs:
    analysis/scripts/alt_dob_design/ab20260718_results.json
    analysis/scripts/alt_dob_design/figs/ab20260718_*.png
Prints a human-readable summary to stdout. Does not commit anything.
引数なし。上記に出力。標準出力に要約を表示。コミットは行わない。
"""

import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import welch, coherence as scipy_coherence, butter, filtfilt
from scipy.ndimage import median_filter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "analysis" / "scripts" / "yaw_nt_kanazawa"))
sys.path.insert(0, str(REPO / "analysis" / "scripts"))
import step1_actuation_id as s1        # noqa: E402  (log load / f_up / gap-mask / zoh reuse)
import step2_dob_design as s2          # noqa: E402  (closed-loop replay simulator reuse)
from yawlib import load_jsonl, detect_flight_segments, mode_str  # noqa: E402

FIGS_DIR = HERE / "figs"
FIGS_DIR.mkdir(parents=True, exist_ok=True)
RESULTS_PATH = HERE / "ab20260718_results.json"

LOGS_DIR = REPO / "logs"
LOG_DOB = LOGS_DIR / "stampfly_udp_20260718T022929.jsonl"     # altitude.dob.fc=1.5 flight (target)
LOG_BASE = LOGS_DIR / "stampfly_udp_20260717T231940.jsonl"    # no-DOB, hands-off POS_HOLD 137s (== step1/2's LOG_A)

# =============================================================================
# Physical / DOB constants — copied verbatim from firmware source so this
# reconstruction is bit-faithful, NOT a re-derivation. Cross-checked against:
#   firmware/vehicle/components/sf_controller_pid/pid_controller.cpp
#     (computeDobQCoeffs, dobBiquad, dobWashout, computeDobCorrection)
#   firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp
#     (kDob* constants)
# 物理・DOB定数 — ファームウェアソースからそのまま複写（再導出ではなくビット
# 忠実な移植のため）。上記2ファイルと照合済み。
# =============================================================================
M = 0.037                        # airframe mass [kg]
G = 9.80665                      # standard gravity [m/s^2]
HOVER_CORR = 1.12                # hover.thrust_corr default (used only to SEED the
HOVER = M * G * HOVER_CORR       # DOB states at Airborne entry; see resetDobStates())

DOB_RATE_HZ = 400.0              # kDobRateHz — nominal control rate the Q/washout
DOB_DT = 1.0 / DOB_RATE_HZ       # coefficients are fixed at (NOT re-derived per-cycle
                                  # from measured dt; only the actuation model uses dt)
DOB_MODEL_LAG_S = 0.02           # kDobModelLagS — motor time constant
DOB_DELAY_SAMPLES = 25           # kDobDelaySamples = round(0.0624*400)
DOB_MIN_FUP = 2.0                # kDobMinFupMs2 — specific-force validity guard
DOB_PRIME_CYCLES = 100           # kDobPrimeCycles — 0.25s @ 400Hz
DOB_ENGAGE_RAMP_CYCLES = 800     # kDobEngageRampCycles — 2.0s @ 400Hz
DOB_WASHOUT_HZ = 0.03            # kDobWashoutHz
DOB_WASHOUT_TAU = 1.0 / (2.0 * np.pi * DOB_WASHOUT_HZ)
DOB_WASHOUT_ALPHA = DOB_WASHOUT_TAU / (DOB_WASHOUT_TAU + DOB_DT)         # kDobWashoutAlpha
DOB_WASHOUT_FAST_TAU = 0.5       # kDobWashoutFastTauS
DOB_WASHOUT_FAST_ALPHA = DOB_WASHOUT_FAST_TAU / (DOB_WASHOUT_FAST_TAU + DOB_DT)  # kDobWashoutFastAlpha
DOB_CLAMP_N = 0.10               # kDobClampN
DOB_FC_FLOWN = 1.5               # altitude.dob.fc as flown (task brief)
DOB_SAT_THRESHOLD_N = 0.0995     # "near-clamp" reporting threshold (task brief)

VLIM_SAT_THRESHOLD_N = 0.145     # near-clamp threshold for the PI corr estimate (VLIM=0.15)
MOTOR_DUTY_SAT_THRESHOLD = 0.95

EDGE_TRIM_S = 3.0                # margin excluded around window edges / engage transient
                                  # (matches step1/step2's own discipline)


# =============================================================================
# SECTION 0 — small shared helpers
# 共通ヘルパ
# =============================================================================
def welch_psd(t, x, nperseg_s=20.0):
    """Welch PSD at the signal's own (approximately uniform) native rate."""
    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    nper = min(len(x), max(16, int(nperseg_s * fs)))
    f, Pxx = welch(np.asarray(x, dtype=float), fs=fs, nperseg=nper)
    return f, Pxx, fs


def band_rms_welch(t, x, flo, fhi, nperseg_s=50.0):
    f, Pxx, _ = welch_psd(t, x - np.mean(x), nperseg_s=nperseg_s)
    band = (f >= flo) & (f <= fhi)
    if band.sum() < 2:
        return float("nan")
    return float(np.sqrt(np.trapezoid(Pxx[band], f[band])))


def hover_trend(t, x, win_s=20.0):
    """Slow-trend estimate via a rolling median (approximates hover_thrust_'s
    slow drift so a PI-correction estimate can be backed out — see §2)."""
    dt = float(np.median(np.diff(t)))
    win_n = max(3, int(round(win_s / dt)))
    if win_n % 2 == 0:
        win_n += 1
    return median_filter(x, size=win_n, mode="nearest")


# =============================================================================
# SECTION 1 — flight window, mode, hands-off-ness, voltage, AC changepoint
# 飛行窓・モード・手放し度・電圧・エアコン切替点
# =============================================================================
def flight_conditions(path, label):
    d = load_jsonl(str(path))
    cr = d["ctrl_ref"]
    win_raw = s1.detect_flight_window(cr["ts"], cr["motor_duty"])

    st = d["status"]
    m_win = (st["ts"] >= win_raw[0]) & (st["ts"] <= win_raw[1])
    flying_mask = (st["flight_state"] == 5) & (st["ts"] >= win_raw[0])
    t_airborne = float(st["ts"][flying_mask][0]) if np.any(flying_mask) else win_raw[0]

    ctrl = d["ctrl"]
    m_ctrl = (ctrl["ts"] >= win_raw[0]) & (ctrl["ts"] <= win_raw[1])
    stick_std = {k: float(np.nanstd(ctrl[k][m_ctrl])) for k in ("roll", "pitch", "yaw", "throttle")}
    stick_maxabs = {k: float(np.nanmax(np.abs(ctrl[k][m_ctrl]))) for k in ("roll", "pitch", "yaw", "throttle")}

    m_mode = (cr["ts"] >= win_raw[0]) & (cr["ts"] <= win_raw[1])
    modes = np.unique(cr["mode"][m_mode])

    info = dict(
        label=label, path=str(path),
        flight_window_raw_s=[float(win_raw[0]), float(win_raw[1])],
        flight_duration_s=float(win_raw[1] - win_raw[0]),
        t_airborne_s=t_airborne,
        modes_seen=[int(x) for x in modes],
        mode_names=[mode_str(x) for x in modes],
        voltage_min_V=float(np.nanmin(st["voltage"][m_win])) if m_win.any() else float("nan"),
        voltage_max_V=float(np.nanmax(st["voltage"][m_win])) if m_win.any() else float("nan"),
        stick_std=stick_std, stick_maxabs=stick_maxabs,
    )
    return info, d, win_raw, t_airborne


def sliding_band_power(t, x, flo, fhi, win_s=10.0, step_s=2.0, order=4):
    """NOTE: uses SOS (second-order sections), not the transfer-function (b,a)
    form — at high native rates (e.g. imu's 400Hz) a narrow low-frequency
    band (e.g. 0.2-2Hz) makes the (b,a) polynomial coefficients ill-
    conditioned and filtfilt silently blows up to inf (verified: reproduced
    with f_up at 400Hz). SOS + sosfiltfilt is numerically robust regardless
    of how narrow the band is relative to Nyquist.
    (b,a)形でなくSOS(2次セクション)形を使う — imuの400Hzのような高いネイティブ
    レートで0.2-2Hzのような狭帯域を指定すると(b,a)多項式係数が悪条件化し
    filtfiltが無音でinfに発散する（f_up@400Hzで実際に再現・確認済み）。
    SOS+sosfiltfiltはNyquist比でどれだけ狭い帯域でも数値的に頑健。"""
    from scipy.signal import sosfiltfilt
    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    sos = butter(order, [flo / (fs / 2.0), fhi / (fs / 2.0)], btype="band", output="sos")
    xf = sosfiltfilt(sos, x)
    win_n = int(win_s * fs)
    step_n = max(1, int(step_s * fs))
    n = len(t)
    centers, power = [], []
    for i0 in range(0, n - win_n, step_n):
        seg = xf[i0:i0 + win_n]
        centers.append(t[i0] + win_s / 2.0)
        power.append(float(np.mean(seg ** 2)))
    return np.array(centers), np.array(power)


def detect_ac_changepoint(t, x, search_lo, search_hi, flo=0.2, fhi=2.0):
    """Largest step-like change (10s-median before vs. after) in band power,
    searched over [search_lo, search_hi]. Returns (t_change, relative_jump)."""
    centers, power = sliding_band_power(t, x, flo, fhi)
    m = (centers >= search_lo) & (centers <= search_hi)
    best_t, best_ratio = None, 0.0
    for tc in centers[m]:
        before = power[(centers >= tc - 10) & (centers < tc)]
        after = power[(centers >= tc) & (centers < tc + 10)]
        if len(before) < 2 or len(after) < 2:
            continue
        pb, pa = float(np.median(before)), float(np.median(after))
        ratio = abs(pa - pb) / (pb + 1e-12)
        if ratio > best_ratio:
            best_ratio, best_t = ratio, float(tc)
    return best_t, best_ratio, centers, power


def local_step_test(centers, power, t0, half_win=10.0):
    """Falsification-style DIRECT test at a claimed changepoint t0 (rather
    than the global argmax search above): median band power in
    [t0-half_win,t0) vs [t0,t0+half_win). Used to check the pilot's
    "~70s" claim on its own terms even if it is not the global maximum.
    反証形式の直接検定: 申告時刻t0そのものについて前後の中央値を比較する
    （上のグローバル最大探索とは独立に、70sの主張そのものを検定する）。"""
    before = power[(centers >= t0 - half_win) & (centers < t0)]
    after = power[(centers >= t0) & (centers < t0 + half_win)]
    if len(before) < 2 or len(after) < 2:
        return None
    pb, pa = float(np.median(before)), float(np.median(after))
    return dict(t0=t0, power_before=pb, power_after=pa, ratio=pa / (pb + 1e-12))


# =============================================================================
# SECTION 2 — d_hat offline reconstruction (bit-faithful port)
# d_hat のオフライン再構成（ビット忠実移植）
# =============================================================================
def compute_dob_q_coeffs(fc, fs=DOB_RATE_HZ):
    """RBJ Audio EQ Cookbook 2nd-order Butterworth LPF, Q=1/sqrt(2) —
    verbatim port of PidController::computeDobQCoeffs()."""
    w0 = 2.0 * np.pi * fc / fs
    cosw0 = np.cos(w0)
    Qb = 0.70710678
    alpha = np.sin(w0) / (2.0 * Qb)
    a0 = 1.0 + alpha
    b0 = ((1.0 - cosw0) * 0.5) / a0
    b1 = (1.0 - cosw0) / a0
    b2 = b0
    a1 = (-2.0 * cosw0) / a0
    a2 = (1.0 - alpha) / a0
    return b0, b1, b2, a1, a2


def build_recon_grid(d, t0, t1, fs=DOB_RATE_HZ):
    """Uniform fs-Hz grid (linear-interp quat/accel, ZOH thrust — firmware
    holds the 50Hz command) spanning [t0, t1]. Using a UNIFORM grid (rather
    than raw imu timestamps) keeps the 25-sample delay ring's wall-clock
    meaning exact across this log's rare telemetry gaps (max 85ms, see
    report); it also matches kDobRateHz=400 exactly, which is what the fixed
    (not per-cycle-recomputed) Q/washout coefficients assume.
    一様fs-Hzグリッド。生imuタイムスタンプでなく一様グリッドを使うことで、25サンプル
    遅延リングの実時間的な意味を稀なテレメトリギャップ（最大85ms）をまたいでも正確に
    保つ。kDobRateHz=400と厳密一致（Q/ウォッシュアウト係数は毎サイクル再計算でなく
    固定のため）。"""
    imu = d["imu"]
    tim = imu["ts"]
    accel_corr = s1.bias_corrected_accel(imu)
    f_up_native, cos_native = s1.body_accel_to_f_up(accel_corr, imu["quat"])
    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]
    tg = np.arange(t0, t1, 1.0 / fs)
    fupg = np.interp(tg, tim, f_up_native)
    cosg = np.interp(tg, tim, cos_native)
    thrg = s1.zoh_interp(tg, tcr, thr)
    return tg, fupg, cosg, thrg


def reconstruct_dhat(tg, fupg, cosg, thrg, fc=DOB_FC_FLOWN, hover=HOVER):
    """Sample-by-sample port of computeDobCorrection() + the delay-ring push
    in compute() (pid_controller.cpp:659-676). Runs from tg[0] = Airborne
    entry (resetDobStates(hover_thrust_) seeds the ring/model to `hover`)."""
    n = len(tg)
    b0, b1, b2, a1, a2 = compute_dob_q_coeffs(fc)

    delay_ring = np.full(DOB_DELAY_SAMPLES, hover, dtype=float)
    delay_idx = 0
    model_state = hover
    q_w1 = q_w2 = 0.0
    wo_x_prev = wo_y_prev = 0.0
    prime_count = 0
    prime_accum = 0.0
    engage_count = 0
    d_hat_prev = 0.0
    model_alpha = 1.0 - np.exp(-DOB_DT / DOB_MODEL_LAG_S)   # fixed dt=0.0025 (task brief)

    d_hat = np.zeros(n)
    residual = np.full(n, np.nan)
    guard = np.zeros(n, dtype=bool)
    prime_stage = np.zeros(n, dtype=bool)
    engage_stage = np.zeros(n, dtype=bool)
    raw_clamp_hit = np.zeros(n, dtype=bool)   # |d_raw|>clamp BEFORE the engage ramp scales it down

    for i in range(n):
        delayed_u = delay_ring[delay_idx]
        model_state = model_state + model_alpha * (delayed_u - model_state)

        if fupg[i] < DOB_MIN_FUP:
            guard[i] = True
            d_hat[i] = d_hat_prev
        else:
            res = M * fupg[i] - model_state * cosg[i]
            residual[i] = res
            if prime_count < DOB_PRIME_CYCLES:
                prime_accum += res
                prime_count += 1
                prime_stage[i] = True
                if prime_count == DOB_PRIME_CYCLES:
                    res_avg = prime_accum / DOB_PRIME_CYCLES
                    w_ss = res_avg / (1.0 + a1 + a2)
                    q_w1 = w_ss
                    q_w2 = w_ss
                    wo_x_prev = res_avg
                    wo_y_prev = 0.0
                d_hat[i] = 0.0
            else:
                engaging = engage_count < DOB_ENGAGE_RAMP_CYCLES
                wo_alpha = DOB_WASHOUT_FAST_ALPHA if engaging else DOB_WASHOUT_ALPHA
                w = res - a1 * q_w1 - a2 * q_w2
                q_out = b0 * w + b1 * q_w1 + b2 * q_w2
                q_w2, q_w1 = q_w1, w
                d_raw = wo_alpha * (wo_y_prev + q_out - wo_x_prev)
                wo_x_prev, wo_y_prev = q_out, d_raw
                raw_clamp_hit[i] = abs(d_raw) > DOB_CLAMP_N
                ramp = 1.0
                if engaging:
                    ramp = engage_count / DOB_ENGAGE_RAMP_CYCLES
                    engage_count += 1
                    engage_stage[i] = True
                d_hat[i] = ramp * float(np.clip(d_raw, -DOB_CLAMP_N, DOB_CLAMP_N))
        d_hat_prev = d_hat[i]

        # Delay-ring push — ALWAYS, every cycle, with the final commanded
        # thrust (compute() lines 673-676; happens regardless of the f_up
        # guard/prime/engage state above).
        delay_ring[delay_idx] = thrg[i]
        delay_idx = (delay_idx + 1) % DOB_DELAY_SAMPLES

    return dict(t=tg, d_hat=d_hat, residual=residual, guard=guard,
                prime_stage=prime_stage, engage_stage=engage_stage,
                raw_clamp_hit=raw_clamp_hit)


def saturation_stats(t, d_hat, thr=DOB_SAT_THRESHOLD_N):
    mask = np.abs(d_hat) >= thr
    frac_pct = float(np.mean(mask)) * 100.0
    dt = float(np.median(np.diff(t)))
    events = []
    n = len(mask)
    i = 0
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            events.append((float(t[i]), float(t[j - 1]), (j - i) * dt))
            i = j
        else:
            i += 1
    durations = np.array([e[2] for e in events]) if events else np.array([])
    return dict(
        saturation_frac_pct=frac_pct,
        n_events=len(events),
        duration_mean_s=float(np.mean(durations)) if len(durations) else 0.0,
        duration_median_s=float(np.median(durations)) if len(durations) else 0.0,
        duration_p90_s=float(np.percentile(durations, 90)) if len(durations) else 0.0,
        duration_max_s=float(np.max(durations)) if len(durations) else 0.0,
        events_sample=[(round(a, 2), round(b, 2), round(c, 3)) for a, b, c in events[:30]],
        all_events=[(round(a, 2), round(b, 2), round(c, 3)) for a, b, c in events],
    )


# =============================================================================
# SECTION 2b — PI-correction saturation estimate + motor duty saturation
# PI補正の飽和推定＋motor duty飽和
# =============================================================================
def corr_estimate(d, dhat_result, t0, t1):
    cr = d["ctrl_ref"]
    m = (cr["ts"] >= t0) & (cr["ts"] <= t1)
    t_cr = cr["ts"][m]
    thr_cr = cr["total_thrust"][m]
    dhat_on_cr = np.interp(t_cr, dhat_result["t"], dhat_result["d_hat"])
    trend = hover_trend(t_cr, thr_cr + dhat_on_cr, win_s=20.0)
    corr_est = thr_cr - trend + dhat_on_cr
    return dict(t=t_cr, thrust=thr_cr, d_hat=dhat_on_cr, hover_trend=trend, corr_est=corr_est)


def motor_duty_saturation(d, t0, t1, thr=MOTOR_DUTY_SAT_THRESHOLD):
    cr = d["ctrl_ref"]
    m = (cr["ts"] >= t0) & (cr["ts"] <= t1)
    duty = cr["motor_duty"][m]
    any_sat = np.any(duty >= thr, axis=1)
    per_motor = [float(np.mean(duty[:, k] >= thr) * 100.0) for k in range(duty.shape[1])]
    return dict(any_motor_saturated_pct=float(np.mean(any_sat) * 100.0),
                per_motor_pct=per_motor, duty_max=float(np.max(duty)))


# =============================================================================
# SECTION 3 — frequency identification ("カクカク" source)
# 周波数同定（「カクカク」の駆動源）
# =============================================================================
def gyro_corrected(imu):
    return imu["gyro"] - imu["gyro_bias"]


def clamp_thrust_correlation(sat_events, t_cr, thr_cr, margin_s=0.5):
    dthr = np.abs(np.diff(thr_cr))
    dt_cr = t_cr[1:]
    baseline = float(np.median(dthr)) if len(dthr) else float("nan")
    if len(t_cr) < 3 or not sat_events:
        return dict(n_events_checked=0, near_event_max_dthr_median_mN=float("nan"),
                    baseline_dthr_median_mN=baseline * 1000.0 if baseline == baseline else float("nan"),
                    ratio=float("nan"))
    near_vals = []
    for (a, b, _dur) in sat_events:
        m = (dt_cr >= a - margin_s) & (dt_cr <= b + margin_s)
        if m.any():
            near_vals.append(float(np.max(dthr[m])))
    if not near_vals:
        return dict(n_events_checked=0, near_event_max_dthr_median_mN=float("nan"),
                    baseline_dthr_median_mN=baseline * 1000.0 if baseline == baseline else float("nan"),
                    ratio=float("nan"))
    near_arr = np.array(near_vals)
    return dict(
        n_events_checked=len(near_vals),
        near_event_max_dthr_median_mN=float(np.median(near_arr)) * 1000.0,
        baseline_dthr_median_mN=baseline * 1000.0 if baseline == baseline else float("nan"),
        ratio=float(np.median(near_arr) / baseline) if baseline == baseline and baseline > 0 else float("nan"),
    )


# =============================================================================
# SECTION 4 — performance vs. baseline
# ベースライン比の性能
# =============================================================================
def alt_stats(d, t0, t1):
    pv = d["posvel"]
    m = (pv["ts"] >= t0) & (pv["ts"] <= t1)
    alt = -pv["pos"][m, 2]
    t = pv["ts"][m]
    return dict(std_mm=float(np.std(alt)) * 1000.0, p2p_mm=float(np.max(alt) - np.min(alt)) * 1000.0,
                mean_m=float(np.mean(alt)), n=int(m.sum()),
                band_low_mm=band_rms_welch(t, alt, 0.02, 0.1) * 1000.0,
                band_mid_mm=band_rms_welch(t, alt, 0.1, 0.5) * 1000.0,
                band_high_mm=band_rms_welch(t, alt, 0.5, 2.0) * 1000.0)


def radial_stats(d, t0, t1):
    cr = d["ctrl_ref"]
    pv = d["posvel"]
    m = (cr["ts"] >= t0) & (cr["ts"] <= t1)
    t_cr = cr["ts"][m]
    pos_sp = cr["pos_sp"][m]
    pos_x = np.interp(t_cr, pv["ts"], pv["pos"][:, 0])
    pos_y = np.interp(t_cr, pv["ts"], pv["pos"][:, 1])
    err = np.sqrt((pos_x - pos_sp[:, 0]) ** 2 + (pos_y - pos_sp[:, 1]) ** 2)
    return dict(radial_rms_mm=float(np.sqrt(np.mean(err ** 2))) * 1000.0,
                radial_mean_mm=float(np.mean(err)) * 1000.0,
                pos_sp_is_static=bool(np.allclose(pos_sp, pos_sp[0], atol=1e-6)))


# =============================================================================
# SECTION 5 — improvement-candidate replay sweep (reuses step2_dob_design.py)
# 改善candidateの再生シム（step2_dob_design.py 再利用）
# =============================================================================
def run_replay_sweep(win_trimmed):
    L = s2.load_full_log(LOG_DOB, win_trimmed)
    dext_acc = s2.compute_dext_all(L, "acc")
    meas = s2.measured_metrics(L)

    cases = {
        "0_no_dob_counterfactual": s2.default_cfg(dob_enabled=False),
        "1_current_fc1.5_clamp0.10": s2.default_cfg(dob_enabled=True, dob_fc=1.5, dob_clamp=0.10),
        "2_fc1.0_clamp0.10": s2.default_cfg(dob_enabled=True, dob_fc=1.0, dob_clamp=0.10),
        "3_fc0.7_clamp0.10": s2.default_cfg(dob_enabled=True, dob_fc=0.7, dob_clamp=0.10),
        "4_fc1.5_clamp0.07": s2.default_cfg(dob_enabled=True, dob_fc=1.5, dob_clamp=0.07),
        "5_fc1.0_clamp0.07": s2.default_cfg(dob_enabled=True, dob_fc=1.0, dob_clamp=0.07),
    }

    table = {}
    for name, cfg in cases.items():
        traces = s2.replay_all(L, dext_acc, cfg)
        m = s2.compute_metrics(traces, L["sp"])
        thrust_band = s2.band_rms_segments(
            [(tr["t"], tr["thrust_cmd"] - np.mean(tr["thrust_cmd"])) for tr in traces], 0.5, 5.0, fs=s2.FS)
        table[name] = dict(alt_std_mm=m["alt_std_mm"], alt_rms_mm=m["alt_rms_mm"],
                            band_low_mm=m["band_low_mm"], band_mid_mm=m["band_mid_mm"], band_high_mm=m["band_high_mm"],
                            thrust_diff_std_mN=m["thrust_diff_std_mN"],
                            thrust_band_0p5_5Hz_rms_mN=thrust_band * 1000.0,
                            dhat_std_mN=m["dhat_std_mN"], dob_clamp_sat_pct=m["dob_clamp_sat_pct"],
                            pi_clamp_sat_pct=m["pi_clamp_sat_pct"])
    return dict(measured=meas, cases=table, n_segments=len(L["segs"]),
                coverage_s=float(sum(e - s for s, e in L["segs"]) / s2.FS), sp_m=float(L["sp"]))


# =============================================================================
# Plots
# =============================================================================
def plot_dhat_overview(dhat_result, sat, t_change, t0_analysis, t1_analysis):
    t, dh = dhat_result["t"], dhat_result["d_hat"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    ax = axes[0]
    ax.plot(t, dh * 1000.0, lw=0.4, color="tab:blue")
    ax.axhline(DOB_SAT_THRESHOLD_N * 1000, color="r", ls=":", lw=1, label=f"+/-{DOB_SAT_THRESHOLD_N*1000:.1f}mN (near-clamp)")
    ax.axhline(-DOB_SAT_THRESHOLD_N * 1000, color="r", ls=":", lw=1)
    if t_change is not None:
        ax.axvline(t_change, color="green", ls="--", lw=1.2, label=f"AC-off changepoint~{t_change:.0f}s")
    ax.axvspan(t[0], t0_analysis, color="gray", alpha=0.15, label="excluded (engage transient)")
    ax.set_ylabel("d_hat [mN]")
    ax.set_title(f"Reconstructed d_hat(t) — full flight; saturation={sat['saturation_frac_pct']:.2f}% "
                 f"of samples, {sat['n_events']} events")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3)

    ax2 = axes[1]
    mid = 0.5 * (t0_analysis + t1_analysis)
    m = (t >= mid - 10) & (t <= mid + 10)
    ax2.plot(t[m], dh[m] * 1000.0, lw=0.8, color="tab:blue")
    ax2.axhline(DOB_SAT_THRESHOLD_N * 1000, color="r", ls=":", lw=1)
    ax2.axhline(-DOB_SAT_THRESHOLD_N * 1000, color="r", ls=":", lw=1)
    ax2.set_ylabel("d_hat [mN]")
    ax2.set_xlabel("t [s]")
    ax2.set_title(f"Representative 20s window (centered {mid:.0f}s)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "ab20260718_dhat_timeseries.png", dpi=150)
    plt.close(fig)


def plot_ac_changepoint(centers, power, t_change, best_ratio):
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(centers, power * 1000.0, "-o", ms=3, lw=1)
    if t_change is not None:
        ax.axvline(t_change, color="green", ls="--", label=f"detected changepoint {t_change:.1f}s (jump ratio {best_ratio:.2f})")
    ax.axvline(70.0, color="orange", ls=":", label="pilot-reported AC-off ~70s")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("0.2-2Hz band power of total_thrust [mN^2]")
    ax.set_title("AC-off changepoint detection (sliding 10s band power)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "ab20260718_ac_changepoint.png", dpi=150)
    plt.close(fig)


def plot_psd_overlay(psd_dob, psd_base, key, ylabel, fname):
    fig, ax = plt.subplots(figsize=(8, 5))
    for label, (f, Pxx) in psd_dob.items():
        ax.loglog(f, Pxx, lw=1.3, label=f"DOB flight ({label})")
    for label, (f, Pxx) in psd_base.items():
        ax.loglog(f, Pxx, lw=1.3, ls="--", label=f"baseline (no DOB) ({label})")
    ax.set_xlim(0.1, 10)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{key}: PSD, DOB flight vs. no-DOB baseline")
    ax.legend(fontsize=8)
    ax.grid(which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / fname, dpi=150)
    plt.close(fig)


def plot_coherence(f, coh):
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.semilogx(f, coh, lw=1.2)
    ax.axvspan(0.5, 2.0, color="orange", alpha=0.15, label="0.5-2Hz (design-predicted band)")
    ax.set_xlim(0.05, 10)
    ax.set_ylim(0, 1.05)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("coherence: d_hat vs total_thrust")
    ax.set_title("Is the DOB the driving source of thrust jerkiness (kakukaku)?")
    ax.legend(fontsize=8)
    ax.grid(which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "ab20260718_dhat_thrust_coherence.png", dpi=150)
    plt.close(fig)


def plot_band_performance(perf_table):
    logs = list(perf_table.keys())
    bands = ["band_low_mm", "band_mid_mm", "band_high_mm"]
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(logs))
    width = 0.25
    for bi, band in enumerate(bands):
        vals = [perf_table[k][band] for k in logs]
        ax.bar(x + bi * width, vals, width, label=band.replace("_mm", ""))
    ax.set_xticks(x + width)
    ax.set_xticklabels(logs, rotation=15, fontsize=8)
    ax.set_ylabel("alt band RMS [mm]")
    ax.set_title("Altitude band RMS: low(0.02-0.1Hz)/mid(0.1-0.5Hz)/high(0.5-2Hz)")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "ab20260718_band_performance.png", dpi=150)
    plt.close(fig)


def plot_sweep_table(sweep):
    cases = list(sweep["cases"].keys())
    alt_std = [sweep["cases"][c]["alt_std_mm"] for c in cases]
    thr_jerk = [sweep["cases"][c]["thrust_diff_std_mN"] for c in cases]
    fig, ax1 = plt.subplots(figsize=(10, 5))
    x = np.arange(len(cases))
    ax1.bar(x - 0.2, alt_std, 0.4, color="tab:blue", label="alt std [mm]")
    ax1.set_ylabel("alt std [mm]", color="tab:blue")
    ax1.set_xticks(x)
    ax1.set_xticklabels(cases, rotation=20, fontsize=7, ha="right")
    ax2 = ax1.twinx()
    ax2.bar(x + 0.2, thr_jerk, 0.4, color="tab:red", label="thrust jerk (dthr std) [mN]")
    ax2.set_ylabel("thrust diff std [mN]", color="tab:red")
    ax1.set_title(f"Improvement-candidate replay sweep (measured alt std={sweep['measured']['std_mm']:.1f}mm)")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "ab20260718_sweep_tradeoff.png", dpi=150)
    plt.close(fig)


# =============================================================================
# Main
# =============================================================================
def main():
    results = {}

    print("=" * 100)
    print("SECTION 1 — flight window / mode / hands-off-ness / voltage / AC changepoint")
    print("=" * 100)
    info_dob, d_dob, win_dob, t_air_dob = flight_conditions(LOG_DOB, "DOB (2026-07-18)")
    info_base, d_base, win_base, t_air_base = flight_conditions(LOG_BASE, "baseline (2026-07-17, no DOB)")
    for info in (info_dob, info_base):
        print(f"  {info['label']}: window={info['flight_window_raw_s'][0]:.1f}-{info['flight_window_raw_s'][1]:.1f}s "
              f"({info['flight_duration_s']:.1f}s)  t_airborne={info['t_airborne_s']:.2f}s  "
              f"modes={info['mode_names']}  V={info['voltage_min_V']:.3f}-{info['voltage_max_V']:.3f}  "
              f"stick_std(r/p/y/t)={[round(v,4) for v in info['stick_std'].values()]}")

    t0_analysis = t_air_dob + EDGE_TRIM_S
    t1_analysis = win_dob[1] - EDGE_TRIM_S

    # AC-off changepoint: search the FULL flight (not a window pre-biased toward
    # 70s) for the biggest 0.2-2Hz band-power step, on BOTH total_thrust (50Hz,
    # ctrl_ref) and f_up (400Hz, imu) as independent proxies for "disturbance
    # character changed". A SEPARATE, falsification-style direct test is also
    # run exactly at the pilot's claimed t=70s (local_step_test), independent
    # of whether 70s turns out to be the global maximum.
    # エアコンOFF切替点: 70s周辺に偏らせず全飛行区間で最大の0.2-2Hz帯パワー段差を
    # total_thrust(50Hz)とf_up(400Hz)の両方で独立に探索。さらに反証形式の直接検定
    # として、申告時刻t=70sそのものについても（グローバル最大かどうかに関わらず）
    # 個別に前後比較する。
    cr_dob = d_dob["ctrl_ref"]
    m_thr = (cr_dob["ts"] >= win_dob[0]) & (cr_dob["ts"] <= win_dob[1])
    search_lo, search_hi = t0_analysis + 3, t1_analysis - 10
    t_change, jump_ratio, centers, power = detect_ac_changepoint(
        cr_dob["ts"][m_thr], cr_dob["total_thrust"][m_thr], search_lo=search_lo, search_hi=search_hi)

    imu_dob_all = d_dob["imu"]
    m_imu_full = (imu_dob_all["ts"] >= win_dob[0]) & (imu_dob_all["ts"] <= win_dob[1])
    fup_full, _ = s1.body_accel_to_f_up(s1.bias_corrected_accel(imu_dob_all), imu_dob_all["quat"])
    t_change_fup, jump_ratio_fup, centers_fup, power_fup = detect_ac_changepoint(
        imu_dob_all["ts"][m_imu_full], fup_full[m_imu_full], search_lo=search_lo, search_hi=search_hi)

    local70_thrust = local_step_test(centers, power, 70.0)
    local70_fup = local_step_test(centers_fup, power_fup, 70.0)

    print(f"\n  AC-off changepoint search over full flight [{search_lo:.0f},{search_hi:.0f}]s:")
    print(f"    thrust 0.2-2Hz band power: global-max step at t={t_change}  jump_ratio={jump_ratio:.2f}")
    print(f"    f_up    0.2-2Hz band power: global-max step at t={t_change_fup}  jump_ratio={jump_ratio_fup:.2f}")
    print(f"  Direct falsification test AT the pilot-reported t=70s (not the global max):")
    if local70_thrust:
        print(f"    thrust: power_before={local70_thrust['power_before']:.3g} power_after={local70_thrust['power_after']:.3g} ratio={local70_thrust['ratio']:.2f}")
    if local70_fup:
        print(f"    f_up:   power_before={local70_fup['power_before']:.3g} power_after={local70_fup['power_after']:.3g} ratio={local70_fup['ratio']:.2f}")
    plot_ac_changepoint(centers, power, t_change, jump_ratio)

    results["section1_flight_conditions"] = dict(
        dob_flight=info_dob, baseline_flight=info_base,
        ac_changepoint_search_window_s=[search_lo, search_hi],
        ac_changepoint_thrust_s=t_change, ac_changepoint_thrust_jump_ratio=jump_ratio,
        ac_changepoint_fup_s=t_change_fup, ac_changepoint_fup_jump_ratio=jump_ratio_fup,
        local_test_at_70s_thrust=local70_thrust, local_test_at_70s_fup=local70_fup,
        analysis_window_s=[t0_analysis, t1_analysis])

    print()
    print("=" * 100)
    print("SECTION 2 — d_hat offline reconstruction + saturation")
    print("=" * 100)
    tg, fupg, cosg, thrg = build_recon_grid(d_dob, t_air_dob, win_dob[1])
    dhat_result = reconstruct_dhat(tg, fupg, cosg, thrg, fc=DOB_FC_FLOWN, hover=HOVER)
    m_analysis = (tg >= t0_analysis) & (tg <= t1_analysis)
    sat_all = saturation_stats(tg[m_analysis], dhat_result["d_hat"][m_analysis])
    d_hat_std_mN = float(np.std(dhat_result["d_hat"][m_analysis])) * 1000.0
    guard_frac_pct = float(np.mean(dhat_result["guard"][m_analysis])) * 100.0
    print(f"  d_hat std={d_hat_std_mN:.2f}mN  saturation(|d_hat|>={DOB_SAT_THRESHOLD_N*1000:.1f}mN)="
          f"{sat_all['saturation_frac_pct']:.2f}%  n_events={sat_all['n_events']}  "
          f"dur(mean/median/p90/max)s={sat_all['duration_mean_s']:.2f}/{sat_all['duration_median_s']:.2f}/"
          f"{sat_all['duration_p90_s']:.2f}/{sat_all['duration_max_s']:.2f}  f_up_guard_frac={guard_frac_pct:.3f}%")

    ac_on = m_analysis & (tg < (t_change if t_change else 1e18))
    ac_off = m_analysis & (tg >= (t_change if t_change else -1))
    sat_ac_on = saturation_stats(tg[ac_on], dhat_result["d_hat"][ac_on]) if ac_on.any() else None
    sat_ac_off = saturation_stats(tg[ac_off], dhat_result["d_hat"][ac_off]) if ac_off.any() else None
    if sat_ac_on and sat_ac_off:
        print(f"  AC-ON  window std={np.std(dhat_result['d_hat'][ac_on])*1000:.2f}mN sat={sat_ac_on['saturation_frac_pct']:.2f}%")
        print(f"  AC-OFF window std={np.std(dhat_result['d_hat'][ac_off])*1000:.2f}mN sat={sat_ac_off['saturation_frac_pct']:.2f}%")

    # time-binned saturation (battery-sag correlation check)
    bins_s = np.arange(t0_analysis, t1_analysis, 20.0)
    time_binned_sat = []
    for lo in bins_s:
        hi = min(lo + 20.0, t1_analysis)
        mb = (tg >= lo) & (tg < hi)
        if mb.sum() < 100:
            continue
        s_ = saturation_stats(tg[mb], dhat_result["d_hat"][mb])
        time_binned_sat.append(dict(t_lo=float(lo), t_hi=float(hi), sat_pct=s_["saturation_frac_pct"],
                                     std_mN=float(np.std(dhat_result["d_hat"][mb])) * 1000.0))
    print("  time-binned (20s) saturation %:", [round(x["sat_pct"], 2) for x in time_binned_sat])

    plot_dhat_overview(dhat_result, sat_all, t_change, t0_analysis, t1_analysis)

    results["section2_dhat"] = dict(
        d_hat_std_mN=d_hat_std_mN, f_up_guard_frac_pct=guard_frac_pct,
        saturation_overall=sat_all, saturation_ac_on=sat_ac_on, saturation_ac_off=sat_ac_off,
        time_binned_saturation=time_binned_sat,
    )

    print()
    print("=" * 100)
    print("SECTION 2b — PI correction saturation estimate + motor duty saturation")
    print("=" * 100)
    ce = corr_estimate(d_dob, dhat_result, t0_analysis, t1_analysis)
    corr_sat_pct = float(np.mean(np.abs(ce["corr_est"]) >= VLIM_SAT_THRESHOLD_N)) * 100.0
    corr_std_mN = float(np.std(ce["corr_est"])) * 1000.0
    md_sat = motor_duty_saturation(d_dob, t0_analysis, t1_analysis)
    md_sat_base = motor_duty_saturation(d_base, win_base[0] + EDGE_TRIM_S, win_base[1] - EDGE_TRIM_S)
    print(f"  corr_est std={corr_std_mN:.2f}mN  |corr_est|>={VLIM_SAT_THRESHOLD_N*1000:.0f}mN fraction={corr_sat_pct:.2f}%")
    print(f"  motor_duty>=0.95 (DOB flight): any-motor={md_sat['any_motor_saturated_pct']:.3f}%  per-motor={md_sat['per_motor_pct']}  max_duty={md_sat['duty_max']:.3f}")
    print(f"  motor_duty>=0.95 (baseline):   any-motor={md_sat_base['any_motor_saturated_pct']:.3f}%  per-motor={md_sat_base['per_motor_pct']}  max_duty={md_sat_base['duty_max']:.3f}")
    results["section2b_corr_and_duty"] = dict(corr_est_std_mN=corr_std_mN, corr_sat_frac_pct=corr_sat_pct,
                                               motor_duty_saturation_dob=md_sat, motor_duty_saturation_baseline=md_sat_base)

    # Run the improvement-candidate replay sweep NOW (computation only; its own
    # "SECTION 5" printout happens later) so Section 3 can report the
    # CONTROLLED (same flight, same disturbance environment, DOB on vs. off
    # counterfactual) thrust-activity comparison alongside the raw cross-log
    # one — the design prediction ("0.5-2Hz thrust power ~4.3x") was itself a
    # same-log counterfactual, not a cross-flight comparison, so the raw
    # DOB-flight-vs-baseline-flight PSD ratio in Section 3 is confounded by
    # different real disturbance environments and is NOT the like-for-like test.
    # 改善candidate再生シムをここで計算（出力は後の SECTION 5）— これで Section 3 が
    # 「同一飛行・同一外乱環境でのDOB有無反実仮想」という条件を揃えた比較を、生の
    # 異なるログ間比較と並べて報告できる。設計予測(0.5-2Hz推力パワー約4.3倍)自体が
    # 同一ログ内の反実仮想であり、異なる実飛行同士のPSD比は外乱環境の違いに
    # 交絡するため対応する検定ではない。
    win_trimmed = (win_dob[0] + EDGE_TRIM_S, win_dob[1] - EDGE_TRIM_S)
    sweep = run_replay_sweep(win_trimmed)

    print()
    print("=" * 100)
    print("SECTION 3 — frequency identification (\"カクカク\" source)")
    print("=" * 100)
    t0b, t1b = win_base[0] + EDGE_TRIM_S, win_base[1] - EDGE_TRIM_S
    imu_dob, imu_base = d_dob["imu"], d_base["imu"]
    pv_dob, pv_base = d_dob["posvel"], d_base["posvel"]

    def masked(arr_ts, lo, hi):
        return (arr_ts >= lo) & (arr_ts <= hi)

    m_i_d = masked(imu_dob["ts"], t0_analysis, t1_analysis)
    m_i_b = masked(imu_base["ts"], t0b, t1b)
    m_p_d = masked(pv_dob["ts"], t0_analysis, t1_analysis)
    m_p_b = masked(pv_base["ts"], t0b, t1b)
    m_c_d = masked(cr_dob["ts"], t0_analysis, t1_analysis)
    cr_base = d_base["ctrl_ref"]
    m_c_b = masked(cr_base["ts"], t0b, t1b)

    fup_dob, cos_dob_full = s1.body_accel_to_f_up(s1.bias_corrected_accel(imu_dob), imu_dob["quat"])
    fup_base, _ = s1.body_accel_to_f_up(s1.bias_corrected_accel(imu_base), imu_base["quat"])
    gyro_dob = gyro_corrected(imu_dob)
    gyro_base = gyro_corrected(imu_base)
    alt_dob_full = -pv_dob["pos"][:, 2]
    alt_base_full = -pv_base["pos"][:, 2]

    psd_signals = {}
    for key, (t_d, x_d, t_b, x_b, ylabel) in {
        "total_thrust": (cr_dob["ts"][m_c_d], cr_dob["total_thrust"][m_c_d], cr_base["ts"][m_c_b], cr_base["total_thrust"][m_c_b], "PSD [N^2/Hz]"),
        "f_up": (imu_dob["ts"][m_i_d], fup_dob[m_i_d], imu_base["ts"][m_i_b], fup_base[m_i_b], "PSD [(m/s^2)^2/Hz]"),
        "alt": (pv_dob["ts"][m_p_d], alt_dob_full[m_p_d], pv_base["ts"][m_p_b], alt_base_full[m_p_b], "PSD [m^2/Hz]"),
        "gyro_roll": (imu_dob["ts"][m_i_d], gyro_dob[m_i_d, 0], imu_base["ts"][m_i_b], gyro_base[m_i_b, 0], "PSD [(rad/s)^2/Hz]"),
        "gyro_pitch": (imu_dob["ts"][m_i_d], gyro_dob[m_i_d, 1], imu_base["ts"][m_i_b], gyro_base[m_i_b, 1], "PSD [(rad/s)^2/Hz]"),
    }.items():
        f_d, Pxx_d, _ = welch_psd(t_d, x_d)
        f_b, Pxx_b, _ = welch_psd(t_b, x_b)
        plot_psd_overlay({"full window": (f_d, Pxx_d)}, {"full window": (f_b, Pxx_b)}, key, ylabel,
                          f"ab20260718_psd_{key}.png")
        band = (f_d >= 0.5) & (f_d <= 2.0)
        band_b = (f_b >= 0.5) & (f_b <= 2.0)
        power_ratio_0p5_2 = (float(np.trapezoid(Pxx_d[band], f_d[band])) /
                              float(np.trapezoid(Pxx_b[band_b], f_b[band_b]) + 1e-30))
        psd_signals[key] = dict(power_ratio_0p5_2Hz_dob_over_base=power_ratio_0p5_2,
                                 std_dob=float(np.sqrt(np.trapezoid(Pxx_d, f_d))),
                                 std_base=float(np.sqrt(np.trapezoid(Pxx_b, f_b))))
        print(f"  {key}: DOB/base power ratio (0.5-2Hz)={power_ratio_0p5_2:.2f}x  "
              f"std(DOB)={psd_signals[key]['std_dob']:.4g}  std(base)={psd_signals[key]['std_base']:.4g}")

    # d_hat / thrust coherence
    dhat_on_cr = np.interp(cr_dob["ts"][m_c_d], dhat_result["t"], dhat_result["d_hat"])
    thr_on_cr = cr_dob["total_thrust"][m_c_d]
    dt_cr = float(np.median(np.diff(cr_dob["ts"][m_c_d])))
    fs_cr = 1.0 / dt_cr
    f_coh, coh = scipy_coherence(dhat_on_cr, thr_on_cr, fs=fs_cr, nperseg=min(len(thr_on_cr), int(20 * fs_cr)))
    plot_coherence(f_coh, coh)
    coh_0p5_2 = float(np.mean(coh[(f_coh >= 0.5) & (f_coh <= 2.0)]))
    coh_0p1_5 = float(np.mean(coh[(f_coh >= 0.1) & (f_coh <= 5.0)]))
    print(f"  d_hat<->total_thrust coherence: mean(0.5-2Hz)={coh_0p5_2:.2f}  mean(0.1-5Hz)={coh_0p1_5:.2f}")

    clamp_corr = clamp_thrust_correlation(sat_all["all_events"], cr_dob["ts"][m_c_d], thr_on_cr)
    print(f"  clamp<->thrust-jerk coincidence: n_checked={clamp_corr['n_events_checked']}  "
          f"near-event max|dthr|(median)={clamp_corr.get('near_event_max_dthr_median_mN', float('nan')):.2f}mN  "
          f"baseline|dthr|(median)={clamp_corr.get('baseline_dthr_median_mN', float('nan')):.2f}mN  "
          f"ratio={clamp_corr.get('ratio', float('nan')):.2f}")

    # CONTROLLED comparison (same flight, same reconstructed d_ext_acc, DOB
    # on vs. off) — the like-for-like test of the design-predicted 0.5-5Hz
    # thrust-activity increase (README: "max 4.3x, RMS 15.5->32.1mN").
    # 制御された比較（同一飛行・同一d_ext_acc、DOB有無）— 設計予測
    # (0.5-5Hz推力活動増加、README「最大4.3倍、RMS15.5→32.1mN」)の対応する検定。
    thr_nodob = sweep["cases"]["0_no_dob_counterfactual"]["thrust_band_0p5_5Hz_rms_mN"]
    thr_dob = sweep["cases"]["1_current_fc1.5_clamp0.10"]["thrust_band_0p5_5Hz_rms_mN"]
    controlled_ratio = thr_dob / thr_nodob if thr_nodob else float("nan")
    print(f"  CONTROLLED (same-log replay, DOB on/off counterfactual) thrust 0.5-5Hz RMS: "
          f"no-DOB={thr_nodob:.2f}mN  DOB-on={thr_dob:.2f}mN  ratio={controlled_ratio:.2f}x  "
          f"(design prediction from README: ~4.3x power / ~2.1x RMS on log A)")

    results["section3_frequency"] = dict(psd_power_ratios=psd_signals,
                                          dhat_thrust_coherence_0p5_2Hz=coh_0p5_2,
                                          dhat_thrust_coherence_0p1_5Hz=coh_0p1_5,
                                          clamp_thrust_jerk_correlation=clamp_corr,
                                          controlled_thrust_0p5_5Hz_ratio_dob_over_nodob=controlled_ratio)

    print()
    print("=" * 100)
    print("SECTION 4 — performance vs. baseline")
    print("=" * 100)
    alt_full = alt_stats(d_dob, t0_analysis, t1_analysis)
    alt_on = alt_stats(d_dob, t0_analysis, min(t_change, t1_analysis) if t_change else t1_analysis)
    alt_off = alt_stats(d_dob, max(t_change, t0_analysis) if t_change else t0_analysis, t1_analysis)
    alt_base = alt_stats(d_base, t0b, t1b)
    rad_dob = radial_stats(d_dob, t0_analysis, t1_analysis)
    rad_base = radial_stats(d_base, t0b, t1b)
    print(f"  alt std: DOB full={alt_full['std_mm']:.1f}mm  DOB AC-on={alt_on['std_mm']:.1f}mm  "
          f"DOB AC-off={alt_off['std_mm']:.1f}mm  baseline={alt_base['std_mm']:.1f}mm (memory ref 164-171mm)")
    print(f"  radial rms: DOB={rad_dob['radial_rms_mm']:.1f}mm (pos_sp_static={rad_dob['pos_sp_is_static']})  "
          f"baseline={rad_base['radial_rms_mm']:.1f}mm (pos_sp_static={rad_base['pos_sp_is_static']})")
    perf_table = {"DOB full": alt_full, "DOB AC-on": alt_on, "DOB AC-off": alt_off, "baseline (no DOB)": alt_base}
    plot_band_performance(perf_table)
    results["section4_performance"] = dict(alt_dob_full=alt_full, alt_dob_ac_on=alt_on, alt_dob_ac_off=alt_off,
                                            alt_baseline=alt_base, radial_dob=rad_dob, radial_baseline=rad_base)

    print()
    print("=" * 100)
    print("SECTION 5 — improvement-candidate replay sweep")
    print("=" * 100)
    print(f"  (sweep computed earlier, before Section 3, to support the controlled comparison there)")
    print(f"  measured (this flight): std={sweep['measured']['std_mm']:.1f}mm rms={sweep['measured']['rms_mm']:.1f}mm "
          f"p2p={sweep['measured']['p2p_mm']:.1f}mm  n_segments={sweep['n_segments']} coverage={sweep['coverage_s']:.1f}s")
    print(f"  {'case':30s} {'alt_std':>9s} {'thr_jerk':>10s} {'dhat_std':>9s} {'DOBsat%':>8s} {'thr0.5-5Hz':>11s}")
    for name, row in sweep["cases"].items():
        print(f"  {name:30s} {row['alt_std_mm']:8.1f}mm {row['thrust_diff_std_mN']:9.2f}mN "
              f"{row['dhat_std_mN']:8.2f}mN {row['dob_clamp_sat_pct']:7.2f}% {row['thrust_band_0p5_5Hz_rms_mN']:9.2f}mN")
    plot_sweep_table(sweep)
    results["section5_sweep"] = sweep

    RESULTS_PATH.write_text(json.dumps(results, indent=2, default=lambda o: float(o) if isinstance(o, np.floating) else o))
    print(f"\nResults written to {RESULTS_PATH}")
    print(f"Figures written to {FIGS_DIR}")


if __name__ == "__main__":
    main()
