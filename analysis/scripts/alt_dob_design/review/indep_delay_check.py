#!/usr/bin/env python3
"""
Independent, from-scratch verification of the claimed pure-delay L~62ms
(thrust command -> upward specific force), using IN-FLIGHT small-signal
thrust-correction steps rather than the takeoff/landing transients the
original script relies on.

Does NOT import step1_actuation_id.py or alt_ti_venue_sim.py -- only reuses
the already-independently-verified DCM formula (checked line-by-line against
firmware/vehicle/components/sf_math/include/sf_math.hpp Quat::to_dcm()).
"""
import json
import numpy as np

M = 0.037
G = 9.80665
LOG_A = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260717T231940.jsonl"
LOG_B = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260717T224906.jsonl"


def load(path):
    imu_ts, imu_accel, imu_bias, imu_quat = [], [], [], []
    cr_ts, cr_thrust = [], []
    pv_ts, pv_vel = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            rid = r.get("id")
            if rid == "imu":
                imu_ts.append(r["ts"])
                imu_accel.append(r["accel"])
                imu_bias.append(r["accel_bias"])
                imu_quat.append(r["quat"])
            elif rid == "ctrl_ref":
                cr_ts.append(r["ts"])
                cr_thrust.append(r["total_thrust"])
            elif rid == "posvel":
                pv_ts.append(r["ts"])
                pv_vel.append(r["vel"])
    out = dict(
        imu_ts=np.array(imu_ts, dtype=float) / 1e6,
        imu_accel=np.array(imu_accel, dtype=float),
        imu_bias=np.array(imu_bias, dtype=float),
        imu_quat=np.array(imu_quat, dtype=float),
        cr_ts=np.array(cr_ts, dtype=float) / 1e6,
        cr_thrust=np.array(cr_thrust, dtype=float),
        pv_ts=np.array(pv_ts, dtype=float) / 1e6,
        pv_vel=np.array(pv_vel, dtype=float),
    )
    return out


def f_up(accel_corr, quat):
    # DCM row 3, verified against sf_math.hpp Quat::to_dcm() R[2][*]:
    #   R[2][0] = 2*(xz-wy); R[2][1] = 2*(yz+wx); R[2][2] = 1-2*(xx+yy)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    r31 = 2.0 * (x * z - w * y)
    r32 = 2.0 * (y * z + w * x)
    r33 = 1.0 - 2.0 * (x * x + y * y)
    az_ned = r31 * accel_corr[:, 0] + r32 * accel_corr[:, 1] + r33 * accel_corr[:, 2]
    return -az_ned


def analyze(name, path, flight_lo, flight_hi, thr_jump=0.02, exclude_edges=8.0):
    d = load(path)
    accel_corr = d["imu_accel"] - d["imu_bias"]
    fup = f_up(accel_corr, d["imu_quat"])
    y_native = M * fup
    tim = d["imu_ts"]
    fs_native = 1.0 / np.median(np.diff(tim))

    tcr = d["cr_ts"]
    thr = d["cr_thrust"]

    # sanity: ground bias check (first 2s of file, before flight_lo - 30, if present)
    ground_mask = (tim > tim[0] + 0.2) & (tim < flight_lo - 5)
    if ground_mask.sum() > 20:
        fup_ground_raw = f_up(d["imu_accel"][ground_mask], d["imu_quat"][ground_mask])
        fup_ground_corr = f_up(accel_corr[ground_mask], d["imu_quat"][ground_mask])
        print(f"  [{name}] ground window n={ground_mask.sum()} "
              f"mean f_up raw={np.mean(fup_ground_raw):.4f} corr={np.mean(fup_ground_corr):.4f} "
              f"(G={G:.4f})")

    # --- find in-flight thrust-correction step events (exclude takeoff/landing) ---
    lo, hi = flight_lo + exclude_edges, flight_hi - exclude_edges
    mc = (tcr >= lo) & (tcr <= hi)
    tcr_f, thr_f = tcr[mc], thr[mc]
    dthr = np.diff(thr_f)
    dt_cr = np.diff(tcr_f)
    # events: local extrema of |dthr| exceeding thr_jump, at least 1s apart
    idx_candidates = np.where(np.abs(dthr) > thr_jump)[0]
    events = []
    last_t = -1e9
    for i in idx_candidates:
        t_e = tcr_f[i]
        if t_e - last_t < 1.0:
            continue
        events.append((t_e, dthr[i]))
        last_t = t_e
    print(f"  [{name}] flight window used for event search: [{lo:.1f},{hi:.1f}]s, "
          f"{len(events)} step events found (|d(total_thrust)|>{thr_jump}N, native ctrl_ref cadence)")

    dt = 1.0 / fs_native
    max_lag_pos = int(round(0.20 / dt))
    max_lag_neg = int(round(0.10 / dt))   # also allow NEGATIVE lag (y leads u) to check causality
    pre_win, post_win = 0.35, 0.35

    delays = []
    for t_e, jump in events:
        win_lo, win_hi = t_e - pre_win, t_e + post_win
        m_im = (tim >= win_lo) & (tim <= win_hi)
        if m_im.sum() < 40:
            continue
        # ZOH thrust onto imu grid (vectorized)
        idxu = np.searchsorted(tcr, tim[m_im], side="right") - 1
        idxu = np.clip(idxu, 0, len(thr) - 1)
        u_on_imu = thr[idxu]
        y_on_imu = y_native[m_im]
        u_c = u_on_imu - np.mean(u_on_imu)
        y_c = y_on_imu - np.mean(y_on_imu)
        best_lag, best_corr = 0, -np.inf
        for lag in range(-max_lag_neg, max_lag_pos):
            if lag == 0:
                a, b = u_c, y_c
            elif lag > 0:
                a, b = u_c[:-lag], y_c[lag:]
            else:
                a, b = u_c[-lag:], y_c[:lag]
            if len(a) < 20:
                continue
            denom = np.linalg.norm(a) * np.linalg.norm(b)
            corr = float(np.dot(a, b) / denom) if denom > 0 else 0.0
            if corr > best_corr:
                best_corr, best_lag = corr, lag
        delay = best_lag * dt
        delays.append((t_e, jump, delay, best_corr))

    delays_arr = np.array([x[2] for x in delays])
    corrs_arr = np.array([x[3] for x in delays])
    print(f"  [{name}] {len(delays)} events processed.")
    for t_e, jump, delay, corr in delays:
        print(f"      t={t_e:9.3f}s  d(thrust)={jump:+.3f}N  xcorr_delay={delay*1000:6.1f}ms  peak_corr={corr:.3f}")
    if len(delays_arr):
        # only trust events with reasonable peak correlation
        good = corrs_arr > 0.3
        print(f"  [{name}] ALL events: median={np.median(delays_arr)*1000:.1f}ms "
              f"mean={np.mean(delays_arr)*1000:.1f}ms std={np.std(delays_arr)*1000:.1f}ms")
        if good.sum() >= 3:
            print(f"  [{name}] corr>0.3 events (n={good.sum()}): "
                  f"median={np.median(delays_arr[good])*1000:.1f}ms "
                  f"mean={np.mean(delays_arr[good])*1000:.1f}ms "
                  f"std={np.std(delays_arr[good])*1000:.1f}ms")
        else:
            print(f"  [{name}] WARNING: fewer than 3 events with peak_corr>0.3 -- "
                  f"in-flight xcorr is not well-conditioned for this log/threshold")
    print()
    return delays


def whole_segment_xcorr(name, path, flight_lo, flight_hi, exclude_edges=8.0, seg_len=60.0):
    """Independent whole-segment (not event-triggered) normalized cross-correlation
    of u_eff-mean and y-mean over a long continuous flight stretch, as a completely
    different (non-event-based) delay estimator."""
    d = load(path)
    accel_corr = d["imu_accel"] - d["imu_bias"]
    fup = f_up(accel_corr, d["imu_quat"])
    y_native = M * fup
    tim = d["imu_ts"]
    fs_native = 1.0 / np.median(np.diff(tim))
    tcr = d["cr_ts"]
    thr = d["cr_thrust"]

    lo = flight_lo + exclude_edges
    hi = min(flight_hi - exclude_edges, lo + seg_len)
    m_im = (tim >= lo) & (tim <= hi)
    tt = tim[m_im]
    # ZOH thrust onto imu grid (vectorized via searchsorted)
    idx = np.searchsorted(tcr, tt, side="right") - 1
    idx = np.clip(idx, 0, len(thr) - 1)
    u_on_imu = thr[idx]
    y_on_imu = y_native[m_im]
    u_c = u_on_imu - np.mean(u_on_imu)
    y_c = y_on_imu - np.mean(y_on_imu)
    dt = 1.0 / fs_native
    max_lag = int(round(0.25 / dt))
    lags = range(-max_lag, max_lag + 1)
    corrs = []
    for lag in lags:
        if lag >= 0:
            a, b = u_c[:len(u_c) - lag] if lag > 0 else u_c, y_c[lag:]
        else:
            a, b = u_c[-lag:], y_c[:len(y_c) + lag]
        n = min(len(a), len(b))
        a, b = a[:n], b[:n]
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        corrs.append(float(np.dot(a, b) / denom) if denom > 0 else 0.0)
    corrs = np.array(corrs)
    lags_arr = np.array(list(lags)) * dt
    i_best = np.argmax(corrs)
    print(f"  [{name}] whole-segment xcorr over [{lo:.1f},{hi:.1f}]s "
          f"({hi-lo:.0f}s): peak at lag={lags_arr[i_best]*1000:.1f}ms  corr={corrs[i_best]:.3f}")
    # print a few lags around peak for context
    for k in range(max(0, i_best - 3), min(len(lags_arr), i_best + 4)):
        print(f"      lag={lags_arr[k]*1000:6.1f}ms  corr={corrs[k]:.3f}")
    return lags_arr[i_best], corrs[i_best]


if __name__ == "__main__":
    print("=" * 90)
    print("Independent in-flight small-signal delay check (log A)")
    print("=" * 90)
    # flight window per step1_results.json step2a_flight_windows.A
    analyze("A", LOG_A, 58.742436, 199.447425, thr_jump=0.015)
    whole_segment_xcorr("A", LOG_A, 58.742436, 199.447425)

    print("=" * 90)
    print("Independent in-flight small-signal delay check (log B, richer excitation)")
    print("=" * 90)
    analyze("B", LOG_B, 38.399776, 95.997276, thr_jump=0.015)
    whole_segment_xcorr("B", LOG_B, 38.399776, 95.997276)
