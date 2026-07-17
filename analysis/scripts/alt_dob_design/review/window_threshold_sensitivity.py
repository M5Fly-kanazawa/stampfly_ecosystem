#!/usr/bin/env python3
"""
Sensitivity checks for the L=62.4ms takeoff-transient estimate:
  (a) crossing-fraction sensitivity (does 50% crossing threshold vs 30%/70%
      change the delay estimate materially?)
  (b) window-size sensitivity (does post_win choice for the TAKEOFF events --
      not landing, which the script already flags as window-sensitive --
      change delay_crossing/delay_xcorr?)

Reuses s1's find_step_edge/moving_average/M/HOVER (pure utility functions,
not the claim-bearing computation itself) but reimplements plateau_crossing
with a parameterized fraction, and the xcorr scan with parameterized windows,
independently.
"""
import sys
import numpy as np

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/alt_dob_design")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")

import step1_actuation_id as s1  # noqa: E402
from yawlib import load_jsonl    # noqa: E402


def plateau_crossing_frac(t, x, t_edge, pre_win, post_win, rising, frac=0.5):
    before = (t >= t_edge - pre_win) & (t < t_edge - 0.02)
    after = (t > t_edge + 0.02) & (t <= t_edge + post_win)
    if before.sum() < 2 or after.sum() < 2:
        return None
    lo_level = np.median(x[before]) if rising else np.median(x[after])
    hi_level = np.median(x[after]) if rising else np.median(x[before])
    thr = lo_level + frac * (hi_level - lo_level)
    win = (t >= t_edge - pre_win) & (t <= t_edge + post_win)
    tt, xx = t[win], x[win]
    if rising:
        idx = np.where((xx[:-1] < thr) & (xx[1:] >= thr))[0]
    else:
        idx = np.where((xx[:-1] > thr) & (xx[1:] <= thr))[0]
    if len(idx) == 0:
        return None
    i = idx[0]
    t0, t1, x0, x1 = tt[i], tt[i + 1], xx[i], xx[i + 1]
    fr = 0.0 if x1 == x0 else (thr - x0) / (x1 - x0)
    return float(t0 + fr * (t1 - t0))


def get_takeoff_data(d, t_event_approx):
    imu = d["imu"]
    tim = imu["ts"]
    accel_corr = s1.bias_corrected_accel(imu)
    f_up_native, cos_native = s1.body_accel_to_f_up(accel_corr, imu["quat"])
    y_native = s1.M * f_up_native
    y_native_smooth = s1.moving_average(y_native, 8)

    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]
    cos_on_tcr = np.interp(tcr, tim, cos_native)
    u_native_on_tcr = thr * cos_on_tcr

    edge = s1.find_step_edge(tcr, thr, t_event_approx, search_half=3.0, rising=True,
                              min_jump=0.1 * s1.HOVER)
    t_before, t_after, jump = edge
    t_edge = 0.5 * (t_before + t_after)
    return tim, y_native, y_native_smooth, tcr, u_native_on_tcr, t_edge


def crossing_fraction_sweep(name, d, t_event_approx, pre_win=0.4, post_win=0.6):
    tim, y_native, y_native_smooth, tcr, u_on_tcr, t_edge = get_takeoff_data(d, t_event_approx)
    print(f"  [{name}] t_edge={t_edge:.4f}s  crossing-fraction sweep (pre={pre_win}s post={post_win}s):")
    for frac in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8):
        t_u = plateau_crossing_frac(tcr, u_on_tcr, t_edge, pre_win, post_win, True, frac)
        t_y = plateau_crossing_frac(tim, y_native_smooth, t_edge, pre_win, post_win, True, frac)
        if t_u is None or t_y is None:
            print(f"      frac={frac:.1f}: n/a")
            continue
        print(f"      frac={frac:.1f}: delay={  (t_y - t_u)*1000:6.1f}ms")


def window_size_sweep(name, d, t_event_approx, pre_win=0.4):
    tim, y_native, y_native_smooth, tcr, u_on_tcr, t_edge = get_takeoff_data(d, t_event_approx)
    print(f"  [{name}] t_edge={t_edge:.4f}s  post_win sweep (crossing, frac=0.5, pre={pre_win}s):")
    for post_win in (0.15, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.5):
        t_u = plateau_crossing_frac(tcr, u_on_tcr, t_edge, pre_win, post_win, True, 0.5)
        t_y = plateau_crossing_frac(tim, y_native_smooth, t_edge, pre_win, post_win, True, 0.5)
        if t_u is None or t_y is None:
            print(f"      post_win={post_win:.2f}s: n/a")
            continue
        print(f"      post_win={post_win:.2f}s: delay={(t_y - t_u)*1000:6.1f}ms")

    # also xcorr-based, window swept
    dt = 1.0 / 400.0
    print(f"  [{name}] xcorr delay, post_win (search window) sweep:")
    for post_win in (0.1, 0.15, 0.2, 0.3, 0.4, 0.6, 0.8):
        win_lo, win_hi = t_edge - pre_win, t_edge + post_win
        m_im = (tim >= win_lo) & (tim <= win_hi)
        m_cr = (tcr >= win_lo) & (tcr <= win_hi)
        if m_im.sum() < 20 or m_cr.sum() < 2:
            continue
        cos_native_local = np.interp(tim, tcr, np.ones_like(tcr))  # placeholder unused
        u_z = s1.zoh_interp(tim[m_im], tcr, u_on_tcr)
        y_z = y_native[m_im]
        max_lag = min(int(round(0.3 / dt)), int(round(post_win / dt)))
        u_c = u_z - np.mean(u_z)
        y_c = y_z - np.mean(y_z)
        best_lag, best_corr = 0, -np.inf
        for lag in range(0, max_lag):
            a, b = (u_c, y_c) if lag == 0 else (u_c[:-lag], y_c[lag:])
            if len(a) < 20:
                break
            denom = np.linalg.norm(a) * np.linalg.norm(b)
            corr = float(np.dot(a, b) / denom) if denom > 0 else 0.0
            if corr > best_corr:
                best_corr, best_lag = corr, lag
        print(f"      post_win={post_win:.2f}s: delay={best_lag*dt*1000:6.1f}ms  peak_corr={best_corr:.3f}")


if __name__ == "__main__":
    dA = load_jsonl(str(s1.LOG_A))
    dB = load_jsonl(str(s1.LOG_B))

    print("=" * 100)
    print("TAKEOFF crossing-fraction sensitivity (does 50% threshold choice matter?)")
    print("=" * 100)
    crossing_fraction_sweep("A takeoff", dA, 58.742436)
    crossing_fraction_sweep("B takeoff", dB, 38.399776)

    print()
    print("=" * 100)
    print("TAKEOFF window-size sensitivity (does post_win choice matter, unlike landing?)")
    print("=" * 100)
    window_size_sweep("A takeoff", dA, 58.742436)
    window_size_sweep("B takeoff", dB, 38.399776)
