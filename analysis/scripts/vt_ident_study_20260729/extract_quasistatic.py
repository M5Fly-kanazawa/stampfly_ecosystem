#!/usr/bin/env python3
"""
Step 1 (self-verification) + Step 2 (quasi-static sample extraction) of the
V(T) motor identification study.

Step 1 — falsification check: reconstruct T_actual(t) = m * f_up(t) from the
IMU (accel_raw - accel_bias, rotated to NED via quaternion, vertical component
-- see parse_log_vt.py docstring for the sign-convention derivation, ported
directly from eskf_core.cpp) and check T_actual/(m*g) -> 1.00 with small
spread over MECHANICALLY-DEFINED steady-hover windows (same criteria as
ct_switch_study_20260727/hover_segments.py). If this fails, the projection/
bias handling is wrong and everything downstream is suspect.

Step 2 — quasi-static sample extraction across the FULL duty range (not just
hover): armed & non-saturated duty in [DUTY_MIN, DUTY_MAX], tilt < TILT_MAX_DEG,
|vz| < VELZ_MAX, and duty "settled" for >= SETTLE_WINDOW_SAMPLES consecutive
50Hz samples (see SETTLE_WINDOW derivation below, tied to the firmware's motor
transport-delay time constant tau ~= 16ms).
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from parse_log_vt import load_or_parse

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
OUT_DIR = Path(__file__).parent
CACHE_DIR = OUT_DIR / "cache"

MASS_KG = 0.037
GRAVITY = 9.80665
HOVER_THRUST_N = MASS_KG * GRAVITY

# --- Step 1: strict hover-window criteria (identical to ct_switch_study) ---
HOVER_WINDOW_SAMPLES = 75      # 1.5s at 50Hz
HOVER_ALT_STD_MAX = 0.02       # [m]
HOVER_VELZ_MAX = 0.10          # [m/s]
HOVER_STICK_MAX = 0.05
HOVER_DUTY_MIN = 0.05
HOVER_DUTY_SAT = 0.95
HOVER_MIN_SEG_SAMPLES = 50

# --- Step 2: quasi-static (wide duty range) criteria, per task brief ---
DUTY_MIN = 0.08
DUTY_MAX = 0.92
TILT_MAX_DEG = 10.0
VELZ_MAX = 0.8

# Motor transport delay tau ~= 16ms (bb409e93 "configurable motor transport
# delay in duty path"). A duty step needs ~3*tau (~48ms, ~90% settled for a
# 1st-order lag) to ~5*tau (~80ms, ~99% settled) before the ACTUAL thrust has
# caught up with the commanded duty; fitting a *steady-state* V(T) curve to a
# sample still inside that transient would bias the fit toward the transient
# dynamics, not the steady-state motor curve. Chosen here: a trailing window
# of 4 samples at 50Hz = 80ms ~= 5*tau, requiring duty_mean to stay within
# DUTY_STEADY_TOL of its own window max-min. 80ms/5tau is the conservative
# (safer) end of the 48-80ms range; a sensitivity sweep across window sizes
# is reported separately (see sensitivity_sweep.py) so the conclusions do not
# hinge on this exact choice.
SETTLE_WINDOW_SAMPLES = 4       # 80ms @ 50Hz ~= 5*tau
DUTY_STEADY_TOL = 0.01          # max-min duty allowed within the trailing window

# QC filter: reject samples whose +-10ms IMU averaging window had high internal
# scatter (vibration/impulsive event corrupting the window mean). Threshold
# chosen from the pooled distribution of f_up_std_mps2 (median 1.8, 99th pct
# ~5.0 m/s^2) -- 5.0 keeps ~99% of samples and removes the small tail that
# produces unphysical (negative) T_actual.
F_UP_STD_MAX = 5.0


def find_hover_segments(df: pd.DataFrame) -> pd.DataFrame:
    """Verbatim logic from ct_switch_study_20260727/hover_segments.py, ported
    here (not imported) since the repo copy must stay untouched, and extended
    to also report f_up_mps2 (T_actual/mg) instead of the old motor-chain
    implied_corr."""
    m = df[df["mode"].isin([2, 3])].reset_index(drop=True)
    if len(m) < HOVER_WINDOW_SAMPLES:
        return pd.DataFrame()

    alt_std = m["altitude"].rolling(HOVER_WINDOW_SAMPLES, center=True).std()
    velz_absmax = m["velz_up"].abs().rolling(HOVER_WINDOW_SAMPLES, center=True).max()
    stick_absmax = (
        m[["stick_roll", "stick_pitch", "stick_yaw"]].abs().max(axis=1)
        .rolling(HOVER_WINDOW_SAMPLES, center=True).max()
    )
    duty_min = m["duty_mean"].rolling(HOVER_WINDOW_SAMPLES, center=True).min()
    duty_max = m["duty_mean"].rolling(HOVER_WINDOW_SAMPLES, center=True).max()

    cand = (
        (alt_std < HOVER_ALT_STD_MAX)
        & (velz_absmax < HOVER_VELZ_MAX)
        & (stick_absmax < HOVER_STICK_MAX)
        & (duty_min > HOVER_DUTY_MIN)
        & (duty_max < HOVER_DUTY_SAT)
    ).fillna(False).to_numpy()

    segments = []
    i = 0
    n = len(cand)
    while i < n:
        if not cand[i]:
            i += 1
            continue
        j = i
        while j < n and cand[j]:
            j += 1
        if (j - i) >= HOVER_MIN_SEG_SAMPLES:
            segments.append((i, j))
        i = j

    rows = []
    for (i, j) in segments:
        seg = m.iloc[i:j]
        f_up = seg["f_up_mps2"].to_numpy()
        valid = np.isfinite(f_up) & np.isfinite(seg["voltage"].to_numpy())
        f_up = f_up[valid]
        if len(f_up) < HOVER_MIN_SEG_SAMPLES // 2:
            continue
        t_over_mg = (MASS_KG * f_up) / HOVER_THRUST_N
        rows.append({
            "t_start_s": seg["t_s"].iloc[0],
            "t_end_s": seg["t_s"].iloc[-1],
            "duration_s": seg["t_s"].iloc[-1] - seg["t_s"].iloc[0],
            "n_samples": len(seg),
            "duty_mean": float(seg["duty_mean"].mean()),
            "voltage_mean": float(seg["voltage"].mean()),
            "tilt_deg_mean": float(seg["tilt_deg"].mean()),
            "t_over_mg_mean": float(np.mean(t_over_mg)),
            "t_over_mg_std": float(np.std(t_over_mg)),
        })
    return pd.DataFrame(rows)


def step1_falsification_check(selected_files):
    """Pool hover-window T_actual/(mg) across all selected logs; report the
    distribution. This must center near 1.00 with small spread or the sign/
    bias handling has a bug."""
    all_rows = []
    for name in selected_files:
        path = LOG_DIR / name
        df = load_or_parse(path, CACHE_DIR)
        segs = find_hover_segments(df)
        if segs.empty:
            continue
        segs["file"] = name
        segs["date"] = name.split("_")[2][:8]
        all_rows.append(segs)

    if not all_rows:
        raise RuntimeError("No hover segments found for step-1 self-check -- cannot proceed.")

    segs_df = pd.concat(all_rows, ignore_index=True)
    segs_df.to_csv(OUT_DIR / "step1_hover_segments.csv", index=False)

    pooled_mean = float(segs_df["t_over_mg_mean"].mean())
    pooled_std = float(segs_df["t_over_mg_mean"].std())
    per_date = segs_df.groupby("date")["t_over_mg_mean"].agg(["mean", "std", "count"])
    print("=== STEP 1: T_actual/(m*g) over strict hover windows ===")
    print(f"n_segments={len(segs_df)}, pooled mean={pooled_mean:.4f}, "
          f"pooled std-of-segment-means={pooled_std:.4f}")
    print("Per-date:")
    print(per_date)

    result = {
        "n_segments": int(len(segs_df)),
        "n_dates": int(segs_df["date"].nunique()),
        "pooled_mean_t_over_mg": pooled_mean,
        "pooled_std_of_segment_means": pooled_std,
        "pooled_within_segment_std_mean": float(segs_df["t_over_mg_std"].mean()),
        "per_date": {k: {"mean": float(v["mean"]), "std": float(v["std"]), "n": int(v["count"])}
                     for k, v in per_date.iterrows()},
    }
    with open(OUT_DIR / "step1_falsification_result.json", "w") as f:
        json.dump(result, f, indent=2)
    return result


def compute_duty_settle_flag(m: pd.DataFrame) -> np.ndarray:
    """Trailing-window duty steadiness flag: True if duty_mean has stayed
    within DUTY_STEADY_TOL over the trailing SETTLE_WINDOW_SAMPLES samples
    (including the current one)."""
    duty = m["duty_mean"]
    roll_max = duty.rolling(SETTLE_WINDOW_SAMPLES, min_periods=SETTLE_WINDOW_SAMPLES).max()
    roll_min = duty.rolling(SETTLE_WINDOW_SAMPLES, min_periods=SETTLE_WINDOW_SAMPLES).min()
    spread = (roll_max - roll_min)
    return (spread < DUTY_STEADY_TOL).fillna(False).to_numpy()


def step2_extract_quasistatic(selected_files, duty_min=DUTY_MIN, duty_max=DUTY_MAX,
                               tilt_max=TILT_MAX_DEG, velz_max=VELZ_MAX,
                               settle_window=SETTLE_WINDOW_SAMPLES,
                               duty_steady_tol=DUTY_STEADY_TOL,
                               f_up_std_max=F_UP_STD_MAX,
                               modes=(0, 1, 2, 3), label="main"):
    """Extract per-sample quasi-static records across the full duty range for
    the V(T) fit. Returns a DataFrame with one row per accepted 50Hz sample."""
    rows = []
    for name in selected_files:
        path = LOG_DIR / name
        df = load_or_parse(path, CACHE_DIR)
        if df.empty:
            continue
        m = df[df["mode"].isin(list(modes))].reset_index(drop=True)
        if len(m) < settle_window:
            continue

        duty = m["duty_mean"]
        if settle_window > 1:
            roll_max = duty.rolling(settle_window, min_periods=settle_window).max()
            roll_min = duty.rolling(settle_window, min_periods=settle_window).min()
            settled = ((roll_max - roll_min) < duty_steady_tol).fillna(False)
        else:
            settled = pd.Series(True, index=m.index)

        mask = (
            (duty >= duty_min) & (duty <= duty_max)
            & (m["tilt_deg"] < tilt_max)
            & (m["velz_up"].abs() < velz_max)
            & settled
            & np.isfinite(m["voltage"])
            & np.isfinite(m["f_up_mps2"])
            & (m["f_up_n_samples"] >= 4)
            & (m["f_up_std_mps2"] < f_up_std_max)
        )
        sel = m[mask].copy()
        if sel.empty:
            continue
        sel["file"] = name
        sel["date"] = name.split("_")[2][:8]
        rows.append(sel[["file", "date", "t_s", "mode", "duty_mean", "duty_m1", "duty_m2",
                          "duty_m3", "duty_m4", "duty_std_across_motors", "voltage",
                          "f_up_mps2", "f_up_std_mps2", "tilt_deg", "velz_up", "altitude",
                          "total_thrust_logged"]])

    if not rows:
        return pd.DataFrame()
    out = pd.concat(rows, ignore_index=True)
    out["T_actual_N"] = MASS_KG * out["f_up_mps2"]
    out["T_per_motor_N"] = out["T_actual_N"] / 4.0
    out["V_bar"] = out["duty_mean"] * out["voltage"]
    return out


if __name__ == "__main__":
    with open(OUT_DIR / "selected_logs.txt") as f:
        selected = [l.strip() for l in f if l.strip()]

    step1 = step1_falsification_check(selected)

    qs = step2_extract_quasistatic(selected)
    qs.to_csv(OUT_DIR / "quasistatic_samples.csv", index=False)
    print(f"\n=== STEP 2: quasi-static samples ===")
    print(f"n_samples={len(qs)}, n_dates={qs['date'].nunique()}, n_files={qs['file'].nunique()}")
    print("duty_mean histogram (10 bins):")
    counts, edges = np.histogram(qs["duty_mean"], bins=10, range=(0, 1))
    for c, lo, hi in zip(counts, edges[:-1], edges[1:]):
        print(f"  [{lo:.2f},{hi:.2f}): {c}")
    print("\nper-date sample counts:")
    print(qs.groupby("date").agg(n=("t_s", "count"), duty_min=("duty_mean", "min"),
                                  duty_max=("duty_mean", "max"),
                                  duty_mean=("duty_mean", "mean")))
    print("\nmotor cross-spread (duty_std_across_motors) stats:")
    print(qs["duty_std_across_motors"].describe())
