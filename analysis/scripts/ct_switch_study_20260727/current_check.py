#!/usr/bin/env python3
"""
Optional secondary check (step 5 of the task): for the flight logs that
carry `current_ma` (battery current, 2026-07 sessions only), compare the
LOGGED battery current against a PREDICTED current computed from the
motor-electrical model:

    I_motor = (V_terminal - Km*omega) / Rm            (Ohm's law, back-EMF
                                                         subtracted)
    I_total_predicted = 4 * I_motor + I_avionics       (I_avionics ~ 0.1 A)

using the NEW chain's Rm/Km (bench-measured) and the omega implied by each
hover segment's (duty, vbat). This is independent of the thrust/Ct question
(current depends on Rm/Km only, not on Ct/Cq), so it is a genuinely
different observable — if it also matches well, that is separate
corroboration for the Rm/Km bench numbers (not for Ct, which the main
implied_corr analysis addresses).
"""
import numpy as np
import pandas as pd
from pathlib import Path

from parse_log import load_or_parse
from motor_chains import NEW_CHAIN, _Rm_new, _Km_new

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
OUT_DIR = Path(__file__).parent
CACHE_DIR = OUT_DIR / "cache"
I_AVIONICS_A = 0.1  # [A], rough estimate for ESP32 + sensors + WiFi, per task brief


def omega_of_duty(duty, vbat, chain=NEW_CHAIN):
    v = duty * vbat
    disc = np.maximum(chain.Bm**2 - 4.0 * chain.Am * (chain.Cm - v), 0.0)
    return np.maximum((-chain.Bm + np.sqrt(disc)) / (2.0 * chain.Am), 0.0)


def main():
    seg = pd.read_csv(OUT_DIR / "hover_segments.csv")
    seg = seg[seg["file"].str.contains("202607")]
    if seg.empty:
        print("No July (current_ma-carrying) hover segments found.")
        return

    rows = []
    for _, row in seg.iterrows():
        path = LOG_DIR / row["file"]
        df = load_or_parse(path, CACHE_DIR)
        mask = (df["t_s"] >= row["t_start_s"]) & (df["t_s"] <= row["t_end_s"])
        sub = df[mask]
        cur_ma = sub["current_ma"].dropna()
        if cur_ma.empty:
            continue
        duties = sub[["duty_m1", "duty_m2", "duty_m3", "duty_m4"]].to_numpy()
        vbat = sub["voltage"].to_numpy()
        valid = np.isfinite(vbat)
        duties = duties[valid]
        vbat = vbat[valid]
        omega = omega_of_duty(duties, vbat[:, None])
        v_terminal = duties * vbat[:, None]
        i_motor = (v_terminal - _Km_new * omega) / _Rm_new
        i_total_pred_a = i_motor.sum(axis=1) + I_AVIONICS_A
        pred_ma_mean = float(np.mean(i_total_pred_a) * 1000.0)
        logged_ma_mean = float(cur_ma.mean())
        rows.append({
            "file": row["file"], "t_start_s": row["t_start_s"],
            "logged_current_ma": logged_ma_mean,
            "predicted_current_ma": pred_ma_mean,
            "ratio_pred_over_logged": pred_ma_mean / logged_ma_mean if logged_ma_mean else np.nan,
        })

    out = pd.DataFrame(rows)
    if out.empty:
        print("No overlapping current_ma samples in the hover windows.")
        return
    out.to_csv(OUT_DIR / "current_check.csv", index=False)
    print(out.to_string(index=False))
    print("\nratio_pred_over_logged stats:")
    print(out["ratio_pred_over_logged"].describe())


if __name__ == "__main__":
    main()
