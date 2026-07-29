#!/usr/bin/env python3
"""
Back out Rm_cold / Rm_hot / implied winding-temperature rise from the
observed V_app range, using the steady-hover torque-balance relation quoted
in the task brief:

    V* = Km*omega_h + (Rm/Km) * (Cq*omega_h^2 + Qf)
       = Km*omega_h + Rm * I_hover              [I_hover = (Cq*omega_h^2+Qf)/Km]

so  Rm = (V* - Km*omega_h) / I_hover.

All of Km, Cq, Qf, omega_h are themselves uncertain (documented multi-way
disagreements in qa_log.md and vt_ident_study/ct_switch_study: implied thrust
efficiency is only ~0.84-0.91 of the theoretical chain, so the *true* in-
flight omega_h could differ from the theoretical 3670 rad/s by several
percent). Treat these Rm numbers as order-of-magnitude / self-consistent
within this study's own gauge choice, not absolute ground truth.

課題ブリーフのホバー・トルク釣り合い式を使い、観測されたV_appのレンジから
Rm_cold/Rm_hot/推定巻線温度上昇を逆算する。Km/Cq/Qf/omega_hはいずれもこの
リポジトリの既存研究で多値併存(不確か)であることが分かっており、ここで出す
Rm値は絶対値ではなく本スタディの前提(gauge)内で自己無矛盾な目安として扱う。
"""
import json
from pathlib import Path

KM = 5.682e-4
CQ = 4.10e-11
QF = 9.507e-6
OMEGA_H = 3670.0
ALPHA_CU = 0.00393  # 1/degC

I_HOVER = (CQ * OMEGA_H**2 + QF) / KM
EMF_TERM = KM * OMEGA_H

# Reference LCR/firmware values already in the repo (for comparison only).
RM_LCR_PAPER = 0.593
RM_OLD_FIRMWARE = 0.34


def rm_of_vstar(vstar):
    return (vstar - EMF_TERM) / I_HOVER


def main():
    # Observed "coldest" V_app -- earliest hover-segment samples across the
    # long flights and the pause-pair flight-A-start references (see
    # hover_timeseries.csv / pause_pair_raw.py output). Using the pooled
    # range rather than a single point to show sensitivity.
    v_cold_candidates = {
        "020137_first_seg (t=11.8s)": 2.4840,
        "022929_first_seg (t=7.3s)": 2.4530,
        "231940_first_seg (t=6.5s)": 2.4327,
        "020050->020137 pause pair, flight-A start": 2.4328,
        "161016->161055 pause pair, flight-A start": 2.4229,
        "161016->161055 pause pair, flight-B start (post-gap)": 2.4262,
        "020050->020137 pause pair, flight-B start (post-gap)": 2.4536,
    }
    # Observed "hottest"/latest V_app in the longest flights (lower bound on
    # the saturated value -- V_app is still rising at the end of every long
    # flight we have, so the true saturated dV is >= these numbers).
    v_hot_candidates = {
        "020137_last_seg (t=172s)": 2.5943,
        "020137_near-end plateau (t=136-172s, mean)": 2.580,
        "pause pair implied saturated estimate (V0+dV/0.39, tau~15-30s prior)": None,
    }

    print(f"I_hover = {I_HOVER:.4f} A/motor   Km*omega_h (EMF term) = {EMF_TERM:.4f} V\n")

    print("--- Rm_cold candidates ---")
    rm_cold_vals = []
    for label, v in v_cold_candidates.items():
        rm = rm_of_vstar(v)
        rm_cold_vals.append(rm)
        print(f"  {label}: V*={v:.4f}V -> Rm={rm:.4f} Ohm")
    rm_cold_mean = sum(rm_cold_vals) / len(rm_cold_vals)
    print(f"  mean Rm_cold = {rm_cold_mean:.4f} Ohm  (range {min(rm_cold_vals):.3f}-{max(rm_cold_vals):.3f})")

    print("\n--- Rm_hot candidates (lower bounds -- flights hadn't saturated yet) ---")
    rm_hot_vals = []
    for label, v in v_hot_candidates.items():
        if v is None:
            continue
        rm = rm_of_vstar(v)
        rm_hot_vals.append(rm)
        print(f"  {label}: V*={v:.4f}V -> Rm={rm:.4f} Ohm")

    # Pause-pair-implied saturated estimate: 10s of flight-A accumulated
    # ~0.06V of rise (average of the two pairs' flight-A start->end deltas);
    # if a saturating exponential with tau ~ 15-30s (the pause-pair reset
    # test's implied fast time constant) has covered a fraction
    # f = 1 - exp(-10/tau) of its total rise in that first ~10s, the
    # extrapolated *total* saturated dV = observed_10s_rise / f.
    rise_10s = ((2.5045 - 2.4328) + (2.4887 - 2.4229)) / 2  # avg of both pairs
    for tau in (15, 20, 30):
        f = 1 - pow(2.718281828, -10.0 / tau)
        dv_sat = rise_10s / f
        v_hot_est = rm_cold_mean * I_HOVER + EMF_TERM + dv_sat  # = v_cold+dv_sat, just recomposed
        v_hot_est = (2.4328 + 2.4229) / 2 + dv_sat
        rm_hot = rm_of_vstar(v_hot_est)
        rm_hot_vals.append(rm_hot)
        print(f"  tau={tau}s prior -> f(10s)={f:.3f}, dV_sat(extrap)={dv_sat:.4f}V, "
              f"V*_hot={v_hot_est:.4f}V -> Rm_hot={rm_hot:.4f} Ohm")

    rm_hot_mean = sum(rm_hot_vals) / len(rm_hot_vals) if rm_hot_vals else float("nan")
    print(f"\n  mean/range Rm_hot (all estimators) = {rm_hot_mean:.4f} Ohm "
          f"(range {min(rm_hot_vals):.3f}-{max(rm_hot_vals):.3f})")

    dR = rm_hot_mean - rm_cold_mean
    frac = dR / rm_cold_mean
    dT = frac / ALPHA_CU
    print(f"\nDelta R = {dR:.4f} Ohm  (fractional {frac*100:.1f}%)")
    print(f"Implied winding temperature rise (copper alpha={ALPHA_CU}/degC): "
          f"{dT:.0f} degC")

    print(f"\nComparison: Rm_cold (this study) = {rm_cold_mean:.3f} Ohm  vs  "
          f"'old firmware' Rm = {RM_OLD_FIRMWARE} Ohm  "
          f"(diff {100*(rm_cold_mean-RM_OLD_FIRMWARE)/RM_OLD_FIRMWARE:+.1f}%)")
    print(f"Comparison: Rm_hot  (this study) = {rm_hot_mean:.3f} Ohm  vs  "
          f"'LCR paper' Rm = {RM_LCR_PAPER} Ohm  "
          f"(diff {100*(rm_hot_mean-RM_LCR_PAPER)/RM_LCR_PAPER:+.1f}%)")

    out = {
        "I_hover_A": I_HOVER, "EMF_term_V": EMF_TERM,
        "Rm_cold_mean_ohm": rm_cold_mean, "Rm_cold_candidates": {k: rm_of_vstar(v) for k, v in v_cold_candidates.items()},
        "Rm_hot_mean_ohm": rm_hot_mean,
        "dR_ohm": dR, "dR_fraction": frac, "implied_dT_degC": dT,
        "Rm_old_firmware": RM_OLD_FIRMWARE, "Rm_lcr_paper": RM_LCR_PAPER,
    }
    with open(Path(__file__).parent / "rm_estimate_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nsaved: rm_estimate_results.json")


if __name__ == "__main__":
    main()
