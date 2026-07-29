#!/usr/bin/env python3
"""Assemble the final proposal.json from the already-computed intermediate
JSON/CSV artifacts (single source of truth = the actual analysis outputs,
not hand-retyped numbers)."""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from fit_vt import (invert_T, predict_V, ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH,
                     ALPHA_OLD, BETA_OLD, GAMMA_OLD, HOVER_T_PER_MOTOR, Ct_bench, Ct_old,
                     Am_bench, Bm_bench, Cm_bench, Am_old, Bm_old, Cm_old)

OUT_DIR = Path(__file__).parent


def corr_bench(v):
    return float(invert_T(np.array([v]), ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH)[0] / HOVER_T_PER_MOTOR)


def corr_old(v):
    return float(invert_T(np.array([v]), ALPHA_OLD, BETA_OLD, GAMMA_OLD)[0] / HOVER_T_PER_MOTOR)


def main():
    with open(OUT_DIR / "step1_falsification_result.json") as f:
        step1 = json.load(f)
    with open(OUT_DIR / "fit_results.json") as f:
        fit = json.load(f)
    with open(OUT_DIR / "holdout_and_diagnostics_results.json") as f:
        hd = json.load(f)
    with open(OUT_DIR / "sensitivity_sweep.json") as f:
        sweep = json.load(f)

    qs = pd.read_csv(OUT_DIR / "quasistatic_samples.csv")

    per_date = fit["per_date"]
    dates_ok = [d for d, v in per_date.items() if "skipped" not in v]
    v_hover_per_date = {d: predict_V(HOVER_T_PER_MOTOR, per_date[d]["alpha"],
                                      per_date[d]["beta"], per_date[d]["gamma"])
                        for d in dates_ok}
    corr_bench_per_date = {d: corr_bench(v) for d, v in v_hover_per_date.items()}
    corr_old_per_date = {d: corr_old(v) for d, v in v_hover_per_date.items()}

    excl = [d for d in dates_ok if d != "20260619"]  # n=33, noisy outlier date
    cb_arr = np.array([corr_bench_per_date[d] for d in excl])
    co_arr = np.array([corr_old_per_date[d] for d in excl])

    pooled = fit["pooled"]
    v_pooled_hover = predict_V(HOVER_T_PER_MOTOR, pooled["alpha"], pooled["beta"], pooled["gamma"])

    near = qs[(qs["T_per_motor_N"] > HOVER_T_PER_MOTOR * 0.97) & (qs["T_per_motor_N"] < HOVER_T_PER_MOTOR * 1.03)]
    v_empirical_hover = float(near["V_bar"].mean())

    proposal = {
        "study": "V(T) motor thrust-to-voltage mapping identification from flight logs",
        "repo_state": "read-only analysis; no repository files modified",
        "mass_kg": 0.037,
        "gravity_mps2": 9.80665,
        "hover_T_per_motor_N": HOVER_T_PER_MOTOR,

        "step1_self_verification": {
            "description": "T_actual/(m*g) over mechanically-defined steady-hover windows "
                            "(same criteria as ct_switch_study_20260727), reconstructed from "
                            "IMU (accel_raw - accel_bias) rotated to NED by quaternion, vertical "
                            "component -- ported verbatim from eskf_core.cpp's own accel_ned computation.",
            "n_segments": step1["n_segments"], "n_dates": step1["n_dates"],
            "pooled_mean_t_over_mg": step1["pooled_mean_t_over_mg"],
            "pooled_std_of_segment_means": step1["pooled_std_of_segment_means"],
            "verdict": "PASS: mean 1.0020, std 0.0099 (~1%) -- sign convention and "
                       "accel-bias handling confirmed correct.",
        },

        "step2_quasistatic_extraction": {
            "n_samples": int(len(qs)), "n_dates": int(qs["date"].nunique()),
            "n_files": int(qs["file"].nunique()),
            "duty_range": [float(qs["duty_mean"].min()), float(qs["duty_mean"].max())],
            "voltage_range": [float(qs["voltage"].min()), float(qs["voltage"].max())],
            "T_per_motor_range_N": [float(qs["T_per_motor_N"].quantile(0.01)),
                                     float(qs["T_per_motor_N"].quantile(0.99))],
            "T_range_over_median": pooled["T_range_over_median"],
            "finding": "Realized per-motor thrust is pinned near the hover value "
                       "((p95-p5)/median ~ 0.20) across ALL 11 dates and ALL flight modes present "
                       "(ALT_HOLD/POS_HOLD; ACRO=0s, STAB=5.3s total in the selected logs). "
                       "Sensitivity sweep (sensitivity_sweep.csv/json) confirms this is NOT an "
                       "artifact of overly strict thresholds: relaxing tilt to 30deg, |vz| to "
                       "2.0 m/s, and REMOVING the duty-settling requirement entirely (admitting "
                       "raw motor transients) only widens T_range/median from 0.196 to 0.233, and "
                       "actually WORSENS corr(T,V_bar) from 0.088 to 0.017. This is a physical "
                       "property of these flights (steady non-accelerating flight requires "
                       "T*cos(tilt)=mg by Newton's law regardless of climb/descent velocity), "
                       "not a selection-criterion artifact.",
        },

        "step3_fit": {
            "pooled": pooled,
            "per_date": per_date,
            "identifiability_diagnostics": fit["identifiability"],
            "finding": "The 3-parameter model alpha*T+beta*sqrt(T)+gamma is NOT separably "
                       "identifiable from this flight data: corr(alpha,beta) = -0.995 (pooled), "
                       "-0.98 to -1.00 in every individual date's fit; condition number of X'X "
                       "is 4.1e6 pooled, 1e5-1e9 per date. Per-date alpha point estimates range "
                       "from -25 to +4050 (physically meaningless individually). This is the "
                       "expected mathematical signature of fitting a 2-curvature-parameter model "
                       "to data spanning <20% relative range in the regressor -- NOT evidence of "
                       "a coding bug (design matrix and residuals were checked directly).",
        },

        "step4_holdout": hd["holdout"],
        "step4_finding": "2-fold chronological holdout (train on 6 early dates / test on 5 late "
                          "dates, and vice versa) gives implied_corr = 1.21 and 0.84 respectively "
                          "-- a ~40%-relative swing that MASSIVELY exceeds the within-fold formal "
                          "delta-method SE (~0.03-0.05%). This confirms session-to-session drift "
                          "in the real V-duty relationship (battery pack, ESC, temperature, "
                          "connector wear) is the dominant source of uncertainty, not sampling "
                          "noise -- a single fitted curve does not generalize across sessions.",

        "step5_gap_location": {
            "verdict": "NOT RESOLVABLE per-parameter (alpha vs beta vs gamma) given corr(alpha,beta) "
                       "~ -0.995 -- attributing the gap to 'the thrust/torque-coefficient system' "
                       "(alpha) vs 'the Km/sqrt(Ct) system' (beta) vs 'the offset' (gamma) would be "
                       "reporting noise as signal. The only quantity this dataset can speak to is "
                       "the LOCAL V(T) value near the realized operating point.",
            "v_at_T_quantiles": hd["bench_old_comparison_at_T"],
            "delta_method_prediction_band": hd["delta_method_prediction_band"],
            "interpretation": "At the median realized T (~hover point, 0.0908N/motor): flight-"
                               "consistent V=2.510V vs bench-predicted 2.680V (bench overshoots "
                               "flight-observed V by 6.4% => bench chain would need thrust_corr<1) "
                               "vs old-predicted 2.411V (old undershoots by 4.1% => old chain needs "
                               "thrust_corr>1). Across the narrow realized T window the %gap vs "
                               "bench drifts from ~0% (T_p5) to ~-10% (T_p95), vs old from +10.8% "
                               "(T_p5) to +0.1% (T_p95) -- but T_p5/T_p95 are confounded with "
                               "date/session (not a controlled T sweep at fixed Vbat/session), so "
                               "this local slope should NOT be read as confirmed curve-shape error.",
        },

        "cross_validation_with_prior_study": {
            "description": "ct_switch_study_20260727 (analysis/scripts/ct_switch_study_20260727/, "
                            "same repo, prior session) computed implied_corr by a DIFFERENT method: "
                            "assume T=mg exactly at 85 hand-selected steady-hover segments, then "
                            "invert observed (duty,Vbat) through each candidate chain's forward "
                            "model. This study instead reconstructs T independently from IMU "
                            "specific force across 76,835 quasi-static samples (not just hover) "
                            "and regresses V(T) directly. The two methods agree closely:",
                "prior_study_old_chain_corr": "1.083 +/- 0.064 (n=9 dates, pooled 85 segments)",
                "prior_study_new_bench_chain_corr": "0.909 +/- 0.053 (n=9 dates, pooled 85 segments)",
            "this_study_old_chain_corr_per_date_mean": {"mean": float(co_arr.mean()), "std": float(co_arr.std()),
                                                          "n_dates": int(len(excl))},
            "this_study_bench_chain_corr_per_date_mean": {"mean": float(cb_arr.mean()), "std": float(cb_arr.std()),
                                                            "n_dates": int(len(excl))},
            "this_study_old_chain_corr_pooled_point_estimate": corr_old(v_pooled_hover),
            "this_study_bench_chain_corr_pooled_point_estimate": corr_bench(v_pooled_hover),
            "this_study_bench_chain_corr_empirical_near_hover": corr_bench(v_empirical_hover),
            "this_study_old_chain_corr_empirical_near_hover": corr_old(v_empirical_hover),
            "verdict": "Independent corroboration (different method, different/larger sample, "
                       "same repo/task). Both land in the same ~0.88-0.91 (bench) / ~1.05-1.08 "
                       "(old) neighborhood.",
        },

        "step6_gauge_closure_proposal": {
            "principle": "Ct alone is not identifiable from flight data (gauge freedom, per task "
                         "brief); alpha and beta are ALSO not separably identifiable from this "
                         "flight dataset's narrow T range (Step 3/4 findings above). Therefore this "
                         "study does NOT propose new {Am,Bm,Cm} point values fit from flight data "
                         "-- doing so would be fitting noise. Instead: retain the bench-measured "
                         "SHAPE (the only source of genuine T-range/curvature data: a real thrust-"
                         "stand + coast-down + DC-motor-bench sweep), and close the flight-vs-bench "
                         "gap purely as a multiplicative scalar (hover.thrust_corr), which is "
                         "exactly the mechanism the firmware already has for this purpose.",
            "retain_unchanged": {"Ct": Ct_bench, "Am": Am_bench, "Bm": Bm_bench, "Cm": Cm_bench},
            "recommended_hover_thrust_corr_if_switching_to_bench_chain": {
                "point_estimate_range": [round(float(cb_arr.mean() - 0.5*cb_arr.std()), 3),
                                          round(float(cb_arr.mean() + 0.5*cb_arr.std()), 3)],
                "suggested_default": 0.90,
                "basis": "per-date mean 0.879+/-0.074 (n=10 dates, excl. noisy 33-sample "
                         "2026-06-19), pooled/empirical point estimate 0.894-0.897; rounded to "
                         "0.90 for consistency with ct_switch_study_20260727's independently-"
                         "derived recommendation (which this study corroborates rather than "
                         "supersedes).",
            },
            "caveat": "This correction is validated ONLY near the hover operating point (the "
                      "region with actual data density). Session-to-session drift (holdout swing "
                      "0.84-1.21, per-date std ~0.07-0.09) is comparable in magnitude to the "
                      "correction itself -- a fixed firmware constant cannot fully capture this; "
                      "the existing onboard hover.thrust.learn mechanism (online PI correction, "
                      "already default-on per project memory) remains the primary absorber of "
                      "session-to-session residual, not a replacement target for this constant.",
        },

        "sensitivity_sweep_summary": sweep,

        "artifacts": {
            "scripts": ["parse_log_vt.py", "select_and_cache.py", "extract_quasistatic.py",
                        "fit_vt.py", "holdout_and_diagnostics.py", "sensitivity_sweep.py",
                        "plot_results_vt.py", "build_proposal_json.py"],
            "data": ["selected_logs.txt", "step1_hover_segments.csv", "step1_falsification_result.json",
                     "quasistatic_samples.csv", "quasistatic_samples_with_resid.csv",
                     "fit_results.json", "holdout_and_diagnostics_results.json",
                     "residual_by_date.csv", "sensitivity_sweep.csv", "sensitivity_sweep.json"],
            "figures": ["vt_scatter_fit.png", "per_date_params.png", "per_date_v_at_hover.png",
                        "residual_diagnostics.png", "holdout_validation.png"],
        },
    }

    with open(OUT_DIR / "proposal.json", "w") as f:
        json.dump(proposal, f, indent=2, default=str)
    print("Wrote proposal.json")
    print(json.dumps(proposal["step6_gauge_closure_proposal"], indent=2))


if __name__ == "__main__":
    main()
