#!/usr/bin/env python3
"""
Sensitivity sweep required by the task brief: "ホバー1点に実質縮退していた場合
(準静的条件が厳しすぎ)、条件を段階的に緩めた感度も報告(結論がデータ範囲に依存
しないか)". The baseline extraction (extract_quasistatic.py, TILT_MAX=10deg,
VELZ_MAX=0.8, SETTLE_WINDOW=4 samples/80ms) showed T_per_motor_N essentially
pinned to the hover value ((p95-p5)/median ~ 0.20, corr(T,V_bar) ~ 0.09,
corr(alpha,beta) ~ -0.995). This script relaxes each threshold one at a time
(and the settle window down to 1 sample = no settling requirement at all, an
intentionally-too-loose upper bound that reintroduces motor transients) to see
whether ANY reasonable relaxation meaningfully widens the achievable T range,
or whether the near-single-point collapse is a structural property of these
flights (ALT_HOLD/POS_HOLD, near-zero net vertical acceleration by design of
the control law) rather than an artifact of the specific thresholds chosen.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from extract_quasistatic import step2_extract_quasistatic
from fit_vt import fit_lsq

OUT_DIR = Path(__file__).parent

with open(OUT_DIR / "selected_logs.txt") as f:
    SELECTED = [l.strip() for l in f if l.strip()]

BASELINE = dict(duty_min=0.08, duty_max=0.92, tilt_max=10.0, velz_max=0.8,
                settle_window=4, duty_steady_tol=0.01, f_up_std_max=5.0)

SWEEPS = [
    {"label": "baseline", **BASELINE},
    {"label": "tilt<=15deg", **{**BASELINE, "tilt_max": 15.0}},
    {"label": "tilt<=20deg", **{**BASELINE, "tilt_max": 20.0}},
    {"label": "tilt<=30deg", **{**BASELINE, "tilt_max": 30.0}},
    {"label": "vz<=1.2", **{**BASELINE, "velz_max": 1.2}},
    {"label": "vz<=2.0", **{**BASELINE, "velz_max": 2.0}},
    {"label": "settle_window=2(40ms~2.5tau)", **{**BASELINE, "settle_window": 2}},
    {"label": "settle_window=3(60ms~3.75tau)", **{**BASELINE, "settle_window": 3}},
    {"label": "settle_window=6(120ms~7.5tau)", **{**BASELINE, "settle_window": 6}},
    {"label": "settle_tol=0.02(loose)", **{**BASELINE, "duty_steady_tol": 0.02}},
    {"label": "settle_tol=0.005(tight)", **{**BASELINE, "duty_steady_tol": 0.005}},
    {"label": "NO_SETTLING(window=1, upper bound, admits transients)",
     **{**BASELINE, "settle_window": 1, "duty_steady_tol": 1.0}},
    {"label": "ALL_RELAXED(tilt30,vz2.0,no-settle)",
     **{**BASELINE, "tilt_max": 30.0, "velz_max": 2.0, "settle_window": 1, "duty_steady_tol": 1.0}},
]


def run():
    rows = []
    for cfg in SWEEPS:
        label = cfg.pop("label")
        qs = step2_extract_quasistatic(SELECTED, **cfg)
        if qs.empty or len(qs) < 50:
            rows.append({"label": label, **cfg, "n": int(len(qs))})
            continue
        T = qs["T_per_motor_N"].to_numpy()
        Vb = qs["V_bar"].to_numpy()
        fit = fit_lsq(T, Vb)
        corr_T_Vbar = float(np.corrcoef(T, Vb)[0, 1])
        rows.append({
            "label": label, **cfg,
            "n": int(len(qs)),
            "n_dates": int(qs["date"].nunique()),
            "T_p5": fit["T_p5"], "T_p50": fit["T_p50"], "T_p95": fit["T_p95"],
            "T_range_over_median": fit["T_range_over_median"],
            "corr_T_Vbar": corr_T_Vbar,
            "corr_alpha_beta": fit["corr_alpha_beta"],
            "cond_number_XtX": fit["cond_number_XtX"],
            "r2": fit["r2"],
            "alpha": fit["alpha"], "alpha_se": fit["alpha_se"],
            "beta": fit["beta"], "beta_se": fit["beta_se"],
            "gamma": fit["gamma"], "gamma_se": fit["gamma_se"],
        })
        print(f"{label:55s} n={len(qs):7d} T_range/med={fit['T_range_over_median']:.3f} "
              f"corr(T,Vbar)={corr_T_Vbar:+.4f} corr(a,b)={fit['corr_alpha_beta']:+.4f} "
              f"R2={fit['r2']:.4f} cond={fit['cond_number_XtX']:.2e}")

    df = pd.DataFrame(rows)
    df.to_csv(OUT_DIR / "sensitivity_sweep.csv", index=False)
    with open(OUT_DIR / "sensitivity_sweep.json", "w") as f:
        json.dump(rows, f, indent=2)
    return df


if __name__ == "__main__":
    run()
