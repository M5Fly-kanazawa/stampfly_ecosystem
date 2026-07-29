#!/usr/bin/env python3
"""
Step 4 (holdout validation) + residual diagnostics (structural-break check) +
delta-method prediction-uncertainty at the realized T range (the honest
alternative to reporting individual alpha/beta, which Step 3's fit showed are
not separable from this data: corr(alpha,beta) ~ -0.995 in every date).

Holdout split: 11 dates chronologically bisected into a FIXED a priori split
(not chosen after looking at results) --
  Fold A = 2026-06-14,18,19,20,21,22 (6 dates, earlier)
  Fold B = 2026-06-24,27,29 + 2026-07-17,18 (5 dates, later)
Two-fold cross-validation: fit on A -> evaluate on B's hover segments, and
vice versa.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from fit_vt import fit_lsq, predict_V, invert_T, HOVER_T_PER_MOTOR, MASS_KG, GRAVITY
from fit_vt import ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH, ALPHA_OLD, BETA_OLD, GAMMA_OLD

OUT_DIR = Path(__file__).parent

FOLD_A_DATES = {"20260614", "20260618", "20260619", "20260620", "20260621", "20260622"}
FOLD_B_DATES = {"20260624", "20260627", "20260629", "20260717", "20260718"}


def holdout_eval(qs, hover_segs):
    results = {}
    for train_dates, test_dates, name in [
        (FOLD_A_DATES, FOLD_B_DATES, "train=A(early6)_test=B(late5)"),
        (FOLD_B_DATES, FOLD_A_DATES, "train=B(late5)_test=A(early6)"),
    ]:
        train = qs[qs["date"].astype(str).isin(train_dates)]
        test_hover = hover_segs[hover_segs["date"].astype(str).isin(test_dates)]
        if len(train) < 100 or len(test_hover) == 0:
            results[name] = {"error": "insufficient data", "n_train": len(train), "n_test": len(test_hover)}
            continue
        fit = fit_lsq(train["T_per_motor_N"], train["V_bar"])
        alpha, beta, gamma = fit["alpha"], fit["beta"], fit["gamma"]

        # invert each held-out hover segment's (duty_mean, voltage_mean) -> per-motor T
        V_obs = test_hover["duty_mean"].to_numpy() * test_hover["voltage_mean"].to_numpy()
        T_implied = invert_T(V_obs, alpha, beta, gamma)
        implied_corr = T_implied / HOVER_T_PER_MOTOR

        results[name] = {
            "n_train_samples": int(len(train)),
            "train_dates": sorted(train_dates),
            "test_dates": sorted(test_dates),
            "n_test_hover_segments": int(len(test_hover)),
            "fit_alpha": alpha, "fit_beta": beta, "fit_gamma": gamma,
            "fit_r2": fit["r2"], "fit_cond": fit["cond_number_XtX"],
            "implied_corr_mean": float(np.mean(implied_corr)),
            "implied_corr_std": float(np.std(implied_corr)),
            "implied_corr_min": float(np.min(implied_corr)),
            "implied_corr_max": float(np.max(implied_corr)),
        }
        print(f"{name}: train_n={len(train)} test_hover_segs={len(test_hover)} "
              f"implied_corr={np.mean(implied_corr):.4f}+-{np.std(implied_corr):.4f} "
              f"(range {implied_corr.min():.4f}-{implied_corr.max():.4f})")
    return results


def delta_method_prediction_band(T_query, X_train, alpha, beta, gamma, cov):
    """Var(V_hat(T)) = x(T)^T Cov x(T), x(T)=[T, sqrt(T), 1]."""
    out = []
    for T in T_query:
        x = np.array([T, np.sqrt(max(T, 0)), 1.0])
        var = float(x @ cov @ x)
        v_hat = alpha * T + beta * np.sqrt(max(T, 0)) + gamma
        out.append({"T": float(T), "V_hat": float(v_hat), "V_hat_se": float(np.sqrt(max(var, 0)))})
    return out


def fit_lsq_with_cov(T, V):
    """Like fit_lsq but also returns the 3x3 covariance matrix (needed for the
    delta-method prediction band -- fit_lsq only returns marginal SEs)."""
    X = np.column_stack([T, np.sqrt(np.clip(T, 0, None)), np.ones_like(T)])
    y = np.asarray(V, dtype=float)
    n, k = X.shape
    beta_hat, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta_hat
    dof = max(n - k, 1)
    sigma2 = float(np.sum(resid ** 2) / dof)
    XtX_inv = np.linalg.inv(X.T @ X)
    cov = sigma2 * XtX_inv
    return beta_hat, cov


def residual_diagnostics(qs):
    """Structural-break check: does the residual of the pooled fit correlate
    with duty, date, or Vbat beyond what T/sqrt(T) already explain? A nonzero
    trend here signals the 3-parameter alpha*T+beta*sqrt(T)+gamma model is
    missing a mechanism (e.g. voltage-dependent internal resistance drop not
    captured by the steady-state V-omega curve, or motor-to-motor asymmetry)."""
    alpha, beta, gamma = qs.attrs["alpha"], qs.attrs["beta"], qs.attrs["gamma"]
    V_pred = predict_V(qs["T_per_motor_N"], alpha, beta, gamma)
    resid = qs["V_bar"] - V_pred

    out = {
        "resid_vs_duty_corr": float(np.corrcoef(qs["duty_mean"], resid)[0, 1]),
        "resid_vs_voltage_corr": float(np.corrcoef(qs["voltage"], resid)[0, 1]),
        "resid_vs_tilt_corr": float(np.corrcoef(qs["tilt_deg"], resid)[0, 1]),
        "resid_vs_motor_spread_corr": float(np.corrcoef(qs["duty_std_across_motors"], resid)[0, 1]),
        "resid_mean": float(resid.mean()),
        "resid_std": float(resid.std()),
    }
    by_date = qs.groupby("date").apply(
        lambda g: pd.Series({"resid_mean": (g["V_bar"] - predict_V(g["T_per_motor_N"], alpha, beta, gamma)).mean(),
                              "resid_std": (g["V_bar"] - predict_V(g["T_per_motor_N"], alpha, beta, gamma)).std(),
                              "n": len(g)}),
        include_groups=False,
    )
    print("\n=== Residual diagnostics (pooled fit) ===")
    print(json.dumps(out, indent=2))
    print("\nresidual by date:")
    print(by_date)
    return out, by_date


def main():
    qs = pd.read_csv(OUT_DIR / "quasistatic_samples.csv")
    hover_segs = pd.read_csv(OUT_DIR / "step1_hover_segments.csv")

    with open(OUT_DIR / "fit_results.json") as f:
        fit_results = json.load(f)
    pooled = fit_results["pooled"]
    qs.attrs["alpha"] = pooled["alpha"]
    qs.attrs["beta"] = pooled["beta"]
    qs.attrs["gamma"] = pooled["gamma"]

    print("=== HOLDOUT VALIDATION (2-fold, chronological date split) ===")
    holdout = holdout_eval(qs, hover_segs)

    resid_out, resid_by_date = residual_diagnostics(qs)
    resid_by_date.to_csv(OUT_DIR / "residual_by_date.csv")

    # Delta-method prediction band at realized T range for pooled fit
    beta_hat, cov = fit_lsq_with_cov(qs["T_per_motor_N"], qs["V_bar"])
    T_query = [pooled["T_p5"], pooled["T_p50"], pooled["T_p95"], HOVER_T_PER_MOTOR]
    band = delta_method_prediction_band(T_query, None, *beta_hat, cov)
    print("\n=== Delta-method V(T) prediction band (pooled fit, within realized T range) ===")
    for b in band:
        print(b)

    # Same band for bench/old (no uncertainty, deterministic theory curves) at
    # the same T points, for direct gap-at-operating-point comparison.
    bench_old_at_T = []
    for T in T_query:
        bench_old_at_T.append({
            "T": T,
            "flight_pooled": float(predict_V(T, pooled["alpha"], pooled["beta"], pooled["gamma"])),
            "bench": float(predict_V(T, ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH)),
            "old": float(predict_V(T, ALPHA_OLD, BETA_OLD, GAMMA_OLD)),
        })
    print("\n=== V(T) comparison: flight-fit vs bench vs old, at realized-range T points ===")
    for row in bench_old_at_T:
        gap_bench = (row["flight_pooled"] - row["bench"]) / row["bench"] * 100
        gap_old = (row["flight_pooled"] - row["old"]) / row["old"] * 100
        print(f"T={row['T']:.5f}N: flight={row['flight_pooled']:.4f}V bench={row['bench']:.4f}V "
              f"({gap_bench:+.2f}%) old={row['old']:.4f}V ({gap_old:+.2f}%)")

    out = {
        "holdout": holdout,
        "residual_diagnostics": resid_out,
        "delta_method_prediction_band": band,
        "bench_old_comparison_at_T": bench_old_at_T,
    }
    with open(OUT_DIR / "holdout_and_diagnostics_results.json", "w") as f:
        json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
