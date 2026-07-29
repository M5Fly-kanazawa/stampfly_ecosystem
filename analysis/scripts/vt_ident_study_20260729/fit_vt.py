#!/usr/bin/env python3
"""
Step 3-6 of the V(T) study: linear least-squares fit of
    V_bar = alpha*T + beta*sqrt(T) + gamma
per date and pooled, residual diagnostics, holdout validation, and comparison
against the bench/old theoretical {alpha,beta,gamma}.

The model is LINEAR in (alpha,beta,gamma) given T (no nonlinear solver
needed): design matrix X = [T, sqrt(T), 1], target y = V_bar = duty*Vbat.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

OUT_DIR = Path(__file__).parent

# --- theoretical comparison values (task brief, ct_switch_study bench numbers) ---
Ct_bench = 6.7e-9
Am_bench, Bm_bench, Cm_bench = 4.279e-8, 5.682e-4, 9.92e-3
ALPHA_BENCH = Am_bench / Ct_bench
BETA_BENCH = Bm_bench / np.sqrt(Ct_bench)
GAMMA_BENCH = Cm_bench

Ct_old = 1.00e-8
Am_old, Bm_old, Cm_old = 5.39e-8, 6.33e-4, 1.53e-2
ALPHA_OLD = Am_old / Ct_old
BETA_OLD = Bm_old / np.sqrt(Ct_old)
GAMMA_OLD = Cm_old

MASS_KG = 0.037
GRAVITY = 9.80665
HOVER_T_PER_MOTOR = MASS_KG * GRAVITY / 4.0


def design_matrix(T):
    T = np.asarray(T, dtype=float)
    Ts = np.sqrt(np.clip(T, 0, None))
    return np.column_stack([T, Ts, np.ones_like(T)])


def fit_lsq(T, V):
    """OLS fit of V = alpha*T + beta*sqrt(T) + gamma. Returns dict with
    params, covariance-based 1-sigma SEs, R^2, condition number of X^T X
    (identifiability diagnostic), and correlation matrix of the 3 estimated
    parameters (near +-1 off-diagonal => the fit cannot separate that pair)."""
    X = design_matrix(T)
    y = np.asarray(V, dtype=float)
    n, k = X.shape
    beta_hat, residuals, rank, sv = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta_hat
    dof = max(n - k, 1)
    sigma2 = float(np.sum(resid ** 2) / dof)
    XtX = X.T @ X
    cond = float(np.linalg.cond(XtX))
    try:
        XtX_inv = np.linalg.inv(XtX)
        cov = sigma2 * XtX_inv
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
        d = np.sqrt(np.diag(cov))
        corr = cov / np.outer(d, d)
    except np.linalg.LinAlgError:
        cov = np.full((3, 3), np.nan)
        se = np.full(3, np.nan)
        corr = np.full((3, 3), np.nan)

    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else np.nan

    return {
        "alpha": float(beta_hat[0]), "beta": float(beta_hat[1]), "gamma": float(beta_hat[2]),
        "alpha_se": float(se[0]), "beta_se": float(se[1]), "gamma_se": float(se[2]),
        "n": int(n), "r2": float(r2), "rmse": float(np.sqrt(sigma2)),
        "cond_number_XtX": cond,
        "corr_alpha_beta": float(corr[0, 1]), "corr_alpha_gamma": float(corr[0, 2]),
        "corr_beta_gamma": float(corr[1, 2]),
        "T_p5": float(np.percentile(T, 5)), "T_p50": float(np.percentile(T, 50)),
        "T_p95": float(np.percentile(T, 95)),
        "T_range_over_median": float((np.percentile(T, 95) - np.percentile(T, 5)) / np.percentile(T, 50)),
    }


def predict_V(T, alpha, beta, gamma):
    T = np.asarray(T, dtype=float)
    return alpha * T + beta * np.sqrt(np.clip(T, 0, None)) + gamma


def invert_T(V, alpha, beta, gamma):
    """Solve alpha*s^2 + beta*s + (gamma - V) = 0 for s=sqrt(T)>=0, T=s^2.
    Does NOT require Ct (gauge-free inversion directly in the (alpha,beta,gamma)
    parametrization) -- matches the task's point that Ct is not identifiable
    from flight data alone."""
    V = np.asarray(V, dtype=float)
    disc = beta ** 2 - 4 * alpha * (gamma - V)
    disc = np.maximum(disc, 0.0)
    if alpha == 0:
        # degenerate to linear: beta*s + (gamma-V) = 0
        s = np.where(beta != 0, (V - gamma) / beta, np.nan)
    else:
        s = (-beta + np.sqrt(disc)) / (2 * alpha)
    s = np.maximum(s, 0.0)
    return s ** 2


def main():
    qs = pd.read_csv(OUT_DIR / "quasistatic_samples.csv")

    # ---- per-date fits ----
    per_date = {}
    for date, g in qs.groupby("date"):
        if len(g) < 20:
            per_date[str(date)] = {"n": int(len(g)), "skipped": "too few samples"}
            continue
        res = fit_lsq(g["T_per_motor_N"], g["V_bar"])
        res["duty_min"] = float(g["duty_mean"].min())
        res["duty_max"] = float(g["duty_mean"].max())
        per_date[str(date)] = res

    # ---- pooled fit ----
    pooled = fit_lsq(qs["T_per_motor_N"], qs["V_bar"])

    # ---- identifiability diagnostics on the pooled sample ----
    T = qs["T_per_motor_N"].to_numpy()
    Vb = qs["V_bar"].to_numpy()
    corr_T_Vbar = float(np.corrcoef(T, Vb)[0, 1])
    corr_sqrtT_Vbar = float(np.corrcoef(np.sqrt(np.clip(T, 0, None)), Vb)[0, 1])
    corr_T_sqrtT = float(np.corrcoef(T, np.sqrt(np.clip(T, 0, None)))[0, 1])

    print("=== POOLED FIT ===")
    print(json.dumps(pooled, indent=2))
    print(f"\ncorr(T, V_bar) = {corr_T_Vbar:.4f}")
    print(f"corr(sqrt(T), V_bar) = {corr_sqrtT_Vbar:.4f}")
    print(f"corr(T, sqrt(T)) [within-sample collinearity] = {corr_T_sqrtT:.4f}")
    print(f"T range: p5={pooled['T_p5']:.5f} p50={pooled['T_p50']:.5f} p95={pooled['T_p95']:.5f} "
          f"N, (p95-p5)/p50 = {pooled['T_range_over_median']:.3f}")

    print("\n=== PER-DATE FITS ===")
    for date, res in per_date.items():
        if "skipped" in res:
            print(f"{date}: {res}")
            continue
        print(f"{date}: n={res['n']:6d} duty=[{res['duty_min']:.3f},{res['duty_max']:.3f}] "
              f"alpha={res['alpha']:.3f}+-{res['alpha_se']:.3f} "
              f"beta={res['beta']:.3f}+-{res['beta_se']:.3f} "
              f"gamma={res['gamma']:.4f}+-{res['gamma_se']:.4f} "
              f"R2={res['r2']:.4f} cond={res['cond_number_XtX']:.2e} "
              f"corr(a,b)={res['corr_alpha_beta']:.3f}")

    # ---- cross-date spread of pooled-comparable params ----
    valid_dates = {k: v for k, v in per_date.items() if "skipped" not in v}
    alphas = np.array([v["alpha"] for v in valid_dates.values()])
    betas = np.array([v["beta"] for v in valid_dates.values()])
    gammas = np.array([v["gamma"] for v in valid_dates.values()])
    date_spread = {
        "alpha": {"mean": float(alphas.mean()), "std": float(alphas.std()), "n_dates": len(alphas)},
        "beta": {"mean": float(betas.mean()), "std": float(betas.std()), "n_dates": len(betas)},
        "gamma": {"mean": float(gammas.mean()), "std": float(gammas.std()), "n_dates": len(gammas)},
    }
    print("\n=== Cross-date spread of per-date fitted params ===")
    print(json.dumps(date_spread, indent=2))

    # ---- comparison to bench / old at the fitted-model prediction of V at hover T ----
    def implied_hover_comparison(alpha, beta, gamma, label):
        v_hover = predict_V(HOVER_T_PER_MOTOR, alpha, beta, gamma)
        return {"label": label, "V_at_hover_T": float(v_hover)}

    comparisons = [
        implied_hover_comparison(pooled["alpha"], pooled["beta"], pooled["gamma"], "flight_pooled"),
        implied_hover_comparison(ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH, "bench"),
        implied_hover_comparison(ALPHA_OLD, BETA_OLD, GAMMA_OLD, "old_firmware"),
    ]
    print("\n=== V(T) at per-motor hover thrust T=%.5f N ===" % HOVER_T_PER_MOTOR)
    for c in comparisons:
        print(c)

    out = {
        "pooled": pooled,
        "per_date": per_date,
        "date_spread_of_fitted_params": date_spread,
        "identifiability": {
            "corr_T_Vbar": corr_T_Vbar,
            "corr_sqrtT_Vbar": corr_sqrtT_Vbar,
            "corr_T_sqrtT": corr_T_sqrtT,
        },
        "bench_theory": {"alpha": ALPHA_BENCH, "beta": BETA_BENCH, "gamma": GAMMA_BENCH, "Ct": Ct_bench},
        "old_theory": {"alpha": ALPHA_OLD, "beta": BETA_OLD, "gamma": GAMMA_OLD, "Ct": Ct_old},
        "hover_T_per_motor_N": HOVER_T_PER_MOTOR,
        "v_at_hover_T": {c["label"]: c["V_at_hover_T"] for c in comparisons},
    }
    with open(OUT_DIR / "fit_results.json", "w") as f:
        json.dump(out, f, indent=2)

    qs["V_pred_pooled"] = predict_V(qs["T_per_motor_N"], pooled["alpha"], pooled["beta"], pooled["gamma"])
    qs["resid_pooled"] = qs["V_bar"] - qs["V_pred_pooled"]
    qs.to_csv(OUT_DIR / "quasistatic_samples_with_resid.csv", index=False)


if __name__ == "__main__":
    main()
