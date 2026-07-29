#!/usr/bin/env python3
"""
H1 (thermal, function of elapsed flight time) vs H2 (function of Vbat alone)
model comparison on the fine-grained hover time series.

H1: V_app(t) = V0 + dV * (1 - exp(-t/tau))         t = time since this takeoff
H2: V_app(Vbat) = a + b * Vbat                       (linear, same functional
                                                       form as the 85-segment
                                                       pooled regression)

For each flight (file, episode) with enough points we fit both, report
in-sample RMSE/R2/AICc, and the within-flight corr(t, Vbat) (the collinearity
that makes single-flight discrimination hard). We also do the multivariate
fit V_app = a0 + b1*Vbat + c1*t to show the condition-number blow-up directly.

熱仮説(H1, 経過時間の飽和指数)と電圧仮説(H2, Vbatの1次関数)を1フライトごとに
フィットし、RMSE/R2/AICcを比較する。あわせて corr(t,Vbat) と重回帰の条件数を
報告し、単一フライト内では時間と電圧がどれだけ共線かを直接示す。
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

CSV = Path(__file__).parent / "hover_timeseries.csv"

# Physical constants quoted in the task brief (bench / theory values), used
# only for the Rm back-out in section 4 -- NOT used anywhere in the H1/H2
# model comparison itself (that part is purely a fit to V_app, t, Vbat).
KM = 5.682e-4      # back-EMF constant [V*s/rad]
CQ = 4.10e-11      # torque coefficient [Nm/(rad/s)^2]
QF = 9.507e-6      # Coulomb friction torque [Nm]
OMEGA_H = 3670.0   # theoretical hover omega [rad/s]
I_HOVER = (CQ * OMEGA_H**2 + QF) / KM  # steady hover current per motor [A]


def h1_model(t, V0, dV, tau):
    tau = max(tau, 1e-3)
    return V0 + dV * (1.0 - np.exp(-t / tau))


def fit_h1(t, v):
    # Initial guess: V0 = first value, dV = span, tau = half the time range.
    p0 = [v[0], max(v[-1] - v[0], 1e-3), max((t[-1] - t[0]) / 3.0, 1.0)]
    bounds = ([1.5, -1.0, 0.5], [3.5, 1.0, 600.0])
    try:
        popt, _ = curve_fit(h1_model, t, v, p0=p0, bounds=bounds, maxfev=20000)
    except Exception as e:
        return None, None, str(e)
    pred = h1_model(t, *popt)
    resid = v - pred
    return popt, pred, resid


def fit_h2(vbat, v):
    A = np.vstack([np.ones_like(vbat), vbat]).T
    coef, *_ = np.linalg.lstsq(A, v, rcond=None)
    pred = A @ coef
    resid = v - pred
    return coef, pred, resid


def fit_linear_time(t, v):
    """H1' -- linear-in-time model V = c0 + k*t. Same parameter count (2) as
    H2, so RMSE/AICc are directly comparable without an extra-parameter
    penalty confound. This is also what the saturating exponential H1
    degenerates to when tau >> flight duration (data can't see the
    curvature), which turns out to be the common case here (see tau hitting
    the fit's upper bound in most flights)."""
    A = np.vstack([np.ones_like(t), t]).T
    coef, *_ = np.linalg.lstsq(A, v, rcond=None)
    pred = A @ coef
    resid = v - pred
    return coef, pred, resid


def aicc(resid, k, n):
    rss = float(np.sum(resid**2))
    if rss <= 0 or n <= k + 1:
        return np.nan
    aic = n * np.log(rss / n) + 2 * k
    return aic + (2 * k * (k + 1)) / max(n - k - 1, 1)


def analyze_flight(g: pd.DataFrame, min_points: int = 5) -> dict:
    g = g.sort_values("t_since_takeoff_s")
    t = g["t_since_takeoff_s"].to_numpy()
    vbat = g["voltage_mean"].to_numpy()
    v = g["Vapp_mean"].to_numpy()
    n = len(g)
    out = {"file": g["file"].iloc[0], "episode_id": int(g["episode_id"].iloc[0]), "n": n}
    if n < min_points:
        out["note"] = f"n<{min_points}, skipped"
        return out

    corr_t_vbat = float(np.corrcoef(t, vbat)[0, 1])
    out["corr_t_vbat"] = corr_t_vbat

    popt, pred1, resid1 = fit_h1(t, v)
    if popt is not None:
        rmse1 = float(np.sqrt(np.mean(resid1**2)))
        out.update({
            "h1_V0": popt[0], "h1_dV": popt[1], "h1_tau_s": popt[2],
            "h1_rmse": rmse1, "h1_aicc": aicc(resid1, 3, n),
        })
    else:
        out["h1_fit_error"] = resid1  # error string stashed here

    coef2, pred2, resid2 = fit_h2(vbat, v)
    rmse2 = float(np.sqrt(np.mean(resid2**2)))
    out.update({
        "h2_a": float(coef2[0]), "h2_b": float(coef2[1]),
        "h2_rmse": rmse2, "h2_aicc": aicc(resid2, 2, n),
    })

    # H1' linear-in-time, same #params (2) as H2 -- the fair apples-to-apples
    # comparison (H1 above has 3 params so its lower RMSE is partly just
    # extra flexibility; AICc corrects for that but a matched-df model is a
    # cleaner story to report).
    coefL, predL, residL = fit_linear_time(t, v)
    rmseL = float(np.sqrt(np.mean(residL**2)))
    out.update({
        "h1lin_c0": float(coefL[0]), "h1lin_k": float(coefL[1]),
        "h1lin_rmse": rmseL, "h1lin_aicc": aicc(residL, 2, n),
    })

    # Multivariate: V = a0 + b1*Vbat + c1*t  -- condition number / VIF check.
    A3 = np.vstack([np.ones_like(t), vbat, t]).T
    # standardize columns 1,2 for a meaningful condition number
    A3s = A3.copy()
    for j in (1, 2):
        A3s[:, j] = (A3[:, j] - A3[:, j].mean()) / (A3[:, j].std() + 1e-12)
    cond = float(np.linalg.cond(A3s))
    coef3, *_ = np.linalg.lstsq(A3, v, rcond=None)
    pred3 = A3 @ coef3
    resid3 = v - pred3
    out.update({
        "mv_b_vbat": float(coef3[1]), "mv_c_t": float(coef3[2]),
        "mv_cond_number": cond, "mv_rmse": float(np.sqrt(np.mean(resid3**2))),
    })
    return out


def per_motor_h1(g: pd.DataFrame) -> dict:
    """Robust per-motor diagnostic: simple linear-in-time OLS slope (2 free
    params, always well-conditioned with n>=8), NOT the 3-param saturating
    exponential -- that one turned out to be degenerate per-motor (V0 pinned
    at its bound, tau pinned at 600s for several motors/flights with only
    13-27 points split 4 ways), so its dV/tau numbers are not trustworthy
    at this sample size. The slope sign/magnitude comparison across the 4
    motors is the falsifiable H1 prediction we can actually test: thermal
    self-heating should raise ~all 4 motors' V_app with the same sign and
    a similar order of magnitude; a single-motor outlier points at a motor-
    specific (non-thermal, e.g. mechanical/electrical fault) effect instead."""
    g = g.sort_values("t_since_takeoff_s")
    t = g["t_since_takeoff_s"].to_numpy()
    out = {}
    for i in range(1, 5):
        v = g[f"Vapp_m{i}_mean"].to_numpy()
        coef, pred, resid = fit_linear_time(t, v)
        out[f"m{i}_c0"] = float(coef[0])
        out[f"m{i}_k"] = float(coef[1])  # V/s slope
        out[f"m{i}_rmse"] = float(np.sqrt(np.mean(resid**2)))
    return out


def main():
    df = pd.read_csv(CSV)
    results = []
    for (f, ep), g in df.groupby(["file", "episode_id"]):
        results.append(analyze_flight(g))
    res_df = pd.DataFrame(results)
    pd.set_option("display.width", 220)
    pd.set_option("display.max_columns", 30)
    print("=== per-flight H1 vs H2 comparison (n>=5 points) ===")
    cols = ["file", "episode_id", "n", "corr_t_vbat",
            "h1_V0", "h1_dV", "h1_tau_s", "h1_rmse", "h1_aicc",
            "h1lin_c0", "h1lin_k", "h1lin_rmse", "h1lin_aicc",
            "h2_a", "h2_b", "h2_rmse", "h2_aicc",
            "mv_b_vbat", "mv_c_t", "mv_cond_number", "mv_rmse"]
    cols = [c for c in cols if c in res_df.columns]
    print(res_df[cols].to_string(index=False))
    res_df.to_csv(Path(__file__).parent / "fit_results_per_flight.csv", index=False)

    print("\n=== per-motor H1 fits (flights with n>=8 points) ===")
    for (f, ep), g in df.groupby(["file", "episode_id"]):
        if len(g) < 8:
            continue
        pm = per_motor_h1(g)
        print(f"{f} ep{ep} (n={len(g)}):")
        for i in range(1, 5):
            if f"m{i}_k" in pm:
                print(f"  m{i}: c0={pm[f'm{i}_c0']:.4f} slope_k={pm[f'm{i}_k']*1000:+.3f} mV/s "
                      f"rmse={pm[f'm{i}_rmse']:.4f}")

    print(f"\nI_hover (theory) = {I_HOVER:.4f} A/motor, Km*omega_h = {KM*OMEGA_H:.4f} V")
    print("saved: fit_results_per_flight.csv")


if __name__ == "__main__":
    main()
