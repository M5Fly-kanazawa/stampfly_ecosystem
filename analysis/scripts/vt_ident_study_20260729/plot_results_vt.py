#!/usr/bin/env python3
"""Plots for the V(T) identification study (Step 7 of the task brief)."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from fit_vt import (predict_V, HOVER_T_PER_MOTOR, ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH,
                     ALPHA_OLD, BETA_OLD, GAMMA_OLD)

OUT_DIR = Path(__file__).parent


def plot_scatter_fit(qs, pooled):
    fig, ax = plt.subplots(figsize=(9, 6))
    # Subsample for scatter (77k points is too dense to render usefully)
    rng = np.random.default_rng(0)
    idx = rng.choice(len(qs), size=min(6000, len(qs)), replace=False)
    sub = qs.iloc[idx]
    sc = ax.scatter(sub["T_per_motor_N"], sub["V_bar"], s=3, alpha=0.15, c=sub["voltage"],
                     cmap="viridis", label="quasi-static samples (colored by Vbat)")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Vbat [V]")

    T_line = np.linspace(qs["T_per_motor_N"].quantile(0.001), qs["T_per_motor_N"].quantile(0.999), 200)
    ax.plot(T_line, predict_V(T_line, pooled["alpha"], pooled["beta"], pooled["gamma"]),
            "r-", lw=2.5, label="flight pooled fit")
    ax.plot(T_line, predict_V(T_line, ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH),
            "b--", lw=2, label="bench theory")
    ax.plot(T_line, predict_V(T_line, ALPHA_OLD, BETA_OLD, GAMMA_OLD),
            "g:", lw=2, label="old firmware theory")
    ax.axvline(HOVER_T_PER_MOTOR, color="k", ls="-", lw=0.8, alpha=0.5)
    ax.text(HOVER_T_PER_MOTOR, ax.get_ylim()[0], " hover T/motor", rotation=90,
            va="bottom", ha="right", fontsize=8, alpha=0.7)
    ax.set_xlabel("Reconstructed per-motor thrust T (from IMU specific force) [N]")
    ax.set_ylabel("V_bar = duty_mean * Vbat [V]")
    ax.set_title("V(T): flight-fit vs bench vs old firmware theory\n"
                  "(realized T range is narrow -- see identifiability caveat in proposal.md)")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "vt_scatter_fit.png", dpi=150)
    plt.close(fig)


def plot_per_date_params(per_date):
    dates = [d for d, v in per_date.items() if "skipped" not in v]
    alphas = [per_date[d]["alpha"] for d in dates]
    alpha_se = [per_date[d]["alpha_se"] for d in dates]
    betas = [per_date[d]["beta"] for d in dates]
    beta_se = [per_date[d]["beta_se"] for d in dates]
    gammas = [per_date[d]["gamma"] for d in dates]
    gamma_se = [per_date[d]["gamma_se"] for d in dates]

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    for ax, vals, se, name, bench, old in [
        (axes[0], alphas, alpha_se, "alpha [V/N]", ALPHA_BENCH, ALPHA_OLD),
        (axes[1], betas, beta_se, "beta [V/sqrt(N)]", BETA_BENCH, BETA_OLD),
        (axes[2], gammas, gamma_se, "gamma [V]", GAMMA_BENCH, GAMMA_OLD),
    ]:
        ax.errorbar(range(len(dates)), vals, yerr=se, fmt="o", capsize=4, label="per-date fit +-1SE")
        ax.axhline(bench, color="b", ls="--", label="bench")
        ax.axhline(old, color="g", ls=":", label="old")
        ax.set_ylabel(name)
        ax.legend(fontsize=8, loc="best")
        ax.grid(alpha=0.3)
    axes[-1].set_xticks(range(len(dates)))
    axes[-1].set_xticklabels(dates, rotation=45, ha="right")
    axes[0].set_title("Per-date fitted (alpha,beta,gamma) -- NOTE: individually unidentifiable\n"
                       "(corr(alpha,beta) ~ -0.995-1.000 every date; huge SEs/date-to-date swings "
                       "are the EXPECTED signature of collinearity, not noise to smooth over)")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "per_date_params.png", dpi=150)
    plt.close(fig)


def plot_per_date_v_at_hover(per_date, pooled):
    """The actually-meaningful, well-conditioned per-date quantity: V predicted
    at T=hover using each date's own fit (interpolation, not extrapolation,
    since T_p50 per date ~= hover T for all dates -- see per-date duty ranges)."""
    dates = [d for d, v in per_date.items() if "skipped" not in v]
    v_hover = [predict_V(HOVER_T_PER_MOTOR, per_date[d]["alpha"], per_date[d]["beta"], per_date[d]["gamma"])
               for d in dates]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(range(len(dates)), v_hover, color="tab:orange", alpha=0.8, label="per-date V(hover T)")
    ax.axhline(predict_V(HOVER_T_PER_MOTOR, ALPHA_BENCH, BETA_BENCH, GAMMA_BENCH),
               color="b", ls="--", label="bench")
    ax.axhline(predict_V(HOVER_T_PER_MOTOR, ALPHA_OLD, BETA_OLD, GAMMA_OLD),
               color="g", ls=":", label="old")
    ax.axhline(predict_V(HOVER_T_PER_MOTOR, pooled["alpha"], pooled["beta"], pooled["gamma"]),
               color="r", ls="-", lw=2, label="pooled flight fit")
    ax.set_xticks(range(len(dates)))
    ax.set_xticklabels(dates, rotation=45, ha="right")
    ax.set_ylabel("V at per-motor hover thrust T=%.5fN [V]" % HOVER_T_PER_MOTOR)
    ax.set_title("Per-date V(T=hover) -- interpolation-level quantity, well-conditioned\n"
                 "(unlike raw alpha/beta/gamma; this is the number that matters for hover.thrust_corr)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "per_date_v_at_hover.png", dpi=150)
    plt.close(fig)


def plot_residuals(qs):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    rng = np.random.default_rng(1)
    idx = rng.choice(len(qs), size=min(8000, len(qs)), replace=False)
    sub = qs.iloc[idx]

    axes[0].scatter(sub["duty_mean"], sub["resid_pooled"], s=3, alpha=0.15)
    axes[0].set_xlabel("duty_mean")
    axes[0].set_ylabel("residual V_bar - V_pred_pooled [V]")
    axes[0].set_title(f"resid vs duty (r={qs['duty_mean'].corr(qs['resid_pooled']):.2f})")
    axes[0].grid(alpha=0.3)
    axes[0].axhline(0, color="k", lw=0.5)

    axes[1].scatter(sub["voltage"], sub["resid_pooled"], s=3, alpha=0.15, color="tab:orange")
    axes[1].set_xlabel("Vbat [V]")
    axes[1].set_title(f"resid vs Vbat (r={qs['voltage'].corr(qs['resid_pooled']):.2f})")
    axes[1].grid(alpha=0.3)
    axes[1].axhline(0, color="k", lw=0.5)

    dates = sorted(qs["date"].astype(str).unique())
    data_by_date = [qs.loc[qs["date"].astype(str) == d, "resid_pooled"].to_numpy() for d in dates]
    axes[2].boxplot(data_by_date, labels=dates, showfliers=False)
    axes[2].set_xticklabels(dates, rotation=45, ha="right")
    axes[2].set_title("resid by date (systematic date-to-date offsets)")
    axes[2].axhline(0, color="k", lw=0.5)
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "residual_diagnostics.png", dpi=150)
    plt.close(fig)


def plot_holdout(holdout):
    fig, ax = plt.subplots(figsize=(6, 5))
    labels, means, stds = [], [], []
    for name, r in holdout.items():
        if "error" in r:
            continue
        labels.append(name.replace("_", "\n"))
        means.append(r["implied_corr_mean"])
        stds.append(r["implied_corr_std"])
    ax.bar(range(len(labels)), means, yerr=stds, capsize=6, color="tab:purple", alpha=0.8)
    ax.axhline(1.0, color="k", ls="--", label="corr=1.0 (no correction needed)")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("implied_corr on held-out dates' hover segments")
    ax.set_title("2-fold holdout: train-fit params applied to OTHER half's hover duty\n"
                  "(wide swing 0.84-1.21 = the fit does not generalize across sessions)")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "holdout_validation.png", dpi=150)
    plt.close(fig)


def main():
    qs = pd.read_csv(OUT_DIR / "quasistatic_samples_with_resid.csv")
    with open(OUT_DIR / "fit_results.json") as f:
        fit_results = json.load(f)
    with open(OUT_DIR / "holdout_and_diagnostics_results.json") as f:
        hd = json.load(f)

    plot_scatter_fit(qs, fit_results["pooled"])
    plot_per_date_params(fit_results["per_date"])
    plot_per_date_v_at_hover(fit_results["per_date"], fit_results["pooled"])
    plot_residuals(qs)
    plot_holdout(hd["holdout"])
    print("Wrote: vt_scatter_fit.png, per_date_params.png, per_date_v_at_hover.png, "
          "residual_diagnostics.png, holdout_validation.png")


if __name__ == "__main__":
    main()
