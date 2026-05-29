#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# Generate validation plots for the integrated SIL (M1-M3).
# 統合SIL(M1-M3)の検証プロットを生成する。
#
# Runs ./integrated_sil for each scenario, parses the CSV from stdout, and
# writes figures to docs/assets/. Deterministic (fixed seed in the SIL).
# 各シナリオで ./integrated_sil を実行し stdout の CSV を解析、docs/assets/ に
# 図を出力する。SIL は固定シードで決定論的。
#
# Usage: python3 plot_sil_results.py   (run from simulator/sil/)

import io
import os
import re
import subprocess

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCENARIOS = ["nominal", "comm_lost", "impact", "low_battery", "eskf_diverged"]

STATE_NAMES = {0: "INIT", 1: "IDLE_GND", 2: "IDLE_HELD", 3: "ARMED_GND",
               4: "TAKEOFF", 5: "FLYING", 6: "LANDING"}
ALERT_NAMES = {1: "COMM_LOST", 2: "IMPACT", 3: "LOW_BATT", 5: "ESKF_DIV", 6: "GYRO"}

COLORS = {"nominal": "#1f77b4", "comm_lost": "#ff7f0e", "impact": "#d62728",
          "low_battery": "#9467bd", "eskf_diverged": "#2ca02c"}

OUT_DIR = "docs/assets"


def run_scenario(scn):
    """Run the SIL and return the CSV as a structured numpy array."""
    args = ["./integrated_sil"]
    if scn != "nominal":
        args += ["--scenario", scn]
    res = subprocess.run(args, capture_output=True, text=True)
    data = np.genfromtxt(io.StringIO(res.stdout), delimiter=",", names=True)
    return data


def first_alert(data):
    """Return (time, alert_code) of the first non-zero alert, or None."""
    at = data["alert_type"].astype(int)
    nz = np.nonzero(at > 0)[0]
    if len(nz) == 0:
        return None
    i = nz[0]
    return float(data["time"][i]), int(at[i])


def plot_nominal_detail(data):
    """Figure 1: altitude + attitude + state timeline for nominal flight."""
    t = data["time"]
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)

    ax[0].plot(t, -data["true_z"], label="true altitude", color="#1f77b4")
    ax[0].plot(t, -data["eskf_z"], label="ESKF altitude", color="#ff7f0e", ls="--")
    ax[0].set_ylabel("Altitude [m]")
    ax[0].legend(loc="upper right")
    ax[0].grid(alpha=0.3)
    ax[0].set_title("Nominal flight — altitude / attitude / flight state")

    ax[1].plot(t, data["roll"], label="roll", color="#2ca02c")
    ax[1].plot(t, data["pitch"], label="pitch", color="#d62728")
    ax[1].set_ylabel("Attitude [deg]")
    ax[1].legend(loc="upper right")
    ax[1].grid(alpha=0.3)

    ax[2].step(t, data["flight_state"], where="post", color="#9467bd")
    ax[2].set_yticks(list(STATE_NAMES.keys()))
    ax[2].set_yticklabels(list(STATE_NAMES.values()))
    ax[2].set_ylabel("Flight state")
    ax[2].set_xlabel("Time [s]")
    ax[2].grid(alpha=0.3)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "fig1_nominal_detail.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_state_timelines(datasets):
    """Figure 2: flight-state timeline for every scenario, with alert markers."""
    fig, axes = plt.subplots(len(SCENARIOS), 1, figsize=(9, 9), sharex=True)
    for ax, scn in zip(axes, SCENARIOS):
        d = datasets[scn]
        t = d["time"]
        ax.step(t, d["flight_state"], where="post", color=COLORS[scn])
        ax.set_yticks(list(STATE_NAMES.keys()))
        ax.set_yticklabels(list(STATE_NAMES.values()), fontsize=7)
        ax.set_ylabel(scn, fontsize=9)
        ax.grid(alpha=0.3)
        fa = first_alert(d)
        if fa is not None:
            tt, code = fa
            ax.axvline(tt, color="k", ls=":", lw=1)
            ax.annotate(ALERT_NAMES.get(code, "?"), xy=(tt, 5.2),
                        fontsize=7, ha="left",
                        xytext=(tt + 0.2, 5.6))
    axes[0].set_title("Flight-state timeline per scenario (dotted = alert raised)")
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "fig2_state_timelines.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_altitude_overlay(datasets):
    """Figure 3: altitude profile of every scenario overlaid."""
    fig, ax = plt.subplots(figsize=(9, 5))
    for scn in SCENARIOS:
        d = datasets[scn]
        ax.plot(d["time"], -d["true_z"], label=scn, color=COLORS[scn])
        fa = first_alert(d)
        if fa is not None:
            tt, _ = fa
            idx = int(np.argmin(np.abs(d["time"] - tt)))
            ax.plot(tt, -d["true_z"][idx], "o", color=COLORS[scn], ms=7)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude [m]")
    ax.set_title("Altitude profile per scenario (markers = alert raised)")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "fig3_altitude_overlay.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def run_eval(extra_args):
    """Run the SIL and parse roll RMS [deg] from the evaluation summary (stderr)."""
    res = subprocess.run(["./integrated_sil"] + extra_args, capture_output=True, text=True)
    m = re.search(r"roll : RMS=([\d.]+)", res.stderr)
    return float(m.group(1)) if m else float("nan")


def plot_diagnostic_ladder():
    """Figure 4: attitude error vs noise stage (the diagnostic ladder, M5)."""
    stages = ["N0", "N1", "N2"]
    rms = [run_eval(["--feedback", "eskf", "--noise-stage", s]) for s in stages]
    fig, ax = plt.subplots(figsize=(6, 4))
    bars = ax.bar(stages, rms, color=["#2ca02c", "#ff7f0e", "#d62728"])
    for b, v in zip(bars, rms):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.03, f"{v:.2f}", ha="center")
    ax.set_ylabel("roll RMS [deg]")
    ax.set_title("Diagnostic ladder — attitude error vs noise stage (--feedback eskf)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "fig4_diagnostic_ladder.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_ab_sweep():
    """Figure 5: control-design A/B sweeps of LPF alpha and ESKF R (M6)."""
    alphas = [0.32, 0.20, 0.10, 0.05]
    rms_a = [run_eval(["--feedback", "eskf", "--noise-stage", "N2", "--lpf-alpha", str(a)])
             for a in alphas]
    Rs = [0.3, 1.0, 3.0, 10.0]
    rms_r = [run_eval(["--feedback", "eskf", "--noise-stage", "N2", "--accel-noise", str(r)])
             for r in Rs]
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].plot(alphas, rms_a, "o-", color="#1f77b4")
    ax[0].set_xlabel("LPF alpha (smaller = stronger filter)")
    ax[0].set_ylabel("roll RMS [deg]")
    ax[0].set_title("LPF sweep (eskf, N2)")
    ax[0].grid(alpha=0.3)
    ax[0].invert_xaxis()
    ax[1].plot(Rs, rms_r, "s-", color="#9467bd")
    ax[1].set_xlabel("ESKF accel-noise R (larger = trust accel less)")
    ax[1].set_ylabel("roll RMS [deg]")
    ax[1].set_title("ESKF R sweep (eskf, N2)")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "fig5_ab_sweep.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    datasets = {scn: run_scenario(scn) for scn in SCENARIOS}
    p1 = plot_nominal_detail(datasets["nominal"])
    p2 = plot_state_timelines(datasets)
    p3 = plot_altitude_overlay(datasets)
    p4 = plot_diagnostic_ladder()
    p5 = plot_ab_sweep()
    for p in (p1, p2, p3, p4, p5):
        print("wrote", p)


if __name__ == "__main__":
    main()
