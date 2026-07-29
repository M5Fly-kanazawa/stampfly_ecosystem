#!/usr/bin/env python3
"""Figures for the thermal-drift identification study. Reads
hover_timeseries.csv (+ re-derives the pause-pair numbers inline, matching
pause_pair_raw.py's printed output) and saves PNGs into this directory."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

HERE = Path(__file__).parent
df = pd.read_csv(HERE / "hover_timeseries.csv")

PRIMARY = "stampfly_udp_20260627T020137.jsonl"
ANOMALY = "stampfly_udp_20260629T145526.jsonl"
THIRD = "stampfly_udp_20260718T022929.jsonl"

# ---------------------------------------------------------------------------
# Figure 1: 2x2 -- V_app(t)&Vbat(t) for primary + anomaly flights, per-motor
# breakdown for primary, pause-pair reset comparison.
# ---------------------------------------------------------------------------
fig, axs = plt.subplots(2, 2, figsize=(12, 9))

for ax, fname, title in [(axs[0, 0], PRIMARY, "2026-06-27 02:01 (primary, 176s)"),
                          (axs[0, 1], ANOMALY, "2026-06-29 14:55 (anomaly, flat V_app)")]:
    g = df[df.file == fname].sort_values("t_since_takeoff_s")
    ax2 = ax.twinx()
    ax.plot(g.t_since_takeoff_s, g.Vapp_mean, "o-", color="tab:red", label="V_app=duty*Vbat")
    ax2.plot(g.t_since_takeoff_s, g.voltage_mean, "s--", color="tab:blue", alpha=0.6, label="Vbat")
    ax.set_xlabel("time since takeoff [s]")
    ax.set_ylabel("V_app [V]", color="tab:red")
    ax2.set_ylabel("Vbat [V]", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:blue")
    ax.set_title(title)
    ax.grid(alpha=0.3)

# per-motor V_app for primary
g = df[df.file == PRIMARY].sort_values("t_since_takeoff_s")
ax = axs[1, 0]
for i, c in zip(range(1, 5), ["tab:blue", "tab:orange", "tab:green", "tab:purple"]):
    ax.plot(g.t_since_takeoff_s, g[f"Vapp_m{i}_mean"], "o-", ms=3, color=c, label=f"motor {i}")
ax.plot(g.t_since_takeoff_s, g.Vapp_mean, "k-", lw=2, label="4-motor mean")
ax.set_xlabel("time since takeoff [s]")
ax.set_ylabel("V_app,i [V]")
ax.set_title("2026-06-27 02:01: per-motor V_app(t)")
ax.legend(fontsize=8, ncol=2)
ax.grid(alpha=0.3)

# pause-pair reset comparison (hardcoded from pause_pair_raw.py output --
# kept in sync manually; see that script for the underlying computation)
ax = axs[1, 1]
pairs = {
    "2026-06-27\n(37.4s gap)": {"A start (cold)": 2.4328, "A end (pre-gap)": 2.5045, "B start (post-gap)": 2.4536},
    "2026-06-22\n(30.4s gap)": {"A start (cold)": 2.4229, "A end (pre-gap)": 2.4887, "B start (post-gap)": 2.4262},
}
labels = list(pairs.keys())
x = np.arange(len(labels))
width = 0.25
keys = ["A start (cold)", "A end (pre-gap)", "B start (post-gap)"]
colors = ["tab:gray", "tab:red", "tab:green"]
for i, (k, c) in enumerate(zip(keys, colors)):
    vals = [pairs[lab][k] for lab in labels]
    ax.bar(x + (i - 1) * width, vals, width, label=k, color=c)
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylabel("V_app [V]")
ax.set_ylim(2.35, 2.55)
ax.set_title("Ground-idle pause pairs: V_app resets toward cold\n(H2/Vbat-only cannot explain this — Vbat barely moved)")
ax.legend(fontsize=8)
ax.grid(alpha=0.3, axis="y")

fig.tight_layout()
fig.savefig(HERE / "fig1_within_flight_and_pause.png", dpi=140)
print("saved fig1_within_flight_and_pause.png")

# ---------------------------------------------------------------------------
# Figure 2: cross-flight matched-Vbat scatter, colored by file/date.
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(9, 6))
files = sorted(df.file.unique())
cmap = plt.get_cmap("tab20")
for i, f in enumerate(files):
    g = df[df.file == f]
    date = f.split("_")[2][:8]
    marker = "*" if f == ANOMALY else "o"
    size = 90 if f == ANOMALY else 35
    ax.scatter(g.voltage_mean, g.Vapp_mean, color=cmap(i % 20), marker=marker, s=size,
               label=date if f in (PRIMARY, ANOMALY, THIRD) else None, alpha=0.85,
               edgecolors="k" if f == ANOMALY else "none", linewidths=0.8)
ax.set_xlabel("Vbat [V]")
ax.set_ylabel("V_app = duty*Vbat [V]")
ax.set_title("Cross-flight V_app at matched Vbat\n(2026-06-29, star markers, sits ~0.2-0.25V above every other flight)")
ax.invert_xaxis()
ax.grid(alpha=0.3)
ax.legend(fontsize=8, title="highlighted flights")
fig.tight_layout()
fig.savefig(HERE / "fig2_cross_flight_matched_vbat.png", dpi=140)
print("saved fig2_cross_flight_matched_vbat.png")

# ---------------------------------------------------------------------------
# Figure 3: Rm_cold vs Rm_hot summary.
# ---------------------------------------------------------------------------
import json
with open(HERE / "rm_estimate_results.json") as fh:
    rm = json.load(fh)

fig, ax = plt.subplots(figsize=(7, 5))
cats = ["Rm_cold\n(this study,\nflight-start)", "Rm_hot\n(this study,\nlate-flight, lower bound)"]
vals = [rm["Rm_cold_mean_ohm"], rm["Rm_hot_mean_ohm"]]
bars = ax.bar(cats, vals, color=["tab:cyan", "tab:red"], width=0.5)
ax.axhline(rm["Rm_old_firmware"], color="tab:blue", ls="--", label=f"old firmware Rm={rm['Rm_old_firmware']}Ω")
ax.axhline(rm["Rm_lcr_paper"], color="tab:green", ls="--", label=f"LCR/paper Rm={rm['Rm_lcr_paper']}Ω")
for b, v in zip(bars, vals):
    ax.text(b.get_x() + b.get_width() / 2, v + 0.01, f"{v:.3f}Ω", ha="center")
ax.set_ylabel("Rm [Ohm]")
ax.set_title(f"Thermal Rm drift implied by within-flight V_app rise\n"
             f"ΔR={rm['dR_ohm']:.3f}Ω ({rm['dR_fraction']*100:.0f}%), "
             f"implied ΔT≈{rm['implied_dT_degC']:.0f}°C (α_Cu)")
ax.set_ylim(0, 0.7)
ax.legend()
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(HERE / "fig3_rm_cold_hot.png", dpi=140)
print("saved fig3_rm_cold_hot.png")
