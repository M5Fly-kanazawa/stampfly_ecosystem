#!/usr/bin/env python3
"""
Session time-series plot: implied_corr (old chain vs new chain) per flight
session (one point per file, ordered by date/time), with the file-internal
segment-to-segment std as an error bar. A dashed neutral line at y=1.0 marks
the "no correction needed" reference the NEW chain was hypothesized to hit.

Two-series categorical chart -> fixed hue order (never cycled), single axis,
thin marks with visible error bars, legend always present for >=2 series,
recessive gridlines. Colors are the Okabe-Ito colorblind-safe pair
(blue #0072B2 / orange #E69F00), which passes standard CVD-separation checks.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
import pandas as pd

OUT_DIR = Path(__file__).parent

COLOR_OLD = "#0072B2"   # blue   — old_firmware chain
COLOR_NEW = "#E69F00"   # orange — new_measured chain
COLOR_NEUTRAL = "#8A8A8A"

def main():
    seg = pd.read_csv(OUT_DIR / "hover_segments.csv")
    seg["datetime"] = pd.to_datetime(seg["date"].astype(str) + seg["time"].astype(str).str.zfill(6),
                                      format="%Y%m%d%H%M%S")

    file_g = seg.groupby(["file", "datetime"]).agg(
        old_mean=("implied_corr_old_mean", "mean"),
        old_std=("implied_corr_old_mean", "std"),
        new_mean=("implied_corr_new_mean", "mean"),
        new_std=("implied_corr_new_mean", "std"),
        n_seg=("implied_corr_old_mean", "count"),
    ).reset_index().sort_values("datetime")
    file_g["old_std"] = file_g["old_std"].fillna(0.0)
    file_g["new_std"] = file_g["new_std"].fillna(0.0)

    x = file_g["datetime"]

    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.axhline(1.0, color=COLOR_NEUTRAL, linestyle="--", linewidth=1.2, zorder=1,
               label="corr = 1.0 (no correction needed)")

    ax.errorbar(x, file_g["old_mean"], yerr=file_g["old_std"], fmt="o-",
                color=COLOR_OLD, ecolor=COLOR_OLD, elinewidth=1.4, capsize=3,
                markersize=6, linewidth=1.6, label="old chain (current firmware, Ct=1.0e-8)",
                zorder=3)
    ax.errorbar(x, file_g["new_mean"], yerr=file_g["new_std"], fmt="s-",
                color=COLOR_NEW, ecolor=COLOR_NEW, elinewidth=1.4, capsize=3,
                markersize=6, linewidth=1.6, label="new chain (measured, Ct=6.7e-9)",
                zorder=3)

    ax.set_ylabel("Implied hover thrust correction  (implied thrust / mg)")
    ax.set_xlabel("Session (date/time)")
    ax.set_title("MOTOR_CT switch consistency check — implied hover correction per flight session\n"
                  "(2026-06/07 real-flight logs, ALT_HOLD/POS_HOLD steady-hover segments)")
    ax.grid(True, axis="y", alpha=0.25, linewidth=0.8)
    ax.grid(False, axis="x")
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.spines["left"].set_color("#CCCCCC")
    ax.spines["bottom"].set_color("#CCCCCC")

    ax.xaxis.set_major_formatter(mdates.DateFormatter("%m/%d\n%H:%M"))
    fig.autofmt_xdate(rotation=0, ha="center")

    ax.legend(loc="upper left", frameon=False, fontsize=10)

    ymin = min(file_g["old_mean"].min() - file_g["old_std"].max(),
               file_g["new_mean"].min() - file_g["new_std"].max(), 0.95)
    ymax = max(file_g["old_mean"].max() + file_g["old_std"].max(),
               file_g["new_mean"].max() + file_g["new_std"].max(), 1.05)
    ax.set_ylim(ymin - 0.03, ymax + 0.03)

    fig.tight_layout()
    out_path = OUT_DIR / "implied_corr_timeseries.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
