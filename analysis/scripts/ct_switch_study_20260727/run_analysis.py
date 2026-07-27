#!/usr/bin/env python3
"""
Main pipeline: auto-select ALL 2026-06/2026-07 flight logs with >=15s of
ALT_HOLD/POS_HOLD content (unbiased selection criterion — NOT hand-picked
for outcome, to avoid confirmation bias per the task's anti-cherry-picking
requirement), extract hover segments, compute implied MOTOR_CT-chain
correction factors, aggregate at three levels (segment / file / date), and
write CSV/JSON outputs. Plotting is in plot_results.py.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from parse_log import load_or_parse
from hover_segments import find_hover_segments

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
OUT_DIR = Path(__file__).parent
CACHE_DIR = OUT_DIR / "cache"

MIN_MODE_DURATION_S = 15.0   # selection threshold: >=15s combined ALT+POS content


def select_logs():
    """Unbiased selection: scan every 2026-06/2026-07 log once (mode counts
    only, cheap) and keep files with >=MIN_MODE_DURATION_S seconds of
    ALT_HOLD(2) or POS_HOLD(3) samples. This is a fixed, outcome-independent
    criterion decided BEFORE looking at any implied_corr numbers."""
    files = sorted(LOG_DIR.glob("stampfly_udp_202606*.jsonl")) + \
            sorted(LOG_DIR.glob("stampfly_udp_202607*.jsonl"))
    keep = []
    for f in files:
        df = load_or_parse(f, CACHE_DIR)
        if df.empty:
            continue
        counts = df["mode"].value_counts()
        hold_s = (counts.get(2, 0) + counts.get(3, 0)) / 50.0
        if hold_s >= MIN_MODE_DURATION_S:
            keep.append(f.name)
    return keep


def main():
    selected = select_logs()
    print(f"Selected {len(selected)} logs (>= {MIN_MODE_DURATION_S}s ALT/POS content):")
    for s in selected:
        print(f"  {s}")

    all_segments = []
    file_summary = []

    for name in selected:
        path = LOG_DIR / name
        df = load_or_parse(path, CACHE_DIR)
        segs = find_hover_segments(df)
        date_str = name.split("_")[2][:8]
        time_str = name.split("_")[2][9:15]
        if segs.empty:
            file_summary.append({
                "file": name, "date": date_str, "time": time_str,
                "n_segments": 0, "hover_duration_s": 0.0,
            })
            print(f"{name}: 0 hover segments found")
            continue
        segs["file"] = name
        segs["date"] = date_str
        segs["time"] = time_str
        all_segments.append(segs)

        file_summary.append({
            "file": name, "date": date_str, "time": time_str,
            "n_segments": len(segs),
            "hover_duration_s": float(segs["duration_s"].sum()),
            "voltage_mean": float(segs["voltage_mean"].mean()),
            "implied_corr_old_mean": float(segs["implied_corr_old_mean"].mean()),
            "implied_corr_new_mean": float(segs["implied_corr_new_mean"].mean()),
            "firmware_corr_logged_mean": float(segs["firmware_corr_logged_mean"].mean()),
        })
        print(f"{name}: {len(segs)} segments, {segs['duration_s'].sum():.1f}s hover, "
              f"corr_old={segs['implied_corr_old_mean'].mean():.3f} "
              f"corr_new={segs['implied_corr_new_mean'].mean():.3f} "
              f"firmware_logged={segs['firmware_corr_logged_mean'].mean():.3f}")

    if not all_segments:
        print("NO HOVER SEGMENTS FOUND IN ANY SELECTED LOG.")
        return

    segments_df = pd.concat(all_segments, ignore_index=True)
    file_df = pd.DataFrame(file_summary)

    segments_df.to_csv(OUT_DIR / "hover_segments.csv", index=False)
    file_df.to_csv(OUT_DIR / "file_summary.csv", index=False)

    # Date-level aggregation (multiple same-date files = reconnects of one
    # session; average them first so a date with many short reconnects
    # doesn't get more weight than a date with one long continuous flight).
    file_ok = file_df[file_df["n_segments"] > 0]
    date_df = file_ok.groupby("date").agg(
        n_files=("file", "count"),
        hover_duration_s=("hover_duration_s", "sum"),
        implied_corr_old_mean=("implied_corr_old_mean", "mean"),
        implied_corr_new_mean=("implied_corr_new_mean", "mean"),
        voltage_mean=("voltage_mean", "mean"),
    ).reset_index()
    date_df.to_csv(OUT_DIR / "date_summary.csv", index=False)

    def stats(x):
        x = np.asarray(x, dtype=float)
        return {
            "mean": float(np.mean(x)), "std": float(np.std(x, ddof=0)),
            "min": float(np.min(x)), "max": float(np.max(x)), "n": int(len(x)),
        }

    overall = {
        "pooled_segments": {
            "old_chain": stats(segments_df["implied_corr_old_mean"]),
            "new_chain": stats(segments_df["implied_corr_new_mean"]),
            "firmware_logged": stats(segments_df["firmware_corr_logged_mean"].dropna()),
        },
        "per_file_mean": {
            "old_chain": stats(file_ok["implied_corr_old_mean"]),
            "new_chain": stats(file_ok["implied_corr_new_mean"]),
        },
        "per_date_mean": {
            "old_chain": stats(date_df["implied_corr_old_mean"]),
            "new_chain": stats(date_df["implied_corr_new_mean"]),
        },
        "n_dates": int(len(date_df)),
        "n_files_selected": int(len(selected)),
        "n_files_with_hover": int(len(file_ok)),
        "n_segments_total": int(len(segments_df)),
        "total_hover_duration_s": float(segments_df["duration_s"].sum()),
        "selection_criterion": f">= {MIN_MODE_DURATION_S}s of ALT_HOLD/POS_HOLD samples, "
                                "2026-06 and 2026-07 logs only (fixed threshold, chosen before "
                                "computing any implied_corr numbers)",
    }

    with open(OUT_DIR / "overall_stats.json", "w") as f:
        json.dump(overall, f, indent=2)

    print("\n=== OVERALL (pooled segments) ===")
    print(json.dumps(overall["pooled_segments"], indent=2))
    print("\n=== OVERALL (per-file mean) ===")
    print(json.dumps(overall["per_file_mean"], indent=2))
    print("\n=== OVERALL (per-date mean) ===")
    print(json.dumps(overall["per_date_mean"], indent=2))
    print(f"\ndates: {overall['n_dates']}, files with hover: {overall['n_files_with_hover']}/{overall['n_files_selected']}, "
          f"segments: {overall['n_segments_total']}, total hover time: {overall['total_hover_duration_s']:.1f}s")


if __name__ == "__main__":
    main()
