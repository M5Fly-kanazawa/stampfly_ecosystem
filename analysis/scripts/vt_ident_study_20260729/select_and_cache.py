#!/usr/bin/env python3
"""
Select logs (same fixed, outcome-independent criterion as the earlier
ct_switch_study_20260727: >=15s combined ALT_HOLD(2)/POS_HOLD(3) content,
2026-06 and 2026-07 logs only) and parse+cache them all via parse_log_vt.

This is a superset requirement of the earlier study's own selection (that
study only needed steady hover; here we ALSO want the climb/descend/transient
duty range within the same flights, so the same file-level selection is
appropriate -- narrower filtering to "quasi-static" happens per-sample later).
"""
import sys
from pathlib import Path

from parse_log_vt import load_or_parse

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
OUT_DIR = Path(__file__).parent
CACHE_DIR = OUT_DIR / "cache"

MIN_MODE_DURATION_S = 15.0


def select_logs():
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


if __name__ == "__main__":
    selected = select_logs()
    print(f"Selected {len(selected)} logs (>= {MIN_MODE_DURATION_S}s ALT/POS content):")
    for s in selected:
        print(f"  {s}")
    with open(OUT_DIR / "selected_logs.txt", "w") as f:
        f.write("\n".join(selected) + "\n")
