#!/usr/bin/env python3
"""Scan all candidate logs and report flight-mode composition + duration, so
we can pick sessions that actually contain ALT_HOLD(2)/POS_HOLD(3) hover data
before running the full hover-extraction analysis on them."""
import sys
from pathlib import Path
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from parse_log import load_or_parse

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
CACHE_DIR = Path(__file__).parent / "cache"

def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else "stampfly_udp_202606*.jsonl"
    files = sorted(LOG_DIR.glob(pattern))
    rows = []
    for f in files:
        try:
            df = load_or_parse(f, CACHE_DIR)
        except Exception as e:
            print(f"{f.name}: ERROR {e}")
            continue
        if df.empty:
            print(f"{f.name}: EMPTY")
            continue
        dur = df["t_s"].max()
        mode_counts = df["mode"].value_counts().to_dict()
        n_alt = mode_counts.get(2, 0)
        n_pos = mode_counts.get(3, 0)
        alt_dur = n_alt / 50.0
        pos_dur = n_pos / 50.0
        volt_mean = df["voltage"].mean()
        rows.append({
            "file": f.name, "dur_s": dur, "n": len(df),
            "alt_s": alt_dur, "pos_s": pos_dur,
            "modes": mode_counts, "volt_mean": volt_mean,
        })
        print(f"{f.name}: dur={dur:6.1f}s  ALT={alt_dur:6.1f}s  POS={pos_dur:6.1f}s  "
              f"modes={mode_counts}  Vmean={volt_mean:.2f}")
    return rows

if __name__ == "__main__":
    main()
