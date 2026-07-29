#!/usr/bin/env python3
"""
Raw (non-hover-filtered) samples right at the end of one flight and right at
the start of the next flight, for same-power-session (ts-continuous, so same
battery / no reboot) pairs separated by a real ground-idle gap. The strict
rolling-window hover extractor returns zero segments for these very short
flights (they never sustain steady hover long enough), so we fall back to raw
armed/FLYING samples, excluding the first/last ~0.6s of each flight
(takeoff/landing transients) as the only filter.

This is the closest thing in this dataset to the "cool, then re-fly" protocol
the task asks for: if H1 (thermal) is right, V_app at the START of flight 2
should sit close to the COLD baseline (flight-1-start-like), not continue from
where flight 1 left off, despite Vbat being similar/only slightly recovered
across the gap.

同一電源セッション(ts連続=同一バッテリ、電源再投入なし)で、地上待機ギャップを
挟んだ2フライトの、フライト1終端付近とフライト2始端付近の生サンプル(離陸/着陸
遷移の前後0.6秒を除く以外はフィルタなし)を比較する。H1(熱)が正しいなら、
Vbatがギャップ中にわずかしか回復していなくても、フライト2冒頭のV_appは
「冷えた」フライト1冒頭に近い値へ戻るはず。
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from parse_log import load_or_parse  # noqa: E402

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
CACHE_DIR = Path(__file__).parent / "cache"
EDGE_TRIM_S = 0.6  # exclude this many seconds at each end of a FLYING block


def flying_blocks(df):
    fs = df["flight_state"].to_numpy()
    flying = (fs == 5)
    blocks = []
    i = 0
    n = len(flying)
    while i < n:
        if not flying[i]:
            i += 1
            continue
        j = i
        while j < n and flying[j]:
            j += 1
        blocks.append((i, j))
        i = j
    return blocks


def block_stats(df, i0, i1, edge_trim_s=EDGE_TRIM_S, which="all"):
    seg = df.iloc[i0:i1]
    t0 = seg["t_s"].iloc[0]
    t1 = seg["t_s"].iloc[-1]
    if which == "start":
        seg = seg[(seg["t_s"] >= t0 + edge_trim_s) & (seg["t_s"] <= t0 + edge_trim_s + 2.0)]
    elif which == "end":
        seg = seg[(seg["t_s"] <= t1 - edge_trim_s) & (seg["t_s"] >= t1 - edge_trim_s - 2.0)]
    else:
        seg = seg[(seg["t_s"] >= t0 + edge_trim_s) & (seg["t_s"] <= t1 - edge_trim_s)]
    duty4 = seg[["duty_m1", "duty_m2", "duty_m3", "duty_m4"]].to_numpy()
    vbat = seg["voltage"].to_numpy()
    valid = np.isfinite(vbat) & np.all(np.isfinite(duty4), axis=1) & (seg["mode"].isin([2, 3]))
    duty4 = duty4[valid]
    vbat = vbat[valid]
    if len(vbat) == 0:
        return None
    vapp = duty4.mean(axis=1) * vbat
    return {
        "n": len(vbat), "t_start": float(seg["t_s"].iloc[0]) if len(seg) else np.nan,
        "t_end": float(seg["t_s"].iloc[-1]) if len(seg) else np.nan,
        "voltage_mean": float(np.mean(vbat)), "voltage_std": float(np.std(vbat)),
        "duty_mean": float(np.mean(duty4)),
        "Vapp_mean": float(np.mean(vapp)), "Vapp_std": float(np.std(vapp)),
    }


def analyze_pair(file_a, file_b, label):
    pa = LOG_DIR / file_a
    pb = LOG_DIR / file_b
    dfa = load_or_parse(pa, CACHE_DIR)
    dfb = load_or_parse(pb, CACHE_DIR)
    blocks_a = flying_blocks(dfa)
    blocks_b = flying_blocks(dfb)
    if not blocks_a or not blocks_b:
        print(f"{label}: missing flying block(s) in {file_a} or {file_b}")
        return

    # last flying block of file A, first flying block of file B
    i0a, i1a = blocks_a[-1]
    i0b, i1b = blocks_b[0]

    end_a_early = block_stats(dfa, i0a, i1a, which="start")   # start of flight A (coldest reference)
    end_a_late = block_stats(dfa, i0a, i1a, which="end")      # end of flight A (warmest point before gap)
    start_b = block_stats(dfb, i0b, i1b, which="start")       # start of flight B (post-gap)

    ts_end_a = dfa["ts_us"].iloc[i1a - 1]
    ts_start_b = dfb["ts_us"].iloc[i0b]
    gap_s = (ts_start_b - ts_end_a) / 1e6

    print(f"=== {label}: {file_a} -> {file_b}  (ground-idle gap = {gap_s:.1f}s, same power session) ===")
    for name, s in [("flight-A start (coldest ref)", end_a_early),
                     ("flight-A end (just before gap)", end_a_late),
                     ("flight-B start (after gap)", start_b)]:
        if s is None:
            print(f"  {name}: no valid samples")
        else:
            print(f"  {name}: n={s['n']:3d}  Vbat={s['voltage_mean']:.3f}V  "
                  f"duty={s['duty_mean']:.3f}  V_app={s['Vapp_mean']:.4f}V (std {s['Vapp_std']:.4f})")
    print()


if __name__ == "__main__":
    analyze_pair("stampfly_udp_20260627T020050.jsonl", "stampfly_udp_20260627T020137.jsonl",
                 "2026-06-27 02:0x session")
    analyze_pair("stampfly_udp_20260622T161016.jsonl", "stampfly_udp_20260622T161055.jsonl",
                 "2026-06-22 16:10 session")
