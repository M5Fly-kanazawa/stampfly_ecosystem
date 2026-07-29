#!/usr/bin/env python3
"""
Fine-grained within-flight hover time series extraction.

Same mechanical "steady hover" criterion as
analysis/scripts/ct_switch_study_20260727/hover_segments.py (rolling-window
alt/velz/stick/duty thresholds), but:
  - additionally requires flight_state == FLYING (5), so ground-armed periods
    can never leak in as "hover"
  - uses a shorter MIN_SEG_SAMPLES (0.5s instead of 1.0s) to get more, finer
    time-resolved points across a single flight (needed to fit a thermal
    time-constant, which the coarse 85-segment study never attempted)
  - splits the file into "episodes" = contiguous FLYING blocks, and reports
    both absolute in-file time (t_s) and per-episode elapsed time since that
    takeoff (t_since_takeoff_s) -- the latter is what the thermal hypothesis
    (H1) actually predicts should govern V_app, not absolute file time.

同一の「定常ホバー」判定基準(ct_switch_studyのhover_segments.pyと同じ)を使うが、
flight_state==FLYING を追加条件とし、最小区間長を0.5秒に短縮して1フライト内の
時間分解能を上げる。ファイルを「エピソード」(連続FLYING区画)に分割し、
ファイル内絶対時刻 t_s とエピソード内経過時間 t_since_takeoff_s の両方を出力する
(H1=熱仮説が予言するのは後者)。
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from parse_log import load_or_parse  # noqa: E402

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
CACHE_DIR = Path(__file__).parent / "cache"

# Thresholds -- identical to ct_switch_study_20260727/hover_segments.py except
# MIN_SEG_SAMPLES (halved, to get finer time resolution; fixed a priori before
# looking at any thermal-fit result, same rationale documented there: chosen
# once against a real log to give a non-trivial hover fraction, not tuned per
# log or to favor a hypothesis).
WINDOW_SAMPLES = 75      # 1.5s at 50Hz ctrl_ref rate
ALT_STD_MAX = 0.02       # [m]
VELZ_MAX = 0.10          # [m/s]
STICK_MAX = 0.05         # [-1..1]
DUTY_MIN = 0.05
DUTY_SAT = 0.95
MIN_SEG_SAMPLES = 25     # >=0.5s contiguous (vs 50/1.0s in the original study)


def find_episodes(df: pd.DataFrame) -> list:
    """Contiguous flight_state==5 (FLYING) blocks -> list of (i0, i1) index
    ranges into df, plus the takeoff time (t_s at block start)."""
    fs = df["flight_state"].to_numpy()
    flying = (fs == 5)
    episodes = []
    i = 0
    n = len(flying)
    while i < n:
        if not flying[i]:
            i += 1
            continue
        j = i
        while j < n and flying[j]:
            j += 1
        episodes.append((i, j))
        i = j
    return episodes


def find_hover_segments_in_range(df: pd.DataFrame, i0: int, i1: int) -> np.ndarray:
    """Boolean hover mask computed on the WHOLE df (so rolling windows don't
    see artificial edges at episode boundaries get truncated at the edges
    naturally by pandas), then sliced to [i0, i1)."""
    m = df
    alt_std = m["altitude"].rolling(WINDOW_SAMPLES, center=True).std()
    velz_absmax = m["velz_up"].abs().rolling(WINDOW_SAMPLES, center=True).max()
    stick_absmax = (
        m[["stick_roll", "stick_pitch", "stick_yaw"]].abs().max(axis=1)
        .rolling(WINDOW_SAMPLES, center=True).max()
    )
    duty_min = m["duty_mean"].rolling(WINDOW_SAMPLES, center=True).min()
    duty_max = m["duty_mean"].rolling(WINDOW_SAMPLES, center=True).max()
    cand = (
        (alt_std < ALT_STD_MAX)
        & (velz_absmax < VELZ_MAX)
        & (stick_absmax < STICK_MAX)
        & (duty_min > DUTY_MIN)
        & (duty_max < DUTY_SAT)
        & (m["mode"].isin([2, 3]))
        & (m["flight_state"] == 5)
    ).fillna(False).to_numpy()
    return cand[i0:i1]


def extract_file(path: Path) -> pd.DataFrame:
    df = load_or_parse(path, CACHE_DIR)
    if df.empty:
        return pd.DataFrame()
    episodes = find_episodes(df)
    rows = []
    for ep_id, (i0, i1) in enumerate(episodes):
        t0 = df["t_s"].iloc[i0]  # takeoff instant for this episode
        cand = find_hover_segments_in_range(df, i0, i1)
        n = len(cand)
        i = 0
        seg_idx = 0
        while i < n:
            if not cand[i]:
                i += 1
                continue
            j = i
            while j < n and cand[j]:
                j += 1
            if (j - i) >= MIN_SEG_SAMPLES:
                seg = df.iloc[i0 + i:i0 + j]
                duty4 = seg[["duty_m1", "duty_m2", "duty_m3", "duty_m4"]].to_numpy()
                vbat = seg["voltage"].to_numpy()
                valid = np.isfinite(vbat) & np.all(np.isfinite(duty4), axis=1)
                duty4 = duty4[valid]
                vbat = vbat[valid]
                if len(vbat) >= MIN_SEG_SAMPLES // 2:
                    vapp4 = duty4 * vbat[:, None]  # per-sample per-motor V_app
                    t_start = seg["t_s"].iloc[0]
                    t_end = seg["t_s"].iloc[-1]
                    t_mid = 0.5 * (t_start + t_end)
                    rows.append({
                        "file": path.name,
                        "episode_id": ep_id,
                        "seg_idx": seg_idx,
                        "t_start_s": t_start,
                        "t_end_s": t_end,
                        "t_mid_s": t_mid,
                        "t_takeoff_s": t0,
                        "t_since_takeoff_s": t_mid - t0,
                        "duration_s": t_end - t_start,
                        "n_samples": len(vbat),
                        "voltage_mean": float(np.mean(vbat)),
                        "duty_m1_mean": float(np.mean(duty4[:, 0])),
                        "duty_m2_mean": float(np.mean(duty4[:, 1])),
                        "duty_m3_mean": float(np.mean(duty4[:, 2])),
                        "duty_m4_mean": float(np.mean(duty4[:, 3])),
                        "duty_mean": float(np.mean(duty4)),
                        "Vapp_m1_mean": float(np.mean(vapp4[:, 0])),
                        "Vapp_m2_mean": float(np.mean(vapp4[:, 1])),
                        "Vapp_m3_mean": float(np.mean(vapp4[:, 2])),
                        "Vapp_m4_mean": float(np.mean(vapp4[:, 3])),
                        "Vapp_mean": float(np.mean(vapp4)),
                        "altitude_mean": float(seg["altitude"].mean()),
                    })
                    seg_idx += 1
            i = j
    return pd.DataFrame(rows)


def main():
    files = sys.argv[1:]
    if not files:
        print("usage: extract_timeseries.py <log1.jsonl> [log2.jsonl ...]")
        sys.exit(1)
    all_rows = []
    for fname in files:
        p = LOG_DIR / fname if not Path(fname).is_absolute() else Path(fname)
        print(f"parsing {p.name} ...", file=sys.stderr)
        d = extract_file(p)
        print(f"  {len(d)} hover segments across {d['episode_id'].nunique() if len(d) else 0} episode(s)",
              file=sys.stderr)
        all_rows.append(d)
    out = pd.concat(all_rows, ignore_index=True) if all_rows else pd.DataFrame()
    out_path = Path(__file__).parent / "hover_timeseries.csv"
    out.to_csv(out_path, index=False)
    print(f"saved {len(out)} rows -> {out_path}")


if __name__ == "__main__":
    main()
