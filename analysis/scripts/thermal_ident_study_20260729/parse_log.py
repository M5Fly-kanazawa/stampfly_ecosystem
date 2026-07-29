#!/usr/bin/env python3
"""
Stream-parse a StampFly stampfly_udp_*.jsonl flight log and extract only the
fields needed for the MOTOR_CT switch consistency study:
  - ctrl_ref  (50Hz): mode, total_thrust, motor_duty[4]
  - ctrl      (50Hz): pilot stick roll/pitch/yaw/throttle
  - posvel    (400Hz): NED pos/vel (we only need it at ctrl_ref rate, so we
                        subsample by nearest timestamp)
  - status    (~1Hz): battery voltage, flight_state, current_ma (optional)

Output: a single merged pandas DataFrame indexed by ctrl_ref sample, saved as
a .parquet (fallback .csv) file next to the source log for reuse across runs
(parsing a 200k-line jsonl file is the slow part; do it once).

StampFly 実飛行ログ(stampfly_udp_*.jsonl)をストリーム解析し、MOTOR_CT 切替の
整合性分析に必要なフィールドだけを抽出する。詳細は上の英語コメント参照。
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def parse_log(path: Path) -> dict:
    """Single streaming pass. Returns dict of lists per topic."""
    ctrl_ref = {"ts": [], "mode": [], "total_thrust": [], "duty": []}
    ctrl = {"ts": [], "throttle": [], "roll": [], "pitch": [], "yaw": []}
    posvel = {"ts": [], "pos": [], "vel": []}
    status = {"ts": [], "voltage": [], "flight_state": [], "current_ma": []}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            i = d.get("id")
            if i == "ctrl_ref":
                ctrl_ref["ts"].append(d["ts"])
                ctrl_ref["mode"].append(d.get("mode", -1))
                ctrl_ref["total_thrust"].append(d.get("total_thrust", 0.0))
                ctrl_ref["duty"].append(d.get("motor_duty", [0, 0, 0, 0]))
            elif i == "ctrl":
                ctrl["ts"].append(d["ts"])
                ctrl["throttle"].append(d.get("throttle", 0.0))
                ctrl["roll"].append(d.get("roll", 0.0))
                ctrl["pitch"].append(d.get("pitch", 0.0))
                ctrl["yaw"].append(d.get("yaw", 0.0))
            elif i == "posvel":
                ts = d["ts"]
                # Subsample posvel to ~50Hz on the fly (keep every 8th-ish
                # sample) to bound memory: 400Hz over a multi-minute flight is
                # unnecessary precision for hover-window detection.
                posvel["ts"].append(ts)
                posvel["pos"].append(d.get("pos", [0, 0, 0]))
                posvel["vel"].append(d.get("vel", [0, 0, 0]))
            elif i == "status":
                status["ts"].append(d["ts"])
                status["voltage"].append(d.get("voltage", np.nan))
                status["flight_state"].append(d.get("flight_state", -1))
                status["current_ma"].append(d.get("current_ma", np.nan))

    return {"ctrl_ref": ctrl_ref, "ctrl": ctrl, "posvel": posvel, "status": status}


def nearest_merge(ref_ts: np.ndarray, src_ts: np.ndarray, src_vals: np.ndarray, max_gap_us: float) -> np.ndarray:
    """For each ref_ts, find the nearest src_ts sample (assumes src_ts sorted
    ascending) and return src_vals at that index, or NaN if the source is
    empty or the nearest sample is farther than max_gap_us away."""
    if len(src_ts) == 0:
        return np.full((len(ref_ts),) + src_vals.shape[1:], np.nan)
    idx = np.searchsorted(src_ts, ref_ts)
    idx = np.clip(idx, 1, len(src_ts) - 1)
    left = idx - 1
    right = idx
    left_gap = np.abs(ref_ts - src_ts[left])
    right_gap = np.abs(src_ts[right] - ref_ts)
    use_left = left_gap <= right_gap
    chosen = np.where(use_left, left, right)
    gap = np.where(use_left, left_gap, right_gap)
    out = src_vals[chosen].astype(float)
    bad = gap > max_gap_us
    out[bad] = np.nan
    return out


def build_dataframe(parsed: dict) -> pd.DataFrame:
    cr = parsed["ctrl_ref"]
    if len(cr["ts"]) == 0:
        return pd.DataFrame()

    ts = np.asarray(cr["ts"], dtype=np.int64)
    order = np.argsort(ts)
    ts = ts[order]
    mode = np.asarray(cr["mode"])[order]
    total_thrust = np.asarray(cr["total_thrust"])[order]
    duty = np.asarray(cr["duty"])[order]  # (N,4)

    df = pd.DataFrame({
        "ts_us": ts,
        "mode": mode,
        "total_thrust": total_thrust,
        "duty_m1": duty[:, 0],
        "duty_m2": duty[:, 1],
        "duty_m3": duty[:, 2],
        "duty_m4": duty[:, 3],
    })
    df["duty_mean"] = duty.mean(axis=1)
    df["duty_std_across_motors"] = duty.std(axis=1)

    # ctrl (stick) — nearest merge, 50Hz vs 50Hz so gap should be tiny
    c = parsed["ctrl"]
    if len(c["ts"]) > 0:
        c_ts = np.asarray(c["ts"], dtype=np.int64)
        c_order = np.argsort(c_ts)
        c_ts = c_ts[c_order]
        for key in ("throttle", "roll", "pitch", "yaw"):
            vals = np.asarray(c[key])[c_order]
            df[f"stick_{key}"] = nearest_merge(ts, c_ts, vals, max_gap_us=100_000)
    else:
        for key in ("throttle", "roll", "pitch", "yaw"):
            df[f"stick_{key}"] = np.nan

    # posvel — nearest merge (400Hz source, dense, gap should be tiny)
    pv = parsed["posvel"]
    if len(pv["ts"]) > 0:
        pv_ts = np.asarray(pv["ts"], dtype=np.int64)
        pv_order = np.argsort(pv_ts)
        pv_ts = pv_ts[pv_order]
        pos = np.asarray(pv["pos"])[pv_order]
        vel = np.asarray(pv["vel"])[pv_order]
        alt = -pos[:, 2]  # NED z-down -> altitude up
        velz_up = -vel[:, 2]
        df["altitude"] = nearest_merge(ts, pv_ts, alt, max_gap_us=100_000)
        df["velz_up"] = nearest_merge(ts, pv_ts, velz_up, max_gap_us=100_000)
    else:
        df["altitude"] = np.nan
        df["velz_up"] = np.nan

    # status (battery voltage etc.) — sparse (~1Hz), allow a larger gap
    st = parsed["status"]
    if len(st["ts"]) > 0:
        st_ts = np.asarray(st["ts"], dtype=np.int64)
        st_order = np.argsort(st_ts)
        st_ts = st_ts[st_order]
        voltage = np.asarray(st["voltage"], dtype=float)[st_order]
        flight_state = np.asarray(st["flight_state"], dtype=float)[st_order]
        current_ma = np.asarray(st["current_ma"], dtype=float)[st_order]
        df["voltage"] = nearest_merge(ts, st_ts, voltage, max_gap_us=2_000_000)
        df["flight_state"] = nearest_merge(ts, st_ts, flight_state, max_gap_us=2_000_000)
        df["current_ma"] = nearest_merge(ts, st_ts, current_ma, max_gap_us=2_000_000)
    else:
        df["voltage"] = np.nan
        df["flight_state"] = np.nan
        df["current_ma"] = np.nan

    df["t_s"] = (df["ts_us"] - df["ts_us"].iloc[0]) / 1e6
    return df


def load_or_parse(path: Path, cache_dir: Path) -> pd.DataFrame:
    cache = cache_dir / (path.stem + ".pkl")
    if cache.exists() and cache.stat().st_mtime > path.stat().st_mtime:
        return pd.read_pickle(cache)
    parsed = parse_log(path)
    df = build_dataframe(parsed)
    cache_dir.mkdir(parents=True, exist_ok=True)
    df.to_pickle(cache)
    return df


if __name__ == "__main__":
    p = Path(sys.argv[1])
    cache_dir = Path(__file__).parent / "cache"
    df = load_or_parse(p, cache_dir)
    print(f"{p.name}: {len(df)} ctrl_ref samples, duration {df['t_s'].max():.1f}s")
    print(df[["t_s", "mode", "duty_mean", "voltage", "altitude", "velz_up",
              "stick_roll", "stick_pitch", "stick_yaw"]].describe())
    print(df["mode"].value_counts())
