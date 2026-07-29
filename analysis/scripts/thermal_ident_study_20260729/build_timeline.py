#!/usr/bin/env python3
"""
Build a coarse per-log timeline of flight_state/mode/voltage transitions so we
can identify:
  (a) which log files are continuations of the same power-on session (ts_us
      continuous across file boundary -> same battery, no reboot), and
  (b) whether there is a real ground-idle gap (motors off, duty~0,
      flight_state IDLE/ARMED_GROUND) between two flights within such a
      session -- that is the structure needed to separate H1 (thermal, should
      reset toward cold baseline across a cooling gap) from H2 (voltage-only,
      no reset expected beyond what Vbat itself does).

複数ログファイルにまたがる状態遷移の粗いタイムラインを作る。ts_us が連続なら
同一電源セッション(同一バッテリ)。その中で地上待機(モータ停止)を挟む区間が
あれば、H1(熱)vs H2(電圧)の判別に使える「冷却付き複数フライト」候補となる。
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from parse_log import load_or_parse  # noqa: E402

LOG_DIR = Path("/Users/kouhei/tmp/github/stampfly_ecosystem/logs")
CACHE_DIR = Path(__file__).parent / "cache"

FLIGHT_STATE_NAMES = {
    0: "INIT", 1: "IDLE_GROUND", 2: "IDLE_HELD", 3: "ARMED_GROUND",
    4: "TAKEOFF", 5: "FLYING", 6: "LANDING",
}


def summarize(path: Path) -> dict:
    df = load_or_parse(path, CACHE_DIR)
    if df.empty:
        return {"file": path.name, "n": 0}
    ts0 = df["ts_us"].iloc[0]
    ts1 = df["ts_us"].iloc[-1]
    fs = df["flight_state"].dropna()
    flying = df[df["flight_state"] == 5]
    v0 = df["voltage"].dropna()
    row = {
        "file": path.name,
        "n": len(df),
        "ts_start_us": int(ts0),
        "ts_end_us": int(ts1),
        "duration_s": float(df["t_s"].iloc[-1]),
        "voltage_first": float(v0.iloc[0]) if len(v0) else np.nan,
        "voltage_last": float(v0.iloc[-1]) if len(v0) else np.nan,
        "flying_s": float(len(flying)) / 50.0 if len(flying) else 0.0,
        "t_flying_start_s": float(flying["t_s"].iloc[0]) if len(flying) else np.nan,
        "t_flying_end_s": float(flying["t_s"].iloc[-1]) if len(flying) else np.nan,
        "fs_seq": "".join(
            str(int(v)) for v in fs[(fs.diff().fillna(1) != 0)].to_numpy()
        ) if len(fs) else "",
    }
    return row


def main():
    files = sorted(LOG_DIR.glob("stampfly_udp_*.jsonl"))
    if len(sys.argv) > 1:
        pattern = sys.argv[1]
        files = [f for f in files if pattern in f.name]

    rows = []
    for f in files:
        try:
            rows.append(summarize(f))
        except Exception as e:
            print(f"ERROR {f.name}: {e}", file=sys.stderr)
    out = pd.DataFrame(rows)
    out = out.sort_values("file")  # filename embeds wall-clock time -> chronological order
    pd.set_option("display.width", 200)
    pd.set_option("display.max_colwidth", 40)
    print(out.to_string(index=False))

    # Detect continuous-power-session groups (ts_start_us[i] >= ts_end_us[i-1]
    # roughly, i.e. small forward gap; a large negative jump means reboot).
    out = out.reset_index(drop=True)
    out["gap_from_prev_s"] = np.nan
    for i in range(1, len(out)):
        out.loc[i, "gap_from_prev_s"] = (out.loc[i, "ts_start_us"] - out.loc[i - 1, "ts_end_us"]) / 1e6
    print("\n--- gap from previous file (same boot if positive & small; reboot if negative) ---")
    print(out[["file", "gap_from_prev_s"]].to_string(index=False))

    out_path = Path(__file__).parent / "timeline_summary.csv"
    out.to_csv(out_path, index=False)
    print(f"\nsaved: {out_path}")


if __name__ == "__main__":
    main()
