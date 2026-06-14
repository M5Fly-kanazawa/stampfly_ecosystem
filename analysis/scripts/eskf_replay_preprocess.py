#!/usr/bin/env python3
"""
eskf_replay_preprocess.py — flatten a flight log into a time-sorted event file for the
offline ESKF replay harness (eskf_replay.cpp).

飛行ログを時刻順のイベントファイルに平坦化（オフライン ESKF 再生ハーネス用）。

Output (whitespace-separated, sorted by timestamp):
  INIT gbx gby gbz abx aby abz      # initial gyro/accel bias (boot calibration)
  I t gx gy gz ax ay az             # IMU (rad/s, m/s²)
  T t dist status                   # ToF
  F t dx dy squal                   # optical flow
  M t mx my mz                      # mag
  B t alt                           # baro

Usage: python3 eskf_replay_preprocess.py <log.jsonl> <events.txt>
"""
import sys, json

def main(src, dst):
    streams = {}
    with open(src) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            streams.setdefault(d["id"], []).append(d)
    t0 = streams["imu"][0]["ts"]
    gb = streams["imu"][0]["gyro_bias"]
    ab = streams["imu"][0]["accel_bias"]
    ev = []
    for d in streams["imu"]:
        t = (d["ts"] - t0) / 1e6
        g, a = d["gyro"], d["accel"]
        ev.append((t, f"I {t:.6f} {g[0]:.6f} {g[1]:.6f} {g[2]:.6f} {a[0]:.6f} {a[1]:.6f} {a[2]:.6f}"))
    for d in streams.get("tof_b", []):
        t = (d["ts"] - t0) / 1e6
        ev.append((t, f"T {t:.6f} {d['distance']:.6f} {d['status']}"))
    for d in streams.get("flow", []):
        t = (d["ts"] - t0) / 1e6
        ev.append((t, f"F {t:.6f} {d['dx']} {d['dy']} {d['quality']}"))
    for d in streams.get("mag", []):
        t = (d["ts"] - t0) / 1e6
        ev.append((t, f"M {t:.6f} {d['x']:.4f} {d['y']:.4f} {d['z']:.4f}"))
    for d in streams.get("baro", []):
        t = (d["ts"] - t0) / 1e6
        ev.append((t, f"B {t:.6f} {d['altitude']:.4f}"))
    ev.sort(key=lambda x: x[0])
    with open(dst, "w") as f:
        f.write(f"INIT {gb[0]:.6f} {gb[1]:.6f} {gb[2]:.6f} {ab[0]:.6f} {ab[1]:.6f} {ab[2]:.6f}\n")
        for _, s in ev:
            f.write(s + "\n")
    print(f"wrote {len(ev)} events to {dst}")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
