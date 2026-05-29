#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# M7 differential diagnosis — flight-log to replay-CSV converter.
# Reads a real WiFi-UDP flight log (JSON Lines) recorded by the LEGACY firmware
# (firmware/vehicle) and emits a time-aligned CSV with ONE row per IMU cycle
# (~400 Hz). Each row carries the exact ESKF inputs the firmware fed (filtered
# gyro/accel, raw flow pixels, bottom-ToF distance) plus the firmware's OWN ESKF
# outputs (quat / pos / vel / biases) as the reference to compare against.
#
# M7 差分診断 — フライトログ→再生CSV変換器。
# 旧ファーム(firmware/vehicle)が記録した実機 WiFi-UDP ログ(JSON Lines)を読み、
# 1行=1 IMU周期(約400Hz)の時刻整列CSVを出力する。各行はファームがESKFに与えた
# 入力そのもの(フィルタ済 gyro/accel、生flow画素、底面ToF距離)に加え、ファーム
# 自身のESKF出力(quat/pos/vel/バイアス)を比較用の参照値として併記する。
#
# Why this works (Code Identity): the log stores `imu.gyro`/`imu.accel`, which
# are the FILTERED, body-frame values the firmware actually passed to predict()
# / updateAccelAttitude(). Replaying them verbatim reproduces the exact ESKF
# input stream without reconstructing the sensor frame-conversion or LPF.
# なぜ成立するか(Code Identity): ログの `imu.gyro`/`imu.accel` は、ファームが
# predict()/updateAccelAttitude() に実際に渡したフィルタ済・機体座標の値。これを
# そのまま再生すれば、座標変換やLPFを再構成せずにESKF入力列を厳密に再現できる。

import json
import sys
import argparse


# Sensor-event priority at equal timestamp: sensor updates must be seen BEFORE
# the IMU row that consumes them (matches the firmware's data-ready gating, which
# checks the flag on the NEXT IMU cycle).
# 同一タイムスタンプでのイベント優先度: センサ更新は、それを消費するIMU行より
# 先に処理する必要がある(ファームのdata-readyゲート=次のIMU周期でフラグを見る
# 挙動に一致)。
_PRIORITY = {
    "flow": 0, "tof_b": 0, "tof_f": 0, "baro": 0, "mag": 0,
    "posvel": 1, "rate_ref": 1, "ctrl": 1, "ctrl_ref": 1, "status": 1,
    "imu": 2,
}

# CSV column order (kept in one place so log_replay.cpp can rely on it).
# CSV列順(log_replay.cpp が依存するため一箇所に集約)。
COLUMNS = [
    "ts",
    "gx", "gy", "gz",            # filtered gyro fed to predict() [rad/s]
    "ax", "ay", "az",            # filtered accel fed to predict() [m/s^2]
    "flow_new", "fdx", "fdy", "fq",   # optical-flow update (raw int16 pixels)
    "tof_new", "tof",            # bottom-ToF update [m]
    "ref_qw", "ref_qx", "ref_qy", "ref_qz",   # firmware ESKF attitude [w,x,y,z]
    "ref_px", "ref_py", "ref_pz",             # firmware ESKF position [m]
    "ref_vx", "ref_vy", "ref_vz",             # firmware ESKF velocity [m/s]
    "ref_bgx", "ref_bgy", "ref_bgz",          # firmware ESKF gyro bias [rad/s]
    "ref_bax", "ref_bay", "ref_baz",          # firmware ESKF accel bias [m/s^2]
]


def load_events(path):
    """Read the JSONL log into a list of (ts, priority, obj) sorted by time.
    JSONLログを (ts, priority, obj) のリストへ読み込み時刻順にソートする。"""
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            ts = obj.get("ts")
            mid = obj.get("id")
            if ts is None or mid is None:
                continue
            events.append((ts, _PRIORITY.get(mid, 1), obj))
    # Stable sort by (ts, priority): sensors before the IMU row at equal ts.
    # (ts, priority) で安定ソート: 同一 ts ではセンサをIMU行より先に。
    events.sort(key=lambda e: (e[0], e[1]))
    return events


def convert(path, out):
    events = load_events(path)

    # Latest carried-forward sensor values + freshness flags.
    # 直近のセンサ値(キャリーフォワード)と更新フラグ。
    posvel = None                      # {"pos":[...], "vel":[...]}
    flow = {"dx": 0, "dy": 0, "q": 0}
    tof_b = 0.0
    flow_fresh = False
    tof_fresh = False

    out.write(",".join(COLUMNS) + "\n")
    n_rows = 0

    for ts, _pri, obj in events:
        mid = obj["id"]

        if mid == "posvel":
            posvel = obj
            continue
        if mid == "flow":
            flow = {"dx": obj.get("dx", 0), "dy": obj.get("dy", 0),
                    "q": obj.get("quality", 0)}
            flow_fresh = True
            continue
        if mid == "tof_b":
            tof_b = obj.get("distance", 0.0)
            tof_fresh = True
            continue
        if mid != "imu":
            continue  # baro/mag/tof_f/ctrl/... : not ESKF inputs in this firmware

        # --- An IMU sample defines a replay row. Emit it now. ---
        # --- IMUサンプルが1再生行を定義。ここで出力する。 ---
        gyro = obj.get("gyro", [0, 0, 0])
        accel = obj.get("accel", [0, 0, 0])
        quat = obj.get("quat", [1, 0, 0, 0])
        gb = obj.get("gyro_bias", [0, 0, 0])
        ba = obj.get("accel_bias", [0, 0, 0])

        # posvel shares the IMU timestamp (both at ~398 Hz); fall back to zeros
        # only at the very first row before any posvel has arrived.
        # posvel は IMU と同一 ts(共に約398Hz)。最初の posvel 到着前のみ 0 で代替。
        if posvel is not None:
            pos = posvel.get("pos", [0, 0, 0])
            vel = posvel.get("vel", [0, 0, 0])
        else:
            pos = [0, 0, 0]
            vel = [0, 0, 0]

        row = [
            ts,
            gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            1 if flow_fresh else 0, flow["dx"], flow["dy"], flow["q"],
            1 if tof_fresh else 0, tof_b,
            quat[0], quat[1], quat[2], quat[3],
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            gb[0], gb[1], gb[2],
            ba[0], ba[1], ba[2],
        ]
        out.write(",".join(repr(x) if isinstance(x, float) else str(x)
                            for x in row) + "\n")
        n_rows += 1
        flow_fresh = False
        tof_fresh = False

    return n_rows


def main():
    ap = argparse.ArgumentParser(
        description="Convert a flight JSONL log into a 400 Hz replay CSV for "
                    "log_replay (M7 differential diagnosis).")
    ap.add_argument("log", help="input flight log (.jsonl)")
    ap.add_argument("-o", "--output", help="output CSV (default: stdout)")
    args = ap.parse_args()

    if args.output:
        with open(args.output, "w") as f:
            n = convert(args.log, f)
        sys.stderr.write(f"[log_to_replay] wrote {n} IMU rows -> {args.output}\n")
    else:
        n = convert(args.log, sys.stdout)
        sys.stderr.write(f"[log_to_replay] wrote {n} IMU rows -> stdout\n")


if __name__ == "__main__":
    main()
