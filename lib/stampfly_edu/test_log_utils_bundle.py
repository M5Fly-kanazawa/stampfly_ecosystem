#!/usr/bin/env python3
"""
test_log_utils_bundle.py - tests for lib/stampfly_edu/log_utils.py's
load_flight_log() bundle support (docs/plans/flight-log-format-plan.md
section 3.2, stampfly_edu row).
test_log_utils_bundle.py - lib/stampfly_edu/log_utils.py の
load_flight_log() の一式対応の試験（計画書 3.2節 stampfly_edu の行）。

Covers:
  - loading a synthetic StampFly flight-log v1 bundle and getting back the
    educational short column names (time [s], x/y/z, vx/vy/vz, p/q/r,
    ax/ay/az, m1-m4, vbat, tof, alt) per log_utils.py's _COLUMN_ALIASES;
  - the timestamp_us [us] -> time [s] unit conversion (not just a rename);
  - quat_w/x/y/z are left unaliased ("kept");
  - time_zero=True shifts the bundle's own (non-zero) boot-relative start
    time to 0, same contract as the plain-CSV path;
  - the plain-CSV backward-compatibility path still works unchanged
    (covered already by log_utils.py's own doctest, re-asserted here too
    for a single source of truth on this file's own pass/fail).

対象:
  - 合成 StampFly フライトログ v1 一式を読み込み、log_utils.py の
    _COLUMN_ALIASES どおり教育用の短い列名（time[s], x/y/z, vx/vy/vz,
    p/q/r, ax/ay/az, m1-m4, vbat, tof, alt）が得られること;
  - timestamp_us[us] -> time[s] の単位変換（単なる改名でないこと）;
  - quat_w/x/y/z はエイリアスされず「そのまま」残ること;
  - time_zero=True が一式自身の（ゼロでない）起動基準開始時刻を0へ
    シフトすること（素のCSV経路と同じ契約）;
  - 素のCSVとの後方互換経路が変わらず動くこと（log_utils.py 自身の
    doctest で既に確認済みだが、この試験ファイル単体の合否のため
    ここでも確認する）。
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from stampfly_edu.log_utils import load_flight_log
from sflog.bundle import FlightLog, make_meta


def _synthetic_bundle(tmp_path, start_us: int = 5_000_000):
    """imu (400Hz) + posvel (400Hz) + attitude (400Hz) + motor (400Hz) +
    baro (50Hz) + tof_bottom (30Hz) + status (1Hz), starting at a
    NON-ZERO boot-relative timestamp (5s in) so time_zero's shift is
    actually exercised.
    imu/posvel/attitude/motor（400Hz）+ baro（50Hz）+ tof_bottom（30Hz）+
    status（1Hz）。time_zero のシフトを実際に確認するため、起動基準の
    ゼロでない時刻（5秒後）から開始する。
    """
    n = 20  # 50ms @ 400Hz
    dt = 1.0 / 400.0
    t = np.arange(n) * dt
    ts_us = start_us + np.round(t * 1e6).astype(np.int64)
    seq = np.arange(n)

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.full(n, 0.11), "gyro_y": np.full(n, 0.22), "gyro_z": np.full(n, 0.33),
        "accel_x": np.full(n, 1.1), "accel_y": np.full(n, 2.2), "accel_z": np.full(n, -9.81),
        "gyro_raw_x": np.full(n, 9.9), "gyro_raw_y": np.full(n, 9.9), "gyro_raw_z": np.full(n, 9.9),
        "accel_raw_x": np.full(n, 9.9), "accel_raw_y": np.full(n, 9.9), "accel_raw_z": np.full(n, 9.9),
    })
    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": np.full(n, 1.0), "pos_y": np.full(n, 2.0), "pos_z": np.full(n, -0.5),
        "vel_x": np.full(n, 0.1), "vel_y": np.full(n, 0.2), "vel_z": np.full(n, 0.0),
    })
    attitude = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "quat_w": np.full(n, 0.999), "quat_x": np.full(n, 0.01),
        "quat_y": np.full(n, 0.02), "quat_z": np.full(n, 0.03),
        "gyro_bias_x": np.zeros(n), "gyro_bias_y": np.zeros(n), "gyro_bias_z": np.zeros(n),
        "accel_bias_x": np.zeros(n), "accel_bias_y": np.zeros(n), "accel_bias_z": np.zeros(n),
    })
    motor = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "duty_FR": np.full(n, 0.51), "duty_RR": np.full(n, 0.52),
        "duty_RL": np.full(n, 0.53), "duty_FL": np.full(n, 0.54),
    })

    n_baro = 3
    t_baro = np.arange(n_baro) / 50.0
    baro = pd.DataFrame({
        "timestamp_us": start_us + np.round(t_baro * 1e6).astype(np.int64),
        "altitude": np.full(n_baro, 0.75),
        "pressure": np.full(n_baro, 101325.0),
    })

    n_tof = 2
    t_tof = np.arange(n_tof) / 30.0
    tof_bottom = pd.DataFrame({
        "timestamp_us": start_us + np.round(t_tof * 1e6).astype(np.int64),
        "distance": np.full(n_tof, 0.74),
        "status": np.zeros(n_tof, dtype=int),
    })

    status = pd.DataFrame({
        "timestamp_us": np.array([start_us], dtype=np.int64),
        "uptime_ms": np.array([start_us // 1000], dtype=np.int64),
        "voltage": np.array([3.82]),
    })

    streams = {
        "imu": imu, "posvel": posvel, "attitude": attitude, "motor": motor,
        "baro": baro, "tof_bottom": tof_bottom, "status": status,
    }
    meta = make_meta(source="sim", tool_name="test_log_utils_bundle.py",
                     tool_version="0", streams=streams)
    path = tmp_path / "edu_test.sflog.zip"
    FlightLog(meta=meta, schema={}, streams=streams).save(path)
    return path, start_us


def test_load_flight_log_bundle_column_aliases(tmp_path):
    path, start_us = _synthetic_bundle(tmp_path)
    df = load_flight_log(path)

    # timestamp_us -> time, unit-converted (us -> s), then zeroed
    # timestamp_us -> time、単位変換（us -> s）の上でゼロ開始
    assert "time" in df.columns
    assert "timestamp_us" not in df.columns
    assert df["time"].iloc[0] == pytest.approx(0.0)
    assert df["time"].iloc[1] == pytest.approx(1.0 / 400.0, abs=1e-6)

    # gyro_x/y/z -> p/q/r, accel_x/y/z -> ax/ay/az
    # gyro_x/y/z -> p/q/r、accel_x/y/z -> ax/ay/az
    assert np.allclose(df["p"].to_numpy(), 0.11)
    assert np.allclose(df["q"].to_numpy(), 0.22)
    assert np.allclose(df["r"].to_numpy(), 0.33)
    assert np.allclose(df["ax"].to_numpy(), 1.1)
    assert np.allclose(df["ay"].to_numpy(), 2.2)
    assert np.allclose(df["az"].to_numpy(), -9.81)
    # gyro_raw_*/accel_raw_* are intentionally left unaliased
    # gyro_raw_*/accel_raw_* は意図的にエイリアスしない
    assert "gyro_raw_x" in df.columns
    assert np.allclose(df["gyro_raw_x"].to_numpy(), 9.9)

    # pos_x/y/z -> x/y/z, vel_x/y/z -> vx/vy/vz
    # pos_x/y/z -> x/y/z、vel_x/y/z -> vx/vy/vz
    assert np.allclose(df["x"].to_numpy(), 1.0)
    assert np.allclose(df["y"].to_numpy(), 2.0)
    assert np.allclose(df["z"].to_numpy(), -0.5)
    assert np.allclose(df["vx"].to_numpy(), 0.1)
    assert np.allclose(df["vy"].to_numpy(), 0.2)

    # quaternion is kept as-is (no short educational alias exists for it)
    # クォータニオンはそのまま（短い教育用エイリアスは存在しない）
    for col in ("quat_w", "quat_x", "quat_y", "quat_z"):
        assert col in df.columns

    # duty_FR/RR/RL/FL -> m1/m2/m3/m4 (FR=M1, RR=M2, RL=M3, FL=M4)
    # duty_FR/RR/RL/FL -> m1/m2/m3/m4（FR=M1, RR=M2, RL=M3, FL=M4）
    assert np.allclose(df["m1"].to_numpy(), 0.51)
    assert np.allclose(df["m2"].to_numpy(), 0.52)
    assert np.allclose(df["m3"].to_numpy(), 0.53)
    assert np.allclose(df["m4"].to_numpy(), 0.54)

    # baro.altitude -> alt (held); tof_bottom.distance -> tof (held)
    # baro.altitude -> alt（保持）; tof_bottom.distance -> tof（保持）
    assert np.allclose(df["alt"].to_numpy(), 0.75)
    assert np.allclose(df["tof"].to_numpy(), 0.74)

    # status.voltage -> vbat (held)
    # status.voltage -> vbat（保持）
    assert np.allclose(df["vbat"].to_numpy(), 3.82)


def test_load_flight_log_bundle_time_zero_false_keeps_boot_relative(tmp_path):
    """time_zero=False leaves the bundle's boot-relative start time
    (converted to seconds, but not shifted) -- same contract as the
    plain-CSV path.
    time_zero=False は一式の起動基準開始時刻を（秒に変換はするが）
    シフトしない -- 素のCSV経路と同じ契約。
    """
    path, start_us = _synthetic_bundle(tmp_path)
    df = load_flight_log(path, time_zero=False)
    assert df["time"].iloc[0] == pytest.approx(start_us / 1e6)


def test_load_flight_log_directory_bundle(tmp_path):
    """An extracted-directory bundle works the same as a `.sflog.zip`.
    展開済みフォルダの一式も `.sflog.zip` と同様に動作する。"""
    n = 5
    ts_us = np.arange(n) * 2500 + 1_000_000
    seq = np.arange(n)
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    directory = tmp_path / "edu_dir_bundle"
    meta = make_meta(source="sim", tool_name="test_log_utils_bundle.py",
                     tool_version="0", streams={"imu": imu})
    FlightLog(meta=meta, schema={}, streams={"imu": imu}).save(directory)

    df = load_flight_log(directory)
    assert len(df) == n
    assert df["time"].iloc[0] == pytest.approx(0.0)


def test_load_flight_log_plain_csv_backward_compat(tmp_path):
    """A plain CSV (not a bundle) still goes through the original
    pd.read_csv() path unchanged.
    素のCSV（一式でない）は従来どおり pd.read_csv() 経路をそのまま通る。"""
    csv_path = tmp_path / "hand_authored.csv"
    csv_path.write_text("time,pos_x,pos_y\n0.0,1.0,2.0\n0.1,1.1,2.1\n")
    df = load_flight_log(csv_path)
    assert list(df.columns) == ["time", "x", "y"]
    assert df["time"].iloc[0] == pytest.approx(0.0)
