#!/usr/bin/env python3
"""
test_trim.py - tests for lib/sfcli/commands/trim.py's analyze_trim() reading
a StampFly flight-log v1 bundle (docs/plans/flight-log-format-plan.md
section 3.2, "sf trim analyze" row).
test_trim.py - lib/sfcli/commands/trim.py の analyze_trim() が StampFly
フライトログ v1 一式を読む部分の試験（計画書 3.2節 "sf trim analyze" の行）。

Builds a synthetic bundle with `attitude` (quaternion) and `posvel`
(position/velocity) streams holding a constant attitude and a linearly
drifting NED velocity (a deliberately simple, noise-free scenario so the
expected trim numbers can be checked against the documented formula in
analyze_trim() directly, rather than merely asserting "it runs").

合成一式を作る: `attitude`（クォータニオン）・`posvel`（位置・速度）
ストリームに、一定姿勢と線形にドリフトする NED 速度を持たせる（意図的に
単純・無ノイズのシナリオにし、「動くことだけ」ではなく analyze_trim() の
文書化された式どおりの数値が出ることを直接確認できるようにする）。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.commands.trim import analyze_trim, G  # noqa: E402

from sflog.bundle import FlightLog, make_meta  # noqa: E402


def _euler_to_quat(roll, pitch, yaw):
    """Same ZYX convention as lib/stampfly_edu/generate_samples.py's
    _euler_deg_to_quat(), but taking radians (trim.py's own _q2eul() is the
    inverse of this).
    lib/stampfly_edu/generate_samples.py の _euler_deg_to_quat() と同じ
    ZYX規約（ラジアン入力。trim.py 自身の _q2eul() がこの逆変換）。
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def _build_bundle(tmp_path, roll=math.radians(2.0), pitch=math.radians(-1.0),
                  yaw=math.radians(30.0), a_north=0.01, a_east=-0.02,
                  duration_s=20.0, dt=0.1, include_flow=True):
    """A constant-attitude, linearly-drifting-velocity hover bundle.
    一定姿勢・線形ドリフト速度のホバー一式。"""
    n = int(duration_s / dt)
    t = np.arange(n) * dt
    ts_us = np.round(t * 1e6).astype(np.int64)
    seq = np.arange(n)

    w, x, y, z = _euler_to_quat(roll, pitch, yaw)
    attitude = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "quat_w": np.full(n, w), "quat_x": np.full(n, x),
        "quat_y": np.full(n, y), "quat_z": np.full(n, z),
        "gyro_bias_x": np.zeros(n), "gyro_bias_y": np.zeros(n), "gyro_bias_z": np.zeros(n),
        "accel_bias_x": np.zeros(n), "accel_bias_y": np.zeros(n), "accel_bias_z": np.zeros(n),
    })

    vel_north = a_north * t
    vel_east = a_east * t
    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": np.cumsum(vel_north) * dt, "pos_y": np.cumsum(vel_east) * dt,
        "pos_z": np.full(n, -0.5),
        "vel_x": vel_north, "vel_y": vel_east, "vel_z": np.zeros(n),
    })

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -9.81),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -9.81),
    })

    streams = {"imu": imu, "attitude": attitude, "posvel": posvel}
    if include_flow:
        streams["flow"] = pd.DataFrame({
            "timestamp_us": ts_us,
            "dx": np.zeros(n, dtype=int), "dy": np.zeros(n, dtype=int),
            "quality": np.full(n, 180, dtype=int),
        })

    meta = make_meta(source="sim", tool_name="test_trim.py", tool_version="0", streams=streams)
    path = tmp_path / "trim_test.sflog.zip"
    FlightLog(meta=meta, schema={}, streams=streams).save(path)
    return path


def test_analyze_trim_matches_documented_formula(tmp_path):
    roll, pitch, yaw = math.radians(2.0), math.radians(-1.0), math.radians(30.0)
    a_north, a_east = 0.01, -0.02
    path = _build_bundle(tmp_path, roll=roll, pitch=pitch, yaw=yaw,
                         a_north=a_north, a_east=a_east)

    result = analyze_trim(path, hover_start_s=6.0)

    # Held attitude is exactly what we injected (constant, no noise) --
    # tolerance is loose enough to absorb the bundle CSV's "%.7g" float
    # serialization (see lib/sflog/bundle.py _dataframe_to_csv_text()),
    # not the analysis math.
    # 保った姿勢は注入した値そのもの（一定・無ノイズ）-- 許容誤差は一式CSVの
    # "%.7g" 浮動小数点シリアライズ（lib/sflog/bundle.py の
    # _dataframe_to_csv_text() 参照）を吸収する程度に緩め、解析側の数式の
    # 誤差ではない。
    assert result["attitude_mean_deg"]["roll"] == pytest.approx(math.degrees(roll), abs=1e-4)
    assert result["attitude_mean_deg"]["pitch"] == pytest.approx(math.degrees(pitch), abs=1e-4)
    assert result["attitude_mean_deg"]["yaw"] == pytest.approx(math.degrees(yaw), abs=1e-4)

    # a_north/a_east recovered exactly by the linear fit (noise-free linear
    # velocity ramp).
    # a_north/a_east は線形フィットで厳密に復元される（無ノイズの線形速度
    # ランプ）。
    an, ae = result["drift"]["accel_ned_mps2"]
    assert an == pytest.approx(a_north, abs=1e-6)
    assert ae == pytest.approx(a_east, abs=1e-6)

    # Body-frame rotation (yaw-only, per analyze_trim()'s own comment) and
    # the trim formula itself, reproduced here independently to cross-check
    # against the function's output (not just re-running its own code).
    # 機体座標系への回転（ヨーのみ、analyze_trim() 自身のコメントどおり）と
    # トリム式そのものを、ここで独立に再現して関数の出力と突き合わせる
    # （関数自身のコードを再実行するだけではない）。
    cy, sy = math.cos(yaw), math.sin(yaw)
    ax_body = cy * an + sy * ae
    ay_body = -sy * an + cy * ae
    expected_roll_trim = roll - ay_body / G
    expected_pitch_trim = pitch + ax_body / G

    st = result["suggested_trim_rad"]
    assert st["roll"] == pytest.approx(expected_roll_trim, abs=1e-5)
    assert st["pitch"] == pytest.approx(expected_pitch_trim, abs=1e-5)

    # current_trim defaults to 0 -> delta == suggested
    # current_trim の既定は0 -> delta == suggested
    assert result["delta_trim_rad"]["roll"] == pytest.approx(st["roll"], abs=1e-9)

    # flow stream present -> squal reported
    # flow ストリームあり -> squal が報告される
    assert result["drift"]["flow_squal_mean"] == pytest.approx(180.0)


def test_analyze_trim_current_trim_offsets_delta(tmp_path):
    """--current-roll/--current-pitch shift delta but not the suggested
    (absolute) trim -- delta = suggested - current.
    --current-roll/--current-pitch は delta を動かすが suggested（絶対値）の
    トリムは動かさない -- delta = suggested - current。
    """
    path = _build_bundle(tmp_path)
    baseline = analyze_trim(path, hover_start_s=6.0)
    offset = analyze_trim(path, hover_start_s=6.0, current_roll=0.01, current_pitch=-0.02)

    assert offset["suggested_trim_rad"]["roll"] == pytest.approx(
        baseline["suggested_trim_rad"]["roll"], abs=1e-9)
    assert offset["delta_trim_rad"]["roll"] == pytest.approx(
        baseline["suggested_trim_rad"]["roll"] - 0.01, abs=1e-6)
    assert offset["delta_trim_rad"]["pitch"] == pytest.approx(
        baseline["suggested_trim_rad"]["pitch"] - (-0.02), abs=1e-6)


def test_analyze_trim_no_flow_stream_reports_none(tmp_path):
    """A bundle without a `flow` stream is accepted (flow is optional) and
    reports flow_squal_mean=None, mirroring the old JSONL loader's handling
    of a log with no flow records.
    `flow` ストリームの無い一式も受け付ける（flow は任意）。
    flow_squal_mean は None を報告する（flow レコードの無い JSONL に対する
    旧ローダーの扱いと同じ）。
    """
    path = _build_bundle(tmp_path, include_flow=False)
    result = analyze_trim(path, hover_start_s=6.0)
    assert result["drift"]["flow_squal_mean"] is None


def test_analyze_trim_missing_streams_raises(tmp_path):
    """A bundle missing `attitude`/`posvel` raises ValueError (not a
    silent/garbage result).
    `attitude`/`posvel` の無い一式は ValueError（無言で誤った結果を返さない）。
    """
    n = 10
    ts_us = np.arange(n) * 2500
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": np.arange(n),
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    meta = make_meta(source="sim", tool_name="test_trim.py", tool_version="0", streams={"imu": imu})
    path = tmp_path / "no_attitude.sflog.zip"
    FlightLog(meta=meta, schema={}, streams={"imu": imu}).save(path)

    with pytest.raises(ValueError):
        analyze_trim(path)


def test_analyze_trim_short_window_raises(tmp_path):
    """A hover window shorter than 10s raises ValueError.
    10秒未満のホバー区間は ValueError。"""
    path = _build_bundle(tmp_path, duration_s=8.0)  # < hover_start_s(6) + 10s minimum
    with pytest.raises(ValueError):
        analyze_trim(path, hover_start_s=6.0)
