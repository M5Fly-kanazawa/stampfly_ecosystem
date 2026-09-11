#!/usr/bin/env python3
"""
test_bundle_commands.py - `sf trim analyze` and `sf cal plot` read a
StampFly flight-log v1 bundle (docs/plans/flight-log-format-plan.md
section 3.2: trim reads attitude/posvel/flow, cal plot reads mag).
test_bundle_commands.py - `sf trim analyze` と `sf cal plot` が StampFly
フライトログ v1 一式を読めることの試験（計画書 3.2節: trim は
attitude/posvel/flow、cal plot は mag を読む）。

Both commands used to read formats that no longer exist (JSONL and the
vehicle_old USB `.bin` blackbox). These tests build a small synthetic
bundle with `lib/sflog` and drive the same functions the CLI calls, so a
future change to the bundle schema or to `lib/sflog/align.py` that breaks
either command is caught here rather than on a real flight.
両コマンドはもう存在しない形式（JSONL と vehicle_old の USB `.bin`
ブラックボックス）を読んでいた。この試験は `lib/sflog` で小さな合成一式を
作り、CLI が呼ぶのと同じ関数を直接叩く -- 一式のスキーマや
`lib/sflog/align.py` の変更でどちらかが壊れたとき、実飛行ではなくここで
見つかるようにする。
"""

from __future__ import annotations

import argparse
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

import matplotlib
matplotlib.use("Agg")  # headless -- plot_mag_xy() calls plt.show() / 非対話（plt.show() 対策）

from sflog.bundle import FlightLog, make_meta  # noqa: E402

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "calibration"))
import plot_mag_xy  # noqa: E402
from sfcli.commands import cal as cal_cmd  # noqa: E402
from sfcli.commands.trim import analyze_trim  # noqa: E402

# Synthetic hover: long enough to clear analyze_trim()'s default
# hover_start_s=6.0 plus its minimum-window guard (10 s).
# 合成ホバー: analyze_trim() の既定 hover_start_s=6.0 と最小窓（10 s）を
# 超える長さにする。
_DURATION_S = 20.0
_IMU_RATE_HZ = 400.0
_FLOW_RATE_HZ = 100.0
_MAG_RATE_HZ = 25.0
_ROLL_TILT_RAD = 0.01          # injected equilibrium tilt / 注入する平衡傾き
_FLOW_QUALITY = 200
_MAG_CENTER_UT = (20.0, -5.0)  # hard-iron offset / ハードアイアンオフセット
_MAG_RADIUS_UT = 3.0
_MAG_Z_UT = 40.0
_TILT_TOL_RAD = 2e-3
_CSV_PRECISION = 1e-3          # %.7g float round-trip / %.7g の往復精度


def _lockstep(n: int, rate_hz: float) -> tuple:
    ts_us = np.round(np.arange(n) / rate_hz * 1e6).astype(np.int64)
    return ts_us, np.arange(n)


def _hover_bundle(tmp_path: Path, with_flow: bool = True, with_mag: bool = True) -> Path:
    """imu + attitude (constant small roll tilt) + posvel (stationary) at
    400Hz, plus optional flow (100Hz) and mag (25Hz, circle around a
    hard-iron offset).
    imu + attitude（一定の小さなロール傾き）+ posvel（静止）を 400Hz で、
    加えて任意の flow（100Hz）・mag（25Hz、ハードアイアンオフセット周りの
    円）。
    """
    n = int(_DURATION_S * _IMU_RATE_HZ)
    ts_us, seq = _lockstep(n, _IMU_RATE_HZ)

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -9.81),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -9.81),
    })
    half = _ROLL_TILT_RAD / 2.0
    attitude = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "quat_w": np.full(n, np.cos(half)), "quat_x": np.full(n, np.sin(half)),
        "quat_y": np.zeros(n), "quat_z": np.zeros(n),
        "gyro_bias_x": np.zeros(n), "gyro_bias_y": np.zeros(n), "gyro_bias_z": np.zeros(n),
        "accel_bias_x": np.zeros(n), "accel_bias_y": np.zeros(n), "accel_bias_z": np.zeros(n),
    })
    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": np.zeros(n), "pos_y": np.zeros(n), "pos_z": np.full(n, -0.5),
        "vel_x": np.zeros(n), "vel_y": np.zeros(n), "vel_z": np.zeros(n),
    })
    streams = {"imu": imu, "attitude": attitude, "posvel": posvel}

    if with_flow:
        n_flow = int(_DURATION_S * _FLOW_RATE_HZ)
        streams["flow"] = pd.DataFrame({
            "timestamp_us": np.round(np.arange(n_flow) / _FLOW_RATE_HZ * 1e6).astype(np.int64),
            "dx": np.zeros(n_flow, dtype=int), "dy": np.zeros(n_flow, dtype=int),
            "quality": np.full(n_flow, _FLOW_QUALITY, dtype=int),
        })
    if with_mag:
        n_mag = int(_DURATION_S * _MAG_RATE_HZ)
        theta = np.linspace(0.0, 2.0 * np.pi, n_mag)
        streams["mag"] = pd.DataFrame({
            "timestamp_us": np.round(np.arange(n_mag) / _MAG_RATE_HZ * 1e6).astype(np.int64),
            "x": _MAG_CENTER_UT[0] + _MAG_RADIUS_UT * np.cos(theta),
            "y": _MAG_CENTER_UT[1] + _MAG_RADIUS_UT * np.sin(theta),
            "z": np.full(n_mag, _MAG_Z_UT),
        })

    meta = make_meta(source="vehicle", tool_name="test_bundle_commands.py",
                     tool_version="0", streams=streams)
    path = tmp_path / "hover.sflog.zip"
    FlightLog(meta=meta, schema={}, streams=streams).save(path)
    return path


# =============================================================================
# sf trim analyze
# =============================================================================

def test_trim_analyze_recovers_injected_roll_tilt(tmp_path):
    """The equilibrium tilt baked into attitude.csv comes back as the
    suggested trim delta (magnitude; the sign convention is the analysis'
    own business), pitch stays ~0, and flow quality is averaged.
    attitude.csv に埋め込んだ平衡傾きが提案トリム差分（大きさ。符号規約は
    解析側の責任）として返り、ピッチは ~0、flow 品質は平均される。
    """
    path = _hover_bundle(tmp_path)
    result = analyze_trim(path)

    assert result["log"] == path.name
    assert result["window"]["duration_s"] > 0
    assert abs(result["delta_trim_rad"]["roll"]) == pytest.approx(_ROLL_TILT_RAD, abs=_TILT_TOL_RAD)
    assert result["delta_trim_rad"]["pitch"] == pytest.approx(0.0, abs=_TILT_TOL_RAD)
    assert result["drift"]["horiz_displacement_m"] == pytest.approx(0.0, abs=1e-6)
    assert result["drift"]["flow_squal_mean"] == pytest.approx(_FLOW_QUALITY)


def test_trim_analyze_flow_is_optional(tmp_path):
    """A bundle without flow.csv still analyzes (flow_squal_mean is None).
    flow.csv の無い一式でも解析できる（flow_squal_mean は None）。"""
    path = _hover_bundle(tmp_path, with_flow=False)
    result = analyze_trim(path)
    assert result["drift"]["flow_squal_mean"] is None


def test_trim_analyze_requires_attitude_and_posvel(tmp_path):
    """imu-only bundle -> clear ValueError, not a KeyError deep inside.
    imu だけの一式 -> 内部の KeyError ではなく明確な ValueError。"""
    n = 10
    ts_us, seq = _lockstep(n, _IMU_RATE_HZ)
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    path = tmp_path / "imu_only.sflog.zip"
    meta = make_meta(source="vehicle", tool_name="test_bundle_commands.py",
                     tool_version="0", streams={"imu": imu})
    FlightLog(meta=meta, schema={}, streams={"imu": imu}).save(path)
    with pytest.raises(ValueError):
        analyze_trim(path)


# =============================================================================
# sf cal plot
# =============================================================================

def test_cal_plot_mag_xy_from_bundle_writes_png(tmp_path):
    """plot_mag_xy reads mag.csv (x/y/z in uT) and saves the XY figure.
    plot_mag_xy が mag.csv（x/y/z、uT）を読んで XY 図を保存する。"""
    path = _hover_bundle(tmp_path)
    mag_df = plot_mag_xy.load_mag_bundle(path)
    assert list(mag_df.columns) == ["timestamp_us", "x", "y", "z"]
    # Bundle CSVs are written with %.7g, so compare at that precision.
    # 一式の CSV は %.7g で書かれるため、その桁数で比較する。
    assert mag_df["x"].min() == pytest.approx(_MAG_CENTER_UT[0] - _MAG_RADIUS_UT, abs=_CSV_PRECISION)
    assert mag_df["y"].max() == pytest.approx(_MAG_CENTER_UT[1] + _MAG_RADIUS_UT, abs=_CSV_PRECISION)

    png = tmp_path / "mag_xy.png"
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # plt.show() on Agg / Agg 上の plt.show()
        plot_mag_xy.plot_mag_xy(mag_df, str(png))
    assert png.exists() and png.stat().st_size > 0


def test_cal_plot_command_end_to_end(tmp_path):
    """`sf cal plot <bundle> -o <png>` through the command's own run
    function (same wiring the CLI uses).
    `sf cal plot <bundle> -o <png>` をコマンド自身の実行関数経由で（CLI と
    同じ配線）。"""
    path = _hover_bundle(tmp_path)
    png = tmp_path / "cal_plot.png"
    args = argparse.Namespace(file=str(path), output=str(png))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        rc = cal_cmd.run_plot(args)
    assert rc == 0
    assert png.exists() and png.stat().st_size > 0


def test_cal_plot_rejects_bundle_without_mag(tmp_path):
    """No mag.csv -> ValueError from the loader, non-zero exit from the
    command (a clean message, never a traceback).
    mag.csv 無し -> ローダーは ValueError、コマンドは非ゼロ終了（トレース
    バックではなく明確なメッセージ）。"""
    path = _hover_bundle(tmp_path, with_mag=False)
    with pytest.raises(ValueError):
        plot_mag_xy.load_mag_bundle(path)
    args = argparse.Namespace(file=str(path), output=str(tmp_path / "unused.png"))
    assert cal_cmd.run_plot(args) != 0
