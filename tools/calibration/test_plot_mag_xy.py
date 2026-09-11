#!/usr/bin/env python3
"""
test_plot_mag_xy.py - tests for tools/calibration/plot_mag_xy.py reading a
StampFly flight-log v1 bundle's `mag` stream (docs/plans/flight-log-format-
plan.md section 3.2, "sf cal plot" row).
test_plot_mag_xy.py - tools/calibration/plot_mag_xy.py が StampFly
フライトログ v1 一式の `mag` ストリームを読む部分の試験（計画書 3.2節
"sf cal plot" の行）。

Covers:
  - load_mag_bundle() returns the mag stream's timestamp_us/x/y/z columns
    unchanged (uT);
  - a bundle with NO mag stream raises a clear ValueError (matches the old
    `.bin`-parsing code's "no valid magnetometer data" failure mode, but
    now caught before any plotting is attempted);
  - plot_mag_xy() runs end-to-end (matplotlib Agg backend, headless) and
    actually writes a PNG file to a tmp path.

対象:
  - load_mag_bundle() が mag ストリームの timestamp_us/x/y/z 列をそのまま
    （uT）返すこと;
  - `mag` ストリームの無い一式は明確な ValueError を出すこと（旧 `.bin`
    解析コードの「有効な地磁気データが無い」失敗モードに相当するが、
    プロット処理に入る前に検出される点が異なる）;
  - plot_mag_xy() が一気通貫で動作し（matplotlib Agg バックエンド、
    ヘッドレス）、実際に tmp パスへ PNG ファイルを書き出すこと。
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless -- must be set before importing plot_mag_xy,
                        # which imports matplotlib.pyplot at module load time.
                        # ヘッドレス -- plot_mag_xy を import する前に設定する
                        # こと（同モジュールは読み込み時に matplotlib.pyplot
                        # を import する）。

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))  # tools/calibration/
import plot_mag_xy  # noqa: E402

from sflog.bundle import FlightLog, make_meta  # noqa: E402


def _bundle_with_mag(tmp_path, include_mag: bool = True) -> Path:
    n = 20
    ts_us = np.arange(n) * 40_000  # 25Hz, matches mag's nominal rate
    imu = pd.DataFrame({
        "timestamp_us": np.arange(n) * 2_500, "seq": np.arange(n),
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    streams = {"imu": imu}
    if include_mag:
        theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
        streams["mag"] = pd.DataFrame({
            "timestamp_us": ts_us,
            "x": 30.0 * np.cos(theta) + 5.0,   # hard-iron offset +5 uT on X
            "y": 30.0 * np.sin(theta) - 3.0,   # and -3 uT on Y
            "z": np.full(n, 45.0),
        })
    meta = make_meta(source="sim", tool_name="test_plot_mag_xy.py", tool_version="0",
                     streams=streams)
    path = tmp_path / ("with_mag.sflog.zip" if include_mag else "no_mag.sflog.zip")
    FlightLog(meta=meta, schema={}, streams=streams).save(path)
    return path


def test_load_mag_bundle_returns_xyz(tmp_path):
    path = _bundle_with_mag(tmp_path)
    mag_df = plot_mag_xy.load_mag_bundle(str(path))
    assert len(mag_df) == 20
    for col in ("timestamp_us", "x", "y", "z"):
        assert col in mag_df.columns
    assert mag_df["z"].iloc[0] == pytest.approx(45.0)


def test_load_mag_bundle_missing_stream_raises(tmp_path):
    path = _bundle_with_mag(tmp_path, include_mag=False)
    with pytest.raises(ValueError, match="no mag stream"):
        plot_mag_xy.load_mag_bundle(str(path))


def test_plot_mag_xy_writes_png(tmp_path):
    """End-to-end: load the bundle's mag stream, then actually render and
    save a PNG (not just "does it raise").
    一気通貫: 一式の mag ストリームを読み、実際に PNG をレンダリング・
    保存する（「例外が出ないか」だけでなく）。
    """
    path = _bundle_with_mag(tmp_path)
    mag_df = plot_mag_xy.load_mag_bundle(str(path))

    output_png = tmp_path / "mag_xy.png"
    plot_mag_xy.plot_mag_xy(mag_df, str(output_png))

    assert output_png.exists()
    assert output_png.stat().st_size > 0
