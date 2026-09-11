"""
test_bundle_roundtrip.py - FlightLog.save()/.load() round trip, is_bundle().
test_bundle_roundtrip.py - FlightLog の save()/load() 往復、is_bundle()。
"""

from __future__ import annotations

import pandas as pd
import pytest

from sflog.bundle import FlightLog, is_bundle, make_meta


def _synthetic_streams() -> dict:
    """A tiny but schema-valid set of streams: the required "imu" stream
    plus one optional stream ("mag") to exercise multi-stream save/load.
    小さいがスキーマに沿ったストリーム集合: 必須の "imu" と、複数ストリーム
    の保存/読込を確認するための任意ストリーム "mag"。
    """
    imu = pd.DataFrame(
        {
            "timestamp_us": [1000, 2500, 4000],
            "gyro_x": [0.01, 0.02, -0.01],
            "gyro_y": [0.0, -0.01, 0.02],
            "gyro_z": [-0.02, 0.0, 0.01],
            "accel_x": [0.1, 0.2, 0.15],
            "accel_y": [0.0, -0.1, 0.05],
            "accel_z": [-9.8, -9.81, -9.79],
            "gyro_raw_x": [0.01, 0.02, -0.01],
            "gyro_raw_y": [0.0, -0.01, 0.02],
            "gyro_raw_z": [-0.02, 0.0, 0.01],
            "accel_raw_x": [0.1, 0.2, 0.15],
            "accel_raw_y": [0.0, -0.1, 0.05],
            "accel_raw_z": [-9.8, -9.81, -9.79],
        }
    )
    mag = pd.DataFrame(
        {
            "timestamp_us": [1200, 5200],
            "x": [10.5, 11.0],
            "y": [-20.25, -19.75],
            "z": [30.0, 31.5],
        }
    )
    return {"imu": imu, "mag": mag}


def _synthetic_log() -> FlightLog:
    streams = _synthetic_streams()
    meta = make_meta(
        source="vehicle",
        tool_name="test_bundle_roundtrip",
        tool_version="0.0.0",
        streams=streams,
    )
    from sflog import schema

    schema_json = schema.schema_for(streams.keys())
    return FlightLog(meta=meta, schema=schema_json, streams=streams)


def _assert_logs_equal(a: FlightLog, b: FlightLog) -> None:
    assert a.meta["format"] == b.meta["format"]
    assert a.meta["version"] == b.meta["version"]
    assert set(a.streams.keys()) == set(b.streams.keys())
    for name in a.streams:
        left, right = a.streams[name], b.streams[name]
        assert list(left.columns) == list(right.columns), name
        pd.testing.assert_frame_equal(
            left.reset_index(drop=True),
            right.reset_index(drop=True),
            check_dtype=False,
            atol=1e-6,
        )


def test_save_load_zip_roundtrip(tmp_path):
    log = _synthetic_log()
    out_path = tmp_path / "flight_test.sflog.zip"
    log.save(out_path)
    assert out_path.exists()

    loaded = FlightLog.load(out_path)
    _assert_logs_equal(log, loaded)


def test_save_load_directory_roundtrip(tmp_path):
    log = _synthetic_log()
    out_dir = tmp_path / "flight_test_dir"
    log.save(out_dir)
    assert (out_dir / "meta.json").exists()
    assert (out_dir / "schema.json").exists()
    assert (out_dir / "imu.csv").exists()
    assert (out_dir / "mag.csv").exists()

    loaded = FlightLog.load(out_dir)
    _assert_logs_equal(log, loaded)


def test_missing_stream_is_tolerated(tmp_path):
    """A bundle with only "imu" (no optional streams) loads fine and simply
    has no other keys in `.streams`.
    "imu" だけ（任意ストリーム無し）の一式も問題なく読み込め、`.streams`
    に他のキーが単に存在しないだけになる。
    """
    streams = {"imu": _synthetic_streams()["imu"]}
    meta = make_meta(source="vehicle", tool_name="t", tool_version="0", streams=streams)
    from sflog import schema

    log = FlightLog(meta=meta, schema=schema.schema_for(streams.keys()), streams=streams)
    out_path = tmp_path / "imu_only.sflog.zip"
    log.save(out_path)

    loaded = FlightLog.load(out_path)
    assert set(loaded.streams.keys()) == {"imu"}


def test_is_bundle_true_for_zip_and_dir(tmp_path):
    log = _synthetic_log()
    zip_path = tmp_path / "a.sflog.zip"
    dir_path = tmp_path / "a_dir"
    log.save(zip_path)
    log.save(dir_path)

    assert is_bundle(zip_path) is True
    assert is_bundle(dir_path) is True


def test_is_bundle_false_for_unrelated_files(tmp_path):
    not_a_bundle_dir = tmp_path / "not_a_bundle"
    not_a_bundle_dir.mkdir()
    (not_a_bundle_dir / "readme.txt").write_text("hello", encoding="utf-8")
    assert is_bundle(not_a_bundle_dir) is False

    not_a_zip = tmp_path / "not_a_zip.zip"
    not_a_zip.write_bytes(b"not actually a zip file")
    assert is_bundle(not_a_zip) is False

    assert is_bundle(tmp_path / "does_not_exist.sflog.zip") is False


def test_unknown_file_in_bundle_is_ignored(tmp_path):
    """A bundle directory may contain files this schema version does not
    know about (e.g. SILS's results.json) -- loading must ignore them, not
    fail.
    一式のフォルダには本スキーマ版が知らないファイル（SILS の
    results.json 等）が同居してよい -- 読み込みはそれらを無視するべきで
    あり、失敗してはならない。
    """
    log = _synthetic_log()
    out_dir = tmp_path / "with_extra"
    log.save(out_dir)
    (out_dir / "results.json").write_text('{"passed": true}', encoding="utf-8")

    loaded = FlightLog.load(out_dir)
    assert set(loaded.streams.keys()) == {"imu", "mag"}


def test_make_meta_stream_stats():
    streams = _synthetic_streams()
    meta = make_meta(source="vehicle", tool_name="t", tool_version="0", streams=streams)
    imu_stats = meta["streams"]["imu"]
    assert imu_stats["rows"] == 3
    assert imu_stats["first_timestamp_us"] == 1000
    assert imu_stats["last_timestamp_us"] == 4000
    assert imu_stats["nominal_rate_hz"] == 400
    # 2 intervals over (4000-1000)us = 3ms -> (3-1) / 3e-3 s ~= 666.7 Hz
    assert imu_stats["measured_rate_hz"] == pytest.approx((3 - 1) / ((4000 - 1000) / 1e6))
    assert meta["derived"] is False
