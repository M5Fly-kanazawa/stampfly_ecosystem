"""Tests for the bundle output-path rule shared by `sf log wifi -o` and
`sf log convert -o`, and for content-based bundle discovery.
`sf log wifi -o` / `sf log convert -o` 共通の出力パス規則と、中身による
一式検出のテスト。

Run: python3 -m pytest tools/log_analyzer/test_log_output_path.py -q
"""
from pathlib import Path
import shutil

import pytest

import sflog
from sfcli.commands.log import _bundle_output_path, _iter_bundles

REFERENCE = Path(__file__).resolve().parents[2] / "analysis" / "datasets" / "flightlog" / \
    "vehicle_hover_20260908T121243.sflog.zip"


def test_bare_name_gets_sflog_zip(tmp_path):
    out = _bundle_output_path("flight1", tmp_path / "default.sflog.zip")
    assert out == Path("flight1.sflog.zip")


def test_zip_path_is_kept(tmp_path):
    assert _bundle_output_path("a/b.sflog.zip", tmp_path / "d.sflog.zip") == Path("a/b.sflog.zip")


def test_existing_directory_gets_default_zip_inside(tmp_path):
    out = _bundle_output_path(str(tmp_path), tmp_path / "flight_x.sflog.zip")
    assert out == tmp_path / "flight_x.sflog.zip"


def test_trailing_separator_means_directory(tmp_path):
    out = _bundle_output_path("logs/", Path("logs/flight_x.sflog.zip"))
    assert out == Path("logs") / "flight_x.sflog.zip"


@pytest.mark.parametrize("name", ["x.csv", "x.jsonl", "x.bin"])
def test_derived_extensions_are_rejected(name, tmp_path):
    with pytest.raises(ValueError):
        _bundle_output_path(name, tmp_path / "d.sflog.zip")


def test_none_returns_default(tmp_path):
    assert _bundle_output_path(None, tmp_path / "d.sflog.zip") == tmp_path / "d.sflog.zip"


@pytest.mark.skipif(not REFERENCE.exists(), reason="reference bundle not present")
def test_renamed_bundle_is_still_detected_and_loaded(tmp_path):
    # Detection is by content (meta.json inside the zip), not by extension.
    # 判定は拡張子ではなく中身（zip 内の meta.json）。
    renamed = tmp_path / "flight_renamed.sflog"
    shutil.copy(REFERENCE, renamed)
    other = tmp_path / "not_a_bundle.zip"
    other.write_bytes(b"PK\x05\x06" + b"\x00" * 18)  # empty zip, no meta.json
    assert sflog.is_bundle(renamed)
    assert not sflog.is_bundle(other)
    log = sflog.FlightLog.load(renamed)
    assert "imu" in log.streams
    found = [p for p, _meta in _iter_bundles(tmp_path)]
    assert found == [renamed]
