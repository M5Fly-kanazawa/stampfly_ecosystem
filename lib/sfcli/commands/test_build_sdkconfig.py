#!/usr/bin/env python3
"""
test_build_sdkconfig.py - tests for espidf.find_sdkconfig_mismatches (stale
sdkconfig vs sdkconfig.defaults detection used by `sf build`).
test_build_sdkconfig.py - espidf.find_sdkconfig_mismatches（`sf build` が使う、
古い sdkconfig と sdkconfig.defaults の食い違い検出）の試験。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.utils import espidf  # noqa: E402


def _write(tmp_path, defaults, sdkconfig, target_defaults=None, target="esp32s3"):
    """Create the project files / プロジェクトのファイルを作る"""
    if defaults is not None:
        (tmp_path / "sdkconfig.defaults").write_text(defaults)
    if sdkconfig is not None:
        (tmp_path / "sdkconfig").write_text(sdkconfig)
    if target_defaults is not None:
        (tmp_path / f"sdkconfig.defaults.{target}").write_text(target_defaults)
    return tmp_path


def test_clean_match(tmp_path):
    _write(tmp_path, "# comment\nCONFIG_A=y\n", "CONFIG_A=y\nCONFIG_B=1\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_n_equals_not_set(tmp_path):
    _write(tmp_path, "CONFIG_A=n\r\n", "# CONFIG_A is not set\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_choice_switch_reported(tmp_path):
    _write(tmp_path, "CONFIG_A=y\n", "# CONFIG_A is not set\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == [("CONFIG_A", "y", "n")]


def test_quoted_string_differs(tmp_path):
    _write(tmp_path, 'CONFIG_P="8MB"\n', 'CONFIG_P="4MB"\n')
    assert espidf.find_sdkconfig_mismatches(tmp_path) == [("CONFIG_P", '"8MB"', '"4MB"')]


def test_key_absent_from_sdkconfig_not_reported(tmp_path):
    _write(tmp_path, "CONFIG_GONE=y\n", "CONFIG_OTHER=y\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_missing_sdkconfig(tmp_path):
    _write(tmp_path, "CONFIG_A=y\n", None)
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_missing_defaults(tmp_path):
    _write(tmp_path, None, "CONFIG_A=y\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_target_defaults_override_base(tmp_path):
    sdkconfig = 'CONFIG_IDF_TARGET="esp32s3"\nCONFIG_A=y\n'
    # Base differs from sdkconfig but the target file overrides it to match.
    # 基本ファイルは食い違うが、ターゲット別ファイルが上書きして一致する。
    _write(tmp_path, "CONFIG_A=n\n", sdkconfig, target_defaults="CONFIG_A=y\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []
    # And the reverse: the target file introduces the mismatch.
    # 逆に、ターゲット別ファイルが食い違いを生む場合。
    (tmp_path / "sdkconfig.defaults.esp32s3").write_text("CONFIG_A=n\n")
    (tmp_path / "sdkconfig.defaults").write_text("CONFIG_A=y\n")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == [("CONFIG_A", "n", "y")]


def test_unreadable_returns_empty(tmp_path):
    _write(tmp_path, "CONFIG_A=y\n", None)
    (tmp_path / "sdkconfig").write_bytes(b"\xff\xfe\x00bad")
    assert espidf.find_sdkconfig_mismatches(tmp_path) == []


def test_warning_lists_key_and_path(tmp_path, capsys):
    _write(tmp_path, "CONFIG_A=y\n", "# CONFIG_A is not set\n")
    espidf.warn_sdkconfig_mismatches(tmp_path, espidf.find_sdkconfig_mismatches(tmp_path))
    captured = capsys.readouterr()
    assert "does not match sdkconfig.defaults" in captured.err
    assert "CONFIG_A: defaults=y, sdkconfig=n (not set)" in captured.out
    assert str(tmp_path / "sdkconfig") in captured.out


def test_no_warning_when_matching(tmp_path, capsys):
    espidf.warn_sdkconfig_mismatches(tmp_path, [])
    captured = capsys.readouterr()
    assert captured.out == "" and captured.err == ""
