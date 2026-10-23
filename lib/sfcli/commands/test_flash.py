#!/usr/bin/env python3
"""
test_flash.py - tests for the non-legacy path of lib/sfcli/commands/flash.py
(esptool --after watchdog_reset, optional monitor follow-up). subprocess is mocked.
test_flash.py - flash.py の通常経路（esptool --after watchdog_reset、
任意のモニタ後続実行）の試験。subprocess はモックする。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.commands import flash  # noqa: E402


def _setup(tmp_path, chip_json=True, flash_args=True):
    build = tmp_path / "build"
    build.mkdir()
    if flash_args:
        (build / "flash_args").write_text("--flash_mode dio\n")
    if chip_json:
        (build / "flasher_args.json").write_text(
            json.dumps({"extra_esptool_args": {"chip": "esp32s3"}}))
    return build


def _run(tmp_path, **kw):
    args = flash.make_run_args(port="/dev/ttyACM0", **kw)
    with mock.patch.object(flash.paths, "firmware_target_dir", return_value=tmp_path), \
         mock.patch.object(flash.platform, "esp_idf_path", return_value=Path("/idf")), \
         mock.patch.object(flash.espidf, "prepare_idf_env", return_value={}), \
         mock.patch.object(flash.espidf, "verify_idf_env", return_value=None), \
         mock.patch.object(flash.subprocess, "run",
                           return_value=mock.Mock(returncode=0)) as run:
        rc = flash.run(args)
    return rc, run


def test_incremental_build_precedes_esptool(tmp_path):
    _setup(tmp_path)
    rc, run = _run(tmp_path)
    assert rc == 0
    assert run.call_count == 2
    build_cmd = run.call_args_list[0].args[0]
    assert build_cmd[-1] == "build"
    assert run.call_args_list[0].kwargs["cwd"] == tmp_path
    assert "esptool" in run.call_args_list[1].args[0]


def test_failed_build_skips_flash(tmp_path):
    _setup(tmp_path)
    args = flash.make_run_args(port="p")
    with mock.patch.object(flash.paths, "firmware_target_dir", return_value=tmp_path), \
         mock.patch.object(flash.platform, "esp_idf_path", return_value=Path("/idf")), \
         mock.patch.object(flash.espidf, "prepare_idf_env", return_value={}), \
         mock.patch.object(flash.espidf, "verify_idf_env", return_value=None), \
         mock.patch.object(flash.subprocess, "run",
                           return_value=mock.Mock(returncode=3)) as run:
        assert flash.run(args) == 3
    assert run.call_count == 1


def test_esptool_watchdog_reset(tmp_path):
    build = _setup(tmp_path)
    rc, run = _run(tmp_path)
    assert rc == 0
    assert run.call_count == 2
    cmd = run.call_args.args[0]
    assert cmd[:3] == [sys.executable, "-m", "esptool"]
    assert cmd[cmd.index("--after") + 1] == "watchdog_reset"
    assert cmd[cmd.index("--before") + 1] == "default_reset"
    assert cmd[cmd.index("--chip") + 1] == "esp32s3"
    assert cmd[-2:] == ["write_flash", "@flash_args"]
    assert run.call_args.kwargs["cwd"] == build


def test_monitor_follows_flash(tmp_path):
    _setup(tmp_path)
    rc, run = _run(tmp_path, monitor=True)
    assert rc == 0
    assert run.call_count == 3
    mon = run.call_args_list[2]
    assert mon.args[0][-3:] == ["-p", "/dev/ttyACM0", "monitor"]
    assert mon.kwargs["cwd"] == tmp_path


def test_monitor_skipped_on_flash_failure(tmp_path):
    _setup(tmp_path)
    args = flash.make_run_args(port="p", monitor=True)
    with mock.patch.object(flash.paths, "firmware_target_dir", return_value=tmp_path), \
         mock.patch.object(flash.platform, "esp_idf_path", return_value=Path("/idf")), \
         mock.patch.object(flash.espidf, "prepare_idf_env", return_value={}), \
         mock.patch.object(flash.espidf, "verify_idf_env", return_value=None), \
         mock.patch.object(flash.subprocess, "run",
                           side_effect=[mock.Mock(returncode=0),
                                     mock.Mock(returncode=2)]) as run:
        assert flash.run(args) == 2
    assert run.call_count == 2


def test_chip_fallback_and_missing_flash_args(tmp_path):
    _setup(tmp_path, chip_json=False)
    rc, run = _run(tmp_path)
    assert run.call_args.args[0][run.call_args.args[0].index("--chip") + 1] == "esp32s3"


def test_missing_flash_args_errors(tmp_path):
    _setup(tmp_path, flash_args=False)
    rc, run = _run(tmp_path)
    assert rc == 1
    assert run.call_count == 1  # only the build ran, no esptool
