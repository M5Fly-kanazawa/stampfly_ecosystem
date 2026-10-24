#!/usr/bin/env python3
"""
test_doctor_sdkconfig.py - tests for doctor._check_sdkconfig and
espidf.backup_stale_sdkconfig (stale sdkconfig detection / backup).
test_doctor_sdkconfig.py - doctor._check_sdkconfig と
espidf.backup_stale_sdkconfig（古い sdkconfig の検出・退避）の試験。
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.commands import doctor  # noqa: E402
from sfcli.utils import espidf  # noqa: E402

DEFAULTS = "CONFIG_A=y\n"
STALE = "# CONFIG_A is not set\n"
MATCHING = "CONFIG_A=y\n"


def _project(root, name, defaults, sdkconfig):
    """Create a project dir / プロジェクトのディレクトリを作る"""
    d = root / name
    d.mkdir(parents=True)
    if defaults is not None:
        (d / "sdkconfig.defaults").write_text(defaults)
    if sdkconfig is not None:
        (d / "sdkconfig").write_text(sdkconfig)
    return d


def _tree(tmp_path):
    """Build the tmp firmware tree / 試験用ファームウェアツリーを作る"""
    _project(tmp_path, "ok", DEFAULTS, MATCHING)
    _project(tmp_path, "stale", DEFAULTS, STALE)
    _project(tmp_path, "fresh", DEFAULTS, None)
    _project(tmp_path, "nodefaults", None, STALE)
    _project(tmp_path, "apps/myapp", DEFAULTS, STALE)
    return tmp_path


def _snapshot(root):
    return {p.relative_to(root): p.read_text() for p in root.rglob("*") if p.is_file()}


def test_check_without_fix_warns_only(tmp_path):
    root = _tree(tmp_path)
    before = _snapshot(root)
    warnings = []
    doctor._check_sdkconfig(root, warnings, fix=False)
    assert sorted(warnings) == [
        "firmware/apps/myapp/sdkconfig does not match sdkconfig.defaults",
        "firmware/stale/sdkconfig does not match sdkconfig.defaults",
    ]
    assert _snapshot(root) == before


def test_check_with_fix_backs_up(tmp_path):
    root = _tree(tmp_path)
    warnings = []
    doctor._check_sdkconfig(root, warnings, fix=True)
    assert warnings == []
    for name in ("stale", "apps/myapp"):
        assert not (root / name / "sdkconfig").exists()
        backups = list((root / name).glob("sdkconfig.stale-backup-*"))
        assert len(backups) == 1
        assert backups[0].read_text() == STALE
    assert (root / "ok" / "sdkconfig").read_text() == MATCHING
    assert (root / "nodefaults" / "sdkconfig").read_text() == STALE
    assert not (root / "fresh" / "sdkconfig").exists()


def test_fix_failure_keeps_warning(tmp_path):
    root = _tree(tmp_path)
    with mock.patch.object(espidf, "backup_stale_sdkconfig", return_value=None):
        warnings = []
        doctor._check_sdkconfig(root, warnings, fix=True)
    assert len(warnings) == 2


def test_no_projects(tmp_path):
    warnings = []
    doctor._check_sdkconfig(tmp_path, warnings, fix=False)
    assert warnings == []


def test_backup_does_not_overwrite(tmp_path):
    (tmp_path / "sdkconfig").write_text("new\n")
    (tmp_path / "sdkconfig.stale-backup-20261024-120000").write_text("old\n")
    (tmp_path / "sdkconfig.stale-backup-20261024-120000-1").write_text("old1\n")
    fixed = mock.Mock()
    fixed.strftime.return_value = "20261024-120000"
    with mock.patch.object(espidf.datetime, "datetime") as dt:
        dt.now.return_value = fixed
        backup = espidf.backup_stale_sdkconfig(tmp_path)
    assert backup == tmp_path / "sdkconfig.stale-backup-20261024-120000-2"
    assert backup.read_text() == "new\n"
    assert (tmp_path / "sdkconfig.stale-backup-20261024-120000").read_text() == "old\n"
    assert not (tmp_path / "sdkconfig").exists()


def test_backup_without_sdkconfig(tmp_path):
    assert espidf.backup_stale_sdkconfig(tmp_path) is None


def test_backup_rename_failure_returns_none(tmp_path):
    (tmp_path / "sdkconfig").write_text("x\n")
    with mock.patch.object(Path, "rename", side_effect=OSError("boom")):
        assert espidf.backup_stale_sdkconfig(tmp_path) is None
