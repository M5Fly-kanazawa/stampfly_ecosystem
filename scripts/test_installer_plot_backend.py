r"""
test_installer_plot_backend.py - Tests for the installer's matplotlib GUI
backend root-fix.

インストーラのmatplotlib GUIバックエンド根本対処のテスト。

scripts/installer.py's Installer._ensure_plot_backend() (Step 3/4 sub-step)
and its _probe_plot_backend() helper are the install-time ROOT fix for `sf
log viz` opening no window on a Windows attendee PC with an incomplete
Tcl/Tk (see lib/sfcli/utils/plotting.py's module docstring for the
run-time PNG-fallback safety net this complements). This test loads
scripts/installer.py the same way scripts/test_gui_installer_parity.py
does (importlib, not a package import, since scripts/ is not a package)
and drives both functions with fakes -- no real subprocess, no real pip,
no real ESP-IDF venv.

scripts/installer.py の Installer._ensure_plot_backend()（Step3/4の
サブステップ）と、その補助関数 _probe_plot_backend() は、不完全な
Tcl/TkのWindows受講者PCで `sf log viz` がウィンドウを一切開かない問題の
インストール時の根本対処である（これが補完する実行時のPNGフォールバック
安全網は lib/sfcli/utils/plotting.py のモジュールdocstring参照）。本テストは
scripts/test_gui_installer_parity.py と同じ方法（scripts/ はパッケージで
はないため importlib）で scripts/installer.py を読み込み、両関数を偽物で
駆動する -- 実subprocess・実pip・実ESP-IDF venvは一切使わない。

Run: pytest scripts/test_installer_plot_backend.py -v
"""

import importlib.util
import json
import subprocess
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
INSTALLER_PATH = REPO_ROOT / "scripts" / "installer.py"

# Requirement string used across these tests, deliberately hardcoded
# rather than imported from lib/sfcli/utils/plotting.QT_FALLBACK_REQUIREMENT
# -- these tests exercise scripts/installer.py in isolation from sfcli
# (see _probe_plot_backend()'s module comment on why installer.py itself
# never imports sfcli), and stand in for whatever a real
# `--probe` JSON payload would report.
# 全テストで使う要件文字列。意図的に
# lib/sfcli/utils/plotting.QT_FALLBACK_REQUIREMENT から import せず
# ハードコードする -- 本テストはscripts/installer.pyをsfcliから切り離して
# 単体検証する（installer.py自身がsfcliを一切importしない理由は
# _probe_plot_backend()のモジュールコメント参照）。実際の`--probe`のJSON
# 応答が返しうる値の代役。
FAKE_REQUIREMENT = "PyQt6>=6.5,<7"


def _load_module(module_name: str, path: Path) -> ModuleType:
    """Load `path` as a standalone module without adding it to sys.modules
    under its real package name (mirrors
    scripts/test_gui_installer_parity.py's own copy of this helper --
    duplicated rather than shared since these two test files have no
    common import target).
    `path` を、実パッケージ名で sys.modules に登録せず単独モジュールとして
    ロードする（scripts/test_gui_installer_parity.py 自身の同名ヘルパーを
    模す -- 共有先が無いため複製する）。"""
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def installer() -> ModuleType:
    return _load_module("_test_installer_plot_backend", INSTALLER_PATH)


def _fake_probe(payload: dict):
    """Return a `_probe_plot_backend`-shaped stand-in that ignores its
    venv_python argument and always answers with `payload`.
    venv_python 引数を無視して常に `payload` を返す、
    `_probe_plot_backend` 形の代役を返す。"""
    return lambda venv_python: payload


# --- _probe_plot_backend() ---

def test_probe_plot_backend_parses_last_json_line(installer: ModuleType, monkeypatch, tmp_path) -> None:
    payload = {"backend": "qtagg", "interactive": True, "reason": "",
               "has_display": True, "fallback_requirement": FAKE_REQUIREMENT}
    stdout = "some unrelated noise on stdout\n" + json.dumps(payload) + "\n"
    monkeypatch.setattr(
        installer.subprocess, "run",
        lambda *a, **k: subprocess.CompletedProcess(args=a, returncode=0, stdout=stdout, stderr=""),
    )

    assert installer._probe_plot_backend(tmp_path / "python") == payload


def test_probe_plot_backend_none_on_nonzero_exit(installer: ModuleType, monkeypatch, tmp_path) -> None:
    # Exit 1 is what plotting.py's --probe uses for "matplotlib is not
    # even installed" -- still valid JSON, but the installer must treat
    # it as "could not check", not misread it as backend info.
    # exit 1 は plotting.py の --probe が「matplotlibがそもそも
    # 未インストール」に使う値 -- JSONとしては有効だが、インストーラは
    # これを「確認できなかった」として扱うべきで、バックエンド情報として
    # 誤読してはならない。
    monkeypatch.setattr(
        installer.subprocess, "run",
        lambda *a, **k: subprocess.CompletedProcess(
            args=a, returncode=1, stdout=json.dumps({"error": "matplotlib not installed"}) + "\n", stderr="",
        ),
    )

    assert installer._probe_plot_backend(tmp_path / "python") is None


def test_probe_plot_backend_none_on_bad_json(installer: ModuleType, monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(
        installer.subprocess, "run",
        lambda *a, **k: subprocess.CompletedProcess(args=a, returncode=0, stdout="not json at all\n", stderr=""),
    )

    assert installer._probe_plot_backend(tmp_path / "python") is None


def test_probe_plot_backend_none_on_timeout(installer: ModuleType, monkeypatch, tmp_path) -> None:
    def _boom(*_args, **_kwargs):
        raise subprocess.TimeoutExpired(cmd="probe", timeout=1)

    monkeypatch.setattr(installer.subprocess, "run", _boom)

    assert installer._probe_plot_backend(tmp_path / "python") is None


# --- Installer._ensure_plot_backend() ---

def test_ensure_plot_backend_no_venv_python_is_noop(installer: ModuleType, monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(installer, "_find_idf_python", lambda idf_path: None)
    probe_calls = []
    monkeypatch.setattr(installer, "_probe_plot_backend", lambda venv_python: probe_calls.append(venv_python))

    installer.Installer()._ensure_plot_backend(tmp_path)

    assert probe_calls == []


def test_ensure_plot_backend_already_interactive_skips_install(
    installer: ModuleType, monkeypatch, tmp_path
) -> None:
    monkeypatch.setattr(installer, "_find_idf_python", lambda idf_path: tmp_path / "python")
    monkeypatch.setattr(installer, "_probe_plot_backend", _fake_probe({
        "backend": "macosx", "interactive": True, "reason": "",
        "has_display": True, "fallback_requirement": FAKE_REQUIREMENT,
    }))
    pip_calls = []
    monkeypatch.setattr(
        installer, "_run_in_idf_env",
        lambda idf_path, pip_args: pip_calls.append(pip_args) or 0,
    )

    installer.Installer()._ensure_plot_backend(tmp_path)

    assert pip_calls == []


def test_ensure_plot_backend_skips_without_display(installer: ModuleType, monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(installer, "_find_idf_python", lambda idf_path: tmp_path / "python")
    monkeypatch.setattr(installer, "_probe_plot_backend", _fake_probe({
        "backend": "agg", "interactive": False, "reason": "no DISPLAY/WAYLAND_DISPLAY (headless session)",
        "has_display": False, "fallback_requirement": FAKE_REQUIREMENT,
    }))
    pip_calls = []
    monkeypatch.setattr(
        installer, "_run_in_idf_env",
        lambda idf_path, pip_args: pip_calls.append(pip_args) or 0,
    )

    installer.Installer()._ensure_plot_backend(tmp_path)

    assert pip_calls == []


def test_ensure_plot_backend_installs_qt_when_headless(installer: ModuleType, monkeypatch, tmp_path) -> None:
    probes = [
        {"backend": "agg", "interactive": False, "reason": "tkagg: TclError: cannot find init.tcl",
         "has_display": True, "fallback_requirement": FAKE_REQUIREMENT},
        {"backend": "qtagg", "interactive": True, "reason": "",
         "has_display": True, "fallback_requirement": FAKE_REQUIREMENT},
    ]
    probe_calls = []

    def _sequenced_probe(venv_python):
        probe_calls.append(venv_python)
        return probes[len(probe_calls) - 1]

    monkeypatch.setattr(installer, "_find_idf_python", lambda idf_path: tmp_path / "python")
    monkeypatch.setattr(installer, "_probe_plot_backend", _sequenced_probe)
    pip_calls = []
    monkeypatch.setattr(
        installer, "_run_in_idf_env",
        lambda idf_path, pip_args: pip_calls.append(pip_args) or 0,
    )

    installer.Installer()._ensure_plot_backend(tmp_path)

    assert pip_calls == [["install", FAKE_REQUIREMENT]]
    assert len(probe_calls) == 2


def test_ensure_plot_backend_pip_failure_stops_without_reprobe(
    installer: ModuleType, monkeypatch, tmp_path
) -> None:
    probe_calls = []
    monkeypatch.setattr(installer, "_find_idf_python", lambda idf_path: tmp_path / "python")
    monkeypatch.setattr(installer, "_probe_plot_backend", lambda venv_python: probe_calls.append(venv_python) or {
        "backend": "agg", "interactive": False, "reason": "no Qt binding",
        "has_display": True, "fallback_requirement": FAKE_REQUIREMENT,
    })
    pip_calls = []
    monkeypatch.setattr(
        installer, "_run_in_idf_env",
        lambda idf_path, pip_args: pip_calls.append(pip_args) or 1,  # pip install fails
    )

    installer.Installer()._ensure_plot_backend(tmp_path)

    assert pip_calls == [["install", FAKE_REQUIREMENT]]
    assert len(probe_calls) == 1  # no re-probe after a failed install


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
