#!/usr/bin/env python3
"""
test_viz_backend_fallback.py - Tests for the matplotlib backend fallback
matplotlib バックエンドフォールバックのテスト

At a 2026-09-10 tutorial, `sf log viz flight.csv` failed on Windows PCs
with a matplotlib "drawing error" (reported as a backend problem; the
exact message was not recorded). Two failure modes of a Python without a
usable matplotlib GUI backend were then reproduced on macOS by faking the
Windows conditions: a "FigureCanvasAgg is non-interactive" warning with no
window, and a TclError "Can't find a usable init.tcl". This tests
lib/sfcli/utils/plotting.py's backend probing and lib/sfcli/commands/
log.py's run_viz() PNG-fallback path that handles both -- see
docs/guides/troubleshooting.md, section 6, for the user-facing writeup.

2026-09-10のチュートリアル講習で、Windows PCで `sf log viz flight.csv` が
matplotlibの「描画エラー」で失敗した（バックエンドの問題と報告されたが、
正確なメッセージは記録されていない）。その後、matplotlib の GUI
バックエンドが使えない Python の2つの故障モードを、Windows の条件を
模擬して macOS 上で再現した: "FigureCanvasAgg is non-interactive" の警告だけで
ウィンドウが出ない場合と、TclError "Can't find a usable init.tcl" の場合。
本テストは lib/sfcli/utils/plotting.py のバックエンド探索と、両方を扱う
lib/sfcli/commands/log.py の run_viz() のPNGフォールバック経路を検証する --
ユーザー向けの説明は docs/guides/troubleshooting.md 第6章を参照。

Usage:
    python3 test_viz_backend_fallback.py
    pytest test_viz_backend_fallback.py
"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行（テストプロセス自体は実
                        # ウィンドウを一切開かない）

import pytest

_TOOLS_LOG_ANALYZER_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _TOOLS_LOG_ANALYZER_DIR.parent.parent
sys.path.insert(0, str(_TOOLS_LOG_ANALYZER_DIR))  # for udp_capture
sys.path.insert(0, str(_REPO_ROOT / "lib"))  # for sfcli

import udp_capture  # noqa: E402
from sfcli.commands import log  # noqa: E402
from sfcli.utils import plotting  # noqa: E402
from sfcli.utils.plotting import BackendInfo  # noqa: E402

# Minimum PNG size used as a smoke check that matplotlib actually rendered
# panels (an empty/failed figure saves far smaller than this). Matches
# test_visualize_stream.py's own MIN_PNG_BYTES.
# matplotlib が実際にパネルを描画したことを確認する下限サイズ（空/失敗図は
# これより大幅に小さく保存される）。test_visualize_stream.py の
# MIN_PNG_BYTES と同じ値。
MIN_PNG_BYTES = 10_000

SAMPLE_RATE_HZ = 400
DURATION_S = 5
N_SAMPLES = SAMPLE_RATE_HZ * DURATION_S
DT_US = 1_000_000 // SAMPLE_RATE_HZ
CTRL_REF_STRIDE = 8  # 50Hz CtrlRef among 400Hz IMU+ESKF samples / 400Hz中50Hz

# Timeout for the real `python -m sfcli.utils.plotting --probe` subprocess
# spawned by test_probe_cli_prints_json_headless() -- generous enough for
# a cold matplotlib import (font cache build) on a slow CI runner.
# test_probe_cli_prints_json_headless() が起動する実際の
# `python -m sfcli.utils.plotting --probe` サブプロセスのタイムアウト。
# 低速なCIランナーでのmatplotlibの初回import（フォントキャッシュ構築）
# にも十分な余裕を持たせる。
PROBE_SUBPROCESS_TIMEOUT_S = 30


def _build_stream_csv(tmp_path) -> Path:
    """Same synthetic 5s/400Hz Data Stream CSV as
    test_visualize_stream.py's _build_stream_csv() -- see that file for
    why this exact shape is used (real save_stream_csv(), not a hand-rolled
    approximation).
    test_visualize_stream.py の _build_stream_csv() と同じ合成 5秒/400Hz
    Data Stream CSV -- なぜこの形にするかは同ファイル参照（本物の
    save_stream_csv() を使い、手組みの近似にしない）。"""
    cap = udp_capture.UDPTelemetryCapture()
    for i in range(N_SAMPLES):
        ts = 1_000_000 + i * DT_US
        cap.samples[udp_capture.PKT_IMU_ESKF].append({
            'timestamp_us': ts,
            'gyro_x': 0.1 * math.sin(i / 50), 'gyro_y': 0.0, 'gyro_z': 0.0,
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': -9.81,
            'gyro_raw_x': 0.0, 'gyro_raw_y': 0.0, 'gyro_raw_z': 0.0,
            'accel_raw_x': 0.0, 'accel_raw_y': 0.0, 'accel_raw_z': -9.81,
            'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0,
            'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
            'accel_bias_x': 0.0, 'accel_bias_y': 0.0, 'accel_bias_z': 0.0,
        })
        rate_ref_roll = 100 if i >= N_SAMPLES // 2 else 0
        cap.samples[udp_capture.PKT_RATE_REF].append({
            'timestamp_us': ts, 'rate_ref_roll': rate_ref_roll,
            'rate_ref_pitch': 0, 'rate_ref_yaw': 0,
        })
        if i % CTRL_REF_STRIDE == 0:
            cap.samples[udp_capture.PKT_CTRL_REF].append({
                'timestamp_us': ts, 'flight_mode': 1, 'reserved': 0,
                'angle_ref_roll': 0, 'angle_ref_pitch': 0, 'total_thrust': 0.5,
                'motor_duty_FR': 0.5, 'motor_duty_RR': 0.5,
                'motor_duty_RL': 0.5, 'motor_duty_FL': 0.5,
            })

    csv_path = tmp_path / "stream.csv"
    cap.save_stream_csv(str(csv_path))
    return csv_path


def _viz_args(csv_path, save=None) -> argparse.Namespace:
    """Build the argparse.Namespace run_viz() reads -- every attribute the
    `sf log viz` parser registers in lib/sfcli/commands/log.py's register().
    run_viz() が読む argparse.Namespace を組み立てる -- lib/sfcli/commands/
    log.py の register() が登録する `sf log viz` の全属性。"""
    return argparse.Namespace(
        file=str(csv_path),
        mode="all",
        save=save,
        time_range=None,
        no_eskf=False,
        no_sensors=False,
        show_invalid=False,
        interactive=False,
        layout=None,
        groups=None,
    )


# --- plotting.select_backend() ---

def test_select_backend_honors_mplbackend(monkeypatch):
    monkeypatch.setenv("MPLBACKEND", "agg")
    info = plotting.select_backend(want_window=True)

    assert info.name == "agg"
    assert info.interactive is False


def test_select_backend_falls_back_when_all_probes_fail(monkeypatch):
    monkeypatch.delenv("MPLBACKEND", raising=False)
    monkeypatch.setattr(plotting, "gui_backend_candidates", lambda: ["tkagg", "qtagg"])

    def _boom_tkagg():
        raise RuntimeError("Can't find a usable init.tcl")

    def _boom_qtagg():
        raise ImportError("no Qt bindings installed")

    monkeypatch.setattr(plotting, "PROBES", {"tkagg": _boom_tkagg, "qtagg": _boom_qtagg})

    info = plotting.select_backend(want_window=True)

    assert info.name == "agg"
    assert info.interactive is False
    assert "tkagg" in info.reason
    assert "qtagg" in info.reason


def test_select_backend_picks_first_usable(monkeypatch):
    monkeypatch.delenv("MPLBACKEND", raising=False)
    # "agg" stands in for a GUI backend here -- only used as a candidate
    # name so this test never activates a real window backend.
    # ここでは "agg" を GUI バックエンドの代役として使う -- 候補名として
    # 使うだけで、このテストが実ウィンドウのバックエンドを有効化することは
    # ない。
    monkeypatch.setattr(plotting, "gui_backend_candidates", lambda: ["tkagg", "agg"])

    def _boom_tkagg():
        raise RuntimeError("Can't find a usable init.tcl")

    def _ok():
        return None

    monkeypatch.setattr(plotting, "PROBES", {"tkagg": _boom_tkagg, "agg": _ok})

    used = []
    monkeypatch.setattr(matplotlib, "use", lambda name: used.append(name))

    info = plotting.select_backend(want_window=True)

    assert info.name == "agg"
    assert info.interactive is True
    assert used == ["agg"]


# --- plotting.ensure_gui_backend() / _probe_qtagg() / --probe CLI ---

class _FakeLog:
    """Minimal stand-in for the sf console's info/success/warning/print
    methods, recording every call so a test can assert on it without any
    real (colored) console output.
    sfコンソールのinfo/success/warning/printメソッドの最小限の代役。
    実際の（色付き）コンソール出力を伴わず検証できるよう、全呼び出しを
    記録する。"""

    def __init__(self):
        self.lines = []

    def info(self, message: str) -> None:
        self.lines.append(("info", message))

    def success(self, message: str) -> None:
        self.lines.append(("success", message))

    def warning(self, message: str) -> None:
        self.lines.append(("warning", message))

    def print(self, message: str = "") -> None:
        self.lines.append(("print", message))


def test_probe_cli_prints_json_headless(monkeypatch):
    monkeypatch.setenv("MPLBACKEND", "agg")

    result = subprocess.run(
        [sys.executable, "-m", "sfcli.utils.plotting", "--probe"],
        capture_output=True, text=True, timeout=PROBE_SUBPROCESS_TIMEOUT_S,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["backend"] == "agg"
    assert payload["interactive"] is False
    assert payload["fallback_requirement"] == plotting.QT_FALLBACK_REQUIREMENT


def test_ensure_gui_backend_installs_qt_when_headless(monkeypatch):
    monkeypatch.delenv("MPLBACKEND", raising=False)
    monkeypatch.setattr(plotting, "has_display", lambda: True)

    responses = [
        BackendInfo("agg", False, "tkagg: TclError: cannot find init.tcl"),
        BackendInfo("qtagg", True, ""),
    ]
    calls = {"select_backend": 0}

    def _fake_select_backend(want_window=True):
        info = responses[calls["select_backend"]]
        calls["select_backend"] += 1
        return info

    monkeypatch.setattr(plotting, "select_backend", _fake_select_backend)

    pip_calls = []

    def _fake_pip_install(requirements):
        pip_calls.append(requirements)
        return True

    log = _FakeLog()
    result = plotting.ensure_gui_backend(_fake_pip_install, log)

    assert result.interactive is True
    assert result.name == "qtagg"
    assert pip_calls == [[plotting.QT_FALLBACK_REQUIREMENT]]
    assert calls["select_backend"] == 2


def test_ensure_gui_backend_skips_without_display(monkeypatch):
    monkeypatch.delenv("MPLBACKEND", raising=False)
    monkeypatch.setattr(plotting, "has_display", lambda: False)

    pip_calls = []
    log = _FakeLog()

    result = plotting.ensure_gui_backend(lambda reqs: pip_calls.append(reqs) or True, log)

    assert pip_calls == []
    assert result.interactive is False


def test_ensure_gui_backend_reports_pip_failure(monkeypatch):
    monkeypatch.delenv("MPLBACKEND", raising=False)
    monkeypatch.setattr(plotting, "has_display", lambda: True)

    calls = {"select_backend": 0}

    def _fake_select_backend(want_window=True):
        calls["select_backend"] += 1
        return BackendInfo("agg", False, "no Qt binding installed")

    monkeypatch.setattr(plotting, "select_backend", _fake_select_backend)

    log = _FakeLog()
    result = plotting.ensure_gui_backend(lambda reqs: False, log)

    assert result.interactive is False
    assert calls["select_backend"] == 1
    assert any(level == "warning" for level, _ in log.lines)


def test_ensure_gui_backend_respects_mplbackend(monkeypatch):
    monkeypatch.setenv("MPLBACKEND", "agg")

    pip_calls = []
    log = _FakeLog()

    plotting.ensure_gui_backend(lambda reqs: pip_calls.append(reqs) or True, log)

    assert pip_calls == []


def test_probe_qtagg_reports_subprocess_failure(monkeypatch):
    monkeypatch.setattr(
        plotting.subprocess, "run",
        lambda *a, **k: subprocess.CompletedProcess(
            args=a, returncode=134, stdout="",
            stderr="qt.qpa.plugin: Could not load the Qt platform plugin",
        ),
    )

    with pytest.raises(RuntimeError) as exc_info:
        plotting._probe_qtagg()

    assert "qt.qpa.plugin" in str(exc_info.value)


# --- log.run_viz() PNG fallback ---

def test_run_viz_saves_png_and_opens_viewer_when_headless(tmp_path, monkeypatch):
    csv_path = _build_stream_csv(tmp_path)
    args = _viz_args(csv_path)

    monkeypatch.setattr(
        plotting, "select_backend",
        lambda want_window=True: BackendInfo("agg", False, "simulated"),
    )
    opened = []
    monkeypatch.setattr(
        plotting, "open_with_default_viewer",
        lambda path: opened.append(path) or True,
    )

    assert log.run_viz(args) == 0

    out_png = tmp_path / "stream.png"
    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES
    assert opened == [out_png]


def test_run_viz_retries_headless_when_window_backend_fails(tmp_path, monkeypatch):
    csv_path = _build_stream_csv(tmp_path)
    args = _viz_args(csv_path)

    monkeypatch.setattr(
        plotting, "select_backend",
        lambda want_window=True: BackendInfo("tkagg", True, ""),
    )
    opened = []
    monkeypatch.setattr(
        plotting, "open_with_default_viewer",
        lambda path: opened.append(path) or True,
    )

    import matplotlib.pyplot as plt

    def _boom_show(*_args, **_kwargs):
        raise RuntimeError("simulated Tk failure")

    monkeypatch.setattr(plt, "show", _boom_show)

    assert log.run_viz(args) == 0

    out_png = tmp_path / "stream.png"
    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES
    assert opened == [out_png]


def test_run_viz_explicit_save_unaffected(tmp_path, monkeypatch):
    csv_path = _build_stream_csv(tmp_path)
    explicit_path = tmp_path / "out.png"
    args = _viz_args(csv_path, save=str(explicit_path))

    # Even a headless backend must not trigger the fallback when the user
    # already gave an explicit --save path -- there is no window to fall
    # back from.
    # ユーザーが既に --save で保存先を明示している場合、ヘッドレスな
    # バックエンドであってもフォールバックを起動してはならない --
    # そもそも代替すべきウィンドウが無い。
    monkeypatch.setattr(
        plotting, "select_backend",
        lambda want_window=True: BackendInfo("agg", False, "simulated"),
    )
    opened = []
    monkeypatch.setattr(
        plotting, "open_with_default_viewer",
        lambda path: opened.append(path) or True,
    )

    assert log.run_viz(args) == 0

    assert explicit_path.exists()
    assert explicit_path.stat().st_size > MIN_PNG_BYTES
    assert opened == []


def _run_all():
    """Standalone runner using a throwaway tmp dir and pytest's
    MonkeyPatch outside of a pytest session (mirrors
    test_visualize_stream.py's own _run_all()).
    使い捨ての一時ディレクトリと、pytestセッション外で使う pytest の
    MonkeyPatch を使ったスタンドアロンランナー
    （test_visualize_stream.py 自身の _run_all() を模す）。"""
    import shutil
    import tempfile

    tmp_dir = Path(tempfile.mkdtemp(prefix="test_viz_backend_fallback_"))
    tests = [
        (test_select_backend_honors_mplbackend, ()),
        (test_select_backend_falls_back_when_all_probes_fail, ()),
        (test_select_backend_picks_first_usable, ()),
        (test_probe_cli_prints_json_headless, ()),
        (test_ensure_gui_backend_installs_qt_when_headless, ()),
        (test_ensure_gui_backend_skips_without_display, ()),
        (test_ensure_gui_backend_reports_pip_failure, ()),
        (test_ensure_gui_backend_respects_mplbackend, ()),
        (test_probe_qtagg_reports_subprocess_failure, ()),
        (test_run_viz_saves_png_and_opens_viewer_when_headless, (tmp_dir,)),
        (test_run_viz_retries_headless_when_window_backend_fails, (tmp_dir,)),
        (test_run_viz_explicit_save_unaffected, (tmp_dir,)),
    ]
    failures = 0
    try:
        for fn, fn_args in tests:
            with pytest.MonkeyPatch.context() as mp:
                try:
                    fn(*fn_args, mp)
                    print(f"  [TEST] {fn.__name__:<55} PASS")
                except AssertionError as e:
                    failures += 1
                    print(f"  [TEST] {fn.__name__:<55} FAIL: {e}")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
