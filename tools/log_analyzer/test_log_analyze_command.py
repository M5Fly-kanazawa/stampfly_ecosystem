"""
test_log_analyze_command.py - `sf log analyze` on flight-log bundles
`sf log analyze` のフライトログ一式対応のテスト

Exercises lib/sfcli/commands/log.py's run_analyze() end to end on the
real-vehicle reference bundle and on synthetic bundles: the flight
analysis (PNG + report), `--health` (motor-health verdict as JSON) and
`--health --batch` over a glob of bundles.
lib/sfcli/commands/log.py の run_analyze() を、実機由来の基準一式と合成
一式で端から端まで通す: 飛行解析（PNG + レポート）、`--health`
（モータ健全性判定の JSON）、`--health --batch`（一式のグロブ）。

Usage:
    pytest test_log_analyze_command.py
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import pytest

_TOOLS_LOG_ANALYZER_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _TOOLS_LOG_ANALYZER_DIR.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "lib"))  # for sfcli
sys.path.insert(0, str(_TOOLS_LOG_ANALYZER_DIR))  # for conftest

import conftest  # noqa: E402
import sflog  # noqa: E402
from sfcli.commands import log  # noqa: E402

# Same smoke threshold as the viz tests: a rendered analysis figure is far
# larger than an empty one.
# viz テストと同じ下限: 描画済みの解析図は空図よりずっと大きい。
MIN_PNG_BYTES = 10_000

# Duty asymmetry that gives the synthetic hover a clean window (sum > 2.0)
# and a clear CW-weak yaw trim: FR/RL (CCW) at 0.55, RR/FL (CW) at 0.65.
# 合成ホバーに明確な区間（合計 > 2.0）と CW 群が弱いヨートリムを与える
# duty の非対称: FR/RL（CCW）= 0.55、RR/FL（CW）= 0.65。
DUTY_CCW = 0.55
DUTY_CW = 0.65

# The health report rejects hover windows shorter than motor_health's
# MIN_HOVER_S (10 s) after trimming spin-up/landing, so the batch bundles
# fly longer than conftest's default 5 s.
# 健全性レポートはスピンアップ/着陸を除いた区間が motor_health の
# MIN_HOVER_S（10 秒）未満だと不採用にするため、バッチ用の一式は conftest
# 既定の 5 秒より長く飛ばす。
HOVER_BUNDLE_DURATION_S = 15.0


def _analyze_args(bundle, save=None, health=False, batch=False, json_out=False):
    """Build the argparse.Namespace run_analyze() reads -- every attribute
    the `sf log analyze` parser registers in log.py's register().
    run_analyze() が読む argparse.Namespace を組み立てる -- log.py の
    register() が登録する `sf log analyze` の全属性。"""
    return argparse.Namespace(
        bundle=None if bundle is None else str(bundle),
        save=save, health=health, batch=batch, json=json_out,
    )


def _write_hover_bundle(path: Path) -> Path:
    """Synthetic bundle whose motor/ctrl_ref duties form a clean hover with
    a CW-weak yaw trim (see DUTY_CCW / DUTY_CW).
    motor/ctrl_ref の duty が明確なホバー区間と CW 群の弱いヨートリムを
    持つ合成一式（DUTY_CCW / DUTY_CW 参照）。"""
    flight_log = conftest.build_synthetic_log(duration_s=HOVER_BUNDLE_DURATION_S)
    for stream_name in ("motor", "ctrl_ref"):
        df = flight_log.streams[stream_name]
        df["duty_FR"] = DUTY_CCW
        df["duty_RL"] = DUTY_CCW
        df["duty_RR"] = DUTY_CW
        df["duty_FL"] = DUTY_CW
    flight_log.meta = sflog.make_meta(
        source="sim", tool_name="test_log_analyze_command", tool_version="0.0.0",
        streams=flight_log.streams,
    )
    flight_log.save(path)
    return path


def test_run_analyze_reference_bundle_writes_png(tmp_path, reference_bundle):
    png = tmp_path / "analysis.png"

    assert log.run_analyze(_analyze_args(reference_bundle, save=str(png))) == 0
    assert png.stat().st_size > MIN_PNG_BYTES


def test_run_analyze_default_png_next_to_bundle(tmp_path, synthetic_bundle):
    """Without --save the figure lands next to the bundle as
    <stem>_analysis.png (stem = name without .sflog.zip).
    --save 無しなら図は一式の隣に <stem>_analysis.png として置かれる
    （stem は .sflog.zip を外した名前）。"""
    assert log.run_analyze(_analyze_args(synthetic_bundle)) == 0
    assert (tmp_path / "flight_synthetic_analysis.png").exists()


def test_run_analyze_health_reference_bundle_json(capsys, reference_bundle):
    assert log.run_analyze(_analyze_args(reference_bundle, health=True, json_out=True)) == 0

    payload = json.loads(_last_json_block(capsys))
    assert payload["n_logs"] == 1
    assert "verdict" in payload


def _last_json_block(capsys) -> str:
    """The JSON object printed last on stdout (console.info lines precede
    it), found by scanning back to the first line that starts with '{'.
    stdout の最後に印字された JSON オブジェクト（前に console.info 行が
    ある）。'{' で始まる行まで遡って切り出す。"""
    lines = capsys.readouterr().out.splitlines()
    start = max(i for i, line in enumerate(lines) if line.startswith("{"))
    return "\n".join(lines[start:])


def test_run_analyze_health_batch_glob(tmp_path, monkeypatch, capsys):
    """`--health --batch` with a glob: both synthetic hover bundles are
    analyzed together and the JSON verdict reports n_logs == 2.
    グロブ付き `--health --batch`: 合成ホバー一式 2 本をまとめて解析し、
    JSON 判定は n_logs == 2 を報告する。"""
    logs_dir = tmp_path / "logs"
    logs_dir.mkdir()
    _write_hover_bundle(logs_dir / "flight_20260911T100000.sflog.zip")
    _write_hover_bundle(logs_dir / "flight_20260911T100500.sflog.zip")
    monkeypatch.setattr(log, "get_log_dir", lambda: logs_dir)

    args = _analyze_args("flight_20260911T*.sflog.zip", health=True, batch=True, json_out=True)
    assert log.run_analyze(args) == 0

    payload = json.loads(_last_json_block(capsys))
    assert payload["n_logs"] == 2
    assert payload["verdict"]["group"] is not None


def test_run_analyze_health_batch_default_newest(tmp_path, monkeypatch, capsys):
    """`--health --batch` without a glob takes the newest bundles in logs/.
    グロブ無しの `--health --batch` は logs/ 内の最新の一式を対象にする。"""
    logs_dir = tmp_path / "logs"
    logs_dir.mkdir()
    _write_hover_bundle(logs_dir / "flight_a.sflog.zip")
    monkeypatch.setattr(log, "get_log_dir", lambda: logs_dir)

    assert log.run_analyze(_analyze_args(None, health=True, batch=True, json_out=True)) == 0
    assert json.loads(_last_json_block(capsys))["n_logs"] == 1


def test_run_analyze_missing_bundle_fails(tmp_path):
    assert log.run_analyze(_analyze_args(tmp_path / "nope.sflog.zip")) == 1


def test_run_analyze_rejects_non_bundle(tmp_path):
    stray = tmp_path / "flight.csv"
    stray.write_text("timestamp_us,gyro_x\n0,0\n", encoding="utf-8")

    assert log.run_analyze(_analyze_args(stray)) == 1
