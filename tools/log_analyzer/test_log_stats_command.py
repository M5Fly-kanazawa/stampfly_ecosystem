"""
test_log_stats_command.py - `sf log stats` end to end: argument parsing, every mode on a bundle on disk, the
messages and exit codes for a missing stream
`sf log stats` の通し試験: 引数の解釈、ディスク上の一式への各モード、ストリームが無いときのメッセージと終了コード

The arithmetic of each mode is tested against hand-computed cases in lib/sflog/tests/test_stats.py; here the
command around it is tested: a bundle with the status-v4 streams (all modes that its streams allow succeed), a
bundle without them and the real-vehicle reference bundle (older firmware: the command must say which stream is
missing and exit 1, never traceback).
各モードの算術は lib/sflog/tests/test_stats.py の手計算の事例で試験済み。ここではその周りのコマンドを試験する:
ステータス v4 のストリームを持つ一式（そのストリームで答えられるモードは全て成功する）、持たない一式、実機の基準一式
（旧ファーム: どのストリームが無いかを述べて終了コード 1 になり、トレースバックを出さない）。

Usage:
    pytest test_log_stats_command.py
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import pytest

_TOOLS_LOG_ANALYZER_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _TOOLS_LOG_ANALYZER_DIR.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "lib"))
sys.path.insert(0, str(_TOOLS_LOG_ANALYZER_DIR))

import conftest  # noqa: E402
import test_log_new_streams as new_streams  # noqa: E402
from sfcli.commands import log  # noqa: E402


def _args(mode, path, **kw):
    values = dict(mode=mode, bundle=str(path), since=None, until=None, target=0.5, band=0.07)
    values.update(kw)
    return argparse.Namespace(**values)


def _run(capsys, mode, path, **kw):
    code = log.run_stats(_args(mode, path, **kw))
    captured = capsys.readouterr()
    return code, captured.out + captured.err


@pytest.fixture
def with_v4(tmp_path):
    return new_streams._write(tmp_path / "with_v4.sflog.zip", True)


@pytest.fixture
def without_v4(tmp_path):
    return new_streams._write(tmp_path / "without_v4.sflog.zip", False)


def test_the_subcommand_is_registered_with_its_modes():
    top = argparse.ArgumentParser()
    log.register(top.add_subparsers(dest="command"))
    args = top.parse_args(["log", "stats", "spread", "x.sflog.zip", "--since", "1", "--until", "2.5"])
    assert (args.mode, args.bundle, args.since, args.until) == ("spread", "x.sflog.zip", 1.0, 2.5)
    assert args.target == 0.5 and args.band == 0.07 and args.func is log.run_stats
    with pytest.raises(SystemExit):
        top.parse_args(["log", "stats", "nonsense"])


def test_modes_that_the_streams_allow_succeed_and_print_a_titled_table(with_v4, capsys):
    first = conftest.FIRST_TIMESTAMP_US / 1e6
    expected = {
        "boot": ("Boot and battery", {}),
        "loop": ("Armed 400 Hz loop", {}),
        "events": ("Flight events", {}),
        "spread": ("ToF scatter in 1 s windows", dict(since=first + 1, until=first + 3)),
        "turn": ("Turn in the window", dict(since=first + 1, until=first + 3)),
    }
    for mode, (title, window) in expected.items():
        code, out = _run(capsys, mode, with_v4, **window)
        assert code == 0, (mode, out)
        assert title in out and f"mode: {mode}" in out and "Traceback" not in out


def test_spread_of_the_constant_synthetic_tof_is_zero(with_v4, capsys):
    first = conftest.FIRST_TIMESTAMP_US / 1e6
    code, out = _run(capsys, "spread", with_v4, since=first + 1, until=first + 3)
    assert code == 0 and "largest 1 s spread      : 0.0 cm" in out


@pytest.mark.parametrize("mode,stream,window", [
    ("tof", "flight_phase", {}),                       # the synthetic flight is never ARMED_GROUND / 合成飛行は ARMED_GROUND にならない
    ("baro", "baro_height", dict(since=1.0, until=3.0)),
])
def test_missing_stream_or_state_exits_1_and_names_it(with_v4, capsys, mode, stream, window):
    code, out = _run(capsys, mode, with_v4, **window)
    assert code == 1 and stream in out and "Traceback" not in out


def test_loop_on_a_bundle_without_the_v4_streams_exits_1_naming_the_stream(without_v4, capsys):
    code, out = _run(capsys, "loop", without_v4)
    assert code == 1 and "stream 'loop_timing'" in out


def test_windowed_mode_without_a_window_exits_1(with_v4, capsys):
    code, out = _run(capsys, "turn", with_v4)
    assert code == 1 and "needs both --since and --until" in out


def test_real_vehicle_reference_bundle_old_firmware(capsys):
    code, out = _run(capsys, "boot", conftest.REFERENCE_BUNDLE)
    assert code == 0 and "reset reasons" in out
    for mode in ("tof", "loop", "events"):
        code, out = _run(capsys, mode, conftest.REFERENCE_BUNDLE)
        assert code == 1 and "stream '" in out and "is missing or empty" in out and "Traceback" not in out
