"""
test_log_new_streams.py - the `sf log` readers on bundles with and without the status-v4 / text-log streams
`sf log` の読み込み側を、ステータス v4・テキストログのストリームがある一式とない一式で確かめる試験

The five new optional streams (tof_diagnostics, loop_timing, nonfinite, link_health, text_log) must neither
break the readers when present nor be required when absent (a bundle from older firmware). Every reader that
a flight analysis uses is run on BOTH bundles: info (summary and --text with its filters), check, convert
--aligned (the text log stays out of the table) and --jsonl, analyze and viz (a PNG is written).
5 つの新しい任意のストリームは、あっても読み込み側を壊さず、なくても必須にならない（旧ファームの一式）。飛行解析が使う
読み込み側のすべてを、両方の一式で動かす: info（要約と、絞り込みつきの --text）、check、convert --aligned（テキストログは
表に入らない）と --jsonl、analyze、viz（PNG を書く）。

Usage:
    pytest test_log_new_streams.py
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import pandas as pd
import pytest

_TOOLS_LOG_ANALYZER_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _TOOLS_LOG_ANALYZER_DIR.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "lib"))
sys.path.insert(0, str(_TOOLS_LOG_ANALYZER_DIR))

import conftest  # noqa: E402
import sflog  # noqa: E402
from sfcli.commands import log  # noqa: E402

MIN_PNG_BYTES = 10_000


def _loop_timing_columns(task):
    names = ('cycles', 'period_max_us', 'late_count', 'busy_max_us', 'over_budget_count', 'last_late_us',
             'armed_cycles', 'armed_period_max_us', 'armed_late_count', 'armed_busy_max_us',
             'armed_over_budget_count')
    return {f'{task}_{n}': 0 for n in names}


def add_v4_streams(flight_log):
    """Add the five new streams (1 Hz diagnostics for the 5 s of the synthetic flight, a few text lines)."""
    seconds = [1, 2, 3, 4, 5]
    timestamps = [conftest.FIRST_TIMESTAMP_US + s * 1_000_000 for s in seconds]
    def frame(columns):
        return pd.DataFrame({'timestamp_us': timestamps, **columns})
    flight_log.streams['loop_timing'] = frame({**_loop_timing_columns('imu'), **_loop_timing_columns('control')})
    flight_log.streams['nonfinite'] = frame({
        'checked_cycles': 400, 'detected_cycles': 0, 'estimate_input': 0, 'setpoint_input': 0,
        'control_reference': 0, 'control_output': 0, 'mixer_output': 0, 'motor_command': 0,
        'first_us': 0, 'first_stage': 255})
    flight_log.streams['link_health'] = frame({
        'log_stream_overflow': 0, 'send_drops': 0, 'entry_drops': 0, 'text_dropped_lines': 3})
    flight_log.streams['tof_diagnostics'] = frame({
        'bottom_range_status': 0, 'bottom_signal_rate': 3.5, 'bottom_ambient_rate': 0.2, 'bottom_sigma_mm': 1.5,
        'front_range_status': 255, 'front_signal_rate': 0.0, 'front_ambient_rate': 0.0, 'front_sigma_mm': 0.0})
    base = conftest.FIRST_TIMESTAMP_US
    flight_log.streams['text_log'] = pd.DataFrame({
        'timestamp_us': [base + 100_000, base + 200_000, base + 200_000, base + 3_500_000],
        'truncated': [0, 0, 1, 0],
        'dropped_before': [0, 0, 0, 3],
        'text': ['I (100) Boot: started', 'W (200) Failsafe: low battery', 'I (200) Test: NA', 'I (3500) State: Landing detected (ToF)'],
    })
    return flight_log


def _write(path: Path, with_v4: bool) -> Path:
    flight_log = conftest.build_synthetic_log()
    if with_v4:
        add_v4_streams(flight_log)
    flight_log.meta = sflog.make_meta(source='vehicle', tool_name='test', tool_version='0', streams=flight_log.streams)
    flight_log.schema = sflog.schema.schema_for(flight_log.streams.keys())
    flight_log.save(path)
    return path


@pytest.fixture(params=['without_v4', 'with_v4'])
def bundle(request, tmp_path):
    return _write(tmp_path / f"{request.param}.sflog.zip", request.param == 'with_v4'), request.param == 'with_v4'


def _info_args(path, **kw):
    values = dict(bundle=str(path), text=False, grep=None, ignore_case=False, since=None, until=None)
    values.update(kw)
    return argparse.Namespace(**values)


def test_info_summary_works_and_shows_diagnostics_only_when_present(bundle, capsys):
    path, with_v4 = bundle
    assert log.run_info(_info_args(path)) == 0
    out = capsys.readouterr().out
    assert 'imu' in out
    assert ('Diagnostics (status v4 and text log):' in out) == with_v4
    if with_v4:
        assert 'imu      loop: cycles=0' in out and 'nonfinite: detected 0 of 400 armed cycles' in out
        assert 'signal=3.50Mcps' in out and 'text_dropped_lines=3' in out
        assert 'text log : 4 lines' in out


def test_info_text_prints_the_lines_and_the_loss_marker(bundle, capsys):
    path, with_v4 = bundle
    assert log.run_info(_info_args(path, text=True)) == 0
    captured = capsys.readouterr()
    out = captured.out
    if not with_v4:
        assert 'no text log' in captured.out + captured.err   # a warning, not a failure / 警告であり失敗ではない
        return
    lines = out.splitlines()
    first = conftest.FIRST_TIMESTAMP_US / 1e6
    assert lines[0].endswith('I (100) Boot: started') and lines[0].startswith('[')
    assert any(l.endswith('I (200) Test: NA [truncated]') for l in lines)       # truncation visible, "NA" stays text / 切り詰めが見え、「NA」は文字列のまま
    marker = [i for i, l in enumerate(lines) if '*** 3 lines dropped here ***' in l]
    assert marker and lines[marker[0] + 1].endswith('Landing detected (ToF)')    # the loss sits before the line after it / 欠落はその後の行の前にある


def test_info_text_filters_and_a_loss_is_never_hidden(bundle, capsys):
    path, with_v4 = bundle
    if not with_v4:
        return
    assert log.run_info(_info_args(path, text=True, grep='landing', ignore_case=True)) == 0
    out = capsys.readouterr().out
    assert 'Landing detected' in out and 'Boot' not in out
    assert '*** 3 lines dropped here ***' in out                      # the marker survives --grep / 印は --grep でも残る
    window_from = conftest.FIRST_TIMESTAMP_US / 1e6 + 0.15
    window_to = conftest.FIRST_TIMESTAMP_US / 1e6 + 0.25
    assert log.run_info(_info_args(path, text=True, since=window_from, until=window_to)) == 0
    out = capsys.readouterr().out
    assert 'Failsafe: low battery' in out and 'Boot' not in out and 'Landing' not in out


def test_check_passes_on_both(bundle):
    path, _ = bundle
    findings = sflog.check_bundle(path)
    assert [f for f in findings if f.level == 'error'] == [], findings
    assert not any(f.stream == 'text_log' and 'repeated' in f.message for f in findings)   # equal times are normal for text lines / 行の時刻の同値は正常


def test_convert_aligned_leaves_the_text_log_out(bundle, tmp_path):
    path, with_v4 = bundle
    out = tmp_path / "aligned.csv"
    args = argparse.Namespace(input=str(path), output=str(out), aligned=True, base='imu', method='hold', jsonl=False)
    assert log.run_convert(args) == 0
    columns = set(pd.read_csv(out).columns)
    assert 'text' not in columns
    assert ('bottom_signal_rate' in columns) == with_v4


def test_convert_jsonl_ignores_the_new_streams(tmp_path):
    """The legacy JSONL conversion (a real vehicle bundle) writes the same lines with or without the new streams."""
    if not conftest.REFERENCE_BUNDLE.exists():
        pytest.skip("reference bundle missing")
    plain = sflog.load(conftest.REFERENCE_BUNDLE)
    extended = add_v4_streams(sflog.load(conftest.REFERENCE_BUNDLE))
    assert sflog.bundle_to_jsonl(plain, tmp_path / "plain.jsonl") == sflog.bundle_to_jsonl(extended, tmp_path / "extended.jsonl")
    assert (tmp_path / "plain.jsonl").read_text() == (tmp_path / "extended.jsonl").read_text()


def test_analyze_and_viz_work_on_both(bundle, tmp_path):
    path, _ = bundle
    png = tmp_path / "analysis.png"
    args = argparse.Namespace(bundle=str(path), save=str(png), health=False, batch=False, json=False)
    assert log.run_analyze(args) == 0
    assert png.stat().st_size > MIN_PNG_BYTES
    viz_png = tmp_path / "viz.png"
    viz_args = argparse.Namespace(bundle=str(path), mode='all', cols=3, save=str(viz_png), time_range=None,
                                  interactive=False)
    assert log.run_viz(viz_args) == 0
    assert viz_png.stat().st_size > MIN_PNG_BYTES


def test_directory_bundle_with_text_log_loads(tmp_path):
    """A bundle directory (the form SILS leaves before zipping) reads the text log as written."""
    directory = tmp_path / "bundle_dir"
    flight_log = add_v4_streams(conftest.build_synthetic_log())
    flight_log.meta = sflog.make_meta(source='vehicle', tool_name='test', tool_version='0', streams=flight_log.streams)
    flight_log.schema = sflog.schema.schema_for(flight_log.streams.keys())
    flight_log.save(directory)
    loaded = sflog.load(directory)
    assert list(loaded.streams['text_log']['text'])[2] == 'I (200) Test: NA'
