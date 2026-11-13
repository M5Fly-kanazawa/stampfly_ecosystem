"""
test_textlog.py - sflog.format_text_log() / diagnostics_summary() (the reader behind `sf log info --text`)
sflog.format_text_log() / diagnostics_summary() の試験（`sf log info --text` の読み込み側）
"""

import pandas as pd

import sflog
from sflog.bundle import FlightLog


def _log(rows):
    df = pd.DataFrame(rows, columns=["timestamp_us", "truncated", "dropped_before", "text"])
    return FlightLog(streams={"text_log": df})


ROWS = [
    (1_000_000, 0, 0, "I (1000) A: first"),
    (2_500_000, 1, 0, "W (2500) B: cut"),
    (2_500_000, 0, 4, "E (2500) A: after a loss"),
    (9_000_000, 0, 0, "I (9000) C: last"),
]


def test_all_lines_in_order_with_vehicle_time_truncation_and_loss():
    lines = sflog.format_text_log(_log(ROWS))
    assert lines == [
        "[  1.000000 s] I (1000) A: first",
        "[  2.500000 s] W (2500) B: cut [truncated]",
        "            *** 4 lines dropped here ***",
        "[  2.500000 s] E (2500) A: after a loss",
        "[  9.000000 s] I (9000) C: last",
    ]


def test_grep_is_a_plain_substring_and_never_hides_a_loss():
    lines = sflog.format_text_log(_log(ROWS), grep="A:")
    assert [l for l in lines if "dropped" not in l] == [
        "[  1.000000 s] I (1000) A: first", "[  2.500000 s] E (2500) A: after a loss"]
    assert any("4 lines dropped" in l for l in lines)
    assert [l for l in sflog.format_text_log(_log(ROWS), grep="a:") if "dropped" not in l] == []              # case matters / 大文字小文字を区別する
    assert len([l for l in sflog.format_text_log(_log(ROWS), grep="a:", ignore_case=True) if "dropped" not in l]) == 2
    assert [l for l in sflog.format_text_log(_log(ROWS), grep=".*") if "dropped" not in l] == []   # not a pattern / パターンではない


def test_time_window_is_inclusive():
    lines = sflog.format_text_log(_log(ROWS), since_s=2.5, until_s=2.5)
    assert [l for l in lines if "dropped" not in l] == [
        "[  2.500000 s] W (2500) B: cut [truncated]", "[  2.500000 s] E (2500) A: after a loss"]


def test_missing_stream_gives_nothing_and_diagnostics_are_optional():
    empty = FlightLog(streams={})
    assert sflog.format_text_log(empty) == []
    assert not sflog.has_text_log(empty)
    assert sflog.diagnostics_summary(empty) == []


def test_empty_text_cell_is_a_blank_line():
    lines = sflog.format_text_log(_log([(5, 0, 0, None)]))
    assert lines == ["[  0.000005 s] "]
