#!/usr/bin/env python3
"""
test_sils_nonfinite.py - tests for the non-finite number check of `sf sils scenario`
(lib/sfcli/commands/sils.py: `_nonfinite_check()` and its use in `_eval_expect()`).
The emulator prints "[emu] nonfinite: detected N of M armed cycles, first boundary: <name>";
a run passes only if N is 0, unless the .expect has a `nonfinite_cycles <op> <count>` line.
`sf sils scenario` の有限でない数の検査（sils.py の `_nonfinite_check()` と `_eval_expect()` での
使い方）の試験。エミュレータは "[emu] nonfinite: detected N of M armed cycles, first boundary: <名前>"
を出し、.expect に `nonfinite_cycles <op> <回数>` の行がなければ N が 0 のときだけ合格する。

Usage:
    python -m pytest lib/sfcli/commands/test_sils_nonfinite.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sfcli.commands import sils  # noqa: E402

CLEAN = "[emu] nonfinite: detected 0 of 5200 armed cycles, first boundary: none\n"
HIT = "[emu] nonfinite: detected 7 of 5200 armed cycles, first boundary: estimate input\n"


def _evaluate(tmp_path: Path, expect_text: str, stdout: str):
    expect = tmp_path / "case.expect"
    expect.write_text(expect_text, encoding="utf-8")
    checks, verdict, _ = sils._eval_expect(expect, stdout, "", 0)
    return checks, verdict


def test_line_absent_gives_no_check():
    assert sils._nonfinite_check("[emu] final state=1\n", None) is None


def test_default_requires_zero_and_passes_on_a_clean_run():
    check = sils._nonfinite_check(CLEAN, None)
    assert check["pass"] and check["name"] == "nonfinite_cycles == 0"


def test_default_fails_on_a_detection_and_names_the_first_boundary():
    check = sils._nonfinite_check(HIT, None)
    assert not check["pass"]
    assert "first boundary: estimate input" in check["detail"]
    assert "detected 7 of 5200" in check["detail"]


def test_every_operator_of_the_limit_is_honoured():
    assert sils._nonfinite_check(HIT, ("==", 7))["pass"]
    assert not sils._nonfinite_check(HIT, ("==", 6))["pass"]
    assert sils._nonfinite_check(HIT, (">", 0))["pass"]
    assert sils._nonfinite_check(HIT, (">=", 7))["pass"]
    assert sils._nonfinite_check(HIT, ("<", 8))["pass"]
    assert sils._nonfinite_check(HIT, ("<=", 7))["pass"]
    assert not sils._nonfinite_check(CLEAN, (">", 0))["pass"]


def test_expect_without_the_line_gates_on_zero(tmp_path):
    _, verdict = _evaluate(tmp_path, 'log_contains any "ok"\n', "ok\n" + CLEAN)
    assert verdict
    checks, verdict = _evaluate(tmp_path, 'log_contains any "ok"\n', "ok\n" + HIT)
    assert not verdict
    assert any(c["name"] == "nonfinite_cycles == 0" and not c["pass"] for c in checks)


def test_expect_line_overrides_the_default(tmp_path):
    checks, verdict = _evaluate(tmp_path, 'log_contains any "ok"\nnonfinite_cycles > 0\n', "ok\n" + HIT)
    assert verdict
    assert any(c["name"] == "nonfinite_cycles > 0" and c["pass"] for c in checks)


def test_the_check_alone_does_not_make_an_empty_expect_pass(tmp_path):
    _, verdict = _evaluate(tmp_path, "# nothing\n", CLEAN)
    assert not verdict


def test_output_without_the_line_is_unchanged(tmp_path):
    checks, verdict = _evaluate(tmp_path, 'log_contains any "ok"\n', "ok\n")
    assert verdict and len(checks) == 1
