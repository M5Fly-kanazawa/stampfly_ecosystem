#!/usr/bin/env python3
"""
test_sils_core_list.py - consistency tests for the core scenario list
(simulator/sils/scenarios/CORE.txt) and the selection of `sf sils regression`.
最重要シナリオの一覧（CORE.txt）と `sf sils regression` の選択の整合の試験。

Usage:
    python -m pytest lib/sfcli/commands/test_sils_core_list.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sfcli.commands import sils  # noqa: E402

SCN_DIR = Path(__file__).resolve().parents[3] / "simulator" / "sils" / "scenarios"


def _raw_names() -> list:
    """Names in CORE.txt without validation. 検証なしで CORE.txt の名前を読む。"""
    names = []
    for raw in (SCN_DIR / sils.CORE_LIST_FILE).read_text(encoding="utf-8").splitlines():
        name = raw.split("#", 1)[0].strip()
        if name:
            names.append(name)
    return names


def test_every_listed_scenario_exists_with_expect():
    # _load_core_list raises on a missing .scn or .expect / 欠落があれば例外
    assert sils._load_core_list(SCN_DIR)


def test_no_duplicates():
    names = _raw_names()
    assert len(names) == len(set(names))


def test_size_is_30():
    assert len(_raw_names()) == sils.CORE_LIST_SIZE == 30


def test_no_known_fail_and_no_workshop_target():
    for name in _raw_names():
        expect = SCN_DIR / f"{name}.expect"
        assert sils._read_xfail(expect) is None, f"{name} carries an xfail marker"
        assert "workshop" not in sils._scenario_invocation(SCN_DIR / f"{name}.scn"), (
            f"{name} targets workshop")


def test_default_selection_is_the_core_list():
    chosen, label, available = sils._select_regression(SCN_DIR, False, None)
    assert label == "core"
    assert sorted(p.stem for p in chosen) == sorted(_raw_names())
    assert available >= len(chosen)


def test_all_and_match_selection():
    everything, label, available = sils._select_regression(SCN_DIR, True, None)
    assert label == "all" and len(everything) == available
    tof, _, _ = sils._select_regression(SCN_DIR, False, ["tof_"])
    assert tof and all(p.stem.startswith("tof_") for p in tof)
    assert {p.stem for p in tof} <= {p.stem for p in everything}


def _write_pair(directory: Path, name: str, with_expect: bool = True) -> None:
    (directory / f"{name}.scn").write_text("0 rc 2048 2048 2048 2048 0 100 50\n", encoding="utf-8")
    if with_expect:
        (directory / f"{name}.expect").write_text("exit 0\n", encoding="utf-8")


def test_missing_scenario_or_expect_or_duplicate_is_an_error(tmp_path):
    _write_pair(tmp_path, "a")
    _write_pair(tmp_path, "b", with_expect=False)
    (tmp_path / sils.CORE_LIST_FILE).write_text("a\nb\nghost\na  # again\n", encoding="utf-8")
    with pytest.raises(ValueError) as err:
        sils._load_core_list(tmp_path)
    text = str(err.value)
    assert "b: no b.expect" in text and "ghost: no ghost.scn" in text and "duplicate entry: a" in text
