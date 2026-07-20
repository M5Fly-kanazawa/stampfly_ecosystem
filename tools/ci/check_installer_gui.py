#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/ci/check_installer_gui.py

Dependency-free automated test for the "StampFly Setup" installer GUI's CI
contracts (docs/plans/gui-installer-plan.md). Unlike
tools/ci/check_flasher_install.py (which tests a Python module's pure
logic), this script mostly tests *artifacts* -- a subprocess run of
tools/installer_gui/stampfly_installer.py and the text of
.github/workflows/release.yml -- so it needs no network access and never
touches a real install location, but it is not a hermetic unit test either.
「StampFly Setup」インストーラGUIのCI契約(docs/plans/gui-installer-plan.md)を
検証する、依存ゼロの自動テスト。Pythonモジュールの純粋なロジックを検証する
tools/ci/check_flasher_install.py と異なり、本スクリプトは主に「成果物」
(tools/installer_gui/stampfly_installer.py のsubprocess実行結果と
.github/workflows/release.yml のテキスト)を検証する。ネットワークアクセス
は不要で実インストール先には一切触れないが、厳密な意味でのユニットテスト
ではない。

Covers / 対象:
    1. Running `tools/installer_gui/stampfly_installer.py --selftest` with
       the system Python and SF_SETUP_REPO pointed at this repository exits
       0 (docs/plans/gui-installer-plan.md §4 "selftest" contract).
       システムPythonで `tools/installer_gui/stampfly_installer.py
       --selftest` を、SF_SETUP_REPO をこのリポジトリに向けて実行し、
       終了コード0を確認する(docs/plans/gui-installer-plan.md §4の
       「selftest」契約)。
    2. Every top-level module that scripts/installer.py imports (via `ast`,
       covering imports nested inside functions too) also appears as an
       import somewhere in tools/installer_gui/stampfly_installer.py's own
       source. This guards the "stdlib同梱漏れ" contract in
       docs/plans/gui-installer-plan.md §5: scripts/installer.py is
       process-in-imported at runtime from a clone that does not exist at
       PyInstaller analysis time, so PyInstaller can only discover its
       stdlib dependencies if stampfly_installer.py's own source also
       spells out each one.
       scripts/installer.py がimportする全モジュール(トップレベルだけで
       なく、関数内のimportも `ast` で網羅)が、
       tools/installer_gui/stampfly_installer.py 自身のソースにも import
       として現れることを検証する。docs/plans/gui-installer-plan.md §5の
       「stdlib同梱漏れ」契約のガード: scripts/installer.py は実行時に
       (PyInstallerの解析時点では存在しない)クローン先からプロセス内
       importされるため、stampfly_installer.py自身のソースが各stdlib
       依存を明記して初めてPyInstallerがそれを検出できる。
    3. .github/workflows/release.yml contains the literal
       `StampFlySetup_<suffix>` asset-name strings (tag-less stable
       names) for all four
       platforms (windows-x64.exe / macos-arm64.zip / macos-x64.zip /
       linux-x64), so the release body actually documents every asset the
       installer_gui job publishes.
       .github/workflows/release.yml に、4プラットフォーム分の
       `StampFlySetup_<suffix>` アセット名文字列(タグ無し固定名)
       (windows-x64.exe / macos-arm64.zip / macos-x64.zip / linux-x64)が
       全て含まれることを検証する。installer_gui ジョブが公開する全アセットを
       リリース本文が実際に案内していることを保証する。

Usage / 使い方:
    python3 tools/ci/check_installer_gui.py

Exit code 0 = all checks passed, 1 = at least one failed (a summary of
failures is printed either way; no external test framework required).
終了コード 0=全チェック合格、1=1つ以上失敗（結果に関わらずサマリを
表示する。外部テストフレームワーク不要）。
"""

import ast
import os
import subprocess
import sys
from pathlib import Path
from typing import Set

REPO_ROOT = Path(__file__).resolve().parents[2]
INSTALLER_GUI_SCRIPT = REPO_ROOT / "tools" / "installer_gui" / "stampfly_installer.py"
CLI_INSTALLER_SCRIPT = REPO_ROOT / "scripts" / "installer.py"
RELEASE_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "release.yml"

# The four per-platform asset-name substrings the release body
# (.github/workflows/release.yml `body:`) must document, one per
# installer_gui matrix leg. Kept literal (not built from a template) so a
# typo in either this list or release.yml's prose is caught by a plain
# string search, with no YAML/template parsing involved.
# release本文(.github/workflows/release.yml の `body:`)が案内すべき、
# installer_gui の各マトリクスレッグに対応する4つのアセット名部分文字列。
# テンプレートから組み立てず直書きにすることで、本リストか release.yml の
# 文面のどちらかに誤字があっても、YAML/テンプレート解析なしの単純な文字列
# 検索で検出できるようにする。
# Tag-less stable names: GitHub's releases/latest/download/<asset>
# permalink (used by the landing page's OS-detected download button and
# the README's one-click links) requires the filename to stay identical
# across releases.
# タグ無し固定名: GitHub の releases/latest/download/<asset> 恒久リンク
# (Landing ページの OS 自動判定ダウンロードボタンと README の1クリック
# リンクが使用)は、リリースを跨いでファイル名が同一であることを要求する。
EXPECTED_ASSET_NAME_SUBSTRINGS = [
    "StampFlySetup_windows-x64.exe",
    "StampFlySetup_macos-arm64.zip",
    "StampFlySetup_macos-x64.zip",
    "StampFlySetup_linux-x64",
]

# Deadline for the --selftest subprocess. installer.py's own prerequisite
# probing (docs/plans/gui-installer-plan.md §4) may touch the network
# (e.g. checking git/reachability), so this is generous CI headroom, not a
# tight local-loop timeout -- mirrors check_upgrade.py's timeout-minutes
# convention (a guard rail, not an expected duration).
# --selftest subprocessの制限時間。installer.py自身の前提条件プローブ
# (docs/plans/gui-installer-plan.md §4)はネットワークに触れる場合がある
# (git疎通確認等)ため、ローカルループ向けの厳しいタイムアウトではなく
# CI向けに余裕を持たせている -- check_upgrade.py の timeout-minutes と
# 同じ「想定所要時間ではなくガードレール」という考え方に倣う。
SELFTEST_TIMEOUT_SECONDS = 600


# ---------------------------------------------------------------------------
# Checks
# 各チェック
# ---------------------------------------------------------------------------


def check_selftest_exit_zero() -> None:
    """`stampfly_installer.py --selftest`, run with the system Python and
    SF_SETUP_REPO pointed at this checkout, exits 0."""
    if not INSTALLER_GUI_SCRIPT.exists():
        raise AssertionError(
            f"{INSTALLER_GUI_SCRIPT} not found. tools/installer_gui/stampfly_installer.py "
            "is authored by a separate, parallel task (see docs/plans/gui-installer-plan.md); "
            "this check starts passing once that file lands."
        )

    env = os.environ.copy()
    env["SF_SETUP_REPO"] = str(REPO_ROOT)
    try:
        result = subprocess.run(
            [sys.executable, str(INSTALLER_GUI_SCRIPT), "--selftest"],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            encoding="utf-8",
            errors="replace",  # cp932-safe on Windows runners / Windowsランナーでもcp932で落ちない
            timeout=SELFTEST_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise AssertionError(
            f"--selftest did not finish within {SELFTEST_TIMEOUT_SECONDS}s"
        ) from exc

    assert result.returncode == 0, (
        f"--selftest exited {result.returncode}\n"
        f"--- stdout ---\n{result.stdout}\n"
        f"--- stderr ---\n{result.stderr}"
    )


def _imported_module_names(source_path: Path) -> Set[str]:
    """Return every module name imported anywhere in `source_path`
    (top-level and nested inside functions/classes/try-blocks alike),
    using `ast.walk` so no import site is missed regardless of nesting.

    For `import a.b.c` the full dotted path ("a.b.c") is returned, not
    just the top-level package -- this deliberately requires an exact
    matching import (not just "the top package happens to be imported")
    on the stampfly_installer.py side, since a bare `import ctypes` does
    not guarantee PyInstaller's analyzer also bundles the `ctypes.util`
    submodule.

    `source_path` 内のあらゆる場所(トップレベルだけでなく関数/クラス/
    try節の内側も含む)でimportされているモジュール名を全て返す。
    `ast.walk` を使うことで入れ子の深さに関わらずimport箇所を漏らさない。

    `import a.b.c` の場合はトップレベルパッケージ名だけでなく完全な
    ドット区切りパス("a.b.c")を返す -- stampfly_installer.py 側でも
    厳密に一致するimportを要求するのが意図的な設計で、単に `import ctypes`
    があるだけでは PyInstaller の解析器が `ctypes.util` サブモジュールも
    同梱する保証にならないため。
    """
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    modules: Set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                modules.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            # node.level > 0 means a relative import (`from . import x`),
            # which cannot apply here (both files are standalone scripts,
            # not package members) -- skip rather than record `None`.
            # node.level > 0 は相対import(`from . import x`)を意味し、
            # 両ファイルともパッケージの一部ではない単独スクリプトのため
            # 該当しない -- `None` を記録せずスキップする。
            if node.module is not None and node.level == 0:
                modules.add(node.module)
    return modules


def check_stdlib_imports_covered() -> None:
    """Every module scripts/installer.py imports must also be spelled out
    as an import somewhere in stampfly_installer.py's own source (the
    PyInstaller hidden-import coverage contract)."""
    if not INSTALLER_GUI_SCRIPT.exists():
        raise AssertionError(
            f"{INSTALLER_GUI_SCRIPT} not found. tools/installer_gui/stampfly_installer.py "
            "is authored by a separate, parallel task (see docs/plans/gui-installer-plan.md); "
            "this check starts passing once that file lands."
        )
    assert CLI_INSTALLER_SCRIPT.exists(), f"{CLI_INSTALLER_SCRIPT} not found"

    required = _imported_module_names(CLI_INSTALLER_SCRIPT)
    present = _imported_module_names(INSTALLER_GUI_SCRIPT)

    missing = sorted(required - present)
    assert not missing, (
        f"scripts/installer.py imports {sorted(required)}, but "
        f"tools/installer_gui/stampfly_installer.py's source is missing an import "
        f"for: {missing}. Per docs/plans/gui-installer-plan.md §5, add "
        f"`import {missing[0]}` (or equivalent) to stampfly_installer.py so "
        f"PyInstaller's static analysis discovers it -- scripts/installer.py "
        f"is process-in-imported at runtime from a clone that does not exist "
        f"at PyInstaller build time, so it cannot be discovered any other way."
    )


def check_release_yml_has_assets() -> None:
    """.github/workflows/release.yml documents all four StampFlySetup_
    asset-name patterns in its release body."""
    assert RELEASE_WORKFLOW.exists(), f"{RELEASE_WORKFLOW} not found"
    text = RELEASE_WORKFLOW.read_text(encoding="utf-8")

    missing = [s for s in EXPECTED_ASSET_NAME_SUBSTRINGS if s not in text]
    assert not missing, (
        f".github/workflows/release.yml is missing the following StampFlySetup_ "
        f"asset-name string(s) in its release body: {missing}"
    )


# ---------------------------------------------------------------------------
# Runner
# 実行部
# ---------------------------------------------------------------------------

CHECKS = [
    ("release.yml asset names", check_release_yml_has_assets),
    ("stdlib hidden-import coverage", check_stdlib_imports_covered),
    ("--selftest exit code", check_selftest_exit_zero),
]


def main() -> int:
    failures = []
    for name, check_fn in CHECKS:
        try:
            check_fn()
        except AssertionError as exc:
            failures.append((name, str(exc)))
            print(f"FAIL {name}: {exc}")
        except Exception as exc:  # noqa: BLE001 - report any unexpected error, not just AssertionError
            failures.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"PASS {name}")

    print()
    if failures:
        print(f"{len(failures)}/{len(CHECKS)} check(s) FAILED")
        return 1
    print(f"All {len(CHECKS)} checks PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
