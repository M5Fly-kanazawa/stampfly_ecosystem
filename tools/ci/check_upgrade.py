#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/ci/check_upgrade.py

Dependency-free automated test for `sf upgrade` (lib/sfcli/commands/upgrade.py).
`sf upgrade`（lib/sfcli/commands/upgrade.py）の依存フリーな自動テスト。

Everything runs against a local, throwaway bare-remote + clone fixture
built under a temp directory. The real repository is never touched --
each check builds its own {seed, bare "origin", clone} triple from
scratch, copies THIS repo's current lib/sfcli source tree into the seed
so the code under test is the real implementation (not a stub), and
invokes `sf upgrade` as a subprocess with:
    PYTHONPATH=<clone>/lib python -m sfcli.cli upgrade [args...]
run with cwd=<clone> -- so paths.root() (which walks up from its own
__file__) resolves to the fixture clone, never the real repository.
すべてローカルの使い捨てbareリモート+クローン一式を一時ディレクトリ配下に
構築して検証する。メインリポジトリには一切触れない -- 各チェックは
{seed, bare "origin", clone} の3つ組をゼロから構築し、このリポジトリの
現在のlib/sfcliソースツリーをseedへコピーする（テスト対象がスタブでは
なく実装そのものになるように）。`sf upgrade` は
`PYTHONPATH=<clone>/lib python -m sfcli.cli upgrade [args...]` の形で
cwd=<clone> のサブプロセスとして呼び出す -- これにより
paths.root()（自分自身の__file__から上に辿る）はメインリポジトリでは
なく必ずこのフィクスチャのクローンに解決される。

Covers / 対象（仕様C1テスト項目 (a)〜(f)）:
    (a) Clean local state -> successful fast-forward update.
        クリーン時のff更新成功。
    (b) Non-conflicting local edit is preserved via stash push/pop.
        非衝突ローカル変更がstash経由で保全される。
    (c) A stash-pop conflict leaves the stash intact, exits 2, and
        prints the required plain-language guidance.
        衝突時にstashが残りexit 2 + 案内文言が出る。
    (d) Dependency resync (`pip install -e .`) is reached on every
        successful upgrade; SF_UPGRADE_SKIP_PIP=1 stubs the real pip
        call so this is verifiable without network access.
        依存同期(pip install -e .)に必ず到達する。
        SF_UPGRADE_SKIP_PIP=1で実pip呼び出しをスタブ化し検証する。
    (e) An upstream sdkconfig.defaults change backs up the existing
        firmware/<target>/sdkconfig.
        sdkconfig.defaults変更でsdkconfigが退避される。
    (f) --discard-local discards tracked local edits (after an explicit
        confirmation) instead of stashing them.
        --discard-localでローカル変更が破棄される。

Usage / 使い方:
    python3 tools/ci/check_upgrade.py

Exit code 0 = all checks passed, 1 = at least one failed (a summary is
printed either way; no external test framework required).
終了コード 0=全チェック合格、1=1つ以上失敗（結果に関わらずサマリを
表示、外部テストフレームワーク不要）。
"""

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Callable, List, Optional, Tuple

# Repo root, from this script's own location (tools/ci/check_upgrade.py).
# このスクリプト自身の位置から見たリポジトリルート
REPO_ROOT = Path(__file__).resolve().parents[2]

# --- Fixture constants -----------------------------------------------------
# フィクスチャ用の定数

MAIN_BRANCH = "main"
GIT_TIMEOUT_SEC = 30.0
UPGRADE_SUBPROCESS_TIMEOUT_SEC = 60.0

# Must match lib/sfcli/commands/upgrade.py's SKIP_PIP_ENV_VAR exactly --
# this is the C1/test contract that lets case (d) verify the dependency
# resync step is reached without a real pip / network call.
# lib/sfcli/commands/upgrade.py の SKIP_PIP_ENV_VAR と厳密に一致させる
# 必要がある -- ケース(d)が実pip/ネットワーク呼び出し無しで依存同期
# ステップへの到達を検証するためのテスト契約。
SKIP_PIP_ENV_VAR = "SF_UPGRADE_SKIP_PIP"

BASELINE_README = "seed v1\nline2\nline3\n"
BASELINE_PYPROJECT = '[project]\nname = "stampfly-ecosystem-fixture"\nversion = "0.1.0"\n'
BASELINE_REQUIREMENTS = "# minimal fixture requirements\n"
BASELINE_SDKCONFIG_DEFAULTS = "CONFIG_A=y\n"
BASELINE_PARTITIONS = "# partitions v1\n"
BASELINE_SDKCONFIG = "CONFIG_A=y\n# generated (baseline)\n"


# ---------------------------------------------------------------------------
# Fixture builders
# フィクスチャ構築
# ---------------------------------------------------------------------------


def _git(cwd: Optional[Path], git_args: List[str]) -> subprocess.CompletedProcess:
    """Run `git <git_args>`, raising with full output on failure. Used
    only for fixture setup, not the code under test.
    `git <git_args>` を実行し、失敗時は全出力付きで例外を出す。
    フィクスチャ構築専用（テスト対象コードではない）。
    """
    result = subprocess.run(
        ["git", *git_args],
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        timeout=GIT_TIMEOUT_SEC,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"fixture setup: git {' '.join(git_args)} failed (cwd={cwd})\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}"
        )
    return result


def _copy_sfcli_source(dest_lib_dir: Path) -> None:
    """Copy the real lib/sfcli tree into dest_lib_dir/sfcli, so the
    subprocess invocations under test run the actual implementation
    being verified (not a stub) while resolving their own repo root to
    the fixture clone (see module docstring).
    実際のlib/sfcliツリーをdest_lib_dir/sfcliへコピーする。これにより
    テスト対象の呼び出しは実装そのものを実行しつつ、自身のリポジトリ
    ルートをフィクスチャのクローンへ解決する（モジュールdocstring参照）。
    """
    shutil.copytree(
        REPO_ROOT / "lib" / "sfcli",
        dest_lib_dir / "sfcli",
        ignore=shutil.ignore_patterns("__pycache__"),
    )


def _write_baseline_files(seed_dir: Path) -> None:
    """Write the v1 baseline content shared by every fixture."""
    (seed_dir / "pyproject.toml").write_text(BASELINE_PYPROJECT, encoding="utf-8")
    (seed_dir / "requirements.txt").write_text(BASELINE_REQUIREMENTS, encoding="utf-8")
    (seed_dir / "README.md").write_text(BASELINE_README, encoding="utf-8")

    firmware_dir = seed_dir / "firmware" / "vehicle"
    firmware_dir.mkdir(parents=True, exist_ok=True)
    (firmware_dir / "sdkconfig.defaults").write_text(BASELINE_SDKCONFIG_DEFAULTS, encoding="utf-8")
    (firmware_dir / "partitions.csv").write_text(BASELINE_PARTITIONS, encoding="utf-8")
    (firmware_dir / "sdkconfig").write_text(BASELINE_SDKCONFIG, encoding="utf-8")


def _build_fixture(tmp_root: Path, case_name: str) -> Tuple[Path, Path, Path]:
    """Build a fresh {seed, bare "origin", clone} triple for one check,
    seeded at v1 with this repo's real lib/sfcli copied in.

    `seed` plays the role of "someone else's machine that pushes
    upstream changes"; `clone` is "the user's local checkout" that
    `sf upgrade` actually operates on.
    1チェック分の {seed, bare "origin", clone} の3つ組をv1状態で新規に
    構築する（このリポジトリの実際のlib/sfcliを組み込み済み）。
    `seed` は「上流に変更をpushする他人のマシン」、`clone` は
    `sf upgrade` が実際に操作する「ユーザーのローカルチェックアウト」
    の役を果たす。

    Returns (seed_dir, bare_dir, clone_dir).
    """
    case_dir = tmp_root / case_name
    seed_dir = case_dir / "seed"
    bare_dir = case_dir / "origin.git"
    clone_dir = case_dir / "clone"

    seed_dir.mkdir(parents=True)
    _git(seed_dir, ["init", "-b", MAIN_BRANCH])
    _git(seed_dir, ["config", "user.email", "seed@example.invalid"])
    _git(seed_dir, ["config", "user.name", "check_upgrade seed"])

    _copy_sfcli_source(seed_dir / "lib")
    _write_baseline_files(seed_dir)

    _git(seed_dir, ["add", "-A"])
    _git(seed_dir, ["commit", "-m", "v1: baseline"])

    _git(case_dir, ["clone", "--bare", str(seed_dir), str(bare_dir)])
    _git(seed_dir, ["remote", "add", "origin", str(bare_dir)])

    _git(case_dir, ["clone", str(bare_dir), str(clone_dir)])
    _git(clone_dir, ["config", "user.email", "clone@example.invalid"])
    _git(clone_dir, ["config", "user.name", "check_upgrade clone"])

    return seed_dir, bare_dir, clone_dir


def _push_upstream_change(seed_dir: Path, mutate_fn: Callable[[Path], None], message: str) -> None:
    """Apply `mutate_fn(seed_dir)`, commit it, and push to origin -- the
    "someone else pushed to main" step each scenario builds on top of
    the v1 baseline.
    `mutate_fn(seed_dir)` を適用してコミットし、originへpushする --
    各シナリオがv1ベースラインの上に積む「誰かがmainへpushした」手順。
    """
    mutate_fn(seed_dir)
    _git(seed_dir, ["add", "-A"])
    _git(seed_dir, ["commit", "-m", message])
    _git(seed_dir, ["push", "origin", MAIN_BRANCH])


def _run_upgrade(
    clone_dir: Path,
    extra_args: List[str],
    extra_env: Optional[dict] = None,
    stdin_text: str = "",
) -> subprocess.CompletedProcess:
    """Invoke `sf upgrade` against `clone_dir` as a real subprocess:
        PYTHONPATH=<clone_dir>/lib python -m sfcli.cli upgrade [args...]
    with cwd=<clone_dir> (spec C1 test harness requirement).
    `clone_dir` に対して `sf upgrade` を実サブプロセスとして呼び出す
    （仕様C1テストハーネス要件）。
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = str(clone_dir / "lib")
    # Deterministic, colorless output so substring assertions are stable
    # regardless of the invoking terminal.
    # 呼び出し元端末に依存せず部分文字列アサーションが安定するよう、
    # 決定的でカラーコード無しの出力にする。
    env["NO_COLOR"] = "1"
    # Stub pip in EVERY case, not just (d): this harness must never run a
    # real pip against a fixture clone (network access, environment
    # pollution, and — since dependency-sync failure is reflected in the
    # exit code — a sandboxed/offline run would fail unrelated cases).
    # Case (d) passes the same value explicitly because verifying the stub
    # marker is its entire point.
    # 全ケースで pip をスタブ化する（(d)だけではない）: 本ハーネスは
    # fixture クローンに対して本物の pip を絶対に実行しない（ネットワーク
    # アクセス・環境汚染に加え、依存同期の失敗は終了コードに反映される
    # ため、サンドボックス/オフライン実行では無関係なケースまで失敗して
    # しまう）。ケース(d)はスタブマーカーの検証自体が目的なので、同じ値を
    # 明示的に渡している。
    env.setdefault(SKIP_PIP_ENV_VAR, "1")
    if extra_env:
        env.update(extra_env)

    return subprocess.run(
        [sys.executable, "-m", "sfcli.cli", "upgrade", *extra_args],
        cwd=str(clone_dir),
        env=env,
        capture_output=True,
        text=True,
        input=stdin_text,
        timeout=UPGRADE_SUBPROCESS_TIMEOUT_SEC,
    )


# ---------------------------------------------------------------------------
# Checks (a)-(f)
# チェック (a)〜(f)
# ---------------------------------------------------------------------------


def check_clean_fast_forward(tmp_root: Path) -> None:
    """(a) Clean local state -> `sf upgrade --yes` fast-forwards cleanly."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_a_clean_ff")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "README.md").write_text(BASELINE_README + "upstream v2 line\n", encoding="utf-8")

    _push_upstream_change(seed_dir, mutate, "v2: upstream readme update")

    result = _run_upgrade(clone_dir, ["--yes"])
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )
    assert "upstream v2 line" in (clone_dir / "README.md").read_text(encoding="utf-8"), (
        "upstream change should be present after fast-forward"
    )

    head = _git(clone_dir, ["rev-parse", "HEAD"]).stdout.strip()
    origin_head = _git(clone_dir, ["rev-parse", f"origin/{MAIN_BRANCH}"]).stdout.strip()
    assert head == origin_head, "HEAD should have fast-forwarded to origin/main"


def check_noconflict_stash(tmp_root: Path) -> None:
    """(b) A non-conflicting local edit survives via stash push/pop."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_b_stash")

    # Local uncommitted edit to a tracked file the upstream commit does
    # not touch, so the stash pop cannot conflict.
    # 上流コミットが触れない追跡ファイルへのローカル未コミット編集
    # （stash popが衝突しないようにするため）。
    local_marker = "LOCAL WIP NOTE"
    (clone_dir / "README.md").write_text(BASELINE_README + f"\n{local_marker}\n", encoding="utf-8")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "firmware" / "vehicle" / "partitions.csv").write_text(
            BASELINE_PARTITIONS + "# v2 partition row\n", encoding="utf-8"
        )

    _push_upstream_change(seed_dir, mutate, "v2: upstream partitions update")

    result = _run_upgrade(clone_dir, ["--yes"])
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )

    readme = (clone_dir / "README.md").read_text(encoding="utf-8")
    assert local_marker in readme, "local uncommitted edit should survive via stash pop"

    partitions = (clone_dir / "firmware" / "vehicle" / "partitions.csv").read_text(encoding="utf-8")
    assert "v2 partition row" in partitions, "upstream change should also be present"

    stash_list = _git(clone_dir, ["stash", "list"]).stdout.strip()
    assert stash_list == "", f"stash should have been popped cleanly, but: {stash_list!r}"


def check_conflict_stash(tmp_root: Path) -> None:
    """(c) A stash-pop conflict: stash stays, exit 2, guidance printed."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_c_conflict")

    # Local edit and upstream edit both touch line 1 of README.md, so
    # `git stash pop` (after the upstream line-1 change is fast-forwarded
    # in) must conflict.
    # ローカル編集と上流の編集がREADME.mdの1行目を同時に変更するため、
    # (上流の1行目変更がff mergeで取り込まれた後の) `git stash pop`は
    # 必ず衝突する。
    local_lines = BASELINE_README.splitlines()
    local_lines[0] = "LOCAL CONFLICTING EDIT"
    (clone_dir / "README.md").write_text("\n".join(local_lines) + "\n", encoding="utf-8")

    def mutate(seed_dir: Path) -> None:
        upstream_lines = BASELINE_README.splitlines()
        upstream_lines[0] = "UPSTREAM CONFLICTING EDIT"
        (seed_dir / "README.md").write_text("\n".join(upstream_lines) + "\n", encoding="utf-8")

    _push_upstream_change(seed_dir, mutate, "v2: upstream conflicting readme edit")

    result = _run_upgrade(clone_dir, ["--yes"])
    assert result.returncode == 2, (
        f"expected exit 2 (needs attention), got {result.returncode}\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )

    combined_output = result.stdout + result.stderr
    required_phrases = ("stash", "git add", "git stash drop", "<<<<<<<")
    for phrase in required_phrases:
        assert phrase in combined_output, (
            f"guidance text should mention {phrase!r}; full output:\n{combined_output}"
        )

    stash_list = _git(clone_dir, ["stash", "list"]).stdout.strip()
    assert stash_list != "", "the stash entry must remain so the user's edit is not lost"


def check_pip_sync_invoked(tmp_root: Path) -> None:
    """(d) Dependency resync (`pip install -e .`) is reached and would
    run, verified via the SF_UPGRADE_SKIP_PIP stub (no real pip/network
    call needed)."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_d_pip_stub")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "pyproject.toml").write_text(
            BASELINE_PYPROJECT + "# v2: dependency bump\n", encoding="utf-8"
        )

    _push_upstream_change(seed_dir, mutate, "v2: pyproject dependency bump")

    result = _run_upgrade(clone_dir, ["--yes"], extra_env={SKIP_PIP_ENV_VAR: "1"})
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )
    assert SKIP_PIP_ENV_VAR in result.stdout, (
        f"stub marker should mention {SKIP_PIP_ENV_VAR}; STDOUT:\n{result.stdout}"
    )
    assert "-e" in result.stdout and str(clone_dir) in result.stdout, (
        f"stubbed dependency sync should report the 'pip install -e <root>' command it would "
        f"run; STDOUT:\n{result.stdout}"
    )
    assert "-r" in result.stdout and "requirements.txt" in result.stdout, (
        f"requirements.txt is present in the fixture, so its -r install should also be reported; "
        f"STDOUT:\n{result.stdout}"
    )


def check_sdkconfig_backup(tmp_root: Path) -> None:
    """(e) An upstream sdkconfig.defaults change backs up the existing
    firmware/<target>/sdkconfig."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_e_sdkconfig")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "firmware" / "vehicle" / "sdkconfig.defaults").write_text(
            BASELINE_SDKCONFIG_DEFAULTS + "CONFIG_NEW_FEATURE=y\n", encoding="utf-8"
        )

    _push_upstream_change(seed_dir, mutate, "v2: sdkconfig.defaults changed")

    result = _run_upgrade(clone_dir, ["--yes"])
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )

    sdkconfig_path = clone_dir / "firmware" / "vehicle" / "sdkconfig"
    backup_path = clone_dir / "firmware" / "vehicle" / "sdkconfig.pre-upgrade-backup"
    assert not sdkconfig_path.exists(), "stale sdkconfig should have been renamed away"
    assert backup_path.exists(), "renamed backup should exist as sdkconfig.pre-upgrade-backup"
    assert backup_path.read_text(encoding="utf-8") == BASELINE_SDKCONFIG, (
        "backup should hold the untouched pre-upgrade sdkconfig content"
    )


def check_discard_local(tmp_root: Path) -> None:
    """(f) --discard-local discards tracked local edits (after an
    explicit confirmation, fed via stdin) instead of stashing them."""
    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_f_discard")

    (clone_dir / "README.md").write_text("LOCAL edit to be discarded\n", encoding="utf-8")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "README.md").write_text(
            BASELINE_README + "upstream v2 for discard case\n", encoding="utf-8"
        )

    _push_upstream_change(seed_dir, mutate, "v2: upstream readme for discard case")

    # --discard-local always confirms even with --yes (spec C1#4) --
    # supply the confirmation via stdin.
    # --discard-localは--yesでも必ず確認する（仕様C1項目4） --
    # 確認はstdin経由で与える。
    result = _run_upgrade(clone_dir, ["--yes", "--discard-local"], stdin_text="y\n")
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )

    readme = (clone_dir / "README.md").read_text(encoding="utf-8")
    assert "LOCAL edit to be discarded" not in readme, "local edit should have been discarded"
    assert "upstream v2 for discard case" in readme, "upstream content should be present after ff-merge"


# ---------------------------------------------------------------------------
# Runner
# 実行部
# ---------------------------------------------------------------------------

CHECKS = [
    ("(a) clean fast-forward", check_clean_fast_forward),
    ("(b) non-conflicting stash", check_noconflict_stash),
    ("(c) conflicting stash pop", check_conflict_stash),
    ("(d) dependency resync reached", check_pip_sync_invoked),
    ("(e) sdkconfig staleness backup", check_sdkconfig_backup),
    ("(f) --discard-local", check_discard_local),
]


def main() -> int:
    failures = []
    with tempfile.TemporaryDirectory(prefix="check_upgrade_") as tmp_root_str:
        # .resolve() matters on Windows: GitHub runners hand out %TEMP% in
        # 8.3 short form (C:\Users\RUNNERA~1\...), while the code under
        # test resolves its repo root to the LONG form — a raw-string
        # substring assertion (case (d)) then never matches. Resolving here
        # makes every fixture path long-form on all platforms.
        # Windows では .resolve() が重要: GitHub ランナーの %TEMP% は 8.3
        # 短縮形（C:\Users\RUNNERA~1\...）で渡される一方、テスト対象の
        # コードはリポジトリルートを「長い形式」に解決するため、生文字列の
        # 部分一致アサーション（ケース(d)）が一致しなくなる。ここで resolve
        # しておけば全プラットフォームで fixture パスが長い形式に揃う。
        tmp_root = Path(tmp_root_str).resolve()
        for name, check_fn in CHECKS:
            try:
                check_fn(tmp_root)
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
