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

Covers / 対象（仕様C1テスト項目 (a)〜(g)）:
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
    (g) When the native GUI Flasher is NOT installed, `sf upgrade` offers
        to install it exactly once per checkout: shown on the first run
        (declined via stdin EOF, falling back to the default No), then
        suppressed on a second run because the marker file was written.
        SKIPped (not failed) if this runner machine actually has the
        native flasher installed, since the branch under test could
        never trigger for real.
        フラッシャ未導入時、`sf upgrade`はチェックアウトにつき一回だけ
        インストールを提案する: 初回は表示（stdin EOFで辞退＝既定Noに
        フォールバック）、2回目はマーカーファイルにより抑止される。
        このマシンに実際にネイティブフラッシャが導入済みの場合はSKIP
        （失敗ではない）-- 未導入分岐が実際には発生し得ないため。

Usage / 使い方:
    python3 tools/ci/check_upgrade.py

Exit code 0 = all checks passed or intentionally skipped, 1 = at least
one failed (a summary is printed either way; no external test framework
required).
終了コード 0=全チェック合格または意図的スキップ、1=1つ以上失敗
（結果に関わらずサマリを表示、外部テストフレームワーク不要）。
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

# Marker file lib/sfcli/commands/upgrade.py writes under <repo>/.sf/ after
# the one-time "install the Flasher?" offer (case (g)). Must match
# FLASHER_OFFER_MARKER_FILENAME there exactly.
# lib/sfcli/commands/upgrade.py が一回限りの「フラッシャを入れますか？」
# 提案後に <repo>/.sf/ 配下へ書き込むマーカーファイル（ケース(g)）。
# 同ファイルの FLASHER_OFFER_MARKER_FILENAME と厳密に一致させる。
FLASHER_OFFER_MARKER_FILENAME = "flasher_install_offered"


class CheckSkipped(Exception):
    """Raised by a check function to mean "intentionally not run" rather
    than "failed" -- e.g. case (g) when this runner machine already has
    the native Flasher installed, so the not-installed branch under test
    could never trigger for real. main() reports these separately and
    does not count them toward the failure total.
    チェック関数が「失敗」ではなく「意図的に未実行」であることを示すために
    送出する -- 例: このマシンに既にネイティブフラッシャが導入済みで、
    テスト対象の未導入分岐が実際には発生し得ない場合のケース(g)。main()は
    これを個別に報告し、失敗数には数えない。
    """


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


def _native_flasher_skip_reason() -> Optional[str]:
    """Reason to SKIP case (g), or None if it is safe to run on this
    machine.

    Case (g) exercises the "native GUI Flasher is NOT installed" branch,
    so it needs THIS runner machine (not the fixture) to genuinely have
    no flasher installed. We check that by importing
    sfcli.utils.flasher_install from the MAIN repo's lib/ (not the
    fixture clone's copy -- the two are identical at this point in the
    script, but the main repo's is simpler to reach without first
    building a fixture). The module is stdlib-only and
    installed_app_executable() is a pure, read-only per-OS path check (no
    network, see lib/sfcli/utils/flasher_install/_macos.py etc.), so
    calling it directly here is safe and cheap.

    ケース(g)のSKIP理由を返す。このマシンで実行して問題なければNone。

    ケース(g)は「ネイティブGUIフラッシャが未導入」の分岐を検証するため、
    フィクスチャではなく実行マシン自体が本当に未導入である必要がある。
    これを確認するため、フィクスチャクローンのコピーではなく、メイン
    リポジトリの lib/ から sfcli.utils.flasher_install を import する
    （この時点では両者は同一内容だが、メインリポジトリの方はフィクスチャ
    構築なしに到達できるぶん単純）。同モジュールは標準ライブラリのみで
    実装され、installed_app_executable() は読み取り専用のOS別パス確認
    のみ（ネットワーク無し、lib/sfcli/utils/flasher_install/_macos.py 等
    を参照）のため、ここで直接呼んでも安全かつ軽量。
    """
    main_lib_dir = str(REPO_ROOT / "lib")
    sys.path.insert(0, main_lib_dir)
    try:
        from sfcli.utils import flasher_install  # noqa: PLC0415 - deliberately deferred, see docstring

        if flasher_install.installed_app_executable() is not None:
            return "native flasher present on this machine"
        return None
    except Exception as exc:  # noqa: BLE001 - any failure means "cannot confirm it's absent"
        return f"could not determine native flasher status ({type(exc).__name__}: {exc})"
    finally:
        sys.path.remove(main_lib_dir)


def check_flasher_first_time_offer(tmp_root: Path) -> None:
    """(g) When the native GUI Flasher is NOT installed, `sf upgrade`
    offers to install it exactly once per checkout.

    First run (no --yes): the update-preview prompt ("Proceed with
    upgrade?") consumes "y" from stdin; the flasher offer prompt that
    follows then hits EOF on stdin and falls back to its default (No) --
    declined, so `flasher_install.install()` (and therefore the network)
    is never reached. Asserts the offer text and the "sf flasher install"
    pointer are shown, and that the marker file was written.

    Second run (another upstream commit, same stdin): asserts the offer
    text does NOT appear again, because the marker from the first run
    suppresses it.

    (g) ネイティブGUIフラッシャが未導入の場合、`sf upgrade`はチェック
    アウトにつき一回だけインストールを提案することを検証する。

    初回実行（--yes無し）: 更新プレビューの確認プロンプト
    （"Proceed with upgrade?"）がstdinの"y"を消費し、続くフラッシャ提案の
    プロンプトはstdinのEOFに達して既定値（No）にフォールバックする --
    辞退となるため`flasher_install.install()`（＝ネットワーク）には
    到達しない。提案文言と"sf flasher install"への案内が表示されること、
    マーカーファイルが書かれることを確認する。

    2回目実行（さらに1コミットpush、同じstdin）: 1回目のマーカーにより
    提案文言が再表示されないことを確認する。
    """
    skip_reason = _native_flasher_skip_reason()
    if skip_reason is not None:
        raise CheckSkipped(skip_reason)

    seed_dir, _, clone_dir = _build_fixture(tmp_root, "case_g_flasher_offer")

    def mutate(seed_dir: Path) -> None:
        (seed_dir / "README.md").write_text(
            BASELINE_README + "upstream v2 for flasher offer case\n", encoding="utf-8"
        )

    _push_upstream_change(seed_dir, mutate, "v2: upstream readme for flasher offer case")

    # No --yes: the proceed-prompt consumes "y"; the flasher offer prompt
    # then hits EOF -> declined -> no network is ever touched.
    # --yes無し: proceedプロンプトが"y"を消費し、続くフラッシャ提案は
    # EOFに達する -> 辞退 -> ネットワークには一切触れない。
    result = _run_upgrade(clone_dir, [], stdin_text="y\n")
    assert result.returncode == 0, (
        f"expected exit 0, got {result.returncode}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )

    combined_output = result.stdout + result.stderr
    assert "one-time offer" in combined_output, (
        f"first run should show the one-time install offer; full output:\n{combined_output}"
    )
    assert "sf flasher install" in combined_output, (
        f"declined offer should point at 'sf flasher install'; full output:\n{combined_output}"
    )

    marker_path = clone_dir / ".sf" / FLASHER_OFFER_MARKER_FILENAME
    assert marker_path.exists(), f"marker file should exist at {marker_path} after the one-time offer"

    # --- Second run: another upstream commit; the marker must suppress the offer. ---
    # --- 2回目実行: 別のupstreamコミット。マーカーが提案を抑止するはず ---
    def mutate2(seed_dir: Path) -> None:
        (seed_dir / "README.md").write_text(
            BASELINE_README + "upstream v2 for flasher offer case\nupstream v3 line\n",
            encoding="utf-8",
        )

    _push_upstream_change(seed_dir, mutate2, "v3: second upstream commit")

    result2 = _run_upgrade(clone_dir, [], stdin_text="y\n")
    assert result2.returncode == 0, (
        f"expected exit 0, got {result2.returncode}\nSTDOUT:\n{result2.stdout}\nSTDERR:\n{result2.stderr}"
    )

    combined_output2 = result2.stdout + result2.stderr
    assert "one-time offer" not in combined_output2, (
        f"second run should NOT show the offer again (marker should suppress it); "
        f"full output:\n{combined_output2}"
    )

    # --- Third run: ALREADY-UP-TO-DATE path must also fire the offer. This is
    # the bootstrap audience's actual arrival path (plain `git pull` first,
    # then `sf upgrade` -> "Already up to date"), so the offer must not be
    # gated behind an actual pull. Delete the marker to re-arm the offer,
    # run with no new upstream commits, and expect the offer text.
    # --- 3回目実行: 「最新です」経路でも提案が出ること。ブートストラップ組の
    # 実際の到達経路（先に素の `git pull` → `sf upgrade` → "Already up to
    # date"）なので、提案が「実際に pull が起きた時」に閉じ込められていては
    # ならない。マーカーを削除して提案を再度有効化し、upstream に新コミットが
    # 無い状態で実行して提案文言が出ることを確認する。 ---
    marker_path.unlink()
    result3 = _run_upgrade(clone_dir, [], stdin_text="")
    assert result3.returncode == 0, (
        f"expected exit 0, got {result3.returncode}\nSTDOUT:\n{result3.stdout}\nSTDERR:\n{result3.stderr}"
    )
    combined_output3 = result3.stdout + result3.stderr
    assert "Already up to date" in combined_output3, (
        f"third run should hit the up-to-date path; full output:\n{combined_output3}"
    )
    assert "one-time offer" in combined_output3, (
        f"the up-to-date path must also show the one-time offer; full output:\n{combined_output3}"
    )
    assert marker_path.exists(), "the up-to-date offer must also write the marker"


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
    ("(g) one-time flasher install offer", check_flasher_first_time_offer),
]


def main() -> int:
    failures = []
    skipped = []
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
            except CheckSkipped as exc:
                # Intentional, not a failure -- e.g. case (g) on a machine
                # that already has the native flasher installed.
                # 意図的なもので失敗ではない -- 例: ネイティブフラッシャが
                # 既に導入済みのマシンでのケース(g)。
                skipped.append((name, str(exc)))
                print(f"SKIPPED {name}: {exc}")
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
    ran_count = len(CHECKS) - len(skipped)
    if skipped:
        print(f"All {ran_count}/{len(CHECKS)} checks PASSED ({len(skipped)} SKIPPED)")
    else:
        print(f"All {len(CHECKS)} checks PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
