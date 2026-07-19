"""
sf upgrade - Pull the latest ecosystem changes and resync the environment

Fetches the tracked remote branch, safely folds in the newest commits
while preserving any local edits, resyncs Python dependencies, flags
stale ESP-IDF sdkconfig files, and optionally updates the native GUI
Flasher app. Intended to be the one command a non-expert user needs to
run after being told "there's an update" -- see docs/guides/upgrading.md
for the plain-language walkthrough this command implements.

最新のリモート変更を取得し、ローカルの変更を保持したまま安全に取り込み、
Python依存関係を再同期し、陳腐化したESP-IDFのsdkconfigを検出し、必要なら
ネイティブGUIフラッシャも更新する。「アップデートがあります」と言われた
非エキスパートのユーザーが実行すべき唯一のコマンドを目指す
（この実装の平易な解説は docs/guides/upgrading.md を参照）。

STDLIB ONLY: this module must import cleanly even when every third-party
dependency (numpy, PyYAML, ...) is missing, because it is the documented
recovery path for a broken/partial install (see cli.py's
assert_all_commands_loadable() / scripts/installer.py's post-install
probe, which relies on this contract).
標準ライブラリのみを使用: 壊れた／中途半端なインストールからの復旧経路
として文書化されているため、サードパーティ依存（numpy、PyYAML 等）が
全て欠けていてもこのモジュール自体は import できなければならない
（cli.py の assert_all_commands_loadable() / scripts/installer.py の
インストール後プローブがこの契約に依存している）。
"""

import argparse
import datetime
import os
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

from ..utils import console, paths, flasher_install

COMMAND_NAME = "upgrade"
COMMAND_HELP = "Pull the latest changes and resync the environment"

# --- Named constants (no magic numbers scattered through the logic) -------
# 名前付き定数（マジックナンバーを禁止し、ロジック中に散らばらせない）

# Branch developers are expected to be on for a "normal" (non-developer)
# upgrade; anything else only produces an informational warning.
# 通常ユーザーが想定するブランチ。これ以外は情報warningのみ（続行する）。
MAIN_BRANCH_NAME = "main"

# `git log --oneline HEAD..origin/<branch> | head -15` — matches spec C1#3.
# 仕様C1項目3の "head -15" と同数
PREVIEW_LOG_LINE_LIMIT = 15

# Subprocess timeouts, tuned per operation (network fetch needs more slack
# than a purely local git call).
# サブプロセスのタイムアウト（ネットワークを伴うfetchはローカル操作より
# 余裕を持たせる）
GIT_NETWORK_TIMEOUT_SEC = 120.0
GIT_LOCAL_TIMEOUT_SEC = 30.0
PIP_SYNC_TIMEOUT_SEC = 600.0

# Suffix appended to a stale `sdkconfig` when its `.defaults` changed
# upstream (spec C1#6: rename to "sdkconfig.pre-upgrade-backup").
# 仕様C1項目6: defaults変化時の退避ファイル名サフィックス
SDKCONFIG_BACKUP_SUFFIX = ".pre-upgrade-backup"
SDKCONFIG_TRIGGER_FILENAMES = ("sdkconfig.defaults", "partitions.csv")

# `git status --porcelain` two-letter codes that mean "both sides touched
# this file" -- i.e. an unresolved merge/stash-pop conflict.
# "両者がこのファイルを変更した"＝未解決の衝突を意味する2文字コード
CONFLICT_STATUS_CODES = frozenset({"UU", "AA", "DD", "AU", "UA", "DU", "UD"})

# Exit codes. 0/1 follow the Unix convention (success / generic failure);
# 2 is reserved for "the upgrade did something safe but needs your
# attention" (stash conflict, diverged branch) per spec C1#4.
# 終了コード。0/1はUnix慣例（成功/一般エラー）。2は仕様C1項目4が定める
# 「安全に処理はしたがユーザー対応が必要」（stash衝突・分岐）専用。
EXIT_OK = 0
EXIT_GENERAL_ERROR = 1
EXIT_NEEDS_ATTENTION = 2

UPGRADING_DOC_PATH = "docs/guides/upgrading.md"

# Marker file recording that the one-time "install the Flasher?" offer
# (Step 7, when no native app is installed yet) has already been shown --
# accepted or declined -- for this checkout, so it is asked at most once.
# Lives in the same repo-local config directory as the installer's other
# .sf/ state (scripts/installer.py:670), which is already gitignored.
# 「フラッシャを入れますか？」という一回限りの提案（Step 7、ネイティブ
# アプリが未導入の場合）を、このチェックアウトで既に表示済み
# （承諾/辞退いずれか）であることを記録するマーカーファイル。インストーラの
# 他の .sf/ 状態と同じリポジトリローカル設定ディレクトリに置く
# （scripts/installer.py:670、既にgitignore済み）。
FLASHER_OFFER_MARKER_FILENAME = "flasher_install_offered"

# Test-only escape hatch (spec C1 test item (d)): when set, dependency
# resync prints what it *would* run instead of invoking pip, so
# tools/ci/check_upgrade.py can verify the step is reached without a real
# network / pip call.
# テスト専用の逃げ道（仕様テスト項目(d)）: 設定時は実際にpipを呼ばず
# 「これを実行するはずだった」と表示するだけにする。
# tools/ci/check_upgrade.py がネットワーク/pip呼び出し無しでこの手順に
# 到達したことを検証できるようにするため。
SKIP_PIP_ENV_VAR = "SF_UPGRADE_SKIP_PIP"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI / CLIへコマンドを登録する"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=(
            "Pull the latest changes from the tracked remote branch, keep local "
            "edits safe (auto-stash), resync Python dependencies, and flag stale "
            "ESP-IDF sdkconfig files.\n\n"
            "リモートの追跡ブランチから最新の変更を取得し、ローカルの変更は"
            "自動stashで保護しつつ取り込み、Python依存関係を再同期し、"
            "陳腐化したESP-IDFのsdkconfigを検出します。"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--yes", "-y",
        action="store_true",
        help="Do not prompt for the update preview (--discard-local is always confirmed)",
    )
    parser.add_argument(
        "--discard-local",
        action="store_true",
        help="Discard local changes to tracked files instead of stashing them "
             "(destructive; always confirmed, even with --yes)",
    )
    parser.add_argument(
        "--no-flasher",
        action="store_true",
        help="Skip the offer to also update the native GUI Flasher app",
    )
    parser.add_argument(
        "--skip-deps",
        action="store_true",
        help="Skip the Python dependency resync step",
    )
    parser.set_defaults(func=run)


def _run_git(
    root: Path,
    git_args: List[str],
    timeout: float = GIT_LOCAL_TIMEOUT_SEC,
) -> Optional[subprocess.CompletedProcess]:
    """Run `git <git_args>` in `root`.

    Returns None (after printing an error) if git cannot even be invoked
    (not installed) or the call times out. A non-zero return code is NOT
    treated as failure here -- e.g. `git merge --ff-only` failing is an
    expected, caller-handled outcome, not a tool malfunction.
    `root` 内で `git <git_args>` を実行する。gitが起動できない場合
    （未インストール等）やタイムアウト時はエラーを表示してNoneを返す。
    非ゼロの終了コードはここでは失敗扱いしない
    （例えば `git merge --ff-only` の失敗は想定内で呼び出し元が処理する）。
    """
    try:
        return subprocess.run(
            ["git", *git_args],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError:
        console.error("git executable not found. Install git and retry.")
        return None
    except subprocess.TimeoutExpired:
        console.error(f"git {' '.join(git_args)} timed out after {timeout:.0f}s")
        return None


def _git_short_hash(root: Path, ref: str) -> str:
    """Short commit hash for `ref`, or "unknown" if it cannot be read."""
    result = _run_git(root, ["rev-parse", "--short", ref])
    if result is None or result.returncode != 0:
        return "unknown"
    return result.stdout.strip()


def _confirm(prompt: str, default_yes: bool) -> bool:
    """Interactive Y/n confirmation.

    On non-interactive stdin (piped/CI, e.g. EOF immediately) falls back
    to `default_yes` rather than hanging forever.
    対話的なY/n確認。標準入力が非対話（パイプ/CI等でEOF）の場合は
    ハングを避けるため既定値にフォールバックする。
    """
    suffix = "[Y/n]" if default_yes else "[y/N]"
    try:
        answer = input(f"{prompt} {suffix} ").strip().lower()
    except EOFError:
        return default_yes
    if not answer:
        return default_yes
    return answer in ("y", "yes")


def _resolve_repo_root() -> Optional[Path]:
    """Resolve the repository root and confirm it is a git checkout with
    an 'origin' remote (spec C1#1). Returns None (after an error message)
    if either check fails.
    リポジトリルートを解決し、git チェックアウトかつ 'origin' リモートを
    持つことを確認する（仕様C1項目1）。いずれか失敗時はエラーを表示し
    Noneを返す。
    """
    root = paths.root()
    if not (root / ".git").exists():
        console.error(f"Not a git checkout: {root}")
        console.error("`sf upgrade` requires a git clone (a plain zip/tarball "
                       "download cannot be updated this way).")
        return None

    origin_result = _run_git(root, ["remote", "get-url", "origin"])
    if origin_result is None or origin_result.returncode != 0:
        console.error("No 'origin' git remote configured; cannot determine "
                       "where to pull updates from.")
        return None

    return root


def _list_conflicted_files(root: Path) -> List[str]:
    """Tracked files currently in an unresolved-conflict state, per
    `git status --porcelain`'s two-letter status codes.
    `git status --porcelain` の2文字コードから、未解決の衝突状態にある
    追跡ファイルを列挙する。
    """
    status_result = _run_git(root, ["status", "--porcelain"])
    if status_result is None or status_result.returncode != 0:
        return []
    conflicted = []
    for line in status_result.stdout.splitlines():
        if len(line) < 4:
            continue
        code, file_path = line[:2], line[3:]
        if code in CONFLICT_STATUS_CODES:
            conflicted.append(file_path)
    return conflicted


def _report_stash_pop_conflict(root: Path, stash_message: str) -> None:
    """Plain-language guidance for a `git stash pop` conflict (spec
    C1#4): nothing is lost, here is exactly how to find and resolve it.
    `git stash pop` 衝突時の平易な案内（仕様C1項目4）: 何も失われて
    いないこと、見つけ方・直し方を具体的に示す。
    """
    console.print()
    console.warning("Your local changes are safely saved in the stash -- nothing is lost.")
    console.warning(f'  Stash entry: "{stash_message}"')
    console.print()
    conflicted_files = _list_conflicted_files(root)
    console.print("Conflicting file(s):")
    for file_path in conflicted_files or ["(see the git output above for the file list)"]:
        console.print(f"  {file_path}")
    console.print()
    console.print("Conflict markers look like: <<<<<<< / ======= / >>>>>>>")
    console.print("Edit each file to keep the lines you want and remove the markers.")
    console.print()
    console.print("Once every file is resolved:")
    console.print("  git add <file>")
    console.print("  git stash drop")
    console.print()
    console.print(f"Still stuck? See {UPGRADING_DOC_PATH} (section: Resolving conflicts).")


def _ff_merge(root: Path, remote_ref: str) -> int:
    """Fast-forward-only merge to `remote_ref`.

    If that is impossible because local commits have diverged, explains
    the situation and points at the docs -- never rebases or resets on
    its own (spec C1#4).
    `remote_ref` へのff-onlyマージ。ローカルコミットで分岐していて
    不可能な場合は状況を説明しドキュメントへ誘導する。勝手に
    rebase/resetは行わない（仕様C1項目4）。
    """
    merge_result = _run_git(root, ["merge", "--ff-only", remote_ref])
    if merge_result is not None and merge_result.returncode == 0:
        return EXIT_OK

    console.error(f"Cannot fast-forward to {remote_ref}: your branch has local "
                   "commits that have diverged from it.")
    console.error("`sf upgrade` never rebases or resets automatically, to avoid "
                   "losing work.")
    console.print()
    console.print("To resolve manually, see:")
    console.print(f"  {UPGRADING_DOC_PATH}")
    return EXIT_NEEDS_ATTENTION


def _apply_local_changes_and_merge(
    root: Path,
    remote_ref: str,
    args: argparse.Namespace,
    actions_taken: List[str],
) -> int:
    """Handle local (tracked-file) changes, then fast-forward merge
    (spec C1#4). Returns EXIT_OK on success, or a non-zero exit code the
    caller should return immediately (the reason has already been
    printed).
    ローカル（追跡ファイル）の変更を処理してからff-onlyマージする
    （仕様C1項目4）。成功時はEXIT_OK、失敗時は理由をすでに表示済みの
    非ゼロ終了コードを返す。
    """
    status_result = _run_git(root, ["status", "--porcelain"])
    if status_result is None:
        return EXIT_GENERAL_ERROR

    # "?? " marks untracked files; spec C1#4 explicitly scopes local-change
    # handling to tracked files only (untracked files never conflict with
    # a fast-forward merge, so they are simply ignored).
    # "?? " はuntracked。仕様C1項目4はローカル変更の扱いを追跡ファイルに
    # 限定している（untrackedはff mergeと衝突しないため無視してよい）。
    tracked_changes = [
        line for line in status_result.stdout.splitlines()
        if line and not line.startswith("??")
    ]

    if not tracked_changes:
        exit_code = _ff_merge(root, remote_ref)
        if exit_code == EXIT_OK:
            actions_taken.append(f"Fast-forwarded to {remote_ref}")
        return exit_code

    changed_files = [line[3:] for line in tracked_changes]

    if args.discard_local:
        console.print()
        console.warning("The following tracked files have local changes that "
                         "will be DISCARDED:")
        for file_path in changed_files:
            console.print(f"  {file_path}")
        console.print()
        # Destructive: confirmed even with --yes (spec C1#4).
        # 破壊的操作のため --yes でも確認必須（仕様C1項目4）
        if not _confirm(
            "Discard these local changes? This cannot be undone.",
            default_yes=False,
        ):
            console.print("Aborted; no changes made.")
            return EXIT_OK

        checkout_result = _run_git(root, ["checkout", "--", *changed_files])
        if checkout_result is None or checkout_result.returncode != 0:
            console.error("Failed to discard local changes.")
            return EXIT_GENERAL_ERROR
        actions_taken.append(f"Discarded local changes to {len(changed_files)} file(s)")

        exit_code = _ff_merge(root, remote_ref)
        if exit_code == EXIT_OK:
            actions_taken.append(f"Fast-forwarded to {remote_ref}")
        return exit_code

    # Default: stash local changes, ff-merge, then restore them.
    # 既定: ローカル変更をstash → ff merge → 復元
    stash_message = (
        f"sf-upgrade-autostash {datetime.datetime.now().isoformat(timespec='seconds')}"
    )
    stash_result = _run_git(root, ["stash", "push", "-m", stash_message])
    if stash_result is None or stash_result.returncode != 0:
        console.error("Failed to stash local changes; aborting to avoid data loss.")
        return EXIT_GENERAL_ERROR
    actions_taken.append(
        f"Stashed local changes to {len(changed_files)} file(s): \"{stash_message}\""
    )

    exit_code = _ff_merge(root, remote_ref)
    if exit_code != EXIT_OK:
        # The stash still holds the user's edits -- leave it in place so
        # nothing is lost, and tell them where to find it.
        # stashにはユーザーの変更がまだ残っている -- 何も失わないよう
        # そのままにし、見つけ方を伝える。
        console.warning(f'Your local changes remain safely stashed: "{stash_message}"')
        console.warning("Run 'git stash pop' once the situation above is resolved.")
        return exit_code
    actions_taken.append(f"Fast-forwarded to {remote_ref}")

    pop_result = _run_git(root, ["stash", "pop"])
    if pop_result is None:
        return EXIT_GENERAL_ERROR
    if pop_result.returncode != 0:
        _report_stash_pop_conflict(root, stash_message)
        return EXIT_NEEDS_ATTENTION

    actions_taken.append("Restored your local changes (git stash pop)")
    return EXIT_OK


def _run_pip_install(pip_args: List[str], root: Path) -> bool:
    """Run `python -m pip install --quiet <pip_args>`.

    Honors SKIP_PIP_ENV_VAR for tests: prints what it would have run
    instead of invoking pip (see module docstring / spec test item (d)).
    `python -m pip install --quiet <pip_args>` を実行する。テスト用に
    SKIP_PIP_ENV_VAR を尊重し、設定時は実行の代わりに実行予定の内容を
    表示する（モジュールdocstring／仕様テスト項目(d)を参照）。
    """
    cmd = [sys.executable, "-m", "pip", "install", "--quiet", *pip_args]

    if os.environ.get(SKIP_PIP_ENV_VAR):
        console.print(
            f"  [stub, {SKIP_PIP_ENV_VAR} set] would run: {' '.join(cmd)}"
        )
        return True

    try:
        result = subprocess.run(cmd, cwd=str(root), timeout=PIP_SYNC_TIMEOUT_SEC)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        console.error(f"pip install timed out after {PIP_SYNC_TIMEOUT_SEC:.0f}s")
        return False
    except Exception as e:
        console.error(f"Failed to run pip: {e}")
        return False


def _sync_dependencies(root: Path) -> bool:
    """Reinstall sfcli + its dependencies so the environment matches the
    pulled code (spec C1#5: always runs; idempotent, so a no-op pull
    costs only a couple of seconds).

    Mirrors scripts/installer.py's Step 3 (non-minimal path): install
    requirements.txt (the full dependency set, including simulator
    extras) if present, then `pip install -e .` for the package itself
    -- the same target set, so `sf upgrade` never leaves the environment
    less complete than a fresh `./install.sh` would.
    プルの有無に関わらず常時実行する依存同期（仕様C1項目5、冪等で
    数秒しかかからない）。scripts/installer.py のStep3（非minimal経路）
    と同じ対象を再インストールする: requirements.txt（存在すれば
    シミュレータ含む全依存）→ パッケージ自体の `pip install -e .`。
    これにより `sf upgrade` が `./install.sh` 新規実行より環境を
    不完全にすることはない。
    """
    console.info("Syncing Python dependencies...")

    ok = True
    requirements_file = root / "requirements.txt"
    if requirements_file.exists():
        ok = _run_pip_install(["-r", str(requirements_file)], root) and ok
    ok = _run_pip_install(["-e", str(root)], root) and ok

    if ok:
        console.success("Dependencies are up to date.")
    else:
        console.error("Dependency sync failed. Try manually:")
        if requirements_file.exists():
            console.error(f"  {sys.executable} -m pip install -r {requirements_file}")
        console.error(f"  {sys.executable} -m pip install -e {root}")
    return ok


def _backup_stale_sdkconfigs(root: Path, old_head: str, new_head: str) -> List[str]:
    """For each firmware/<target> whose sdkconfig.defaults or
    partitions.csv changed in the pulled range, rename any existing
    firmware/<target>/sdkconfig out of the way (spec C1#6) so it
    regenerates from the new defaults on the next build. ESP-IDF does not
    auto-reapply changed defaults onto an existing sdkconfig.

    Returns the list of target names that were backed up.
    プルした範囲でsdkconfig.defaultsまたはpartitions.csvが変化した
    firmware/<target>について、既存のsdkconfigを退避する（仕様C1項目6、
    次回ビルドで新しい既定から再生成されるように）。ESP-IDFは変更された
    defaultsを既存のsdkconfigへ自動反映しない。退避したターゲット名の
    一覧を返す。
    """
    diff_result = _run_git(root, ["diff", "--name-only", f"{old_head}..{new_head}"])
    if diff_result is None or diff_result.returncode != 0:
        return []

    stale_targets = set()
    for changed_path in diff_result.stdout.splitlines():
        parts = Path(changed_path).parts
        # firmware/<target>/(sdkconfig.defaults|partitions.csv)
        if len(parts) >= 3 and parts[0] == "firmware" and parts[-1] in SDKCONFIG_TRIGGER_FILENAMES:
            stale_targets.add(parts[1])

    backed_up_targets = []
    for target in sorted(stale_targets):
        sdkconfig_path = root / "firmware" / target / "sdkconfig"
        if not sdkconfig_path.exists():
            continue
        backup_path = sdkconfig_path.with_name(sdkconfig_path.name + SDKCONFIG_BACKUP_SUFFIX)
        sdkconfig_path.rename(backup_path)
        console.warning(
            f"  firmware/{target}: sdkconfig.defaults changed upstream; existing "
            f"sdkconfig backed up to {backup_path.name} "
            "(next build regenerates it from the new defaults)"
        )
        backed_up_targets.append(target)

    return backed_up_targets


def _offer_flasher_update(args: argparse.Namespace, actions_taken: List[str]) -> None:
    """Native GUI Flasher app: offer to update it if installed, or offer
    a one-time install if it is not (spec C1#7).

    Two branches:
      - Installed: ask to update it (auto-accepted with --yes), same as
        before this one-time-offer feature was added.
      - Not installed: ask ONCE per checkout whether to install it. Never
        asked with --yes (an unattended `sf upgrade` should not pull down
        a whole new desktop app) or --no-flasher. A marker file records
        the answer (accepted or declined) so the offer is never repeated
        on subsequent `sf upgrade` runs -- declining just prints a
        pointer to `sf flasher install` for later.

    ネイティブGUIフラッシャ: 導入済みなら更新を提案し、未導入なら一回限り
    のインストール提案を行う（仕様C1項目7）。

    2つの分岐:
      - 導入済み: 更新するか確認する（--yesなら自動承諾）。この一回限り
        提案機能を追加する前と同じ挙動。
      - 未導入: このチェックアウトで一回だけインストールするか尋ねる。
        --yes（無人実行の`sf upgrade`が新しいデスクトップアプリを勝手に
        入れるべきではないため）または --no-flasher では絶対に尋ねない。
        マーカーファイルに回答（承諾/辞退）を記録し、以後の`sf upgrade`
        では二度と提案しない -- 辞退した場合は後で使える
        `sf flasher install` への案内だけを表示する。
    """
    if args.no_flasher:
        return

    installed_executable = flasher_install.installed_app_executable()
    if installed_executable is not None:
        # Already installed: existing update-offer behavior, unchanged.
        # 導入済み: 既存の更新提案の挙動（変更なし）。
        console.print()
        if not (args.yes or _confirm("Update the native GUI Flasher too?", default_yes=True)):
            return

        flasher_exit_code = flasher_install.install(assume_yes=True)
        if flasher_exit_code == 0:
            actions_taken.append("Updated the native GUI Flasher app")
        else:
            actions_taken.append("GUI Flasher update FAILED -- see the output above")
        return

    # Not installed: one-time interactive install offer only. --yes never
    # triggers it (see docstring) -- an unattended upgrade must not
    # install a new app the user never asked for.
    # 未導入: 一回限りの対話的インストール提案のみ。--yesでは絶対に発生
    # しない（docstring参照）-- 無人実行のupgradeがユーザーが求めていない
    # 新規アプリを勝手に入れてはならない。
    if args.yes:
        return

    marker_path = paths.config_dir() / FLASHER_OFFER_MARKER_FILENAME
    if marker_path.exists():
        # Already offered once (accepted or declined) -- stay silent.
        # 既に一度提案済み（承諾/辞退いずれか）-- 何も表示しない。
        return

    console.print()
    accepted = _confirm(
        "Install the StampFly Flasher desktop app? (one-time offer)",
        default_yes=False,
    )

    # Write the marker regardless of the answer -- this is a ONE-TIME
    # offer, so "declined" must suppress future prompts just as much as
    # "accepted" does.
    # 回答に関わらずマーカーを書き込む -- 一回限りの提案なので、「辞退」も
    # 「承諾」と同様に以後のプロンプトを抑止しなければならない。
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().isoformat(timespec="seconds")
    marker_path.write_text(
        f"{timestamp} {'accepted' if accepted else 'declined'}\n", encoding="utf-8"
    )

    if not accepted:
        console.print("You can install it anytime with: sf flasher install")
        return

    flasher_exit_code = flasher_install.install(assume_yes=True)
    if flasher_exit_code == 0:
        actions_taken.append("Installed the native GUI Flasher app")
    else:
        actions_taken.append("GUI Flasher install FAILED -- see the output above")


def _print_summary(old_head: str, new_head: str, actions_taken: List[str]) -> None:
    """Summary: before -> after hash, actions taken, recommended next
    step (spec C1#8).
    サマリ表示: 更新前→後のハッシュ、実行した処置、推奨次アクション
    （仕様C1項目8）。
    """
    console.print()
    console.header("Summary")
    console.print(f"  {old_head} -> {new_head}")
    console.print()

    if actions_taken:
        console.print("Actions taken:")
        for action in actions_taken:
            console.print(f"  - {action}")
    else:
        console.print("No local actions were necessary.")

    console.print()
    console.print("Recommended next step:")
    console.print("  sf build vehicle   # (or: sf build controller)")

    if any("sdkconfig" in action.lower() for action in actions_taken):
        console.print()
        console.warning(
            "A firmware target's sdkconfig was backed up above -- the next "
            "build regenerates it from the new defaults. Any customizations "
            "you had are preserved in the *.pre-upgrade-backup file if you "
            "need to reapply them."
        )


def run(args: argparse.Namespace) -> int:
    """Execute `sf upgrade` / `sf upgrade` を実行する"""
    console.header("StampFly Ecosystem Upgrade")
    console.print()

    # --- Step 1: repository sanity check ---------------------------------
    root = _resolve_repo_root()
    if root is None:
        return EXIT_GENERAL_ERROR

    branch_result = _run_git(root, ["rev-parse", "--abbrev-ref", "HEAD"])
    if branch_result is None or branch_result.returncode != 0:
        console.error("Failed to determine the current branch.")
        return EXIT_GENERAL_ERROR
    branch = branch_result.stdout.strip()
    console.info(f"Repository: {root}")
    console.info(f"Current branch: {branch}")
    if branch != MAIN_BRANCH_NAME:
        console.warning(
            f"You are on '{branch}', not '{MAIN_BRANCH_NAME}'. Continuing "
            "(expected if you are a developer working on a feature branch)."
        )

    old_head = _git_short_hash(root, "HEAD")

    # --- Step 2: fetch + how far behind are we? ---------------------------
    console.print()
    console.info(f"Fetching origin/{branch}...")
    fetch_result = _run_git(root, ["fetch", "origin", branch], timeout=GIT_NETWORK_TIMEOUT_SEC)
    if fetch_result is None or fetch_result.returncode != 0:
        console.error(f"git fetch origin {branch} failed:")
        if fetch_result is not None:
            console.error(f"  {fetch_result.stderr.strip()}")
        return EXIT_GENERAL_ERROR

    remote_ref = f"origin/{branch}"
    behind_result = _run_git(root, ["rev-list", "--count", f"HEAD..{remote_ref}"])
    if behind_result is None or behind_result.returncode != 0:
        console.error(f"Failed to compare HEAD with {remote_ref}. Does that branch "
                       "exist on origin?")
        return EXIT_GENERAL_ERROR
    behind_count = int(behind_result.stdout.strip() or "0")

    if behind_count == 0:
        console.success("Already up to date.")
        if args.skip_deps:
            console.print("(--skip-deps: dependency resync skipped)")
            return EXIT_OK
        console.print()
        return EXIT_OK if _sync_dependencies(root) else EXIT_GENERAL_ERROR

    console.info(f"{behind_count} new commit(s) available on {remote_ref}.")

    # --- Step 3: preview + confirm ----------------------------------------
    console.print()
    console.print("Changes to be pulled:")
    log_result = _run_git(
        root,
        ["log", "--oneline", f"HEAD..{remote_ref}", "-n", str(PREVIEW_LOG_LINE_LIMIT)],
    )
    if log_result is not None and log_result.returncode == 0:
        for line in log_result.stdout.rstrip("\n").splitlines():
            console.print(f"  {line}")
    console.print()

    if not args.yes and not _confirm("Proceed with upgrade?", default_yes=True):
        console.print("Aborted.")
        return EXIT_OK

    # --- Step 4: local change handling + fast-forward merge ---------------
    actions_taken: List[str] = []
    merge_exit_code = _apply_local_changes_and_merge(root, remote_ref, args, actions_taken)
    if merge_exit_code != EXIT_OK:
        return merge_exit_code

    # --- Step 5: dependency resync (always, unless --skip-deps) ----------
    # A sync failure does not abort the remaining steps (they are
    # independent and the manual fix was already printed), but it MUST be
    # reflected in the exit code -- scripts and the installer rely on it.
    # 依存同期の失敗で残りの手順は中断しない（互いに独立で、手動での
    # 修復コマンドは表示済み）が、終了コードには必ず反映する --
    # スクリプトやインストーラが終了コードを見るため。
    deps_ok = True
    console.print()
    if args.skip_deps:
        console.print("(--skip-deps: dependency resync skipped)")
    elif _sync_dependencies(root):
        actions_taken.append("Resynced Python dependencies (pip install -e .)")
    else:
        deps_ok = False
        actions_taken.append("Dependency resync FAILED -- see the output above")

    # --- Step 6: sdkconfig staleness check ---------------------------------
    console.print()
    new_head = _git_short_hash(root, "HEAD")
    backed_up_targets = _backup_stale_sdkconfigs(root, old_head, new_head)
    if backed_up_targets:
        actions_taken.append("Backed up stale sdkconfig for: " + ", ".join(backed_up_targets))

    # --- Step 7: native GUI Flasher ----------------------------------------
    _offer_flasher_update(args, actions_taken)

    # --- Step 8: summary -----------------------------------------------------
    _print_summary(old_head, new_head, actions_taken)
    return EXIT_OK if deps_ok else EXIT_GENERAL_ERROR
