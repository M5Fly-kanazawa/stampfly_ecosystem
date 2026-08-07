"""
StampFly CLI - Main entry point

sf <command> [options]

Unified command-line interface for StampFly drone development.
StampFlyドローン開発のための統合コマンドラインインターフェース。

Dependency resilience / 依存欠落耐性:
Command modules are loaded via sfcli.commands (see that package's
docstring), which imports each one in its own try/except. A missing
third-party dependency therefore disables only the one command that
needs it -- `sf` itself, and every other command, keep working
(V3 finding: a single missing package used to kill `sf` entirely).
assert_all_commands_loadable() below exposes the same information as a
hard failure for scripts/installer.py's post-install probe, which needs
to tell "installed but broken" apart from "actually installed" (the
C2/C3 contract).
コマンドモジュールは sfcli.commands 経由でロードされる（同パッケージの
docstring参照）。同パッケージは各モジュールを個別のtry/exceptで
importするため、サードパーティ依存の欠落は本当にそれを必要とする
1コマンドだけを無効化し、`sf` 自体や他の全コマンドは動き続ける
（V3の実測: 以前は1パッケージの欠落が `sf` 全体を殺していた）。
下記の assert_all_commands_loadable() は同じ情報を
scripts/installer.py のインストール後プローブ向けにハード失敗として
公開する。プローブは「インストール済みだが壊れている」と「実際に
インストール済み」を区別する必要がある（C2/C3の契約）。
"""

import argparse
import sys
from typing import Optional, List

from . import __version__
from . import commands
from .utils import console


def assert_all_commands_loadable() -> None:
    """Raise if any command module failed to import.

    Used by scripts/installer.py's post-install probe as:
        python -c "import sfcli.cli; sfcli.cli.assert_all_commands_loadable()"
    which must exit non-zero when a dependency is missing, so the
    installer can tell a partially-broken install apart from a genuinely
    working one and (re)run Step 3 instead of trusting it (C2/C3
    contract; see commands/__init__.py's FAILED_COMMANDS).

    scripts/installer.py のインストール後プローブが上記コマンドの形で
    使用する。依存欠落時は非ゼロ終了しなければならず、これにより
    インストーラは「中途半端に壊れたインストール」を「正常に動く
    インストール」と区別してStep3を再実行できる（C2/C3の契約。
    commands/__init__.py の FAILED_COMMANDS を参照）。
    """
    if commands.FAILED_COMMANDS:
        details = ", ".join(f"{name} (missing {dep})" for name, dep in commands.FAILED_COMMANDS)
        raise ImportError(f"{len(commands.FAILED_COMMANDS)} command(s) failed to import: {details}")


def _warn_if_commands_unavailable() -> None:
    """Print a one-line warning listing any commands that could not be
    loaded due to a missing dependency, and how to fix it.

    Printed at CLI startup, before argument parsing, rather than strictly
    "after parsing" -- argparse's built-in `-h`/`--version` actions exit
    from inside parse_args() itself, and the warning must still be
    visible for `sf --help` / `sf -V` (this is what a user with a broken
    env is most likely to run first).
    欠落依存のため読み込めなかったコマンドと直し方を1行で警告する。
    CLI起動時、引数パース前に表示する（「パース後」ではない） --
    argparse標準の `-h`/`--version` は parse_args() 内部でexitするため、
    `sf --help` / `sf -V` でも警告が見えるようにする必要がある
    （環境が壊れているユーザーがまず最初に試す操作である可能性が高い）。
    """
    if not commands.FAILED_COMMANDS:
        return
    command_names = ", ".join(name for name, _ in commands.FAILED_COMMANDS)
    missing_deps = ", ".join(sorted({dep for _, dep in commands.FAILED_COMMANDS}))
    console.warning(
        f"{len(commands.FAILED_COMMANDS)} command(s) unavailable ({command_names}); "
        f"missing: {missing_deps}. Fix with 'sf upgrade' or "
        f"'pip install -e <repo-root>'."
    )


def create_parser() -> argparse.ArgumentParser:
    """Create the main argument parser"""
    parser = argparse.ArgumentParser(
        prog="sf",
        description="StampFly Ecosystem CLI - Drone development tools",
        epilog="Run 'sf <command> --help' for more information on a command.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "-V", "--version",
        action="version",
        version=f"sf {__version__}",
    )

    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable colored output",
    )

    # Create subparsers for commands
    subparsers = parser.add_subparsers(
        dest="command",
        title="commands",
        metavar="<command>",
    )

    # Register every command that imported successfully, in
    # commands.__all__'s display order. Commands whose dependency is
    # missing (commands.FAILED_COMMANDS) are simply absent from
    # __all__ and thus skipped here -- see _warn_if_commands_unavailable().
    # importに成功したコマンドのみ、commands.__all__の表示順で登録する。
    # 依存欠落コマンド(commands.FAILED_COMMANDS)は__all__に含まれない
    # ためここでは単純にスキップされる（_warn_if_commands_unavailable()参照）。
    for command_name in commands.__all__:
        getattr(commands, command_name).register(subparsers)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point

    Args:
        argv: Command line arguments (uses sys.argv if None)

    Returns:
        Exit code (0 = success, non-zero = error)
    """
    # Console-encoding hardening: never let an unencodable character kill a
    # command. Windows consoles decode/encode with a code page (cp932 on
    # Japanese Windows, cp1252 on English) that cannot represent every
    # character UTF-8 content may carry -- printing a git commit title with
    # an em-dash / Japanese raised UnicodeEncodeError and aborted
    # `sf upgrade` (observed 2026-07-19: cp932 on a DXH loaner PC, then
    # cp1252 on the windows-latest CI leg via the regression fixture).
    # errors="replace" renders such characters as '?' instead of crashing —
    # the same remedy the v2026.07.1 flasher adopted for its CLI paths.
    # コンソールエンコーディングの防御: エンコードできない文字でコマンドを
    # 死なせない。Windows のコンソールはコードページ（日本語 Windows は
    # cp932、英語は cp1252）で入出力し、UTF-8 コンテンツの全文字は表現でき
    # ない -- 全角ダッシュ/日本語を含む git コミット題名の表示が
    # UnicodeEncodeError を起こし `sf upgrade` を中断させた（2026-07-19 に
    # 実測: DXH 貸出PCの cp932、続いて退行フィクスチャ経由で windows-latest
    # CI レッグの cp1252）。errors="replace" はそうした文字を '?' として
    # 表示し、クラッシュさせない — v2026.07.1 のフラッシャが CLI 経路に
    # 採用したのと同じ対処。
    for _stream in (sys.stdout, sys.stderr):
        try:
            _stream.reconfigure(errors="replace")
        except (AttributeError, ValueError, OSError):
            # Non-reconfigurable stream (exotic redirection): leave as-is.
            # 再構成できないストリーム（特殊なリダイレクト）: そのまま。
            pass

    # See _warn_if_commands_unavailable()'s docstring for why this runs
    # before parser creation / argument parsing.
    # このタイミングで実行する理由は _warn_if_commands_unavailable() の
    # docstring を参照。
    _warn_if_commands_unavailable()

    # Legacy environment-variable names: command modules that renamed
    # their environment variables expose promote_legacy_environment(),
    # which maps old names onto new ones (currently only sils: SIL_* ->
    # SILS_*, renamed 2026-08-07). Run before dispatch so subprocesses
    # inherit the promoted names.
    # 旧環境変数名: 環境変数を改名したコマンドモジュールは
    # promote_legacy_environment() を公開し、旧名を新名へ写像する
    # （現在は sils のみ: SIL_* -> SILS_*、2026-08-07 改名）。
    # サブプロセスが昇格済みの名前を継承できるよう、ディスパッチ前に実行。
    for _name in commands.__all__:
        _promote = getattr(getattr(commands, _name), "promote_legacy_environment", None)
        if _promote is not None:
            _promote()

    parser = create_parser()

    # Parse arguments
    args = parser.parse_args(argv)

    # Legacy command names: a renamed command keeps its old name as an
    # argparse alias for a transition period (see the module's
    # LEGACY_COMMAND_ALIASES, e.g. "sil" -> "sils"). Tell the user the
    # canonical name once per invocation.
    # 旧コマンド名: 改名されたコマンドは移行期間中、旧名を argparse の
    # エイリアスとして維持する（各モジュールの LEGACY_COMMAND_ALIASES
    # 参照。例: "sil" -> "sils"）。実行のたびに正式名を1行で案内する。
    if args.command:
        for _name in commands.__all__:
            _module = getattr(commands, _name)
            if args.command in getattr(_module, "LEGACY_COMMAND_ALIASES", ()):
                console.warning(
                    f"'sf {args.command}' has been renamed to "
                    f"'sf {_module.COMMAND_NAME}'; the old name still works "
                    f"for now, but please switch (renamed 2026-08-07)."
                )
                break

    # Handle no-color option
    if args.no_color:
        console.set_color(False)

    # Show help if no command specified
    if not args.command:
        parser.print_help()
        return 0

    # Execute command
    if hasattr(args, "func"):
        try:
            return args.func(args)
        except KeyboardInterrupt:
            console.print("\nInterrupted")
            return 130
        except Exception as e:
            console.error(f"Unexpected error: {e}")
            return 1
    else:
        parser.print_help()
        return 0


if __name__ == "__main__":
    sys.exit(main())
