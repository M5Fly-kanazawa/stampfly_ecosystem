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
    # See _warn_if_commands_unavailable()'s docstring for why this runs
    # before parser creation / argument parsing.
    # このタイミングで実行する理由は _warn_if_commands_unavailable() の
    # docstring を参照。
    _warn_if_commands_unavailable()

    parser = create_parser()

    # Parse arguments
    args = parser.parse_args(argv)

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
