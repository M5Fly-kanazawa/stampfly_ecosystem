"""
sf flasher - Manage the native StampFly Flasher GUI app installation

Install, update, remove, or check the status of the standalone StampFly
Flasher GUI (tools/flasher_gui/) as a native desktop app: after
installing, it launches without a build environment or a Python
interpreter (`sf flash --gui` prefers the installed app automatically,
see flash.py).

スタンドアロン GUI「StampFly Flasher」(tools/flasher_gui/) をネイティブ
デスクトップアプリとしてインストール・更新・削除・状態確認します。
インストール後はビルド環境も Python インタプリタも不要で起動できます
（`sf flash --gui` はインストール済みアプリを自動的に優先します。
flash.py 参照）。

Subcommands:
    install    - Install (or reinstall) the GUI as a native app
    uninstall  - Remove the installed native app
    status     - Show whether the GUI is installed and its version
    update     - Alias for install: re-fetch and install the latest release
"""

import argparse
from pathlib import Path

from ..utils import console, flasher_install

COMMAND_NAME = "flasher"
COMMAND_HELP = "Manage the native StampFly Flasher GUI app"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers
    flasher_subparsers = parser.add_subparsers(
        dest="flasher_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- install ---
    install_parser = flasher_subparsers.add_parser(
        "install",
        help="Install (or reinstall) the GUI as a native app",
        description=(
            "Download the latest StampFly Flasher release (or use a local build "
            "artifact with --from-file) and install it as a native desktop app."
        ),
    )
    install_parser.add_argument(
        "--from-file",
        metavar="PATH",
        default=None,
        help=(
            "Install from a local build artifact instead of fetching the latest "
            "GitHub release (.exe / .zip / .app directory / bare executable, "
            "interpreted per-OS)"
        ),
    )
    install_parser.add_argument(
        "--no-desktop-shortcut",
        action="store_true",
        help="Skip creating a desktop shortcut (Windows only; no effect on macOS/Linux)",
    )
    install_parser.add_argument(
        "--yes", "-y",
        action="store_true",
        help="Do not prompt for confirmation (for CI / scripts/installer.py)",
    )
    install_parser.set_defaults(func=run_install)

    # --- uninstall ---
    uninstall_parser = flasher_subparsers.add_parser(
        "uninstall",
        help="Remove the installed native app",
        description="Remove the StampFly Flasher native app and its shortcuts/registry entries.",
    )
    uninstall_parser.add_argument(
        "--yes", "-y",
        action="store_true",
        help="Do not prompt for confirmation",
    )
    uninstall_parser.set_defaults(func=run_uninstall)

    # --- status ---
    status_parser = flasher_subparsers.add_parser(
        "status",
        help="Show whether the GUI is installed and its version",
        description="Show installation status: installed version, executable path, install date.",
    )
    status_parser.set_defaults(func=run_status)

    # --- update ---
    update_parser = flasher_subparsers.add_parser(
        "update",
        help="Re-fetch and install the latest release (alias for install)",
        description=(
            "Alias for 'sf flasher install' without --from-file: fetches the "
            "latest GitHub release and reinstalls it."
        ),
    )
    update_parser.add_argument(
        "--no-desktop-shortcut",
        action="store_true",
        help="Skip creating a desktop shortcut (Windows only; no effect on macOS/Linux)",
    )
    update_parser.add_argument(
        "--yes", "-y",
        action="store_true",
        help="Do not prompt for confirmation",
    )
    update_parser.set_defaults(func=run_update)

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf flasher <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  install    Install (or reinstall) the GUI as a native app")
    console.print("  uninstall  Remove the installed native app")
    console.print("  status     Show whether the GUI is installed and its version")
    console.print("  update     Re-fetch and install the latest release (alias for install)")
    console.print()
    console.print("Examples:")
    console.print("  sf flasher install                       # Install the latest release")
    console.print("  sf flasher install --from-file dist/StampFlyFlasher.app")
    console.print("  sf flasher status                        # Check install status")
    console.print("  sf flasher uninstall --yes                # Remove, no prompt")
    console.print()
    console.print("Run 'sf flasher <subcommand> --help' for details.")
    return 0


def run_install(args: argparse.Namespace) -> int:
    """Install (or reinstall) the native app"""
    from_file = Path(args.from_file) if args.from_file else None
    return flasher_install.install(
        from_file=from_file,
        desktop_shortcut=not args.no_desktop_shortcut,
        assume_yes=args.yes,
    )


def run_uninstall(args: argparse.Namespace) -> int:
    """Remove the installed native app"""
    return flasher_install.uninstall(assume_yes=args.yes)


def run_status(args: argparse.Namespace) -> int:
    """Show install status"""
    return flasher_install.status()


def run_update(args: argparse.Namespace) -> int:
    """Alias for install: always re-fetches the latest GitHub release"""
    return flasher_install.install(
        from_file=None,
        desktop_shortcut=not args.no_desktop_shortcut,
        assume_yes=args.yes,
    )
