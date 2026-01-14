"""
StampFly CLI - Main entry point

sf <command> [options]

Unified command-line interface for StampFly drone development.
StampFlyドローン開発のための統合コマンドラインインターフェース。
"""

import argparse
import sys
from typing import Optional, List

from . import __version__
from .commands import version, doctor, build, flash, monitor
from .utils import console


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

    # Register all commands
    version.register(subparsers)
    doctor.register(subparsers)
    build.register(subparsers)
    flash.register(subparsers)
    monitor.register(subparsers)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point

    Args:
        argv: Command line arguments (uses sys.argv if None)

    Returns:
        Exit code (0 = success, non-zero = error)
    """
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
