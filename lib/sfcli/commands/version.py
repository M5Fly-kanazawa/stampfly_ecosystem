"""
sf version - Show version information

Displays StampFly CLI version and environment information.
"""

import argparse
from .. import __version__
from ..utils import console, paths, platform

COMMAND_NAME = "version"
COMMAND_HELP = "Show version information"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed environment information",
    )
    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute version command"""
    console.print(f"StampFly CLI version {__version__}")

    if args.verbose:
        console.print()
        console.header("Environment")

        # Python
        console.print(f"Python: {platform.python_version()}")
        console.print(f"Platform: {_get_platform_name()}")

        # ESP-IDF
        idf_path = platform.esp_idf_path()
        idf_version = platform.esp_idf_version()
        if idf_path:
            console.print(f"ESP-IDF: {idf_version or 'unknown'} ({idf_path})")
        else:
            console.print("ESP-IDF: not found")

        # Paths
        console.print()
        console.header("Paths")
        console.print(f"Root: {paths.root()}")
        console.print(f"Firmware: {paths.firmware()}")
        console.print(f"Simulator: {paths.simulator()}")

        # Serial ports
        ports = platform.serial_ports()
        if ports:
            console.print()
            console.header("Serial Ports")
            for port in ports:
                console.print(f"  {port}")

    return 0


def _get_platform_name() -> str:
    """Get human-readable platform name"""
    if platform.is_macos():
        return "macOS"
    elif platform.is_windows():
        return "Windows"
    elif platform.is_wsl():
        return "WSL (Windows Subsystem for Linux)"
    elif platform.is_linux():
        return "Linux"
    else:
        return "Unknown"
