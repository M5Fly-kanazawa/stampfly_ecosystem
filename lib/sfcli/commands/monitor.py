"""
sf monitor - Serial monitor

Opens serial monitor to connected device.
接続されたデバイスのシリアルモニタを開きます。
"""

import argparse
import subprocess
import os
from pathlib import Path
from ..utils import console, paths, platform

COMMAND_NAME = "monitor"
COMMAND_HELP = "Open serial monitor"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    parser.add_argument(
        "target",
        nargs="?",
        default="vehicle",
        choices=["vehicle", "controller"],
        help="Target project (for ELF file, default: vehicle)",
    )
    parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (auto-detect if not specified)",
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute monitor command"""
    # Determine target directory
    if args.target == "vehicle":
        target_dir = paths.vehicle()
    else:
        target_dir = paths.controller()

    if not target_dir.exists():
        console.error(f"Target directory not found: {target_dir}")
        return 1

    # Check ESP-IDF
    idf_path = platform.esp_idf_path()
    if not idf_path:
        console.error("ESP-IDF not found. Please install ESP-IDF first.")
        return 1

    # Detect or use specified port
    port = args.port
    if not port:
        port = platform.default_serial_port()
        if not port:
            console.error("No serial port detected. Please specify with -p/--port")
            available = platform.serial_ports()
            if available:
                console.print("Available ports:")
                for p in available:
                    console.print(f"  {p}")
            return 1

    console.info(f"Opening monitor for {args.target}...")
    console.print(f"  Port: {port}")
    console.print(f"  Baud: {args.baud}")
    console.print("  Press Ctrl+] to exit")
    console.print()

    # Prepare environment
    env = _prepare_idf_env(idf_path)

    # Monitor command
    cmd = ["idf.py", "-p", port, "-b", str(args.baud), "monitor"]

    # Execute monitor
    result = subprocess.run(cmd, cwd=target_dir, env=env)

    return result.returncode


def _prepare_idf_env(idf_path: Path) -> dict:
    """Prepare environment with ESP-IDF settings"""
    env = os.environ.copy()
    env["IDF_PATH"] = str(idf_path)

    if not platform.is_windows():
        export_script = idf_path / "export.sh"
        if export_script.exists():
            try:
                result = subprocess.run(
                    f'source "{export_script}" > /dev/null 2>&1 && env',
                    shell=True,
                    capture_output=True,
                    text=True,
                    executable="/bin/bash",
                )
                if result.returncode == 0:
                    for line in result.stdout.split("\n"):
                        if "=" in line:
                            key, _, value = line.partition("=")
                            env[key.strip()] = value.strip()
            except Exception:
                pass

    return env
