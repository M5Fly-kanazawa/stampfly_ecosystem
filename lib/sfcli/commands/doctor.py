"""
sf doctor - Diagnose environment issues

Checks the development environment for common issues.
開発環境の問題を診断します。
"""

import argparse
import sys
from pathlib import Path
from ..utils import console, paths, platform

COMMAND_NAME = "doctor"
COMMAND_HELP = "Diagnose environment issues"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Attempt to fix issues automatically",
    )
    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute doctor command"""
    console.header("StampFly Environment Diagnostics")
    console.print()

    issues = []
    warnings = []

    # Check Python version
    console.info("Checking Python...")
    py_version = sys.version_info
    if py_version < (3, 10):
        issues.append(f"Python 3.10+ required, found {py_version.major}.{py_version.minor}")
        console.error(f"  Python {py_version.major}.{py_version.minor} - 3.10+ required")
    else:
        console.success(f"  Python {py_version.major}.{py_version.minor}.{py_version.micro}")

    # Check repository structure
    console.print()
    console.info("Checking repository structure...")
    root = paths.root()

    required_dirs = [
        ("firmware", "Firmware source"),
        ("firmware/vehicle", "Vehicle firmware"),
        ("simulator", "Simulator"),
        ("docs", "Documentation"),
    ]

    for dir_name, description in required_dirs:
        dir_path = root / dir_name
        if dir_path.exists():
            console.success(f"  {description}: {dir_path}")
        else:
            issues.append(f"Missing directory: {dir_name}")
            console.error(f"  {description}: NOT FOUND")

    # Check ESP-IDF
    console.print()
    console.info("Checking ESP-IDF...")
    idf_path = platform.esp_idf_path()
    if idf_path:
        idf_version = platform.esp_idf_version()
        console.success(f"  ESP-IDF: {idf_version or 'found'}")
        console.success(f"  Path: {idf_path}")

        # Check if export.sh exists
        export_script = idf_path / "export.sh"
        if not export_script.exists():
            warnings.append("ESP-IDF export.sh not found")
            console.warning("  export.sh: NOT FOUND")
    else:
        warnings.append("ESP-IDF not found")
        console.warning("  ESP-IDF: NOT FOUND")
        console.print("    Install from: https://docs.espressif.com/projects/esp-idf/")

    # Check Python packages
    console.print()
    console.info("Checking Python packages...")
    required_packages = [
        ("numpy", "numpy"),
        ("scipy", "scipy"),
        ("matplotlib", "matplotlib"),
        ("pandas", "pandas"),
        ("serial", "pyserial"),
    ]

    for import_name, package_name in required_packages:
        try:
            __import__(import_name)
            console.success(f"  {package_name}")
        except ImportError:
            warnings.append(f"Missing package: {package_name}")
            console.warning(f"  {package_name}: NOT INSTALLED")

    # Check serial ports
    console.print()
    console.info("Checking serial ports...")
    ports = platform.serial_ports()
    if ports:
        for port in ports:
            console.success(f"  {port}")
    else:
        console.print("  No serial ports found (connect device to see ports)")

    # Check StampFly configuration
    console.print()
    console.info("Checking StampFly configuration...")
    config_file = paths.root() / ".sf" / "config.toml"
    if config_file.exists():
        console.success(f"  Config: {config_file}")
    else:
        warnings.append("Configuration not found (run ./install.sh)")
        console.warning("  Config: NOT FOUND")
        console.print("    Run: ./install.sh")

    # Summary
    console.print()
    console.header("Summary")

    if issues:
        console.error(f"Found {len(issues)} error(s):")
        for issue in issues:
            console.print(f"  - {issue}")
        console.print()

    if warnings:
        console.warning(f"Found {len(warnings)} warning(s):")
        for warning in warnings:
            console.print(f"  - {warning}")
        console.print()

    if not issues and not warnings:
        console.success("All checks passed!")
        return 0
    elif issues:
        console.error("Environment has critical issues. Please fix before proceeding.")
        return 1
    else:
        console.warning("Environment has warnings but should work.")
        return 0
