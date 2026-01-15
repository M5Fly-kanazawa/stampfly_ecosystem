"""
sf setup - Install optional dependencies

Install optional dependencies for specific features.
特定機能のオプション依存パッケージをインストールします。

Subcommands:
    sim       - Install simulator dependencies (vpython, pygame)
    full      - Install all optional dependencies
    list      - List available dependency groups
"""

import argparse
import subprocess
import sys
from pathlib import Path

from ..utils import console, paths

COMMAND_NAME = "setup"
COMMAND_HELP = "Install optional dependencies"

# Dependency groups
GROUPS = {
    "sim": {
        "name": "VPython Simulator",
        "description": "VPython simulator dependencies (vpython, pygame, numpy-stl)",
        "packages": ["vpython>=7.6.0", "pygame>=2.5.0", "numpy-stl>=3.0.0", "hid>=1.0.0"],
    },
    "genesis": {
        "name": "Genesis Simulator",
        "description": "Genesis physics simulator (requires GPU, large download)",
        "packages": ["genesis-world", "torch", "pygame>=2.5.0"],
        "warning": "This will install PyTorch (~2GB). GPU recommended.",
    },
    "dev": {
        "name": "Development",
        "description": "Development tools (pytest, pytest-cov)",
        "packages": ["pytest>=7.0.0", "pytest-cov>=4.0.0"],
    },
    "full": {
        "name": "Full",
        "description": "All optional dependencies (excluding Genesis)",
        "packages": [
            "esptool>=4.6",
            "Pillow>=10.0.0",
            "opencv-python>=4.8.0",
            "vpython>=7.6.0",
            "pygame>=2.5.0",
            "numpy-stl>=3.0.0",
        ],
    },
}


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers
    setup_subparsers = parser.add_subparsers(
        dest="setup_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- list ---
    list_parser = setup_subparsers.add_parser(
        "list",
        help="List available dependency groups",
        description="Show all available optional dependency groups.",
    )
    list_parser.set_defaults(func=run_list)

    # --- sim ---
    sim_parser = setup_subparsers.add_parser(
        "sim",
        help="Install VPython simulator dependencies",
        description="Install VPython simulator dependencies.",
    )
    sim_parser.set_defaults(func=lambda args: run_install(args, "sim"))

    # --- genesis ---
    genesis_parser = setup_subparsers.add_parser(
        "genesis",
        help="Install Genesis simulator dependencies",
        description="Install Genesis physics simulator (requires GPU, large download).",
    )
    genesis_parser.set_defaults(func=lambda args: run_install(args, "genesis"))

    # --- dev ---
    dev_parser = setup_subparsers.add_parser(
        "dev",
        help="Install development dependencies",
        description="Install development and testing tools.",
    )
    dev_parser.set_defaults(func=lambda args: run_install(args, "dev"))

    # --- full ---
    full_parser = setup_subparsers.add_parser(
        "full",
        help="Install all optional dependencies",
        description="Install all optional dependencies.",
    )
    full_parser.set_defaults(func=lambda args: run_install(args, "full"))

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf setup <subcommand>")
    console.print()
    console.print("Subcommands:")
    console.print("  list     List available dependency groups")
    console.print("  sim      Install VPython simulator dependencies")
    console.print("  genesis  Install Genesis simulator (GPU, ~2GB)")
    console.print("  dev      Install development dependencies")
    console.print("  full     Install all optional dependencies")
    console.print()
    console.print("Examples:")
    console.print("  sf setup sim      # Install VPython simulator")
    console.print("  sf setup genesis  # Install Genesis simulator")
    console.print("  sf setup full     # Install all optional dependencies")
    console.print()
    console.print("Alternative (using pip):")
    console.print("  pip install -e '.[sim]'")
    console.print("  pip install -e '.[full]'")
    return 0


def run_list(args: argparse.Namespace) -> int:
    """List available dependency groups"""
    console.info("Available dependency groups:")
    console.print()

    for group_id, group in GROUPS.items():
        console.print(f"  {group_id:8s} - {group['name']}")
        console.print(f"             {group['description']}")
        if "warning" in group:
            console.print(f"             Warning: {group['warning']}")
        console.print(f"             Packages: {', '.join(group['packages'])}")
        console.print()

    console.print("Usage:")
    console.print("  sf setup <group>         # Install using pip")
    console.print("  pip install -e '.[sim]'  # Alternative using extras")

    return 0


def run_install(args: argparse.Namespace, group_id: str) -> int:
    """Install a dependency group"""
    group = GROUPS.get(group_id)

    if not group:
        console.error(f"Unknown group: {group_id}")
        return 1

    console.info(f"Installing {group['name']} dependencies...")

    # Show warning if present
    if "warning" in group:
        console.print()
        console.print(f"  Warning: {group['warning']}")

    console.print()
    console.print("Packages:")
    for pkg in group["packages"]:
        console.print(f"  - {pkg}")
    console.print()

    # Use pip to install
    cmd = [sys.executable, "-m", "pip", "install"] + group["packages"]

    try:
        result = subprocess.run(cmd)
        if result.returncode == 0:
            console.info(f"{group['name']} dependencies installed successfully!")
            if group_id == "sim":
                console.print()
                console.print("You can now run the VPython simulator:")
                console.print("  sf sim run vpython")
            elif group_id == "genesis":
                console.print()
                console.print("You can now run the Genesis simulator:")
                console.print("  sf sim run genesis")
        return result.returncode

    except Exception as e:
        console.error(f"Failed to install: {e}")
        return 1
