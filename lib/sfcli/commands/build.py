"""
sf build - Build firmware

Builds vehicle or controller firmware using ESP-IDF.
ESP-IDFを使用してファームウェアをビルドします。
"""

import argparse
import subprocess
import os
from pathlib import Path
from ..utils import console, paths, platform

COMMAND_NAME = "build"
COMMAND_HELP = "Build firmware"


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
        help="Target to build (default: vehicle)",
    )
    parser.add_argument(
        "-c", "--clean",
        action="store_true",
        help="Clean build (fullclean before build)",
    )
    parser.add_argument(
        "-j", "--jobs",
        type=int,
        default=None,
        help="Number of parallel jobs",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose build output",
    )
    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute build command"""
    # Determine target directory
    if args.target == "vehicle":
        target_dir = paths.vehicle()
    else:
        target_dir = paths.controller()

    if not target_dir.exists():
        console.error(f"Target directory not found: {target_dir}")
        return 1

    console.info(f"Building {args.target} firmware...")
    console.print(f"  Directory: {target_dir}")

    # Check ESP-IDF
    idf_path = platform.esp_idf_path()
    if not idf_path:
        console.error("ESP-IDF not found. Please install ESP-IDF first.")
        console.print("  See: https://docs.espressif.com/projects/esp-idf/")
        return 1

    # Prepare environment
    env = _prepare_idf_env(idf_path)
    if env is None:
        console.error("Failed to prepare ESP-IDF environment")
        return 1

    # Clean if requested
    if args.clean:
        console.info("Cleaning build directory...")
        result = subprocess.run(
            ["idf.py", "fullclean"],
            cwd=target_dir,
            env=env,
        )
        if result.returncode != 0:
            console.warning("Clean failed, continuing with build...")

    # Build command
    cmd = ["idf.py", "build"]

    if args.jobs:
        cmd.extend(["-j", str(args.jobs)])

    if args.verbose:
        cmd.append("-v")

    console.print()
    console.info(f"Running: {' '.join(cmd)}")
    console.print()

    # Execute build
    result = subprocess.run(cmd, cwd=target_dir, env=env)

    if result.returncode == 0:
        console.print()
        console.success(f"Build successful: {args.target}")

        # Show binary info
        binary_path = target_dir / "build" / f"{_get_project_name(target_dir)}.bin"
        if binary_path.exists():
            size_kb = binary_path.stat().st_size / 1024
            console.print(f"  Binary: {binary_path}")
            console.print(f"  Size: {size_kb:.1f} KB")

        return 0
    else:
        console.print()
        console.error(f"Build failed: {args.target}")
        return result.returncode


def _prepare_idf_env(idf_path: Path) -> dict:
    """Prepare environment with ESP-IDF settings"""
    env = os.environ.copy()
    env["IDF_PATH"] = str(idf_path)

    # Source export.sh and capture environment
    if platform.is_windows():
        # Windows: use export.bat
        export_script = idf_path / "export.bat"
        if not export_script.exists():
            return env

        # Run export.bat and capture environment
        try:
            result = subprocess.run(
                f'cmd /c "{export_script}" && set',
                shell=True,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                for line in result.stdout.split("\n"):
                    if "=" in line:
                        key, _, value = line.partition("=")
                        env[key.strip()] = value.strip()
        except Exception:
            pass
    else:
        # Unix: source export.sh
        export_script = idf_path / "export.sh"
        if not export_script.exists():
            return env

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


def _get_project_name(project_dir: Path) -> str:
    """Get project name from CMakeLists.txt"""
    cmake_file = project_dir / "CMakeLists.txt"
    if cmake_file.exists():
        content = cmake_file.read_text()
        for line in content.split("\n"):
            if "project(" in line:
                # Extract project name from project(name)
                start = line.find("(") + 1
                end = line.find(")")
                if start > 0 and end > start:
                    return line[start:end].strip()
    return "firmware"
