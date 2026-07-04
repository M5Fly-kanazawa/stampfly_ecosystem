"""
sf doctor - Diagnose environment issues

Checks the development environment for common issues.
開発環境の問題を診断します。
"""

import argparse
import subprocess
import sys
from pathlib import Path
from ..utils import console, paths, platform

COMMAND_NAME = "doctor"
COMMAND_HELP = "Diagnose environment issues"

MANIFEST_FILE = "lesson_manifest.yaml"


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


def _check_manifest(warnings: list, issues: list) -> None:
    """Validate lesson manifest against filesystem.

    Checks:
    1. Manifest file exists and is valid YAML
    2. Each lesson's firmware_dir exists (if specified)
    3. Each firmware_dir has student.cpp and solution.cpp
    4. Each lesson's slide_file exists in chapters/
    5. No orphan lesson directories (dirs not in manifest)
    """
    lessons_dir = paths.workshop() / "lessons"
    manifest_path = lessons_dir / MANIFEST_FILE

    if not manifest_path.exists():
        warnings.append("Lesson manifest not found")
        console.warning(f"  {MANIFEST_FILE}: NOT FOUND")
        return

    try:
        import yaml
    except ImportError:
        warnings.append("PyYAML not installed — cannot validate manifest")
        console.warning("  PyYAML: NOT INSTALLED (pip install pyyaml)")
        return

    try:
        with open(manifest_path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        lessons = data.get("lessons", [])
    except Exception as e:
        issues.append(f"Manifest parse error: {e}")
        console.error(f"  {MANIFEST_FILE}: PARSE ERROR — {e}")
        return

    console.success(f"  {MANIFEST_FILE}: {len(lessons)} lessons")

    # Slide chapters directory
    chapters_dir = (paths.root() / "docs" / "workshop"
                    / "slides" / "beamer" / "chapters")

    # Track manifest firmware dirs for orphan check
    manifest_fw_dirs = set()

    for entry in lessons:
        num = entry.get("number", "?")
        lid = entry.get("id", "unknown")
        fw_dir = entry.get("firmware_dir")
        slide_file = entry.get("slide_file")

        # Check firmware directory
        if fw_dir is not None:
            manifest_fw_dirs.add(fw_dir)
            fw_path = lessons_dir / fw_dir
            if not fw_path.exists():
                issues.append(f"Lesson {num} ({lid}): firmware_dir '{fw_dir}' missing")
                console.error(f"  Lesson {num}: firmware_dir '{fw_dir}' NOT FOUND")
            else:
                # Check for required files
                for fname in ("student.cpp", "solution.cpp"):
                    if not (fw_path / fname).exists():
                        warnings.append(f"Lesson {num} ({lid}): {fname} missing")
                        console.warning(f"  Lesson {num}: {fw_dir}/{fname} MISSING")

        # Check slide file
        if slide_file:
            slide_path = chapters_dir / f"{slide_file}.tex"
            if not slide_path.exists():
                issues.append(f"Lesson {num} ({lid}): slide '{slide_file}.tex' missing")
                console.error(f"  Lesson {num}: chapters/{slide_file}.tex NOT FOUND")

    # Orphan check: lesson directories not in manifest
    if lessons_dir.exists():
        for d in sorted(lessons_dir.iterdir()):
            if not d.is_dir() or not d.name.startswith("lesson_"):
                continue
            if d.name not in manifest_fw_dirs:
                warnings.append(f"Orphan directory: {d.name} (not in manifest)")
                console.warning(f"  Orphan: {d.name} (not in manifest)")


def run(args: argparse.Namespace) -> int:
    """Execute doctor command"""
    console.header("StampFly Environment Diagnostics")
    console.print()

    issues = []
    warnings = []

    # Check Python version
    console.info("Checking Python...")
    py_version = sys.version_info
    if py_version < (3, 8):
        issues.append(f"Python 3.8+ required, found {py_version.major}.{py_version.minor}")
        console.error(f"  Python {py_version.major}.{py_version.minor} - 3.8+ required")
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

    # Legacy firmware (optional, not a hard requirement)
    # レガシーファーム（任意、必須ではない）
    vehicle_old_path = root / "firmware" / "vehicle_old"
    if vehicle_old_path.exists():
        console.success(f"  Legacy vehicle firmware: {vehicle_old_path}")

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

    # Check hidapi native library (for joystick / simulator)
    # hidapiネイティブライブラリの確認（ジョイスティック／シミュレータ用）
    console.print()
    console.info("Checking hidapi (joystick support)...")
    try:
        import hid
        hid.enumerate()
        console.success("  hidapi native library available")
    except ImportError:
        warnings.append("hid package not installed")
        console.warning("  hid package: NOT INSTALLED")
        console.print("    pip install hid")
    except OSError:
        warnings.append("hidapi native library not found")
        console.warning("  hidapi native library: NOT FOUND")
        if sys.platform == "darwin":
            console.print("    Install with: brew install hidapi")
        elif sys.platform == "linux":
            console.print("    Install with: sudo apt install libhidapi-dev")

    # Check serial ports
    console.print()
    console.info("Checking serial ports...")
    ports = platform.serial_ports()
    if ports:
        for port in ports:
            console.success(f"  {port}")
    else:
        if platform.is_wsl():
            console.warning("  No serial ports found")
            console.print("    WSL2 requires usbipd-win to access USB devices.")
            console.print("    See WSL2 section below for details.")
        else:
            console.print("  No serial ports found (connect device to see ports)")

    # Check WSL2 environment
    # WSL2環境チェック
    if platform.is_wsl():
        console.print()
        console.info("Checking WSL2...")
        console.success("  WSL2 environment detected")

        # Check usbipd-win availability
        # usbipd-win の存在確認
        try:
            result = subprocess.run(
                ["usbipd.exe", "--version"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                usbipd_ver = result.stdout.strip()
                console.success(f"  usbipd-win: {usbipd_ver}")
            else:
                warnings.append("usbipd-win not working properly")
                console.warning("  usbipd-win: NOT WORKING")
                console.print("    Install: winget install usbipd")
                console.print("    See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
        except FileNotFoundError:
            warnings.append("usbipd-win not found (required for USB device access in WSL2)")
            console.warning("  usbipd-win: NOT FOUND")
            console.print("    Install on Windows side: winget install usbipd")
            console.print("    Then: sudo apt install linux-tools-generic hwdata")
            console.print("    See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
        except Exception:
            console.warning("  usbipd-win: could not check")

        # Check for Windows PATH pollution
        # Windows PATHの汚染チェック
        import os
        path_dirs = os.environ.get("PATH", "").split(":")
        mnt_paths = [p for p in path_dirs if p.startswith("/mnt/")]
        if mnt_paths:
            console.warning(f"  Windows PATH entries: {len(mnt_paths)} found")
            console.print("    These may cause issues with ESP-IDF tools.")
            console.print("    The installer filters these automatically during sfcli install.")

    # Check lesson manifest
    # レッスンマニフェスト検証
    console.print()
    console.info("Checking lesson manifest...")
    _check_manifest(warnings, issues)

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
