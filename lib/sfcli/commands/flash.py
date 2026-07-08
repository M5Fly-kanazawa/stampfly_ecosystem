"""
sf flash - Flash firmware to device

Flashes vehicle or controller firmware to connected device.
接続されたデバイスにファームウェアを書き込みます。
"""

import argparse
import subprocess
import sys
from pathlib import Path
from ..utils import console, paths, platform, espidf

COMMAND_NAME = "flash"
COMMAND_HELP = "Flash firmware to device"


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
        help="Target to flash (default: vehicle). Use 'sf app list' to see available targets.",
    )
    parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (auto-detect if not specified)",
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=460800,
        help="Baud rate (default: 460800)",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="Flash factory legacy firmware (vehicle or controller)",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Build before flashing",
    )
    parser.add_argument(
        "-m", "--monitor",
        action="store_true",
        help="Start monitor after flashing",
    )
    # Standalone Tkinter GUI flasher, for users without a build environment
    # (also works around the Web Flasher's Chromium Web Serial crash on macOS)
    # ビルド環境を持たないユーザー向けのスタンドアロン Tkinter GUI ライタ
    # （macOS で Web Flasher が Chromium Web Serial のバグでクラッシュする問題の回避策でもある）
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Launch the standalone GUI flasher (StampFly Flasher) instead of building/flashing here",
    )
    parser.set_defaults(func=run)


# Legacy binary definitions per target
# ターゲットごとのレガシーバイナリ定義
LEGACY_BINARIES = {
    "vehicle": {
        "file": "StampFlyVehicle_V1.0.0_merged_0x0.bin",
        "offset": "0x0",
        "label": "StampFly Vehicle (factory)",
    },
    "controller": {
        "file": "StampFlyController_V1.0.0_merged_0x0.bin",
        "offset": "0x0",
        "label": "StampFly Controller (factory)",
    },
}


def _flash_legacy(args: argparse.Namespace) -> int:
    """Flash legacy factory firmware (PlatformIO merged binary)
    レガシーファクトリーファームウェアを書き込む"""
    target = args.target

    if target not in LEGACY_BINARIES:
        console.error(f"No legacy firmware available for target: {target}")
        console.print(f"  Available targets: {', '.join(LEGACY_BINARIES.keys())}")
        return 1

    info = LEGACY_BINARIES[target]
    legacy_bin = paths.firmware() / "legacy" / info["file"]
    if not legacy_bin.exists():
        console.error(f"Legacy binary not found: {legacy_bin}")
        return 1

    # Detect or use specified port
    # シリアルポートの検出または指定ポートの使用
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
        console.info(f"Auto-detected port: {port}")

    # Check ESP-IDF for esptool
    idf_path = platform.esp_idf_path()
    if not idf_path:
        console.error("ESP-IDF not found. esptool.py is required.")
        return 1

    env = espidf.prepare_idf_env(idf_path)

    # Use esptool.py to write binary at target-specific offset
    # esptool.py でバイナリをターゲット固有のオフセットに書き込む
    cmd = [
        "python", "-m", "esptool",
        "--chip", "esp32s3",
        "--port", port,
        "--baud", str(args.baud),
        "write_flash", info["offset"], str(legacy_bin),
    ]

    console.info(f"Flashing legacy firmware: {info['label']}...")
    console.print(f"  Binary: {legacy_bin}")
    console.print(f"  Offset: {info['offset']}")
    console.print(f"  Port:   {port}")
    console.print(f"  Baud:   {args.baud}")
    console.print()

    result = subprocess.run(cmd, env=env)

    if result.returncode == 0:
        console.print()
        console.success(f"Legacy firmware flashed successfully: {info['label']}")
        return 0
    else:
        console.print()
        console.error(f"Legacy firmware flash failed: {target}")
        return result.returncode


def _launch_gui(args: argparse.Namespace) -> int:
    """Launch the standalone StampFly Flasher GUI (tools/flasher_gui/) and
    return its exit code, instead of running the console build/flash flow.
    スタンドアロンの StampFly Flasher GUI（tools/flasher_gui/）を起動し、
    その終了コードを返す。コンソールのビルド/書き込みフローは実行しない。"""
    gui_script = paths.tools() / "flasher_gui" / "stampfly_flasher.py"
    if not gui_script.exists():
        console.error(f"GUI script not found: {gui_script}")
        return 1

    # Forward the port (if specified) and target so the GUI starts pre-filled
    # ポート（指定時）とターゲットを転送し、GUI 側を事前入力させる
    cmd = [sys.executable, str(gui_script)]
    if args.port:
        cmd.extend(["--port", args.port])
    if args.target:
        cmd.extend(["--target", args.target])

    console.info("Launching StampFly Flasher GUI...")
    result = subprocess.run(cmd)
    return result.returncode


def run(args: argparse.Namespace) -> int:
    """Execute flash command"""
    # GUI shortcut: hand off to the standalone Tkinter flasher and stop here
    # GUI 起動: スタンドアロン Tkinter ライタに処理を委譲しここで終了
    if args.gui:
        return _launch_gui(args)

    # Legacy firmware shortcut
    # レガシーファームウェアの書き込み
    if args.legacy:
        return _flash_legacy(args)

    # Determine target directory
    # 動的ターゲット検出: firmware/ 配下の CMakeLists.txt を持つディレクトリ
    target_dir = paths.firmware_target_dir(args.target)

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
        console.info(f"Auto-detected port: {port}")

    # Build if requested
    if args.build:
        console.info("Building before flash...")
        from . import build as build_cmd
        build_args = argparse.Namespace(
            target=args.target,
            clean=False,
            jobs=None,
            verbose=False,
        )
        result = build_cmd.run(build_args)
        if result != 0:
            console.error("Build failed, aborting flash")
            return result
        console.print()

    # Prepare environment (uses ESP-IDF's Python, not our venv)
    env = espidf.prepare_idf_env(idf_path)

    # Flash command
    cmd = espidf.idf_command(["-p", port, "-b", str(args.baud)])

    if args.monitor:
        cmd.append("flash")
        cmd.append("monitor")
    else:
        cmd.append("flash")

    console.info(f"Flashing {args.target} firmware...")
    console.print(f"  Port: {port}")
    console.print(f"  Baud: {args.baud}")
    console.print()

    # Execute flash
    result = subprocess.run(cmd, cwd=target_dir, env=env)

    if result.returncode == 0:
        if not args.monitor:
            console.print()
            console.success(f"Flash successful: {args.target}")
        return 0
    else:
        console.print()
        console.error(f"Flash failed: {args.target}")
        return result.returncode
