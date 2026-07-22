"""
sf flash - Flash firmware to device

Flashes vehicle or controller firmware to connected device.
接続されたデバイスにファームウェアを書き込みます。
"""

import argparse
import subprocess
import sys
from pathlib import Path
from ..utils import console, paths, platform, espidf, flasher_install

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

    # Verify the inherited environment is actually usable before invoking
    # esptool (see build.py for the failure modes this catches: a missing
    # or stale ESP-IDF Python env leads to a bare-Python "No module named
    # 'esptool'" instead of a clear cause).
    # esptool実行前に継承環境が使えるか検証する。ESP-IDF Python環境が
    # 未ロード・陳腐化していると素のPythonにフォールバックし、
    # 「No module named 'esptool'」のような分かりにくいエラーになる
    env_error = espidf.verify_idf_env(env)
    if env_error:
        console.error(env_error)
        return 1

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
    """Launch the StampFly Flasher GUI and return its exit code, instead
    of running the console build/flash flow.

    Prefers the native app installed via `sf flasher install` (starts
    faster, needs no Python/deps on the machine); falls back to running
    tools/flasher_gui/stampfly_flasher.py directly with this process's
    Python interpreter when nothing is installed.

    StampFly Flasher GUI を起動し、その終了コードを返す。コンソールの
    ビルド/書き込みフローは実行しない。

    `sf flasher install` でインストール済みのネイティブアプリを優先する
    （起動が速く、実行マシンに Python/依存関係が不要）。未インストールの
    場合は tools/flasher_gui/stampfly_flasher.py をこのプロセスの Python
    インタプリタで直接起動するフォールバックに回る。"""
    installed_executable = flasher_install.installed_app_executable()
    if installed_executable is not None:
        console.info(f"Launching installed StampFly Flasher app: {installed_executable}")
        cmd = [str(installed_executable)]
    else:
        gui_script = paths.tools() / "flasher_gui" / "stampfly_flasher.py"
        if not gui_script.exists():
            console.error(f"GUI script not found: {gui_script}")
            return 1
        console.info(
            "Launching StampFly Flasher GUI (script fallback; "
            "run 'sf flasher install' to install it as a native app)..."
        )
        cmd = [sys.executable, str(gui_script)]

    # Forward the port (if specified) and target so the GUI starts pre-filled
    # ポート（指定時）とターゲットを転送し、GUI 側を事前入力させる
    if args.port:
        cmd.extend(["--port", args.port])
    if args.target:
        cmd.extend(["--target", args.target])

    result = subprocess.run(cmd)
    return result.returncode


def make_run_args(**overrides) -> argparse.Namespace:
    """Build a Namespace with all attributes run() expects.

    Single source of truth for the defaults consumed by run() (and the
    helpers it dispatches to: _flash_legacy(), _launch_gui()), so callers
    that delegate to this module (e.g. `sf lesson flash`) do not need to
    hand-assemble a Namespace and silently drift out of sync when run()
    grows a new attribute. Defaults mirror register()'s argparse defaults.

    This replaces the previous pattern where each delegating command built
    its own ad-hoc Namespace and forgot to add new attributes (e.g. --gui
    added in 7022efc broke `sf lesson flash` because its hand-built
    Namespace lacked a `gui` attribute).

    run() が参照する全属性を備えた Namespace を生成する。

    run()（および run() が呼ぶ _flash_legacy()・_launch_gui()）が読む属性の
    デフォルト値を一箇所にまとめたもの。このモジュールへ委譲する呼び出し元
    （例: `sf lesson flash`）が Namespace を手作りせずに済み、run() に属性
    が増えても追従漏れが起きない構造にする。デフォルト値は register() の
    argparse 定義と一致させること。

    以前は委譲元コマンドがそれぞれ独自に Namespace を組み立てており、新規
    属性の追加漏れが発生していた（例: 7022efc で --gui を追加した際、
    `sf lesson flash` の手作り Namespace に gui 属性がなく実行時エラーに
    なった）。
    """
    defaults = dict(
        target="vehicle",
        port=None,
        baud=460800,
        legacy=False,
        build=False,
        monitor=False,
        gui=False,
    )
    # Reject unknown keys so a typo fails loudly instead of silently
    # leaving the real attribute at its default.
    # 未知のキーは即エラーにする。タイポが黙って無視され、本来の属性が
    # デフォルトのまま残る事故を防ぐ。
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ValueError(f"Unknown flash run() attribute(s): {sorted(unknown)}")
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


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
        build_args = build_cmd.make_run_args(target=args.target)
        result = build_cmd.run(build_args)
        if result != 0:
            console.error("Build failed, aborting flash")
            return result
        console.print()

    # Prepare environment (uses ESP-IDF's Python, not our venv)
    env = espidf.prepare_idf_env(idf_path)

    # Verify the inherited environment is actually usable before invoking
    # idf.py flash (see build.py for the failure modes this catches).
    # idf.py flash実行前に継承環境が使えるか検証する
    # （catchする失敗モードはbuild.pyのコメント参照）
    env_error = espidf.verify_idf_env(env)
    if env_error:
        console.error(env_error)
        return 1

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
