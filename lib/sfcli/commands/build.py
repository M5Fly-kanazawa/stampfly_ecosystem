"""
sf build - Build firmware

Builds vehicle or controller firmware using ESP-IDF.
ESP-IDFを使用してファームウェアをビルドします。
"""

import argparse
import subprocess
from pathlib import Path
from ..utils import console, paths, platform, espidf

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
        help="Target to build (default: vehicle). Use 'sf app list' to see available targets.",
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


def make_run_args(**overrides) -> argparse.Namespace:
    """Build a Namespace with all attributes run() expects.

    Single source of truth for the defaults consumed by run(), so callers
    that delegate to this module (e.g. `sf lesson build`) do not need to
    hand-assemble a Namespace and silently drift out of sync when run()
    grows a new attribute. Defaults mirror register()'s argparse defaults.

    run() が参照する全属性を備えた Namespace を生成する。

    run() が読む属性のデフォルト値を一箇所にまとめたもの。このモジュールへ
    委譲する呼び出し元（例: `sf lesson build`）が Namespace を手作りせずに
    済み、run() に属性が増えても追従漏れが起きない構造にする。デフォルト値
    は register() の argparse 定義と一致させること。
    """
    defaults = dict(
        target="vehicle",
        clean=False,
        jobs=None,
        verbose=False,
    )
    # Reject unknown keys so a typo fails loudly instead of silently
    # leaving the real attribute at its default.
    # 未知のキーは即エラーにする。タイポが黙って無視され、本来の属性が
    # デフォルトのまま残る事故を防ぐ。
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ValueError(f"Unknown build run() attribute(s): {sorted(unknown)}")
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def run(args: argparse.Namespace) -> int:
    """Execute build command"""
    # Determine target directory
    # 動的ターゲット検出: firmware/ 配下の CMakeLists.txt を持つディレクトリ
    target_dir = paths.firmware_target_dir(args.target)

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

    # Prepare environment (uses ESP-IDF's Python, not our venv)
    env = espidf.prepare_idf_env(idf_path)
    if env is None:
        console.error("Failed to prepare ESP-IDF environment")
        return 1

    # Verify the inherited environment is actually usable before invoking
    # idf.py, so a missing/stale `source setup_env.sh` fails with clear
    # guidance instead of a confusing "No module named 'click'".
    # idf.py実行前に継承した環境が実際に使えるか検証する。これにより
    # setup_env.sh未実行/陳腐化を「No module named 'click'」のような
    # 分かりにくいエラーではなく、明確な案内で失敗させる。
    env_error = espidf.verify_idf_env(env)
    if env_error:
        console.error(env_error)
        return 1

    # Clean if requested
    if args.clean:
        console.info("Cleaning build directory...")
        result = subprocess.run(
            espidf.idf_command(["fullclean"]),
            cwd=target_dir,
            env=env,
        )
        if result.returncode != 0:
            console.warning("Clean failed, continuing with build...")

    # Build command
    cmd = espidf.idf_command(["build"])

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


def _get_project_name(project_dir: Path) -> str:
    """Get project name from CMakeLists.txt"""
    cmake_file = project_dir / "CMakeLists.txt"
    if cmake_file.exists():
        content = cmake_file.read_text(encoding="utf-8")
        for line in content.split("\n"):
            if "project(" in line:
                # Extract project name from project(name)
                start = line.find("(") + 1
                end = line.find(")")
                if start > 0 and end > start:
                    return line[start:end].strip()
    return "firmware"
