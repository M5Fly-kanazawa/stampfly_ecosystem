"""
ESP-IDF utilities for StampFly CLI

Handles ESP-IDF environment setup and command execution.
ESP-IDF環境のセットアップとコマンド実行を処理
"""

import os
import shutil
import sys
import subprocess
from pathlib import Path
from typing import Optional, List
from .platform import platform


def find_idf_path() -> Optional[Path]:
    """Find ESP-IDF installation path"""
    return platform.esp_idf_path()


def idf_command(args: List[str]) -> List[str]:
    """Build command list to invoke idf.py cross-platform.
    idf.pyをクロスプラットフォームで呼び出すコマンドリストを構築

    On Windows, idf.py cannot be executed directly (WinError 193).
    We invoke it via sys.executable (the ESP-IDF venv Python).

    Args:
        args: Arguments to pass to idf.py (e.g., ["build"], ["-p", "COM3", "flash"])

    Returns:
        Command list suitable for subprocess.run()
        ['build'] → [sys.executable, '<IDF_PATH>/tools/idf.py', 'build']  (Windows)
        ['build'] → ['idf.py', 'build']                                    (Unix)
    """
    if platform.is_windows():
        idf_path = find_idf_path()
        if idf_path:
            idf_py = idf_path / "tools" / "idf.py"
            return [sys.executable, str(idf_py)] + args
        # Fallback: try python + idf.py on PATH
        # フォールバック: PATH上のidf.pyをpython経由で実行
        return [sys.executable, "idf.py"] + args
    else:
        return ["idf.py"] + args


def prepare_idf_env(idf_path: Optional[Path] = None) -> dict:
    """Prepare environment with ESP-IDF settings

    Since sfcli is now installed in ESP-IDF's Python environment,
    we assume the user has already sourced export.sh before running sf commands.
    We simply inherit the current environment.

    Args:
        idf_path: Path to ESP-IDF, or None to auto-detect (unused, kept for compatibility)

    Returns:
        Environment dictionary (copy of current environment)
    """
    # Just use the current environment - user should have sourced export.sh
    return os.environ.copy()


def verify_idf_env(env: dict) -> Optional[str]:
    """Verify that the ESP-IDF environment carried in `env` is loaded and fresh.
    `env` に含まれるESP-IDF環境がロード済みかつ有効(陳腐化していない)か検証する

    prepare_idf_env() just inherits whatever the shell already has. If the
    caller's terminal never sourced setup_env.sh, idf.py is simply missing
    from PATH ("[Errno 2] No such file or directory: 'idf.py'"). If the
    terminal sourced it a long time ago and the referenced ESP-IDF Python
    venv was since deleted/recreated (e.g. by a reinstall), idf.py silently
    falls back to a bare system Python and fails with an unrelated-looking
    "No module named 'click'". Both are confusing to debug from the error
    alone, so this check catches them up front with actionable guidance.

    prepare_idf_env() は現在のシェル環境をそのまま継承するだけである。呼び出し元の
    ターミナルで setup_env.sh が一度も source されていない場合、idf.py が単に
    PATH に無く「[Errno 2] No such file or directory: 'idf.py'」となる。以前に
    source した後、参照先の ESP-IDF Python venv が削除・再作成された場合
    （再インストール等）は、idf.py が素の system Python にフォールバックし、
    一見無関係な「No module named 'click'」で失敗する。どちらもエラーメッセージ
    単体では原因が分かりにくいため、実行前にこの検証で検出し対処法を提示する。

    Args:
        env: Environment dict to verify (as returned by prepare_idf_env()).

    Returns:
        None if the environment looks usable. Otherwise a bilingual,
        actionable error message ready to hand to console.error().
    """
    venv_path_str = env.get("IDF_PYTHON_ENV_PATH")
    if not venv_path_str:
        return (
            "ESP-IDF environment is not loaded in this terminal.\n"
            "ESP-IDF環境がこのターミナルにロードされていません。\n"
            "  -> Run 'source setup_env.sh' in this terminal, then retry.\n"
            "  -> このターミナルで 'source setup_env.sh' を実行してから再試行してください。"
        )

    venv_dir = Path(venv_path_str)
    bin_subdir = "Scripts" if platform.is_windows() else "bin"
    python_name = "python.exe" if platform.is_windows() else "python"
    venv_python = venv_dir / bin_subdir / python_name

    if not venv_dir.is_dir() or not venv_python.is_file():
        return (
            "This terminal references a deleted/stale ESP-IDF Python "
            f"environment ({venv_path_str}).\n"
            "このターミナルは削除済みまたは古いESP-IDF Python環境を参照しています "
            f"({venv_path_str})。\n"
            "  -> Open a NEW terminal and run 'source setup_env.sh' again.\n"
            "  -> 新しいターミナルを開いて 'source setup_env.sh' をやり直してください。"
        )

    # idf_command() invokes 'idf.py' via PATH on Unix (via sys.executable on
    # Windows, so PATH is irrelevant there).
    # idf_command() はUnixではPATH経由でidf.pyを起動する
    # （Windowsではsys.executable経由のためPATHは無関係）
    if not platform.is_windows():
        if shutil.which("idf.py", path=env.get("PATH")) is None:
            return (
                "idf.py was not found on PATH.\n"
                "idf.py がPATH上に見つかりません。\n"
                "  -> Run 'source setup_env.sh' in this terminal, then retry.\n"
                "  -> このターミナルで 'source setup_env.sh' を実行してから再試行してください。"
            )

    return None


def run_idf_command(
    cmd: list,
    cwd: Path,
    idf_path: Optional[Path] = None,
) -> subprocess.CompletedProcess:
    """Run an idf.py command with proper environment

    Args:
        cmd: Command and arguments (e.g., ["idf.py", "build"])
        cwd: Working directory
        idf_path: Path to ESP-IDF, or None to auto-detect

    Returns:
        CompletedProcess from subprocess.run
    """
    if idf_path is None:
        idf_path = find_idf_path()

    env = prepare_idf_env(idf_path)

    # Use idf_command() for cross-platform compatibility
    # クロスプラットフォーム対応のため idf_command() を使用
    if cmd and cmd[0] == "idf.py":
        cmd = idf_command(cmd[1:])

    return subprocess.run(cmd, cwd=cwd, env=env)
