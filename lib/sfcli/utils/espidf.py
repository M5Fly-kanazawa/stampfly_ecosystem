"""
ESP-IDF utilities for StampFly CLI

Handles ESP-IDF environment setup and command execution.
ESP-IDF環境のセットアップとコマンド実行を処理
"""

import os
import subprocess
from pathlib import Path
from typing import Optional
from .platform import platform


def find_idf_path() -> Optional[Path]:
    """Find ESP-IDF installation path"""
    return platform.esp_idf_path()


def prepare_idf_env(idf_path: Optional[Path] = None) -> dict:
    """Prepare environment with ESP-IDF settings

    Important: We need to use ESP-IDF's Python environment, not our venv.
    This is because ESP-IDF tools like esp_idf_monitor are installed in
    ESP-IDF's Python environment.

    Args:
        idf_path: Path to ESP-IDF, or None to auto-detect

    Returns:
        Environment dictionary with ESP-IDF settings
    """
    if idf_path is None:
        idf_path = find_idf_path()
        if idf_path is None:
            return os.environ.copy()

    # Start with essential environment variables only
    env = {}
    essential_vars = [
        "HOME", "USER", "PATH", "SHELL", "TERM", "LANG", "LC_ALL",
        "TMPDIR", "TEMP", "TMP",
        # macOS specific
        "DEVELOPER_DIR", "SDKROOT",
        # Windows specific
        "SYSTEMROOT", "COMSPEC", "USERPROFILE", "APPDATA", "LOCALAPPDATA",
    ]
    for var in essential_vars:
        if var in os.environ:
            env[var] = os.environ[var]

    env["IDF_PATH"] = str(idf_path)

    # Source export.sh and capture the FULL environment
    if platform.is_windows():
        export_script = idf_path / "export.bat"
        if not export_script.exists():
            return os.environ.copy()

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
            return os.environ.copy()
    else:
        export_script = idf_path / "export.sh"
        if not export_script.exists():
            return os.environ.copy()

        try:
            # Source export.sh in a clean bash environment
            result = subprocess.run(
                f'bash -c \'source "{export_script}" > /dev/null 2>&1 && env\'',
                shell=True,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                for line in result.stdout.split("\n"):
                    if "=" in line:
                        key, _, value = line.partition("=")
                        env[key.strip()] = value.strip()
            else:
                return os.environ.copy()
        except Exception:
            return os.environ.copy()

    return env


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

    return subprocess.run(cmd, cwd=cwd, env=env)
