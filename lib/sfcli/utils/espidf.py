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


def _prepare_idf_env_unix(idf_path: Path) -> dict:
    """Prepare ESP-IDF environment for Unix (macOS/Linux)"""
    export_script = idf_path / "export.sh"
    if not export_script.exists():
        return os.environ.copy()

    # Build minimal environment to pass to bash
    # CRITICAL: Do NOT include PATH from current env (it has venv)
    home = os.environ.get("HOME", "")
    user = os.environ.get("USER", "")
    shell = os.environ.get("SHELL", "/bin/bash")
    term = os.environ.get("TERM", "xterm-256color")
    lang = os.environ.get("LANG", "en_US.UTF-8")

    # Use env -i to start with clean environment, then source export.sh
    # Pass only essential vars that ESP-IDF needs
    cmd = f'''env -i \
        HOME="{home}" \
        USER="{user}" \
        SHELL="{shell}" \
        TERM="{term}" \
        LANG="{lang}" \
        PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin" \
        IDF_PATH="{idf_path}" \
        bash -c 'source "{export_script}" > /dev/null 2>&1 && env' '''

    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            env = {}
            for line in result.stdout.split("\n"):
                if "=" in line:
                    key, _, value = line.partition("=")
                    key = key.strip()
                    value = value.strip()
                    if key:  # Skip empty keys
                        env[key] = value
            # Ensure IDF_PATH is set
            env["IDF_PATH"] = str(idf_path)
            return env
    except Exception:
        pass

    return os.environ.copy()


def _prepare_idf_env_windows(idf_path: Path) -> dict:
    """Prepare ESP-IDF environment for Windows"""
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
            env = {}
            for line in result.stdout.split("\n"):
                if "=" in line:
                    key, _, value = line.partition("=")
                    key = key.strip()
                    value = value.strip()
                    if key:
                        env[key] = value
            env["IDF_PATH"] = str(idf_path)
            return env
    except Exception:
        pass

    return os.environ.copy()


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
