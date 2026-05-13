#!/usr/bin/env python3
"""
StampFly Ecosystem Installer

Installs sfcli into ESP-IDF's Python environment.
ESP-IDFのPython環境にsfcliをインストールします。

Usage:
    python scripts/installer.py [options]

Options:
    --idf-path PATH    Specify ESP-IDF path
    --skip-deps        Skip dependency installation
    --minimal          Install minimal dependencies (skip simulator)
    --uninstall        Remove sfcli from ESP-IDF environment
    --clean            Clean install (remove config and sfcli, then reinstall)
    --force            Force reinstall all steps (skip probe checks)
"""

import os
import shlex
import sys
import subprocess
import shutil
from pathlib import Path
from typing import Optional, List, Tuple

# Ensure we're running Python 3.8+
if sys.version_info < (3, 8):
    print(f"Error: Python 3.8+ required, found {sys.version_info.major}.{sys.version_info.minor}")
    sys.exit(1)


def is_wsl() -> bool:
    """Check if running in WSL2
    WSL2環境を検出"""
    if sys.platform != "linux":
        return False
    try:
        with open("/proc/version", "r") as f:
            return "microsoft" in f.read().lower()
    except Exception:
        return False


class Colors:
    """ANSI color codes"""
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    CYAN = "\033[36m"
    BOLD = "\033[1m"

    @classmethod
    def disable(cls):
        for attr in ["RESET", "RED", "GREEN", "YELLOW", "BLUE", "CYAN", "BOLD"]:
            setattr(cls, attr, "")


def info(msg: str) -> None:
    print(f"{Colors.BLUE}[INFO]{Colors.RESET} {msg}")


def success(msg: str) -> None:
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}")


def warn(msg: str) -> None:
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}")


def error(msg: str) -> None:
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {msg}", file=sys.stderr)


def header(title: str) -> None:
    line = "=" * 60
    print(f"\n{Colors.CYAN}{line}{Colors.RESET}")
    print(f"{Colors.BOLD} {title}{Colors.RESET}")
    print(f"{Colors.CYAN}{line}{Colors.RESET}\n")


def prompt(message: str, default: str = "") -> str:
    """Prompt user for input"""
    if default:
        message = f"{message} [{default}]: "
    else:
        message = f"{message}: "

    try:
        response = input(message).strip()
        return response if response else default
    except (EOFError, KeyboardInterrupt):
        print()
        return default


def prompt_choice(message: str, choices: List[str], default: int = 1) -> int:
    """Prompt user to select from choices"""
    print(f"\n{message}\n")
    for i, choice in enumerate(choices, 1):
        marker = " <- recommended" if i == default else ""
        print(f"  [{i}] {choice}{marker}")
    print()

    while True:
        try:
            response = input(f"Select [{default}]: ").strip()
            if not response:
                return default
            idx = int(response)
            if 1 <= idx <= len(choices):
                return idx
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f"Please enter a number between 1 and {len(choices)}")


def _clean_env_for_cmd() -> dict:
    """Return environment suitable for running .bat scripts via cmd.exe.
    cmd.exe 経由で .bat を実行するための環境を構築

    - Strips MSYSTEM (ESP-IDF .bat refuses to run under MINGW/Git Bash)
    - Appends current Python's directory to PATH as fallback so ESP-IDF
      install.bat can pass its python.exe prerequisite check.
    """
    env = os.environ.copy()
    env.pop("MSYSTEM", None)
    # Append Python as fallback for install.bat prerequisite check
    # install.batの前提条件チェック用フォールバックとしてPythonを末尾に追加
    python_dir = str(Path(sys.executable).parent)
    current_path = env.get("PATH", "")
    if python_dir.lower() not in current_path.lower():
        env["PATH"] = current_path + os.pathsep + python_dir
    return env


def _find_idf_python(idf_path: Path) -> Optional[Path]:
    """Find ESP-IDF's virtual environment Python directly.
    ESP-IDFの仮想環境Pythonを直接検索する

    Scans IDF_TOOLS_PATH (default: ~/.espressif or C:\\Espressif) for
    python_env/idf*_py*/Scripts/python.exe (Windows) or bin/python (Unix).
    """
    # Determine IDF_TOOLS_PATH
    # IDF_TOOLS_PATH を決定
    tools_path = os.environ.get("IDF_TOOLS_PATH")
    if tools_path:
        candidates = [Path(tools_path)]
    elif sys.platform == "win32":
        candidates = [Path("C:/Espressif"), Path.home() / ".espressif"]
    else:
        candidates = [Path.home() / ".espressif"]

    for base in candidates:
        python_env_dir = base / "python_env"
        if not python_env_dir.exists():
            continue
        # Find the newest idf*_py*_env directory
        # 最新の idf*_py*_env ディレクトリを探す
        try:
            venvs = sorted(python_env_dir.iterdir(), reverse=True)
        except OSError:
            continue
        for venv_dir in venvs:
            if not venv_dir.is_dir():
                continue
            if sys.platform == "win32":
                python_exe = venv_dir / "Scripts" / "python.exe"
            else:
                python_exe = venv_dir / "bin" / "python"
            if python_exe.exists():
                return python_exe
    return None


def _build_idf_env_command(idf_path: Path) -> str:
    """Build shell prefix that sources ESP-IDF and filters WSL2 PATH.
    ESP-IDF環境を読み込み、WSL2ではWindowsパスを除外するシェルプレフィックスを構築"""
    export_script = idf_path / "export.sh"
    # WSL2: strip /mnt/ paths to avoid Windows executables with CRLF
    # WSL2: /mnt/ パスを除外してCRLFのWindows実行ファイルを回避
    if is_wsl():
        path_filter = 'export PATH=$(echo "$PATH" | tr ":" "\\n" | grep -v "^/mnt/" | tr "\\n" ":"); '
    else:
        path_filter = ""
    return f'{path_filter}source "{export_script}" > /dev/null 2>&1'


def _run_in_idf_env(idf_path: Path, pip_args: list[str]) -> int:
    """Run pip in ESP-IDF Python environment.
    ESP-IDFのPython環境でpipを実行

    NOTE: Callers must pass pip_args as plain (unquoted) strings. This function
    applies shell-appropriate quoting internally. Passing pre-quoted values
    (e.g. f'"{path}"') will double-quote and break commands containing version
    specifiers like 'setuptools>=68.0,<81' where '>' and '<' would otherwise be
    interpreted as shell redirections.
    """
    if sys.platform == "win32":
        # Use venv Python directly instead of export.bat to avoid PATH conflicts
        # PATH競合を避けるため、export.batではなくvenv Pythonを直接使用
        venv_python = _find_idf_python(idf_path)
        if venv_python:
            cmd = [str(venv_python), "-m", "pip"] + pip_args
            return subprocess.run(cmd).returncode
        # Fallback: export.bat (if venv not found yet, e.g. during initial install)
        # cmd.exe quoting via subprocess.list2cmdline
        export_script = idf_path / "export.bat"
        escaped = subprocess.list2cmdline(pip_args)
        cmd = f'call "{export_script}" && python -m pip {escaped}'
        return subprocess.run(cmd, shell=True, env=_clean_env_for_cmd()).returncode
    else:
        # POSIX-safe quoting prevents shell metacharacters (<, >, |, &, ;, $, *)
        # in version specifiers or paths from being interpreted by bash
        # POSIX安全なクォートで <, >, | 等のメタ文字がリダイレクト等に
        # 解釈されないようにする
        escaped = " ".join(shlex.quote(arg) for arg in pip_args)
        env_prefix = _build_idf_env_command(idf_path)
        inner = f'{env_prefix} && python -m pip {escaped}'
        return subprocess.run(["bash", "-c", inner]).returncode


class ESPIDFDetector:
    """Detect ESP-IDF installations.
    All platforms use ~/esp/esp-idf as the standard location.
    全プラットフォームで ~/esp/esp-idf を標準パスとする"""

    COMMON_PATHS = [
        Path.home() / "esp" / "esp-idf",
        Path.home() / "esp" / "esp-idf-v5.4",
        Path.home() / "esp" / "esp-idf-v5.3",
        Path.home() / "esp" / "esp-idf-v5.2",
        Path.home() / "esp" / "esp-idf-v5.1",
        Path.home() / ".espressif" / "esp-idf",
        Path("/opt/esp-idf"),
    ]

    @classmethod
    def find_all(cls) -> List[Tuple[Path, str]]:
        """Find all ESP-IDF installations with versions"""
        installations = []
        seen_paths = set()

        # Check IDF_PATH environment variable
        if "IDF_PATH" in os.environ:
            idf_path = Path(os.environ["IDF_PATH"])
            if idf_path.exists() and cls._is_valid_idf(idf_path):
                version = cls._get_version(idf_path)
                installations.append((idf_path.resolve(), version))
                seen_paths.add(idf_path.resolve())

        # Check common paths
        for path in cls.COMMON_PATHS:
            path = path.resolve()
            if path not in seen_paths and path.exists() and cls._is_valid_idf(path):
                version = cls._get_version(path)
                installations.append((path, version))
                seen_paths.add(path)

        # Also check ~/esp/ for any esp-idf* directories
        esp_dir = Path.home() / "esp"
        if esp_dir.exists():
            for child in esp_dir.iterdir():
                if child.is_dir() and child.name.startswith("esp-idf"):
                    child = child.resolve()
                    if child not in seen_paths and cls._is_valid_idf(child):
                        version = cls._get_version(child)
                        installations.append((child, version))
                        seen_paths.add(child)

        # Sort by version (newest first)
        installations.sort(key=lambda x: x[1], reverse=True)
        return installations

    @classmethod
    def _is_valid_idf(cls, path: Path) -> bool:
        """Check if path is a valid ESP-IDF installation.
        ESP-IDF repos ship both export.sh and export.bat; accept either.
        ESP-IDFリポジトリはexport.shとexport.batの両方を含む。どちらかがあれば有効"""
        return (path / "export.sh").exists() or (path / "export.bat").exists()

    @classmethod
    def _get_version(cls, path: Path) -> str:
        """Get ESP-IDF version"""
        version_file = path / "version.txt"
        if version_file.exists():
            return version_file.read_text().strip()

        # Try git describe
        try:
            result = subprocess.run(
                ["git", "describe", "--tags", "--abbrev=0"],
                cwd=path,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception:
            pass

        return "unknown"

    @classmethod
    def get_python_env(cls, idf_path: Path) -> Optional[Path]:
        """Get the Python environment for an ESP-IDF installation.
        ESP-IDFインストールのPython環境を取得"""
        if sys.platform == "win32":
            # Find venv Python directly (avoids export.bat PATH issues)
            # venv Pythonを直接検索（export.batのPATH問題を回避）
            venv_python = _find_idf_python(idf_path)
            if venv_python:
                return venv_python
        else:
            # Use _build_idf_env_command to handle WSL2 PATH filtering
            # WSL2 PATH除外を含むコマンドを使用
            env_prefix = _build_idf_env_command(idf_path)
            inner = f'{env_prefix} && which python'
            try:
                result = subprocess.run(
                    ["bash", "-c", inner],
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    python_path = result.stdout.strip().split('\n')[0]
                    return Path(python_path)
            except Exception:
                pass

        return None


class ESPIDFInstaller:
    """Install ESP-IDF with recovery support.
    リカバリ対応のESP-IDFインストーラー"""

    # Use specific stable release tag, not branch name
    # v5.5.2 is the latest stable release as of January 2026
    # Update this when new stable releases are available
    DEFAULT_VERSION = "v5.5.2"
    REPO_URL = "https://github.com/espressif/esp-idf.git"

    @classmethod
    def _is_partial_clone(cls, path: Path) -> bool:
        """Detect incomplete clone (has .git but no export script).
        不完全なクローンを検出（.gitはあるがexportスクリプトがない）"""
        if not path.exists() or not (path / ".git").exists():
            return False
        # A complete clone has both export.sh and export.bat; accept either
        # 完全なクローンはexport.shとexport.batの両方を持つ。どちらかあればOK
        return not ESPIDFDetector._is_valid_idf(path)

    @classmethod
    def install(cls, target_dir: Optional[Path] = None, version: str = DEFAULT_VERSION) -> Optional[Path]:
        """Install ESP-IDF with 3-stage clone separation.
        3段階分離でESP-IDFをインストール"""
        if target_dir is None:
            target_dir = Path.home() / "esp" / "esp-idf"

        # Create parent directory
        target_dir.parent.mkdir(parents=True, exist_ok=True)

        info(f"Installing ESP-IDF {version} to {target_dir}...")
        print()

        # Stage 1: Check existing directory state
        # ステージ1: 既存ディレクトリの状態確認
        if target_dir.exists():
            if cls._is_partial_clone(target_dir):
                warn(f"Incomplete clone detected at {target_dir}, cleaning up...")
                shutil.rmtree(target_dir)
            elif ESPIDFDetector._is_valid_idf(target_dir):
                info("ESP-IDF repository already cloned, skipping to install step...")
                return cls._run_install_script(target_dir, version)
            else:
                # Directory exists but not a git repo or ESP-IDF
                # ディレクトリは存在するがgitリポジトリでもESP-IDFでもない
                error(f"Directory exists but is not ESP-IDF: {target_dir}")
                error("Remove it manually or specify a different path.")
                return None

        # Stage 2: Clone main repository (without submodules)
        # ステージ2: メインリポジトリのクローン（サブモジュールなし）
        info("Cloning ESP-IDF repository (main repo)...")
        try:
            subprocess.run(
                [
                    "git", "clone",
                    "--branch", version,
                    "--depth", "1",
                    cls.REPO_URL,
                    str(target_dir),
                ],
                check=True,
            )
        except subprocess.CalledProcessError as e:
            error(f"Failed to clone ESP-IDF: {e}")
            # Clean up failed clone
            # 失敗したクローンをクリーンアップ
            if target_dir.exists():
                shutil.rmtree(target_dir)
            return None

        # Stage 3: Initialize submodules (retryable)
        # ステージ3: サブモジュール初期化（リトライ可能）
        info("Initializing submodules (this may take a while)...")
        try:
            subprocess.run(
                [
                    "git", "submodule", "update",
                    "--init", "--depth", "1", "--recursive",
                ],
                cwd=target_dir,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            error(f"Failed to initialize submodules: {e}")
            warn("Main repository is preserved. Re-run installer to retry submodule init.")
            # Don't delete - main repo is intact, user can retry
            # 削除しない - メインリポジトリはそのまま、再実行でリトライ可能
            return None

        # Stage 4: Run install script (idempotent)
        # ステージ4: install.sh 実行（冪等）
        return cls._run_install_script(target_dir, version)

    @classmethod
    def _run_install_script(cls, target_dir: Path, version: str) -> Optional[Path]:
        """Run ESP-IDF install script (idempotent).
        ESP-IDFのinstall.shを実行（冪等）"""
        info("Installing ESP-IDF tools (this may take a while)...")
        try:
            if sys.platform == "win32":
                install_script = target_dir / "install.bat"
                # Use shell=True + call for .bat execution from any shell
                # shell=True + call で任意のシェルから .bat を確実に実行
                cmd = f'call "{install_script}" esp32s3'
                subprocess.run(cmd, shell=True, check=True, env=_clean_env_for_cmd())
            else:
                install_script = target_dir / "install.sh"
                subprocess.run(
                    ["bash", str(install_script), "esp32s3"], check=True,
                )
        except subprocess.CalledProcessError as e:
            error(f"Failed to install ESP-IDF tools: {e}")
            return None

        success(f"ESP-IDF {version} installed successfully!")
        return target_dir


class Installer:
    """Main installer"""

    def __init__(self):
        self.root = Path(__file__).parent.parent.resolve()
        self.config_dir = self.root / ".sf"
        self.config_file = self.config_dir / "config.toml"

    def _is_sfcli_installed(self, idf_path: Path) -> bool:
        """Check if sfcli is installed in ESP-IDF Python environment.
        sfcliがESP-IDF Python環境にインストール済みか確認"""
        if sys.platform == "win32":
            # Use venv Python directly
            # venv Pythonを直接使用
            venv_python = _find_idf_python(idf_path)
            if venv_python:
                try:
                    result = subprocess.run(
                        [str(venv_python), "-c", "import sfcli"],
                        capture_output=True, text=True,
                    )
                    return result.returncode == 0
                except Exception:
                    return False
            return False
        else:
            env_prefix = _build_idf_env_command(idf_path)
            inner = f'{env_prefix} && python -c "import sfcli"'
            try:
                result = subprocess.run(
                    ["bash", "-c", inner],
                    capture_output=True,
                    text=True,
                )
                return result.returncode == 0
            except Exception:
                return False

    def run(
        self,
        idf_path: Optional[Path] = None,
        skip_deps: bool = False,
        minimal: bool = False,
        force: bool = False,
    ) -> int:
        """Run installation"""

        # Step 1: Find or install ESP-IDF
        header("Step 1/3: ESP-IDF")

        if idf_path:
            # User specified path
            if not ESPIDFDetector._is_valid_idf(idf_path):
                error(f"Invalid ESP-IDF path: {idf_path}")
                return 1
            version = ESPIDFDetector._get_version(idf_path)
            info(f"Using specified ESP-IDF: {idf_path} ({version})")
        else:
            # Detect ESP-IDF installations
            info("Checking ESP-IDF installations...")
            installations = ESPIDFDetector.find_all()

            if not installations:
                # No ESP-IDF found, offer to install
                warn("No ESP-IDF installation found.")
                print()

                choices = [
                    f"Install ESP-IDF {ESPIDFInstaller.DEFAULT_VERSION} (recommended)",
                    "Specify custom path",
                    "Cancel",
                ]
                choice = prompt_choice("ESP-IDF is required for StampFly development.", choices)

                if choice == 1:
                    idf_path = ESPIDFInstaller.install()
                    if not idf_path:
                        return 1
                elif choice == 2:
                    path_str = prompt("Enter ESP-IDF path")
                    idf_path = Path(path_str).expanduser().resolve()
                    if not ESPIDFDetector._is_valid_idf(idf_path):
                        error(f"Invalid ESP-IDF path: {idf_path}")
                        return 1
                else:
                    info("Installation cancelled.")
                    return 1

                version = ESPIDFDetector._get_version(idf_path)

            elif len(installations) == 1:
                # Single installation found
                idf_path, version = installations[0]
                info(f"Found ESP-IDF {version} at {idf_path}")

                response = prompt("Use this installation? [Y/n]", "Y")
                if response.lower() not in ("y", "yes", ""):
                    info("Installation cancelled.")
                    return 1

            else:
                # Multiple installations found
                choices = [f"{ver:8} {path}" for path, ver in installations]
                choices.append("Install new ESP-IDF")

                choice = prompt_choice(
                    f"Found {len(installations)} ESP-IDF installations:",
                    choices
                )

                if choice <= len(installations):
                    idf_path, version = installations[choice - 1]
                else:
                    idf_path = ESPIDFInstaller.install()
                    if not idf_path:
                        return 1
                    version = ESPIDFDetector._get_version(idf_path)

        success(f"Using ESP-IDF {version}")
        print()

        # Step 2: Get ESP-IDF Python environment
        header("Step 2/3: Python Environment")

        info("Getting ESP-IDF Python environment...")
        idf_python = ESPIDFDetector.get_python_env(idf_path)

        if not idf_python:
            error("Failed to get ESP-IDF Python environment.")
            error("Please ensure ESP-IDF is properly installed:")
            error(f"  cd {idf_path}")
            error("  ./install.sh")
            if is_wsl():
                error("")
                error("WSL2 detected: Windows Python (pyenv-win) may be interfering.")
                error("Check that /mnt/c/... paths are not providing python.")
            return 1

        success(f"ESP-IDF Python: {idf_python}")
        print()

        # Step 3: Install sfcli
        header("Step 3/3: StampFly CLI")

        if not skip_deps:
            # Probe: check if already installed (skip if not --force)
            # プローブ: インストール済みか確認（--forceでなければスキップ）
            if not force and self._is_sfcli_installed(idf_path):
                success("sfcli is already installed, skipping (use --force to reinstall)")
            else:
                if force:
                    info("Force reinstalling...")

                if minimal:
                    info("Installing sfcli with core dependencies...")
                    pip_args = ["install"]
                    if force:
                        pip_args.append("--force-reinstall")
                    pip_args.extend(["-e", str(self.root)])
                    rc = _run_in_idf_env(idf_path, pip_args)
                    if rc != 0:
                        error("Failed to install sfcli")
                        return 1

                    info("Simulator dependencies skipped. Install later with: sf setup sim")
                else:
                    # Install full dependencies including simulator
                    # シミュレータを含むすべての依存関係をインストール
                    requirements = self.root / "requirements.txt"
                    if requirements.exists():
                        info("Installing all dependencies...")
                        pip_args = ["install", "-r", str(requirements)]
                        rc = _run_in_idf_env(idf_path, pip_args)
                        if rc != 0:
                            warn("Some dependencies may have failed to install")
                            warn("You can install simulator dependencies later with: sf setup sim")

                    # Install sfcli in editable mode
                    # sfcliを開発モードでインストール
                    info("Installing sfcli...")
                    pip_args = ["install"]
                    if force:
                        pip_args.append("--force-reinstall")
                    pip_args.extend(["-e", str(self.root)])
                    rc = _run_in_idf_env(idf_path, pip_args)
                    if rc != 0:
                        error("Failed to install sfcli")
                        return 1

        success("StampFly CLI installed!")
        print()

        # Fix setuptools: ESP-IDF install.sh may upgrade to 82+ which removes
        # pkg_resources, breaking vpython. Pin back to <81.
        # setuptools修正: ESP-IDFが82+にアップグレードするとpkg_resourcesが
        # 削除されvpythonが壊れる。<81に固定する。
        self._fix_setuptools(idf_path)

        # Check hidapi native library for joystick support
        # ジョイスティック用のhidapiネイティブライブラリを確認
        self._check_hidapi()

        # Install udev rules on Linux (for USB HID access without root)
        # Linux: udevルールをインストール（root不要でUSB HIDアクセス）
        if sys.platform == "linux" and not is_wsl():
            self._install_udev_rules()

        # Save configuration
        self._save_config(idf_path)

        # Show completion message
        header("Installation Complete!")

        print("To start using StampFly CLI:")
        print()
        if sys.platform == "win32":
            setup_env = self.root / "setup_env.bat"
            print(f"  {setup_env}")
        else:
            setup_env = self.root / "setup_env.sh"
            print(f"  source {setup_env}")
        print()
        print("Then run:")
        print("  sf --help          # Show all commands")
        print("  sf doctor          # Check environment")
        print("  sf sim run         # Run VPython simulator")
        print("  sf build vehicle   # Build firmware")
        print()

        # WSL2-specific guidance
        # WSL2固有の案内
        if is_wsl():
            print("WSL2 Notes:")
            print("  - USB device access requires usbipd-win on Windows side")
            print("    Install: winget install usbipd")
            print("    See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
            print("  - Run 'sf doctor' to check WSL2-specific configuration")
            print()

        return 0

    def clean(self, idf_path: Optional[Path] = None) -> int:
        """Clean install: remove config and sfcli, then reinstall.
        クリーンインストール: 設定とsfcliを削除後、再インストール"""
        header("Cleaning StampFly installation...")

        # Find ESP-IDF path from config or argument
        # 設定またはコマンドライン引数からESP-IDFパスを取得
        resolved_idf_path = idf_path
        if not resolved_idf_path and self.config_file.exists():
            for line in self.config_file.read_text().split('\n'):
                if line.startswith('path = "'):
                    resolved_idf_path = Path(line.split('"')[1])
                    break

        # Uninstall sfcli from ESP-IDF Python environment
        # ESP-IDFのPython環境からsfcliをアンインストール
        if resolved_idf_path and ESPIDFDetector._is_valid_idf(resolved_idf_path):
            info("Uninstalling sfcli...")
            _run_in_idf_env(resolved_idf_path, ["uninstall", "-y", "stampfly-ecosystem"])

        # Remove config file
        # 設定ファイルを削除
        if self.config_file.exists():
            self.config_file.unlink()
            info("Removed configuration file")

        success("Clean complete. Re-running installer...")
        print()

        # Re-run installation
        return self.run(idf_path=idf_path, force=True)

    def _install_udev_rules(self) -> None:
        """Install udev rules for StampFly USB devices on Linux.
        Linux用のudevルールをインストール"""
        rules_src = self.root / "tools" / "udev" / "99-stampfly.rules"
        rules_dst = Path("/etc/udev/rules.d/99-stampfly.rules")

        if not rules_src.exists():
            warn("udev rules file not found, skipping")
            return

        if rules_dst.exists():
            # Check if already up to date
            # 既にインストール済みか確認
            try:
                if rules_dst.read_text() == rules_src.read_text():
                    success("udev rules already installed")
                    return
            except PermissionError:
                pass

        info("Installing udev rules for USB device access...")
        print("  sudo permission is required to copy rules to /etc/udev/rules.d/")
        print("  USB デバイスアクセスに sudo 権限が必要です")
        print()

        try:
            rc = subprocess.run(
                ["sudo", "cp", str(rules_src), str(rules_dst)],
            ).returncode
            if rc != 0:
                warn("Failed to install udev rules. Install manually:")
                warn(f"  sudo cp {rules_src} /etc/udev/rules.d/")
                warn("  sudo udevadm control --reload-rules && sudo udevadm trigger")
                return

            subprocess.run(
                ["sudo", "udevadm", "control", "--reload-rules"],
            )
            subprocess.run(
                ["sudo", "udevadm", "trigger"],
            )
            success("udev rules installed (reconnect USB devices to apply)")
        except Exception as e:
            warn(f"Failed to install udev rules: {e}")
            warn("Install manually:")
            warn(f"  sudo cp {rules_src} /etc/udev/rules.d/")
            warn("  sudo udevadm control --reload-rules && sudo udevadm trigger")

    def _fix_setuptools(self, idf_path: Path) -> None:
        """Pin setuptools<81 to keep pkg_resources for vpython.
        vpythonのためにpkg_resourcesを維持するようsetuptools<81に固定"""
        info("Checking setuptools version (vpython compatibility)...")
        rc = _run_in_idf_env(idf_path, ["install", "setuptools>=68.0,<81"])
        if rc == 0:
            success("setuptools pinned to <81 (pkg_resources available)")
        else:
            warn("Failed to pin setuptools. vpython may not work.")
            warn("Manual fix: pip install 'setuptools>=68.0,<81'")

    def _check_hidapi(self) -> None:
        """Check if hidapi native library is available (needed for joystick).
        ジョイスティック用のhidapiネイティブライブラリの存在を確認"""
        if sys.platform == "darwin":
            # macOS: check for libhidapi via Homebrew
            # macOS: Homebrewのlibhidapiを確認
            brew_lib = Path("/opt/homebrew/lib/libhidapi.dylib")
            brew_lib_intel = Path("/usr/local/lib/libhidapi.dylib")
            if brew_lib.exists() or brew_lib_intel.exists():
                success("hidapi native library found (Homebrew)")
            else:
                warn("hidapi native library not found (needed for joystick)")
                warn("Install with: brew install hidapi")
        elif sys.platform == "linux":
            # Linux: check for libhidapi-hidraw.so
            # Linux: libhidapi-hidraw.soを確認
            import ctypes.util
            lib = ctypes.util.find_library("hidapi-hidraw") or ctypes.util.find_library("hidapi-libusb")
            if lib:
                success("hidapi native library found")
            else:
                warn("hidapi native library not found (needed for joystick)")
                warn("Install with: sudo apt install libhidapi-dev")

    def _save_config(self, idf_path: Path) -> None:
        """Save configuration file"""
        self.config_dir.mkdir(parents=True, exist_ok=True)

        version = ESPIDFDetector._get_version(idf_path)

        config_content = f'''# StampFly Ecosystem Configuration
# Auto-generated by installer

[esp_idf]
path = "{idf_path}"
version = "{version}"

[project]
default_target = "vehicle"
'''

        self.config_file.write_text(config_content)
        info(f"Configuration saved to {self.config_file}")

    def uninstall(self) -> int:
        """Uninstall sfcli from ESP-IDF environment"""
        header("StampFly Ecosystem Uninstaller")

        # Load config to find ESP-IDF path
        if not self.config_file.exists():
            error("No configuration found. Nothing to uninstall.")
            return 1

        # Parse config (simple TOML parsing)
        idf_path = None
        for line in self.config_file.read_text().split('\n'):
            if line.startswith('path = "'):
                idf_path = Path(line.split('"')[1])
                break

        if not idf_path:
            error("Could not determine ESP-IDF path from config.")
            return 1

        info(f"ESP-IDF path: {idf_path}")

        # Uninstall sfcli
        info("Uninstalling sfcli...")
        _run_in_idf_env(idf_path, ["uninstall", "-y", "stampfly-ecosystem"])

        # Remove config
        if self.config_file.exists():
            self.config_file.unlink()
            info("Removed configuration file")

        success("Uninstall complete!")
        return 0


def main() -> int:
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description="StampFly Ecosystem Installer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--idf-path",
        type=Path,
        help="Specify ESP-IDF path",
    )
    parser.add_argument(
        "--skip-deps",
        action="store_true",
        help="Skip dependency installation",
    )
    parser.add_argument(
        "--minimal",
        action="store_true",
        help="Install minimal dependencies (skip simulator)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable colored output",
    )
    parser.add_argument(
        "--uninstall",
        action="store_true",
        help="Uninstall sfcli",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Clean install (remove config and sfcli, then reinstall)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force reinstall all steps (skip probe checks)",
    )

    args = parser.parse_args()

    # Disable colors if requested or not a TTY
    if args.no_color or not sys.stdout.isatty():
        Colors.disable()

    installer = Installer()

    if args.uninstall:
        return installer.uninstall()
    elif args.clean:
        return installer.clean(idf_path=args.idf_path)
    else:
        return installer.run(
            idf_path=args.idf_path,
            skip_deps=args.skip_deps,
            minimal=args.minimal,
            force=args.force,
        )


if __name__ == "__main__":
    sys.exit(main())
