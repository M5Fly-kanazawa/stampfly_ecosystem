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


def _idf_tools_path_candidates() -> list[Path]:
    """Return likely IDF_TOOLS_PATH locations in priority order.
    IDF_TOOLS_PATH の候補を優先順に返す"""
    tools_path = os.environ.get("IDF_TOOLS_PATH")
    if tools_path:
        return [Path(tools_path)]
    if sys.platform == "win32":
        return [Path("C:/Espressif"), Path.home() / ".espressif"]
    return [Path.home() / ".espressif"]


def _idf_major_minor(idf_path: Path) -> Optional[str]:
    """Extract MAJOR.MINOR from ESP-IDF version (e.g. v5.5.2 -> '5.5').
    ESP-IDF バージョンから MAJOR.MINOR を抽出

    Tries version.txt / git describe first, falls back to parsing the
    directory name (esp-idf-v5.4 -> 5.4) so multi-version setups can be
    distinguished even when one of the trees has no .git/version.txt.
    """
    import re
    version = ESPIDFDetector._get_version(idf_path)
    if version.startswith("v"):
        parts = version.lstrip("v").split(".")
        if len(parts) >= 2:
            return f"{parts[0]}.{parts[1]}"
    # Fallback: parse directory name e.g. esp-idf-v5.4
    # フォールバック: ディレクトリ名から抽出 (esp-idf-v5.4 等)
    m = re.search(r"v(\d+)\.(\d+)", idf_path.name)
    if m:
        return f"{m.group(1)}.{m.group(2)}"
    return None


def _find_idf_python(idf_path: Path) -> Optional[Path]:
    """Find the ESP-IDF venv Python that matches `idf_path`.
    `idf_path` に対応する ESP-IDF venv の Python を探す

    ESP-IDF names its venv directories ``idf<MAJOR.MINOR>_py<MAJOR.MINOR>_env``.
    When multiple ESP-IDF versions coexist (e.g. v5.4 and v5.5), the user may
    pick one in the installer prompt — we MUST return the venv that matches
    that specific ESP-IDF version, not just any venv that happens to be
    present. Otherwise pip will silently install into the wrong venv.

    複数バージョン共存時、選択した ESP-IDF と無関係な venv に install されない
    よう、idf_path のバージョンに一致する venv のみ返す。マッチが見つから
    なければ None (ESP-IDF 未インストール、未活性化、または命名規約違反)。
    """
    target = _idf_major_minor(idf_path)
    if not target:
        # Without a confirmed version we cannot pick a venv safely; a wrong
        # guess would silently install into the wrong ESP-IDF's venv.
        # バージョン不明時に推測で venv を返すと別 ESP-IDF の venv に
        # 誤って install してしまうため、fail closed する
        return None
    prefix = f"idf{target}_py"

    bin_subdir = "Scripts" if sys.platform == "win32" else "bin"
    python_name = "python.exe" if sys.platform == "win32" else "python"

    for base in _idf_tools_path_candidates():
        python_env_dir = base / "python_env"
        if not python_env_dir.exists():
            continue
        try:
            entries = list(python_env_dir.iterdir())
        except OSError:
            continue
        # Pick newest Python (sort descending) that matches the ESP-IDF prefix
        # ESP-IDF バージョンに一致する venv のうち Python 版が一番新しいものを選ぶ
        candidates = [
            d for d in entries
            if d.is_dir() and d.name.startswith(prefix) and d.name.endswith("_env")
        ]
        for venv_dir in sorted(candidates, reverse=True):
            python_exe = venv_dir / bin_subdir / python_name
            if python_exe.exists():
                return python_exe

    # Strict match failed — caller can treat None as "venv not built yet"
    # 厳密マッチに失敗 — venv 未作成と同じ扱い
    return None


def _find_idf_constraint_file(idf_path: Path) -> Optional[Path]:
    """Find ESP-IDF's pip constraint file for the given installation.
    ESP-IDF のインストールに対応する pip constraint ファイルを探す

    ESP-IDF writes a version-specific constraint file like
    `<IDF_TOOLS_PATH>/espidf.constraints.v<major.minor>.txt` during install.
    Passing this via `pip install -c <file>` keeps user-installed packages
    inside the version ranges ESP-IDF's own scripts validate against, so
    `source export.sh`'s activate_venv.py check never fails because a
    transitive dep (e.g. pyparsing pulled by matplotlib) was upgraded past
    ESP-IDF's allowed range.

    ESP-IDF はインストール時に
    `<IDF_TOOLS_PATH>/espidf.constraints.v<メジャー.マイナー>.txt`
    というバージョン固有の constraint ファイルを書き出す。これを
    `pip install -c <file>` で渡せば、ユーザ追加パッケージが ESP-IDF の
    許容範囲外に押し出されることを防げる(例: matplotlib が引き込む
    pyparsing が ESP-IDF の <3.3 範囲を超えて upgrade される問題)。
    """
    # Try exact match by ESP-IDF version (vMAJOR.MINOR)
    # ESP-IDF バージョンに正確にマッチするファイルを探す
    version = ESPIDFDetector._get_version(idf_path)
    major_minor: Optional[str] = None
    if version.startswith("v"):
        parts = version.split(".")
        if len(parts) >= 2:
            major_minor = ".".join(parts[:2])  # e.g. "v5.5"

    for base in _idf_tools_path_candidates():
        if not base.exists():
            continue
        if major_minor:
            specific = base / f"espidf.constraints.{major_minor}.txt"
            if specific.exists():
                return specific
        # Fallback: pick newest matching glob in this base
        # フォールバック: globで一番新しいものを選ぶ
        try:
            matches = sorted(base.glob("espidf.constraints.v*.txt"), reverse=True)
        except OSError:
            continue
        if matches:
            return matches[0]
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

    Strategy: always prefer calling the venv's python executable directly.
    Sourcing export.sh is unreliable because: (1) ESP-IDF's activate_venv.py
    can fail silently (exit non-zero internally but `source` returns 0), and
    (2) pyenv shims or other PATH-priority tools can intercept `python` even
    after a successful activation, causing pip to install into the WRONG
    interpreter. Calling the venv python by absolute path bypasses both.

    戦略: 常に venv の python を絶対パスで直接呼び出す。export.sh の
    source は信頼できない: 内部で失敗しても `source` 自体は 0 を返すこと
    があり、また pyenv shim 等が PATH 優先で `python` を奪うことがある。

    NOTE: Callers must pass pip_args as plain (unquoted) strings. This
    function applies shell-appropriate quoting internally for fallback paths.
    """
    # Auto-inject ESP-IDF's constraint file for `install` commands so that
    # transitive deps (e.g. pyparsing pulled by matplotlib) cannot be
    # upgraded past ESP-IDF's pinned ranges. Without this, every
    # pip install --force-reinstall -e . will resolve dependencies fresh
    # from pyproject.toml and re-break the venv.
    # install サブコマンドには ESP-IDF の constraint ファイルを自動付与し、
    # 推移的依存が ESP-IDF の許容範囲外に upgrade されることを防ぐ
    if pip_args and pip_args[0] == "install":
        constraint = _find_idf_constraint_file(idf_path)
        if constraint and "-c" not in pip_args and "--constraint" not in pip_args:
            pip_args = [pip_args[0], "-c", str(constraint)] + pip_args[1:]

    # Primary path: call venv python by absolute path (no shell, no PATH)
    # 主経路: venv の python を絶対パスで直接呼ぶ (シェル/PATH に依存しない)
    venv_python = _find_idf_python(idf_path)
    if venv_python:
        cmd = [str(venv_python), "-m", "pip"] + pip_args
        # Strip env vars that would steer pip toward a user venv / conda env
        # instead of the ESP-IDF venv we are explicitly targeting. pip respects
        # VIRTUAL_ENV/CONDA_PREFIX for warnings and PYTHONHOME/PYTHONPATH for
        # interpreter setup; leaving them set can cause subtle missteps even
        # when the python binary itself is the right one.
        # 既に user venv / conda が activate されていると VIRTUAL_ENV 等が
        # 継承され pip が誤誘導されうるので、ESP-IDF venv に絞った subprocess
        # ではこれらを必ず除去する
        env = os.environ.copy()
        for var in ("VIRTUAL_ENV", "CONDA_PREFIX", "CONDA_DEFAULT_ENV",
                    "CONDA_SHLVL", "PYTHONHOME", "PYTHONPATH",
                    "PIP_REQUIRE_VIRTUALENV", "PIP_TARGET", "PIP_PREFIX",
                    "PIP_USER"):
            env.pop(var, None)
        return subprocess.run(cmd, env=env).returncode

    # Fallback: venv not yet created (e.g. mid-install). Source export script.
    # フォールバック: venv 未作成時のみ export スクリプトを source する
    if sys.platform == "win32":
        export_script = idf_path / "export.bat"
        escaped = subprocess.list2cmdline(pip_args)
        cmd = f'call "{export_script}" && python -m pip {escaped}'
        return subprocess.run(cmd, shell=True, env=_clean_env_for_cmd()).returncode
    else:
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
        ESP-IDFインストールのPython環境を取得

        Locates the venv Python by absolute path. Sourcing export.sh and
        running 'which python' is unreliable: if activate_venv.py fails
        (e.g. constraint violation), `source` still exits 0 and `which
        python` returns pyenv's shim instead of the venv interpreter.
        絶対パスで venv の python を特定する。`source export.sh` 経由は
        activate_venv.py が失敗しても `source` 自体は 0 を返すため、
        pyenv shim 等を誤検出する危険がある。
        """
        venv_python = _find_idf_python(idf_path)
        if venv_python:
            return venv_python

        # Fallback only when venv has not been created yet
        # venv 未作成時のみフォールバック
        if sys.platform != "win32":
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
        """Check if sfcli is installed AND importable in the ESP-IDF venv.
        sfcli が ESP-IDF venv で実際に import できるか確認

        We deliberately do this with the venv python by absolute path (not via
        a sourced export.sh) so the check answers "is sfcli importable from
        this specific venv" rather than "is sfcli importable from whatever
        python happens to be on PATH after activation." The latter has been
        a source of false positives when pyenv shims override the venv.
        絶対パスで venv python を呼び、activate 経由ではなく直接 import を
        試す。PATH 経由だと pyenv 等が誤誘導して false positive になる。
        """
        venv_python = _find_idf_python(idf_path)
        if not venv_python:
            return False
        try:
            result = subprocess.run(
                [str(venv_python), "-c", "import sfcli"],
                capture_output=True, text=True,
            )
            return result.returncode == 0
        except Exception:
            return False

    # Alias to make the intent explicit at the post-install verification site.
    # 同じチェックを post-install 検証用に別名で公開しておく
    _verify_sfcli_import = _is_sfcli_installed

    def _diagnose_broken_install(self, idf_path: Path) -> None:
        """Print actionable diagnostic info when sfcli is unimportable
        despite pip having reported a successful install.
        pip が成功と報告したのに sfcli が import できない状態の診断情報を出す"""
        venv_python = _find_idf_python(idf_path)
        if not venv_python:
            error(f"  No ESP-IDF venv matching {idf_path} was found.")
            error("  Run ESP-IDF's install.sh first, then re-run this installer.")
            return

        site_pkgs = venv_python.parent.parent / "lib"
        # Find sfcli-related artifacts
        # sfcli 関連の成果物を列挙
        try:
            artifacts = []
            for p in site_pkgs.rglob("__editable__.stampfly*"):
                artifacts.append(("pth", p))
            for p in site_pkgs.rglob("stampfly_ecosystem-*.dist-info"):
                artifacts.append(("dist-info", p))
            for p in (venv_python.parent / "sf",):
                if p.exists():
                    artifacts.append(("shim", p))
        except OSError:
            artifacts = []

        error("  Diagnostic info:")
        error(f"    venv python : {venv_python}")
        error(f"    repo root   : {self.root}")
        kinds = {kind for kind, _ in artifacts}
        if not artifacts:
            error("    No sfcli artifacts found in venv.")
            error("    pip likely installed into a different python (e.g. a")
            error("    pre-activated user venv / conda env).")
        else:
            for kind, p in artifacts:
                error(f"    {kind:10s}: {p}")
            # Diagnose the asymmetry between what pip recorded and what
            # actually resolves at import time
            # pip の記録と実際の import 解決の食い違いを診断
            if "pth" not in kinds and ("dist-info" in kinds or "shim" in kinds):
                error("    >>> No editable .pth file. pip records the package as")
                error("    >>> installed (dist-info exists) but sys.path will not")
                error("    >>> include the source dir → import fails.")
            for kind, p in artifacts:
                if kind == "pth":
                    try:
                        target = p.read_text().strip().splitlines()[0]
                        target_p = Path(target)
                        if not target_p.exists():
                            error(f"    >>> .pth points to NON-EXISTENT path: {target}")
                            error("    >>> The repo was moved/renamed, or installed from")
                            error("    >>> a symlinked path that no longer resolves.")
                    except (OSError, IndexError):
                        pass
        error("")
        error("  Recovery: ensure no user venv / conda env is active, then re-run")
        error("    deactivate 2>/dev/null; conda deactivate 2>/dev/null; ./install.sh --clean")

    def _warn_if_env_preactivated(self) -> None:
        """Detect pre-activated user venv / conda env and warn the user.
        ユーザの venv / conda env が事前 activate されていたら警告

        Even with our subprocess env-sanitization, an active VIRTUAL_ENV
        or CONDA_PREFIX is a strong signal the user expected pip to land
        somewhere other than the ESP-IDF venv. Surface it loudly.
        subprocess 側で環境変数を消すように直したが、ユーザの意図と齟齬が
        ある可能性が高いので、警告だけは出しておく。
        """
        venv = os.environ.get("VIRTUAL_ENV")
        conda = os.environ.get("CONDA_PREFIX")
        if not venv and not conda:
            return
        warn("An active Python environment was detected:")
        if venv:
            warn(f"  VIRTUAL_ENV  = {venv}")
        if conda:
            warn(f"  CONDA_PREFIX = {conda}")
        warn("The installer will still target the ESP-IDF venv directly, but")
        warn("you should consider deactivating before running this script:")
        if venv:
            warn("  deactivate")
        if conda:
            warn("  conda deactivate")
        print()

    def run(
        self,
        idf_path: Optional[Path] = None,
        skip_deps: bool = False,
        minimal: bool = False,
        force: bool = False,
    ) -> int:
        """Run installation"""

        # Surface pre-activated venv / conda env early so the user can
        # course-correct before pip operations begin.
        # 事前 activate 済みの環境は最初に警告して、pip が動き始める前に
        # ユーザが軌道修正できるようにする
        self._warn_if_env_preactivated()

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

        # Post-install verification: actually try to import sfcli using the
        # ESP-IDF venv's python. pip can report success while leaving the
        # editable install in a half-broken state (e.g. .pth pointing to a
        # path that no longer exists, or the install having landed in a
        # different venv that happened to be on PATH). The shim alone is not
        # enough — we must verify the module actually loads.
        # インストール後検証: ESP-IDF venv の python で実際に sfcli を
        # import できることを確認する。pip が「成功」と表示しても editable
        # の .pth が壊れていたり、別 venv に着地したりすることがあるため、
        # shim の有無だけでなく実際の import 成否を確認しなければ意味がない
        if not self._verify_sfcli_import(idf_path):
            error("sfcli was reported as installed but cannot be imported "
                  "from the ESP-IDF venv.")
            self._diagnose_broken_install(idf_path)
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
