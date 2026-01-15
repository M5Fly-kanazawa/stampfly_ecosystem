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
    --uninstall        Remove sfcli from ESP-IDF environment
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path
from typing import Optional, List, Tuple

# Ensure we're running Python 3.10+
if sys.version_info < (3, 10):
    print(f"Error: Python 3.10+ required, found {sys.version_info.major}.{sys.version_info.minor}")
    sys.exit(1)


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


class ESPIDFDetector:
    """Detect ESP-IDF installations"""

    COMMON_PATHS = [
        Path.home() / "esp" / "esp-idf",
        Path.home() / "esp" / "esp-idf-v5.4",
        Path.home() / "esp" / "esp-idf-v5.3",
        Path.home() / "esp" / "esp-idf-v5.2",
        Path.home() / "esp" / "esp-idf-v5.1",
        Path.home() / ".espressif" / "esp-idf",
        Path("/opt/esp-idf"),
    ]

    # Windows paths
    if sys.platform == "win32":
        COMMON_PATHS.extend([
            Path("C:/Espressif/frameworks/esp-idf"),
            Path("C:/esp-idf"),
        ])

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
        """Check if path is a valid ESP-IDF installation"""
        export_script = path / ("export.bat" if sys.platform == "win32" else "export.sh")
        return export_script.exists()

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
        """Get the Python environment for an ESP-IDF installation"""
        # Source export.sh and get the Python path
        if sys.platform == "win32":
            export_script = idf_path / "export.bat"
            cmd = f'cmd /c "{export_script}" && where python'
        else:
            export_script = idf_path / "export.sh"
            cmd = f'bash -c \'source "{export_script}" > /dev/null 2>&1 && which python\''

        try:
            result = subprocess.run(
                cmd,
                shell=True,
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
    """Install ESP-IDF"""

    DEFAULT_VERSION = "v5.4"
    REPO_URL = "https://github.com/espressif/esp-idf.git"

    @classmethod
    def install(cls, target_dir: Optional[Path] = None, version: str = DEFAULT_VERSION) -> Optional[Path]:
        """Install ESP-IDF"""
        if target_dir is None:
            target_dir = Path.home() / "esp" / "esp-idf"

        # Create parent directory
        target_dir.parent.mkdir(parents=True, exist_ok=True)

        info(f"Installing ESP-IDF {version} to {target_dir}...")
        print()

        # Clone repository
        info("Cloning ESP-IDF repository...")
        try:
            subprocess.run(
                [
                    "git", "clone",
                    "--branch", version,
                    "--depth", "1",
                    "--recursive",
                    cls.REPO_URL,
                    str(target_dir),
                ],
                check=True,
            )
        except subprocess.CalledProcessError as e:
            error(f"Failed to clone ESP-IDF: {e}")
            return None

        # Run install script
        info("Installing ESP-IDF tools (this may take a while)...")
        if sys.platform == "win32":
            install_script = target_dir / "install.bat"
            cmd = [str(install_script), "esp32s3"]
        else:
            install_script = target_dir / "install.sh"
            cmd = ["bash", str(install_script), "esp32s3"]

        try:
            subprocess.run(cmd, check=True)
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

    def run(self, idf_path: Optional[Path] = None, skip_deps: bool = False) -> int:
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
            return 1

        success(f"ESP-IDF Python: {idf_python}")
        print()

        # Step 3: Install sfcli
        header("Step 3/3: StampFly CLI")

        if not skip_deps:
            info("Installing dependencies...")

            # Get pip from ESP-IDF environment
            if sys.platform == "win32":
                export_script = idf_path / "export.bat"
                pip_cmd = f'cmd /c "{export_script}" && python -m pip'
            else:
                export_script = idf_path / "export.sh"
                pip_cmd = f'bash -c \'source "{export_script}" > /dev/null 2>&1 && python -m pip'

            # Install dependencies
            requirements = self.root / "requirements.txt"
            if requirements.exists():
                if sys.platform == "win32":
                    cmd = f'{pip_cmd} install -r "{requirements}"'
                else:
                    cmd = f'{pip_cmd} install -r "{requirements}"\''

                result = subprocess.run(cmd, shell=True)
                if result.returncode != 0:
                    warn("Some dependencies may have failed to install")

            # Install sfcli in editable mode
            info("Installing sfcli...")
            if sys.platform == "win32":
                cmd = f'{pip_cmd} install -e "{self.root}"'
            else:
                cmd = f'{pip_cmd} install -e "{self.root}"\''

            result = subprocess.run(cmd, shell=True)
            if result.returncode != 0:
                error("Failed to install sfcli")
                return 1

        success("StampFly CLI installed!")
        print()

        # Save configuration
        self._save_config(idf_path)

        # Show completion message
        header("Installation Complete!")

        print("To start using StampFly CLI:")
        print()
        if sys.platform == "win32":
            print(f"  {idf_path}\\export.bat")
        else:
            print(f"  source {idf_path}/export.sh")
        print()
        print("Then run:")
        print("  sf --help")
        print("  sf doctor")
        print()

        return 0

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
        if sys.platform == "win32":
            export_script = idf_path / "export.bat"
            cmd = f'cmd /c "{export_script}" && python -m pip uninstall -y stampfly-ecosystem'
        else:
            export_script = idf_path / "export.sh"
            cmd = f'bash -c \'source "{export_script}" > /dev/null 2>&1 && python -m pip uninstall -y stampfly-ecosystem\''

        subprocess.run(cmd, shell=True)

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
        "--no-color",
        action="store_true",
        help="Disable colored output",
    )
    parser.add_argument(
        "--uninstall",
        action="store_true",
        help="Uninstall sfcli",
    )

    args = parser.parse_args()

    # Disable colors if requested or not a TTY
    if args.no_color or not sys.stdout.isatty():
        Colors.disable()

    installer = Installer()

    if args.uninstall:
        return installer.uninstall()
    else:
        return installer.run(
            idf_path=args.idf_path,
            skip_deps=args.skip_deps,
        )


if __name__ == "__main__":
    sys.exit(main())
