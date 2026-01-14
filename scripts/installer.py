#!/usr/bin/env python3
"""
StampFly Ecosystem Installer

Creates a self-contained development environment.
自己完結型の開発環境を構築します。

Usage:
    python scripts/installer.py [options]

Options:
    --no-venv       Skip virtual environment creation
    --no-deps       Skip dependency installation
    --no-shell      Skip shell configuration
    --uninstall     Remove virtual environment and configuration
"""

import os
import sys
import subprocess
import platform as sys_platform
import shutil
from pathlib import Path
from typing import Optional


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
        cls.RESET = ""
        cls.RED = ""
        cls.GREEN = ""
        cls.YELLOW = ""
        cls.BLUE = ""
        cls.CYAN = ""
        cls.BOLD = ""


def info(msg: str) -> None:
    print(f"{Colors.BLUE}[INFO]{Colors.RESET} {msg}")


def success(msg: str) -> None:
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}")


def warning(msg: str) -> None:
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}")


def error(msg: str) -> None:
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {msg}", file=sys.stderr)


def header(title: str) -> None:
    line = "=" * 60
    print(f"{Colors.CYAN}{line}{Colors.RESET}")
    print(f"{Colors.BOLD} {title}{Colors.RESET}")
    print(f"{Colors.CYAN}{line}{Colors.RESET}")


class Installer:
    """StampFly Ecosystem Installer"""

    def __init__(self):
        self.root = Path(__file__).parent.parent.resolve()
        self.venv_dir = self.root / ".venv"
        self.bin_dir = self.root / "bin"
        self.is_windows = sys_platform.system() == "Windows"

    def run(
        self,
        skip_venv: bool = False,
        skip_deps: bool = False,
        skip_shell: bool = False,
    ) -> int:
        """Run installation"""
        header("StampFly Ecosystem Installer")
        print()

        info(f"Root directory: {self.root}")
        info(f"Platform: {sys_platform.system()}")
        info(f"Python: {sys.version.split()[0]}")
        print()

        # Check Python version
        if sys.version_info < (3, 10):
            error(f"Python 3.10+ required, found {sys.version_info.major}.{sys.version_info.minor}")
            return 1

        # Create virtual environment
        if not skip_venv:
            if not self._create_venv():
                return 1
        else:
            info("Skipping virtual environment creation")

        # Install dependencies
        if not skip_deps:
            if not self._install_deps():
                return 1
        else:
            info("Skipping dependency installation")

        # Install sfcli as editable package
        if not skip_deps:
            if not self._install_sfcli():
                return 1

        # Configure shell
        if not skip_shell:
            self._configure_shell()
        else:
            info("Skipping shell configuration")

        # Print success message
        print()
        header("Installation Complete")
        print()
        success("StampFly Ecosystem has been installed!")
        print()
        print("To activate the environment:")
        if self.is_windows:
            print(f"    {self.root}\\scripts\\activate.bat")
        else:
            print(f"    source {self.root}/scripts/activate.sh")
        print()
        print("Then run:")
        print("    sf --help")
        print()

        return 0

    def _create_venv(self) -> bool:
        """Create Python virtual environment"""
        info("Creating virtual environment...")

        if self.venv_dir.exists():
            warning(f"Virtual environment already exists: {self.venv_dir}")
            return True

        try:
            subprocess.run(
                [sys.executable, "-m", "venv", str(self.venv_dir)],
                check=True,
            )
            success(f"Created: {self.venv_dir}")
            return True
        except subprocess.CalledProcessError as e:
            error(f"Failed to create virtual environment: {e}")
            return False

    def _get_pip(self) -> str:
        """Get path to pip in virtual environment"""
        if self.is_windows:
            return str(self.venv_dir / "Scripts" / "pip.exe")
        else:
            return str(self.venv_dir / "bin" / "pip")

    def _get_python(self) -> str:
        """Get path to python in virtual environment"""
        if self.is_windows:
            return str(self.venv_dir / "Scripts" / "python.exe")
        else:
            return str(self.venv_dir / "bin" / "python")

    def _install_deps(self) -> bool:
        """Install Python dependencies"""
        info("Installing dependencies...")

        pip = self._get_pip()
        requirements = self.root / "requirements.txt"

        if not requirements.exists():
            warning(f"requirements.txt not found: {requirements}")
            return True

        try:
            # Upgrade pip first
            subprocess.run(
                [pip, "install", "--upgrade", "pip"],
                check=True,
                capture_output=True,
            )

            # Install requirements
            subprocess.run(
                [pip, "install", "-r", str(requirements)],
                check=True,
            )
            success("Dependencies installed")
            return True
        except subprocess.CalledProcessError as e:
            error(f"Failed to install dependencies: {e}")
            return False

    def _install_sfcli(self) -> bool:
        """Install sfcli package in editable mode"""
        info("Installing sfcli...")

        pip = self._get_pip()

        try:
            # Install as editable package
            subprocess.run(
                [pip, "install", "-e", str(self.root)],
                check=True,
            )
            success("sfcli installed")
            return True
        except subprocess.CalledProcessError as e:
            # If pyproject.toml doesn't exist, just add to PYTHONPATH
            warning(f"Editable install failed (pyproject.toml may be missing): {e}")
            info("sfcli will be available via PYTHONPATH in activate script")
            return True

    def _configure_shell(self) -> None:
        """Configure shell for easy access"""
        info("Configuring shell...")

        # Determine shell config file
        shell = os.environ.get("SHELL", "")
        home = Path.home()

        if "zsh" in shell:
            rc_file = home / ".zshrc"
        elif "bash" in shell:
            rc_file = home / ".bashrc"
        else:
            rc_file = None

        if rc_file and rc_file.exists():
            # Check if already configured
            content = rc_file.read_text()
            marker = "# StampFly Ecosystem"

            if marker in content:
                info("Shell already configured")
                return

            # Add activation alias
            alias_line = f'\n{marker}\nalias sf-activate="source {self.root}/scripts/activate.sh"\n'

            # Ask user for confirmation
            print()
            info(f"Add alias to {rc_file}?")
            print(f"    alias sf-activate=\"source {self.root}/scripts/activate.sh\"")
            print()

            try:
                response = input("Add alias? [Y/n]: ").strip().lower()
                if response in ("", "y", "yes"):
                    with open(rc_file, "a") as f:
                        f.write(alias_line)
                    success(f"Added alias to {rc_file}")
                    info("Run 'source ~/.zshrc' or start a new terminal")
                else:
                    info("Skipped shell configuration")
            except EOFError:
                info("Skipped shell configuration (non-interactive)")
        else:
            info("Shell configuration skipped (unsupported shell or missing rc file)")

    def uninstall(self) -> int:
        """Remove virtual environment and configuration"""
        header("StampFly Ecosystem Uninstaller")
        print()

        if self.venv_dir.exists():
            info(f"Removing virtual environment: {self.venv_dir}")
            shutil.rmtree(self.venv_dir)
            success("Virtual environment removed")
        else:
            info("Virtual environment not found")

        # Note: We don't automatically remove shell configuration
        # to avoid accidentally breaking user's setup
        print()
        info("Note: Shell configuration (alias) was not removed.")
        info("Edit ~/.zshrc or ~/.bashrc manually if needed.")

        return 0


def main() -> int:
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description="StampFly Ecosystem Installer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--no-venv",
        action="store_true",
        help="Skip virtual environment creation",
    )
    parser.add_argument(
        "--no-deps",
        action="store_true",
        help="Skip dependency installation",
    )
    parser.add_argument(
        "--no-shell",
        action="store_true",
        help="Skip shell configuration",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable colored output",
    )
    parser.add_argument(
        "--uninstall",
        action="store_true",
        help="Remove virtual environment and configuration",
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
            skip_venv=args.no_venv,
            skip_deps=args.no_deps,
            skip_shell=args.no_shell,
        )


if __name__ == "__main__":
    sys.exit(main())
