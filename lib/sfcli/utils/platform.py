"""
Platform detection utilities for StampFly CLI

Cross-platform support utilities.
クロスプラットフォームサポートユーティリティ
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path
from typing import Optional, List

# Sibling submodule, not the parent package -- safe to import directly
# (paths.py has no dependency back on platform.py, so there is no import
# cycle regardless of which of the two a caller imports first).
# 兄弟サブモジュール(親パッケージではない)であり直接importして安全
# (paths.py はplatform.pyへ依存しないため、どちらを先にimportしても
# 循環importにはならない)。
from .paths import paths as _paths


class Platform:
    """Platform detection and utilities"""

    def is_windows(self) -> bool:
        """Check if running on Windows"""
        return sys.platform == "win32"

    def is_macos(self) -> bool:
        """Check if running on macOS"""
        return sys.platform == "darwin"

    def is_linux(self) -> bool:
        """Check if running on Linux"""
        return sys.platform.startswith("linux")

    def is_wsl(self) -> bool:
        """Check if running in Windows Subsystem for Linux"""
        if not self.is_linux():
            return False
        try:
            with open("/proc/version", "r") as f:
                return "microsoft" in f.read().lower()
        except Exception:
            return False

    def python_executable(self) -> str:
        """Get Python executable path"""
        return sys.executable

    def python_version(self) -> str:
        """Get Python version string"""
        return f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"

    def find_executable(self, name: str) -> Optional[Path]:
        """Find executable in PATH"""
        path = shutil.which(name)
        return Path(path) if path else None

    def esp_idf_path(self) -> Optional[Path]:
        """Find ESP-IDF installation"""
        # Check environment variable first -- setup_env.sh/.bat sets
        # IDF_PATH itself from .sf/config.toml (dedicated or legacy),
        # so once activated this is already authoritative.
        # 環境変数を最初に確認する -- setup_env.sh/.bat が(専用・旧来
        # いずれの場合も) .sf/config.toml からIDF_PATHを自ら設定するため、
        # 有効化済みならこれが既に正となる。
        if "IDF_PATH" in os.environ:
            idf_path = Path(os.environ["IDF_PATH"])
            if idf_path.exists():
                return idf_path

        # Check .sf/config.toml next, before falling back to the
        # home-dir guesses below -- a configured path (from either the
        # dedicated or legacy install flow) is more specific than a
        # generic default location.
        # 次に .sf/config.toml を確認する(下のホームディレクトリ既定への
        # フォールバックより先)-- 設定済みパス(専用・旧来どちらの導入
        # フローでも)は汎用の既定位置より具体的な情報のため優先する。
        config_idf_path = _paths.read_config_value("esp_idf", "path")
        if config_idf_path:
            config_idf = Path(config_idf_path)
            if config_idf.exists():
                return config_idf

        # Check common locations
        common_paths = [
            Path.home() / "esp" / "esp-idf",
            Path.home() / ".espressif" / "esp-idf",
        ]

        if self.is_windows():
            common_paths.extend([
                Path("C:/Espressif/frameworks/esp-idf"),
                Path("C:/esp-idf"),
            ])
        else:
            common_paths.append(Path("/opt/esp-idf"))

        for p in common_paths:
            if p.exists() and (p / "export.sh").exists():
                return p

        return None

    def esp_idf_version(self) -> Optional[str]:
        """Get ESP-IDF version if installed"""
        idf_path = self.esp_idf_path()
        if not idf_path:
            return None

        version_file = idf_path / "version.txt"
        if version_file.exists():
            return version_file.read_text().strip()

        # Try git describe
        try:
            result = subprocess.run(
                ["git", "describe", "--tags"],
                cwd=idf_path,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception:
            pass

        return None

    def run_command(
        self,
        cmd: List[str],
        cwd: Optional[Path] = None,
        env: Optional[dict] = None,
        capture: bool = False,
    ) -> subprocess.CompletedProcess:
        """Run a command with platform-appropriate settings"""
        # Merge environment
        full_env = os.environ.copy()
        if env:
            full_env.update(env)

        kwargs = {
            "cwd": cwd,
            "env": full_env,
        }

        if capture:
            kwargs["capture_output"] = True
            kwargs["text"] = True

        # On Windows, might need shell=True for some commands
        if self.is_windows() and not self.find_executable(cmd[0]):
            kwargs["shell"] = True

        return subprocess.run(cmd, **kwargs)

    def serial_ports(self) -> List[str]:
        """List available serial ports"""
        ports = []

        if self.is_windows():
            # Windows COM ports
            import winreg
            try:
                key = winreg.OpenKey(
                    winreg.HKEY_LOCAL_MACHINE,
                    r"HARDWARE\DEVICEMAP\SERIALCOMM"
                )
                i = 0
                while True:
                    try:
                        _, value, _ = winreg.EnumValue(key, i)
                        ports.append(value)
                        i += 1
                    except OSError:
                        break
            except Exception:
                pass
        else:
            # Unix-like systems
            import glob
            patterns = [
                "/dev/ttyUSB*",
                "/dev/ttyACM*",
                "/dev/cu.usbserial*",
                "/dev/cu.usbmodem*",
                "/dev/cu.SLAB*",
            ]
            for pattern in patterns:
                ports.extend(glob.glob(pattern))

        return sorted(ports)

    def default_serial_port(self) -> Optional[str]:
        """Get default serial port"""
        ports = self.serial_ports()
        if ports:
            return ports[0]
        return None

    def open_file_manager(self, path: Path) -> None:
        """Open file manager at path"""
        if self.is_macos():
            subprocess.run(["open", str(path)])
        elif self.is_windows():
            subprocess.run(["explorer", str(path)])
        else:
            subprocess.run(["xdg-open", str(path)])

    def open_url(self, url: str) -> None:
        """Open URL in default browser"""
        import webbrowser
        webbrowser.open(url)


# Global platform instance
platform = Platform()
