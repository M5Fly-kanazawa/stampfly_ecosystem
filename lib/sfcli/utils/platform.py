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
        # Check environment variable first
        if "IDF_PATH" in os.environ:
            idf_path = Path(os.environ["IDF_PATH"])
            if idf_path.exists():
                return idf_path

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
