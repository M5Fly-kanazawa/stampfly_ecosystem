"""
Path utilities for StampFly CLI

Provides consistent path resolution across the project.
プロジェクト全体で一貫したパス解決を提供
"""

import os
from pathlib import Path
from typing import Optional


class Paths:
    """Path manager for StampFly Ecosystem"""

    def __init__(self):
        self._root: Optional[Path] = None

    def _find_root(self) -> Path:
        """Find the repository root directory"""
        if self._root is not None:
            return self._root

        # Start from this file's location
        current = Path(__file__).resolve()

        # Walk up looking for markers
        markers = [".git", "CLAUDE.md", "PROJECT_PLAN.md"]

        for parent in [current] + list(current.parents):
            for marker in markers:
                if (parent / marker).exists():
                    self._root = parent
                    return self._root

        # Fallback: use environment variable or current directory
        if "STAMPFLY_ROOT" in os.environ:
            self._root = Path(os.environ["STAMPFLY_ROOT"])
        else:
            self._root = Path.cwd()

        return self._root

    def root(self) -> Path:
        """Get repository root directory"""
        return self._find_root()

    def lib(self) -> Path:
        """Get lib/ directory"""
        return self.root() / "lib"

    def bin(self) -> Path:
        """Get bin/ directory"""
        return self.root() / "bin"

    def scripts(self) -> Path:
        """Get scripts/ directory"""
        return self.root() / "scripts"

    def firmware(self) -> Path:
        """Get firmware/ directory"""
        return self.root() / "firmware"

    def vehicle(self) -> Path:
        """Get firmware/vehicle/ directory"""
        return self.firmware() / "vehicle"

    def controller(self) -> Path:
        """Get firmware/controller/ directory"""
        return self.firmware() / "controller"

    def simulator(self) -> Path:
        """Get simulator/ directory"""
        return self.root() / "simulator"

    def tools(self) -> Path:
        """Get tools/ directory"""
        return self.root() / "tools"

    def docs(self) -> Path:
        """Get docs/ directory"""
        return self.root() / "docs"

    def logs(self) -> Path:
        """Get log storage directory (created if not exists)"""
        log_dir = self.root() / "logs"
        log_dir.mkdir(exist_ok=True)
        return log_dir

    def venv(self) -> Path:
        """Get .venv/ directory"""
        return self.root() / ".venv"

    def esp_idf(self) -> Optional[Path]:
        """Get ESP-IDF directory (searches multiple locations)"""
        # Check local installation first
        local_idf = self.root() / ".esp-idf"
        if local_idf.exists():
            return local_idf

        # Check symlink
        if local_idf.is_symlink():
            return local_idf.resolve()

        # Check common locations
        common_paths = [
            Path.home() / "esp" / "esp-idf",
            Path.home() / ".espressif" / "esp-idf",
            Path("/opt/esp-idf"),
        ]

        for p in common_paths:
            if p.exists():
                return p

        # Check environment variable
        if "IDF_PATH" in os.environ:
            return Path(os.environ["IDF_PATH"])

        return None

    def config_file(self) -> Path:
        """Get CLI configuration file path"""
        return self.root() / ".sfconfig.json"

    def templates(self) -> Path:
        """Get templates directory"""
        return self.lib() / "sfcli" / "templates"

    def ensure_dir(self, path: Path) -> Path:
        """Ensure directory exists, create if not"""
        path.mkdir(parents=True, exist_ok=True)
        return path


# Global paths instance
paths = Paths()
