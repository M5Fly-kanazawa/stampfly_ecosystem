"""
Console output utilities with color support

Provides consistent, colored console output across the CLI.
カラー出力対応のコンソールユーティリティ
"""

import sys
from typing import Optional


class Console:
    """Console output with ANSI color support"""

    # ANSI color codes
    COLORS = {
        "reset": "\033[0m",
        "bold": "\033[1m",
        "dim": "\033[2m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "gray": "\033[90m",
    }

    def __init__(self):
        self._verbose = False
        self._color_enabled = self._detect_color_support()

    def _detect_color_support(self) -> bool:
        """Detect if terminal supports colors"""
        # Disable colors if not a TTY
        if not sys.stdout.isatty():
            return False

        # Check for NO_COLOR environment variable
        import os
        if os.environ.get("NO_COLOR"):
            return False

        # Check for Windows
        if sys.platform == "win32":
            # Enable ANSI on Windows 10+
            try:
                import ctypes
                kernel32 = ctypes.windll.kernel32
                kernel32.SetConsoleMode(
                    kernel32.GetStdHandle(-11), 7
                )
                return True
            except Exception:
                return False

        return True

    def _colorize(self, text: str, color: str) -> str:
        """Apply color to text if colors are enabled"""
        if not self._color_enabled:
            return text
        return f"{self.COLORS.get(color, '')}{text}{self.COLORS['reset']}"

    def set_verbose(self, enabled: bool) -> None:
        """Enable or disable verbose output"""
        self._verbose = enabled

    def set_color(self, enabled: bool) -> None:
        """Enable or disable color output"""
        self._color_enabled = enabled

    def info(self, message: str, prefix: str = "INFO") -> None:
        """Print info message (blue)"""
        colored_prefix = self._colorize(f"[{prefix}]", "blue")
        print(f"{colored_prefix} {message}")

    def success(self, message: str, prefix: str = "OK") -> None:
        """Print success message (green)"""
        colored_prefix = self._colorize(f"[{prefix}]", "green")
        print(f"{colored_prefix} {message}")

    def warning(self, message: str, prefix: str = "WARN") -> None:
        """Print warning message (yellow)"""
        colored_prefix = self._colorize(f"[{prefix}]", "yellow")
        print(f"{colored_prefix} {message}", file=sys.stderr)

    def error(self, message: str, prefix: str = "ERROR") -> None:
        """Print error message (red)"""
        colored_prefix = self._colorize(f"[{prefix}]", "red")
        print(f"{colored_prefix} {message}", file=sys.stderr)

    def debug(self, message: str, prefix: str = "DEBUG") -> None:
        """Print debug message (gray, only in verbose mode)"""
        if self._verbose:
            colored_prefix = self._colorize(f"[{prefix}]", "gray")
            print(f"{colored_prefix} {message}")

    def print(self, message: str = "", end: str = "\n") -> None:
        """Print plain message"""
        print(message, end=end)

    def header(self, title: str, char: str = "=", width: int = 60) -> None:
        """Print section header"""
        line = char * width
        print(self._colorize(line, "cyan"))
        print(self._colorize(f" {title}", "bold"))
        print(self._colorize(line, "cyan"))

    def table(self, headers: list, rows: list, col_widths: Optional[list] = None) -> None:
        """Print simple table"""
        if not col_widths:
            # Calculate column widths
            col_widths = [len(h) for h in headers]
            for row in rows:
                for i, cell in enumerate(row):
                    col_widths[i] = max(col_widths[i], len(str(cell)))

        # Header
        header_line = " | ".join(
            str(h).ljust(col_widths[i]) for i, h in enumerate(headers)
        )
        print(self._colorize(header_line, "bold"))
        print("-" * len(header_line))

        # Rows
        for row in rows:
            row_line = " | ".join(
                str(cell).ljust(col_widths[i]) for i, cell in enumerate(row)
            )
            print(row_line)

    def progress(self, current: int, total: int, prefix: str = "", width: int = 40) -> None:
        """Print progress bar"""
        percent = current / total if total > 0 else 0
        filled = int(width * percent)
        bar = "█" * filled + "░" * (width - filled)
        percent_str = f"{percent * 100:.1f}%"
        print(f"\r{prefix}[{bar}] {percent_str}", end="", flush=True)
        if current >= total:
            print()  # New line when complete


# Global console instance
console = Console()
