"""
StampFly CLI Commands

Each module in this package implements a CLI command.
"""

from . import version
from . import doctor
from . import build
from . import flash
from . import monitor
from . import telemetry
from . import log
from . import sim
from . import sil
from . import cal
from . import setup
from . import sysid
from . import trim
from . import flight
from . import query
from . import rc
from . import lesson
from . import competition
from . import app
from . import docs

__all__ = [
    "version",
    "doctor",
    "build",
    "flash",
    "monitor",
    "telemetry",
    "log",
    "sim",
    "sil",
    "cal",
    "setup",
    "sysid",
    "trim",
    "flight",
    "query",
    "rc",
    "lesson",
    "competition",
    "app",
    "docs",
]
