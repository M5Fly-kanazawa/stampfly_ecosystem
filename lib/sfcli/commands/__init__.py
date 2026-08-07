"""
StampFly CLI Commands

Each module in this package implements a CLI command.
このパッケージの各モジュールが1つのCLIコマンドを実装する。

Modules are imported individually, each in its own try/except, instead of
one big `from . import a, b, c, ...` block. A single missing third-party
dependency (e.g. PyYAML for `sysid`, NumPy for `trim`) used to raise while
this package's own __init__ ran, which killed `sf` entirely for every
command -- not just the one that actually needed the missing package
(V3 finding). This mirrors the same defense cli.py applies when it loads
commands (see cli.py's module docstring); keeping both layers resilient
means neither an eager `import sfcli.commands` elsewhere nor cli.py's own
loader can be taken down by one broken command module.
モジュールを1つの `from . import a, b, c, ...` にまとめず、個別に
try/except で import する。以前は1つのサードパーティ依存欠落
（例: sysidのPyYAML、trimのNumPy）がこのパッケージの __init__ 実行中に
例外を出し、本当にその依存が必要なコマンドだけでなく `sf` 全体を
道連れにしていた（V3で実測）。cli.py がコマンドをロードする際の防御
（cli.pyのdocstring参照）と同じ方式をここでも適用することで、他所からの
`import sfcli.commands` も cli.py 自身のローダーも、1つの壊れた
コマンドモジュールに巻き込まれないようにする。
"""

import importlib
from typing import List, Tuple

# Command modules, listed in the order `sf --help` should display them.
# `sf --help` に表示させたい順のコマンドモジュール一覧。
_COMMAND_MODULE_NAMES: Tuple[str, ...] = (
    "version",
    "doctor",
    "setup",
    "build",
    "flash",
    "flasher",
    "monitor",
    "telemetry",
    "blocks",
    "log",
    "sim",
    "sils",
    "cal",
    "sysid",
    "params",
    "trim",
    "flight",
    "motor",
    "query",
    "rc",
    "lesson",
    "competition",
    "app",
    "docs",
    "upgrade",
)

# Names of modules that imported successfully, in the display order above.
# Read by cli.py to know which commands to register.
# import に成功したモジュール名（表示順）。cli.py がどのコマンドを
# 登録すべきか判断するために参照する。
__all__: List[str] = []

# (module name, missing dependency name) for modules that failed to
# import. Read by cli.py to build its one-line "N commands unavailable"
# warning and by assert_all_commands_loadable() (the C2/C3 contract used
# by scripts/installer.py's post-install probe).
# importに失敗した (モジュール名, 欠落依存名) の一覧。cli.py が
# 「N個のコマンドが利用不可」警告の生成に、また
# assert_all_commands_loadable()（scripts/installer.py のインストール後
# プローブが使うC2/C3の契約）が使用する。
FAILED_COMMANDS: List[Tuple[str, str]] = []

for _module_name in _COMMAND_MODULE_NAMES:
    try:
        globals()[_module_name] = importlib.import_module(f".{_module_name}", __name__)
    except ImportError as _import_error:
        FAILED_COMMANDS.append((_module_name, _import_error.name or str(_import_error)))
        continue
    __all__.append(_module_name)
