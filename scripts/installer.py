#!/usr/bin/env python3
"""
StampFly Ecosystem Installer

Installs sfcli into ESP-IDF's Python environment.
ESP-IDFのPython環境にsfcliをインストールします。

Usage:
    python scripts/installer.py [options]

Options:
    --idf-path PATH     Specify ESP-IDF path
    --skip-deps         Skip dependency installation
    --minimal           Install minimal dependencies (skip simulator)
    --uninstall         Remove sfcli from ESP-IDF environment
    --clean             Clean install (remove config and sfcli, then reinstall)
    --force             Force reinstall all steps (skip probe checks)
    --no-flasher        Skip the optional Step 4/4 GUI Flasher app install
    --no-sils-toolchain     Skip the SILS development toolchain (Windows:
                            MSYS2/MinGW-w64, ~2GB) used to build
                            simulator/sils/ from source -- it installs by
                            DEFAULT otherwise. Interactive mode still asks
                            via a y/n prompt (default Yes) regardless of
                            this flag; this flag only controls
                            --non-interactive mode. macOS/Linux only print
                            guidance either way (no unattended package
                            install there). `--with-sils-toolchain` is kept
                            as a no-op for backward compatibility (it is
                            already the default)
    --non-interactive   Never call input(); return defaults instead
    --auto-install-python  Attempt to auto-install a system Python (3.10-3.12)
                            via winget/brew/apt when none is found. Only takes
                            effect in --non-interactive mode (interactive mode
                            always asks via a y/n prompt regardless of this
                            flag); Linux's sudo-gated install command is never
                            run non-interactively (guidance only)

Stability contract / 安定契約
------------------------------
This file is executed both as a standalone CLI and, in-process, by the GUI
installer (tools/installer_gui/stampfly_installer.py, see
docs/plans/gui-installer-plan.md). The GUI imports this module with
importlib and drives it directly, so the following are a contract with
that caller and must not change without updating it too:
本ファイルはスタンドアロンCLIとしてだけでなく、GUIインストーラ
（tools/installer_gui/stampfly_installer.py、docs/plans/gui-installer-plan.md
参照）からもプロセス内 import されて実行される。GUIは本モジュールを
importlib でimportして直接操作するため、以下はその呼び出し元との契約で
あり、変更する場合はGUI側も合わせて更新すること:

1. The `Installer` class and the signatures of its `run()` / `uninstall()`
   / `clean()` methods must stay backward compatible — the GUI calls them
   directly (no subprocess, no CLI parsing).
   `Installer` クラス、および `run()`/`uninstall()`/`clean()` メソッドの
   シグネチャは後方互換を保つこと — GUIはこれらをsubprocessもCLI引数
   解析も経由せず直接呼び出す。
2. The progress header format `Step N/4: <title>` (see header(), used by
   Steps 1-4 in Installer.run()) must not change: the GUI parses these
   lines from captured stdout to advance its step indicator.
   進捗ヘッダの書式 `Step N/4: <タイトル>`（header() 参照、
   Installer.run() のStep1〜4で使用）は変更しないこと — GUIはキャプチャ
   したstdoutからこの行をパースしてステップインジケータを進める。
3. This file must keep using only the Python standard library. If a new
   stdlib import is added here, also add it to the hidden-import list in
   tools/installer_gui/stampfly_installer.py (the GUI runs under a frozen
   PyInstaller build, so undeclared stdlib modules silently fail to import
   there even though they work fine when run as a normal script).
   本ファイルはstdlibのみを使い続けること。新しいstdlib importを追加
   する場合は tools/installer_gui/stampfly_installer.py の
   hidden-import一覧にも追記する（GUIはPyInstallerで凍結された環境で
   動くため、通常のスクリプト実行では問題なくても、宣言されていない
   stdlibモジュールはそこで暗黙にimport失敗しうる）。

Also see the SF_INSTALLER_NONINTERACTIVE contract on prompt() /
prompt_choice() below, and --non-interactive in main(): the GUI relies on
both to drive this script with no TTY attached.
prompt()/prompt_choice() の SF_INSTALLER_NONINTERACTIVE 契約（下記）と
main() の --non-interactive も参照。GUIはTTYなしで本スクリプトを駆動する
ためにこの両方に依存する。
"""

# `from __future__ import annotations` MUST come before any other code
# (docstrings/comments excepted): it defers evaluation of every type
# annotation in this file to a string, so a builtin-generic annotation like
# `-> list[Path]` (PEP 585, only usable unquoted from Python 3.9+) no longer
# raises `TypeError: 'type' object is not subscriptable` at *def* time on
# Python 3.8. Without this, that TypeError fires while the module is being
# loaded -- BEFORE the explicit sys.version_info check below ever runs --
# so a Python 3.8/3.9 user got a confusing traceback instead of the
# friendly "Python 3.10+ required" message this file is supposed to show.
# `from __future__ import annotations` は(docstring/コメントを除き)他の
# どのコードよりも前に置く必要がある: これにより本ファイル内の全ての型
# 注釈の評価が文字列として遅延され、`-> list[Path]` のような組み込み
# ジェネリクス注釈(PEP 585、クォート無しで使えるのは Python 3.9+のみ)が
# Python 3.8 の def 時点で `TypeError: 'type' object is not subscriptable`
# を送出しなくなる。これが無いと、このTypeErrorはモジュール読み込み中 --
# 下の明示的な sys.version_info チェックが走るより前 -- に発生するため、
# Python 3.8/3.9 ユーザーは本来表示されるべき親切な「Python 3.10+ required」
# メッセージではなく、不可解なトレースバックを見ることになっていた。
from __future__ import annotations

import os
import re
import shlex
import sys
import subprocess
import shutil
import tempfile
from pathlib import Path
from typing import Optional, List, Tuple

# Ensure we're running Python 3.10+ (the ecosystem's actual floor: see
# pyproject.toml's `requires-python = ">=3.10"`; CI validates 3.12). 3.8/3.9
# used to be accepted here, but nothing downstream (sfcli, its
# dependencies) has ever actually supported them.
# Python 3.10+ を必須とする(エコシステムの実質要求。pyproject.toml の
# `requires-python = ">=3.10"` を参照。CIは3.12で検証)。かつては3.8/3.9も
# ここで受理していたが、下流(sfcliおよびその依存関係)がそれらを実際に
# サポートしたことは一度も無い。
if sys.version_info < (3, 10):
    print(f"Error: Python 3.10+ required (3.12 recommended), "
          f"found {sys.version_info.major}.{sys.version_info.minor}")
    sys.exit(1)


# The ecosystem's actually-tested Python range. Selection logic throughout
# this file (see _find_system_python_dir()) prefers a system Python inside
# this range and rejects anything outside it -- both older AND 3.13+ --
# offering auto-install (winget/brew/apt, which installs 3.12) instead.
# 3.13+ used to be accepted with a warning, but real-world failures were
# observed with it, so it is now rejected outright just like an older,
# unsupported version (2026-07-22 policy change).
# このエコシステムが実際に検証済みのPython範囲。本ファイル全体の選択
# ロジック(_find_system_python_dir() 参照)は、この範囲内のシステム
# Pythonを優先し、範囲外のもの -- 古いバージョンと3.13以降の両方 -- は
# 不採用とし、代わりに自動インストール(winget/brew/apt経由、3.12を導入)を
# 提案する。以前は3.13+を警告付きで受け入れていたが、実際に動作しない
# 事例が報告されたため、対応外の古いバージョンと同様に無条件で不採用と
# するよう変更した(2026-07-22の方針変更)。
PYTHON_PREFERRED_MIN = (3, 10)
PYTHON_PREFERRED_MAX = (3, 12)

# Stability ranking for candidate seeds WITHIN the same version-preference
# band (see _find_system_python_dir()'s composite ordering: version band
# > stability > version recency). Lower number = more stable / preferred.
# A canonical install (python.org, winget, Program Files, the `py`
# launcher's resolution, or a distro package manager on Linux) is the most
# predictable seed for ESP-IDF's own venv creation; a version-manager
# (pyenv/uv/asdf) shim is next; conda's own DLL/shared-library resolution
# quirks put it below that; and a candidate that turned out to be a venv
# whose seed we had to resolve via pyvenv.cfg (see _resolve_venv_seed())
# is the least deliberate, most incidental discovery, so it ranks last.
# 同一バージョン適合バンド内での候補の安定度ランク(_find_system_python_dir()
# の合成順序「バージョンバンド > 安定度 > バージョンの新しさ」を参照)。
# 数値が小さいほど安定/優先。正規インストール(python.org、winget、
# Program Files、`py` ランチャーの解決先、Linuxのディストリビューション
# パッケージマネージャ)が ESP-IDF 自身の venv 作成にとって最も予測可能な
# 種であり、次にバージョン管理ツール(pyenv/uv/asdf)の shim が続く。conda
# は自身の DLL/共有ライブラリ解決の癖によりその下に位置し、venv の
# pyvenv.cfg 経由で実体解決せざるを得なかった候補(_resolve_venv_seed()
# 参照)は最も「狙って選ばれたのではない」発見のため最下位とする。
STABILITY_CANONICAL = 0
STABILITY_VERSION_MANAGER = 1
STABILITY_CONDA = 2
STABILITY_VENV_RESOLVED = 3


def version_sort_key(version: str) -> Tuple[int, int, int]:
    """Convert an ESP-IDF version string (e.g. "v5.10.0") into a numeric
    tuple so it sorts correctly against other versions.
    ESP-IDFのバージョン文字列(例: "v5.10.0")を数値タプルに変換し、
    他のバージョンと正しく比較できるようにする

    Plain string comparison is wrong here: "v5.10.0" < "v5.5.2" lexically
    (the character '1' sorts before '5'), which made find_all() rank an
    older v5.5.2 ahead of a newer v5.10.0. Comparing (5, 10, 0) against
    (5, 5, 2) as tuples of ints sorts them correctly.
    単純な文字列比較では "v5.10.0" < "v5.5.2" になってしまう(文字'1'が
    '5'より前にソートされるため)。これによりfind_all()が新しいv5.10.0を
    古いv5.5.2より下位にランク付けしていた。(5, 10, 0)と(5, 5, 2)を
    整数タプルとして比較すれば正しい順序になる。

    Unrecognized strings (e.g. "unknown") sort lowest so they never shadow
    a real version in the "newest first" ordering.
    認識できない文字列(例: "unknown")は最下位にソートし、"newest first"
    の並びで実バージョンより上に来ないようにする。
    """
    import re
    match = re.match(r"v?(\d+)(?:\.(\d+))?(?:\.(\d+))?", version)
    if not match:
        return (-1, -1, -1)
    major, minor, patch = match.groups()
    return (int(major), int(minor or 0), int(patch or 0))


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
    print(f"{Colors.BLUE}[INFO]{Colors.RESET} {msg}", flush=True)


def success(msg: str) -> None:
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}", flush=True)


def warn(msg: str) -> None:
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}", flush=True)


def error(msg: str) -> None:
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {msg}", file=sys.stderr, flush=True)


def header(title: str) -> None:
    line = "=" * 60
    print(f"\n{Colors.CYAN}{line}{Colors.RESET}", flush=True)
    print(f"{Colors.BOLD} {title}{Colors.RESET}", flush=True)
    print(f"{Colors.CYAN}{line}{Colors.RESET}\n", flush=True)


def prompt(message: str, default: str = "") -> str:
    """Prompt user for input.
    ユーザーに入力を促す

    Non-interactive contract: when SF_INSTALLER_NONINTERACTIVE=1 is set in
    the environment (e.g. by --non-interactive, see main()), returns
    `default` immediately without calling input(). This lets a GUI
    frontend drive this script with no TTY attached (see the module
    docstring's "Stability contract" section).
    非対話化契約: 環境変数 SF_INSTALLER_NONINTERACTIVE=1 が設定されている
    場合（--non-interactive 経由、main() 参照）、input() を呼ばず即座に
    default を返す。これにより GUI フロントエンドが TTY 無しで本スクリプト
    を駆動できる（モジュールdocstringの「安定契約」節を参照）。

    Also EOF-safe regardless of the flag above: if stdin is closed or
    absent, input() raises EOFError, which is caught here and treated the
    same as an empty response (return `default`) rather than propagating.
    上記フラグの有無によらず EOF セーフ: stdin が閉じている／存在しない
    環境では input() が EOFError を送出するが、ここで捕捉して空応答と
    同様に扱い（default を返す）、例外を外へ伝播させない。
    """
    if os.environ.get("SF_INSTALLER_NONINTERACTIVE") == "1":
        return default

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
    """Prompt user to select from choices.
    選択肢から選ぶようユーザーに促す

    Non-interactive contract: when SF_INSTALLER_NONINTERACTIVE=1 is set,
    returns the `default` choice number immediately without calling
    input() (see prompt() above and the module docstring's "Stability
    contract" section).
    非対話化契約: SF_INSTALLER_NONINTERACTIVE=1 のとき、input() を呼ばず
    即座に default の選択番号を返す（prompt() およびモジュールdocstring
    の「安定契約」節を参照）。

    Also EOF-safe: previously, an EOFError from input() (closed/absent
    stdin) fell into the same except branch as an invalid number and the
    loop retried input() forever, printing "Please enter a number..."
    indefinitely. EOFError/KeyboardInterrupt are now handled separately
    from a bad number (ValueError) so EOF returns `default` instead of
    looping.
    EOF セーフ: 以前は input() の EOFError（stdin が閉じている／無い）が
    不正な数値入力と同じ except 節に落ち、ループが input() を永久に
    再試行して "Please enter a number..." を出し続けていた。現在は
    EOFError/KeyboardInterrupt を不正な数値（ValueError）と分けて処理し、
    EOF 時は default を返してループを抜ける。
    """
    if os.environ.get("SF_INSTALLER_NONINTERACTIVE") == "1":
        return default

    print(f"\n{message}\n")
    for i, choice in enumerate(choices, 1):
        marker = " <- recommended" if i == default else ""
        print(f"  [{i}] {choice}{marker}")
    print()

    while True:
        try:
            response = input(f"Select [{default}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return default
        if not response:
            return default
        try:
            idx = int(response)
            if 1 <= idx <= len(choices):
                return idx
        except ValueError:
            pass
        print(f"Please enter a number between 1 and {len(choices)}")


def _windows_python_dir_candidates() -> list[Path]:
    """Windows: likely directories containing a CANONICAL system
    python.exe (python.org / winget / scoop installs), in the same
    priority order as install.bat's discovery block. Version managers
    (pyenv-win, uv) and conda distributions are intentionally NOT
    included here -- see _windows_pyenv_win_python_dir(),
    _uv_python_candidates(), and _windows_conda_dir_candidates(), which
    are tagged with a different STABILITY_* rank by _all_python_candidates()
    (see "種の安定度順位付け" in the module's design notes).
    Windows: 正規(python.org / winget / scoop)インストールの python.exe を
    含むと思われるディレクトリ一覧。install.bat の発見ブロックと同じ優先順
    で返す。バージョン管理ツール(pyenv-win、uv)や conda ディストリビュー
    ションは意図的にここへ含めない -- _windows_pyenv_win_python_dir()、
    _uv_python_candidates()、_windows_conda_dir_candidates() を参照。これらは
    _all_python_candidates() で異なる STABILITY_* ランクを付けて扱う
    (モジュール設計メモの「種の安定度順位付け」参照)。

    Kept in sync with install.bat (per-user Programs\\Python, machine-wide
    C:\\Python*, python.org's all-users Program Files location, scoop).
    Duplicated here — not shared — because installer.py is a standalone
    stdlib-only script AND because the GUI (StampFly Setup) never runs
    install.bat at all: the frozen app imports installer.py in-process, so
    this discovery must live here for the GUI path to find Python for
    ESP-IDF's own install.bat.
    install.bat と同期を保つ(ユーザー毎の Programs\\Python、マシン全体の
    C:\\Python*、python.org の all-users インストール先の Program Files、
    scoop)。共有せず複製する理由: 本ファイルは stdlib のみの独立スクリプト
    であり、かつ GUI(StampFly Setup)は install.bat を一切実行しない —
    凍結アプリが installer.py をプロセス内 import するため、GUI 経路が
    ESP-IDF の install.bat 用に Python を見つけられるよう、この発見ロジック
    はここに置く必要がある。
    """
    candidates: list[Path] = []
    userprofile = os.environ.get("USERPROFILE", "")
    localappdata = os.environ.get("LOCALAPPDATA", "")

    # Common install locations (newest first). Program Files / Program
    # Files (x86) are python.org's default "Install for all users" targets
    # -- distinct from the per-user LOCALAPPDATA\Programs\Python default.
    # 一般的なインストール先(新しい順)。Program Files / Program Files
    # (x86) は python.org の既定「全ユーザー用にインストール」先で、
    # ユーザー毎既定の LOCALAPPDATA\Programs\Python とは別物。
    for ver in ("313", "312", "311", "310", "39", "38"):
        if localappdata:
            candidates.append(Path(localappdata) / "Programs" / "Python" / f"Python{ver}")
        candidates.append(Path(f"C:/Python{ver}"))
        candidates.append(Path(f"C:/Program Files/Python{ver}"))
        candidates.append(Path("C:/Program Files (x86)") / f"Python{ver}")
    if userprofile:
        candidates.append(Path(userprofile) / "scoop" / "apps" / "python" / "current")
    return candidates


def _windows_pyenv_win_python_dir() -> Optional[Path]:
    """Windows: the directory of the version pyenv-win currently has
    selected (its `version` file), or None if pyenv-win is not set up.
    Split out of _windows_python_dir_candidates() so callers can tag it
    with STABILITY_VERSION_MANAGER instead of STABILITY_CANONICAL.
    Windows: pyenv-win が現在選択しているバージョンのディレクトリ
    (`version` ファイルの内容)、未設定なら None。
    _windows_python_dir_candidates() から分離し、呼び出し側が
    STABILITY_CANONICAL ではなく STABILITY_VERSION_MANAGER でタグ付け
    できるようにする。
    """
    userprofile = os.environ.get("USERPROFILE", "")
    if not userprofile:
        return None
    pyenv_root = Path(userprofile) / ".pyenv" / "pyenv-win"
    version_file = pyenv_root / "version"
    try:
        if version_file.is_file():
            pyenv_ver = version_file.read_text(encoding="utf-8", errors="replace").strip()
            if pyenv_ver:
                return pyenv_root / "versions" / pyenv_ver
    except OSError:
        pass
    return None


def _windows_conda_dir_candidates() -> list[Path]:
    """Windows: known conda/miniconda/anaconda/miniforge base-environment
    directories (each holds python.exe directly at its root, unlike a
    normal venv's Scripts\\python.exe). Covers both a per-user install
    (the common "Install for me only" default) and an all-users install
    under %PROGRAMDATA% or directly at the drive root.
    Windows: 既知の conda/miniconda/anaconda/miniforge ベース環境
    ディレクトリ(通常の venv の Scripts\\python.exe と異なり、python.exe
    がルート直下にある)。ユーザー毎インストール(「自分専用にインストール」
    という一般的な既定)と、%PROGRAMDATA% またはドライブ直下への全ユーザー
    インストールの両方をカバーする。
    """
    candidates: list[Path] = []
    userprofile = os.environ.get("USERPROFILE", "")
    programdata = os.environ.get("PROGRAMDATA", "C:/ProgramData")
    if userprofile:
        for name in ("anaconda3", "miniconda3", "miniforge3"):
            candidates.append(Path(userprofile) / name)
    if programdata:
        for name in ("Anaconda3", "Miniconda3", "Miniforge3"):
            candidates.append(Path(programdata) / name)
    for name in ("Anaconda3", "Miniconda3"):
        candidates.append(Path("C:/") / name)
    return candidates


def _py_launcher_python_dir() -> Optional[Path]:
    """Windows: ask the `py` launcher for the real python.exe it would run,
    so an install that only registered `py` on PATH (not `python`/
    `python3`, e.g. some python.org installs) is still found. Checked with
    highest priority by _find_system_python_dir() since `py` is the most
    authoritative way to ask "what is THE system Python" on Windows.
    Windows: `py` ランチャーに実際に実行する python.exe を問い合わせる。
    `python`/`python3` ではなく `py` だけを PATH に登録したインストール
    (一部の python.org インストール等)でも見つけられるようにする。
    `py` は Windows上で「システムの Python は何か」を尋ねる最も権威ある
    手段のため、_find_system_python_dir() は最優先でこれを確認する。

    Returns None (never raises) on any failure: `py` absent, the subprocess
    timing out or erroring, or the resolved path being the non-functional
    WindowsApps Store stub (same exclusion as the PATH lookup below).
    いかなる失敗でも None を返す(例外は送出しない): `py` が無い、
    subprocess がタイムアウト/エラーになる、解決先が機能しない
    WindowsApps ストアスタブである(下の PATH 探索と同じ除外)、のいずれも。
    """
    py_launcher = shutil.which("py")
    if not py_launcher:
        return None
    try:
        result = subprocess.run(
            [py_launcher, "-3", "-c", "import sys; print(sys.executable)"],
            capture_output=True, timeout=15,
            encoding="utf-8", errors="replace",
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    output = result.stdout.strip()
    if not output:
        return None
    exe = Path(output.splitlines()[0])
    if "windowsapps" in str(exe).lower() or not exe.is_file():
        return None
    return exe.parent


def _python_version_info(python_exe: Path) -> Optional[Tuple[int, int]]:
    """Return (major, minor) reported by python_exe, or None on any
    failure (does not exist, times out, non-zero exit, unparsable output).
    Never raises.
    python_exe が報告する (major, minor) を返す。起動不可・タイムアウト・
    非ゼロ終了・出力が解析不能ないずれの場合も None(例外は送出しない)。

    Kept in the same style as the pre-existing _py_launcher_python_dir():
    a short subprocess timeout, utf-8/replace decoding, and CREATE_NO_WINDOW
    on Windows so no console flashes open when this runs inside a frozen
    --windowed GUI.
    既存の _py_launcher_python_dir() と同じ流儀(短いタイムアウト、
    utf-8/replace デコード、Windows では CREATE_NO_WINDOW で凍結
    --windowed GUI 内でもコンソールが一瞬開かないようにする)を踏襲する。
    """
    try:
        result = subprocess.run(
            [str(python_exe), "-c",
             "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')"],
            capture_output=True, timeout=15,
            encoding="utf-8", errors="replace",
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    match = re.match(r"(\d+)\.(\d+)", (result.stdout or "").strip())
    if not match:
        return None
    return (int(match.group(1)), int(match.group(2)))


def _pyenv_unix_python_candidates() -> list[Path]:
    """macOS/Linux: python3 executables installed under pyenv's versions
    directory (e.g. ~/.pyenv/versions/3.12.3/bin/python3), newest
    version-string first. Tagged STABILITY_VERSION_MANAGER by
    _all_python_candidates() -- pyenv installs are deliberate and stable,
    but rank below a canonical python.org/package-manager install (see
    "種の安定度順位付け" in the module's design notes).
    macOS/Linux: pyenv の versions ディレクトリ配下にインストールされた
    python3 実行ファイル(例: ~/.pyenv/versions/3.12.3/bin/python3)。
    バージョン文字列の新しい順。_all_python_candidates() で
    STABILITY_VERSION_MANAGER としてタグ付けする -- pyenv インストールは
    意図的で安定しているが、正規(python.org/パッケージマネージャ)
    インストールより下位に順位付けする(モジュール設計メモの
    「種の安定度順位付け」参照)。
    """
    try:
        return sorted(
            (Path.home() / ".pyenv" / "versions").glob("*/bin/python3"),
            reverse=True,
        )
    except OSError:
        return []


def _asdf_unix_python_candidates() -> list[Path]:
    """macOS/Linux: python3 executables installed under asdf's python
    plugin (e.g. ~/.asdf/installs/python/3.12.3/bin/python3). Same
    STABILITY_VERSION_MANAGER rank as pyenv/uv -- see
    _pyenv_unix_python_candidates().
    macOS/Linux: asdf の python プラグインでインストールされた python3
    実行ファイル(例: ~/.asdf/installs/python/3.12.3/bin/python3)。
    pyenv/uv と同じ STABILITY_VERSION_MANAGER ランク --
    _pyenv_unix_python_candidates() を参照。
    """
    try:
        return sorted(
            (Path.home() / ".asdf" / "installs" / "python").glob("*/bin/python3"),
            reverse=True,
        )
    except OSError:
        return []


def _uv_python_candidates() -> list[Path]:
    """uv-managed Python interpreters (astral-sh/uv), covering both
    platforms this file supports directly:
    Windows: %APPDATA%\\uv\\python\\cpython-3.*\\python.exe
    macOS/Linux: ~/.local/share/uv/python/cpython-3.*/bin/python3

    Also asks `uv python find 3.12` when `uv` is on PATH, so a uv install
    that used a non-default UV_PYTHON_INSTALL_DIR is still found -- the
    directory glob above only covers uv's own default install location.
    Tagged STABILITY_VERSION_MANAGER by _all_python_candidates().

    uv(astral-sh/uv)が管理する Python インタプリタ。本ファイルが直接
    対応する両プラットフォームを対象にする:
    Windows: %APPDATA%\\uv\\python\\cpython-3.*\\python.exe
    macOS/Linux: ~/.local/share/uv/python/cpython-3.*/bin/python3

    `uv` が PATH にあれば `uv python find 3.12` にも問い合わせる。これに
    より、既定と異なる UV_PYTHON_INSTALL_DIR を使った uv インストールも
    見つけられる -- 上のディレクトリ glob は uv 自身の既定インストール先
    のみをカバーするため。_all_python_candidates() で
    STABILITY_VERSION_MANAGER としてタグ付けする。
    """
    candidates: list[Path] = []
    if sys.platform == "win32":
        appdata = os.environ.get("APPDATA", "")
        if appdata:
            try:
                candidates.extend(
                    sorted(Path(appdata).glob("uv/python/cpython-3.*/python.exe"), reverse=True)
                )
            except OSError:
                pass
    else:
        try:
            uv_python_dir = Path.home() / ".local" / "share" / "uv" / "python"
            candidates.extend(
                sorted(uv_python_dir.glob("cpython-3.*/bin/python3"), reverse=True)
            )
        except OSError:
            pass

    uv = shutil.which("uv")
    if uv:
        try:
            result = subprocess.run(
                [uv, "python", "find", "3.12"],
                capture_output=True, timeout=15,
                encoding="utf-8", errors="replace",
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
            if result.returncode == 0:
                lines = (result.stdout or "").strip().splitlines()
                if lines and Path(lines[0]).is_file():
                    candidates.append(Path(lines[0]))
        except (OSError, subprocess.SubprocessError):
            pass
    return candidates


def _unix_conda_python_candidates() -> list[Path]:
    """macOS/Linux: known conda/miniconda/anaconda/miniforge base-
    environment python3, in priority order. Checked last among managed
    interpreters -- see STABILITY_CONDA in _all_python_candidates() --
    because conda's own DLL/shared-library resolution quirks make it the
    least predictable seed for ESP-IDF's venv creation.
    macOS/Linux: 既知の conda/miniconda/anaconda/miniforge ベース環境の
    python3。管理系の中で最後に確認する -- _all_python_candidates() の
    STABILITY_CONDA を参照 -- conda 自身の DLL/共有ライブラリ解決の癖により、
    ESP-IDF venv 作成の種として最も予測しづらいため。
    """
    home = Path.home()
    bases = [
        home / "miniconda3", home / "anaconda3", home / "miniforge3",
        Path("/opt/miniconda3"), Path("/opt/anaconda3"), Path("/opt/conda"),
    ]
    return [base / "bin" / "python3" for base in bases]


def _macos_python_exe_candidates() -> list[Path]:
    """macOS: likely python3 executable paths, in priority order.
    macOS: python3 実行ファイルの候補(優先順)。

    Covers: whatever `python3` resolves to on PATH; Homebrew's Apple
    Silicon (/opt/homebrew) and Intel (/usr/local) prefixes, both as
    plain `bin/python3*` (for formulas that DO symlink into bin, e.g. a
    non-keg-only `python3`) and as the keg-only `opt/python@3.1x/bin`
    layout Homebrew actually uses for versioned python@3.x formulas (these
    are deliberately NOT symlinked into Homebrew's main bin/ to avoid
    fighting over which python3 wins); python.org's framework installs;
    and explicit python3.12/3.11/3.10 command names for a pyenv or
    similar version manager that only shims the versioned names.
    対象: PATH上の `python3` が指す先、Homebrew の Apple Silicon
    (/opt/homebrew) と Intel (/usr/local) の各プレフィックス
    (bin/python3* にシンボリックリンクする formula 向けと、Homebrew が
    バージョン付き python@3.x formula に実際に使う keg-only な
    opt/python@3.1x/bin レイアウトの両方 -- 後者は「どの python3 が
    勝つか」の衝突を避けるため意図的に Homebrew の主 bin/ にはリンク
    されない)、python.org のフレームワークインストール、pyenv 等
    バージョン管理ツールがバージョン付き名前のみ shim する場合に備えた
    python3.12/3.11/3.10 の明示コマンド名。
    """
    candidates: list[Path] = []
    on_path = shutil.which("python3")
    if on_path:
        candidates.append(Path(on_path))

    homebrew_prefixes: list[Path] = []
    brew = shutil.which("brew")
    if brew:
        try:
            result = subprocess.run(
                [brew, "--prefix"], capture_output=True, timeout=15,
                encoding="utf-8", errors="replace",
            )
            if result.returncode == 0:
                prefix = result.stdout.strip()
                if prefix:
                    homebrew_prefixes.append(Path(prefix))
        except (OSError, subprocess.SubprocessError):
            pass
    # Fall back to both well-known default prefixes even without (or in
    # addition to) a working `brew --prefix`, since a fresh `brew install`
    # in _auto_install_python_macos() may run before `brew` itself is
    # resolvable in this process's PATH snapshot.
    # 動作する `brew --prefix` が無くても(あるいはそれに加えて)両方の
    # 既知既定プレフィックスを候補に含める。_auto_install_python_macos()
    # 内での新規 `brew install` 直後は、このプロセスの PATH スナップ
    # ショットではまだ `brew` 自体が解決できないことがあるため。
    for default in (Path("/opt/homebrew"), Path("/usr/local")):
        if default not in homebrew_prefixes:
            homebrew_prefixes.append(default)

    for prefix in homebrew_prefixes:
        for ver in ("3.12", "3.11", "3.10"):
            candidates.append(prefix / "opt" / f"python@{ver}" / "bin" / f"python{ver}")
        try:
            candidates.extend(sorted(prefix.glob("bin/python3*"), reverse=True))
        except OSError:
            pass

    try:
        candidates.extend(
            sorted(Path("/Library/Frameworks/Python.framework/Versions").glob("3.*/bin/python3"), reverse=True)
        )
    except OSError:
        pass

    for ver in ("3.12", "3.11", "3.10"):
        found = shutil.which(f"python{ver}")
        if found:
            candidates.append(Path(found))
    return candidates


def _linux_python_exe_candidates() -> list[Path]:
    """Linux: likely python3 executable names on PATH, in priority order.
    Linux: PATH上の python3 実行ファイル名の候補(優先順)。

    Unlike Windows/macOS, there is no single well-known "install location"
    to scan off-PATH -- distributions install versioned interpreters
    (python3.12, python3.10, ...) via their package manager straight onto
    PATH. So this just tries specific version names most-preferred first,
    then python3.13 (still probed so a rejection message can name it --
    see _find_system_python_dir() -- even though it is no longer accepted),
    then the bare `python3` that a fresh `apt install python3.12` etc. may
    not have repointed.
    Windows/macOSと異なり、PATH外を走査すべき単一の既知インストール先は
    無い -- ディストリビューションはパッケージマネージャでバージョン付き
    インタプリタ(python3.12, python3.10, ...)を直接PATHへ導入する。
    そのため、優先度の高いバージョン名から順に試し、次に python3.13
    (もはや受理はしないが、不採用メッセージでバージョンを名指しできるよう
    引き続き探索する -- _find_system_python_dir() 参照)、最後に(新規
    `apt install python3.12` 等で向き先が変わっていないかもしれない)
    素の `python3` を試す。
    """
    candidates: list[Path] = []
    for ver in ("3.12", "3.11", "3.10", "3.13"):
        found = shutil.which(f"python{ver}")
        if found:
            candidates.append(Path(found))
    on_path = shutil.which("python3")
    if on_path:
        candidates.append(Path(on_path))
    return candidates


def _all_python_candidates() -> list[Tuple[Path, int]]:
    """Return every plausible system-python executable for the current
    platform, in priority order (not yet filtered by version), each
    paired with its STABILITY_* rank (see the constants' docstring above).
    A PATH-resolved `python`/`python3` is tagged STABILITY_CANONICAL even
    though it *could* turn out to be a venv interpreter -- that
    possibility is handled uniformly for every candidate afterward by
    _resolve_venv_seed() in _find_system_python_dir(), which re-tags a
    resolved venv seed as STABILITY_VENV_RESOLVED regardless of where it
    was originally found.
    現在のプラットフォーム向けの、あり得るシステムPython実行ファイルを
    全て優先順で返す(まだバージョンで絞り込んでいない)。各候補に
    STABILITY_* ランク(上の定数のdocstring参照)を付与する。PATH上で
    解決した `python`/`python3` は、それが venv のインタプリタである
    可能性があっても STABILITY_CANONICAL としてタグ付けする -- その
    可能性は _find_system_python_dir() 内の _resolve_venv_seed() が全候補
    に対して事後に統一的に扱い、実体解決された venv の種は元の発見場所に
    関わらず STABILITY_VENV_RESOLVED に付け替える。
    """
    candidates: list[Tuple[Path, int]] = []
    if sys.platform == "win32":
        # 1. Highest priority: ask the `py` launcher directly. Some
        # python.org installs register only `py` (not `python`/`python3`)
        # on PATH, which the name-based lookup right below would
        # otherwise miss entirely.
        # 1. 最優先: `py` ランチャーに直接問い合わせる。一部の python.org
        # インストールは `py` だけを PATH に登録し `python`/`python3` を
        # 登録しないため、これが無いと直後の名前ベース探索では見つからない。
        py_dir = _py_launcher_python_dir()
        if py_dir is not None:
            candidates.append((py_dir / "python.exe", STABILITY_CANONICAL))
        # 2. Already resolvable on PATH? Skip the WindowsApps stub, which
        # is a non-functional placeholder that only opens the Store.
        # 2. 既に PATH で解決できるか? Store を開くだけの機能しない
        # プレースホルダである WindowsApps スタブは除外する。
        for name in ("python", "python3"):
            found = shutil.which(name)
            if found and "windowsapps" not in found.lower():
                candidates.append((Path(found), STABILITY_CANONICAL))
        # 3. Known canonical install locations (same as install.bat).
        # 3. 既知の正規インストール先(install.bat と同一)。
        for candidate_dir in _windows_python_dir_candidates():
            candidates.append((candidate_dir / "python.exe", STABILITY_CANONICAL))
        # 4. Version managers (pyenv-win, uv).
        # 4. バージョン管理ツール(pyenv-win、uv)。
        pyenv_dir = _windows_pyenv_win_python_dir()
        if pyenv_dir is not None:
            candidates.append((pyenv_dir / "python.exe", STABILITY_VERSION_MANAGER))
        for uv_exe in _uv_python_candidates():
            candidates.append((uv_exe, STABILITY_VERSION_MANAGER))
        # 5. conda/miniconda/anaconda/miniforge (lowest of the managed tiers).
        # 5. conda/miniconda/anaconda/miniforge(管理系の中で最下位)。
        for conda_dir in _windows_conda_dir_candidates():
            candidates.append((conda_dir / "python.exe", STABILITY_CONDA))
        return candidates

    if sys.platform == "darwin":
        for exe in _macos_python_exe_candidates():
            candidates.append((exe, STABILITY_CANONICAL))
    else:
        for exe in _linux_python_exe_candidates():
            candidates.append((exe, STABILITY_CANONICAL))
    # Unix (macOS + Linux) version managers and conda, shared between the
    # two platforms since both install pyenv/uv/asdf/conda the same way
    # under $HOME.
    # Unix(macOS + Linux)のバージョン管理ツールと conda。両プラットフォーム
    # とも $HOME 配下への導入方法が同じため共有する。
    for exe in _pyenv_unix_python_candidates():
        candidates.append((exe, STABILITY_VERSION_MANAGER))
    for exe in _uv_python_candidates():
        candidates.append((exe, STABILITY_VERSION_MANAGER))
    for exe in _asdf_unix_python_candidates():
        candidates.append((exe, STABILITY_VERSION_MANAGER))
    for exe in _unix_conda_python_candidates():
        candidates.append((exe, STABILITY_CONDA))
    return candidates


def _venv_root_if_any(python_exe: Path) -> Optional[Path]:
    """Return the venv root directory if python_exe lives inside one, else
    None. A venv's root is the directory holding pyvenv.cfg -- the parent
    of Scripts\\ (Windows) or bin/ (Unix), i.e. python_exe.parent.parent.
    python_exe が venv 内にある場合はその venv ルートディレクトリを返す、
    そうでなければ None。venv のルートは pyvenv.cfg があるディレクトリ --
    Scripts\\(Windows)/bin/(Unix)の親、すなわち python_exe.parent.parent。
    """
    venv_root = python_exe.parent.parent
    if (venv_root / "pyvenv.cfg").is_file():
        return venv_root
    return None


def _parse_pyvenv_cfg_home(pyvenv_cfg: Path) -> Optional[Path]:
    """Read a pyvenv.cfg file's `home = ...` line and return it as a Path,
    or None if the file is unreadable or has no `home` entry. `home` is
    the directory of the BASE interpreter that created the venv (not the
    venv's own bin/Scripts).
    pyvenv.cfg の `home = ...` 行を読み Path として返す。ファイルが読めない、
    または `home` エントリが無ければ None。`home` は venv を作成した
    ベースインタプリタのディレクトリ(venv 自身の bin/Scripts ではない)。
    """
    try:
        text = pyvenv_cfg.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep and key.strip().lower() == "home":
            home = value.strip()
            if home:
                return Path(home)
    return None


def _resolve_venv_seed(python_exe: Path) -> Optional[Path]:
    """If python_exe is itself a venv interpreter, resolve it to the BASE
    interpreter recorded in its pyvenv.cfg `home =` entry instead, so
    version/stability comparisons operate on the real, durable seed Python
    rather than an ephemeral venv that could vanish (e.g. a project venv
    the user deletes). Structurally eliminates nesting: follows the
    `home` chain exactly ONE hop; if the resolved base interpreter is
    ITSELF inside another venv, the candidate is discarded (returns None)
    rather than chased indefinitely -- a venv-of-a-venv is pathological
    and not worth the risk of an infinite loop.

    Returns python_exe unchanged (not None) if it was never inside a venv
    to begin with. Returns None if it IS a venv but the base interpreter
    cannot be resolved to a usable python (malformed pyvenv.cfg, missing
    `home`, or the home directory/interpreter no longer exists) -- the
    caller should then discard this candidate rather than treat a
    half-resolved path as usable.
    python_exe 自体が venv のインタプリタである場合、そのバージョン/安定度
    比較を(消えうる一時的な venv ではなく)実体のある永続的な種 Python に
    対して行えるよう、pyvenv.cfg の `home =` エントリが指すベース
    インタプリタに解決する。入れ子を構造的に排除する: `home` の連鎖は
    ちょうど1回だけ追跡する。解決したベースインタプリタ自体がさらに別の
    venv 内にある場合は、無限に追跡するのではなく候補を破棄する(None を
    返す) -- venv の入れ子は病的なケースであり、無限ループのリスクを
    冒してまで対応する価値はない。

    そもそも venv 内になければ python_exe をそのまま返す(None ではない)。
    venv ではあるがベースインタプリタを解決できない場合(pyvenv.cfg が
    壊れている、`home` が無い、home ディレクトリ/インタプリタが既に存在
    しない)は None を返す -- 呼び出し側は、中途半端に解決したパスを
    使用可能とみなさず、この候補を破棄すべき。
    """
    venv_root = _venv_root_if_any(python_exe)
    if venv_root is None:
        return python_exe  # not a venv at all -- use as-is / venvではないためそのまま使う

    home = _parse_pyvenv_cfg_home(venv_root / "pyvenv.cfg")
    if home is None:
        return None  # malformed/unreadable pyvenv.cfg -- discard / 破損pyvenv.cfg -- 破棄

    exe_names = ["python.exe"] if sys.platform == "win32" else ["python3", "python"]
    base_exe: Optional[Path] = None
    for name in exe_names:
        candidate = home / name
        if candidate.is_file():
            base_exe = candidate
            break
    if base_exe is None:
        return None  # home's interpreter is gone -- a dead venv seed / homeのインタプリタが消失 -- 破棄

    if _venv_root_if_any(base_exe) is not None:
        # One hop already used; a venv-of-a-venv beyond this is discarded.
        # 1回分のhopは既に使用済み。これ以上先の venv の入れ子は破棄する。
        return None

    info(f"Detected a virtual environment ({python_exe}); using its base "
         f"Python instead: {base_exe}")
    info(f"仮想環境を検出したため({python_exe})ベースの Python を使用します: {base_exe}")
    return base_exe


def _find_system_python_dir() -> Optional[Path]:
    """Directory of the best available system python executable, or None.
    使用可能な中で最善のシステム python 実行ファイルのディレクトリ、
    無ければ None。

    Composite ordering (applied across ALL candidates from
    _all_python_candidates(), not just the first one found; each
    candidate's venv seed is resolved first via _resolve_venv_seed() so
    a discovered venv interpreter is judged by its real base Python):
    version-preference BAND first, then STABILITY_* rank, then version
    recency within a tie -- i.e. "band > stability > newness":
    (a) a Python inside PYTHON_PREFERRED_MIN..PYTHON_PREFERRED_MAX
    (3.10-3.12) always wins over anything outside that band; among those,
    the most stable source wins (STABILITY_CANONICAL over
    STABILITY_VERSION_MANAGER over STABILITY_CONDA over
    STABILITY_VENV_RESOLVED); among ties on stability, the highest
    version wins. (b) anything OUTSIDE that band -- both older than 3.10
    AND 3.13+ -- is rejected outright, not merely deprioritized: 3.13+
    used to be accepted with a warning, but real-world failures were
    observed with it (2026-07-22 policy change), so it is now treated the
    same as an unsupported older version. The caller
    (_run_install_script()) offers an auto-install (winget/brew/apt, which
    installs 3.12) instead of silently proceeding with an unsupported
    interpreter. When the only candidates found are 3.13+, a bilingual
    warning names the rejected version before returning None, so the
    auto-install offer that follows has context instead of appearing out
    of nowhere.
    合成順序(_all_python_candidates() の全候補に対して適用する。最初に
    見つかった1つだけではない。各候補はまず _resolve_venv_seed() で venv
    の種を解決してから判定するため、発見された venv インタプリタはその
    実体のベース Python として評価される): まずバージョン適合バンド、
    次に STABILITY_* ランク、同点ならバージョンの新しさ -- すなわち
    「バンド > 安定度 > 新しさ」: (a) PYTHON_PREFERRED_MIN〜MAX(3.10〜
    3.12)の範囲内の Python が、範囲外の何よりも常に優先される。その中では
    最も安定した種が勝つ(STABILITY_CANONICAL > STABILITY_VERSION_MANAGER
    > STABILITY_CONDA > STABILITY_VENV_RESOLVED の順)。安定度が同点なら
    最も新しいバージョンが勝つ。(b) この範囲外のもの -- 3.10未満と3.13以降
    の両方 -- は単に優先度を下げるのではなく無条件に不採用とする:
    以前は3.13+を警告付きで受理していたが、実際に動作しない事例が報告
    されたため(2026-07-22の方針変更)、対応外の古いバージョンと同様の
    扱いに変更した。呼び出し元(_run_install_script())は、未対応の
    インタプリタで黙って続行する代わりに自動インストール(winget/brew/apt
    経由、3.12を導入)を提案する。見つかった候補が3.13+のみだった場合は、
    None を返す前に不採用としたバージョンを名指しした英日併記の警告を
    出すことで、後続の自動インストール提案に脈絡を持たせる。

    Why this exists (the 9009 bug, Windows): ESP-IDF's own install.bat
    calls `python` by name, so a real python.exe must be on PATH for it.
    Under the CLI, install.bat already put one there; under the GUI, the
    frozen StampFly Setup app is `sys.executable` (no python.exe beside
    it), so we must discover a system Python separately -- otherwise
    ESP-IDF tool install fails with exit code 9009 ("command not found"),
    observed on a workshop Windows laptop 2026-07-20. The same
    "install.sh calls python3 by name" reasoning applies on macOS/Linux,
    which is why this function covers all three platforms rather than
    Windows alone.
    存在理由(9009 バグ、Windows): ESP-IDF の install.bat は `python` を
    名前で呼ぶため、本物の python.exe が PATH 上に必要。CLI では
    install.bat が既に配置済みだが、GUI では凍結された StampFly Setup
    アプリが `sys.executable`(隣に python.exe は無い)のため、システム
    Python を別途発見しないと ESP-IDF ツール導入が exit 9009
    ("コマンドが見つからない")で失敗する(2026-07-20 に講習用 Windows
    ノートで観測)。「install.sh も python3 を名前で呼ぶ」という同じ理屈は
    macOS/Linux にも当てはまるため、本関数は Windows 単独ではなく3
    プラットフォーム全てを対象にする。
    """
    # (path, version, stability_rank) for the best in-band candidate seen
    # so far; `rejected_newer` tracks only the LOWEST 3.13+ version seen,
    # purely to name it in the warning below -- it is never returned.
    # 範囲内で見つけた最善候補の (パス, バージョン, 安定度ランク)。
    # `rejected_newer` は下の警告でバージョンを名指しするためだけに、
    # 見つかった3.13+の中で最も低いバージョンのみを追跡する -- 戻り値には
    # 使わない。
    preferred: Optional[Tuple[Path, Tuple[int, int], int]] = None
    rejected_newer: Optional[Tuple[int, int]] = None
    for exe, stability in _all_python_candidates():
        if not exe.is_file():
            continue
        resolved = _resolve_venv_seed(exe)
        if resolved is None:
            continue  # dead / doubly-nested venv seed -- discard candidate
        if resolved != exe:
            # A venv seed was resolved to its base Python: rank by the
            # "least deliberate discovery" tier regardless of the
            # original source (see STABILITY_VENV_RESOLVED's docstring).
            # venv の種をベース Python に解決した: 元の発見元に関わらず
            # 「最も狙って選ばれたのではない発見」の階層でランク付けする
            # (STABILITY_VENV_RESOLVED のdocstring参照)。
            stability = STABILITY_VENV_RESOLVED
            exe = resolved
        if not exe.is_file():
            continue
        # Windows: the returned directory's whole purpose is to make the
        # literal name `python.exe` resolvable for ESP-IDF's install.bat
        # (see the 9009 note above). A candidate that lives in a directory
        # with no real python.exe -- e.g. pyenv-win's `python.bat` shim,
        # which shutil.which("python") returns via PATHEXT -- must be
        # discarded: steering PATH to its shims directory reproduces the
        # exact 9009 failure this function exists to prevent (observed
        # 2026-07-23, GUI installer on a pyenv-win machine). pyenv installs
        # are still honored via _windows_pyenv_win_python_dir(), which
        # points at the real versions\<ver> directory instead.
        # Windows: この関数が返すディレクトリの目的は、ESP-IDF の
        # install.bat が呼ぶ `python.exe` という名前をそのまま解決可能に
        # すること(上の 9009 の注記参照)。本物の python.exe が無い
        # ディレクトリに居る候補 -- 例: shutil.which("python") が PATHEXT
        # 経由で返す pyenv-win の `python.bat` shim -- は除外必須:
        # shims ディレクトリへ PATH を誘導すると、この関数が防ぐはずの
        # 9009 障害をそのまま再現する(2026-07-23、pyenv-win 環境の GUI
        # インストーラーで観測)。pyenv のインストール自体は、実体の
        # versions\<ver> ディレクトリを指す _windows_pyenv_win_python_dir()
        # が引き続き拾う。
        if sys.platform == "win32" and not (exe.parent / "python.exe").is_file():
            continue
        version = _python_version_info(exe)
        if version is None:
            continue

        if PYTHON_PREFERRED_MIN <= version <= PYTHON_PREFERRED_MAX:
            if preferred is None or stability < preferred[2] or (
                stability == preferred[2] and version > preferred[1]
            ):
                preferred = (exe.parent, version, stability)
        elif version > PYTHON_PREFERRED_MAX:
            # Rejected outright (2026-07-22 policy change) -- tracked only
            # for the informative message below, never selected.
            # 無条件に不採用(2026-07-22の方針変更) -- 下の警告メッセージの
            # ためだけに記録し、選出はしない。
            if rejected_newer is None or version < rejected_newer:
                rejected_newer = version
        # else: older than PYTHON_PREFERRED_MIN -- rejected, not tracked.
        # それ以外(PYTHON_PREFERRED_MIN未満)は不採用のため記録しない。

    if preferred is not None:
        return preferred[0]
    if rejected_newer is not None:
        warn(f"Found Python {rejected_newer[0]}.{rejected_newer[1]}, but "
             f"versions newer than {PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} "
             f"are not supported (real failures have been observed) -- "
             f"{PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
             f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} is required.")
        warn(f"Python {rejected_newer[0]}.{rejected_newer[1]} が見つかりましたが、"
             f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} より新しい"
             "バージョンは未対応です(動作しない事例が確認されています) -- "
             f"{PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}〜"
             f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} が必要です。")
    return None


def _detect_linux_package_manager() -> Optional[str]:
    """Return the first of apt/dnf/pacman found on PATH, or None.
    PATH上で最初に見つかった apt/dnf/pacman を返す、無ければ None。"""
    for manager in ("apt", "dnf", "pacman"):
        if shutil.which(manager):
            return manager
    return None


# -- SILS development toolchain guidance (macOS/Linux) -----------------------
# SILS開発ツールチェーン案内（macOS/Linux）
#
# macOS/Linux print guidance only (the system's own gcc/clang builds
# simulator/sils/ fine there -- see simulator/sils/README.md). The Windows
# install itself delegates to `sf sils install-toolchain`
# (lib/sfcli/utils/msys2_install.py) via _run_sf_in_idf_env() -- see
# _install_sils_toolchain()'s docstring for why it is not reimplemented here.
#
# macOS/Linuxでは案内表示のみ（システム自身のgcc/clangでsimulator/sils/を
# 問題なくビルドできる -- simulator/sils/README.md参照）。Windows側の実際の
# 導入は `_run_sf_in_idf_env()` 経由で `sf sils install-toolchain`
# （lib/sfcli/utils/msys2_install.py）に委譲する -- ここで再実装しない理由は
# _install_sils_toolchain() のdocstring参照。


def _linux_sils_toolchain_hint(manager: Optional[str]) -> str:
    """Build a one-line package-manager hint for the C++17 toolchain +
    cmake + ninja the SILS host build needs on Linux.
    Linux上でSILSホストビルドが必要とするC++17ツールチェーン+cmake+ninja
    のパッケージマネージャコマンドを1行で組み立てる。"""
    if manager == "apt":
        return "sudo apt install -y build-essential cmake ninja-build"
    if manager == "dnf":
        return "sudo dnf install -y gcc-c++ cmake ninja-build"
    if manager == "pacman":
        return "sudo pacman -S --noconfirm --needed base-devel cmake ninja"
    return "install a C++17 toolchain (gcc/g++), cmake, and ninja via your package manager"


def _linux_python_install_command(manager: str) -> List[str]:
    """Build the argv for installing Python 3.12 with `manager`.
    `manager` で Python 3.12 を導入する argv を組み立てる。

    apt's `python3.12-venv` is included because Debian/Ubuntu split venv
    support out of the base python3.X package -- without it, ESP-IDF's own
    venv creation (idf_tools.py) fails even though `python3.12` itself
    runs fine. dnf/pacman do not split this out the same way.
    apt の `python3.12-venv` を含める理由: Debian/Ubuntu は venv 機能を
    python3.X 本体パッケージから分離しているため、これが無いと
    `python3.12` 自体は動いても ESP-IDF 自身の venv 作成
    (idf_tools.py)が失敗する。dnf/pacman はこの分離を行わない。
    """
    if manager == "apt":
        return ["sudo", "apt", "install", "-y", "python3.12", "python3.12-venv"]
    if manager == "dnf":
        return ["sudo", "dnf", "install", "-y", "python3.12"]
    if manager == "pacman":
        return ["sudo", "pacman", "-S", "--noconfirm", "python"]
    return []


def _print_manual_python_install_hint() -> None:
    """Print platform-specific manual install guidance as a last resort,
    when auto-install was declined, unavailable, or failed.
    自動インストールが辞退・不可・失敗のいずれかだった場合の、最後の
    手段としてのプラットフォーム別手動インストール案内を表示する。
    """
    error(f"Please install Python {PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
          f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} manually, then re-run this installer:")
    if sys.platform == "win32":
        error("    winget install Python.Python.3.12")
        error("  or download from https://www.python.org/downloads/windows/")
        error("  (make sure to keep 'Add python.exe to PATH' checked)")
    elif sys.platform == "darwin":
        error("    brew install python@3.12")
        error("  or download from https://www.python.org/downloads/macos/")
    else:
        manager = _detect_linux_package_manager()
        if manager:
            error(f"    {' '.join(_linux_python_install_command(manager))}")
        else:
            error("    Install python3.12 via your distribution's package manager,")
            error("    or from https://www.python.org/downloads/source/")
    error(f"Python {PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
          f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]}"
          " を手動でインストールし、このインストーラーを再実行してください。")


def _auto_install_python_windows() -> bool:
    """Windows: install Python 3.12 via winget if available.
    Windows: winget があれば経由で Python 3.12 をインストールする。

    Returns whether a preferred-range Python is present afterward. winget
    installing to a fresh location does not update this process's own PATH
    (Windows broadcasts an environment-change message, but only new
    processes pick it up), so success is verified by re-running
    _find_system_python_dir() rather than assuming winget's own exit code
    means the interpreter is now reachable -- it re-scans known install
    locations (see _windows_python_dir_candidates()), which does not
    depend on PATH at all.
    winget が新規インストールしても、このプロセス自身の PATH は更新
    されない(Windows は環境変更通知を送るが、新規プロセスのみがそれを
    反映する)。そのため、winget自体の終了コードだけでインタプリタに
    到達可能になったとみなさず、_find_system_python_dir() を再実行して
    確認する -- これは PATH に一切依存せず既知インストール先を再走査する
    (_windows_python_dir_candidates() 参照)。
    """
    winget = shutil.which("winget")
    if not winget:
        _print_manual_python_install_hint()
        return False
    info("Installing Python 3.12 via winget (this may take a while)...")
    try:
        rc = _stream_subprocess([
            winget, "install", "--id", "Python.Python.3.12", "--silent",
            "--accept-package-agreements", "--accept-source-agreements",
        ])
    except FileNotFoundError:
        _print_manual_python_install_hint()
        return False
    if rc != 0:
        warn(f"winget install exited with code {rc}.")
        _print_manual_python_install_hint()
        return False
    return _find_system_python_dir() is not None


def _auto_install_python_macos() -> bool:
    """macOS: install Python 3.12 via Homebrew if available.
    macOS: Homebrew があれば経由で Python 3.12 をインストールする。

    Success is verified the same way as _auto_install_python_windows():
    re-running _find_system_python_dir(), whose macOS candidate list
    (_macos_python_exe_candidates()) already includes the keg-only
    `$(brew --prefix)/opt/python@3.12/bin` layout `brew install python@3.12`
    actually produces.
    _auto_install_python_windows() と同じ方法で成否を確認する:
    _find_system_python_dir() を再実行する。そのmacOS候補リスト
    (_macos_python_exe_candidates())は、`brew install python@3.12` が
    実際に作る keg-only な `$(brew --prefix)/opt/python@3.12/bin`
    レイアウトを既にカバーしている。
    """
    brew = shutil.which("brew")
    if not brew:
        _print_manual_python_install_hint()
        return False
    info("Installing Python 3.12 via Homebrew (this may take a while)...")
    try:
        rc = _stream_subprocess([brew, "install", "python@3.12"])
    except FileNotFoundError:
        _print_manual_python_install_hint()
        return False
    if rc != 0:
        warn(f"brew install exited with code {rc}.")
        _print_manual_python_install_hint()
        return False
    return _find_system_python_dir() is not None


def _auto_install_python_linux() -> bool:
    """Linux: install Python 3.12 via apt/dnf/pacman, gated on an
    interactive terminal and explicit y/n consent regardless of any
    --auto-install-python flag.
    Linux: apt/dnf/pacman 経由で Python 3.12 をインストールする。
    --auto-install-python フラグの有無に関わらず、対話端末での明示的な
    y/n 同意を必須とする。

    Deliberately never runs non-interactively (SF_INSTALLER_NONINTERACTIVE=1
    or no TTY): sudo's password prompt must go to the user's own terminal,
    which a GUI frontend or a scripted/CI run cannot supply. In both of
    those cases this prints the command and returns False rather than
    attempting it -- see the module's "Stability contract" for why the
    non-interactive path must never block on stdin.
    非対話(SF_INSTALLER_NONINTERACTIVE=1 またはTTY無し)では意図的に
    絶対実行しない: sudo のパスワード入力はユーザー自身の端末に委ねる
    必要があり、GUIフロントエンドやスクリプト/CI実行はそれを提供
    できない。どちらの場合もコマンドを表示するに留め、実行は試みない --
    非対話経路が stdin でブロックしてはならない理由は本ファイル冒頭の
    「安定契約」節を参照。

    Uses plain subprocess.run() (not _stream_subprocess()) so sudo's own
    password prompt and any package-manager confirmation prompts pass
    through to/from the user's real terminal via inherited stdin/stdout,
    rather than being captured into our own streaming pipe.
    素の subprocess.run()(_stream_subprocess() ではない)を使う。これに
    より sudo 自身のパスワード入力やパッケージマネージャの確認プロンプトが
    継承された stdin/stdout 経由でユーザーの実端末とやり取りされる
    (自前のストリーミングパイプに捕捉されない)。
    """
    manager = _detect_linux_package_manager()
    if not manager:
        _print_manual_python_install_hint()
        return False
    command = _linux_python_install_command(manager)
    command_str = " ".join(command)

    if os.environ.get("SF_INSTALLER_NONINTERACTIVE") == "1" or not sys.stdin.isatty():
        info(f"To install Python 3.12, run this yourself: {command_str}")
        info(f"Python 3.12 を導入するには、以下を自分で実行してください: {command_str}")
        return False

    response = prompt(f"Run `{command_str}` now? (requires sudo) [y/N]", "N")
    if response.lower() not in ("y", "yes"):
        info(f"Skipped. Run manually when ready: {command_str}")
        return False

    info(f"Running: {command_str}")
    try:
        result = subprocess.run(command)
    except (OSError, subprocess.SubprocessError) as exc:
        warn(f"Failed to run install command: {exc}")
        _print_manual_python_install_hint()
        return False
    if result.returncode != 0:
        warn(f"Install command exited with code {result.returncode}.")
        return False
    return _find_system_python_dir() is not None


def _offer_python_auto_install(auto_install_python: bool) -> bool:
    """Offer, and if accepted attempt, an automatic install of a
    preferred-range system Python. Returns whether a usable one is
    present afterward.
    優先範囲のシステムPythonの自動インストールを提案し、同意されれば
    試みる。事後に使用可能なものが存在するかを返す。

    Consent (module docstring's "Stability contract"): in interactive
    mode (no SF_INSTALLER_NONINTERACTIVE), always asks via the existing
    prompt() y/n regardless of `auto_install_python`. In non-interactive
    mode, only proceeds (Windows/macOS) when `auto_install_python` is
    True -- this is what --auto-install-python / Installer.run(...,
    auto_install_python=True) controls. Linux's sudo-gated path ignores
    `auto_install_python` entirely and is delegated to
    _auto_install_python_linux(), which never runs unattended (see its
    own docstring).
    同意の取り方(モジュールdocstringの「安定契約」参照): 対話モード
    (SF_INSTALLER_NONINTERACTIVE 無し)では、`auto_install_python` の値に
    関わらず既存の prompt() で常に y/n を尋ねる。非対話モードでは
    (Windows/macOSのみ)`auto_install_python` が True の場合に限り進める --
    これが --auto-install-python / Installer.run(...,
    auto_install_python=True) が制御する内容。Linuxのsudo経路は
    `auto_install_python` を一切無視して _auto_install_python_linux() に
    委ねる(無人実行は絶対にしない。同関数のdocstring参照)。
    """
    header("System Python not found / システムPythonが見つかりません")
    warn(f"ESP-IDF setup needs a system Python in the "
         f"{PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
         f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} range, but none was found.")
    warn(f"ESP-IDFのセットアップには {PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}〜"
         f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} のシステムPythonが必要ですが、"
         "見つかりませんでした。")

    if sys.platform == "linux":
        return _auto_install_python_linux()

    non_interactive = os.environ.get("SF_INSTALLER_NONINTERACTIVE") == "1"
    if non_interactive:
        if not auto_install_python:
            _print_manual_python_install_hint()
            return False
    else:
        response = prompt("Attempt to automatically install Python 3.12 now? [y/N]", "N")
        if response.lower() not in ("y", "yes"):
            _print_manual_python_install_hint()
            return False

    if sys.platform == "win32":
        return _auto_install_python_windows()
    if sys.platform == "darwin":
        return _auto_install_python_macos()
    _print_manual_python_install_hint()  # unreachable in practice (linux handled above)
    return False


# Environment variables that signal an already-activated Python
# environment (a user venv, conda env, or pyenv shim-selection). Their
# presence in a CHILD process's environment can steer that child toward
# the wrong interpreter/site-packages even when we already invoke the
# target venv's python by absolute path -- e.g. pip still consults
# VIRTUAL_ENV/PYTHONHOME for some of its own behavior, and a pyenv shim on
# PATH honors PYENV_VERSION over the absolute path it was invoked with in
# some edge cases. Shared by every subprocess launch site in this file
# (_clean_env_for_cmd(), _run_in_idf_env(), _run_sf_in_idf_env()) via
# _sanitize_activated_env() so a newly-discovered "activated environment"
# variable only needs to be added once, here.
# 既に activate 済みの Python 環境(ユーザーの venv、conda 環境、pyenv の
# シェル選択)を示す環境変数。これらが子プロセスの環境に残っていると、
# 対象 venv の python を絶対パスで直接呼んでいても誤ったインタプリタ/
# site-packages へ誘導されうる -- 例えば pip は自身の一部の挙動で今も
# VIRTUAL_ENV/PYTHONHOME を参照するし、PATH 上の pyenv shim は場合によって
# 絶対パス呼び出しより PYENV_VERSION を優先することがある。本ファイル内の
# 全ての子プロセス起動箇所(_clean_env_for_cmd()、_run_in_idf_env()、
# _run_sf_in_idf_env())が _sanitize_activated_env() 経由でこれを共有する
# ことで、新たに見つかった「activate済み環境」変数はここに一度追加する
# だけで済む。
_ACTIVATED_ENV_VARS = (
    "VIRTUAL_ENV",
    "PYTHONHOME",
    "PYTHONPATH",
    "CONDA_PREFIX",
    "CONDA_DEFAULT_ENV",
    "CONDA_SHLVL",
    "PYENV_VERSION",
)


def _sanitize_activated_env(env: dict) -> dict:
    """Remove all _ACTIVATED_ENV_VARS from `env` in place, and return it
    (for chaining). Deliberately does NOT touch PATH -- stripping an
    activated venv/conda env's Scripts/bin directory from PATH has wider
    side effects (e.g. removing access to other tools the user's shell
    session depends on) than this installer's job warrants; every
    subprocess call site that matters already invokes the TARGET venv's
    python by absolute path rather than relying on `python` resolving
    correctly via PATH.
    `env` から _ACTIVATED_ENV_VARS を全て除去し(その場で変更)、`env` 自体を
    返す(呼び出しの連結用)。PATH には意図的に触れない -- activate 済みの
    venv/conda 環境の Scripts/bin ディレクトリを PATH から取り除くのは、
    本インストーラーの役目を超えた副作用が広い(例: ユーザーのシェル
    セッションが依存する他のツールへのアクセスを失わせる)。重要な
    subprocess 呼び出し箇所は、いずれも `python` が PATH 経由で正しく
    解決されることに頼らず、既に対象 venv の python を絶対パスで直接
    呼んでいる。
    """
    for var in _ACTIVATED_ENV_VARS:
        env.pop(var, None)
    return env


def _clean_env_for_cmd() -> dict:
    """Return environment suitable for running .bat scripts via cmd.exe.
    cmd.exe 経由で .bat を実行するための環境を構築

    - Strips MSYSTEM (ESP-IDF .bat refuses to run under MINGW/Git Bash)
    - Strips activated-environment variables (see _sanitize_activated_env())
      so a pre-activated user venv/conda/pyenv selection cannot steer
      install.bat's own `python` resolution.
    - Ensures a REAL system python.exe directory is on PATH so ESP-IDF's
      install.bat (which calls `python` by name) can find it. Under the CLI
      that is `sys.executable`'s own directory; under the frozen GUI
      `sys.executable` is StampFly Setup itself (no python.exe there), so we
      fall back to discovering a system Python -- see _find_system_python_dir().
    - MSYSTEM を除去(ESP-IDF の .bat は MINGW/Git Bash 下での実行を拒否)
    - activate済み環境変数を除去(_sanitize_activated_env() 参照)し、事前
      activate済みのユーザー venv/conda/pyenv 選択が install.bat 自身の
      `python` 解決を誤誘導しないようにする
    - 本物のシステム python.exe のディレクトリを PATH に載せ、ESP-IDF の
      install.bat(`python` を名前で呼ぶ)が見つけられるようにする。CLI では
      `sys.executable` の隣がそれだが、凍結 GUI では `sys.executable` が
      StampFly Setup 自身(python.exe は無い)なので、システム Python の
      発見にフォールバックする -- _find_system_python_dir() 参照。
    """
    env = os.environ.copy()
    env.pop("MSYSTEM", None)
    _sanitize_activated_env(env)

    # Prefer sys.executable's directory when it actually holds a python
    # interpreter (the CLI case); otherwise discover a system Python (the
    # frozen-GUI case).
    # sys.executable の隣に実際に python インタプリタがある場合(CLIの場合)は
    # それを優先し、無ければシステム Python を発見する(凍結GUIの場合)。
    python_dir: Optional[Path] = None
    exe_dir = Path(sys.executable).parent
    exe_python = exe_dir / ("python.exe" if sys.platform == "win32" else "python")
    if not getattr(sys, "frozen", False) and exe_python.exists():
        python_dir = exe_dir
    elif sys.platform == "win32":
        python_dir = _find_system_python_dir()

    if python_dir is not None:
        python_dir_str = str(python_dir)
        current_path = env.get("PATH", "")
        # Prepend UNCONDITIONALLY (not append, and not skip-if-present):
        # a WindowsApps python stub or a pyenv shims directory earlier on
        # PATH must not win over the real interpreter we found. The old
        # "skip if the directory already appears anywhere in PATH" check
        # left the stub winning when the real directory sat AFTER it; a
        # duplicated entry further down PATH is harmless.
        # 無条件で先頭に付ける(末尾ではなく、既存チェックによる省略も
        # しない): PATH 上でより前にある WindowsApps の python スタブや
        # pyenv の shims ディレクトリが、発見した本物のインタプリタに
        # 勝ってはならない。従来の「PATH のどこかに既にあれば省略」では、
        # 実体ディレクトリがスタブより後ろにある場合にスタブが勝って
        # いた。PATH 後方の重複エントリは無害。
        env["PATH"] = python_dir_str + os.pathsep + current_path
    return env


def _env_with_python3_steering() -> dict:
    """Environment for running ESP-IDF's install.sh on macOS/Linux, with
    PATH steered so its own ``tools/detect_python.sh`` -- which tries the
    bare ``python3`` name FIRST in its candidate list -- resolves to an
    interpreter inside the PYTHON_PREFERRED_MIN..MAX band, instead of
    whatever ``/usr/bin/python3`` happens to be.

    Why this exists (2026-07-22 observed failure, macOS): a GUI-launched
    (frozen) installer runs with none of the user's pyenv/Homebrew PATH
    entries, only the OS default PATH. On stock macOS that PATH resolves
    `python3` to /usr/bin/python3 (3.9), so ESP-IDF's install.sh created
    ~/.espressif/python_env/idf5.5_py3.9_env -- outside this ecosystem's
    tested range -- and the subsequent `pip install -e` of sfcli failed on
    its `requires-python >=3.10,<3.13`. The Windows branch of
    _run_install_script() already avoids the analogous problem via
    _clean_env_for_cmd()'s PATH steering; this function gives the Unix
    branch the same protection.

    Resolves the steering directory from _find_system_python_dir() (the
    same in-band system Python already used elsewhere in this file). If
    that directory already contains a literal `python3` executable, it is
    prepended to PATH as-is. Otherwise (e.g. a pyenv version directory that
    only has `python3.12`), a small temp directory containing a `python3`
    symlink to the real interpreter is created and prepended instead,
    since detect_python.sh looks for the bare name -- `python3.12` alone on
    PATH would not be found. The real interpreter is picked, in order:
    `python_dir/python3` (already handled above -- unreachable here),
    then the newest `python_dir/python3.*` whose _python_version_info() is
    inside the preferred band, then `python_dir/python` as a last resort.

    Falls back to no PATH steering (a plain copy of the ambient
    environment) if no system Python could be found at all -- the caller
    is expected to have already handled that case (see
    _run_install_script()'s own _find_system_python_dir() check just
    above its call site).
    macOS/Linux で ESP-IDF の install.sh を実行するための環境を、その
    内部の ``tools/detect_python.sh``(候補リストの**先頭**で素の
    ``python3`` という名前を試す)が PYTHON_PREFERRED_MIN〜MAX の範囲内の
    インタプリタを解決するよう PATH を誘導して返す(たまたま存在する
    `/usr/bin/python3` にではなく)。

    存在理由(2026-07-22 に観測した実障害、macOS): GUI から起動される
    (凍結された)インストーラーは、ユーザーの pyenv/Homebrew の PATH
    エントリを一切持たず、OS既定の PATH のみで動く。素の macOS ではこの
    PATH で `python3` は /usr/bin/python3(3.9)に解決されるため、
    ESP-IDF の install.sh が本エコシステムの検証範囲外である
    ~/.espressif/python_env/idf5.5_py3.9_env を作成してしまい、後続の
    sfcli の `pip install -e` がその `requires-python >=3.10,<3.13` で
    失敗した。_run_install_script() の Windows 分岐は
    _clean_env_for_cmd() の PATH 誘導で既に同種の問題を回避済みであり、
    本関数は Unix 分岐にも同じ保護を与える。

    誘導先ディレクトリは _find_system_python_dir()(本ファイルの他箇所でも
    使う、範囲内のシステム Python と同じもの)から取得する。そのディレ
    クトリに文字通り `python3` という実行ファイルが既にあればそのまま
    PATH 先頭に付ける。無い場合(例: `python3.12` しか無い pyenv の
    バージョンディレクトリ)は、実インタプリタへの `python3` という
    シンボリックリンクを含む小さな一時ディレクトリを作り、それを代わりに
    PATH 先頭に付ける -- detect_python.sh は素の名前を探すため、
    `python3.12` だけが PATH にあっても見つけられない。実インタプリタの
    特定順は: `python_dir/python3`(上で既に判定済み -- ここには来ない)、
    次に `_python_version_info()` が適合バンド内と判定した最新の
    `python_dir/python3.*`、最後に `python_dir/python`。

    システム Python が全く見つからない場合は PATH 誘導なし(環境の単純
    コピー)にフォールバックする -- その場合の対処は呼び出し元
    (_run_install_script() 自身の呼び出し直前にある
    _find_system_python_dir() チェック)が既に済ませている前提。
    """
    env = os.environ.copy()
    _sanitize_activated_env(env)

    python_dir = _find_system_python_dir()
    if python_dir is None:
        # Defensive fallback -- callers are expected to have already
        # bailed out via their own _find_system_python_dir() check.
        # 防御的フォールバック -- 呼び出し元は自身の
        # _find_system_python_dir() チェックで既に処理済みのはず。
        return env

    steering_dir = python_dir
    bare_python3 = python_dir / "python3"
    bare_version = _python_version_info(bare_python3) if bare_python3.exists() else None
    if bare_version is None or not (PYTHON_PREFERRED_MIN <= bare_version <= PYTHON_PREFERRED_MAX):
        # Either no literal `python3` in this directory (e.g. only
        # `python3.12` is present), or the `python3` that IS there is
        # outside the preferred band (e.g. Homebrew's bin holding both
        # `python3.12` and a `python3` that points at 3.14+ -- the
        # in-band exe is what got this directory selected, not the bare
        # name). detect_python.sh looks for the bare name, so synthesize
        # a shim directory with a `python3` symlink to the in-band
        # interpreter instead.
        # このディレクトリに素の `python3` が無い(例: `python3.12` のみ)、
        # または存在する `python3` が適合バンド外(例: Homebrew の bin に
        # `python3.12` と 3.14+ を指す `python3` が同居 -- この
        # ディレクトリが選ばれた理由は範囲内の実行ファイルであり、素の
        # 名前ではない)。detect_python.sh は素の名前を探すため、範囲内の
        # インタプリタへの `python3` シンボリックリンクを含む shim
        # ディレクトリを合成する。
        real_interpreter: Optional[Path] = None
        for exe in sorted(python_dir.glob("python3.*"), reverse=True):
            version = _python_version_info(exe)
            if version is not None and PYTHON_PREFERRED_MIN <= version <= PYTHON_PREFERRED_MAX:
                real_interpreter = exe
                break
        if real_interpreter is None:
            fallback = python_dir / "python"
            if fallback.exists():
                real_interpreter = fallback
        if real_interpreter is not None:
            shim_dir = Path(tempfile.mkdtemp(prefix="sf_python3_shim_"))
            (shim_dir / "python3").symlink_to(real_interpreter)
            steering_dir = shim_dir

    steering_dir_str = str(steering_dir)
    current_path = env.get("PATH", "")
    if steering_dir_str not in current_path.split(os.pathsep):
        # Prepend (not append): PATH may already contain another
        # `python3` (e.g. a stale one) that must not win over this one.
        # 先頭に付ける(末尾ではない): PATH に既に別の `python3`(古いもの
        # 等)がある場合でも、これに勝たないようにする。
        env["PATH"] = steering_dir_str + os.pathsep + current_path
    return env


def _report_git_not_found() -> None:
    """Print an actionable, bilingual error when git itself is missing
    from PATH (a bare FileNotFoundError from subprocess otherwise looks
    like an obscure crash to a newcomer running this installer for the
    first time). This runs on every OS (ESP-IDF's git clone/submodule
    steps are cross-platform), so the URL/hint must not be Windows-only.
    git 自体が PATH に無い場合、対処可能な英日併記エラーを表示する
    (subprocess の素の FileNotFoundError だけでは、初めてこの
    インストーラーを実行する初心者には不可解なクラッシュに見えてしまう)。
    全OSで実行されうる経路のため(ESP-IDFのgit clone/submodule手順は
    クロスプラットフォーム)、URL/案内をWindows専用にしてはならない。
    """
    error("Git was not found on PATH.")
    if sys.platform == "win32":
        error("Install it with: winget install Git.Git")
        error("Or download from: https://git-scm.com/download/win")
    elif sys.platform == "darwin":
        error("Install it with: brew install git")
        error("Or install the Xcode Command Line Tools: xcode-select --install")
    else:
        error("Install it with your package manager, e.g.: sudo apt install git")
    error("Then re-run this installer.")
    error("Git が見つかりません。")
    if sys.platform == "win32":
        error("winget install Git.Git でインストールするか、")
        error("https://git-scm.com/download/win からダウンロードしてください。")
    elif sys.platform == "darwin":
        error("brew install git でインストールするか、")
        error("Xcode Command Line Tools（xcode-select --install）を導入してください。")
    else:
        error("パッケージマネージャでインストールしてください（例: sudo apt install git）。")
    error("導入後、このインストーラーを再実行してください。")


def _stream_subprocess(
    cmd,
    cwd: Optional[Path] = None,
    shell: bool = False,
    env: Optional[dict] = None,
) -> int:
    """Run cmd, streaming its merged stdout+stderr to our own stdout line
    by line instead of capturing it silently, and return the exit code.

    Why: a frozen/redirected caller (the GUI installer's stdout capture,
    see the module docstring's Stability contract) sees nothing at all
    from a plain `subprocess.run(..., capture_output=True)` until the
    whole command finishes -- for a multi-minute `git clone` or
    `install.bat`, that reads as an indefinite freeze rather than
    progress. Streaming line-by-line with `print(..., flush=True)` lets
    the caller's own stdout redirection (e.g. contextlib.redirect_stdout
    into a queue) surface each line as it happens.

    `\\r` (used by progress meters like git's own `--progress` output,
    which does not end lines with `\\n`) is normalized to `\\n` so those
    updates still appear as discrete log lines rather than being lost
    inside a single unterminated read.

    On Windows, CREATE_NO_WINDOW suppresses the console window that would
    otherwise flash open for a python.exe- or cmd.exe-hosted child when
    this runs inside a --windowed (no-console) frozen GUI.

    Raises FileNotFoundError if cmd itself cannot be spawned (e.g. `git`
    is not on PATH); callers are expected to catch this and print a
    specific, actionable message (see _report_git_not_found() above) --
    unlike a plain non-zero exit code, which this function reports
    through its return value rather than an exception.

    cmd を実行し、標準出力+標準エラーを結合して黙って捕捉するのではなく
    1行ずつ自身の標準出力へ流し、終了コードを返す。

    理由: 凍結/リダイレクトされた呼び出し元(GUIインストーラーの標準出力
    捕捉、本ファイル冒頭の安定契約を参照)は、素の
    `subprocess.run(..., capture_output=True)` だとコマンド全体が終わる
    まで何も見えない -- 数分かかる `git clone` や `install.bat` では、
    進捗ではなく無期限のフリーズに見えてしまう。`print(..., flush=True)`
    で1行ずつ流すことで、呼び出し元自身の標準出力リダイレクト(例:
    contextlib.redirect_stdout でキューへ流す)が発生と同時に各行を
    表示できる。

    `\\r`(git 自身の `--progress` 出力のような進捗表示が使う。`\\n` で
    行を終端しない)は `\\n` に正規化し、1つの未終端読み込みの中に
    埋もれさせず個別のログ行として見せる。

    Windows では CREATE_NO_WINDOW を付け、--windowed(コンソール無し)の
    凍結GUI内で実行した際に python.exe や cmd.exe をホストする子プロセス
    用のコンソール窓がちらつくのを抑止する。

    cmd 自体を起動できない場合(例: `git` が PATH に無い)は
    FileNotFoundError を送出する。呼び出し側でこれを捕捉し、具体的で
    対処可能なメッセージ(上の _report_git_not_found() 参照)を表示する
    こと -- 単純な非ゼロ終了コードは(例外ではなく)この関数の戻り値で
    報告される。
    """
    popen_kwargs = dict(
        cwd=cwd,
        shell=shell,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding="utf-8",
        errors="replace",
        env=env,
    )
    if os.name == "nt":
        popen_kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
    process = subprocess.Popen(cmd, **popen_kwargs)

    assert process.stdout is not None  # guaranteed by stdout=subprocess.PIPE above
    buffer = ""
    while True:
        chunk = process.stdout.read(4096)
        if not chunk:
            break
        buffer += chunk.replace("\r", "\n")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            print(line, flush=True)
    if buffer:
        print(buffer, flush=True)
    return process.wait()


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
        # Pick the venv whose Python version is (1) inside the
        # PYTHON_PREFERRED_MIN..MAX band, preferred over anything outside
        # it, and (2) among ties on that, numerically newest -- NOT a
        # lexicographic (dictionary-order) sort of the directory name.
        # A plain `sorted(candidates, reverse=True)` on directory names like
        # "idf5.5_py3.9_env" vs "idf5.5_py3.12_env" compares the strings
        # character-by-character, so "9" > "1" makes py3.9 sort ahead of
        # py3.12 -- exactly the real failure observed 2026-07-22: ESP-IDF's
        # own install.sh (via tools/detect_python.sh, which tries the bare
        # `python3` name first) created idf5.5_py3.9_env on a machine whose
        # PATH had no pyenv/Homebrew, and this dictionary-order sort then
        # picked that 3.9 venv over the already-present, correct
        # idf5.5_py3.12_env, so `pip install -e` failed on the sfcli
        # package's `requires-python >=3.10,<3.13`.
        # 選ぶ venv は (1) PYTHON_PREFERRED_MIN〜MAX の範囲内を範囲外より
        # 優先し、(2) その中では数値としてのバージョンが新しい順とする --
        # ディレクトリ名の辞書順ソートでは**ない**。単純な
        # `sorted(candidates, reverse=True)` はディレクトリ名
        # (例: "idf5.5_py3.9_env" と "idf5.5_py3.12_env")を文字列として
        # 1文字ずつ比較するため、"9" > "1" となり py3.9 が py3.12 より
        # 前に来てしまう -- これがまさに 2026-07-22 に観測した実障害:
        # PATH に pyenv/Homebrew が無いマシンで ESP-IDF 自身の install.sh
        # (内部の tools/detect_python.sh は素の `python3` を最初に試す)が
        # idf5.5_py3.9_env を新規作成し、この辞書順ソートが既存の正しい
        # idf5.5_py3.12_env より 3.9 の venv を選んでしまったため、
        # sfcli パッケージの `requires-python >=3.10,<3.13` により
        # `pip install -e` が失敗した。
        version_re = re.compile(r"^idf[\d.]+_py(\d+)\.(\d+)_env$")

        def _venv_sort_key(venv_dir: Path) -> Tuple[bool, Tuple[int, int]]:
            match = version_re.match(venv_dir.name)
            # Unreachable in practice (candidates are pre-filtered by
            # prefix/suffix above), but keep a safe fallback version tuple.
            # 実際には到達しない(候補は上で prefix/suffix 済み)が、
            # 安全側のフォールバック版タプルを用意しておく。
            version = (int(match.group(1)), int(match.group(2))) if match else (0, 0)
            in_band = PYTHON_PREFERRED_MIN <= version <= PYTHON_PREFERRED_MAX
            return (in_band, version)

        candidates = [
            d for d in entries
            if d.is_dir() and d.name.startswith(prefix) and d.name.endswith("_env")
            and version_re.match(d.name)
        ]
        for venv_dir in sorted(candidates, key=_venv_sort_key, reverse=True):
            python_exe = venv_dir / bin_subdir / python_name
            if python_exe.exists():
                return python_exe

    # Strict match failed — caller can treat None as "venv not built yet"
    # 厳密マッチに失敗 — venv 未作成と同じ扱い
    return None


def _find_idf_venv_dirs(idf_path: Path) -> list[Path]:
    """All existing venv DIRECTORIES matching idf_path's ESP-IDF version
    prefix (``idf<MAJOR.MINOR>_py*_env``), newest first, regardless of
    whether a python interpreter still exists inside -- unlike
    _find_idf_python(), which silently skips a directory whose interpreter
    is missing.

    Needed by the dead-venv detector (_is_idf_venv_dir_dead() /
    _recreate_dead_idf_venvs()), which must be able to say "this venv
    folder is still here, but its interpreter or base Python is gone"
    rather than just "no venv found" -- the latter is the pre-existing,
    unrelated "ESP-IDF's own install.sh has never been run yet" case.
    idf_path の ESP-IDF バージョン接頭辞(``idf<メジャー.マイナー>_py*_env``)
    に一致する既存 venv ディレクトリを全て新しい順に返す。中の
    python インタプリタの有無にかかわらず対象とする点が _find_idf_python()
    と異なる(あちらはインタプリタが無いディレクトリを黙ってスキップする)。

    壊死検出器(_is_idf_venv_dir_dead() / _recreate_dead_idf_venvs())が
    「venv フォルダはあるがインタプリタ/ベースPythonが消えている」状態を、
    単なる「venv が見つからない」(= ESP-IDF自身のinstall.shが未実行という
    既存の別ケース)と区別できるようにするために必要。
    """
    target = _idf_major_minor(idf_path)
    if not target:
        return []
    prefix = f"idf{target}_py"

    dirs: list[Path] = []
    for base in _idf_tools_path_candidates():
        python_env_dir = base / "python_env"
        if not python_env_dir.exists():
            continue
        try:
            entries = list(python_env_dir.iterdir())
        except OSError:
            continue
        dirs.extend(
            d for d in entries
            if d.is_dir() and d.name.startswith(prefix) and d.name.endswith("_env")
        )
    return sorted(dirs, reverse=True)


def _is_idf_venv_dir_dead(venv_dir: Path) -> bool:
    """Return whether the ESP-IDF-managed venv at `venv_dir` looks dead:
    its pyvenv.cfg is missing/malformed, the base ("seed") Python recorded
    in pyvenv.cfg's `home =` line no longer exists, or the venv's own
    python interpreter is missing or fails to report a version.

    This is the classic failure mode when the system Python that ORIGINALLY
    seeded the venv (e.g. a pyenv-win version, or a python.org install) is
    later removed/upgraded out from under it: the venv directory survives,
    but every invocation of its python.exe silently fails to find its
    stdlib/DLLs.
    `venv_dir` の ESP-IDF 管理 venv が壊死しているように見えるかを返す:
    pyvenv.cfg が欠落/破損している、pyvenv.cfg の `home =` 行が指す
    ベース("種")Pythonが既に存在しない、あるいは venv 自身の python
    インタプリタが欠落しているかバージョンを報告できない、のいずれか。

    これは、venv を元々 seed したシステム Python(例: pyenv-win の
    あるバージョン、python.org インストール)が後から削除/更新されて
    しまう典型的な故障モード: venv ディレクトリ自体は残るが、その
    python.exe を起動するたびに標準ライブラリ/DLLが見つからず暗黙に
    失敗する。
    """
    bin_subdir = "Scripts" if sys.platform == "win32" else "bin"
    python_name = "python.exe" if sys.platform == "win32" else "python"
    venv_python = venv_dir / bin_subdir / python_name

    pyvenv_cfg = venv_dir / "pyvenv.cfg"
    if not pyvenv_cfg.is_file():
        return True
    home = _parse_pyvenv_cfg_home(pyvenv_cfg)
    if home is None or not home.is_dir():
        return True
    base_exe_name = "python.exe" if sys.platform == "win32" else "python3"
    base_exe = home / base_exe_name
    if not base_exe.is_file() and sys.platform != "win32":
        base_exe = home / "python"
    if not base_exe.is_file():
        return True

    if not venv_python.is_file():
        return True
    return _python_version_info(venv_python) is None


def _recreate_dead_idf_venvs(idf_path: Path, version: str, auto_install_python: bool) -> None:
    """Detect any dead ESP-IDF-managed venv matching idf_path's version
    (see _is_idf_venv_dir_dead()) and recreate it by deleting the venv
    directory and re-running ESP-IDF's own install script (idempotent --
    it only (re)creates whatever is missing; healthy sibling venvs for
    other ESP-IDF versions are untouched).

    Runs unconditionally at the top of every Step 2/4 (both a normal run
    and a --clean/repair run), so a dead venv left over from a removed/
    upgraded system Python self-heals without the user needing to know
    this check exists. Does nothing if no matching venv directory exists
    at all yet -- that is the pre-existing, unrelated "install.sh has
    never been run" case, which the caller's existing error message
    already covers.

    User-installed packages inside the venv (sfcli itself, plus anything
    `sf setup sim` added) are assumed to be gone once the directory is
    deleted -- sfcli is reinstalled by Step 3 immediately after this
    Step 2 call site returns -- but the deletion is always announced,
    bilingually, BEFORE it happens.
    idf_path のバージョンに一致する壊死 ESP-IDF 管理 venv を検出し
    (_is_idf_venv_dir_dead() 参照)、venv ディレクトリを削除して ESP-IDF
    自身の install スクリプトを再実行することで再作成する(冪等 --
    欠けているものだけを(再)作成する。他の ESP-IDF バージョン向けの
    健全な兄弟 venv には触れない)。

    毎回の Step 2/4 の先頭(通常実行・--clean/修復実行のいずれも)で無条件に
    実行するため、削除/更新されたシステム Python に起因する壊死 venv は、
    ユーザーがこのチェックの存在を意識せずとも自己修復する。一致する venv
    ディレクトリがそもそも存在しない場合は何もしない -- それは既存の別
    ケース(「install.sh がまだ一度も実行されていない」)であり、呼び出し元
    の既存エラーメッセージが既にカバーしている。

    venv 内のユーザーインストール済みパッケージ(sfcli 自身、および
    `sf setup sim` が追加したもの)は、ディレクトリ削除と同時に失われる
    前提(sfcli はこの Step 2 呼び出し直後の Step 3 で再インストールされる)
    だが、削除は必ず実行前に英日併記でログに明示する。
    """
    for venv_dir in _find_idf_venv_dirs(idf_path):
        if _is_idf_venv_dir_dead(venv_dir):
            warn(f"ESP-IDF Python venv looks dead (base Python missing or "
                 f"unresponsive): {venv_dir}")
            warn(f"ESP-IDF の Python venv が壊死しています"
                 f"(ベースPythonの欠落または無応答): {venv_dir}")
            warn(f"Deleting it now and recreating via ESP-IDF's install script...")
            warn(f"これから削除し、ESP-IDFのinstallスクリプトで再作成します...")
            shutil.rmtree(venv_dir, ignore_errors=True)
            ESPIDFInstaller._run_install_script(idf_path, version, auto_install_python=auto_install_python)
            continue
        # Alive but possibly built on an unsupported Python -- warn only,
        # never recreate (see _warn_if_idf_venv_python_too_new()'s docstring).
        # 生存しているが未対応のPythonを基盤にしている可能性 -- 警告のみで
        # 再作成はしない(_warn_if_idf_venv_python_too_new() のdocstring参照)。
        _warn_if_idf_venv_python_too_new(venv_dir)


def _warn_if_idf_venv_python_too_new(venv_dir: Path) -> None:
    """If a healthy (not dead) ESP-IDF venv's own Python is newer than
    PYTHON_PREFERRED_MAX (3.13+), print a bilingual warning but take no
    further action. This venv is not "dead" -- _is_idf_venv_dir_dead()
    already confirmed it responds -- and the 2026-07-22 policy change to
    reject 3.13+ outright applies to NEW seed selection
    (_find_system_python_dir()), not to recreating an already-working
    venv purely because of its Python version. Recreating a working venv
    on version grounds alone would be disruptive and is out of scope here.
    健全な(壊死していない)ESP-IDF venv 自身の Python が
    PYTHON_PREFERRED_MAX(3.13+)より新しい場合、英日併記の警告のみ表示し
    それ以上は何もしない。この venv は「壊死」ではない --
    _is_idf_venv_dir_dead() が既に応答することを確認済み -- また、3.13+を
    無条件に不採用とする2026-07-22の方針変更は新規の種選択
    (_find_system_python_dir())に対するものであり、バージョンのみを理由に
    既に動作している venv を再作成することは対象外。動作中の venv を
    バージョンだけを根拠に再作成するのは影響が大きく、本関数の対象外とする。
    """
    bin_subdir = "Scripts" if sys.platform == "win32" else "bin"
    python_name = "python.exe" if sys.platform == "win32" else "python"
    venv_python = venv_dir / bin_subdir / python_name
    version = _python_version_info(venv_python)
    if version is None or version <= PYTHON_PREFERRED_MAX:
        return
    warn(f"ESP-IDF venv {venv_dir} is built on Python {version[0]}.{version[1]}, "
         f"newer than the supported {PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
         f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} range. It may still "
         f"work, but this combination is untested and not recreated automatically.")
    warn(f"ESP-IDF venv {venv_dir} は Python {version[0]}.{version[1]} を基盤と"
         f"しており、対応範囲 {PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}〜"
         f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} より新しいです。"
         "動作する可能性はありますが、この組み合わせは未検証であり、自動的な"
         "再作成は行いません。")


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


# -- Post-install package verification ---------------------------------------
# インストール後のパッケージ検証
#
# `pip install -r requirements.txt` (see Installer.run()'s Step 3/4) is only
# ever treated as a soft warn() on failure, by design -- a single missing
# optional package (e.g. no arm64 wheel for vpython on a given macOS/Python
# combination) should not fail the whole install. But that softness means a
# real gap -- e.g. vpython silently absent from the ESP-IDF venv -- can slip
# by completely unnoticed: the log scrolls past a `warn()` among hundreds of
# pip lines, Step 3/4 still prints "StampFly CLI installed!", and the final
# "Installation Complete!" banner has nothing that says otherwise. This was
# observed on a real macOS install (2026-07-21): the GUI installer finished
# looking successful, but `sf sim run` then failed with "vpython module not
# found", and the user's own `pip3 install vpython` landed in an unrelated
# system Python instead of the sf/ESP-IDF venv (see sim.py's
# `_get_python_cmd()` guidance text, updated alongside this).
#
# _verify_key_packages() closes that gap: after the bulk requirements.txt
# install, it individually import-checks each package below inside the
# ESP-IDF venv, retries a missing one with a targeted `pip install <pkg>`,
# and returns whatever is STILL missing after the retry so the caller can
# surface it prominently in the completion banner instead of letting it
# hide among earlier warnings.
#
# Installer.run() の Step 3/4 で行う `pip install -r requirements.txt` は、
# 失敗しても意図的に warn() だけで済ませる(1つの任意パッケージが欠けた
# だけ(例: 特定の macOS/Python 組み合わせで vpython の arm64 wheel が
# 無い)ことでインストール全体を失敗にすべきではないため)。しかしこの
# 緩さゆえに、本当のギャップ(例: vpython が ESP-IDF venv に無言で欠落)が
# 完全に見過ごされうる: 何百行もの pip 出力に紛れた warn() はログで
# 流れ去り、Step 3/4 は "StampFly CLI installed!" と表示し、最後の
# "Installation Complete!" バナーにも異常を示すものが何も無い。実際に
# macOS 実機で観測された(2026-07-21): GUI インストーラは成功したように
# 見えて完了したが、その後 `sf sim run` が "vpython module not found" で
# 失敗し、ユーザー自身が実行した `pip3 install vpython` は sf/ESP-IDF
# venv とは無関係なシステム Python に着地していた(この案内文自体も
# sim.py の `_get_python_cmd()` 側で合わせて修正済み -- そちらのコメント
# 参照)。
#
# _verify_key_packages() はこのギャップを塞ぐ: requirements.txt 一括導入の
# 後、下記の各パッケージを ESP-IDF venv 内で個別に import 検証し、欠けて
# いるものは的を絞った `pip install <pkg>` で再試行し、再試行後もなお
# 欠けているものだけを返す。呼び出し元はそれを完了バナーで目立つ形に
# 提示でき、以前の警告群に埋もれさせずに済む。

# (import module name, pip package name) pairs always installed as part of
# pyproject.toml's core `dependencies` -- present in BOTH minimal and full
# installs (minimal only skips the simulator-only extras below).
# pyproject.toml のコア `dependencies` として常に導入される(import モジュール名,
# pipパッケージ名)の組。minimal/フル導入の両方に存在する
# (minimal が省略するのは下のシミュレータ専用パッケージのみ)。
CORE_IMPORT_CHECKS: list[Tuple[str, str]] = [
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("matplotlib", "matplotlib"),
    ("pandas", "pandas"),
    ("yaml", "PyYAML"),
    ("serial", "pyserial"),
]

# (import module name, pip package name) pairs installed only by the full
# `requirements.txt` path (see Installer.run()'s `if minimal: ... else: ...`
# branch) -- deliberately skipped under --minimal, so these are only
# checked when minimal is False.
# フル(requirements.txt)経路でのみ導入される(import モジュール名,
# pipパッケージ名)の組。--minimal では意図的に省略するため、
# minimal が False のときのみ検証する。
SIMULATOR_IMPORT_CHECKS: list[Tuple[str, str]] = [
    ("vpython", "vpython"),
    ("pygame", "pygame"),
    ("cv2", "opencv-python"),
]

# Timeout for each individual `import <module>` probe subprocess. Generous
# enough for a slow-importing package (matplotlib's font cache build on
# first import) without letting one hung interpreter stall the whole
# verification pass for long.
# 個々の `import <module>` プローブ subprocess のタイムアウト。matplotlib の
# 初回 import 時のフォントキャッシュ構築のような重い import にも十分な
# 余裕を持たせつつ、1つのハングしたインタプリタが検証全体を長時間
# 止めないようにする。
IMPORT_CHECK_TIMEOUT_SECONDS = 20


def _module_importable(python_exe: Path, module_name: str) -> bool:
    """Return whether `import <module_name>` succeeds under python_exe.
    Never raises: a missing interpreter, a timeout, or any other
    subprocess failure all count as "not importable" (False) rather than
    propagating -- this is a best-effort probe, not a hard requirement.
    python_exe で `import <module_name>` が成功するか返す。例外は送出
    しない: インタプリタ不在・タイムアウト・その他の subprocess 失敗は
    全て「import不可」(False)として扱う -- これはベストエフォートな
    プローブであり、必須要件のチェックではない。
    """
    try:
        result = subprocess.run(
            [str(python_exe), "-c", f"import {module_name}"],
            capture_output=True, timeout=IMPORT_CHECK_TIMEOUT_SECONDS,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


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
        _sanitize_activated_env(env)
        for var in ("PIP_REQUIRE_VIRTUALENV", "PIP_TARGET", "PIP_PREFIX", "PIP_USER"):
            env.pop(var, None)
        # Stream pip's output line-by-line instead of a silent
        # subprocess.run(): a bare subprocess.run() here shows nothing to
        # the GUI installer until pip exits, so a multi-minute dependency
        # install (e.g. -r requirements.txt) reads as a frozen progress
        # bar instead of live log lines. See _stream_subprocess() docstring.
        # subprocess.run() で無言実行するのではなく pip の出力を1行ずつ
        # 流す: ここで素の subprocess.run() を使うと GUI インストーラには
        # pip 終了まで何も見えず、数分かかる依存関係インストール
        # (-r requirements.txt 等) が進捗停止に見えてしまう。
        # _stream_subprocess() の docstring 参照。
        return _stream_subprocess(cmd, env=env)

    # Fallback: venv not yet created (e.g. mid-install). Source export script.
    # フォールバック: venv 未作成時のみ export スクリプトを source する
    if sys.platform == "win32":
        export_script = idf_path / "export.bat"
        escaped = subprocess.list2cmdline(pip_args)
        cmd = f'call "{export_script}" && python -m pip {escaped}'
        return _stream_subprocess(cmd, shell=True, env=_clean_env_for_cmd())
    else:
        escaped = " ".join(shlex.quote(arg) for arg in pip_args)
        env_prefix = _build_idf_env_command(idf_path)
        inner = f'{env_prefix} && python -m pip {escaped}'
        return _stream_subprocess(["bash", "-c", inner])


def _run_sf_in_idf_env(idf_path: Path, sf_args: list[str]) -> int:
    """Run `sf <sf_args>` inside the ESP-IDF Python environment (Step 4/4:
    GUI Flasher install).
    ESP-IDFのPython環境で `sf <sf_args>` を実行（Step4/4: GUIフラッシャの導入）

    Reuses the same "call the venv python by absolute path, no shell, no
    PATH" strategy as _run_in_idf_env() (see its docstring for why sourcing
    export.sh is unreliable). sfcli is invoked as `python -m sfcli.cli`
    rather than the installed `sf` console-script shim: the shim's location
    depends on the venv's Scripts/bin layout and PATH, both of which this
    function deliberately avoids depending on. This works because sfcli's
    editable install puts `lib/` (where the `sfcli` package lives) directly
    on the venv's sys.path (pyproject.toml: package-dir "" = "lib"), so
    `-m sfcli.cli` needs no extra PYTHONPATH wiring.
    _run_in_idf_env() と同じ「venvのpythonを絶対パスで、シェルもPATHも
    介さず直接呼ぶ」戦略を使う（export.sh の source が信頼できない理由は
    _run_in_idf_env の docstring 参照）。sfcli はインストール済み `sf`
    コンソールスクリプトシムではなく `python -m sfcli.cli` として起動する。
    シムの場所は venv の Scripts/bin レイアウトと PATH に依存し、本関数は
    そのいずれにも意図的に依存しないため。sfcli の editable インストールは
    `lib/`（`sfcli` パッケージの実体があるディレクトリ）を venv の
    sys.path に直接乗せる（pyproject.toml: package-dir "" = "lib"）ため、
    `-m sfcli.cli` に追加の PYTHONPATH 配線は不要。

    Returns 1 (rather than raising) when no matching venv exists yet, since
    this is Step 4/4 and Step 2/3 already validated venv discovery earlier
    in the same run — reaching here without a venv would be an unexpected
    internal inconsistency, not a normal Step 4 failure mode, and the
    caller treats any non-zero return the same way (warn + continue).
    対応する venv が見つからない場合は例外を投げず 1 を返す。Step2/3で
    既に同じ実行内で venv 探索を検証済みのため、ここに到達して venv が
    無いのは通常の Step4 失敗モードではなく想定外の内部不整合だが、
    呼び出し側はどちらの非ゼロ戻り値も同じ扱い(warn + 続行)にする。
    """
    venv_python = _find_idf_python(idf_path)
    if not venv_python:
        return 1
    cmd = [str(venv_python), "-m", "sfcli.cli"] + sf_args
    # Same env sanitization as _run_in_idf_env(): strip vars that could
    # steer sfcli's own path resolution toward a pre-activated user venv.
    # _run_in_idf_env() と同じ環境変数サニタイズ: 事前activate済みの
    # user venv へ sfcli 自身のパス解決が誤誘導されないようにする
    env = os.environ.copy()
    _sanitize_activated_env(env)
    return subprocess.run(cmd, env=env).returncode


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

        # Sort by version (newest first). Use the numeric key, not the raw
        # string, so "v5.10.0" correctly outranks "v5.5.2".
        # 新しい順にソート。生の文字列ではなく数値キーを使うことで
        # "v5.10.0" が "v5.5.2" より正しく上位になる
        installations.sort(key=lambda x: version_sort_key(x[1]), reverse=True)
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
                encoding="utf-8",
                errors="replace",
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
                    encoding="utf-8",
                    errors="replace",
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
    def install(
        cls,
        target_dir: Optional[Path] = None,
        version: str = DEFAULT_VERSION,
        auto_install_python: bool = False,
    ) -> Optional[Path]:
        """Install ESP-IDF with 3-stage clone separation.
        3段階分離でESP-IDFをインストール

        `auto_install_python` is threaded through to _run_install_script()
        -- see _offer_python_auto_install() for what it controls.
        `auto_install_python` は _run_install_script() へそのまま渡される
        -- 何を制御するかは _offer_python_auto_install() 参照。
        """
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
                return cls._run_install_script(target_dir, version, auto_install_python=auto_install_python)
            else:
                # Directory exists but not a git repo or ESP-IDF
                # ディレクトリは存在するがgitリポジトリでもESP-IDFでもない
                error(f"Directory exists but is not ESP-IDF: {target_dir}")
                error("Remove it manually or specify a different path.")
                return None

        # Stage 2: Clone main repository (without submodules)
        # ステージ2: メインリポジトリのクローン（サブモジュールなし）
        # --progress and _stream_subprocess() together surface live clone
        # progress instead of a multi-minute silence in a frozen/redirected
        # caller (e.g. the GUI installer's captured stdout).
        # --progress と _stream_subprocess() の組み合わせで、凍結/
        # リダイレクトされた呼び出し元(GUIインストーラーの捕捉した標準
        # 出力など)でも数分間の沈黙ではなくクローンの進捗が見えるようにする。
        info("Cloning ESP-IDF repository (main repo)...")
        try:
            rc = _stream_subprocess(
                [
                    "git", "clone", "--progress",
                    "--branch", version,
                    "--depth", "1",
                    cls.REPO_URL,
                    str(target_dir),
                ],
            )
        except FileNotFoundError:
            _report_git_not_found()
            return None
        if rc != 0:
            error(f"Failed to clone ESP-IDF (git exited with code {rc})")
            # Clean up failed clone
            # 失敗したクローンをクリーンアップ
            if target_dir.exists():
                shutil.rmtree(target_dir)
            return None

        # Stage 3: Initialize submodules (retryable)
        # ステージ3: サブモジュール初期化（リトライ可能）
        info("Initializing submodules (this may take a while)...")
        try:
            rc = _stream_subprocess(
                [
                    "git", "submodule", "update",
                    "--init", "--depth", "1", "--recursive",
                ],
                cwd=target_dir,
            )
        except FileNotFoundError:
            _report_git_not_found()
            return None
        if rc != 0:
            error(f"Failed to initialize submodules (git exited with code {rc})")
            warn("Main repository is preserved. Re-run installer to retry submodule init.")
            # Don't delete - main repo is intact, user can retry
            # 削除しない - メインリポジトリはそのまま、再実行でリトライ可能
            return None

        # Stage 4: Run install script (idempotent)
        # ステージ4: install.sh 実行（冪等）
        return cls._run_install_script(target_dir, version, auto_install_python=auto_install_python)

    @classmethod
    def _run_install_script(
        cls, target_dir: Path, version: str, auto_install_python: bool = False,
    ) -> Optional[Path]:
        """Run ESP-IDF install script (idempotent).
        ESP-IDFのinstall.shを実行（冪等）"""
        # ESP-IDF's own install.bat/install.sh calls `python`/`python3` by
        # name, so a real interpreter in the ecosystem's tested range
        # (3.10-3.12; see PYTHON_PREFERRED_MIN/MAX) must be resolvable.
        # Without one on Windows this fails with the cryptic exit code
        # 9009 ("command not found") -- especially important for the GUI
        # (StampFly Setup), whose bundled Python is NOT usable here (see
        # _find_system_python_dir()); observed on a workshop laptop
        # 2026-07-20. The same "install script calls python by name"
        # reasoning applies on macOS/Linux, which is why this check (and
        # the auto-install offer below) runs on all three platforms, not
        # Windows alone.
        # ESP-IDF自身の install.bat/install.sh は `python`/`python3` を
        # 名前で呼ぶため、エコシステムの検証済み範囲(3.10〜3.12。
        # PYTHON_PREFERRED_MIN/MAX 参照)にある本物のインタプリタが解決
        # できる必要がある。Windowsでこれが無いと難解な exit 9009
        # ("コマンドが見つからない")で失敗する -- 特に GUI(StampFly Setup)
        # で重要で、同梱 Python はここでは使えない
        # (_find_system_python_dir() 参照)。2026-07-20 に講習用ノートで
        # 観測。「installスクリプトがpythonを名前で呼ぶ」という同じ理屈は
        # macOS/Linux にも当てはまるため、このチェック(および下の
        # 自動インストール提案)は Windows 単独ではなく3プラットフォーム
        # 全てで走る。
        if _find_system_python_dir() is None:
            if not _offer_python_auto_install(auto_install_python):
                error(f"ESP-IDF tool installation needs a system Python "
                      f"{PYTHON_PREFERRED_MIN[0]}.{PYTHON_PREFERRED_MIN[1]}-"
                      f"{PYTHON_PREFERRED_MAX[0]}.{PYTHON_PREFERRED_MAX[1]} on this PC,")
                error("but none was found. ESP-IDF's own install script requires it.")
                return None
            if _find_system_python_dir() is None:
                error("Still no usable system Python after the auto-install attempt.")
                error("自動インストール後も使用可能なシステムPythonが見つかりません。")
                return None

        info("Installing ESP-IDF tools (this may take a while)...")
        # Streamed via _stream_subprocess() (not subprocess.run(capture_output))
        # so a frozen/redirected caller sees live progress instead of a
        # multi-minute silence -- see that function's docstring.
        # _stream_subprocess() 経由で流す(subprocess.run(capture_output) では
        # ない) -- 凍結/リダイレクトされた呼び出し元でも数分間の沈黙では
        # なく進捗が見えるようにする。詳細は同関数の docstring 参照。
        try:
            if sys.platform == "win32":
                install_script = target_dir / "install.bat"
                # Use shell=True + call for .bat execution from any shell
                # shell=True + call で任意のシェルから .bat を確実に実行
                cmd = f'call "{install_script}" esp32s3'
                rc = _stream_subprocess(cmd, shell=True, env=_clean_env_for_cmd())
            else:
                install_script = target_dir / "install.sh"
                # Steer PATH so ESP-IDF's own detect_python.sh (which
                # tries the bare `python3` name FIRST) resolves to an
                # in-band interpreter instead of whatever `/usr/bin/
                # python3` happens to be -- see
                # _env_with_python3_steering()'s docstring for the full
                # 2026-07-22 macOS failure this guards against (a
                # PATH-less GUI launch let /usr/bin/python3 (3.9) win and
                # create an out-of-band idf5.5_py3.9_env).
                # ESP-IDF 自身の detect_python.sh(素の `python3` という
                # 名前を**最初に**試す)が、たまたま存在する
                # `/usr/bin/python3` ではなく範囲内のインタプリタを
                # 解決するよう PATH を誘導する -- 本修正が防ぐ 2026-07-22
                # の macOS 実障害(PATH無しのGUI起動で /usr/bin/python3
                # (3.9)が勝ち、範囲外の idf5.5_py3.9_env が作成された)の
                # 詳細は _env_with_python3_steering() の docstring 参照。
                rc = _stream_subprocess(
                    ["bash", str(install_script), "esp32s3"],
                    env=_env_with_python3_steering(),
                )
        except FileNotFoundError as e:
            error(f"Failed to install ESP-IDF tools: {e}")
            return None
        if rc != 0:
            error(f"Failed to install ESP-IDF tools (exit code {rc})")
            return None

        success(f"ESP-IDF {version} installed successfully!")
        return target_dir


# StampFly Terminal launcher (Installer._create_terminal_launcher() /
# Installer._remove_terminal_launcher(), called from uninstall()). Fixed
# names shared by creation and removal so the two never drift apart,
# mirroring the BINARY_NAME/EXE_NAME pattern in
# lib/sfcli/utils/flasher_install/_linux.py / _windows.py.
# StampFly Terminal ランチャー(Installer._create_terminal_launcher() /
# Installer._remove_terminal_launcher()、uninstall() から呼ばれる)。作成と
# 削除の間で名称がずれないよう固定値として共有する
# (lib/sfcli/utils/flasher_install/_linux.py / _windows.py の
# BINARY_NAME/EXE_NAME と同じパターン)。
TERMINAL_LAUNCHER_NAME = "StampFly Terminal"
TERMINAL_LAUNCHER_MACOS_FILENAME = f"{TERMINAL_LAUNCHER_NAME}.command"
TERMINAL_LAUNCHER_MACOS_MODE = 0o755  # owner rwx, group/other rx (no write) / 所有者rwx、グループ/その他rx
# macOS ships a real .app bundle (not a bare .command) so the launcher
# appears in Launchpad/Spotlight with its own icon; the .command lives
# inside Contents/Resources and still does the actual work. The bare
# ~/Applications/<name>.command from the first iteration is treated as
# legacy and removed on create/uninstall.
# macOS では(素の .command ではなく)正式な .app バンドルを作る。これにより
# Launchpad/Spotlight に専用アイコン付きで表示される。実際の処理は
# Contents/Resources 内の .command が担う。初期実装の素の
# ~/Applications/<name>.command はレガシー扱いとし、作成時/アンインストール
# 時に削除する。
TERMINAL_LAUNCHER_MACOS_APP_DIRNAME = f"{TERMINAL_LAUNCHER_NAME}.app"
TERMINAL_LAUNCHER_MACOS_BUNDLE_ID = "jp.stampfly.terminal"
TERMINAL_LAUNCHER_MACOS_EXECUTABLE = "StampFlyTerminal"
# Icon assets generated by tools/terminal_launcher/assets/gen_icon_3d.py
# (repo-relative; missing icons degrade gracefully to the OS default).
# tools/terminal_launcher/assets/gen_icon_3d.py が生成するアイコン資産
# (リポジトリ相対。無い場合はOS既定アイコンに優雅に劣化)。
TERMINAL_LAUNCHER_ICON_RELDIR = Path("tools") / "terminal_launcher" / "assets"
TERMINAL_LAUNCHER_LINUX_DESKTOP_ID = "stampfly-terminal.desktop"
# Reuses the same Start Menu folder name as the GUI Flasher
# (lib/sfcli/utils/flasher_install/_windows.py START_MENU_FOLDER_NAME) so
# both shortcuts live side by side under one "StampFly" group.
# GUIフラッシャ(lib/sfcli/utils/flasher_install/_windows.py の
# START_MENU_FOLDER_NAME)と同じスタートメニューフォルダ名を再利用し、
# 両方のショートカットが1つの「StampFly」グループにまとまるようにする。
TERMINAL_LAUNCHER_WINDOWS_START_MENU_FOLDER = "StampFly"
TERMINAL_LAUNCHER_WINDOWS_LNK_NAME = f"{TERMINAL_LAUNCHER_NAME}.lnk"


def _ps_escape(value: str) -> str:
    """Escape a string for embedding inside a PowerShell single-quoted literal.
    PowerShell のシングルクォート文字列に埋め込むための文字列エスケープ

    A single quote is escaped by doubling it (`''`), the same rule as
    classic Pascal/SQL string literals. Duplicated from
    lib/sfcli/utils/flasher_install/_windows.py's identical helper (not
    imported) because this file must stay a fully standalone, stdlib-only
    script per its Stability contract (see the module docstring) -- it
    cannot depend on the sfcli package it exists to install.
    シングルクォートの二重化(`''`)でエスケープする(Pascal/SQL の文字列
    リテラルと同じ規則)。lib/sfcli/utils/flasher_install/_windows.py の
    同名ヘルパーと同一処理だが import はしない -- 本ファイルは(自身が
    導入するsfcliパッケージに依存できない)完全に独立したstdlib限定
    スクリプトであるべきため(モジュールdocstringの安定契約を参照)。
    """
    return value.replace("'", "''")


def _refresh_linux_desktop_database(applications_dir: Path) -> None:
    """Run `update-desktop-database` if available, so a new/removed .desktop
    entry is picked up by application launchers without a logout/login.
    `update-desktop-database` が利用可能なら実行し、.desktop エントリの
    追加/削除をログアウト/ログイン無しでアプリランチャーに反映させる

    Mirrors lib/sfcli/utils/flasher_install/_linux.py's
    _refresh_desktop_database(); duplicated here (not imported) for the
    same stdlib-only-standalone-script reason as _ps_escape() above.
    Failure (tool absent, or a minimal desktop environment without it) is
    ignored -- this is a convenience refresh, not a correctness requirement.
    lib/sfcli/utils/flasher_install/_linux.py の
    _refresh_desktop_database() と同じ処理。_ps_escape() と同じ理由で
    import せず複製する。失敗(ツール未導入、あるいは持たない最小構成
    デスクトップ環境)は無視する -- あくまで利便性のための更新であり、
    必須要件ではない。
    """
    tool = shutil.which("update-desktop-database")
    if not tool:
        return
    try:
        subprocess.run([tool, str(applications_dir)], capture_output=True, check=False)
    except OSError:
        pass


class Installer:
    """Main installer"""

    def __init__(self):
        self.root = Path(__file__).parent.parent.resolve()
        self.config_dir = self.root / ".sf"
        self.config_file = self.config_dir / "config.toml"

    def _is_sfcli_installed(self, idf_path: Path) -> bool:
        """Check if sfcli is installed, importable, AND served from THIS repo.
        sfcli が ESP-IDF venv で import でき、かつ本リポジトリを参照して
        いるか確認

        We deliberately do this with the venv python by absolute path (not via
        a sourced export.sh) so the check answers "is sfcli importable from
        this specific venv" rather than "is sfcli importable from whatever
        python happens to be on PATH after activation." The latter has been
        a source of false positives when pyenv shims override the venv.
        絶対パスで venv python を呼び、activate 経由ではなく直接 import を
        試す。PATH 経由だと pyenv 等が誤誘導して false positive になる。

        Besides importability, the probe also verifies the module actually
        resolves to this checkout (self.root). An editable install records a
        directory path in the venv, so a venv set up from another clone keeps
        serving that clone's stale code forever; without this check a re-run
        of the installer from the right clone would just say "already
        installed" and skip (observed 2026-07-24: sf doctor kept warning from
        a tmp clone that predated the has_solution fix). The same probe is
        aliased as _verify_sfcli_import, so post-install verification also
        catches an install that landed pointing somewhere unexpected.
        import 可否に加えて、モジュールの実体がこのチェックアウト
        (self.root) に解決されることも検証する。editable インストールは
        venv にディレクトリパスを記録するため、別クローンから構築された
        venv は古いコードを参照し続け、正しいクローンからインストーラを
        再実行しても「インストール済み」でスキップされてしまう
        (2026-07-24 実例: has_solution 修正前の tmp クローンを参照し続け
        sf doctor が警告を出し続けた)。本プローブは _verify_sfcli_import
        の別名でもあるため、インストール直後の検証でも参照先ずれを検出
        できる。
        """
        venv_python = _find_idf_python(idf_path)
        if not venv_python:
            return False
        try:
            # `import sfcli` alone is not enough: __init__.py does not read
            # any third-party dependency, so a venv with sfcli's *package*
            # present but a dependency missing (e.g. after a `pip install -e`
            # that partially failed) would still report "installed" here.
            # We import sfcli.cli (which imports every command module) and
            # call assert_all_commands_loadable() (added in cli.py under the
            # C2 dependency-resilience work) so a missing dependency makes
            # this probe correctly report "not installed" and Step 3 reruns.
            # The trailing print reports where sfcli actually resolves from,
            # for the location check below.
            # `import sfcli` だけでは不十分: __init__.py はサードパーティ
            # 依存を一切読まないため、sfcliパッケージ自体はあるが依存が
            # 欠けている venv (`pip install -e` が部分失敗した後など) でも
            # ここは「インストール済み」と誤診断してしまう。sfcli.cli を
            # import し（全コマンドモジュールをimportする）、
            # assert_all_commands_loadable()（C2の依存耐性対応でcli.pyに
            # 追加）を呼ぶことで、依存欠落時はこのプローブが正しく
            # 「未インストール」と判定し、Step3が再実行されるようにする。
            # 末尾の print は下の参照先チェック用に sfcli の実体位置を返す。
            result = subprocess.run(
                [
                    str(venv_python), "-c",
                    "import sfcli.cli; sfcli.cli.assert_all_commands_loadable(); "
                    "print(sfcli.__file__)",
                ],
                capture_output=True, text=True,
                encoding="utf-8", errors="replace",
            )
            if result.returncode != 0:
                return False
            # Location check: the venv must serve sfcli from THIS repo.
            # normcase() so the comparison is case-insensitive on Windows.
            # 参照先チェック: venv の sfcli が本リポジトリの実体であること。
            # Windows で大文字小文字の揺れを吸収するため normcase() で比較。
            reported = Path(result.stdout.strip().splitlines()[-1]).resolve()
            expected = (self.root / "lib" / "sfcli" / "__init__.py").resolve()
            if os.path.normcase(str(reported)) != os.path.normcase(str(expected)):
                warn("sfcli in the ESP-IDF venv resolves to a different "
                     f"location: {reported.parent}")
                warn(f"  this installer runs from: {self.root}")
                warn("venv の sfcli が別の場所を参照しています（このリポジトリ"
                     "から再インストールして修正します）")
                return False
            return True
        except Exception:
            return False

    # Alias to make the intent explicit at the post-install verification site.
    # 同じチェックを post-install 検証用に別名で公開しておく
    _verify_sfcli_import = _is_sfcli_installed

    def _diagnose_broken_install(self, idf_path: Path) -> None:
        """Print actionable diagnostic info when sfcli is unimportable
        despite pip having reported a successful install.
        pip が成功と報告したのに sfcli が import できない状態の診断情報を出す

        Always ends with both a "preferred" recovery (re-run ./install.sh
        --clean after deactivating any user env) and a "manual escape hatch"
        that bypasses install.sh entirely — so even when the high-level
        installer cannot make progress for some reason (e.g. broken stdin,
        no network, intermediate scripts misbehaving), the user has a
        concrete shell command they can paste to recover.
        ./install.sh --clean が何らかの理由で先に進めないケースに備えて、
        必ず手動の脱出経路(venv python を絶対パスで叩く pip コマンド)も
        併記する。
        """
        venv_python = _find_idf_python(idf_path)
        if not venv_python:
            error("  No ESP-IDF venv matching this ESP-IDF installation was found.")
            error(f"    idf_path : {idf_path}")
            error("  This typically means ESP-IDF's own install.sh has not been run")
            error("  for this version yet. Run it manually, then re-run this installer:")
            error(f"    bash {idf_path}/install.sh esp32s3")
            error("    ./install.sh")
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
        scenario = None  # what kind of break we detected
        if not artifacts:
            error("    No sfcli artifacts found in venv.")
            error("    pip likely installed into a different python (e.g. a")
            error("    pre-activated user venv / conda env).")
            scenario = "wrong_target"
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
                scenario = "missing_pth"
            for kind, p in artifacts:
                if kind == "pth":
                    try:
                        target = p.read_text().strip().splitlines()[0]
                        target_p = Path(target)
                        if not target_p.exists():
                            error(f"    >>> .pth points to NON-EXISTENT path: {target}")
                            error("    >>> The repo was moved/renamed, or installed from")
                            error("    >>> a symlinked path that no longer resolves.")
                            scenario = "stale_pth"
                    except (OSError, IndexError):
                        pass
        # Surface any pre-activated user env that probably contributed
        # 寄与している可能性のある pre-activated 環境を表示
        active_venv = os.environ.get("VIRTUAL_ENV")
        active_conda = os.environ.get("CONDA_PREFIX")
        if active_venv or active_conda:
            error("")
            error("  Detected an active Python environment that may be interfering:")
            if active_venv:
                error(f"    VIRTUAL_ENV  = {active_venv}")
            if active_conda:
                error(f"    CONDA_PREFIX = {active_conda}")

        error("")
        error("  Recommended recovery (preferred):")
        if active_venv or active_conda:
            error("    deactivate 2>/dev/null; conda deactivate 2>/dev/null")
        error("    ./install.sh --clean")
        error("")
        error("  Manual escape hatch (if ./install.sh --clean does not work):")
        error(f"    {venv_python} -m pip uninstall -y stampfly-ecosystem")
        constraint = _find_idf_constraint_file(idf_path)
        if constraint:
            error(f"    {venv_python} -m pip install \\")
            error(f"        -c {constraint} \\")
            error(f"        -e {self.root}")
        else:
            error(f"    {venv_python} -m pip install -e {self.root}")
        error(f"    {venv_python} -c 'import sfcli; print(sfcli.__file__)'")
        error("")
        if scenario == "stale_pth":
            error("  If the manual escape hatch is run from the CURRENT repo path,")
            error("  the new .pth will point at the right place and the import will work.")
        elif scenario == "wrong_target":
            error("  The manual escape hatch bypasses PATH/env entirely by calling")
            error("  the venv python with its absolute path, so a pre-activated env")
            error("  cannot misroute the install.")
        elif scenario == "missing_pth":
            error("  Uninstall + reinstall via the manual escape hatch will rewrite")
            error("  the editable .pth file and unblock the import.")

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
        no_flasher: bool = False,
        auto_install_python: bool = False,
        with_sils_toolchain: bool = True,
    ) -> int:
        """Run installation.

        `auto_install_python`: only takes effect in non-interactive mode
        (SF_INSTALLER_NONINTERACTIVE=1) -- see _offer_python_auto_install().
        Interactive mode always asks via a y/n prompt regardless of this
        value; Linux's sudo-gated install path ignores it entirely (never
        runs unattended).
        `auto_install_python`: 非対話モード(SF_INSTALLER_NONINTERACTIVE=1)
        でのみ効果を持つ -- _offer_python_auto_install() 参照。対話モードは
        この値に関わらず常にプロンプトで y/n を尋ねる。Linuxのsudoゲート
        付きインストール経路はこの値を一切無視する(無人実行は絶対にしない)。
        """

        # Surface pre-activated venv / conda env early so the user can
        # course-correct before pip operations begin.
        # 事前 activate 済みの環境は最初に警告して、pip が動き始める前に
        # ユーザが軌道修正できるようにする
        self._warn_if_env_preactivated()

        # Step 1: Find or install ESP-IDF
        header("Step 1/4: ESP-IDF")

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
                    idf_path = ESPIDFInstaller.install(auto_install_python=auto_install_python)
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
                    idf_path = ESPIDFInstaller.install(auto_install_python=auto_install_python)
                    if not idf_path:
                        return 1
                    version = ESPIDFDetector._get_version(idf_path)

        success(f"Using ESP-IDF {version}")
        print()

        # Step 2: Get ESP-IDF Python environment
        header("Step 2/4: Python Environment")

        # Self-heal a dead venv (its seed system Python was removed/
        # upgraded out from under it) BEFORE trying to use it, in both a
        # normal run and a --clean/repair run -- see
        # _recreate_dead_idf_venvs()'s docstring.
        # 使用を試みる前に、壊死した venv(それを seed したシステム Python
        # が後から削除/更新された状態)を自己修復する。通常実行・
        # --clean/修復実行のいずれでも行う -- _recreate_dead_idf_venvs()
        # のdocstring参照。
        _recreate_dead_idf_venvs(idf_path, version, auto_install_python=auto_install_python)

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
        header("Step 3/4: StampFly CLI")

        # Fix setuptools BEFORE any other Step 3/4 pip install runs (moved up
        # from after the CLI install, 2026-07-22): ESP-IDF's own install.sh
        # (Step 1/4) may leave the venv on setuptools 82+, which removed
        # pkg_resources and breaks vpython at IMPORT time even when
        # vpython's own `pip install` reported success. Pinning
        # setuptools<81 here -- before requirements.txt/vpython get
        # installed -- ensures pkg_resources is already back by the time pip
        # processes them. requirements.txt's own `setuptools>=68.0,<81`
        # first line is belt-and-suspenders within that SAME transaction,
        # but doing it here too also guards the --minimal path (which never
        # touches requirements.txt at all) and guards against a
        # partial/aborted requirements.txt run leaving the pin never
        # applied.
        # setuptools修正: 他のどのStep3/4 pipインストールよりも前に実行する
        # (2026-07-22、CLIインストール後から前倒し): ESP-IDF自身の
        # install.sh(Step1/4)がvenvをsetuptools 82+のままにすることがあり、
        # これはpkg_resourcesを削除しvpythonをimport時に壊す(vpython自身の
        # `pip install`が成功と報告していても)。requirements.txt/vpythonが
        # 導入される前にここでsetuptools<81に固定しておけば、pipがそれらを
        # 処理する時点で既にpkg_resourcesが戻っている。requirements.txt
        # 自身の`setuptools>=68.0,<81`という1行目は同じトランザクション内の
        # 保険だが、ここでも行うことで--minimal経路(requirements.txtに
        # 一切触れない)も保護し、requirements.txt実行が途中で中断してpinが
        # 適用されないまま終わるケースからも保護する。
        self._fix_setuptools(idf_path)

        if not skip_deps:
            # Probe: check if already installed (skip if not --force)
            # プローブ: インストール済みか確認（--forceでなければスキップ）
            if not force and self._is_sfcli_installed(idf_path):
                success("sfcli is already installed, skipping (use --force to reinstall)")
                info("(after `git pull`, use `sf upgrade` or --force instead)")
                info("(git pull後は `sf upgrade` か --force を使うこと)")
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

        # Verify the key packages requirements.txt (or, under --minimal,
        # pyproject.toml's core deps alone) were supposed to provide are
        # actually importable -- see _verify_key_packages()'s docstring and
        # the CORE_IMPORT_CHECKS/SIMULATOR_IMPORT_CHECKS module comment for
        # why this exists (a `pip install -r requirements.txt` failure a
        # few lines above is only ever a warn(), which is easy to miss).
        # Skipped entirely when --skip-deps was passed: nothing was
        # (re)installed this run, so a "missing" verdict here would not
        # reflect anything this run actually did. Stashed on self so the
        # "Installation Complete!" banner further down can surface it
        # prominently instead of letting it hide among earlier warnings.
        # requirements.txt(--minimalの場合はpyproject.tomlのコア依存のみ)が
        # 導入するはずだった主要パッケージが実際にimport可能か検証する --
        # 存在理由は_verify_key_packages()のdocstringと
        # CORE_IMPORT_CHECKS/SIMULATOR_IMPORT_CHECKSのモジュールコメント
        # 参照(数行上の`pip install -r requirements.txt`失敗はwarn()のみで
        # 見過ごしやすい)。--skip-depsが指定された場合は完全にスキップする
        # (今回の実行では何も(再)導入していないため)。selfに保持し、
        # 後述の"Installation Complete!"バナーで目立つ形に提示できるように
        # する(以前の警告群に埋もれさせない)。
        self._missing_packages: List[str] = []
        if not skip_deps:
            checks = CORE_IMPORT_CHECKS if minimal else CORE_IMPORT_CHECKS + SIMULATOR_IMPORT_CHECKS
            self._missing_packages = self._verify_key_packages(idf_path, checks)

        # Check hidapi native library for joystick support
        # ジョイスティック用のhidapiネイティブライブラリを確認
        self._check_hidapi()

        # Install udev rules on Linux (for USB HID access without root)
        # Linux: udevルールをインストール（root不要でUSB HIDアクセス）
        if sys.platform == "linux" and not is_wsl():
            self._install_udev_rules()

        # Save configuration
        self._save_config(idf_path)

        # Create the "StampFly Terminal" double-click launcher (setup_env
        # pre-loaded) right after Step 3/4 (StampFly CLI) succeeds, and
        # before the Step 4/4 header below. Deliberately NOT its own
        # "Step N/4": the Stability contract at the top of this file says
        # the GUI parses "Step N/4:" header lines to advance its step
        # indicator, and that count must stay at 4.
        # Step3/4（StampFly CLI）成功直後・下のStep4/4ヘッダの前に、
        # 「StampFly Terminal」ダブルクリックランチャー（setup_env読み込み
        # 済み）を作成する。意図的に独立した「Step N/4」にはしない —
        # 本ファイル冒頭の安定契約のとおり、GUIは"Step N/4:"ヘッダ行を
        # パースしてステップインジケータを進めるため、その数は4のまま
        # 維持しなければならない。
        self._create_terminal_launcher()

        # Step 4: GUI Flasher (optional). Deliberately best-effort: its own
        # failure never turns the overall installer result into a failure
        # (return value is not checked), unlike Steps 1-3 which `return 1`
        # on error. `sf flash --gui` (the pre-existing script-launch path)
        # remains available either way.
        # Step4: GUIフラッシャ（任意）。意図的にベストエフォートとし、失敗しても
        # インストーラー全体の結果を失敗にしない（戻り値を確認しない）。
        # Step1-3はエラー時に`return 1`するが、これとは異なる扱い。どちらに
        # せよ既存のスクリプト起動経路`sf flash --gui`は引き続き使える。
        header("Step 4/4: GUI Flasher (optional)")
        self._install_flasher_gui(idf_path, no_flasher=no_flasher)

        # SILS development toolchain (optional, not its own "Step N/4" --
        # see _install_sils_toolchain()'s docstring for why).
        # SILS開発ツールチェーン（任意。独立した「Step N/4」にしない理由は
        # _install_sils_toolchain() のdocstring参照）。
        self._install_sils_toolchain(idf_path, with_sils_toolchain=with_sils_toolchain)

        # Show completion message
        header("Installation Complete!")

        if self._missing_packages:
            # Deliberately its own visually distinct block, printed right
            # in the completion banner rather than folded into the warn()
            # lines back in Step 3/4 -- see _verify_key_packages() and the
            # CORE_IMPORT_CHECKS/SIMULATOR_IMPORT_CHECKS module comment for
            # why a warn()-only failure there is easy to miss. This must
            # never read as "All OK".
            # Step3/4中のwarn()行に埋もれさせず、完了バナーの中に意図的に
            # 見た目上独立したブロックとして出す -- 理由は
            # _verify_key_packages()とCORE_IMPORT_CHECKS/
            # SIMULATOR_IMPORT_CHECKSのモジュールコメント参照(そこでの
            # warn()のみの失敗は見過ごしやすい)。これは決して「全てOK」に
            # 見えてはならない。
            warn("Some packages could not be installed automatically:")
            warn("以下のパッケージは自動導入できませんでした:")
            for package_name in self._missing_packages:
                warn(f"  - {package_name}")
            idf_python = _find_idf_python(idf_path)
            fix_cmd = (
                f"{idf_python} -m pip install {' '.join(self._missing_packages)}"
                if idf_python
                else f"pip install {' '.join(self._missing_packages)}  (inside the ESP-IDF venv)"
            )
            warn(f"Fix: {fix_cmd}")
            print()

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

    def clean(
        self,
        idf_path: Optional[Path] = None,
        no_flasher: bool = False,
        auto_install_python: bool = False,
        with_sils_toolchain: bool = True,
    ) -> int:
        """Clean install: remove config and sfcli, then reinstall.
        クリーンインストール: 設定とsfcliを削除後、再インストール"""
        header("Cleaning StampFly installation...")

        # Find ESP-IDF path from config or argument
        # 設定またはコマンドライン引数からESP-IDFパスを取得
        resolved_idf_path = idf_path
        if not resolved_idf_path and self.config_file.exists():
            for line in self.config_file.read_text(encoding="utf-8").split('\n'):
                if line.startswith('path = "'):
                    resolved_idf_path = Path(line.split('"')[1])
                    break

        # Uninstall the GUI Flasher desktop app first, while sfcli is still
        # importable in the venv (flasher uninstall shells out to sfcli).
        # sfcliがまだvenvでimportできるうちに、先にGUIフラッシャの
        # デスクトップアプリをアンインストールする（flasher uninstallは
        # sfcli経由で動くため）
        self._uninstall_flasher_gui(resolved_idf_path)

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
        self._print_uninstall_leftovers_table()

        # Re-run installation
        return self.run(idf_path=idf_path, force=True, no_flasher=no_flasher,
                         auto_install_python=auto_install_python,
                         with_sils_toolchain=with_sils_toolchain)

    def _uninstall_flasher_gui(self, idf_path: Optional[Path]) -> None:
        """Uninstall the GUI Flasher desktop app before removing sfcli.
        sfcliを削除する前にGUIフラッシャのデスクトップアプリをアンインストール

        Best-effort: if no ESP-IDF venv matching `idf_path` can be found
        (e.g. sfcli's install is already half-removed, or no ESP-IDF was
        ever configured), skip silently rather than error — there is
        nothing to uninstall the flasher *from*. Any other failure (the
        flasher was never installed, or its own uninstall logic errors) is
        a warning only: it must never block the surrounding
        `--uninstall` / `--clean` flow.
        ベストエフォート: `idf_path` に対応するESP-IDF venvが見つからない
        場合(sfcliのインストールが既に半分消えている、あるいはESP-IDFが
        一度も設定されていない等)は、エラーにせず黙ってスキップする —
        アンインストールする対象のフラッシャ自体が存在しない。それ以外の
        失敗(フラッシャが未インストールだった、アンインストール処理自体が
        エラーになった等)は警告に留め、周囲の`--uninstall`/`--clean`の
        流れを絶対にブロックしない。
        """
        if not idf_path or not ESPIDFDetector._is_valid_idf(idf_path):
            return
        if not _find_idf_python(idf_path):
            return
        info("Uninstalling GUI Flasher (if installed)...")
        rc = _run_sf_in_idf_env(idf_path, ["flasher", "uninstall", "--yes"])
        if rc != 0:
            warn("GUI Flasher uninstall skipped or failed (continuing; "
                 "this is not fatal to the rest of the uninstall).")

    def _print_uninstall_leftovers_table(self) -> None:
        """Show what `--uninstall` / `--clean` deliberately do NOT remove.
        `--uninstall`/`--clean` が意図的に削除しないものの一覧を表示する

        sfcli only owns its own package (+ its config file + the GUI
        Flasher app it installed). ESP-IDF itself, its toolchain download
        cache, the rest of the venv's dependencies, the udev rules, and the
        repository checkout are left untouched: they may be shared with
        other tools (a different project's ESP-IDF setup, another venv) or
        the user may simply want to keep them. Each row gives the manual
        command to remove it if desired. Mirrors
        docs/guides/upgrading.md §アンインストール (same table, aimed at
        end users rather than this CLI's stdout).
        sfcliが所有するのは自身のパッケージ(+ configファイル + 自身が
        導入したGUIフラッシャアプリ)のみ。ESP-IDF本体・そのツールチェーン
        ダウンロードキャッシュ・venvの他の依存関係・udevルール・
        リポジトリ本体は、他のツール(別プロジェクトのESP-IDF環境や別の
        venv)と共有されていたり、ユーザが単に残しておきたい場合があるため
        あえて削除しない。各行に手動削除コマンドを添える。
        docs/guides/upgrading.md §アンインストール(同じ表をエンドユーザ
        向けに整理したもの)と対応する。
        """
        header("Not removed (manual cleanup if you want it gone)")
        # (what, where, how to remove it manually)
        # (対象, 場所, 手動削除コマンド)
        rows = [
            ("ESP-IDF checkout", "~/esp/esp-idf (or your --idf-path)",
             "rm -rf ~/esp/esp-idf"),
            ("IDF_TOOLS_PATH", "~/.espressif (or C:\\Espressif on Windows)",
             "rm -rf ~/.espressif"),
            ("venv dependencies", "<IDF_TOOLS_PATH>/python_env/idf<ver>_py*_env",
             "rm -rf <that venv dir>"),
            ("udev rules (Linux)", "/etc/udev/rules.d/99-stampfly.rules",
             "sudo rm /etc/udev/rules.d/99-stampfly.rules"),
            ("repository checkout", str(self.root),
             f"rm -rf {self.root}"),
        ]
        what_w = max(len(r[0]) for r in rows)
        where_w = max(len(r[1]) for r in rows)
        print(f"  {'What':{what_w}}  {'Where':{where_w}}  Manual removal")
        print(f"  {'-' * what_w}  {'-' * where_w}  {'-' * 30}")
        for what, where, cmd in rows:
            print(f"  {what:{what_w}}  {where:{where_w}}  {cmd}")
        print()
        print("  See docs/guides/upgrading.md (§アンイン"
              "ストール) for more detail.")
        print()

    def _install_flasher_gui(self, idf_path: Path, no_flasher: bool) -> None:
        """Offer to install the GUI Flasher as a native desktop app (Step 4/4).
        GUIフラッシャをネイティブデスクトップアプリとしてインストールする案内（Step4/4）

        Deliberately best-effort and strictly optional: unlike Steps 1-3,
        failure here is never propagated as an installer failure — callers
        do not check a return value. `sf flash --gui` (the pre-existing
        script-launch fallback in flash.py) remains available regardless of
        whether this step succeeds, is declined, or fails, so skipping or
        failing this step never blocks the rest of the ecosystem install.
        意図的にベストエフォート・完全に任意とする: Step1-3と異なり、ここでの
        失敗はインストーラーの失敗として伝播しない（呼び出し側は戻り値を
        確認しない）。このステップが成功・辞退・失敗のいずれであっても、
        既存のスクリプト起動フォールバック `sf flash --gui`（flash.py）は
        引き続き使えるため、本ステップのスキップ・失敗がエコシステム全体の
        インストールを妨げることはない。
        """
        if no_flasher:
            info("Skipping GUI Flasher install (--no-flasher).")
            return

        response = prompt("Install the GUI Flasher as a desktop app? [Y/n]", "Y")
        if response.lower() not in ("y", "yes", ""):
            info("Skipping GUI Flasher install.")
            info("You can install it later with: sf flasher install")
            return

        # The desktop-shortcut question only changes behavior on Windows
        # (Start Menu shortcut is always created there regardless; macOS
        # copies to ~/Applications and Linux registers a .desktop launcher,
        # neither of which has a separate "desktop icon" concept in this
        # design). It is still asked on every OS for one uniform installer
        # flow — non-Windows backends simply ignore --no-desktop-shortcut.
        # デスクトップショートカットの質問が意味を持つのはWindowsのみ
        # （スタートメニューショートカットは常に作成される。macOSは
        # ~/Applicationsへコピー、Linuxは.desktopランチャーを登録し、
        # どちらもこの設計では別個の「デスクトップアイコン」概念を持たない）。
        # インストーラーの流れを全OSで統一するため質問自体は毎回行う。
        # Windows以外のバックエンドは --no-desktop-shortcut を単に無視する。
        shortcut_response = prompt(
            "Also create a desktop shortcut? (Windows only) [Y/n]", "Y"
        )
        desktop_shortcut = shortcut_response.lower() in ("y", "yes", "")

        sf_args = ["flasher", "install", "--yes"]
        if not desktop_shortcut:
            sf_args.append("--no-desktop-shortcut")

        info("Installing GUI Flasher (StampFly Flasher)...")
        rc = _run_sf_in_idf_env(idf_path, sf_args)
        if rc == 0:
            success("GUI Flasher installed!")
        else:
            warn("GUI Flasher install failed (this does not affect the rest of the install).")
            warn("You can install it manually later with: sf flasher install")

    def _install_sils_toolchain(self, idf_path: Path, with_sils_toolchain: bool) -> None:
        """Offer to install the SILS development toolchain (Windows:
        MSYS2/MinGW-w64) used to build simulator/sils/ from source. Installs
        by DEFAULT (both the interactive prompt and the non-interactive
        flag default to Yes) -- opt out with --no-sils-toolchain.
        SILS開発ツールチェーン（Windows: MSYS2/MinGW-w64）の導入を案内する。
        既定でインストールする（対話プロンプト・非対話フラグとも既定はYes）
        -- 除外するには --no-sils-toolchain を使う。

        Deliberately best-effort, same mindset as _install_flasher_gui():
        failure here is never propagated as an installer failure, and this
        is NOT its own "Step N/4" -- same reasoning as
        _create_terminal_launcher() (see the module docstring's Stability
        contract point 2: the GUI's step indicator is hardcoded to 4 steps).

        Non-interactive contract mirrors _install_flasher_gui()'s, not
        auto_install_python's: interactive mode ALWAYS asks via a y/n prompt
        (default Yes) regardless of `with_sils_toolchain`; non-interactive
        mode installs unless `with_sils_toolchain` is False (--no-sils-toolchain).

        The actual Windows install (MSYS2 + MinGW-w64 toolchain) is NOT
        reimplemented here: it delegates to `sf sils install-toolchain`
        (lib/sfcli/utils/msys2_install.py) via _run_sf_in_idf_env(), the
        same pattern _install_flasher_gui() uses for `sf flasher install`.
        This method only runs after Step 3/4 (StampFly CLI) has succeeded,
        so sfcli is guaranteed importable in the target venv by then --
        reimplementing the MSYS2/pacman sequence a second time here would
        just be a second copy that can silently drift from the one in
        lib/sfcli (exactly what happened before this was fixed: commit
        3c4b85f2 had to hand-port a winget-avoidance fix from
        msys2_install.py into a duplicate copy that lived here).

        _install_flasher_gui() と同じくベストエフォート: ここでの失敗は
        インストーラーの失敗として伝播せず、独立した「Step N/4」にもしない
        -- _create_terminal_launcher() と同じ理由（モジュールdocstringの
        安定契約項目2参照: GUIのステップインジケータは4ステップ固定）。

        非対話契約は auto_install_python 型ではなく _install_flasher_gui()型:
        対話モードは `with_sils_toolchain` の値に関わらず常にy/nプロンプト
        （既定Yes）で尋ねる。非対話モードは `with_sils_toolchain` がFalse
        （--no-sils-toolchain）の時のみスキップする。

        Windows での実際の導入（MSYS2 + MinGW-w64ツールチェーン）はここで
        再実装しない: `_run_sf_in_idf_env()` 経由で `sf sils install-toolchain`
        （lib/sfcli/utils/msys2_install.py）に委譲する -- `_install_flasher_gui()`
        が `sf flasher install` に対して使うのと同じパターン。本メソッドは
        Step3/4（StampFly CLI）成功後にのみ呼ばれるため、この時点で対象venv
        内の sfcli は import 可能であることが保証されている -- ここで
        MSYS2/pacman手順を再実装すると、lib/sfcli 側の実装と気付かぬうちに
        乖離しうる二つ目のコピーを作るだけになる（実際に過去そうなった:
        コミット 3c4b85f2 で、ここにあった独自コピーへ
        msys2_install.py のwinget回避修正を手作業で移植する羽目になった）。
        """
        header("SILS Development Toolchain")

        if os.environ.get("SF_INSTALLER_NONINTERACTIVE") == "1":
            want = with_sils_toolchain
        else:
            response = prompt(
                "Install the SILS development toolchain (MSYS2/MinGW-w64, "
                "~2GB)? Needed to build/run the SILS simulator from source "
                "(control systems development). / SILSシミュレータを"
                "ソースからビルドして制御開発を行うために必要です"
                "（約2GB）[Y/n]",
                "Y",
            )
            want = response.lower() in ("y", "yes")

        if not want:
            info("Skipping SILS development toolchain install.")
            info("(install later: see simulator/sils/README.md, or run "
                 "`sf sils install-toolchain` once the StampFly CLI is set up)")
            return

        if sys.platform != "win32":
            info("SILS can be built with this machine's own gcc/clang -- no "
                 "separate toolchain install is needed here.")
            if sys.platform == "darwin":
                info("macOS: install the Xcode Command Line Tools if not "
                     "already present:")
                info("  xcode-select --install")
            else:
                manager = _detect_linux_package_manager()
                info("Linux: ensure a C++17 toolchain, cmake, and ninja are "
                     "installed, e.g.:")
                info(f"  {_linux_sils_toolchain_hint(manager)}")
            info("See simulator/sils/README.md for build instructions.")
            return

        info("Installing the MinGW-w64 toolchain via `sf sils "
             "install-toolchain` (direct MSYS2 download, ~2GB -- can take "
             "several to twenty-plus minutes depending on connection "
             "speed)... / `sf sils install-toolchain` 経由でMinGW-w64"
             "ツールチェーンを導入します（MSYS2直接ダウンロード、約2GB "
             "-- 回線速度により数分〜十数分かかります）")
        rc = _run_sf_in_idf_env(idf_path, ["sils", "install-toolchain", "--yes"])
        if rc != 0:
            warn(f"`sf sils install-toolchain` exited with code {rc}.")
            warn("Manual steps: see simulator/sils/README.md (Windows "
                 "native build section).")

    def _create_terminal_launcher(self) -> None:
        """Create a double-click launcher that opens a terminal with
        setup_env already sourced, so a newcomer can start using `sf`
        without learning "open a terminal, cd, source setup_env.sh" first.
        ダブルクリックすると setup_env 読み込み済みの端末が開くランチャーを
        作成する。初心者が「ターミナルを開いて cd して source する」を
        覚えずに `sf` を使い始められるようにするため

        Deliberately best-effort, same mindset as _install_flasher_gui()
        (Step 4/4, spec §4-2): failure here must never fail the overall
        installer -- a newcomer who does not get the launcher can still
        fall back to sourcing setup_env.sh by hand, exactly as before this
        feature existed.
        _install_flasher_gui()（Step4/4、仕様4-2）と同じくベストエフォート:
        ここでの失敗はインストーラー全体を失敗にしない -- ランチャーが
        作れなくても、この機能が無かった頃と同じく手動の
        `source setup_env.sh` に頼れる。
        """
        info(f'Creating "{TERMINAL_LAUNCHER_NAME}" launcher...')
        try:
            if sys.platform == "darwin":
                self._create_terminal_launcher_macos()
            elif sys.platform == "linux":
                self._create_terminal_launcher_linux()
            elif sys.platform == "win32":
                self._create_terminal_launcher_windows()
            else:
                # Unknown platform: no well-defined "double-click launcher"
                # concept here, so skip quietly rather than guess.
                # 未知プラットフォーム: 「ダブルクリックランチャー」の概念が
                # 定まらないため、推測せず黙ってスキップする
                info(f'Skipping "{TERMINAL_LAUNCHER_NAME}" launcher (unsupported platform: {sys.platform}).')
                return
        except Exception as e:
            warn(f'Failed to create "{TERMINAL_LAUNCHER_NAME}" launcher: {e}')
            warn(f"You can still start manually: source {self.root / 'setup_env.sh'}")
            return

        success(f'"{TERMINAL_LAUNCHER_NAME}" launcher created')

    def _create_terminal_launcher_macos(self) -> None:
        """macOS: ~/Applications/StampFly Terminal.app (a real bundle).
        macOS: ~/Applications/StampFly Terminal.app（正式なバンドル）

        Bundle layout / バンドル構成:
            Contents/Info.plist                  -- name, id, icon wiring
            Contents/MacOS/StampFlyTerminal      -- `open`s the inner .command
            Contents/Resources/icon.icns         -- from the repo (optional)
            Contents/Resources/<name>.command    -- the actual worker script

        Why an app bundle: a bare .command never shows up in Launchpad and
        cannot carry its own icon. The bundle's executable only `open`s the
        inner .command, so which terminal app runs it is still decided by
        the user's `.command` file association (Terminal.app by default,
        iTerm2 etc. if the user changed it -- see docs/guides/
        gui-installer.md FAQ).
        素の .command は Launchpad に出ず専用アイコンも持てないため、
        バンドル化する。バンドルの実行体は内部の .command を `open` する
        だけなので、どのターミナルで開くかは従来どおりユーザーの
        `.command` 関連付け（既定 Terminal.app、変更していれば iTerm2 等
        -- docs/guides/gui-installer.md の FAQ 参照）が決める。

        The worker script hands off to an interactive shell whose rc
        wrapper sources the user's own config (.zshenv/.zprofile/.zshrc)
        FIRST and setup_env.sh LAST. Ordering is load-bearing: the first
        iteration sourced setup_env.sh in the worker and then exec'd
        `$SHELL -i`, which let .zshrc's pyenv init prepend its shims OVER
        the ESP-IDF venv on PATH -- idf.py's `#!/usr/bin/env python`
        shebang then resolved to a bare interpreter and died with
        "No module named 'click'" (observed 2026-07-22, macOS). Sourcing
        setup_env.sh after the user's rc keeps the venv first.
        ワーカースクリプトは、rc ラッパーがユーザー自身の設定
        (.zshenv/.zprofile/.zshrc)を**先に**、setup_env.sh を**最後に**
        source する対話シェルへ引き継ぐ。この順序が本質: 初期実装は
        ワーカー内で setup_env.sh を source してから `$SHELL -i` に exec
        していたため、.zshrc の pyenv init が ESP-IDF venv より前に shims
        を PATH へ積んでしまい、idf.py の `#!/usr/bin/env python` shebang
        が素のインタプリタに解決されて「No module named 'click'」で死んだ
        (2026-07-22, macOS で観測)。ユーザー rc の後に setup_env.sh を
        source することで venv が先頭を維持する。
        """
        apps_dir = Path.home() / "Applications"
        apps_dir.mkdir(parents=True, exist_ok=True)

        # Migrate away from the first iteration's bare .command.
        # 初期実装の素の .command からの移行(削除)。
        legacy_command = apps_dir / TERMINAL_LAUNCHER_MACOS_FILENAME
        if legacy_command.exists():
            legacy_command.unlink()

        app_dir = apps_dir / TERMINAL_LAUNCHER_MACOS_APP_DIRNAME
        macos_dir = app_dir / "Contents" / "MacOS"
        resources_dir = app_dir / "Contents" / "Resources"
        macos_dir.mkdir(parents=True, exist_ok=True)
        resources_dir.mkdir(parents=True, exist_ok=True)

        command_path = resources_dir / TERMINAL_LAUNCHER_MACOS_FILENAME
        command_content = (
            "#!/bin/zsh\n"
            "# Worker script: open a terminal with the StampFly dev\n"
            "# environment pre-loaded.\n"
            "# ワーカースクリプト: StampFly 開発環境入りの端末を開く\n"
            "#\n"
            "# The interactive shell must load the user's OWN config first\n"
            "# and setup_env.sh LAST, so the ESP-IDF venv stays ahead of\n"
            "# pyenv shims and similar PATH prepends on PATH. Sourcing\n"
            "# setup_env.sh here and exec'ing '$SHELL -i' afterwards put\n"
            "# pyenv in front and broke idf.py (\"No module named 'click'\").\n"
            "# 対話シェルはユーザー自身の設定を先に、setup_env.sh を最後に\n"
            "# 読み込む必要がある(ESP-IDF venv を pyenv shims 等の PATH\n"
            "# 先頭追加より前に保つため)。ここで setup_env.sh を source\n"
            "# してから '$SHELL -i' に exec する旧方式は pyenv が前に来て\n"
            "# idf.py が壊れた(\"No module named 'click'\")。\n"
            '_sf_user_shell="${SHELL:-/bin/zsh}"\n'
            '_sf_rc_dir="$(mktemp -d)"\n'
            'case "${_sf_user_shell##*/}" in\n'
            "    zsh)\n"
            '        cat > "${_sf_rc_dir}/.zshrc" <<\'SF_WRAP\'\n'
            "unset ZDOTDIR\n"
            '[ -f "$HOME/.zshenv" ] && source "$HOME/.zshenv"\n'
            '[ -f "$HOME/.zprofile" ] && source "$HOME/.zprofile"\n'
            '[ -f "$HOME/.zshrc" ] && source "$HOME/.zshrc"\n'
            f'cd "{self.root}"\n'
            "source setup_env.sh\n"
            '[ -n "$_SF_RC_DIR" ] && rm -rf "$_SF_RC_DIR"\n'
            "unset _SF_RC_DIR\n"
            "SF_WRAP\n"
            '        export _SF_RC_DIR="${_sf_rc_dir}"\n'
            '        ZDOTDIR="${_sf_rc_dir}" exec "${_sf_user_shell}" -i\n'
            "        ;;\n"
            "    bash)\n"
            '        cat > "${_sf_rc_dir}/.sf_bashrc" <<\'SF_WRAP\'\n'
            '[ -f "$HOME/.bash_profile" ] && source "$HOME/.bash_profile"\n'
            '[ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc"\n'
            f'cd "{self.root}"\n'
            "source setup_env.sh\n"
            '[ -n "$_SF_RC_DIR" ] && rm -rf "$_SF_RC_DIR"\n'
            "unset _SF_RC_DIR\n"
            "SF_WRAP\n"
            '        export _SF_RC_DIR="${_sf_rc_dir}"\n'
            '        exec "${_sf_user_shell}" --rcfile "${_sf_rc_dir}/.sf_bashrc" -i\n'
            "        ;;\n"
            "    *)\n"
            "        # Unknown shell: fall back to the pre-wrapper behavior.\n"
            "        # 未知のシェル: ラッパー導入前の挙動にフォールバック。\n"
            '        rm -rf "${_sf_rc_dir}"\n'
            f'        cd "{self.root}"\n'
            "        source setup_env.sh\n"
            '        exec "${_sf_user_shell}" -i\n'
            "        ;;\n"
            "esac\n"
        )
        command_path.write_text(command_content)
        command_path.chmod(TERMINAL_LAUNCHER_MACOS_MODE)

        executable_path = macos_dir / TERMINAL_LAUNCHER_MACOS_EXECUTABLE
        executable_content = (
            "#!/bin/zsh\n"
            "# App-bundle entry point: delegate to the worker .command via\n"
            "# `open` so the user's .command file association (Terminal.app,\n"
            "# iTerm2, ...) decides which terminal application opens it.\n"
            "# アプリバンドルのエントリポイント: `open` 経由でワーカーの\n"
            "# .command へ委譲し、どのターミナルで開くかはユーザーの\n"
            "# .command 関連付け(Terminal.app / iTerm2 等)に委ねる\n"
            'HERE="$(cd "$(dirname "$0")" && pwd)"\n'
            f'exec open "$HERE/../Resources/{TERMINAL_LAUNCHER_MACOS_FILENAME}"\n'
        )
        executable_path.write_text(executable_content)
        executable_path.chmod(TERMINAL_LAUNCHER_MACOS_MODE)

        icon_source = self.root / TERMINAL_LAUNCHER_ICON_RELDIR / "icon.icns"
        bundle_icon_line = ""
        if icon_source.is_file():
            shutil.copy2(icon_source, resources_dir / "icon.icns")
            bundle_icon_line = (
                "  <key>CFBundleIconFile</key>\n"
                "  <string>icon.icns</string>\n"
            )

        info_plist = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"\n'
            '  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">\n'
            '<plist version="1.0">\n'
            "<dict>\n"
            "  <key>CFBundleName</key>\n"
            f"  <string>{TERMINAL_LAUNCHER_NAME}</string>\n"
            "  <key>CFBundleDisplayName</key>\n"
            f"  <string>{TERMINAL_LAUNCHER_NAME}</string>\n"
            "  <key>CFBundleIdentifier</key>\n"
            f"  <string>{TERMINAL_LAUNCHER_MACOS_BUNDLE_ID}</string>\n"
            "  <key>CFBundleVersion</key>\n"
            "  <string>1.0</string>\n"
            "  <key>CFBundlePackageType</key>\n"
            "  <string>APPL</string>\n"
            "  <key>CFBundleExecutable</key>\n"
            f"  <string>{TERMINAL_LAUNCHER_MACOS_EXECUTABLE}</string>\n"
            + bundle_icon_line +
            "  <key>LSApplicationCategoryType</key>\n"
            "  <string>public.app-category.developer-tools</string>\n"
            "</dict>\n"
            "</plist>\n"
        )
        (app_dir / "Contents" / "Info.plist").write_text(info_plist, encoding="utf-8")

        # Nudge LaunchServices to (re)register the bundle so Launchpad and
        # Spotlight pick up the new app and its icon without a re-login.
        # LaunchServices にバンドルを(再)登録させ、再ログイン無しで
        # Launchpad/Spotlight に新しいアプリとアイコンを認識させる。
        app_dir.touch()

    def _create_terminal_launcher_linux(self) -> None:
        """Linux: ~/.local/share/applications/stampfly-terminal.desktop
        Linux: ~/.local/share/applications/stampfly-terminal.desktop

        The whole Exec= value must parse as ONE shell word (freedesktop.org
        Desktop Entry spec), so the entire `bash -c '...'` invocation is
        wrapped in single quotes. If the install path itself contains a
        single quote, that quoting cannot be expressed safely -- skip
        creation and warn rather than emit a broken/misinterpreted .desktop
        entry.
        Exec= の値全体が(freedesktop.org の Desktop Entry 仕様上)1つの
        シェル単語として解釈される必要があるため、`bash -c '...'` 全体を
        シングルクォートで囲む。インストール先パス自体にシングルクォートが
        含まれる場合はこの引用を安全に表現できないため、壊れた/誤解釈される
        .desktop エントリを作るのではなく、作成をスキップして警告する。
        """
        root_str = str(self.root)
        if "'" in root_str:
            warn(
                f'"{TERMINAL_LAUNCHER_NAME}" launcher skipped: install path contains '
                f"a single quote, which cannot be safely embedded in a .desktop "
                f"Exec= line: {root_str}"
            )
            return

        applications_dir = Path.home() / ".local" / "share" / "applications"
        applications_dir.mkdir(parents=True, exist_ok=True)
        desktop_path = applications_dir / TERMINAL_LAUNCHER_LINUX_DESKTOP_ID

        exec_line = f'bash -c \'cd "{root_str}" && source setup_env.sh && exec bash -i\''
        # Prefer the repo's generated icon; fall back to the theme's generic
        # terminal icon when the asset is absent (icon must never block the
        # launcher).
        # リポジトリ同梱の生成済みアイコンを優先し、無ければテーマの汎用
        # ターミナルアイコンへフォールバック(アイコンのせいでランチャーが
        # 作れない事態にはしない)。
        icon_png = self.root / TERMINAL_LAUNCHER_ICON_RELDIR / "icon_256.png"
        icon_value = str(icon_png) if icon_png.is_file() else "utilities-terminal"
        content = (
            "[Desktop Entry]\n"
            "Type=Application\n"
            f"Name={TERMINAL_LAUNCHER_NAME}\n"
            "Comment=Open a terminal with the StampFly dev environment pre-loaded / "
            "StampFly開発環境を読み込み済みの端末を開く\n"
            f"Exec={exec_line}\n"
            "Terminal=true\n"
            f"Icon={icon_value}\n"
            "Categories=Development;\n"
        )
        desktop_path.write_text(content, encoding="utf-8")
        desktop_path.chmod(0o644)

        _refresh_linux_desktop_database(applications_dir)

    def _create_terminal_launcher_windows(self) -> None:
        """Windows: Start Menu shortcut launching cmd.exe with setup_env.bat.
        Windows: setup_env.bat を実行する cmd.exe のスタートメニューショートカット

        %APPDATA%\\Microsoft\\Windows\\Start Menu\\Programs\\StampFly\\StampFly Terminal.lnk

        Creates the .lnk via PowerShell's WScript.Shell COM object, the same
        approach lib/sfcli/utils/flasher_install/_windows.py's
        _create_shortcut() uses for the GUI Flasher's shortcuts (avoids a
        pywin32 dependency, which installer.py cannot take on -- see the
        Stability contract at the top of this file). The .ps1 is written as
        utf-8-sig so a non-ASCII user profile path (e.g. C:\\Users\\山田)
        round-trips correctly, for the same reason documented there.
        PowerShell の WScript.Shell COM オブジェクト経由で .lnk を作成する。
        lib/sfcli/utils/flasher_install/_windows.py の _create_shortcut() が
        GUIフラッシャのショートカット作成に使うのと同じ手法(pywin32依存を
        避ける -- installer.pyはそれを持てない。本ファイル冒頭の安定契約
        参照)。.ps1 は utf-8-sig で書き出し、非ASCIIなユーザープロファイル
        パス(例: C:\\Users\\山田)でも正しく往復させる(同ファイルに記載の
        理由と同じ)。
        """
        appdata = os.environ.get("APPDATA")
        if not appdata:
            warn(f'"{TERMINAL_LAUNCHER_NAME}" launcher skipped: APPDATA is not set.')
            return

        start_menu_dir = (
            Path(appdata) / "Microsoft" / "Windows" / "Start Menu" / "Programs"
            / TERMINAL_LAUNCHER_WINDOWS_START_MENU_FOLDER
        )
        start_menu_dir.mkdir(parents=True, exist_ok=True)
        link_path = start_menu_dir / TERMINAL_LAUNCHER_WINDOWS_LNK_NAME

        # TargetPath=%ComSpec% (cmd.exe); Arguments keep the shell open
        # after running setup_env.bat (/k, not /c) so the user lands in an
        # interactive prompt with the environment already loaded. The batch
        # path is double-quoted within Arguments so a root path containing
        # spaces still parses as one argument.
        # TargetPath=%ComSpec%(cmd.exe)。Arguments は setup_env.bat 実行後も
        # シェルを閉じない(/c ではなく /k)ことで、環境読み込み済みの対話
        # プロンプトにそのまま入れるようにする。root パスに空白が含まれても
        # 1引数として解釈されるよう、Arguments 内でバッチパスを二重引用する。
        comspec = os.environ.get("ComSpec", "cmd.exe")
        setup_env_bat = self.root / "setup_env.bat"
        arguments = f'/k "{setup_env_bat}"'

        # Give the shortcut the repo's generated icon when available; a
        # missing .ico simply leaves cmd.exe's default icon (never fatal).
        # リポジトリ同梱の生成済みアイコンがあればショートカットに設定する。
        # .ico が無ければ cmd.exe の既定アイコンのまま(失敗要因にはしない)。
        icon_ico = self.root / TERMINAL_LAUNCHER_ICON_RELDIR / "icon.ico"
        icon_line = (
            f"$sc.IconLocation = '{_ps_escape(str(icon_ico))},0'\n"
            if icon_ico.is_file() else ""
        )
        ps_script = (
            "$ws = New-Object -ComObject WScript.Shell\n"
            f"$sc = $ws.CreateShortcut('{_ps_escape(str(link_path))}')\n"
            f"$sc.TargetPath = '{_ps_escape(comspec)}'\n"
            f"$sc.Arguments = '{_ps_escape(arguments)}'\n"
            f"$sc.WorkingDirectory = '{_ps_escape(str(self.root))}'\n"
            + icon_line +
            "$sc.Save()\n"
        )

        # Written as a temp .ps1 (rather than passed via -Command) so
        # quoting is handled once here instead of twice (Python -> shell ->
        # PowerShell), mirroring _windows.py's _create_shortcut().
        # -Command 経由だと引用符がPython→シェル→PowerShellと二重に解釈
        # されるため、_windows.py の _create_shortcut() に倣い一時 .ps1
        # ファイルに書き出して -File で実行する。
        fd, tmp_name = tempfile.mkstemp(suffix=".ps1", prefix="stampfly_terminal_launcher_")
        ps1_path = Path(tmp_name)
        try:
            with os.fdopen(fd, "w", encoding="utf-8-sig") as f:
                f.write(ps_script)
            result = subprocess.run(
                [
                    "powershell", "-NoProfile", "-NonInteractive",
                    "-ExecutionPolicy", "Bypass", "-File", str(ps1_path),
                ],
                capture_output=True, text=True,
                encoding="utf-8", errors="replace",
            )
            if result.returncode != 0:
                raise RuntimeError(f"powershell exited {result.returncode}: {result.stderr.strip()}")
        finally:
            ps1_path.unlink(missing_ok=True)

    def _remove_terminal_launcher(self) -> None:
        """Remove the "StampFly Terminal" launcher created by
        _create_terminal_launcher() (best-effort, called from uninstall()).
        _create_terminal_launcher() が作成した「StampFly Terminal」
        ランチャーを削除する（ベストエフォート、uninstall() から呼ばれる）

        Missing paths are not an error -- uninstall must stay idempotent,
        mirroring _remove_path() in
        lib/sfcli/utils/flasher_install/_linux.py / _windows.py. On
        Windows, the shared "StampFly" Start Menu folder (see
        TERMINAL_LAUNCHER_WINDOWS_START_MENU_FOLDER) is removed only if it
        is now empty: it may still hold the GUI Flasher's own shortcut,
        which _uninstall_flasher_gui() owns and removes separately.
        存在しなくてもエラーにしない -- アンインストールは冪等でなければ
        ならない(lib/sfcli/utils/flasher_install/_linux.py / _windows.py の
        _remove_path() と同じ考え方)。Windowsでは、共有の「StampFly」
        スタートメニューフォルダ(TERMINAL_LAUNCHER_WINDOWS_START_MENU_FOLDER
        参照)は空になった場合のみ削除する -- GUIフラッシャ自身の
        ショートカットも同じフォルダに残っている可能性があり、そちらは
        _uninstall_flasher_gui() が別途所有・削除するため。
        """
        try:
            if sys.platform == "darwin":
                # Remove the app bundle, plus the first iteration's bare
                # .command if it is still around (legacy migration).
                # アプリバンドルを削除。初期実装の素の .command が残って
                # いればそれも削除(レガシー移行)。
                app_dir = Path.home() / "Applications" / TERMINAL_LAUNCHER_MACOS_APP_DIRNAME
                if app_dir.exists():
                    shutil.rmtree(app_dir, ignore_errors=True)
                launcher = Path.home() / "Applications" / TERMINAL_LAUNCHER_MACOS_FILENAME
                if launcher.exists():
                    launcher.unlink()
            elif sys.platform == "linux":
                applications_dir = Path.home() / ".local" / "share" / "applications"
                desktop_path = applications_dir / TERMINAL_LAUNCHER_LINUX_DESKTOP_ID
                if desktop_path.exists():
                    desktop_path.unlink()
                    _refresh_linux_desktop_database(applications_dir)
            elif sys.platform == "win32":
                appdata = os.environ.get("APPDATA")
                if not appdata:
                    return
                start_menu_dir = (
                    Path(appdata) / "Microsoft" / "Windows" / "Start Menu" / "Programs"
                    / TERMINAL_LAUNCHER_WINDOWS_START_MENU_FOLDER
                )
                link_path = start_menu_dir / TERMINAL_LAUNCHER_WINDOWS_LNK_NAME
                if link_path.exists():
                    link_path.unlink()
                # Only remove the shared folder if now empty (see docstring).
                # 共有フォルダは空になった場合のみ削除(docstring参照)
                if start_menu_dir.exists() and not any(start_menu_dir.iterdir()):
                    start_menu_dir.rmdir()
        except Exception as e:
            warn(f'Failed to remove "{TERMINAL_LAUNCHER_NAME}" launcher: {e}')

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

    def _verify_key_packages(self, idf_path: Path, checks: List[Tuple[str, str]]) -> List[str]:
        """Import-check each (module, pip package) pair in `checks` inside
        the ESP-IDF venv; for any that fails, retry once with a targeted
        `pip install <package>` and re-check. Returns the pip package names
        still unimportable after the retry (empty list if everything is
        OK). See the CORE_IMPORT_CHECKS/SIMULATOR_IMPORT_CHECKS module
        comment (near _module_importable()) for why this exists.
        checks の各(モジュール名, pipパッケージ名)の組を ESP-IDF venv 内で
        import検証する。失敗したものは的を絞った `pip install <package>` で
        1回だけ再試行し、再検証する。再試行後もimportできなかったpip
        パッケージ名を返す(全てOKなら空リスト)。存在理由は
        (_module_importable() 付近の)CORE_IMPORT_CHECKS/
        SIMULATOR_IMPORT_CHECKS のモジュールコメント参照。
        """
        venv_python = _find_idf_python(idf_path)
        if not venv_python:
            # No venv found at all -- treat every check as missing rather
            # than silently reporting "all OK".
            # venvが全く見つからない -- 「全てOK」と偽って報告するのでは
            # なく全チェックを欠落扱いにする
            return [package_name for _module_name, package_name in checks]

        info("Verifying key packages are importable...")
        still_missing: List[str] = []
        for module_name, package_name in checks:
            if _module_importable(venv_python, module_name):
                continue
            warn(f"{package_name} is not importable yet; retrying install...")
            _run_in_idf_env(idf_path, ["install", package_name])
            if _module_importable(venv_python, module_name):
                success(f"{package_name} installed on retry")
            else:
                still_missing.append(package_name)

        if not still_missing:
            success("All key packages verified")
        return still_missing

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
                warn("Install with: sudo apt install libhidapi-dev  # Debian/Ubuntu")
                warn("Or: sudo dnf install hidapi-devel  # Fedora")

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

        self.config_file.write_text(config_content, encoding="utf-8")
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
        for line in self.config_file.read_text(encoding="utf-8").split('\n'):
            if line.startswith('path = "'):
                idf_path = Path(line.split('"')[1])
                break

        if not idf_path:
            error("Could not determine ESP-IDF path from config.")
            return 1

        info(f"ESP-IDF path: {idf_path}")

        # Uninstall the GUI Flasher desktop app first (needs sfcli, which
        # we are about to remove).
        # GUIフラッシャのデスクトップアプリを先にアンインストール
        # (これから削除するsfcliに依存するため)
        self._uninstall_flasher_gui(idf_path)

        # Remove the "StampFly Terminal" launcher. Independent of sfcli/the
        # ESP-IDF venv (unlike the GUI Flasher uninstall above), so its
        # position relative to the sfcli uninstall below does not matter.
        # 「StampFly Terminal」ランチャーを削除する。(上のGUIフラッシャ
        # アンインストールと異なり)sfcli/ESP-IDF venvに依存しないため、
        # 下のsfcliアンインストールとの前後関係は問わない。
        self._remove_terminal_launcher()

        # Uninstall sfcli
        info("Uninstalling sfcli...")
        _run_in_idf_env(idf_path, ["uninstall", "-y", "stampfly-ecosystem"])

        # Remove config
        if self.config_file.exists():
            self.config_file.unlink()
            info("Removed configuration file")

        success("Uninstall complete!")
        self._print_uninstall_leftovers_table()
        return 0


def _guard_console_encoding() -> None:
    """Keep the installer alive on consoles that cannot encode every character.

    Windows consoles use a code page (cp932 on Japanese Windows, cp1252 on
    English) that cannot represent all UTF-8 text; the installer prints
    Japanese in several info()/warn() paths, which would raise
    UnicodeEncodeError and abort the install. errors="replace" prints '?'
    instead -- the same guard lib/sfcli/cli.py applies to `sf`.
    エンコードできない文字でインストーラを死なせない。Windows のコンソールは
    コードページ（日本語 Windows は cp932、英語は cp1252）で動き、UTF-8 の
    全文字は表現できない。本スクリプトは info()/warn() で日本語を出力する
    箇所があり、そこで UnicodeEncodeError が起きると導入が中断する。
    errors="replace" は '?' に置き換えて続行する（lib/sfcli/cli.py と同じ対処）。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError, OSError):
            # Non-reconfigurable stream (exotic redirection): leave as-is.
            # 再構成できないストリーム（特殊なリダイレクト）: そのまま。
            pass


def main() -> int:
    """Main entry point"""
    import argparse

    _guard_console_encoding()

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
    parser.add_argument(
        "--no-flasher",
        action="store_true",
        help="Skip the optional Step 4/4 GUI Flasher app install (sf flasher install)",
    )
    parser.add_argument(
        "--with-sils-toolchain",
        dest="with_sils_toolchain",
        action="store_true",
        default=True,
        help="Install the SILS development toolchain (Windows: "
             "MSYS2/MinGW-w64, ~2GB) used to build simulator/sils/ from "
             "source. This is now the DEFAULT (kept only for backward "
             "compatibility -- passing it changes nothing); use "
             "--no-sils-toolchain to opt out. Only --non-interactive mode "
             "reads this flag directly (interactive mode always asks via a "
             "y/n prompt, default Yes); on macOS/Linux this only prints "
             "guidance either way (no unattended package install there)",
    )
    parser.add_argument(
        "--no-sils-toolchain",
        dest="with_sils_toolchain",
        action="store_false",
        help="Skip the SILS development toolchain in --non-interactive "
             "mode (see --with-sils-toolchain)",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="Never call input(); prompt()/prompt_choice() return their default "
             "immediately (sets SF_INSTALLER_NONINTERACTIVE=1 for this process "
             "and any subprocesses it spawns)",
    )
    parser.add_argument(
        "--auto-install-python",
        action="store_true",
        help="Attempt to auto-install a system Python (3.10-3.12) via "
             "winget/brew when none is found. Only takes effect together "
             "with --non-interactive (interactive mode always asks via a "
             "y/n prompt regardless of this flag); Linux's sudo-gated "
             "install command is never run non-interactively (printed as "
             "guidance only, requires a real terminal either way)",
    )

    args = parser.parse_args()

    # Set this before anything else runs so both this process' own
    # prompt()/prompt_choice() calls and any child processes that inherit
    # os.environ see it (e.g. a GUI frontend importing this module in-process).
    # 他の処理より先に設定することで、この process 自身の prompt()/
    # prompt_choice() だけでなく os.environ を継承する子プロセス
    # （例: 本モジュールをプロセス内 import する GUI フロントエンド）
    # からも見えるようにする
    if args.non_interactive:
        os.environ["SF_INSTALLER_NONINTERACTIVE"] = "1"

    # Disable colors if requested or not a TTY
    if args.no_color or not sys.stdout.isatty():
        Colors.disable()

    installer = Installer()

    if args.uninstall:
        return installer.uninstall()
    elif args.clean:
        return installer.clean(
            idf_path=args.idf_path,
            no_flasher=args.no_flasher,
            auto_install_python=args.auto_install_python,
            with_sils_toolchain=args.with_sils_toolchain,
        )
    else:
        return installer.run(
            idf_path=args.idf_path,
            skip_deps=args.skip_deps,
            minimal=args.minimal,
            force=args.force,
            no_flasher=args.no_flasher,
            auto_install_python=args.auto_install_python,
            with_sils_toolchain=args.with_sils_toolchain,
        )


if __name__ == "__main__":
    sys.exit(main())
