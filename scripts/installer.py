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
    --non-interactive   Never call input(); return defaults instead

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

import os
import shlex
import sys
import subprocess
import shutil
import tempfile
from pathlib import Path
from typing import Optional, List, Tuple

# Ensure we're running Python 3.8+
if sys.version_info < (3, 8):
    print(f"Error: Python 3.8+ required, found {sys.version_info.major}.{sys.version_info.minor}")
    sys.exit(1)


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
    """Windows: likely directories containing a system python.exe, in the
    same priority order as install.bat's discovery block.
    Windows: システム python.exe を含むと思われるディレクトリ一覧。
    install.bat の発見ブロックと同じ優先順で返す。

    Kept in sync with install.bat (pyenv-win, per-user Programs\\Python,
    machine-wide C:\\Python*, scoop, conda). Duplicated here — not shared —
    because installer.py is a standalone stdlib-only script AND because the
    GUI (StampFly Setup) never runs install.bat at all: the frozen app
    imports installer.py in-process, so this discovery must live here for
    the GUI path to find Python for ESP-IDF's own install.bat.
    install.bat と同期を保つ(pyenv-win、ユーザー毎の Programs\\Python、
    マシン全体の C:\\Python*、scoop、conda)。共有せず複製する理由: 本
    ファイルは stdlib のみの独立スクリプトであり、かつ GUI(StampFly Setup)
    は install.bat を一切実行しない — 凍結アプリが installer.py をプロセス内
    import するため、GUI 経路が ESP-IDF の install.bat 用に Python を
    見つけられるよう、この発見ロジックはここに置く必要がある。
    """
    candidates: list[Path] = []
    userprofile = os.environ.get("USERPROFILE", "")
    localappdata = os.environ.get("LOCALAPPDATA", "")

    # 1. pyenv-win: the version selected in its `version` file.
    # 1. pyenv-win: `version` ファイルで選択中のバージョン。
    if userprofile:
        pyenv_root = Path(userprofile) / ".pyenv" / "pyenv-win"
        version_file = pyenv_root / "version"
        try:
            if version_file.is_file():
                pyenv_ver = version_file.read_text(encoding="utf-8", errors="replace").strip()
                if pyenv_ver:
                    candidates.append(pyenv_root / "versions" / pyenv_ver)
        except OSError:
            pass

    # 2. Common install locations (newest first).
    # 2. 一般的なインストール先(新しい順)。
    for ver in ("313", "312", "311", "310", "39", "38"):
        if localappdata:
            candidates.append(Path(localappdata) / "Programs" / "Python" / f"Python{ver}")
        candidates.append(Path(f"C:/Python{ver}"))
    if userprofile:
        candidates.append(Path(userprofile) / "scoop" / "apps" / "python" / "current")
        candidates.append(Path(userprofile) / "anaconda3")
        candidates.append(Path(userprofile) / "miniconda3")
    return candidates


def _python_is_3_8_plus(python_exe: Path) -> bool:
    """Return whether python_exe runs and reports Python >= 3.8.
    python_exe が起動し、Python 3.8 以上を報告するか。"""
    try:
        result = subprocess.run(
            [str(python_exe), "-c",
             "import sys; sys.exit(0 if sys.version_info[:2] >= (3, 8) else 1)"],
            capture_output=True, timeout=15,
        )
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _find_system_python_dir() -> Optional[Path]:
    """Windows: directory of a usable system python.exe (3.8+), or None.
    Windows: 使用可能なシステム python.exe(3.8+) のディレクトリ、無ければ None。

    Why this exists (the 9009 bug): ESP-IDF's own install.bat calls
    `python` by name, so a real python.exe must be on PATH for it. Under
    the CLI, install.bat already put one there; under the GUI, the frozen
    StampFly Setup app is `sys.executable` (no python.exe beside it), so we
    must discover a system Python separately -- otherwise ESP-IDF tool
    install fails with exit code 9009 ("command not found"), observed on a
    workshop Windows laptop 2026-07-20.
    存在理由(9009 バグ): ESP-IDF の install.bat は `python` を名前で呼ぶため、
    本物の python.exe が PATH 上に必要。CLI では install.bat が既に配置済み
    だが、GUI では凍結された StampFly Setup アプリが `sys.executable`
    (隣に python.exe は無い)のため、システム Python を別途発見しないと
    ESP-IDF ツール導入が exit 9009("コマンドが見つからない")で失敗する
    (2026-07-20 に講習用 Windows ノートで観測)。
    """
    if sys.platform != "win32":
        return None
    # 1. Already resolvable on PATH? Skip the WindowsApps stub, which is a
    # non-functional placeholder that only opens the Store.
    # 1. 既に PATH で解決できるか? Store を開くだけの機能しないプレース
    # ホルダである WindowsApps スタブは除外する。
    for name in ("python", "python3"):
        found = shutil.which(name)
        if not found:
            continue
        exe = Path(found)
        if "windowsapps" in str(exe).lower():
            continue
        if _python_is_3_8_plus(exe):
            return exe.parent
    # 2. Known install locations (same as install.bat).
    # 2. 既知のインストール先(install.bat と同一)。
    for candidate_dir in _windows_python_dir_candidates():
        exe = candidate_dir / "python.exe"
        if exe.is_file() and _python_is_3_8_plus(exe):
            return candidate_dir
    return None


def _clean_env_for_cmd() -> dict:
    """Return environment suitable for running .bat scripts via cmd.exe.
    cmd.exe 経由で .bat を実行するための環境を構築

    - Strips MSYSTEM (ESP-IDF .bat refuses to run under MINGW/Git Bash)
    - Ensures a REAL system python.exe directory is on PATH so ESP-IDF's
      install.bat (which calls `python` by name) can find it. Under the CLI
      that is `sys.executable`'s own directory; under the frozen GUI
      `sys.executable` is StampFly Setup itself (no python.exe there), so we
      fall back to discovering a system Python -- see _find_system_python_dir().
    - MSYSTEM を除去(ESP-IDF の .bat は MINGW/Git Bash 下での実行を拒否)
    - 本物のシステム python.exe のディレクトリを PATH に載せ、ESP-IDF の
      install.bat(`python` を名前で呼ぶ)が見つけられるようにする。CLI では
      `sys.executable` の隣がそれだが、凍結 GUI では `sys.executable` が
      StampFly Setup 自身(python.exe は無い)なので、システム Python の
      発見にフォールバックする -- _find_system_python_dir() 参照。
    """
    env = os.environ.copy()
    env.pop("MSYSTEM", None)

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
        if python_dir_str.lower() not in current_path.lower():
            # Prepend (not append): a WindowsApps python stub already on
            # PATH must not win over the real interpreter we found.
            # 先頭に付ける(末尾ではない): PATH 上に既にある WindowsApps の
            # python スタブが、発見した本物のインタプリタに勝たないように。
            env["PATH"] = python_dir_str + os.pathsep + current_path
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
    for var in ("VIRTUAL_ENV", "CONDA_PREFIX", "CONDA_DEFAULT_ENV",
                "CONDA_SHLVL", "PYTHONHOME", "PYTHONPATH"):
        env.pop(var, None)
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
        # ESP-IDF's install.bat calls `python` by name; without a real
        # system python.exe on PATH it fails with the cryptic exit code
        # 9009 ("command not found"). Detect that up front and print an
        # actionable message instead -- especially important for the GUI
        # (StampFly Setup), whose bundled Python is NOT usable here (see
        # _find_system_python_dir()). Observed on a workshop laptop
        # 2026-07-20.
        # ESP-IDF の install.bat は `python` を名前で呼ぶため、本物の
        # システム python.exe が PATH に無いと難解な exit 9009
        # ("コマンドが見つからない")で失敗する。先に検出して対処可能な
        # メッセージを出す -- 特に GUI(StampFly Setup)で重要で、同梱 Python
        # はここでは使えない(_find_system_python_dir() 参照)。
        # 2026-07-20 に講習用ノートで観測。
        if sys.platform == "win32" and _find_system_python_dir() is None:
            error("ESP-IDF tool installation needs a system Python 3.8+ on this PC,")
            error("but none was found. ESP-IDF's own install.bat requires it.")
            error("Install Python (make sure to keep 'Add python.exe to PATH' checked):")
            error("    winget install Python.Python.3.12")
            error("  or download from https://www.python.org/downloads/windows/")
            error("Then run this installer again.")
            return None

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
            # `import sfcli` alone is not enough: __init__.py does not read
            # any third-party dependency, so a venv with sfcli's *package*
            # present but a dependency missing (e.g. after a `pip install -e`
            # that partially failed) would still report "installed" here.
            # We import sfcli.cli (which imports every command module) and
            # call assert_all_commands_loadable() (added in cli.py under the
            # C2 dependency-resilience work) so a missing dependency makes
            # this probe correctly report "not installed" and Step 3 reruns.
            # `import sfcli` だけでは不十分: __init__.py はサードパーティ
            # 依存を一切読まないため、sfcliパッケージ自体はあるが依存が
            # 欠けている venv (`pip install -e` が部分失敗した後など) でも
            # ここは「インストール済み」と誤診断してしまう。sfcli.cli を
            # import し（全コマンドモジュールをimportする）、
            # assert_all_commands_loadable()（C2の依存耐性対応でcli.pyに
            # 追加）を呼ぶことで、依存欠落時はこのプローブが正しく
            # 「未インストール」と判定し、Step3が再実行されるようにする。
            result = subprocess.run(
                [
                    str(venv_python), "-c",
                    "import sfcli.cli; sfcli.cli.assert_all_commands_loadable()",
                ],
                capture_output=True, text=True,
                encoding="utf-8", errors="replace",
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
    ) -> int:
        """Run installation"""

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
        header("Step 2/4: Python Environment")

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

    def clean(self, idf_path: Optional[Path] = None, no_flasher: bool = False) -> int:
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
        return self.run(idf_path=idf_path, force=True, no_flasher=no_flasher)

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

        Inside the worker script, `exec "${SHELL:-/bin/zsh}" -i` hands off
        to an interactive shell after sourcing setup_env.sh so the
        variables exported there (PATH, IDF_PATH, ...) survive into the
        shell the user actually types into.
        ワーカースクリプト内の `exec "${SHELL:-/bin/zsh}" -i` は
        setup_env.sh を source した後に対話シェルへ引き継ぎ、export された
        変数(PATH, IDF_PATH等)をユーザーが実際に入力するシェルへ残す。
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
            "# environment pre-loaded (setup_env.sh already sourced).\n"
            "# ワーカースクリプト: StampFly 開発環境(setup_env.sh 読み込み\n"
            "# 済み)の端末を開く\n"
            f'cd "{self.root}"\n'
            "source setup_env.sh\n"
            "# Hand off to an interactive shell so the environment exported\n"
            "# above survives into the shell the user actually types into.\n"
            "# 上でexportされた環境が、ユーザーが実際に入力する対話シェルへ\n"
            "# 引き継がれるようにする\n"
            'exec "${SHELL:-/bin/zsh}" -i\n'
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
    parser.add_argument(
        "--no-flasher",
        action="store_true",
        help="Skip the optional Step 4/4 GUI Flasher app install (sf flasher install)",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="Never call input(); prompt()/prompt_choice() return their default "
             "immediately (sets SF_INSTALLER_NONINTERACTIVE=1 for this process "
             "and any subprocesses it spawns)",
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
        return installer.clean(idf_path=args.idf_path, no_flasher=args.no_flasher)
    else:
        return installer.run(
            idf_path=args.idf_path,
            skip_deps=args.skip_deps,
            minimal=args.minimal,
            force=args.force,
            no_flasher=args.no_flasher,
        )


if __name__ == "__main__":
    sys.exit(main())
