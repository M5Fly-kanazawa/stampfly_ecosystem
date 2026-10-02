"""
Editor detection utilities, shared by `sf lesson edit` and `sf app edit`

エディタ検出ユーティリティ。`sf lesson edit` と `sf app edit` で共有する。

Both commands open a single source file in the user's editor with the same
priority order: an explicitly requested editor -> VSCode (`code` on PATH) ->
a platform-specific VSCode install location -> vi -> vim -> Notepad
(Windows only). Extracted out of lesson.py (where it originated) so that
app.py can reuse it without importing a command module -- command modules
should stay siblings, not depend on each other, so utils/ is the right home.

両コマンドとも同じ優先順位でエディタを起動する: 明示指定 -> VSCode
（PATH上の`code`） -> プラットフォーム別VSCodeインストール先 -> vi -> vim ->
Notepad（Windowsのみ）。元々 lesson.py にあったロジックをここへ切り出し、
app.py がコマンドモジュールに依存せず再利用できるようにした
（コマンドモジュール同士は兄弟関係を保ち、依存させない方針。共有ロジックは
utils/ に置く）。
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple


def vscode_app_candidates() -> List[Tuple[str, List[str]]]:
    """Platform-specific VSCode install locations not on PATH.

    Returns list of (display_name, launch_command) for VSCode installations
    that exist on disk but whose CLI may not be on PATH. The launch_command
    must accept VSCode CLI flags (e.g., -n) directly.
    PATH 上に CLI がない VSCode インストールの (表示名, 起動コマンド) リスト。
    起動コマンドは VSCode CLI のフラグ（例: -n）を直接受け取れる形式であること。
    """
    candidates: List[Tuple[str, List[str]]] = []

    if sys.platform == "darwin":
        # macOS: prefer the `code` script inside the app bundle so VSCode CLI
        # flags (e.g. -n) can be passed directly. This avoids the awkward
        # `open -a "..." --args` invocation.
        # macOS: app バンドル内の `code` スクリプトを優先（-n 等のフラグを直接渡せる）
        bundled_code = Path("/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code")
        if bundled_code.exists():
            candidates.append(("VSCode", [str(bundled_code)]))

    elif sys.platform == "win32":
        # Windows: per-user and system-wide install locations
        # Windows: ユーザー単位とシステム全体のインストール先
        local_appdata = os.environ.get("LOCALAPPDATA", "")
        program_files = os.environ.get("ProgramFiles", "")
        program_files_x86 = os.environ.get("ProgramFiles(x86)", "")
        for base in (local_appdata, program_files, program_files_x86):
            if not base:
                continue
            cmd = Path(base) / "Programs" / "Microsoft VS Code" / "bin" / "code.cmd"
            if cmd.exists():
                candidates.append(("VSCode", [str(cmd)]))
                break
            cmd = Path(base) / "Microsoft VS Code" / "bin" / "code.cmd"
            if cmd.exists():
                candidates.append(("VSCode", [str(cmd)]))
                break

    elif sys.platform.startswith("linux"):
        # Linux: Snap and Flatpak installations may not put `code` on PATH
        # Linux: Snap や Flatpak は `code` を PATH に置かないことがある
        for path in ("/snap/bin/code", "/var/lib/flatpak/exports/bin/com.visualstudio.code"):
            if Path(path).exists():
                candidates.append(("VSCode", [path]))
                break

    return candidates


def find_editor(preferred: Optional[str] = None) -> Optional[Tuple[str, List[str]]]:
    """Find available editor.

    Search order: explicit preferred -> VSCode (code on PATH) -> platform VSCode app
    -> vi -> vim -> Windows Notepad.
    検索順: 明示指定 -> VSCode (PATH 上) -> プラットフォーム別 VSCode -> vi -> vim -> Notepad (Windows)

    Returns:
        (display_name, command_list) tuple, or None if no editor found.
    """
    if preferred:
        path = shutil.which(preferred)
        if path:
            return (preferred, [path])
        return None

    # VSCode CLI on PATH (handles `code`/`code.cmd` via PATHEXT on Windows)
    # PATH 上の VSCode CLI（Windows では PATHEXT 経由で `code.cmd` も検出）
    code_path = shutil.which("code")
    if code_path:
        return ("VSCode", [code_path])

    # Platform-specific VSCode locations
    # プラットフォーム別の VSCode インストール先
    for candidate in vscode_app_candidates():
        return candidate

    # vi / vim fallback (POSIX, also if installed via Git for Windows)
    # vi / vim フォールバック（POSIX、Git for Windows 経由のインストールも検出）
    for candidate_editor in ("vi", "vim"):
        path = shutil.which(candidate_editor)
        if path:
            return (candidate_editor, [path])

    # Windows last resort: Notepad
    # Windows 最後の手段: Notepad
    if sys.platform == "win32":
        notepad = shutil.which("notepad")
        if notepad:
            return ("Notepad", [notepad])

    return None


def install_hint(explicit_editor_example: str) -> List[str]:
    """Platform-specific install instructions for editors.
    プラットフォーム別のエディタインストール手順

    Args:
        explicit_editor_example: the caller's own command line for
            specifying an editor explicitly (e.g.
            "sf lesson edit --editor <command>" or
            "sf app edit <name> --editor <command>"), shown as the last
            line of the hint. Callers differ here, so it is not hardcoded.
            呼び出し元がエディタを明示指定する際のコマンド例
            （例: "sf lesson edit --editor <command>"）。呼び出し元ごとに
            異なるため引数で受け取る。
    """
    lines = [
        "  Install one of the following:",
        "    VSCode:  https://code.visualstudio.com/",
    ]
    if sys.platform == "darwin":
        lines.append("             After install, run from VSCode command palette:")
        lines.append("             'Shell Command: Install \"code\" command in PATH'")
        lines.append("    vim:     brew install vim")
    elif sys.platform == "win32":
        lines.append("             Or:  winget install Microsoft.VisualStudioCode")
        lines.append("             During install, check 'Add to PATH'")
        lines.append("    vim:     winget install vim.vim")
    elif sys.platform.startswith("linux"):
        lines.append("             Or via package manager (snap install code --classic etc.)")
        lines.append("    vim:     sudo apt install vim   /   sudo dnf install vim")
    else:
        lines.append("    vim:     install via your platform package manager")
    lines.append("")
    lines.append(f"  Or specify explicitly:  {explicit_editor_example}")
    return lines


# Editors that run inside the terminal and therefore must be waited for.
# Anything else (VSCode, Notepad, ...) is a GUI window and is launched
# without waiting, so the prompt returns immediately and the learner can
# keep the editor open while running `sf lesson build` etc.
# 端末内で動くエディタ（終了まで待つ必要がある）。それ以外（VSCode・メモ帳等）は
# GUI ウィンドウなので待たずに起動し、すぐプロンプトを返す。エディタを開いたまま
# `sf lesson build` 等を実行できるようにするため（Windows でメモ帳を閉じるまで
# コマンドプロンプトが固まる問題の対策）。
TERMINAL_EDITORS = ("vi", "vim", "nvim", "nano", "emacs", "micro", "hx")


def is_terminal_editor(cmd: List[str]) -> bool:
    """True if the editor runs inside the terminal (must be waited for).
    エディタが端末内で動く（終了を待つ必要がある）なら True"""
    stem = Path(cmd[0]).stem.lower()
    return stem in TERMINAL_EDITORS


def launch(cmd: List[str]) -> int:
    """Launch the editor; wait only for terminal editors.
    エディタを起動する。終了を待つのは端末内エディタのときだけ

    Returns the editor's exit code for terminal editors, 0 once a GUI
    editor has been started.
    端末内エディタなら終了コード、GUI エディタなら起動できた時点で 0 を返す。
    """
    if is_terminal_editor(cmd):
        return subprocess.run(cmd).returncode

    # Detach from the console: no shared stdio, and on POSIX a new session
    # so Ctrl+C in this terminal does not kill the editor.
    # コンソールから切り離す: 標準入出力を共有せず、POSIX では別セッションにして
    # この端末での Ctrl+C がエディタを巻き込まないようにする。
    subprocess.Popen(
        cmd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=(sys.platform != "win32"),
    )
    return 0
