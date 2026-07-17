"""
sf lesson - Workshop lesson management

Manages workshop lessons: list, switch, view solutions, build, flash.
ワークショップレッスンの管理: 一覧、切替、解答表示、ビルド、フラッシュ。

Subcommands:
    list      - List all available lessons
    switch    - Switch to a lesson (copy student.cpp to user_code.cpp)
    solution  - Show solution diff for a lesson
    info      - Show detailed lesson information
    edit      - Open user_code.cpp in editor (VSCode > vi)
    build     - Build workshop firmware (= sf build workshop)
    flash     - Flash workshop firmware (= sf flash workshop -m)
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..utils import console, paths

COMMAND_NAME = "lesson"
COMMAND_HELP = "Workshop lesson management"

# Lesson directory naming convention: lesson_NN_name
LESSONS_DIR = "lessons"
USER_CODE = "user_code.cpp"
MANIFEST_FILE = "lesson_manifest.yaml"

# Day grouping for display
DAY_GROUPS = [
    (1, [0, 1, 2]),
    (2, [3, 4, 5]),
    (3, [6, 7, 8]),
    (4, [9, 10, 11, 12]),
    (5, [13]),
]


def _get_lessons_dir() -> Path:
    """Get the lessons directory path"""
    return paths.workshop() / LESSONS_DIR


def _get_user_code_path() -> Path:
    """Get the user_code.cpp path"""
    return paths.workshop() / "main" / USER_CODE


def _ensure_user_code_exists() -> bool:
    """Bootstrap user_code.cpp from the Lesson 0 seed if absent.

    Mirrors the CMake configure-time bootstrap in
    firmware/workshop/main/CMakeLists.txt so that commands which only
    read or edit user_code.cpp (without invoking a build) still work on a
    fresh clone, before any `idf.py reconfigure` has been run.

    user_code.cpp は CMake configure 時にも自動生成されるが、
    `sf lesson edit` のようにビルドを経由しないコマンドのために、
    sf 側にも同等の bootstrap を置く（新規 clone 直後でも動作させる）。

    Returns True if the file exists (or was created), False if bootstrap
    failed (seed missing or copy error).
    """
    user_code = _get_user_code_path()
    if user_code.exists():
        return True

    seed = _get_lessons_dir() / "lesson_00_setup" / "student.cpp"
    if not seed.exists():
        return False

    try:
        user_code.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(seed, user_code)
    except OSError:
        return False

    console.info("Bootstrapped user_code.cpp from Lesson 0 template")
    console.print(f"  Path: {user_code}")
    return True


def _load_manifest() -> Optional[List[Dict[str, Any]]]:
    """Load lesson manifest YAML. Returns None if unavailable."""
    manifest_path = _get_lessons_dir() / MANIFEST_FILE
    if not manifest_path.exists():
        return None
    try:
        import yaml
        with open(manifest_path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data.get("lessons", [])
    except ImportError:
        return None
    except Exception:
        return None


def _discover_lessons() -> List[Tuple[int, str, Path]]:
    """Discover all available lessons.

    Returns list of (number, name, path) tuples sorted by number.
    """
    lessons_dir = _get_lessons_dir()
    if not lessons_dir.exists():
        return []

    lessons = []
    for d in sorted(lessons_dir.iterdir()):
        if not d.is_dir():
            continue
        # Parse lesson_NN_name format
        name = d.name
        if not name.startswith("lesson_"):
            continue
        parts = name.split("_", 2)
        if len(parts) < 3:
            continue
        try:
            num = int(parts[1])
        except ValueError:
            continue
        lesson_name = parts[2]
        lessons.append((num, lesson_name, d))

    return lessons


def _find_lesson(identifier) -> Optional[Path]:
    """Find lesson directory by number or ID.

    Args:
        identifier: int (lesson number) or str (lesson ID like 'motor_control')
    """
    manifest = _load_manifest()

    # Try manifest first for ID-based lookup
    if manifest and isinstance(identifier, str):
        for entry in manifest:
            if entry.get("id") == identifier:
                fw_dir = entry.get("firmware_dir")
                if fw_dir and fw_dir is not None:
                    path = _get_lessons_dir() / fw_dir
                    if path.exists():
                        return path
                return None

    # Number-based lookup (works with or without manifest)
    num = identifier if isinstance(identifier, int) else None
    if num is None:
        try:
            num = int(identifier)
        except (ValueError, TypeError):
            return None

    # If manifest available, use firmware_dir mapping
    if manifest:
        for entry in manifest:
            if entry.get("number") == num:
                fw_dir = entry.get("firmware_dir")
                if fw_dir and fw_dir is not None:
                    path = _get_lessons_dir() / fw_dir
                    if path.exists():
                        return path
                return None

    # Fallback: directory scan
    for n, _, path in _discover_lessons():
        if n == num:
            return path
    return None


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    lesson_subparsers = parser.add_subparsers(
        dest="lesson_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- list ---
    list_parser = lesson_subparsers.add_parser(
        "list",
        help="List all available lessons",
        description="Show all workshop lessons with status.",
    )
    list_parser.set_defaults(func=run_list)

    # --- switch ---
    switch_parser = lesson_subparsers.add_parser(
        "switch",
        help="Switch to a lesson",
        description="Copy lesson student.cpp to user_code.cpp for building.",
    )
    switch_parser.add_argument(
        "identifier",
        help="Lesson number (e.g., 0, 5) or ID (e.g., motor_control)",
    )
    switch_parser.add_argument(
        "--solution",
        action="store_true",
        help="Use solution.cpp instead of student.cpp",
    )
    switch_parser.set_defaults(func=run_switch)

    # --- solution ---
    solution_parser = lesson_subparsers.add_parser(
        "solution",
        help="Show solution for a lesson",
        description="Display diff between student.cpp and solution.cpp.",
    )
    solution_parser.add_argument(
        "identifier",
        help="Lesson number (e.g., 5) or ID (e.g., pid_control)",
    )
    solution_parser.set_defaults(func=run_solution)

    # --- info ---
    info_parser = lesson_subparsers.add_parser(
        "info",
        help="Show detailed lesson information",
        description="Display detailed information for a lesson.",
    )
    info_parser.add_argument(
        "identifier",
        help="Lesson number (e.g., 5) or ID (e.g., pid_control)",
    )
    info_parser.set_defaults(func=run_info)

    # --- edit ---
    edit_parser = lesson_subparsers.add_parser(
        "edit",
        help="Open user_code.cpp in editor",
        description="Open the workshop user_code.cpp in your editor (VSCode > vi).",
    )
    edit_parser.add_argument(
        "--editor",
        default=None,
        help="Override editor command (e.g., --editor nvim)",
    )
    edit_parser.add_argument(
        "--reuse-window",
        action="store_true",
        help="Open in existing VSCode window instead of a new one (default: new window)",
    )
    edit_parser.set_defaults(func=run_edit)

    # --- build ---
    build_parser = lesson_subparsers.add_parser(
        "build",
        help="Build workshop firmware",
        description="Build the workshop firmware (equivalent to 'sf build workshop').",
    )
    build_parser.add_argument(
        "-c", "--clean",
        action="store_true",
        default=False,
        help="Clean build before building",
    )
    build_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose build output",
    )
    build_parser.set_defaults(func=run_build)

    # --- flash ---
    flash_parser = lesson_subparsers.add_parser(
        "flash",
        help="Flash workshop firmware",
        description="Flash workshop firmware with monitor (equivalent to 'sf flash workshop -m').",
    )
    flash_parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (auto-detect if not specified)",
    )
    flash_parser.add_argument(
        "-b", "--baud",
        type=int,
        default=460800,
        help="Baud rate (default: 460800)",
    )
    flash_parser.add_argument(
        "--no-monitor",
        action="store_true",
        help="Don't start monitor after flashing",
    )
    flash_parser.set_defaults(func=run_flash)

    # Default: show help
    parser.set_defaults(func=lambda args: (parser.print_help(), 0)[1])


def _parse_identifier(raw: str):
    """Parse identifier as int if possible, otherwise return as string."""
    try:
        return int(raw)
    except ValueError:
        return raw


def run_list(args: argparse.Namespace) -> int:
    """List all available lessons"""
    manifest = _load_manifest()

    if manifest:
        return _run_list_manifest(manifest)
    return _run_list_fallback()


def _run_list_manifest(manifest: List[Dict[str, Any]]) -> int:
    """List lessons from manifest with day grouping"""
    # Check current lesson
    user_code_path = _get_user_code_path()
    current_content = ""
    if user_code_path.exists():
        current_content = user_code_path.read_text()

    # Build number->entry lookup
    by_number = {e["number"]: e for e in manifest}

    console.header("Workshop Lessons")
    console.print()

    for day, numbers in DAY_GROUPS:
        console.print(f"  Day {day}:")
        for num in numbers:
            entry = by_number.get(num)
            if not entry:
                continue

            fw_dir = entry.get("firmware_dir")
            lesson_path = None
            if fw_dir and fw_dir is not None:
                lesson_path = _get_lessons_dir() / fw_dir
                if not lesson_path.exists():
                    lesson_path = None

            # Check if current
            is_current = False
            if lesson_path and current_content:
                for fname in ("student.cpp", "solution.cpp"):
                    fpath = lesson_path / fname
                    if fpath.exists() and fpath.read_text() == current_content:
                        is_current = True
                        break

            marker = " >> " if is_current else "    "
            title_ja = entry.get("title_ja", "")
            title_en = entry.get("title_en", "")
            desc_ja = entry.get("description_ja", "")

            console.print(f"{marker}Lesson {num:2d}: {title_ja} / {title_en}")
            if desc_ja:
                console.print(f"              {desc_ja}")
        console.print()

    # Event templates (number >= 90, outside the Day 1-5 sequence)
    # イベント用テンプレート（90番以降、Day 1〜5 の通常レッスンとは別系統）
    day_numbers = {n for _, numbers in DAY_GROUPS for n in numbers}
    event_entries = [e for e in manifest if e["number"] not in day_numbers]
    if event_entries:
        console.print("  Events（教育イベント）:")
        for entry in sorted(event_entries, key=lambda e: e["number"]):
            fw_dir = entry.get("firmware_dir")
            lesson_path = None
            if fw_dir and fw_dir is not None:
                lesson_path = _get_lessons_dir() / fw_dir
                if not lesson_path.exists():
                    lesson_path = None

            is_current = False
            if lesson_path and current_content:
                for fname in ("student.cpp", "solution.cpp"):
                    fpath = lesson_path / fname
                    if fpath.exists() and fpath.read_text() == current_content:
                        is_current = True
                        break

            marker = " >> " if is_current else "    "
            title_ja = entry.get("title_ja", "")
            title_en = entry.get("title_en", "")
            desc_ja = entry.get("description_ja", "")
            console.print(
                f"{marker}Lesson {entry['number']:2d}: {title_ja} / {title_en}"
                f"  (id: {entry.get('id', '')})"
            )
            if desc_ja:
                console.print(f"              {desc_ja}")
        console.print()

    console.print(f"  Total: {len(manifest)} lessons")
    console.print(f"  Switch: sf lesson switch <N or id>")

    return 0


def _run_list_fallback() -> int:
    """List lessons by directory scan (fallback when no manifest)"""
    lessons = _discover_lessons()

    if not lessons:
        console.error("No lessons found")
        console.print(f"  Expected at: {_get_lessons_dir()}")
        return 1

    user_code_path = _get_user_code_path()
    current_content = ""
    if user_code_path.exists():
        current_content = user_code_path.read_text()

    console.header("Workshop Lessons")
    console.print()

    for num, name, path in lessons:
        has_student = (path / "student.cpp").exists()
        has_solution = (path / "solution.cpp").exists()

        is_current = False
        if current_content:
            for fname, exists in [("student.cpp", has_student), ("solution.cpp", has_solution)]:
                if exists and (path / fname).read_text() == current_content:
                    is_current = True
                    break

        marker = " >> " if is_current else "    "
        console.print(f"{marker}Lesson {num:02d}: {name}")
        console.print()

    console.print(f"  Total: {len(lessons)} lessons")
    console.print(f"  Switch: sf lesson switch <N>")

    return 0


def run_switch(args: argparse.Namespace) -> int:
    """Switch to a lesson"""
    identifier = _parse_identifier(args.identifier)
    lesson_dir = _find_lesson(identifier)

    if lesson_dir is None:
        console.error(f"Lesson '{args.identifier}' not found")
        manifest = _load_manifest()
        if manifest:
            ids = [f"{e['number']} ({e['id']})" for e in manifest if e.get("firmware_dir") and e["firmware_dir"] != "null"]
            console.print(f"  Available: {', '.join(ids)}")
        else:
            lessons = _discover_lessons()
            if lessons:
                nums = [str(n) for n, _, _ in lessons]
                console.print(f"  Available: {', '.join(nums)}")
        return 1

    # Determine source file
    if args.solution:
        src = lesson_dir / "solution.cpp"
        label = "solution"
    else:
        src = lesson_dir / "student.cpp"
        label = "student"

    if not src.exists():
        console.error(f"{label}.cpp not found for Lesson '{args.identifier}'")
        return 1

    dst = _get_user_code_path()

    # Copy file
    shutil.copy2(src, dst)

    # Clean build directory to ensure the new user_code.cpp is compiled
    build_dir = paths.workshop() / "build"
    if build_dir.exists():
        shutil.rmtree(build_dir, ignore_errors=True)
        console.info("Build directory cleaned")

    # Display lesson info from manifest
    num_display = args.identifier
    manifest = _load_manifest()
    if manifest:
        num = identifier if isinstance(identifier, int) else None
        for entry in manifest:
            if entry.get("number") == num or entry.get("id") == identifier:
                num_display = f"{entry['number']:02d} - {entry['title_ja']}"
                break

    console.success(f"Switched to Lesson {num_display} ({label})")
    console.print(f"  Source: {src}")
    console.print(f"  Target: {dst}")
    console.print()
    console.info("Next: sf lesson build && sf lesson flash")

    return 0


def run_solution(args: argparse.Namespace) -> int:
    """Show solution diff for a lesson"""
    identifier = _parse_identifier(args.identifier)
    lesson_dir = _find_lesson(identifier)

    if lesson_dir is None:
        console.error(f"Lesson '{args.identifier}' not found")
        return 1

    student = lesson_dir / "student.cpp"
    solution = lesson_dir / "solution.cpp"

    if not student.exists():
        console.error(f"student.cpp not found for Lesson '{args.identifier}'")
        return 1

    if not solution.exists():
        console.error(f"solution.cpp not found for Lesson '{args.identifier}'")
        return 1

    console.header(f"Lesson {args.identifier} Solution Diff")
    console.print()

    result = subprocess.run(
        ["diff", "-u", "--color=auto", str(student), str(solution)],
        capture_output=False,
    )

    if result.returncode == 0:
        console.info("student.cpp and solution.cpp are identical")

    return 0


def run_info(args: argparse.Namespace) -> int:
    """Show detailed lesson information"""
    identifier = _parse_identifier(args.identifier)
    manifest = _load_manifest()

    if not manifest:
        console.error("Manifest not found — cannot show lesson info")
        return 1

    # Find entry
    entry = None
    for e in manifest:
        if e.get("number") == identifier or e.get("id") == identifier:
            entry = e
            break

    if not entry:
        console.error(f"Lesson '{args.identifier}' not found in manifest")
        return 1

    num = entry["number"]
    console.header(f"Lesson {num}: {entry['title_ja']} / {entry['title_en']}")
    console.print()
    console.print(f"  ID:          {entry['id']}")
    console.print(f"  Description: {entry.get('description_ja', '-')}")
    console.print(f"               {entry.get('description_en', '-')}")
    console.print(f"  Slide:       chapters/{entry.get('slide_file', '-')}.tex")

    fw_dir = entry.get("firmware_dir")
    if fw_dir and fw_dir is not None:
        fw_path = _get_lessons_dir() / fw_dir
        console.print(f"  Firmware:    {fw_path}")
        has_student = (fw_path / "student.cpp").exists()
        has_solution = (fw_path / "solution.cpp").exists()
        console.print(f"  Files:       student={'yes' if has_student else 'no'}, solution={'yes' if has_solution else 'no'}")
    else:
        console.print(f"  Firmware:    (no firmware directory)")

    return 0


def _vscode_app_candidates() -> List[Tuple[str, List[str]]]:
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


def _find_editor(preferred: Optional[str] = None) -> Optional[Tuple[str, List[str]]]:
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
    for candidate in _vscode_app_candidates():
        return candidate

    # vi / vim fallback (POSIX, also if installed via Git for Windows)
    # vi / vim フォールバック（POSIX、Git for Windows 経由のインストールも検出）
    for editor in ("vi", "vim"):
        path = shutil.which(editor)
        if path:
            return (editor, [path])

    # Windows last resort: Notepad
    # Windows 最後の手段: Notepad
    if sys.platform == "win32":
        notepad = shutil.which("notepad")
        if notepad:
            return ("Notepad", [notepad])

    return None


def _editor_install_hint() -> List[str]:
    """Platform-specific install instructions for editors.
    プラットフォーム別のエディタインストール手順
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
    lines.append("  Or specify explicitly:  sf lesson edit --editor <command>")
    return lines


def run_edit(args: argparse.Namespace) -> int:
    """Open user_code.cpp in the editor"""
    if not _ensure_user_code_exists():
        user_code = _get_user_code_path()
        console.error(
            f"user_code.cpp not found and Lesson 0 seed is missing: {user_code}"
        )
        console.print()
        console.print("  Switch to a lesson first:")
        console.print("    sf lesson switch <N>")
        return 1

    user_code = _get_user_code_path()

    preferred = getattr(args, "editor", None)
    found = _find_editor(preferred)

    if found is None:
        if preferred:
            console.error(f"Editor not found: '{preferred}'")
        else:
            console.error("No editor found (tried: code, vi, vim)")
        console.print()
        for line in _editor_install_hint():
            console.print(line)
        return 1

    name, cmd = found
    reuse_window = bool(getattr(args, "reuse_window", False))

    # VSCode: open in a new window by default so user_code.cpp does not
    # piggyback into an unrelated workspace already open in VSCode.
    # VSCode: 既に開いている別プロジェクトのウィンドウに紛れないよう、デフォルトで新規ウィンドウ
    extra_flags: List[str] = []
    if name.startswith("VSCode") and not reuse_window:
        extra_flags = ["-n"]

    full_cmd = cmd + extra_flags + [str(user_code)]

    window_note = "" if not extra_flags else " (new window)"
    console.info(f"Opening user_code.cpp in {name}{window_note}")
    console.print(f"  Path: {user_code}")

    try:
        # shell=False is safe: command + path are passed as a list
        # shell=False で安全: コマンドとパスをリストで渡す
        result = subprocess.run(full_cmd)
        return result.returncode
    except FileNotFoundError:
        console.error(f"Failed to launch editor: {' '.join(full_cmd)}")
        return 1


def run_build(args: argparse.Namespace) -> int:
    """Build workshop firmware"""
    from . import build as build_cmd

    # Delegate through build.py's factory so newly added run() attributes
    # (e.g. a future --jobs default change) stay in sync automatically.
    # build.py のファクトリ経由で委譲する。run() に属性が増えても
    # (例: 将来の --jobs デフォルト変更) 自動的に追従する。
    build_args = build_cmd.make_run_args(
        target="workshop",
        clean=args.clean,
        verbose=args.verbose,
    )
    return build_cmd.run(build_args)


def run_flash(args: argparse.Namespace) -> int:
    """Flash workshop firmware"""
    from . import flash as flash_cmd

    # Delegate through flash.py's factory so newly added run() attributes
    # (e.g. --gui, added in 7022efc) stay in sync automatically instead of
    # causing an AttributeError at runtime.
    # flash.py のファクトリ経由で委譲する。run() に属性が増えても
    # (例: 7022efc で追加された --gui) 自動的に追従し、実行時の
    # AttributeError を防ぐ。
    flash_args = flash_cmd.make_run_args(
        target="workshop",
        port=args.port,
        baud=args.baud,
        monitor=not args.no_monitor,
    )
    return flash_cmd.run(flash_args)
