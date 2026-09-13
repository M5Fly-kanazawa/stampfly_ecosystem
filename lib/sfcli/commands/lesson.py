"""
sf lesson - Lesson management (実習管理)

Manages the tutorial lessons: list, switch, view solutions, build, flash,
monitor, run in SILS.
実習（レッスン）の管理: 一覧、切替、解答表示、ビルド、フラッシュ、
シリアルモニタ、SILS実行。

Subcommands:
    list      - List all available lessons
    switch    - Switch to a lesson (copy student.cpp to user_code.cpp)
    solution  - Show solution diff for a lesson
    info      - Show detailed lesson information
    edit      - Open user_code.cpp in editor (VSCode > vi)
    build     - Build the lesson firmware
    flash     - Flash the lesson firmware
    monitor   - Open the lesson firmware's serial monitor
    sils      - Run the lesson code in SILS (simulation)
"""

import argparse
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..utils import console, editor, paths

COMMAND_NAME = "lesson"
COMMAND_HELP = "Lesson management (実習管理)"

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
        # Content only (fresh mtime) — see run_switch for why not copy2.
        # 内容のみコピー（更新時刻は新しく）— 理由は run_switch を参照。
        shutil.copyfile(seed, user_code)
    except OSError:
        return False

    console.info("Bootstrapped user_code.cpp from Lesson 0 template")
    console.print(f"  Path: {user_code}")
    return True


def _load_manifest_data() -> Optional[Dict[str, Any]]:
    """Load the raw manifest YAML document (both `lessons:` and `courses:`).

    Kept separate from _load_manifest() so both the lesson list and the
    course list can share a single file read/parse instead of each
    re-reading the YAML file.
    マニフェストYAMLの生ドキュメント全体（`lessons:` と `courses:` の
    両方）を読み込む。_load_manifest() とは分離し、レッスン一覧と
    コース一覧がYAMLファイルの読み込み・パースを1回で共有できるようにする。

    Returns None if the file is missing, PyYAML is unavailable, or
    parsing fails.
    """
    manifest_path = _get_lessons_dir() / MANIFEST_FILE
    if not manifest_path.exists():
        return None
    try:
        import yaml
        with open(manifest_path, encoding="utf-8") as f:
            return yaml.safe_load(f)
    except ImportError:
        return None
    except Exception:
        return None


def _load_manifest() -> Optional[List[Dict[str, Any]]]:
    """Load lesson manifest YAML. Returns None if unavailable.

    Backward compatible: returns only the `lessons:` list, exactly as
    before this module gained `courses:` support.
    後方互換: このモジュールが `courses:` に対応する前と同じく、
    `lessons:` リストのみを返す。
    """
    data = _load_manifest_data()
    if data is None:
        return None
    return data.get("lessons", [])


def _load_courses() -> List[Dict[str, Any]]:
    """Load the top-level `courses:` list. Empty list if unavailable.

    トップレベルの `courses:` リストを読み込む。取得できない場合は
    空リストを返す。
    """
    data = _load_manifest_data()
    if data is None:
        return []
    return data.get("courses", [])


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


def _find_manifest_entry(identifier) -> Optional[Dict[str, Any]]:
    """Find the raw manifest entry for a lesson by number or ID.

    Mirrors _find_lesson()'s lookup rules but returns the manifest dict
    itself (needed for fields like has_solution) instead of resolving to
    a filesystem path.

    番号またはIDでマニフェストの生エントリを検索する。_find_lesson() と
    同じ検索規則だが、（has_solution等の判定に使うため）ファイルパスではなく
    マニフェスト辞書そのものを返す。

    Args:
        identifier: int (lesson number) or str (lesson ID)

    Returns:
        The manifest entry dict, or None if no manifest or no match.
    """
    manifest = _load_manifest()
    if not manifest:
        return None

    if isinstance(identifier, str):
        for entry in manifest:
            if entry.get("id") == identifier:
                return entry

    num = identifier if isinstance(identifier, int) else None
    if num is None:
        try:
            num = int(identifier)
        except (ValueError, TypeError):
            return None

    for entry in manifest:
        if entry.get("number") == num:
            return entry
    return None


def _find_course(course_id: str) -> Optional[Dict[str, Any]]:
    """Find a course entry by its id (e.g. 'sci2026').

    id（例: 'sci2026'）でコースエントリを検索する。
    """
    for course in _load_courses():
        if course.get("id") == course_id:
            return course
    return None


def _find_course_step(course: Dict[str, Any], number: int) -> Optional[Dict[str, Any]]:
    """Find a step within a course by its course-local step number.

    コース内の課の番号（コース固有の連番）でステップを検索する。
    """
    for step in course.get("steps", []):
        if step.get("number") == number:
            return step
    return None


def _resolve_course_identifier(raw: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """Resolve a `<course_id>:<N>` identifier to the underlying lesson id.

    `<course_id>:<N>` 形式の識別子を、実体のレッスンidへ解決する。

    Args:
        raw: the full identifier string, e.g. "sci2026:8".

    Returns:
        (lesson_id, course_display, error_message). On success,
        lesson_id/course_display are set and error_message is None. On
        failure, lesson_id/course_display are None and error_message
        describes the problem (already bilingual, ready for
        console.error()).
        成功時は (レッスンid, "sci2026:8" のような表示用文字列, None)。
        失敗時は (None, None, エラーメッセージ) — エラーメッセージは
        console.error() にそのまま渡せるバイリンガル文言。
    """
    course_id, _, number_str = raw.partition(":")

    course = _find_course(course_id)
    if course is None:
        available = ", ".join(c["id"] for c in _load_courses()) or "(none)"
        return None, None, (
            f"Unknown course '{course_id}'.\n"
            f"未知のコース '{course_id}' です。\n"
            f"  Available courses / 利用可能なコース: {available}"
        )

    try:
        number = int(number_str)
    except ValueError:
        return None, None, (
            f"Invalid course step '{number_str}' — must be an integer.\n"
            f"コースのステップ '{number_str}' は整数である必要があります。"
        )

    step = _find_course_step(course, number)
    if step is None:
        valid = ", ".join(str(s["number"]) for s in course.get("steps", [])) or "(none)"
        return None, None, (
            f"Unknown step {number} in course '{course_id}'.\n"
            f"コース '{course_id}' にステップ {number} はありません。\n"
            f"  Available steps / 利用可能なステップ: {valid}"
        )

    return step["lesson"], f"{course_id}:{number}", None


def _resolve_identifier_arg(raw: str) -> Optional[Tuple[Any, Optional[str]]]:
    """Resolve a CLI identifier argument, handling `<course>:<N>` syntax.

    CLIの識別子引数を解決する（`<course>:<N>` 構文に対応）。

    Plain identifiers (no ':') pass through _parse_identifier() unchanged,
    exactly like before course support existed. A `<course>:<N>` argument
    is resolved via the course's `steps:` list down to the underlying
    manifest lesson id, so every existing lookup (_find_lesson,
    _find_manifest_entry) keeps working unmodified.
    ':'を含まない識別子は、コース対応前と同じく _parse_identifier() を
    そのまま通す。`<course>:<N>` は該当コースの `steps:` を経由して
    実体のマニフェストレッスンidへ解決するため、既存の検索処理
    （_find_lesson, _find_manifest_entry）は無変更のまま動作する。

    Returns:
        (identifier, course_display) on success (course_display is None
        for a plain, non-course identifier). None if resolution failed —
        the error has already been printed via console.error().
    """
    if ":" not in raw:
        return _parse_identifier(raw), None

    lesson_id, course_display, error = _resolve_course_identifier(raw)
    if error is not None:
        console.error(error)
        return None

    return _parse_identifier(lesson_id), course_display


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
        description="Show all lessons with status.",
    )
    list_parser.add_argument(
        "--course",
        default=None,
        metavar="<id>",
        help="List steps for one course (e.g. sci2026) instead of all lessons",
    )
    list_parser.set_defaults(func=run_list)

    # Identifier help text shared by switch/solution/info: every one of
    # them also accepts the `<course_id>:<N>` course-step syntax.
    # switch/solution/info共通の識別子ヘルプ文言: いずれも
    # `<course_id>:<N>` というコースのステップ構文を受け付ける。
    identifier_help = (
        "Lesson number (e.g., 0, 5), ID (e.g., motor_control), "
        "or <course_id>:<N> (e.g., sci2026:8)"
    )

    # --- switch ---
    switch_parser = lesson_subparsers.add_parser(
        "switch",
        help="Switch to a lesson",
        description="Copy lesson student.cpp to user_code.cpp for building.",
    )
    switch_parser.add_argument("identifier", help=identifier_help)
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
    solution_parser.add_argument("identifier", help=identifier_help)
    solution_parser.set_defaults(func=run_solution)

    # --- info ---
    info_parser = lesson_subparsers.add_parser(
        "info",
        help="Show detailed lesson information",
        description="Display detailed information for a lesson.",
    )
    info_parser.add_argument("identifier", help=identifier_help)
    info_parser.set_defaults(func=run_info)

    # --- edit ---
    edit_parser = lesson_subparsers.add_parser(
        "edit",
        help="Open user_code.cpp in editor",
        description="Open the current lesson's user_code.cpp in your editor (VSCode > vi).",
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
        help="Build the lesson firmware",
        description="Build the lesson firmware.",
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
        help="Flash the lesson firmware",
        description="Flash the lesson firmware with monitor.",
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

    # --- monitor ---
    monitor_parser = lesson_subparsers.add_parser(
        "monitor",
        help="Open the lesson firmware's serial monitor",
        description="Open the serial monitor for the lesson firmware.",
    )
    monitor_parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (auto-detect if not specified)",
    )
    monitor_parser.add_argument(
        "-b", "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    monitor_parser.set_defaults(func=run_monitor)

    # --- sils ---
    # Local import so the sci2026-style "just run my code" flow can reach
    # sils.NOISE_LEVELS for --noise's choices without a module-level
    # dependency (sils.py is already imported eagerly by commands/__init__.py,
    # but keeping the import local mirrors run_build/run_flash's convention
    # of not committing to another command module's internals at import time).
    # ローカルimport: sci2026のような「自分のコードをとにかく動かす」導線が
    # --noise の choices に sils.NOISE_LEVELS を使えるようにする。sils.py は
    # commands/__init__.py が既に先行importしているが、run_build/run_flashの
    # 慣例（import時に他コマンドモジュールの内部へ結びつけない）に合わせ
    # ローカルimportのままにする。
    from . import sils as sils_cmd

    sils_parser = lesson_subparsers.add_parser(
        "sils",
        help="Run the lesson code in SILS (simulation)",
        description="Build the lesson firmware and run it against a SILS scenario.",
    )
    sils_parser.add_argument(
        "--solution",
        default=None,
        metavar="<course_id>:<N> | <N>",
        help="Switch to a lesson's solution.cpp before building (same identifier as 'sf lesson switch')",
    )
    sils_parser.add_argument(
        "--scenario",
        default="acro",
        metavar="acro|step|<path>",
        help="Scenario to run: 'acro' (default), 'step' (a step disturbance), or a path to a .scn file",
    )
    sils_parser.add_argument(
        "--noise",
        choices=sils_cmd.NOISE_LEVELS,
        default="off",
        help="Sensor noise level for the simulated plant (default: off)",
    )
    sils_parser.add_argument(
        "--seed",
        type=int,
        default=12345,
        help="Noise RNG seed (determinism)",
    )
    sils_parser.set_defaults(func=run_sils)

    # Default: show help
    parser.set_defaults(func=lambda args: (parser.print_help(), 0)[1])


def _parse_identifier(raw: str):
    """Parse identifier as int if possible, otherwise return as string."""
    try:
        return int(raw)
    except ValueError:
        return raw


def run_list(args: argparse.Namespace) -> int:
    """List all available lessons, or one course's steps with --course"""
    course_id = getattr(args, "course", None)
    if course_id:
        return _run_list_course(course_id)

    manifest = _load_manifest()
    if manifest:
        result = _run_list_manifest(manifest)
    else:
        result = _run_list_fallback()

    _print_courses_summary()
    return result


def _is_seed_content(content: str) -> bool:
    """True if `content` equals some lesson's student.cpp or solution.cpp
    (i.e. it carries no learner edits).
    `content` がどれかのレッスンの student.cpp / solution.cpp と一致する
    （＝学習者の編集が入っていない）なら True。
    """
    if not content:
        return True
    for _num, _name, lesson_dir in _discover_lessons():
        for fname in ("student.cpp", "solution.cpp"):
            fpath = lesson_dir / fname
            if fpath.exists() and fpath.read_text(encoding="utf-8") == content:
                return True
    return False


def _backup_edited_user_code(user_code: Path) -> Optional[Path]:
    """Before `sf lesson switch` overwrites user_code.cpp, save a copy of it
    under firmware/workshop/my_code/ if it contains learner edits (its
    content matches no lesson seed). Returns the backup path, or None.
    `sf lesson switch` が user_code.cpp を上書きする前に、学習者の編集が
    入っていれば（どのレッスンの雛形とも一致しなければ）
    firmware/workshop/my_code/ に退避する。退避先のパス、無ければ None。
    """
    if not user_code.exists():
        return None
    content = user_code.read_text(encoding="utf-8")
    if _is_seed_content(content):
        return None
    from datetime import datetime
    backup_dir = paths.workshop() / "my_code"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = backup_dir / f"user_code_{stamp}.cpp"
    shutil.copyfile(user_code, backup)
    return backup


def _current_user_code() -> str:
    """Read user_code.cpp's current content, or "" if not switched yet.

    user_code.cppの現在の内容を読む。まだ切り替えられていなければ""。
    """
    user_code_path = _get_user_code_path()
    if user_code_path.exists():
        return user_code_path.read_text(encoding="utf-8")
    return ""


def _is_lesson_current(entry: Dict[str, Any], current_content: str) -> bool:
    """Check whether a manifest lesson's student/solution.cpp is the one
    currently copied into user_code.cpp.

    マニフェストのレッスンのstudent/solution.cppが、現在user_code.cppに
    コピーされている内容と一致するか判定する。
    """
    if not current_content:
        return False
    fw_dir = entry.get("firmware_dir")
    if not fw_dir:
        return False
    lesson_path = _get_lessons_dir() / fw_dir
    if not lesson_path.exists():
        return False
    for fname in ("student.cpp", "solution.cpp"):
        fpath = lesson_path / fname
        if fpath.exists() and fpath.read_text(encoding="utf-8") == current_content:
            return True
    return False


def _run_list_manifest(manifest: List[Dict[str, Any]]) -> int:
    """List lessons from manifest with day grouping"""
    current_content = _current_user_code()

    # Build number->entry lookup
    by_number = {e["number"]: e for e in manifest}

    console.header("Lessons")
    console.print()

    for day, numbers in DAY_GROUPS:
        console.print(f"  Day {day}:")
        for num in numbers:
            entry = by_number.get(num)
            if not entry:
                continue

            marker = " >> " if _is_lesson_current(entry, current_content) else "    "
            title_ja = entry.get("title_ja", "")
            title_en = entry.get("title_en", "")
            desc_ja = entry.get("description_ja", "")

            console.print(f"{marker}Lesson {num:2d}: {title_ja} / {title_en}")
            if desc_ja:
                console.print(f"              {desc_ja}")
        console.print()

    _print_event_lessons(manifest, current_content)

    console.print(f"  Total: {len(manifest)} lessons")
    console.print(f"  Switch: sf lesson switch <N or id>")

    return 0


def _print_event_lessons(manifest: List[Dict[str, Any]], current_content: str) -> None:
    """Print the "Events" section (number >= 90, outside Day 1-5).

    「Events」節（90番以降、Day 1〜5の通常レッスンとは別系統）を表示する。
    """
    day_numbers = {n for _, numbers in DAY_GROUPS for n in numbers}
    event_entries = [e for e in manifest if e["number"] not in day_numbers]
    if not event_entries:
        return

    console.print("  Events（教育イベント）:")
    for entry in sorted(event_entries, key=lambda e: e["number"]):
        marker = " >> " if _is_lesson_current(entry, current_content) else "    "
        title_ja = entry.get("title_ja", "")
        title_en = entry.get("title_en", "")
        desc_ja = entry.get("description_ja", "")
        console.print(
            f"{marker}Lesson {entry['number']:2d}: {title_ja} / {title_en}"
            f"  (id: {entry.get('id', '')})"
        )
        if desc_ja:
            console.print(f"              {desc_ja}")

    # Show the next free event number so new tutorials know where to start
    # 次イベントの空き番号を表示（新規チュートリアル追加時の起点が一目で分かる）
    next_event = max(e["number"] for e in event_entries) + 1
    console.print(
        f"    (next event number / 次のイベント番号: {next_event} — "
        f"lesson_{next_event}_<name> + manifest 追記で追加)"
    )
    console.print()


def _run_list_course(course_id: str) -> int:
    """List one course's steps under its own tutorial-local numbering.

    コース1件分のステップを、チュートリアル固有の番号で一覧表示する。
    """
    course = _find_course(course_id)
    if course is None:
        available = ", ".join(c["id"] for c in _load_courses()) or "(none)"
        console.error(f"Unknown course '{course_id}'")
        console.print(f"  Available courses / 利用可能なコース: {available}")
        return 1

    manifest = _load_manifest() or []
    by_id = {e["id"]: e for e in manifest}
    current_content = _current_user_code()

    title_ja = course.get("title_ja", course_id)
    title_en = course.get("title_en", course_id)
    console.header(f"{title_ja} / {title_en}")
    console.print()

    for step in course.get("steps", []):
        lesson_entry = by_id.get(step["lesson"])
        step_title_ja = lesson_entry.get("title_ja", "") if lesson_entry else step["lesson"]
        step_title_en = lesson_entry.get("title_en", "") if lesson_entry else ""

        is_current = bool(lesson_entry) and _is_lesson_current(lesson_entry, current_content)
        marker = " >> " if is_current else "    "

        # Speak only in the course's own step numbering / session (S2, S3,
        # ...) — never the underlying Workshop lesson number, so sci2026
        # participants never need to know Workshop exists.
        # コース自身のステップ番号・セッション(S2, S3, ...)のみで話す —
        # 実体のWorkshopレッスン番号は出さない。sci2026参加者がWorkshopの
        # 存在を知る必要をなくすため。
        console.print(
            f"{marker}実習 {step['number']}: {step_title_ja} / {step_title_en}"
            f"  ({step.get('session', '-')})"
        )

    console.print()
    console.print(f"  Switch: sf lesson switch {course_id}:<N>")

    return 0


def _print_courses_summary() -> None:
    """Append a short "Courses:" section to `sf lesson list`'s output.

    `sf lesson list` の出力末尾に短い「Courses:」節を付け加える。

    Silently does nothing if no courses are declared, so callers can
    invoke this unconditionally after either listing path.
    コースが1件も宣言されていなければ何もしない。呼び出し側はどちらの
    一覧表示の後でも無条件に呼び出せる。
    """
    courses = _load_courses()
    if not courses:
        return

    console.print("  Courses:")
    for course in courses:
        title_ja = course.get("title_ja", "")
        title_en = course.get("title_en", "")
        console.print(f"    {course['id']}: {title_ja} / {title_en}")
    console.print(f"    Detail: sf lesson list --course <id>")
    console.print()


def _run_list_fallback() -> int:
    """List lessons by directory scan (fallback when no manifest)"""
    lessons = _discover_lessons()

    if not lessons:
        console.error("No lessons found")
        console.print(f"  Expected at: {_get_lessons_dir()}")
        return 1

    current_content = _current_user_code()

    console.header("Lessons")
    console.print()

    for num, name, path in lessons:
        has_student = (path / "student.cpp").exists()
        has_solution = (path / "solution.cpp").exists()

        is_current = False
        if current_content:
            for fname, exists in [("student.cpp", has_student), ("solution.cpp", has_solution)]:
                if exists and (path / fname).read_text(encoding="utf-8") == current_content:
                    is_current = True
                    break

        marker = " >> " if is_current else "    "
        console.print(f"{marker}Lesson {num:02d}: {name}")
        console.print()

    console.print(f"  Total: {len(lessons)} lessons")
    console.print(f"  Switch: sf lesson switch <N>")

    return 0


def _resolve_switch_source(
    lesson_dir: Path, identifier: Any, raw_identifier: str, use_solution: bool
) -> Optional[Tuple[Path, str]]:
    """Pick student.cpp or solution.cpp as the `switch` copy source.

    switchのコピー元としてstudent.cppかsolution.cppを選ぶ。

    Returns (path, label) on success. Returns None after printing an
    error (via console.error()) if the lesson has no solution by design
    (has_solution: false) or the chosen file is simply missing.
    成功時は (パス, ラベル)。設計上solutionが存在しない
    （has_solution: false）場合、または該当ファイルが単に無い場合は
    console.error()でエラー出力の上Noneを返す。
    """
    if not use_solution:
        src = lesson_dir / "student.cpp"
        label = "student"
    else:
        # Some lessons (event templates like Lesson 90) declare
        # has_solution: false because no single "correct answer" exists
        # by design. Fail with a clear reason instead of the generic
        # "solution.cpp not found" below.
        # 一部のレッスン（Lesson 90のようなイベント用テンプレート）は設計上
        # 唯一の「正解」が存在しないため has_solution: false を宣言している。
        # 下の汎用的な「solution.cpp not found」ではなく、理由を明示して失敗させる。
        entry = _find_manifest_entry(identifier)
        if entry is not None and not entry.get("has_solution", True):
            console.error(
                f"Lesson '{raw_identifier}' has no solution — "
                "it is an event template with no single correct answer by design.\n"
                f"Lesson '{raw_identifier}' にsolutionはありません — "
                "設計上、唯一の正解が存在しないイベント用テンプレートです。"
            )
            return None
        src = lesson_dir / "solution.cpp"
        label = "solution"

    if not src.exists():
        console.error(f"{label}.cpp not found for Lesson '{raw_identifier}'")
        return None

    return src, label


def run_switch(args: argparse.Namespace) -> int:
    """Switch to a lesson"""
    resolved = _resolve_identifier_arg(args.identifier)
    if resolved is None:
        return 1
    identifier, course_display = resolved
    lesson_dir = _find_lesson(identifier)

    if lesson_dir is None:
        if course_display:
            # Course-based lookup already failed above (unknown course/step)
            # with its own bilingual error if the identifier itself was bad;
            # reaching here means the manifest points at a firmware dir that
            # doesn't exist. Speak only in the course's own step number —
            # never dump the underlying Workshop lesson numbers below.
            # コース経由の検索は識別子自体が不正なら上で既にエラー済み —
            # ここに来るのはマニフェストが存在しないファームウェアディレクトリを
            # 指している場合。コース自身のステップ番号のみで話し、下記の
            # 実体Workshopレッスン番号一覧は出さない。
            console.error(f"実習 '{course_display}' not found (firmware missing)")
        else:
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

    source = _resolve_switch_source(lesson_dir, identifier, args.identifier, args.solution)
    if source is None:
        return 1
    src, label = source

    dst = _get_user_code_path()

    # Copy the file CONTENT only (shutil.copyfile), not its metadata:
    # shutil.copy2 would carry the source's mtime over, which can be older
    # than the last build's object files, so `sf build workshop` / `sf sils
    # build --target workshop` would silently skip recompiling the switched
    # code. A fresh mtime means no `touch` is ever needed after switching.
    # ファイルの内容だけをコピーする（shutil.copyfile）。shutil.copy2 はコピー元
    # の更新時刻を引き継ぐため、直前ビルドのオブジェクトより古くなり、
    # `sf build workshop` / `sf sils build --target workshop` が切替後のコードを
    # 再コンパイルせずに済ませてしまうことがあった。更新時刻が新しくなるので、
    # 切替後に `touch` は一切不要。
    # Keep the learner's edits: an edited user_code.cpp is saved to
    # firmware/workshop/my_code/ before it is overwritten.
    # 学習者の編集を守る: 編集済みの user_code.cpp は上書き前に
    # firmware/workshop/my_code/ に退避する。
    backup = _backup_edited_user_code(dst)
    if backup is not None:
        console.warning(f"Edited user_code.cpp saved to {backup.relative_to(paths.root())}")
        console.print("  編集済みの user_code.cpp を上書き前に退避しました")
    shutil.copyfile(src, dst)

    # Clean build directory to ensure the new user_code.cpp is compiled
    build_dir = paths.workshop() / "build"
    if build_dir.exists():
        shutil.rmtree(build_dir, ignore_errors=True)
        console.info("Build directory cleaned")

    # Display lesson info from manifest
    num_display = args.identifier
    title_ja = None
    manifest = _load_manifest()
    if manifest:
        num = identifier if isinstance(identifier, int) else None
        for entry in manifest:
            if entry.get("number") == num or entry.get("id") == identifier:
                title_ja = entry.get("title_ja", "")
                num_display = f"{entry['number']:02d} - {title_ja}"
                break

    if course_display:
        # Course-based switch: speak only in the course's own step number
        # (e.g. "sci2026:8" -> 実習 8) that participants already see from
        # `sf lesson list --course`; never expose the underlying Workshop
        # lesson number.
        # コース経由の切替: 参加者が `sf lesson list --course` で既に見て
        # いるコース自身のステップ番号（例: "sci2026:8" -> 実習 8）のみで
        # 話す。実体のWorkshopレッスン番号は出さない。
        step_number = course_display.rpartition(":")[2]
        console.success(f"Switched to 実習 {step_number}: {title_ja} ({label})")
    else:
        console.success(f"Switched to Lesson {num_display} ({label})")

    console.print(f"  Source: {src}")
    console.print(f"  Target: {dst}")
    console.print()
    console.info("Next: sf lesson build && sf lesson flash")

    return 0


def run_solution(args: argparse.Namespace) -> int:
    """Show solution diff for a lesson"""
    resolved = _resolve_identifier_arg(args.identifier)
    if resolved is None:
        return 1
    identifier, course_display = resolved
    lesson_dir = _find_lesson(identifier)

    if lesson_dir is None:
        console.error(f"Lesson '{args.identifier}' not found")
        return 1

    # Some lessons (event templates like Lesson 90) declare
    # has_solution: false because no single "correct answer" exists by
    # design. Fail with a clear reason instead of the generic
    # "solution.cpp not found" below.
    # 一部のレッスン（Lesson 90のようなイベント用テンプレート）は設計上
    # 唯一の「正解」が存在しないため has_solution: false を宣言している。
    # 下の汎用的な「solution.cpp not found」ではなく、理由を明示して失敗させる。
    entry = _find_manifest_entry(identifier)
    if entry is not None and not entry.get("has_solution", True):
        console.error(
            f"Lesson '{args.identifier}' has no solution — "
            "it is an event template with no single correct answer by design.\n"
            f"Lesson '{args.identifier}' にsolutionはありません — "
            "設計上、唯一の正解が存在しないイベント用テンプレートです。"
        )
        return 1

    student = lesson_dir / "student.cpp"
    solution = lesson_dir / "solution.cpp"

    if not student.exists():
        console.error(f"student.cpp not found for Lesson '{args.identifier}'")
        return 1

    if not solution.exists():
        console.error(f"solution.cpp not found for Lesson '{args.identifier}'")
        return 1

    header_suffix = f"  [{course_display}]" if course_display else ""
    console.header(f"Lesson {args.identifier} Solution Diff{header_suffix}")
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
    resolved = _resolve_identifier_arg(args.identifier)
    if resolved is None:
        return 1
    identifier, course_display = resolved
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
    header_suffix = f"  [{course_display}]" if course_display else ""
    console.header(f"Lesson {num}: {entry['title_ja']} / {entry['title_en']}{header_suffix}")
    console.print()
    console.print(f"  ID:          {entry['id']}")
    console.print(f"  Description: {entry.get('description_ja', '-')}")
    console.print(f"               {entry.get('description_en', '-')}")
    # Event templates have no slide chapter — show "none" instead of a bogus path
    # イベント雛形にはスライド章がないため、偽パスでなく「なし」を表示する
    slide_file = entry.get("slide_file")
    if slide_file:
        console.print(f"  Slide:       chapters/{slide_file}.tex")
    else:
        console.print("  Slide:       (none)")

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


# Editor detection (find_editor, install_hint, ...) lives in
# ../utils/editor.py, shared with `sf app edit`. It originated here; see
# that module's docstring for why it moved.
# エディタ検出（find_editor, install_hint 等）は `sf app edit` と共有する
# ../utils/editor.py にある。元々はここにあった — 移動の理由はそのモジュールの
# docstring を参照。


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
    found = editor.find_editor(preferred)

    if found is None:
        if preferred:
            console.error(f"Editor not found: '{preferred}'")
        else:
            console.error("No editor found (tried: code, vi, vim)")
        console.print()
        for line in editor.install_hint("sf lesson edit --editor <command>"):
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


def run_monitor(args: argparse.Namespace) -> int:
    """Open the lesson firmware's serial monitor (= sf monitor workshop)"""
    from . import monitor as monitor_cmd

    # monitor.py has no make_run_args() factory (unlike build.py/flash.py),
    # so the Namespace is hand-built here — same pattern sils.py's
    # run_sysid_gate() uses to call run_scenario() directly.
    # monitor.py には（build.py/flash.pyと違い）make_run_args()が無いため、
    # ここでNamespaceを手組みする — sils.pyのrun_sysid_gate()が
    # run_scenario()を直接呼ぶのと同じパターン。
    monitor_args = argparse.Namespace(
        target="workshop",
        port=args.port,
        baud=args.baud,
    )
    return monitor_cmd.run(monitor_args)


def _resolve_sils_scenario(raw: str) -> Optional[Path]:
    """Resolve `sf lesson sils --scenario`'s shorthand or a path to a .scn file.

    `sf lesson sils --scenario` の短縮形、またはパスを .scn ファイルへ解決する。

    'acro' and 'step' are shorthands for the two scenarios used across the
    sci2026 slides (workshop_acro.scn / workshop_acro_step.scn); anything
    else is treated as a path, so an instructor can point at any other
    scenario under simulator/sils/scenarios/ without lesson.py knowing
    about it by name.
    'acro'/'step' はsci2026スライド全体で使う2本のシナリオ
    （workshop_acro.scn / workshop_acro_step.scn）の短縮形。それ以外は
    パスとして扱うため、講師は lesson.py がその名前の情報を持っていなくても
    simulator/sils/scenarios/ 配下の他のシナリオを指定できる。
    """
    shortcuts = {
        "acro": "workshop_acro.scn",
        "step": "workshop_acro_step.scn",
    }
    scenarios_dir = paths.root() / "simulator" / "sils" / "scenarios"
    filename = shortcuts.get(raw)
    scn_path = scenarios_dir / filename if filename else Path(raw)

    if not scn_path.exists():
        console.error(f"Scenario not found: {scn_path}")
        return None
    return scn_path


def run_sils(args: argparse.Namespace) -> int:
    """Build the lesson code and run it against a SILS scenario, without
    the caller needing to know the underlying Workshop firmware/target name.
    実体のWorkshopファームウェア/ターゲット名の情報を呼び出し側が持っていなくても、
    実習コードをビルドしSILSシナリオで実行する。
    """
    from . import sils as sils_cmd

    console.info("実習コード（user_code.cpp）を SILS（シミュレーション）で飛ばします")

    if args.solution:
        switch_args = argparse.Namespace(identifier=args.solution, solution=True)
        result = run_switch(switch_args)
        if result != 0:
            return result

    scn_path = _resolve_sils_scenario(args.scenario)
    if scn_path is None:
        return 1

    # `sf lesson switch` copies content only, so user_code.cpp always carries
    # a fresh mtime and the SILS build below recompiles it — no `touch` needed.
    # `sf lesson switch` は内容だけをコピーするため user_code.cpp の更新時刻は
    # 常に新しく、下の SILS ビルドが再コンパイルする — `touch` は不要。
    _ensure_user_code_exists()

    build_args = argparse.Namespace(
        jobs=8, target="workshop",
        yes=False, no_auto_toolchain=False, winget=False,
    )
    build_result = sils_cmd.run_build(build_args)
    if build_result != 0:
        return build_result

    scenario_args = argparse.Namespace(
        scenario=str(scn_path), target="workshop", expect=None,
        duration=25_000_000, noise=args.noise, seed=args.seed,
        video=False, ground_effect=None, turbulence=None,
        motor_delay=None, thrust_eff=None, torque_authority=None,
        flow_scale=None, unpaired=False, params=None,
    )
    return sils_cmd.run_scenario(scenario_args)
