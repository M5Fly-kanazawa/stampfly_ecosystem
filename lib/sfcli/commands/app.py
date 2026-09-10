"""
sf app - Manage your own drone-firmware projects

Create, edit, build, and flash your own StampFly firmware project. A new
project is cloned from a `firmware/vehicle/examples/<N>` example (e.g. the
Topic API read-only example, or the custom-controller example) into
`firmware/apps/<name>`, so a research user gets exactly the same
new -> edit -> build -> flash flow that workshop participants already know
from `sf lesson`.

自分のドローンファームウェアプロジェクトを作成・編集・ビルド・書き込みする。
新規プロジェクトは `firmware/vehicle/examples/<N>` の例題（Topic API 読取専用
例題、自作コントローラ例題 等）を `firmware/apps/<name>` に複製して作る。
研究利用者にも `sf lesson` でワークショップ参加者が既に知っている
new -> edit -> build -> flash と同じ流れを提供する。

Subcommands:
    new     - Create a new project cloned from an example
    edit    - Open the learner-facing source file in an editor
    build   - Build a project (= sf build apps/<name>)
    flash   - Flash a project (= sf flash apps/<name>)
    sils    - Run an embedded-type app's SILS scenario
    list    - List your projects and the examples available for --from
"""

import argparse
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..utils import console, editor, espidf, paths, platform

COMMAND_NAME = "app"
COMMAND_HELP = "Manage your own drone-firmware projects"

# Reserved names that cannot be used for an app project. Projects actually
# live one level deeper (firmware/apps/<name>), so none of these would
# literally collide on disk -- they are rejected anyway because reusing a
# top-level firmware/ directory name here would be confusing to read.
# アプリのプロジェクト名として使えない予約名。実際には1階層下の
# firmware/apps/<name> に置かれるためディスク上で衝突はしないが、
# firmware/ 直下の既存ディレクトリ名を流用すると紛らわしいため禁止する。
RESERVED_NAMES = {"vehicle", "vehicle_old", "controller", "workshop", "common", "apps"}

# Default --from example: the L1 (Topic API) embedded-type template. It builds
# INTO vehicle's own main component (see app.yaml's `type: embedded`) so the
# same source runs on both `sf app build` (real hardware) and `sf app sils`
# (SILS) -- see docs/plans/sf-app-sils-plan.md Phase 2. 10_custom_controller
# (a standalone bench, no SILS) remains available via --from for anyone who
# wants the older single-exercise, no-hardware-needed flow.
# --from 省略時の既定値: L1（Topic API）組み込み型テンプレート。vehicle 本体の
# main コンポーネントに直接組み込まれる（app.yaml の `type: embedded`）ため、
# 同一ソースが `sf app build`（実機）と `sf app sils`（SILS）の両方で動く。
# 10_custom_controller（実機不要の単独ベンチ、SILS 不可）は --from で
# 引き続き利用できる。
DEFAULT_EXAMPLE = "11_app_controller"

# The app.yaml manifest file every project carries (see load_app_manifest()).
# 全プロジェクトが持つ app.yaml マニフェストファイル名（load_app_manifest() 参照）。
APP_MANIFEST_FILE = "app.yaml"

# Manifest used when a project has no app.yaml at all (09/10 and any other
# pre-L1 example clone) -- treated as a standalone bench project that cannot
# run in the SILS vehicle emulator (see sf-app-sils-plan.md §2 "例題 09 / 10
# の扱い").
# app.yaml を一切持たないプロジェクト（09/10 等の L1 以前の例題の複製）に
# 使う既定マニフェスト -- SILS の vehicle エミュレータでは動かせない独立
# ベンチ型として扱う。
_DEFAULT_MANIFEST: Dict[str, Any] = {"type": "bench", "sils": False}

# Never copy these from an example into a new project -- build output and
# dependency-manager state, which must be regenerated fresh for the new
# project (they cache paths/names tied to the example's own project name).
# 例題から新規プロジェクトへコピーしない物 -- ビルド成果物と依存関係
# マネージャの状態。例題自身のプロジェクト名に紐づくキャッシュのため、
# 新しいプロジェクト用に作り直させる。
_COPY_EXCLUDE = ("build", "sdkconfig", "managed_components", "dependencies.lock")

# The file a learner is expected to write, checked in this priority order
# inside a project's main/ directory. 10_custom_controller (and any project
# cloned from it) separates the exercise into learner_controller.cpp; older
# single-file examples (e.g. 09_topic_api_hello) only have main.cpp.
# 学習者が書くことを想定したファイル。プロジェクトの main/ 内でこの優先順位
# で確認する。10_custom_controller（から複製したプロジェクト）は演習部分を
# learner_controller.cpp に分離しているが、単一ファイル構成の例題
# （例: 09_topic_api_hello）は main.cpp のみを持つ。
_LEARNER_FILE_PRIORITY = ("learner_controller.cpp", "main.cpp")

# The file to open for an embedded-type project (11/12 and clones thereof),
# checked in this priority order directly under the project directory (these
# templates have no main/ subdirectory -- they have no CMakeLists.txt of
# their own either, see run_new()). app_controller.cpp holds the
# IController hook (11_app_controller); app.cpp is every embedded template's
# sf::app::start()/controller()/estimator() hook file (12_app_task_hello has
# only this one) -- see firmware/vehicle/main/app_hooks.hpp.
# 組み込み型プロジェクト（11/12 とその複製）を開く際の優先順位。これらの
# テンプレートは main/ サブディレクトリを持たない（自身の CMakeLists.txt も
# 持たない -- run_new() 参照）。app_controller.cpp は IController フック
# （11_app_controller）、app.cpp は全組み込みテンプレート共通のフック
# ファイル（12_app_task_hello はこれのみ）。
_EMBEDDED_LEARNER_FILE_PRIORITY = ("app_controller.cpp", "app.cpp")

# Default scenario for `sf app sils <name>` when none is given.
# `sf app sils <name>` でシナリオ省略時の既定値。
DEFAULT_SILS_SCENARIO = Path("simulator") / "sils" / "scenarios" / "alt_flight.scn"

# Binary produced by an embedded app's vehicle-firmware build (project name
# from firmware/vehicle/CMakeLists.txt's project()).
# 組み込み型 app の実機ビルドが生成するバイナリ名（project() 名に由来）。
_VEHICLE_BINARY_NAME = "stampfly_vehicle.bin"


# =============================================================================
# app.yaml manifest
# =============================================================================

def load_app_manifest(app_dir: Path) -> Dict[str, Any]:
    """Load `app_dir`/app.yaml. Returns a copy of _DEFAULT_MANIFEST (bench,
    no SILS) if the file is missing, PyYAML is unavailable, or the document
    fails to parse or is not a mapping -- matching lesson.py's manifest-
    loading style (see lib/sfcli/commands/lesson.py's _load_manifest_data),
    so a pre-L1 clone (09/10) with no app.yaml at all is classified the same
    as a bench template that never got one.
    `app_dir`/app.yaml を読み込む。ファイルが無い・PyYAML が使えない・
    パース失敗・トップレベルが辞書でない場合は _DEFAULT_MANIFEST
    （bench・SILS 不可）のコピーを返す -- app.yaml を一切持たない L1 以前の
    複製（09/10）を、未設定のベンチテンプレートと同じに分類する。
    """
    manifest_path = app_dir / APP_MANIFEST_FILE
    if not manifest_path.exists():
        return dict(_DEFAULT_MANIFEST)
    try:
        import yaml
        with open(manifest_path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else dict(_DEFAULT_MANIFEST)
    except ImportError:
        return dict(_DEFAULT_MANIFEST)
    except Exception:
        return dict(_DEFAULT_MANIFEST)


def write_app_manifest(app_dir: Path, data: Dict[str, Any]) -> None:
    """Write `app_dir`/app.yaml, overwriting any existing one. Field order in
    `data` is preserved (sort_keys=False) since a small, hand-edited
    manifest reads better in a deliberate order than an alphabetical one.
    `app_dir`/app.yaml を書き込む（既存ファイルは上書き）。`data` のフィールド
    順序を保持する（sort_keys=False）-- 少数項目の手編集向けマニフェストは
    アルファベット順より意図した順序の方が読みやすいため。
    """
    import yaml
    manifest_path = app_dir / APP_MANIFEST_FILE
    with open(manifest_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _project_dir_valid(d: Path) -> bool:
    """A project/example directory is either a standalone ESP-IDF project
    (bench type, has its own CMakeLists.txt) or an embedded-type L1
    template/app (no CMakeLists.txt of its own -- compiled into vehicle's
    main component instead, identified by app.yaml; see run_new()).
    プロジェクト/例題ディレクトリは、独立した ESP-IDF プロジェクト（ベンチ型、
    自身の CMakeLists.txt を持つ）か、組み込み型 L1 テンプレート/app
    （自身の CMakeLists.txt を持たず app.yaml で識別。run_new() 参照）の
    どちらか。
    """
    return d.is_dir() and ((d / "CMakeLists.txt").exists() or (d / APP_MANIFEST_FILE).exists())


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    app_subparsers = parser.add_subparsers(
        dest="app_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- new ---
    new_parser = app_subparsers.add_parser(
        "new",
        help="Create a new project cloned from an example",
        description="Copy a firmware/vehicle/examples/<N> example into firmware/apps/<name>.",
    )
    new_parser.add_argument("name", help="Project name (e.g. my_ctrl); must be a valid C identifier")
    new_parser.add_argument(
        "--from",
        dest="from_example",
        default=DEFAULT_EXAMPLE,
        metavar="<example>",
        help=f"Example directory under firmware/vehicle/examples/ to clone (default: {DEFAULT_EXAMPLE})",
    )
    new_parser.set_defaults(func=run_new)

    # --- edit ---
    edit_parser = app_subparsers.add_parser(
        "edit",
        help="Open the learner-facing source file in an editor",
        description="Open main/learner_controller.cpp (or main/main.cpp) in your editor (VSCode > vi).",
    )
    edit_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
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
    build_parser = app_subparsers.add_parser(
        "build",
        help="Build a project",
        description="Build a project (equivalent to 'sf build apps/<name>').",
    )
    build_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
    build_parser.add_argument(
        "-c", "--clean",
        action="store_true",
        help="Clean build (fullclean before build)",
    )
    build_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose build output",
    )
    build_parser.set_defaults(func=run_build)

    # --- flash ---
    flash_parser = app_subparsers.add_parser(
        "flash",
        help="Flash a project",
        description="Flash a project (equivalent to 'sf flash apps/<name>').",
    )
    flash_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
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
        "-m", "--monitor",
        action="store_true",
        help="Start monitor after flashing",
    )
    flash_parser.set_defaults(func=run_flash)

    # --- sils ---
    sils_parser = app_subparsers.add_parser(
        "sils",
        help="Run an embedded-type project's SILS scenario",
        description=(
            "Build and run an embedded-type app's Software-in-the-Loop "
            "(host-built firmware physics test) scenario -- equivalent to "
            "`sf sils scenario <scenario> --target apps/<name>`. Bench-type "
            "projects (--from 10_custom_controller and older) cannot run "
            "here; see the error message this prints for those."
        ),
    )
    sils_parser.add_argument("name", help="Project name (e.g. my_ctrl)")
    sils_parser.add_argument(
        "scenario",
        nargs="?",
        default=None,
        help=f"path to a .scn scenario file (default: {DEFAULT_SILS_SCENARIO})",
    )
    sils_parser.add_argument(
        "--expect",
        default=None,
        help="assertions file (default: <scenario>.expect if it exists)",
    )
    sils_parser.add_argument(
        "--noise",
        choices=["off", "n0", "n1", "n2"],
        default="off",
        help="sensor noise level on the emulator Plant (default off)",
    )
    sils_parser.add_argument(
        "--seed",
        type=int,
        default=12345,
        help="noise RNG seed (determinism: same seed -> byte-identical run)",
    )
    sils_parser.add_argument(
        "--duration",
        type=int,
        default=25_000_000,
        help="sim duration in microseconds (default 25 s)",
    )
    sils_parser.set_defaults(func=run_app_sils)

    # --- list ---
    list_parser = app_subparsers.add_parser(
        "list",
        help="List your projects and the examples available for --from",
        description="Show firmware/apps/ projects and firmware/vehicle/examples/ sources for --from.",
    )
    list_parser.set_defaults(func=run_list)

    # Default: show help
    parser.set_defaults(func=lambda args: (parser.print_help(), 0)[1])


# =============================================================================
# new
# =============================================================================

def _validate_new_name(name: str) -> Optional[str]:
    """Validate a new project name. Returns an error message, or None if OK."""
    if not name.isidentifier():
        return f"Invalid project name: '{name}' (must be a valid C identifier)"

    if name in RESERVED_NAMES:
        return f"Reserved name: '{name}' (cannot use {', '.join(sorted(RESERVED_NAMES))})"

    project_dir = paths.apps() / name
    if project_dir.exists():
        return f"Project already exists: {project_dir}"

    return None


def _examples_dir() -> Path:
    """Get firmware/vehicle/examples/ directory."""
    return paths.vehicle() / "examples"


def _example_summary(example_dir: Path) -> str:
    """First Markdown heading (`# ...`) of an example's README.md, or "" if
    there is no README or no such heading.
    例題の README.md にある最初の Markdown 見出し（`# ...`）。README が無い、
    または見出しが見つからない場合は ""。

    Every example's heading follows "# <dirname> — <description>"
    (verified above); that leading "<dirname> — " is stripped here since
    the caller already prints the directory name right before this summary.
    全例題の見出しは "# <dirname> — <description>" 形式（上記で確認済み）。
    呼び出し側は既にディレクトリ名をこの要約の直前に表示するため、先頭の
    "<dirname> — " はここで取り除く。
    """
    readme = example_dir / "README.md"
    if not readme.exists():
        return ""
    for line in readme.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("# "):
            heading = stripped[2:].strip()
            for separator in (" — ", " - "):
                prefix = f"{example_dir.name}{separator}"
                if heading.startswith(prefix):
                    return heading[len(prefix):]
            return heading
    return ""


def _print_available_examples(examples_dir: Path) -> None:
    """Print every example directory (with its README's first heading, if any)."""
    if not examples_dir.exists():
        console.print("  (no examples found)")
        return

    found = False
    for d in sorted(examples_dir.iterdir()):
        if _project_dir_valid(d):
            summary = _example_summary(d)
            suffix = f" -- {summary}" if summary else ""
            console.print(f"  {d.name}{suffix}")
            found = True

    if not found:
        console.print("  (no examples found)")


def _rewrite_cmake_project(project_dir: Path, name: str) -> None:
    """Point the copied top-level CMakeLists.txt at vehicle/components and
    rename its project() to the new app name.

    複製したトップレベル CMakeLists.txt の部品探索先を vehicle/components に
    向け直し、project() を新しいアプリ名に書き換える。

    Every firmware/vehicle/examples/<N>/CMakeLists.txt sets
    `EXTRA_COMPONENT_DIRS ../../components`, which resolves (relative to
    that file's own directory, firmware/vehicle/examples/<N>/) to
    firmware/vehicle/components. A copy living one level shallower, at
    firmware/apps/<name>/, needs `../../vehicle/components` to resolve to
    the same directory -- verified with `os.path.relpath`, not guessed.
    firmware/vehicle/examples/<N>/CMakeLists.txt はいずれも
    `EXTRA_COMPONENT_DIRS ../../components` を設定しており、これは
    （そのファイル自身のディレクトリ firmware/vehicle/examples/<N>/ から見て）
    firmware/vehicle/components に解決される。1階層浅い
    firmware/apps/<name>/ に置かれた複製が同じディレクトリに解決するには
    `../../vehicle/components` が必要 -- 推測ではなく `os.path.relpath` で
    検証済み。
    """
    cmake_file = project_dir / "CMakeLists.txt"
    content = cmake_file.read_text(encoding="utf-8")

    component_dirs = "../../vehicle/components"
    content, count = re.subn(
        r"set\(EXTRA_COMPONENT_DIRS\s+[^)]*\)",
        f"set(EXTRA_COMPONENT_DIRS {component_dirs})",
        content,
    )
    if count == 0:
        console.warning(
            f"Could not find EXTRA_COMPONENT_DIRS in {cmake_file} -- check it manually"
        )

    content, count = re.subn(r"project\([^)]*\)", f"project({name})", content)
    if count == 0:
        console.warning(f"Could not find project(...) in {cmake_file} -- check it manually")

    cmake_file.write_text(content, encoding="utf-8")


def _write_new_app_manifest(project_dir: Path, name: str, from_example: str) -> Dict[str, Any]:
    """Ensure the freshly-copied project's app.yaml records its own `name`
    (and `from`). If the source template carried no app.yaml at all (09/10
    and any other pre-L1 example), synthesize a bench manifest so `sf app
    list`/`sf app sils` can still classify it (see sf-app-sils-plan.md §2
    "例題 09 / 10 の扱い"). Returns the manifest written, so the caller can
    branch its "Next steps" message on `type` without a second file read.
    複製直後のプロジェクトの app.yaml に自身の `name`（と `from`）を記録する。
    複製元テンプレートに app.yaml が無ければ（09/10 等 L1 以前の例題）、
    ベンチ型マニフェストを合成する。書き込んだマニフェストを返し、呼び出し側が
    再読込せずに `type` で「次のステップ」表示を分岐できるようにする。
    """
    manifest_path = project_dir / APP_MANIFEST_FILE
    if manifest_path.exists():
        manifest = load_app_manifest(project_dir)
        manifest["from"] = from_example
        manifest["name"] = name
    else:
        manifest = {"type": "bench", "from": from_example, "sils": False, "name": name}
        summary = _example_summary(_examples_dir() / from_example)
        if summary:
            manifest["description"] = summary
    write_app_manifest(project_dir, manifest)
    return manifest


def run_new(args: argparse.Namespace) -> int:
    """Create a new project cloned from an example"""
    name = args.name
    from_example = args.from_example

    error = _validate_new_name(name)
    if error:
        console.error(error)
        return 1

    examples_dir = _examples_dir()
    example_dir = examples_dir / from_example
    if not example_dir.exists() or not _project_dir_valid(example_dir):
        console.error(f"Example not found: '{from_example}'")
        console.print()
        console.print("Available examples (firmware/vehicle/examples/):")
        _print_available_examples(examples_dir)
        return 1

    project_dir = paths.apps() / name

    console.info(f"Creating new app project: {name}")
    console.print(f"  From: {example_dir}")
    console.print(f"  To:   {project_dir}")

    paths.ensure_dir(paths.apps())
    shutil.copytree(example_dir, project_dir, ignore=shutil.ignore_patterns(*_COPY_EXCLUDE))

    # Bench-type examples are standalone ESP-IDF projects (their own
    # CMakeLists.txt); embedded-type L1 templates (11/12) have none -- they
    # compile into vehicle's own main component via SF_APP_DIR instead (see
    # docs/plans/sf-app-sils-plan.md Phase 1), so there is no project() to
    # rename.
    # ベンチ型例題は独立した ESP-IDF プロジェクト（自身の CMakeLists.txt を
    # 持つ）。組み込み型 L1 テンプレート（11/12）は持たない -- vehicle 本体の
    # main コンポーネントに SF_APP_DIR 経由で組み込まれるため、書き換えるべき
    # project() が無い。
    if (project_dir / "CMakeLists.txt").exists():
        _rewrite_cmake_project(project_dir, name)

    manifest = _write_new_app_manifest(project_dir, name, from_example)

    console.success(f"Project created: {project_dir}")
    console.print()
    console.print("Next steps:")
    if manifest.get("type") == "embedded":
        console.print(f"  sf app sils {name}")
        console.print(f"  sf app build {name}")
        console.print(f"  sf app flash {name} -m")
    else:
        console.print(f"  sf app edit {name}")
        console.print(f"  sf app build {name}")
        console.print(f"  sf app flash {name} -m")

    return 0


# =============================================================================
# edit
# =============================================================================

def _learner_file(project_dir: Path) -> Optional[Path]:
    """The file the learner is expected to edit, or None if project_dir/main
    has neither of the files in _LEARNER_FILE_PRIORITY.
    学習者が編集すべきファイル。project_dir/main に
    _LEARNER_FILE_PRIORITY のいずれも無ければ None。
    """
    main_dir = project_dir / "main"
    for filename in _LEARNER_FILE_PRIORITY:
        candidate = main_dir / filename
        if candidate.exists():
            return candidate
    return None


def _embedded_learner_file(project_dir: Path) -> Optional[Path]:
    """The file to open for an embedded-type app, or None if project_dir has
    neither of the files in _EMBEDDED_LEARNER_FILE_PRIORITY. These templates
    have no main/ subdirectory (see run_new()) -- files live directly under
    project_dir.
    組み込み型 app を開く際のファイル。project_dir に
    _EMBEDDED_LEARNER_FILE_PRIORITY のいずれも無ければ None。これらの
    テンプレートは main/ サブディレクトリを持たない（run_new() 参照）--
    ファイルは project_dir 直下にある。
    """
    for filename in _EMBEDDED_LEARNER_FILE_PRIORITY:
        candidate = project_dir / filename
        if candidate.exists():
            return candidate
    return None


def run_edit(args: argparse.Namespace) -> int:
    """Open the learner-facing source file in the editor"""
    project_dir = paths.apps() / args.name
    if not project_dir.exists():
        console.error(f"Project not found: {project_dir}")
        console.print(f"  Create it first: sf app new {args.name}")
        return 1

    manifest = load_app_manifest(project_dir)
    if manifest.get("type") == "embedded":
        target_file = _embedded_learner_file(project_dir)
        search_hint = f"{project_dir} (looked for {' or '.join(_EMBEDDED_LEARNER_FILE_PRIORITY)})"
    else:
        target_file = _learner_file(project_dir)
        search_hint = (f"{project_dir / 'main'} "
                        f"(looked for {' or '.join(_LEARNER_FILE_PRIORITY)})")
    if target_file is None:
        console.error(f"No editable source file found under {search_hint}")
        return 1

    preferred = getattr(args, "editor", None)
    found = editor.find_editor(preferred)

    if found is None:
        if preferred:
            console.error(f"Editor not found: '{preferred}'")
        else:
            console.error("No editor found (tried: code, vi, vim)")
        console.print()
        for line in editor.install_hint(f"sf app edit {args.name} --editor <command>"):
            console.print(line)
        return 1

    editor_name, cmd = found
    reuse_window = bool(getattr(args, "reuse_window", False))

    # VSCode: open in a new window by default so this project does not
    # piggyback into an unrelated workspace already open in VSCode.
    # VSCode: 既に開いている別プロジェクトのウィンドウに紛れないよう、デフォルトで新規ウィンドウ
    extra_flags: List[str] = []
    if editor_name.startswith("VSCode") and not reuse_window:
        extra_flags = ["-n"]

    full_cmd = cmd + extra_flags + [str(target_file)]

    window_note = "" if not extra_flags else " (new window)"
    console.info(f"Opening {target_file.name} in {editor_name}{window_note}")
    console.print(f"  Path: {target_file}")

    try:
        # shell=False is safe: command + path are passed as a list
        # shell=False で安全: コマンドとパスをリストで渡す
        result = subprocess.run(full_cmd)
        return result.returncode
    except FileNotFoundError:
        console.error(f"Failed to launch editor: {' '.join(full_cmd)}")
        return 1


# =============================================================================
# build / flash
# =============================================================================

def _embedded_app_common_flags(app_dir: Path) -> List[str]:
    """idf.py global flags shared by an embedded app's build and flash: a
    build tree under the app's own directory (-B, so switching apps never
    shares/clobbers a build cache) plus -D SF_APP_DIR pointing vehicle's
    CMakeLists.txt at this app's sources (see firmware/vehicle/main/
    CMakeLists.txt's SF_APP_DIR handling, sf-app-sils-plan.md Phase 1).
    組み込み型 app のビルド・書き込みで共有する idf.py グローバルフラグ:
    app 自身のディレクトリ配下のビルドツリー（-B、app を切り替えても
    ビルドキャッシュを共有・汚染しない）と、vehicle の CMakeLists.txt に
    この app のソースを指させる -D SF_APP_DIR。
    """
    return ["-B", str(app_dir / "build"), "-D", f"SF_APP_DIR={app_dir.resolve()}"]


def _build_embedded_app(name: str, clean: bool, verbose: bool) -> int:
    """Build an embedded-type app INTO vehicle's own main component: `idf.py
    -B firmware/apps/<name>/build -D SF_APP_DIR=<abs app dir> build`, run
    with cwd=firmware/vehicle (an embedded app has no CMakeLists.txt of its
    own -- vehicle's own CMakeLists.txt/main/CMakeLists.txt pick up
    SF_APP_DIR's sources).
    組み込み型 app を vehicle 本体の main コンポーネントへ組み込んでビルドする。
    cwd は firmware/vehicle（組み込み型 app は自身の CMakeLists.txt を持たない
    -- vehicle 自身の CMakeLists.txt / main/CMakeLists.txt が SF_APP_DIR の
    ソースを取り込む）。
    """
    app_dir = paths.apps() / name
    vehicle_dir = paths.vehicle()
    build_dir = app_dir / "build"

    console.info(f"Building embedded app '{name}' into vehicle firmware...")
    console.print(f"  App:   {app_dir}")
    console.print(f"  Build: {build_dir}")

    idf_path = platform.esp_idf_path()
    if not idf_path:
        console.error("ESP-IDF not found. Please install ESP-IDF first.")
        return 1

    env = espidf.prepare_idf_env(idf_path)
    env_error = espidf.verify_idf_env(env)
    if env_error:
        console.error(env_error)
        return 1

    common_flags = _embedded_app_common_flags(app_dir)

    if clean:
        console.info("Cleaning build directory...")
        result = subprocess.run(
            espidf.idf_command(common_flags + ["fullclean"]), cwd=vehicle_dir, env=env,
        )
        if result.returncode != 0:
            console.warning("Clean failed, continuing with build...")

    cmd = espidf.idf_command(common_flags + ["build"])
    if verbose:
        cmd.append("-v")

    console.print()
    console.info(f"Running: {' '.join(cmd)}")
    console.print()

    result = subprocess.run(cmd, cwd=vehicle_dir, env=env)
    if result.returncode == 0:
        console.print()
        console.success(f"Build successful: {name}")
        binary_path = build_dir / _VEHICLE_BINARY_NAME
        if binary_path.exists():
            size_kb = binary_path.stat().st_size / 1024
            console.print(f"  Binary: {binary_path}")
            console.print(f"  Size: {size_kb:.1f} KB")
        return 0

    console.print()
    console.error(f"Build failed: {name}")
    return result.returncode


def _flash_embedded_app(name: str, port: Optional[str], baud: int, monitor: bool) -> int:
    """Flash an embedded-type app's vehicle build: `idf.py -B
    firmware/apps/<name>/build -D SF_APP_DIR=<abs app dir> -p <port> -b
    <baud> flash [monitor]`, run with cwd=firmware/vehicle (same rationale
    as _build_embedded_app()).
    組み込み型 app の vehicle ビルドを書き込む。cwd は firmware/vehicle
    （_build_embedded_app() と同じ理由）。
    """
    app_dir = paths.apps() / name
    vehicle_dir = paths.vehicle()

    idf_path = platform.esp_idf_path()
    if not idf_path:
        console.error("ESP-IDF not found. Please install ESP-IDF first.")
        return 1

    resolved_port = port
    if not resolved_port:
        resolved_port = platform.default_serial_port()
        if not resolved_port:
            console.error("No serial port detected. Please specify with -p/--port")
            available = platform.serial_ports()
            if available:
                console.print("Available ports:")
                for p in available:
                    console.print(f"  {p}")
            return 1
        console.info(f"Auto-detected port: {resolved_port}")

    env = espidf.prepare_idf_env(idf_path)
    env_error = espidf.verify_idf_env(env)
    if env_error:
        console.error(env_error)
        return 1

    common_flags = _embedded_app_common_flags(app_dir)
    cmd = espidf.idf_command(common_flags + ["-p", resolved_port, "-b", str(baud)])
    cmd.append("flash")
    if monitor:
        cmd.append("monitor")

    console.info(f"Flashing embedded app '{name}'...")
    console.print(f"  Port: {resolved_port}")
    console.print(f"  Baud: {baud}")
    console.print()

    result = subprocess.run(cmd, cwd=vehicle_dir, env=env)
    if result.returncode == 0:
        if not monitor:
            console.print()
            console.success(f"Flash successful: {name}")
        return 0

    console.print()
    console.error(f"Flash failed: {name}")
    return result.returncode


def run_build(args: argparse.Namespace) -> int:
    """Build a project: an embedded-type app builds INTO vehicle's own main
    component; a bench-type app delegates to `sf build apps/<name>` as
    before (its own standalone ESP-IDF project)."""
    project_dir = paths.apps() / args.name
    if not project_dir.exists():
        console.error(f"Project not found: {project_dir}")
        console.print(f"  Create it first: sf app new {args.name}")
        return 1

    manifest = load_app_manifest(project_dir)
    if manifest.get("type") == "embedded":
        return _build_embedded_app(args.name, clean=args.clean, verbose=args.verbose)

    from . import build as build_cmd

    # Delegate through build.py's factory so newly added run() attributes
    # stay in sync automatically (see build.make_run_args docstring).
    # build.py のファクトリ経由で委譲する。run() に属性が増えても自動的に
    # 追従する（build.make_run_args の docstring 参照）。
    build_args = build_cmd.make_run_args(
        target=f"apps/{args.name}",
        clean=args.clean,
        verbose=args.verbose,
    )
    return build_cmd.run(build_args)


def run_flash(args: argparse.Namespace) -> int:
    """Flash a project: an embedded-type app flashes vehicle's own build; a
    bench-type app delegates to `sf flash apps/<name>` as before (its own
    standalone ESP-IDF project)."""
    project_dir = paths.apps() / args.name
    if not project_dir.exists():
        console.error(f"Project not found: {project_dir}")
        console.print(f"  Create it first: sf app new {args.name}")
        return 1

    manifest = load_app_manifest(project_dir)
    if manifest.get("type") == "embedded":
        return _flash_embedded_app(args.name, port=args.port, baud=args.baud, monitor=args.monitor)

    from . import flash as flash_cmd

    # Delegate through flash.py's factory so newly added run() attributes
    # (e.g. --gui) stay in sync automatically (see flash.make_run_args
    # docstring).
    # flash.py のファクトリ経由で委譲する。run() に属性が増えても
    # （例: --gui）自動的に追従する（flash.make_run_args の docstring参照）。
    flash_args = flash_cmd.make_run_args(
        target=f"apps/{args.name}",
        port=args.port,
        baud=args.baud,
        monitor=args.monitor,
    )
    return flash_cmd.run(flash_args)


# =============================================================================
# sils
# =============================================================================

def run_app_sils(args: argparse.Namespace) -> int:
    """Run an embedded-type app's SILS scenario (`sf sils scenario ...
    --target apps/<name>`, built via sils.build_app_emulator()). Bench-type
    apps are standalone projects that never get compiled into vehicle's main
    component, so they cannot run in the SILS vehicle emulator at all.
    組み込み型 app の SILS シナリオを実行する（sils.build_app_emulator() で
    ビルドした `sf sils scenario ... --target apps/<name>`）。ベンチ型 app は
    vehicle の main コンポーネントに一切組み込まれない独立プロジェクトのため、
    SILS の vehicle エミュレータでは実行できない。
    """
    from . import sils as sils_cmd

    project_dir = paths.apps() / args.name
    if not project_dir.exists():
        console.error(f"Project not found: {project_dir}")
        console.print(f"  Create it first: sf app new {args.name}")
        return 1

    manifest = load_app_manifest(project_dir)
    if manifest.get("type") != "embedded":
        console.error(
            f"'{args.name}' is a standalone bench project's clone and cannot be embedded "
            "into the vehicle firmware for SILS."
        )
        console.print(f"  Use: sf app new <name> --from {DEFAULT_EXAMPLE}")
        return 2

    scenario = Path(args.scenario) if args.scenario else paths.root() / DEFAULT_SILS_SCENARIO
    if not scenario.exists():
        console.error(f"scenario not found: {scenario}")
        return 1

    build_dir = sils_cmd.app_build_dir(args.name)
    exe = sils_cmd.build_app_emulator(project_dir, build_dir)
    if not exe.exists():
        console.error(f"SILS build failed for app '{args.name}' -- see cmake/build output above")
        return 1

    # A plain argparse.Namespace mirroring `sf sils scenario`'s own attributes
    # (see sils.py register()'s `scenario` subparser) -- run_scenario_with_exe()
    # reads these via getattr() with the same defaults `sf sils scenario` uses,
    # same pattern as sils.py's own run_sysid_gate().
    # `sf sils scenario` 自身の属性を模した素の argparse.Namespace（sils.py の
    # register() の scenario サブパーサ参照）-- run_scenario_with_exe() は
    # `sf sils scenario` と同じ既定値で getattr() 経由で読む。sils.py 自身の
    # run_sysid_gate() と同じパターン。
    scenario_args = argparse.Namespace(
        target=f"apps/{args.name}", scenario=str(scenario), expect=args.expect,
        duration=args.duration, noise=args.noise, seed=args.seed,
        video=False, ground_effect=None, turbulence=None, motor_delay=None,
        thrust_eff=None, torque_authority=None, flow_scale=None,
        unpaired=False, params=None, extra_env=None,
    )
    return sils_cmd.run_scenario_with_exe(exe, scenario, scenario_args)


# =============================================================================
# list
# =============================================================================

_LIST_TABLE_HEADERS = ["Name", "Type", "Hardware", "SILS", "Description"]


def _list_row(d: Path) -> List[str]:
    """One `sf app list` table row for project/example directory `d`: name,
    type (embedded/bench), whether it can be flashed to real hardware
    (always yes -- both types are ESP-IDF projects, either standalone or via
    vehicle's own build), whether it can run in the SILS vehicle emulator,
    and a description (app.yaml's own, falling back to the example's
    README.md heading for pre-L1 examples that have no app.yaml).
    `sf app list` の1行分: 名前、種別（embedded/bench）、実機書き込み可否
    （常に可 -- どちらの型も ESP-IDF プロジェクト、独立またはvehicle本体
    経由）、SILS の vehicle エミュレータでの実行可否、説明（app.yaml 自身の
    説明。app.yaml を持たない L1 以前の例題は README.md の見出しにフォール
    バック）。
    """
    manifest = load_app_manifest(d)
    app_type = manifest.get("type", "bench")
    sils_ok = "yes" if manifest.get("sils") else "no (bench)"
    description = manifest.get("description") or _example_summary(d)
    return [d.name, app_type, "yes", sils_ok, description]


def run_list(args: argparse.Namespace) -> int:
    """List firmware/apps/ projects and firmware/vehicle/examples/ sources"""
    apps_dir = paths.apps()

    console.header("Your Projects (firmware/apps/)")
    console.print()
    rows = [_list_row(d) for d in sorted(apps_dir.iterdir()) if _project_dir_valid(d)] \
        if apps_dir.exists() else []
    if rows:
        console.table(_LIST_TABLE_HEADERS, rows)
    else:
        console.print("  (none yet -- create one: sf app new <name>)")
    console.print()

    console.header("Examples Available for --from (firmware/vehicle/examples/)")
    console.print()
    examples_dir = _examples_dir()
    example_rows = [_list_row(d) for d in sorted(examples_dir.iterdir()) if _project_dir_valid(d)] \
        if examples_dir.exists() else []
    if example_rows:
        console.table(_LIST_TABLE_HEADERS, example_rows)
    else:
        console.print("  (no examples found)")
    console.print()
    console.print(f"  Default: {DEFAULT_EXAMPLE}")

    return 0
