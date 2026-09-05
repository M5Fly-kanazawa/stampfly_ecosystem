"""
sf doctor - Diagnose environment issues

Checks the development environment for common issues.
開発環境の問題を診断します。
"""

import argparse
import os
import re
import subprocess
import sys
from importlib import metadata as importlib_metadata
from pathlib import Path
from typing import Optional, Tuple
from ..utils import console, paths, platform
from . import sils as sils_cmd

COMMAND_NAME = "doctor"
COMMAND_HELP = "Diagnose environment issues"

MANIFEST_FILE = "lesson_manifest.yaml"

# Distribution name as declared in pyproject.toml's [project].name --
# the key importlib.metadata looks packages up by.
# pyproject.tomlの[project].nameに宣言された配布名 --
# importlib.metadataがパッケージを引く際のキー。
DISTRIBUTION_NAME = "stampfly-ecosystem"

# A requirement string's distribution name is everything before the first
# version specifier / environment marker character.
# 要件文字列の配布名は、最初のバージョン指定子／環境マーカー文字の
# 手前までの部分。
_REQUIREMENT_NAME_SPLIT_RE = re.compile(r"[<>=!~; \[]")

# Some distribution names don't match their import name; keep this list
# only for the (rare) exceptions -- everything else falls back to
# `dist_name.replace("-", "_")`.
# 配布名とimport名が一致しない例外だけをここに列挙する。それ以外は
# `dist_name.replace("-", "_")` にフォールバックする。
_IMPORT_NAME_OVERRIDES = {
    "pyyaml": "yaml",
    "pyserial": "serial",
}

# Fallback list, used only if package metadata cannot be read at all
# (e.g. sfcli was added to PYTHONPATH rather than `pip install`ed, so
# there is no installed-distribution metadata to introspect).
# パッケージメタデータが全く読めない場合（pip install されず
# PYTHONPATHに追加されただけ等、参照できるメタデータが無い場合）専用の
# フォールバックリスト。
_FALLBACK_REQUIRED_PACKAGES = [
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("matplotlib", "matplotlib"),
    ("pandas", "pandas"),
    ("serial", "pyserial"),
    ("yaml", "PyYAML"),
]


def _base_dependency_import_names() -> list:
    """(import_name, distribution_name) pairs for stampfly-ecosystem's
    base runtime dependencies (no extras), read from installed package
    metadata so this list can never drift out of sync with pyproject.toml.

    Falls back to _FALLBACK_REQUIRED_PACKAGES if the metadata cannot be
    read at all.
    stampfly-ecosystem のベース実行時依存関係（extra指定なし）を、
    インストール済みパッケージのメタデータから
    (import名, 配布名) のタプル一覧として動的に生成する。これにより
    pyproject.tomlとの乖離が起こらない。メタデータが全く読めない場合は
    _FALLBACK_REQUIRED_PACKAGES へフォールバックする。
    """
    try:
        requirements = importlib_metadata.requires(DISTRIBUTION_NAME) or []
    except importlib_metadata.PackageNotFoundError:
        return _FALLBACK_REQUIRED_PACKAGES

    packages = []
    for requirement in requirements:
        # A requirement with an "extra ==" marker belongs to an
        # optional-dependencies group (sim/full/edu/dev) -- doctor only
        # checks the base dependencies that are always installed.
        # "extra ==" マーカー付きはoptional-dependenciesグループ
        # (sim/full/edu/dev)のもの -- doctorは常時インストールされる
        # ベース依存のみを確認する。
        if "extra ==" in requirement:
            continue
        dist_name = _REQUIREMENT_NAME_SPLIT_RE.split(requirement, maxsplit=1)[0].strip()
        if not dist_name:
            continue
        import_name = _IMPORT_NAME_OVERRIDES.get(
            dist_name.lower(), dist_name.replace("-", "_")
        )
        packages.append((import_name, dist_name))

    return packages or _FALLBACK_REQUIRED_PACKAGES


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Attempt to fix issues automatically",
    )
    parser.set_defaults(func=run)


def _check_manifest(warnings: list, issues: list) -> None:
    """Validate lesson manifest against filesystem.

    Checks:
    1. Manifest file exists and is valid YAML
    2. Each lesson's firmware_dir exists (if specified)
    3. Each firmware_dir has student.cpp and solution.cpp
    4. Each lesson's slide_file exists in chapters/
    5. No orphan lesson directories (dirs not in manifest)
    """
    lessons_dir = paths.workshop() / "lessons"
    manifest_path = lessons_dir / MANIFEST_FILE

    if not manifest_path.exists():
        warnings.append("Lesson manifest not found")
        console.warning(f"  {MANIFEST_FILE}: NOT FOUND")
        return

    try:
        import yaml
    except ImportError:
        warnings.append("PyYAML not installed — cannot validate manifest")
        console.warning("  PyYAML: NOT INSTALLED (pip install pyyaml)")
        return

    try:
        with open(manifest_path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        lessons = data.get("lessons", [])
    except Exception as e:
        issues.append(f"Manifest parse error: {e}")
        console.error(f"  {MANIFEST_FILE}: PARSE ERROR — {e}")
        return

    console.success(f"  {MANIFEST_FILE}: {len(lessons)} lessons")

    # Slide chapters directory
    chapters_dir = (paths.root() / "docs" / "workshop"
                    / "slides" / "beamer" / "chapters")

    # Track manifest firmware dirs for orphan check
    manifest_fw_dirs = set()

    for entry in lessons:
        num = entry.get("number", "?")
        lid = entry.get("id", "unknown")
        fw_dir = entry.get("firmware_dir")
        slide_file = entry.get("slide_file")

        # Check firmware directory
        if fw_dir is not None:
            manifest_fw_dirs.add(fw_dir)
            fw_path = lessons_dir / fw_dir
            if not fw_path.exists():
                issues.append(f"Lesson {num} ({lid}): firmware_dir '{fw_dir}' missing")
                console.error(f"  Lesson {num}: firmware_dir '{fw_dir}' NOT FOUND")
            else:
                # Check for required files. student.cpp is always required;
                # solution.cpp is skipped for lessons that declare
                # has_solution: false (e.g. event templates with no single
                # "correct answer" by design — see lesson_manifest.yaml).
                # student.cppは常に必須。solution.cppはhas_solution: false
                # を宣言したレッスン（設計上「正解」が存在しないイベント用
                # テンプレート等。lesson_manifest.yaml参照）ではスキップする
                has_solution = entry.get("has_solution", True)
                required_files = ("student.cpp", "solution.cpp") if has_solution else ("student.cpp",)
                for fname in required_files:
                    if not (fw_path / fname).exists():
                        warnings.append(f"Lesson {num} ({lid}): {fname} missing")
                        console.warning(f"  Lesson {num}: {fw_dir}/{fname} MISSING")

        # Check slide file
        if slide_file:
            slide_path = chapters_dir / f"{slide_file}.tex"
            if not slide_path.exists():
                issues.append(f"Lesson {num} ({lid}): slide '{slide_file}.tex' missing")
                console.error(f"  Lesson {num}: chapters/{slide_file}.tex NOT FOUND")

    # Orphan check: lesson directories not in manifest
    if lessons_dir.exists():
        for d in sorted(lessons_dir.iterdir()):
            if not d.is_dir() or not d.name.startswith("lesson_"):
                continue
            if d.name not in manifest_fw_dirs:
                warnings.append(f"Orphan directory: {d.name} (not in manifest)")
                console.warning(f"  Orphan: {d.name} (not in manifest)")

    _check_courses(data.get("courses", []), lessons, warnings, issues)


def _check_courses(courses: list, lessons: list, warnings: list, issues: list) -> None:
    """Validate the manifest's `courses:` list against `lessons:`.

    マニフェストの `courses:` リストを `lessons:` と照合検証する。

    Checks:
    1. Every step's `lesson` field references an existing lesson id.
    2. Step numbers are unique within a course.
    3. Course ids are unique across the file.
    チェック内容:
    1. 各ステップの `lesson` フィールドが実在するレッスンidを指しているか
    2. コース内でステップ番号が重複していないか
    3. ファイル内でコースidが重複していないか
    """
    if not courses:
        return

    known_lesson_ids = {e.get("id") for e in lessons}
    console.success(f"  courses: {len(courses)} course(s)")

    seen_course_ids = set()
    for course in courses:
        cid = course.get("id", "unknown")
        if cid in seen_course_ids:
            issues.append(f"Course '{cid}': duplicate course id")
            console.error(f"  Course '{cid}': DUPLICATE course id")
        seen_course_ids.add(cid)

        seen_numbers = set()
        for step in course.get("steps", []):
            number = step.get("number", "?")
            lesson_id = step.get("lesson")

            if number in seen_numbers:
                issues.append(f"Course '{cid}' step {number}: duplicate step number")
                console.error(f"  Course '{cid}' step {number}: DUPLICATE step number")
            seen_numbers.add(number)

            if lesson_id not in known_lesson_ids:
                issues.append(f"Course '{cid}' step {number}: unknown lesson id '{lesson_id}'")
                console.error(f"  Course '{cid}' step {number}: unknown lesson id '{lesson_id}'")


# The ecosystem's actually-tested Python range, mirroring
# scripts/installer.py's PYTHON_PREFERRED_MIN/MAX (2026-07-22 policy:
# 3.13+ has real failure reports and is treated as unsupported, same as
# anything older than 3.10). Duplicated here rather than imported: sfcli
# is installed *inside* the ESP-IDF venv scripts/installer.py sets up, so
# it cannot import that standalone top-level script back (no package
# relationship between them -- see scripts/installer.py's own "Stability
# contract" for why it must stay a separate, stdlib-only file).
# scripts/installer.py の PYTHON_PREFERRED_MIN/MAX と同じ値
# (2026-07-22の方針: 3.13+は実際に動作しない事例があり、3.10未満と同様に
# 未対応として扱う)。ここで複製する理由: sfcli は scripts/installer.py が
# セットアップする ESP-IDF venv の「内側」にインストールされるため、その
# 独立したトップレベルスクリプトを逆に import することはできない
# (両者にパッケージ関係は無い -- スタンドアロン・stdlib限定でなければ
# ならない理由は scripts/installer.py 自身の「安定契約」参照)。
_PYTHON_PREFERRED_MIN = (3, 10)
_PYTHON_PREFERRED_MAX = (3, 12)


def _idf_tools_path_candidates() -> list:
    """Likely IDF_TOOLS_PATH locations, in priority order. Minimal
    reimplementation of scripts/installer.py's
    _idf_tools_path_candidates() (see _PYTHON_PREFERRED_MIN's docstring
    for why it cannot be imported instead).
    IDF_TOOLS_PATH の候補を優先順に返す。scripts/installer.py の
    _idf_tools_path_candidates() の最小限の再実装
    (import できない理由は _PYTHON_PREFERRED_MIN のdocstring参照)。
    """
    tools_path = os.environ.get("IDF_TOOLS_PATH")
    if tools_path:
        return [Path(tools_path)]
    if platform.is_windows():
        return [Path("C:/Espressif"), Path.home() / ".espressif"]
    return [Path.home() / ".espressif"]


def _idf_major_minor(idf_version: Optional[str]) -> Optional[str]:
    """Extract MAJOR.MINOR from an ESP-IDF version string (e.g.
    "v5.5.2" -> "5.5"), or None if it cannot be parsed.
    ESP-IDF バージョン文字列(例: "v5.5.2")から MAJOR.MINOR を抽出する。
    解析できなければ None。
    """
    if not idf_version:
        return None
    match = re.search(r"v?(\d+)\.(\d+)", idf_version)
    return f"{match.group(1)}.{match.group(2)}" if match else None


def _find_idf_venv_dir(idf_version: Optional[str]) -> Optional[Path]:
    """Find the newest ESP-IDF-managed venv directory matching
    `idf_version`'s MAJOR.MINOR (``idf<MAJOR.MINOR>_py*_env`` under
    ``<IDF_TOOLS_PATH>/python_env/``), or None if none exists yet.
    idf_version の MAJOR.MINOR に一致する ESP-IDF 管理 venv ディレクトリの
    うち最新のものを返す(``<IDF_TOOLS_PATH>/python_env/`` 配下の
    ``idf<メジャー.マイナー>_py*_env``)。まだ存在しなければ None。
    """
    target = _idf_major_minor(idf_version)
    if not target:
        return None
    prefix = f"idf{target}_py"

    for base in _idf_tools_path_candidates():
        python_env_dir = base / "python_env"
        if not python_env_dir.exists():
            continue
        try:
            entries = list(python_env_dir.iterdir())
        except OSError:
            continue
        candidates = sorted(
            (d for d in entries
             if d.is_dir() and d.name.startswith(prefix) and d.name.endswith("_env")),
            reverse=True,
        )
        if candidates:
            return candidates[0]
    return None


def _parse_pyvenv_cfg_home(pyvenv_cfg: Path) -> Optional[Path]:
    """Read a pyvenv.cfg file's `home = ...` line and return it as a Path,
    or None if unreadable / has no `home` entry. Minimal reimplementation
    of scripts/installer.py's identically-named helper (see
    _PYTHON_PREFERRED_MIN's docstring for why it cannot be imported).
    pyvenv.cfg の `home = ...` 行を読み Path として返す。読めない、または
    `home` エントリが無ければ None。scripts/installer.py の同名ヘルパーの
    最小限の再実装(importできない理由は _PYTHON_PREFERRED_MIN の
    docstring参照)。
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


def _venv_python_version(venv_python: Path) -> Optional[Tuple[int, int]]:
    """Return (major, minor) reported by venv_python, or None on any
    failure. Never raises.
    venv_python が報告する (major, minor) を返す。失敗時は None
    (例外は送出しない)。
    """
    try:
        result = subprocess.run(
            [str(venv_python), "-c",
             "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')"],
            capture_output=True, timeout=15,
            encoding="utf-8", errors="replace",
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    match = re.match(r"(\d+)\.(\d+)", (result.stdout or "").strip())
    return (int(match.group(1)), int(match.group(2))) if match else None


def _check_esp_idf_venv_health(idf_path: Path, idf_version: Optional[str], warnings: list) -> None:
    """Check that the ESP-IDF-managed venv sfcli lives in has not gone
    dead: its pyvenv.cfg / recorded base ("seed") Python still exist, and
    the venv's own python still responds. This is a DIAGNOSIS-only check
    (unlike scripts/installer.py's _recreate_dead_idf_venvs(), doctor
    never deletes or recreates anything) -- if dead, it points the user at
    the installer's repair mode instead.

    Also warns (but does not treat as dead) if the venv's own Python is
    3.13+: that combination is untested and unsupported for NEW installs
    (see scripts/installer.py's 2026-07-22 policy change), but an
    already-working venv built on it is left alone here too, matching
    scripts/installer.py's _warn_if_idf_venv_python_too_new().

    Does nothing (no warning) if no matching venv directory exists yet --
    that is a normal pre-install state, not a health problem.
    sfcli が住んでいる ESP-IDF 管理 venv が壊死していないかを確認する:
    pyvenv.cfg / そこに記録されたベース("種")Pythonがまだ存在するか、
    venv 自身の python がまだ応答するか。これは診断のみ(削除・再作成は
    行わない)のチェックであり、scripts/installer.py の
    _recreate_dead_idf_venvs() とは異なる -- 壊死していれば、代わりに
    インストーラの修復モードを案内する。

    venv 自身の Python が3.13+の場合も警告する(壊死扱いにはしない):
    その組み合わせは新規インストールにおいて未検証・未対応だが
    (scripts/installer.py の2026-07-22方針変更参照)、既に動作している
    venv はここでもそのままにする
    (scripts/installer.py の _warn_if_idf_venv_python_too_new() と同じ)。

    一致する venv ディレクトリがまだ存在しない場合は何もしない(警告なし)
    -- インストール前の正常な状態であり、健全性の問題ではない。
    """
    venv_dir = _find_idf_venv_dir(idf_version)
    if venv_dir is None:
        return  # not created yet -- not this check's concern / 未作成 -- 対象外

    bin_subdir = "Scripts" if platform.is_windows() else "bin"
    python_name = "python.exe" if platform.is_windows() else "python"
    venv_python = venv_dir / bin_subdir / python_name

    pyvenv_cfg = venv_dir / "pyvenv.cfg"
    home = _parse_pyvenv_cfg_home(pyvenv_cfg) if pyvenv_cfg.is_file() else None
    base_exe_name = "python.exe" if platform.is_windows() else "python3"
    base_exe = (home / base_exe_name) if home else None
    if base_exe is not None and not base_exe.is_file() and not platform.is_windows():
        base_exe = home / "python"

    is_dead = (
        not pyvenv_cfg.is_file()
        or home is None
        or not home.is_dir()
        or base_exe is None
        or not base_exe.is_file()
        or not venv_python.is_file()
    )
    version = None if is_dead else _venv_python_version(venv_python)
    if not is_dead and version is None:
        is_dead = True

    if is_dead:
        warnings.append(f"ESP-IDF Python venv looks dead: {venv_dir}")
        console.warning(f"  ESP-IDF venv: DEAD ({venv_dir})")
        console.print("    Its base (\"seed\") Python appears to have been removed or upgraded.")
        console.print("    ベースの(\"種\")Pythonが削除または更新された可能性があります。")
        console.print("    Fix: re-run the installer (or its repair mode) to recreate it:")
        console.print(f"      python {paths.root() / 'scripts' / 'installer.py'} --clean")
        return

    console.success(f"  ESP-IDF venv: {venv_dir} (Python {version[0]}.{version[1]})")
    if version > _PYTHON_PREFERRED_MAX:
        warnings.append(
            f"ESP-IDF venv is built on Python {version[0]}.{version[1]}, "
            f"newer than the supported {_PYTHON_PREFERRED_MIN[0]}.{_PYTHON_PREFERRED_MIN[1]}-"
            f"{_PYTHON_PREFERRED_MAX[0]}.{_PYTHON_PREFERRED_MAX[1]} range"
        )
        console.warning(
            f"    Python {version[0]}.{version[1]} is newer than the supported "
            f"{_PYTHON_PREFERRED_MIN[0]}.{_PYTHON_PREFERRED_MIN[1]}-"
            f"{_PYTHON_PREFERRED_MAX[0]}.{_PYTHON_PREFERRED_MAX[1]} range "
            "(untested; real failures have been reported for 3.13+)."
        )
        console.warning(
            f"    対応範囲({_PYTHON_PREFERRED_MIN[0]}.{_PYTHON_PREFERRED_MIN[1]}〜"
            f"{_PYTHON_PREFERRED_MAX[0]}.{_PYTHON_PREFERRED_MAX[1]})より新しい Python "
            f"{version[0]}.{version[1]} です(未検証。3.13以降は動作しない事例が"
            "報告されています)。"
        )


def _check_sils_toolchain(warnings: list) -> None:
    """Check the SILS host-bench build toolchain (simulator/sils/). This is
    OPTIONAL — only people building/running the SILS need it — so any gap is a
    WARN, never a hard error.

    Windows only, so far: looks for a MinGW-w64 toolchain (GCC/G++ + Ninja),
    since the SILS's host build is C++17 + std::thread-based and needs a real
    g++ (MSVC alone cannot build it — see simulator/sils/README.md's Windows
    section). On Linux/macOS the system's own gcc/clang is assumed to work
    and is not separately checked here (in spirit already covered by "is
    there a working C/C++ toolchain on this machine" elsewhere in doctor).

    Also confirms the found MinGW's THREAD MODEL is "posix", not "win32": a
    MinGW-w64 build configured without pthread support cannot link
    std::thread at all (the SILS's RTOS emulator and MuJoCo both need it), so
    a non-posix thread model is flagged even though gcc itself was found.

    SILS ホストベンチ（simulator/sils/）のビルドツールチェーンを確認する。任意
    （SILS を使わない人には無関係）なので欠落は常に警告（エラーにしない）。

    今のところ Windows のみ: MinGW-w64（GCC/G++ + Ninja）を探す。SILS のホスト
    ビルドは C++17・std::thread ベースで実 g++ が要る（MSVC 単体ではビルド
    不可 — simulator/sils/README.md の Windows 節参照）。Linux/macOS はシステム
    の gcc/clang が動く前提で別チェックはしない（doctor の他所で「動く C/C++
    ツールチェーンがあるか」を趣旨的にカバー済み）。

    見つかった MinGW のスレッドモデルが "posix"（"win32" でない）ことも確認する:
    pthread 対応無しでビルドされた MinGW-w64 は std::thread を一切リンクできない
    （SILS の RTOS エミュレータと MuJoCo の両方が必要とする）ため、gcc 自体は
    見つかってもスレッドモデルが posix でなければ警告する。
    """
    if not platform.is_windows():
        # On Linux/macOS, the system's gcc/clang is assumed to work.
        # No separate MinGW check is needed here (already covered by system
        # toolchain validation elsewhere in doctor).
        # Linux/macOS ではシステムの gcc/clang が動く前提。個別の MinGW チェック
        # は不要（doctor の他所でシステムツールチェーン検証済み）。
        console.info("  Not Windows — skipping MinGW check (system gcc/clang assumed)")
        return

    mingw = sils_cmd.mingw_bin()
    if mingw is None:
        warnings.append("SILS host toolchain: no MinGW-w64 found (simulator/sils/ build)")
        console.warning("  SILS host toolchain (MinGW-w64): NOT FOUND")
        console.print("    Only needed to build/run the SILS host bench (simulator/sils/).")
        console.print("    SILS ホストベンチ（simulator/sils/）のビルド・実行にのみ必要。")
        console.print("    Install / 導入: winget install --id MSYS2.MSYS2 "
                      "--silent --accept-package-agreements --accept-source-agreements")
        console.print('    Then / 続けて: C:\\msys64\\usr\\bin\\bash -lc "pacman -S '
                      "--noconfirm --needed mingw-w64-x86_64-toolchain "
                      'mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja"')
        return

    console.success(f"  SILS host toolchain (MinGW-w64): {mingw}")
    try:
        result = subprocess.run([str(mingw / "g++.exe"), "-v"], capture_output=True,
                                text=True, timeout=15, encoding="utf-8", errors="replace")
        thread_model = None
        for line in (result.stderr or "").splitlines():
            if line.strip().startswith("Thread model:"):
                thread_model = line.split(":", 1)[1].strip()
        if thread_model == "posix":
            console.success(f"    thread model: {thread_model} (std::thread OK)")
        else:
            warnings.append(
                f"SILS host toolchain: MinGW thread model is '{thread_model}', "
                "not 'posix' — std::thread will not link"
            )
            console.warning(
                f"    thread model: {thread_model or '?'} (need 'posix' for "
                "std::thread — reinstall mingw-w64-x86_64-toolchain)"
            )
    except (OSError, subprocess.SubprocessError) as e:
        warnings.append(f"SILS host toolchain: could not query g++ ({e})")
        console.warning(f"    could not query g++ -v: {e}")


def run(args: argparse.Namespace) -> int:
    """Execute doctor command"""
    console.header("StampFly Environment Diagnostics")
    console.print()

    issues = []
    warnings = []

    # Check Python version
    console.info("Checking Python...")
    py_version = sys.version_info
    if py_version < (3, 8):
        issues.append(f"Python 3.8+ required, found {py_version.major}.{py_version.minor}")
        console.error(f"  Python {py_version.major}.{py_version.minor} - 3.8+ required")
    else:
        console.success(f"  Python {py_version.major}.{py_version.minor}.{py_version.micro}")

    # Check repository structure
    console.print()
    console.info("Checking repository structure...")
    root = paths.root()

    required_dirs = [
        ("firmware", "Firmware source"),
        ("firmware/vehicle", "Vehicle firmware"),
        ("simulator", "Simulator"),
        ("docs", "Documentation"),
    ]

    for dir_name, description in required_dirs:
        dir_path = root / dir_name
        if dir_path.exists():
            console.success(f"  {description}: {dir_path}")
        else:
            issues.append(f"Missing directory: {dir_name}")
            console.error(f"  {description}: NOT FOUND")

    # Legacy firmware (optional, not a hard requirement)
    # レガシーファーム（任意、必須ではない）
    vehicle_old_path = root / "firmware" / "vehicle_old"
    if vehicle_old_path.exists():
        console.success(f"  Legacy vehicle firmware: {vehicle_old_path}")

    # Check ESP-IDF
    console.print()
    console.info("Checking ESP-IDF...")
    idf_path = platform.esp_idf_path()
    if idf_path:
        idf_version = platform.esp_idf_version()
        console.success(f"  ESP-IDF: {idf_version or 'found'}")
        console.success(f"  Path: {idf_path}")

        # Check if export.sh exists
        export_script = idf_path / "export.sh"
        if not export_script.exists():
            warnings.append("ESP-IDF export.sh not found")
            console.warning("  export.sh: NOT FOUND")

        # Check the ESP-IDF-managed Python venv's own health (dead-seed
        # detection, diagnosis only -- see _check_esp_idf_venv_health()).
        # ESP-IDF管理Python venv自身の健全性を確認(壊死種の検出、診断のみ
        # -- _check_esp_idf_venv_health() 参照)。
        _check_esp_idf_venv_health(idf_path, idf_version, warnings)
    else:
        warnings.append("ESP-IDF not found")
        console.warning("  ESP-IDF: NOT FOUND")
        console.print("    Install from: https://docs.espressif.com/projects/esp-idf/")

    # Check Python packages
    console.print()
    console.info("Checking Python packages...")
    required_packages = _base_dependency_import_names()

    for import_name, package_name in required_packages:
        try:
            __import__(import_name)
            console.success(f"  {package_name}")
        except ImportError:
            warnings.append(f"Missing package: {package_name}")
            console.warning(f"  {package_name}: NOT INSTALLED")

    # Check hidapi native library (for joystick / simulator)
    # hidapiネイティブライブラリの確認（ジョイスティック／シミュレータ用）
    console.print()
    console.info("Checking hidapi (joystick support)...")
    try:
        import hid
        hid.enumerate()
        console.success("  hidapi native library available")
    except ImportError:
        warnings.append("hid package not installed")
        console.warning("  hid package: NOT INSTALLED")
        console.print("    pip install hid")
    except OSError:
        warnings.append("hidapi native library not found")
        console.warning("  hidapi native library: NOT FOUND")
        if sys.platform == "darwin":
            console.print("    Install with: brew install hidapi")
        elif sys.platform == "linux":
            console.print("    Install with: sudo apt install libhidapi-dev")

    # Check the SILS host-bench toolchain (Windows: MinGW-w64; optional, WARN-only)
    # SILS ホストベンチのツールチェーン確認（Windows: MinGW-w64。任意、警告のみ）
    console.print()
    console.info("Checking SILS host toolchain...")
    _check_sils_toolchain(warnings)

    # Check serial ports
    console.print()
    console.info("Checking serial ports...")
    ports = platform.serial_ports()
    if ports:
        for port in ports:
            console.success(f"  {port}")
    else:
        if platform.is_wsl():
            console.warning("  No serial ports found")
            console.print("    WSL2 requires usbipd-win to access USB devices.")
            console.print("    See WSL2 section below for details.")
        else:
            console.print("  No serial ports found (connect device to see ports)")

    # Check WSL2 environment
    # WSL2環境チェック
    if platform.is_wsl():
        console.print()
        console.info("Checking WSL2...")
        console.success("  WSL2 environment detected")

        # Check usbipd-win availability
        # usbipd-win の存在確認
        try:
            result = subprocess.run(
                ["usbipd.exe", "--version"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                usbipd_ver = result.stdout.strip()
                console.success(f"  usbipd-win: {usbipd_ver}")
            else:
                warnings.append("usbipd-win not working properly")
                console.warning("  usbipd-win: NOT WORKING")
                console.print("    Install: winget install usbipd")
                console.print("    See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
        except FileNotFoundError:
            warnings.append("usbipd-win not found (required for USB device access in WSL2)")
            console.warning("  usbipd-win: NOT FOUND")
            console.print("    Install on Windows side: winget install usbipd")
            console.print("    Then: sudo apt install linux-tools-generic hwdata")
            console.print("    See: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
        except Exception:
            console.warning("  usbipd-win: could not check")

        # Check for Windows PATH pollution
        # Windows PATHの汚染チェック
        import os
        path_dirs = os.environ.get("PATH", "").split(":")
        mnt_paths = [p for p in path_dirs if p.startswith("/mnt/")]
        if mnt_paths:
            console.warning(f"  Windows PATH entries: {len(mnt_paths)} found")
            console.print("    These may cause issues with ESP-IDF tools.")
            console.print("    The installer filters these automatically during sfcli install.")

    # Check lesson manifest
    # レッスンマニフェスト検証
    console.print()
    console.info("Checking lesson manifest...")
    _check_manifest(warnings, issues)

    # Check StampFly configuration
    console.print()
    console.info("Checking StampFly configuration...")
    config_file = paths.root() / ".sf" / "config.toml"
    if config_file.exists():
        console.success(f"  Config: {config_file}")
    else:
        warnings.append("Configuration not found (run ./install.sh)")
        console.warning("  Config: NOT FOUND")
        console.print("    Run: ./install.sh")

    # Summary
    console.print()
    console.header("Summary")

    if issues:
        console.error(f"Found {len(issues)} error(s):")
        for issue in issues:
            console.print(f"  - {issue}")
        console.print()

    if warnings:
        console.warning(f"Found {len(warnings)} warning(s):")
        for warning in warnings:
            console.print(f"  - {warning}")
        console.print()

    if not issues and not warnings:
        console.success("All checks passed!")
        return 0
    elif issues:
        console.error("Environment has critical issues. Please fix before proceeding.")
        return 1
    else:
        console.warning("Environment has warnings but should work.")
        return 0
