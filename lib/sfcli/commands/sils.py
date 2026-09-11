"""
sf sils - Software-in-the-Loop bench (physics-based, MuJoCo, algorithm-independent)

物理ベースの SILS ベンチを操作する。ファームの本物ループをホストで走らせ、MuJoCo で
ループを閉じ、機械可読な合否(results.json)＋レビュー動画を成果物として出す。
アウトプット主導のマイルストーン(RESET_PLAN §8〜§10)を CLI から回す。

Subcommands:
  build      Build the host SILS (compat + firmware sources + MuJoCo). On Windows,
             auto-installs the MinGW-w64 toolchain first if missing (see
             install-toolchain).
  install-toolchain
             Windows only: install MinGW-w64 (winget, falling back to a direct
             download if winget fails — see lib/sfcli/utils/msys2_install.py)
  run        Run the closed loop and write the bundle (flight log + results.json)
  video      Render the review video (MuJoCo 3D + state graphs)
  status     Show the machine verdict (results.json) for a milestone
  gate       Gate check: bundle complete AND verdict passes (output-driven)
  scenario   Run a *.scn input scenario (console/ESP-NOW) and assert outputs (E6)
  regression Run every *.scn that has a matching *.expect and gate on the aggregate
             (the automated form of the manual A/B loop in release-workflow.md; CI
             entry point for sils-regression.yml)
  milestone  build → run → video → gate in one shot (the /sils-milestone skill)
  sysid-gate Model-match gate: fit SILS rate-loop (b,T,L) vs real-hardware sysid
             (simulation-policy.md §4; distinct from the "gate" milestone check)
  fly        Real-time keyboard-piloted SILS flight (P6 stage 1)
  compare    Side-by-side ESKF vs complementary video (P4/P6)
  gui        Launch the SILS Web GUI (scenarios, graphs, 3D playback)
"""

import argparse
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Optional

from ..utils import console, msys2_install, paths, platform

# Terminal raw mode (Unix only) — used by `sf sils fly`'s keyboard control,
# same guarded-import pattern as lib/sfcli/commands/rc.py.
# ターミナル生入力（Unix専用）— `sf sils fly` のキーボード操縦が使う。
# lib/sfcli/commands/rc.py と同じガード付きインポート。
try:
    import select
    import termios
    import tty
    _HAS_TERMIOS = True
except ImportError:
    _HAS_TERMIOS = False

COMMAND_NAME = "sils"
COMMAND_HELP = "Software-in-the-Loop bench (closed-loop hover, review video, gate)"

# Transition alias: "sil" was renamed to "sils" on 2026-08-07 (SILS
# terminology unification; SITL stays reserved for the ArduPilot/PX4
# tools). cli.py registers these as argparse aliases and prints a
# one-line notice when one is used. Remove after the 2026-09-10 SCI
# tutorial season.
# 移行用エイリアス: 2026-08-07 の SILS 用語統一で "sil" を "sils" に改名
# （SITL は ArduPilot/PX4 のツール名として予約）。cli.py が argparse の
# エイリアスとして登録し、使用時に1行の案内を表示する。2026-09-10 の
# SCI チュートリアル期が過ぎたら削除する。
LEGACY_COMMAND_ALIASES = ("sil",)

# Legacy environment-variable prefix: every SILS_* variable used to be
# SIL_* before the rename. promote_legacy_environment() below maps old
# names onto new ones so existing user scripts keep working.
# 旧環境変数プレフィクス: 改名前は SILS_* が全て SIL_* だった。下の
# promote_legacy_environment() が旧名を新名に写像し、既存のユーザー
# スクリプトを動かし続ける。
_LEGACY_ENV_PREFIX = "SIL_"


def promote_legacy_environment() -> None:
    """Map legacy SIL_* environment variables onto their SILS_* names.

    Called once from cli.py at startup. Only fills a SILS_* name that is
    not already set (an explicit new-name value always wins), and prints
    a one-line notice listing what was mapped so the user learns the new
    names. The emulator subprocesses inherit os.environ, so promoting
    here covers every sf entry point.
    旧 SIL_* 環境変数を SILS_* 名へ写像する。cli.py の起動時に1回呼ぶ。
    SILS_* 名が未設定の場合のみ埋める（明示された新名が常に優先）。
    何を写像したか1行で案内し、ユーザーが新名を覚えられるようにする。
    エミュレータのサブプロセスは os.environ を継承するため、ここで
    昇格すれば全ての sf 経路をカバーできる。
    """
    promoted = []
    for key in sorted(os.environ):
        if not key.startswith(_LEGACY_ENV_PREFIX) or key.startswith("SILS_"):
            continue
        new_key = "SILS_" + key[len(_LEGACY_ENV_PREFIX):]
        if new_key not in os.environ:
            os.environ[new_key] = os.environ[key]
            promoted.append(f"{key}->{new_key}")
    if promoted:
        console.warning(
            "Legacy SIL_* environment variable(s) mapped to SILS_* "
            f"({', '.join(promoted)}); please switch to the new names "
            "(renamed 2026-08-07)."
        )

ESTIMATORS = {"eskf": 0, "complementary": 1}
ESTIMATOR_LABELS = {"eskf": "ESKF", "complementary": "Complementary"}
NOISE_LEVELS = ["off", "n0", "n1", "n2"]

# Friendly firmware target name -> its emulator exe / CMake target name
# (CMakeLists.txt names every emulator "emu_<target>"). "workshop" runs the
# learner's own setup()/loop_400Hz() (main/user_code.cpp, copied by `sf lesson
# switch`) through the same reused vehicle sensor/state tasks as "vehicle" —
# see simulator/sils/CMakeLists.txt's emu_workshop comment.
# 親しみやすいファーム名 -> エミュレータ exe / CMake ターゲット名（CMakeLists.txt は
# 全エミュレータを "emu_<target>" と命名）。"workshop" は学習者自身の
# setup()/loop_400Hz()（`sf lesson switch` がコピーする main/user_code.cpp）を、
# "vehicle" と同じ再利用センサ/状態タスク経由で走らせる — CMakeLists.txt の
# emu_workshop コメント参照。
SILS_TARGETS = ("vehicle", "vehicle_old", "workshop")
_TARGET_EXE_NAME = {name: f"emu_{name}" for name in SILS_TARGETS}
# Default sensor-noise level per milestone (RESET_PLAN §13): noise milestones run N0.
# マイルストーン別の既定ノイズ（§13）: ノイズ系マイルストーンは N0 で走る。
MILESTONE_NOISE = {"P5": "n0", "P6": "n0", "P7": "n0"}

# CMake target/exe name for an embedded `sf app` project's SILS emulator: always
# emu_vehicle (the app's sources are compiled INTO vehicle's own main component
# via SF_APP_DIR, not a separate emulator target — sf-app-sils-plan.md Phase 1).
# 組み込み型 `sf app` プロジェクトの SILS エミュレータの CMake ターゲット/実行ファイル名:
# 常に emu_vehicle（app のソースは SF_APP_DIR 経由で vehicle 自身の main コンポーネントに
# 直接コンパイルされる。別のエミュレータターゲットではない）。
_APP_EMULATOR_TARGET = _TARGET_EXE_NAME["vehicle"]


# --- path helpers / パスヘルパ -------------------------------------------------
def _sils_dir() -> Path:
    return paths.root() / "simulator" / "sils"


def _model() -> Path:
    return _sils_dir() / "models" / "stampfly.xml"


def app_build_dir(name: str) -> Path:
    """Build directory for an embedded `sf app` project's own SILS emulator,
    kept separate per app (simulator/sils/build/apps/<name>) so switching
    between apps never shares or clobbers a CMake build cache (each app's
    SF_APP_DIR is baked into its own build dir's CMakeCache.txt at first
    configure — see build_app_emulator()).
    組み込み型 `sf app` プロジェクト自身の SILS エミュレータのビルドディレクトリ。
    app ごとに分離する（simulator/sils/build/apps/<name>）ことで、app を切り替えても
    CMake ビルドキャッシュを共有・汚染しない。
    """
    return _sils_dir() / "build" / "apps" / name


def _embedded_app_manifest_or_error(name: str):
    """Load firmware/apps/<name>/app.yaml and verify it declares `type:
    embedded`. Returns (manifest_dict, None) on success, or (None,
    error_message) on failure.

    Shared by `sils scenario --target apps/<name>`'s argparse `type=`
    validator (_resolve_sils_target) and `sils build --target apps/<name>`'s
    own hand-rolled check inside run_build() — `build -t/--target` has no
    choices=/type= restriction (it stays a free-form cmake target string,
    e.g. cores_smoke/hover_smoke/emu_vehicle), so it cannot use argparse
    type= the way `scenario --target` does.
    firmware/apps/<name>/app.yaml を読み `type: embedded` であることを確認する。
    成功時は (manifest_dict, None)、失敗時は (None, error_message) を返す。

    `sils scenario --target apps/<name>` の argparse type= バリデータ
    （_resolve_sils_target）と `sils build --target apps/<name>` 自身の手作り
    チェック（run_build() 内）の両方が共有する — `build -t/--target` は
    choices=/type= 制約を持たない（cores_smoke/hover_smoke/emu_vehicle 等の
    自由形式 cmake ターゲット文字列のまま）ため、`scenario --target` と同じ
    argparse type= は使えない。
    """
    manifest_path = paths.apps() / name / "app.yaml"
    if not manifest_path.exists():
        return None, f"no {manifest_path} -- run `sf app new {name}` first"
    try:
        import yaml
        with open(manifest_path, encoding="utf-8") as f:
            manifest = yaml.safe_load(f) or {}
    except Exception as exc:  # missing PyYAML, malformed YAML, ...
        return None, f"failed to read {manifest_path}: {exc}"
    app_type = manifest.get("type", "bench")
    if app_type != "embedded":
        return None, (
            f"app.yaml declares type={app_type!r}, not 'embedded' -- bench apps are "
            "standalone projects and cannot run inside the vehicle SILS emulator "
            "(use `sf app new <name> --from 11_app_controller` for an embedded app)"
        )
    return manifest, None


def _resolve_sils_target(value: str) -> str:
    """argparse `type=` for `sils scenario --target`: accepts a friendly
    firmware name (SILS_TARGETS) unchanged, or `apps/<name>` for an
    embedded-type `sf app` project (see _embedded_app_manifest_or_error()).
    Anything else raises ArgumentTypeError with the reason, mirroring
    argparse's own `choices=` error style — this replaces the plain
    `choices=list(SILS_TARGETS)` that used to reject `apps/<name>` outright,
    since that value is an open-ended set validated against the filesystem,
    not a fixed list.
    `sils scenario --target` の argparse type=: 親しみやすいファーム名
    （SILS_TARGETS）はそのまま、`apps/<name>` は組み込み型 `sf app`
    プロジェクトであれば受理する。それ以外は理由付きで ArgumentTypeError を
    送出する。
    """
    if value in SILS_TARGETS:
        return value
    if value.startswith("apps/"):
        name = value[len("apps/"):]
        _, error = _embedded_app_manifest_or_error(name)
        if error:
            raise argparse.ArgumentTypeError(f"'{value}': {error}")
        return value
    raise argparse.ArgumentTypeError(
        f"invalid choice: '{value}' (choose from {', '.join(SILS_TARGETS)}, or "
        "'apps/<name>' for an embedded `sf app` project)")


# Well-known MSYS2 MinGW-w64 install location (winget/MSYS2 installer default).
# The SILS host bench was validated against this exact toolchain (GCC 16,
# posix-thread model — see mingw_toolchain() below); other MinGW distributions
# are not verified.
# MSYS2 MinGW-w64 の既定インストール先（winget/MSYS2 インストーラの既定）。
# SILS ホストベンチはこのツールチェーン（GCC 16, posix スレッドモデル — 下記
# mingw_toolchain() 参照）で検証済み。他の MinGW ディストリビューションは未検証。
_MSYS2_MINGW64_BIN = Path("C:/msys64/mingw64/bin")


def mingw_bin() -> Optional[Path]:
    """Locate a MinGW-w64 toolchain's bin/ directory on Windows (need
    g++/gcc/ninja together — cmake itself can come from anywhere on PATH).
    Returns None on non-Windows, or on Windows when no MinGW toolchain is
    found (the SILS build then falls back to whatever generator/compiler CMake
    picks up from PATH by default, e.g. Visual Studio).

    Checked in order: (1) a g++ already on PATH whose path contains "mingw"
    (a user's own MSYS2/MinGW setup, respected as-is); (2) the MSYS2 default
    install path. Used both to configure the build (run_build) and to run the
    resulting .exe (its libstdc++-6.dll/libgcc_s_seh-1.dll/libwinpthread-1.dll
    are not on PATH otherwise — see win_run_env()).

    Windows で MinGW-w64 ツールチェーンの bin/ を探す（g++/gcc/ninja が揃って
    いる必要がある — cmake 自体は PATH 上のどこにあってもよい）。非 Windows、
    または Windows でも MinGW が見つからない場合は None（その場合 SILS ビルドは
    CMake が PATH から拾う既定のジェネレータ/コンパイラ、例えば Visual Studio に
    フォールバックする）。

    確認順序: (1) PATH 上に既にある g++ のパスに "mingw" を含む場合（ユーザー
    自身の MSYS2/MinGW 環境をそのまま尊重）、(2) MSYS2 既定インストール先。
    ビルド設定（run_build）と、出来上がった .exe の実行（libstdc++-6.dll 等が
    他に PATH 上に無い — win_run_env() 参照）の両方に使う。
    """
    if not platform.is_windows():
        return None
    on_path = shutil.which("g++")
    if on_path and "mingw" in on_path.lower():
        return Path(on_path).parent
    if (_MSYS2_MINGW64_BIN / "g++.exe").exists() and (_MSYS2_MINGW64_BIN / "ninja.exe").exists():
        return _MSYS2_MINGW64_BIN
    return None


def win_run_env(build_dir: Path) -> dict:
    """Environment for configuring/building/running the SILS on Windows via
    MinGW: PATH gets the MinGW bin/ prepended (compiler + the runtime DLLs
    every MinGW-built .exe needs: libstdc++-6.dll, libgcc_s_seh-1.dll,
    libwinpthread-1.dll) plus the build dir's own bin/ (libmujoco.dll — MuJoCo
    builds as a shared library there). A plain copy of os.environ (no-op) on
    non-Windows, or on Windows when mingw_bin() finds nothing (nothing to add).

    MinGW 経由で SILS を設定・ビルド・実行する Windows 向け環境: PATH の先頭に
    MinGW の bin/（コンパイラ＋ MinGW ビルドの exe が全て要る実行時 DLL:
    libstdc++-6.dll, libgcc_s_seh-1.dll, libwinpthread-1.dll）と、ビルドディレクトリ
    自身の bin/（libmujoco.dll — MuJoCo はそこに共有ライブラリとしてビルドされる）を
    足す。非 Windows、または Windows でも mingw_bin() が何も見つけない場合は
    os.environ のそのままのコピー（no-op、足すものが無い）。
    """
    env = dict(os.environ)
    mingw = mingw_bin()
    if mingw is None:
        return env
    extra_dirs = [str(mingw)]
    dll_dir = build_dir / "bin"
    if dll_dir.exists():
        extra_dirs.append(str(dll_dir))
    env["PATH"] = os.pathsep.join(extra_dirs + [env.get("PATH", "")])
    return env


def _build_dir() -> Path:
    # Windows: use a SEPARATE build directory when building with MinGW so a
    # pre-existing MSVC build/ (a different generator's CMakeCache — Visual
    # Studio project files, different ABI) is never touched/clobbered. Falls
    # back to the shared build/ when MinGW is not available (e.g. an existing
    # MSVC-only setup keeps working exactly as before).
    # Windows: MinGW でビルドする場合は別ディレクトリを使い、既存の MSVC build/
    # （別ジェネレータの CMakeCache — Visual Studio プロジェクトファイル、別ABI）
    # に触れない・壊さない。MinGW が無ければ共有の build/ にフォールバック
    # （既存の MSVC 専用環境はそのまま動き続ける）。
    if mingw_bin() is not None:
        return _sils_dir() / "build-mingw"
    return _sils_dir() / "build"


def _bundle_dir(milestone: str) -> Path:
    return _sils_dir() / "viz" / f"out_{milestone.lower()}"


def _venv_python() -> Path:
    bin_dir = "Scripts" if platform.is_windows() else "bin"
    exe = "python.exe" if platform.is_windows() else "python"
    return _sils_dir() / "viz" / "venv" / bin_dir / exe


def _exe(name: str) -> str:
    """Executable filename for `name` on this platform (CMake's own naming;
    adds .exe on Windows, bare name elsewhere).
    このプラットフォームでの実行ファイル名（CMake 自体の命名規則に従う。
    Windows は .exe を付け、それ以外は無印）。
    """
    return f"{name}.exe" if platform.is_windows() else name


# --- build freshness advisory / ビルド鮮度の参考警告 ---------------------------
# Directory names to prune while walking for sources (build outputs, venvs,
# caches — none of these feed the compiled binary, and build/ alone holds
# ~19k files in simulator/sils/). Kept as a set (not a glob) so os.walk can
# prune dirnames in place without re-descending into a pruned subtree.
# ソース走査時に剪定するディレクトリ名（ビルド成果物・venv・キャッシュ — どれも
# コンパイル済みバイナリを構成しない。simulator/sils/ だけで build/ 配下は
# 約19000ファイル）。glob ではなく集合にして os.walk が dirnames をその場で
# 剪定し、剪定済みサブツリーに再度降りないようにする。
_SILS_STALE_SKIP_DIRS = {"build", "build-mingw", "venv", ".cache",
                       "managed_components", "__pycache__", ".git"}
_SILS_STALE_EXTS = (".cpp", ".hpp", ".c", ".h")


def _sils_source_mtime() -> float:
    """Latest mtime among simulator/sils/ and firmware/vehicle/ source files that
    feed the SILS binary (Code Identity: SILS links firmware/vehicle sources
    directly — see architecture.md). Restricted to .cpp/.hpp/.c/.h/CMakeLists.txt
    and walked with build/venv/cache dirs pruned in-place (os.walk, not
    Path.rglob — rglob would still descend into an excluded subtree before
    discarding it) to keep the scan cheap: ~600 files, well under 0.1 s, versus
    the ~19k files under simulator/sils/build/ alone if unfiltered.
    SILSバイナリを構成するソースの最新mtime（simulator/sils/・firmware/vehicle/。
    Code Identity: SILS は firmware/vehicle のソースを直接リンクする）。
    .cpp/.hpp/.c/.h/CMakeLists.txt に限定し、build/venv/cache 等を os.walk で
    その場剪定（Path.rglob は除外対象のサブツリーにも一旦降りてから捨てるため
    不利）して走査コストを抑える: 約600ファイル・0.1秒未満（build/ 単体で
    約19000ファイルある無フィルタ走査に対して）。
    """
    latest = 0.0
    for base in (paths.root() / "simulator" / "sils", paths.root() / "firmware" / "vehicle"):
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d not in _SILS_STALE_SKIP_DIRS]
            for name in filenames:
                if name == "CMakeLists.txt" or os.path.splitext(name)[1] in _SILS_STALE_EXTS:
                    try:
                        latest = max(latest, os.path.getmtime(os.path.join(dirpath, name)))
                    except OSError:
                        pass  # file vanished mid-walk (e.g. concurrent edit) — ignore
    return latest


def _check_build_freshness(exe: Path, src_mtime: Optional[float] = None) -> bool:
    """Warn (never blocks — this is advisory, not a build gate) if `exe` predates
    the newest tracked SILS/firmware source file: a likely sign the sources were
    edited after the last `sf sils build`. Returns True iff a warning was printed,
    so callers with more than one exe (run_regression: vehicle + vehicle_old) can
    share a single source scan (pass a precomputed src_mtime) instead of walking
    the tree once per exe.
    `exe` が最新の SILS/ファームソースより古ければ警告する（ブロックはしない —
    あくまで参考情報。ビルドゲートではない）。最後の `sf sils build` の後にソースが
    編集された可能性のサイン。警告を出したら True を返す — 複数 exe を持つ呼び出し
    元（run_regression: vehicle と vehicle_old）が exe ごとに再走査せず、1回の
    ソース走査結果（src_mtime を渡す）を使い回せるようにする。
    """
    try:
        exe_mtime = exe.stat().st_mtime
    except OSError:
        return False
    if src_mtime is None:
        src_mtime = _sils_source_mtime()
    if src_mtime > exe_mtime:
        # console.warning() already prints the "[WARN]" prefix itself (see
        # lib/sfcli/utils/console.py) — do not repeat it in the message text.
        # console.warning() は自身で "[WARN]" 接頭辞を付ける（console.py 参照）
        # — メッセージ本文で二重に付けない。
        console.warning(f"SILS binary is older than sources — run `sf sils build` "
                       f"({exe.name})")
        return True
    return False


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the sils command with the CLI."""
    parser = subparsers.add_parser(COMMAND_NAME, help=COMMAND_HELP, description=__doc__,
                                   aliases=list(LEGACY_COMMAND_ALIASES),
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="sils_command", metavar="<subcommand>")

    p = sub.add_parser("build", help="Build the host SILS")
    p.add_argument("-j", "--jobs", type=int, default=8)
    p.add_argument("-t", "--target", default=None,
                   help="cmake target (default: all). A friendly firmware name "
                        f"({', '.join(SILS_TARGETS)}) is mapped to its emu_<name> "
                        "CMake target; apps/<name> builds an embedded-type `sf app` "
                        "project's own emulator; any other value is passed through "
                        "as-is (e.g. cores_smoke, hover_smoke, emu_vehicle).")
    p.add_argument("-y", "--yes", action="store_true",
                   help="Windows only: don't prompt before auto-installing the "
                        "MinGW-w64 toolchain if it is missing")
    p.add_argument("--no-auto-toolchain", action="store_true",
                   help="Windows only: never auto-install MinGW-w64; just warn if missing")
    p.add_argument("--winget", action="store_true",
                   help="Windows only: try winget install --id MSYS2.MSYS2 first (not the "
                        "default — MSYS2's own docs don't list winget as a supported "
                        "install method; falls back to the direct download either way)")
    p.set_defaults(func=run_build)

    p = sub.add_parser("install-toolchain",
                        help="Install the MinGW-w64 toolchain needed to build the SILS on "
                             "Windows (direct download by default; --winget to try winget first)")
    p.add_argument("-y", "--yes", action="store_true", help="Don't prompt for confirmation")
    p.add_argument("--winget", action="store_true",
                   help="Try winget install --id MSYS2.MSYS2 first (not the default — MSYS2's "
                        "own docs don't list winget as a supported install method; falls back "
                        "to the direct download either way)")
    p.set_defaults(func=run_install_toolchain)

    p = sub.add_parser("run", help="Run the closed loop and write the bundle")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("-e", "--estimator", choices=list(ESTIMATORS), default="eskf")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off", help="sensor noise level (§13)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.set_defaults(func=run_run)

    p = sub.add_parser("video", help="Render the review video for a milestone bundle")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("--fps", type=int, default=50)
    p.set_defaults(func=run_video)

    p = sub.add_parser("status", help="Show the machine verdict (results.json)")
    p.add_argument("-m", "--milestone", default="P1")
    p.set_defaults(func=run_status)

    p = sub.add_parser("gate", help="Gate check: bundle complete and verdict passes")
    p.add_argument("-m", "--milestone", default="P1")
    p.set_defaults(func=run_gate)

    # E6: run a deterministic *.scn input scenario and assert the firmware output.
    # E6: 決定論的な *.scn 入力シナリオを走らせ、ファーム出力をアサートする。
    p = sub.add_parser("scenario", help="Run a *.scn input scenario and assert outputs (E6)")
    p.add_argument("scenario", help="path to the .scn scenario file")
    p.add_argument("--target", type=_resolve_sils_target, default="vehicle",
                   help="emulator binary (default: vehicle = current firmware; "
                        "vehicle_old = legacy firmware; workshop = learner "
                        "setup()/loop_400Hz() from `sf lesson switch`, on the "
                        "same reused vehicle sensor/state tasks; apps/<name> = an "
                        "embedded-type `sf app` project, built on demand)")
    p.add_argument("--expect", default=None,
                   help="assertions file (default: <scenario>.expect if it exists)")
    p.add_argument("--duration", type=int, default=25_000_000,
                   help="sim duration in microseconds (default 25 s)")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off",
                   help="sensor noise level on the emulator Plant (§13 P5; default off)")
    p.add_argument("--seed", type=int, default=12345,
                   help="noise RNG seed (determinism: same seed → byte-identical run)")
    p.add_argument("--video", action="store_true",
                   help="on PASS, render a review MP4 (MuJoCo 3D + state graphs) from the run")
    p.add_argument("--ground-effect", nargs="?", const="1", default=None, metavar="GAIN",
                   help="enable the plant ground-effect model (near-floor lift boost) so the "
                        "touchdown 'float' is reproduced. Bare flag = default strength; pass a "
                        "number to set the gain (e.g. --ground-effect 0.4). Default OFF "
                        "(byte-identical clean path).")
    p.add_argument("--turbulence", nargs="?", const="0.03", default=None, metavar="AMP_N",
                   help="enable a deterministic 1-3 Hz lateral turbulence force [N] to excite "
                        "the attitude-wobble band (wobble-minimization study). Bare flag = 0.03 N. "
                        "Default OFF.")
    p.add_argument("--motor-delay", type=float, default=None, metavar="MS",
                   help="motor transport delay in ms (model-match retrofit #1: real-hardware "
                        "system ID measured L=14.7/8.4/11.0 ms roll/pitch/yaw vs the current "
                        "SILS's ~0 ms explicit dead time, docs/architecture/simulation-policy.md "
                        "backlog #1). Inserted in the duty-path before the motor's first-order "
                        "lag. Default OFF (0 ms, byte-identical clean path).")
    p.add_argument("--thrust-eff", type=float, default=None, metavar="RATIO",
                   help="override the plant's real-vs-ideal thrust efficiency (Plant::Config::"
                        "thrust_efficiency; scales the MEAN of all 4 motor thrusts, so it cuts "
                        "net vertical/hover authority too — a 0.4-0.7x cut was found to stall "
                        "takeoff/hover outright, see --torque-authority for the differential-"
                        "only knob used by the hikoki64 §3.3 study). Default OFF (Config "
                        "default 1.0, byte-identical).")
    p.add_argument("--torque-authority", type=float, default=None, metavar="RATIO",
                   help="override the plant's roll/pitch DIFFERENTIAL torque authority "
                        "(Plant::Config::torque_authority; scales only the per-motor thrust "
                        "deviation from the 4-motor mean, so NET vertical thrust/hover "
                        "authority is exactly unaffected). Used by the hikoki64 §3.3 SILS "
                        "gain-deficit injection study to model the in-flight-identified "
                        "~0.4-0.7x motor torque effectiveness without also cutting net thrust "
                        "(unlike --thrust-eff, which was found to stall takeoff at this "
                        "magnitude). Default OFF (Config default 1.0, byte-identical).")
    p.add_argument("--flow-scale", type=float, default=None, metavar="RATIO",
                   help="override the plant's optical-flow velocity under-read model "
                        "(Plant::Config::flow_vel_scale; multiplies synthesized flow dx/dy "
                        "counts). Used by the hikoki64 §3.3 SILS gain-deficit injection study "
                        "to model the in-flight-identified ~0.66x flow velocity under-read. "
                        "Default OFF (Config default 1.0, byte-identical).")
    p.add_argument("--unpaired", action="store_true",
                   help="boot the vehicle UNPAIRED (skip the SILS pairing NVS seed) so it "
                        "auto-enters Pairing and binds via the injected RC — exercises the "
                        "real pairing handshake (pairing.scn)")
    p.add_argument("--param", action="append", dest="params", metavar="NAME=VALUE",
                   help="override a firmware param for this run only (repeatable, e.g. "
                        "--param rate.roll.kp=0.5e-3). Written to a temp file wired through "
                        "SILS_EMU_PARAMS_FILE; unknown names are skipped by the emulator "
                        "itself with a warning (SSOT stays in firmware's params table).")
    p.set_defaults(func=run_scenario)

    # P6 stage 1: real-time, keyboard-piloted SILS flight. Runs emu_vehicle with
    # SILS_EMU_REALTIME (paces the deterministic scheduler to the wall clock) +
    # SILS_EMU_RC_STDIN (accepts live 'rc'/'arm'/'land'/'quit' lines — see
    # simulator/sils/devices/rc_stdin.cpp) and renders its 'STATE ...' HUD line.
    # P6 stage 1: リアルタイム・キーボード操縦SILS飛行。emu_vehicle を
    # SILS_EMU_REALTIME（決定論スケジューラを壁時計にペーシング）＋
    # SILS_EMU_RC_STDIN（ライブ 'rc'/'arm'/'land'/'quit' 行を受理 — rc_stdin.cpp
    # 参照）付きで起動し、'STATE ...' HUD行を描画する。
    p = sub.add_parser("fly", help="Real-time keyboard-piloted SILS flight (P6 stage 1)",
                       description=(
                           "Fly the real, unmodified vehicle firmware in the SILS from the "
                           "terminal keyboard, in real time.\n"
                           "See simulator/sils/README.md's `sf sils fly` section for the key "
                           "map and the ARM-by-keyboard sequence."
                       ))
    p.add_argument("--scenario", default=None,
                   help="optional .scn to run first (e.g. to reach ARMED_GROUND) before "
                        "keyboard control takes over. Wait for it to finish before touching "
                        "the sticks — see README for the ARM-by-keyboard alternative")
    p.add_argument("--param", action="append", dest="params", metavar="NAME=VALUE",
                   help="override a firmware param for this session only (repeatable, e.g. "
                        "--param rate.roll.kp=0.5e-3). Same mechanism as "
                        "`sf sils scenario --param` — see its help for details.")
    p.add_argument("--duration", type=int, default=600,
                   help="max session length in seconds (default 600 = 10 min). The "
                        "emulator process ends on its own if you exceed this before "
                        "landing/quitting yourself")
    p.set_defaults(func=run_fly)

    # Regression: run every *.scn that has a matching *.expect (README/TEST_MATRIX
    # "32 scenarios"; the .expect glob is authoritative) and gate on the aggregate.
    # This is the CI/pre-release entry point (sils-regression.yml, versioning.md §5).
    # 退行: .expect を伴う全 *.scn を実行し集約判定でゲートする(README/TEST_MATRIX
    # の「32本」。.expect グロブが正)。CI・リリース前のエントリポイント。
    p = sub.add_parser("regression", help="Run all *.scn/*.expect scenarios and gate (CI)")
    p.add_argument("--json-out", default=None,
                   help="write a machine-readable summary (per-scenario pass/fail) to this path")
    p.add_argument("--include-workshop", action="store_true",
                   help="also run workshop-target scenarios (default: SKIPPED — they "
                        "depend on whatever firmware/workshop/main/user_code.cpp "
                        "currently holds, which `sf lesson switch` owns and a fresh "
                        "checkout's CMake bootstrap seeds with Lesson 0 (drives no "
                        "motors), so a liftoff gate fails by construction there, not "
                        "from a real regression. Run `sf lesson switch N --solution` "
                        "first, then pass this flag)")
    p.set_defaults(func=run_regression)

    p = sub.add_parser("compare", help="Side-by-side ESKF vs complementary video (P4/P6)")
    p.add_argument("-m", "--milestone", default="P4")
    p.add_argument("--ea", choices=list(ESTIMATORS), default="eskf", help="run A estimator")
    p.add_argument("--eb", choices=list(ESTIMATORS), default="complementary", help="run B estimator")
    p.add_argument("--noise", choices=NOISE_LEVELS, default=None, help="sensor noise (default per milestone)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (same for both runs → fair)")
    p.add_argument("--fps", type=int, default=50)
    p.set_defaults(func=run_compare)

    # Web GUI: scenario authoring + runs + interactive graphs + live 3D playback.
    # Web GUI: シナリオ作成＋実行＋インタラクティブグラフ＋ライブ 3D 再生。
    p = sub.add_parser("gui", help="Launch the SILS Web GUI (scenarios, graphs, 3D playback)")
    p.add_argument("--port", type=int, default=8765, help="HTTP port (default 8765)")
    p.add_argument("--no-browser", action="store_true",
                   help="do not auto-open the browser")
    p.set_defaults(func=run_gui)

    p = sub.add_parser("milestone", help="build → run → video → gate in one shot")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("-e", "--estimator", choices=list(ESTIMATORS), default="eskf")
    # default None → resolved per-milestone (MILESTONE_NOISE) unless the user overrides.
    # 既定 None → ユーザー指定が無ければマイルストーン別(MILESTONE_NOISE)で解決。
    p.add_argument("--noise", choices=NOISE_LEVELS, default=None, help="sensor noise level (§13)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.set_defaults(func=run_milestone)

    # Model-match gate (docs/architecture/simulation-policy.md §4, backlog #0): run
    # simulator/sils/scenarios/sysid_gate.scn with the 400Hz rate-loop recorder on,
    # fit (b,T,L) per axis with the SAME code used on real-hardware logs
    # (tools/log_analyzer/rate_sysid.py), and compare against the real-hardware
    # reference (analysis/reports/rate_sysid_reference/reference.json). Named
    # "sysid-gate", NOT "gate" — that name is already the milestone bundle
    # verdict check (run_gate above); the two are unrelated concepts.
    # モデル一致ゲート（simulation-policy.md §4、backlog #0）: sysid_gate.scn を
    # 400Hzレートループ記録付きで実行し、実機ログに使ったのと同一コード（rate_sysid.py）
    # で軸別 (b,T,L) をフィットして実機基準値と比較する。"gate" ではなく "sysid-gate" と
    # 命名 — "gate" は既にマイルストーン束バンドル判定（上の run_gate）で使用済みの
    # 別概念のため。
    p = sub.add_parser("sysid-gate",
                       help="Model-match gate: fit SILS rate-loop (b,T,L) vs real-hardware "
                            "sysid (simulation-policy.md §4)")
    p.add_argument("--motor-delay", type=float, default=None, metavar="MS",
                   help="motor transport delay in ms, passed through to the scenario run "
                        "(see `sf sils scenario --motor-delay`). Default OFF (0 ms).")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off",
                   help="sensor noise level on the emulator Plant (default off — a clean "
                        "excitation run is what the real-hardware sysid pipeline assumes)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.add_argument("--duration", type=int, default=44_000_000,
                   help="sim duration in microseconds (default 44s, matches sysid_gate.scn)")
    p.add_argument("--param", action="append", dest="params", metavar="NAME=VALUE",
                   help="override a firmware param for this run only (repeatable, e.g. "
                        "--param rate.roll.kp=0.5e-3). Same mechanism as "
                        "`sf sils scenario --param` — see its help for details.")
    p.add_argument("--report", default=None,
                   help="write the gate verdict + per-axis fit JSON here")
    p.set_defaults(func=run_sysid_gate)

    parser.set_defaults(func=lambda a: (parser.print_help(), 0)[1])


# --- handlers / ハンドラ -------------------------------------------------------
def run_install_toolchain(args: argparse.Namespace) -> int:
    """`sf sils install-toolchain`: install MinGW-w64 (direct MSYS2 download
    by default; `--winget` tries winget first instead) so `sf sils build`
    has a compiler that can build this codebase (MSVC cannot — see
    simulator/sils/README.md).
    `sf sils install-toolchain`: MinGW-w64 を導入する（既定は MSYS2 の直接
    ダウンロード。`--winget` を渡すと先に winget を試す）。`sf sils build`
    がこのコードベースをビルドできるコンパイラを持つようにする
    （MSVC では不可 — simulator/sils/README.md 参照）。"""
    if not platform.is_windows():
        console.info("Not Windows — the SILS build uses the system gcc/clang here; no toolchain install needed.")
        return 0

    found = mingw_bin()
    if found is not None:
        console.success(f"MinGW-w64 toolchain already installed: {found}")
        return 0

    installed = msys2_install.install_toolchain(
        assume_yes=getattr(args, "yes", False),
        prefer_winget=getattr(args, "winget", False),
    )
    if installed is None:
        return 1
    console.success(f"MinGW-w64 toolchain installed: {installed}")
    return 0


def _sils_generator_flags(build_dir: Path) -> list:
    """Extra `-G`/`-D` cmake configure flags chosen on a build dir's FIRST
    configure only (Windows: MinGW-w64 → Ninja+GCC/G++; Linux/macOS: Ninja if
    on PATH). Shared by run_build() and build_app_emulator() so an app's SILS
    build picks the same toolchain/generator as every other SILS build. Once
    CMakeCache.txt exists the generator/compiler are already locked in, so a
    reconfigure returns no flags (re-passing -G would error on a generator
    mismatch).
    ビルドディレクトリの初回 configure でのみ選ばれる追加の `-G`/`-D` フラグ
    （Windows: MinGW-w64 の Ninja+GCC/G++、Linux/macOS: PATH にあれば Ninja）。
    run_build() と build_app_emulator() の両方が共有し、app の SILS ビルドも
    他の SILS ビルドと同じツールチェーン/ジェネレータを選ぶ。CMakeCache.txt が
    既にあれば空リストを返す（再設定で -G を渡し直すとジェネレータ不一致で
    エラーになるため）。
    """
    if (build_dir / "CMakeCache.txt").exists():
        return []
    mingw = mingw_bin()
    if mingw is not None:
        console.info(f"Windows: configuring for MinGW-w64 ({mingw})")
        return ["-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release",
                "-DCMAKE_C_COMPILER=gcc", "-DCMAKE_CXX_COMPILER=g++"]
    if not platform.is_windows() and shutil.which("ninja"):
        console.info("Configuring with Ninja (found on PATH)")
        return ["-G", "Ninja"]
    return []


def run_build(args: argparse.Namespace) -> int:
    # getattr defaults so the milestone flow (which lacks -j/-t) can reuse this.
    # milestone フローは -j/-t を持たないので getattr で既定値化して再利用できるようにする。
    jobs = getattr(args, "jobs", 8)
    target = getattr(args, "target", None)

    # apps/<name>: build an embedded-type `sf app` project's own emulator
    # (build_app_emulator()) instead of the friendly-target build below. `-t/
    # --target` has no choices=/type= restriction (free-form cmake target
    # string), so apps/<name> is validated by hand here rather than by
    # argparse — see docs/plans/sf-app-sils-plan.md Phase 2.
    # apps/<name>: 以下の通常ターゲットビルドではなく、組み込み型 `sf app`
    # プロジェクト自身のエミュレータ（build_app_emulator()）をビルドする。
    # `-t/--target` は choices=/type= 制約を持たない（自由形式の cmake
    # ターゲット文字列）ため、apps/<name> はここで手動検証する。
    if target and str(target).startswith("apps/"):
        name = target[len("apps/"):]
        _, error = _embedded_app_manifest_or_error(name)
        if error:
            console.error(f"'{target}': {error}")
            return 1
        exe = build_app_emulator(paths.apps() / name, app_build_dir(name), jobs)
        return 0 if exe.exists() else 1

    # Map a friendly firmware name (vehicle/vehicle_old/workshop) onto its
    # actual CMake target (emu_<name>); any other value (cores_smoke,
    # hover_smoke, an explicit emu_vehicle, ...) passes through unchanged.
    # 親しみやすいファーム名を実際の CMake ターゲット（emu_<name>）へ写像する。
    # それ以外の値（cores_smoke・hover_smoke・明示的な emu_vehicle 等）はそのまま。
    target = _TARGET_EXE_NAME.get(target, target)

    # Windows has no usable C++17 compiler without MinGW-w64 (MSVC rejects
    # designated initializers the firmware relies on — README's "Windows
    # native build" section). Auto-install it here rather than letting the
    # build silently fall through to MSVC and fail later with a confusing
    # compiler error. `--no-auto-toolchain` opts out (just warn); `--yes`
    # skips the confirmation prompt (for CI/scripts).
    # Windows には MinGW-w64 無しで使える C++17 コンパイラが無い（MSVC は
    # ファームが使う指定初期化子を拒否する — README の「Windows ネイティブ
    # ビルド」節）。ビルドを黙って MSVC に流し後で分かりにくいコンパイル
    # エラーで失敗させるより、ここで自動導入する。`--no-auto-toolchain` で
    # 無効化（警告のみ）、`--yes` で確認プロンプトを省略（CI/スクリプト向け）。
    if (platform.is_windows() and mingw_bin() is None
            and not getattr(args, "no_auto_toolchain", False)):
        console.warning("MinGW-w64 not found — MSVC cannot build the SILS (see simulator/sils/README.md).")
        if msys2_install.install_toolchain(
            assume_yes=getattr(args, "yes", False),
            prefer_winget=getattr(args, "winget", False),
        ) is None:
            console.warning("Proceeding without MinGW-w64; CMake will fall back to whatever "
                             "compiler it finds on PATH (likely MSVC, which will fail to build).")
    sd, bd = _sils_dir(), _build_dir()
    env = win_run_env(bd)

    # Resolve "cmake" against env["PATH"] (not just check it's findABLE) rather
    # than passing the bare name to subprocess.run: on Windows, CreateProcess's
    # own executable search uses the CALLING process's PATH, not the PATH
    # inside the env= block handed to the child — so a bare "cmake" would
    # raise WinError 2 (file not found) whenever MinGW's cmake.exe is on
    # win_run_env()'s PATH but not on sf CLI's own process PATH (the common
    # case: MinGW is installed but nothing added it to the user/system PATH).
    # "cmake" を env["PATH"] に対して解決してから使う（存在確認だけでなく）:
    # Windows では CreateProcess 自身の実行ファイル探索は「呼び出し元プロセスの
    # PATH」を使い、子に渡す env= ブロック内の PATH は使わない —
    # そのため win_run_env() の PATH には MinGW の cmake.exe があっても sf CLI
    # 自身のプロセス PATH に無ければ（MinGW 導入済みだが誰も PATH に足していない
    # ケースは普通に起こる）素の "cmake" は WinError 2 になる。
    cmake_exe = shutil.which("cmake", path=env.get("PATH")) or "cmake"

    cmake_cmd = [cmake_exe, "-S", str(sd), "-B", str(bd)] + _sils_generator_flags(bd)

    console.info("Configuring SILS (cmake)...")
    r = subprocess.run(cmake_cmd, env=env)
    if r.returncode != 0:
        console.error("cmake configure failed"); return r.returncode
    console.info("Building SILS (first time fetches MuJoCo — can take minutes)...")
    cmd = [cmake_exe, "--build", str(bd), "-j", str(jobs)]
    if target:
        cmd += ["--target", target]
    r = subprocess.run(cmd, env=env)
    if r.returncode == 0:
        console.success("SILS build OK")
    return r.returncode


def build_app_emulator(app_dir: Path, build_dir: Path, jobs: int = 8) -> Path:
    """Configure+build an embedded `sf app` project's own SILS emulator: the
    same base configure as `sf sils build` (MinGW/Ninja chosen on a build
    dir's first configure — see _sils_generator_flags()), plus -DSF_APP_DIR
    pointing CMake at the app's own sources (simulator/sils/CMakeLists.txt's
    SF_APP_DIR handling), building the emu_vehicle target into `build_dir`
    (kept separate per app — see app_build_dir() — so switching between apps
    never shares/clobbers a build cache). Returns the built exe's path; the
    caller checks `.exists()` to detect a failed build (this function already
    prints its own success/error via console).

    組み込み型 `sf app` プロジェクト自身の SILS エミュレータを configure+build
    する: `sf sils build` と同じ基本 configure（初回のみ MinGW/Ninja選択、
    _sils_generator_flags() 参照）に -DSF_APP_DIR を追加し（CMake に app 自身の
    ソースを指す）、`build_dir`（app ごとに分離。app_build_dir() 参照 —
    app を切り替えてもビルドキャッシュを共有・汚染しない）に emu_vehicle
    ターゲットをビルドする。ビルド済み exe のパスを返す。失敗の検出は
    呼び出し側が `.exists()` で行う（成功/失敗は本関数が自前で console
    出力済み）。
    """
    paths.ensure_dir(build_dir)
    env = win_run_env(build_dir)
    cmake_exe = shutil.which("cmake", path=env.get("PATH")) or "cmake"
    exe = build_dir / _exe(_APP_EMULATOR_TARGET)

    cmake_cmd = ([cmake_exe, "-S", str(_sils_dir()), "-B", str(build_dir)]
                 + _sils_generator_flags(build_dir)
                 + [f"-DSF_APP_DIR={app_dir.resolve()}"])
    console.info(f"Configuring SILS for app '{app_dir.name}' (cmake)...")
    r = subprocess.run(cmake_cmd, env=env)
    if r.returncode != 0:
        console.error("cmake configure failed")
        return exe

    console.info(f"Building SILS emulator ({_APP_EMULATOR_TARGET}) for app '{app_dir.name}'...")
    cmd = [cmake_exe, "--build", str(build_dir), "-j", str(jobs), "--target", _APP_EMULATOR_TARGET]
    r = subprocess.run(cmd, env=env)
    if r.returncode == 0:
        console.success(f"SILS build OK: {exe}")
    else:
        console.error("SILS build failed")
    return exe


def _events_jsonl_to_csv(events_jsonl: Path, out_csv: Path) -> None:
    """Convert the scenario's events.jsonl (simulator/sils/devices/emu_record.cpp
    — one JSON object per line: {"t_us","ch","n","bytes"} for an injected byte
    stream, or {"t_us","ch","note"} for a text note) into the flight-log
    bundle's events.csv stream (`timestamp_us,event,value` —
    protocol/spec/flight_log.yaml). `event` is the channel name (`ch`); `value`
    is the rest of the payload rendered as one compact string (a byte-stream
    line becomes "n=<len> bytes=<hex>", a note line is just its text) —
    pandas quotes it correctly on write regardless of embedded commas/quotes.
    シナリオの events.jsonl（emu_record.cpp。1行1JSON: バイト列は
    {"t_us","ch","n","bytes"}、テキスト注記は {"t_us","ch","note"}）を、フライト
    ログ一式の events ストリーム（`timestamp_us,event,value`）へ変換する。
    `event` はチャネル名（`ch`）、`value` は残りのペイロードを1つのコンパクトな
    文字列にしたもの（バイト列は "n=<長さ> bytes=<16進>"、注記はそのままの文
    字列）— カンマ・引用符が混ざっても pandas が書き出し時に正しく引用する。
    """
    import pandas as pd

    rows = []
    for line in events_jsonl.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        ts, ch = rec.get("t_us"), rec.get("ch")
        if ts is None or ch is None:
            continue
        if "note" in rec:
            value = rec["note"]
        else:
            value = f"n={rec.get('n', 0)} bytes={rec.get('bytes', '')}"
        rows.append({"timestamp_us": int(ts), "event": ch, "value": value})
    pd.DataFrame(rows, columns=["timestamp_us", "event", "value"]).to_csv(out_csv, index=False)


def _finalize_flightlog(flightlog_dir: Path, zip_path: Path, *, notes: str,
                        events_jsonl: Optional[Path] = None) -> Optional[Path]:
    """Assemble one `*.sflog.zip` StampFly flight-log v1 bundle from the CSV
    directory an emulator run just wrote via SILS_EMU_FLIGHTLOG=<flightlog_dir>
    (simulator/sils/devices/emu_flightlog*.cpp — imu/attitude/posvel/rate_ref/
    motor/ctrl_output/ctrl_ref/pilot/status/truth CSVs + a gains.json sidecar),
    plus the scenario's own events.jsonl (scripted-input log,
    simulator/sils/devices/emu_record.cpp), converted here to events.csv.

    Steps: (a) events.jsonl -> events.csv, if given and non-empty; (b) load
    the directory with sflog.FlightLog.load() (unknown files such as
    gains.json are ignored by the loader — see lib/sflog/bundle.py), set
    `.meta`/`.schema`, and save to `zip_path`; (c) append gains.json into the
    zip as an extra member (bypassing the loader) so `sysid-gate` can read
    it back — see simulator/sils/devices/emu_flightlog_vehicle.cpp's
    sils_emu_flightlog_write_gains(); (d) remove `flightlog_dir`. Any stale
    `*.sflog.zip` already in `zip_path`'s directory is removed first, so
    exactly one bundle remains per run directory (docs/plans/
    flight-log-format-plan.md section 2).

    Returns `zip_path`, or None (after a console.error) if `flightlog_dir`
    holds no stream CSV at all (e.g. the emulator crashed before writing
    anything, or SILS_EMU_FLIGHTLOG was never wired for this target) — in
    that case nothing is deleted, so a previous run's bundle is left intact.

    `flightlog_dir`（SILS_EMU_FLIGHTLOG で指定したディレクトリ。エミュレータが
    直接書いた CSV 一式）と、変換した events.csv から `*.sflog.zip` を1個
    組み立てる。CSV が1つも無ければ何もせず None を返す（前回の有効な zip は
    残す）。手順は英語側参照。
    """
    if events_jsonl is not None and events_jsonl.exists() and events_jsonl.stat().st_size > 0:
        _events_jsonl_to_csv(events_jsonl, flightlog_dir / "events.csv")

    if not flightlog_dir.exists() or not any(flightlog_dir.glob("*.csv")):
        console.error(f"no flight-log CSV written to {flightlog_dir} — nothing to bundle "
                      "(is this target wired for SILS_EMU_FLIGHTLOG?)")
        return None

    # Exactly one bundle per run directory: clear any stale zip(s) from a
    # previous run of this same scenario/milestone before writing the new one.
    # 実行ディレクトリごとに一式は1個だけ: 新しい zip を書く前に前回実行の
    # 残骸を消す。
    for stale in zip_path.parent.glob("*.sflog.zip"):
        stale.unlink()

    import sflog

    log = sflog.FlightLog.load(flightlog_dir)
    log.meta = sflog.make_meta(source="sils", tool_name="sf sils",
                               tool_version=sflog.__version__, notes=notes,
                               streams=log.streams)
    log.schema = sflog.schema.schema_for(log.streams.keys())
    log.save(zip_path)

    gains_path = flightlog_dir / "gains.json"
    if gains_path.exists():
        with zipfile.ZipFile(zip_path, "a", zipfile.ZIP_DEFLATED) as zf:
            zf.write(gains_path, arcname="gains.json")

    shutil.rmtree(flightlog_dir, ignore_errors=True)
    return zip_path


def run_run(args: argparse.Namespace) -> int:
    bd = _build_dir()
    exe = bd / _exe("hover_smoke")
    if not exe.exists():
        console.error("hover_smoke not built — run 'sf sils build' first"); return 1
    bundle = _bundle_dir(args.milestone)
    bundle.mkdir(parents=True, exist_ok=True)
    et = ESTIMATORS[args.estimator]
    noise = getattr(args, "noise", "off") or "off"
    seed = getattr(args, "seed", 12345)
    console.info(f"Running closed loop (milestone={args.milestone}, "
                 f"estimator={args.estimator}, noise={noise})...")
    # argv: model, bundle, estimator_type, milestone, noise_level, seed.
    # 引数: モデル, バンドル, 推定器種別, マイルストーン, ノイズ準位, シード。
    r = subprocess.run([str(exe), str(_model()), str(bundle), str(et),
                        str(args.milestone), noise, str(seed)], env=win_run_env(bd))
    # hover_smoke writes its own flight-log v1 CSVs directly under
    # <bundle>/flightlog/ (argv[2]=bundle — see simulator/sils/smoke/
    # hover_smoke.cpp); bundle them into one *.sflog.zip here.
    # hover_smoke は自分で <bundle>/flightlog/ 配下にフライトログ一式 CSV を
    # 書く（argv[2]=bundle）。ここで1個の *.sflog.zip にまとめる。
    zip_path = _finalize_flightlog(
        bundle / "flightlog",
        bundle / f"sils_hover_{args.milestone.lower()}_{datetime.now():%Y%m%dT%H%M%S}.sflog.zip",
        notes=f"milestone={args.milestone} estimator={args.estimator} noise={noise} seed={seed}",
    )
    if r.returncode == 0:
        console.success(f"Bundle written to {bundle}")
        if zip_path is not None:
            console.print(f"  flight log: {zip_path}")
    else:
        console.error(f"Closed loop FAILED (exit {r.returncode}) — see output / results.json")
    return 0  # the verdict lives in results.json; gate decides pass/fail


def _quat_to_euler(qw, qx, qy, qz):
    """Roll/pitch/yaw [rad] from a body->NED quaternion (aerospace ZYX Euler
    convention — matches protocol/spec/flight_log.yaml's quat_w/x/y/z columns
    used by both truth.csv and attitude.csv). Vectorized: qw/qx/qy/qz may be
    numpy arrays, in which case the return values are arrays of the same
    shape.
    body->NED クォータニオンから roll/pitch/yaw [rad] を求める（航空機の
    ZYX オイラー角規約。truth.csv/attitude.csv 共通の quat_w/x/y/z 列に対応）。
    ベクトル化対応: qw/qx/qy/qz が numpy 配列なら戻り値も同じ形の配列になる。
    """
    import numpy as np
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


# Cache of loaded FlightLog bundles, keyed by str(bundle_path) — so a single
# .expect file with several `metric` lines reads/unzips the bundle only once
# (see _bundle_metric()). A load failure is cached too (as None) so a broken
# bundle is not re-attempted for every metric line.
# 読み込み済み FlightLog のキャッシュ（str(bundle_path) をキーに）— 1つの
# .expect が複数の `metric` 行を持っていても一式を1回しか読み込まない
# （_bundle_metric() 参照）。読み込み失敗も None としてキャッシュし、壊れた
# 一式を metric 行の数だけ再試行しない。
_BUNDLE_METRIC_CACHE: dict = {}


def _load_bundle_for_metric(bundle_path: Path):
    key = str(bundle_path)
    if key not in _BUNDLE_METRIC_CACHE:
        import sflog
        try:
            _BUNDLE_METRIC_CACHE[key] = sflog.FlightLog.load(bundle_path)
        except (OSError, KeyError, ValueError, zipfile.BadZipFile) as e:
            console.warning(f"could not load flight-log bundle {bundle_path}: {e}")
            _BUNDLE_METRIC_CACHE[key] = None
    return _BUNDLE_METRIC_CACHE[key]


# Angular metrics: _bundle_metric() returns these in RADIANS (SI, per
# protocol/spec/flight_log.yaml) — _eval_expect() appends a "(x deg)" hint to
# the check detail for these names so a human reading `sf sils scenario`'s
# console output doesn't have to convert in their head.
# 角度系メトリクス: _bundle_metric() はこれらをラジアン（SI、flight_log.yaml
# 準拠）で返す — _eval_expect() はこれらの名前に限り検査結果の detail に
# 「(x deg)」の換算値を添え、コンソール出力を見る人が暗算しなくて済むようにする。
_ANGULAR_METRICS = {"roll_rmse", "pitch_rmse", "att_rmse", "tilt_max", "yaw_band"}


def _bundle_metric(bundle_path: Optional[Path], name: str, t0=None, t1=None):
    """Compute a physical-truth metric from a StampFly flight-log v1 bundle
    for the numerical gates (G2 estimate tracking, G3 bounded attitude/
    position, G4 actuator health). The expect DSL is log-only (≈G1); this
    turns the bundle's truth+estimate streams into machine-judgeable
    numbers, all in SI units (rad/rad/s/m — protocol/spec/flight_log.yaml;
    replaces the old degrees-based trajectory.csv this function superseded).
    Returns None on a missing bundle / missing stream / unknown metric /
    empty window. Optional [t0, t1] SECONDS (mapped onto the bundle's
    absolute virtual-clock `timestamp_us` as [t0*1e6, t1*1e6]) restrict the
    metric to a flight phase (e.g. the POS_HOLD window).

    Estimate-vs-truth metrics (roll_rmse/pitch_rmse/att_rmse/alt_rmse) match
    each truth row to the nearest estimate row within 5 ms
    (pandas.merge_asof(..., direction="nearest", tolerance=5000)) — truth
    rows outside that tolerance of any estimate sample are dropped, not
    zero-filled, since a held/interpolated value would not be a real
    observation (docs/plans/flight-log-format-plan.md's "no fill values"
    rule).

    StampFly フライトログ v1 一式から物理真値メトリクスを算出する（G2/G3/G4）。
    旧 degrees 単位の trajectory.csv を置き換え、全て SI 単位（rad/rad/s/m）で
    返す。一式が無い/対象ストリームが無い/未知のメトリクス/窓が空のときは
    None。[t0, t1] は秒（絶対仮想クロック timestamp_us の [t0*1e6, t1*1e6] に
    写像）。推定値対真値系メトリクスは真値の各行を5ms以内の最近傍推定値行に
    照合し（tolerance=5000us）、それより離れた行は0埋めせず捨てる（一次記録に
    埋め値を持たせない方針に合わせる）。
    """
    if bundle_path is None:
        return None
    import numpy as np
    import pandas as pd

    log = _load_bundle_for_metric(bundle_path)
    if log is None or "truth" not in log.streams:
        return None

    def _window(df):
        if t0 is None or t1 is None:
            return df
        return df[(df["timestamp_us"] >= t0 * 1e6) & (df["timestamp_us"] <= t1 * 1e6)]

    truth = _window(log.streams["truth"]).sort_values("timestamp_us")
    if truth.empty:
        return None
    roll, pitch, yaw = _quat_to_euler(truth["quat_w"].to_numpy(), truth["quat_x"].to_numpy(),
                                       truth["quat_y"].to_numpy(), truth["quat_z"].to_numpy())
    alt = -truth["pos_z"].to_numpy()

    if name == "horizontal_drift_max":   # G3: max planar distance from the window's start
        px, py = truth["pos_x"].to_numpy(), truth["pos_y"].to_numpy()
        return float(np.max(np.hypot(px - px[0], py - py[0])))
    if name == "tilt_max":                # G3: max true tilt magnitude (no tumble)
        return float(np.max(np.hypot(roll, pitch)))
    if name == "alt_band":                # G3: peak-to-peak altitude over the window
        return float(np.max(alt) - np.min(alt))
    if name == "alt_mean":
        return float(np.mean(alt))
    if name == "alt_min":
        return float(np.min(alt))
    if name == "alt_max":
        return float(np.max(alt))
    if name == "yaw_band":                # G3: peak-to-peak true heading over the window
        # (heading-hold gate). Unwrapped so a continuous rotation is not
        # hidden by the +-pi seam.
        # 窓内の真値方位 p-p（ヘディングホールド用ゲート）。連続回転が ±π の
        # 継ぎ目で隠れないようアンラップする。
        yaw_unwrapped = np.unwrap(yaw)
        return float(np.max(yaw_unwrapped) - np.min(yaw_unwrapped))
    if name == "duty_max":                # G4: peak motor duty (saturation guard)
        motor = log.streams.get("motor")
        if motor is None:
            return None   # target not wired for motor recording at all -- genuinely unknown
        motor = _window(motor)
        cols = [c for c in ("duty_FR", "duty_RR", "duty_RL", "duty_FL") if c in motor.columns]
        if not cols:
            return None
        if motor.empty:
            # motor.csv holds one row per control cycle that actually RAN (the
            # armed window; emu_flightlog_vehicle.cpp) -- unlike the old
            # trajectory.csv, which recorded an explicit 0.0 duty every cycle
            # even while disarmed. Zero rows in [t0, t1] therefore means "no
            # control cycle ran here" (never armed / motors never driven in
            # this window), which is the SAME fact the old format encoded as
            # duty==0.0 -- e.g. pairing.expect's crosstalk-rejection window
            # asserts `duty_max < 0.05` while the foreign transmitter's ARM is
            # dropped and the vehicle is never actually armed.
            # motor.csv は実際に「走った」制御周期だけ1行を持つ（armed window。
            # emu_flightlog_vehicle.cpp）—— disarm中も明示的に duty==0.0 を
            # 毎周期記録していた旧 trajectory.csv とは違う。[t0, t1] に行が
            # 0件ということは「この窓では制御周期が一度も走らなかった」
            # （一度も arm されていない/モータを駆動していない）ことを意味し、
            # 旧形式が duty==0.0 で表していたのと同じ事実 —— 例えば
            # pairing.expect の混信拒否窓は、誤送信機の ARM が破棄され機体が
            # 実際には arm されない間 `duty_max < 0.05` をアサートする。
            return 0.0
        return float(motor[cols].to_numpy().max())
    if name in ("roll_rmse", "pitch_rmse", "att_rmse"):   # G2: est attitude vs truth
        attitude = log.streams.get("attitude")
        if attitude is None:
            return None
        a = attitude.sort_values("timestamp_us")
        a_roll, a_pitch, _ = _quat_to_euler(a["quat_w"].to_numpy(), a["quat_x"].to_numpy(),
                                             a["quat_y"].to_numpy(), a["quat_z"].to_numpy())
        est = pd.DataFrame({"timestamp_us": a["timestamp_us"].to_numpy(),
                            "roll_est": a_roll, "pitch_est": a_pitch})
        truth_small = pd.DataFrame({"timestamp_us": truth["timestamp_us"].to_numpy(),
                                    "roll": roll, "pitch": pitch})
        merged = pd.merge_asof(truth_small, est, on="timestamp_us",
                               direction="nearest", tolerance=5000).dropna(
                                   subset=["roll_est", "pitch_est"])
        if merged.empty:
            return None
        if name == "roll_rmse":
            return float(np.sqrt(np.mean((merged["roll_est"] - merged["roll"]) ** 2)))
        if name == "pitch_rmse":
            return float(np.sqrt(np.mean((merged["pitch_est"] - merged["pitch"]) ** 2)))
        return float(np.sqrt(np.mean(np.hypot(merged["roll_est"] - merged["roll"],
                                               merged["pitch_est"] - merged["pitch"]) ** 2)))
    if name == "alt_rmse":                # G2: est alt vs truth alt
        posvel = log.streams.get("posvel")
        if posvel is None:
            return None
        p = posvel.sort_values("timestamp_us")
        est = pd.DataFrame({"timestamp_us": p["timestamp_us"].to_numpy(),
                            "alt_est": -p["pos_z"].to_numpy()})
        truth_small = pd.DataFrame({"timestamp_us": truth["timestamp_us"].to_numpy(), "alt": alt})
        merged = pd.merge_asof(truth_small, est, on="timestamp_us",
                               direction="nearest", tolerance=5000).dropna(subset=["alt_est"])
        if merged.empty:
            return None
        return float(np.sqrt(np.mean((merged["alt_est"] - merged["alt"]) ** 2)))
    return None  # unknown metric name / 未知のメトリクス名


_METRIC_OPS = {
    "<":  lambda a, b: a < b,   "<=": lambda a, b: a <= b,
    ">":  lambda a, b: a > b,   ">=": lambda a, b: a >= b,
}


def _read_xfail(expect_path: Path) -> Optional[str]:
    """Return the reason string from a leading 'xfail: <reason>' directive in
    an .expect file, or None if the file carries no such marker. A known-fail
    marker documents that THIS scenario's current failure is a real, tracked
    firmware/plant-fidelity issue (see docs/architecture/simulation-policy.md
    backlog) rather than a broken assertion — `sf sils regression` reports it
    as [KNOWN-FAIL] instead of counting it against the gate, and flips to
    [XPASS] (gate failure) if the scenario starts passing while the marker is
    still there, so a fixed issue can't silently go untracked.

    Directive placement: must be the FIRST non-blank, non-'#'-comment line in
    the file (i.e. before "exit 0" / any assertion) — this keeps the scanner
    trivial (stop at the first such line) and matches "reserved header slot"
    rather than a marker that could be confused with prose deeper in the file.

    .expect ファイル先頭の 'xfail: <理由>' 指令行から理由文字列を返す（無ければ
    None）。既知失敗マーカーは「このシナリオの現在の失敗はファーム/プラント忠実度の
    既知の追跡課題であり、壊れたアサーションではない」ことを明示する
    （docs/architecture/simulation-policy.md のバックログ参照）。`sf sils regression`
    はこれをゲート対象外の [KNOWN-FAIL] として報告し、マーカーが残ったまま合格に
    転じたら [XPASS]（ゲート失敗）にして直った課題の見逃しを防ぐ。

    配置規則: ファイル内で最初の非空行・非 '#' コメント行でなければならない
    （"exit 0" 等のアサーションより前）。走査を単純にし（最初の該当行で確定）、
    ファイル深部の文章と紛れない「予約されたヘッダ位置」という扱いにする。
    """
    for raw in expect_path.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("xfail:"):
            return stripped[len("xfail:"):].strip()
        return None  # first real line is not xfail: → no marker
    return None


def _eval_expect(expect_path: Path, out_text: str, err_text: str, exit_code: int,
                 bundle_path: Path = None):
    """Evaluate an assertions file against the captured output. Returns
    (checks, all_pass, xfail_reason). Assertions are anchored to OUTPUT
    TEXT/ORDER, never wall-clock, so a check is deterministic exactly because
    the scenario's output is byte-identical across runs. 出力テキスト/順序に
    アンカー（壁時計不使用）＝決定論的。

      xfail: <reason>                         known-fail marker (see _read_xfail);
                                              not an assertion — parsed out here so
                                              it never becomes a "bad assertion"
      log_contains <out|err|any> "<text>"   stream contains the text
      log_absent   <out|err|any> "<text>"   stream does NOT contain the text
      order "<a>" "<b>"                       a's first occurrence precedes b's
      exit <code>                             process exit code matches
      skip <reason...>                        record a capability-gated check as
                                              SKIPPED (passes, not evaluated) —
                                              e.g. stable hover gated on E3 INA3221
      metric <name> <op> <value> [in <t0> <t1>]
                                              numerical physical-truth gate from the
                                              StampFly flight-log v1 bundle (G2/G3/G4),
                                              in SI units (rad/m — see _bundle_metric()).
                                              op ∈ < <= > >= ; optional "in t0 t1"
                                              restricts to a phase. names:
                                              horizontal_drift_max, roll_rmse,
                                              pitch_rmse, att_rmse, alt_rmse, tilt_max,
                                              alt_band, alt_mean, alt_min, alt_max,
                                              duty_max, yaw_band
    """
    merged = out_text + err_text
    streams = {"out": out_text, "err": err_text, "any": merged}
    checks = []
    xfail_reason = _read_xfail(expect_path)
    for raw in expect_path.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("xfail:"):
            continue  # known-fail marker, not an assertion — see _read_xfail()
        if stripped.startswith("skip"):
            reason = stripped[len("skip"):].strip()
            checks.append({"name": f"skipped: {reason}", "pass": True, "skipped": True,
                           "detail": "capability-gated; not evaluated"})
            continue
        # Quote-aware tokenize that also strips an UNQUOTED trailing '#' comment,
        # so a '#' inside a quoted assertion text survives. A malformed line is
        # recorded as a failing check rather than crashing the command.
        # 引用符を尊重したトークン化（引用符外の '#' のみコメント除去）。不正行はクラッシュ
        # させず失敗チェックとして記録する。
        try:
            toks = shlex.split(raw, comments=True)
        except ValueError as e:
            checks.append({"name": f"bad assertion: {stripped!r}", "pass": False, "detail": str(e)})
            continue
        if not toks:
            continue
        kind = toks[0]
        if kind in ("log_contains", "log_absent") and len(toks) >= 3:
            stream, text = toks[1], toks[2]
            found = text in streams.get(stream, merged)
            ok = found if kind == "log_contains" else not found
            checks.append({"name": f"{kind} {stream} {text!r}", "pass": bool(ok),
                           "detail": "found" if found else "absent"})
        elif kind == "order" and len(toks) >= 3:
            a, b = toks[1], toks[2]
            ia, ib = merged.find(a), merged.find(b)
            ok = ia >= 0 and ib >= 0 and ia < ib
            checks.append({"name": f"order {a!r} before {b!r}", "pass": bool(ok),
                           "detail": f"idx_a={ia} idx_b={ib}"})
        elif kind == "exit" and len(toks) >= 2:
            try:
                want = int(toks[1])
            except ValueError:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": "non-numeric exit code"})
                continue
            checks.append({"name": f"exit == {want}", "pass": exit_code == want,
                           "detail": f"got {exit_code}"})
        elif kind == "metric" and len(toks) >= 4:
            # metric <name> <op> <value> [in <t0> <t1>]
            # Numerical physical-truth gate from the flight-log bundle (G2/G3/G4).
            # The optional "in <t0> <t1>" window restricts it to a flight phase.
            # フライトログ一式からの数値ゲート。"in t0 t1" で飛行フェーズに限定。
            name, op, valstr = toks[1], toks[2], toks[3]
            t0 = t1 = None
            if len(toks) >= 7 and toks[4] == "in":
                try:
                    t0, t1 = float(toks[5]), float(toks[6])
                except ValueError:
                    checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                                   "detail": "non-numeric window"}); continue
            if op not in _METRIC_OPS:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": f"unknown op {op!r}"}); continue
            try:
                want = float(valstr)
            except ValueError:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": "non-numeric threshold"}); continue
            m = _bundle_metric(bundle_path, name, t0, t1) if bundle_path else None
            win = f" in [{t0},{t1}]" if t0 is not None else ""
            if m is None:
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": False,
                               "detail": "no flight-log bundle / unknown metric / empty window"})
            else:
                ok = _METRIC_OPS[op](m, want)
                # Angular metrics are SI radians (flight_log.yaml) — append a "(x deg)"
                # hint so a human reading the console output need not convert by hand.
                # 角度系メトリクスは SI のラジアン — 人間が読む際に暗算しなくて済むよう
                # 「(x deg)」の換算値を添える。
                detail = f"{name}={m:.4f}"
                if name in _ANGULAR_METRICS:
                    detail += f" ({math.degrees(m):.2f} deg)"
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": bool(ok),
                               "detail": detail})
        else:
            checks.append({"name": f"bad assertion: {stripped!r}", "pass": False, "detail": "syntax"})

    # An .expect that evaluates NO real assertion (empty / all-comment / all-skip)
    # must FAIL, not pass vacuously (all([]) is True in Python).
    # 実評価が1つも無い .expect は空虚 PASS にせず FAIL させる（all([])==True 対策）。
    evaluated = [c for c in checks if not c.get("skipped")]
    if not evaluated:
        checks.append({"name": "no assertions evaluated", "pass": False,
                       "detail": "expect file has no evaluable assertion"})
    all_pass = all(c["pass"] for c in checks)
    return checks, all_pass, xfail_reason


def run_scenario(args: argparse.Namespace) -> int:
    target = getattr(args, "target", "vehicle")

    # apps/<name>: build (or incrementally rebuild — CMake is idempotent)
    # this embedded `sf app` project's own emulator, rather than requiring a
    # pre-built exe like every other target below (see
    # docs/plans/sf-app-sils-plan.md Phase 2). _resolve_sils_target() already
    # validated apps/<name> at argparse parse time.
    # apps/<name>: 他の全ターゲットのようにビルド済み exe を要求せず、この
    # 組み込み型 `sf app` プロジェクト自身のエミュレータをビルド（または
    # 差分再ビルド — CMake は冪等）する。apps/<name> の妥当性は argparse の
    # 解析時点で _resolve_sils_target() が検証済み。
    if str(target).startswith("apps/"):
        name = target[len("apps/"):]
        exe = build_app_emulator(paths.apps() / name, app_build_dir(name),
                                  jobs=getattr(args, "jobs", 8))
        if not exe.exists():
            console.error(f"SILS build failed for app '{name}' — see cmake/build output above")
            return 1
    else:
        bd = _build_dir()
        exe = bd / _exe(_TARGET_EXE_NAME.get(target, f"emu_{target}"))
        if not exe.exists():
            console.error(f"{exe.name} not built — run 'sf sils build' first"); return 1
        # Build-freshness advisory (skippable — see SF_SILS_SKIP_FRESHNESS_CHECK): warn if
        # this exe predates its own sources. run_regression sets the env var after doing
        # this check ONCE up front, so its 30+ per-scenario subprocess calls into this
        # function don't each re-warn (or re-walk the source tree) redundantly.
        # ビルド鮮度の参考警告（SF_SILS_SKIP_FRESHNESS_CHECK でスキップ可）: この exe が
        # 自身のソースより古ければ警告する。run_regression は冒頭で1回だけ判定した後に
        # この環境変数を立てるので、以降30本超のシナリオ用子プロセスがこの関数を呼んでも
        # 重複警告・重複走査は起きない。
        if not os.environ.get("SF_SILS_SKIP_FRESHNESS_CHECK"):
            _check_build_freshness(exe)

    scn = Path(args.scenario)
    if not scn.exists():
        console.error(f"scenario not found: {scn}"); return 1

    return run_scenario_with_exe(exe, scn, args)


def run_scenario_with_exe(exe: Path, scenario: Path, args: argparse.Namespace) -> int:
    """Run one *.scn scenario against an already-built `exe`, apply its
    .expect assertions, and return the PASS/FAIL exit convention (0 PASS / 2
    FAIL) — the part of `run_scenario` that does not care how `exe` was
    resolved (a friendly-target build vs. an `apps/<name>` emulator freshly
    built by build_app_emulator()). `run_scenario` itself now just resolves
    `exe` and the scenario path, then calls this; behavior for every
    existing target is unchanged.
    既にビルド済みの `exe` に対して1本の *.scn シナリオを実行し .expect の
    アサーションを適用、合否の終了コード規約（0 PASS / 2 FAIL）を返す —
    exe がどう解決されたか（従来ターゲットのビルド vs build_app_emulator()
    で作った apps/<name> エミュレータ）を気にしない部分。`run_scenario` は
    exe とシナリオパスを解決してこれを呼ぶだけになった。既存ターゲットの
    挙動は変わらない。
    """
    scn = scenario
    target = getattr(args, "target", "vehicle")
    bd = exe.parent  # build dir this exe lives in (win_run_env's DLL search dir)

    bundle = _sils_dir() / "viz" / f"out_scn_{scn.stem}"
    bundle.mkdir(parents=True, exist_ok=True)
    events = bundle / "events.jsonl"
    flightlog_dir = bundle / "flightlog"
    # SILS_EMU_FLIGHTLOG tells the emulator to record a StampFly flight-log v1
    # bundle (imu/attitude/posvel/rate_ref/motor/ctrl_output/ctrl_ref/pilot/
    # status/truth CSVs + gains.json) directly into this directory — the
    # single artifact --video, `sf sysid`, and `sf log *` all read. Harmless
    # and deterministic; the recorder is a no-op in any emulator that ignores it.
    # SILS_EMU_FLIGHTLOG は StampFly フライトログ v1 一式をこのディレクトリへ
    # 直接記録させる — --video・`sf sysid`・`sf log *` が読む唯一の成果物。
    # 決定論的で無害、未対応エミュレータでは no-op。
    env = dict(win_run_env(bd), SILS_EMU_EVENTS=str(events),
              SILS_EMU_FLIGHTLOG=str(flightlog_dir))

    # SILS_EMU_NOISE/SILS_EMU_SEED turn on the seeded N0 sensor-noise model on the
    # emulator Plant (§13 P5). Default "off" → env passed but the emulator keeps the
    # clean path (byte-identical), so the no-noise scenario verdict is unaffected.
    # SILS_EMU_NOISE/SILS_EMU_SEED でエミュレータ Plant の N0 ノイズを ON（§13 P5）。
    # 既定 "off" では従来のクリーン経路（byte-identical）を保つ。
    noise = getattr(args, "noise", "off") or "off"
    seed = getattr(args, "seed", 12345)
    env["SILS_EMU_NOISE"] = noise
    env["SILS_EMU_SEED"] = str(seed)

    # --ground-effect enables the plant's near-floor lift boost (default OFF → clean path
    # byte-identical) so the touchdown "float" is reproduced and the firmware stalled-descent
    # land detector can be verified. Bare flag = default strength; a number sets the gain.
    # --ground-effect でプラントの接地近傍揚力ブーストを有効化（既定 OFF→クリーン経路バイト一致）。
    # 着陸「フロート」を再現しファームの降下停滞接地検出器を検証。素のフラグ=既定強度、数値で gain 指定。
    ge = getattr(args, "ground_effect", None)
    if ge is not None:
        env["SILS_EMU_GROUND_EFFECT"] = str(ge)

    # --turbulence enables a deterministic 1-3 Hz lateral disturbance (force [N]) to excite
    # the attitude-wobble band for the wobble-minimization study. Default OFF.
    tb = getattr(args, "turbulence", None)
    if tb is not None:
        env["SILS_EMU_TURBULENCE"] = str(tb)

    # --motor-delay sets the duty-path transport delay (model-match retrofit #1: real-hw
    # identified L=14.7/8.4/11.0ms roll/pitch/yaw vs the SILS's current ~0ms explicit dead
    # time). Default OFF (byte-identical clean path).
    # --motor-delay で duty 経路の輸送遅れを設定（モデル一致改修#1）。既定 OFF。
    md = getattr(args, "motor_delay", None)
    if md is not None:
        env["SILS_EMU_MOTOR_DELAY"] = str(md)

    # --thrust-eff / --flow-scale: hikoki64 §3.3 SILS gain-deficit injection study —
    # override the plant's torque-authority and flow-velocity-under-read knobs
    # (Plant::Config::thrust_efficiency / flow_vel_scale, emu_main.cpp). Default OFF
    # (byte-identical clean path unless explicitly passed).
    # --thrust-eff / --flow-scale: hikoki64 §3.3 SILSゲイン欠損注入実験 — プラントの
    # トルク効き・フロー速度過小読みノブを上書き。既定 OFF（明示指定しない限りバイト一致）。
    te = getattr(args, "thrust_eff", None)
    if te is not None:
        env["SILS_EMU_THRUST_EFF"] = str(te)
    ta = getattr(args, "torque_authority", None)
    if ta is not None:
        env["SILS_EMU_TORQUE_AUTHORITY"] = str(ta)
    fsc = getattr(args, "flow_scale", None)
    if fsc is not None:
        env["SILS_EMU_FLOW_SCALE"] = str(fsc)

    # --param NAME=VALUE (repeatable): temporary, single-run firmware param overrides,
    # routed through the EXISTING SILS_EMU_PARAMS_FILE mechanism (emu_main.cpp reads
    # "<name> <value>" lines from a text file and applies each through the SSOT params
    # table's own type-correct setter, warning+skipping unknown names itself — see
    # simulator/sils/emu/emu_main.cpp). This CLI option is a thin wrapper: parse
    # "NAME=VALUE" tokens into that line format, write them into THIS scenario's OWN
    # bundle directory (not a tempfile — easier to inspect while debugging a run, and
    # nothing to clean up afterward), and point SILS_EMU_PARAMS_FILE at it. No parameter
    # name validation happens here — that stays the emulator's job, so the params table
    # remains the single source of truth (no duplicate allowlist to drift).
    # --param NAME=VALUE（繰り返し指定可）: 既存の SILS_EMU_PARAMS_FILE 機構を通した、
    # 単発実行限りの一時的なファームパラメータ上書き（emu_main.cpp が "<名前> <値>" 行の
    # テキストファイルを読み、SSOT パラメータテーブル自身の型正しいセッタで適用し、未知の
    # 名前は自身で警告してスキップする — simulator/sils/emu/emu_main.cpp 参照）。この CLI
    # オプションは薄いラッパーに過ぎない: "NAME=VALUE" トークンをその行形式に変換し、
    # このシナリオ自身のバンドルディレクトリ（tempfile ではなく — 実行内容をデバッグ時に
    # 確認しやすく、後始末も不要）に書き込み、SILS_EMU_PARAMS_FILE をそこへ向ける。名前の
    # 妥当性検証はここでは行わない（エミュレータの役目のまま — パラメータテーブルを唯一の
    # 正に保ち、重複した許可リストで陳腐化させない）。
    params_list = getattr(args, "params", None) or []
    if params_list:
        valid_lines = []
        for raw_param in params_list:
            if "=" not in raw_param:
                console.warning(f"--param {raw_param!r} — missing '=', expected NAME=VALUE, skipped")
                continue
            name, _, value = raw_param.partition("=")
            valid_lines.append(f"{name.strip()} {value.strip()}")
        # An externally-set SILS_EMU_PARAMS_FILE (e.g. from the SILS GUI's own parameter
        # panel, or a wrapping script) would otherwise be silently clobbered below —
        # say so explicitly rather than leaving a confusing "my env var was ignored".
        # 外部で既に設定済みの SILS_EMU_PARAMS_FILE（SILS GUI のパラメータパネルや、
        # 呼び出し元スクリプト等）は以下で無言のまま上書きされてしまう — 「自分の環境
        # 変数が無視された」という混乱を避けるため、ここで明示的に表示する。
        if env.get("SILS_EMU_PARAMS_FILE"):
            console.info("--param overrides externally-set "
                         f"SILS_EMU_PARAMS_FILE={env['SILS_EMU_PARAMS_FILE']}")
        override_path = bundle / "params_override.txt"
        override_path.write_text(
            "\n".join(valid_lines) + ("\n" if valid_lines else ""), encoding="utf-8")
        env["SILS_EMU_PARAMS_FILE"] = str(override_path)
        console.info(f"{len(valid_lines)} param(s) queued via --param")

    # --unpaired: skip the pairing NVS seed so the vehicle boots unpaired and runs the
    # real pairing handshake (auto-enter Pairing → bind via injected RC). Default keeps
    # the vehicle pre-paired so flight scenarios are not gated on pairing timing.
    # --unpaired: ペアリング NVS seed をスキップし機体を未ペア起動させ、実ハンドシェイク
    # （自動 Pairing 突入→注入 RC で bind）を走らせる。既定はペア済み起動で飛行シナリオを
    # ペアリングのタイミングに依存させない。
    if getattr(args, "unpaired", False):
        env["SILS_EMU_UNPAIRED"] = "1"

    # extra_env: additional env vars for sils subcommands BUILT ON TOP of run_scenario
    # (e.g. `sf app`'s own Namespace construction, lib/sfcli/commands/app.py). Not part
    # of the `scenario` CLI surface itself — plain argparse.Namespace from run_scenario's
    # own parser never sets this. No built-in `sils` subcommand needs it any more now
    # that every scenario records a flight-log bundle unconditionally (formerly used by
    # `sysid-gate` to request the now-retired rate-loop CSV recorder).
    # extra_env: run_scenario の上に構築される他の sils サブコマンド用の追加 env
    # （例: `sf app` 自身の Namespace 構築、app.py）。`scenario` 自体の CLI 引数では
    # ない。全シナリオが無条件にフライトログ一式を記録するようになった今、組み込みの
    # `sils` サブコマンドはどれも使わない（かつては `sysid-gate` が今は廃止した
    # レートループ CSV レコーダを要求するのに使っていた）。
    extra_env = getattr(args, "extra_env", None)
    if extra_env:
        env.update(extra_env)

    console.info(f"Running scenario {scn.name} on {exe.name} ({args.duration} us, "
                 f"noise={noise}, seed={seed})...")
    # The emulator reads stdin (the firmware CLI); feed /dev/null so any non-key
    # read yields EAGAIN. Capture stdout and stderr SEPARATELY (ESP_LOGx → stderr).
    # ファーム CLI の stdin は /dev/null。stdout/stderr を分離捕捉（ESP_LOGx は stderr）。
    # encoding="utf-8" (not the platform default, e.g. cp932 on Japanese
    # Windows): the emulator's own log lines are UTF-8 (bilingual EN/JA
    # comments and messages throughout the firmware), which is not valid
    # cp932 and would otherwise crash the subprocess module's own stdout/
    # stderr reader threads with UnicodeDecodeError before r.stdout is even
    # populated. errors="replace" keeps a single stray non-UTF-8 byte from
    # doing the same.
    # encoding="utf-8"（プラットフォーム既定、例えば日本語 Windows の cp932 では
    # ない）: エミュレータ自身のログ行は UTF-8（ファーム全体の英日併記コメント・
    # メッセージ）で cp932 として不正なため、指定しないと subprocess モジュール
    # 自身の stdout/stderr 読み取りスレッドが r.stdout に値が入る前に
    # UnicodeDecodeError でクラッシュする。errors="replace" は迷い込んだ単発の
    # 非 UTF-8 バイトでも同様に落ちないようにする。
    run_started_at = datetime.now()
    with open(os.devnull) as devnull:
        r = subprocess.run([str(exe), str(_model()), str(args.duration), str(scn)],
                           stdin=devnull, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True, encoding="utf-8", errors="replace", env=env)
    (bundle / "console.out").write_text(r.stdout, encoding="utf-8")
    (bundle / "console.err").write_text(r.stderr, encoding="utf-8")
    (bundle / "console.log").write_text(r.stdout + r.stderr, encoding="utf-8")

    # --param visibility: the emulator's own PARAMS_FILE log lines (emu_main.cpp —
    # the "N param(s) overridden" summary and any "unknown param"/"rejected" lines)
    # would otherwise only land in bundle/console.log, invisible on the terminal.
    # Surface them here too, ONLY when --param was actually used (params_list
    # non-empty), so the normal (no --param) path gets no extra noise.
    # --param の可視化: エミュレータ自身の PARAMS_FILE ログ行（emu_main.cpp の
    # 「N個上書き」サマリ、"unknown param"/"rejected" 行）は放置すると
    # bundle/console.log にしか残らずターミナルから見えない。--param を実際に
    # 使った場合のみ（params_list が非空のときだけ）ここで転送する — 通常経路
    # （--param 不使用）には余計な出力を増やさない。
    if params_list:
        for line in r.stdout.splitlines():
            if "SILS_EMU_PARAMS_FILE=" in line:
                console.info(f"  {line}")
            elif "PARAMS_FILE: unknown param" in line:
                console.warning(f"  {line}")
            elif "PARAMS_FILE:" in line and "rejected" in line:
                console.warning(f"  {line}")

    # Guardrail: every scenario injects at least one event, so events.jsonl must
    # exist and be non-empty. A target that is not wired for scripted input (e.g.
    # an emulator entry that ignores argv[3]/SILS_EMU_EVENTS) would inject NOTHING
    # and could otherwise masquerade as a passing run on exit==0 — fail it loudly.
    # ガードレール: シナリオは必ず1事象以上注入するので events.jsonl は非空のはず。台本入力
    # 未配線のターゲット（argv[3]/SILS_EMU_EVENTS を無視）は何も注入せず exit==0 で偽合格しうる。
    injected = events.exists() and events.stat().st_size > 0
    inject_check = {"name": "input injected (events.jsonl non-empty)", "pass": injected,
                    "detail": f"{events.stat().st_size if events.exists() else 0} bytes"
                              + ("" if injected else f" — is {exe.name} wired for scripted input?")}

    # Bundle the flight-log CSVs the run just wrote into one *.sflog.zip BEFORE
    # evaluating .expect's `metric` assertions — they read this same zip (see
    # _bundle_metric()). zip_path is None if the target wrote no flight-log CSV.
    # .expect の `metric` アサーション評価より前に、この実行が書いたフライト
    # ログ CSV を1個の *.sflog.zip にまとめる（_bundle_metric() が同じ zip を
    # 読む）。ターゲットがフライトログ CSV を書かなければ zip_path は None。
    zip_path = _finalize_flightlog(
        flightlog_dir,
        bundle / f"sils_{scn.stem}_{run_started_at:%Y%m%dT%H%M%S}.sflog.zip",
        notes=f"scenario={scn.name} target={target} duration_us={args.duration} "
              f"noise={noise} seed={seed}",
        events_jsonl=events,
    )

    expect = Path(args.expect) if args.expect else scn.with_suffix(".expect")
    xfail_reason = None
    if expect.exists():
        checks, verdict, xfail_reason = _eval_expect(expect, r.stdout, r.stderr, r.returncode, zip_path)
    else:
        console.info(f"(no .expect at {expect.name} — verdict = injection + exit code)")
        checks = [{"name": "exit == 0", "pass": r.returncode == 0, "detail": f"got {r.returncode}"}]
        verdict = (r.returncode == 0)
    checks.insert(0, inject_check)
    verdict = bool(verdict and injected)

    results = {
        "gate": "scenario", "milestone": f"scn_{scn.stem}", "kind": "scenario",
        "scenario": str(scn), "target": target, "exit_code": r.returncode,
        "noise": noise, "seed": seed,
        "pass": bool(verdict), "checks": checks,
        "flightlog": str(zip_path) if zip_path else None,
        # Known-fail marker (docs/architecture/simulation-policy.md backlog), if the
        # .expect carries one — informational only here; `sf sils scenario` still
        # reports the RAW verdict above (see xfail_reason annotation below). Only
        # `sf sils regression` changes gating behavior on this field.
        # 既知失敗マーカー（simulation-policy.md バックログ）。ここでは情報表示のみ
        # — `sf sils scenario` 単体では素の合否を報告する。ゲート挙動を変えるのは
        # `sf sils regression` のみ。
        "xfail_reason": xfail_reason,
        # Accurate flight label for the review-video title (no takeoff/landing claim).
        # レビュー動画タイトル用の正確な飛行ラベル（離着陸を主張しない）。
        "flight": "scripted-input scenario",
    }
    (bundle / "results.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    console.info(f"scenario {scn.name}: {'PASS' if verdict else 'FAIL'} "
                 f"(exit {r.returncode}, {len(checks)} checks, events={events.name})")
    for c in checks:
        if c.get("skipped"):
            print(f"  [SKIP] {c['name']}")
        else:
            print(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['name']}  ({c.get('detail','')})")
    if verdict:
        console.success(f"bundle: {bundle}")
    else:
        console.error(f"scenario FAILED — see {bundle}/console.log")
    # xfail annotation: a single `sf sils scenario` run always shows the RAW verdict
    # above (PASS is PASS, FAIL is FAIL) — this is just a one-line note that the
    # .expect carries a known-fail marker. Only `sf sils regression` changes gating
    # behavior (KNOWN-FAIL / XPASS) on this field.
    # xfail 注記: 単体の `sf sils scenario` は常に上の素の合否を表示する（xfail が
    # あっても FAIL は FAIL と見せる）。これは .expect に既知失敗マーカーがある
    # ことを示す1行注記のみ。ゲート挙動を変えるのは `sf sils regression` のみ。
    if xfail_reason:
        console.warning(f"[xfail marker] {xfail_reason}")

    # --video: render a review MP4 (MuJoCo 3D + state graphs) from the flight log the
    # run just recorded. Only on PASS and only if a flight-log bundle was actually
    # written. render_video.py finds the newest sils_*.sflog.zip under --bundle itself.
    # --video: 実行が記録したフライトログからレビュー動画（MuJoCo 3D＋状態グラフ）を
    # 描画。PASS かつ一式が書かれた場合のみ。render_video.py 自身が --bundle 配下の
    # 最新 sils_*.sflog.zip を探す。
    if verdict and getattr(args, "video", False):
        py = _venv()
        if py is None:
            console.error("viz venv missing — cannot render (see simulator/sils/viz)")
        elif zip_path is None:
            console.error(f"no flight-log bundle in {bundle} — is {exe.name} wired for "
                          "SILS_EMU_FLIGHTLOG recording?")
        else:
            out = bundle / f"scn_{scn.stem}.mp4"
            console.info("Rendering review video (MuJoCo 3D + state graphs)...")
            rv = subprocess.run([str(py), str(_sils_dir() / "viz" / "render_video.py"),
                                 "--model", str(_model()), "--bundle", str(bundle),
                                 "--out", str(out), "--fps", "50"])
            if rv.returncode == 0:
                console.success(f"Video: {out}")
            else:
                console.error("render_video.py failed")
    return 0 if verdict else 2


# =============================================================================
# `sf sils fly` — P6 stage 1: real-time, keyboard-piloted SILS flight.
# `sf sils fly` — P6 stage 1: リアルタイム・キーボード操縦SILS飛行。
#
# Spawns emu_vehicle with SILS_EMU_REALTIME=1 (paces the deterministic
# scheduler to the wall clock — simulator/sils/devices/emu_realtime.cpp) and
# SILS_EMU_RC_STDIN=1 (accepts live 'rc <roll> <pitch> <yaw> <throttle>' /
# 'arm' / 'land' / 'quit' lines — simulator/sils/devices/rc_stdin.cpp),
# translates raw-mode keystrokes into that protocol at ~50Hz, and renders the
# emulator's own 'STATE ...' HUD line as a live one-line status readout —
# mirroring `sf rc`'s interactive keyboard mode (lib/sfcli/commands/rc.py) as
# closely as the different transport (stdin pipe vs. WebSocket/CLI) allows.
#
# emu_vehicle を SILS_EMU_REALTIME=1（決定論スケジューラを壁時計にペーシング —
# emu_realtime.cpp）＋SILS_EMU_RC_STDIN=1（ライブ 'rc ...'/'arm'/'land'/'quit'
# 行を受理 — rc_stdin.cpp）付きで起動し、rawモードのキー入力を~50Hzで
# そのプロトコルへ変換、エミュレータ自身の 'STATE ...' HUD行を1行ステータス
# として描画する — `sf rc` の対話キーボードモード（rc.py）に、輸送経路の違い
# （stdin pipe 対 WebSocket/CLI）が許す限り揃える。
# =============================================================================

_FLY_KEY_HELP = """
  [sf sils fly — real-time keyboard control / リアルタイム・キーボード操縦]

  W/S      Pitch forward / back      前進 / 後退（ピッチ）
  A/D      Roll left / right         左 / 右ロール
  ,/.      Yaw left / right          左 / 右ヨー
  Space/Z  Throttle up / down        スロットル 上げ / 下げ
  R        ARM / DISARM (tap once)   ARM/DISARM切替（1回タップ — 実機送信機の
                                      モーメンタリボタンと同じ）
  +/-      Stick deflection (10-100) スティック振れ幅
  Q        Land, then quit           着陸してから終了
  Ctrl+C   Quit immediately (no land) 即終了（着陸なし）

  Note: yaw is ,/. here (not sf rc's q/e) because Q is reserved for land+quit.
  注: ヨーは（sf rc の q/e でなく），/. — Q は着陸＋終了に予約されているため。
"""


def _fly_adc(v: int) -> int:
    """Tello-SDK-style axis value (-100..100) -> raw 12-bit ADC (0..4095,
    centre 2048) — the SAME scale *.scn `rc` events and scenario_inject.hpp
    use, so a value typed here means the same stick deflection a scenario
    file would encode.
    Tello SDK風の軸値(-100..100) -> raw 12bit ADC(0..4095, 中央2048) —
    *.scn の `rc` イベント・scenario_inject.hpp と同一スケール。
    """
    raw = 2048 + round(v * 20.47)
    return max(0, min(4095, raw))


def _fly_parse_state(line: str) -> dict:
    """Parse emu_vehicle's "STATE k=v k=v ..." HUD line into a dict."""
    fields: dict = {}
    for tok in line.split()[1:]:   # skip the leading "STATE" token
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if k == "mode":
            fields[k] = v   # e.g. "FLYING:STABILIZE*" — not numeric
            continue
        try:
            fields[k] = float(v)
        except ValueError:
            pass
    return fields


def _fly_handle_key(ch: str, state: dict, timers: dict, now: float,
                    proc: subprocess.Popen) -> None:
    """Handle one keypress for `sf sils fly`'s interactive loop.

    `sf sils fly` の対話ループで1キー入力を処理する。
    """
    spd = state["speed"]
    if ch == '\x03':   # Ctrl+C — raw mode delivers it as a plain byte (ISIG off)
        state["running"] = False
        state["_immediate_quit"] = True
    elif ch == 'w':
        state["b"] = spd; timers["b+"] = now
    elif ch == 's':
        state["b"] = -spd; timers["b-"] = now
    elif ch == 'a':
        state["a"] = -spd; timers["a-"] = now
    elif ch == 'd':
        state["a"] = spd; timers["a+"] = now
    elif ch == ',':
        state["d"] = -spd; timers["d-"] = now
    elif ch == '.':
        state["d"] = spd; timers["d+"] = now
    elif ch == ' ':
        state["c"] = spd; timers["c+"] = now
    elif ch == 'z':
        state["c"] = -spd; timers["c-"] = now
    elif ch in ('+', '='):
        state["speed"] = min(100, spd + 10)
    elif ch in ('-', '_'):
        state["speed"] = max(10, spd - 10)
    elif ch == 'r':
        # One-shot pulse (idempotent while already high — see rc_stdin.cpp);
        # a quick tap is enough, holding the key just re-arms the pulse window.
        # 1回のパルス（既に高の間は再武装するだけ — rc_stdin.cpp参照）。
        # 軽くタップすれば十分、押し続けてもパルス窓が延長されるだけ。
        try:
            proc.stdin.write("arm\n"); proc.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
    elif ch == 'q':
        state["a"] = state["b"] = state["c"] = state["d"] = 0
        state["_land"] = True


def _fly_update_release(state: dict, timers: dict, now: float, timeout: float) -> None:
    """Zero out an axis whose key hasn't been pressed recently (same 80ms
    release-detection heuristic as `sf rc`'s interactive mode).
    最近押されていない軸をゼロにする（`sf rc` 対話モードと同じ80msリリース検出）。
    """
    a_plus, a_minus = timers.get("a+", 0), timers.get("a-", 0)
    if now - max(a_plus, a_minus) > timeout: state["a"] = 0
    b_plus, b_minus = timers.get("b+", 0), timers.get("b-", 0)
    if now - max(b_plus, b_minus) > timeout: state["b"] = 0
    c_plus, c_minus = timers.get("c+", 0), timers.get("c-", 0)
    if now - max(c_plus, c_minus) > timeout: state["c"] = 0
    d_plus, d_minus = timers.get("d+", 0), timers.get("d-", 0)
    if now - max(d_plus, d_minus) > timeout: state["d"] = 0


def _fly_display_status(state: dict, sim_state: dict, start_time: float) -> None:
    """Overwrite the current terminal line with the local stick state AND the
    emulator's own latest STATE HUD line (may be empty if none arrived yet).
    現在の端末行をローカルのスティック状態＋エミュレータ自身の最新STATE HUD
    （まだ届いていなければ空）で上書きする。
    """
    a, b, c, d = state["a"], state["b"], state["c"], state["d"]
    spd = state["speed"]
    elapsed = time.monotonic() - start_time

    if sim_state:
        sim_part = (f"mode={sim_state.get('mode', '?')} alt={sim_state.get('alt', 0.0):5.2f}m "
                   f"R={sim_state.get('roll', 0.0):+5.1f} P={sim_state.get('pitch', 0.0):+5.1f} "
                   f"Y={sim_state.get('yaw', 0.0):+5.1f} batt={sim_state.get('vbatt', 0.0):.2f}V")
    else:
        sim_part = "(waiting for emu_vehicle...)"

    line = (f"  [FLY] R={a:+4d} P={b:+4d} Y={d:+4d} T={c:+4d} spd={spd:3d} | "
            f"{sim_part} | {elapsed:5.1f}s")
    pad = max(0, state["last_line_len"] - len(line))
    sys.stdout.write(f"\r{line}{' ' * pad}")
    sys.stdout.flush()
    state["last_line_len"] = len(line)


def _fly_stdout_reader(proc: subprocess.Popen, log_tail: list, latest_state: dict,
                       lock: threading.Lock) -> None:
    """Background reader: splits emu_vehicle's combined stdout/stderr into the
    'STATE ...' HUD channel (kept as the single latest snapshot) and everything
    else (kept as a bounded tail for post-mortem on an unexpected exit).
    バックグラウンド読み取り: emu_vehicle の合成stdout/stderrを 'STATE ...'
    HUDチャネル（最新1件のみ保持）とそれ以外（想定外終了時の事後診断用に
    有界tailで保持）に分ける。
    """
    for raw_line in proc.stdout:
        line = raw_line.rstrip("\n")
        if line.startswith("STATE "):
            with lock:
                latest_state.clear()
                latest_state.update(_fly_parse_state(line))
        else:
            log_tail.append(line)
            if len(log_tail) > 200:
                del log_tail[0]


def run_fly(args: argparse.Namespace) -> int:
    """Real-time, keyboard-piloted SILS flight (P6 stage 1) — see the module
    comment above `_FLY_KEY_HELP` for the design.
    """
    if not _HAS_TERMIOS:
        console.error("`sf sils fly` requires a Unix terminal (termios) — not supported on Windows")
        return 1
    if not sys.stdin.isatty():
        console.error("`sf sils fly` requires an interactive terminal")
        return 1

    bd = _build_dir()
    exe = bd / _exe("emu_vehicle")
    if not exe.exists():
        console.error(f"{exe.name} not built — run 'sf sils build' first")
        return 1
    if not os.environ.get("SF_SILS_SKIP_FRESHNESS_CHECK"):
        _check_build_freshness(exe)

    bundle = _sils_dir() / "viz" / "out_fly"
    bundle.mkdir(parents=True, exist_ok=True)
    flightlog_dir = bundle / "flightlog"
    env = dict(win_run_env(bd), SILS_EMU_REALTIME="1", SILS_EMU_RC_STDIN="1",
               SILS_EMU_FLIGHTLOG=str(flightlog_dir))

    # --param NAME=VALUE: same SILS_EMU_PARAMS_FILE mechanism as `sf sils
    # scenario --param` (see its own help text for the full rationale).
    params_list = getattr(args, "params", None) or []
    if params_list:
        valid_lines = []
        for raw_param in params_list:
            if "=" not in raw_param:
                console.warning(f"--param {raw_param!r} — missing '=', expected NAME=VALUE, skipped")
                continue
            name, _, value = raw_param.partition("=")
            valid_lines.append(f"{name.strip()} {value.strip()}")
        override_path = bundle / "params_override.txt"
        override_path.write_text(
            "\n".join(valid_lines) + ("\n" if valid_lines else ""), encoding="utf-8")
        env["SILS_EMU_PARAMS_FILE"] = str(override_path)
        console.info(f"{len(valid_lines)} param(s) queued via --param")

    argv = [str(exe), str(_model()), str(int(args.duration * 1e6))]
    scenario_path = None
    if getattr(args, "scenario", None):
        scenario_path = Path(args.scenario)
        if not scenario_path.exists():
            console.error(f"scenario not found: {scenario_path}")
            return 1
        argv.append(str(scenario_path))

    console.info("Starting sf sils fly (real-time SILS, keyboard control)")
    console.print(_FLY_KEY_HELP)
    if scenario_path:
        console.info(f"Pre-arm scenario queued: {scenario_path.name} — wait for it to "
                     "finish before touching the sticks (its own RC injection and your "
                     "keyboard's both write live; see README's known-limitations note)")

    try:
        proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                errors="replace", bufsize=1, env=env)
    except OSError as e:
        console.error(f"failed to launch {exe.name}: {e}")
        return 1

    log_tail: list = []
    latest_state: dict = {}
    state_lock = threading.Lock()
    reader_thread = threading.Thread(target=_fly_stdout_reader,
                                     args=(proc, log_tail, latest_state, state_lock),
                                     daemon=True)
    reader_thread.start()

    rc_state = {"a": 0, "b": 0, "c": 0, "d": 0, "speed": 50,
               "running": True, "last_line_len": 0}
    key_timers: dict = {}
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    start_time = time.monotonic()
    rate = 50
    interval = 1.0 / rate

    try:
        tty.setraw(fd)
        while rc_state["running"]:
            loop_start = time.monotonic()

            if proc.poll() is not None:
                console.print("")
                console.error(f"emu_vehicle exited unexpectedly (code {proc.returncode})")
                for line in log_tail[-20:]:
                    console.print(f"  {line}")
                break

            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                _fly_handle_key(ch, rc_state, key_timers, loop_start, proc)

            _fly_update_release(rc_state, key_timers, loop_start, timeout=0.08)

            if rc_state.pop("_immediate_quit", False):
                try:
                    proc.stdin.write("quit\n"); proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
                break
            if rc_state.pop("_land", False):
                try:
                    proc.stdin.write("land\n"); proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
                time.sleep(1.5)   # grace period for a graceful ALT/POS auto-land
                try:
                    proc.stdin.write("quit\n"); proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
                break

            roll, pitch = _fly_adc(rc_state["a"]), _fly_adc(rc_state["b"])
            yaw, thr = _fly_adc(rc_state["d"]), _fly_adc(rc_state["c"])
            try:
                proc.stdin.write(f"rc {roll} {pitch} {yaw} {thr}\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                console.print("")
                console.error("lost connection to emu_vehicle stdin")
                break

            with state_lock:
                state_snapshot = dict(latest_state)
            _fly_display_status(rc_state, state_snapshot, start_time)

            elapsed = time.monotonic() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    except KeyboardInterrupt:
        try:
            proc.stdin.write("quit\n"); proc.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        console.print("")
        if proc.poll() is None:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
        reader_thread.join(timeout=2)

    console.success(f"sf sils fly session ended (emu_vehicle exit {proc.returncode})")
    zip_path = _finalize_flightlog(
        flightlog_dir, bundle / f"sils_fly_{datetime.now():%Y%m%dT%H%M%S}.sflog.zip",
        notes="sf sils fly (real-time keyboard-piloted session)",
    )
    if zip_path is not None:
        console.print(f"  flight log: {zip_path}")
    return 0 if proc.returncode == 0 else 1


# --- regression: run every *.scn/*.expect pair and gate on the aggregate ------------
# Each scenario documents its OWN required invocation (target/duration/unpaired/...)
# in a "sf sils scenario simulator/sils/scenarios/<name>.scn ..." comment inside its own
# header (e.g. api_flight.scn: "# Run: ... --target vehicle --duration 40000000") —
# the .scn file is the single source of truth for how it must be run, so this parses
# that line instead of hand-maintaining a duplicate table that would drift.
# 各シナリオは自身のヘッダコメント内の "sf sils scenario .../<name>.scn ..." 行で
# 自分の必要な呼び出し(target/duration/unpaired等)を宣言する（例:
# api_flight.scn の "# Run: ... --target vehicle --duration 40000000"）。.scn 自身が
# 自分の呼び出し方法の唯一の正なので、重複管理で陳腐化するテーブルを持たず、この行を
# 解析する。
_RUN_LINE_RE = re.compile(r"sf sils scenario\s+\S*?scenarios/(?P<name>[\w.]+)\.scn(?P<rest>.*)")

# Two scenarios whose header comment does NOT document the target they actually need
# (unlike api_flight/pairing/etc. above) — verified empirically (2026-07) and matching
# TEST_MATRIX.md's "その他のシナリオ" table: the CLI feeder and ESP-NOW virtual-pilot
# input channels are wired for vehicle_old only, so these two FAIL under the default
# vehicle target. Keep this table tiny and named so it stays an obvious exception, not
# a growing parallel manifest.
# ヘッダコメントに実際必要な target が書かれていない2本（上記 api_flight/pairing 等とは
# 違う）。2026-07 に実測で確認済み、TEST_MATRIX.md の「その他のシナリオ」表とも一致:
# CLI フィーダと ESP-NOW 仮想パイロット入力チャネルは vehicle_old 専用配線のため、既定の
# vehicle ターゲットでは FAIL する。肥大化する並行マニフェストにならないよう、この表は
# 小さく・例外だと分かる名前に留める。
_TARGET_OVERRIDE = {
    "console_cli": "vehicle_old",
    "hover_espnow": "vehicle_old",
}


def _scenario_invocation(scn: Path) -> list:
    """Return the extra `sf sils scenario` CLI args this .scn documents for itself
    (parsed from its own header comment), always including an explicit --target
    (falling back to _TARGET_OVERRIDE, then "vehicle"). Returns e.g.
    ['--target', 'vehicle', '--duration', '40000000'].
    この .scn が自身のヘッダコメントで宣言する追加CLI引数を返す。--target は
    常に明示する（宣言が無ければ _TARGET_OVERRIDE、それも無ければ "vehicle"）。
    """
    extra = []
    for line in scn.read_text(encoding="utf-8").splitlines():
        m = _RUN_LINE_RE.search(line)
        if m and m.group("name") == scn.stem:
            extra = shlex.split(m.group("rest"))
            break
    if "--target" not in extra:
        extra = ["--target", _TARGET_OVERRIDE.get(scn.stem, "vehicle")] + extra
    return extra


def _check_param_consistency() -> tuple:
    """Run tools/params_audit's consistency audit — the same check `sf params
    check --strict` performs — and return (passed, total_checks).
    tools/params_audit の整合検査（`sf params check --strict` と同じ判定）を
    実行し、(合格したか, 総チェック数) を返す。

    Imported in-process rather than via subprocess, following the
    sys.path-insert pattern used throughout sfcli for tools/ submodules (see
    run_check() in lib/sfcli/commands/params.py, and sf sysid's run_fit() for
    another example). tools/params_audit is stdlib-only, so this never drags
    a heavy dependency into the SILS regression path.
    subprocess ではなくプロセス内 import。sfcli 全体で tools/ 配下モジュールに
    使われる sys.path 差し込みの流儀に従う（lib/sfcli/commands/params.py の
    run_check()、sf sysid の run_fit() も同様の例）。params_audit は標準
    ライブラリのみに依存するため、SILS 回帰の経路に重い依存を持ち込まない。
    """
    tools_dir = str(paths.root() / "tools")
    sys.path.insert(0, tools_dir)
    try:
        from params_audit import check_params
    except ImportError as e:
        console.error(f"Failed to import params_audit.check_params: {e}")
        return False, 0
    finally:
        if tools_dir in sys.path:
            sys.path.remove(tools_dir)

    rows = check_params.run_audit(paths.root())
    summary = check_params.summarize(rows)
    # strict=True: mirrors `sf params check --strict`, so an UNRESOLVED
    # parameter also gates the regression, not just MISMATCH/ERROR.
    # strict=True: `sf params check --strict` と同じ判定。UNRESOLVED も
    # MISMATCH/ERROR と同様に退行の合否対象にする。
    passed = check_params.compute_exit_code(summary, strict=True) == 0
    return passed, summary["total"]


def _check_pid_lockstep() -> tuple:
    """Run simulator/tests/test_pid_lockstep.py (P5 stage 1) and return
    (passed, stdout+stderr). That test drives the REAL firmware sf::PID
    (firmware/vehicle/components/sf_controller_pid/include/pid.hpp, via the
    stampfly_control pybind11 module) and tools/log_analyzer/rate_sysid.py's
    hand-ported replay_pid() on identical input sequences and asserts their
    outputs match step-by-step.
    simulator/tests/test_pid_lockstep.py（P5 stage 1）を実行し、(合格したか,
    stdout+stderr) を返す。このテストは本物のファーム sf::PID（pid.hpp、
    stampfly_control pybind11 モジュール経由）と rate_sysid.py の手動移植
    replay_pid() を同一入力系列で駆動し、出力が1ステップずつ一致することを
    確認する。

    Equivalence between the sysid pipeline's replay_pid() and the firmware
    PID is a Code Identity precondition (development_roadmap.md's 3
    principles: Code/Param/Model Identity) — every rate-loop gain fit `sf
    sysid` produces is only meaningful for the ACTUAL firmware if the two
    stay identical. The moment either side is edited (pid.hpp or
    replay_pid()) and they diverge, `sf sysid` would silently keep fitting
    gains against a plant model the firmware no longer runs. This must gate
    the regression immediately, before any *.scn scenario runs, same as
    _check_param_consistency() above.
    同定パイプライン(replay_pid)とファームPIDの等価性は Code Identity
    （development_roadmap.md の3原則）の前提 — `sf sysid` が出すレート系
    ゲインフィットは、両者が同一である限りにおいてのみ実ファームに対して
    意味を持つ。どちらか（pid.hpp か replay_pid()）が編集されて乖離した瞬間、
    `sf sysid` は黙って「もうファームが実行していないプラント」に対する
    ゲインを算出し続けてしまう。上の _check_param_consistency() と同様、
    *.scn シナリオを1本も走らせる前に回帰全体をゲートする。

    Run as its own subprocess (not in-process pytest.main()) so a hard crash
    inside the pybind11 extension cannot take down `sf sils regression`
    itself — only fail this one gate, with pytest's own output (including
    the module's actionable "run `sf sils build`" ImportError message when
    stampfly_control isn't built) captured for the caller to print.
    プロセス内 pytest.main() ではなくサブプロセスとして実行する: pybind11
    拡張内のクラッシュが `sf sils regression` 自体を道連れにせず、このゲート
    1つだけを失敗させるため。pytest 自身の出力（stampfly_control 未ビルド時の
    「sf sils build を実行せよ」という実用的な ImportError メッセージを含む）
    を呼び出し元が表示できるよう捕捉する。
    """
    test_path = paths.root() / "simulator" / "tests" / "test_pid_lockstep.py"
    r = subprocess.run([sys.executable, "-m", "pytest", str(test_path), "-q"],
                       capture_output=True, text=True)
    return r.returncode == 0, r.stdout + r.stderr


def run_regression(args: argparse.Namespace) -> int:
    scn_dir = _sils_dir() / "scenarios"
    # The .expect glob is authoritative (README/TEST_MATRIX "32 scenarios" as of
    # 2026-07; scenarios without an .expect are exploratory/manual benches not
    # gated here — see TEST_MATRIX.md "その他のシナリオ").
    # .expect グロブが正（2026-07時点で「32本」。.expect の無いものは探索的・手動用の
    # ベンチでありここではゲートしない — TEST_MATRIX.md「その他のシナリオ」参照）。
    scenarios = sorted(p for p in scn_dir.glob("*.scn") if p.with_suffix(".expect").exists())
    if not scenarios:
        console.error(f"no *.scn/*.expect pairs found under {scn_dir}"); return 1

    # Gate on hand-copied physical-parameter consistency BEFORE running any
    # scenario (tools/params_audit — C_T/C_Q/kappa/inertia are hand-duplicated
    # across firmware/SILS/simulator sources, no code-gen pipeline yet, Phase 1
    # of docs/architecture/simulation-policy.md). A silent drift here would
    # make every scenario below PASS against the wrong plant, so this must run
    # first and fail the whole regression immediately — no point spending
    # minutes on scenarios whose physics reference is already known-wrong.
    # シナリオ実行前に、手動複製された物理パラメータの整合性
    # （tools/params_audit — C_T/C_Q/kappa/慣性等をファーム/SILS/シミュレータへ
    # 手書きコピー。コード生成パイプライン未整備、Phase 1、simulation-policy.md
    # 参照）をゲートする。ここでの食い違いを見逃すと、以下の全シナリオが
    # 「間違った機体モデル」に対して PASS してしまう。よって最初に検査し、
    # 不合格なら回帰全体を即座に失敗させる — 物理基準が既に誤りと分かっている
    # のに数分かけてシナリオを回す意味が無い。
    params_ok, n_param_checks = _check_param_consistency()
    if not params_ok:
        console.error("[FAIL] parameter consistency check — run `sf params check` for details")
        return 1
    console.info(f"[PASS] parameter consistency ({n_param_checks} checks)")

    # Build-freshness advisory: judged ONCE here, up front (same "gate the loop
    # before it starts" pattern as _check_param_consistency() above), rather than
    # once per scenario — the loop below spawns each scenario as its OWN subprocess
    # (see _RUN_LINE_RE comment), so without this the freshness check inside
    # run_scenario() would print once per scenario (32 times). Both target binaries
    # are checked (regression can exercise vehicle AND vehicle_old — see
    # _TARGET_OVERRIDE) against a single shared source scan. Setting the env var on
    # THIS process is enough: subprocess.run(cmd) below has no env= kwarg, so it
    # inherits the parent's os.environ (incl. this var) into every child scenario run.
    # ビルド鮮度の参考警告: ここで冒頭に1回だけ判定する（上の _check_param_consistency
    # と同じ「ループ開始前にゲート」パターン）。以下のループは各シナリオを別
    # プロセスとして起動するため、これが無いと run_scenario 内の判定がシナリオ数
    # (32回)ぶん重複表示されてしまう。回帰は vehicle と vehicle_old 両方を実行し
    # うるため両方の exe を、ソース走査は共有した1回分でチェックする。この
    # プロセスで環境変数を立てるだけでよい — 以下の subprocess.run(cmd) は env=
    # を指定していないため親プロセスの os.environ（この変数を含む）をそのまま
    # 継承し、全ての子シナリオ実行に伝播する。
    src_mtime = _sils_source_mtime()
    bd = _build_dir()
    for exe_name in ("emu_vehicle", "emu_vehicle_old"):
        exe = bd / _exe(exe_name)
        if exe.exists():
            _check_build_freshness(exe, src_mtime=src_mtime)
    os.environ["SF_SILS_SKIP_FRESHNESS_CHECK"] = "1"

    # Gate on PID lockstep equivalence BEFORE running any scenario (same
    # "gate the loop before it starts" pattern as _check_param_consistency()
    # above). Equivalence between the identification pipeline (replay_pid)
    # and the firmware PID is a Code Identity precondition: the moment either
    # side is edited and the two diverge, this must be caught here — see
    # _check_pid_lockstep()'s docstring for the full rationale.
    # シナリオ実行前に PID ロックステップ等価性をゲートする（上の
    # _check_param_consistency() と同じ「ループ開始前にゲート」パターン）。
    # 同定パイプライン(replay_pid)とファームPIDの等価性は Code Identity の
    # 前提 — どちらかが編集されて乖離した瞬間にここで検出する。詳細な理由は
    # _check_pid_lockstep() の docstring 参照。
    lockstep_ok, lockstep_output = _check_pid_lockstep()
    if not lockstep_ok:
        # Flush explicitly: stdout is fully-buffered when not a tty (redirected
        # to a file/CI log), while console.error()'s stderr print below is not
        # — without this, the summary line below can land in the log BEFORE
        # this raw pytest dump it is meant to follow.
        # 明示的に flush する: tty でない場合（ファイル/CIログへリダイレクト時）
        # stdout は完全バッファリングされる一方、下の console.error() の stderr
        # 出力はされない — これが無いと、下の要約行がこの生 pytest 出力より
        # 先にログへ現れてしまう。
        sys.stdout.write(lockstep_output)
        sys.stdout.flush()
        console.error("[FAIL] PID lockstep — firmware pid.hpp and rate_sysid "
                     "replay_pid have diverged; run pytest "
                     "simulator/tests/test_pid_lockstep.py -v")
        return 1
    console.info("[PASS] PID lockstep (firmware pid.hpp == replay_pid)")

    console.info(f"SILS regression: {len(scenarios)} scenario(s) with a matching .expect...")
    results = []
    include_workshop = getattr(args, "include_workshop", False)
    for scn in scenarios:
        extra = _scenario_invocation(scn)
        target = extra[extra.index("--target") + 1]

        # workshop-target scenarios depend on whatever main/user_code.cpp CURRENTLY
        # contains — that file is `sf lesson switch`'s target, gitignored, not repo
        # state. A fresh checkout's CMake bootstrap (simulator/sils/CMakeLists.txt)
        # seeds it from the Lesson 0 template, which drives no motors, so a liftoff
        # gate like workshop_acro.expect's `metric alt_max > 0.1` fails there BY
        # CONSTRUCTION, not because of a real regression — this would otherwise
        # redden CI on every PR regardless of content. Skip by default (does not
        # count as FAIL — see the SKIP status below, same non-gating treatment as
        # xfail's KNOWN-FAIL); --include-workshop opts in once the caller has
        # switched to a known lesson (see workshop_acro.scn's header).
        # workshop 対象シナリオは main/user_code.cpp の「現在の」中身に依存する —
        # このファイルは `sf lesson switch` の対象であり gitignore 対象、リポジトリ
        # 状態ではない。新規チェックアウトの CMake ブートストラップ
        # （simulator/sils/CMakeLists.txt）は Lesson 0 テンプレートから種付けし、
        # これはモータを動かさないため、workshop_acro.expect の
        # `metric alt_max > 0.1` のような離陸ゲートは実回帰ではなく構造的に失敗する
        # — これを放置すると内容に関わらず全 PR で CI が赤くなる。既定でスキップし
        # （FAIL 扱いしない — 下の SKIP ステータス、xfail の KNOWN-FAIL と同じ
        # 非ゲート扱い）、--include-workshop は呼び出し側が既知のレッスンへ切り替えた
        # 後にのみ opt-in する（workshop_acro.scn のヘッダ参照）。
        if target == "workshop" and not include_workshop:
            status = "SKIP"
            console.info(f"  [SKIP] {scn.stem} (target=workshop; run manually after "
                         f"`sf lesson switch N --solution`, or pass --include-workshop)")
            results.append({"name": scn.stem, "target": target, "extra_args": extra,
                            "pass": True, "exit_code": 0, "elapsed_s": 0.0,
                            "xfail_reason": None, "status": status})
            continue

        cmd = [sys.executable, "-m", "sfcli", "sils", "scenario", str(scn)] + extra
        t0 = time.monotonic()
        r = subprocess.run(cmd)
        elapsed = time.monotonic() - t0
        verdict = (r.returncode == 0)
        # xfail: a "xfail: <reason>" directive at the head of the .expect marks
        # this scenario's current failure as a tracked, known issue (see
        # docs/architecture/simulation-policy.md backlog), not a broken
        # assertion. Classification:
        #   no marker,   PASS  -> PASS        (normal)
        #   no marker,   FAIL  -> FAIL        (gates the regression)
        #   marker,      FAIL  -> KNOWN-FAIL  (reported, does NOT gate)
        #   marker,      PASS  -> XPASS       (the issue is gone but the marker
        #                                      wasn't removed — SILS is
        #                                      deterministic, so this MUST gate
        #                                      the regression, or a fixed issue
        #                                      could silently stay "known-fail"
        #                                      forever)
        # xfail: .expect 先頭の "xfail: <理由>" 指令は、このシナリオの現在の失敗が
        # 追跡中の既知課題（simulation-policy.md バックログ）であり壊れたアサー
        # ションではないことを示す。分類は上記コメント参照。SILS は決定論的なので
        # XPASS（直ったのにマーカーが残っている）は必ず回帰全体を失敗させる。
        xfail_reason = _read_xfail(scn.with_suffix(".expect"))
        if xfail_reason is None:
            status = "PASS" if verdict else "FAIL"
            console.info(f"  [{status}] {scn.stem} (target={target}, {elapsed:.1f}s)")
        elif not verdict:
            status = "KNOWN-FAIL"
            console.warning(f"  [KNOWN-FAIL] {scn.stem} — {xfail_reason} "
                            f"(target={target}, {elapsed:.1f}s)")
        else:
            status = "XPASS"
            console.error(f"  [XPASS] {scn.stem} — remove the xfail marker "
                         f"(target={target}, {elapsed:.1f}s)")
        results.append({"name": scn.stem, "target": target, "extra_args": extra,
                        "pass": verdict, "exit_code": r.returncode,
                        "elapsed_s": round(elapsed, 1),
                        "xfail_reason": xfail_reason, "status": status})

    n_pass = sum(1 for r in results if r["status"] == "PASS")
    n_known_fail = sum(1 for r in results if r["status"] == "KNOWN-FAIL")
    n_xpass = sum(1 for r in results if r["status"] == "XPASS")
    n_fail = sum(1 for r in results if r["status"] == "FAIL")
    n_skip = sum(1 for r in results if r["status"] == "SKIP")
    # n_fail (real, un-marked FAILs) and n_xpass (fixed-but-still-marked) are the
    # only two statuses that gate the regression — KNOWN-FAIL and SKIP (workshop
    # scenarios skipped by default — see the loop above) are informational only.
    # n_fail（無印の実失敗）と n_xpass（直ったのにマーカー残存）だけが回帰をゲート
    # する — KNOWN-FAIL と SKIP（既定でスキップされる workshop シナリオ。上のループ
    # 参照）は情報表示のみ。
    n_unexpected = n_fail + n_xpass

    if getattr(args, "json_out", None):
        summary = {"total": len(results), "pass": n_pass, "known_fail": n_known_fail,
                  "xpass": n_xpass, "fail": n_unexpected, "skip": n_skip, "scenarios": results}
        Path(args.json_out).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        console.info(f"Summary written to {args.json_out}")

    if n_unexpected:
        console.error(f"SILS regression: {n_fail} FAIL + {n_xpass} XPASS "
                     f"(of {len(results)}; known-fail={n_known_fail}):")
        for r in results:
            if r["status"] == "FAIL":
                console.error(f"  FAIL {r['name']} (target={r['target']})")
            elif r["status"] == "XPASS":
                console.error(f"  XPASS {r['name']} (target={r['target']}) — "
                             f"remove the xfail marker in {r['name']}.expect")
        return 1
    console.success(f"SILS regression: {n_pass} PASS + {n_known_fail} KNOWN-FAIL "
                    f"+ {n_skip} SKIP ({len(results)} total) known-fail={n_known_fail}")
    return 0


def run_gui(args: argparse.Namespace) -> int:
    """Launch the SILS Web GUI (simulator/sils/gui/server.py). Serves the single-page app and
    a small JSON API that drives `sf sils scenario` runs and reads the bundles back for the
    browser (graphs + live 3D). Localhost only. SILS Web GUI を起動。localhost のみ。"""
    gui_dir = _sils_dir() / "gui"
    server = gui_dir / "server.py"
    if not server.exists():
        console.error(f"GUI server not found: {server}")
        return 1
    sys.path.insert(0, str(gui_dir))
    import importlib
    mod = importlib.import_module("server")
    try:
        mod.serve(port=args.port, open_browser=not args.no_browser)
    except OSError as e:
        console.error(f"could not bind port {args.port}: {e} "
                      f"(try `sf sils gui --port <other>`)")
        return 1
    return 0


def run_video(args: argparse.Namespace) -> int:
    py = _venv()
    if py is None:
        return 1
    bundle = _bundle_dir(args.milestone)
    if not any(bundle.glob("*.sflog.zip")):
        console.error(f"No flight-log bundle in {bundle} — run 'sf sils run' first"); return 1
    fps = getattr(args, "fps", 50)
    out = bundle / f"{args.milestone.lower()}_flight.mp4"
    console.info("Rendering review video (MuJoCo 3D + state graphs)...")
    r = subprocess.run([str(py), str(_sils_dir() / "viz" / "render_video.py"),
                        "--model", str(_model()), "--bundle", str(bundle),
                        "--out", str(out), "--fps", str(fps)])
    if r.returncode == 0:
        console.success(f"Video: {out}")
    return r.returncode


def run_compare(args: argparse.Namespace) -> int:
    # P4: run both estimators through the SAME flight and render a side-by-side
    # review video (twin 3D + overlay graphs), then write the aggregate verdict.
    # The bundle is unchanged between runs — that is the algorithm-independence
    # proof (RESET_PLAN P2/§9) turned into one shareable artifact.
    # P4: 同じ飛行で両推定器を走らせ並置レビュー動画を描く → 集約判定を書く。
    # ベンチは実行間で無改変 ＝ アルゴリズム非依存の実証を1本の共有素材に。
    if args.ea == args.eb:
        # Comparing an estimator to itself would falsely "prove" independence.
        # 同じ推定器同士の比較は非依存を偽証する。
        console.error(f"--ea and --eb must differ (both '{args.ea}') — a comparison "
                      "needs two distinct estimators"); return 1
    bd = _build_dir()
    exe = bd / _exe("hover_smoke")
    if not exe.exists():
        console.error("hover_smoke not built — run 'sf sils build' first"); return 1
    py = _venv()
    if py is None:
        return 1
    bundle = _bundle_dir(args.milestone)          # out_p4 / out_p6
    bundle.mkdir(parents=True, exist_ok=True)
    # Same noise level AND seed for both runs → identical noise realization → a fair
    # estimator contrast. Noise default resolves per milestone (P6 → n0).
    # 両実行に同じノイズ準位＋同じシード＝同一ノイズ実現で公平に比較。既定はマイルストーン別。
    noise = getattr(args, "noise", None)
    if noise is None:
        noise = MILESTONE_NOISE.get(str(args.milestone).upper(), "off")
    seed = getattr(args, "seed", 12345)
    runs = {}
    for est in (args.ea, args.eb):
        sub = bundle / est
        sub.mkdir(parents=True, exist_ok=True)
        console.info(f"Running closed loop for comparison ({est}, noise={noise})...")
        rc = subprocess.run([str(exe), str(_model()), str(sub), str(ESTIMATORS[est]),
                             f"{args.milestone}-{est}", noise, str(seed)],
                            env=win_run_env(bd)).returncode
        # hover_smoke writes its own flight-log v1 CSVs directly under
        # <sub>/flightlog/ (argv[2]=sub); bundle them into one *.sflog.zip.
        # hover_smoke は <sub>/flightlog/ 配下にフライトログ一式 CSV を直接書く
        # （argv[2]=sub）。1個の *.sflog.zip にまとめる。
        _finalize_flightlog(
            sub / "flightlog",
            sub / f"sils_hover_{args.milestone.lower()}_{est}_{datetime.now():%Y%m%dT%H%M%S}.sflog.zip",
            notes=f"milestone={args.milestone} estimator={est} noise={noise} seed={seed}",
        )
        # hover_smoke writes the bundle even on a G3 fail; only a hard early exit
        # (e.g. model load) leaves no files. Require both before render/aggregate so
        # a crash surfaces here, not as an opaque traceback downstream.
        # G3不合格でもバンドルは書かれる。ファイルが無いのは早期異常終了のみ。先に要求する。
        if not any(sub.glob("*.sflog.zip")) or not (sub / "results.json").exists():
            console.error(f"run '{est}' wrote no bundle (exit {rc}) — aborting comparison")
            return 1
        runs[est] = sub

    out = bundle / f"{args.milestone.lower()}_compare.mp4"
    console.info("Rendering side-by-side comparison (twin 3D + overlay graphs)...")
    fps = getattr(args, "fps", 50)
    r = subprocess.run([str(py), str(_sils_dir() / "viz" / "render_video.py"),
                        "--model", str(_model()),
                        "--bundle", str(runs[args.ea]), "--compare", str(runs[args.eb]),
                        "--label-a", ESTIMATOR_LABELS[args.ea],
                        "--label-b", ESTIMATOR_LABELS[args.eb],
                        "--out", str(out), "--fps", str(fps)])
    if r.returncode != 0:
        console.error("comparison render failed"); return r.returncode

    # Aggregate verdict: P4 passes iff BOTH runs pass G3 with the bench unchanged.
    # results.json stays the single source of truth (status/gate read only this).
    # 集約判定: 両実行がベンチ無改変で G3 合格のとき P4 合格。results.json が唯一の正。
    a = json.loads((runs[args.ea] / "results.json").read_text(encoding="utf-8"))
    b = json.loads((runs[args.eb] / "results.json").read_text(encoding="utf-8"))
    both = bool(a.get("pass") and b.get("pass"))
    agg = {
        "gate": "compare",
        "milestone": args.milestone,
        "kind": "comparison",
        "pass": both,
        "noise": noise,
        "flight": a.get("flight", "takeoff-hover-yaw-stop-landing"),
        "runs": [
            {"estimator": args.ea, "bundle": args.ea,
             "g3_pass": bool(a.get("pass")), "metrics": a.get("metrics", {})},
            {"estimator": args.eb, "bundle": args.eb,
             "g3_pass": bool(b.get("pass")), "metrics": b.get("metrics", {})},
        ],
        "checks": [
            {"name": f"{args.ea}_g3_pass", "pass": bool(a.get("pass"))},
            {"name": f"{args.eb}_g3_pass", "pass": bool(b.get("pass"))},
            {"name": "algorithm_independent", "pass": both},
        ],
    }
    (bundle / "results.json").write_text(json.dumps(agg, indent=2) + "\n", encoding="utf-8")
    console.success(f"Comparison bundle written to {bundle}")
    # Gate the comparison bundle (bundle complete AND both runs passing).
    return run_gate(args)


def run_status(args: argparse.Namespace) -> int:
    res = _bundle_dir(args.milestone) / "results.json"
    if not res.exists():
        console.error(f"No results.json for {args.milestone} — run 'sf sils run' first"); return 1
    r = json.loads(res.read_text(encoding="utf-8"))
    verdict = "PASS" if r.get("pass") else "FAIL"
    console.info(f"{r.get('milestone','?')}/{r.get('gate','?')}: {verdict}")
    # A comparison bundle (P4) carries per-run metrics under "runs"; a single
    # bundle carries flat "metrics". Show whichever shape this results.json has.
    # 比較バンドル(P4)は "runs" に各実行の metrics を持つ。単一は "metrics"。
    if r.get("kind") == "comparison":
        for run in r.get("runs", []):
            m = run.get("metrics", {})
            tilt = m.get("max_tilt_deg", "?")
            alt = m.get("max_alt_m", "?")
            g3 = "PASS" if run.get("g3_pass") else "FAIL"
            name = run.get("estimator") or "?"
            print(f"  {name:14s} G3 {g3}  max_alt={alt} m  max_tilt={tilt} deg")
    else:
        for k, v in r.get("metrics", {}).items():
            print(f"  {k:20s} {v}")
    for c in r.get("checks", []):
        mark = "PASS" if c.get("pass") else "FAIL"
        print(f"  [{mark}] {c.get('name')}")
    return 0 if r.get("pass") else 2


def run_gate(args: argparse.Namespace) -> int:
    py = _venv()
    if py is None:
        return 1
    r = subprocess.run([str(py), str(_sils_dir() / "tools" / "sils_gate.py"),
                        str(_bundle_dir(args.milestone))])
    return r.returncode


def run_milestone(args: argparse.Namespace) -> int:
    # The milestone only needs hover_smoke (run) + render_video (video, via venv) —
    # build just that target so an unrelated SILS target can't break the milestone.
    # milestone に必要なのは hover_smoke（run）と render_video（video, venv経由）だけ。
    # 無関係な SILS ターゲットが milestone を壊さないよう、そのターゲットだけビルドする。
    if not getattr(args, "target", None):
        args.target = "hover_smoke"
    # Resolve the per-milestone sensor-noise default unless the user set --noise.
    # ユーザーが --noise 指定しなければマイルストーン別の既定ノイズを解決。
    if getattr(args, "noise", None) is None:
        args.noise = MILESTONE_NOISE.get(str(args.milestone).upper(), "off")
    # P4 is the side-by-side comparison milestone: build once, then run BOTH
    # estimators and render one compare video (run_compare gates at the end).
    # P4 は並置比較マイルストーン: 1回ビルドし両推定器を走らせ1本の比較動画を描く。
    if str(args.milestone).upper() == "P4":
        if run_build(args) != 0:
            console.error("Milestone P4 stopped at build"); return 1
        args.ea = getattr(args, "ea", "eskf")
        args.eb = getattr(args, "eb", "complementary")
        return run_compare(args)
    for step in (run_build, run_run, run_video, run_gate):
        rc = step(args)
        if rc != 0 and step is not run_run:  # run_run defers its verdict to the gate
            console.error(f"Milestone {args.milestone} stopped at {step.__name__} (rc={rc})")
            return rc
    console.success(f"Milestone {args.milestone} bundle complete and gated.")
    return 0


# =============================================================================
# Model-match gate (docs/architecture/simulation-policy.md §4, backlog #0)
# モデル一致ゲート
# =============================================================================

# Pass criteria (simulation-policy.md §4): b within ±50%, L_total=T+L within ±20%.
# T/L individually are NOT judged (their split is degenerate in some fits — see
# analysis/reports/rate_sysid_reference/README.md); shown for reference only.
# 合格基準: b は±50%、L_total=T+L は±20%。T・L個別はフィットで退化しうるため
# 判定に使わない（参考表示のみ）。
SYSID_GATE_B_TOL = 0.50
SYSID_GATE_LTOTAL_TOL = 0.20


def _rate_backend():
    """Import tools/log_analyzer/rate_sysid.py — the SAME identification code the
    real-hardware `sf sysid rate-fit` command uses (Code Identity: the gate's
    verdict comes from running identical code, not a reimplementation).
    実機の `sf sysid rate-fit` と同一の同定コードをインポートする（Code Identity:
    ゲートの判定は再実装ではなく同一コードの実行に由来する）。"""
    import sys as _sys
    backend = paths.root() / "tools" / "log_analyzer"
    if str(backend) not in _sys.path:
        _sys.path.insert(0, str(backend))
    import rate_sysid
    return rate_sysid


def _reference_path() -> Path:
    return paths.root() / "analysis" / "reports" / "rate_sysid_reference" / "reference.json"


def run_sysid_gate(args: argparse.Namespace) -> int:
    """Run sysid_gate.scn (writing a StampFly flight-log v1 bundle via
    SILS_EMU_FLIGHTLOG, same as every other scenario), fit (b,T,L) per axis
    with rate_sysid.py's fit_from_df() on the bundle's aligned DataFrame
    (tools/sysid/loader.load_aligned() — the SAME entry point real-hardware
    `sf sysid rate-fit` uses), and compare against the real-hardware
    reference. sysid_gate.scn を実行し（他の全シナリオと同じ SILS_EMU_FLIGHTLOG
    でフライトログ一式を書く）、一式の整列済み DataFrame
    （tools/sysid/loader.load_aligned() — 実機の `sf sysid rate-fit` と同じ
    入口）に rate_sysid.py の fit_from_df() で軸別 (b,T,L) をフィットし、
    実機基準値と比較する。"""
    scn = _sils_dir() / "scenarios" / "sysid_gate.scn"
    if not scn.exists():
        console.error(f"scenario not found: {scn}"); return 1
    reference_path = _reference_path()
    if not reference_path.exists():
        console.error(f"reference missing: {reference_path} — run "
                      "analysis/reports/rate_sysid_reference/make_reference.py first")
        return 1

    # Bundle dir matches what run_scenario derives on its own
    # (_sils_dir()/viz/out_scn_<stem>) — computed here only to locate
    # results.json after the run (run_scenario creates the directory itself).
    # バンドルディレクトリは run_scenario が自前で導出するのと同じ場所
    # （_sils_dir()/viz/out_scn_<stem>）— run 後に results.json を見つけるため
    # だけにここで計算する（ディレクトリ自体は run_scenario が作る）。
    bundle = _sils_dir() / "viz" / f"out_scn_{scn.stem}"

    # Drive the same run_scenario() path every other scenario uses (bundle layout,
    # flight-log recording, console capture, .expect check, injection guardrail) —
    # no extra env needed now that every scenario records a flight-log bundle.
    # Constructing the Namespace by hand (rather than argparse) keeps this
    # independent of the `scenario` subcommand's own flags.
    # 他の全シナリオと同じ run_scenario() 経路（バンドル構造・フライトログ記録・
    # コンソール捕捉・.expect チェック・注入ガードレール）を通す — 全シナリオが
    # フライトログ一式を記録するようになったため追加の env は不要。
    scenario_ns = argparse.Namespace(
        scenario=str(scn), target="vehicle", expect=None,
        duration=getattr(args, "duration", 44_000_000),
        noise=getattr(args, "noise", "off"), seed=getattr(args, "seed", 12345),
        video=False, ground_effect=None, turbulence=None,
        motor_delay=getattr(args, "motor_delay", None), unpaired=False,
        params=getattr(args, "params", None),
    )
    run_scenario(scenario_ns)   # its own PASS/FAIL print already happened; see below for gating

    # Gate the FIT on a real crash (process exit code / no bundle), NOT on
    # run_scenario's full .expect verdict. At nonzero --motor-delay the duty_max
    # check in sysid_gate.expect is EXPECTED to trip (gains were tuned on the
    # no-delay plant — the whole point of this measurement is to see how far L_total
    # moves before a retune, per simulation-policy.md backlog #1/#2 sequencing) —
    # that must not block the identification fit itself.
    # フィットの可否は「本当のクラッシュ」（プロセス終了コード／一式欠落）でゲートし、
    # run_scenario の完全な .expect 判定では止めない。--motor-delay>0 では
    # sysid_gate.expect の duty_max チェックが意図的に発火しうる（ゲインは無遅延
    # プラントでチューニング済み — L_total がリチューン前にどれだけ動くかを見るのが
    # 本計測の目的そのもの）— これで同定フィット自体を止めてはならない。
    results_path = bundle / "results.json"
    scenario_pass = None
    exit_code = None
    zip_path = None
    if results_path.exists():
        bundle_results = json.loads(results_path.read_text())
        scenario_pass = bundle_results.get("pass")
        exit_code = bundle_results.get("exit_code")
        flightlog = bundle_results.get("flightlog")
        zip_path = Path(flightlog) if flightlog else None
    if exit_code not in (0, None):
        console.error(f"emu_vehicle exited {exit_code} — a real crash, aborting fit "
                      f"(see {bundle / 'console.log'})")
        return 1
    if zip_path is None or not zip_path.exists():
        console.error(f"no flight-log bundle recorded for {scn.name} — is emu_vehicle wired "
                      "for SILS_EMU_FLIGHTLOG "
                      "(simulator/sils/devices/emu_flightlog_vehicle.cpp)?")
        return 1
    with zipfile.ZipFile(zip_path) as zf:
        if "gains.json" not in zf.namelist():
            console.error(f"no gains.json inside {zip_path}"); return 1
        gains_all = json.loads(zf.read("gains.json").decode("utf-8"))
    if scenario_pass is False:
        console.warning(f"sysid_gate.expect did NOT fully pass (see {bundle / 'console.log'}) — "
                        "proceeding with the identification fit anyway; this is expected at "
                        "nonzero --motor-delay (duty saturates on untuned gains)")

    reference = json.loads(reference_path.read_text())
    rs = _rate_backend()

    tools_dir = str(paths.root() / "tools")
    sys.path.insert(0, tools_dir)
    try:
        from sysid.loader import load_aligned
        df = load_aligned(zip_path)
    finally:
        if tools_dir in sys.path:
            sys.path.remove(tools_dir)

    console.info("Model-match gate (simulation-policy.md §4): SILS plant vs real-hardware sysid "
                 f"(motor_delay={getattr(args, 'motor_delay', None)} ms)")
    header = (f"  {'axis':6s} {'b_sils':>10s} {'b_ref':>10s} {'b_err%':>8s}  "
              f"{'Ltot_sils':>9s} {'Ltot_ref':>9s} {'Ltot_err%':>10s}  "
              f"{'T[ms]':>6s} {'L[ms]':>6s} {'coh':>5s}  verdict")
    print(header)
    rows = []
    all_pass = True
    for axis in ("roll", "pitch", "yaw"):
        fit = rs.fit_from_df(df, axis, gains=gains_all[axis])
        l_total_s = fit["T"] + fit["L"]
        ref = reference["axes"][axis]
        b_err = (fit["b"] - ref["b"]) / ref["b"]
        lt_err = (l_total_s - ref["L_total_s"]) / ref["L_total_s"]
        b_ok = abs(b_err) <= SYSID_GATE_B_TOL
        lt_ok = abs(lt_err) <= SYSID_GATE_LTOTAL_TOL
        axis_pass = bool(b_ok and lt_ok)
        all_pass = all_pass and axis_pass
        row = {
            "axis": axis,
            "b_sils": fit["b"], "b_ref": ref["b"], "b_err_pct": b_err * 100.0, "b_pass": b_ok,
            "L_total_sils_ms": l_total_s * 1e3, "L_total_ref_ms": ref["L_total_s"] * 1e3,
            "L_total_err_pct": lt_err * 100.0, "L_total_pass": lt_ok,
            "T_ms": fit["T"] * 1e3, "L_ms": fit["L"] * 1e3,
            "coherence_mean": fit["coherence_mean"], "pass": axis_pass,
        }
        rows.append(row)
        verdict = "PASS" if axis_pass else "FAIL"
        print(f"  {axis:6s} {fit['b']:10.0f} {ref['b']:10.0f} {b_err * 100.0:7.1f}%  "
              f"{l_total_s * 1e3:7.2f}ms {ref['L_total_s'] * 1e3:7.2f}ms {lt_err * 100.0:9.1f}%  "
              f"{fit['T'] * 1e3:6.2f} {fit['L'] * 1e3:6.2f} {fit['coherence_mean']:.2f}  {verdict}")

    if all_pass:
        console.success("model-match gate: PASS (all axes within tolerance)")
    else:
        console.error("model-match gate: FAIL (see per-axis table above)")

    if getattr(args, "report", None):
        report = {
            "pass": all_pass,
            "tolerances": {"b_rel": SYSID_GATE_B_TOL, "L_total_rel": SYSID_GATE_LTOTAL_TOL},
            "reference": str(reference_path),
            "motor_delay_ms": getattr(args, "motor_delay", None),
            "noise": getattr(args, "noise", "off"),
            "bundle": str(zip_path),
            "scenario_pass": scenario_pass,   # sysid_gate.expect verdict (informational — see gating note above)
            "axes": rows,
        }
        Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
        console.info(f"report written: {args.report}")

    return 0 if all_pass else 2


def _venv():
    """Return the SILS venv python, creating it (mujoco + deps) if missing."""
    py = _venv_python()
    if py.exists():
        _ensure_viz_venv_pandas(py)
        return py
    console.info("Creating SILS viz venv (mujoco 3.9.0 + matplotlib + imageio + pandas)...")
    venv_dir = _sils_dir() / "viz" / "venv"
    if subprocess.run([sys.executable, "-m", "venv", str(venv_dir)]).returncode != 0:
        console.error("venv creation failed"); return None
    pip_bin = "Scripts" if platform.is_windows() else "bin"
    pip_exe = "pip.exe" if platform.is_windows() else "pip"
    pip = venv_dir / pip_bin / pip_exe
    if subprocess.run([str(pip), "install", "-q", "mujoco==3.9.0", "numpy",
                       "matplotlib", "imageio", "imageio-ffmpeg", "pandas"]).returncode != 0:
        console.error("pip install failed"); return None
    return py


def _ensure_viz_venv_pandas(py: Path) -> None:
    """render_video.py reads the flight-log bundle through lib/sflog, which
    needs pandas -- a viz venv created before 2026-09-11 lacks it. Install it
    once into an existing venv (no-op when already importable).
    render_video.py は lib/sflog 経由でフライトログ一式を読むため pandas が要る
    -- 2026-09-11 より前に作られた viz venv には無い。既存 venv へ一度だけ入れる
    （import できれば何もしない）。"""
    probe = subprocess.run([str(py), "-c", "import pandas"], capture_output=True)
    if probe.returncode == 0:
        return
    console.info("Adding pandas to the SILS viz venv (flight-log bundle reader)...")
    subprocess.run([str(py), "-m", "pip", "install", "-q", "pandas"])
