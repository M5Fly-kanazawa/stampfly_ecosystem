"""
sf sil - Software-in-the-Loop bench (physics-based, MuJoCo, algorithm-independent)

物理ベースの SIL ベンチを操作する。ファームの本物ループをホストで走らせ、MuJoCo で
ループを閉じ、機械可読な合否(results.json)＋レビュー動画を成果物として出す。
アウトプット主導のマイルストーン(RESET_PLAN §8〜§10)を CLI から回す。

Subcommands:
  build      Build the host SIL (compat + firmware sources + MuJoCo)
  run        Run the closed loop and write the bundle (trajectory + results.json)
  video      Render the review video (MuJoCo 3D + state graphs)
  status     Show the machine verdict (results.json) for a milestone
  gate       Gate check: bundle complete AND verdict passes (output-driven)
  scenario   Run a *.scn input scenario (console/ESP-NOW) and assert outputs (E6)
  regression Run every *.scn that has a matching *.expect and gate on the aggregate
             (the automated form of the manual A/B loop in release-workflow.md; CI
             entry point for sil-regression.yml)
  milestone  build → run → video → gate in one shot (the /sil-milestone skill)
  sysid-gate Model-match gate: fit SIL rate-loop (b,T,L) vs real-hardware sysid
             (simulation-policy.md §4; distinct from the "gate" milestone check)
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
import time
from pathlib import Path
from typing import Optional

from ..utils import console, paths, platform

COMMAND_NAME = "sil"
COMMAND_HELP = "Software-in-the-Loop bench (closed-loop hover, review video, gate)"

ESTIMATORS = {"eskf": 0, "complementary": 1}
ESTIMATOR_LABELS = {"eskf": "ESKF", "complementary": "Complementary"}
NOISE_LEVELS = ["off", "n0", "n1", "n2"]
# Default sensor-noise level per milestone (RESET_PLAN §13): noise milestones run N0.
# マイルストーン別の既定ノイズ（§13）: ノイズ系マイルストーンは N0 で走る。
MILESTONE_NOISE = {"P5": "n0", "P6": "n0", "P7": "n0"}


# --- path helpers / パスヘルパ -------------------------------------------------
def _sil_dir() -> Path:
    return paths.root() / "simulator" / "sil"


def _model() -> Path:
    return _sil_dir() / "models" / "stampfly.xml"


# Well-known MSYS2 MinGW-w64 install location (winget/MSYS2 installer default).
# The SIL host bench was validated against this exact toolchain (GCC 16,
# posix-thread model — see mingw_toolchain() below); other MinGW distributions
# are not verified.
# MSYS2 MinGW-w64 の既定インストール先（winget/MSYS2 インストーラの既定）。
# SIL ホストベンチはこのツールチェーン（GCC 16, posix スレッドモデル — 下記
# mingw_toolchain() 参照）で検証済み。他の MinGW ディストリビューションは未検証。
_MSYS2_MINGW64_BIN = Path("C:/msys64/mingw64/bin")


def mingw_bin() -> Optional[Path]:
    """Locate a MinGW-w64 toolchain's bin/ directory on Windows (need
    g++/gcc/ninja together — cmake itself can come from anywhere on PATH).
    Returns None on non-Windows, or on Windows when no MinGW toolchain is
    found (the SIL build then falls back to whatever generator/compiler CMake
    picks up from PATH by default, e.g. Visual Studio).

    Checked in order: (1) a g++ already on PATH whose path contains "mingw"
    (a user's own MSYS2/MinGW setup, respected as-is); (2) the MSYS2 default
    install path. Used both to configure the build (run_build) and to run the
    resulting .exe (its libstdc++-6.dll/libgcc_s_seh-1.dll/libwinpthread-1.dll
    are not on PATH otherwise — see win_run_env()).

    Windows で MinGW-w64 ツールチェーンの bin/ を探す（g++/gcc/ninja が揃って
    いる必要がある — cmake 自体は PATH 上のどこにあってもよい）。非 Windows、
    または Windows でも MinGW が見つからない場合は None（その場合 SIL ビルドは
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
    """Environment for configuring/building/running the SIL on Windows via
    MinGW: PATH gets the MinGW bin/ prepended (compiler + the runtime DLLs
    every MinGW-built .exe needs: libstdc++-6.dll, libgcc_s_seh-1.dll,
    libwinpthread-1.dll) plus the build dir's own bin/ (libmujoco.dll — MuJoCo
    builds as a shared library there). A plain copy of os.environ (no-op) on
    non-Windows, or on Windows when mingw_bin() finds nothing (nothing to add).

    MinGW 経由で SIL を設定・ビルド・実行する Windows 向け環境: PATH の先頭に
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
        return _sil_dir() / "build-mingw"
    return _sil_dir() / "build"


def _bundle_dir(milestone: str) -> Path:
    return _sil_dir() / "viz" / f"out_{milestone.lower()}"


def _venv_python() -> Path:
    bin_dir = "Scripts" if platform.is_windows() else "bin"
    exe = "python.exe" if platform.is_windows() else "python"
    return _sil_dir() / "viz" / "venv" / bin_dir / exe


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
# ~19k files in simulator/sil/). Kept as a set (not a glob) so os.walk can
# prune dirnames in place without re-descending into a pruned subtree.
# ソース走査時に剪定するディレクトリ名（ビルド成果物・venv・キャッシュ — どれも
# コンパイル済みバイナリを構成しない。simulator/sil/ だけで build/ 配下は
# 約19000ファイル）。glob ではなく集合にして os.walk が dirnames をその場で
# 剪定し、剪定済みサブツリーに再度降りないようにする。
_SIL_STALE_SKIP_DIRS = {"build", "build-mingw", "venv", ".cache",
                       "managed_components", "__pycache__", ".git"}
_SIL_STALE_EXTS = (".cpp", ".hpp", ".c", ".h")


def _sil_source_mtime() -> float:
    """Latest mtime among simulator/sil/ and firmware/vehicle/ source files that
    feed the SIL binary (Code Identity: SIL links firmware/vehicle sources
    directly — see architecture.md). Restricted to .cpp/.hpp/.c/.h/CMakeLists.txt
    and walked with build/venv/cache dirs pruned in-place (os.walk, not
    Path.rglob — rglob would still descend into an excluded subtree before
    discarding it) to keep the scan cheap: ~600 files, well under 0.1 s, versus
    the ~19k files under simulator/sil/build/ alone if unfiltered.
    SILバイナリを構成するソースの最新mtime（simulator/sil/・firmware/vehicle/。
    Code Identity: SIL は firmware/vehicle のソースを直接リンクする）。
    .cpp/.hpp/.c/.h/CMakeLists.txt に限定し、build/venv/cache 等を os.walk で
    その場剪定（Path.rglob は除外対象のサブツリーにも一旦降りてから捨てるため
    不利）して走査コストを抑える: 約600ファイル・0.1秒未満（build/ 単体で
    約19000ファイルある無フィルタ走査に対して）。
    """
    latest = 0.0
    for base in (paths.root() / "simulator" / "sil", paths.root() / "firmware" / "vehicle"):
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d not in _SIL_STALE_SKIP_DIRS]
            for name in filenames:
                if name == "CMakeLists.txt" or os.path.splitext(name)[1] in _SIL_STALE_EXTS:
                    try:
                        latest = max(latest, os.path.getmtime(os.path.join(dirpath, name)))
                    except OSError:
                        pass  # file vanished mid-walk (e.g. concurrent edit) — ignore
    return latest


def _check_build_freshness(exe: Path, src_mtime: Optional[float] = None) -> bool:
    """Warn (never blocks — this is advisory, not a build gate) if `exe` predates
    the newest tracked SIL/firmware source file: a likely sign the sources were
    edited after the last `sf sil build`. Returns True iff a warning was printed,
    so callers with more than one exe (run_regression: vehicle + vehicle_old) can
    share a single source scan (pass a precomputed src_mtime) instead of walking
    the tree once per exe.
    `exe` が最新の SIL/ファームソースより古ければ警告する（ブロックはしない —
    あくまで参考情報。ビルドゲートではない）。最後の `sf sil build` の後にソースが
    編集された可能性のサイン。警告を出したら True を返す — 複数 exe を持つ呼び出し
    元（run_regression: vehicle と vehicle_old）が exe ごとに再走査せず、1回の
    ソース走査結果（src_mtime を渡す）を使い回せるようにする。
    """
    try:
        exe_mtime = exe.stat().st_mtime
    except OSError:
        return False
    if src_mtime is None:
        src_mtime = _sil_source_mtime()
    if src_mtime > exe_mtime:
        # console.warning() already prints the "[WARN]" prefix itself (see
        # lib/sfcli/utils/console.py) — do not repeat it in the message text.
        # console.warning() は自身で "[WARN]" 接頭辞を付ける（console.py 参照）
        # — メッセージ本文で二重に付けない。
        console.warning(f"SIL binary is older than sources — run `sf sil build` "
                       f"({exe.name})")
        return True
    return False


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the sil command with the CLI."""
    parser = subparsers.add_parser(COMMAND_NAME, help=COMMAND_HELP, description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="sil_command", metavar="<subcommand>")

    p = sub.add_parser("build", help="Build the host SIL")
    p.add_argument("-j", "--jobs", type=int, default=8)
    p.add_argument("-t", "--target", default=None, help="cmake target (default: all)")
    p.set_defaults(func=run_build)

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
    p.add_argument("--target", choices=["vehicle", "vehicle_old"], default="vehicle",
                   help="emulator binary (default: vehicle = current firmware; "
                        "vehicle_old = legacy firmware)")
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
                        "SIL's ~0 ms explicit dead time, docs/architecture/simulation-policy.md "
                        "backlog #1). Inserted in the duty-path before the motor's first-order "
                        "lag. Default OFF (0 ms, byte-identical clean path).")
    p.add_argument("--unpaired", action="store_true",
                   help="boot the vehicle UNPAIRED (skip the SIL pairing NVS seed) so it "
                        "auto-enters Pairing and binds via the injected RC — exercises the "
                        "real pairing handshake (pairing.scn)")
    p.add_argument("--param", action="append", dest="params", metavar="NAME=VALUE",
                   help="override a firmware param for this run only (repeatable, e.g. "
                        "--param rate.roll.kp=0.5e-3). Written to a temp file wired through "
                        "SIL_EMU_PARAMS_FILE; unknown names are skipped by the emulator "
                        "itself with a warning (SSOT stays in firmware's params table).")
    p.set_defaults(func=run_scenario)

    # Regression: run every *.scn that has a matching *.expect (README/TEST_MATRIX
    # "32 scenarios"; the .expect glob is authoritative) and gate on the aggregate.
    # This is the CI/pre-release entry point (sil-regression.yml, versioning.md §5).
    # 退行: .expect を伴う全 *.scn を実行し集約判定でゲートする(README/TEST_MATRIX
    # の「32本」。.expect グロブが正)。CI・リリース前のエントリポイント。
    p = sub.add_parser("regression", help="Run all *.scn/*.expect scenarios and gate (CI)")
    p.add_argument("--json-out", default=None,
                   help="write a machine-readable summary (per-scenario pass/fail) to this path")
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
    p = sub.add_parser("gui", help="Launch the SIL Web GUI (scenarios, graphs, 3D playback)")
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
    # simulator/sil/scenarios/sysid_gate.scn with the 400Hz rate-loop recorder on,
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
                       help="Model-match gate: fit SIL rate-loop (b,T,L) vs real-hardware "
                            "sysid (simulation-policy.md §4)")
    p.add_argument("--motor-delay", type=float, default=None, metavar="MS",
                   help="motor transport delay in ms, passed through to the scenario run "
                        "(see `sf sil scenario --motor-delay`). Default OFF (0 ms).")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off",
                   help="sensor noise level on the emulator Plant (default off — a clean "
                        "excitation run is what the real-hardware sysid pipeline assumes)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.add_argument("--duration", type=int, default=44_000_000,
                   help="sim duration in microseconds (default 44s, matches sysid_gate.scn)")
    p.add_argument("--param", action="append", dest="params", metavar="NAME=VALUE",
                   help="override a firmware param for this run only (repeatable, e.g. "
                        "--param rate.roll.kp=0.5e-3). Same mechanism as "
                        "`sf sil scenario --param` — see its help for details.")
    p.add_argument("--report", default=None,
                   help="write the gate verdict + per-axis fit JSON here")
    p.set_defaults(func=run_sysid_gate)

    parser.set_defaults(func=lambda a: (parser.print_help(), 0)[1])


# --- handlers / ハンドラ -------------------------------------------------------
def run_build(args: argparse.Namespace) -> int:
    # getattr defaults so the milestone flow (which lacks -j/-t) can reuse this.
    # milestone フローは -j/-t を持たないので getattr で既定値化して再利用できるようにする。
    jobs = getattr(args, "jobs", 8)
    target = getattr(args, "target", None)
    sd, bd = _sil_dir(), _build_dir()
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

    # Windows: configure for MinGW (Ninja + GCC/G++) the FIRST time only — once
    # CMakeCache.txt exists the generator/compiler are already locked in, and
    # re-passing -G would just make CMake error on a generator mismatch.
    # Windows: MinGW 向け設定（Ninja + GCC/G++）は初回のみ — CMakeCache.txt が
    # 既にあればジェネレータ/コンパイラは確定済みで、-G を渡し直すと
    # ジェネレータ不一致で CMake がエラーになる。
    cmake_cmd = [cmake_exe, "-S", str(sd), "-B", str(bd)]
    mingw = mingw_bin()
    if mingw is not None and not (bd / "CMakeCache.txt").exists():
        console.info(f"Windows: configuring for MinGW-w64 ({mingw})")
        cmake_cmd += ["-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release",
                      "-DCMAKE_C_COMPILER=gcc", "-DCMAKE_CXX_COMPILER=g++"]
    elif (not platform.is_windows() and shutil.which("ninja")
          and not (bd / "CMakeCache.txt").exists()):
        # Linux/macOS: prefer Ninja over the default Unix Makefiles generator when
        # it is on PATH (first configure only, same reasoning as the Windows branch
        # above) — faster parallel builds, notably for the FetchContent MuJoCo
        # build in CI (sil-regression.yml apt-installs ninja for exactly this).
        # Linux/macOS: PATH に ninja があれば既定の Unix Makefiles より優先する
        # （初回のみ、理由は上の Windows 分岐と同じ）— FetchContent の MuJoCo
        # ビルドで特に効く並列ビルドの高速化（sil-regression.yml はまさにこの
        # 目的で ninja を導入する）。
        console.info("Configuring with Ninja (found on PATH)")
        cmake_cmd += ["-G", "Ninja"]

    console.info("Configuring SIL (cmake)...")
    r = subprocess.run(cmake_cmd, env=env)
    if r.returncode != 0:
        console.error("cmake configure failed"); return r.returncode
    console.info("Building SIL (first time fetches MuJoCo — can take minutes)...")
    cmd = [cmake_exe, "--build", str(bd), "-j", str(jobs)]
    if target:
        cmd += ["--target", target]
    r = subprocess.run(cmd, env=env)
    if r.returncode == 0:
        console.success("SIL build OK")
    return r.returncode


def run_run(args: argparse.Namespace) -> int:
    bd = _build_dir()
    exe = bd / _exe("hover_smoke")
    if not exe.exists():
        console.error("hover_smoke not built — run 'sf sil build' first"); return 1
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
    if r.returncode == 0:
        console.success(f"Bundle written to {bundle}")
    else:
        console.error(f"Closed loop FAILED (exit {r.returncode}) — see output / results.json")
    return 0  # the verdict lives in results.json; gate decides pass/fail


def _traj_metric(traj_path: Path, name: str, t0=None, t1=None):
    """Compute a physical-truth metric from trajectory.csv for the numerical gates
    (G2 estimate tracking, G3 bounded attitude/position, G4 actuator health). The
    expect DSL is log-only (≈G1); this turns the bundle's truth+estimate columns into
    machine-judgeable numbers. Returns None on a missing file / unknown metric / empty
    window. Optional [t0, t1] seconds restrict the metric to a flight phase (e.g. the
    POS_HOLD window). trajectory.csv から物理真値メトリクスを算出（G2/G3/G4）。expect の
    ログ判定(≈G1)に対し、真値＋推定列を機械判定可能な数値にする。
    """
    import csv as _csv
    try:
        with open(traj_path, encoding="utf-8") as f:
            rows = list(_csv.DictReader(f))
    except OSError:
        return None
    def fv(r, k): return float(r[k])
    if t0 is not None and t1 is not None:
        rows = [r for r in rows if t0 <= fv(r, "t") <= t1]
    if not rows:
        return None
    def col(k): return [fv(r, k) for r in rows]
    def rms(xs): return math.sqrt(sum(x * x for x in xs) / len(xs))

    if name == "horizontal_drift_max":   # G3: max planar distance from the window's start
        cx, cy = fv(rows[0], "px"), fv(rows[0], "py")
        return max(math.hypot(fv(r, "px") - cx, fv(r, "py") - cy) for r in rows)
    if name == "roll_rmse":              # G2: est roll vs truth roll
        return rms([fv(r, "roll_est") - fv(r, "roll") for r in rows])
    if name == "pitch_rmse":             # G2: est pitch vs truth pitch
        return rms([fv(r, "pitch_est") - fv(r, "pitch") for r in rows])
    if name == "att_rmse":               # G2: combined roll+pitch attitude error magnitude
        return rms([math.hypot(fv(r, "roll_est") - fv(r, "roll"),
                               fv(r, "pitch_est") - fv(r, "pitch")) for r in rows])
    if name == "alt_rmse":               # G2: est alt vs truth alt
        return rms([fv(r, "alt_est") - fv(r, "alt") for r in rows])
    if name == "tilt_max":               # G3: max true tilt magnitude (no tumble)
        return max(math.hypot(fv(r, "roll"), fv(r, "pitch")) for r in rows)
    if name == "alt_band":               # G3: peak-to-peak altitude over the window
        a = col("alt"); return max(a) - min(a)
    if name == "alt_mean":
        a = col("alt"); return sum(a) / len(a)
    if name == "alt_min":  return min(col("alt"))
    if name == "alt_max":  return max(col("alt"))
    if name == "duty_max":               # G4: peak motor duty (saturation guard)
        return max(max(fv(r, "m0"), fv(r, "m1"), fv(r, "m2"), fv(r, "m3")) for r in rows)
    if name == "yaw_band":               # G3: peak-to-peak true heading [deg] over the
        # window (heading-hold gate). Yaw from the truth quaternion, unwrapped so a
        # continuous rotation is not hidden by the ±180° seam.
        # 窓内の真値方位 p-p [deg]（ヘディングホールド用ゲート）。真値クォータニオン
        # から方位を取り、連続回転が ±180° の継ぎ目で隠れないようアンラップする。
        yaws = []
        prev = None
        off = 0.0
        for r in rows:
            qw, qx, qy, qz = fv(r, "qw"), fv(r, "qx"), fv(r, "qy"), fv(r, "qz")
            y = math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                        1 - 2 * (qy * qy + qz * qz)))
            if prev is not None:
                d = y - prev
                if d > 180.0:
                    off -= 360.0
                elif d < -180.0:
                    off += 360.0
            prev = y
            yaws.append(y + off)
        return max(yaws) - min(yaws)
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
    backlog) rather than a broken assertion — `sf sil regression` reports it
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
    （docs/architecture/simulation-policy.md のバックログ参照）。`sf sil regression`
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
                 traj_path: Path = None):
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
                                              numerical physical-truth gate from
                                              trajectory.csv (G2/G3/G4). op ∈ < <= > >= ;
                                              optional "in t0 t1" restricts to a phase.
                                              names: horizontal_drift_max, roll_rmse,
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
            # Numerical physical-truth gate from trajectory.csv (G2/G3/G4). The
            # optional "in <t0> <t1>" window restricts it to a flight phase.
            # trajectory.csv からの数値ゲート。"in t0 t1" で飛行フェーズに限定。
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
            m = _traj_metric(traj_path, name, t0, t1) if traj_path else None
            win = f" in [{t0},{t1}]" if t0 is not None else ""
            if m is None:
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": False,
                               "detail": "no trajectory / unknown metric / empty window"})
            else:
                ok = _METRIC_OPS[op](m, want)
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": bool(ok),
                               "detail": f"{name}={m:.4f}"})
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
    bd = _build_dir()
    exe = bd / _exe("emu_vehicle" if target == "vehicle" else "emu_vehicle_old")
    if not exe.exists():
        console.error(f"{exe.name} not built — run 'sf sil build' first"); return 1
    # Build-freshness advisory (skippable — see SF_SIL_SKIP_FRESHNESS_CHECK): warn if
    # this exe predates its own sources. run_regression sets the env var after doing
    # this check ONCE up front, so its 30+ per-scenario subprocess calls into this
    # function don't each re-warn (or re-walk the source tree) redundantly.
    # ビルド鮮度の参考警告（SF_SIL_SKIP_FRESHNESS_CHECK でスキップ可）: この exe が
    # 自身のソースより古ければ警告する。run_regression は冒頭で1回だけ判定した後に
    # この環境変数を立てるので、以降30本超のシナリオ用子プロセスがこの関数を呼んでも
    # 重複警告・重複走査は起きない。
    if not os.environ.get("SF_SIL_SKIP_FRESHNESS_CHECK"):
        _check_build_freshness(exe)
    scn = Path(args.scenario)
    if not scn.exists():
        console.error(f"scenario not found: {scn}"); return 1

    bundle = _sil_dir() / "viz" / f"out_scn_{scn.stem}"
    bundle.mkdir(parents=True, exist_ok=True)
    events = bundle / "events.jsonl"
    traj = bundle / "trajectory.csv"
    # SIL_EMU_TRAJ tells the emulator to record a render_video.py-compatible
    # trajectory.csv into the bundle (so --video can render the run). Harmless and
    # deterministic; the recorder is a no-op in any emulator that ignores it.
    # SIL_EMU_TRAJ は render_video.py 互換の trajectory.csv をバンドルへ記録させる
    # （--video で描画可能に）。決定論的で無害、未対応エミュレータでは no-op。
    env = dict(win_run_env(bd), SIL_EMU_EVENTS=str(events), SIL_EMU_TRAJ=str(traj))

    # SIL_EMU_NOISE/SIL_EMU_SEED turn on the seeded N0 sensor-noise model on the
    # emulator Plant (§13 P5). Default "off" → env passed but the emulator keeps the
    # clean path (byte-identical), so the no-noise scenario verdict is unaffected.
    # SIL_EMU_NOISE/SIL_EMU_SEED でエミュレータ Plant の N0 ノイズを ON（§13 P5）。
    # 既定 "off" では従来のクリーン経路（byte-identical）を保つ。
    noise = getattr(args, "noise", "off") or "off"
    seed = getattr(args, "seed", 12345)
    env["SIL_EMU_NOISE"] = noise
    env["SIL_EMU_SEED"] = str(seed)

    # --ground-effect enables the plant's near-floor lift boost (default OFF → clean path
    # byte-identical) so the touchdown "float" is reproduced and the firmware stalled-descent
    # land detector can be verified. Bare flag = default strength; a number sets the gain.
    # --ground-effect でプラントの接地近傍揚力ブーストを有効化（既定 OFF→クリーン経路バイト一致）。
    # 着陸「フロート」を再現しファームの降下停滞接地検出器を検証。素のフラグ=既定強度、数値で gain 指定。
    ge = getattr(args, "ground_effect", None)
    if ge is not None:
        env["SIL_EMU_GROUND_EFFECT"] = str(ge)

    # --turbulence enables a deterministic 1-3 Hz lateral disturbance (force [N]) to excite
    # the attitude-wobble band for the wobble-minimization study. Default OFF.
    tb = getattr(args, "turbulence", None)
    if tb is not None:
        env["SIL_EMU_TURBULENCE"] = str(tb)

    # --motor-delay sets the duty-path transport delay (model-match retrofit #1: real-hw
    # identified L=14.7/8.4/11.0ms roll/pitch/yaw vs the SIL's current ~0ms explicit dead
    # time). Default OFF (byte-identical clean path).
    # --motor-delay で duty 経路の輸送遅れを設定（モデル一致改修#1）。既定 OFF。
    md = getattr(args, "motor_delay", None)
    if md is not None:
        env["SIL_EMU_MOTOR_DELAY"] = str(md)

    # --param NAME=VALUE (repeatable): temporary, single-run firmware param overrides,
    # routed through the EXISTING SIL_EMU_PARAMS_FILE mechanism (emu_main.cpp reads
    # "<name> <value>" lines from a text file and applies each through the SSOT params
    # table's own type-correct setter, warning+skipping unknown names itself — see
    # simulator/sil/emu/emu_main.cpp). This CLI option is a thin wrapper: parse
    # "NAME=VALUE" tokens into that line format, write them into THIS scenario's OWN
    # bundle directory (not a tempfile — easier to inspect while debugging a run, and
    # nothing to clean up afterward), and point SIL_EMU_PARAMS_FILE at it. No parameter
    # name validation happens here — that stays the emulator's job, so the params table
    # remains the single source of truth (no duplicate allowlist to drift).
    # --param NAME=VALUE（繰り返し指定可）: 既存の SIL_EMU_PARAMS_FILE 機構を通した、
    # 単発実行限りの一時的なファームパラメータ上書き（emu_main.cpp が "<名前> <値>" 行の
    # テキストファイルを読み、SSOT パラメータテーブル自身の型正しいセッタで適用し、未知の
    # 名前は自身で警告してスキップする — simulator/sil/emu/emu_main.cpp 参照）。この CLI
    # オプションは薄いラッパーに過ぎない: "NAME=VALUE" トークンをその行形式に変換し、
    # このシナリオ自身のバンドルディレクトリ（tempfile ではなく — 実行内容をデバッグ時に
    # 確認しやすく、後始末も不要）に書き込み、SIL_EMU_PARAMS_FILE をそこへ向ける。名前の
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
        # An externally-set SIL_EMU_PARAMS_FILE (e.g. from the SIL GUI's own parameter
        # panel, or a wrapping script) would otherwise be silently clobbered below —
        # say so explicitly rather than leaving a confusing "my env var was ignored".
        # 外部で既に設定済みの SIL_EMU_PARAMS_FILE（SIL GUI のパラメータパネルや、
        # 呼び出し元スクリプト等）は以下で無言のまま上書きされてしまう — 「自分の環境
        # 変数が無視された」という混乱を避けるため、ここで明示的に表示する。
        if env.get("SIL_EMU_PARAMS_FILE"):
            console.info("--param overrides externally-set "
                         f"SIL_EMU_PARAMS_FILE={env['SIL_EMU_PARAMS_FILE']}")
        override_path = bundle / "params_override.txt"
        override_path.write_text(
            "\n".join(valid_lines) + ("\n" if valid_lines else ""), encoding="utf-8")
        env["SIL_EMU_PARAMS_FILE"] = str(override_path)
        console.info(f"{len(valid_lines)} param(s) queued via --param")

    # --unpaired: skip the pairing NVS seed so the vehicle boots unpaired and runs the
    # real pairing handshake (auto-enter Pairing → bind via injected RC). Default keeps
    # the vehicle pre-paired so flight scenarios are not gated on pairing timing.
    # --unpaired: ペアリング NVS seed をスキップし機体を未ペア起動させ、実ハンドシェイク
    # （自動 Pairing 突入→注入 RC で bind）を走らせる。既定はペア済み起動で飛行シナリオを
    # ペアリングのタイミングに依存させない。
    if getattr(args, "unpaired", False):
        env["SIL_EMU_UNPAIRED"] = "1"

    # extra_env: additional env vars for sil subcommands BUILT ON TOP of run_scenario
    # (e.g. sysid-gate's SIL_EMU_RATE_STREAM). Not part of the `scenario` CLI surface
    # itself — plain argparse.Namespace from run_scenario's own parser never sets this.
    # extra_env: run_scenario の上に構築される他の sil サブコマンド用の追加 env
    # （例: sysid-gate の SIL_EMU_RATE_STREAM）。`scenario` 自体の CLI 引数ではない。
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
            if "SIL_EMU_PARAMS_FILE=" in line:
                console.info(f"  {line}")
            elif "PARAMS_FILE: unknown param" in line:
                console.warning(f"  {line}")
            elif "PARAMS_FILE:" in line and "rejected" in line:
                console.warning(f"  {line}")

    # Guardrail: every scenario injects at least one event, so events.jsonl must
    # exist and be non-empty. A target that is not wired for scripted input (e.g.
    # an emulator entry that ignores argv[3]/SIL_EMU_EVENTS) would inject NOTHING
    # and could otherwise masquerade as a passing run on exit==0 — fail it loudly.
    # ガードレール: シナリオは必ず1事象以上注入するので events.jsonl は非空のはず。台本入力
    # 未配線のターゲット（argv[3]/SIL_EMU_EVENTS を無視）は何も注入せず exit==0 で偽合格しうる。
    injected = events.exists() and events.stat().st_size > 0
    inject_check = {"name": "input injected (events.jsonl non-empty)", "pass": injected,
                    "detail": f"{events.stat().st_size if events.exists() else 0} bytes"
                              + ("" if injected else f" — is {exe.name} wired for scripted input?")}

    expect = Path(args.expect) if args.expect else scn.with_suffix(".expect")
    xfail_reason = None
    if expect.exists():
        checks, verdict, xfail_reason = _eval_expect(expect, r.stdout, r.stderr, r.returncode, traj)
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
        # Known-fail marker (docs/architecture/simulation-policy.md backlog), if the
        # .expect carries one — informational only here; `sf sil scenario` still
        # reports the RAW verdict above (see xfail_reason annotation below). Only
        # `sf sil regression` changes gating behavior on this field.
        # 既知失敗マーカー（simulation-policy.md バックログ）。ここでは情報表示のみ
        # — `sf sil scenario` 単体では素の合否を報告する。ゲート挙動を変えるのは
        # `sf sil regression` のみ。
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
    # xfail annotation: a single `sf sil scenario` run always shows the RAW verdict
    # above (PASS is PASS, FAIL is FAIL) — this is just a one-line note that the
    # .expect carries a known-fail marker. Only `sf sil regression` changes gating
    # behavior (KNOWN-FAIL / XPASS) on this field.
    # xfail 注記: 単体の `sf sil scenario` は常に上の素の合否を表示する（xfail が
    # あっても FAIL は FAIL と見せる）。これは .expect に既知失敗マーカーがある
    # ことを示す1行注記のみ。ゲート挙動を変えるのは `sf sil regression` のみ。
    if xfail_reason:
        console.warning(f"[xfail marker] {xfail_reason}")

    # --video: render a review MP4 (MuJoCo 3D + state graphs) from the trajectory the
    # run just recorded. Only on PASS and only if a trajectory was actually written.
    # --video: 実行が記録した軌跡からレビュー動画（MuJoCo 3D＋状態グラフ）を描画。
    # PASS かつ軌跡が書かれた場合のみ。
    if verdict and getattr(args, "video", False):
        py = _venv()
        if py is None:
            console.error("viz venv missing — cannot render (see simulator/sil/viz)")
        elif not (traj.exists() and traj.stat().st_size > 0):
            console.error(f"no trajectory.csv in {bundle} — is {exe.name} wired for "
                          "SIL_EMU_TRAJ recording?")
        else:
            out = bundle / f"scn_{scn.stem}.mp4"
            console.info("Rendering review video (MuJoCo 3D + state graphs)...")
            rv = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
                                 "--model", str(_model()), "--bundle", str(bundle),
                                 "--out", str(out), "--fps", "50"])
            if rv.returncode == 0:
                console.success(f"Video: {out}")
            else:
                console.error("render_video.py failed")
    return 0 if verdict else 2


# --- regression: run every *.scn/*.expect pair and gate on the aggregate ------------
# Each scenario documents its OWN required invocation (target/duration/unpaired/...)
# in a "sf sil scenario simulator/sil/scenarios/<name>.scn ..." comment inside its own
# header (e.g. api_flight.scn: "# Run: ... --target vehicle --duration 40000000") —
# the .scn file is the single source of truth for how it must be run, so this parses
# that line instead of hand-maintaining a duplicate table that would drift.
# 各シナリオは自身のヘッダコメント内の "sf sil scenario .../<name>.scn ..." 行で
# 自分の必要な呼び出し(target/duration/unpaired等)を宣言する（例:
# api_flight.scn の "# Run: ... --target vehicle --duration 40000000"）。.scn 自身が
# 自分の呼び出し方法の唯一の正なので、重複管理で陳腐化するテーブルを持たず、この行を
# 解析する。
_RUN_LINE_RE = re.compile(r"sf sil scenario\s+\S*?scenarios/(?P<name>[\w.]+)\.scn(?P<rest>.*)")

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
    """Return the extra `sf sil scenario` CLI args this .scn documents for itself
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
    a heavy dependency into the SIL regression path.
    subprocess ではなくプロセス内 import。sfcli 全体で tools/ 配下モジュールに
    使われる sys.path 差し込みの流儀に従う（lib/sfcli/commands/params.py の
    run_check()、sf sysid の run_fit() も同様の例）。params_audit は標準
    ライブラリのみに依存するため、SIL 回帰の経路に重い依存を持ち込まない。
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


def run_regression(args: argparse.Namespace) -> int:
    scn_dir = _sil_dir() / "scenarios"
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
    # across firmware/SIL/simulator sources, no code-gen pipeline yet, Phase 1
    # of docs/architecture/simulation-policy.md). A silent drift here would
    # make every scenario below PASS against the wrong plant, so this must run
    # first and fail the whole regression immediately — no point spending
    # minutes on scenarios whose physics reference is already known-wrong.
    # シナリオ実行前に、手動複製された物理パラメータの整合性
    # （tools/params_audit — C_T/C_Q/kappa/慣性等をファーム/SIL/シミュレータへ
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
    src_mtime = _sil_source_mtime()
    bd = _build_dir()
    for exe_name in ("emu_vehicle", "emu_vehicle_old"):
        exe = bd / _exe(exe_name)
        if exe.exists():
            _check_build_freshness(exe, src_mtime=src_mtime)
    os.environ["SF_SIL_SKIP_FRESHNESS_CHECK"] = "1"

    console.info(f"SIL regression: {len(scenarios)} scenario(s) with a matching .expect...")
    results = []
    for scn in scenarios:
        extra = _scenario_invocation(scn)
        target = extra[extra.index("--target") + 1]
        cmd = [sys.executable, "-m", "sfcli", "sil", "scenario", str(scn)] + extra
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
        #                                      wasn't removed — SIL is
        #                                      deterministic, so this MUST gate
        #                                      the regression, or a fixed issue
        #                                      could silently stay "known-fail"
        #                                      forever)
        # xfail: .expect 先頭の "xfail: <理由>" 指令は、このシナリオの現在の失敗が
        # 追跡中の既知課題（simulation-policy.md バックログ）であり壊れたアサー
        # ションではないことを示す。分類は上記コメント参照。SIL は決定論的なので
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
    # n_fail (real, un-marked FAILs) and n_xpass (fixed-but-still-marked) are the
    # only two statuses that gate the regression — KNOWN-FAIL is informational.
    # n_fail（無印の実失敗）と n_xpass（直ったのにマーカー残存）だけが回帰をゲート
    # する — KNOWN-FAIL は情報表示のみ。
    n_unexpected = n_fail + n_xpass

    if getattr(args, "json_out", None):
        summary = {"total": len(results), "pass": n_pass, "known_fail": n_known_fail,
                  "xpass": n_xpass, "fail": n_unexpected, "scenarios": results}
        Path(args.json_out).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        console.info(f"Summary written to {args.json_out}")

    if n_unexpected:
        console.error(f"SIL regression: {n_fail} FAIL + {n_xpass} XPASS "
                     f"(of {len(results)}; known-fail={n_known_fail}):")
        for r in results:
            if r["status"] == "FAIL":
                console.error(f"  FAIL {r['name']} (target={r['target']})")
            elif r["status"] == "XPASS":
                console.error(f"  XPASS {r['name']} (target={r['target']}) — "
                             f"remove the xfail marker in {r['name']}.expect")
        return 1
    console.success(f"SIL regression: {n_pass} PASS + {n_known_fail} KNOWN-FAIL "
                    f"({len(results)} total) known-fail={n_known_fail}")
    return 0


def run_gui(args: argparse.Namespace) -> int:
    """Launch the SIL Web GUI (simulator/sil/gui/server.py). Serves the single-page app and
    a small JSON API that drives `sf sil scenario` runs and reads the bundles back for the
    browser (graphs + live 3D). Localhost only. SIL Web GUI を起動。localhost のみ。"""
    gui_dir = _sil_dir() / "gui"
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
                      f"(try `sf sil gui --port <other>`)")
        return 1
    return 0


def run_video(args: argparse.Namespace) -> int:
    py = _venv()
    if py is None:
        return 1
    bundle = _bundle_dir(args.milestone)
    if not (bundle / "trajectory.csv").exists():
        console.error(f"No trajectory in {bundle} — run 'sf sil run' first"); return 1
    fps = getattr(args, "fps", 50)
    out = bundle / f"{args.milestone.lower()}_flight.mp4"
    console.info("Rendering review video (MuJoCo 3D + state graphs)...")
    r = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
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
        console.error("hover_smoke not built — run 'sf sil build' first"); return 1
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
        # hover_smoke writes the bundle even on a G3 fail; only a hard early exit
        # (e.g. model load) leaves no files. Require both before render/aggregate so
        # a crash surfaces here, not as an opaque traceback downstream.
        # G3不合格でもバンドルは書かれる。ファイルが無いのは早期異常終了のみ。先に要求する。
        if not (sub / "trajectory.csv").exists() or not (sub / "results.json").exists():
            console.error(f"run '{est}' wrote no bundle (exit {rc}) — aborting comparison")
            return 1
        runs[est] = sub

    out = bundle / f"{args.milestone.lower()}_compare.mp4"
    console.info("Rendering side-by-side comparison (twin 3D + overlay graphs)...")
    fps = getattr(args, "fps", 50)
    r = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
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
        console.error(f"No results.json for {args.milestone} — run 'sf sil run' first"); return 1
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
    r = subprocess.run([str(py), str(_sil_dir() / "tools" / "sil_gate.py"),
                        str(_bundle_dir(args.milestone))])
    return r.returncode


def run_milestone(args: argparse.Namespace) -> int:
    # The milestone only needs hover_smoke (run) + render_video (video, via venv) —
    # build just that target so an unrelated SIL target can't break the milestone.
    # milestone に必要なのは hover_smoke（run）と render_video（video, venv経由）だけ。
    # 無関係な SIL ターゲットが milestone を壊さないよう、そのターゲットだけビルドする。
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
    """Run sysid_gate.scn with the 400Hz rate-loop recorder on (SIL_EMU_RATE_STREAM),
    fit (b,T,L) per axis with rate_sysid.py, and compare against the real-hardware
    reference. sysid_gate.scn を SIL_EMU_RATE_STREAM 付きで実行し、rate_sysid.py で
    軸別 (b,T,L) をフィットして実機基準値と比較する。"""
    scn = _sil_dir() / "scenarios" / "sysid_gate.scn"
    if not scn.exists():
        console.error(f"scenario not found: {scn}"); return 1
    reference_path = _reference_path()
    if not reference_path.exists():
        console.error(f"reference missing: {reference_path} — run "
                      "analysis/reports/rate_sysid_reference/make_reference.py first")
        return 1

    # Bundle dir matches what run_scenario would derive on its own
    # (_sil_dir()/viz/out_scn_<stem>) — computed here only to know the rate-stream
    # CSV path BEFORE the run (run_scenario creates the directory itself).
    # バンドルディレクトリは run_scenario が自前で導出するのと同じ場所
    # （_sil_dir()/viz/out_scn_<stem>）— run前に CSV パスを知るためだけにここで計算
    # する（ディレクトリ自体は run_scenario が作る）。
    bundle = _sil_dir() / "viz" / f"out_scn_{scn.stem}"
    csv_path = bundle / "rate_stream.csv"
    gains_path = Path(str(csv_path) + ".gains.json")

    # Drive the same run_scenario() path every other scenario uses (bundle layout,
    # console capture, .expect check, injection guardrail) — only difference is the
    # extra SIL_EMU_RATE_STREAM env var. Constructing the Namespace by hand (rather
    # than argparse) keeps this independent of the `scenario` subcommand's own flags.
    # 他の全シナリオと同じ run_scenario() 経路（バンドル構造・コンソール捕捉・.expect
    # チェック・注入ガードレール）を通す — 違いは SIL_EMU_RATE_STREAM だけ。
    scenario_ns = argparse.Namespace(
        scenario=str(scn), target="vehicle", expect=None,
        duration=getattr(args, "duration", 44_000_000),
        noise=getattr(args, "noise", "off"), seed=getattr(args, "seed", 12345),
        video=False, ground_effect=None, turbulence=None,
        motor_delay=getattr(args, "motor_delay", None), unpaired=False,
        params=getattr(args, "params", None),
        extra_env={"SIL_EMU_RATE_STREAM": str(csv_path)},
    )
    run_scenario(scenario_ns)   # its own PASS/FAIL print already happened; see below for gating

    # Gate the FIT on a real crash (process exit code / no CSV), NOT on
    # run_scenario's full .expect verdict. At nonzero --motor-delay the duty_max
    # check in sysid_gate.expect is EXPECTED to trip (gains were tuned on the
    # no-delay plant — the whole point of this measurement is to see how far L_total
    # moves before a retune, per simulation-policy.md backlog #1/#2 sequencing) —
    # that must not block the identification fit itself.
    # フィットの可否は「本当のクラッシュ」（プロセス終了コード／CSV欠落）でゲートし、
    # run_scenario の完全な .expect 判定では止めない。--motor-delay>0 では
    # sysid_gate.expect の duty_max チェックが意図的に発火しうる（ゲインは無遅延
    # プラントでチューニング済み — L_total がリチューン前にどれだけ動くかを見るのが
    # 本計測の目的そのもの）— これで同定フィット自体を止めてはならない。
    results_path = bundle / "results.json"
    scenario_pass = None
    exit_code = None
    if results_path.exists():
        bundle_results = json.loads(results_path.read_text())
        scenario_pass = bundle_results.get("pass")
        exit_code = bundle_results.get("exit_code")
    if exit_code not in (0, None):
        console.error(f"emu_vehicle exited {exit_code} — a real crash, aborting fit "
                      f"(see {bundle / 'console.log'})")
        return 1
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        console.error(f"no rate stream at {csv_path} — is emu_vehicle wired for "
                      "SIL_EMU_RATE_STREAM (simulator/sil/devices/emu_rate_stream.cpp)?")
        return 1
    if not gains_path.exists():
        console.error(f"no gains sidecar at {gains_path}"); return 1
    if scenario_pass is False:
        console.warning(f"sysid_gate.expect did NOT fully pass (see {bundle / 'console.log'}) — "
                        "proceeding with the identification fit anyway; this is expected at "
                        "nonzero --motor-delay (duty saturates on untuned gains)")

    gains_all = json.loads(gains_path.read_text())
    reference = json.loads(reference_path.read_text())
    rs = _rate_backend()

    console.info("Model-match gate (simulation-policy.md §4): SIL plant vs real-hardware sysid "
                 f"(motor_delay={getattr(args, 'motor_delay', None)} ms)")
    header = (f"  {'axis':6s} {'b_sil':>10s} {'b_ref':>10s} {'b_err%':>8s}  "
              f"{'Ltot_sil':>9s} {'Ltot_ref':>9s} {'Ltot_err%':>10s}  "
              f"{'T[ms]':>6s} {'L[ms]':>6s} {'coh':>5s}  verdict")
    print(header)
    rows = []
    all_pass = True
    for axis in ("roll", "pitch", "yaw"):
        fit = rs.fit_from_csv(str(csv_path), axis, gains=gains_all[axis])
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
            "b_sil": fit["b"], "b_ref": ref["b"], "b_err_pct": b_err * 100.0, "b_pass": b_ok,
            "L_total_sil_ms": l_total_s * 1e3, "L_total_ref_ms": ref["L_total_s"] * 1e3,
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
            "csv": str(csv_path),
            "scenario_pass": scenario_pass,   # sysid_gate.expect verdict (informational — see gating note above)
            "axes": rows,
        }
        Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
        console.info(f"report written: {args.report}")

    return 0 if all_pass else 2


def _venv():
    """Return the SIL venv python, creating it (mujoco + deps) if missing."""
    py = _venv_python()
    if py.exists():
        return py
    console.info("Creating SIL viz venv (mujoco 3.9.0 + matplotlib + imageio)...")
    venv_dir = _sil_dir() / "viz" / "venv"
    if subprocess.run([sys.executable, "-m", "venv", str(venv_dir)]).returncode != 0:
        console.error("venv creation failed"); return None
    pip_bin = "Scripts" if platform.is_windows() else "bin"
    pip_exe = "pip.exe" if platform.is_windows() else "pip"
    pip = venv_dir / pip_bin / pip_exe
    if subprocess.run([str(pip), "install", "-q", "mujoco==3.9.0", "numpy",
                       "matplotlib", "imageio", "imageio-ffmpeg"]).returncode != 0:
        console.error("pip install failed"); return None
    return py
