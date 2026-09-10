"""
Matplotlib backend selection for sf CLI plot commands.
sf CLI のプロットコマンド向け matplotlib バックエンド選択。

sf CLI plot commands (log viz, sysid fit/noise) must not depend on
matplotlib's automatic backend selection, because Windows Pythons
frequently lack a working tcl/tk: matplotlib silently falls back to the
headless "Agg" backend (plt.show() then prints a UserWarning and no
window ever appears) or tkinter raises TclError ("Can't find a usable
init.tcl") when the pyenv-win install has no complete Tcl/Tk runtime.
A tutorial session on Windows hit this (2026-09-10); both failure modes
were then reproduced on macOS by faking those conditions -- see
docs/guides/troubleshooting.md.

This module probes the available GUI backends up front (before anyone
imports matplotlib.pyplot, which locks in a backend on first import),
falls back to "Agg" when none work, and provides a "save PNG next to the
log and open it with the OS default image viewer" path so a plot command
still produces something useful with zero GUI backend.

sf CLI のプロットコマンド（log viz、sysid fit/noise）は matplotlib の自動
バックエンド選択に依存してはならない。Windows の Python には tcl/tk が
正しく動作しないことが多く、matplotlib は無言でヘッドレスな "Agg"
バックエンドへフォールバックする（plt.show() は UserWarning を出すだけで
ウィンドウは一切現れない）か、あるいは pyenv-win 環境で Tcl/Tk 一式が
不完全だと tkinter が TclError（"Can't find a usable init.tcl"）を送出する。
Windows でのチュートリアル講習（2026-09-10）でこれが起き、その後
どちらの故障モードも条件を模擬して macOS 上で再現した -- 詳細は
docs/guides/troubleshooting.md を参照。

このモジュールは（最初の import で選択が固定される matplotlib.pyplot を
誰かが import する前に）利用可能な GUI バックエンドを先に調べ、どれも
使えなければ "Agg" にフォールバックし、「ログの隣に PNG を保存して OS 標準の
画像ビューアで開く」という経路を提供する。これにより GUI バックエンドが
ゼロでもプロットコマンドは何かしら役に立つ結果を残す。
"""

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

# Longest single-line probe-failure reason kept in BackendInfo.reason
# before truncation (keeps console output to one readable line per probe).
# BackendInfo.reason に残す1個のプローブ失敗理由の最大文字数（切り詰め前）。
# コンソール出力をプローブ1件あたり1行の読みやすい長さに保つ。
REASON_MAX_CHARS = 120


@dataclass
class BackendInfo:
    """Result of select_backend(): which matplotlib backend ended up
    active, whether it can show an interactive window, and (when a
    headless backend was chosen) why.
    select_backend() の結果: 実際に有効化された matplotlib バックエンド、
    インタラクティブなウィンドウを表示できるか、（ヘッドレスなバックエンドを
    選んだ場合は）その理由。"""

    name: str          # Activated backend, lowercase, e.g. "macosx", "tkagg", "agg"
    interactive: bool  # True when a plot window can actually be shown
    reason: str        # Why a headless backend was chosen ("" when interactive)


# Backend names that never open a window (matplotlib's own headless/file
# renderers). Used to classify an MPLBACKEND override without probing it.
# ウィンドウを開かないバックエンド名一覧（matplotlib 本体のヘッドレス/
# ファイル出力系レンダラ）。MPLBACKEND による上書き値をプローブせずに
# 分類するために使う。
HEADLESS_BACKENDS = frozenset({"agg", "pdf", "ps", "svg", "cairo", "template", "pgf"})


def gui_backend_candidates() -> List[str]:
    """GUI backends to try, in probe order, for the current platform.
    現在のプラットフォーム向けに試す GUI バックエンドを、試す順番で返す。"""
    if sys.platform == "darwin":
        return ["macosx", "qtagg", "tkagg"]
    if sys.platform == "win32":
        # tkagg is listed before qtagg on Windows on purpose: tk ships
        # with the python.org installer, while a broken/partial Qt
        # install can abort the whole process on import (an OS-level
        # crash, not a catchable exception) -- so Qt must not be probed
        # first.
        # tkagg を Windows で qtagg より先に置くのは意図的: tk は
        # python.org 版インストーラに同梱されているが、壊れた/不完全な
        # Qt インストールは import 時にプロセスごと落ちることがある
        # （捕捉できない OS レベルのクラッシュ）ため、Qt を最初に
        # 試してはならない。
        return ["tkagg", "qtagg", "wxagg"]
    return ["qtagg", "gtk3agg", "tkagg", "wxagg"]


def _probe_tkagg() -> None:
    """Raises if Tk cannot actually open a window (e.g. pyenv-win's
    "Can't find a usable init.tcl"); importing tkinter alone is not
    enough to catch that.
    Tk が実際にウィンドウを開けない場合（pyenv-win の "Can't find a
    usable init.tcl" 等）に例外を送出する。tkinter の import だけでは
    この故障は検出できない。"""
    import tkinter

    root = tkinter.Tk()
    try:
        root.withdraw()
    finally:
        root.destroy()


def _probe_qtagg() -> None:
    """Import only -- do not create a QApplication (that would be a
    visible side effect just for probing).
    import のみ行う -- QApplication は作らない（プローブのためだけに
    目に見える副作用を起こさないため）。"""
    from matplotlib.backends.qt_compat import QtWidgets  # noqa: F401


def _probe_macosx() -> None:
    import matplotlib.backends._macosx  # noqa: F401


def _probe_wxagg() -> None:
    import wx  # noqa: F401


def _probe_gtk3agg() -> None:
    import gi

    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk  # noqa: F401


# Each probe raises on failure, returns None on success.
# 各プローブは失敗時に例外を送出し、成功時は None を返す。
PROBES = {
    "tkagg": _probe_tkagg,
    "qtagg": _probe_qtagg,
    "macosx": _probe_macosx,
    "wxagg": _probe_wxagg,
    "gtk3agg": _probe_gtk3agg,
}


def _describe_failure(name: str, exc: Exception) -> str:
    """One-line "backend: ExceptionType: message" summary of a probe
    failure, truncated to REASON_MAX_CHARS.
    プローブ失敗の1行要約「backend: 例外型: メッセージ」を
    REASON_MAX_CHARS に切り詰めて返す。"""
    first_line = str(exc).splitlines()[0] if str(exc) else ""
    if len(first_line) > REASON_MAX_CHARS:
        first_line = first_line[:REASON_MAX_CHARS] + "..."
    return f"{name}: {type(exc).__name__}: {first_line}"


def select_backend(want_window: bool = True) -> BackendInfo:
    """Activate a matplotlib backend and report whether it can show a
    window. Call this BEFORE importing matplotlib.pyplot or any tools/
    plotting module that imports it -- the backend can only be chosen
    before pyplot's first import.
    matplotlib バックエンドを有効化し、ウィンドウを表示できるかを返す。
    matplotlib.pyplot や、それを import する tools/ 配下のプロット
    モジュールより先に呼ぶこと -- バックエンドは pyplot が最初に import
    される前にしか選べない。

    Raises ImportError naturally if matplotlib itself is not installed
    (callers already handle ImportError around their plotting imports).
    matplotlib 自体が未インストールなら自然に ImportError を送出する
    （呼び出し側はプロット関連 import の周りで ImportError を既に処理する）。
    """
    import matplotlib

    env_backend = os.environ.get("MPLBACKEND")
    if env_backend:
        # An explicit override wins outright -- we neither probe nor
        # call matplotlib.use() here; matplotlib itself will honor it.
        # 明示的な上書きは無条件で優先する -- ここではプローブも
        # matplotlib.use() も行わない。matplotlib 自身が尊重する。
        name = env_backend.lower()
        interactive = name not in HEADLESS_BACKENDS
        return BackendInfo(name, interactive, "" if interactive else f"MPLBACKEND={env_backend}")

    if not want_window:
        matplotlib.use("agg")
        return BackendInfo("agg", False, "")

    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY") \
            and not os.environ.get("WAYLAND_DISPLAY"):
        matplotlib.use("agg")
        return BackendInfo("agg", False, "no DISPLAY/WAYLAND_DISPLAY (headless session)")

    failures: List[str] = []
    for name in gui_backend_candidates():
        probe = PROBES[name]
        try:
            probe()
            matplotlib.use(name)
        except Exception as exc:  # noqa: BLE001 - probing must not crash sf
            failures.append(_describe_failure(name, exc))
            continue
        return BackendInfo(name, True, "")

    matplotlib.use("agg")
    return BackendInfo("agg", False, "; ".join(failures))


def force_headless() -> None:
    """Switch an already-imported pyplot to Agg. Used when a GUI backend
    passed its import-time probe but still fails at draw/show time (a
    runtime-only Tk/Qt error) -- the caller retries the same plot headless.
    import 済みの pyplot を Agg へ切り替える。GUI バックエンドが import 時の
    プローブは通過したのに描画/表示時に失敗する場合（実行時のみ現れる
    Tk/Qt エラー）に使う -- 呼び出し側は同じプロットをヘッドレスで再試行する。
    """
    import matplotlib.pyplot as plt

    plt.close("all")
    plt.switch_backend("agg")


def default_png_path(source: Path, suffix: str = "") -> Path:
    """<source stem><suffix>.png next to source, e.g. flight.csv ->
    flight.png, or flight_fit.png when suffix="_fit".
    ソースの隣に <ソースのstem><suffix>.png を作る。例: flight.csv ->
    flight.png、suffix="_fit" なら flight_fit.png。"""
    return source.parent / f"{source.stem}{suffix}.png"


def open_with_default_viewer(path: Path) -> bool:
    """Open path with the OS default viewer. Never raises; returns False
    on any failure so callers can keep going with just the saved file.
    OS 標準のビューアで path を開く。例外は送出しない -- 失敗時は
    False を返し、呼び出し側は保存済みファイルだけで処理を続けられる。"""
    try:
        if sys.platform == "win32":
            os.startfile(str(path))  # type: ignore[attr-defined]
            return True
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
            return True
        opener = shutil.which("xdg-open")
        if opener:
            subprocess.Popen([opener, str(path)])
            return True
        return False
    except Exception:  # noqa: BLE001 - opening a viewer is best-effort
        return False


def headless_fix_hints() -> List[str]:
    """Short, ASCII-only, platform-specific tips for getting a real GUI
    backend working (printed under the headless warning).
    実際に動く GUI バックエンドを用意するための、短い ASCII 専用・
    プラットフォーム別のヒント（ヘッドレス警告の下に表示する）。"""
    hints: List[str] = []
    if sys.platform == "win32":
        hints.append("Reinstall Python from python.org with 'tcl/tk and IDLE' checked, then run install.bat again.")
        hints.append("Or install a Qt backend into this environment: pip install PyQt5")
    elif sys.platform == "darwin":
        hints.append("pip install PyQt5   (or: brew install python-tk@3.12)")
    else:
        hints.append("sudo apt install python3-tk   (and make sure DISPLAY is set)")
    hints.append("Details: docs/guides/troubleshooting.md (Plot window)")
    return hints


def report_headless(console: Any, info: BackendInfo, saved_to: Path) -> None:
    """Print the standard "no GUI backend, saved a PNG instead" message
    via the sf CLI console (lib/sfcli/utils/console.py's Console, but any
    object with warning/info/print methods works).
    「GUIバックエンドが無いのでPNGを保存した」という定型メッセージを sf CLI
    のコンソール経由で出力する（lib/sfcli/utils/console.py の Console を
    想定するが、warning/info/print メソッドを持つオブジェクトなら何でもよい）。
    """
    console.warning("No GUI backend for matplotlib is usable in this Python.")
    if info.reason:
        console.print(f"  ({info.reason})")
    console.info(f"Saving the plot to {saved_to} and opening it with the default image viewer instead.")
    for hint in headless_fix_hints():
        console.print(f"  {hint}")
