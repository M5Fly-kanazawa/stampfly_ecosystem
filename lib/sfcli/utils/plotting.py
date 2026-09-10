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

import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional

# Longest single-line probe-failure reason kept in BackendInfo.reason
# before truncation (keeps console output to one readable line per probe).
# BackendInfo.reason に残す1個のプローブ失敗理由の最大文字数（切り詰め前）。
# コンソール出力をプローブ1件あたり1行の読みやすい長さに保つ。
REASON_MAX_CHARS = 120

# Qt binding installed as the automatic "no GUI backend" fix (see
# ensure_gui_backend() below). PyQt6, not PyQt5 or PySide6, because:
# (1) it ships prebuilt wheels for every platform/Python combination sf
# supports (win/mac/linux x cp310-cp312, thanks to the abi3 stable ABI),
# so `pip install` never falls back to a slow/failing source build;
# (2) it is a modest, one-time ~80 MB download (PyQt6 6.11: 6.5 MB + PyQt6-Qt6 74 MB on Windows, measured on PyPI 2026-09-11); (3) it is the binding
# matplotlib.backends.qt_compat probes for FIRST, so once it is present
# matplotlib's own "qtagg" backend just works with no further
# configuration; and (4) it is actively maintained (PyQt5 is in
# maintenance-only mode).
# 「GUIバックエンドが無い」場合に自動導入するQtバインディング（下記
# ensure_gui_backend() 参照）。PyQt5でもPySide6でもなくPyQt6を選ぶ理由:
# (1) sfが対応する全プラットフォーム/Python組み合わせ(win/mac/linux ×
# cp310-cp312、abi3安定ABIのおかげ)向けにビルド済みwheelが配布されており
# `pip install`が遅い/失敗するソースビルドに落ちることがない、(2) 一度きり
# の約80MB（PyQt6 6.11: 6.5MB + PyQt6-Qt6 74MB、Windows。2026-09-11 PyPI 実測）というほどよいサイズ、(3) matplotlib.backends.qt_compatが最初に
# 探すバインディングであり、導入さえすればmatplotlibの"qtagg"バックエンドが
# 追加設定なしでそのまま動く、(4) 現役でメンテナンスされている(PyQt5は
# メンテナンスのみのモード)。
QT_FALLBACK_PACKAGE = "PyQt6"
QT_FALLBACK_REQUIREMENT = "PyQt6>=6.5,<7"  # matplotlib >= 3.7 supports PyQt6; abi3 wheels cover py3.10-3.12 on win/mac/linux
QT_PROBE_TIMEOUT_S = 60


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


# Code run inside the throwaway probe subprocess spawned by _probe_qtagg().
# Mirrors what matplotlib's own qtagg backend does on first use (import the
# bindings, then construct the QApplication that owns the event loop).
# _probe_qtagg() が起動する使い捨てプローブ subprocess の中で実行するコード。
# matplotlib 自身の qtagg バックエンドが初回使用時に行うこと（バインディングの
# import、そしてイベントループを持つ QApplication の生成）をそのまま模す。
_QT_PROBE_CODE = (
    "from matplotlib.backends.qt_compat import QtWidgets\n"
    "app = QtWidgets.QApplication([])\n"
    "app.quit()\n"
)


def _probe_qtagg() -> None:
    """Probe whether a Qt GUI backend actually works, OUT OF PROCESS.

    Qt aborts the entire process (an OS-level abort(), not a catchable
    Python exception) when its platform plugin cannot load -- e.g. Linux
    without libxcb-cursor0, or a broken Qt install. Probing in-process
    (just importing qt_compat, as this function used to do) misses that
    failure mode entirely, and actually building a QApplication in-process
    to catch it would crash sf log viz itself instead of falling back to a
    saved PNG. Running the same "import + build a QApplication" check in a
    throwaway subprocess turns that would-be process abort into an
    ordinary, catchable non-zero exit code this function can raise as a
    RuntimeError.
    Qt バックエンドが実際に動くかを、別プロセスでプローブする。
    Qt はプラットフォームプラグインを読み込めない場合（例: libxcb-cursor0が
    無い Linux、壊れた Qt 導入）、プロセス全体を（Python の例外としては
    捕まえられない OS レベルの abort() で）異常終了させる。in-process で
    qt_compat を import するだけの旧プローブではこの故障を検出できず、
    かといって in-process で QApplication まで作って検出しようとすると
    sf log viz 自体がクラッシュし、PNG保存へのフォールバックにすら
    到達できない。全く同じ「import して QApplication を作る」チェックを
    使い捨てのサブプロセスで実行することで、この起こりうるプロセス異常
    終了を、この関数が RuntimeError として送出できる通常の非ゼロ終了
    コードに変換する。
    """
    result = subprocess.run(
        [sys.executable, "-c", _QT_PROBE_CODE],
        capture_output=True,
        text=True,
        timeout=QT_PROBE_TIMEOUT_S,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    if result.returncode == 0:
        return
    stderr_lines = [line for line in result.stderr.splitlines() if line.strip()]
    reason = stderr_lines[-1] if stderr_lines else f"exit code {result.returncode}"
    raise RuntimeError(reason)


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


def has_display() -> bool:
    """True if this platform/session can plausibly show a GUI window.

    Windows and macOS always can -- neither has a "headless display
    server" concept a desktop process needs to check for. Everywhere else
    (Linux, BSD, ...) a GUI needs an X11 or Wayland display server, which
    is only present when DISPLAY or WAYLAND_DISPLAY is set (absent on a
    plain SSH session or a CI runner). Used to decide whether probing/
    installing a GUI backend is worth attempting at all: select_backend()
    skips probing GUI candidates entirely when this is False, and
    ensure_gui_backend() will not install PyQt6 on a headless box.
    このプラットフォーム/セッションでGUIウィンドウを表示できる見込みが
    あるかを返す。Windows/macOSには「ヘッドレスなディスプレイサーバ」という
    概念自体が無く常にTrue。それ以外（Linux、BSD等）ではGUIにX11か
    Waylandのディスプレイサーバが必要で、これはDISPLAYかWAYLAND_DISPLAYが
    設定されている場合のみ存在する（素のSSHセッションやCIランナーには無い）。
    GUIバックエンドのプローブ/導入をそもそも試す価値があるかの判断に使う:
    select_backend()はこれがFalseならGUI候補のプローブ自体を省略し、
    ensure_gui_backend()はヘッドレスな環境にPyQt6を導入しない。
    """
    if sys.platform in ("win32", "darwin"):
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


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

    if not has_display():
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


def ensure_gui_backend(pip_install: Callable[[List[str]], bool], log: Any) -> BackendInfo:
    """Make sure a GUI backend works; if not, install the Qt fallback into
    THIS Python and re-probe. This is the install-time / upgrade-time ROOT
    fix -- select_backend()'s PNG fallback (see the module docstring) is
    the run-time safety net for whenever this was skipped, declined, or
    still failed.

    log: object with info/success/warning/print methods (sf console, or
    the installer's shim).

    Rules: (1) MPLBACKEND set -> respect, do nothing. (2) No display
    (has_display() False) -> do nothing (installing Qt on a headless box
    is pointless; also keeps CI quiet). (3) select_backend() interactive
    -> report and return. (4) else log the reason, log "Installing PyQt6
    (about 80 MB) so plot windows work...", call
    pip_install([QT_FALLBACK_REQUIREMENT]); if it returns False -> warning
    + hints, return the headless info. (5) re-probe with select_backend();
    success -> log "GUI backend: qtagg"; still headless -> warning with
    reason + headless_fix_hints(). Never raises (wraps unexpected
    exceptions into a warning and returns a headless BackendInfo).

    確実にGUIバックエンドが動くようにする。動かなければQtのフォールバックを
    「この」Pythonに導入し、再プローブする。これがインストール時/アップ
    グレード時の根本対処であり、select_backend()のPNGフォールバック
    （モジュールdocstring参照）はこれが省略・辞退・失敗した場合の
    実行時の安全網に過ぎない。

    log: info/success/warning/print メソッドを持つオブジェクト（sfの
    コンソール、またはインストーラのシム）。

    規則: (1) MPLBACKENDが設定済みなら尊重して何もしない。(2) ディスプレイ
    が無い（has_display()がFalse）なら何もしない（ヘッドレスな環境への
    Qt導入は無意味であり、CIも静かに保てる）。(3) select_backend()が
    インタラクティブなら報告して戻る。(4) それ以外は理由をログし、
    "Installing PyQt6 (about 80 MB) so plot windows work..." をログし、
    pip_install([QT_FALLBACK_REQUIREMENT])を呼ぶ。Falseが返れば警告+ヒント
    を出しヘッドレスな情報を返す。(5) select_backend()で再プローブし、
    成功すれば"GUI backend: qtagg"をログ、依然ヘッドレスなら理由+
    headless_fix_hints()付きで警告する。例外は決して送出しない
    （予期しない例外は警告に変換しヘッドレスなBackendInfoを返す）。
    """
    try:
        if os.environ.get("MPLBACKEND"):
            return select_backend(want_window=True)

        if not has_display():
            return select_backend(want_window=True)

        info = select_backend(want_window=True)
        if info.interactive:
            log.success(f"GUI backend: {info.name}")
            return info

        log.info(f"No GUI backend for matplotlib is usable in this Python ({info.reason})")
        log.info(f"Installing {QT_FALLBACK_PACKAGE} (about 80 MB) so plot windows work...")
        if not pip_install([QT_FALLBACK_REQUIREMENT]):
            log.warning(f"{QT_FALLBACK_PACKAGE} install failed; plots will still be saved as PNG files.")
            for hint in headless_fix_hints():
                log.print(f"  {hint}")
            return info

        info = select_backend(want_window=True)
        if info.interactive:
            log.success(f"GUI backend: {info.name}")
        else:
            log.warning(f"Still no usable GUI backend after installing {QT_FALLBACK_PACKAGE} ({info.reason})")
            for hint in headless_fix_hints():
                log.print(f"  {hint}")
        return info
    except Exception as exc:  # noqa: BLE001 - this helper must never crash its caller
        log.warning(f"Could not set up a GUI backend for matplotlib: {exc}")
        return BackendInfo("agg", False, str(exc))


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

    The first hint is always `sf doctor --fix`, which runs
    ensure_gui_backend() above and does everything below automatically --
    the remaining hints are the manual fallback for when that is not
    available or does not help.
    実際に動く GUI バックエンドを用意するための、短い ASCII 専用・
    プラットフォーム別のヒント（ヘッドレス警告の下に表示する）。

    最初のヒントは常に `sf doctor --fix` -- 上の ensure_gui_backend() を
    実行し、以下を自動で行う。残りのヒントは、それが使えない/効かない
    場合の手動フォールバック。"""
    hints: List[str] = [
        f"Run: sf doctor --fix   (installs {QT_FALLBACK_PACKAGE} into the sf Python environment)",
    ]
    if sys.platform == "win32":
        hints.append(
            "Reinstall Python from python.org with 'tcl/tk and IDLE' checked, "
            f'then run install.bat again. Or: pip install "{QT_FALLBACK_REQUIREMENT}"'
        )
    elif sys.platform == "darwin":
        hints.append(f'pip install "{QT_FALLBACK_REQUIREMENT}"   (or: brew install python-tk@3.12)')
    else:
        hints.append("sudo apt install python3-tk   (or libxcb-cursor0 if PyQt6 is installed but fails to start)")
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


def _main(argv: List[str]) -> int:
    """CLI entry point: `python -m sfcli.utils.plotting --probe`.

    Lets a caller running under a DIFFERENT Python get this module's
    backend-probe result without importing it directly -- the one caller
    that matters is scripts/installer.py, which must probe the ESP-IDF
    venv's python (the one `sf` actually runs under), not its own
    (system) interpreter. Prints one JSON object to stdout and always
    exits 0 once the probe actually ran, because a headless result is a
    valid, successful answer; exit 1 is reserved for "matplotlib is not
    even installed", a different failure the caller must handle
    differently (there is no backend to report on).
    別のPythonから、このモジュールを直接importせずにバックエンドプローブ
    結果を取得できるようにするCLIエントリポイント -- 実際に使うのは
    scripts/installer.py で、`sf` が実際に動くESP-IDF venvのpythonを
    プローブする必要があり、自分自身の(システム)インタプリタでは
    ないため。JSONオブジェクトを1つ標準出力に印字し、プローブが実際に
    走った時点で常にexit 0とする（ヘッドレスという結果自体は正常な
    回答）-- exit 1は「matplotlibがそもそも未インストール」という
    別種の失敗のみで、これは呼び出し側が別扱いする必要がある
    （報告すべきバックエンドが無い）。
    """
    if "--probe" not in argv:
        print(json.dumps({"error": "usage: python -m sfcli.utils.plotting --probe"}))
        return 1

    try:
        info = select_backend(want_window=True)
    except ImportError as exc:
        print(json.dumps({"error": f"matplotlib not installed: {exc}"}))
        return 1

    print(json.dumps({
        "backend": info.name,
        "interactive": info.interactive,
        "reason": info.reason,
        "has_display": has_display(),
        "fallback_requirement": QT_FALLBACK_REQUIREMENT,
    }))
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
