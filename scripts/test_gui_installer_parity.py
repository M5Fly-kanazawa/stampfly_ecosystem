r"""
Drift-detection test: scripts/installer.py vs
tools/installer_gui/stampfly_installer.py's Python auto-install logic.

これら2ファイルはやむを得ず重複している / These two files necessarily duplicate
some logic
------------------------------------------------------------------------------
tools/installer_gui/stampfly_installer.py's Python detection/auto-install code
(_detect_linux_package_manager, _linux_python_install_command,
auto_install_python, PYTHON_PREFERRED_MIN/MAX, ...) runs on its "Environment
Check" screen BEFORE the ecosystem repository has been cloned -- so it cannot
`import scripts.installer` (that file does not exist on disk yet at that
point). Both files' docstrings already cross-reference each other ("Mirrors
scripts/installer.py's X()") as a manual keep-in-sync convention.

This test does not eliminate that duplication (see
C:\Users\kouhe\.claude\plans\rosy-enchanting-scott.md's Phase C -- import-based
de-duplication is not possible here); it is only a safety net so future drift
between the two copies fails CI instead of silently shipping (exactly what
happened with the MSYS2/winget duplication this same audit found and fixed in
lib/sfcli/utils/msys2_install.py vs scripts/installer.py, commits 57a85e5a /
3c4b85f2 -- this test covers the analogous Python-install duplication that
audit found could not be removed the same way).

これら2ファイルの Python 検出・自動導入コード
（_detect_linux_package_manager, _linux_python_install_command,
auto_install_python, PYTHON_PREFERRED_MIN/MAX 等）は、GUI 側がリポジトリの
クローン前に動く必要があるため（その時点では scripts/installer.py 自体が
まだディスク上に存在しない）import による一本化ができない。両ファイルの
docstring は既に相互参照コメント（"Mirrors scripts/installer.py's X()"）で
手動同期の規約を示している。

本テストはその重複自体を解消しない（一本化不可能 -- 上記計画のPhase C参照）;
将来のドリフトを黙って見逃さずCIで検知するための安全網に過ぎない
（今回の調査で見つかった MSYS2/winget の重複——lib/sfcli/utils/msys2_install.py
と scripts/installer.py（コミット 57a85e5a / 3c4b85f2）で実際に起きたのと
同種のドリフトを、一本化できない Python 自動導入の重複について検知する）。

Run: pytest scripts/test_gui_installer_parity.py -v
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
INSTALLER_PATH = REPO_ROOT / "scripts" / "installer.py"
GUI_INSTALLER_PATH = REPO_ROOT / "tools" / "installer_gui" / "stampfly_installer.py"


def _load_module(module_name: str, path: Path) -> ModuleType:
    """Load `path` as a standalone module without adding it to sys.modules
    under its real package name (avoids clashing with any other import of
    the same-named file elsewhere in the test session).
    `path` を、実パッケージ名で sys.modules に登録せず単独モジュールとして
    ロードする（テストセッション内の他の同名ファイルimportと衝突しない
    ようにする）。"""
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def installer() -> ModuleType:
    return _load_module("_test_scripts_installer", INSTALLER_PATH)


@pytest.fixture(scope="module")
def gui_installer() -> ModuleType:
    return _load_module("_test_gui_installer", GUI_INSTALLER_PATH)


def test_python_version_band_matches(installer: ModuleType, gui_installer: ModuleType) -> None:
    """The accepted Python version band (inclusive) must be identical --
    a mismatch means one file would accept/reject a system Python the
    other file disagrees on.
    受理するPythonバージョン帯（両端含む）は一致していなければならない --
    不一致は、一方が受理し他方が拒否するシステムPythonが生まれることを
    意味する。"""
    assert installer.PYTHON_PREFERRED_MIN == gui_installer.PYTHON_PREFERRED_MIN
    assert installer.PYTHON_PREFERRED_MAX == gui_installer.PYTHON_PREFERRED_MAX


@pytest.mark.parametrize("manager", ["apt", "dnf", "pacman"])
def test_linux_python_install_command_matches(
    installer: ModuleType, gui_installer: ModuleType, manager: str
) -> None:
    """scripts/installer.py returns argv (List[str], it executes this);
    the GUI returns a joined display string (it only ever shows this for
    the user to copy -- see gui-installer-plan.md's "no sudo from a GUI"
    stance). Compare them as equivalent shell command text.
    scripts/installer.py は argv（List[str]、実際に実行する）を返し、GUIは
    連結済みの表示用文字列を返す（ユーザーがコピーするために表示するのみ --
    gui-installer-plan.md の「GUIからsudoしない」方針参照）。両者を等価な
    シェルコマンド文字列として比較する。"""
    installer_argv = installer._linux_python_install_command(manager)
    gui_command_str = gui_installer._linux_python_install_command(manager)
    assert " ".join(installer_argv) == gui_command_str


def test_windows_winget_python_argv_matches(installer: ModuleType, gui_installer: ModuleType) -> None:
    """Both files independently build the exact same winget argv (only the
    resolved executable path in argv[0] legitimately differs -- both call
    shutil.which("winget") into a local variable of that name). Verified
    via source inspection rather than executing winget.
    両ファイルは独立して同一のwinget argvを組み立てる（argv[0]の解決済み
    実行ファイルパスのみが正当に異なりうる -- どちらも shutil.which("winget")
    を同名のローカル変数に代入している）。winget を実行せず、ソース検査で
    検証する。"""
    import inspect

    installer_source = _normalize(inspect.getsource(installer._auto_install_python_windows))
    gui_source = _normalize(inspect.getsource(gui_installer.auto_install_python))

    expected = _normalize(
        '"install", "--id", "Python.Python.3.12", "--silent", '
        '"--accept-package-agreements", "--accept-source-agreements",'
    )
    assert expected in installer_source, "scripts/installer.py's winget argv changed"
    assert expected in gui_source, "the GUI's winget argv changed"


def test_macos_brew_python_argv_matches(installer: ModuleType, gui_installer: ModuleType) -> None:
    """Both files independently run `brew install python@3.12` (only the
    resolved `brew` executable path differs).
    両ファイルは独立して `brew install python@3.12` を実行する
    （解決済みの brew 実行ファイルパスのみが異なる）。"""
    import inspect

    installer_source = _normalize(inspect.getsource(installer._auto_install_python_macos))
    gui_source = _normalize(inspect.getsource(gui_installer.auto_install_python))

    expected = _normalize('"install", "python@3.12"')
    assert expected in installer_source, "scripts/installer.py's brew argv changed"
    assert expected in gui_source, "the GUI's brew argv changed"


def _normalize(text: str) -> str:
    """Collapse all whitespace runs to a single space, so multi-line
    argv literals formatted differently (line breaks, indentation) still
    compare equal.
    全ての空白の連続を1個の空白に畳み込む -- 複数行にまたがるargvリテラルの
    改行/インデントの違いを比較対象から除く。"""
    return " ".join(text.split())


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
