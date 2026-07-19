"""
Windows backend for the StampFly Flasher native installer
StampFly Flasher ネイティブインストーラーの Windows バックエンド

Implements the OS backend contract consumed by
lib/sfcli/utils/flasher_install/__init__.py (see FLASHER_SPEC.md section 2):

    install(artifact: Path, version: str, opts: dict) -> list[str]
    uninstall(manifest: dict | None) -> None
    installed_app_executable() -> Path | None
    manifest_path() -> Path

Layout on disk / 配置:
    %LOCALAPPDATA%\\Programs\\StampFly\\StampFlyFlasher.exe   (the app)
    %LOCALAPPDATA%\\Programs\\StampFly\\uninstall.cmd          (standalone uninstaller)
    %APPDATA%\\Microsoft\\Windows\\Start Menu\\Programs\\StampFly\\StampFly Flasher.lnk
    Desktop\\StampFly Flasher.lnk                              (optional)
    HKCU\\...\\Uninstall\\StampFlyFlasher                       (Apps & features entry)

`winreg` is Windows-only and is imported lazily, inside the functions that
need it, rather than at module scope. That keeps `import _windows` safe on
macOS/Linux (e.g. CI matrix jobs, or a developer running the test suite on a
non-Windows machine) — every other symbol in this module (os.environ
lookups, pathlib, subprocess) is cross-platform and only *fails at call
time* on the wrong OS, which is the intended behavior.
winreg は Windows 専用のため、モジュールスコープではなく必要な関数の内部で
のみ import する。これにより mac/Linux 上でも `import _windows` 自体は安全に
通る（CIマトリクスや非Windows開発機でのテスト容易性のため）。それ以外の
シンボル（os.environ 参照・pathlib・subprocess）はクロスプラットフォームで、
誤ったOSで呼び出された場合のみ実行時に失敗する（意図した挙動）。
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from . import FlasherInstallError
from .. import console

# Fixed names shared by install()/uninstall() so the two never drift apart.
# install()/uninstall() 間で名称がずれないよう固定値として共有する
EXE_NAME = "StampFlyFlasher.exe"
SHORTCUT_DISPLAY_NAME = "StampFly Flasher"
START_MENU_FOLDER_NAME = "StampFly"
UNINSTALL_SCRIPT_NAME = "uninstall.cmd"
MANIFEST_FILE_NAME = "stampfly_flasher_manifest.json"
REGISTRY_KEY_PATH = r"Software\Microsoft\Windows\CurrentVersion\Uninstall\StampFlyFlasher"
PUBLISHER_NAME = "StampFly Ecosystem (M5Fly-kanazawa)"

# Delay (seconds) uninstall.cmd waits before removing its own directory, so
# the parent `sf flasher uninstall` (or double-clicking the .cmd) has time
# to exit and release its file handles on the script itself.
# uninstall.cmd が自分自身のディレクトリを削除するまでの待機秒数。呼び出し元
# （`sf flasher uninstall` やダブルクリックしたシェル）がスクリプト自身への
# ファイルハンドルを解放できるよう猶予を与える
SELF_DELETE_DELAY_SECONDS = 2


def _install_dir() -> Path:
    """%LOCALAPPDATA%\\Programs\\StampFly (spec's fixed install location).
    %LOCALAPPDATA%\\Programs\\StampFly（仕様で固定された配置先）"""
    local_appdata = os.environ.get("LOCALAPPDATA")
    if not local_appdata:
        raise FlasherInstallError(
            "LOCALAPPDATA environment variable is not set (expected on Windows). "
            "Manual install: copy the .exe to %LOCALAPPDATA%\\Programs\\StampFly\\"
        )
    return Path(local_appdata) / "Programs" / "StampFly"


def _start_menu_dir() -> Path:
    """Per-app Start Menu folder.
    アプリ専用のスタートメニューフォルダ"""
    appdata = os.environ.get("APPDATA")
    if not appdata:
        raise FlasherInstallError(
            "APPDATA environment variable is not set (expected on Windows)."
        )
    return Path(appdata) / "Microsoft" / "Windows" / "Start Menu" / "Programs" / START_MENU_FOLDER_NAME


def _desktop_dir() -> Path:
    """User's Desktop folder.
    ユーザーのデスクトップフォルダ

    NOTE: uses %USERPROFILE%\\Desktop, matching the other paths in this
    module which are resolved from plain environment variables (spec §4-1
    is written in terms of %LOCALAPPDATA%/%APPDATA%). This does not follow
    an OneDrive-redirected Desktop; that edge case is out of scope here and
    left as a known limitation for a future revision.
    注意: 本モジュールの他パスと同様、単純な環境変数
    (%USERPROFILE%\\Desktop) で解決する（仕様4-1が%LOCALAPPDATA%/%APPDATA%
    という素の環境変数で書かれているため）。OneDriveでリダイレクトされた
    デスクトップには対応しない。既知の制約として将来の改訂に委ねる。
    """
    user_profile = os.environ.get("USERPROFILE")
    if not user_profile:
        raise FlasherInstallError(
            "USERPROFILE environment variable is not set (expected on Windows)."
        )
    return Path(user_profile) / "Desktop"


def _ps_escape(value: str) -> str:
    """Escape a string for embedding inside a PowerShell single-quoted literal.
    PowerShell のシングルクォート文字列に埋め込むための文字列エスケープ

    PowerShell escapes an embedded single quote by doubling it (`''`), the
    same rule as classic Pascal/SQL string literals.
    PowerShell はシングルクォートの二重化 (`''`) でエスケープする
    （Pascal/SQL の文字列リテラルと同じ規則）。
    """
    return value.replace("'", "''")


def _create_shortcut(link_path: Path, target_exe: Path) -> None:
    """Create a Windows .lnk shortcut via PowerShell's WScript.Shell COM object.
    PowerShell の WScript.Shell COM オブジェクト経由で .lnk ショートカットを作成する

    No extra Python dependency (e.g. pywin32) is required: PowerShell +
    WScript.Shell is the standard scripting approach for .lnk creation and
    ships with every supported Windows version.
    pywin32 等の追加 Python 依存は不要。PowerShell + WScript.Shell は .lnk
    作成の標準的な手法で、サポート対象の全 Windows バージョンに標準搭載。
    """
    link_path.parent.mkdir(parents=True, exist_ok=True)

    ps_script = (
        "$ws = New-Object -ComObject WScript.Shell\n"
        f"$sc = $ws.CreateShortcut('{_ps_escape(str(link_path))}')\n"
        f"$sc.TargetPath = '{_ps_escape(str(target_exe))}'\n"
        f"$sc.WorkingDirectory = '{_ps_escape(str(target_exe.parent))}'\n"
        f"$sc.IconLocation = '{_ps_escape(str(target_exe))},0'\n"
        "$sc.Save()\n"
    )

    # Written as a temp .ps1 (rather than passed via -Command) so quoting
    # is handled once here instead of twice (Python -> shell -> PowerShell).
    # -Command 経由だと引用符がPython→シェル→PowerShellと二重に解釈される
    # ため、一時 .ps1 ファイルに書き出して -File で実行する
    fd, tmp_name = tempfile.mkstemp(suffix=".ps1", prefix="stampfly_shortcut_")
    ps1_path = Path(tmp_name)
    try:
        # utf-8-sig (BOM), NOT ascii: the embedded paths contain the user's
        # profile directory, which is frequently non-ASCII on Japanese
        # Windows (e.g. C:\Users\山田). Windows PowerShell 5.1 decodes a
        # BOM-less .ps1 in the system ANSI code page, but reads BOM'd UTF-8
        # correctly — so utf-8-sig is the one encoding that round-trips any
        # username on any locale.
        # ascii ではなく utf-8-sig（BOM付き）で書く: 埋め込むパスには
        # ユーザープロファイルが含まれ、日本語版 Windows では非ASCII
        # （例: C:\Users\山田）が普通にあり得る。Windows PowerShell 5.1 は
        # BOM無し .ps1 をシステム ANSI コードページで解釈するが、BOM付き
        # UTF-8 は正しく読める — どのロケール・どのユーザー名でも確実に
        # 往復できるのは utf-8-sig だけ。
        with os.fdopen(fd, "w", encoding="utf-8-sig") as f:
            f.write(ps_script)
        result = subprocess.run(
            [
                "powershell", "-NoProfile", "-NonInteractive",
                "-ExecutionPolicy", "Bypass", "-File", str(ps1_path),
            ],
            capture_output=True, text=True,
            encoding="utf-8",
            errors="replace",
        )
        if result.returncode != 0:
            raise FlasherInstallError(
                f"Failed to create shortcut {link_path}: {result.stderr.strip()}"
            )
    finally:
        ps1_path.unlink(missing_ok=True)


def _dir_size_kb(path: Path) -> int:
    """Total size (KB) of all files under path, for the registry's
    EstimatedSize value.
    path 配下の全ファイル合計サイズ(KB)。レジストリの EstimatedSize 用"""
    total_bytes = sum(p.stat().st_size for p in path.rglob("*") if p.is_file())
    return total_bytes // 1024


def _write_uninstall_registry(install_dir: Path, exe_path: Path, version: str) -> None:
    """Register under HKCU Uninstall so the app shows up in Windows
    'Apps & features' with a working Uninstall button.
    HKCU の Uninstall キーへ登録し、Windows の「アプリと機能」に表示させ、
    アンインストールボタンを機能させる

    HKCU (not HKLM) is used deliberately: it needs no administrator
    privileges, which matters on school-managed PCs where students/teachers
    often lack admin rights (see project memory: DXH workshop).
    管理者権限が不要な HKCU をあえて使う（学校PC等、生徒・教員が管理者権限を
    持たない環境での利用を想定。プロジェクトの記憶: DXH講座）。
    """
    import winreg  # noqa: local import, see module docstring / モジュールdocstring参照

    with winreg.CreateKey(winreg.HKEY_CURRENT_USER, REGISTRY_KEY_PATH) as key:
        winreg.SetValueEx(key, "DisplayName", 0, winreg.REG_SZ, SHORTCUT_DISPLAY_NAME)
        winreg.SetValueEx(key, "DisplayVersion", 0, winreg.REG_SZ, version)
        winreg.SetValueEx(key, "Publisher", 0, winreg.REG_SZ, PUBLISHER_NAME)
        winreg.SetValueEx(key, "InstallLocation", 0, winreg.REG_SZ, str(install_dir))
        winreg.SetValueEx(key, "DisplayIcon", 0, winreg.REG_SZ, str(exe_path))
        winreg.SetValueEx(
            key, "UninstallString", 0, winreg.REG_SZ,
            f'"{install_dir / UNINSTALL_SCRIPT_NAME}"',
        )
        winreg.SetValueEx(key, "NoModify", 0, winreg.REG_DWORD, 1)
        winreg.SetValueEx(key, "NoRepair", 0, winreg.REG_DWORD, 1)
        winreg.SetValueEx(key, "EstimatedSize", 0, winreg.REG_DWORD, _dir_size_kb(install_dir))


def _remove_registry_entry() -> None:
    """Delete the HKCU Uninstall key created by _write_uninstall_registry().
    _write_uninstall_registry() が作成した HKCU Uninstall キーを削除する"""
    import winreg  # noqa: local import, see module docstring / モジュールdocstring参照

    try:
        winreg.DeleteKey(winreg.HKEY_CURRENT_USER, REGISTRY_KEY_PATH)
    except FileNotFoundError:
        pass  # Already absent: uninstall is idempotent / 既に無ければ何もしない


def _write_uninstall_cmd(install_dir: Path) -> Path:
    """Generate uninstall.cmd inside the install directory.

    This is the standalone uninstaller: it is what Windows 'Apps & features'
    runs (via the UninstallString registered in the registry), and it also
    works for a user who no longer has `sf` available. It removes both
    shortcuts, the registry entry, and finally deletes its own install
    directory (including itself) using the well-known delayed self-delete
    trick: spawn a detached `cmd /c` that waits a couple of seconds (so this
    process can exit and release its file handle) before running `rmdir /s`.

    インストールディレクトリ内に uninstall.cmd を生成する。これは単独で動く
    アンインストーラで、Windows の「アプリと機能」から
    （レジストリに登録した UninstallString 経由で）実行されるほか、`sf` が
    使えないユーザーでも動作する。両方のショートカット・レジストリエントリを
    削除し、最後に定石の遅延自己削除トリックで自分自身を含むインストール
    ディレクトリを削除する: 分離した `cmd /c` を起動し数秒待って
    （このプロセスが終了しファイルハンドルを解放できるように）から
    `rmdir /s` を実行する。

    ASCII only, no bilingual (Japanese) comments inside the generated file:
    a prior CI incident showed Windows batch files containing non-ASCII text
    can hit cp1252 decode errors depending on the invoking shell's active
    code page (project memory: v2026.07.1 Windows CI hang, "GITHUB CI固有"
    print/UnicodeEncodeError root cause). This is a deliberate,
    spec-mandated (§4-1) exception to the repo's usual bilingual-comment
    convention — it applies only to the *contents of this generated file*,
    not to this Python module.
    生成ファイル内は ASCII のみで日本語コメントを書かない: 過去の CI 障害
    （プロジェクトの記憶: v2026.07.1 Windows CI ハング、print文の
    UnicodeEncodeErrorが根本原因）で、Windows バッチファイル内の非ASCII
    文字が呼び出し元シェルのアクティブコードページ次第でcp1252デコード
    エラーを起こすことが判明したため。仕様4-1で明示された意図的な例外で
    あり、この生成ファイルの中身にのみ適用され、本 Python モジュール自体
    には適用されない。

    Windows verification of this script is deferred: it is exercised in CI
    (release.yml install smoke test, this session) and on a real Windows
    machine in a later session — the author has no Windows hardware.
    このスクリプトの Windows 実機検証は先送りする: CI（release.yml の
    インストールスモークテスト、本セッション）と、後日の実 Windows 環境
    セッションで検証する（担当者は Windows 実機を保有しない）。
    """
    script_path = install_dir / UNINSTALL_SCRIPT_NAME

    # Shortcut paths are written as %APPDATA%/%USERPROFILE% environment-
    # variable references (expanded by cmd.exe at RUN time), never as
    # absolute paths baked in at install time. Two reasons: (1) the file
    # stays pure ASCII even when the user's profile directory is non-ASCII
    # (e.g. C:\Users\山田 on Japanese Windows) — batch files in a non-matching
    # code page mis-parse non-ASCII bytes, and .encode("ascii") below would
    # simply crash at install time; (2) the script keeps working even if the
    # profile is relocated between install and uninstall.
    # ショートカットのパスは、インストール時の絶対パスではなく
    # %APPDATA%/%USERPROFILE% の環境変数参照（cmd.exe が実行時に展開）で
    # 書く。理由は2つ: (1) 日本語版 Windows のように ユーザープロファイルが
    # 非ASCII（例: C:\Users\山田）でもファイルが純ASCIIのままになる —
    # コードページ不一致のバッチは非ASCIIバイトを誤解釈するし、下の
    # .encode("ascii") がインストール時に落ちてしまう。(2) インストールと
    # アンインストールの間にプロファイルが移動されても動き続ける。
    start_menu_dir_cmd = (
        f"%APPDATA%\\Microsoft\\Windows\\Start Menu\\Programs\\{START_MENU_FOLDER_NAME}"
    )
    start_menu_link_cmd = f"{start_menu_dir_cmd}\\{SHORTCUT_DISPLAY_NAME}.lnk"
    desktop_link_cmd = f"%USERPROFILE%\\Desktop\\{SHORTCUT_DISPLAY_NAME}.lnk"

    lines = [
        "@echo off",
        "rem Uninstall script for StampFly Flasher",
        "rem Generated by: sf flasher install / sf flasher uninstall (do not edit by hand)",
        "setlocal",
        'set "INSTALL_DIR=%~dp0"',
        'set "INSTALL_DIR=%INSTALL_DIR:~0,-1%"',
        "",
        "rem Remove shortcuts",
        f'del /f /q "{start_menu_link_cmd}" >nul 2>nul',
        f'rmdir "{start_menu_dir_cmd}" >nul 2>nul',
        f'del /f /q "{desktop_link_cmd}" >nul 2>nul',
        "",
        "rem Remove the Apps and features entry",
        f'reg delete "HKCU\\{REGISTRY_KEY_PATH}" /f >nul 2>nul',
        "",
        "rem Delayed self-delete of the install directory (including this file)",
        f'start "" /min cmd /c "timeout /t {SELF_DELETE_DELAY_SECONDS} /nobreak >nul & rmdir /s /q "%INSTALL_DIR%""',
        "exit /b 0",
        "",
    ]
    content = "\r\n".join(lines)
    script_path.write_bytes(content.encode("ascii"))
    return script_path


def install(artifact: Path, version: str, opts: dict) -> list[str]:
    """Install the given artifact (a StampFlyFlasher.exe) for Windows.
    与えられた成果物（StampFlyFlasher.exe）を Windows 向けにインストールする

    `artifact` must be a bare .exe file: it is the only asset shape the
    Windows release job produces (release.yml packages
    `StampFlyFlasher_<tag>_windows-x64.exe` directly, no zip/directory).
    `artifact` は素の .exe ファイルでなければならない。Windowsリリース
    ジョブが生成する唯一の形式のため（release.yml は
    `StampFlyFlasher_<tag>_windows-x64.exe` を直接パッケージし、zip や
    ディレクトリにはしない）。
    """
    if artifact.is_dir() or artifact.suffix.lower() != ".exe":
        raise FlasherInstallError(
            f"Windows installer expects a .exe file, got: {artifact}\n"
            "Manual install: copy the .exe to "
            "%LOCALAPPDATA%\\Programs\\StampFly\\StampFlyFlasher.exe"
        )

    install_dir = _install_dir()
    install_dir.mkdir(parents=True, exist_ok=True)
    exe_path = install_dir / EXE_NAME
    shutil.copy2(artifact, exe_path)

    created: list[str] = [str(exe_path)]

    # Start Menu shortcut is always created / スタートメニューショートカットは常に作成
    start_menu_link = _start_menu_dir() / f"{SHORTCUT_DISPLAY_NAME}.lnk"
    _create_shortcut(start_menu_link, exe_path)
    created.append(str(start_menu_link))

    # Desktop shortcut is opt-out (default True) / デスクトップショートカットはopt-out（既定True）
    if opts.get("desktop_shortcut", True):
        desktop_link = _desktop_dir() / f"{SHORTCUT_DISPLAY_NAME}.lnk"
        _create_shortcut(desktop_link, exe_path)
        created.append(str(desktop_link))

    uninstall_cmd_path = _write_uninstall_cmd(install_dir)
    created.append(str(uninstall_cmd_path))

    # Registry entry is not a "created file" (no filesystem path), so it is
    # intentionally left out of the returned list; uninstall() removes it
    # unconditionally (see below), independent of the manifest.
    # レジストリエントリはファイルパスを持たないため、戻り値の一覧には
    # 意図的に含めない。uninstall() は manifest の有無に関わらず常にこれを
    # 削除する（下記参照）
    _write_uninstall_registry(install_dir, exe_path, version)

    console.info(f"Installed to {exe_path}")
    return created


def _remove_path(path: Path) -> None:
    """Best-effort removal of a file or empty directory; missing paths are
    not an error (uninstall must be idempotent).
    ファイルまたは空ディレクトリのベストエフォート削除。存在しなくてもエラー
    にしない（アンインストールは冪等でなければならない）"""
    try:
        if path.is_dir():
            path.rmdir()
        elif path.exists() or path.is_symlink():
            path.unlink()
    except OSError:
        pass


def uninstall(manifest: dict | None) -> None:
    """Remove the Windows install: exe + shortcuts (via manifest, falling
    back to well-known paths), the registry entry, and the install
    directory.
    Windows のインストールを削除する: exe・ショートカット
    （manifestを優先、無ければ既知パスへフォールバック）・レジストリ
    エントリ・インストールディレクトリ

    Called by `sf flasher uninstall`, a normal Python process running
    outside the install directory — unlike uninstall.cmd, no self-delete
    trick is needed here; shutil.rmtree() can remove the directory directly.
    `sf flasher uninstall` から呼ばれる通常の Python プロセスはインストール
    ディレクトリの外で動くため、uninstall.cmd と異なり自己削除トリックは
    不要で、shutil.rmtree() で直接ディレクトリを削除できる。
    """
    files = manifest.get("files") if manifest else None
    if files:
        for f in files:
            _remove_path(Path(f))
    else:
        # No manifest (lost/corrupted): fall back to the well-known default
        # paths so uninstall still works.
        # manifest なし（消失・破損）: 既知の既定パスへフォールバックし、
        # それでもアンインストールできるようにする
        install_dir = _install_dir()
        _remove_path(install_dir / EXE_NAME)
        _remove_path(install_dir / UNINSTALL_SCRIPT_NAME)
        _remove_path(_start_menu_dir() / f"{SHORTCUT_DISPLAY_NAME}.lnk")
        _remove_path(_desktop_dir() / f"{SHORTCUT_DISPLAY_NAME}.lnk")

    # Registry entry is not manifest-tracked (no filesystem path); always
    # attempt removal, same as the fallback file paths above.
    # レジストリエントリは manifest 管理外（ファイルパスを持たない）ため、
    # 常に削除を試みる（上のフォールバックのファイルパスと同様の扱い）
    _remove_registry_entry()

    start_menu_dir = _start_menu_dir()
    _remove_path(start_menu_dir)  # only removes if now empty / 空の場合のみ削除される

    install_dir = _install_dir()
    if install_dir.exists():
        shutil.rmtree(install_dir, ignore_errors=True)

    console.info("StampFly Flasher uninstalled")


def installed_app_executable() -> Path | None:
    """Return the installed .exe path, or None if not installed.
    インストール済み .exe のパスを返す。未導入なら None"""
    exe_path = _install_dir() / EXE_NAME
    return exe_path if exe_path.is_file() else None


def manifest_path() -> Path:
    """Path to the install manifest JSON, inside the install directory.
    インストールマニフェストJSONのパス（インストールディレクトリ内）"""
    return _install_dir() / MANIFEST_FILE_NAME
