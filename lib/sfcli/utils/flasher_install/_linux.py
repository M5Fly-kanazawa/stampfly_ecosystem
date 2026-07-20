"""
Linux backend for the StampFly Flasher native installer
StampFly Flasher ネイティブインストーラーの Linux バックエンド

Implements the OS backend contract consumed by
lib/sfcli/utils/flasher_install/__init__.py (see FLASHER_SPEC.md section 2):

    install(artifact: Path, version: str, opts: dict) -> list[str]
    uninstall(manifest: dict | None) -> None
    installed_app_executable() -> Path | None
    manifest_path() -> Path

Layout on disk / 配置 (XDG base-directory conventions, no root required):
    ~/.local/opt/stampfly/StampFlyFlasher                (the app, chmod 0o755)
    ~/.local/share/icons/hicolor/<N>x<N>/apps/stampfly-flasher.png
                                          (N = 16..512, see ICON_SIZES)
    ~/.local/share/applications/stampfly-flasher.desktop
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from . import FlasherInstallError
from .. import console, paths

# Fixed names shared by install()/uninstall() so the two never drift apart.
# install()/uninstall() 間で名称がずれないよう固定値として共有する
BINARY_NAME = "StampFlyFlasher"
ICON_FILE_NAME = "stampfly-flasher.png"
DESKTOP_ENTRY_NAME = "stampfly-flasher.desktop"
MANIFEST_FILE_NAME = "stampfly_flasher_manifest.json"

# hicolor theme sizes installed from the repo's pre-generated icon set
# (tools/flasher_gui/assets/icon_<N>.png). Installing the real sizes lets
# GNOME pick an exact match for its standard renditions (app grid 96 px,
# dash 64 px) instead of downscaling a single 256 px image; 32/24/16 are
# a simplified large-bolt variant generated for small-size legibility.
# hicolorテーマへ設置するサイズ一覧（リポジトリ同梱の生成済み
# tools/flasher_gui/assets/icon_<N>.png から）。実サイズを設置することで
# GNOME が標準表示（アプリ一覧96px・ドック64px）に正確なサイズを選べる
# （256px 1枚の縮小より鮮明）。32/24/16 は小サイズ視認性のために生成した
# 稲妻拡大の簡略版。
ICON_SIZES = (16, 24, 32, 48, 64, 96, 128, 256, 512)

# File permission for the installed binary: owner rwx, group/other rx (no
# write) — a normal executable, not world-writable.
# インストールする実行体の権限: 所有者rwx、グループ/その他rx（書き込み不可）
# ── 通常の実行ファイル権限であり、誰でも書き込み可能にはしない
BINARY_MODE = 0o755


def _install_dir() -> Path:
    """~/.local/opt/stampfly (spec's fixed install location).
    ~/.local/opt/stampfly（仕様で固定された配置先）"""
    return Path.home() / ".local" / "opt" / "stampfly"


def _binary_path() -> Path:
    return _install_dir() / BINARY_NAME


def _icon_dest_path(size: int) -> Path:
    """XDG hicolor icon theme path for a <size>x<size> app icon.
    XDG hicolorアイコンテーマの <size>x<size> アプリアイコンパス"""
    return (Path.home() / ".local" / "share" / "icons" / "hicolor"
            / f"{size}x{size}" / "apps" / ICON_FILE_NAME)


def _desktop_entry_path() -> Path:
    """XDG applications directory .desktop entry path.
    XDG applicationsディレクトリの.desktopエントリパス"""
    return Path.home() / ".local" / "share" / "applications" / DESKTOP_ENTRY_NAME


def _repo_icon_source(size: int) -> Path | None:
    """Locate the repo's pre-generated <size>px icon, if available.
    リポジトリに同梱された生成済み <size>px アイコンがあれば、その場所を返す

    Returns None (rather than raising) when the repo checkout is not
    reachable from here — e.g. an artifact installed via `--from-file` from
    outside the repo, or a packaged sfcli with no adjacent tools/ tree.
    Callers must treat a None return as "skip the icon, keep going": a
    missing icon must never fail the whole install (spec §4-2).
    ここからリポジトリのチェックアウトに到達できない場合（例: リポジトリ外
    から `--from-file` で導入した場合、隣接する tools/ ツリーを持たない
    パッケージ化された sfcli 等）は None を返す（例外は投げない）。
    呼び出し側は None を「アイコンを諦めて続行」として扱うこと。
    アイコン欠落だけでインストール全体を失敗させてはならない（仕様4-2）。
    """
    candidate = paths.tools() / "flasher_gui" / "assets" / f"icon_{size}.png"
    return candidate if candidate.is_file() else None


def _desktop_entry_content(binary_path: Path) -> str:
    """Build the .desktop file content.
    .desktopファイルの内容を組み立てる

    Comment is a single bilingual line (English then Japanese) per the
    project's usual convention; unlike Windows' uninstall.cmd, .desktop
    files are UTF-8 by the freedesktop.org spec, so this is not subject to
    the same cp1252/ASCII constraint that applies to generated .cmd files.
    Comment は本プロジェクトの通常の慣習どおり英語→日本語の1行併記。
    Windows の uninstall.cmd と異なり、.desktop ファイルは
    freedesktop.org 仕様でUTF-8のため、生成 .cmd ファイルに適用される
    cp1252/ASCII制約はここには当てはまらない。
    """
    exec_line = f'"{binary_path}"'
    return (
        "[Desktop Entry]\n"
        "Type=Application\n"
        "Name=StampFly Flasher\n"
        "Comment=Flash StampFly firmware over USB / USB経由でStampFlyファームウェアを書き込む\n"
        f"Exec={exec_line}\n"
        f"Icon={ICON_FILE_NAME.removesuffix('.png')}\n"
        "Terminal=false\n"
        "Categories=Development;Utility;\n"
    )


def _refresh_desktop_database() -> None:
    """Run `update-desktop-database` if available, so the new .desktop entry
    shows up in application launchers without a logout/login. Failure (tool
    absent, or a minimal desktop that lacks it) is ignored — it is a
    convenience refresh, not a correctness requirement.
    `update-desktop-database` が利用可能なら実行し、ログアウト/ログイン
    無しでアプリランチャーに新しい .desktop エントリを反映させる。
    失敗（ツール未導入、あるいは持たない最小構成デスクトップ環境）は無視
    する ── あくまで利便性のための更新であり、必須要件ではない。
    """
    tool = shutil.which("update-desktop-database")
    if not tool:
        return
    try:
        subprocess.run(
            [tool, str(_desktop_entry_path().parent)],
            capture_output=True, check=False,
        )
    except OSError:
        pass


def install(artifact: Path, version: str, opts: dict) -> list[str]:
    """Install the given artifact (a bare StampFlyFlasher binary) for Linux.
    与えられた成果物（素の StampFlyFlasher バイナリ）を Linux 向けにインストールする

    `artifact` must be a bare executable: it is the only asset shape the
    Linux release job produces (release.yml packages
    `StampFlyFlasher_<tag>_linux-x64` directly — a PyInstaller --onefile
    binary, no zip/directory/.exe).
    `artifact` は素の実行ファイルでなければならない。Linuxリリースジョブが
    生成する唯一の形式のため（release.yml は
    `StampFlyFlasher_<tag>_linux-x64` を直接パッケージする ──
    PyInstaller --onefile のバイナリであり、zip/ディレクトリ/.exe ではない）。
    """
    if artifact.is_dir():
        raise FlasherInstallError(
            f"Linux installer expects a bare executable file, got a directory: {artifact}"
        )
    if artifact.suffix.lower() in (".exe", ".zip"):
        raise FlasherInstallError(
            f"Linux installer expects a bare executable file, got: {artifact} "
            "(that looks like a Windows/macOS asset)"
        )

    install_dir = _install_dir()
    install_dir.mkdir(parents=True, exist_ok=True)
    binary_path = _binary_path()
    shutil.copy2(artifact, binary_path)
    binary_path.chmod(BINARY_MODE)

    created: list[str] = [str(binary_path)]

    icons_installed = 0
    for size in ICON_SIZES:
        icon_src = _repo_icon_source(size)
        if icon_src is None:
            continue
        icon_dest = _icon_dest_path(size)
        icon_dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(icon_src, icon_dest)
        created.append(str(icon_dest))
        icons_installed += 1
    if icons_installed == 0:
        console.warning(
            f"Icon assets not found ({paths.tools() / 'flasher_gui' / 'assets' / 'icon_<size>.png'}); "
            "installing without a desktop icon (the app itself is unaffected)."
        )

    desktop_path = _desktop_entry_path()
    desktop_path.parent.mkdir(parents=True, exist_ok=True)
    desktop_path.write_text(_desktop_entry_content(binary_path), encoding="utf-8")
    desktop_path.chmod(0o644)
    created.append(str(desktop_path))

    _refresh_desktop_database()

    console.info(f"Installed to {binary_path}")
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
    """Remove the Linux install: binary, icon, .desktop entry (via manifest,
    falling back to well-known paths), then the install directory if empty.
    Linux のインストールを削除する: バイナリ・アイコン・.desktopエントリ
    （manifestを優先、無ければ既知パスへフォールバック）、その後空であれば
    インストールディレクトリも削除する
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
        _remove_path(_binary_path())
        for size in ICON_SIZES:
            _remove_path(_icon_dest_path(size))
        _remove_path(_desktop_entry_path())

    install_dir = _install_dir()
    _remove_path(install_dir)  # only removes if now empty / 空の場合のみ削除される

    _refresh_desktop_database()

    console.info("StampFly Flasher uninstalled")


def installed_app_executable() -> Path | None:
    """Return the installed binary path, or None if not installed.
    インストール済みバイナリのパスを返す。未導入なら None"""
    binary_path = _binary_path()
    return binary_path if binary_path.is_file() else None


def manifest_path() -> Path:
    """Path to the install manifest JSON, inside the install directory.
    インストールマニフェストJSONのパス（インストールディレクトリ内）"""
    return _install_dir() / MANIFEST_FILE_NAME
