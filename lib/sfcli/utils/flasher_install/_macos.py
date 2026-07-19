"""
sf flasher install - macOS backend
GUIフラッシャのネイティブインストール機能 - macOSバックエンド

Implements the backend contract documented in
lib/sfcli/utils/flasher_install/__init__.py:

    def install(artifact: Path, version: str, opts: dict) -> list[str]
    def uninstall(manifest: dict | None) -> None
    def installed_app_executable() -> Path | None
    def manifest_path() -> Path

lib/sfcli/utils/flasher_install/__init__.py に記載のバックエンド契約
（上記4関数）を実装する。
"""

import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Optional

from . import MANIFEST_FILENAME, FlasherInstallError
from ..console import console

# ---------------------------------------------------------------------------
# Constants
# 定数
# ---------------------------------------------------------------------------

# Install location: a per-user app bundle, no admin/sudo required (mirrors
# the reasoning for Windows' HKCU-only registry use -- school/lab
# computers where students have no admin rights).
# インストール先: 管理者権限(sudo)不要のユーザー単位アプリバンドル
# （Windows側がHKCUのみを使う理由と同じ -- 生徒に管理者権限の無い
# 学校/研究室のPCを想定）。
APPLICATIONS_DIR = Path.home() / "Applications"
APP_BUNDLE_NAME = "StampFlyFlasher.app"
# The manifest lives in Application Support, NOT next to the .app in
# ~/Applications: a stray JSON file sitting in the user-visible
# Applications folder would look like clutter (or worse, something to
# delete). Application Support is the macOS-sanctioned home for exactly
# this kind of per-app bookkeeping data.
# マニフェストは ~/Applications の .app の隣ではなく Application Support に
# 置く。ユーザーに見える Applications フォルダに JSON が転がっていると
# ゴミに見える（最悪、消されてしまう）。この種のアプリ管理データの
# macOS 流の置き場所が Application Support である。
APP_SUPPORT_DIR = Path.home() / "Library" / "Application Support" / "StampFly"
# Matches the PyInstaller `--name StampFlyFlasher` used to build the app,
# so Contents/MacOS/<this> is always the frozen executable's name.
# ビルド時の PyInstaller `--name StampFlyFlasher` と一致させる。これにより
# Contents/MacOS/<この名前> が常に frozen 実行ファイル名になる。
APP_EXECUTABLE_NAME = "StampFlyFlasher"
BACKUP_SUFFIX = ".old"


def _app_bundle_path() -> Path:
    """Path to the installed (or about-to-be-installed) .app bundle.
    インストール済み(またはこれからインストールする).appバンドルのパス。"""
    return APPLICATIONS_DIR / APP_BUNDLE_NAME


def manifest_path() -> Path:
    """Path to the install manifest (see backend contract and the
    APP_SUPPORT_DIR comment above for why it is not in ~/Applications).
    インストールマニフェストのパス（バックエンド契約、および
    ~/Applications に置かない理由は上の APP_SUPPORT_DIR コメントを参照）。"""
    return APP_SUPPORT_DIR / MANIFEST_FILENAME


def installed_app_executable() -> Optional[Path]:
    """Path to the installed app's launchable executable, or None if not
    installed (see backend contract).
    インストール済みアプリの起動可能な実行ファイルパス。未導入なら None
    （バックエンド契約を参照）。"""
    executable = _app_bundle_path() / "Contents" / "MacOS" / APP_EXECUTABLE_NAME
    return executable if executable.exists() else None


def install(artifact: Path, version: str, opts: dict) -> list:
    """Install `artifact` (a .zip containing a .app, or a bare .app
    directory) to ~/Applications/StampFlyFlasher.app, replacing any
    previous install, and return the list of absolute paths created.
    `artifact`(.appを含む.zip、または素の.appディレクトリ)を
    ~/Applications/StampFlyFlasher.app へインストールする(既存インストール
    は置換)。作成した絶対パスの一覧を返す。"""
    # opts["desktop_shortcut"] has no meaning on macOS -- the Applications
    # folder + Launchpad/Spotlight already surface the app; this parameter
    # only matters to the Windows backend. Accepted (and ignored) here so
    # the caller does not need per-OS branching.
    # opts["desktop_shortcut"] は macOS では無意味（Applicationsフォルダ+
    # Launchpad/Spotlightで既に見つかる。この引数はWindowsバックエンド
    # 専用）。呼び出し側がOSごとの分岐をしなくて済むよう、ここでは
    # 受け取って無視する。
    del opts

    APPLICATIONS_DIR.mkdir(parents=True, exist_ok=True)

    extract_dir: Optional[Path] = None
    try:
        source_app, extract_dir = _resolve_source_app(artifact)
        dest_app = _app_bundle_path()
        backup_app = dest_app.parent / (dest_app.name + BACKUP_SUFFIX)

        # Replace any previous install: move the old bundle aside first,
        # restore it if the copy fails, delete it once the new copy has
        # succeeded. This avoids ever leaving ~/Applications with neither
        # a working old nor new bundle.
        # 既存インストールの置き換え: まず旧バンドルを退避し、コピー失敗
        # 時は復元、成功時に削除する。これにより ~/Applications が
        # 「新旧どちらの動くバンドルも無い」状態になることを防ぐ。
        if backup_app.exists():
            shutil.rmtree(backup_app, ignore_errors=True)
        if dest_app.exists():
            dest_app.rename(backup_app)

        try:
            _ditto_copy(source_app, dest_app)
        except FlasherInstallError:
            if backup_app.exists():
                if dest_app.exists():
                    shutil.rmtree(dest_app, ignore_errors=True)
                backup_app.rename(dest_app)
            raise

        if backup_app.exists():
            shutil.rmtree(backup_app, ignore_errors=True)

        _remove_quarantine_attribute(dest_app)

        return [str(dest_app)]
    finally:
        if extract_dir is not None:
            shutil.rmtree(extract_dir, ignore_errors=True)


def uninstall(manifest: Optional[dict]) -> None:
    """Remove the installed app (and the manifest itself). Uses the
    manifest's file list when available; falls back to the well-known
    default .app path otherwise (see backend contract).
    インストール済みアプリ(とマニフェスト自体)を削除する。マニフェストが
    あればそのファイル一覧を使い、無ければ既知の既定 .app パスへ
    フォールバックする（バックエンド契約を参照）。"""
    targets = [Path(f) for f in manifest["files"]] if manifest and manifest.get("files") else [
        _app_bundle_path()
    ]

    for target in targets:
        if not target.exists():
            continue
        if target.is_dir():
            shutil.rmtree(target, ignore_errors=True)
        else:
            target.unlink()

    manifest_file = manifest_path()
    if manifest_file.exists():
        manifest_file.unlink()

    # Remove the Application Support dir too if the manifest was its only
    # content (leave it alone if other StampFly tools ever store data there).
    # Application Support ディレクトリも、マニフェスト以外に中身が無ければ
    # 削除する（他の StampFly ツールがデータを置いている場合は残す）。
    try:
        APP_SUPPORT_DIR.rmdir()
    except OSError:
        pass


# ---------------------------------------------------------------------------
# Internal helpers
# 内部ヘルパー
# ---------------------------------------------------------------------------


def _resolve_source_app(artifact: Path):
    """Return (source .app path, temp extraction dir or None) for
    `artifact`. Raises FlasherInstallError if `artifact` is neither a
    .app directory nor a .zip containing one.
    `artifact` から (コピー元の.appパス, 一時展開ディレクトリまたはNone)
    を返す。`artifact` が .app ディレクトリでも、.app を含む .zip でも
    ない場合は FlasherInstallError を送出する。"""
    if artifact.is_dir() and artifact.suffix == ".app":
        return artifact, None

    if artifact.is_file() and artifact.suffix == ".zip":
        extract_dir = Path(tempfile.mkdtemp(prefix="stampfly_flasher_install_"))
        _ditto_extract(artifact, extract_dir)
        candidates = sorted(extract_dir.glob("*.app"))
        if not candidates:
            shutil.rmtree(extract_dir, ignore_errors=True)
            raise FlasherInstallError(f"No .app bundle found inside archive: {artifact}")
        return candidates[0], extract_dir

    raise FlasherInstallError(
        f"Unsupported artifact for macOS install: {artifact} "
        "(expected a .app bundle directory or a .zip containing one)"
    )


def _ditto_extract(zip_path: Path, dest_dir: Path) -> None:
    """Extract zip_path into dest_dir with `ditto`, which (unlike `zip`/
    `unzip`) preserves the app bundle's symlinks and resource forks
    correctly -- the same tool release.yml uses to create the archive.
    `ditto` で zip_path を dest_dir へ展開する。`zip`/`unzip` と異なり
    アプリバンドルのシンボリックリンクやリソースフォークを正しく保持
    する -- release.yml がアーカイブ作成に使うのと同じツール。"""
    result = subprocess.run(
        ["ditto", "-x", "-k", str(zip_path), str(dest_dir)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if result.returncode != 0:
        raise FlasherInstallError(f"ditto extraction failed: {result.stderr.strip()}")


def _ditto_copy(src: Path, dst: Path) -> None:
    """Copy an app bundle with `ditto` (preserves symlinks/resource forks,
    unlike shutil.copytree for some bundle contents).
    `ditto` でアプリバンドルをコピーする(一部のバンドル内容に対して
    shutil.copytree と異なりシンボリックリンク/リソースフォークを保持)。"""
    result = subprocess.run(
        ["ditto", str(src), str(dst)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if result.returncode != 0:
        raise FlasherInstallError(f"ditto copy failed: {result.stderr.strip()}")


def _remove_quarantine_attribute(app_path: Path) -> None:
    """Strip the com.apple.quarantine extended attribute Gatekeeper adds
    to anything downloaded from the internet. Without this, macOS
    refuses to launch an unsigned --onefile PyInstaller bundle at all
    (not even via right-click -> Open). Failure here is a warning, not
    fatal: code signing/notarization (once a Developer ID is obtained) is
    the real long-term fix; this is a workaround for un-notarized builds.
    Gatekeeper がインターネット経由の全ダウンロードに付与する
    com.apple.quarantine 拡張属性を除去する。これが無いと、未署名の
    --onefile PyInstaller バンドルは(右クリック→開く でも)一切起動でき
    ない。失敗しても警告に留め致命的エラーにはしない: 本来の恒久対応は
    コード署名/公証(Developer ID 取得後)であり、これは未公証ビルド向けの
    回避策に過ぎない。"""
    result = subprocess.run(
        ["xattr", "-dr", "com.apple.quarantine", str(app_path)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if result.returncode != 0:
        console.warning(
            f"Failed to remove quarantine attribute (Gatekeeper may block launch): "
            f"{result.stderr.strip()}"
        )
