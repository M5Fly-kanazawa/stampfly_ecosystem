"""
sf flasher install - core logic (OS-independent)
GUIフラッシャのネイティブインストール機能 - コア実装（OS非依存）

This module implements the platform-independent half of "install the
StampFly Flasher GUI as a native desktop app": detecting the current
platform, talking to the GitHub Releases API, verifying downloads
against SHA256SUMS.txt, and dispatching to a per-OS backend module
(_macos.py / _windows.py / _linux.py) that knows how to actually place
files, create shortcuts, and register/unregister the app with the OS.

このモジュールは「StampFly Flasher GUI をネイティブデスクトップアプリとして
インストールする」機能のうち、OSに依存しない部分（現在のプラットフォーム判定、
GitHub Releases API との通信、SHA256SUMS.txt によるダウンロード検証、
OSごとのバックエンドモジュール(_macos.py / _windows.py / _linux.py)への
振り分け）を実装する。ファイルの配置・ショートカット作成・OSへの
登録/解除といった実作業は各バックエンドが担う。

The public functions below (FlasherInstallError, detect_platform_key,
fetch_latest_release, select_assets, download_and_verify, install,
uninstall, status, installed_app_executable) form a contract shared with
the per-OS backend modules and with lib/sfcli/commands/flasher.py /
flash.py -- do not change their signatures without updating every
caller.

以下の公開関数（FlasherInstallError・detect_platform_key・
fetch_latest_release・select_assets・download_and_verify・install・
uninstall・status・installed_app_executable）は、OSごとのバックエンドや
lib/sfcli/commands/flasher.py・flash.py と共有される契約である。
シグネチャを変更する場合は全ての呼び出し元を合わせて更新すること。
"""

import hashlib
import importlib
import json
import os
import platform
import re
import shutil
import ssl
import stat
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Optional

from ..console import console

# ---------------------------------------------------------------------------
# Constants (no magic numbers/strings inlined below this block)
# 定数（この節より下にマジックナンバー/マジックストリングは埋め込まない）
# ---------------------------------------------------------------------------

# GitHub repository that publishes the StampFly Flasher releases; kept in
# sync with the same constants in tools/flasher_gui/stampfly_flasher.py.
# StampFly Flasher のリリースを公開している GitHub リポジトリ。
# tools/flasher_gui/stampfly_flasher.py の同名定数と一致させること。
GITHUB_OWNER = "M5Fly-kanazawa"
GITHUB_REPO = "stampfly_ecosystem"
GITHUB_API_LATEST_RELEASE_URL = (
    f"https://api.github.com/repos/{GITHUB_OWNER}/{GITHUB_REPO}/releases/latest"
)
GITHUB_RELEASES_PAGE_URL = f"https://github.com/{GITHUB_OWNER}/{GITHUB_REPO}/releases/latest"

# The GitHub API rejects requests without a User-Agent header (HTTP 403).
# GitHub API は User-Agent ヘッダの無いリクエストを拒否する(HTTP 403)。
HTTP_USER_AGENT = "sf-flasher-install/1.0"
HTTP_ACCEPT_HEADER = "application/vnd.github+json"

# Network / download tuning, mirrored from stampfly_flasher.py so both
# tools behave the same way against the same GitHub API.
# ネットワーク/ダウンロードの調整値。同じ GitHub API に対して両ツールが
# 同じ挙動になるよう stampfly_flasher.py と揃えている。
NETWORK_TIMEOUT_SECONDS = 20
DOWNLOAD_CHUNK_SIZE_BYTES = 65536  # 64 KiB per read() call
VERSION_PROBE_TIMEOUT_SECONDS = 10  # generous but bounded: never hang `sf flasher install`

# Release asset naming, as produced by .github/workflows/release.yml's
# `flasher_gui` job ("StampFlyFlasher_<tag>_<platform_key>.<ext>").
# release.yml の flasher_gui ジョブが生成するリリース資産の命名規則。
ASSET_NAME_PREFIX = "StampFlyFlasher"
SHA256SUMS_ASSET_NAME = "SHA256SUMS.txt"
# Extension used per platform key ("" = no extension, e.g. a bare Linux
# PyInstaller onefile binary).
# プラットフォームキーごとの拡張子("" = 拡張子なし。Linuxの素の
# PyInstaller onefile 実行ファイルなど)。
PLATFORM_ASSET_EXTENSIONS = {
    "windows-x64": "exe",
    "macos-arm64": "zip",
    "macos-x64": "zip",
    "linux-x64": "",
}
# First release tag expected to publish a Linux asset; used only to give
# a more specific error message when an older release lacks one.
# Linux 資産を初めて公開する予定のリリースタグ。古いリリースに Linux
# 資産が無い場合、より具体的なエラーメッセージを出すためだけに使う。
FIRST_RELEASE_WITH_LINUX_ASSET = "v2026.07.2"

# Manifest file name, written into the OS-specific install directory by
# each backend's manifest_path(). Shared across all backends so uninstall
# logic (in this module) can be written once.
# マニフェストファイル名。各バックエンドの manifest_path() が示す
# インストール先ディレクトリに書き込む。全バックエンド共通にすることで
# アンインストール処理をここで一本化できる。
MANIFEST_FILENAME = "stampfly_flasher_manifest.json"

# sha256sum-format line: "<64 hex chars>  <filename>" (optionally with a
# "*" before the filename for binary mode).
# sha256sum 形式の行: "<64桁16進数> <ファイル名>"（バイナリモードでは
# ファイル名の前に "*" が付くこともある）。
SHA256SUMS_LINE_PATTERN = re.compile(r"^([0-9a-fA-F]{64})\s+\*?(\S.*)$")

BYTES_PER_KIB = 1024.0
SIZE_UNITS = ("B", "KB", "MB", "GB", "TB")


def _create_https_context() -> ssl.SSLContext:
    """SSL context for the GitHub API / asset downloads, preferring
    certifi's CA bundle when available.

    macOS Pythons frequently lack usable OpenSSL default verify paths
    (they do not use the system Keychain), which makes every HTTPS call
    fail with CERTIFICATE_VERIFY_FAILED — observed on a real Mac
    (2026-07-20) in the frozen flasher app; the same failure mode can
    hit any venv here. certifi is imported lazily inside this function:
    this module's import-time stdlib-only contract (it is part of the
    broken-environment recovery path) stays intact, and environments
    without certifi simply fall back to the default context (Windows
    always works there via the OS cert store).

    GitHub API / 資産ダウンロード用の SSL コンテキスト。certifi の CA
    バンドルがあれば優先して使う。macOS の Python は OpenSSL の既定検証
    パスが使えないことが多く（システムのキーチェーンを参照しない）、
    全 HTTPS 呼び出しが CERTIFICATE_VERIFY_FAILED で失敗する — 凍結
    フラッシャアプリの実機 Mac（2026-07-20）で確認済みで、venv でも同じ
    故障モードがあり得る。certifi はこの関数内で遅延 import する:
    本モジュールの「import 時は標準ライブラリのみ」という契約（壊れた
    環境の復旧経路の一部であるため）を保ちつつ、certifi が無い環境では
    既定コンテキストにフォールバックする（Windows は OS の証明書ストア
    経由で常に動く）。
    """
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()


class FlasherInstallError(Exception):
    """Raised for any expected failure in the install/uninstall flow
    (unsupported platform, network failure, checksum mismatch, missing
    asset, ...). Callers should catch this and print a friendly message
    instead of a raw traceback.
    インストール/アンインストール処理中に想定されるあらゆる失敗
    （未対応プラットフォーム、通信失敗、チェックサム不一致、資産欠落等）
    で送出する。呼び出し側はこれを捕捉し、生のトレースバックではなく
    親しみやすいメッセージを表示すること。"""


# ---------------------------------------------------------------------------
# Platform detection
# プラットフォーム判定
# ---------------------------------------------------------------------------


def detect_platform_key() -> str:
    """Return this machine's platform key: "windows-x64" | "macos-arm64" |
    "macos-x64" | "linux-x64". Raises FlasherInstallError for anything
    else (32-bit, unsupported OS, unsupported CPU architecture).
    このマシンのプラットフォームキーを返す。非対応(32bit・未対応OS・
    未対応CPUアーキテクチャ)の場合は FlasherInstallError を送出する。"""
    machine = platform.machine().lower()

    if sys.platform == "win32":
        # ESP-IDF/StampFly targets 64-bit desktops only; 32-bit Windows is
        # not published as a release asset.
        # StampFly は64bitデスクトップのみ対象。32bit Windows 向け資産は
        # リリースに存在しない。
        return "windows-x64"

    if sys.platform == "darwin":
        if machine in ("arm64", "aarch64"):
            return "macos-arm64"
        if machine in ("x86_64", "amd64"):
            return "macos-x64"
        raise FlasherInstallError(f"Unsupported macOS architecture: {machine}")

    if sys.platform.startswith("linux"):
        if machine in ("x86_64", "amd64"):
            return "linux-x64"
        raise FlasherInstallError(f"Unsupported Linux architecture: {machine}")

    raise FlasherInstallError(f"Unsupported platform: {sys.platform}")


# ---------------------------------------------------------------------------
# GitHub release lookup
# GitHub リリースの取得
# ---------------------------------------------------------------------------


def fetch_latest_release() -> dict:
    """Fetch the latest release metadata (JSON) from the GitHub API.
    GitHub API から最新リリースのメタデータ(JSON)を取得する。"""
    headers = {"User-Agent": HTTP_USER_AGENT, "Accept": HTTP_ACCEPT_HEADER}
    # Use an authenticated request when a CI token is available: hosted CI
    # runners share egress IPs across many jobs, so anonymous API calls
    # from CI are frequently rate-limited (HTTP 403); authenticated calls
    # are not. End users normally have no token set and stay anonymous.
    # CIトークンがあれば認証付きリクエストにする。ホスト型CIランナーは
    # 多数のジョブで送信元IPを共有するため、匿名呼び出しはレート制限
    # (HTTP 403)になりやすい。認証付きなら制限に掛からない。通常利用の
    # エンドユーザーはトークン未設定のため従来どおり匿名で動く。
    api_token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if api_token:
        headers["Authorization"] = "Bearer " + api_token

    request = urllib.request.Request(GITHUB_API_LATEST_RELEASE_URL, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=NETWORK_TIMEOUT_SECONDS, context=_create_https_context()) as response:
            return json.loads(response.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
        raise FlasherInstallError(
            f"Failed to fetch the latest release info from GitHub: {exc}"
        ) from exc


def select_assets(release: dict, platform_key: str) -> tuple[dict, dict]:
    """Pick the firmware asset and the SHA256SUMS.txt asset for
    platform_key out of a GitHub release JSON's "assets" list. Raises
    FlasherInstallError if either is missing (e.g. an older release that
    predates Linux support).
    GitHub リリース JSON の "assets" 一覧から、platform_key 向けの資産と
    SHA256SUMS.txt 資産を選ぶ。どちらかが無い場合(Linux対応前の古い
    リリース等)は FlasherInstallError を送出する。"""
    if platform_key not in PLATFORM_ASSET_EXTENSIONS:
        raise FlasherInstallError(f"Unsupported platform key: {platform_key}")

    extension = PLATFORM_ASSET_EXTENSIONS[platform_key]
    name_suffix = f"_{platform_key}.{extension}" if extension else f"_{platform_key}"

    firmware_asset: Optional[dict] = None
    sums_asset: Optional[dict] = None
    for asset in release.get("assets", []):
        name = asset.get("name", "")
        if name == SHA256SUMS_ASSET_NAME:
            sums_asset = asset
        elif name.startswith(ASSET_NAME_PREFIX) and name.endswith(name_suffix):
            firmware_asset = asset

    tag = release.get("tag_name", "unknown")

    if firmware_asset is None:
        hint = ""
        if platform_key == "linux-x64":
            hint = (
                f" Linux ビルドは {FIRST_RELEASE_WITH_LINUX_ASSET} 以降のリリースが必要です。/ "
                f"Linux builds require release {FIRST_RELEASE_WITH_LINUX_ASSET} or later."
            )
        raise FlasherInstallError(
            f"No flasher asset found for platform '{platform_key}' in release {tag}.{hint}"
        )

    if sums_asset is None:
        raise FlasherInstallError(
            f"No {SHA256SUMS_ASSET_NAME} asset found in release {tag}."
        )

    return firmware_asset, sums_asset


# ---------------------------------------------------------------------------
# Download + SHA256 verification
# ダウンロードと SHA256 検証
# ---------------------------------------------------------------------------


def download_and_verify(asset: dict, sums_asset: dict, dest_dir: Path) -> Path:
    """Download `asset` and `sums_asset` into dest_dir, verify `asset`'s
    SHA256 against the matching line in sums_asset's content, and return
    the path to the downloaded (and verified) asset file. Raises
    FlasherInstallError and removes any partial files on any failure.
    asset と sums_asset を dest_dir にダウンロードし、asset の SHA256 を
    sums_asset の内容中の該当行と照合する。検証済みの asset ファイルの
    パスを返す。失敗時は FlasherInstallError を送出し、部分ファイルを
    残さず削除する。"""
    dest_dir.mkdir(parents=True, exist_ok=True)
    bin_path = dest_dir / asset["name"]
    sums_path = dest_dir / sums_asset["name"]

    try:
        _download_to_file(asset["browser_download_url"], bin_path)
        _download_to_file(sums_asset["browser_download_url"], sums_path)

        checksums = _parse_sha256sums(sums_path.read_text(encoding="utf-8"))
        expected_digest = checksums.get(asset["name"])
        if expected_digest is None:
            raise FlasherInstallError(
                f"No checksum entry for {asset['name']} in {sums_asset['name']}"
            )

        actual_digest = _sha256_of_file(bin_path)
        if actual_digest.lower() != expected_digest.lower():
            raise FlasherInstallError(
                f"SHA256 mismatch for {asset['name']}: expected {expected_digest}, "
                f"got {actual_digest} (download may be corrupted)"
            )

        return bin_path

    except FlasherInstallError:
        _cleanup_partial_files(bin_path, sums_path)
        raise
    except (urllib.error.URLError, OSError) as exc:
        _cleanup_partial_files(bin_path, sums_path)
        raise FlasherInstallError(f"Download failed: {exc}") from exc


def _download_to_file(url: str, destination: Path) -> None:
    """Stream url into destination in fixed-size chunks (keeps memory use
    flat regardless of file size).
    url を destination へ固定サイズのチャンクでストリーミングダウンロード
    する(ファイルサイズに関わらずメモリ使用量が一定)。"""
    request = urllib.request.Request(url, headers={"User-Agent": HTTP_USER_AGENT})
    with urllib.request.urlopen(request, timeout=NETWORK_TIMEOUT_SECONDS, context=_create_https_context()) as response:
        with open(destination, "wb") as out_file:
            while True:
                chunk = response.read(DOWNLOAD_CHUNK_SIZE_BYTES)
                if not chunk:
                    break
                out_file.write(chunk)


def _cleanup_partial_files(*paths: Path) -> None:
    """Best-effort removal of partially-downloaded files.
    ダウンロード途中で残った部分ファイルのベストエフォート削除。"""
    for path in paths:
        try:
            if path.exists():
                path.unlink()
        except OSError:
            pass


def _parse_sha256sums(text: str) -> dict:
    """Parse SHA256SUMS.txt content into {filename: lowercase hex digest}.
    SHA256SUMS.txt の内容を {ファイル名: 小文字16進ダイジェスト} に変換。"""
    checksums = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        match = SHA256SUMS_LINE_PATTERN.match(line)
        if match is None:
            continue
        digest, filename = match.group(1), match.group(2).strip()
        checksums[filename] = digest.lower()
    return checksums


def _sha256_of_file(path: Path) -> str:
    """Compute the SHA256 hex digest of a file in fixed-size chunks.
    ファイルの SHA256 16進ダイジェストを固定サイズのチャンクで計算する。"""
    hasher = hashlib.sha256()
    with open(path, "rb") as file_handle:
        while True:
            chunk = file_handle.read(DOWNLOAD_CHUNK_SIZE_BYTES)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


# ---------------------------------------------------------------------------
# Backend dispatch
# バックエンドの振り分け
# ---------------------------------------------------------------------------


def _load_backend():
    """Lazily import and return the OS-specific backend module for the
    current sys.platform.

    Uses importlib *inside this function*, not a module-level import, so
    that this module keeps working even if a sibling backend file (e.g.
    _windows.py / _linux.py, written independently) is missing or broken
    at import time -- only the backend for the machine we are actually
    running on is ever touched.

    現在の sys.platform に対応するバックエンドモジュールを遅延インポート
    して返す。

    関数内で importlib を使い、モジュールレベルでは import しない。これに
    より、兄弟バックエンドファイル(_windows.py / _linux.py。別担当が独立に
    書く)が import 時点で存在しない/壊れていても、このモジュール自体は
    動作し続ける——実際に触るのは、いま動いているマシンに対応する
    バックエンドだけ。
    """
    if sys.platform == "darwin":
        backend_name = "_macos"
    elif sys.platform == "win32":
        backend_name = "_windows"
    elif sys.platform.startswith("linux"):
        backend_name = "_linux"
    else:
        raise FlasherInstallError(f"Unsupported platform: {sys.platform}")

    try:
        return importlib.import_module(f".{backend_name}", package=__name__)
    except ImportError as exc:
        raise FlasherInstallError(
            f"Flasher install backend for this platform is not available "
            f"({backend_name}.py): {exc}"
        ) from exc


# ---------------------------------------------------------------------------
# Manifest read/write
# マニフェストの読み書き
# ---------------------------------------------------------------------------


def _write_manifest(manifest_path: Path, version: str, files: list) -> None:
    """Write the install manifest recording what was installed, so
    uninstall() can remove exactly (and only) those files.
    何がインストールされたかを記録するマニフェストを書き込む。これにより
    uninstall() はそのファイルだけを正確に削除できる。"""
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest = {
        "version": version,
        "installed_at": datetime.now().isoformat(),
        "files": [str(Path(f)) for f in files],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")


def _read_manifest(manifest_path: Path) -> Optional[dict]:
    """Read the install manifest, or None if absent/unreadable/corrupt.
    マニフェストを読み込む。存在しない/読めない/壊れている場合は None。"""
    if not manifest_path.exists():
        return None
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


# ---------------------------------------------------------------------------
# Version probing for --from-file installs
# --from-file インストール時のバージョン特定
# ---------------------------------------------------------------------------


def _probe_artifact_version(artifact: Path) -> str:
    """Best-effort: run the artifact's own `--version` flag to discover
    its app version, for the --from-file install path where no GitHub
    release tag is available to use instead. Never raises; falls back to
    "unknown" if the artifact cannot be run on this machine (e.g. it is a
    build for a different OS) or does not print a parseable version.
    --from-file 経由のインストールでは GitHub リリースタグが無いため、
    アーティファクト自身の --version を実行してバージョンを推定する
    (ベストエフォート)。このマシンで実行できない(別OS向けビルド等)場合や
    解析可能な出力が無い場合は例外を出さず "unknown" にフォールバックする。
    """
    if artifact.is_dir() and artifact.suffix == ".app":
        return _run_version_probe(artifact / "Contents" / "MacOS" / artifact.stem)

    if artifact.is_file() and artifact.suffix == ".zip":
        return _probe_version_from_zip(artifact)

    if artifact.is_file():
        return _run_version_probe(artifact)

    return "unknown"


def _probe_version_from_zip(zip_path: Path) -> str:
    """Extract zip_path to a throwaway temp dir just long enough to find
    and run its bundled executable's --version output. The real install
    extraction happens again, independently, inside the OS backend's
    install() -- this duplication is a deliberate, cheap trade-off for
    keeping version probing simple and self-contained.
    zip_path を使い捨ての一時ディレクトリへ展開し、同梱の実行ファイルの
    --version 出力を取得するためだけに使う。実際のインストール時の展開は
    OSバックエンドの install() 内で独立にもう一度行う——バージョン特定を
    単純・自己完結にするための意図的な二重展開である。"""
    import zipfile

    extract_dir = Path(tempfile.mkdtemp(prefix="stampfly_flasher_probe_"))
    try:
        with zipfile.ZipFile(zip_path) as zip_file:
            zip_file.extractall(extract_dir)
        for app_dir in sorted(extract_dir.glob("*.app")):
            executable = app_dir / "Contents" / "MacOS" / app_dir.stem
            if executable.exists():
                return _run_version_probe(executable)
        for entry in sorted(extract_dir.iterdir()):
            if entry.is_file():
                return _run_version_probe(entry)
    except (OSError, zipfile.BadZipFile):
        pass
    finally:
        shutil.rmtree(extract_dir, ignore_errors=True)
    return "unknown"


def _run_version_probe(executable: Path) -> str:
    """Run `executable --version` and return the last whitespace-separated
    token of its first stdout line (the version, per print_version_info()
    in stampfly_flasher.py: "StampFly Flasher 0.1.0"). Returns "unknown"
    on any failure.
    `executable --version` を実行し、標準出力1行目の最後のトークン
    (stampfly_flasher.py の print_version_info() が出す
    "StampFly Flasher 0.1.0" のバージョン部分)を返す。失敗時は
    "unknown"。"""
    if not executable.exists():
        return "unknown"
    try:
        os.chmod(
            executable,
            executable.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH,
        )
    except OSError:
        pass
    try:
        result = subprocess.run(
            [str(executable), "--version"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=VERSION_PROBE_TIMEOUT_SECONDS,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "unknown"

    lines = (result.stdout or "").strip().splitlines()
    if not lines:
        return "unknown"
    tokens = lines[0].split()
    return tokens[-1] if tokens else "unknown"


# ---------------------------------------------------------------------------
# Confirmation prompt
# 確認プロンプト
# ---------------------------------------------------------------------------


def _confirm(prompt: str, assume_yes: bool) -> bool:
    """Ask a Y/n question on the console; always True when assume_yes
    (the --yes flag, used by CI / scripts/installer.py). In a
    non-interactive terminal without --yes, refuses (returns False)
    rather than blocking forever on input().
    コンソールで Y/n を尋ねる。assume_yes(--yes フラグ。CI /
    scripts/installer.py 用)なら常に True。--yes 無しの非対話端末では
    input() で無限に待つのではなく False を返して拒否する。"""
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        console.warning(
            "Not running in an interactive terminal; re-run with --yes to proceed non-interactively."
        )
        return False
    console.print(f"{prompt} [Y/n] ", end="")
    try:
        answer = input().strip().lower()
    except EOFError:
        return False
    return answer in ("", "y", "yes")


def _format_size(num_bytes) -> str:
    """Format a byte count as a human-readable string (e.g. "12.3MB").
    バイト数を人間に読みやすい文字列("12.3MB" 等)に整形する。"""
    if not isinstance(num_bytes, (int, float)):
        return "unknown size"
    size = float(num_bytes)
    for unit in SIZE_UNITS:
        if size < BYTES_PER_KIB:
            return f"{size:.1f}{unit}"
        size /= BYTES_PER_KIB
    return f"{size:.1f}{SIZE_UNITS[-1]}"


def _print_manual_install_hint(platform_key: str) -> None:
    """Print manual-installation instructions -- shown whenever the
    automatic release lookup/download fails, so a network problem never
    leaves the user stuck with no way forward.
    手動インストール手順を表示する。自動のリリース取得/ダウンロードが
    失敗した際に必ず表示し、通信トラブルでユーザーが手詰まりにならない
    ようにする。"""
    console.print()
    console.print("Manual installation / 手動インストール手順:")
    console.print(f"  1. Open {GITHUB_RELEASES_PAGE_URL}")
    console.print(f"  2. Download the asset for your platform ({platform_key})")
    console.print("  3. Re-run: sf flasher install --from-file <downloaded file>")


# ---------------------------------------------------------------------------
# Public API: install / uninstall / status / installed_app_executable
# 公開API: install / uninstall / status / installed_app_executable
# ---------------------------------------------------------------------------


def install(
    from_file: Optional[Path] = None,
    desktop_shortcut: bool = True,
    assume_yes: bool = False,
) -> int:
    """Install the StampFly Flasher GUI as a native app for this OS.

    from_file=None fetches and verifies the latest GitHub release for
    the detected platform; from_file=<path> installs from that local
    build artifact instead (.exe / .zip / .app directory / bare
    executable, interpreted by the OS backend). Returns a process exit
    code (0 = success).

    StampFly Flasher GUI をこのOS向けのネイティブアプリとしてインストール
    する。from_file=None なら検出したプラットフォーム向けの最新 GitHub
    リリースを取得・検証する。from_file=<path> ならその場のビルド成果物
    (.exe / .zip / .app ディレクトリ / 素の実行ファイル。解釈はOSバック
    エンドが行う)からインストールする。プロセス終了コードを返す
    (0=成功)。
    """
    try:
        platform_key = detect_platform_key()
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1

    try:
        backend = _load_backend()
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1

    download_dir: Optional[Path] = None
    try:
        if from_file is not None:
            artifact = Path(from_file).expanduser().resolve()
            if not artifact.exists():
                console.error(f"File not found: {artifact}")
                return 1
            console.info(f"Determining version from local file: {artifact.name}")
            version = _probe_artifact_version(artifact)
            console.info(f"Detected version: {version}")
        else:
            console.info("Checking the latest StampFly Flasher release on GitHub...")
            try:
                release = fetch_latest_release()
                asset, sums_asset = select_assets(release, platform_key)
            except FlasherInstallError as exc:
                console.error(str(exc))
                _print_manual_install_hint(platform_key)
                return 1

            version = release.get("tag_name", "unknown")
            console.info(
                f"Latest release: {version} ({asset['name']}, {_format_size(asset.get('size'))})"
            )

            if not _confirm(f"Install StampFly Flasher {version} to this computer?", assume_yes):
                console.warning("Installation cancelled.")
                return 1

            download_dir = Path(tempfile.mkdtemp(prefix="stampfly_flasher_dl_"))
            console.info("Downloading and verifying SHA256...")
            try:
                artifact = download_and_verify(asset, sums_asset, download_dir)
            except FlasherInstallError as exc:
                console.error(str(exc))
                return 1
            console.success("SHA256 verified.")

        console.info(f"Installing for platform: {platform_key}")
        try:
            created_files = backend.install(
                artifact, version, {"desktop_shortcut": desktop_shortcut}
            )
        except FlasherInstallError as exc:
            console.error(str(exc))
            return 1

        _write_manifest(backend.manifest_path(), version, created_files)

        console.success(f"StampFly Flasher {version} installed.")
        for created_file in created_files:
            console.print(f"  {created_file}")
        return 0
    finally:
        if download_dir is not None:
            shutil.rmtree(download_dir, ignore_errors=True)


def uninstall(assume_yes: bool = False) -> int:
    """Remove the installed StampFly Flasher native app (files listed in
    the manifest; falls back to known default paths if the manifest is
    missing). Returns a process exit code (0 = success, including the
    "already not installed" case).
    インストール済みの StampFly Flasher ネイティブアプリを削除する
    (マニフェストに記載のファイル。マニフェストが無い場合は既知の
    既定パスへフォールバック)。プロセス終了コードを返す(0=成功。
    「もともと未導入」の場合も含む)。"""
    try:
        backend = _load_backend()
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1

    manifest = _read_manifest(backend.manifest_path())

    if manifest is None and backend.installed_app_executable() is None:
        console.warning("StampFly Flasher is not installed.")
        return 0

    if manifest:
        console.info(f"Installed version: {manifest.get('version', 'unknown')}")
        for installed_file in manifest.get("files", []):
            console.print(f"  {installed_file}")
    else:
        console.warning("No install manifest found; falling back to known default paths.")

    if not _confirm("Remove StampFly Flasher from this computer?", assume_yes):
        console.warning("Uninstall cancelled.")
        return 1

    try:
        backend.uninstall(manifest)
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1

    console.success("StampFly Flasher uninstalled.")
    return 0


def status() -> int:
    """Print whether StampFly Flasher is installed, and if so, its
    version/executable path/install date. Always returns 0 (this is a
    read-only query, not something that can meaningfully "fail").
    StampFly Flasher が導入済みか、導入済みならバージョン/実行ファイル
    パス/導入日時を表示する。常に0を返す(読み取り専用の照会であり、
    意味のある「失敗」は無い)。"""
    try:
        platform_key = detect_platform_key()
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1
    console.print(f"Platform: {platform_key}")

    try:
        backend = _load_backend()
    except FlasherInstallError as exc:
        console.error(str(exc))
        return 1

    executable = backend.installed_app_executable()
    if executable is None:
        console.print("Installed: no")
        console.print("Run 'sf flasher install' to install it.")
        return 0

    manifest = _read_manifest(backend.manifest_path())
    console.print("Installed: yes")
    console.print(f"  Executable: {executable}")
    if manifest:
        console.print(f"  Version:    {manifest.get('version', 'unknown')}")
        console.print(f"  Installed:  {manifest.get('installed_at', 'unknown')}")
    return 0


def installed_app_executable() -> Optional[Path]:
    """Return the path to the installed native app's executable, or None
    if not installed (or the platform is unsupported). Never raises --
    callers like flash.py's _launch_gui() use this to decide whether to
    launch the native app or fall back to the script.
    インストール済みネイティブアプリの実行ファイルパスを返す。未導入
    (または非対応プラットフォーム)なら None。例外は送出しない --
    flash.py の _launch_gui() 等の呼び出し元はこれでネイティブアプリを
    起動するかスクリプトにフォールバックするかを判断する。"""
    try:
        backend = _load_backend()
    except FlasherInstallError:
        return None
    return backend.installed_app_executable()
