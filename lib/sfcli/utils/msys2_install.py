"""
sf sils build's MSYS2 / MinGW-w64 toolchain auto-install (Windows only)
sf sils build 用の MSYS2 / MinGW-w64 ツールチェーン自動導入（Windows専用）

The canonical install method here is downloading MSYS2's own official
"base" self-extracting archive directly from GitHub and extracting it —
the same unattended approach the official msys2/setup-msys2 GitHub Action
uses in CI, with no GUI installer and no interactive wizard. This is
deliberately NOT winget: msys2.org's own installer docs
(https://www.msys2.org/docs/installer/) list only the GUI installer, the
sfx archive, and the tarballs as install methods — winget is not among
them. The `MSYS2.MSYS2` winget manifest lives in Microsoft's
community-editable winget-pkgs repo, not something the MSYS2 project
maintains itself, and its integration has known, long-unresolved problems
(e.g. "NoApplicableInstallers"; see microsoft/winget-pkgs#287981 and
msys2/msys2-installer#47) — not a broken link in this repo. winget is
therefore only used if a caller explicitly opts in (prefer_winget=True);
by default this module never touches it.

ここでの正規のインストール方法は、MSYS2 公式の "base" 自己解凍アーカイブを
GitHub から直接ダウンロードして展開することである — GUI インストーラや対話
ウィザードを使わない、公式の msys2/setup-msys2 GitHub Action が CI で使うのと
同じ無人インストール方式。これは意図的に winget を使わない選択である:
msys2.org 自身のインストーラ文書（https://www.msys2.org/docs/installer/）が
挙げるインストール方法は GUI インストーラ・sfx アーカイブ・tarball のみで、
winget は含まれていない。`MSYS2.MSYS2` の winget マニフェストは Microsoft が
運営する誰でも編集できる winget-pkgs リポジトリに存在するものであり、MSYS2
プロジェクト自身が保守しているものではなく、連携には既知の長期未解決の問題が
ある（例: "NoApplicableInstallers"。参照: microsoft/winget-pkgs#287981,
msys2/msys2-installer#47）— 本リポジトリ側のリンク切れではない。したがって
winget は呼び出し側が明示的にオプトイン（prefer_winget=True）した場合のみ使い、
既定では一切触れない。
"""

import shutil
import ssl
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

from .console import console

# ---------------------------------------------------------------------------
# Constants (no magic numbers/strings inlined below this block)
# 定数（この節より下にマジックナンバー/マジックストリングは埋め込まない）
# ---------------------------------------------------------------------------

# MSYS2's own default install location (winget and the GUI installer both
# use this path; simulator/sils/commands/sils.py's mingw_bin() checks the
# same location independently).
# MSYS2 自身の既定インストール先（winget・GUI インストーラともにこのパスを
# 使う。simulator/sils/commands/sils.py の mingw_bin() も同じ場所を独立に
# 確認している）。
MSYS2_ROOT = Path("C:/msys64")
_BASH_EXE = MSYS2_ROOT / "usr" / "bin" / "bash.exe"
_MINGW64_BIN = MSYS2_ROOT / "mingw64" / "bin"

# msys2-installer publishes a stable "latest" alias for its base tarball's
# self-extracting archive; using it avoids a GitHub API call (and its rate
# limit) just to resolve a version tag.
# msys2-installer は base tarball の自己解凍アーカイブに固定の "latest"
# エイリアスを公開している。バージョンタグ解決だけのために GitHub API を
# 呼ぶ（レート制限もある）必要がない。
MSYS2_BASE_SFX_URL = (
    "https://github.com/msys2/msys2-installer/releases/latest/download/"
    "msys2-base-x86_64-latest.sfx.exe"
)

HTTP_USER_AGENT = "sf-sils-msys2-install/1.0"
NETWORK_TIMEOUT_SECONDS = 30
DOWNLOAD_CHUNK_SIZE_BYTES = 65536

WINGET_TIMEOUT_SECONDS = 600
EXTRACT_TIMEOUT_SECONDS = 300
PACMAN_TIMEOUT_SECONDS = 1800

# mingw_bin() in sils.py checks for these two together; kept here too so
# this module never depends on importing the commands package.
# sils.py の mingw_bin() もこの2つの組を確認している。commands パッケージへの
# 依存を作らないよう、ここでも同じ組を独立に確認する。
_TOOLCHAIN_MARKER_EXES = ("g++.exe", "ninja.exe")

TOOLCHAIN_PACKAGES = (
    "mingw-w64-x86_64-toolchain",
    "mingw-w64-x86_64-cmake",
    "mingw-w64-x86_64-ninja",
)

WINGET_ISSUE_HINT = (
    "known, unresolved winget/MSYS2 integration issue (e.g. "
    "'NoApplicableInstallers'; see "
    "https://github.com/microsoft/winget-pkgs/issues/287981 and "
    "https://github.com/msys2/msys2-installer/issues/47), not something "
    "sf can fix — falling back to a direct download."
)


def mingw_bin() -> Optional[Path]:
    """Return MSYS2's mingw64/bin dir if g++ and ninja are both present
    there, else None. Mirrors sils.py's mingw_bin() default-path check
    (kept independent so this module has no dependency on the commands
    package).
    MSYS2 の mingw64/bin に g++ と ninja が両方あればそのパスを、無ければ
    None を返す。sils.py の mingw_bin() の既定パス確認と同じ内容を、
    commands パッケージへ依存せずここで独立に持つ。"""
    if all((_MINGW64_BIN / exe).exists() for exe in _TOOLCHAIN_MARKER_EXES):
        return _MINGW64_BIN
    return None


def _has_msys2_root() -> bool:
    return _BASH_EXE.exists()


def _confirm(prompt: str, assume_yes: bool) -> bool:
    """Ask a Y/n question; always True when assume_yes. In a non-interactive
    terminal without assume_yes, refuses rather than blocking on input().
    Y/n を尋ねる。assume_yes なら常に True。--yes 無しの非対話端末では
    input() で待たず False を返して拒否する。"""
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


def _create_https_context() -> ssl.SSLContext:
    """SSL context preferring certifi's CA bundle when available (see the
    same pattern and rationale in flasher_install/__init__.py).
    certifi の CA バンドルを優先する SSL コンテキスト
    （flasher_install/__init__.py と同じ方式・理由）。"""
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()


def _download_to_file(url: str, destination: Path) -> None:
    """Stream url into destination in fixed-size chunks.
    url を destination へ固定サイズのチャンクでストリーミングダウンロードする。"""
    request = urllib.request.Request(url, headers={"User-Agent": HTTP_USER_AGENT})
    with urllib.request.urlopen(
        request, timeout=NETWORK_TIMEOUT_SECONDS, context=_create_https_context()
    ) as response:
        with open(destination, "wb") as out_file:
            while True:
                chunk = response.read(DOWNLOAD_CHUNK_SIZE_BYTES)
                if not chunk:
                    break
                out_file.write(chunk)


def _try_winget_install() -> bool:
    """Attempt `winget install --id MSYS2.MSYS2` (the documented manual
    command). Returns True only if MSYS2 actually ends up installed —
    winget can exit 0 without installing anything, and is also known to
    fail outright (see WINGET_ISSUE_HINT).
    `winget install --id MSYS2.MSYS2`（案内している手動コマンド）を試みる。
    実際に MSYS2 が導入された場合のみ True を返す — winget は何も導入せず
    exit 0 することがあり、そもそも失敗することも知られている
    （WINGET_ISSUE_HINT 参照）。"""
    winget = shutil.which("winget")
    if winget is None:
        console.warning("  winget not found on PATH; skipping the winget install attempt.")
        return False

    console.info("  Trying: winget install --id MSYS2.MSYS2 ...")
    try:
        result = subprocess.run(
            [winget, "install", "--id", "MSYS2.MSYS2", "--silent",
             "--accept-package-agreements", "--accept-source-agreements"],
            timeout=WINGET_TIMEOUT_SECONDS,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        console.warning(f"  winget install failed to run: {exc}")
        return False

    if result.returncode != 0:
        console.warning(f"  winget install exited with code {result.returncode} — {WINGET_ISSUE_HINT}")
        return False

    if not _has_msys2_root():
        console.warning(f"  winget reported success but {MSYS2_ROOT} still does not exist — {WINGET_ISSUE_HINT}")
        return False

    return True


def _direct_install_msys2() -> bool:
    """Download msys2-base-x86_64-latest.sfx.exe and self-extract it
    straight into C:\\ (it contains a top-level msys64/ folder). This is
    the same unattended base-tarball approach the official
    msys2/setup-msys2 GitHub Action uses in CI, so it needs no GUI and no
    interactive wizard.
    msys2-base-x86_64-latest.sfx.exe をダウンロードし、C:\\ へ直接自己解凍する
    （アーカイブ直下に msys64/ フォルダを含む）。公式の msys2/setup-msys2
    GitHub Action が CI で使うのと同じ無人の base tarball 方式であり、GUI も
    対話ウィザードも不要。"""
    console.info(f"  Downloading {MSYS2_BASE_SFX_URL} ...")
    download_dir = Path(tempfile.mkdtemp(prefix="sf_msys2_dl_"))
    sfx_path = download_dir / "msys2-base-x86_64-latest.sfx.exe"
    try:
        _download_to_file(MSYS2_BASE_SFX_URL, sfx_path)
    except (urllib.error.URLError, OSError) as exc:
        console.error(f"  Download failed: {exc}")
        shutil.rmtree(download_dir, ignore_errors=True)
        return False

    console.info(r"  Extracting to C:\ (self-extracting archive, unattended: -y -oC:\)...")
    try:
        result = subprocess.run([str(sfx_path), "-y", "-oC:\\"], timeout=EXTRACT_TIMEOUT_SECONDS)
    except (OSError, subprocess.SubprocessError) as exc:
        console.error(f"  Extraction failed: {exc}")
        return False
    finally:
        shutil.rmtree(download_dir, ignore_errors=True)

    if result.returncode != 0 or not _has_msys2_root():
        console.error(f"  Extraction did not produce {MSYS2_ROOT} (exit {result.returncode})")
        return False

    # First launch performs MSYS2's own post-extract setup (rebasing the
    # bundled DLLs); one no-op shell command triggers it, same as the
    # official setup-msys2 action does before ever calling pacman.
    # 初回起動で MSYS2 自身の展開後セットアップ（同梱DLLのリベース）が走る。
    # pacman を呼ぶ前に無害なシェルコマンドを1回実行して発火させる —
    # 公式の setup-msys2 action と同じ手順。
    try:
        subprocess.run([str(_BASH_EXE), "-lc", "exit 0"], timeout=120)
    except (OSError, subprocess.SubprocessError):
        pass  # best-effort; a real problem will surface in the pacman steps below

    return True


def ensure_msys2(assume_yes: bool = False, prefer_winget: bool = False) -> bool:
    """Ensure an MSYS2 root exists at MSYS2_ROOT, installing it if missing.

    By default goes straight to the direct download+extract of MSYS2's
    official base-tarball sfx archive (the canonical method here — see this
    module's docstring for why winget is not the default). Pass
    prefer_winget=True to try `winget install --id MSYS2.MSYS2` first
    instead, falling back to the direct method only if that fails; use this
    only when a caller specifically wants MSYS2 registered with winget
    (e.g. for `winget upgrade`/uninstall tracking), not as a general
    workaround. Prompts for confirmation unless assume_yes (this writes
    ~500MB under C:\\).

    MSYS2_ROOT に MSYS2 が無ければ導入する。既定では MSYS2 公式の base
    tarball sfx アーカイブの直接ダウンロード+展開（本モジュールの正規の方法 —
    winget を既定にしない理由はモジュール先頭のdocstring参照）に直行する。
    prefer_winget=True を渡すと、代わりにまず `winget install --id
    MSYS2.MSYS2` を試し、失敗した場合のみ直接方式にフォールバックする —
    winget によるアップグレード/アンインストール管理下に置きたいという
    明確な要望がある場合のみ使うオプションであり、一般的な回避策としては
    使わない。assume_yes が無ければ確認を求める（C:\\ 配下に約500MB
    書き込むため）。"""
    if _has_msys2_root():
        return True

    if not _confirm(f"MSYS2 not found at {MSYS2_ROOT}. Install it now (~500MB download)?", assume_yes):
        return False

    if prefer_winget and _try_winget_install():
        console.success("  MSYS2 installed via winget.")
        return True

    if not _direct_install_msys2():
        console.error(
            "  Automatic MSYS2 install failed. See simulator/sils/README.md's "
            "'Windows native build' section for manual install steps."
        )
        return False

    console.success(f"  MSYS2 installed directly to {MSYS2_ROOT}.")
    return True


def install_toolchain(assume_yes: bool = False, prefer_winget: bool = False) -> Optional[Path]:
    """Ensure MSYS2 plus the mingw-w64 toolchain (gcc/g++, cmake, ninja) are
    installed, installing whatever is missing via ensure_msys2() + pacman.
    Returns the mingw64/bin path on success, or None (with an explanatory
    message already printed) on failure or if the user declines. See
    ensure_msys2() for prefer_winget.
    MSYS2 と mingw-w64 ツールチェーン（gcc/g++・cmake・ninja）の導入を保証する
    （不足分は ensure_msys2() + pacman で導入）。成功時は mingw64/bin のパスを、
    失敗時・ユーザーが拒否した場合は None を返す（説明メッセージは表示済み）。
    prefer_winget は ensure_msys2() 参照。"""
    if not ensure_msys2(assume_yes, prefer_winget=prefer_winget):
        return None

    existing = mingw_bin()
    if existing is not None:
        return existing

    console.info("  Updating MSYS2 core packages (pacman -Syu)...")
    try:
        # MSYS2's first -Syu commonly updates its own runtime/bash and gets
        # killed mid-way -- a documented MSYS2 quirk, not a failure. Running
        # it twice and ignoring the first pass's exit code is the official
        # recommendation.
        # MSYS2 の最初の -Syu は自身の runtime/bash を更新して途中で強制終了
        # されることが多い -- これは MSYS2 の既知の仕様であり失敗ではない。
        # 2回実行し1回目の終了コードを無視するのが公式の推奨。
        subprocess.run([str(_BASH_EXE), "-lc", "pacman -Syu --noconfirm"], timeout=PACMAN_TIMEOUT_SECONDS)
        subprocess.run([str(_BASH_EXE), "-lc", "pacman -Syu --noconfirm"], timeout=PACMAN_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        console.error("  pacman -Syu timed out.")
        return None

    console.info("  Installing " + ", ".join(TOOLCHAIN_PACKAGES) + " ...")
    try:
        result = subprocess.run(
            [str(_BASH_EXE), "-lc", "pacman -S --noconfirm --needed " + " ".join(TOOLCHAIN_PACKAGES)],
            timeout=PACMAN_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired:
        console.error("  pacman -S (toolchain) timed out.")
        return None

    if result.returncode != 0:
        console.error(f"  pacman -S (toolchain) failed (exit {result.returncode})")
        return None

    found = mingw_bin()
    if found is None:
        console.error(f"  Toolchain install finished but g++/ninja were not found under {_MINGW64_BIN}")
    return found
