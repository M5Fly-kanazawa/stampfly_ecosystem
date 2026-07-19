#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StampFly Flasher
================

Purpose / 目的
--------------
A cross-platform (Windows / macOS / Linux) desktop GUI for flashing
pre-built StampFly firmware ("full.bin" images) to real hardware using
esptool. It exists as the all-OS fallback for the browser-based Web
Flasher, which is known to crash Chrome on macOS due to a Chromium Web
Serial implementation bug.

ビルド環境なしで StampFly 機体へファームウェア(「full.bin」イメージ)を
esptool 経由で書き込める、クロスプラットフォーム(Windows/macOS/Linux)の
デスクトップ GUI アプリ。ブラウザ版 Web Flasher の全OS向け代替として
存在する。Web Flasher は Chromium の Web Serial 実装のバグにより
macOS の Chrome をクラッシュさせることが知られている。

Usage / 使い方
--------------
    python3 stampfly_flasher.py                # Launch the GUI / GUIを起動
    python3 stampfly_flasher.py --selftest      # Headless CI check / CI向け無人チェック
    python3 stampfly_flasher.py --version       # Print version info / バージョン表示
    python3 stampfly_flasher.py --port COM3 --target vehicle   # Preset GUI fields / GUIの初期値を指定

Only two flags launch without the GUI: --version and --selftest. Every
other invocation opens the Tk window.
GUI を開かずに終了するのは --version と --selftest の2つだけで、それ以外は
Tk のウィンドウを開く。

Packaging with PyInstaller / PyInstallerでのパッケージング
-----------------------------------------------------------
CI builds a standalone, no-Python-required executable for each OS with:

    pyinstaller --onefile --windowed --name StampFlyFlasher \\
        tools/flasher_gui/stampfly_flasher.py

The CI self-test job instead omits --windowed (or overrides it with
--console) so that stdout/stderr from `--selftest` remain visible in
the build log. Because esptool is called in-process (never via
`sys.executable -m esptool`), the frozen --onefile executable can flash
hardware without a Python interpreter on the target machine and without
spawning a subprocess that would not exist inside the bundle.

CI は各OS向けに Python 不要のスタンドアロン実行ファイルを上記コマンドで
ビルドする。CI のセルフテストジョブでは `--windowed` を外す(または
`--console` で上書きする)ことで `--selftest` の標準出力/標準エラーが
ビルドログに残るようにする。esptool を(`sys.executable -m esptool` では
なく)同一プロセス内で呼び出しているため、Python 未インストールの実機でも
--onefile の実行ファイル単体でフラッシュ書き込みができ、バンドル内に
存在しないサブプロセスを起動する心配もない。
"""

import argparse
import contextlib
import hashlib
import json
import os
import queue
import re
import shutil
import ssl
import sys
import tempfile
import threading
import traceback
import urllib.request


def _create_https_context() -> ssl.SSLContext:
    """SSL context for every HTTPS call in this app, preferring certifi's
    CA bundle over the interpreter's default verify paths.

    Why: a PyInstaller-frozen Python on macOS does not use the system
    Keychain, and the bundled OpenSSL's default cert path usually does
    not exist on end-user machines — every HTTPS request then fails with
    CERTIFICATE_VERIFY_FAILED (observed on a real Mac, 2026-07-20; the
    CI runners passed only because their filesystems happen to have the
    build-time cert path). certifi ships a self-contained CA bundle that
    PyInstaller packages into the app. Falling back to the default
    context keeps the script fully functional without certifi (e.g. run
    from a venv whose Python already has working verify paths — Windows
    always does, via the OS cert store).

    なぜ必要か: PyInstaller で凍結した macOS の Python はシステムの
    キーチェーンを使わず、同梱 OpenSSL の既定証明書パスはエンドユーザーの
    マシンには通常存在しない — その結果すべての HTTPS リクエストが
    CERTIFICATE_VERIFY_FAILED で失敗する（2026-07-20 に実機 Mac で確認。
    CI ランナーはビルド時の証明書パスがたまたま存在したため通っていた）。
    certifi は自己完結の CA バンドルを持ち、PyInstaller がアプリに同梱する。
    certifi が無い場合は既定コンテキストへフォールバックし、検証パスが
    生きている Python（venv 実行や、OS の証明書ストアを使う Windows）では
    従来どおり動作する。
    """
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()


# One shared context for the process (certifi lookup done once).
# プロセスで1つ共有（certifi の解決は一度だけ）。
_HTTPS_CONTEXT = _create_https_context()

# Tkinter is part of the Python standard library; importing it does not
# require a display connection, only instantiating Tk() does. Importing
# it unconditionally at module level therefore keeps --selftest and
# --version usable inside headless CI environments (no X server / no
# window manager).
#
# tkinter は標準ライブラリの一部で、import すること自体はディスプレイ接続を
# 必要としない(必要になるのは Tk() をインスタンス化した時だけ)。そのため
# モジュールレベルで無条件に import しても、--selftest や --version は
# ディスプレイの無い(Xサーバ/ウィンドウマネージャ無し) CI 環境でも
# 問題なく動作する。
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

# esptool depends on pyserial, so "from serial.tools import list_ports"
# is expected to work whenever esptool is installed. We still guard the
# import so a missing dependency produces a friendly message instead of
# a raw traceback (requirement #9).
#
# esptool は pyserial に依存しているため、esptool がインストールされて
# いれば "from serial.tools import list_ports" は通常動作する。それでも
# 依存が欠けている場合に生のトレースバックではなく親しみやすい
# メッセージを出せるよう、import はガードしておく(要件9)。
try:
    import serial
    from serial.tools import list_ports as serial_list_ports
except ImportError as _serial_import_error:  # pragma: no cover - only hit without pyserial
    serial = None
    serial_list_ports = None
    _SERIAL_IMPORT_ERROR = _serial_import_error
else:
    _SERIAL_IMPORT_ERROR = None

try:
    import esptool
except ImportError as _esptool_import_error:  # pragma: no cover - only hit without esptool
    esptool = None
    _ESPTOOL_IMPORT_ERROR = _esptool_import_error
else:
    _ESPTOOL_IMPORT_ERROR = None


# ---------------------------------------------------------------------------
# Constants
# 定数
# ---------------------------------------------------------------------------
# No magic numbers/strings are inlined below the constants block; every
# value that appears more than once, or that a reader would need to
# look up to understand a comparison, is named here instead.
# 以下のコードにマジックナンバー/マジックストリングは埋め込まない。
# 2回以上使う値や、読み手が意味を確認したくなる値は、必ずここで
# 名前を付ける。

# Application identity, shown in the window title and by --version.
# アプリケーションの識別情報。ウィンドウタイトルと --version に表示する。
APP_NAME = "StampFly Flasher"
APP_VERSION = "0.1.0"

# GitHub repository that publishes pre-built firmware releases.
# 完成済みファームウェアのリリースを公開している GitHub リポジトリ。
GITHUB_OWNER = "M5Fly-kanazawa"
GITHUB_REPO = "stampfly_ecosystem"
GITHUB_LATEST_RELEASE_URL = "https://api.github.com/repos/{owner}/{repo}/releases/latest".format(
    owner=GITHUB_OWNER, repo=GITHUB_REPO
)

# The GitHub API rejects requests without a User-Agent header (HTTP 403),
# so we identify ourselves with the app name and version.
# GitHub API は User-Agent ヘッダの無いリクエストを拒否する(HTTP 403)ため、
# アプリ名とバージョンで自身を名乗る。
HTTP_USER_AGENT = "{name}/{version}".format(name=APP_NAME.replace(" ", ""), version=APP_VERSION)
HTTP_ACCEPT_HEADER = "application/vnd.github+json"

# Network and download tuning.
# ネットワーク・ダウンロードの調整値。
NETWORK_TIMEOUT_SECONDS = 20
DOWNLOAD_CHUNK_SIZE_BYTES = 65536  # 64 KiB per read() call / 1回のread()で読むバイト数

# The checksum manifest asset published alongside every firmware binary.
# 各ファームウェアと共に公開されるチェックサム一覧のファイル名。
SHA256SUMS_ASSET_NAME = "SHA256SUMS.txt"

# Firmware sources the user can pick between in the GUI.
# GUI 上でユーザーが選択できるファームウェアの入手元。
SOURCE_MODE_RELEASE = "release"
SOURCE_MODE_LOCAL = "local"

# Flashing targets and their bilingual, human-readable labels.
# 書き込み対象と、その表示用バイリンガルラベル。
TARGET_VEHICLE = "vehicle"
TARGET_CONTROLLER = "controller"
TARGET_LABELS = {
    TARGET_VEHICLE: "StampFly 本体 / Vehicle",
    TARGET_CONTROLLER: "Atom JoyStick / Controller",
}

# esptool / flashing configuration.
# esptool・書き込みの設定値。
ESPTOOL_CHIP_NAME = "esp32s3"
FLASH_BAUD_RATE = 460800
FLASH_START_OFFSET = "0x0"
ESPTOOL_MAJOR_VERSION_WITH_HYPHEN_COMMANDS = 5  # v5 renamed write_flash -> write-flash

# Espressif's built-in USB-JTAG/Serial VID, used to auto-select the
# StampFly's port in the serial port dropdown.
# Espressif 内蔵 USB-JTAG/シリアルの VID。シリアルポートのドロップダウンで
# StampFly のポートを自動選択するために使う。
ESPRESSIF_USB_JTAG_VID = 0x303A

# GUI layout constants.
# GUI レイアウトの定数。
WINDOW_WIDTH_PX = 640
WINDOW_HEIGHT_PX = 560
WINDOW_MIN_HEIGHT_PX = 480
QUEUE_POLL_INTERVAL_MS = 50
LOG_TEXT_HEIGHT_ROWS = 12
LOG_TEXT_FONT = "TkFixedFont"  # Tk's built-in cross-platform monospace font / Tk標準のクロスプラットフォーム等幅フォント
HINT_TEXT_WRAP_PX = 560
WARN_TEXT_COLOR = "#a05a00"
NOTE_TEXT_COLOR = "#555555"

# Regexes used to parse esptool output and the release manifest.
#
# esptool v4 prints progress in parentheses, e.g. "(42%)", while esptool
# v5's write-flash prints a bare float percentage with no parentheses,
# e.g. "Writing at 0x00010000 [====>   ]  42.0% 4096/9600 bytes...".
# This app's own download-progress log lines also use the v4-style
# "(NN%)" form. The pattern below matches the bare "NN" / "NN.N" run
# in either style; parse_progress_percent() below picks out the value.
#
# esptool の出力とリリースのチェックサム一覧を解析する正規表現。
#
# esptool v4 は進捗を括弧付きで "(42%)" のように表示するが、v5 の
# write-flash は括弧無しの小数パーセントをそのまま
# "Writing at 0x00010000 [====>   ]  42.0% 4096/9600 bytes..." のように
# 表示する。このアプリ自身のダウンロード進捗ログも v4 形式の "(NN%)" を
# 使っている。以下のパターンはどちらの形式でも数値部分("NN"/"NN.N")に
# マッチする。値の取り出しは下の parse_progress_percent() で行う。
PROGRESS_PERCENT_PATTERN = re.compile(r"(\d+(?:\.\d+)?)\s*%")
SHA256SUMS_LINE_PATTERN = re.compile(r"^([0-9a-fA-F]{64})\s+\*?(\S.*)$")

# Friendly install hint shown whenever esptool cannot be imported.
# esptool が import できない場合に表示する、親しみやすいインストール案内。
ESPTOOL_INSTALL_HINT = (
    "esptool が見つかりません。次のコマンドでインストールしてください:\n"
    "  pip install esptool\n\n"
    "esptool was not found. Install it with:\n"
    "  pip install esptool"
)


def asset_name_pattern_for_target(target):
    """
    Build the regex that matches a release asset's filename for the
    given flashing target (e.g. "stampfly_vehicle_v2026.07.0_full.bin").
    指定した書き込み対象(target)に対応するリリースアセットのファイル名に
    マッチする正規表現を作る(例: "stampfly_vehicle_v2026.07.0_full.bin")。
    """
    return re.compile(r"^stampfly_" + re.escape(target) + r"_v.*_full\.bin$")


# ---------------------------------------------------------------------------
# Errors
# エラー
# ---------------------------------------------------------------------------


class FlashError(Exception):
    """
    Raised for any failure in the download/verify/flash pipeline. The
    message is already translated into a human-friendly, bilingual
    string, ready to show directly in a messagebox or the log.
    ダウンロード/検証/書き込みの一連の処理で失敗した際に送出する。
    メッセージは既に人間向けのバイリンガルな文字列へ変換済みで、
    そのままメッセージボックスやログに表示できる。
    """


# ---------------------------------------------------------------------------
# GitHub release helpers (pure functions, no Tk, no esptool)
# GitHub リリース関連のヘルパー(Tk/esptool に依存しない純粋関数)
# ---------------------------------------------------------------------------


def fetch_latest_release_info():
    """
    Fetch the latest release metadata (JSON) from the GitHub API.

    This performs blocking network I/O and must only ever be called
    from a worker thread, never from the Tk main thread (doing so would
    freeze the whole GUI while waiting for the network).

    GitHub API から最新リリースのメタデータ(JSON)を取得する。

    ブロッキングするネットワークI/Oを行うため、必ずワーカースレッドから
    呼び出すこと。Tk のメインスレッドから呼ぶと、ネットワーク応答待ちの
    間 GUI 全体が固まってしまう。
    """
    headers = {"User-Agent": HTTP_USER_AGENT, "Accept": HTTP_ACCEPT_HEADER}
    # Use an authenticated request when a CI token is available. GitHub-hosted
    # runners share egress IPs across many jobs, so ANONYMOUS API calls from CI
    # are frequently rate-limited (HTTP 403); authenticated calls are not.
    # End users normally have no token set and stay anonymous as before.
    # CI トークンがあれば認証付きリクエストにする。GitHub ホステッドランナーは
    # 多数のジョブで送信元IPを共有するため、CI からの匿名 API 呼び出しは
    # レート制限(HTTP 403)になりやすい。認証付きなら制限に掛からない。
    # 通常利用のエンドユーザーはトークン未設定のため従来どおり匿名で動く。
    api_token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if api_token:
        headers["Authorization"] = "Bearer " + api_token
    request = urllib.request.Request(
        GITHUB_LATEST_RELEASE_URL,
        headers=headers,
    )
    with urllib.request.urlopen(request, timeout=NETWORK_TIMEOUT_SECONDS, context=_HTTPS_CONTEXT) as response:
        return json.loads(response.read().decode("utf-8"))


def find_release_asset(release_info, name_pattern):
    """
    Return the first release asset whose filename matches name_pattern,
    or None if no asset matches.
    name_pattern にファイル名がマッチする最初のリリースアセットを返す。
    マッチするアセットが無ければ None を返す。
    """
    for asset in release_info.get("assets", []):
        if name_pattern.match(asset.get("name", "")):
            return asset
    return None


def find_sha256sums_asset(release_info):
    """
    Return the SHA256SUMS.txt asset from the release, or None if absent.
    リリース内の SHA256SUMS.txt アセットを返す。無ければ None を返す。
    """
    for asset in release_info.get("assets", []):
        if asset.get("name") == SHA256SUMS_ASSET_NAME:
            return asset
    return None


def parse_sha256sums(text):
    """
    Parse the contents of a SHA256SUMS.txt file (the standard
    "sha256sum" command output format: "<hex digest>  <filename>",
    optionally with a "*" before the filename for binary mode) into a
    dict mapping filename -> lowercase hex digest.

    SHA256SUMS.txt の内容(標準的な "sha256sum" コマンドの出力形式:
    "<16進ダイジェスト>  <ファイル名>"。バイナリモードではファイル名の前に
    "*" が付くこともある)を、ファイル名 -> 小文字16進ダイジェストの
    辞書に変換する。
    """
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


def compute_sha256(path):
    """
    Compute the SHA256 hex digest of a file, reading it in fixed-size
    chunks so memory usage stays flat regardless of file size.
    ファイルの SHA256 16進ダイジェストを計算する。ファイルサイズに
    関わらずメモリ使用量が一定になるよう、固定サイズのチャンク単位で読む。
    """
    hasher = hashlib.sha256()
    with open(path, "rb") as file_handle:
        while True:
            chunk = file_handle.read(DOWNLOAD_CHUNK_SIZE_BYTES)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


def download_to_file(url, destination_path, progress_callback=None):
    """
    Download url to destination_path, streaming in fixed-size chunks.

    If progress_callback is given, it is called after every chunk as
    progress_callback(bytes_downloaded_so_far, total_bytes_or_none) so
    the caller can report progress without loading the whole response
    into memory first.

    url を destination_path へ、固定サイズのチャンクでストリーミング
    ダウンロードする。

    progress_callback を渡した場合、各チャンクの後に
    progress_callback(これまでにダウンロード済みのバイト数, 総バイト数
    またはNone) を呼ぶ。応答全体を先にメモリへ読み込まずに進捗を
    報告できる。
    """
    request = urllib.request.Request(url, headers={"User-Agent": HTTP_USER_AGENT})
    with urllib.request.urlopen(request, timeout=NETWORK_TIMEOUT_SECONDS, context=_HTTPS_CONTEXT) as response:
        content_length = response.headers.get("Content-Length")
        total_bytes = int(content_length) if content_length else None
        downloaded_bytes = 0
        with open(destination_path, "wb") as out_file:
            while True:
                chunk = response.read(DOWNLOAD_CHUNK_SIZE_BYTES)
                if not chunk:
                    break
                out_file.write(chunk)
                downloaded_bytes += len(chunk)
                if progress_callback is not None:
                    progress_callback(downloaded_bytes, total_bytes)


# ---------------------------------------------------------------------------
# Serial port helpers
# シリアルポート関連ヘルパー
# ---------------------------------------------------------------------------


def list_serial_ports():
    """
    Return the list of currently connected serial ports (pyserial
    ListPortInfo objects), or an empty list if pyserial is unavailable.
    現在接続されているシリアルポートの一覧(pyserial の ListPortInfo
    オブジェクト)を返す。pyserial が使えない場合は空リストを返す。
    """
    if serial_list_ports is None:
        return []
    return list(serial_list_ports.comports())


def format_port_label(port_info):
    """
    Format a port for display as "device — description", matching the
    style requested for the combobox entries.
    ポートを "device — description" の形式で表示用に整形する。
    コンボボックスの表示形式として指定された書式。
    """
    return "{device} — {description}".format(
        device=port_info.device, description=port_info.description
    )


def guess_espressif_port(ports):
    """
    Return the device path of the first port whose USB VID matches
    Espressif's built-in USB-JTAG/Serial controller, or None if none
    match. Used to auto-select the StampFly's port on startup/refresh.
    USB VID が Espressif 内蔵 USB-JTAG/シリアルコントローラに一致する
    最初のポートのデバイスパスを返す。一致するものが無ければ None。
    起動時/更新時に StampFly のポートを自動選択するために使う。
    """
    for port in ports:
        if getattr(port, "vid", None) == ESPRESSIF_USB_JTAG_VID:
            return port.device
    return None


# ---------------------------------------------------------------------------
# esptool invocation (in-process — safe inside a PyInstaller --onefile bundle)
# esptool 呼び出し(同一プロセス内 — PyInstaller --onefile バンドル内でも安全)
# ---------------------------------------------------------------------------


def resolve_write_flash_subcommand():
    """
    esptool renamed its CLI subcommands from underscore_case to
    hyphen-case starting in v5 (e.g. "write_flash" -> "write-flash").
    Pick the spelling that matches the installed esptool version.

    esptool は v5 から CLI サブコマンド名をアンダースコア区切りから
    ハイフン区切りへ改名した(例: "write_flash" -> "write-flash")。
    インストールされている esptool のバージョンに合う表記を選ぶ。
    """
    major_version = int(esptool.__version__.split(".")[0])
    if major_version >= ESPTOOL_MAJOR_VERSION_WITH_HYPHEN_COMMANDS:
        return "write-flash"
    return "write_flash"


def build_esptool_argv(port, bin_path, erase):
    """
    Build the esptool argv list (WITHOUT a leading program name — that
    is only needed for sys.argv, not for esptool.main()) for writing
    bin_path to flash offset 0x0 on the given serial port.
    与えられたシリアルポート上のフラッシュオフセット 0x0 に bin_path を
    書き込むための esptool の argv リストを組み立てる(先頭のプログラム名
    無し版。プログラム名は sys.argv 用であって esptool.main() には不要)。
    """
    argv = [
        "--chip", ESPTOOL_CHIP_NAME,
        "--port", port,
        "--baud", str(FLASH_BAUD_RATE),
        resolve_write_flash_subcommand(),
    ]
    if erase:
        argv.append("--erase-all")
    argv += [FLASH_START_OFFSET, bin_path]
    return argv


def parse_progress_percent(line):
    """
    Extract a percentage from a line containing esptool's flashing
    progress (either esptool v4's "(42%)" or v5's bare "42.0%"), or
    this app's own "Downloading... (37%)" log lines. If more than one
    percentage appears on the line, the LAST one is used (esptool
    sometimes prints an intermediate ratio before the running total).
    The result is rounded down to an int and clamped to 0-100 in case
    of float rounding at the boundaries. Returns None if the line has
    no percentage at all.
    esptool のフラッシュ進捗(v4 の "(42%)" または v5 の括弧無し
    "42.0%")、またはこのアプリ自身の "Downloading... (37%)" ログ行から
    パーセント値を取り出す。1行に複数のパーセント表記があれば最後の
    ものを使う(esptool は合計値の前に途中の比率を出すことがあるため)。
    結果は int に切り捨て、境界での浮動小数点誤差に備えて 0-100 の
    範囲へクランプする。パーセント表記が全く無ければ None を返す。
    """
    matches = PROGRESS_PERCENT_PATTERN.findall(line)
    if not matches:
        return None
    percent = int(float(matches[-1]))
    return max(0, min(100, percent))


class QueueLineWriter:
    """
    A minimal file-like object used with contextlib.redirect_stdout /
    redirect_stderr to capture esptool's console output into a
    thread-safe queue, one complete line at a time.

    esptool prints its flashing progress using a carriage return ("\\r")
    rather than a newline, so the terminal overwrites the previous line
    in place. We treat both "\\n" and "\\r" as line terminators so no
    progress update is lost.

    contextlib.redirect_stdout / redirect_stderr と組み合わせて使う、
    esptool のコンソール出力をスレッドセーフなキューへ1行ずつ捕捉する
    最小限のファイルライクオブジェクト。

    esptool はフラッシュ進捗の表示に、ターミナル上で前の行を上書きする
    ための復帰("\\r")を使い、改行("\\n")は使わない。そのため "\\n" と
    "\\r" の両方を行末記号として扱い、進捗表示を取りこぼさないようにする。
    """

    def __init__(self, output_queue):
        self._queue = output_queue
        self._buffer = ""

    def write(self, text):
        """Buffer incoming text and push out any complete lines it forms."""
        # 受け取ったテキストをバッファへ追加し、完成した行があれば送り出す
        self._buffer += text
        while True:
            newline_pos = self._buffer.find("\n")
            carriage_pos = self._buffer.find("\r")
            candidate_positions = [pos for pos in (newline_pos, carriage_pos) if pos != -1]
            if not candidate_positions:
                break
            split_at = min(candidate_positions)
            line, self._buffer = self._buffer[:split_at], self._buffer[split_at + 1:]
            if line:
                self._queue.put(("log", line))
        return len(text)

    def flush(self):
        """Required by the file-like protocol; buffering is handled in write()."""
        # ファイルライクプロトコルの要件として必要。バッファ処理は write() 側で行う
        pass

    def flush_remaining(self):
        """
        Push any trailing partial line once esptool has finished
        running (it may not end its final message with \\n or \\r).
        esptool の実行終了後に残っている末尾の未完成行をキューへ積む
        (最後のメッセージが \\n や \\r で終わっているとは限らないため)。
        """
        if self._buffer:
            self._queue.put(("log", self._buffer))
            self._buffer = ""


def _resolve_fatal_error_types():
    """
    Resolve esptool's FatalError exception class, tolerating the exact
    module path changing between esptool versions. Falls back to the
    generic Exception if esptool is unavailable or its layout changed.
    esptool の FatalError 例外クラスを取得する。esptool のバージョンに
    よって正確なモジュールパスが変わっても許容する。esptool が無い、
    またはモジュール構成が変わっている場合は汎用の Exception にフォールバックする。
    """
    if esptool is None:
        return (Exception,)
    try:
        return (esptool.util.FatalError,)
    except AttributeError:
        pass
    fatal_error_cls = getattr(esptool, "FatalError", None)
    if fatal_error_cls is not None:
        return (fatal_error_cls,)
    return (Exception,)


_FATAL_ERROR_TYPES = _resolve_fatal_error_types()
_SERIAL_EXCEPTION_TYPES = (serial.SerialException,) if serial is not None else (Exception,)

# Substrings (matched case-insensitively) used to recognize common
# esptool/pyserial failure modes and translate them into friendly
# bilingual guidance instead of a raw exception message.
# esptool/pyserial のよくある失敗パターンを見分けるための部分文字列
# (大文字小文字を無視して照合)。生の例外メッセージの代わりに親しみやすい
# バイリンガルな案内へ変換するために使う。
_PORT_BUSY_MARKERS = ("could not open port", "permission denied", "resource busy", "access is denied")
_CONNECT_FAILURE_MARKERS = (
    "failed to connect",
    "no serial data received",
    "invalid head of packet",
    "timed out waiting for packet header",
)


def translate_flash_exception(exc):
    """
    Convert a raw exception from esptool/pyserial into a friendly,
    bilingual message. Falls back to the original exception text when
    no known failure pattern matches.
    esptool/pyserial から送出された生の例外を、既知の失敗パターンに
    基づいて親しみやすいバイリンガルメッセージへ変換する。既知の
    パターンに一致しなければ元の例外テキストをそのまま使う。
    """
    original_text = str(exc)
    lowered_text = original_text.lower()

    if any(marker in lowered_text for marker in _PORT_BUSY_MARKERS):
        return (
            "他のアプリがポートを使用中です。sf monitor 等ポートを使用しているアプリを"
            "閉じてから再試行してください。 / Another app is holding the port. Close it "
            "(e.g. sf monitor) and retry.\n(" + original_text + ")"
        )

    if any(marker in lowered_text for marker in _CONNECT_FAILURE_MARKERS):
        return (
            "デバイスとの接続に失敗しました。ケーブルとポートを確認し、必要であれば "
            "本体の BOOT ボタンを押しながら再試行してください。 / Failed to connect to "
            "the device. Check the cable and port, and retry while holding the BOOT "
            "button if necessary.\n(" + original_text + ")"
        )

    return original_text


def run_esptool_flash(argv, output_queue):
    """
    Run esptool in-process (never via a subprocess, so this keeps
    working when frozen into a PyInstaller --onefile executable) with
    its stdout/stderr redirected line-by-line into output_queue.

    Raises FlashError with an already-translated message on failure.
    Returns normally on success.

    esptool をサブプロセスとしてではなく同一プロセス内で実行する
    (これにより PyInstaller --onefile 実行ファイルに固めても動作し続ける)。
    標準出力/標準エラーは1行ずつ output_queue へリダイレクトする。

    失敗時は変換済みメッセージを持つ FlashError を送出する。
    成功時は普通に return する。
    """
    if esptool is None:
        raise FlashError(ESPTOOL_INSTALL_HINT)

    writer = QueueLineWriter(output_queue)
    try:
        with contextlib.redirect_stdout(writer), contextlib.redirect_stderr(writer):
            esptool.main(argv)
    except SystemExit as exc:
        # esptool.main() usually re-raises SystemExit only when the
        # underlying CLI exited with a non-zero code, swallowing a
        # clean exit(0) internally. We do not rely on that being true
        # for every esptool version/code path: only a genuine integer
        # exit code of 0 counts as success. SystemExit(None) (e.g. a
        # bare `return` reaching argparse's exit machinery) and
        # non-zero/non-int codes (e.g. 2 from an argparse/click usage
        # error) are all treated as failures with a readable bilingual
        # message that includes the raw code, instead of surfacing a
        # bare "2" or being silently swallowed.
        # esptool.main() は通常、内部の CLI がゼロ以外の終了コードで
        # 終了した場合のみ SystemExit を再送出し、正常終了(exit(0))は
        # 内部で握りつぶす。ただし全ての esptool バージョン/コード経路で
        # それが成り立つとは限らないため、ここでは「真に整数の終了コード
        # 0」であるときのみ成功とみなす。SystemExit(None)(例えば
        # argparse の終了処理に素の `return` で辿り着いた場合)や、
        # 非ゼロ/非int のコード(argparse/click の使用方法エラーによる
        # 2 など)は全て失敗として扱い、生の "2" だけを表示するのではなく
        # コードを含む読みやすいバイリンガルメッセージにする。
        if isinstance(exc.code, int) and exc.code == 0:
            pass
        else:
            raise FlashError(
                "esptool が予期せず終了しました (code: {0}) / "
                "esptool exited unexpectedly (code: {0})".format(exc.code)
            )
    except _FATAL_ERROR_TYPES as exc:
        raise FlashError(translate_flash_exception(exc))
    except _SERIAL_EXCEPTION_TYPES as exc:
        raise FlashError(translate_flash_exception(exc))
    finally:
        writer.flush_remaining()


# ---------------------------------------------------------------------------
# Flashing workflow (runs entirely on a worker thread)
# 書き込みワークフロー(全てワーカースレッド上で実行する)
# ---------------------------------------------------------------------------


def _resolve_release_binary(output_queue, target, cached_release_info):
    """
    Resolve, download, and SHA256-verify the firmware binary for
    target from the latest (or cached) GitHub release.

    Returns (bin_path, resolved_tag_name, work_dir). This function
    only downloads and verifies; it does NOT remove work_dir. The
    caller owns work_dir and is responsible for deleting it (e.g. with
    shutil.rmtree) once bin_path is no longer needed — typically after
    esptool has finished flashing it, not before. Raises FlashError on
    any failure (missing asset, download error, checksum mismatch); in
    that case work_dir may still contain partial downloads, so the
    caller must clean it up in a finally block regardless of outcome.

    target 向けのファームウェアバイナリを、最新(またはキャッシュ済み)の
    GitHub リリースから解決・ダウンロードし、SHA256 で検証する。

    (bin_path, 解決したタグ名, 作業ディレクトリ work_dir) を返す。この
    関数はダウンロードと検証のみを行い、work_dir の削除は行わない。
    work_dir の所有者は呼び出し側であり、bin_path が不要になった時点
    (通常は esptool による書き込みが終わった後、終わる前ではない)で
    shutil.rmtree 等により削除する責任を持つ。失敗時(アセット無し、
    ダウンロードエラー、チェックサム不一致)は FlashError を送出するが、
    その場合も work_dir にダウンロード途中のファイルが残り得るため、
    呼び出し側は結果に関わらず finally ブロックで後始末すること。
    """
    output_queue.put(("status", "GitHub リリース情報を取得中... / Fetching GitHub release info..."))
    release_info = cached_release_info if cached_release_info is not None else fetch_latest_release_info()
    resolved_tag = release_info.get("tag_name", "unknown")
    output_queue.put(("log", "Resolved release: {0}".format(resolved_tag)))

    firmware_asset = find_release_asset(release_info, asset_name_pattern_for_target(target))
    sums_asset = find_sha256sums_asset(release_info)
    if firmware_asset is None:
        raise FlashError(
            "{0} 用のファームウェア資産が見つかりません / "
            "No firmware asset found for target '{0}' in release {1}".format(target, resolved_tag)
        )
    if sums_asset is None:
        raise FlashError(
            "SHA256SUMS.txt が見つかりません / SHA256SUMS.txt not found in release {0}".format(resolved_tag)
        )

    work_dir = tempfile.mkdtemp(prefix="stampfly_flasher_")
    bin_path = os.path.join(work_dir, firmware_asset["name"])
    sums_path = os.path.join(work_dir, sums_asset["name"])

    output_queue.put(("status", "ファームウェアをダウンロード中... / Downloading firmware..."))

    def report_download_progress(downloaded_bytes, total_bytes):
        if total_bytes:
            percent = int(downloaded_bytes * 100 / total_bytes)
            output_queue.put(("log", "Downloading firmware... ({0}%)".format(percent)))

    download_to_file(firmware_asset["browser_download_url"], bin_path, report_download_progress)
    download_to_file(sums_asset["browser_download_url"], sums_path)

    output_queue.put(("status", "SHA256 を検証中... / Verifying SHA256..."))
    with open(sums_path, "r", encoding="utf-8") as sums_file:
        checksums = parse_sha256sums(sums_file.read())

    expected_digest = checksums.get(firmware_asset["name"])
    if expected_digest is None:
        raise FlashError(
            "SHA256SUMS.txt に {0} のハッシュ値がありません / "
            "No checksum entry for {0} in SHA256SUMS.txt".format(firmware_asset["name"])
        )

    actual_digest = compute_sha256(bin_path)
    if actual_digest.lower() != expected_digest.lower():
        raise FlashError(
            "SHA256 チェックサムが一致しません(ダウンロード破損の可能性があります) / "
            "SHA256 checksum mismatch (download may be corrupted): "
            "expected {0}, got {1}".format(expected_digest, actual_digest)
        )
    output_queue.put(("log", "SHA256 OK: {0}".format(actual_digest)))

    return bin_path, resolved_tag, work_dir


def perform_flash_workflow(output_queue, source_mode, local_path, target, port, erase, cached_release_info):
    """
    Run the full flash workflow on a worker thread: resolve the
    firmware binary (download + verify for release mode, or use the
    local file as-is), then invoke esptool to write it.

    Returns the resolved release tag name, or None in local-file mode.
    Raises FlashError on any failure; the caller (GUI worker thread) is
    responsible for catching it and reporting to the queue.

    Release-mode downloads land in a temporary directory (see
    _resolve_release_binary); it is removed here in a finally block so
    it happens exactly once, after esptool has finished with the file
    (success or failure) rather than before.

    ワーカースレッド上でフラッシュ書き込みの全工程を実行する:
    ファームウェアバイナリを解決し(リリースモードならダウンロード+検証、
    ローカルモードならそのまま使用)、esptool で書き込む。

    解決したリリースのタグ名を返す(ローカルファイルモードでは None)。
    失敗時は FlashError を送出する。呼び出し側(GUIのワーカースレッド)が
    それを捕捉してキューへ報告する責任を持つ。

    リリースモードのダウンロード先は一時ディレクトリになる
    (_resolve_release_binary 参照)。ここで finally ブロックにより
    ちょうど1回だけ削除する。esptool がそのファイルを使い終わった後
    (成功・失敗を問わず)に削除するのであって、使い終わる前ではない。
    """
    resolved_tag = None
    work_dir = None
    try:
        if source_mode == SOURCE_MODE_RELEASE:
            bin_path, resolved_tag, work_dir = _resolve_release_binary(
                output_queue, target, cached_release_info
            )
        else:
            bin_path = local_path
            if not bin_path or not os.path.isfile(bin_path):
                raise FlashError("ローカルファイルが選択されていません / No local firmware file selected")
            output_queue.put(("log", "Using local file: {0}".format(bin_path)))

        output_queue.put(("status", "esptool で書き込み中... / Flashing with esptool..."))
        argv = build_esptool_argv(port, bin_path, erase)
        output_queue.put(("log", "esptool argv: {0}".format(" ".join(argv))))
        run_esptool_flash(argv, output_queue)

        return resolved_tag
    finally:
        # Remove the downloaded work dir (if any) now that esptool is
        # done with it, whether flashing succeeded or raised.
        # esptool がファイルを使い終わった今、ダウンロード先の作業
        # ディレクトリ(あれば)を削除する。書き込みの成功/失敗は問わない。
        if work_dir is not None:
            shutil.rmtree(work_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# Self-test (headless, CI-friendly, no Tk, no hardware writes)
# セルフテスト(ヘッドレス・CI向け・Tk不使用・実機書き込みなし)
# ---------------------------------------------------------------------------


def run_selftest():
    """
    Headless, CI-friendly self-check: resolve the latest release,
    download and SHA256-verify the vehicle firmware, and enumerate
    serial ports. Never instantiates Tk and never writes to any device.

    Returns 0 for PASS, 1 for FAIL, matching standard process
    exit-code conventions so CI can gate on it directly.

    ヘッドレスでCI向けのセルフチェック: 最新リリースを解決し、vehicle
    ファームウェアをダウンロードして SHA256 を検証し、シリアルポートを
    列挙する。Tk のインスタンス化は一切行わず、デバイスへの書き込みも
    行わない。

    PASS なら 0、FAIL なら 1 を返す。プロセス終了コードの標準的な慣習に
    合わせているため、CI が直接ゲート条件として使える。
    """
    print("=== StampFly Flasher self-test / セルフテスト ===")

    if esptool is not None:
        print("esptool version: {0}".format(esptool.__version__))
    else:
        print("esptool: not installed (--selftest does not require it for the network/hash checks)")

    try:
        print("Fetching latest release info from GitHub...")
        release_info = fetch_latest_release_info()
    except Exception as exc:
        print("FAIL: could not fetch latest release info: {0}".format(exc))
        return 1

    resolved_tag = release_info.get("tag_name", "unknown")
    print("Resolved release tag: {0}".format(resolved_tag))

    firmware_asset = find_release_asset(release_info, asset_name_pattern_for_target(TARGET_VEHICLE))
    sums_asset = find_sha256sums_asset(release_info)
    if firmware_asset is None:
        print("FAIL: no vehicle firmware asset found in release {0}".format(resolved_tag))
        return 1
    if sums_asset is None:
        print("FAIL: no {0} asset found in release {1}".format(SHA256SUMS_ASSET_NAME, resolved_tag))
        return 1
    print("Firmware asset: {0}".format(firmware_asset["name"]))

    work_dir = tempfile.mkdtemp(prefix="stampfly_flasher_selftest_")
    bin_path = os.path.join(work_dir, firmware_asset["name"])
    sums_path = os.path.join(work_dir, sums_asset["name"])

    # Everything below downloads into work_dir; the finally block below
    # removes it exactly once, on every return path (PASS or FAIL).
    # 以下は work_dir へダウンロードする処理。下の finally ブロックが
    # (PASS・FAIL どちらの return 経路でも)ちょうど1回だけ削除する。
    try:
        try:
            print("Downloading firmware binary...")
            download_to_file(firmware_asset["browser_download_url"], bin_path)
            print("Downloading {0}...".format(SHA256SUMS_ASSET_NAME))
            download_to_file(sums_asset["browser_download_url"], sums_path)
        except Exception as exc:
            print("FAIL: download error: {0}".format(exc))
            return 1

        with open(sums_path, "r", encoding="utf-8") as sums_file:
            checksums = parse_sha256sums(sums_file.read())

        expected_digest = checksums.get(firmware_asset["name"])
        if expected_digest is None:
            print("FAIL: no checksum entry for {0} in {1}".format(firmware_asset["name"], SHA256SUMS_ASSET_NAME))
            return 1

        actual_digest = compute_sha256(bin_path)
        if actual_digest.lower() != expected_digest.lower():
            print("FAIL: SHA256 mismatch: expected {0}, got {1}".format(expected_digest, actual_digest))
            return 1
        print("PASS: SHA256 verified ({0})".format(actual_digest))

        print("Scanning serial ports...")
        ports = list_serial_ports()
        espressif_ports = [p for p in ports if getattr(p, "vid", None) == ESPRESSIF_USB_JTAG_VID]
        if espressif_ports:
            for port in espressif_ports:
                print("Found Espressif device: {0}".format(format_port_label(port)))
        else:
            # Not finding a device is expected in most CI environments and
            # is explicitly NOT treated as a failure.
            # ほとんどの CI 環境ではデバイスが見つからないのが通常であり、
            # これは明示的に失敗として扱わない。
            print(
                "No Espressif (VID 0x{0:04X}) serial port found (this is not a failure).".format(
                    ESPRESSIF_USB_JTAG_VID
                )
            )

        print("=== PASS ===")
        return 0
    finally:
        shutil.rmtree(work_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# GUI
# GUI
# ---------------------------------------------------------------------------


class StampFlyFlasherApp:
    """
    The Tk application. All widget access happens on the Tk main
    thread. Background work (network requests, esptool) always runs on
    a single worker thread and reports back exclusively by pushing
    messages onto self._queue; _poll_queue(), driven by root.after(),
    is the only place worker results are read on the main thread.

    Tk アプリケーション本体。ウィジェットへのアクセスは全て Tk の
    メインスレッドで行う。バックグラウンド作業(ネットワーク要求・
    esptool)は常に単一のワーカースレッドで実行し、結果は
    self._queue へメッセージを積むことでのみ報告する。
    root.after() で駆動される _poll_queue() だけが、メインスレッド側で
    ワーカーの結果を読み出す場所となる。
    """

    def __init__(self, root, preset_port=None, preset_target=None):
        self.root = root
        self.root.title(APP_NAME)
        self.root.geometry("{0}x{1}".format(WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX))
        self.root.minsize(WINDOW_WIDTH_PX, WINDOW_MIN_HEIGHT_PX)

        # Thread-safe channel from worker threads to the main thread.
        # ワーカースレッドからメインスレッドへのスレッドセーフな通信路。
        self._queue = queue.Queue()
        self._busy = False
        # The JSON dict from fetch_latest_release_info(), or None until
        # the first "Check latest" / Flash in release mode.
        # fetch_latest_release_info() が返す JSON 辞書。初回の
        # 「最新版を確認」またはリリースモードでの書き込みまでは None。
        self._cached_release_info = None
        # List[str]: combobox index -> serial device path.
        # コンボボックスのインデックス→シリアルデバイスパスの対応表。
        self._port_devices = []

        self._source_mode_var = tk.StringVar(value=SOURCE_MODE_RELEASE)
        self._local_path_var = tk.StringVar(value="")
        self._release_tag_var = tk.StringVar(value="(未確認 / not checked yet)")
        self._target_var = tk.StringVar(value=preset_target or TARGET_VEHICLE)
        self._erase_var = tk.BooleanVar(value=False)
        self._status_var = tk.StringVar(value="待機中 / Idle")

        self._build_widgets()
        self._refresh_ports(preset_port)
        self.root.after(QUEUE_POLL_INTERVAL_MS, self._poll_queue)

    # -- widget construction -------------------------------------------------

    def _build_widgets(self):
        """Lay out every section of the window, top to bottom."""
        # ウィンドウの各セクションを上から順に配置する
        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        self._build_source_section(outer, row=0)
        self._build_target_section(outer, row=1)
        self._build_port_section(outer, row=2)
        self._build_erase_section(outer, row=3)
        self._build_action_section(outer, row=4)
        self._build_log_section(outer, row=5)

        # Only the log area grows when the window is resized.
        # ウィンドウをリサイズしたとき伸びるのはログ欄だけ
        outer.rowconfigure(5, weight=1)

    def _build_source_section(self, parent, row):
        frame = ttk.Labelframe(parent, text="ファーム入手元 / Firmware source", padding=8)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(2, weight=1)

        ttk.Radiobutton(
            frame,
            text="最新の GitHub Release / Latest GitHub Release",
            variable=self._source_mode_var,
            value=SOURCE_MODE_RELEASE,
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Button(
            frame, text="最新版を確認 / Check latest", command=self._on_check_latest_release
        ).grid(row=1, column=1, sticky="w", padx=(20, 8))
        ttk.Label(frame, textvariable=self._release_tag_var).grid(row=1, column=2, sticky="w")

        ttk.Radiobutton(
            frame,
            text="ローカルファイル / Local file",
            variable=self._source_mode_var,
            value=SOURCE_MODE_LOCAL,
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(6, 0))

        ttk.Button(frame, text="参照 / Browse", command=self._on_browse_local_file).grid(
            row=3, column=0, sticky="w", padx=(20, 0)
        )
        ttk.Entry(frame, textvariable=self._local_path_var, state="readonly").grid(
            row=3, column=1, columnspan=2, sticky="ew", padx=(8, 0)
        )

    def _build_target_section(self, parent, row):
        frame = ttk.Labelframe(parent, text="対象 / Target", padding=8)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        ttk.Radiobutton(
            frame, text=TARGET_LABELS[TARGET_VEHICLE], variable=self._target_var, value=TARGET_VEHICLE
        ).grid(row=0, column=0, sticky="w", padx=(0, 20))
        ttk.Radiobutton(
            frame, text=TARGET_LABELS[TARGET_CONTROLLER], variable=self._target_var, value=TARGET_CONTROLLER
        ).grid(row=0, column=1, sticky="w")

    def _build_port_section(self, parent, row):
        frame = ttk.Labelframe(parent, text="シリアルポート / Serial port", padding=8)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(0, weight=1)

        self.port_combo = ttk.Combobox(frame, state="readonly")
        self.port_combo.grid(row=0, column=0, sticky="ew")
        ttk.Button(frame, text="更新 / Refresh", command=lambda: self._refresh_ports(None)).grid(
            row=0, column=1, padx=(8, 0)
        )

        # Hidden by default; _refresh_ports() reveals it when no ports
        # are found, per requirement #3's friendly-hint behaviour.
        # デフォルトでは非表示。要件3の「親切なヒント」に従い、ポートが
        # 見つからないときだけ _refresh_ports() が表示する。
        self.port_hint_label = ttk.Label(
            frame,
            text=(
                "ポートが見つかりません。データ通信対応のUSB-Cケーブルを使い、"
                "sf monitor 等ポートを使用中のアプリを閉じてください。 /\n"
                "No port found. Use a data-capable USB-C cable and close any "
                "app holding the port (e.g. sf monitor)."
            ),
            foreground=WARN_TEXT_COLOR,
            wraplength=HINT_TEXT_WRAP_PX,
            justify="left",
        )
        self.port_hint_label.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self.port_hint_label.grid_remove()

    def _build_erase_section(self, parent, row):
        frame = ttk.Labelframe(parent, text="消去オプション / Erase option", padding=8)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        ttk.Checkbutton(
            frame, text="全消去 (NVS含む) / Erase all (includes NVS)", variable=self._erase_var
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            frame,
            text=(
                "NVS には校正値・パラメータが保存されています。初回書き込みは ON、"
                "更新時は OFF を推奨します。 /\n"
                "NVS stores calibration values and parameters. ON for a first "
                "install, OFF recommended for updates."
            ),
            foreground=NOTE_TEXT_COLOR,
            wraplength=HINT_TEXT_WRAP_PX,
            justify="left",
        ).grid(row=1, column=0, sticky="w")

    def _build_action_section(self, parent, row):
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        self.flash_button = ttk.Button(frame, text="書き込み / Flash", command=self._on_flash_clicked)
        self.flash_button.grid(row=0, column=0, sticky="w")

        ttk.Label(frame, textvariable=self._status_var).grid(row=0, column=1, sticky="w", padx=(12, 0))

        self.progress_bar = ttk.Progressbar(frame, orient="horizontal", mode="determinate", maximum=100)
        self.progress_bar.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))

    def _build_log_section(self, parent, row):
        frame = ttk.Labelframe(parent, text="ログ / Log", padding=4)
        frame.grid(row=row, column=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        # Read-only monospace log: users can select/copy text but the
        # widget itself is only ever written to by _append_log().
        # 読み取り専用の等幅ログ: ユーザーはテキストの選択/コピーはできるが、
        # ウィジェットへの書き込みは _append_log() からのみ行う。
        self.log_text = scrolledtext.ScrolledText(
            frame, height=LOG_TEXT_HEIGHT_ROWS, font=LOG_TEXT_FONT, state="disabled", wrap="word"
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

    # -- event handlers -------------------------------------------------

    def _refresh_ports(self, preset_port):
        """
        Re-scan serial ports and repopulate the combobox, auto-selecting
        preset_port if given, otherwise the first Espressif VID match,
        otherwise the first port in the list.
        シリアルポートを再スキャンしてコンボボックスへ反映する。
        preset_port が与えられていればそれを、無ければ最初に見つかった
        Espressif VID のポートを、それも無ければ先頭のポートを自動選択する。
        """
        ports = list_serial_ports()
        self._port_devices = [p.device for p in ports]
        self.port_combo["values"] = [format_port_label(p) for p in ports]

        if not self._port_devices:
            self.port_hint_label.grid()
            return
        self.port_hint_label.grid_remove()

        preferred_device = preset_port or guess_espressif_port(ports)
        selected_index = self._port_devices.index(preferred_device) if preferred_device in self._port_devices else 0
        self.port_combo.current(selected_index)

    def _on_browse_local_file(self):
        """Open a file picker for a local .bin and switch to local-file mode."""
        # ローカルの .bin を選ぶファイルダイアログを開き、ローカルモードへ切り替える
        path = filedialog.askopenfilename(
            title="ファームウェアファイルを選択 / Select firmware file",
            filetypes=[("Firmware binary", "*.bin"), ("All files", "*.*")],
        )
        if path:
            self._local_path_var.set(path)
            self._source_mode_var.set(SOURCE_MODE_LOCAL)

    def _on_check_latest_release(self):
        """Fetch the latest release tag in the background and display it."""
        # 最新リリースのタグをバックグラウンドで取得して表示する
        if self._busy:
            return
        self._set_busy(True, "GitHub リリース情報を取得中... / Checking latest release...")

        def worker():
            try:
                release_info = fetch_latest_release_info()
            except Exception as exc:
                message = "リリース情報の取得に失敗しました / Failed to fetch release info: {0}".format(exc)
                self._queue.put(("done", False, message, "check"))
                return
            self._cached_release_info = release_info
            self._queue.put(("release_tag", release_info.get("tag_name", "unknown")))
            self._queue.put(("done", True, None, "check"))

        self._start_worker(worker)

    def _on_flash_clicked(self):
        """Validate inputs, then run the full download/verify/flash workflow in a worker thread."""
        # 入力値を検証し、ダウンロード/検証/書き込みの全工程をワーカースレッドで実行する
        if self._busy:
            return

        source_mode = self._source_mode_var.get()
        target = self._target_var.get()
        erase = self._erase_var.get()
        local_path = self._local_path_var.get()

        port_index = self.port_combo.current()
        if port_index < 0 or not self._port_devices:
            messagebox.showwarning(
                "ポート未選択 / No port selected",
                "シリアルポートを選択してください / Please select a serial port.",
            )
            return
        port = self._port_devices[port_index]

        if source_mode == SOURCE_MODE_LOCAL and not local_path:
            messagebox.showwarning(
                "ファイル未選択 / No file selected",
                "ローカルのファームウェアファイルを選択してください / "
                "Please select a local firmware file.",
            )
            return

        self._clear_log()
        self._set_busy(True, "開始しています... / Starting...")

        def worker():
            try:
                resolved_tag = perform_flash_workflow(
                    self._queue, source_mode, local_path, target, port, erase, self._cached_release_info
                )
                if resolved_tag:
                    self._queue.put(("release_tag", resolved_tag))
                self._queue.put(("done", True, None, "flash"))
            except FlashError as exc:
                self._queue.put(("done", False, str(exc), "flash"))
            except Exception as exc:  # unexpected bug, not a known failure mode / 想定外の不具合
                message = "予期しないエラー / Unexpected error: {0}".format(exc)
                self._queue.put(("done", False, message, "flash"))

        self._start_worker(worker)

    # -- worker/queue plumbing -------------------------------------------------

    def _start_worker(self, target_fn):
        """Start target_fn on a daemon thread; daemon so it never blocks app exit."""
        # target_fn をデーモンスレッドで開始する。デーモンにすることでアプリ終了を妨げない
        threading.Thread(target=target_fn, daemon=True).start()

    def _set_busy(self, busy, status_text):
        """Toggle the busy state: disables Flash while a worker is running."""
        # busy 状態を切り替える: ワーカー実行中は Flash ボタンを無効化する
        self._busy = busy
        self.flash_button.configure(state="disabled" if busy else "normal")
        self._status_var.set(status_text)
        if not busy:
            self.progress_bar["value"] = 0

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.progress_bar["value"] = 0

    def _append_log(self, line):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _poll_queue(self):
        """
        Drain every message currently in the queue (non-blocking), then
        reschedule itself. This is the only method that ever touches
        both self._queue and Tk widgets, so it is the single point
        where worker-thread results cross over to the main thread.
        キューに溜まっているメッセージを(ブロッキングせず)全て処理し、
        自身を再スケジュールする。self._queue と Tk ウィジェットの両方に
        触れるのはこのメソッドだけであり、ワーカースレッドの結果が
        メインスレッドへ渡る唯一の経路となる。
        """
        try:
            while True:
                self._handle_message(self._queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(QUEUE_POLL_INTERVAL_MS, self._poll_queue)

    def _handle_message(self, message):
        kind = message[0]
        if kind == "log":
            line = message[1]
            self._append_log(line)
            percent = parse_progress_percent(line)
            if percent is not None:
                self.progress_bar["value"] = percent
        elif kind == "status":
            self._status_var.set(message[1])
        elif kind == "release_tag":
            self._release_tag_var.set("タグ / tag: {0}".format(message[1]))
        elif kind == "done":
            _, success, error_message, op_kind = message
            self._on_worker_done(success, error_message, op_kind)

    def _on_worker_done(self, success, error_message, op_kind):
        """
        Re-enable the UI and report the outcome. Only a successful
        flash (not a "check latest" fetch) shows the post-flash gyro
        calibration reminder from requirement #6.
        UI を再度有効化し、結果を報告する。書き込み成功時("check latest"
        の取得成功時は除く)のみ、要件6のジャイロ校正リマインダーを表示する。
        """
        self._set_busy(False, "完了 / Done" if success else "失敗 / Failed")
        if success:
            if op_kind == "flash":
                messagebox.showinfo(
                    "書き込み完了 / Flash complete",
                    "重要: 機体は再起動後にジャイロキャリブレーションを自動実行します。"
                    "キャリブレーションが終わるまで、水平な場所に置いて静止させたまま"
                    "待ってください。\n\n"
                    "IMPORTANT: the device reboots and runs gyro calibration. Keep it "
                    "still on a level surface until boot completes.",
                )
        else:
            messagebox.showerror("エラー / Error", error_message)


def launch_gui(preset_port, preset_target):
    """
    Build the Tk root window and enter its event loop.

    This is the ONLY function that instantiates tk.Tk(); it is reached
    from main() exclusively when neither --version nor --selftest was
    passed, so a headless CI invocation never creates a Tk window.

    Tk のルートウィンドウを構築し、イベントループへ入る。

    tk.Tk() をインスタンス化するのはこの関数だけ。main() からは
    --version も --selftest も指定されなかった場合にのみ到達するため、
    ヘッドレスな CI 実行が Tk ウィンドウを作ることは無い。
    """
    root = tk.Tk()
    if _ESPTOOL_IMPORT_ERROR is not None:
        # Requirement #9: a friendly bilingual install hint instead of
        # a stack trace when esptool is missing.
        # 要件9: esptool が無い場合はスタックトレースではなく親しみやすい
        # バイリンガルのインストール案内を表示する。
        root.withdraw()
        messagebox.showerror("esptool が見つかりません / esptool not found", ESPTOOL_INSTALL_HINT)
        root.destroy()
        return 1

    StampFlyFlasherApp(root, preset_port=preset_port, preset_target=preset_target)
    root.mainloop()
    return 0


# ---------------------------------------------------------------------------
# CLI entry point
# CLI エントリポイント
# ---------------------------------------------------------------------------


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="stampfly_flasher.py",
        description="StampFly Flasher: cross-platform desktop GUI for flashing pre-built StampFly firmware.",
    )
    parser.add_argument(
        "--version",
        action="store_true",
        help="Print the app and esptool versions, then exit / アプリとesptoolのバージョンを表示して終了",
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help=(
            "Run a headless, hardware-free self-test and exit / "
            "GUIを表示せず実機不要のセルフテストを実行して終了"
        ),
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Preset serial port device path for the GUI / GUI起動時に選択するシリアルポート",
    )
    parser.add_argument(
        "--target",
        default=None,
        choices=[TARGET_VEHICLE, TARGET_CONTROLLER],
        help="Preset flashing target for the GUI / GUI起動時に選択する書き込み対象",
    )
    return parser


def print_version_info():
    print("{0} {1}".format(APP_NAME, APP_VERSION))
    if esptool is not None:
        print("esptool {0}".format(esptool.__version__))
    else:
        print("esptool: not installed / 未インストール")
        print(ESPTOOL_INSTALL_HINT)


def _ensure_stdio_streams():
    """
    Make sys.stdout / sys.stderr safe for CLI output in every packaging
    and console-encoding scenario. Two independent failure modes are
    handled here:

    1. The streams may be None. PyInstaller's --windowed build on
       Windows has no console to attach to, so Python leaves both
       streams unset, and any print() call — including the ones in
       print_version_info() and run_selftest() — would raise
       AttributeError before printing anything.
       -> Redirect to os.devnull.

    2. The streams may use a non-UTF-8 encoding that cannot represent
       this app's bilingual output (e.g. cp1252 on an English-locale
       Windows cannot encode "セルフテスト"). print() would then raise
       UnicodeEncodeError — and in the --windowed build the exception
       escapes main(), which makes PyInstaller's bootloader show a
       modal "unhandled exception" dialog and wait for a click that
       never comes on CI (observed: the first v2026.07.1 Windows
       release job hung this exact way for hours).
       -> Reconfigure the streams with errors="replace" so that
       unencodable characters degrade to "?" instead of crashing.
       The encoding itself is left untouched: a Japanese cp932
       console keeps displaying Japanese correctly.

    Note: esptool imports colorama, which on Windows replaces
    sys.stdout / sys.stderr with ANSI-translating wrappers that expose
    no reconfigure(). The original streams stay reachable as
    sys.__stdout__ / sys.__stderr__ and still perform the actual
    character encoding inside those wrappers, so reconfiguring them
    fixes the wrapped streams too.

    sys.stdout / sys.stderr をあらゆるパッケージング・コンソール文字
    コード環境で CLI 出力に対して安全な状態に整える。独立した2つの
    故障モードをここで処理する:

    1. ストリームが None の場合。Windows の PyInstaller --windowed
       ビルドには接続先のコンソールが無く、Python は両ストリームを
       未設定のままにするため、print() の呼び出し(print_version_info()
       や run_selftest() 内のものを含む)は何かを表示する前に
       AttributeError を送出してしまう。
       -> os.devnull へリダイレクトする。

    2. ストリームの文字コードが UTF-8 以外で、本アプリのバイリンガル
       出力を表現できない場合(例: 英語ロケール Windows の cp1252 は
       「セルフテスト」をエンコードできない)。print() は
       UnicodeEncodeError を送出し、--windowed ビルドでは例外が
       main() の外へ漏れて PyInstaller の bootloader がモーダルの
       「unhandled exception」ダイアログを表示し、CI では誰も押せない
       クリックを待ち続ける(v2026.07.1 の初回 Windows リリースジョブが
       まさにこの形で数時間ハングするのを観測した)。
       -> errors="replace" でストリームを再構成し、エンコード不能な
       文字はクラッシュではなく「?」に置換して出力する。文字コード
       自体は変更しないため、日本語 Windows の cp932 コンソールでは
       従来どおり日本語が正しく表示される。

    補足: esptool は colorama を import し、colorama は Windows で
    sys.stdout / sys.stderr を ANSI 変換ラッパーに差し替える。この
    ラッパーは reconfigure() を持たないが、元のストリームは
    sys.__stdout__ / sys.__stderr__ として参照可能なまま残っており、
    ラッパー内部の実際の文字エンコードも元のストリームが担うため、
    そちらを再構成すればラップ後のストリームにも効く。
    """
    if sys.stdout is None:
        sys.stdout = open(os.devnull, "w")
    if sys.stderr is None:
        sys.stderr = open(os.devnull, "w")
    for stream in (sys.stdout, sys.stderr, sys.__stdout__, sys.__stderr__):
        # getattr covers both None streams and colorama's wrappers,
        # neither of which offers reconfigure().
        # getattr は None のストリームと colorama ラッパーの両方に対応
        # する(どちらも reconfigure() を持たない)。
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors="replace")
        except (ValueError, OSError):
            # A closed or otherwise exotic stream must never prevent
            # the app from starting.
            # 閉じられた/特殊なストリームが原因でアプリの起動が妨げ
            # られてはならない。
            pass


def main(argv=None):
    """
    CLI entry point. Argument parsing happens before any Tk code runs,
    so --version and --selftest work in fully headless environments.
    CLI エントリポイント。引数解析は Tk 関連のコードより前に行うため、
    --version と --selftest は完全にヘッドレスな環境でも動作する。
    """
    # Must run before argparse (which can itself write to stderr on
    # --help / usage errors) or any print() call.
    # argparse(--help や使用方法エラーで stderr に書くことがある)や
    # 他の print() 呼び出しより前に実行する必要がある。
    _ensure_stdio_streams()

    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.version or args.selftest:
        # CLI exception barrier: in the --windowed build an exception
        # that escapes main() triggers a MODAL error dialog from
        # PyInstaller's bootloader — on a headless CI runner nobody can
        # click it, so any crash would turn into an infinite hang.
        # Convert every unexpected exception into a printed traceback
        # plus exit code 1, which is what a headless caller can
        # actually consume.
        # CLI 例外バリア: --windowed ビルドでは main() の外へ漏れた
        # 例外が PyInstaller bootloader のモーダルなエラーダイアログを
        # 表示させる。ヘッドレスな CI ランナーでは誰もそれを押せない
        # ため、あらゆるクラッシュが無限ハングに化けてしまう。予期
        # しない例外はすべて traceback の表示 + 終了コード 1 に変換し、
        # ヘッドレスな呼び出し元が実際に扱える形にする。
        try:
            if args.version:
                print_version_info()
                return 0
            return run_selftest()
        except Exception:
            traceback.print_exc()
            return 1

    return launch_gui(args.port, args.target)


if __name__ == "__main__":
    sys.exit(main())
