#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StampFly Setup
==============

Purpose / 目的
--------------
A cross-platform (Windows / macOS / Linux) desktop wizard that installs
the full StampFly development environment (ESP-IDF + sfcli + the GUI
Flasher). It exists so that a newcomer with no Python/git experience can
go from "downloaded one file" to "sf doctor passes" by clicking through
five screens, instead of opening a terminal and reading
docs/guides/getting-started.md.

StampFly の開発環境一式(ESP-IDF + sfcli + GUIフラッシャ)を導入する、
クロスプラットフォーム(Windows/macOS/Linux)のデスクトップ・ウィザード。
Python/git の経験が無い初心者でも、ターミナルを開かず5画面をクリックする
だけで「1ファイルをダウンロードした」状態から「sf doctor が通る」状態まで
辿り着けるようにするために存在する。

Single-source-of-truth design / 単一実体の原則
-----------------------------------------------
This GUI owns NO install logic of its own. It only (1) clones the repo
and (2) loads the freshly-cloned scripts/installer.py in-process and
calls its Installer class. Every install step, prompt, and fix lives in
scripts/installer.py alone, so improvements there reach this GUI without
a GUI release. See docs/plans/gui-installer-plan.md §1 for the full
rationale.

このGUIは自前のインストールロジックを一切持たない。(1) リポジトリを
clone し、(2) clone 直後の scripts/installer.py をプロセス内 import して
その Installer クラスを呼ぶだけ。全てのインストール手順・プロンプト・
修正は scripts/installer.py だけが持つため、そちらの改善はGUIの
リリース無しにそのまま反映される。詳細は docs/plans/gui-installer-plan.md
§1 参照。

Language switching / 言語切替
------------------------------
Earlier versions of this GUI showed every label as a fixed "日本語 /
English" bilingual string. It now shows exactly one language at a time,
selectable at runtime: STRINGS (a {key: {"ja": ..., "en": ...}} table)
holds every UI string, tr(key, *format_args) looks up the current
language's copy of STRINGS[key] and .format()s it, and the current
language itself is process-wide module state (get_language() /
set_language()), not something owned by the Tk app instance -- so it is
also available to the tkinter-missing CLI hint, which can run before any
Tk widget exists. detect_default_language() seeds that state at import
time from LC_ALL/LC_MESSAGES/LANG or locale.getlocale(), defaulting to
Japanese when nothing is determinable. A pair of ttk.Radiobutton widgets
on the Welcome page (always labeled "日本語"/"English" in their own
language, never translated) lets the user switch language for the
remainder of the session; switching tears down and rebuilds the whole
page/nav-bar shell via _build_shell() so every page picks up the new
language, while the underlying tk.StringVar/tk.BooleanVar state (install
dir, checkboxes, etc.) survives the rebuild untouched.

以前のこのGUIは、全ラベルを常に「日本語 / English」併記の固定文字列で
表示していた。現在は実行時に選べる単一言語のみを表示する: STRINGS
({key: {"ja": ..., "en": ...}} の対応表)が全UI文字列を保持し、
tr(key, *format_args) が現在言語に対応する STRINGS[key] を取り出して
.format() する。現在言語そのものはTkアプリのインスタンスではなく、
プロセス全体のモジュールレベル状態(get_language() / set_language())
として持つ -- そのため、Tk ウィジェットが1つも存在する前に走りうる
tkinter 不在時の CLI 案内文からも使える。detect_default_language() が
インポート時に LC_ALL/LC_MESSAGES/LANG または locale.getlocale() から
その状態の初期値を決め、判定できない場合は日本語を既定とする。ようこそ
画面上の ttk.Radiobutton 2つ(常にそれぞれの言語の自言語表記「日本語」/
「English」で、翻訳はしない)で、以降のセッションの表示言語を切り替え
られる。切替時は _build_shell() でページ/ナビバーの表示領域全体を
一旦破棄して再構築することで全画面が新しい言語を反映する一方、
インストール先やチェックボックス等を保持する tk.StringVar/tk.BooleanVar
の状態自体は再構築をまたいでそのまま残る。

Usage / 使い方
--------------
    python3 stampfly_installer.py             # Launch the wizard / ウィザードを起動
    python3 stampfly_installer.py --selftest  # Headless CI check / CI向け無人チェック

--selftest is the only flag that launches without a window; it never
instantiates Tk, so it also works on a headless CI runner (and even on a
runner where tkinter itself failed to build).
--selftest だけが GUI を開かずに終了するフラグで、Tk のインスタンス化を
一切行わない。そのため tkinter 自体がビルドされていないヘッドレスな CI
ランナーでも動作する。

Packaging with PyInstaller / PyInstallerでのパッケージング
-----------------------------------------------------------
CI builds a standalone, no-Python-required executable for each OS with:

    pyinstaller --onefile --windowed --name StampFlySetup \\
        tools/installer_gui/stampfly_installer.py

The CI self-test job omits --windowed (or overrides it with --console)
so --selftest's stdout stays visible in the build log, exactly as
tools/flasher_gui/stampfly_flasher.py's job does.

CI は各OS向けに Python 不要のスタンドアロン実行ファイルを上記コマンドで
ビルドする。CI のセルフテストジョブでは `--windowed` を外す(または
`--console` で上書きする)ことで `--selftest` の標準出力がビルドログに
残るようにする(tools/flasher_gui/stampfly_flasher.py のジョブと同じ)。
"""

import argparse
import contextlib
import ctypes.util  # noqa: F401  -- hidden-import only, see the contract block below
import importlib.util
import inspect
import io
import locale
import os
import queue
import re
import shlex  # noqa: F401  -- hidden-import only, see the contract block below
import shutil
import socket
import subprocess
import sys
import tempfile  # noqa: F401  -- hidden-import only, see the contract block below
import threading
import traceback
import types
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Hidden-import contract with scripts/installer.py
# scripts/installer.py との「隠れ import」契約
# ---------------------------------------------------------------------------
# When frozen with `pyinstaller --onefile`, PyInstaller decides which
# stdlib modules to bundle by STATICALLY scanning this file's own import
# statements. scripts/installer.py is never imported at freeze time -- it
# is cloned onto the user's disk at RUN time and loaded with
# importlib.util (see load_installer_module() below) -- so PyInstaller's
# static scanner cannot see, and will not bundle, any stdlib module that
# only scripts/installer.py imports. Any such module must therefore be
# imported here too, purely so it ends up in the frozen bundle.
#
# `pyinstaller --onefile` で凍結する際、PyInstaller はこのファイル自身の
# import 文を静的に走査して、どの標準ライブラリを同梱するか決める。
# scripts/installer.py は凍結時には一切 import されず、実行時にユーザの
# ディスクへ clone された後 importlib.util で読み込まれる(下の
# load_installer_module() 参照)ため、PyInstaller の静的スキャナは
# scripts/installer.py だけが import している標準ライブラリを見つけられず
# 同梱もしない。そのため、そうしたモジュールは(同梱させる目的だけのために)
# ここでも import しておく必要がある。
#
# Verified against scripts/installer.py's actual `import`/`from ... import`
# statements on 2026-07-20 (`grep -n "^import \|^from " scripts/installer.py`),
# re-verified same day after the StampFly Terminal launcher feature added
# `tempfile`:
#   os, shlex, sys, subprocess, shutil, tempfile, pathlib.Path,
#   typing.{Optional,List,Tuple}, re (used inline in two functions),
#   ctypes.util (used inline, Linux-only branch), tempfile (used inline,
#   Windows-only branch: Installer._create_terminal_launcher_windows()'s
#   .ps1 temp file), argparse (used inline in main()).
# All of these are already imported above for this GUI's own use, EXCEPT
# shlex, ctypes.util, and tempfile, which are imported for this contract
# alone (see the `# noqa: F401` markers). urllib.request/ssl/json/venv were
# considered as generic examples during planning but are NOT currently
# imported by scripts/installer.py -- do not add them speculatively.
#
# CONTRACT: whenever scripts/installer.py starts importing a NEW stdlib
# module, add the same import here (docs/plans/gui-installer-plan.md §5).
#
# 2026-07-20 時点で scripts/installer.py の実際の import 文
# (`grep -n "^import \|^from " scripts/installer.py`)を確認して網羅し、
# 同日 StampFly Terminal ランチャー機能追加による `tempfile` 追加後に
# 再確認した:
#   os, shlex, sys, subprocess, shutil, tempfile, pathlib.Path,
#   typing.{Optional,List,Tuple}, re(2関数内でインライン import)、
#   ctypes.util(インライン import、Linux限定分岐)、
#   tempfile(インライン import、Windows限定分岐:
#   Installer._create_terminal_launcher_windows() の .ps1 一時ファイル用)、
#   argparse(main() 内でインライン import)。
# これらは shlex・ctypes.util・tempfile を除き、既にこのGUI自身の用途で
# import 済み(shlex/ctypes.util/tempfile は本契約のためだけに import
# している。`# noqa: F401` の箇所)。urllib.request/ssl/json/venv は計画時に
# 一般例として挙がったが、現時点の scripts/installer.py は import して
# いない -- 推測で足さないこと。
#
# 契約: scripts/installer.py が新しい標準ライブラリの import を始めたら、
# 必ずこのファイルにも同じ import を追加する
# (docs/plans/gui-installer-plan.md §5)。

# tkinter is part of the Python standard library; importing it does not
# require a display connection, only instantiating Tk() does. Guard it
# anyway (unlike stampfly_flasher.py's unconditional import) because this
# tool must also degrade gracefully -- print CLI instructions and exit 1
# -- on a minimal Linux build with no tkinter at all, per the spec.
#
# tkinter は標準ライブラリの一部で、import 自体はディスプレイ接続を
# 必要としない(必要なのは Tk() のインスタンス化のみ)。それでも
# (stampfly_flasher.py の無条件 import とは異なり)ここではガードする。
# tkinter が全く無い最小構成の Linux でも、CLI手順を案内して exit 1 する
# という優雅な劣化が仕様として求められているため。
try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, scrolledtext, ttk
except ImportError as _tkinter_import_error:  # pragma: no cover - only hit without tkinter
    tk = None
    filedialog = messagebox = scrolledtext = ttk = None
    _TKINTER_IMPORT_ERROR = _tkinter_import_error
else:
    _TKINTER_IMPORT_ERROR = None


# ---------------------------------------------------------------------------
# Constants
# 定数
# ---------------------------------------------------------------------------
# No magic numbers/strings below the constants block; every value used
# more than once, or that a reader would need to look up, is named here.
# 以下のコードにマジックナンバー/マジックストリングは埋め込まない。
# 2回以上使う値や、読み手が意味を確認したくなる値は、必ずここで名前を付ける。

APP_NAME = "StampFly Setup"
APP_VERSION = "0.1.0"

# The repository this wizard clones and then drives via its installer.py.
# このウィザードが clone し、その installer.py を操作するリポジトリ。
REPO_URL = "https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git"
DEFAULT_INSTALL_DIR = Path.home() / "stampfly_ecosystem"

# Read by --selftest to point at a repo other than "2 directories above
# this file" -- see default_setup_repo() / resolve_setup_repo_for_selftest().
# --selftest が「このファイルの2階層上」以外のリポジトリを指すために読む
# 環境変数 -- default_setup_repo() / resolve_setup_repo_for_selftest() 参照。
SELFTEST_REPO_ENV_VAR = "SF_SETUP_REPO"

# Non-interactive contract shared with scripts/installer.py's prompt()/
# prompt_choice() (see run_installer_in_process()).
# scripts/installer.py の prompt()/prompt_choice() と共有する非対話化契約
# (run_installer_in_process() 参照)。
INSTALLER_NONINTERACTIVE_ENV_VAR = "SF_INSTALLER_NONINTERACTIVE"

# Prerequisite-check tuning.
# 前提条件チェックの調整値。
PYTHON_MIN_VERSION = (3, 8)
DISK_SPACE_MINIMUM_GB = 5.0
BYTES_PER_GIB = 1024 ** 3
NETWORK_CHECK_HOST = "github.com"
NETWORK_CHECK_PORT = 443
NETWORK_CHECK_TIMEOUT_SECONDS = 3
SUBPROCESS_PROBE_TIMEOUT_SECONDS = 5

# OS-specific install commands offered (with a "copy" button) for the two
# prerequisites that can actually be missing as external tools.
# 外部ツールとして本当に「無い」ことがある2つの前提について、
# (コピー ボタン付きで)提示する OS別の導入コマンド。
GIT_INSTALL_COMMANDS = {
    "win32": "winget install Git.Git",
    "darwin": "brew install git",
    "linux": "sudo apt install git",
}
PYTHON_INSTALL_COMMANDS = {
    "win32": "winget install Python.Python.3.12",
    "darwin": "brew install python3",
    "linux": "sudo apt install python3",
}


# ---------------------------------------------------------------------------
# Internationalization (i18n)
# 国際化(i18n)
# ---------------------------------------------------------------------------
# Every UI string this wizard displays lives in STRINGS below, keyed by a
# short identifier, with one entry per supported language. tr() looks up
# the current language's copy and (when format_args are given) .format()s
# it, mirroring the {0}-style placeholders the rest of this file already
# uses. The current language is process-wide module state (get_language()/
# set_language()), NOT an attribute of StampFlySetupApp, so it is equally
# usable from launch_gui()'s tkinter-missing CLI hint, which can run
# before any Tk widget -- and therefore before StampFlySetupApp -- exists.
# scripts/installer.py's OWN console output (captured into the log via
# _QueueWriter) is explicitly out of scope: that text is authored and
# owned by scripts/installer.py, not by this GUI.
#
# このウィザードが表示する全UI文字列は、下の STRINGS に短い識別子を
# キーとして格納する(対応言語ごとに1エントリ)。tr() が現在言語の
# コピーを取り出し、format_args が渡されていれば .format() する(この
# ファイルの他の箇所が既に使っている {0} 形式のプレースホルダに合わせる)。
# 現在言語は StampFlySetupApp の属性ではなく、プロセス全体のモジュール
# レベル状態(get_language()/set_language())として持つ。そのため、
# Tk ウィジェット(ひいては StampFlySetupApp)が1つも存在するより前に
# 走りうる launch_gui() の tkinter 不在時 CLI 案内文からも同様に使える。
# scripts/installer.py 自身のコンソール出力(_QueueWriter 経由でログへ
# 捕捉されるもの)は明示的に対象外: そのテキストの著者・所有者は
# scripts/installer.py であり、このGUIではない。

LANG_JA = "ja"
LANG_EN = "en"

STRINGS: Dict[str, Dict[str, str]] = {
    # -- shared navigation bar (see _build_nav_bar()/_configure_nav_for()) --
    "nav_back": {"ja": "< 戻る", "en": "< Back"},
    "nav_next": {"ja": "次へ >", "en": "Next >"},
    "nav_recheck": {"ja": "再チェック", "en": "Recheck"},
    "nav_cancel": {"ja": "キャンセル", "en": "Cancel"},
    "nav_save_log": {"ja": "ログを保存", "en": "Save log"},
    "nav_close": {"ja": "閉じる", "en": "Close"},
    "nav_start_install": {"ja": "インストール開始 >", "en": "Start install >"},
    "nav_start_uninstall": {"ja": "アンインストール開始 >", "en": "Start uninstall >"},

    # -- page 1: welcome (see _build_page_welcome()) -------------------------
    "welcome_intro": {
        "ja": "StampFly の開発環境一式(sf CLI・ESP-IDF・GUIフラッシャ)を導入します。",
        "en": "This wizard installs the full StampFly development environment "
              "(sf CLI, ESP-IDF, and the GUI Flasher).",
    },
    "welcome_estimate": {
        "ja": "目安: ダウンロード数GB・所要時間15〜30分(回線速度による)。",
        "en": "Approx. a few GB download, 15-30 minutes depending on your connection.",
    },
    "welcome_repository": {"ja": "リポジトリ: {0}", "en": "Repository: {0}"},

    # -- page 2: environment check (see _build_page_env_check() etc.) -------
    "env_check_title": {"ja": "環境チェック", "en": "Environment check"},
    "env_check_label_python": {"ja": "システム Python 3.8+", "en": "System Python 3.8+"},
    "env_check_label_disk": {
        "ja": "インストール先の空き容量(目安{0:.0f}GB)",
        "en": "Free disk space (~{0:.0f}GB)",
    },
    "env_check_label_network": {
        "ja": "ネットワーク到達性({0}:{1})",
        "en": "Network reachability ({0}:{1})",
    },
    "env_check_copy_command": {"ja": "コマンドをコピー", "en": "Copy command"},
    "env_check_checking": {"ja": "確認中...", "en": "Checking..."},
    "env_check_cannot_proceed": {
        "ja": "必須項目が未解決のため次へ進めません: {0}",
        "en": "Cannot proceed, required checks failing: {0}",
    },
    "env_check_copied": {"ja": "コピーしました: {0}", "en": "Copied to clipboard: {0}"},
    "env_check_worker_error": {
        "ja": "確認中にエラーが発生しました: {0}",
        "en": "An error occurred while checking: {0}",
    },
    "env_check_disk_free_detail": {"ja": "空き{0:.1f}GB", "en": "{0:.1f} GB free"},
    "python_not_found": {"ja": "見つかりません", "en": "not found"},
    "python_check_failed": {"ja": "確認できません ({0})", "en": "could not check ({0})"},
    "python_version_unknown": {"ja": "不明", "en": "unknown"},

    # -- page 3: options (see _build_page_options() etc.) --------------------
    "options_title": {"ja": "オプション", "en": "Options"},
    "options_install_to": {"ja": "インストール先", "en": "Install to"},
    "options_browse": {"ja": "参照", "en": "Browse"},
    "options_include_flasher": {
        "ja": "GUIフラッシャも入れる",
        "en": "Also install the GUI Flasher",
    },
    "options_desktop_shortcut": {
        "ja": "デスクトップショートカットを作成",
        "en": "Create a desktop shortcut",
    },
    "options_minimal": {
        "ja": "シミュレータ関連の依存を省略(minimal)",
        "en": "Skip simulator dependencies (minimal)",
    },
    "options_existing_checkout_title": {
        "ja": "既存のインストールが見つかりました",
        "en": "Existing install detected",
    },
    "options_repair": {"ja": "修復(再実行)", "en": "Repair (re-run)"},
    "options_uninstall": {"ja": "アンインストール", "en": "Uninstall"},
    "options_browse_dialog_title": {
        "ja": "インストール先を選択",
        "en": "Select install directory",
    },

    # -- page 4: execute (see _build_page_execute() etc.) ---------------------
    "execute_title": {"ja": "実行", "en": "Running"},
    "execute_log_label": {"ja": "ログ", "en": "Log"},
    "execute_starting": {"ja": "開始しています...", "en": "Starting..."},
    "execute_cancelling": {"ja": "キャンセル中...", "en": "Cancelling..."},
    "execute_cannot_cancel": {
        "ja": "インストール実行中はキャンセルできません",
        "en": "Cannot cancel while the installer is running",
    },
    # Step indicator labels (see step_labels() below); "Clone" is the only
    # one whose Japanese and English text actually differ -- the other 4
    # are installer.py's own "Step N/4:" headers plus a proper noun
    # (ESP-IDF/Python/CLI/Flasher), which this project does not translate.
    # ステップインジケータのラベル(下の step_labels() 参照)。「取得/
    # Clone」だけが日英で文言が異なる -- 残り4つは installer.py 自身の
    # "Step N/4:" ヘッダ+固有名詞(ESP-IDF/Python/CLI/Flasher)のため、
    # 本プロジェクトでは翻訳しない。
    "step_label_clone": {"ja": "取得", "en": "Clone"},
    "step_label_esp_idf": {"ja": "Step 1/4\nESP-IDF", "en": "Step 1/4\nESP-IDF"},
    "step_label_python": {"ja": "Step 2/4\nPython", "en": "Step 2/4\nPython"},
    "step_label_cli": {"ja": "Step 3/4\nCLI", "en": "Step 3/4\nCLI"},
    "step_label_flasher": {"ja": "Step 4/4\nFlasher", "en": "Step 4/4\nFlasher"},

    # -- page 5: done (see _render_done_page()) -------------------------------
    "done_failed_title": {"ja": "失敗しました", "en": "Setup failed"},
    "done_failed_body": {
        "ja": "上のログを確認し、CLI から復旧してください。\n\n  {0}",
        "en": "Check the log above, then recover via the CLI:\n\n  {0}",
    },
    "done_uninstall_title": {"ja": "アンインストール完了", "en": "Uninstall complete"},
    "done_uninstall_body": {
        "ja": "sfcli と関連設定を削除しました。詳細は実行画面のログを参照してください。",
        "en": "sfcli and its configuration were removed. See the Execute page's log for details.",
    },
    "done_install_title": {"ja": "インストール完了", "en": "Setup complete"},
    "done_install_body": {
        "ja": "次の一歩 -- ターミナルを開いて実行してください:\n\n  {0}\n  sf doctor"
              "\n\nまたは『StampFly Terminal』（スタートメニュー / ~/Applications / "
              "アプリ一覧）から始められます。",
        "en": "Next steps -- open a terminal and run:\n\n  {0}\n  sf doctor"
              "\n\nOr get started from \"StampFly Terminal\" (Start Menu / "
              "~/Applications / your app launcher).",
    },
    "save_log_saved": {"ja": "ログを保存しました: {0}", "en": "Log saved to: {0}"},

    # -- worker status (see _on_worker_done()) --------------------------------
    "status_done": {"ja": "完了", "en": "Done"},
    "status_failed": {"ja": "失敗 (code {0})", "en": "Failed (code {0})"},

    # -- clone / installer.py workflow queue messages (worker thread) --------
    "clone_status_cloning": {"ja": "リポジトリを取得中...", "en": "Cloning repository..."},
    "clone_cancelled": {"ja": "キャンセルされました", "en": "Cancelled"},
    "clone_reuse_existing": {
        "ja": "既存のクローンを再利用します: {0}",
        "en": "Reusing existing checkout at {0}",
    },
    "repair_updating": {
        "ja": "既存クローンを最新化しています...",
        "en": "Updating the existing checkout...",
    },
    "repair_update_error": {
        "ja": "更新できませんでした（このまま続行します）: {0}",
        "en": "Could not update (continuing): {0}",
    },
    "repair_update_not_ff": {
        "ja": "既存クローンを更新できませんでした。現状のコードのまま修復を続行します",
        "en": "Existing checkout could not be fast-forwarded; repairing with the code as-is.",
    },
    "old_installer_options_ignored": {
        "ja": "この installer.py は古く、次のオプションを解釈できないため無視します: {0}",
        "en": "This installer.py predates these options; ignoring: {0}",
    },
    "git_clone_start_failed": {
        "ja": "git clone を開始できません: {0}",
        "en": "Could not start git clone: {0}",
    },
    "install_error_installer_not_found": {
        "ja": "scripts/installer.py が見つかりません: {0}",
        "en": "scripts/installer.py not found: {0}",
    },
    "install_error_installer_load_failed": {
        "ja": "scripts/installer.py を読み込めません: {0}",
        "en": "Could not load scripts/installer.py: {0}",
    },

    # -- tkinter-missing CLI fallback (see tkinter_missing_cli_hint()) -------
    "tkinter_missing_hint": {
        "ja": (
            "tkinter が見つからないため GUI を起動できません。"
            "CLI から直接インストールしてください:\n"
            "  git clone {0} {1}\n"
            "  cd {1}\n"
            "  ./install.sh          (Windows: install.bat)"
        ),
        "en": (
            "tkinter is not available, so the GUI cannot start. "
            "Install via the CLI instead:\n"
            "  git clone {0} {1}\n"
            "  cd {1}\n"
            "  ./install.sh          (Windows: install.bat)"
        ),
    },

    # -- top-level GUI launch failure (see main()'s launch_gui() guard) -----
    "gui_launch_failed_title": {"ja": "{0} - 起動エラー", "en": "{0} - Launch Error"},
    "gui_launch_failed_body": {
        "ja": "{0} の起動に失敗しました。\n\n{1}",
        "en": "{0} failed to start.\n\n{1}",
    },
}

# Process-wide current-language state. Deliberately module-level rather
# than an attribute of StampFlySetupApp -- see this section's header
# comment for why. Seeded from detect_default_language() right after this
# section (search "_current_language = detect_default_language()"), so it
# always holds a valid LANG_* value by the time anything else in this
# module runs.
# プロセス全体の現在言語状態。StampFlySetupApp の属性ではなくあえて
# モジュールレベルに持つ理由は、本セクション冒頭のコメント参照。この
# セクションの直後("_current_language = detect_default_language()" を
# 検索)で detect_default_language() から初期値を入れるため、このモジュール
# の他のどのコードが動く時点でも常に有効な LANG_* 値を保持している。
_current_language = LANG_JA


def get_language() -> str:
    """Return the language (LANG_JA or LANG_EN) tr() currently resolves against."""
    # tr() が現在参照している言語(LANG_JA または LANG_EN)を返す
    return _current_language


def set_language(language: str) -> None:
    """Set the language tr() resolves against from this call onward."""
    # 以降 tr() が参照する言語を設定する
    global _current_language
    _current_language = language


def tr(key: str, *format_args) -> str:
    """
    Look up STRINGS[key] for the current language (see get_language()) and
    .format(*format_args) it if any were given. Every UI-facing string in
    this file (outside of scripts/installer.py's own captured output)
    should be produced through this function, never a literal.
    STRINGS[key] を現在言語(get_language() 参照)で引き、format_args が
    渡されていれば .format(*format_args) する。このファイルが表示する
    UI文字列(scripts/installer.py 自身が出力しログへ捕捉される文字列を
    除く)は、リテラルではなく必ずこの関数を通して作ること。
    """
    text = STRINGS[key][get_language()]
    return text.format(*format_args) if format_args else text


def detect_default_language() -> str:
    """
    Best-effort guess at the UI language to show before the user has ever
    touched the welcome page's language switcher. Called once, right
    after this function is defined (see "_current_language =
    detect_default_language()" below), to seed _current_language.

    Order of evidence:
      1. LC_ALL / LC_MESSAGES / LANG: if any of these environment
         variables holds a non-empty value, "ja" (case-insensitive)
         anywhere in it means Japanese; a non-empty value that does NOT
         contain "ja" means English (this wizard only ships ja/en, so
         "an explicit non-Japanese locale" resolves unambiguously to the
         other option).
      2. Only if none of those three variables has any value at all, fall
         back to locale.getlocale() the same way: "ja"/"Japanese" in its
         language-code component means Japanese, any other non-empty
         result means English.
      3. If NEITHER source yields any information at all (all three env
         vars empty/unset AND locale.getlocale() raises or returns no
         language code) -- e.g. a frozen PyInstaller app launched by
         double-clicking from Finder/Explorer often has none of
         LC_ALL/LC_MESSAGES/LANG set -- default to Japanese, because this
         tool's primary audience is Japanese educational sites (see the
         module docstring's Purpose section). This is a deliberate
         default for the "no signal at all" case, not a fallback for "the
         signal says something else": a positively-detected non-Japanese
         locale still resolves to English via steps 1/2 above.

    ようこそ画面の言語スイッチャーにユーザーがまだ触れていない時点で
    表示するUI言語の最良推定。この関数定義の直後
    ("_current_language = detect_default_language()" 参照)で一度だけ
    呼ばれ、_current_language の初期値を決める。

    判定の優先順位:
      1. LC_ALL / LC_MESSAGES / LANG: これら環境変数のいずれかが空でない
         値を持てば、その値に(大小文字を無視して)"ja" が含まれる場合は
         日本語、含まれない場合は英語と判定する(本ツールは ja/en の
         2言語のみのため、「明示的に日本語ではないロケール」は一意に
         もう一方=英語に定まる)。
      2. 上記3変数のいずれも値を持たない場合に限り、同じ考え方で
         locale.getlocale() にフォールバックする: 言語コード部分に
         "ja"/"Japanese" が含まれれば日本語、それ以外の空でない結果は
         英語。
      3. どちらの情報源からも一切の手がかりが得られない場合(3変数とも
         空/未設定、かつ locale.getlocale() が例外を送出するか言語コード
         を返さない) -- 例えば Finder/Explorer からダブルクリックで
         起動した凍結 PyInstaller アプリは LC_ALL/LC_MESSAGES/LANG が
         軒並み未設定なことがある -- 日本語を既定とする。本ツールの
         主対象が日本の教育現場であるため(モジュールdocstringのPurpose
         節参照)。これは「手がかりが全く無い」場合だけの意図的な既定値
         であり、「手がかりが何か他のことを言っている」場合のフォール
         バックではない: 積極的に非日本語と判定されたロケールは、上記
         1/2 の時点で英語に定まる。
    """
    # POSIX precedence: LC_ALL overrides LC_MESSAGES overrides LANG, so the
    # FIRST non-empty variable in that order decides alone. (Joining all
    # three and searching the blob would let a session-wide LANG=ja_JP
    # override an explicit LC_ALL=en_US -- the exact opposite of what a
    # user setting LC_ALL asked for.)
    # POSIX の優先順位: LC_ALL > LC_MESSAGES > LANG であり、この順で最初に
    # 空でない変数が単独で決定する。(3つを連結してから "ja" を探すと、
    # シェル全体の LANG=ja_JP が明示的な LC_ALL=en_US を覆してしまい、
    # LC_ALL を設定したユーザーの意図と正反対になる。)
    for env_var_name in ("LC_ALL", "LC_MESSAGES", "LANG"):
        env_value = os.environ.get(env_var_name, "").strip()
        if env_value:
            return LANG_JA if "ja" in env_value.lower() else LANG_EN

    try:
        locale_language_code, _locale_encoding = locale.getlocale()
    except (TypeError, ValueError):
        locale_language_code = None
    if locale_language_code:
        lowered_locale = locale_language_code.lower()
        return LANG_JA if ("ja" in lowered_locale or "japanese" in lowered_locale) else LANG_EN

    return LANG_JA


_current_language = detect_default_language()


# Environment-check row identifiers, in display order.
# 環境チェック各行の識別子(表示順)。
ENV_CHECK_GIT = "git"
ENV_CHECK_PYTHON = "python"
ENV_CHECK_DISK = "disk"
ENV_CHECK_NETWORK = "network"
ENV_CHECK_ORDER = [ENV_CHECK_GIT, ENV_CHECK_PYTHON, ENV_CHECK_DISK, ENV_CHECK_NETWORK]


def env_check_label(name: str) -> str:
    """
    Display label for one Environment Check row (see ENV_CHECK_ORDER),
    translated via tr(). "git" is a literal, not a STRINGS key, because a
    bare product name does not need translating and is identical in both
    languages.
    環境チェック各行(ENV_CHECK_ORDER 参照)の表示ラベルを、tr() 経由で
    翻訳して返す。"git" は STRINGS のキーではなくリテラル -- 素の製品名は
    翻訳不要で、どちらの言語でも同一のため。
    """
    if name == ENV_CHECK_GIT:
        return "git"
    if name == ENV_CHECK_PYTHON:
        return tr("env_check_label_python")
    if name == ENV_CHECK_DISK:
        return tr("env_check_label_disk", DISK_SPACE_MINIMUM_GB)
    if name == ENV_CHECK_NETWORK:
        return tr("env_check_label_network", NETWORK_CHECK_HOST, NETWORK_CHECK_PORT)
    raise ValueError("Unknown environment-check row: {0}".format(name))  # unreachable via ENV_CHECK_ORDER


# Only git and network actually block the "Next" button on the
# Environment Check page. System Python and disk space are advisory only
# (see docs/plans/gui-installer-plan.md §1 前提条件の扱い): this GUI's own
# bundled Python is what runs installer.py, so a missing/old system
# Python does not stop the install, and the disk figure is only a rough
# estimate ("目安"), not a hard requirement.
# 「次へ」を実際にブロックするのは git とネットワークのみ。システム
# Python とディスク容量は参考表示にとどめる
# (docs/plans/gui-installer-plan.md §1 前提条件の扱い参照): installer.py を
# 実行するのはこのGUI自身に同梱された Python であり、システム Python が
# 無い/古くてもインストールは止めない。ディスク容量も「目安」であり
# 厳密な要件ではない。
ENV_CHECK_REQUIRED = frozenset({ENV_CHECK_GIT, ENV_CHECK_NETWORK})
ENV_CHECK_INSTALL_COMMANDS = {
    ENV_CHECK_GIT: GIT_INSTALL_COMMANDS,
    ENV_CHECK_PYTHON: PYTHON_INSTALL_COMMANDS,
}

# Wizard pages, in navigation order.
# ウィザードの画面(遷移順)。
PAGE_WELCOME = "welcome"
PAGE_ENV_CHECK = "env_check"
PAGE_OPTIONS = "options"
PAGE_EXECUTE = "execute"
PAGE_DONE = "done"
PAGE_ORDER = [PAGE_WELCOME, PAGE_ENV_CHECK, PAGE_OPTIONS, PAGE_EXECUTE, PAGE_DONE]

# Options page: how to treat an install directory that already contains a
# git checkout of this repo.
# オプション画面: 既に本リポジトリの git checkout があるインストール先を
# どう扱うか。
EXISTING_CHECKOUT_MODE_REPAIR = "repair"
EXISTING_CHECKOUT_MODE_UNINSTALL = "uninstall"

# Worker-thread workflow: which action to run, and which phase is active
# (the phase gates whether Cancel is enabled -- see perform_setup_workflow()).
# ワーカースレッドの処理: どちらのアクションを実行するか、どのフェーズが
# 進行中か(フェーズがキャンセル可否を決める -- perform_setup_workflow() 参照)。
ACTION_INSTALL = "install"
ACTION_UNINSTALL = "uninstall"
PHASE_CLONE = "clone"
PHASE_INSTALL = "install"

# Step indicator: 1 stage for the git clone, plus installer.py's own 4
# numbered steps. The regex below is the CONTRACT with
# scripts/installer.py's header() calls ("Step N/4: <title>") -- see
# docs/plans/gui-installer-plan.md §1 進捗表示の契約. If installer.py ever
# renumbers or rewords its headers, this pattern (and step_labels()'s
# STRINGS entries) must be updated together.
# ステップインジケータ: git clone の1段 + installer.py 自身の番号付き4段。
# 下の正規表現は scripts/installer.py の header() 呼び出し
# ("Step N/4: <タイトル>")との契約(docs/plans/gui-installer-plan.md §1
# 進捗表示の契約 参照)。installer.py 側でヘッダの番号や文言を変えたら、
# このパターン(と step_labels() の STRINGS エントリ)も合わせて更新すること。
STEP_HEADER_PATTERN = re.compile(r"Step (\d)/4:")

# A stricter pattern used ONLY by run_selftest()'s source-scan check (4).
# STEP_HEADER_PATTERN above is matched against already-captured log LINES
# at runtime, where only genuine header() output can ever appear -- but
# scanning the whole SOURCE TEXT with that same loose pattern also
# matches an incidental docstring mention ("...(Step 4/4: GUI Flasher
# install)." in _run_sf_in_idf_env()'s docstring, prose -- not a
# header() call), which would overcount to 5. Requiring the `header("`
# prefix scopes the source-text count to actual step-indicator call
# sites. Verified: 2026-07-20, `grep -c` confirms 4 real call sites vs 5
# raw substring matches in scripts/installer.py.
# run_selftest() のソーススキャン用チェック(4)でのみ使う、より厳密な
# パターン。上の STEP_HEADER_PATTERN は実行時に既に捕捉済みのログ「行」
# に対して照合するため、genuine な header() 出力しか現れ得ない -- しかし
# 同じ緩いパターンでソース全文をスキャンすると、
# _run_sf_in_idf_env() の docstring 中の地の文("...(Step 4/4: GUI
# Flasher install)." — header() 呼び出しではない)にも偶然マッチし、
# 5件と過大カウントしてしまう。`header("` という前置を要求することで、
# ソーステキストのカウント対象を実際のステップインジケータ呼び出し箇所に
# 限定する。確認済み: 2026-07-20、`grep -c` で scripts/installer.py 中の
# 実呼び出し4件・素朴な部分文字列一致5件を確認。
STEP_HEADER_CALL_PATTERN = re.compile(r'header\("Step \d/4:')

def step_labels() -> List[str]:
    """Step-indicator label texts, in display order, for the current language."""
    # ステップインジケータのラベル文字列(表示順)を現在言語で返す
    return [
        tr("step_label_clone"),
        tr("step_label_esp_idf"),
        tr("step_label_python"),
        tr("step_label_cli"),
        tr("step_label_flasher"),
    ]

# GUI layout, matched to tools/flasher_gui/stampfly_flasher.py's
# StampFlyFlasherApp (window size, poll interval, log font/height, hint
# wrap width, status colors) so the two desktop tools feel like one
# family, per this task's spec.
# GUIレイアウトは tools/flasher_gui/stampfly_flasher.py の
# StampFlyFlasherApp に合わせる(ウィンドウサイズ・ポーリング間隔・
# ログのフォント/行数・ヒントの折り返し幅・状態表示の色)。この2つの
# デスクトップツールが一つのファミリーに見えるように。
WINDOW_WIDTH_PX = 640
WINDOW_HEIGHT_PX = 560
WINDOW_MIN_HEIGHT_PX = 480
QUEUE_POLL_INTERVAL_MS = 50
LOG_TEXT_HEIGHT_ROWS = 12
LOG_TEXT_FONT = "TkFixedFont"  # Tk's built-in cross-platform monospace font / Tk標準のクロスプラットフォーム等幅フォント
HINT_TEXT_WRAP_PX = 560
WARN_TEXT_COLOR = "#a05a00"
NOTE_TEXT_COLOR = "#555555"
OK_TEXT_COLOR = "#1a7f37"

def tkinter_missing_cli_hint() -> str:
    """
    Message printed to stderr (and used as the exit path) when tkinter
    itself cannot be imported -- e.g. a minimal Linux distro packaged
    Python without Tk. Computed fresh on each call (rather than cached at
    import time, as the old TKINTER_MISSING_CLI_HINT constant was) so it
    reflects whatever language detect_default_language() (or a prior
    set_language() call) has selected -- this path can run before any Tk
    widget, and therefore before the language switcher, ever exists.
    tkinter そのものが import できない場合(例: Tk無しでパッケージされた
    最小構成の Linux ディストリの Python)に stderr へ表示する案内文
    (この場合の終了経路そのものでもある)。(旧 TKINTER_MISSING_CLI_HINT
    定数のようにインポート時にキャッシュせず)呼び出しごとに算出する
    ことで、detect_default_language()(または事前の set_language() 呼び
    出し)が選んだ言語を常に反映する -- この経路は Tk ウィジェット、
    ひいては言語スイッチャーが1つも存在するより前に走りうるため。
    """
    return tr("tkinter_missing_hint", REPO_URL, DEFAULT_INSTALL_DIR)


# ---------------------------------------------------------------------------
# Errors
# エラー
# ---------------------------------------------------------------------------


class InstallError(Exception):
    """
    Raised for any failure in the clone/import/install pipeline that this
    GUI itself detects (before installer.py even gets a chance to run, or
    around it). The message is already a human-friendly string in the
    current UI language (built via tr()), ready to push straight into
    the log.
    このGUI自身が検知する(installer.py が動く前後の) clone/import/install
    工程の失敗で送出する。メッセージは既に(tr() 経由で組み立てた)現在の
    UI言語での人間向け文字列で、そのままログへ積める。
    """


# ---------------------------------------------------------------------------
# Prerequisite probes (pure functions, no Tk) -- each returns a plain bool
# so --selftest can verify the contract mechanically (see run_selftest()).
# Human-readable detail strings are separate get_*() functions.
# 前提条件プローブ(純粋関数、Tk非依存) -- --selftest が契約を機械的に
# 検証できるよう、それぞれ素の bool を返す(run_selftest() 参照)。
# 人間可読な詳細文字列は別の get_*() 関数に分離する。
# ---------------------------------------------------------------------------


def check_git_available() -> bool:
    """Return whether a `git` executable is on PATH."""
    # `git` 実行ファイルが PATH 上にあるか
    return shutil.which("git") is not None


def _windows_python_exe_candidates() -> List[Path]:
    """Windows: likely python.exe paths NOT necessarily on PATH, in the
    same priority order installer.py's _find_system_python_dir() uses.
    Windows: 必ずしも PATH 上に無い python.exe の候補。installer.py の
    _find_system_python_dir() と同じ優先順で返す。

    Kept in sync with installer.py's discovery (which in turn mirrors
    install.bat). Needed here because the environment-check page runs
    BEFORE the repo is cloned, so it cannot call installer.py's copy --
    without this, Python installed but not on PATH (the common Windows
    case, when "Add to PATH" was left unchecked) would show a misleading
    "NG" even though the install would actually succeed.
    installer.py の発見(さらに install.bat を反映)と同期を保つ。環境チェック
    画面はリポジトリの clone より前に走るため installer.py の複製を呼べず、
    ここに必要 -- これが無いと、PATH 外に入った Python("Add to PATH" 未
    チェックの一般的な Windows ケース)が、実際には導入成功するのに
    誤って「NG」表示されてしまう。
    """
    candidates: List[Path] = []
    userprofile = os.environ.get("USERPROFILE", "")
    localappdata = os.environ.get("LOCALAPPDATA", "")
    if userprofile:
        pyenv_root = Path(userprofile) / ".pyenv" / "pyenv-win"
        version_file = pyenv_root / "version"
        try:
            if version_file.is_file():
                pyenv_ver = version_file.read_text(encoding="utf-8", errors="replace").strip()
                if pyenv_ver:
                    candidates.append(pyenv_root / "versions" / pyenv_ver / "python.exe")
        except OSError:
            pass
    # Program Files / Program Files (x86) are python.org's default
    # "Install for all users" targets, distinct from the per-user
    # LOCALAPPDATA\Programs\Python default below.
    # Program Files / Program Files (x86) は python.org の既定「全ユーザー
    # 用にインストール」先で、下のユーザー毎既定 LOCALAPPDATA\Programs\Python
    # とは別物。
    for ver in ("313", "312", "311", "310", "39", "38"):
        if localappdata:
            candidates.append(Path(localappdata) / "Programs" / "Python" / f"Python{ver}" / "python.exe")
        candidates.append(Path(f"C:/Python{ver}") / "python.exe")
        candidates.append(Path(f"C:/Program Files/Python{ver}") / "python.exe")
        candidates.append(Path("C:/Program Files (x86)") / f"Python{ver}" / "python.exe")
    if userprofile:
        candidates.append(Path(userprofile) / "scoop" / "apps" / "python" / "current" / "python.exe")
    return candidates


def _py_launcher_python_exe() -> Optional[str]:
    """
    Windows: ask the `py` launcher for the real python.exe it would run,
    so an install that only registered `py` on PATH (not `python`/
    `python3`) is still found. Mirrors scripts/installer.py's
    _py_launcher_python_dir() -- duplicated here (not imported) for the
    same reason as _windows_python_exe_candidates() above: this check runs
    BEFORE the repo (and thus installer.py) is cloned.

    Returns None (never raises) on any failure, including the resolved
    path being the non-functional WindowsApps Store stub.

    Windows: `py` ランチャーに実際に実行する python.exe を問い合わせる。
    `python`/`python3` ではなく `py` だけを PATH に登録したインストール
    でも見つけられるようにする。scripts/installer.py の
    _py_launcher_python_dir() と同じ処理だが、上の
    _windows_python_exe_candidates() と同じ理由(このチェックはリポジトリ
    -- ひいては installer.py -- が clone される前に走る)で import はせず
    複製する。

    いかなる失敗でも(解決先が機能しない WindowsApps ストアスタブである
    場合を含め)None を返す。例外は送出しない。
    """
    py_launcher = shutil.which("py")
    if not py_launcher:
        return None
    try:
        result = subprocess.run(
            [py_launcher, "-3", "-c", "import sys; print(sys.executable)"],
            capture_output=True, timeout=SUBPROCESS_PROBE_TIMEOUT_SECONDS,
            encoding="utf-8", errors="replace",
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    output = result.stdout.strip()
    if not output:
        return None
    exe_path = output.splitlines()[0]
    if "windowsapps" in exe_path.lower() or not Path(exe_path).is_file():
        return None
    return exe_path


def _find_system_python() -> Optional[str]:
    """
    Locate a system Python interpreter -- deliberately NOT this frozen
    app's own bundled interpreter -- for the "system Python 3.8+"
    prerequisite check. Checks the `py` launcher first (highest priority:
    some python.org installs register only `py`, not `python`/`python3`,
    on PATH), then prefers "python3" over "python" since some systems
    still alias "python" to a Python 2 install. On Windows, also scans
    common install locations off PATH so an installed-but-not-on-PATH
    Python is still detected (matching installer.py's discovery).
    「システム Python 3.8+」前提チェックのため、システムの Python
    インタプリタを探す(このアプリ自身に同梱された凍結インタプリタでは
    意図的にない)。最初に `py` ランチャーを確認し(最優先: 一部の
    python.org インストールは `python`/`python3` ではなく `py` だけを
    PATH に登録する)、次に一部の環境で今も Python 2 を指す "python" より
    "python3" を優先する。Windows では PATH 外の一般的なインストール先も
    走査し、PATH に載っていない Python も検出する(installer.py の発見と一致)。
    """
    if sys.platform == "win32":
        py_launcher_exe = _py_launcher_python_exe()
        if py_launcher_exe:
            return py_launcher_exe
    on_path = shutil.which("python3") or shutil.which("python")
    if on_path and "windowsapps" not in on_path.lower():
        return on_path
    if sys.platform == "win32":
        for exe in _windows_python_exe_candidates():
            if exe.is_file():
                return str(exe)
    return on_path


def get_system_python_version_string() -> str:
    """
    Human-readable version string for the system Python found by
    _find_system_python(), or a "not found"/"could not check" message in
    the current UI language (via tr()). Never raises: any failure is
    folded into the returned string so callers can display it directly.
    _find_system_python() が見つけたシステム Python のバージョン文字列
    (人間可読)。見つからない/確認できない場合は、現在のUI言語での
    (tr() 経由の)その旨の文字列を返す。例外は送出しない: 呼び出し側が
    そのまま表示できるよう、失敗も戻り値の文字列に畳み込む。
    """
    python_exe = _find_system_python()
    if not python_exe:
        return tr("python_not_found")
    try:
        result = subprocess.run(
            [python_exe, "--version"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=SUBPROCESS_PROBE_TIMEOUT_SECONDS,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return tr("python_check_failed", exc)
    # "python --version" prints to stdout on Python 3, historically to
    # stderr on Python 2 -- check both so an old system Python still
    # reports a version instead of "unknown".
    # "python --version" は Python 3 では stdout、歴史的に Python 2 では
    # stderr に出力する -- 両方確認することで、古いシステム Python でも
    # "unknown" にならずバージョンを報告できる。
    version_text = (result.stdout or result.stderr).strip()
    return version_text or tr("python_version_unknown")


def check_python_available(min_version: Tuple[int, int] = PYTHON_MIN_VERSION) -> bool:
    """
    Return whether a system Python at or above min_version was found.
    This is a soft prerequisite (see ENV_CHECK_REQUIRED above): this
    app's own bundled interpreter is what actually runs installer.py.
    システム Python が min_version 以上で見つかったか。ソフトな前提条件
    (上の ENV_CHECK_REQUIRED 参照): installer.py を実際に実行するのは
    このアプリ自身に同梱されたインタプリタである。
    """
    match = re.search(r"(\d+)\.(\d+)", get_system_python_version_string())
    if not match:
        return False
    found_version = (int(match.group(1)), int(match.group(2)))
    return found_version >= min_version


def _existing_ancestor(path: Path) -> Path:
    """
    Walk up from path until an existing directory is found. Needed
    because shutil.disk_usage() requires an existing path, but a
    first-time install directory does not exist yet.
    path から親方向へ辿り、実在するディレクトリを見つける。
    shutil.disk_usage() は実在するパスを要求するが、初回インストール先の
    ディレクトリはまだ存在しないため。
    """
    candidate = path.expanduser().resolve()
    while not candidate.exists():
        parent = candidate.parent
        if parent == candidate:  # reached filesystem root / ファイルシステムのルートに到達
            break
        candidate = parent
    return candidate


def get_free_disk_space_gb(path: Path) -> float:
    """Free space, in GiB, on the filesystem containing (an ancestor of) path."""
    # path(の実在する祖先)を含むファイルシステムの空き容量(GiB単位)
    usage = shutil.disk_usage(_existing_ancestor(path))
    return usage.free / BYTES_PER_GIB


def check_disk_space_sufficient(path: Path, minimum_gb: float = DISK_SPACE_MINIMUM_GB) -> bool:
    """Return whether at least minimum_gb GiB is free where path would be created."""
    # path を作成する先に、少なくとも minimum_gb GiB の空きがあるか
    return get_free_disk_space_gb(path) >= minimum_gb


def check_network_reachable(
    host: str = NETWORK_CHECK_HOST,
    port: int = NETWORK_CHECK_PORT,
    timeout: float = NETWORK_CHECK_TIMEOUT_SECONDS,
) -> bool:
    """
    Return whether a TCP connection to host:port succeeds within timeout
    seconds. Used as a rough proxy for "can this machine reach GitHub to
    clone and, later, to let ESP-IDF's own installer download tools".
    host:port への TCP 接続が timeout 秒以内に成功するか。「このマシンが
    GitHub に到達でき、clone や、後で ESP-IDF 自身のインストーラが
    ツールをダウンロードできるか」の簡易な代理指標として使う。
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def install_command_for(commands_by_platform: Dict[str, str]) -> str:
    """
    Pick the install command for sys.platform from a
    {"win32": ..., "darwin": ..., "linux": ...} mapping, falling back to
    the "linux" entry for any other POSIX-ish platform (e.g. *BSD).
    {"win32": ..., "darwin": ..., "linux": ...} の対応表から sys.platform
    向けの導入コマンドを選ぶ。それ以外の POSIX系プラットフォーム
    (*BSD 等)は "linux" の項目にフォールバックする。
    """
    return commands_by_platform.get(sys.platform, commands_by_platform["linux"])


def detect_existing_checkout(install_dir: Path) -> bool:
    """
    Return whether install_dir already holds a git checkout of this
    repo, recognized the same way the Options page does: both
    <install_dir>/.git and <install_dir>/scripts/installer.py exist.
    install_dir に本リポジトリの git checkout が既にあるか。
    オプション画面と同じ判定基準を使う:
    <install_dir>/.git と <install_dir>/scripts/installer.py の両方が存在。
    """
    return (install_dir / ".git").exists() and (install_dir / "scripts" / "installer.py").is_file()


# ---------------------------------------------------------------------------
# In-process installer.py loading
# installer.py のプロセス内読み込み
# ---------------------------------------------------------------------------

# Distinct from this GUI's own module name so a colliding sys.modules
# entry between the two is impossible even though both files could in
# principle be named "stampfly_installer"-ish things.
# このGUI自身のモジュール名と衝突しないよう区別する名前(両ファイルとも
# "stampfly_installer" に近い名前を持ちうるため、sys.modules 上の衝突を
# 構造的に排除する)。
CLONED_INSTALLER_MODULE_NAME = "stampfly_cloned_installer"


def load_installer_module(repo_dir: Path):
    """
    Load <repo_dir>/scripts/installer.py as an in-process module and
    return it. This is the ONLY way this GUI ever runs install logic --
    see the module docstring's "single-source-of-truth design" section.
    Raises InstallError if the file is missing or fails to load/execute
    (a load-time exception, e.g. a syntax error in a future
    installer.py, propagates through exec_module() as-is; the caller
    treats it like any other InstallError-worthy failure).
    <repo_dir>/scripts/installer.py をプロセス内モジュールとして読み込み
    返す。このGUIがインストールロジックを実行する唯一の経路
    (モジュールdocstringの「単一実体の原則」参照)。ファイルが無い、
    または読み込み/実行に失敗した場合は InstallError を送出する
    (将来の installer.py の構文エラー等、読み込み時例外は exec_module()
    からそのまま伝播する。呼び出し側は他の InstallError 相当の失敗と
    同様に扱う)。
    """
    installer_path = Path(repo_dir) / "scripts" / "installer.py"
    if not installer_path.is_file():
        raise InstallError(tr("install_error_installer_not_found", installer_path))
    spec = importlib.util.spec_from_file_location(CLONED_INSTALLER_MODULE_NAME, installer_path)
    if spec is None or spec.loader is None:
        raise InstallError(tr("install_error_installer_load_failed", installer_path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[CLONED_INSTALLER_MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


def default_setup_repo() -> Path:
    """
    Repository root used when SELFTEST_REPO_ENV_VAR is unset: the
    directory 2 levels above this file's own containing directory
    (tools/installer_gui/stampfly_installer.py
     -> tools/installer_gui               (this file's directory)
     -> tools                             (1 level up)
     -> <repo root>                       (2 levels up)).
    Frozen (PyInstaller) execution never reaches this function --
    --selftest is always run from source, in CI, against a real git
    checkout -- so __file__ is always a real filesystem path here.
    SELFTEST_REPO_ENV_VAR 未設定時に使うリポジトリルート:
    このファイル自身の格納ディレクトリから2階層上
    (tools/installer_gui/stampfly_installer.py
     -> tools/installer_gui               (このファイルの格納ディレクトリ)
     -> tools                             (1階層上)
     -> <リポジトリルート>                 (2階層上))。
    凍結(PyInstaller)実行はこの関数に到達しない -- --selftest は常に
    ソースから、CI 上で、実際の git checkout に対して実行されるため、
    ここでの __file__ は常に実在するファイルシステムパスとなる。
    """
    return Path(__file__).resolve().parent.parent.parent


def resolve_setup_repo_for_selftest() -> Path:
    """Resolve the repo root --selftest checks, honoring SELFTEST_REPO_ENV_VAR."""
    # --selftest が検証対象とするリポジトリルートを、SELFTEST_REPO_ENV_VAR を
    # 尊重しつつ解決する
    env_value = os.environ.get(SELFTEST_REPO_ENV_VAR)
    if env_value:
        return Path(env_value).expanduser().resolve()
    return default_setup_repo()


# ---------------------------------------------------------------------------
# Queue-backed stdout/stderr capture
# キュー連動の標準出力/標準エラー捕捉
# ---------------------------------------------------------------------------


class _QueueWriter(io.TextIOBase):
    """
    Write-only text stream that pushes each completed line into a
    thread-safe queue, for use with contextlib.redirect_stdout /
    redirect_stderr while scripts/installer.py's Installer() runs
    in-process on a worker thread.

    Subclassing io.TextIOBase (rather than a bare duck-typed object, as
    tools/flasher_gui/stampfly_flasher.py's QueueLineWriter does) is a
    deliberate choice for this tool: it satisfies
    redirect_stdout/redirect_stderr's expectation of a text-mode file
    object and gives safe, standards-compliant defaults for every method
    we do not override (e.g. isatty() -> False, readable() -> False).

    Both "\\n" and "\\r" are treated as line terminators, mirroring
    QueueLineWriter, in case a future installer.py prints a carriage-
    return progress meter the way esptool does.

    ワーカースレッド上で scripts/installer.py の Installer() を
    インプロセス実行する際、contextlib.redirect_stdout / redirect_stderr
    と組み合わせて使う、完成した行を1行ずつスレッドセーフなキューへ積む
    書き込み専用のテキストストリーム。

    素朴な duck-typing オブジェクト(tools/flasher_gui/stampfly_flasher.py
    の QueueLineWriter はそう)ではなく io.TextIOBase を継承するのは、
    このツールでの意図的な選択: redirect_stdout/redirect_stderr が
    期待するテキストモードのファイルオブジェクトの契約を満たし、
    オーバーライドしない全メソッドに標準準拠の安全な既定動作を持たせる
    (例: isatty() -> False, readable() -> False)。

    "\\n" と "\\r" の両方を行末記号として扱う(QueueLineWriter に倣う)。
    将来の installer.py が esptool のように復帰付き進捗表示を print する
    ようになっても取りこぼさないため。
    """

    def __init__(self, output_queue: "queue.Queue"):
        super().__init__()
        self._queue = output_queue
        self._buffer = ""

    def writable(self) -> bool:
        return True

    def write(self, text: str) -> int:
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

    def flush(self) -> None:
        """Required by the file-like protocol; buffering is handled in write()."""
        # ファイルライクプロトコルの要件として必要。バッファ処理は write() 側で行う

    def flush_remaining(self) -> None:
        """Push any trailing partial line once the redirected call has returned."""
        # リダイレクト対象の呼び出しが戻った時点で残っている末尾の未完成行を送る
        if self._buffer:
            self._queue.put(("log", self._buffer))
            self._buffer = ""


# ---------------------------------------------------------------------------
# Worker-thread workflow (clone + in-process installer.py execution)
# ワーカースレッドの処理(clone + installer.py のインプロセス実行)
# ---------------------------------------------------------------------------


class SetupOptions:
    """
    Plain data holder for one worker-thread run of the setup workflow,
    built from the Options page's widget state at the moment the user
    leaves that page.

    NOTE: there is no `desktop_shortcut` field here. installer.py's
    Installer.run() has no parameter to suppress the desktop shortcut
    specifically -- only `no_flasher`, which skips the GUI Flasher
    entirely. Under non-interactive mode, its own "create a desktop
    shortcut?" prompt always resolves to its default (Yes) regardless.
    The Options page's checkbox is therefore advisory-only until
    installer.py grows a matching parameter -- a known, documented gap,
    not a silently dropped setting.

    ワーカースレッドでの1回のセットアップ実行に使う単純なデータ保持
    クラス。オプション画面のウィジェット状態から、ユーザーがその画面を
    離れる時点で組み立てる。

    注意: `desktop_shortcut` フィールドは存在しない。installer.py の
    Installer.run() には、デスクトップショートカットだけを抑制する引数が
    無い(GUIフラッシャ自体を丸ごとスキップする `no_flasher` のみ)。
    非対話モードでは、自身の「デスクトップショートカットを作る?」prompt
    が常に既定値(Yes)に解決される。オプション画面のチェックボックスは、
    installer.py に対応する引数が追加されるまでは参考表示にとどまる --
    黙って無視しているのではなく、既知のギャップとしてここに明記する。
    """

    def __init__(self, install_dir: Path, action: str, skip_clone: bool, include_flasher: bool, minimal: bool):
        self.install_dir = install_dir
        self.action = action
        self.skip_clone = skip_clone
        self.include_flasher = include_flasher
        self.minimal = minimal


def run_git_clone(
    repo_url: str,
    install_dir: Path,
    output_queue: "queue.Queue",
    cancel_event: threading.Event,
) -> int:
    """
    Clone repo_url into install_dir with --progress, streaming combined
    stdout+stderr line by line into output_queue (git writes its
    progress meter to stderr; merging avoids missing lines). Checked
    after each line, cancel_event being set terminates the subprocess.

    Returns the git process's exit code (0 = success). Never raises for
    a plain non-zero git exit; only OS-level failures to even spawn git
    become an InstallError.

    repo_url を install_dir へ --progress 付きで clone し、標準出力+
    標準エラーを結合して1行ずつ output_queue へ流す(git の進捗表示は
    stderr に出るため、結合しないと取りこぼす)。各行の後に
    cancel_event が立っていればサブプロセスを終了させる。

    git プロセスの終了コードを返す(0=成功)。git が単に非ゼロで終了した
    だけでは例外を送出しない。git の起動自体に失敗するようなOSレベルの
    失敗のみ InstallError にする。
    """
    output_queue.put(("status", tr("clone_status_cloning")))
    try:
        install_dir.parent.mkdir(parents=True, exist_ok=True)
        process = subprocess.Popen(
            ["git", "clone", "--progress", repo_url, str(install_dir)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
    except OSError as exc:
        raise InstallError(tr("git_clone_start_failed", exc))

    assert process.stdout is not None  # guaranteed by stdout=subprocess.PIPE above
    for raw_line in process.stdout:
        line = raw_line.rstrip("\r\n")
        if line:
            output_queue.put(("log", line))
        if cancel_event.is_set():
            process.terminate()
            break
    return process.wait()


def run_git_update(install_dir: Path, output_queue: "queue.Queue") -> None:
    """
    Best-effort `git pull --ff-only` of an existing checkout before repair
    (see the caller in perform_setup_workflow() for the rationale). Output
    is streamed to the log; any failure is reported as a warning line and
    swallowed -- an un-updatable clone must not block the repair attempt.
    修復前に既存チェックアウトを `git pull --ff-only` でベストエフォート
    更新する(理由は呼び出し元 perform_setup_workflow() を参照)。出力は
    ログへ流し、失敗は警告行として報告した上で握りつぶす -- 更新できない
    クローンであっても修復の試行自体は妨げない。
    """
    output_queue.put(("log", tr("repair_updating")))
    try:
        result = subprocess.run(
            ["git", "-C", str(install_dir), "pull", "--ff-only"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except OSError as exc:
        output_queue.put(("log", tr("repair_update_error", exc)))
        return
    for line in (result.stdout + result.stderr).splitlines():
        if line.strip():
            output_queue.put(("log", line))
    if result.returncode != 0:
        output_queue.put(("log", tr("repair_update_not_ff")))


def run_installer_in_process(module, output_queue: "queue.Queue", action_fn) -> int:
    """
    Run action_fn() (a zero-argument callable that invokes something on
    module.Installer(), e.g. .run(...) or .uninstall()) with the
    non-interactive contract established and stdout/stderr captured.

    Contract (docs/plans/gui-installer-plan.md §1 非対話化の契約):
      1. module.Colors.disable() -- no ANSI escapes in the captured log.
      2. os.environ[INSTALLER_NONINTERACTIVE_ENV_VAR] = "1" -- installer.py's
         own prompt()/prompt_choice() see this and return their default
         immediately (installer.py also stays EOF-safe on its own if an
         older, env-var-unaware version is ever cloned -- see the module
         docstring's "Non-interactive contract" note and
         run_selftest()'s check (3)).
      3. contextlib.redirect_stdout/redirect_stderr into a _QueueWriter,
         so every print() from installer.py becomes a "log" queue message.

    Returns the resolved return code (0 = success). Never raises:
    SystemExit and any other exception are translated into a non-zero
    return code plus a traceback pushed to the log, matching the "例外は
    traceback ごとログへ" requirement.

    action_fn()(module.Installer() の .run(...) や .uninstall() 等を
    呼ぶ、引数無しの callable)を、非対話化契約を整えた上で
    stdout/stderr を捕捉しながら実行する。

    契約(docs/plans/gui-installer-plan.md §1 非対話化の契約):
      1. module.Colors.disable() -- 捕捉するログに ANSI エスケープを
         残さない。
      2. os.environ[INSTALLER_NONINTERACTIVE_ENV_VAR] = "1" --
         installer.py 自身の prompt()/prompt_choice() がこれを見て
         即座に既定値を返す(古い、環境変数を知らない版が clone された
         場合でも installer.py 自身が EOF セーフに振る舞う -- モジュール
         docstring の「Non-interactive contract」注記と
         run_selftest() のチェック(3)を参照)。
      3. contextlib.redirect_stdout/redirect_stderr を _QueueWriter へ
         向け、installer.py の print() が全て "log" キューメッセージに
         なるようにする。

    解決した終了コードを返す(0=成功)。例外は送出しない: SystemExit や
    その他の例外は、非ゼロの終了コード + traceback をログへ積む形へ
    変換する(「例外は traceback ごとログへ」の要件に対応)。
    """
    module.Colors.disable()
    os.environ[INSTALLER_NONINTERACTIVE_ENV_VAR] = "1"
    writer = _QueueWriter(output_queue)
    try:
        with contextlib.redirect_stdout(writer), contextlib.redirect_stderr(writer):
            return action_fn()
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 1
        writer.write("installer.py exited via SystemExit (code: {0})\n".format(code))
        return code
    except Exception:
        writer.write(traceback.format_exc())
        return 1
    finally:
        writer.flush_remaining()


def build_supported_run_kwargs(module, options: SetupOptions,
                               output_queue: "queue.Queue") -> Dict[str, object]:
    """
    Build the Installer.run() keyword arguments, dropping any that the
    loaded installer.py's signature does not accept.

    Why: repair mode executes the TARGET checkout's installer.py, which
    can predate newer run() parameters -- observed in the wild on
    2026-07-20 (Windows, v2026.07.2 Setup + a pre-workshop clone):
    `TypeError: Installer.run() got an unexpected keyword argument
    'no_flasher'`. The pull-first step in perform_setup_workflow()
    normally prevents this by updating the clone, but when that pull is
    impossible (local changes, detached state) this filter keeps the old
    installer callable instead of crashing; dropped options are logged so
    the user knows they were ignored.

    Installer.run() のキーワード引数を組み立て、読み込んだ installer.py の
    シグネチャが受け付けないものは落とす。

    理由: 修復モードは対象チェックアウトの installer.py を実行するため、
    新しい run() 引数の追加より古い版でありうる -- 2026-07-20 に実地で
    観測(Windows、v2026.07.2 Setup + 講習前のクローン):
    `TypeError: Installer.run() got an unexpected keyword argument
    'no_flasher'`。通常は perform_setup_workflow() の pull 先行ステップが
    クローンを最新化してこれを防ぐが、pull できない場合(ローカル変更・
    detached 状態)でも、クラッシュせず旧 installer を呼べるようにする。
    落とした引数はログに出し、無視されたことをユーザーに知らせる。
    """
    desired: Dict[str, object] = {
        "idf_path": None,
        "skip_deps": False,
        "minimal": options.minimal,
        "force": False,
        "no_flasher": not options.include_flasher,
    }
    try:
        supported_names = set(inspect.signature(module.Installer.run).parameters)
    except (TypeError, ValueError):
        # Unintrospectable signature (should not happen for plain Python):
        # pass everything and let any mismatch surface as before.
        # イントロスペクション不能(素のPythonでは起きないはず): 全て渡し、
        # 不一致は従来どおり表面化させる。
        return desired
    kwargs = {name: value for name, value in desired.items() if name in supported_names}
    dropped = sorted(set(desired) - set(kwargs))
    if dropped:
        output_queue.put(("log", tr("old_installer_options_ignored", ", ".join(dropped))))
    return kwargs


def run_installer_setup(module, options: SetupOptions, output_queue: "queue.Queue") -> int:
    """
    Call module.Installer().run(...) with the fixed, non-interactive
    argument shape mandated by docs/plans/gui-installer-plan.md §2:
    idf_path is always None (ESP-IDF discovery/confirmation is handled
    entirely by installer.py itself under the non-interactive contract).
    Arguments unknown to an older installer.py are dropped -- see
    build_supported_run_kwargs().
    docs/plans/gui-installer-plan.md §2 が定める固定・非対話の引数形で
    module.Installer().run(...) を呼ぶ: idf_path は常に None
    (ESP-IDF の検出/確認は、非対話化契約の下で installer.py 自身が
    完結して処理する)。古い installer.py が知らない引数は落とす --
    build_supported_run_kwargs() 参照。
    """
    run_kwargs = build_supported_run_kwargs(module, options, output_queue)
    return run_installer_in_process(
        module,
        output_queue,
        lambda: module.Installer().run(**run_kwargs),
    )


def run_installer_uninstall(module, output_queue: "queue.Queue") -> int:
    """Call module.Installer().uninstall() under the same contract as run_installer_setup()."""
    # run_installer_setup() と同じ契約の下で module.Installer().uninstall() を呼ぶ
    return run_installer_in_process(module, output_queue, lambda: module.Installer().uninstall())


def perform_setup_workflow(output_queue: "queue.Queue", options: SetupOptions, cancel_event: threading.Event) -> int:
    """
    Run the whole workflow on a worker thread: clone (unless
    options.skip_clone, i.e. an existing checkout was detected), then
    load and run installer.py in-process. Sends ("phase", PHASE_CLONE)
    before cloning and ("phase", PHASE_INSTALL) once installer.py takes
    over, so the GUI can enable Cancel only for the clone phase per
    docs/plans/gui-installer-plan.md §5 (installer.run() itself is never
    force-interrupted).

    Returns the resolved return code (0 = success). Never raises:
    InstallError and any other exception are logged and turned into a
    non-zero return code.

    ワーカースレッド上で全工程を実行する: clone する(options.skip_clone
    が真、すなわち既存 checkout を検出した場合を除く)、その後
    installer.py をプロセス内で読み込み実行する。clone 前に
    ("phase", PHASE_CLONE) を、installer.py に処理が移った時点で
    ("phase", PHASE_INSTALL) を送ることで、
    docs/plans/gui-installer-plan.md §5 の通り、GUI 側でキャンセルを
    clone フェーズのみ有効にできる(installer.run() 自体を強制中断する
    ことは無い)。

    解決した終了コードを返す(0=成功)。例外は送出しない: InstallError や
    その他の例外はログへ記録し、非ゼロの終了コードに変換する。
    """
    try:
        if not options.skip_clone:
            output_queue.put(("phase", PHASE_CLONE))
            clone_return_code = run_git_clone(REPO_URL, options.install_dir, output_queue, cancel_event)
            if clone_return_code != 0:
                if cancel_event.is_set():
                    output_queue.put(("log", tr("clone_cancelled")))
                return clone_return_code
        else:
            output_queue.put(("log", tr("clone_reuse_existing", options.install_dir)))
            # Repair runs the TARGET checkout's installer.py, so bring that
            # checkout up to date first -- a stale clone may predate the
            # non-interactive contract or the current run() signature (the
            # 2026-07-20 Windows failure: TypeError on no_flasher). ff-only,
            # never destructive; failure (local changes, diverged history,
            # remote unreachable) is logged and repair continues with the
            # code that is there, protected by build_supported_run_kwargs().
            # 修復は対象チェックアウトの installer.py を実行するため、先に
            # そのチェックアウトを最新化する -- 古いクローンは非対話契約や
            # 現行 run() シグネチャより前でありうる(2026-07-20 の Windows
            # での TypeError 失敗)。ff-only で破壊的操作はしない。失敗
            # (ローカル変更・履歴分岐・リモート不達)はログに残して既存
            # コードのまま続行し、build_supported_run_kwargs() が保護する。
            output_queue.put(("phase", PHASE_CLONE))
            run_git_update(options.install_dir, output_queue)

        output_queue.put(("phase", PHASE_INSTALL))
        module = load_installer_module(options.install_dir)

        if options.action == ACTION_UNINSTALL:
            return run_installer_uninstall(module, output_queue)
        return run_installer_setup(module, options, output_queue)
    except InstallError as exc:
        output_queue.put(("log", str(exc)))
        return 1
    except Exception:
        output_queue.put(("log", traceback.format_exc()))
        return 1


# ---------------------------------------------------------------------------
# Self-test (headless, CI-friendly, no Tk, no real clone/install)
# セルフテスト(ヘッドレス・CI向け・Tk不使用・実際の clone/install は行わない)
# ---------------------------------------------------------------------------


def run_selftest() -> int:
    """
    Headless, CI-friendly self-check against a real (already-checked-out)
    repository -- SELFTEST_REPO_ENV_VAR if set, else default_setup_repo().
    Never instantiates Tk, never clones anything, and never actually
    calls Installer().run()/uninstall() (which would perform a real
    install/uninstall) -- it only verifies the CONTRACT this GUI depends
    on:

      (1) scripts/installer.py can be imported in-process.
      (2) module.Installer exists with callable run/uninstall/clean.
      (3) SF_INSTALLER_NONINTERACTIVE=1 makes module.prompt() return its
          default immediately (stdin is also pointed at an already-
          exhausted stream during this one call, as a belt-and-braces
          EOF-safety net -- see docs/plans/gui-installer-plan.md §1).
      (4) installer.py's source has exactly 4 "Step N/4:" headers (the
          step-indicator contract, see STEP_HEADER_PATTERN above).
      (5) this GUI's own prerequisite probe functions each return a
          plain bool.
      (6) STRINGS has both a ja and an en entry for every key (i18n
          completeness -- see the Internationalization section above).
      (7) tr() actually resolves a different string per language after
          set_language(LANG_EN)/set_language(LANG_JA), and the language
          state is restored to what it was on entry once the check is
          done.

    Returns 0 for PASS, 1 for FAIL, matching standard process exit-code
    conventions so CI can gate on it directly.

    実在する(既に checkout 済みの)リポジトリ -- 設定されていれば
    SELFTEST_REPO_ENV_VAR、無ければ default_setup_repo() -- に対する
    ヘッドレスでCI向けのセルフチェック。Tk のインスタンス化は一切
    行わず、何も clone せず、Installer().run()/uninstall()(実際の
    インストール/アンインストールを行ってしまう)も一切呼ばない --
    このGUIが依存する契約のみを検証する:

      (1) scripts/installer.py をプロセス内 import できる。
      (2) module.Installer が存在し、run/uninstall/clean が呼び出し可能。
      (3) SF_INSTALLER_NONINTERACTIVE=1 で module.prompt() が即座に
          既定値を返す(この1回の呼び出し中だけ stdin も既に読み尽くした
          ストリームへ向け、念のための EOF セーフティネットとする --
          docs/plans/gui-installer-plan.md §1 参照)。
      (4) installer.py のソースに "Step N/4:" ヘッダがちょうど4つある
          (ステップインジケータ契約。上の STEP_HEADER_PATTERN 参照)。
      (5) このGUI自身の前提条件プローブ関数群が、それぞれ素の bool を
          返す。
      (6) STRINGS の全キーが ja と en の両方のエントリを持つ(i18n の
          網羅性 -- 上の Internationalization 節参照)。
      (7) set_language(LANG_EN)/set_language(LANG_JA) の後で tr() が
          言語ごとに実際に異なる文字列を返し、かつチェック完了後に
          言語状態が呼び出し時点の状態へ復元されていること。

    PASS なら 0、FAIL なら 1 を返す。プロセス終了コードの標準的な慣習に
    合わせているため、CI が直接ゲート条件として使える。
    """
    print("=== {0} self-test / セルフテスト ===".format(APP_NAME))

    all_checks_passed = True

    def report(label: str, passed: bool) -> bool:
        nonlocal all_checks_passed
        print("[{0}] {1}".format("PASS" if passed else "FAIL", label))
        if not passed:
            all_checks_passed = False
        return passed

    repo_root = resolve_setup_repo_for_selftest()
    print("Repository root: {0}".format(repo_root))

    # (1) In-process import.
    try:
        module = load_installer_module(repo_root)
    except Exception as exc:
        report("scripts/installer.py imported in-process", False)
        print("  {0}".format(exc))
        print("=== FAIL ===")
        return 1
    report("scripts/installer.py imported in-process", True)

    # (2) Installer class + run/uninstall/clean callable.
    installer_cls = getattr(module, "Installer", None)
    report("Installer class present", installer_cls is not None)
    for method_name in ("run", "uninstall", "clean"):
        method = getattr(installer_cls, method_name, None) if installer_cls is not None else None
        report("Installer.{0} is callable".format(method_name), callable(method))

    # (3) SF_INSTALLER_NONINTERACTIVE contract, with a belt-and-braces
    # EOF-safety net on stdin so this can never hang even if a stale
    # installer.py (predating the env-var check) is under test.
    # SF_INSTALLER_NONINTERACTIVE 契約。テスト対象が(環境変数チェック導入前の)
    # 古い installer.py であってもハングしないよう、stdin にも念のための
    # EOF セーフティネットを張る。
    original_noninteractive = os.environ.get(INSTALLER_NONINTERACTIVE_ENV_VAR)
    original_stdin = sys.stdin
    os.environ[INSTALLER_NONINTERACTIVE_ENV_VAR] = "1"
    sys.stdin = io.StringIO("")
    try:
        prompt_response = module.prompt("x [Y/n]", "Y")
    finally:
        sys.stdin = original_stdin
        if original_noninteractive is None:
            os.environ.pop(INSTALLER_NONINTERACTIVE_ENV_VAR, None)
        else:
            os.environ[INSTALLER_NONINTERACTIVE_ENV_VAR] = original_noninteractive
    report(
        "prompt() returns its default immediately under {0}=1".format(INSTALLER_NONINTERACTIVE_ENV_VAR),
        prompt_response == "Y",
    )

    # (4) Exactly 4 "Step N/4:" header() call sites in installer.py's
    # source (see STEP_HEADER_CALL_PATTERN's comment for why this is not
    # simply STEP_HEADER_PATTERN.findall() over the whole file).
    installer_source = (repo_root / "scripts" / "installer.py").read_text(encoding="utf-8")
    step_header_calls_found = STEP_HEADER_CALL_PATTERN.findall(installer_source)
    report(
        "installer.py has exactly 4 'Step N/4:' header() calls (found {0})".format(len(step_header_calls_found)),
        len(step_header_calls_found) == 4,
    )

    # (5) Prerequisite probes each return a plain bool.
    disk_check_path = _existing_ancestor(DEFAULT_INSTALL_DIR)
    probe_calls = [
        ("check_git_available", check_git_available),
        ("check_python_available", check_python_available),
        ("check_disk_space_sufficient", lambda: check_disk_space_sufficient(disk_check_path)),
        ("check_network_reachable", check_network_reachable),
    ]
    for probe_name, probe_fn in probe_calls:
        result = probe_fn()
        report("{0}() returns bool (got {1!r})".format(probe_name, result), isinstance(result, bool))

    # (5b) Signature-tolerant run() kwargs: an older installer.py whose
    # Installer.run() predates newer parameters must not crash repair mode
    # with a TypeError -- regression check for the 2026-07-20 Windows
    # failure ("unexpected keyword argument 'no_flasher'" from a
    # pre-workshop clone). A synthetic old-style class is used so this
    # check needs no git history (CI checkouts are shallow).
    # (5b) シグネチャ耐性のある run() 引数: 新しい引数の追加より古い
    # installer.py の Installer.run() が、修復モードを TypeError で
    # 落とさないこと -- 2026-07-20 の Windows での失敗(講習前クローンでの
    # "unexpected keyword argument 'no_flasher'")の退行チェック。git 履歴に
    # 依存しないよう(CI のチェックアウトは shallow)、合成の旧式クラスを使う。
    class _OldStyleInstaller:
        def run(self, idf_path=None, skip_deps=False, minimal=False, force=False):
            return 0

    _old_module = types.SimpleNamespace(Installer=_OldStyleInstaller)
    _probe_options = SetupOptions(
        install_dir=Path("."), action=ACTION_INSTALL, skip_clone=True,
        include_flasher=True, minimal=False,
    )
    _old_kwargs = build_supported_run_kwargs(_old_module, _probe_options, queue.Queue())
    report(
        "old-signature Installer.run(): unsupported kwargs dropped (got {0})".format(sorted(_old_kwargs)),
        "no_flasher" not in _old_kwargs and "minimal" in _old_kwargs,
    )
    _current_kwargs = build_supported_run_kwargs(module, _probe_options, queue.Queue())
    report(
        "current Installer.run(): all kwargs supported",
        set(_current_kwargs) == {"idf_path", "skip_deps", "minimal", "force", "no_flasher"},
    )

    # (6) i18n completeness: every STRINGS key has both a ja and an en
    # entry, so tr() can never KeyError partway through a language switch.
    # i18n の網羅性: STRINGS の全キーが ja/en 両方のエントリを持つこと。
    # そうでなければ、言語切替の途中で tr() が KeyError しうる。
    incomplete_keys = sorted(
        key for key, translations in STRINGS.items()
        if LANG_JA not in translations or LANG_EN not in translations
    )
    report(
        "STRINGS has both ja and en for all {0} keys ({1} incomplete)".format(len(STRINGS), len(incomplete_keys)),
        not incomplete_keys,
    )
    if incomplete_keys:
        print("  incomplete keys: {0}".format(incomplete_keys))

    # (7) tr() actually resolves a different string per language after
    # set_language(), and this check leaves the module-wide language state
    # (see get_language()/set_language()) exactly as it found it -- this
    # self-test must not have a lingering side effect on whatever runs
    # after it in the same process.
    # tr() が set_language() の後で言語ごとに異なる文字列を実際に返す
    # こと、かつこのチェックがモジュール全体の言語状態
    # (get_language()/set_language() 参照)を、入った時点のまま確実に
    # 復元すること -- このセルフテストが、同一プロセス内でその後に
    # 走る何かへ副作用を残してはならない。
    original_language = get_language()
    try:
        set_language(LANG_EN)
        english_sample = tr("nav_next")
        set_language(LANG_JA)
        japanese_sample = tr("nav_next")
    finally:
        set_language(original_language)
    report(
        "tr() resolves a different string per language (en={0!r} vs ja={1!r})".format(
            english_sample, japanese_sample
        ),
        english_sample != japanese_sample,
    )
    report(
        "language state restored to {0!r} after the check above".format(original_language),
        get_language() == original_language,
    )

    print("=== {0} ===".format("PASS" if all_checks_passed else "FAIL"))
    return 0 if all_checks_passed else 1


# ---------------------------------------------------------------------------
# GUI
# GUI
# ---------------------------------------------------------------------------


class StampFlySetupApp:
    """
    The Tk wizard. Mirrors tools/flasher_gui/stampfly_flasher.py's
    StampFlyFlasherApp threading discipline: all widget access happens on
    the Tk main thread; background work (git clone, in-process
    installer.py execution) always runs on a single worker thread and
    reports back exclusively by pushing messages onto self._queue;
    _poll_queue(), driven by root.after(), is the only place worker
    results are read on the main thread.

    Tk のウィザード本体。tools/flasher_gui/stampfly_flasher.py の
    StampFlyFlasherApp と同じスレッド規律に従う: ウィジェットへの
    アクセスは全て Tk のメインスレッドで行う。バックグラウンド作業
    (git clone・installer.py のインプロセス実行)は常に単一のワーカー
    スレッドで実行し、結果は self._queue へメッセージを積むことでのみ
    報告する。root.after() で駆動される _poll_queue() だけが、メイン
    スレッド側でワーカーの結果を読み出す場所となる。
    """

    def __init__(self, root):
        self.root = root
        self.root.title(APP_NAME)
        self.root.geometry("{0}x{1}".format(WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX))
        self.root.minsize(WINDOW_WIDTH_PX, WINDOW_MIN_HEIGHT_PX)

        # Thread-safe channel from worker threads to the main thread.
        # ワーカースレッドからメインスレッドへのスレッドセーフな通信路。
        self._queue = queue.Queue()
        self._busy = False
        self._cancel_event = threading.Event()
        self._current_page = None
        self._result_success = None
        self._last_action = ACTION_INSTALL

        self._install_dir_var = tk.StringVar(value=str(DEFAULT_INSTALL_DIR))
        self._include_flasher_var = tk.BooleanVar(value=True)
        self._desktop_shortcut_var = tk.BooleanVar(value=True)  # see SetupOptions docstring re: current limits
        self._minimal_var = tk.BooleanVar(value=False)
        self._existing_mode_var = tk.StringVar(value=EXISTING_CHECKOUT_MODE_REPAIR)
        # Keep the primary button's label honest when the user flips the
        # repair/uninstall radio on the Options page.
        # オプション画面で修復/アンインストールのラジオを切り替えたとき、
        # 主ボタンのラベルが実際の動作と食い違わないよう追従させる。
        self._existing_mode_var.trace_add("write", self._on_existing_mode_changed)
        # Backs the Welcome page's language switcher (see
        # _build_page_welcome()/_on_language_changed()). Like every other
        # tk Variable here, this is created once in __init__ and survives
        # _build_shell() being called again on a language switch, so the
        # radio selection itself never needs to be re-derived after a
        # rebuild.
        # ようこそ画面の言語スイッチャー(_build_page_welcome()/
        # _on_language_changed() 参照)の状態を保持する。他の tk 変数と
        # 同様に __init__ で一度だけ作り、言語切替による _build_shell()
        # の再実行をまたいでも生き残るため、再構築後にラジオの選択状態を
        # 作り直す必要が無い。
        self._language_var = tk.StringVar(value=get_language())
        self._language_var.trace_add("write", self._on_language_changed)
        self._status_var = tk.StringVar(value="")
        self._env_status_vars = {name: tk.StringVar(value="") for name in ENV_CHECK_ORDER}
        self._done_title_var = tk.StringVar(value="")
        self._done_body_var = tk.StringVar(value="")

        self._pages = {}
        self._env_rows = {}
        self._step_labels = []
        self._build_shell()
        self._show_page(PAGE_WELCOME)
        self.root.after(QUEUE_POLL_INTERVAL_MS, self._poll_queue)

    # -- shell / navigation --------------------------------------------------

    def _build_shell(self):
        """Build the page host + persistent bottom navigation bar."""
        # ページ表示領域 + 常設の下部ナビゲーションバーを構築する
        container = ttk.Frame(self.root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        self._page_host = ttk.Frame(container)
        self._page_host.grid(row=0, column=0, sticky="nsew")
        self._page_host.rowconfigure(0, weight=1)
        self._page_host.columnconfigure(0, weight=1)

        self._pages[PAGE_WELCOME] = self._build_page_welcome(self._page_host)
        self._pages[PAGE_ENV_CHECK] = self._build_page_env_check(self._page_host)
        self._pages[PAGE_OPTIONS] = self._build_page_options(self._page_host)
        self._pages[PAGE_EXECUTE] = self._build_page_execute(self._page_host)
        self._pages[PAGE_DONE] = self._build_page_done(self._page_host)
        for page_frame in self._pages.values():
            page_frame.grid(row=0, column=0, sticky="nsew")

        self._build_nav_bar(container, row=1)

    def _on_language_changed(self, *_args):
        """
        Handle the Welcome page's language switcher (see
        _build_page_welcome()). Tears down every page and the nav bar,
        flips the module-wide current language (see set_language()), then
        rebuilds the whole shell via _build_shell() and returns to the
        page the user was on -- in practice always the Welcome page,
        since the switcher only lives there (see the module docstring's
        Language switching section). The underlying tk.StringVar/
        tk.BooleanVar state (install dir, checkboxes, etc.) is untouched
        by this and so survives the rebuild.
        ようこそ画面の言語スイッチャー(_build_page_welcome() 参照)を
        処理する。全ページとナビバーを破棄し、モジュール全体の現在言語
        (set_language() 参照)を切り替えた上で、_build_shell() でシェル
        全体を再構築し、ユーザーが元いた画面へ戻す -- スイッチャーが
        ようこそ画面にしか無いため、実質常にようこそ画面(モジュール
        docstringのLanguage switching節参照)。インストール先や
        チェックボックス等を保持する tk.StringVar/tk.BooleanVar の状態は
        これによって触れられないため、再構築をまたいでそのまま残る。
        """
        selected_language = self._language_var.get()
        if selected_language == get_language():
            return
        set_language(selected_language)

        previous_page = self._current_page
        for child in self.root.winfo_children():
            child.destroy()
        self._pages = {}
        self._env_rows = {}
        self._step_labels = []
        self._build_shell()
        self._show_page(previous_page or PAGE_WELCOME)

    def _build_nav_bar(self, parent, row):
        bar = ttk.Frame(parent)
        bar.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        bar.columnconfigure(1, weight=1)

        self.back_button = ttk.Button(bar, text=tr("nav_back"), command=self._on_back_clicked)
        self.back_button.grid(row=0, column=0, sticky="w")

        # Repurposed per page: "再チェック/Recheck" on Env Check,
        # "キャンセル/Cancel" on Execute, "ログを保存/Save log" on Done.
        # 画面ごとに用途を切り替える: 環境チェックでは「再チェック」、
        # 実行画面では「キャンセル」、完了画面では「ログを保存」。
        self.secondary_button = ttk.Button(bar, text="")
        self.secondary_button.grid(row=0, column=1, sticky="w", padx=(10, 0))

        self.primary_button = ttk.Button(bar, text=tr("nav_next"), command=self._on_primary_clicked)
        self.primary_button.grid(row=0, column=2, sticky="e")

    def _show_page(self, name):
        self._current_page = name
        self._pages[name].tkraise()
        self._configure_nav_for(name)
        if name == PAGE_ENV_CHECK:
            self._run_env_checks_async()
        elif name == PAGE_OPTIONS:
            self._refresh_existing_checkout_state()
        elif name == PAGE_EXECUTE:
            self._start_setup_worker()
        elif name == PAGE_DONE:
            self._render_done_page()

    def _configure_nav_for(self, name):
        """Show/hide/relabel the shared nav buttons for the page being entered."""
        # 遷移先の画面に合わせて、共有ナビゲーションボタンの表示/非表示・
        # ラベルを切り替える
        if name == PAGE_WELCOME:
            self.back_button.grid_remove()
            self.secondary_button.grid_remove()
            self.primary_button.grid()
            self.primary_button.configure(text=tr("nav_next"), state="normal")
        elif name == PAGE_ENV_CHECK:
            self.back_button.grid()
            self.back_button.configure(state="normal")
            self.secondary_button.grid()
            self.secondary_button.configure(
                text=tr("nav_recheck"), command=self._run_env_checks_async, state="normal"
            )
            self.primary_button.grid()
            self.primary_button.configure(text=tr("nav_next"), state="normal")
        elif name == PAGE_OPTIONS:
            self.back_button.grid()
            self.back_button.configure(state="normal")
            self.secondary_button.grid_remove()
            self.primary_button.grid()
            self.primary_button.configure(text=self._options_primary_label(), state="normal")
        elif name == PAGE_EXECUTE:
            self.back_button.grid_remove()
            self.secondary_button.grid()
            self.secondary_button.configure(
                text=tr("nav_cancel"), command=self._on_cancel_clicked, state="disabled"
            )
            self.primary_button.grid_remove()
        elif name == PAGE_DONE:
            self.back_button.grid_remove()
            self.secondary_button.grid()
            self.secondary_button.configure(
                text=tr("nav_save_log"), command=self._on_save_log_clicked, state="normal"
            )
            self.primary_button.grid()
            self.primary_button.configure(text=tr("nav_close"), command=self.root.destroy, state="normal")

    def _options_primary_label(self) -> str:
        """Primary-button label for the Options page, tracking the action.
        オプション画面の主ボタンのラベル（選択中の動作に追従）。"""
        install_dir = Path(self._install_dir_var.get()).expanduser()
        if (detect_existing_checkout(install_dir)
                and self._existing_mode_var.get() == EXISTING_CHECKOUT_MODE_UNINSTALL):
            return tr("nav_start_uninstall")
        return tr("nav_start_install")

    def _on_existing_mode_changed(self, *_args):
        if self._current_page == PAGE_OPTIONS:
            self.primary_button.configure(text=self._options_primary_label())

    def _on_back_clicked(self):
        current_index = PAGE_ORDER.index(self._current_page)
        if current_index > 0:
            self._show_page(PAGE_ORDER[current_index - 1])

    def _on_primary_clicked(self):
        if self._current_page == PAGE_WELCOME:
            self._show_page(PAGE_ENV_CHECK)
        elif self._current_page == PAGE_ENV_CHECK:
            self._show_page(PAGE_OPTIONS)
        elif self._current_page == PAGE_OPTIONS:
            self._show_page(PAGE_EXECUTE)

    # -- page 1: welcome ------------------------------------------------------

    def _build_page_welcome(self, parent):
        frame = ttk.Frame(parent, padding=10)

        # Title row: app name on the left, language switcher on the right.
        # This is the ONLY page the switcher appears on -- see
        # _on_language_changed()'s docstring and the module docstring's
        # Language switching section for why that is enough.
        # タイトル行: 左にアプリ名、右に言語スイッチャー。スイッチャーが
        # 表示されるのはこの画面だけ -- その理由は _on_language_changed()
        # のdocstringとモジュールdocstringのLanguage switching節を参照。
        title_row = ttk.Frame(frame)
        title_row.pack(fill="x")
        title_row.columnconfigure(0, weight=1)
        ttk.Label(title_row, text=APP_NAME, font=("TkDefaultFont", 16, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        language_switch_row = ttk.Frame(title_row)
        language_switch_row.grid(row=0, column=1, sticky="e")
        # These two labels are deliberately NOT run through tr(): a
        # language switcher names each option in that option's OWN
        # language ("日本語"/"English"), the same regardless of which
        # language happens to be selected right now.
        # この2つのラベルはあえて tr() を通さない: 言語スイッチャーは
        # 現在どちらの言語が選ばれていても、各選択肢を「その選択肢自身の
        # 言語」で表記する(「日本語」/「English」)。
        ttk.Radiobutton(
            language_switch_row, text="日本語", variable=self._language_var, value=LANG_JA
        ).pack(side="left")
        ttk.Radiobutton(
            language_switch_row, text="English", variable=self._language_var, value=LANG_EN
        ).pack(side="left", padx=(8, 0))

        ttk.Label(
            frame,
            text=tr("welcome_intro"),
            wraplength=HINT_TEXT_WRAP_PX,
            justify="left",
        ).pack(anchor="w", pady=(10, 0))
        ttk.Label(
            frame,
            text=tr("welcome_estimate"),
            foreground=NOTE_TEXT_COLOR,
            wraplength=HINT_TEXT_WRAP_PX,
            justify="left",
        ).pack(anchor="w", pady=(10, 0))
        ttk.Label(
            frame,
            text=tr("welcome_repository", REPO_URL),
            foreground=NOTE_TEXT_COLOR,
        ).pack(anchor="w", pady=(10, 0))
        return frame

    # -- page 2: environment check --------------------------------------------

    def _build_page_env_check(self, parent):
        frame = ttk.Frame(parent, padding=10)
        frame.columnconfigure(1, weight=1)
        ttk.Label(
            frame, text=tr("env_check_title"), font=("TkDefaultFont", 12, "bold")
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        for row_index, name in enumerate(ENV_CHECK_ORDER, start=1):
            ttk.Label(frame, text=env_check_label(name), wraplength=280, justify="left").grid(
                row=row_index, column=0, sticky="w", pady=4
            )
            status_label = ttk.Label(frame, textvariable=self._env_status_vars[name])
            status_label.grid(row=row_index, column=1, sticky="w", padx=(10, 10))
            copy_button = ttk.Button(
                frame,
                text=tr("env_check_copy_command"),
                command=lambda check_name=name: self._on_copy_install_command(check_name),
            )
            copy_button.grid(row=row_index, column=2, sticky="w")
            copy_button.grid_remove()
            self._env_rows[name] = (status_label, copy_button)

        return frame

    def _run_env_checks_async(self):
        """Kick off all 4 prerequisite probes on a worker thread (network can take up to 3s)."""
        # 4種の前提条件プローブをワーカースレッドで開始する(ネットワーク確認は最大3秒かかりうる)
        for name in ENV_CHECK_ORDER:
            self._env_status_vars[name].set(tr("env_check_checking"))
            self._env_rows[name][0].configure(foreground="")
            self._env_rows[name][1].grid_remove()
        self.primary_button.configure(state="disabled")
        self._status_var.set("")

        install_dir = Path(self._install_dir_var.get()).expanduser()

        def worker():
            # Guarded with a broad except: an unhandled exception here would
            # run silently on this daemon thread (nothing surfaces it to the
            # user) and the Environment Check page would stay stuck on
            # "Checking..." forever, since _handle_env_check_result() is the
            # only path that re-enables the Next button. Always push a
            # result -- all rows NG plus the error text -- so the UI can
            # recover instead of hanging.
            # 広めの except で保護する: ここで例外が捕捉されないとこの
            # デーモンスレッド上で黙って落ち、ユーザーには何も表示されない
            # まま「確認中...」に画面が固まる(Next ボタンを再度有効化する
            # 経路は _handle_env_check_result() のみのため)。必ず結果を
            # 積む(全行NG + エラー内容)ことで、固まる代わりに画面が
            # 復帰できるようにする。
            try:
                results = {
                    ENV_CHECK_GIT: check_git_available(),
                    ENV_CHECK_PYTHON: check_python_available(),
                    ENV_CHECK_DISK: check_disk_space_sufficient(install_dir),
                    ENV_CHECK_NETWORK: check_network_reachable(),
                }
                details = {
                    ENV_CHECK_PYTHON: get_system_python_version_string(),
                    ENV_CHECK_DISK: tr("env_check_disk_free_detail", get_free_disk_space_gb(install_dir)),
                }
            except Exception as exc:
                error_text = tr("env_check_worker_error", exc)
                results = {name: False for name in ENV_CHECK_ORDER}
                details = {name: error_text for name in ENV_CHECK_ORDER}
            self._queue.put(("env_check_result", results, details))

        threading.Thread(target=worker, daemon=True).start()

    def _handle_env_check_result(self, results, details):
        for name in ENV_CHECK_ORDER:
            is_ok = results[name]
            status_label, copy_button = self._env_rows[name]
            extra = " ({0})".format(details[name]) if name in details else ""
            status_label.configure(foreground=OK_TEXT_COLOR if is_ok else WARN_TEXT_COLOR)
            self._env_status_vars[name].set(("OK" if is_ok else "NG") + extra)
            if not is_ok and name in ENV_CHECK_INSTALL_COMMANDS:
                copy_button.grid()
            else:
                copy_button.grid_remove()

        can_proceed = all(results[name] for name in ENV_CHECK_REQUIRED)
        self.primary_button.configure(state="normal" if can_proceed else "disabled")
        if not can_proceed:
            missing_labels = [env_check_label(name) for name in ENV_CHECK_REQUIRED if not results[name]]
            self._status_var.set(tr("env_check_cannot_proceed", ", ".join(missing_labels)))
        else:
            self._status_var.set("")

    def _on_copy_install_command(self, name):
        commands_by_platform = ENV_CHECK_INSTALL_COMMANDS.get(name)
        if not commands_by_platform:
            return
        command = install_command_for(commands_by_platform)
        self.root.clipboard_clear()
        self.root.clipboard_append(command)
        self._status_var.set(tr("env_check_copied", command))

    # -- page 3: options --------------------------------------------------

    def _build_page_options(self, parent):
        frame = ttk.Frame(parent, padding=10)
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, text=tr("options_title"), font=("TkDefaultFont", 12, "bold")).grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(frame, text=tr("options_install_to")).grid(row=1, column=0, sticky="w", pady=(10, 0))
        dir_row = ttk.Frame(frame)
        dir_row.grid(row=2, column=0, sticky="ew")
        dir_row.columnconfigure(0, weight=1)
        ttk.Entry(dir_row, textvariable=self._install_dir_var).grid(row=0, column=0, sticky="ew")
        ttk.Button(dir_row, text=tr("options_browse"), command=self._on_browse_install_dir).grid(
            row=0, column=1, padx=(8, 0)
        )

        ttk.Checkbutton(
            frame,
            text=tr("options_include_flasher"),
            variable=self._include_flasher_var,
        ).grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Checkbutton(
            frame,
            text=tr("options_desktop_shortcut"),
            variable=self._desktop_shortcut_var,
        ).grid(row=4, column=0, sticky="w")
        ttk.Checkbutton(
            frame,
            text=tr("options_minimal"),
            variable=self._minimal_var,
        ).grid(row=5, column=0, sticky="w")

        self._existing_checkout_frame = ttk.Labelframe(
            frame, text=tr("options_existing_checkout_title"), padding=8
        )
        self._existing_checkout_frame.grid(row=6, column=0, sticky="ew", pady=(10, 0))
        ttk.Radiobutton(
            self._existing_checkout_frame,
            text=tr("options_repair"),
            variable=self._existing_mode_var,
            value=EXISTING_CHECKOUT_MODE_REPAIR,
        ).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(
            self._existing_checkout_frame,
            text=tr("options_uninstall"),
            variable=self._existing_mode_var,
            value=EXISTING_CHECKOUT_MODE_UNINSTALL,
        ).grid(row=1, column=0, sticky="w")
        self._existing_checkout_frame.grid_remove()

        return frame

    def _on_browse_install_dir(self):
        selected_path = filedialog.askdirectory(
            title=tr("options_browse_dialog_title"),
            initialdir=str(Path(self._install_dir_var.get()).expanduser()),
        )
        if selected_path:
            self._install_dir_var.set(selected_path)
            self._refresh_existing_checkout_state()

    def _refresh_existing_checkout_state(self):
        install_dir = Path(self._install_dir_var.get()).expanduser()
        if detect_existing_checkout(install_dir):
            self._existing_checkout_frame.grid()
        else:
            self._existing_checkout_frame.grid_remove()

    # -- page 4: execute --------------------------------------------------

    def _build_page_execute(self, parent):
        frame = ttk.Frame(parent, padding=10)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(2, weight=1)

        ttk.Label(frame, text=tr("execute_title"), font=("TkDefaultFont", 12, "bold")).grid(
            row=0, column=0, sticky="w"
        )

        indicator_row = ttk.Frame(frame)
        indicator_row.grid(row=1, column=0, sticky="ew", pady=(10, 10))
        for step_index, step_text in enumerate(step_labels()):
            indicator_row.columnconfigure(step_index, weight=1)
            step_label = ttk.Label(indicator_row, text=step_text, padding=(6, 4), relief="groove", anchor="center")
            step_label.grid(row=0, column=step_index, sticky="ew")
            self._step_labels.append(step_label)

        log_frame = ttk.Labelframe(frame, text=tr("execute_log_label"), padding=4)
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        # Read-only monospace log: users can select/copy text but the
        # widget itself is only ever written to by _append_log().
        # 読み取り専用の等幅ログ: ユーザーはテキストの選択/コピーはできるが、
        # ウィジェットへの書き込みは _append_log() からのみ行う。
        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=LOG_TEXT_HEIGHT_ROWS, font=LOG_TEXT_FONT, state="disabled", wrap="word"
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, textvariable=self._status_var).grid(row=3, column=0, sticky="w", pady=(6, 0))
        return frame

    def _set_step_indicator(self, active_step_index):
        """Highlight steps [0, active_step_index) as done, active_step_index as current."""
        # [0, active_step_index) を完了済み、active_step_index を実行中として強調する
        for step_index, step_label in enumerate(self._step_labels):
            if step_index < active_step_index:
                step_label.configure(foreground=OK_TEXT_COLOR, font=("TkDefaultFont", 9, "normal"))
            elif step_index == active_step_index:
                step_label.configure(foreground="", font=("TkDefaultFont", 9, "bold"))
            else:
                step_label.configure(foreground=NOTE_TEXT_COLOR, font=("TkDefaultFont", 9, "normal"))

    def _start_setup_worker(self):
        self._clear_log()
        self._cancel_event.clear()
        self._busy = True
        self._set_step_indicator(0)
        self._status_var.set(tr("execute_starting"))

        install_dir = Path(self._install_dir_var.get()).expanduser()
        checkout_exists = detect_existing_checkout(install_dir)
        action = (
            ACTION_UNINSTALL
            if checkout_exists and self._existing_mode_var.get() == EXISTING_CHECKOUT_MODE_UNINSTALL
            else ACTION_INSTALL
        )
        self._last_action = action
        options = SetupOptions(
            install_dir=install_dir,
            action=action,
            skip_clone=checkout_exists,
            include_flasher=self._include_flasher_var.get(),
            minimal=self._minimal_var.get(),
        )

        def worker():
            return_code = perform_setup_workflow(self._queue, options, self._cancel_event)
            self._queue.put(("done", return_code == 0, return_code))

        threading.Thread(target=worker, daemon=True).start()

    def _on_cancel_clicked(self):
        self._cancel_event.set()
        self.secondary_button.configure(state="disabled")
        self._status_var.set(tr("execute_cancelling"))

    # -- page 5: done --------------------------------------------------

    def _build_page_done(self, parent):
        frame = ttk.Frame(parent, padding=10)
        ttk.Label(frame, textvariable=self._done_title_var, font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
        ttk.Label(
            frame, textvariable=self._done_body_var, wraplength=HINT_TEXT_WRAP_PX, justify="left"
        ).pack(anchor="w", pady=(10, 0))
        return frame

    def _render_done_page(self):
        install_dir = Path(self._install_dir_var.get()).expanduser()
        if not self._result_success:
            self._done_title_var.set(tr("done_failed_title"))
            recovery_command = (
                "cd {0} && install.bat".format(install_dir)
                if sys.platform == "win32"
                else "cd {0} && ./install.sh".format(install_dir)
            )
            self._done_body_var.set(tr("done_failed_body", recovery_command))
            self.secondary_button.grid()
            return

        # Secondary button ("Save log") is only meaningful on failure.
        # 「ログを保存」ボタンは失敗時のみ意味を持つ。
        self.secondary_button.grid_remove()
        if self._last_action == ACTION_UNINSTALL:
            self._done_title_var.set(tr("done_uninstall_title"))
            self._done_body_var.set(tr("done_uninstall_body"))
            return

        self._done_title_var.set(tr("done_install_title"))
        setup_env_command = (
            str(install_dir / "setup_env.bat")
            if sys.platform == "win32"
            else "source {0}".format(install_dir / "setup_env.sh")
        )
        self._done_body_var.set(tr("done_install_body", setup_env_command))

    def _on_save_log_clicked(self):
        save_path = filedialog.asksaveasfilename(
            title=tr("nav_save_log"),
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
        )
        if not save_path:
            return
        log_contents = self.log_text.get("1.0", "end")
        with open(save_path, "w", encoding="utf-8") as log_file:
            log_file.write(log_contents)
        self._status_var.set(tr("save_log_saved", save_path))

    # -- log widget helpers -------------------------------------------------

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _append_log(self, line):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # -- worker/queue plumbing -------------------------------------------------

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
            step_match = STEP_HEADER_PATTERN.search(line)
            if step_match:
                self._set_step_indicator(int(step_match.group(1)))
        elif kind == "status":
            self._status_var.set(message[1])
        elif kind == "phase":
            self._handle_phase_message(message[1])
        elif kind == "env_check_result":
            self._handle_env_check_result(message[1], message[2])
        elif kind == "done":
            self._on_worker_done(message[1], message[2])

    def _handle_phase_message(self, phase):
        if phase == PHASE_CLONE:
            self.secondary_button.configure(state="normal")
        elif phase == PHASE_INSTALL:
            # Cancel only ever covers the clone phase (see
            # perform_setup_workflow()'s docstring and
            # docs/plans/gui-installer-plan.md §5): once installer.py
            # itself is running, disable Cancel and say why.
            # キャンセルは常に clone フェーズのみを対象とする
            # (perform_setup_workflow() の docstring と
            # docs/plans/gui-installer-plan.md §5 参照): installer.py
            # 自体が動き出したら、キャンセルを無効化しその理由を表示する。
            self.secondary_button.configure(state="disabled")
            self._status_var.set(tr("execute_cannot_cancel"))

    def _on_worker_done(self, success, return_code):
        self._busy = False
        self._result_success = success
        self._status_var.set(tr("status_done") if success else tr("status_failed", return_code))
        self._show_page(PAGE_DONE)


def launch_gui() -> int:
    """
    Build the Tk root window and enter its event loop, or -- if tkinter
    itself failed to import -- print CLI recovery instructions to stderr
    and return 1 instead of crashing.

    This is the ONLY function that instantiates tk.Tk(); it is reached
    from main() exclusively when --selftest was not passed, so a
    headless CI invocation never creates a Tk window.

    Tk のルートウィンドウを構築しイベントループへ入る。tkinter 自体の
    import に失敗していた場合は、クラッシュする代わりに CLI 復旧手順を
    stderr へ表示して 1 を返す。

    tk.Tk() をインスタンス化するのはこの関数だけ。main() からは
    --selftest が指定されなかった場合にのみ到達するため、ヘッドレスな
    CI 実行が Tk ウィンドウを作ることは無い。
    """
    if _TKINTER_IMPORT_ERROR is not None:
        print(tkinter_missing_cli_hint(), file=sys.stderr)
        return 1

    root = tk.Tk()
    StampFlySetupApp(root)
    root.mainloop()
    return 0


# ---------------------------------------------------------------------------
# CLI entry point
# CLI エントリポイント
# ---------------------------------------------------------------------------


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="stampfly_installer.py",
        description="{0}: cross-platform wizard that clones stampfly_ecosystem and runs its installer.".format(
            APP_NAME
        ),
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help=(
            "Run a headless, Tk-free self-test and exit / "
            "GUIを表示せずヘッドレスなセルフテストを実行して終了"
        ),
    )
    return parser


def _ensure_stdio_streams() -> None:
    """
    Make sys.stdout / sys.stderr safe for printing in every packaging and
    console-encoding environment (same two failure modes, and the same
    remedy, as tools/flasher_gui/stampfly_flasher.py's
    _ensure_stdio_streams -- see its docstring for the full rationale):

    1. None streams: a Windows PyInstaller --windowed build has no
       attached console, so both streams are None and any print() would
       raise. Point them at os.devnull.
    2. Legacy console code pages (cp1252/cp932): --selftest prints
       Japanese, which a cp1252 stdout cannot encode -- the exact
       UnicodeEncodeError that took down the first Windows CI run of
       this tool (run 29715534777, 2026-07-20). reconfigure(errors=
       "replace") turns unencodable characters into "?" instead of a
       crash.

    sys.stdout / sys.stderr をあらゆるパッケージング・コンソール文字
    コード環境で安全に print できる状態に整える(故障モード2つと対処は
    tools/flasher_gui/stampfly_flasher.py の _ensure_stdio_streams と
    同じ -- 詳細な理由付けはそちらの docstring 参照):

    1. ストリームが None の場合: Windows の PyInstaller --windowed
       ビルドは接続先コンソールを持たず両ストリームが None になり、
       print() が例外になる。os.devnull へ向ける。
    2. レガシーなコンソールコードページ(cp1252/cp932): --selftest は
       日本語を print するが、cp1252 の stdout はこれをエンコード
       できない -- 本ツールの Windows CI 初回実行(run 29715534777、
       2026-07-20)を落とした、まさにその UnicodeEncodeError。
       reconfigure(errors="replace") でエンコード不能文字をクラッシュ
       ではなく "?" 表示にする。
    """
    if sys.stdout is None:
        sys.stdout = open(os.devnull, "w")
    if sys.stderr is None:
        sys.stderr = open(os.devnull, "w")
    for stream in (sys.stdout, sys.stderr, sys.__stdout__, sys.__stderr__):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors="replace")
        except (ValueError, OSError):
            # Non-reconfigurable stream (exotic redirection): leave as-is.
            # 再構成できないストリーム(特殊なリダイレクト): そのまま。
            pass


def main(argv=None) -> int:
    """
    CLI entry point. Stream hardening runs first (see
    _ensure_stdio_streams), then argument parsing -- both before any Tk
    code runs, so --selftest works in fully headless environments.
    CLI エントリポイント。最初にストリーム防御(_ensure_stdio_streams
    参照)、次に引数解析を行う -- どちらも Tk 関連のコードより前のため、
    --selftest は完全にヘッドレスな環境でも動作する。
    """
    _ensure_stdio_streams()
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.selftest:
        # CLI exception barrier: in a --windowed freeze, an exception
        # that escapes main() triggers a MODAL error dialog from
        # PyInstaller's bootloader -- on a headless CI runner nobody can
        # click it, so any crash would turn into an infinite hang (the
        # same failure mode documented in
        # tools/flasher_gui/stampfly_flasher.py's main()). Convert every
        # unexpected exception into a printed traceback plus exit code 1.
        # CLI 例外バリア: --windowed 凍結では main() の外へ漏れた例外が
        # PyInstaller bootloader のモーダルなエラーダイアログを表示させる
        # -- ヘッドレスな CI ランナーでは誰もそれを押せないため、あらゆる
        # クラッシュが無限ハングに化けてしまう
        # (tools/flasher_gui/stampfly_flasher.py の main() と同じ故障
        # モード)。予期しない例外は全て traceback の表示 + 終了コード 1
        # に変換する。
        try:
            return run_selftest()
        except Exception:
            traceback.print_exc()
            return 1

    # Same CLI exception barrier as the --selftest branch above, applied to
    # the GUI path: an exception escaping launch_gui() (e.g. during
    # StampFlySetupApp construction, before the Tk event loop even starts)
    # would otherwise surface as a raw traceback (console build) or a
    # PyInstaller bootloader modal dialog with no useful message
    # (--windowed build), leaving a newcomer with nothing actionable.
    # Print the traceback to stderr, then best-effort show a short
    # bilingual messagebox -- wrapped in its own try/except in case tkinter
    # itself is what is broken -- and exit 1.
    # 上の --selftest 分岐と同じ CLI 例外バリアを GUI 経路にも適用する:
    # launch_gui() を抜ける例外(例: Tk イベントループが始まる前の
    # StampFlySetupApp 構築中)を放置すると、素の traceback でクラッシュ
    # する(コンソールビルド)か、PyInstaller bootloader の有用な情報の
    # 無いモーダルダイアログになり(--windowed ビルド)、初心者には対処
    # しようがない。traceback を stderr へ出力し、ベストエフォートで短い
    # 英日併記のメッセージボックスを表示し(tkinter 自体が壊れている場合に
    # 備えてそれ自体を try/except で包む)、exit 1 する。
    try:
        return launch_gui()
    except Exception:
        traceback.print_exc()
        if messagebox is not None:
            try:
                detail = "\n".join(traceback.format_exc().strip().splitlines()[-5:])
                messagebox.showerror(
                    tr("gui_launch_failed_title", APP_NAME),
                    tr("gui_launch_failed_body", APP_NAME, detail),
                )
            except Exception:
                # tkinter itself may be the thing that is broken (e.g. no
                # display server); the traceback above is already on
                # stderr, so there is nothing more we can safely surface.
                # tkinter 自体が壊れている場合がある(例: ディスプレイ
                # サーバーが無い)。traceback は既に stderr へ出ているため、
                # これ以上安全に表示できるものは無い。
                pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
