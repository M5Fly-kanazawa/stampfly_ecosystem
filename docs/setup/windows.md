# Windows セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

WindowsでのStampFly開発環境セットアップ手順です。`install.bat` を実行するだけで、
このエコシステム専用の Python 3.12・ESP-IDF v5.5.2・sf CLI が自動的にインストール
されます。参加者のPCに既にあるPythonやESP-IDFには一切依存しません。

### 方法A: GUI インストーラ（推奨・ターミナル不要）

ターミナル（CMD）操作に不慣れな場合は、ダブルクリックだけで導入できる GUI 版
「StampFly Setup」がおすすめです。

- Windows 用の実行ファイル（`.exe`）をダウンロードして起動するだけ
- 5画面のウィザードでインストール先やオプションを選択
- 中身は本ガイドの CLI インストーラ（`install.bat`）と同じロジックなので、機能差はない

詳細手順は **[GUI インストーラガイド](../guides/gui-installer.md)** を参照してください。

### 方法B: CLI（このガイドの手順）

ターミナル操作に抵抗がなければ、以下の手順で `install.bat` を直接実行することもできます。

## 2. 前提条件

CMD を開いて以下を確認してください:

| 確認コマンド | 期待される結果 | なければ |
|-------------|---------------|---------|
| `git --version` | git version 2.x | `winget install Git.Git` |
| Windows のバージョン | Windows 10 バージョン1803（2018年4月更新）以降 | Windows Update で更新（`curl.exe`／`tar.exe` が標準搭載され、専用Pythonの取得に使われる） |

**Python のインストールは不要です。** 専用の Python 3.12 がインストーラによって
`SF_HOME` 配下に自動導入されます（詳細は下記「何が、どこに入るか」）。既存の
ESP-IDF・システムPythonをそのまま使いたい開発者は「旧来モード」を参照してください。

## 3. インストール

CMD で以下を実行:

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

`install.bat` が以下を自動的に行います:
- 専用の Python 3.12 と ESP-IDF v5.5.2 のダウンロードとインストール（`SF_HOME` 配下へ自己完結）
- sf CLI のセットアップ
- （Step 4/4・任意、既定 Yes）GUIフラッシャ「StampFly Flasher」のネイティブアプリ
  インストール。承諾するとスタートメニューとデスクトップ（任意）にショートカットが
  作成される（`install.bat --no-flasher` でスキップ可能。詳細: [sf flasher](../commands/sf-flasher.md)）

### 何が、どこに入るか

既定では `SF_HOME`（環境変数 `SF_HOME` で上書き可。未設定時は `C:\StampFly`。
作成できない場合は `%LOCALAPPDATA%\StampFly` にフォールバックし警告が表示される
— ESP-IDF は非ASCII・空白入りのパスに対応していないため）の下に以下が置かれます。
合計の容量は約4〜6 GBです。

| フォルダ／ファイル | 内容 |
|--------------------|------|
| `SF_HOME\python\` | 専用CPython 3.12（`python.exe`） |
| `SF_HOME\esp-idf\` | 専用ESP-IDF v5.5.2（`--depth 1` クローン） |
| `SF_HOME\espressif\` | ツールチェーンと仮想環境（`IDF_TOOLS_PATH`） |
| `SF_HOME\downloads\` | 取得した配布物のキャッシュ（再導入時の再取得を省く） |
| `SF_HOME\manifest.json` | 導入済みPython／ESP-IDFの版・SHA-256・導入日時の記録 |

### 旧来モード（既存のESP-IDFを使う・上級者向け）

自分のPCに既にあるESP-IDF・システムPythonをそのまま使いたい場合は、
`install.bat --use-existing-idf`（または `--idf-path <path>`）を指定します。
この場合はシステムPython 3.10〜3.12が必要です。

| 確認コマンド | 期待される結果 | なければ |
|-------------|---------------|---------|
| `python --version` | Python 3.10〜3.12（推奨3.12） | `winget install Python.Python.3.12` |

インストール済みなら以下の場所を自動検出します:
- `%LOCALAPPDATA%\Programs\Python\Python3XX`
- `C:\Python3XX`
- pyenv-win

それ以外の場所にインストールした場合は、CMD で `set PATH=C:\your\python\path;%PATH%`
を実行してから `install.bat --use-existing-idf` を再実行してください。

## 4. 開発環境のアクティベート

**毎回のセッション開始時**に以下を実行:

```cmd
cd stampfly_ecosystem
setup_env.bat
```

`setup_env.bat` が ESP-IDF 環境を読み込み、`sf` コマンドが使えるようになります。

## 5. USBシリアルドライバ

CH9102F（M5Stack製品）用ドライバをインストール:
- https://docs.m5stack.com/en/download

## 6. 動作確認

```cmd
sf doctor
```

すべて OK になれば環境構築完了です。「Checking environment」の項目に
`Dedicated environment: C:\StampFly` のように表示されれば、専用環境が正しく
使われています。

## 7. アンインストール

```cmd
install.bat --uninstall
```

sf CLI と設定ファイルが環境から削除されます（GUIフラッシャも導入済みなら
一緒に削除）。専用環境（`SF_HOME`、約4〜6 GB）も含めて完全に削除したい場合は
`--purge` を付けます（確認なしで削除されるため注意）。

```cmd
install.bat --uninstall --purge
```

## 8. トラブルシューティング

Windows 固有ではない問題（`curl.exe`/`tar.exe` が見つからない、SF_HOMEのパスに
関する警告、「Dedicated environment is incomplete」、`sf doctor` が「別のPython
で動いている」と警告する等）は
**[トラブルシューティングガイド「7. 環境（専用環境）」](../guides/troubleshooting.md)**
にまとめてあります。旧来モードでPythonが見つからない場合は「旧来モード」を
参照してください。

### シリアルポートが認識されない

CH9102F ドライバがインストールされているか確認してください。デバイスマネージャーの「ポート (COM & LPT)」にデバイスが表示されれば OK です。

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on Windows. Just run
`install.bat` to automatically install a private Python 3.12, ESP-IDF v5.5.2, and sf
CLI for this ecosystem alone. It does not depend on any Python or ESP-IDF already on
your machine.

### Method A: GUI Installer (Recommended — No Terminal Needed)

If you're not comfortable with the terminal (CMD), the GUI version "StampFly Setup" lets you
install with just a few clicks.

- Download and launch a single Windows executable (`.exe`)
- A 5-screen wizard walks you through the install location and options
- Runs the exact same logic as this guide's CLI installer (`install.bat`) internally — no
  feature difference

See the **[GUI Installer Guide](../guides/gui-installer.md)** for details.

### Method B: CLI (This Guide's Steps)

If you're comfortable with the terminal, you can also run `install.bat` directly by following
the steps below.

## 2. Prerequisites

Open CMD and verify the following:

| Command | Expected Result | If Missing |
|---------|----------------|------------|
| `git --version` | git version 2.x | `winget install Git.Git` |
| Windows version | Windows 10 version 1803 (April 2018 Update) or later | Run Windows Update (this ships `curl.exe`/`tar.exe`, used to fetch the private Python) |

**Installing Python yourself is not required.** A private Python 3.12 is installed
automatically under `SF_HOME` by the installer (see "What Gets Installed Where"
below). Developers who want to keep using an existing ESP-IDF and system Python
should see "Legacy Mode" below.

## 3. Installation

Run in CMD:

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

`install.bat` automatically:
- Downloads and installs a private Python 3.12 and ESP-IDF v5.5.2, self-contained under `SF_HOME`
- Sets up sf CLI
- (Step 4/4, optional, default Yes) Installs the GUI flasher "StampFly Flasher" as a
  native app. Accepting adds shortcuts to the Start Menu and Desktop (optional)
  (skip with `install.bat --no-flasher`; details: [sf flasher](../commands/sf-flasher.md))

### What Gets Installed Where

By default, everything is placed under `SF_HOME` (override with the `SF_HOME`
environment variable; defaults to `C:\StampFly` if unset, falling back to
`%LOCALAPPDATA%\StampFly` with a warning if that cannot be created -- ESP-IDF does
not support non-ASCII or space-containing paths). Total disk space is about 4-6 GB.

| Folder / File | Contents |
|----------------|----------|
| `SF_HOME\python\` | A private CPython 3.12 (`python.exe`) |
| `SF_HOME\esp-idf\` | A private ESP-IDF v5.5.2 (`--depth 1` clone) |
| `SF_HOME\espressif\` | Toolchain and virtual environment (`IDF_TOOLS_PATH`) |
| `SF_HOME\downloads\` | Cached downloads (skips re-fetching on a later re-install) |
| `SF_HOME\manifest.json` | Record of the installed Python/ESP-IDF versions, SHA-256, and install date |

### Legacy Mode (Use an Existing ESP-IDF -- Advanced)

If you already have an ESP-IDF and system Python on your machine and want to keep
using them, pass `install.bat --use-existing-idf` (or `--idf-path <path>`). This
mode requires a system Python 3.10-3.12.

| Command | Expected Result | If Missing |
|---------|----------------|------------|
| `python --version` | Python 3.10-3.12 (3.12 recommended) | `winget install Python.Python.3.12` |

It is auto-detected if installed in these locations:
- `%LOCALAPPDATA%\Programs\Python\Python3XX`
- `C:\Python3XX`
- pyenv-win

For other locations, run `set PATH=C:\your\python\path;%PATH%` before re-running
`install.bat --use-existing-idf`.

## 4. Activate Development Environment

Run **at the start of each session**:

```cmd
cd stampfly_ecosystem
setup_env.bat
```

`setup_env.bat` loads the ESP-IDF environment, making the `sf` command available.

## 5. USB Serial Driver

Install CH9102F (M5Stack products) driver:
- https://docs.m5stack.com/en/download

## 6. Verify Installation

```cmd
sf doctor
```

If all checks pass, your environment is ready. The "Checking environment" section
should show something like `Dedicated environment: C:\StampFly`, confirming the
dedicated environment is in use.

## 7. Uninstall

```cmd
install.bat --uninstall
```

Removes sf CLI and its config from the environment (also removes the GUI Flasher if
installed). To also delete the dedicated environment (`SF_HOME`, about 4-6 GB)
entirely, add `--purge` (this deletes without asking, so use it deliberately).

```cmd
install.bat --uninstall --purge
```

## 8. Troubleshooting

Issues that are not Windows-specific (`curl.exe`/`tar.exe` not found, warnings about
the SF_HOME path, "Dedicated environment is incomplete", `sf doctor` warning it is
"running under a different Python", etc.) are covered in
**["7. Environment (Dedicated)" in the Troubleshooting Guide](../guides/troubleshooting.md)**.
If Python cannot be found in legacy mode, see "Legacy Mode" above.

### Serial Port Not Recognized

Verify the CH9102F driver is installed. The device should appear under "Ports (COM & LPT)" in Device Manager.
