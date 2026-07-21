# Windows セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

WindowsでのStampFly開発環境セットアップ手順です。`install.bat` を実行するだけで ESP-IDF と sf CLI が自動的にインストールされます。

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
| `python --version` | Python 3.10〜3.12（推奨3.12） | `winget install Python.Python.3.12` |

## 3. インストール

CMD で以下を実行:

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

`install.bat` が以下を自動的に行います:
- ESP-IDF のダウンロードとインストール
- sf CLI のセットアップ
- （Step 4/4・任意、既定 Yes）GUIフラッシャ「StampFly Flasher」のネイティブアプリ
  インストール。承諾するとスタートメニューとデスクトップ（任意）にショートカットが
  作成される（`install.bat --no-flasher` でスキップ可能。詳細: [sf flasher](../commands/sf-flasher.md)）

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

すべて OK になれば環境構築完了です。

## 7. トラブルシューティング

### `install.bat` がPythonを見つけられない

Python が PATH に含まれていない場合があります。以下の場所にインストールされていれば自動検出されます:
- `%LOCALAPPDATA%\Programs\Python\Python3XX`
- `C:\Python3XX`
- pyenv-win

それ以外の場所にインストールした場合は、CMD で `set PATH=C:\your\python\path;%PATH%` を実行してから `install.bat` を再実行してください。

### シリアルポートが認識されない

CH9102F ドライバがインストールされているか確認してください。デバイスマネージャーの「ポート (COM & LPT)」にデバイスが表示されれば OK です。

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on Windows. Just run `install.bat` to automatically install ESP-IDF and sf CLI.

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
| `python --version` | Python 3.10-3.12 (3.12 recommended) | `winget install Python.Python.3.12` |

## 3. Installation

Run in CMD:

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

`install.bat` automatically:
- Downloads and installs ESP-IDF
- Sets up sf CLI
- (Step 4/4, optional, default Yes) Installs the GUI flasher "StampFly Flasher" as a
  native app. Accepting adds shortcuts to the Start Menu and Desktop (optional)
  (skip with `install.bat --no-flasher`; details: [sf flasher](../commands/sf-flasher.md))

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

If all checks pass, your environment is ready.

## 7. Troubleshooting

### `install.bat` Cannot Find Python

Python may not be in PATH. It is auto-detected if installed in these locations:
- `%LOCALAPPDATA%\Programs\Python\Python3XX`
- `C:\Python3XX`
- pyenv-win

For other locations, run `set PATH=C:\your\python\path;%PATH%` before re-running `install.bat`.

### Serial Port Not Recognized

Verify the CH9102F driver is installed. The device should appear under "Ports (COM & LPT)" in Device Manager.
