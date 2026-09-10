# macOS セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

macOSでのStampFly開発環境セットアップ手順です。既定のインストールでは、
このエコシステム専用のPython 3.12とESP-IDF v5.5.2が `~/.stampfly` 配下に
自己完結導入され、Macに既にあるPythonやESP-IDFには一切依存しません。

### 方法A: GUI インストーラ（推奨・ターミナル不要）

ターミナル操作に不慣れな場合は、ダブルクリックだけで導入できる GUI 版
「StampFly Setup」がおすすめです。

- macOS 用の `.zip`（Apple Silicon / Intel を選択）をダウンロードして展開・起動するだけ
- 5画面のウィザードでインストール先やオプションを選択
- 中身は `scripts/installer.py`（本ガイドの `./install.sh` が最終的に呼び出すのと同じ本体）を
  直接動かす。ただし `./install.sh` 自身が事前に行う Xcode CLT / Homebrew の必須チェックは
  GUI 側では行わないため、事前にそれらを導入していない場合は挙動が異なることがある

詳細手順は **[GUI インストーラガイド](../guides/gui-installer.md)** を参照してください。

### 方法B: CLI（このガイドの手順）

ターミナル操作に抵抗がなければ、以下の手順で Xcode Command Line Tools のインストールから
進めることもできます。

## 2. 前提条件

| 項目 | 要件 |
|------|------|
| macOS | 12.0 (Monterey) 以降 |
| Xcode CLT | 必須 |
| Homebrew | 必須（cmake・ninja・dfu-util・ccacheの導入に使用） |
| Python | **不要**（専用のPython 3.12がインストーラによって自動導入される） |

## 3. Xcode Command Line Toolsのインストール

```bash
xcode-select --install
```

## 4. Homebrewのインストール

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## 5. 依存パッケージのインストール

```bash
brew install cmake ninja dfu-util ccache
```

## 6. インストーラの実行

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

既定では、このエコシステム専用のPython 3.12とESP-IDF v5.5.2を `SF_HOME`
（`~/.stampfly`。環境変数 `SF_HOME` で上書き可）の下に自己完結インストールし、
続けて sf CLI をセットアップします。末尾の「Step 4/4: GUI Flasher」では、
GUIフラッシャ「StampFly Flasher」をネイティブアプリとしてインストールするか
尋ねられます（既定 Yes、`--no-flasher` でスキップ可）。インストールすると
`~/Applications/StampFlyFlasher.app` に置かれ、Launchpad から起動できます。

### 何が、どこに入るか

既定の `SF_HOME` は `~/.stampfly` です。合計の容量は約4〜6 GBです。

| フォルダ／ファイル | 内容 |
|--------------------|------|
| `~/.stampfly/python/` | 専用CPython 3.12（`bin/python3`） |
| `~/.stampfly/esp-idf/` | 専用ESP-IDF v5.5.2（`--depth 1` クローン） |
| `~/.stampfly/espressif/` | ツールチェーンと仮想環境（`IDF_TOOLS_PATH`） |
| `~/.stampfly/downloads/` | 取得した配布物のキャッシュ（再導入時の再取得を省く） |
| `~/.stampfly/manifest.json` | 導入済みPython／ESP-IDFの版・SHA-256・導入日時の記録 |

### 旧来モード（既存のESP-IDFを使う・上級者向け）

自分で管理しているESP-IDFとシステムPythonをそのまま使いたい場合は、まずESP-IDFを
手動で用意します。

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

続けて、システムPython 3.10〜3.12（`brew install python@3.12` で導入可）を用意した上で、
リポジトリのフォルダに戻り、`--use-existing-idf`（または `--idf-path`）を付けて
インストーラを実行します。

```bash
./install.sh --use-existing-idf --idf-path ~/esp/esp-idf
```

> **Note**: 上記のESP-IDF自体のインストール手順では `source ~/esp/esp-idf/export.sh`
> を使いますが、StampFly Ecosystem での日常的な開発では（旧来モードでも）
> `source setup_env.sh` を使用してください。

## 7. シリアルポートドライバ

M5Stack製品（CH9102F）のドライバは通常不要です。認識しない場合は、USBデバイスが
見えているか確認してください。

```bash
ls /dev/tty.usb*
```

## 8. 開発環境の有効化と動作確認

開発環境をセットアップします。

```bash
source setup_env.sh
```

プロジェクトディレクトリに移動し、環境診断とバージョン確認を行います。

```bash
cd path/to/stampfly_ecosystem
sf doctor
sf version
```

`sf doctor` の「Checking environment」の項目に
`Dedicated environment: /Users/<ユーザー名>/.stampfly` のように表示されれば、
専用環境が正しく使われています。

## 9. アンインストール

```bash
./install.sh --uninstall
```

sf CLI と設定ファイルが環境から削除されます（GUIフラッシャも導入済みなら
一緒に削除）。専用環境（`SF_HOME`、約4〜6 GB）も含めて完全に削除したい場合は
`--purge` を付けます（確認なしで削除されるため注意）。

```bash
./install.sh --uninstall --purge
```

## 10. トラブルシューティング

Mac固有ではない問題（SF_HOMEのパスに関する警告、「Dedicated environment is
incomplete」、`sf doctor` が「別のPythonで動いている」と警告する等）は
**[トラブルシューティングガイド「7. 環境（専用環境）」](../guides/troubleshooting.md)**
にまとめてあります。

### Python関連エラー

pyserialが無いというエラーが出た場合はインストールしてください。

```bash
pip3 install pyserial
```

### USB権限エラー

macOSでは通常不要ですが、問題がある場合はシステム環境設定でセキュリティを確認してください。

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on macOS. The default install
self-contains a private Python 3.12 and ESP-IDF v5.5.2 for this ecosystem alone under
`~/.stampfly`, independent of any Python or ESP-IDF already on your Mac.

### Method A: GUI Installer (Recommended — No Terminal Needed)

If you're not comfortable with the terminal, the GUI version "StampFly Setup" lets you install
with just a few clicks.

- Download and unzip a `.zip` for macOS (choose Apple Silicon or Intel), then launch it
- A 5-screen wizard walks you through the install location and options
- Runs `scripts/installer.py` directly (the same program this guide's `./install.sh` eventually
  calls). Note that `./install.sh` itself performs prerequisite checks (Xcode CLT / Homebrew)
  before that, which the GUI does not -- behavior can differ if those aren't already installed

See the **[GUI Installer Guide](../guides/gui-installer.md)** for details.

### Method B: CLI (This Guide's Steps)

If you're comfortable with the terminal, you can also start from installing the Xcode Command
Line Tools below.

## 2. Prerequisites

| Item | Requirement |
|------|-------------|
| macOS | 12.0 (Monterey) or later |
| Xcode CLT | Required |
| Homebrew | Required (used to install cmake, ninja, dfu-util, ccache) |
| Python | **Not required** (a private Python 3.12 is installed automatically) |

## 3. Install Xcode Command Line Tools

```bash
xcode-select --install
```

## 4. Install Homebrew

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## 5. Install Dependencies

```bash
brew install cmake ninja dfu-util ccache
```

## 6. Run the Installer

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

By default, this installs a private Python 3.12 and ESP-IDF v5.5.2 for this
ecosystem alone, self-contained under `SF_HOME` (`~/.stampfly`; override with the
`SF_HOME` environment variable), then sets up sf CLI. The final "Step 4/4: GUI
Flasher" prompt offers to install the GUI flasher "StampFly Flasher" as a native app
(default Yes; skip with `--no-flasher`). Once installed, it lands at
`~/Applications/StampFlyFlasher.app` and appears in Launchpad.

### What Gets Installed Where

The default `SF_HOME` is `~/.stampfly`. Total disk space is about 4-6 GB.

| Folder / File | Contents |
|----------------|----------|
| `~/.stampfly/python/` | A private CPython 3.12 (`bin/python3`) |
| `~/.stampfly/esp-idf/` | A private ESP-IDF v5.5.2 (`--depth 1` clone) |
| `~/.stampfly/espressif/` | Toolchain and virtual environment (`IDF_TOOLS_PATH`) |
| `~/.stampfly/downloads/` | Cached downloads (skips re-fetching on a later re-install) |
| `~/.stampfly/manifest.json` | Record of the installed Python/ESP-IDF versions, SHA-256, and install date |

### Legacy Mode (Use an Existing ESP-IDF -- Advanced)

If you already manage your own ESP-IDF and system Python and want to keep using
them, first set up ESP-IDF by hand.

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

Then, with a system Python 3.10-3.12 in place (`brew install python@3.12`), go back
to the repository folder and run the installer with `--use-existing-idf` (or
`--idf-path`).

```bash
./install.sh --use-existing-idf --idf-path ~/esp/esp-idf
```

> **Note**: The ESP-IDF installation step above uses `source ~/esp/esp-idf/export.sh`,
> but for day-to-day StampFly Ecosystem development (legacy mode included), use
> `source setup_env.sh` instead.

## 7. Serial Port Driver

Driver for M5Stack products (CH9102F) is usually not needed. If not recognized:

```bash
ls /dev/tty.usb*
```

## 8. Activate and Verify

```bash
source setup_env.sh
cd path/to/stampfly_ecosystem
sf doctor
sf version
```

`sf doctor`'s "Checking environment" section should show something like
`Dedicated environment: /Users/<you>/.stampfly`, confirming the dedicated
environment is in use.

## 9. Uninstall

```bash
./install.sh --uninstall
```

Removes sf CLI and its config from the environment (also removes the GUI Flasher if
installed). To also delete the dedicated environment (`SF_HOME`, about 4-6 GB)
entirely, add `--purge` (this deletes without asking, so use it deliberately).

```bash
./install.sh --uninstall --purge
```

## 10. Troubleshooting

Issues that are not Mac-specific (warnings about the SF_HOME path, "Dedicated
environment is incomplete", `sf doctor` warning it is "running under a different
Python", etc.) are covered in
**["7. Environment (Dedicated)" in the Troubleshooting Guide](../guides/troubleshooting.md)**.

### Python-related Errors

```bash
pip3 install pyserial
```

### USB Permission Errors

Usually not needed on macOS. If you encounter issues, check Security settings in System Preferences.
