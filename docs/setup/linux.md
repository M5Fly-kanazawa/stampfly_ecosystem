# Linux セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

Linux（Ubuntu/Debian）でのStampFly開発環境セットアップ手順です。既定のインストールでは、
このエコシステム専用のPython 3.12とESP-IDF v5.5.2が `~/.stampfly` 配下に自己完結導入され、
このPCに既にあるPythonやESP-IDFには一切依存しません。

### 方法A: GUI インストーラ（推奨・ターミナル不要）

ターミナル操作に不慣れな場合は、GUI 版「StampFly Setup」がおすすめです。

- Linux 用の実行ファイル（拡張子なし）をダウンロードし、実行権限を付与して起動するだけ
- 5画面のウィザードでインストール先やオプションを選択
- 中身は本ガイドの CLI インストーラ（`./install.sh`）と同じロジックなので、機能差はない

詳細手順は **[GUI インストーラガイド](../guides/gui-installer.md)** を参照してください。

### 方法B: CLI（このガイドの手順）

ターミナル操作に抵抗がなければ、以下の手順で依存パッケージのインストールから進めることも
できます。

## 2. 前提条件

| 項目 | 要件 |
|------|------|
| Ubuntu | 22.04 LTS 以降 |
| または Debian | 11 以降 |
| Python | **不要**（専用のPython 3.12がインストーラによって自動導入される） |

## 3. 依存パッケージのインストール

```bash
sudo apt update
sudo apt install -y git curl tar cmake ninja-build wget flex bison gperf ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0
```

`curl` と `tar` は専用Pythonの取得に、その他はESP-IDFのビルドツールチェーンに使われます。

## 4. インストーラの実行

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
`~/.local/opt/stampfly/` に置かれ、アプリケーションメニューから起動できます。

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

自分で管理しているESP-IDFとシステムPythonをそのまま使いたい場合は、まず追加の
依存パッケージ（`python3` / `python3-pip` / `python3-venv`）を導入します。

```bash
sudo apt install -y python3 python3-pip python3-venv
```

続けてESP-IDFを手動で用意します。

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

最後に、リポジトリのフォルダに戻り、`--use-existing-idf`（または `--idf-path`）を
付けてインストーラを実行します。

```bash
./install.sh --use-existing-idf --idf-path ~/esp/esp-idf
```

> **Note**: 上記のESP-IDF自体のインストール手順では `source ~/esp/esp-idf/export.sh`
> を使いますが、StampFly Ecosystem での日常的な開発では（旧来モードでも）
> `source setup_env.sh` を使用してください。

## 5. シリアルポートの権限設定

dialoutグループにユーザーを追加します。

```bash
sudo usermod -a -G dialout $USER
```

反映には再ログインが必要です。すぐに反映したい場合は以下を実行します。

```bash
newgrp dialout
```

## 6. udevルールの設定（オプション）

ESP32デバイス用のルールを作成します。

```bash
sudo tee /etc/udev/rules.d/99-esp32.rules << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666"
EOF
```

ルールを再読み込みします。

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 7. 開発環境の有効化と動作確認

開発環境をセットアップします。

```bash
source setup_env.sh
```

プロジェクトディレクトリに移動し、環境診断とシリアルポートの確認を行います。

```bash
cd path/to/stampfly_ecosystem
sf doctor
ls /dev/ttyUSB* /dev/ttyACM*
```

`sf doctor` の「Checking environment」の項目に
`Dedicated environment: /home/<ユーザー名>/.stampfly` のように表示されれば、
専用環境が正しく使われています。

## 8. アンインストール

```bash
./install.sh --uninstall
```

sf CLI と設定ファイルが環境から削除されます（GUIフラッシャも導入済みなら
一緒に削除）。専用環境（`SF_HOME`、約4〜6 GB）も含めて完全に削除したい場合は
`--purge` を付けます（確認なしで削除されるため注意）。

```bash
./install.sh --uninstall --purge
```

## 9. トラブルシューティング

Linux固有ではない問題（SF_HOMEのパスに関する警告、「Dedicated environment is
incomplete」、`sf doctor` が「別のPythonで動いている」と警告する等）は
**[トラブルシューティングガイド「7. 環境（専用環境）」](../guides/troubleshooting.md)**
にまとめてあります。

### シリアルポートが見つからない

デバイスの接続状況をカーネルログで確認します。

```bash
dmesg | tail -20
```

権限を確認します。

```bash
ls -la /dev/ttyUSB0
```

### Python関連エラー

pyserialが無いというエラーが出た場合はインストールしてください。

```bash
pip3 install pyserial
```

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on Linux (Ubuntu/Debian). The
default install self-contains a private Python 3.12 and ESP-IDF v5.5.2 for this
ecosystem alone under `~/.stampfly`, independent of any Python or ESP-IDF already on
this machine.

### Method A: GUI Installer (Recommended — No Terminal Needed)

If you're not comfortable with the terminal, the GUI version "StampFly Setup" is the easier
path.

- Download the extension-less Linux executable, mark it executable, and launch it
- A 5-screen wizard walks you through the install location and options
- Runs the exact same logic as this guide's CLI installer (`./install.sh`) internally — no
  feature difference

See the **[GUI Installer Guide](../guides/gui-installer.md)** for details.

### Method B: CLI (This Guide's Steps)

If you're comfortable with the terminal, you can also start from installing the dependency
packages below.

## 2. Prerequisites

| Item | Requirement |
|------|-------------|
| Ubuntu | 22.04 LTS or later |
| or Debian | 11 or later |
| Python | **Not required** (a private Python 3.12 is installed automatically) |

## 3. Install Dependencies

```bash
sudo apt update
sudo apt install -y git curl tar cmake ninja-build wget flex bison gperf ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0
```

`curl` and `tar` fetch the private Python; the rest are ESP-IDF's own build
toolchain dependencies.

## 4. Run the Installer

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
`~/.local/opt/stampfly/` and appears in your applications menu.

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
them, first install the additional dependency packages (`python3` /
`python3-pip` / `python3-venv`).

```bash
sudo apt install -y python3 python3-pip python3-venv
```

Then set up ESP-IDF by hand.

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

Finally, go back to the repository folder and run the installer with
`--use-existing-idf` (or `--idf-path`).

```bash
./install.sh --use-existing-idf --idf-path ~/esp/esp-idf
```

> **Note**: The ESP-IDF installation step above uses `source ~/esp/esp-idf/export.sh`,
> but for day-to-day StampFly Ecosystem development (legacy mode included), use
> `source setup_env.sh` instead.

## 5. Serial Port Permissions

Add your user to the dialout group.

```bash
sudo usermod -a -G dialout $USER
```

This takes effect after a re-login. To apply it immediately instead, run:

```bash
newgrp dialout
```

## 6. udev Rules (Optional)

Create rules for ESP32 devices.

```bash
sudo tee /etc/udev/rules.d/99-esp32.rules << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666"
EOF
```

Reload the rules.

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 7. Activate and Verify

Activate the development environment.

```bash
source setup_env.sh
```

Navigate to the project and run diagnostics and a serial port check.

```bash
cd path/to/stampfly_ecosystem
sf doctor
ls /dev/ttyUSB* /dev/ttyACM*
```

`sf doctor`'s "Checking environment" section should show something like
`Dedicated environment: /home/<you>/.stampfly`, confirming the dedicated environment
is in use.

## 8. Uninstall

```bash
./install.sh --uninstall
```

Removes sf CLI and its config from the environment (also removes the GUI Flasher if
installed). To also delete the dedicated environment (`SF_HOME`, about 4-6 GB)
entirely, add `--purge` (this deletes without asking, so use it deliberately).

```bash
./install.sh --uninstall --purge
```

## 9. Troubleshooting

Issues that are not Linux-specific (warnings about the SF_HOME path, "Dedicated
environment is incomplete", `sf doctor` warning it is "running under a different
Python", etc.) are covered in
**["7. Environment (Dedicated)" in the Troubleshooting Guide](../guides/troubleshooting.md)**.

### Serial Port Not Found

Check the kernel log for the connection.

```bash
dmesg | tail -20
```

Check permissions.

```bash
ls -la /dev/ttyUSB0
```

### Python-related Errors

```bash
pip3 install pyserial
```
