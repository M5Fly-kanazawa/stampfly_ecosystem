# Linux セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

Linux（Ubuntu/Debian）でのStampFly開発環境セットアップ手順です。

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

## 3. 依存パッケージのインストール

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

## 4. ESP-IDFのインストール

```bash
# インストール先ディレクトリを作成
mkdir -p ~/esp
cd ~/esp

# ESP-IDFをクローン
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git

# ツールチェーンをインストール
cd esp-idf
./install.sh esp32s3

# 環境変数を設定
source ~/esp/esp-idf/export.sh
```

> **Note**: ESP-IDF自体のインストール手順では `source ~/esp/esp-idf/export.sh` を使いますが、StampFly Ecosystem での日常的な開発では `source setup_env.sh` を使用してください。

## 5. シリアルポートの権限設定

```bash
# dialoutグループにユーザーを追加
sudo usermod -a -G dialout $USER

# 再ログインまたは以下を実行
newgrp dialout
```

## 6. udevルールの設定（オプション）

```bash
# ESP32デバイス用ルールを作成
sudo tee /etc/udev/rules.d/99-esp32.rules << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666"
EOF

# ルールを再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 7. 動作確認

```bash
# 開発環境のセットアップ
source setup_env.sh

# プロジェクトディレクトリに移動
cd path/to/stampfly_ecosystem

# 環境診断
sf doctor

# シリアルポートを確認
ls /dev/ttyUSB* /dev/ttyACM*
```

> **Tip**: `./install.sh`（[セットアップガイド](README.md)参照）で sf CLI を導入した場合、
> 末尾の「Step 4/4: GUI Flasher」で GUIフラッシャ「StampFly Flasher」をネイティブアプリ
> としてインストールするか尋ねられます（既定 Yes）。インストールすると
> `~/.local/opt/stampfly/` に置かれ、アプリケーションメニューから起動できます。

## 8. トラブルシューティング

### シリアルポートが見つからない

```bash
# 接続を確認
dmesg | tail -20

# 権限を確認
ls -la /dev/ttyUSB0
```

### Python関連エラー

```bash
pip3 install pyserial
```

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on Linux (Ubuntu/Debian).

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

## 3. Install Dependencies

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

## 4. Install ESP-IDF

```bash
# Create installation directory
mkdir -p ~/esp
cd ~/esp

# Clone ESP-IDF
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git

# Install toolchain
cd esp-idf
./install.sh esp32s3

# Set environment
source ~/esp/esp-idf/export.sh
```

> **Note**: The ESP-IDF installation step uses `source ~/esp/esp-idf/export.sh`, but for day-to-day StampFly Ecosystem development, use `source setup_env.sh` instead.

## 5. Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Re-login or run
newgrp dialout
```

## 6. udev Rules (Optional)

```bash
# Create rules for ESP32 devices
sudo tee /etc/udev/rules.d/99-esp32.rules << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 7. Verify Installation

```bash
# Activate development environment
source setup_env.sh

# Navigate to project
cd path/to/stampfly_ecosystem

# Run diagnostics
sf doctor

# Check serial ports
ls /dev/ttyUSB* /dev/ttyACM*
```

> **Tip**: If you installed via `./install.sh` (see the [setup guide](README.md)), the
> final "Step 4/4: GUI Flasher" prompt offers to install the GUI flasher "StampFly
> Flasher" as a native app (default Yes). Once installed, it lands at
> `~/.local/opt/stampfly/` and appears in your applications menu.

## 8. Troubleshooting

### Serial Port Not Found

```bash
# Check connection
dmesg | tail -20

# Check permissions
ls -la /dev/ttyUSB0
```

### Python-related Errors

```bash
pip3 install pyserial
```
