# Linux セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

Linux（Ubuntu/Debian）でのStampFly開発環境セットアップ手順です。

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

### シェル設定への追加（推奨）

`~/.bashrc` に以下を追加:

```bash
alias get_idf='source ~/esp/esp-idf/export.sh'
```

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
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

# プロジェクトディレクトリに移動
cd path/to/stampfly-ecosystem

# 環境診断
sf doctor

# シリアルポートを確認
ls /dev/ttyUSB* /dev/ttyACM*
```

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

### Add to Shell Config (Recommended)

Add to `~/.bashrc`:

```bash
alias get_idf='source ~/esp/esp-idf/export.sh'
```

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
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Navigate to project
cd path/to/stampfly-ecosystem

# Run diagnostics
sf doctor

# Check serial ports
ls /dev/ttyUSB* /dev/ttyACM*
```
