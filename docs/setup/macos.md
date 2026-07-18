# macOS セットアップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

macOSでのStampFly開発環境セットアップ手順です。

## 2. 前提条件

| 項目 | 要件 |
|------|------|
| macOS | 12.0 (Monterey) 以降 |
| Xcode CLT | 必須 |
| Homebrew | 推奨 |

## 3. Xcode Command Line Toolsのインストール

```bash
xcode-select --install
```

## 4. Homebrewのインストール（推奨）

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## 5. 依存パッケージのインストール

```bash
brew install cmake ninja dfu-util python3
```

## 6. ESP-IDFのインストール

```bash
# インストール先ディレクトリを作成
mkdir -p ~/esp
cd ~/esp

# ESP-IDFをクローン
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git

# ツールチェーンをインストール
cd esp-idf
./install.sh esp32s3

# 環境変数を設定（シェル設定に追加推奨）
source ~/esp/esp-idf/export.sh
```

> **Note**: ESP-IDF自体のインストール手順では `source ~/esp/esp-idf/export.sh` を使いますが、StampFly Ecosystem での日常的な開発では `source setup_env.sh` を使用してください。

## 7. シリアルポートドライバ

M5Stack製品（CH9102F）のドライバは通常不要です。認識しない場合:

```bash
# USBデバイスを確認
ls /dev/tty.usb*
```

## 8. 動作確認

```bash
# 開発環境のセットアップ
source setup_env.sh

# プロジェクトディレクトリに移動
cd path/to/stampfly_ecosystem

# 環境診断
sf doctor

# バージョン確認
sf version
```

> **Tip**: `./install.sh`（[セットアップガイド](README.md)参照）で sf CLI を導入した場合、
> 末尾の「Step 4/4: GUI Flasher」で GUIフラッシャ「StampFly Flasher」をネイティブアプリ
> としてインストールするか尋ねられます（既定 Yes）。インストールすると
> `~/Applications/StampFlyFlasher.app` に置かれ、Launchpad から起動できます。

## 9. トラブルシューティング

### Python関連エラー

```bash
# pyserialをインストール
pip3 install pyserial
```

### USB権限エラー

macOSでは通常不要ですが、問題がある場合はシステム環境設定でセキュリティを確認してください。

---

<a id="english"></a>

## 1. Overview

Setup instructions for StampFly development environment on macOS.

## 2. Prerequisites

| Item | Requirement |
|------|-------------|
| macOS | 12.0 (Monterey) or later |
| Xcode CLT | Required |
| Homebrew | Recommended |

## 3. Install Xcode Command Line Tools

```bash
xcode-select --install
```

## 4. Install Homebrew (Recommended)

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

## 5. Install Dependencies

```bash
brew install cmake ninja dfu-util python3
```

## 6. Install ESP-IDF

```bash
# Create installation directory
mkdir -p ~/esp
cd ~/esp

# Clone ESP-IDF
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git

# Install toolchain
cd esp-idf
./install.sh esp32s3

# Set environment (add to shell config recommended)
source ~/esp/esp-idf/export.sh
```

> **Note**: The ESP-IDF installation step uses `source ~/esp/esp-idf/export.sh`, but for day-to-day StampFly Ecosystem development, use `source setup_env.sh` instead.

## 7. Serial Port Driver

Driver for M5Stack products (CH9102F) is usually not needed. If not recognized:

```bash
# Check USB devices
ls /dev/tty.usb*
```

## 8. Verify Installation

```bash
# Activate development environment
source setup_env.sh

# Navigate to project
cd path/to/stampfly_ecosystem

# Run diagnostics
sf doctor

# Check version
sf version
```

> **Tip**: If you installed via `./install.sh` (see the [setup guide](README.md)), the
> final "Step 4/4: GUI Flasher" prompt offers to install the GUI flasher "StampFly
> Flasher" as a native app (default Yes). Once installed, it lands at
> `~/Applications/StampFlyFlasher.app` and appears in Launchpad.

## 9. Troubleshooting

### Python-related Errors

```bash
# Install pyserial
pip3 install pyserial
```

### USB Permission Errors

Usually not needed on macOS. If you encounter issues, check Security settings in System Preferences.
