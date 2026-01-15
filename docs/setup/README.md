# セットアップガイド / Setup Guide

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFly Ecosystemの開発環境セットアップガイドです。

## 2. 必要要件

| 項目 | 要件 |
|------|------|
| OS | macOS, Linux, Windows (WSL2推奨) |
| Python | 3.9以上 |
| ESP-IDF | v5.4以上（v5.5.2推奨） |
| Git | 最新版 |

## 3. クイックスタート

### ステップ 1: リポジトリをクローン

```bash
git clone https://github.com/your-org/stampfly-ecosystem.git
cd stampfly-ecosystem
```

### ステップ 2: インストーラを実行

```bash
./install.sh
```

これにより以下がインストールされます:
- sf CLI（コマンドラインツール）
- VPythonシミュレータ依存（vpython, pygame等）
- 解析ツール依存（numpy, matplotlib等）

> **Note**: ESP-IDFが未インストールの場合、インストーラが案内します。
> プラットフォーム別の詳細は [macOS](macos.md) / [Linux](linux.md) / [Windows](windows.md) を参照。

### ステップ 3: sf CLIの確認

```bash
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

# sf CLIが利用可能か確認
sf version
```

### ステップ 4: 環境診断

```bash
sf doctor
```

## 4. 基本的なワークフロー

```bash
# 1. ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

# 2. ファームウェアをビルド
sf build vehicle

# 3. デバイスに書き込み
sf flash vehicle

# 4. シリアルモニタを開く
sf monitor
```

## 5. シミュレータのセットアップ

### VPythonシミュレータ（デフォルトでインストール済み）

```bash
sf sim run vpython
```

### Genesisシミュレータ（オプション）

Genesisは高精度物理エンジンですが、PyTorch（~2GB）を含む大きな依存があります。

```bash
# Genesisをインストール
sf setup genesis

# Genesisを起動
sf sim run genesis
```

## 6. プラットフォーム別ガイド

| プラットフォーム | ガイド |
|-----------------|--------|
| macOS | [macos.md](macos.md) |
| Linux (Ubuntu/Debian) | [linux.md](linux.md) |
| Windows | [windows.md](windows.md) |

---

<a id="english"></a>

## 1. Overview

Setup guide for StampFly Ecosystem development environment.

## 2. Requirements

| Item | Requirement |
|------|-------------|
| OS | macOS, Linux, Windows (WSL2 recommended) |
| Python | 3.9 or later |
| ESP-IDF | v5.4 or later (v5.5.2 recommended) |
| Git | Latest version |

## 3. Quick Start

### Step 1: Clone Repository

```bash
git clone https://github.com/your-org/stampfly-ecosystem.git
cd stampfly-ecosystem
```

### Step 2: Run Installer

```bash
./install.sh
```

This installs:
- sf CLI (command-line tool)
- VPython simulator dependencies (vpython, pygame, etc.)
- Analysis tool dependencies (numpy, matplotlib, etc.)

> **Note**: If ESP-IDF is not installed, the installer will guide you.
> See platform guides for details: [macOS](macos.md) / [Linux](linux.md) / [Windows](windows.md)

### Step 3: Verify sf CLI

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Verify sf CLI is available
sf version
```

### Step 4: Run Diagnostics

```bash
sf doctor
```

## 4. Basic Workflow

```bash
# 1. Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# 2. Build firmware
sf build vehicle

# 3. Flash to device
sf flash vehicle

# 4. Open serial monitor
sf monitor
```

## 5. Simulator Setup

### VPython Simulator (Installed by Default)

```bash
sf sim run vpython
```

### Genesis Simulator (Optional)

Genesis is a high-precision physics engine but has large dependencies including PyTorch (~2GB).

```bash
# Install Genesis
sf setup genesis

# Run Genesis
sf sim run genesis
```

## 6. Platform Guides

| Platform | Guide |
|----------|-------|
| macOS | [macos.md](macos.md) |
| Linux (Ubuntu/Debian) | [linux.md](linux.md) |
| Windows | [windows.md](windows.md) |
