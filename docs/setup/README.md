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

### ステップ 2: ESP-IDFをインストール

プラットフォーム別ガイドを参照:
- [macOS](macos.md)
- [Linux](linux.md)
- [Windows](windows.md)

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

## 5. プラットフォーム別ガイド

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

### Step 2: Install ESP-IDF

See platform-specific guides:
- [macOS](macos.md)
- [Linux](linux.md)
- [Windows](windows.md)

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

## 5. Platform Guides

| Platform | Guide |
|----------|-------|
| macOS | [macos.md](macos.md) |
| Linux (Ubuntu/Debian) | [linux.md](linux.md) |
| Windows | [windows.md](windows.md) |
