# セットアップガイド / Setup Guide

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFly Ecosystemの開発環境セットアップガイドです。

## 2. 必要要件

既定のインストール（**専用環境**。以下の表と本節を参照）は、参加者のPCにあるPythonや
ESP-IDFに一切依存しません。

| 項目 | 要件 |
|------|------|
| OS | macOS, Linux, Windows 10 バージョン1803（2018年4月更新）以降 |
| Python | **不要**（専用のPython 3.12がインストーラによって自動導入されます） |
| ESP-IDF | 不要（専用のESP-IDF v5.5.2が自動導入されます） |
| Git | 最新版 |
| ディスク容量 | 約4〜6 GB |

開発者向けに、既存のESP-IDF・システムPythonをそのまま使う**旧来モード**も
`--use-existing-idf`（または `--idf-path`）で選べます。この場合はシステムPython
3.10〜3.12が必要です。

## 3. クイックスタート

### ステップ 1: リポジトリをクローン

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### ステップ 2: インストーラを実行

```bash
./install.sh
```

既定では、このエコシステム専用のPython 3.12・ESP-IDF v5.5.2・ツール一式を
`SF_HOME`（macOS/Linuxは `~/.stampfly`、Windowsは `C:\StampFly`。環境変数
`SF_HOME` で上書き可）の下に自己完結させてインストールします。加えて以下も
インストールされます:
- sf CLI（コマンドラインツール）
- VPythonシミュレータ依存（vpython, pygame等）
- 解析ツール依存（numpy, matplotlib等）

> **Note**: プラットフォーム別の詳細（配置場所の一覧・旧来モードへの切替方法・
> アンインストール手順を含む）は [macOS](macos.md) / [Linux](linux.md) /
> [Windows](windows.md) を参照。

### ステップ 3: sf CLIの確認

```bash
# 開発環境のセットアップ
source setup_env.sh

# sf CLIが利用可能か確認
sf version
```

### ステップ 4: 環境診断

```bash
sf doctor
```

## 4. 基本的なワークフロー

```bash
# 1. 開発環境のセットアップ
source setup_env.sh

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

The default install (the **dedicated environment** -- see the table and this section)
does not depend on any Python or ESP-IDF already on your machine.

| Item | Requirement |
|------|-------------|
| OS | macOS, Linux, Windows 10 version 1803 (April 2018 Update) or later |
| Python | **Not required** (a private Python 3.12 is installed automatically) |
| ESP-IDF | Not required (a private ESP-IDF v5.5.2 is installed automatically) |
| Git | Latest version |
| Disk space | About 4-6 GB |

Developers can opt into a **legacy mode** that uses an existing ESP-IDF and system
Python instead, via `--use-existing-idf` (or `--idf-path`). That mode requires a
system Python 3.10-3.12.

## 3. Quick Start

### Step 1: Clone Repository

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### Step 2: Run Installer

```bash
./install.sh
```

By default, this installs a private Python 3.12, ESP-IDF v5.5.2, and toolchain for
this ecosystem alone, self-contained under `SF_HOME` (macOS/Linux: `~/.stampfly`;
Windows: `C:\StampFly`; override with the `SF_HOME` environment variable). It also
installs:
- sf CLI (command-line tool)
- VPython simulator dependencies (vpython, pygame, etc.)
- Analysis tool dependencies (numpy, matplotlib, etc.)

> **Note**: See the platform guides for details -- including the full layout, how to
> switch to legacy mode, and uninstall steps: [macOS](macos.md) / [Linux](linux.md) /
> [Windows](windows.md)

### Step 3: Verify sf CLI

```bash
# Activate development environment
source setup_env.sh

# Verify sf CLI is available
sf version
```

### Step 4: Run Diagnostics

```bash
sf doctor
```

## 4. Basic Workflow

```bash
# 1. Activate development environment
source setup_env.sh

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
