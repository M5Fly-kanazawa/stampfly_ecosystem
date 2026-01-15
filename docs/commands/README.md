# sf CLI コマンドリファレンス

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`sf` は StampFly Ecosystem の統合コマンドラインツールです。ファームウェア開発、ログ取得、シミュレーション、キャリブレーションを一元管理します。

### インストール

```bash
cd stampfly-ecosystem
./install.sh
source ~/esp/esp-idf/export.sh
```

### 基本構文

```bash
sf <command> [subcommand] [options]
```

## 2. コマンド一覧

| コマンド | 説明 | ドキュメント |
|---------|------|-------------|
| `sf version` | バージョン情報表示 | [sf-version.md](sf-version.md) |
| `sf doctor` | 環境診断 | [sf-doctor.md](sf-doctor.md) |
| `sf build` | ファームウェアビルド | [sf-build.md](sf-build.md) |
| `sf flash` | ファームウェア書き込み | [sf-flash.md](sf-flash.md) |
| `sf monitor` | シリアルモニタ | [sf-monitor.md](sf-monitor.md) |
| `sf log` | ログキャプチャ・解析 | [sf-log.md](sf-log.md) |
| `sf sim` | シミュレータ | [sf-sim.md](sf-sim.md) |
| `sf cal` | センサキャリブレーション | [sf-cal.md](sf-cal.md) |

## 3. クイックリファレンス

### ファームウェア開発

```bash
# ビルド
sf build vehicle           # vehicleファームウェアをビルド
sf build controller        # controllerファームウェアをビルド
sf build vehicle -c        # クリーンビルド

# 書き込み
sf flash vehicle           # vehicleに書き込み
sf flash vehicle -m        # 書き込み後にモニタを開く

# モニタ
sf monitor                 # シリアルモニタを開く
```

### ログ・解析

```bash
# キャプチャ
sf log wifi -d 30          # WiFiで30秒キャプチャ
sf log capture -d 60       # USBシリアルで60秒キャプチャ

# 解析
sf log list                # ログファイル一覧
sf log info                # 最新ログの情報
sf log analyze             # フライト解析
```

### シミュレーション

```bash
sf sim list                # 利用可能なバックエンド一覧
sf sim run                 # VPythonシミュレータ起動
sf sim run genesis         # Genesisシミュレータ起動
sf sim headless -d 30      # 30秒ヘッドレス実行
```

### キャリブレーション

```bash
sf cal list                # キャリブレーション一覧
sf cal gyro                # ジャイロキャリブレーション
sf cal mag start           # 磁気キャリブレーション開始
sf cal mag save            # 磁気キャリブレーション保存
sf cal plot                # 磁気XYプロット
```

### 環境診断

```bash
sf doctor                  # 環境チェック
sf version                 # バージョン情報
```

## 4. グローバルオプション

| オプション | 説明 |
|-----------|------|
| `-h, --help` | ヘルプを表示 |
| `-V, --version` | バージョンを表示 |
| `--no-color` | カラー出力を無効化 |

---

<a id="english"></a>

## 1. Overview

`sf` is the integrated command-line tool for StampFly Ecosystem. It manages firmware development, log capture, simulation, and calibration in one place.

### Installation

```bash
cd stampfly-ecosystem
./install.sh
source ~/esp/esp-idf/export.sh
```

### Basic Syntax

```bash
sf <command> [subcommand] [options]
```

## 2. Command List

| Command | Description | Documentation |
|---------|-------------|---------------|
| `sf version` | Show version info | [sf-version.md](sf-version.md) |
| `sf doctor` | Environment diagnostics | [sf-doctor.md](sf-doctor.md) |
| `sf build` | Build firmware | [sf-build.md](sf-build.md) |
| `sf flash` | Flash firmware | [sf-flash.md](sf-flash.md) |
| `sf monitor` | Serial monitor | [sf-monitor.md](sf-monitor.md) |
| `sf log` | Log capture & analysis | [sf-log.md](sf-log.md) |
| `sf sim` | Simulator | [sf-sim.md](sf-sim.md) |
| `sf cal` | Sensor calibration | [sf-cal.md](sf-cal.md) |

## 3. Quick Reference

### Firmware Development

```bash
# Build
sf build vehicle           # Build vehicle firmware
sf build controller        # Build controller firmware
sf build vehicle -c        # Clean build

# Flash
sf flash vehicle           # Flash to vehicle
sf flash vehicle -m        # Flash and open monitor

# Monitor
sf monitor                 # Open serial monitor
```

### Log & Analysis

```bash
# Capture
sf log wifi -d 30          # Capture 30s via WiFi
sf log capture -d 60       # Capture 60s via USB serial

# Analysis
sf log list                # List log files
sf log info                # Latest log info
sf log analyze             # Flight analysis
```

### Simulation

```bash
sf sim list                # List available backends
sf sim run                 # Start VPython simulator
sf sim run genesis         # Start Genesis simulator
sf sim headless -d 30      # 30s headless run
```

### Calibration

```bash
sf cal list                # List calibration types
sf cal gyro                # Gyro calibration
sf cal mag start           # Start mag calibration
sf cal mag save            # Save mag calibration
sf cal plot                # Plot mag XY
```

### Diagnostics

```bash
sf doctor                  # Environment check
sf version                 # Version info
```

## 4. Global Options

| Option | Description |
|--------|-------------|
| `-h, --help` | Show help |
| `-V, --version` | Show version |
| `--no-color` | Disable colored output |
