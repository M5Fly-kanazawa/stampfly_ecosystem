# sf CLI コマンドリファレンス

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`sf` は StampFly Ecosystem の統合コマンドラインツールです。ファームウェア開発、ログ取得、シミュレーション、キャリブレーションを一元管理します。

### インストール

```bash
cd stampfly_ecosystem
./install.sh
source setup_env.sh
```

### 基本構文

```bash
sf <command> [subcommand] [options]
```

## 2. コマンド一覧

`source setup_env.sh && sf --help` の実測（44 コマンド、2026-09-13 時点）。専用ページが無いコマンドは
`sf <command> --help` で詳細を確認してください。

| コマンド | 説明 | リファレンス |
|---------|------|-------------|
| `sf version` | バージョン情報表示 | [sf-version.md](sf-version.md) |
| `sf doctor` | 環境診断 | [sf-doctor.md](sf-doctor.md) |
| `sf setup` | 追加の依存パッケージをインストール | [sf-setup.md](sf-setup.md) |
| `sf build` | ファームウェアビルド | [sf-build.md](sf-build.md) |
| `sf flash` | ファームウェア書き込み | [sf-flash.md](sf-flash.md) |
| `sf flasher` | ネイティブ GUI フラッシャアプリの管理 | [sf-flasher.md](sf-flasher.md) |
| `sf monitor` | シリアルモニタを開く | [sf-monitor.md](sf-monitor.md) |
| `sf telemetry` | 50Hz テレメトリのライブ表示（既定=ターミナル、`--web` でブラウザ） | `sf telemetry --help` |
| `sf blocks` | Blockly ブロックプログラミング連携（ブラウザ UI ↔ 機体 UDP API） | `sf blocks --help` |
| `sf log` | ログ取得・解析 | [sf-log.md](sf-log.md) |
| `sf sim` | フライトシミュレータ実行 | [sf-sim.md](sf-sim.md) |
| `sf sils`（`sf sil`） | SILS 試験環境（閉ループホバー・レビュー動画・合否判定） | `sf sils --help` |
| `sf cal` | センサキャリブレーション | [sf-cal.md](sf-cal.md) |
| `sf sysid` | フライトログからのシステム同定 | [sf-sysid.md](sf-sysid.md) |
| `sf params` | 物理パラメータ整合検査（C_T・C_Q・κ・慣性 等） | `sf params --help` |
| `sf trim` | ホバリングログから平衡姿勢トリムを同定 | `sf trim --help` |
| `sf takeoff` | 指定高度まで離陸 | [flight-commands.md](flight-commands.md) |
| `sf land` | 着陸 | [flight-commands.md](flight-commands.md) |
| `sf hover` | 指定高度で指定時間ホバリング | [flight-commands.md](flight-commands.md) |
| `sf jump` | クイックジャンプ（上昇後下降） | [flight-commands.md](flight-commands.md) |
| `sf up` | 指定距離 (cm) だけ上昇 | [flight-commands.md](flight-commands.md) |
| `sf down` | 指定距離 (cm) だけ下降 | [flight-commands.md](flight-commands.md) |
| `sf cw` | 時計回りに回転（度） | [flight-commands.md](flight-commands.md) |
| `sf ccw` | 反時計回りに回転（度） | [flight-commands.md](flight-commands.md) |
| `sf emergency` | 緊急モーター停止 | [flight-commands.md](flight-commands.md) |
| `sf forward` | 前方へ指定距離 (cm) 移動 | [flight-commands.md](flight-commands.md) |
| `sf back` | 後方へ指定距離 (cm) 移動 | [flight-commands.md](flight-commands.md) |
| `sf left` | 左へ指定距離 (cm) 移動 | [flight-commands.md](flight-commands.md) |
| `sf right` | 右へ指定距離 (cm) 移動 | [flight-commands.md](flight-commands.md) |
| `sf stop` | 停止して現在位置でホバリング | [flight-commands.md](flight-commands.md) |
| `sf motor` | ベンチモータ試験／CW-CCW 電流スイープ（DISARM 時のみ） | `sf motor --help` |
| `sf battery` | バッテリー残量 (%) を取得 | [query-commands.md](query-commands.md) |
| `sf height` | ESKF 推定高度 (cm) を取得 | [query-commands.md](query-commands.md) |
| `sf tof` | ToF 底面距離 (cm) を取得 | [query-commands.md](query-commands.md) |
| `sf baro` | 気圧高度 (cm) を取得 | [query-commands.md](query-commands.md) |
| `sf attitude` | 姿勢角 (deg) を取得 | [query-commands.md](query-commands.md) |
| `sf acceleration` | 加速度 (cm/s²) を取得 | [query-commands.md](query-commands.md) |
| `sf speed` | 設定速度 (cm/s) を取得 | [query-commands.md](query-commands.md) |
| `sf rc` | RC 操作（単発またはキーボード対話） | [sf-rc.md](sf-rc.md) |
| `sf lesson` | 実習管理 | [sf-lesson.md](sf-lesson.md) |
| `sf competition` | ワークショップ競技会用ツール | [sf-competition.md](sf-competition.md) |
| `sf app` | 自分のドローンファームプロジェクトを管理 | [sf-app.md](sf-app.md) |
| `sf docs` | ドキュメントサイトの配信・ビルド | [sf-docs.md](sf-docs.md) |
| `sf upgrade` | 最新版を取得し環境を再同期 | [sf-upgrade.md](sf-upgrade.md) |

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
sf upgrade                 # 最新版に更新
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
cd stampfly_ecosystem
./install.sh
source setup_env.sh
```

### Basic Syntax

```bash
sf <command> [subcommand] [options]
```

## 2. Command List

Measured from `source setup_env.sh && sf --help` (44 commands, as of 2026-09-13). For a
command with no dedicated page, run `sf <command> --help` for details.

| Command | Description | Reference |
|---------|-------------|-----------|
| `sf version` | Show version information | [sf-version.md](sf-version.md) |
| `sf doctor` | Diagnose environment issues | [sf-doctor.md](sf-doctor.md) |
| `sf setup` | Install optional dependencies | [sf-setup.md](sf-setup.md) |
| `sf build` | Build firmware | [sf-build.md](sf-build.md) |
| `sf flash` | Flash firmware to device | [sf-flash.md](sf-flash.md) |
| `sf flasher` | Manage the native StampFly Flasher GUI app | [sf-flasher.md](sf-flasher.md) |
| `sf monitor` | Open serial monitor | [sf-monitor.md](sf-monitor.md) |
| `sf telemetry` | Live 50Hz telemetry — terminal dashboard, or browser with `--web` | `sf telemetry --help` |
| `sf blocks` | Blockly block-programming bridge (browser UI <-> drone UDP API) | `sf blocks --help` |
| `sf log` | Log capture and analysis | [sf-log.md](sf-log.md) |
| `sf sim` | Run flight simulator | [sf-sim.md](sf-sim.md) |
| `sf sils` (`sf sil`) | Software-in-the-Loop bench (closed-loop hover, review video, gate) | `sf sils --help` |
| `sf cal` | Sensor calibration | [sf-cal.md](sf-cal.md) |
| `sf sysid` | System identification from flight logs | [sf-sysid.md](sf-sysid.md) |
| `sf params` | Physical parameter consistency audit (C_T, C_Q, kappa, inertia, ...) | `sf params --help` |
| `sf trim` | Identify equilibrium attitude trim from hover logs | `sf trim --help` |
| `sf takeoff` | Take off to specified altitude | [flight-commands.md](flight-commands.md) |
| `sf land` | Land the vehicle | [flight-commands.md](flight-commands.md) |
| `sf hover` | Hover at altitude for duration | [flight-commands.md](flight-commands.md) |
| `sf jump` | Quick jump: climb then descend | [flight-commands.md](flight-commands.md) |
| `sf up` | Move up by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf down` | Move down by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf cw` | Rotate clockwise (degrees) | [flight-commands.md](flight-commands.md) |
| `sf ccw` | Rotate counter-clockwise (degrees) | [flight-commands.md](flight-commands.md) |
| `sf emergency` | Emergency motor stop | [flight-commands.md](flight-commands.md) |
| `sf forward` | Move forward by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf back` | Move backward by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf left` | Move left by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf right` | Move right by distance (cm) | [flight-commands.md](flight-commands.md) |
| `sf stop` | Stop and hover at current position | [flight-commands.md](flight-commands.md) |
| `sf motor` | Bench motor test / CW-CCW current sweep (disarmed only) | `sf motor --help` |
| `sf battery` | Query battery level (%) | [query-commands.md](query-commands.md) |
| `sf height` | Query ESKF estimated height (cm) | [query-commands.md](query-commands.md) |
| `sf tof` | Query ToF bottom distance (cm) | [query-commands.md](query-commands.md) |
| `sf baro` | Query barometric altitude (cm) | [query-commands.md](query-commands.md) |
| `sf attitude` | Query attitude angles (deg) | [query-commands.md](query-commands.md) |
| `sf acceleration` | Query acceleration (cm/s2) | [query-commands.md](query-commands.md) |
| `sf speed` | Query configured speed (cm/s) | [query-commands.md](query-commands.md) |
| `sf rc` | RC control (one-shot or interactive keyboard) | [sf-rc.md](sf-rc.md) |
| `sf lesson` | Lesson management | [sf-lesson.md](sf-lesson.md) |
| `sf competition` | Workshop competition tools | [sf-competition.md](sf-competition.md) |
| `sf app` | Manage your own drone-firmware projects | [sf-app.md](sf-app.md) |
| `sf docs` | Serve and build documentation site | [sf-docs.md](sf-docs.md) |
| `sf upgrade` | Pull the latest changes and resync the environment | [sf-upgrade.md](sf-upgrade.md) |

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
sf upgrade                 # Update to the latest version
```

## 4. Global Options

| Option | Description |
|--------|-------------|
| `-h, --help` | Show help |
| `-V, --version` | Show version |
| `--no-color` | Disable colored output |
