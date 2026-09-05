# フライトログの取得と可視化チュートリアル

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このチュートリアルについて

StampFlyのフライトログを取得し、`sf log viz` で可視化する手順を説明します。

### 前提条件

- 開発環境がセットアップ済み（`source setup_env.sh`）
- StampFlyがWiFiモードで起動している
- Python 3.x と必要なライブラリ（numpy, matplotlib）がインストール済み

## 2. ログ取得

### WiFi経由でのログ取得

StampFlyのWiFi APに接続した状態で、以下のコマンドを実行：

```bash
# 開発環境のセットアップ
source setup_env.sh

# WiFi経由で400Hzテレメトリを取得（30秒間）
sf log wifi -d 30 -o logs/flight_001.csv
```

オプション：

| オプション | 説明 |
|-----------|------|
| `-d, --duration` | 取得時間（秒）デフォルト: 30 |
| `-o, --output` | 出力ファイル名 |
| `-i, --ip` | StampFlyのIPアドレス（デフォルト: 192.168.10.1） |
| `--fft` | 取得後にFFT解析を実行 |

### USB経由でのログ取得

バイナリログをUSB経由で取得する場合：

```bash
# USB経由でバイナリログを取得（60秒間）
sf log capture -d 60 -o logs/flight_001.bin

# CSVに変換
sf log convert logs/flight_001.bin
```

### ログファイルの確認

```bash
# 最近のログファイル一覧
sf log list

# ログファイルの情報を表示
sf log info logs/flight_001.csv
```

出力例：
```
File: flight_001.csv
Size: 2345.6 KB
Samples: 12000
Columns: 35
Duration: 30.00 seconds
Rate: 400.0 Hz
```

## 3. 基本的な可視化

### sf log viz を使用

最もシンプルな可視化方法：

```bash
# 最新のCSVログを可視化
sf log viz

# 特定のファイルを可視化
sf log viz logs/flight_001.csv

# 画像として保存
sf log viz logs/flight_001.csv --save flight_analysis.png

# 時間範囲を指定
sf log viz logs/flight_001.csv --time-range 5 15
```

表示モード：

| モード | 説明 |
|-------|------|
| `--mode all` | 全データ（デフォルト） |
| `--mode sensors` | センサーデータのみ |
| `--mode attitude` | 姿勢のみ |
| `--mode position` | 位置のみ |

## 4. 典型的なワークフロー

### 基本フロー

```bash
# 1. ログ取得
sf log wifi -d 30 -o logs/test_flight.csv

# 2. 基本可視化
sf log viz logs/test_flight.csv --save test_flight.png
```

## 5. トラブルシューティング

### よくある問題

**WiFi接続できない**
```bash
# StampFlyのAP（SSID: StampFly_XXXX）に接続されているか確認
# IPアドレスを明示的に指定
sf log wifi -i 192.168.10.1
```

**ログが空またはエラー**
```bash
# ログファイルの情報を確認
sf log info logs/flight.csv
```

**matplotlibエラー**
```bash
# 必要なパッケージをインストール
pip install matplotlib numpy scipy pyyaml
```

---

<a id="english"></a>

## 1. Overview

### About This Tutorial

This tutorial explains how to capture flight logs from StampFly and visualize them with `sf log viz`.

### Prerequisites

- Development environment set up (`source setup_env.sh`)
- StampFly running in WiFi mode
- Python 3.x with required libraries (numpy, matplotlib)

## 2. Log Capture

### WiFi Telemetry Capture

Connect to StampFly's WiFi AP and run:

```bash
# Activate development environment
source setup_env.sh

# Capture 400Hz telemetry via WiFi (30 seconds)
sf log wifi -d 30 -o logs/flight_001.csv
```

### USB Binary Log Capture

```bash
# Capture binary log via USB (60 seconds)
sf log capture -d 60 -o logs/flight_001.bin

# Convert to CSV
sf log convert logs/flight_001.bin
```

## 3. Basic Visualization

```bash
# Visualize latest CSV log
sf log viz

# Visualize specific file
sf log viz logs/flight_001.csv

# Save as image
sf log viz logs/flight_001.csv --save flight_analysis.png
```

## 4. Typical Workflow

```bash
# Capture -> Visualize
sf log wifi -d 30 -o logs/test.csv
sf log viz logs/test.csv --save overview.png
```
