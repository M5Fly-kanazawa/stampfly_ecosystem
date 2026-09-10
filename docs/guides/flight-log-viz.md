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

### Data Stream CSV（sf log wifi -o *.csv）の可視化

`sf log wifi -d 30 -o flight.csv` は400Hzの Data Stream CSV（IMUと姿勢推定
（ESKF: 拡張カルマンフィルタ）に、内側ループのレート指令・外側ループの角度
指令・推力・モータduty・flight_modeをマージした形式）を書き出します。
`sf log viz flight.csv` はこの形式を自動判別し、7段のパネルで表示します。
`motor_duty_FR/RR/RL/FL` 列は、400Hz duty エントリ（kPktDuty400）を送る
ファームでは400Hz実測、送らない旧ファームでは従来通り50Hz指令の前方補完です
（列名・列数は同じ）。`vbat` 列は1Hz Status パケット（0x4F）の
バッテリ電圧を各400Hz行へ前方補完したもので（未受信の行は空欄）、
`sf sysid fit --mixer vehicle`（firmware/vehicle の非線形モータ曲線逆算）
だけが使います。末尾の `ctrl_output_thrust`/`ctrl_output_torque_roll/pitch/yaw`/
`ctrl_output_rate_hz` 列は、400Hz control_output エントリ（kPktCtrlOutput400/
0x4B）を送るファームでのみ埋まる、ミキサー手前の指令推力[N]・トルク[Nm]です
（`sf sysid fit --input control_output`/`sf sysid rate-fit`が使う、ミキサーの
実装を問わないプラント入力。エントリが無いログでは空欄・`ctrl_output_rate_hz`
は0）。それ以外の用途では無視して構いません。

| パネル | 内容 | 単位 |
|-------|------|------|
| Roll Rate | ロールレート実測 vs 指令（ステップ応答の確認向け） | deg/s |
| Pitch Rate | ピッチレート実測 vs 指令 | deg/s |
| Yaw Rate | ヨーレート実測 vs 指令 | deg/s |
| Attitude | クォータニオンから求めたroll/pitch/yawと角度指令 | deg |
| Acceleration | 機体座標系の加速度 x/y/z | m/s^2 |
| Thrust and Motors | 総推力と4モータのduty | N / [0,1] |
| Gyro Bias / Flight Mode | ESKFのジャイロバイアス推定値とflight_mode | deg/s |

`--mode attitude` でレート3段+姿勢のみ、`--time-range 5 15` で時間範囲を
絞り込み、`--save FILE` で画像保存できます（`--mode` のセンサー系は
`--mode sensors` で加速度+生ジャイロに切り替わります）。

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

**プロットウィンドウが開かない／"non-interactive" 警告が出る**
```bash
# sf が自動でログの隣に <ログ名>.png を保存し、既定の画像ビューアで開く
# 詳しい原因と恒久的な直し方は docs/guides/troubleshooting.md
# 第6章「グラフ表示（matplotlib）」を参照
sf doctor   # "Checking plot window support" でGUIバックエンドの状態を確認できる
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

### Visualizing the Data Stream CSV (sf log wifi -o *.csv)

`sf log wifi -d 30 -o flight.csv` writes the 400Hz Data Stream CSV (IMU and
ESKF attitude estimate merged with the inner-loop rate reference, the
outer-loop angle reference, thrust, motor duty, and flight_mode).
`sf log viz flight.csv` auto-detects this format and renders it as 7
panels.

| Panel | Content | Unit |
|-------|---------|------|
| Roll Rate | Measured vs. commanded roll rate (for reading step responses) | deg/s |
| Pitch Rate | Measured vs. commanded pitch rate | deg/s |
| Yaw Rate | Measured vs. commanded yaw rate | deg/s |
| Attitude | Roll/pitch/yaw from quaternion, plus angle reference | deg |
| Acceleration | Body-frame acceleration x/y/z | m/s^2 |
| Thrust and Motors | Total thrust and the 4 motor duty channels | N / [0,1] |
| Gyro Bias / Flight Mode | ESKF gyro bias estimate and flight_mode | deg/s |

Use `--mode attitude` for the 3 rate panels plus attitude only, `--time-range
5 15` to restrict the time window, and `--save FILE` to save an image
(`--mode sensors` switches to acceleration + raw gyro).

## 4. Typical Workflow

```bash
# Capture -> Visualize
sf log wifi -d 30 -o logs/test.csv
sf log viz logs/test.csv --save overview.png
```

## 5. Troubleshooting

### Common Issues

**Cannot connect over WiFi**
```bash
# Check that you are connected to StampFly's AP (SSID: StampFly_XXXX)
# Specify the IP address explicitly
sf log wifi -i 192.168.10.1
```

**Log is empty or errors out**
```bash
# Check the log file's info
sf log info logs/flight.csv
```

**matplotlib error**
```bash
# Install the required packages
pip install matplotlib numpy scipy pyyaml
```

**Plot window does not open / "non-interactive" warning**
```bash
# sf now saves a <log>.png next to the log and opens it with the
# default image viewer automatically. For the cause and a permanent
# fix, see docs/guides/troubleshooting.md section 6
# "Plot Window (matplotlib)".
sf doctor   # "Checking plot window support" shows the GUI backend status
```
