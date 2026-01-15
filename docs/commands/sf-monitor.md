# sf monitor

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

シリアルモニタを開きます。ESP-IDF の `idf.py monitor` をラップします。

## 2. 構文

```bash
sf monitor [options]
```

### オプション

| オプション | 説明 |
|-----------|------|
| `-p, --port` | シリアルポート（自動検出） |
| `-b, --baud` | ボーレート（デフォルト: 115200） |
| `-t, --target` | ターゲット（vehicle/controller） |

## 3. 使用例

```bash
# モニタを開く
sf monitor

# ポートを指定
sf monitor -p /dev/ttyUSB0

# ボーレートを指定
sf monitor -b 921600
```

## 4. モニタ操作

| キー | 動作 |
|------|------|
| `Ctrl+]` | モニタ終了 |
| `Ctrl+T` | メニュー表示 |
| `Ctrl+T` → `H` | ヘルプ |
| `Ctrl+T` → `R` | リセット |

## 5. ファームウェアCLIコマンド

モニタ内で使用できるファームウェアCLIコマンド：

| コマンド | 説明 |
|---------|------|
| `help` | コマンド一覧 |
| `status` | システム状態 |
| `sensor` | センサデータ表示 |
| `calib gyro` | ジャイロキャリブレーション |
| `magcal start` | 磁気キャリブレーション開始 |
| `motor test` | モーターテスト |
| `gain` | PIDゲイン設定 |
| `trim` | トリム調整 |
| `fftmode on` | FFTモード有効化 |

---

<a id="english"></a>

## 1. Overview

Open serial monitor. Wraps ESP-IDF's `idf.py monitor`.

## 2. Syntax

```bash
sf monitor [options]
```

### Options

| Option | Description |
|--------|-------------|
| `-p, --port` | Serial port (auto-detect) |
| `-b, --baud` | Baud rate (default: 115200) |
| `-t, --target` | Target (vehicle/controller) |

## 3. Examples

```bash
# Open monitor
sf monitor

# Specify port
sf monitor -p /dev/ttyUSB0
```

## 4. Monitor Controls

| Key | Action |
|-----|--------|
| `Ctrl+]` | Exit monitor |
| `Ctrl+T` | Show menu |
| `Ctrl+T` → `R` | Reset |

## 5. Firmware CLI Commands

Commands available in monitor:

| Command | Description |
|---------|-------------|
| `help` | List commands |
| `status` | System status |
| `sensor` | Show sensor data |
| `calib gyro` | Gyro calibration |
| `magcal start` | Start mag calibration |
| `motor test` | Motor test |
