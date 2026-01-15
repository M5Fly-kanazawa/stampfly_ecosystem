# sf flash

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

ファームウェアをデバイスに書き込みます。ESP-IDF の `idf.py flash` をラップします。

## 2. 構文

```bash
sf flash [target] [options]
```

### 引数

| 引数 | 説明 | デフォルト |
|------|------|-----------|
| `target` | 書き込み対象（vehicle または controller） | vehicle |

### オプション

| オプション | 説明 |
|-----------|------|
| `-p, --port` | シリアルポート（自動検出） |
| `-b, --baud` | ボーレート |
| `-m, --monitor` | 書き込み後にモニタを開く |

## 3. 使用例

```bash
# vehicleに書き込み
sf flash vehicle

# 書き込み後にモニタを開く
sf flash vehicle -m

# ポートを指定
sf flash vehicle -p /dev/ttyUSB0

# controllerに書き込み
sf flash controller
```

## 4. ワークフロー

典型的な開発ワークフロー：

```bash
# ビルドして書き込み、モニタを開く
sf build vehicle && sf flash vehicle -m
```

---

<a id="english"></a>

## 1. Overview

Flash firmware to device. Wraps ESP-IDF's `idf.py flash`.

## 2. Syntax

```bash
sf flash [target] [options]
```

### Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `target` | Flash target (vehicle or controller) | vehicle |

### Options

| Option | Description |
|--------|-------------|
| `-p, --port` | Serial port (auto-detect) |
| `-b, --baud` | Baud rate |
| `-m, --monitor` | Open monitor after flash |

## 3. Examples

```bash
# Flash vehicle
sf flash vehicle

# Flash and open monitor
sf flash vehicle -m

# Build and flash workflow
sf build vehicle && sf flash vehicle -m
```
