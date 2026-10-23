# sf flash

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

ファームウェアをデバイスに書き込みます。ESP-IDF のビルド成果物（`build/flash_args`）を esptool で書き込みます。

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

### リセット方式

書き込み後は `esptool --after watchdog_reset` でリセットします（`idf.py flash` は使いません）。BOOT ボタンで ROM ダウンロードモードに入れた場合、esptool 既定の `hard_reset` ではダウンロードモードに留まりますが、`watchdog_reset` ならファームウェアが起動します。ESP-IDF の Kconfig は `hard_reset`/`no_reset` しか選べないためです。書き込み前に差分ビルドを自動実行します（`--build` 指定時は二重に行いません）。`-m` 指定時は書き込み成功後に `idf.py monitor` を別途起動します。

---

<a id="english"></a>

## 1. Overview

Flash firmware to device. Writes the ESP-IDF build output (`build/flash_args`) with esptool.

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

### Reset method

After writing, the chip is reset with `esptool --after watchdog_reset` (not `idf.py flash`). When ROM download mode was entered with the BOOT button, esptool's default `hard_reset` leaves the chip in download mode, whereas `watchdog_reset` boots the firmware; ESP-IDF's Kconfig only offers `hard_reset`/`no_reset`. An incremental build runs automatically before flashing (not duplicated when `--build` is given). With `-m`, `idf.py monitor` runs separately after a successful flash.
