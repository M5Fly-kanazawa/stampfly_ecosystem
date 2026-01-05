# log_capture

StampFlyデバイスからUSBシリアル経由でバイナリログをキャプチャするツール。

## 必要なライブラリ

```bash
pip install pyserial numpy
```

## パケットフォーマット

| バージョン | サイズ | ヘッダー | 内容 | 状態 |
|-----------|--------|---------|------|------|
| V2 | 128 bytes | 0xAA 0x56 | センサデータ + ESKF推定結果 | **現行** |
| V1 | 64 bytes | 0xAA 0x55 | センサデータのみ | 非推奨 |

> **注意**: V1は非推奨です。すべてのPythonツールはV2のみをサポートします。

## 使い方

### ログキャプチャ

```bash
# 基本的な使い方
python log_capture.py capture --port /dev/tty.usbmodem* --output sensor.bin --duration 60

# ライブ表示付き
python log_capture.py capture -p /dev/tty.usbmodem* -o sensor.bin -d 30 --live

# デバッグモード
python log_capture.py capture -p /dev/tty.usbmodem* -o sensor.bin -d 30 --debug
```

### CSV変換

```bash
python log_capture.py convert --input sensor.bin --output sensor.csv
```

### ログ情報表示

```bash
python log_capture.py info sensor.bin
```

## オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `--port`, `-p` | シリアルポート | 必須 |
| `--output`, `-o` | 出力ファイル | 必須 |
| `--duration`, `-d` | キャプチャ時間（秒） | 60 |
| `--baudrate`, `-b` | ボーレート | 115200 |
| `--live`, `-l` | リアルタイム表示 | - |
| `--no-auto` | binlog on/offを送信しない | - |
| `--debug` | デバッグ出力 | - |

## シリアルポートの確認

```bash
# macOS
ls /dev/tty.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

## 関連ツール

- `tools/log_analyzer/` - 取得したログの可視化・解析
- `tools/calibration/` - キャリブレーション確認

---

# log_capture

Tool for capturing binary logs from StampFly device via USB serial.

## Requirements

```bash
pip install pyserial numpy
```

## Usage

### Capture

```bash
python log_capture.py capture -p /dev/tty.usbmodem* -o sensor.bin -d 60
```

### Convert to CSV

```bash
python log_capture.py convert --input sensor.bin --output sensor.csv
```

### Show Info

```bash
python log_capture.py info sensor.bin
```
