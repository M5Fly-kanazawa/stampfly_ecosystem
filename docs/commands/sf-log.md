# sf log

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

テレメトリログのキャプチャと解析を行います。USBシリアルとWiFi両方の取得方法をサポートします。

## 2. サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | ログファイル一覧 |
| `capture` | USBシリアルでバイナリログ取得 |
| `wifi` | WiFi WebSocketでテレメトリ取得 |
| `convert` | バイナリ→CSV変換 |
| `info` | ログファイル情報表示 |
| `analyze` | フライトログ解析 |

## 3. sf log list

ログファイルの一覧を表示します。

```bash
sf log list              # 最新20件を表示
sf log list -n 50        # 最新50件を表示
sf log list --all        # 全件表示
```

## 4. sf log wifi

WiFi経由で高レートテレメトリを取得します（FFTモード対応）。

```bash
sf log wifi                    # 30秒キャプチャ（デフォルト）
sf log wifi -d 60              # 60秒キャプチャ
sf log wifi -d 30 --fft        # キャプチャ後にFFT解析
sf log wifi --no-save          # ファイル保存せず統計のみ表示
sf log wifi -o flight.csv      # 出力ファイル指定
```

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-d, --duration` | キャプチャ時間（秒） | 30 |
| `-o, --output` | 出力ファイル名 | 自動生成 |
| `-i, --ip` | StampFly IPアドレス | 192.168.4.1 |
| `--port` | WebSocketポート | 80 |
| `--fft` | キャプチャ後にFFT解析 | - |
| `--no-save` | ファイル保存しない | - |

### ワークフロー

1. USBでStampFlyに接続し、`fftmode on` を実行
2. USBを切断、バッテリーで電源ON
3. PCをStampFly WiFi APに接続
4. `sf log wifi -d 30 --fft` を実行

## 5. sf log capture

USBシリアル経由でバイナリログを取得します。

```bash
sf log capture                  # 60秒キャプチャ（デフォルト）
sf log capture -d 120           # 120秒キャプチャ
sf log capture -p /dev/ttyUSB0  # ポート指定
sf log capture --live           # リアルタイム表示
```

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-d, --duration` | キャプチャ時間（秒） | 60 |
| `-p, --port` | シリアルポート | 自動検出 |
| `-o, --output` | 出力ファイル名 | 自動生成 |
| `-b, --baudrate` | ボーレート | 115200 |
| `--live` | リアルタイム表示 | - |
| `--no-auto` | binlog on/off 自動送信しない | - |

## 6. sf log analyze

フライトログを解析し、安定性やPIDチューニングの情報を表示します。

```bash
sf log analyze                  # 最新CSVを解析
sf log analyze flight.csv       # 指定ファイルを解析
sf log analyze --fft            # FFT解析も実行
sf log analyze --no-plot        # グラフ表示しない
```

### 出力内容

- ジャイロ統計（平均、標準偏差、範囲）
- コントローラ入力統計
- 振動周波数解析
- 入力-応答相関
- 時間区間別安定性
- PIDチューニング推奨事項

## 7. sf log info

ログファイルの情報を表示します。

```bash
sf log info                     # 最新ログの情報
sf log info sensor.bin          # バイナリログの情報
sf log info flight.csv          # CSVログの情報
```

## 8. sf log convert

バイナリログをCSVに変換します。

```bash
sf log convert sensor.bin       # sensor.csv に変換
sf log convert sensor.bin -o out.csv  # 出力ファイル指定
```

---

<a id="english"></a>

## 1. Overview

Capture and analyze telemetry logs. Supports both USB serial and WiFi capture methods.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List log files |
| `capture` | Capture binary log via USB serial |
| `wifi` | Capture telemetry via WiFi WebSocket |
| `convert` | Convert binary to CSV |
| `info` | Show log file information |
| `analyze` | Analyze flight log |

## 3. sf log wifi

Capture high-rate telemetry via WiFi (FFT mode supported).

```bash
sf log wifi                    # 30s capture (default)
sf log wifi -d 60              # 60s capture
sf log wifi -d 30 --fft        # FFT analysis after capture
sf log wifi --no-save          # Show stats only, don't save
```

### Workflow

1. Connect USB to StampFly, run `fftmode on`
2. Disconnect USB, power on with battery
3. Connect PC to StampFly WiFi AP
4. Run `sf log wifi -d 30 --fft`

## 4. sf log analyze

Analyze flight log for stability and PID tuning insights.

```bash
sf log analyze                  # Analyze latest CSV
sf log analyze flight.csv       # Analyze specific file
sf log analyze --fft            # Include FFT analysis
```

### Output

- Gyro statistics (mean, std, range)
- Controller input statistics
- Oscillation frequency analysis
- Input-response correlation
- Time segment stability
- PID tuning recommendations
