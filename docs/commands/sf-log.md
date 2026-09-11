# sf log

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

テレメトリログのキャプチャ・変換・解析・可視化を行います。USBシリアル（バイナリ）とWiFi（UDP）両方の取得方法をサポートします。

| 取得経路 | サブコマンド | 形式 |
|---------|-------------|------|
| USBシリアル | `sf log capture` | バイナリ（`.bin`）。`sf log convert` でCSVへ変換 |
| WiFi（UDP、400Hz） | `sf log wifi` | 既定は1サンプル1行の `.jsonl`。出力を `.csv` 指定するとマージ済みData Stream CSV |

## 2. サブコマンド一覧

| サブコマンド | 説明 |
|-------------|------|
| `list` | ログファイル一覧 |
| `capture` | USBシリアルでバイナリログ取得 |
| `wifi` | WiFi UDPで400Hzテレメトリ取得 |
| `convert` | バイナリ→CSV変換 |
| `info` | ログファイル情報表示 |
| `analyze` | フライトログ解析（`--health`でモータ健全性診断） |
| `viz` | ログ可視化 |

## 3. sf log list

`logs/` と `tools/log_analyzer/` 配下のログファイル一覧を、更新日時の新しい順に表示します。

```bash
sf log list              # 直近20件を表示（既定）
sf log list -n 50        # 直近50件を表示
sf log list --all        # 全件表示（件数制限を無視）
```

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `-n, --limit` | 表示する直近ファイル数 | 20 |
| `--all` | 全件表示（`--limit` を無視） | - |

**注意:** `*.bin` と `*.csv` のみを走査する。`sf log wifi` の既定出力である `.jsonl` はこの一覧に表示されない（`sf log info`/`sf log analyze --health` へは直接パスを指定すればよい）。

## 4. sf log capture

USBシリアル経由でバイナリログ（`.bin`）を取得します。

```bash
sf log capture                  # 60秒キャプチャ（既定）、ポート自動検出
sf log capture -d 120           # 120秒キャプチャ
sf log capture -p /dev/ttyUSB0  # ポート指定
sf log capture --live           # リアルタイムでパケットを表示
sf log capture --no-auto        # binlog on/off を自動送信しない
```

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `-p, --port` | シリアルポート | 自動検出 |
| `-o, --output` | 出力ファイル名 | 自動生成 |
| `-d, --duration` | キャプチャ時間（秒） | 60 |
| `-b, --baudrate` | ボーレート | 115200 |
| `--live` | パケットをリアルタイム表示 | - |
| `--no-auto` | `binlog on`/`off` を自動送信しない | - |
| `--debug` | デバッグ出力を有効化 | - |

## 5. sf log wifi

WiFi経由（**UDP**、WebSocketではない）で400Hzのフルレートテレメトリを取得します。StampFlyの電源を入れてWiFi APに接続するだけでよく、USBで事前に `fftmode on` を実行する手順は不要です（現行ファームにその設定項目はありません）。

```bash
sf log wifi                              # 30秒キャプチャ、自動生成の.jsonlに保存（既定）
sf log wifi -d 60                        # 60秒キャプチャ
sf log wifi -o flight.csv                # マージ済みData Stream CSV（400Hz、sf sysid fit 用）として保存
sf log wifi --no-save                    # 保存せず統計のみ表示
sf log wifi -i 192.168.10.5 --port 8890  # IPアドレス/ポートを明示指定
```

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `-o, --output` | 出力ファイル名。省略時は自動生成の `.jsonl`（1サンプル1行）。拡張子を `.csv` にすると、`sf sysid fit` が読み込める「400Hz 1サイクル=1行」のマージ済みData Stream CSVとして保存される | 自動生成 `.jsonl` |
| `-d, --duration` | キャプチャ時間（秒） | 30 |
| `-i, --ip` | StampFlyのIPアドレス | 192.168.10.1 |
| `--port` | UDPテレメトリのポート番号 | 8890 |
| `--no-save` | ファイルに保存せず統計のみ表示 | - |

### 取得手順

1. StampFlyの電源を入れる（バッテリー駆動）
2. PCをStampFlyのWiFi AP（既定IP `192.168.10.1`）に接続する
3. `sf log wifi -d 30` を実行する

## 6. sf log convert

バイナリログ（`.bin`）をCSVに変換します。

```bash
sf log convert sensor.bin              # sensor.csv に変換
sf log convert sensor.bin -o out.csv   # 出力ファイル指定
```

### オプション

| 引数/オプション | 説明 | 既定値 |
|----------------|------|-------|
| `input`（位置引数） | 入力バイナリログファイル（`.bin`） | 必須 |
| `-o, --output` | 出力CSVファイル | 入力と同名で拡張子 `.csv` |

## 7. sf log info

ログファイルの情報（サンプル数・期間・サンプリングレート・列一覧など）を表示します。

```bash
sf log info                     # 最新ログの情報
sf log info sensor.bin          # バイナリログの情報
sf log info flight.csv          # CSVログの情報
```

### オプション

| 引数 | 説明 | 既定値 |
|------|------|-------|
| `file`（位置引数、省略可） | ログファイルパス | 最新のログ |

**対応形式:** `.bin` と `.csv` のみ（`.jsonl` は非対応。JSONLの内容確認は `sf log analyze --health` または `sf log viz` を使う）。

## 8. sf log analyze

フライトログを解析します。引数なしの通常解析と、`--health` によるモータ健全性診断の2系統があります。

### 通常解析（CSV）

```bash
sf log analyze                  # 最新CSVを解析
sf log analyze flight.csv       # 指定ファイルを解析
```

対応するのは `.csv` のみ（`.bin` は先に `sf log convert` でCSV化する）。解析結果は標準出力に表示され、グラフは常に `<入力ファイル名>_flight_analysis.png` として入力ファイルの隣に自動保存される（画面表示はしない）。

**出力内容:**

| 項目 | 内容 |
|------|------|
| ジャイロ統計 | 平均・標準偏差・範囲（deg/s） |
| コントローラ入力統計 | スティック入力の統計 |
| 振動周波数解析 | FFTによる支配的な振動周波数の検出（常時実行、フラグ不要） |
| 入力-応答相関 | スティック入力とジャイロ応答の相互相関・遅れ |
| 時間区間別安定性 | 5秒窓ごとの安定性 |
| PIDチューニング推奨事項 | ゲイン調整の目安 |

### モータ健全性診断（`--health`）

```bash
# 最新のJSONLログ1本で診断（回転方向グループを確定、隅は傾向のみ）
sf log analyze --health

# 同一機体の複数ログでクロスログ隅特定（CGオフセットを除去）
sf log analyze --health --batch

# セッション/機体をグロブで明示（CG一定の前提を守るため推奨）
sf log analyze --health --batch "stampfly_udp_2026061*.jsonl"

# AI/スクリプト連携用の機械可読JSON
sf log analyze --health --batch --json
```

対応するのは `.jsonl`（`sf log wifi` の既定出力）のみ。バックエンドは `tools/log_analyzer/motor_health.py`。

| オプション | 説明 |
|-----------|------|
| `--health` | ホバートリムから劣化ロータを検出するモータ健全性レポート（JSONL要） |
| `--batch` | `--health` と併用。`logs/` 内の直近12件のJSONL（または `file` にグロブを渡した場合はそれに一致する全件）を横断してCG除去の隅特定を行う |
| `--json` | `--health` と併用。機械可読なJSON判定結果を出力する |

**診断ロジックの要点:**
- ヨートリム `ur = (M1+M3) - (M2+M4)` はCG（機体重心）オフセットに依存せず、弱い回転方向グループ（CW/CCW）を確定できる。
- 隅（M1〜M4のどれか）の特定は、1本のホバーログだけではCGオフセットと交絡してしまうため、`--batch` で重症度の異なる複数ログを使い、`corr(ur, up)` / `corr(ur, uq)`（ロール・ピッチトリムとの相関）のスケーリングで分離する。
- `--batch` はCG一定（同一機体）を前提とするため、既定では最新12件に限定する。別機体のログが混在する環境では、対象セッションをグロブで明示する。
- 最終確認にはベンチでの入れ替え試験（疑わしい隅と対角のモータを入れ替えて再計測）を推奨。

## 9. sf log viz

テレメトリログを可視化します。ファイル形式を自動判定し、対応する描画処理へ振り分けます。

```bash
sf log viz                        # 最新ログ（.jsonl優先、なければ.csv）を可視化
sf log viz log.csv                # 指定ファイルを可視化

# 表示モード
sf log viz log.csv --mode sensors    # センサ生値のみ
sf log viz log.csv --mode attitude   # 姿勢のみ
sf log viz log.csv --mode position   # 位置・速度のみ
sf log viz log.csv --mode eskf       # ESKF推定値のみ

sf log viz log.csv --save output.png     # 画像として保存（ウィンドウ表示しない）
sf log viz log.csv --time-range 5 15     # 時間範囲を指定（秒）
sf log viz log.csv --no-eskf             # ESKFパネルを非表示
sf log viz log.csv --no-sensors          # 追加センサパネル（気圧・ToF・光学フロー）を非表示
sf log viz log.csv --show-invalid        # 無効なセンサ値も表示（既定は欠測として隠す）

sf log viz log.jsonl -i                  # インタラクティブ表示（Plotly、ブラウザで開く）
sf log viz log.jsonl -i --layout 3x2     # インタラクティブ表示のタイル配置
sf log viz log.jsonl -i --groups attitude bias_gyro  # 表示する信号グループを指定
```

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `--mode` | 表示モード（`all`/`sensors`/`attitude`/`position`/`eskf`） | `all` |
| `--save FILE` | 画面表示せずファイルへ保存 | - |
| `--time-range START END` | プロットする時間範囲（秒） | 全範囲 |
| `--no-eskf` | ESKFパネルを非表示 | - |
| `--no-sensors` | 追加センサパネル（気圧・ToF・光学フロー）を非表示 | - |
| `--show-invalid` | 無効なセンサ値も表示（既定は欠測区間として隠す） | - |
| `-i, --interactive` | インタラクティブモード（Plotly、ブラウザで開く） | - |
| `--layout RxC` | インタラクティブモードのタイル配置（例: `3x2`） | 自動 |
| `--groups` | インタラクティブモードで表示する信号グループ（例: `attitude bias_gyro`） | 全グループ |

### 自動フォーマット判定

| 形式 | 判定条件 | 対象コマンド例 |
|------|---------|---------------|
| Data Stream CSV | `timestamp_us`, `gyro_x/y/z`, `rate_ref_roll/pitch/yaw`, `total_thrust` を含む（`timestamp_us`+`quat_w`も持つが、Extended形式より先に判定される） | `sf log wifi -o *.csv` |
| Extended（ESKF付き400Hz） | `timestamp_us` + `quat_w` を含む | 旧`vehicle_old`のWebSocket拡張テレメトリ |
| FFT batch | `timestamp_ms` + `gyro_corrected_x` を含む | 旧FFTストリーミング形式（レガシー） |
| Normal WiFi telemetry | `timestamp_ms` + `roll_deg` を含む | 旧WebSocketテレメトリ |
| SILS trajectory | `t`, `px`, `alt`, `roll`, `yawrate`, `yawcmd`, `alt_est`, `m0`〜`m3` 等を含む | `sf sils scenario` の出力 |
| JSONL | 拡張子 `.jsonl` | `sf log wifi` の既定出力。デフォルトは静的な一覧表示、`-i` でインタラクティブ表示 |

### 表示ウィンドウが使えない環境での動作

matplotlibのGUIバックエンド（Tk/Qt等）が使えない環境（例: GUI無しの仮想環境）では、ウィンドウ表示の代わりに入力ファイルの隣へPNGを自動保存し、OS標準の画像ビューアで開く。ウィンドウを開けた場合は使用したバックエンド名（`macosx`/`tkagg`/`qtagg`等）を1行表示する。インタラクティブモード（`-i`、Plotly）はブラウザで開くためこのフォールバックの対象外。

## 10. 典型的なワークフロー

```bash
# 1. StampFlyの電源を入れ、WiFi APに接続
# 2. 400Hzテレメトリを取得
sf log wifi -d 60

# 3. 可視化
sf log viz

# 4. 詳細解析（振動周波数解析を含む）
sf log analyze
```

モータ健全性を確認したい場合は、複数回のホバーログを取得したうえで:

```bash
sf log wifi -d 30
sf log wifi -d 30
sf log analyze --health --batch
```

---

<a id="english"></a>

## 1. Overview

Capture, convert, analyze, and visualize telemetry logs. Supports both USB serial (binary) and WiFi (UDP) capture methods.

| Path | Subcommand | Format |
|------|-----------|--------|
| USB serial | `sf log capture` | Binary (`.bin`); convert to CSV with `sf log convert` |
| WiFi (UDP, 400Hz) | `sf log wifi` | Per-sample `.jsonl` by default; a `.csv` output produces a merged Data Stream CSV |

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List log files |
| `capture` | Capture binary log via USB serial |
| `wifi` | Capture 400Hz telemetry via WiFi UDP |
| `convert` | Convert binary log to CSV |
| `info` | Show log file information |
| `analyze` | Analyze flight log (`--health` for motor-health diagnosis) |
| `viz` | Visualize log data |

## 3. sf log list

Lists log files under `logs/` and `tools/log_analyzer/`, newest first.

```bash
sf log list              # Show the 20 most recent (default)
sf log list -n 50        # Show the 50 most recent
sf log list --all        # Show all files (ignore the limit)
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-n, --limit` | Number of recent files to show | 20 |
| `--all` | Show all files (ignore `--limit`) | - |

**Note:** Only `*.bin` and `*.csv` are scanned. `.jsonl` (the default `sf log wifi` output) does not appear in this listing — pass its path directly to `sf log info` or `sf log analyze --health` instead.

## 4. sf log capture

Captures a binary log (`.bin`) from StampFly over USB serial.

```bash
sf log capture                  # 60s capture (default), auto-detect port
sf log capture -d 120           # 120s capture
sf log capture -p /dev/ttyUSB0  # Specify port
sf log capture --live           # Show live packet data
sf log capture --no-auto        # Do not auto-send binlog on/off
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-p, --port` | Serial port | Auto-detect |
| `-o, --output` | Output filename | Auto-generated |
| `-d, --duration` | Capture duration (seconds) | 60 |
| `-b, --baudrate` | Baudrate | 115200 |
| `--live` | Show live packet data | - |
| `--no-auto` | Do not auto-send `binlog on`/`off` | - |
| `--debug` | Enable debug output | - |

## 5. sf log wifi

Captures full-rate (400Hz) telemetry over WiFi via **UDP** (not WebSocket). Just power on StampFly and connect to its WiFi AP — there is no USB `fftmode on` step to run beforehand (the current firmware has no such setting).

```bash
sf log wifi                              # 30s capture, saved as auto-generated .jsonl (default)
sf log wifi -d 60                        # 60s capture
sf log wifi -o flight.csv                # Save as a merged Data Stream CSV (400Hz, for sf sysid fit)
sf log wifi --no-save                    # Don't save, just show stats
sf log wifi -i 192.168.10.5 --port 8890  # Explicit IP/port
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-o, --output` | Output filename. If omitted, an auto-generated `.jsonl` (one row per sample). A `.csv` extension instead saves a merged Data Stream CSV (one row per 400Hz cycle) that `sf sysid fit` reads | Auto-generated `.jsonl` |
| `-d, --duration` | Capture duration (seconds) | 30 |
| `-i, --ip` | StampFly IP address | 192.168.10.1 |
| `--port` | UDP telemetry port | 8890 |
| `--no-save` | Don't save to file, just display stats | - |

### Capture procedure

1. Power on StampFly (on battery)
2. Connect the PC to StampFly's WiFi AP (default IP `192.168.10.1`)
3. Run `sf log wifi -d 30`

## 6. sf log convert

Converts a binary log (`.bin`) to CSV.

```bash
sf log convert sensor.bin              # Converts to sensor.csv
sf log convert sensor.bin -o out.csv   # Specify output file
```

### Options

| Argument/Option | Description | Default |
|-----------------|-------------|---------|
| `input` (positional) | Input binary log file (`.bin`) | Required |
| `-o, --output` | Output CSV file | Same name with `.csv` |

## 7. sf log info

Displays information about a log file (sample count, duration, sample rate, column list, etc.).

```bash
sf log info                     # Info for the latest log
sf log info sensor.bin          # Info for a binary log
sf log info flight.csv          # Info for a CSV log
```

### Options

| Argument | Description | Default |
|----------|-------------|---------|
| `file` (positional, optional) | Log file path | Latest log |

**Supported formats:** `.bin` and `.csv` only (`.jsonl` is not supported — use `sf log analyze --health` or `sf log viz` to inspect JSONL content).

## 8. sf log analyze

Analyzes a flight log. There are two paths: plain CSV analysis, and the `--health` motor-health diagnosis.

### Plain analysis (CSV)

```bash
sf log analyze                  # Analyze the latest CSV
sf log analyze flight.csv       # Analyze a specific file
```

Only `.csv` is supported (convert a `.bin` first with `sf log convert`). Results print to stdout, and a plot is always saved next to the input as `<input>_flight_analysis.png` (it is never shown on screen).

**Output:**

| Item | Content |
|------|---------|
| Gyro statistics | Mean, std, range (deg/s) |
| Controller input statistics | Stick input statistics |
| Oscillation frequency analysis | Dominant vibration frequencies via FFT (always run, no flag needed) |
| Input-response correlation | Cross-correlation and lag between stick input and gyro response |
| Time segment stability | Stability per 5-second window |
| PID tuning recommendations | Gain-adjustment guidance |

### Motor health diagnosis (`--health`)

```bash
# Diagnose from the single latest JSONL log (confirms spin-direction group; corner is only a trend)
sf log analyze --health

# Cross-log corner test across multiple logs of the same airframe (removes CG offset)
sf log analyze --health --batch

# Scope explicitly to a session/airframe via glob (recommended to keep the constant-CG assumption)
sf log analyze --health --batch "stampfly_udp_2026061*.jsonl"

# Machine-readable JSON for AI/script integration
sf log analyze --health --batch --json
```

Only `.jsonl` (the default `sf log wifi` output) is supported. Backend: `tools/log_analyzer/motor_health.py`.

| Option | Description |
|--------|-------------|
| `--health` | Motor health report: detect a degraded rotor from hover trim (requires JSONL) |
| `--batch` | With `--health`: cross-log corner test with CG removed, over the 12 most-recent JSONL logs in `logs/` (or every file matching a glob passed as `file`) |
| `--json` | With `--health`: emit a machine-readable JSON verdict |

**How the diagnosis works:**
- The yaw trim `ur = (M1+M3) - (M2+M4)` is independent of CG (center of gravity) offset, and pins down the weaker spin-direction group (CW/CCW).
- Identifying which corner (M1-M4) is weak is confounded with CG offset from a single hover log alone, so `--batch` uses several logs of differing severity and separates it via the scaling of `corr(ur, up)` / `corr(ur, uq)` (correlation with roll/pitch trim).
- `--batch` assumes one airframe with constant CG, so it defaults to the 12 most-recent logs; scope explicitly with a glob when other airframes' logs are mixed in.
- A bench swap test (suspected corner motor swapped with its diagonal) is recommended for final confirmation.

## 9. sf log viz

Visualizes a telemetry log. The file format is auto-detected and dispatched to the matching renderer.

```bash
sf log viz                        # Visualize the latest log (.jsonl preferred, else .csv)
sf log viz log.csv                # Visualize a specific file

# Display modes
sf log viz log.csv --mode sensors    # Raw sensor values only
sf log viz log.csv --mode attitude   # Attitude only
sf log viz log.csv --mode position   # Position/velocity only
sf log viz log.csv --mode eskf       # ESKF estimates only

sf log viz log.csv --save output.png     # Save to file (no window)
sf log viz log.csv --time-range 5 15     # Restrict to a time range (seconds)
sf log viz log.csv --no-eskf             # Hide ESKF panels
sf log viz log.csv --no-sensors          # Hide extra sensor panels (baro, ToF, optical flow)
sf log viz log.csv --show-invalid        # Show invalid sensor data (default: hidden as gaps)

sf log viz log.jsonl -i                  # Interactive (Plotly, opens in browser)
sf log viz log.jsonl -i --layout 3x2     # Tile layout for interactive mode
sf log viz log.jsonl -i --groups attitude bias_gyro  # Signal groups to show
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--mode` | Visualization mode (`all`/`sensors`/`attitude`/`position`/`eskf`) | `all` |
| `--save FILE` | Save to file instead of displaying | - |
| `--time-range START END` | Time range to plot (seconds) | Full range |
| `--no-eskf` | Hide ESKF panels | - |
| `--no-sensors` | Hide extra sensor panels (baro, ToF, optical flow) | - |
| `--show-invalid` | Show invalid sensor data (default: hidden as gaps) | - |
| `-i, --interactive` | Interactive mode (Plotly, opens in browser) | - |
| `--layout RxC` | Tile layout for interactive mode (e.g., `3x2`) | Automatic |
| `--groups` | Signal groups to show in interactive mode (e.g., `attitude bias_gyro`) | All groups |

### Automatic format detection

| Format | Detected by | Produced by |
|--------|------------|-------------|
| Data Stream CSV | `timestamp_us`, `gyro_x/y/z`, `rate_ref_roll/pitch/yaw`, `total_thrust` (also carries `timestamp_us`+`quat_w`, but is checked before the Extended format) | `sf log wifi -o *.csv` |
| Extended (400Hz with ESKF) | `timestamp_us` + `quat_w` | Legacy `vehicle_old` WebSocket extended telemetry |
| FFT batch | `timestamp_ms` + `gyro_corrected_x` | Legacy FFT-streaming format |
| Normal WiFi telemetry | `timestamp_ms` + `roll_deg` | Legacy WebSocket telemetry |
| SILS trajectory | `t`, `px`, `alt`, `roll`, `yawrate`, `yawcmd`, `alt_est`, `m0`-`m3`, etc. | `sf sils scenario` output |
| JSONL | `.jsonl` extension | Default `sf log wifi` output; static overview by default, interactive with `-i` |

### Behavior without a display window

When no matplotlib GUI backend (Tk/Qt, etc.) is usable (e.g. a headless environment), the plot is saved as a PNG next to the input file instead of opening a window, then opened with the OS's default image viewer. When a window does open, the backend name used (`macosx`/`tkagg`/`qtagg`, etc.) is printed on one line. Interactive mode (`-i`, Plotly) opens in a browser and is unaffected by this fallback.

## 10. Typical workflow

```bash
# 1. Power on StampFly and connect to its WiFi AP
# 2. Capture 400Hz telemetry
sf log wifi -d 60

# 3. Visualize
sf log viz

# 4. Detailed analysis (includes oscillation frequency analysis)
sf log analyze
```

To check motor health, capture a few hover logs first:

```bash
sf log wifi -d 30
sf log wifi -d 30
sf log analyze --health --batch
```
