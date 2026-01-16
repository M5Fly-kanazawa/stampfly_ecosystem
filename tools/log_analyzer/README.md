# StampFly ログ解析ツール

StampFlyデバイスのESKF（Error-State Kalman Filter）開発・検証・最適化のためのPythonツール群です。

> **重要:** ツールは全て **sf CLI** 経由で使用することを推奨します。

## クイックスタート

```bash
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

# ログファイル一覧
sf log list

# WiFi経由で400Hzテレメトリをキャプチャ
sf log wifi -d 30

# 最新ログを可視化
sf log viz

# キャプチャ → 可視化の一連の流れ
sf log wifi -d 30 && sf log viz
```

## sf log コマンド一覧

| コマンド | 説明 | 例 |
|---------|------|-----|
| `sf log list` | ログファイル一覧 | `sf log list --all` |
| `sf log wifi` | WiFi経由400Hzキャプチャ | `sf log wifi -d 60 --fft` |
| `sf log capture` | USB経由バイナリキャプチャ | `sf log capture -d 30` |
| `sf log convert` | バイナリ→CSV変換 | `sf log convert log.bin` |
| `sf log info` | ログ情報表示 | `sf log info log.csv` |
| `sf log analyze` | フライト解析 | `sf log analyze log.csv` |
| `sf log viz` | ログ可視化 | `sf log viz log.csv --mode all` |

## sf log wifi - 400Hzテレメトリキャプチャ

StampFlyのWiFi APに接続して400Hzテレメトリをキャプチャします。

```bash
# 基本（30秒キャプチャ）
sf log wifi

# 60秒キャプチャ + FFT解析
sf log wifi -d 60 --fft

# ファイル名指定
sf log wifi -d 30 -o flight_test.csv

# 統計のみ表示（保存なし）
sf log wifi --no-save
```

**含まれるデータ:**
- IMU生データ（ジャイロ、加速度）
- バイアス補正済みジャイロ
- ESKF推定値（姿勢、位置、速度、バイアス）
- センサデータ（気圧高度、ToF、光学フロー）
- コントローラ入力

## sf log viz - ログ可視化

```bash
# 全パネル表示（デフォルト）
sf log viz log.csv

# モード指定
sf log viz log.csv --mode sensors    # センサ生値のみ
sf log viz log.csv --mode attitude   # 姿勢のみ
sf log viz log.csv --mode position   # 位置・速度のみ
sf log viz log.csv --mode eskf       # ESKF推定値のみ

# 画像保存
sf log viz log.csv --save output.png

# 時間範囲指定
sf log viz log.csv --time-range 5 15

# ESKFパネル非表示
sf log viz log.csv --no-eskf
```

**自動フォーマット検出:**
- Extended (400Hz ESKF+sensors): `timestamp_us`, `quat_w` 含む
- FFT batch: `timestamp_ms`, `gyro_corrected_x` 含む
- Normal WiFi: `timestamp_ms`, `roll_deg` 含む

## 典型的なワークフロー

### 1. フライトログ取得と解析

```bash
# 1. StampFly WiFi APに接続
# 2. ログキャプチャ
sf log wifi -d 60

# 3. 可視化
sf log viz

# 4. 詳細解析
sf log analyze
```

### 2. 振動解析（FFT）

```bash
# キャプチャ + FFT解析
sf log wifi -d 30 --fft

# または別途FFT解析
sf log analyze --fft
```

### 3. Q/Rパラメータ最適化

```bash
# 1. テストデータ取得
sf log capture -d 30

# 2. 最適化実行（直接スクリプト使用）
cd tools/log_analyzer
python optimize_eskf.py ../logs/stampfly_*.bin --method sa --iter 500

# 3. 結果をファームウェアに適用
python optimize_eskf.py ../logs/stampfly_*.bin --apply
```

## バックエンドスクリプト

> **注:** これらのスクリプトは sf CLI のバックエンド実装です。直接実行せず、sf CLI を使用してください。

### キャプチャ・変換

| スクリプト | sf コマンド | 説明 |
|-----------|------------|------|
| `wifi_capture.py` | `sf log wifi` | WiFi 400Hzキャプチャ |
| `log_capture.py` | `sf log capture` | USB バイナリキャプチャ |

### 可視化

| スクリプト | sf コマンド | 説明 |
|-----------|------------|------|
| `visualize_extended.py` | `sf log viz` | 拡張テレメトリ可視化 |
| `visualize_telemetry.py` | `sf log viz` | WiFi CSV可視化 |
| `visualize_eskf.py` | `sf log viz` | バイナリログ可視化 |
| `visualize_attitude_3d.py` | - | 姿勢3Dアニメーション |
| `visualize_pose_3d.py` | - | 位置+姿勢3Dアニメーション |

### 解析・最適化

| スクリプト | sf コマンド | 説明 |
|-----------|------------|------|
| `flight_analysis.py` | `sf log analyze` | フライト解析 |
| `optimize_eskf.py` | - | Q/Rパラメータ最適化 |
| `estimate_qr.py` | - | 静止データからQ/R推定 |
| `analyze_fft.py` | `sf log analyze --fft` | FFT解析 |

## 必要なライブラリ

```bash
pip install numpy pandas matplotlib scipy websockets
```

## トラブルシューティング

### WiFi接続できない

```bash
# 環境診断
sf doctor

# WiFi接続確認
ping 192.168.4.1
```

### ログファイルが見つからない

```bash
# 全ログファイル一覧
sf log list --all

# 検索ディレクトリ
#   - logs/
#   - tools/log_analyzer/
```

### 可視化でエラー

```bash
# ログ情報確認（フォーマット検出）
sf log info log.csv

# 必要なライブラリ確認
pip install matplotlib numpy pandas
```
