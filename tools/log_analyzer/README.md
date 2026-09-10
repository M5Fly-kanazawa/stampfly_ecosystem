# StampFly ログ解析ツール

StampFlyデバイスのESKF（Error-State Kalman Filter）開発・検証・最適化のためのPythonツール群です。

> **重要:** ツールは全て **sf CLI** 経由で使用することを推奨します。

## クイックスタート

```bash
# 開発環境のセットアップ
source setup_env.sh

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
| `sf log wifi` | WiFi経由400Hzキャプチャ | `sf log wifi -d 60 -o flight.csv` |
| `sf log capture` | USB経由バイナリキャプチャ | `sf log capture -d 30` |
| `sf log convert` | バイナリ→CSV変換 | `sf log convert log.bin` |
| `sf log info` | ログ情報表示 | `sf log info log.csv` |
| `sf log analyze` | フライト解析 | `sf log analyze log.csv` |
| `sf log analyze --health` | モータ故障診断 | `sf log analyze --health --batch` |
| `sf log viz` | ログ可視化 | `sf log viz log.csv --mode all` |

## sf log analyze --health - モータ健全性レポート

ホバー飛行ログから、劣化したロータ（同じ duty で推力・反トルクが低下したモータ）を
定常ホバートリムで検出する。バックエンドは `motor_health.py`。

```bash
# 最新の JSONL ログ1本で診断（回転方向グループを確定、隅は傾向）
sf log analyze --health

# 同一機体の複数ログでクロスログ隅特定（CG除去）。既定は最新ログ群
sf log analyze --health --batch

# セッション/機体をグロブで明示（CG一定の前提を守るため推奨）
sf log analyze --health --batch "stampfly_udp_2026061*.jsonl"

# AI/スクリプト連携用の機械可読 JSON
sf log analyze --health --batch --json
```

- ヨートリム `ur=(M1+M3)-(M2+M4)` は CG非依存で、弱い回転方向グループ（CW/CCW）を確定する。
- 隅（M1〜M4）は単一ホバーだと CG オフセットと交絡するため、`--batch` で重症度の異なる
  複数ログを使い、`corr(ur, up)` / `corr(ur, uq)` のスケーリングで分離する。
- 確定にはベンチでの入れ替え試験（疑い隅 ↔ 対角）を推奨。

## sf log wifi - 400Hzテレメトリキャプチャ

StampFlyのWiFi APに接続して400Hzテレメトリをキャプチャします。

```bash
# 基本（30秒キャプチャ）
sf log wifi

# 60秒キャプチャ（振動解析したい場合は取得後に sf log analyze を使う）
sf log wifi -d 60 -o flight.csv

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
- SILS trajectory（`sf sils scenario` 出力）: `t`, `px`, `alt`, `roll`, `yawrate`, `yawcmd`, `alt_est`, `m0`-`m3` 含む

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

FFT解析（支配的な振動周波数の検出）は `sf log analyze` の解析処理に常時組み込まれており、
別途フラグを指定する必要はない。

```bash
# キャプチャ
sf log wifi -d 30

# 解析（FFTによる振動周波数の検出を含む）
sf log analyze
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
| `visualize_stream.py` | `sf log viz` | Data Stream CSV可視化（`sf log wifi -o *.csv`、400Hz） |
| `visualize_extended.py` | `sf log viz` | 拡張テレメトリ可視化（400Hz ESKF付き / FFT batch形式） |
| `visualize_telemetry.py` | `sf log viz` | WiFi CSV可視化（通常テレメトリ形式） |
| `visualize_jsonl.py` | `sf log viz` | JSONLログの静的一覧表示（`.jsonl` のデフォルト） |
| `visualize_interactive.py` | `sf log viz -i` | インタラクティブ表示（Plotly、CSV/JSONL共通） |
| `visualize_sils_trajectory.py` | `sf log viz` | SILS trajectory.csv可視化 |
| `visualize_attitude_3d.py` | - | 姿勢3Dアニメーション |
| `visualize_pose_3d.py` | - | 位置+姿勢3Dアニメーション |

### 解析・最適化

| スクリプト | sf コマンド | 説明 |
|-----------|------------|------|
| `flight_analysis.py` | `sf log analyze` | フライト解析（FFTによる振動周波数検出を含む） |

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
