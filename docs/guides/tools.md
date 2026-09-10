# Tools ガイド

StampFly開発支援ツールの使い方ガイドです。

> **重要:** ツールは全て **sf CLI** 経由で使用してください。`tools/` 配下の Python スクリプトは
> sf CLI のバックエンド実装であり、直接実行は非推奨です（3Dアニメーション等、sf コマンド化
> されていない一部のツールを除く）。

## 概要

```
tools/
├── log_capture/      # ログ取得（sf log capture のバックエンド）
├── log_analyzer/     # ログ解析・可視化（sf log analyze / sf log viz のバックエンド）
├── calibration/      # センサキャリブレーション（sf cal のバックエンド）
├── flashing/         # ファームウェア書き込み（sf flash のバックエンド）
└── ci/               # CI用スクリプト
```

## クイックスタート

```bash
# 開発環境のセットアップ
source setup_env.sh

# 環境診断（問題があればまずこれを実行）
sf doctor

# WiFi経由で400Hzテレメトリを取得（30秒間）
sf log wifi -d 30

# 最新ログを可視化
sf log viz
```

---

## ログ取得

### USB経由（sf log capture）

デバイスからバイナリセンサログをUSBシリアル経由でキャプチャします。

```bash
# キャプチャ（60秒、既定）
sf log capture

# ポート・時間・出力ファイルを指定
sf log capture -p /dev/tty.usbmodem* -o sensor.bin -d 30

# ライブ表示付き
sf log capture -d 30 --live

# CSVに変換
sf log convert sensor.bin

# ログ情報表示
sf log info sensor.bin
```

**主なオプション:**

| オプション | 説明 |
|-----------|------|
| `-p, --port` | シリアルポート（未指定時は自動検出） |
| `-o, --output` | 出力ファイル名（未指定時は自動生成） |
| `-d, --duration` | キャプチャ時間（秒）既定: 60 |
| `-b, --baudrate` | ボーレート 既定: 115200 |
| `--live` | パケットデータをライブ表示 |
| `--no-auto` | binlog on/off コマンドを自動送信しない |

### シリアルポート確認

```bash
# macOS
ls /dev/tty.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

### WiFi経由（sf log wifi）400Hzテレメトリ

StampFlyのWiFi APに接続して、USBより高レートの400Hzテレメトリをキャプチャします。

```bash
# 基本（30秒キャプチャ、既定IP 192.168.10.1）
sf log wifi

# 60秒キャプチャ + ファイル名指定
sf log wifi -d 60 -o flight_test.jsonl

# .csv 指定時は Data Stream CSV（sf sysid fit が読む形式）として保存
sf log wifi -d 30 -o flight_test.csv

# 統計のみ表示（保存なし）
sf log wifi --no-save

# IPアドレス・ポートを明示
sf log wifi -i 192.168.10.1 --port 8890
```

**含まれるデータ:** IMU生データ（ジャイロ・加速度）、バイアス補正済みジャイロ、ESKF推定値
（姿勢・位置・速度・バイアス）、センサデータ（気圧高度、ToF、光学フロー）、コントローラ入力。

---

## ログ可視化（sf log viz）

```bash
# 未指定時は最新ログを自動選択（.jsonl優先、なければ.csv）
sf log viz

# ファイル指定
sf log viz log.csv

# モード指定（既定 all）
sf log viz log.csv --mode sensors    # センサ生値のみ
sf log viz log.csv --mode attitude   # 姿勢のみ
sf log viz log.csv --mode position   # 位置・速度のみ
sf log viz log.csv --mode eskf       # ESKF推定値のみ

# 画像保存（GUIバックエンドが無い環境では自動でPNG保存にフォールバック）
sf log viz log.csv --save output.png

# 時間範囲指定
sf log viz log.csv --time-range 5 15

# パネルの表示切り替え
sf log viz log.csv --no-eskf      # ESKFパネル非表示
sf log viz log.csv --no-sensors   # 追加センサパネル（baro/tof/flow）非表示
sf log viz log.csv --show-invalid # 無効センサ値を隠さず表示

# インタラクティブ表示（Plotly、ブラウザで開く）
sf log viz log.csv -i
sf log viz log.csv -i --layout 3x2 --groups attitude bias_gyro
```

**自動フォーマット検出:**
- Data Stream CSV（`sf log wifi -o *.csv`、400Hz、`rate_ref`+50Hz CtrlRefマージ）
- Extended（400Hz ESKF+sensors）: `timestamp_us`, `quat_w` 含む
- FFT batch: `timestamp_ms`, `gyro_corrected_x` 含む
- Normal WiFi: `timestamp_ms`, `roll_deg` 含む
- SILS trajectory（`sf sils scenario` 出力）: `t`, `px`, `alt`, `roll`, `yawrate`, `yawcmd`, `alt_est`, `m0`-`m3` 含む
- JSONL（`sf log wifi` の既定出力）: 静的一覧表示。`-i` を付けるとインタラクティブ表示

---

## フライト解析（sf log analyze）

```bash
# フライト解析（振動周波数のFFT検出を含む。常時実行されフラグ不要）
sf log analyze

# モータ健全性レポート（劣化ロータの検出、JSONLログが必要）
sf log analyze --health

# 複数ログでクロスログ隅特定（CG除去）
sf log analyze --health --batch

# セッション/機体をグロブで明示
sf log analyze --health --batch "stampfly_udp_2026061*.jsonl"

# AI/スクリプト連携用の機械可読 JSON
sf log analyze --health --batch --json
```

詳細は `tools/log_analyzer/README.md` の「sf log analyze --health」節を参照してください。

---

## キャリブレーション確認（sf cal）

### 磁気キャリブレーション確認（sf cal plot）

地磁気キャリブレーションを確認します（バックエンド: `plot_mag_xy.py`）。

```bash
# 最新のログから確認
sf cal plot

# ファイル指定 + 画像保存
sf cal plot sensor.bin -o mag_xy.png
```

**判定:**
- 正常: 原点中心の円
- 要調整: オフセットまたは楕円

その他のキャリブレーション（ジャイロ・加速度）は `sf cal gyro` / `sf cal accel`、一覧は
`sf cal list` を使用してください。詳細は `tools/calibration/README.md` を参照。

---

## ビルド・書き込み

```bash
# ビルド（既定 target: vehicle）
sf build vehicle
sf build controller

# クリーンビルド
sf build vehicle -c

# 書き込み（-m でモニタ付き）
sf flash vehicle -m

# ビルドしてから書き込み
sf flash vehicle --build -m
```

---

## 3Dアニメーション（sf非対応、直接実行が必要）

姿勢・位置の3Dアニメーション表示は sf CLI に未統合のため、`tools/log_analyzer/` 配下の
スクリプトを直接実行します。

```bash
cd tools/log_analyzer

# 姿勢3Dアニメーション（CSVのみ対応）
python3 visualize_attitude_3d.py data.csv

# 位置+姿勢3Dアニメーション（.bin/.csv対応）
python3 visualize_pose_3d.py data.bin

# MP4動画として保存
python3 visualize_pose_3d.py data.bin --mp4
```

---

## 典型的なワークフロー

### 1. フライトログ取得・解析サイクル

```bash
# 1. StampFly WiFi APに接続してログ取得
sf log wifi -d 60

# 2. 可視化
sf log viz

# 3. 詳細解析（FFTによる振動周波数検出を含む）
sf log analyze

# 4. ファームウェア修正後の再ビルド・書き込み
sf build vehicle
sf flash vehicle -m
```

### 2. キャリブレーション確認

```bash
# 1. 静止状態でUSBログ取得
sf log capture -d 30 -o static.bin

# 2. 地磁気確認
sf cal plot static.bin
```

---

## トラブルシューティング

### シリアルポートが見つからない

```bash
# デバイスを接続してから
ls /dev/tty.usbmodem*        # macOS
ls /dev/ttyUSB* /dev/ttyACM* # Linux

# 環境診断
sf doctor
```

### グラフが表示されない

```bash
# 環境診断（matplotlib GUIバックエンドの自動修復を含む）
sf doctor --fix

# 画像保存で確認（GUIバックエンドが無い場合は自動でPNGにフォールバックする）
sf log viz log.csv --save test.png
```

### ログファイルが見つからない

```bash
# 全ログファイル一覧
sf log list --all

# ログ情報確認（フォーマット検出）
sf log info log.csv
```

---

## 関連ドキュメント

- [次のステップ](../next_step.md) - 操縦と開発の詳細
- `tools/log_analyzer/README.md` - ログ解析・可視化の詳細なツールリファレンス
- `tools/log_capture/README.md` - ログ取得の詳細
- `tools/calibration/README.md` - キャリブレーションの詳細
