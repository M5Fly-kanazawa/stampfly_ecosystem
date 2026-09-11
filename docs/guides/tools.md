# Tools ガイド

StampFly開発支援ツールの使い方ガイドです。

> **重要:** ツールは全て **sf CLI** 経由で使用してください。`tools/` 配下の Python スクリプトは
> sf CLI のバックエンド実装であり、直接実行は非推奨です（3Dアニメーション等、sf コマンド化
> されていない一部のツールを除く）。

## 概要

```
tools/
├── log_analyzer/     # ログ取得・解析・可視化（sf log のバックエンド）
├── sysid/            # システム同定（sf sysid のバックエンド）
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

## ログ取得（sf log wifi）400Hzテレメトリ

StampFlyのWiFi APに接続してWiFi UDPで400Hzテレメトリをキャプチャし、**StampFlyフライトログ
v1一式**（`.sflog.zip`。信号ごとのCSVと`meta.json`/`schema.json`をzipにまとめたもの。以下
「一式」）として保存します。USBシリアル経由の取得（旧`sf log capture`）は`vehicle_old`専用
機能として廃止済みです。

基本（30秒キャプチャ、既定IP 192.168.10.1）:

```bash
sf log wifi
```

出力先を明示指定する（`.sflog.zip`で終わるパスはそのzipに書く）:

```bash
sf log wifi -d 30 -o flight_test.sflog.zip
```

統計のみ表示（保存なし）:

```bash
sf log wifi --no-save
```

IPアドレス・ポートを明示する:

```bash
sf log wifi -i 192.168.10.1 --port 8890
```

保存直後に `sf log check` が自動実行され、電文の解析エラーや構造の異常があればその場で報告
されます。

**含まれるデータ:** IMU生データ（ジャイロ・加速度）、バイアス補正済みジャイロ、ESKF推定値
（姿勢・位置・速度・バイアス）、センサデータ（気圧高度、ToF、光学フロー）、コントローラ入力。
一次記録（測定値をそのまま書いた、最初の記録）は原レートのまま埋め値なしで保存され、1枚に
整列した表やJSONL（JSON Lines: 1行に1件のJSONレコードを並べたテキスト形式）が欲しい場合は
`sf log convert --aligned`/`--jsonl` で派生物として作る。

---

## ログ可視化（sf log viz）

未指定時は `logs/` 内の最新の一式を自動選択する:

```bash
sf log viz
```

モード指定（既定 `all`。センサ生値のみ・姿勢のみ・位置速度のみ・ESKF推定値のみ）:

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode sensors
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode attitude
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode position
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode eskf
```

画像保存（GUIバックエンドが無い環境では自動でPNG保存にフォールバック）:

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --save output.png
```

時間範囲指定:

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --time-range 5 15
```

インタラクティブ表示（Plotly、ブラウザで開く）:

```bash
sf log viz logs/flight_20260911T121243.sflog.zip -i
```

全ての信号をそれぞれの原レート（400Hz姿勢/IMU/モータ、50Hz操縦入力、1Hzバッテリ等）のまま
1つの描画処理で表示する。パネルの一覧・モードごとの内訳は `docs/guides/flight-log-viz.md`
参照。

---

## フライト解析（sf log analyze）

フライト解析（振動周波数のFFT検出を含む。常時実行されフラグ不要）:

```bash
sf log analyze
```

モータ健全性レポート（劣化ロータの検出）:

```bash
sf log analyze --health
```

複数の一式でクロスログ隅特定（CG除去）:

```bash
sf log analyze --health --batch
```

セッション/機体をグロブで明示する:

```bash
sf log analyze --health --batch "logs/flight_202609*.sflog.zip"
```

AI/スクリプト連携用の機械可読 JSON:

```bash
sf log analyze --health --batch --json
```

詳細は `tools/log_analyzer/README.md` の「sf log analyze --health」節を参照してください。

---

## キャリブレーション確認（sf cal）

### 磁気キャリブレーション確認（sf cal plot）

地磁気キャリブレーションを確認します（バックエンド: `plot_mag_xy.py`）。入力は
`sf log wifi` が保存するフライトログ一式（`.sflog.zip`）の `mag.csv` で、引数を
省略すると `logs/` 内の最新の一式を使います。

```bash
sf cal plot
```

```bash
sf cal plot logs/flight_20260908T121243.sflog.zip -o mag_xy.png
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
スクリプトを直接実行します。どちらも入力は一式（`.sflog.zip` または展開済みフォルダ）です。

姿勢3Dアニメーション（3D姿勢軸 + オイラー角グラフ）:

```bash
cd tools/log_analyzer
python3 visualize_attitude_3d.py flight_20260911T121243.sflog.zip
```

位置+姿勢3Dアニメーション（3D軌跡/姿勢 + 上面図）:

```bash
cd tools/log_analyzer
python3 visualize_pose_3d.py flight_20260911T121243.sflog.zip
```

MP4/GIF動画として保存する（拡張子で自動判別）:

```bash
cd tools/log_analyzer
python3 visualize_pose_3d.py flight_20260911T121243.sflog.zip --save flight.mp4
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
# 1. 静止状態でWiFiログ取得
sf log wifi -d 30

# 2. 地磁気確認（既定で logs/ 内の最新の一式を使う）
sf cal plot
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
sf log viz --save test.png
```

### ログファイル（一式）が見つからない

```bash
# logs/ 内の一式一覧
sf log list --all

# 一式の情報確認
sf log info
```

---

## 関連ドキュメント

- [次のステップ](../next_step.md) - 操縦と開発の詳細
- `docs/commands/sf-log.md` - `sf log` サブコマンドの詳細リファレンス
- `docs/guides/flight-log-viz.md` - フライトログの取得と可視化チュートリアル
- `tools/log_analyzer/README.md` - ログ解析・可視化の詳細なツールリファレンス
- `tools/sysid/README.md` - システム同定ツールの詳細
- `tools/calibration/README.md` - キャリブレーションの詳細
