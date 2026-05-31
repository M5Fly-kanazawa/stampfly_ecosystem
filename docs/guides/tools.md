# Tools ガイド

StampFly開発支援ツールの使い方ガイドです。

## 概要

```
tools/
├── log_capture/      # ログ取得
├── log_analyzer/     # ログ解析・可視化・最適化
├── calibration/      # センサキャリブレーション
├── flashing/         # ファームウェア書き込み
└── ci/               # CI用スクリプト
```

## クイックスタート

### 1. 環境セットアップ

```bash
cd tools/log_analyzer
pip install -r requirements.txt
```

### 2. ログ取得

```bash
cd tools/log_capture
python log_capture.py capture -p /dev/tty.usbmodem* -o flight.bin -d 60
```

### 3. ログ可視化

```bash
cd tools/log_analyzer
python visualize_eskf.py ../log_capture/flight.bin
```

---

## log_capture - ログ取得

デバイスからセンサログをキャプチャします。

### 基本コマンド

```bash
# キャプチャ（60秒）
python log_capture.py capture -p /dev/tty.usbmodem* -o sensor.bin -d 60

# ライブ表示付き
python log_capture.py capture -p /dev/tty.usbmodem* -o sensor.bin -d 30 --live

# CSV変換
python log_capture.py convert --input sensor.bin --output sensor.csv

# ログ情報表示
python log_capture.py info sensor.bin
```

### シリアルポート確認

```bash
# macOS
ls /dev/tty.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

---

## log_analyzer - ログ解析

### 可視化ツール

#### visualize_eskf.py（メイン）

```bash
# 全パネル表示
python visualize_eskf.py data.bin

# センサ生値と姿勢のみ
python visualize_eskf.py data.bin --sensors --attitude

# 画像に保存
python visualize_eskf.py data.bin --all --save result.png --no-show
```

**オプション:**

| オプション | 説明 |
|-----------|------|
| `--all` | 全パネル表示（デフォルト） |
| `--sensors` | センサ生値 |
| `--attitude` | 姿勢（Roll/Pitch/Yaw） |
| `--position` | 位置・速度 |
| `--biases` | バイアス推定値 |
| `--trajectory` | XY軌跡 |
| `--compare` | PC vs Device比較 |
| `--save FILE` | 画像保存 |

#### ラッパースクリプト

```bash
python viz_all.py data.bin        # 全パネル
python viz_sensors.py data.bin    # センサ生値
python viz_attitude.py data.bin   # 姿勢
python viz_position.py data.bin   # 位置・速度
python viz_compare.py result.csv  # PC vs Device比較
```

#### 3Dアニメーション

```bash
# 姿勢3D
python visualize_attitude_3d.py data.bin

# 位置+姿勢3D
python visualize_pose_3d.py data.bin

# MP4動画に保存
python visualize_pose_3d.py data.bin --mp4
```

### 解析ツール

#### pure_accel_integration.py

加速度をESKFなしで純積分し、ドリフト特性を確認します。

```bash
python pure_accel_integration.py data.bin
```

---

## calibration - キャリブレーション

### plot_mag_xy.py

地磁気キャリブレーションを確認します。

```bash
python plot_mag_xy.py sensor.bin
```

**判定:**
- 正常: 原点中心の円
- 要調整: オフセットまたは楕円

---

## 典型的なワークフロー

### ESKFログ解析サイクル

```bash
# 1. ログ取得
cd tools/log_capture
python log_capture.py capture -p /dev/tty.usbmodem* -o test.bin -d 30

# 2. 現状確認
cd ../log_analyzer
python visualize_eskf.py ../log_capture/test.bin --attitude --position

# 3. ファームウェア再ビルド・テスト
cd ../../firmware/vehicle
idf.py build flash monitor
```

### キャリブレーション確認

```bash
# 1. 静止状態でデータ取得
cd tools/log_capture
python log_capture.py capture -p /dev/tty.usbmodem* -o static.bin -d 30

# 2. 地磁気確認
cd ../calibration
python plot_mag_xy.py ../log_capture/static.bin
```

---

## トラブルシューティング

### シリアルポートが見つからない

```bash
# デバイスを接続してから
ls /dev/tty.usbmodem*  # macOS
ls /dev/ttyUSB* /dev/ttyACM*  # Linux
```

### グラフが表示されない

```bash
# matplotlibバックエンド確認
python -c "import matplotlib; print(matplotlib.get_backend())"

# 画像に保存で確認
python visualize_eskf.py data.bin --save test.png --no-show
```

### 最適化が収束しない

- イテレーション数を増やす: `--iter 1000`
- 複数データセットで実行
- `--roll-weight` を調整

---

## 関連ドキュメント

- [Getting Started](getting-started.md) - 初期セットアップ
- `tools/log_analyzer/README.md` - 詳細なツールリファレンス
- `tools/log_capture/README.md` - ログ取得の詳細
- `tools/calibration/README.md` - キャリブレーションの詳細
