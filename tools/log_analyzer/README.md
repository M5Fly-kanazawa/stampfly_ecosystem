# StampFly ESKF ツール群

StampFlyデバイスのESKF（Error-State Kalman Filter）開発・検証・最適化のためのPythonツール群です。

> **関連ディレクトリ**:
> - `tools/log_capture/` - ログ取得ツール
> - `tools/calibration/` - キャリブレーションツール
> - `firmware/vehicle/logs/` - テストデータ

## ファイル構成

```
tools/log_analyzer/
├── 400Hz WiFiテレメトリツール（★推奨）
│   ├── wifi_capture.py           # WiFi経由400Hzログキャプチャ
│   └── visualize_extended.py     # 拡張テレメトリ包括可視化
├── 可視化ツール
│   ├── visualize_eskf.py         # バイナリログ統合可視化（USB経由）
│   ├── visualize_telemetry.py    # WiFiテレメトリCSV可視化
│   ├── viz_telemetry.py          # WiFi CSV用ラッパー
│   ├── viz_all.py                # 全パネル表示（ラッパー）
│   ├── viz_sensors.py            # センサ生値のみ（ラッパー）
│   ├── viz_attitude.py           # 姿勢のみ（ラッパー）
│   ├── viz_position.py           # 位置・速度のみ（ラッパー）
│   ├── viz_compare.py            # PC vs Device比較（ラッパー）
│   ├── visualize_attitude_3d.py  # 姿勢3Dアニメーション
│   ├── visualize_pose_3d.py      # 位置+姿勢3Dアニメーション
│   └── visualize_optimization.py # 最適化過程の可視化
├── 最適化ツール
│   └── optimize_eskf.py          # Q/Rパラメータ最適化（SA/GD）
├── パラメータ推定
│   └── estimate_qr.py            # 静止データからQ/R推定
└── 解析ツール
    └── pure_accel_integration.py # 加速度純積分解析
```

## 必要なライブラリ

```bash
pip install numpy pandas matplotlib pyserial scipy websockets
```

---

## 0. 400Hz WiFiテレメトリツール（★推奨）

### wifi_capture.py - WiFi経由400Hzログキャプチャ

StampFlyのWiFi APに接続し、400Hzの拡張テレメトリをキャプチャしてCSVに保存します。
ESKF推定値（姿勢・位置・速度・バイアス）とセンサ生データ（IMU・ToF・気圧・光学フロー）を含みます。

**ワークフロー:**
1. StampFlyの電源を入れる（400Hzテレメトリが自動で有効）
2. PCをStampFly WiFi APに接続
3. `wifi_capture.py`を実行

```bash
# 基本的なキャプチャ（30秒、自動ファイル名）
python wifi_capture.py

# 60秒キャプチャ + FFT解析
python wifi_capture.py -d 60 --fft

# 指定ファイルに保存
python wifi_capture.py -d 30 -o flight_test.csv

# 統計のみ表示（ファイル保存なし）
python wifi_capture.py --no-save
```

**オプション:**

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-d, --duration` | キャプチャ時間（秒） | 30 |
| `-o, --output` | 出力CSVファイル名 | 自動生成 |
| `-i, --ip` | StampFly IPアドレス | 192.168.4.1 |
| `-p, --port` | WebSocketポート | 80 |
| `--fft` | キャプチャ後にFFT解析 | - |
| `--no-save` | ファイル保存しない | - |

**出力統計例:**
```
=== Capture Statistics ===
Mode: Extended (552B, 4 samples/frame with ESKF+sensors)
Samples: 12000
Frames: 3000
Duration: 30.00s
Sample rate: 400.0 Hz
Frame rate: 100.0 Hz

ESKF Gyro Bias [rad/s]:
  X: final=+0.001234, mean=+0.001200
  Y: final=-0.000567, mean=-0.000550
  Z: final=+0.000089, mean=+0.000085
```

### visualize_extended.py - 拡張テレメトリ可視化

`wifi_capture.py`で取得したCSVを包括的に可視化します。IMU、ESKF推定値、センサデータを一覧表示。

```bash
# 全状態を可視化
python visualize_extended.py stampfly_fft_20260115T120000.csv

# 画像に保存
python visualize_extended.py flight.csv --save flight_analysis.png

# 時間範囲を指定（5秒〜15秒）
python visualize_extended.py flight.csv --time-range 5 15

# ESKFパネルを非表示
python visualize_extended.py flight.csv --no-eskf

# センサパネルを非表示
python visualize_extended.py flight.csv --no-sensors
```

**出力パネル（全表示時）:**

| セクション | 内容 |
|-----------|------|
| IMU | 生ジャイロ、生加速度、バイアス補正済みジャイロ |
| 制御 | コントローラ入力（スロットル、ロール、ピッチ、ヨー） |
| ESKF | 姿勢（Euler）、位置、速度、ジャイロバイアス、加速度バイアス |
| センサ | 気圧高度、ToF距離、光学フロー |

**拡張CSVカラム（36列）:**
```
timestamp_us, gyro_x/y/z, accel_x/y/z,
gyro_corrected_x/y/z, ctrl_throttle/roll/pitch/yaw,
quat_w/x/y/z, pos_x/y/z, vel_x/y/z,
gyro_bias_x/y/z, accel_bias_x/y/z,
baro_altitude, tof_bottom, tof_front,
flow_x, flow_y, flow_quality
```

---

## 1. visualize_telemetry.py - WiFiテレメトリ可視化ツール

WiFiテレメトリで取得したCSVログを包括的に可視化するツールです。
CSVファイルは `firmware/vehicle/logs/` に保存されています。

### 基本的な使い方

```bash
# 全状態量を可視化（ファイル指定）
python visualize_telemetry.py stampfly_log_20260113T070745.csv

# 引数なしで実行すると最新CSVを自動検出
python visualize_telemetry.py

# 特定パネルのみ表示
python visualize_telemetry.py log.csv --attitude      # 姿勢のみ
python visualize_telemetry.py log.csv --position      # 位置・速度
python visualize_telemetry.py log.csv --sensors       # センサ生値
python visualize_telemetry.py log.csv --control       # 制御入力
python visualize_telemetry.py log.csv --tof           # ToFセンサ

# 画像保存
python visualize_telemetry.py log.csv --save output.png --no-show
```

### 出力パネル（--all時）

5x4グリッドで以下を表示：

| 行 | 内容 |
|----|------|
| 1行目 | Roll、Pitch、Yaw、FlightState |
| 2行目 | 位置X、位置Y、位置Z、XY軌跡 |
| 3行目 | 速度X、速度Y、速度Z、バッテリー電圧 |
| 4行目 | 加速度、ジャイロ、地磁気、ToF Bottom |
| 5行目 | スロットル、制御入力、ToF Front |

### CSVカラム（WiFiテレメトリ）

```
timestamp_ms, roll_deg, pitch_deg, yaw_deg,
pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z,
mag_x, mag_y, mag_z,
throttle, ctrl_roll, ctrl_pitch, ctrl_yaw,
voltage, tof_bottom, tof_front, state
```

---

## 2. visualize_eskf.py - バイナリログ可視化ツール

ESKFの状態量とセンサ生値を包括的に可視化するツールです（USB経由のバイナリログ用）。

### 基本的な使い方

```bash
# 全パネル表示（デフォルト）
python visualize_eskf.py data.bin

# CSV（eskf_replayの出力）も対応
python visualize_eskf.py result.csv
```

### オプション

| オプション | 説明 |
|-----------|------|
| `--all` | 全パネル表示（デフォルト） |
| `--sensors` | センサ生値（加速度、ジャイロ、地磁気、気圧、ToF、フロー） |
| `--attitude` | 姿勢（Roll、Pitch、Yaw） |
| `--position` | 位置・速度（X、Y、Z） |
| `--biases` | バイアス推定値（ジャイロ、加速度） |
| `--trajectory` | XY軌跡 |
| `--compare` | PC ESKF vs Device ESKF 比較表示 |
| `--pc` | PC シミュレーション結果のみ表示（Device非表示） |
| `--save FILE` | 画像ファイルに保存 |
| `--no-show` | ウィンドウを表示しない（--saveと併用） |

### 使用例

```bash
# センサ生値と姿勢のみ表示
python visualize_eskf.py flow01.bin --sensors --attitude

# PC vs Device比較
python visualize_eskf.py result.csv --compare --all

# PCシミュレーション結果のみ表示（Device非表示）
python visualize_eskf.py result.csv --pc --attitude --position

# 画像に保存（表示なし）
python visualize_eskf.py flow01.bin --all --save result.png --no-show

# 複数オプションを組み合わせ
python visualize_eskf.py flow01.bin --attitude --position --trajectory
```

### 出力パネル（--all時）

4x4グリッドで以下を表示：

| 行 | 内容 |
|----|------|
| 1行目 | 加速度、ジャイロ、地磁気、ToF |
| 2行目 | フロー、Roll、Pitch、Yaw |
| 3行目 | 位置X、位置Y、位置Z、XY軌跡 |
| 4行目 | 速度X、速度Y、速度Z、ジャイロバイアス |

---

## 2. viz_*.py - ラッパースクリプト

`visualize_eskf.py`を簡単に使うためのラッパースクリプトです。

```bash
# 全パネル表示
python viz_all.py data.bin

# センサ生値のみ
python viz_sensors.py data.bin

# 姿勢のみ
python viz_attitude.py data.bin

# 位置・速度のみ
python viz_position.py data.bin

# PC vs Device比較
python viz_compare.py result.csv
```

---

## 3. visualize_attitude_3d.py - 姿勢3Dアニメーション

姿勢をリアルタイム3Dアニメーションで表示します。

```bash
python visualize_attitude_3d.py data.bin
python visualize_attitude_3d.py result.csv
```

NED座標系で機体の姿勢をアニメーション表示。デバッグやデモに便利です。

---

## 4. visualize_pose_3d.py - 位置+姿勢3Dアニメーション

位置と姿勢を同時に3Dアニメーションで表示します。

```bash
# 表示
python visualize_pose_3d.py data.bin

# MP4動画に保存
python visualize_pose_3d.py data.bin --mp4
```

飛行軌跡と姿勢変化を同時に確認できます。

---

## 5. optimize_eskf.py - Q/Rパラメータ最適化ツール

ESKFのプロセスノイズ(Q)と観測ノイズ(R)パラメータを最適化します。

### 最適化対象パラメータ

**プロセスノイズ (Q):**
- `gyro_noise`: ジャイロノイズ
- `accel_noise`: 加速度計ノイズ
- `gyro_bias_noise`: ジャイロバイアスランダムウォーク
- `accel_bias_noise`: 加速度計バイアスランダムウォーク

**観測ノイズ (R):**
- `flow_noise`: オプティカルフローノイズ
- `tof_noise`: ToFノイズ
- `accel_att_noise`: 加速度計姿勢補正ノイズ

### 最適化手法

| 手法 | オプション | 説明 |
|------|-----------|------|
| シミュレーテッドアニーリング | `--method sa` | グローバル最適化（推奨、デフォルト） |
| 最急降下法 | `--method gd` | 局所最適化（高速だが局所解に陥る可能性） |

### 基本的な使い方

```bash
# 単一データセットで最適化
python optimize_eskf.py flow01.bin

# 複数データセットで最適化（汎用性向上）
python optimize_eskf.py flow01.bin flow_sa.bin

# 最適化してeskf.hppに自動適用
python optimize_eskf.py flow01.bin flow_sa.bin --apply
```

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `--method {sa,gd}` | 最適化手法 | sa |
| `--iter N` | 最大イテレーション数 | SA:500, GD:80 |
| `--roll-weight W` | コスト関数でのRoll誤差重み | 0.3 |
| `--output FILE` | パラメータをJSONで保存 | - |
| `--apply` | eskf.hppに自動適用 | - |
| `--quiet` | 出力を抑制 | - |

### 使用例

```bash
# 詳細な最適化（イテレーション数増加）
python optimize_eskf.py flow01.bin flow_sa.bin --method sa --iter 1000

# 最急降下法で高速最適化
python optimize_eskf.py flow01.bin --method gd --iter 100

# パラメータをJSONに保存
python optimize_eskf.py flow01.bin --output params.json

# Roll誤差を重視した最適化
python optimize_eskf.py flow01.bin --roll-weight 0.5
```

### 出力例

```
============================================================
OPTIMIZATION COMPLETE
============================================================
Total evaluations: 8753
Best total cost: 11.851

------------------------------------------------------------
Results per dataset:
------------------------------------------------------------
  flow01.bin: Roll=2.06deg, dist=9.94cm
  flow_sa.bin: Roll=4.30deg, dist=0.00cm

------------------------------------------------------------
Optimized Parameters:
------------------------------------------------------------
  gyro_noise           = 0.009655
  accel_noise          = 0.062885
  gyro_bias_noise      = 0.000013
  accel_bias_noise     = 0.050771
  flow_noise           = 0.005232
  tof_noise            = 0.002540
  accel_att_noise      = 0.514334

------------------------------------------------------------
C++ format (for eskf.hpp):
------------------------------------------------------------
cfg.gyro_noise = 0.009655f;
cfg.accel_noise = 0.062885f;
...
```

---

## 6. visualize_optimization.py - 最適化過程の可視化

最適化過程をリアルタイムで実行・可視化します。

```bash
python visualize_optimization.py
```

以下を9パネルで表示：
- コスト関数の収束
- Roll/位置誤差の推移
- 各パラメータの変化
- 勾配ノルムの推移
- パラメータ空間の軌跡
- 最終位置の変化

---

## 7. estimate_qr.py - 静止データからQ/R推定

静止状態で取得したセンサデータからQ/Rパラメータを統計的に推定します。

```bash
python estimate_qr.py --input static_calibration.bin --output eskf_params.json
python estimate_qr.py --input static_calibration.bin --plot noise_analysis.png
```

### 手法

- Allan分散解析によるIMUノイズ特性推定
- 各センサの統計的ノイズ特性を計算
- 理論的な初期値を得るのに有用

### optimize_eskf.pyとの違い

| ツール | 手法 | データ | 用途 |
|--------|------|--------|------|
| `estimate_qr.py` | 統計解析 | 静止データ | 理論的初期値 |
| `optimize_eskf.py` | SA最適化 | 動的データ | 実測性能最適化 |

---

## 8. pure_accel_integration.py - 加速度純積分解析

加速度をESKFなしで純積分し、ドリフト特性を確認します。

```bash
python pure_accel_integration.py data.bin
```

ESKFのセンサフュージョン効果を理解するための参考ツールです。

---

## 典型的なワークフロー

### 1. ESKF検証

```bash
# 1. デバイスでログキャプチャ
cd tools/log_capture
python log_capture.py capture -p /dev/tty.usbmodem* -o flight.bin -d 60

# 2. PC版ESKFでリプレイ
cd ../../firmware/vehicle/tests/eskf_debug/build
./eskf_replay ../../../logs/flight.bin flight_pc.csv

# 3. 結果を可視化
cd ../../../../../tools/log_analyzer
python viz_all.py flight_pc.csv
python viz_compare.py flight_pc.csv
```

### 2. Q/Rパラメータ最適化

```bash
cd tools/log_analyzer

# 1. テストデータ取得（定点でロール動揺など）
cd ../log_capture
python log_capture.py capture -p /dev/tty.usbmodem* -o test.bin -d 30

# 2. 最適化実行
cd ../log_analyzer
python optimize_eskf.py ../log_capture/test.bin --method sa --iter 500

# 3. 結果を確認してeskf.hppに適用
python optimize_eskf.py ../log_capture/test.bin --apply

# 4. eskf_replayをリビルドして検証
cd ../../firmware/vehicle/tests/eskf_debug/build
cmake .. && make -j4
./eskf_replay ../../../logs/test.bin /tmp/result.csv

# 5. 結果を可視化
cd ../../../../../tools/log_analyzer
python viz_all.py /tmp/result.csv
```

### 3. 複数データでの汎用最適化

```bash
cd tools/log_capture

# 異なる条件のデータを取得
python log_capture.py capture -p /dev/tty.usbmodem* -o small_osc.bin -d 30
python log_capture.py capture -p /dev/tty.usbmodem* -o large_osc.bin -d 30

# 両方のデータで最適化（汎用性向上）
cd ../log_analyzer
python optimize_eskf.py ../log_capture/small_osc.bin ../log_capture/large_osc.bin --apply
```

---

## トラブルシューティング

### シリアルポートが見つからない

```bash
# macOS
ls /dev/tty.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

### 最適化が収束しない

- イテレーション数を増やす: `--iter 1000`
- 複数データセットで実行して汎用性確保
- `--roll-weight` を調整

### グラフが表示されない

- `--no-show` オプションを外す
- `--save` で画像に保存して確認
- matplotlibバックエンドを確認: `python -c "import matplotlib; print(matplotlib.get_backend())"`
