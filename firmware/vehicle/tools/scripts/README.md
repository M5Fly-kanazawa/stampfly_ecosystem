# StampFly ESKF ツール群

StampFlyデバイスのESKF（Error-State Kalman Filter）開発・検証・最適化のためのPythonツール群です。

## ファイル構成

```
tools/scripts/
├── 可視化ツール
│   ├── visualize_eskf.py         # 統合可視化ツール（メイン）
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
├── ユーティリティ
│   ├── log_capture.py            # ログキャプチャ・変換
│   ├── plot_mag_xy.py            # 地磁気キャリブレーション確認
│   └── pure_accel_integration.py # 加速度純積分解析
└── データファイル
    ├── flow01.bin                # テストデータ（小振幅ロール動揺）
    └── flow_sa.bin               # テストデータ（大振幅ロール動揺）
```

## 必要なライブラリ

```bash
pip install numpy pandas matplotlib pyserial scipy
```

---

## 1. visualize_eskf.py - 統合可視化ツール

ESKFの状態量とセンサ生値を包括的に可視化するメインツールです。

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

## 8. log_capture.py - ログキャプチャツール

StampFlyデバイスからUSBシリアル経由でバイナリログをキャプチャします。

### パケットフォーマット

| バージョン | サイズ | ヘッダー | 内容 | 状態 |
|-----------|--------|---------|------|------|
| V2 | 128 bytes | 0xAA 0x56 | センサデータ + ESKF推定結果 | **現行** |
| V1 | 64 bytes | 0xAA 0x55 | センサデータのみ | 非推奨 |

> **注意**: V1は非推奨です。すべてのPythonツールはV2のみをサポートします。
>
> **詳細なフォーマット仕様**: [`docs/eskf_workflow.md`](../../docs/eskf_workflow.md#バイナリログ形式) および [`cli.hpp`](../../components/stampfly_cli/include/cli.hpp) の `BinaryLogPacketV2` を参照。

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

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `--port`, `-p` | シリアルポート | 必須 |
| `--output`, `-o` | 出力ファイル | 必須 |
| `--duration`, `-d` | キャプチャ時間（秒） | 60 |
| `--baudrate`, `-b` | ボーレート | 115200 |
| `--live`, `-l` | リアルタイム表示 | - |
| `--no-auto` | binlog on/offを送信しない | - |
| `--debug` | デバッグ出力 | - |

---

## 9. plot_mag_xy.py - 地磁気キャリブレーション確認

地磁気センサのXYプロットでキャリブレーションを確認します。

```bash
python plot_mag_xy.py sensor.bin
```

校正が正しければ、点が原点 (0, 0) を中心とした円状に分布します。

---

## 10. pure_accel_integration.py - 加速度純積分解析

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
python log_capture.py capture -p /dev/tty.usbmodem* -o flight.bin -d 60

# 2. PC版ESKFでリプレイ
cd ../eskf_debug/build
./eskf_replay ../../scripts/flight.bin flight_pc.csv

# 3. 結果を可視化
cd ../../scripts
python viz_all.py flight_pc.csv
python viz_compare.py flight_pc.csv
```

### 2. Q/Rパラメータ最適化

```bash
# 1. テストデータ取得（定点でロール動揺など）
python log_capture.py capture -p /dev/tty.usbmodem* -o test.bin -d 30

# 2. 最適化実行
python optimize_eskf.py test.bin --method sa --iter 500

# 3. 結果を確認してeskf.hppに適用
python optimize_eskf.py test.bin --apply

# 4. eskf_replayをリビルドして検証
cd ../eskf_debug/build
cmake .. && make -j4
./eskf_replay ../../scripts/test.bin /tmp/result.csv

# 5. 結果を可視化
cd ../../scripts
python viz_all.py /tmp/result.csv
```

### 3. 複数データでの汎用最適化

```bash
# 異なる条件のデータを取得
python log_capture.py capture -p /dev/tty.usbmodem* -o small_osc.bin -d 30
python log_capture.py capture -p /dev/tty.usbmodem* -o large_osc.bin -d 30

# 両方のデータで最適化（汎用性向上）
python optimize_eskf.py small_osc.bin large_osc.bin --apply
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
