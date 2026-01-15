# sf cal

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

シリアル接続経由でStampFlyのセンサキャリブレーションを実行します。

## 2. サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能なキャリブレーション一覧 |
| `gyro` | ジャイロバイアスキャリブレーション |
| `accel` | 加速度計キャリブレーション |
| `mag` | 磁気キャリブレーション（インタラクティブ） |
| `level` | 水平キャリブレーション |
| `status` | キャリブレーション状態表示 |
| `plot` | 磁気データプロット |

## 3. sf cal list

利用可能なキャリブレーションタイプを一覧表示します。

```bash
sf cal list
```

## 4. sf cal gyro

ジャイロスコープのバイアスキャリブレーションを実行します。

```bash
sf cal gyro                 # 自動検出ポートを使用
sf cal gyro -p /dev/ttyUSB0 # ポート指定
```

### 手順

1. 機体を完全に静止させる
2. コマンドを実行
3. 2-3秒間静止を維持

## 5. sf cal accel

加速度計のキャリブレーションを実行します。

```bash
sf cal accel
```

### 手順

1. 機体を水平な面に置く
2. コマンドを実行
3. 静止を維持

## 6. sf cal mag

磁気センサのキャリブレーションを実行します（インタラクティブ）。

```bash
sf cal mag start   # キャリブレーション開始
sf cal mag stop    # キャリブレーション停止
sf cal mag status  # 現在の状態確認
sf cal mag save    # キャリブレーション保存
sf cal mag clear   # キャリブレーションクリア
```

### ワークフロー

```bash
# 1. キャリブレーション開始
sf cal mag start

# 2. 機体を回転（8の字を描くように）
#    - 全方向をカバー
#    - 約30秒間継続

# 3. 停止
sf cal mag stop

# 4. 状態確認
sf cal mag status

# 5. 保存
sf cal mag save
```

### 判定基準

- **正常**: XYプロットが原点中心の円形
- **要調整**: 円の中心がオフセット、または楕円形

## 7. sf cal level

水平キャリブレーション（姿勢基準）を実行します。

```bash
sf cal level
```

### 手順

1. 機体を水平な面に置く
2. コマンドを実行
3. 着陸検出時に自動キャリブレーション

## 8. sf cal status

現在のキャリブレーション状態を表示します。

```bash
sf cal status
```

## 9. sf cal plot

バイナリログから磁気データをプロットし、キャリブレーションを検証します。

```bash
sf cal plot                  # 最新ログをプロット
sf cal plot sensor.bin       # 指定ファイルをプロット
sf cal plot sensor.bin -o mag.png  # 画像保存
```

### 出力

- XY散布図（時間で色分け）
- ノルム時系列グラフ
- ハードアイアンオフセット推定値

---

<a id="english"></a>

## 1. Overview

Run sensor calibration via serial connection to StampFly.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List available calibrations |
| `gyro` | Gyroscope bias calibration |
| `accel` | Accelerometer calibration |
| `mag` | Magnetometer calibration (interactive) |
| `level` | Level calibration |
| `status` | Show calibration status |
| `plot` | Plot magnetometer data |

## 3. sf cal mag

Interactive magnetometer calibration.

```bash
sf cal mag start   # Start calibration
sf cal mag stop    # Stop calibration
sf cal mag status  # Check status
sf cal mag save    # Save calibration
sf cal mag clear   # Clear calibration
```

### Workflow

```bash
# 1. Start calibration
sf cal mag start

# 2. Rotate drone (figure-8 pattern)
#    - Cover all orientations
#    - Continue for ~30 seconds

# 3. Stop
sf cal mag stop

# 4. Check status
sf cal mag status

# 5. Save
sf cal mag save
```

## 4. sf cal plot

Plot magnetometer data from binary log to verify calibration.

```bash
sf cal plot                  # Plot latest log
sf cal plot sensor.bin       # Plot specific file
sf cal plot sensor.bin -o mag.png  # Save image
```

### Output

- XY scatter plot (colored by time)
- Norm time series graph
- Hard iron offset estimate
