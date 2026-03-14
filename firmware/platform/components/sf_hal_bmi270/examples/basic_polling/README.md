<!--
SPDX-License-Identifier: MIT

Copyright (c) 2025 Kouhei Ito

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# Basic Polling Example - ポーリングサンプル

BMI270センサーからジャイロスコープと加速度計のデータを定期的にポーリングで読み取るシンプルなサンプルです。

## 特徴

- **シンプル**: 最小限のコードでBMI270を使用開始
- **わかりやすい**: 初心者に最適
- **実用的**: そのまま実アプリケーションに組み込み可能

## 動作概要

1. SPI通信初期化
2. BMI270センサー初期化
3. センサー設定（100Hz, ±4g, ±1000°/s）
4. 10msごとにセンサーデータ（ジャイロ、加速度、温度）をポーリング
5. データをログとTeleplot形式で出力（ジャイロはrad/s単位）

## ハードウェア接続

このサンプルはM5StampFly向けに設定されています：

| BMI270 | ESP32-S3 GPIO |
|--------|---------------|
| MOSI   | GPIO14        |
| MISO   | GPIO43        |
| SCK    | GPIO44        |
| CS     | GPIO46        |
| INT1   | (未使用)      |

## ビルド＆実行

```bash
# 環境セットアップ
source ~/esp/esp-idf/export.sh

# ターゲット設定（初回のみ）
cd examples/basic_polling
idf.py set-target esp32s3

# ビルド＆書き込み
idf.py build flash monitor
```

## 期待される出力

```
I (XXX) BMI270_BASIC: ========================================
I (XXX) BMI270_BASIC:  BMI270 Basic Polling Example
I (XXX) BMI270_BASIC: ========================================
I (XXX) BMI270_BASIC: Initializing SPI bus...
I (XXX) BMI270_BASIC: SPI initialized
I (XXX) BMI270_BASIC: Initializing BMI270 sensor...
I (XXX) BMI270_BASIC: BMI270 initialized (CHIP_ID: 0x24)
I (XXX) BMI270_BASIC: Configuring accelerometer (100Hz, ±4g)...
I (XXX) BMI270_BASIC: Configuring gyroscope (100Hz, ±1000°/s)...
I (XXX) BMI270_BASIC: ========================================
I (XXX) BMI270_BASIC:  Starting data acquisition (100 Hz)
I (XXX) BMI270_BASIC: ========================================
I (XXX) BMI270_BASIC: Sample #10:
I (XXX) BMI270_BASIC:   Gyro  [rad/s]: X=  -0.002  Y=   0.003  Z=  -0.001
I (XXX) BMI270_BASIC:   Accel [g]:     X=   0.012  Y=  -0.024  Z=   0.995
I (XXX) BMI270_BASIC:   Temp  [°C]:     25.50
>gyr_x:-0.002
>gyr_y:0.003
>gyr_z:-0.001
>acc_x:0.012
>acc_y:-0.024
>acc_z:0.995
```

## Teleplot可視化

出力される`>channel:value`形式のデータは[Teleplot](https://github.com/nesnes/teleplot)でリアルタイム可視化できます：

1. VSCode拡張機能「Teleplot」をインストール
2. シリアルポートを開く
3. ジャイロ・加速度データがグラフ表示されます

## パラメータ調整

### サンプリング周波数を変更

`main.c`で以下を変更：

```c
#define SENSOR_ODR_HZ       200       // 100 → 200Hz
#define POLLING_INTERVAL_MS 5         // 10 → 5ms (200Hz)

// センサー設定も変更
bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
```

### 測定レンジを変更

ジャイロスコープ：`±1000°/s` → `±2000°/s`

```c
bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
// その後、rangeを設定
bmi270_set_gyro_range(&g_dev, BMI270_GYR_RANGE_2000);
```

加速度計：`±4g` → `±8g`

```c
bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
// その後、rangeを設定
bmi270_set_accel_range(&g_dev, BMI270_ACC_RANGE_8G);
```

## 次のステップ

より効率的なデータ取得方法：

- [`basic_interrupt`](../basic_interrupt/) - 割り込み駆動（CPU効率向上）
- [`basic_fifo`](../basic_fifo/) - FIFOバッファ（高速・低レイテンシ）

## トラブルシューティング

### "Failed to initialize SPI"

- ピン配置が正しいか確認
- 他のSPIデバイスと競合していないか確認

### "Failed to initialize BMI270"

- センサーの電源が入っているか確認
- SPI配線（特にMISO）が正しいか確認

### データが変化しない

- センサーのODR設定が有効になっているか確認
- 100ms待機後にデータ取得しているか確認

## API仕様

詳細なAPI仕様は[docs/API.md](../../docs/API.md)を参照してください。
