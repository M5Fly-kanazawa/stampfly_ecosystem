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

# Stage 3: BMI270 Polling Data Read

## 目的

BMI270からポーリング方式でセンサーデータを連続的に読み取ります：

- センサー設定（レンジ、ODR、フィルター性能）
- 加速度・ジャイロデータの読み取り
- 生データ（LSB）と物理値（g、°/s）の表示
- ポーリング読み取りパターンの実装

## ハードウェア

- **MCU**: ESP32-S3 (M5StampS3)
- **センサー**: BMI270 6軸IMU
- **通信**: SPI (10 MHz, Mode 0)

### ピン接続

| 信号 | ESP32-S3 GPIO | BMI270ピン |
|------|---------------|------------|
| MOSI | GPIO14 | SDx (SDI) |
| MISO | GPIO43 | SDO |
| SCK  | GPIO44 | SCx |
| CS   | GPIO46 | CSB |

## センサー設定

このサンプルプログラムでは以下の設定を使用します：

| パラメータ | 設定値 | レジスタ値 |
|----------|--------|-----------|
| 加速度レンジ | ±4g | 0x01 |
| ジャイロレンジ | ±1000 °/s | 0x01 |
| 加速度ODR | 100 Hz | 0x08 |
| ジャイロODR | 200 Hz | 0x09 |
| フィルター性能 | Performance mode | bit 7 = 1 |

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage3_polling
idf.py set-target esp32s3
```

### 3. ビルド

```bash
idf.py build
```

### 4. フラッシュ＆モニター

```bash
idf.py flash monitor
```

終了: `Ctrl+]`

## 出力設定

デフォルトでは、物理値（g、°/s、°C）のみを出力します。RAW値（LSB）を表示したい場合は、[main.c:31](main/main.c#L31)を修正してください:

```c
#define OUTPUT_RAW_VALUES   1  // Set to 1 to output raw sensor values (LSB)
```

## 期待される出力

### 初期化ログ

```
I (xxx) BMI270_STAGE3: ========================================
I (xxx) BMI270_STAGE3:  BMI270 Stage 3: Polling Data Read
I (xxx) BMI270_STAGE3: ========================================

I (xxx) BMI270_STAGE3: Step 1: Initializing SPI...
I (xxx) BMI270_SPI: SPI bus initialized on host 1
I (xxx) BMI270_SPI: BMI270 SPI device added successfully
I (xxx) BMI270_STAGE3: ✓ SPI initialized successfully

...

I (xxx) BMI270_STAGE3: Step 5: Starting continuous data reading...
I (xxx) BMI270_STAGE3: Polling interval: 10 ms (100.0 Hz)

I (xxx) BMI270_STAGE3: ========================================
I (xxx) BMI270_STAGE3:  Teleplot Data Stream (press Ctrl+] to stop)
I (xxx) BMI270_STAGE3: ========================================
```

### デフォルト出力（物理値のみ）

Teleplot形式で7チャンネルのデータを出力:

```
>acc_x:-0.0299
>acc_y:0.0156
>acc_z:1.0000
>gyr_x:0.366
>gyr_y:-0.244
>gyr_z:0.091
>temp:25.00
```

### RAW値有効時 (OUTPUT_RAW_VALUES=1)

14チャンネル（RAW値を含む）:

```
>acc_raw_x:-245
>acc_raw_y:128
>acc_raw_z:8192
>acc_x:-0.0299
>acc_y:0.0156
>acc_z:1.0000
>gyr_raw_x:12
>gyr_raw_y:-8
>gyr_raw_z:3
>gyr_x:0.366
>gyr_y:-0.244
>gyr_z:0.091
>temp_raw:1024
>temp:25.00
```

## 成功基準

- [x] SPI通信成功
- [x] BMI270初期化成功
- [x] センサー設定成功（レンジ、ODR、フィルター）
- [x] 加速度データ読み取り成功（生データ + 物理値）
- [x] ジャイロデータ読み取り成功（生データ + 物理値）
- [x] 連続データ読み取り（安定）
- [x] 物理値の妥当性確認（静止時: Z軸 ≈ 1.0g）

## 重要な技術ポイント

### 1. レンジ設定とスケール係数

**加速度センサー**:
- ±2g: 16384 LSB/g
- ±4g: 8192 LSB/g (このサンプルで使用)
- ±8g: 4096 LSB/g
- ±16g: 2048 LSB/g

**ジャイロスコープ**:
- ±125 °/s: 262.4 LSB/(°/s)
- ±250 °/s: 131.2 LSB/(°/s)
- ±500 °/s: 65.6 LSB/(°/s)
- ±1000 °/s: 32.8 LSB/(°/s) (このサンプルで使用)
- ±2000 °/s: 16.4 LSB/(°/s)

### 2. 生データから物理値への変換

```c
// 加速度の変換（±4g レンジ）
float accel_g = (float)raw_value / 8192.0f;

// 角速度の変換（±1000 °/s レンジ）
float gyro_dps = (float)raw_value / 32.8f;
```

### 3. バースト読み取りとシャドウイング

データの完全性を保証するため、6バイトのバースト読み取りを使用：

```c
// 加速度データ: ACC_X_LSB (0x0C) から6バイト
uint8_t data[6];
burst_read(0x0C, data, 6);

// シャドウイング: LSB読み取り時にMSBがロックされる
int16_t acc_x = (int16_t)((data[1] << 8) | data[0]);
int16_t acc_y = (int16_t)((data[3] << 8) | data[2]);
int16_t acc_z = (int16_t)((data[5] << 8) | data[4]);
```

### 4. ODR設定

**ACC_CONF (0x40)** と **GYR_CONF (0x42)** レジスタの構成：

```
bit 7:    Filter performance (1=Performance, 0=Power optimized)
bits 6-4: Reserved
bits 3-0: ODR value
```

例:
- 100Hz, Performance mode: `0x88` (0b10001000)
- 200Hz, Performance mode: `0x89` (0b10001001)

### 5. ポーリング間隔の選択

- **高速ポーリング（≤10ms）**: データ取りこぼしなし、CPU負荷高
- **標準ポーリング（100ms）**: バランス良好（このサンプル）
- **低速ポーリング（≥1000ms）**: 低消費電力、リアルタイム性低

**重要**: ポーリング間隔はODRより長くする必要があります。ODRが200Hzの場合、最小ポーリング間隔は5ms（1/200Hz）です。

## トラブルシューティング

### データ読み取りエラー

**症状**: `Failed to read accelerometer` または `Failed to read gyroscope`

**対策**:
1. 初期化が正常に完了しているか確認
2. SPI通信エラーログを確認
3. 配線を再確認（特にMOSI/MISO）

### 物理値が異常

**症状**: 加速度のZ軸が1.0gから大きくずれる

**対策**:
1. レンジ設定が正しいか確認（ACC_RANGE レジスタ）
2. スケール係数が正しいか確認
3. センサーの向きを確認（Z軸が重力方向を向いているか）

### データが更新されない

**症状**: 同じ値が連続して読み取られる

**対策**:
1. ODR設定を確認（ACC_CONF, GYR_CONF レジスタ）
2. センサーが有効化されているか確認（PWR_CTRL レジスタ）
3. ポーリング間隔がODRより長いか確認

## 次のステップ

Stage 3が成功したら、次は **Stage 4: 割り込み読み取り** に進みます。

Stage 4では以下を実装します：
- 割り込みピンの設定
- Data Ready割り込みの使用
- イベント駆動型データ読み取り
- より効率的なデータ取得パターン

## 参考ドキュメント

- [docs/bmi270_doc_ja.md](../../docs/bmi270_doc_ja.md) - BMI270実装ガイド
  - 4.2節: センサーの有効化と設定
  - 4.3節: ダイレクトレジスタ読み取り
  - 4.4節: レンジ設定とスケール係数
  - 4.5節: データ読み取りの実装パターン
- [include/bmi270_data.h](../../include/bmi270_data.h) - データ読み取りAPI仕様
- [src/bmi270_data.c](../../src/bmi270_data.c) - データ読み取り実装
