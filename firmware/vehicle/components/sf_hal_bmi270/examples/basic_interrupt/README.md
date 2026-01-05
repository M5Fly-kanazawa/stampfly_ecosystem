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

# Basic Interrupt Example - 割り込みサンプル

BMI270のDATA_RDY割り込みを使用して、効率的にセンサーデータを取得するサンプルです。

## 特徴

- **効率的**: CPU使用率が低い（データがない時はスリープ）
- **即応性**: 新しいデータが即座に利用可能
- **実用的**: バッテリー駆動アプリケーションに最適

## ポーリングとの比較

| 項目 | ポーリング | 割り込み（本サンプル） |
|------|-----------|----------------------|
| CPU使用率 | 高い（常時チェック） | 低い（データがある時のみ動作） |
| 応答性 | ポーリング間隔に依存 | 即座（数μs） |
| 消費電力 | 高い | 低い |
| 実装の難易度 | 簡単 | やや複雑 |

## 動作概要

1. SPI通信初期化
2. BMI270センサー初期化
3. センサー設定（100Hz, ±4g, ±1000°/s）
4. GPIO11をINT1割り込みピンとして設定
5. BMI270のDATA_RDY割り込みを有効化
6. **データ準備完了時に割り込み発生**
7. セマフォでタスクを起床
8. データ読み取り＆出力
9. タスクがスリープ（次の割り込みまで）

## ハードウェア接続

このサンプルはM5StampFly向けに設定されています：

| BMI270 | ESP32-S3 GPIO | 用途 |
|--------|---------------|------|
| MOSI   | GPIO14        | SPI データ出力 |
| MISO   | GPIO43        | SPI データ入力 |
| SCK    | GPIO44        | SPI クロック |
| CS     | GPIO46        | SPI チップセレクト |
| **INT1** | **GPIO11** | **DATA_RDY割り込み** |

## ビルド＆実行

```bash
# 環境セットアップ
source ~/esp/esp-idf/export.sh

# ターゲット設定（初回のみ）
cd examples/basic_interrupt
idf.py set-target esp32s3

# ビルド＆書き込み
idf.py build flash monitor
```

## 期待される出力

```
I (XXX) BMI270_BASIC_INT: ========================================
I (XXX) BMI270_BASIC_INT:  BMI270 Basic Interrupt Example
I (XXX) BMI270_BASIC_INT: ========================================
I (XXX) BMI270_BASIC_INT: Step 1: Initializing SPI bus...
I (XXX) BMI270_BASIC_INT: SPI initialized
I (XXX) BMI270_BASIC_INT: Step 2: Initializing BMI270 sensor...
I (XXX) BMI270_BASIC_INT: BMI270 initialized (CHIP_ID: 0x24)
I (XXX) BMI270_BASIC_INT: Step 3: Configuring sensors (100Hz, ±4g, ±1000°/s)...
I (XXX) BMI270_BASIC_INT: Step 4: Configuring GPIO INT1 (GPIO11)...
I (XXX) BMI270_BASIC_INT: Step 5: Creating semaphore...
I (XXX) BMI270_BASIC_INT: Step 6: Configuring DATA_RDY interrupt...
I (XXX) BMI270_BASIC_INT: DATA_RDY interrupt configured on INT1 (GPIO11)
I (XXX) BMI270_BASIC_INT: Step 7: Creating data acquisition task...
I (XXX) BMI270_BASIC_INT: Data acquisition task started (waiting for interrupts)
I (XXX) BMI270_BASIC_INT: ========================================
I (XXX) BMI270_BASIC_INT:  Interrupt-driven data acquisition active
I (XXX) BMI270_BASIC_INT:  Waiting for DATA_RDY interrupts @ 100Hz
I (XXX) BMI270_BASIC_INT: ========================================
I (XXX) BMI270_BASIC_INT: Sample #10:
I (XXX) BMI270_BASIC_INT:   Gyro  [rad/s]: X=  -0.002  Y=   0.003  Z=  -0.001
I (XXX) BMI270_BASIC_INT:   Accel [g]:     X=   0.012  Y=  -0.024  Z=   0.995
I (XXX) BMI270_BASIC_INT:   Temp  [°C]:     25.50
>gyr_x:-0.002
>gyr_y:0.003
>gyr_z:-0.001
>acc_x:0.012
>acc_y:-0.024
>acc_z:0.995
```

## 割り込み動作の仕組み

### INT1ピン設定

```c
// INT1ピン: アクティブハイ、プッシュプル、出力有効
uint8_t int1_io_ctrl = (1 << 1) | (1 << 3);
bmi270_write_register(&g_dev, BMI270_REG_INT1_IO_CTRL, int1_io_ctrl);
```

### DATA_RDY割り込みマッピング

```c
// DATA_RDY割り込みをINT1にマッピング
uint8_t int_map_data = (1 << 2);  // bit2: drdy_int -> INT1
bmi270_write_register(&g_dev, BMI270_REG_INT_MAP_DATA, int_map_data);
```

### GPIO割り込みハンドラ

```c
static void IRAM_ATTR bmi270_int1_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // セマフォを渡してタスクを起床
    xSemaphoreGiveFromISR(data_ready_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
```

## パラメータ調整

### サンプリング周波数を変更

200Hzに変更する場合：

```c
#define SENSOR_ODR_HZ       200       // 100 → 200Hz

// センサー設定も変更
bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
```

**注意**: 割り込み周波数もODRに合わせて自動的に200Hzになります。

### INT1をラッチモードにする

デフォルトは非ラッチモード（パルス）ですが、ラッチモードに変更可能：

```c
// bit0=1でラッチモード有効
uint8_t int1_io_ctrl = (1 << 0) | (1 << 1) | (1 << 3);
bmi270_write_register(&g_dev, BMI270_REG_INT1_IO_CTRL, int1_io_ctrl);
```

ラッチモードの場合、INT_STATUS_1レジスタを読むとクリアされます。

## CPU使用率の最適化

割り込み駆動の利点を最大限活用するために：

1. **メインタスクを低優先度に**
   ```c
   xTaskCreate(data_acquisition_task, "data_acq", 4096, NULL, 5, NULL);  // 優先度5
   ```

2. **データ処理を最小化**
   - ISRハンドラ内では最小限の処理のみ
   - セマフォでタスクに処理を委譲

3. **スリープモードを活用**
   - データがない時はタスクが自動的にブロック
   - FreeRTOSがCPUを低消費電力モードに移行

## 次のステップ

さらに高度なデータ取得方法：

- [`basic_fifo`](../basic_fifo/) - FIFOバッファで複数フレームを一括読み取り

## トラブルシューティング

### 割り込みが発生しない

1. **GPIO配線を確認**
   - INT1ピン（GPIO11）が正しく接続されているか

2. **INT1設定を確認**
   ```c
   // デバッグ用: INT1_IO_CTRLレジスタを読み取り
   uint8_t int1_ctrl;
   bmi270_read_register(&g_dev, BMI270_REG_INT1_IO_CTRL, &int1_ctrl);
   ESP_LOGI(TAG, "INT1_IO_CTRL: 0x%02X", int1_ctrl);
   ```

3. **割り込みマッピングを確認**
   ```c
   // デバッグ用: INT_MAP_DATAレジスタを読み取り
   uint8_t int_map;
   bmi270_read_register(&g_dev, BMI270_REG_INT_MAP_DATA, &int_map);
   ESP_LOGI(TAG, "INT_MAP_DATA: 0x%02X", int_map);
   ```

### "Failed to create semaphore"

- ヒープメモリが不足している可能性
- `menuconfig`でFreeRTOSヒープサイズを増やす

### データ読み取りエラー

- 割り込みハンドラ内でSPI通信を行っていないか確認
- タスク内でのみSPI通信を実行

## API仕様

詳細なAPI仕様は[docs/API.md](../../docs/API.md)を参照してください。
