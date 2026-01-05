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

# Step 4: FIFO Watermark Interrupt - FIFOウォーターマーク割り込み

## 概要

FIFO内のデータが設定した閾値（ウォーターマーク）に達した時に**割り込み（INT1）**を発生させ、効率的にデータを読み取ります。

## 目的

1. **ウォーターマーク割り込み**: FIFO内のデータが1014バイト（78フレーム、半分満杯）に達したら割り込み発生
2. **INT1ピン使用**: M5StampFly GPIO11に接続されたBMI270 INT1ピンで割り込み受信
3. **イベント駆動処理**: ポーリングではなく割り込みベースでFIFO読み取り
4. **FreeRTOS統合**: セマフォとタスクを使用した効率的な処理
5. **データロス防止**: ウォーターマークを半分満杯に設定し、読み取り処理中のオーバーフロー防止

## Step 3との違い

| 項目 | Step 3 | Step 4 |
|------|--------|--------|
| データ取得方法 | ポーリング（100ms間隔） | 割り込み駆動（イベントベース） |
| CPU使用率 | 高い（定期的にチェック） | 低い（データがある時だけ動作） |
| 応答性 | 最大100ms遅延 | 即座に応答（数us） |
| FIFO設定 | ウォーターマークなし | 1014バイト（78フレーム） |
| GPIO使用 | なし | GPIO11（INT1） |
| FreeRTOS | 1タスク | ISR + セマフォ + 専用タスク |

## ハードウェア接続

### M5StampFly BMI270ピン配置

```
ESP32-S3 GPIO11 → BMI270 INT1ピン
```

**注意**: BMI270のINT2ピンは未接続です。割り込み機能はINT1（GPIO11）のみ使用可能です。

## FIFO設定詳細

### ウォーターマーク設定

```c
// FIFO_WTM_0 (0x46): ウォーターマーク LSB
// FIFO_WTM_1 (0x47): ウォーターマーク MSB（下位3ビットのみ有効）
#define FIFO_WATERMARK_BYTES    416     // 32フレーム（13バイト/フレーム）

uint16_t watermark = FIFO_WATERMARK_BYTES;
uint8_t wtm_lsb = watermark & 0xFF;           // 0xA0
uint8_t wtm_msb = (watermark >> 8) & 0x07;    // 0x01

bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_0, wtm_lsb);
bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_1, wtm_msb);
```

**なぜ416バイト（32フレーム）？**
- FIFO総容量: 2048バイト
- 1フレーム: 13バイト（ヘッダー1 + GYR6 + ACC6）
- 最大格納可能: 157フレーム（2041バイト）
- ウォーターマーク: 32フレーム = **約1/5満杯**
- 理由: 1600Hz ODRで50Hz出力を実現（32フレーム ÷ 1600Hz = 20ms間隔）
- 割り込み発生後、読み取り処理中も新しいデータが蓄積されるが、十分な余裕あり

### INT1ピン設定

```c
// INT1_IO_CTRL (0x53): INT1ピン動作設定
#define BMI270_REG_INT1_IO_CTRL     0x53

uint8_t int1_io_ctrl = 0x0A;  // bit3=1 (output enable), bit1=1 (active high)
bmi270_write_register(&g_dev, BMI270_REG_INT1_IO_CTRL, int1_io_ctrl);
```

**ビット設定詳細**:
- bit[3]: `1` = INT1出力有効
- bit[1]: `1` = アクティブハイ（立ち上がりエッジで割り込み）
- bit[0]: `0` = プッシュプル出力

### 割り込みマッピング

```c
// INT_MAP_DATA (0x58): データ割り込みマッピング
#define BMI270_REG_INT_MAP_DATA     0x58

uint8_t int_map_data = 0x02;  // bit1=1 (FIFO watermark → INT1)
bmi270_write_register(&g_dev, BMI270_REG_INT_MAP_DATA, int_map_data);
```

**ビット設定詳細**:
- bit[1]: `1` = FIFOウォーターマーク割り込みをINT1にマッピング
- bit[0]: `0` = FIFOフル割り込みは無効

## GPIO割り込み設定（ESP32-S3側）

### GPIO11設定

```c
#define BMI270_INT1_PIN     11

gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BMI270_INT1_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE  // 立ち上がりエッジ（アクティブハイ）
};
gpio_config(&io_conf);
```

### ISRハンドラ登録

```c
static SemaphoreHandle_t fifo_semaphore = NULL;

static void IRAM_ATTR bmi270_int1_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(fifo_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

gpio_install_isr_service(0);
gpio_isr_handler_add(BMI270_INT1_PIN, bmi270_int1_isr_handler, NULL);
```

**重要**: ISRハンドラは`IRAM_ATTR`属性が必要（RAMから高速実行）

## FreeRTOSタスク構成

### タスクフロー

```
[BMI270] FIFO ≥ 1014バイト
    ↓
[INT1] GPIO11 立ち上がりエッジ
    ↓
[ISR] セマフォを渡す (xSemaphoreGiveFromISR)
    ↓
[FIFO読み取りタスク] セマフォ待機中 → 起床
    ↓
FIFO_LENGTH読み取り → FIFOデータ一括読み取り → パース → 平均値計算 → Teleplot出力
    ↓
再びセマフォ待機（次の割り込みまでブロック）
```

### 専用FIFO読み取りタスク

```c
static void fifo_read_task(void *arg)
{
    while (1) {
        // 割り込み待機（セマフォ取得）
        if (xSemaphoreTake(fifo_semaphore, portMAX_DELAY) == pdTRUE) {
            // タイムスタンプ記録
            int64_t int_time_us = esp_timer_get_time();
            g_interrupt_count++;

            // FIFO_LENGTH読み取り
            uint16_t fifo_length;
            esp_err_t ret = bmi270_read_fifo_length(&g_dev, &fifo_length);

            if (ret == ESP_OK && fifo_length > 0) {
                // FIFO一括読み取り
                ret = bmi270_read_fifo_data(&g_dev, fifo_buffer, fifo_length);

                if (ret == ESP_OK) {
                    // フレームパースと平均値計算
                    bool skip_detected = parse_fifo_buffer(fifo_buffer, fifo_length, int_time_us);

                    // Skip frame検出時はフラッシュ
                    if (skip_detected) {
                        fifo_flush();
                    }
                }
            }
        }
    }
}
```

## 動作フロー

1. センサー初期化（ACC/GYR 1600Hz設定）
2. FIFO設定（ACC+GYR、ヘッダーモード、ストリームモード）
3. ウォーターマーク設定（416バイト）
4. INT1割り込み設定（GPIO11、立ち上がりエッジ）
5. 割り込みマッピング（FIFO watermark → INT1）
6. FIFO読み取りタスク起動（セマフォ待機）
7. ループ:
   - **割り込み発生** → ISRでセマフォを渡す
   - タスク起床 → FIFO_LENGTH読み取り
   - FIFO一括読み取り（バースト転送）
   - フレームパース → データ蓄積
   - 平均値計算 → Teleplot出力
   - 統計情報表示
   - セマフォ待機（次の割り込みまでブロック）

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage5_fifo/step4_watermark_int
idf.py set-target esp32s3
```

### 3. ビルド

```bash
idf.py build
```

### 4. 書き込みとモニタ

```bash
idf.py flash monitor
```

## 期待される出力

### 起動時ログ

```
I (XXX) BMI270_STEP4: ========================================
I (XXX) BMI270_STEP4:  BMI270 Step 4: FIFO Watermark Interrupt
I (XXX) BMI270_STEP4: ========================================
I (XXX) BMI270_STEP4: FIFO watermark set to 1014 bytes (78 frames)
I (XXX) BMI270_STEP4: INT1 pin configured (GPIO 11)
I (XXX) BMI270_STEP4: FIFO watermark interrupt mapped to INT1
I (XXX) BMI270_STEP4: FIFO_CONFIG_1 readback: 0xD0 (expected 0xD0)
I (XXX) BMI270_STEP4: Waiting for FIFO watermark interrupts...
```

### 割り込み発生とデータ読み取り

```
I (XXX) BMI270_STEP4: Interrupt #1, FIFO length: 429 bytes
>gyr_x:-0.05
>gyr_y:0.12
>gyr_z:-0.03
>acc_x:0.012
>acc_y:-0.024
>acc_z:0.995
I (XXX) BMI270_STEP4: Statistics: Total=33 Valid=32 Skip=0 Config=1 Interrupts=1
```

**重要なポイント**:
- 割り込み間隔: 約20ms（32フレーム ÷ 1600Hz = 0.02秒、50Hz出力）
- FIFO長: 416バイト以上（ウォーターマーク以上で割り込み発生）
- フレーム数: 約32フレーム（設定通り）

### 連続動作

```
I (XXX) BMI270_STEP4: Interrupt #2, FIFO length: 416 bytes
>gyr_x:-0.06
>gyr_y:0.11
>gyr_z:-0.03
>acc_x:0.012
>acc_y:-0.024
>acc_z:0.995
I (XXX) BMI270_STEP4: Statistics: Total=65 Valid=64 Skip=0 Config=1 Interrupts=2

I (XXX) BMI270_STEP4: Interrupt #3, FIFO length: 416 bytes
...
```

## 検証ポイント

### ✓ 割り込みが正常に発生しているか

```
I (XXX) BMI270_STEP4: Interrupt #1, FIFO length: 416 bytes
```
→ 割り込みカウンタが増加し、FIFO長が416バイト前後であればOK

### ✓ 割り込み間隔が適切か

```
タイムスタンプ差分: 約20ms間隔
```
→ 32フレーム ÷ 1600Hz = 0.02秒（20ms）の間隔で割り込みが発生すればOK

### ✓ データロスがないか

```
Statistics: ... Skip=0 ... Flush=0
```
→ Skip=0、Flush=0であれば、データロスなく正常動作

### ✓ CPU使用率が低いか

割り込み駆動のため、データがない時はタスクがブロック状態となり、CPU使用率が大幅に削減されます。

## 動作特性

- **イベント駆動**: ポーリングなし、データがある時だけ動作
- **低レイテンシ**: 割り込み発生から数us以内に処理開始
- **低CPU使用率**: データがない時はタスクがブロック状態（電力削減）
- **データロス防止**: ウォーターマークを半分満杯に設定し、読み取り処理中のオーバーフロー防止
- **平均化処理**: 複数フレームを平均化してノイズ低減
- **自動復旧**: Skip frame検出時にFIFOフラッシュ

## 次のステップ

**Step 5 (step5_optimized)**: ヘッダーレスモード最適化
- FIFOフレームからヘッダーを削除（13バイト → 12バイト）
- スループット向上（約8%改善）
- 将来の高ODR対応（400Hz, 800Hz以上）
