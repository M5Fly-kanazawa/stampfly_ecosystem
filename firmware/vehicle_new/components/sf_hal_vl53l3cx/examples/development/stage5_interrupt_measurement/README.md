# Stage 5: VL53L3CX 割り込みベース距離測定

## 概要

このステージでは、GPIO割り込みを使用した効率的な距離測定を実装します。ポーリング方式と比較して、CPU使用率が低く、低消費電力動作が可能です。

## ポーリング方式との比較

### Stage 4: ポーリング方式
```c
while (measurement_count < MAX) {
    // データ準備完了を待機（CPU常時チェック）
    VL53LX_GetMeasurementDataReady(&dev, &data_ready);
    if (data_ready) {
        VL53LX_GetMultiRangingData(&dev, &data);
        // データ処理
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms待機
}
```
- **CPU使用率**: 高（常時ポーリング）
- **応答性**: 1ms遅延
- **実装**: シンプル

### Stage 5: 割り込み方式
```c
while (measurement_count < MAX) {
    // セマフォ待機（割り込みまでブロック）
    if (xSemaphoreTake(semaphore, timeout) == pdTRUE) {
        VL53LX_GetMultiRangingData(&dev, &data);
        // データ処理
    }
}
```
- **CPU使用率**: 低（割り込み待機中は他のタスクが実行可能）
- **応答性**: 即座（ハードウェア割り込み）
- **実装**: やや複雑

## 割り込み処理フロー

```
1. VL53LX_StartMeasurement()
   ↓
2. センサーが測定開始
   ↓
3. 測定完了時にINTピンがLOWになる（割り込み発生）
   ↓
4. ISRがセマフォをGive
   ↓
5. メインタスクがセマフォをTake（ブロック解除）
   ↓
6. VL53LX_GetMultiRangingData()でデータ取得
   ↓
7. VL53LX_ClearInterruptAndStartMeasurement()
   ↓
8. 手順2に戻る
```

## 実装の詳細

### GPIOの割り込み設定

```c
gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << STAMPFLY_TOF_BOTTOM_INT),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,  // INTは active LOW
    .intr_type = GPIO_INTR_NEGEDGE,    // 立ち下がりエッジで割り込み
};
gpio_config(&io_conf);

// ISRサービスをインストール
gpio_install_isr_service(0);

// ISRハンドラを追加
gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, tof_int_isr_handler, NULL);
```

### ISRハンドラ

```c
static void IRAM_ATTR tof_int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // セマフォをGive（ISRから）
    xSemaphoreGiveFromISR(measurement_semaphore, &xHigherPriorityTaskWoken);

    // 必要ならタスク切り替え
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
```

**重要**: ISRハンドラは`IRAM_ATTR`属性を付けてIRAMに配置します。これにより、フラッシュキャッシュが無効な状態でも実行可能になります。

### メインループ

```c
while (measurement_count < MEASUREMENT_COUNT) {
    // セマフォ待機（最大5秒タイムアウト）
    if (xSemaphoreTake(measurement_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
        // 測定データ取得
        VL53LX_GetMultiRangingData(&vl53lx_dev, &multi_ranging_data);

        // データ処理
        // ...

        // 次測定開始
        VL53LX_ClearInterruptAndStartMeasurement(&vl53lx_dev);
    } else {
        ESP_LOGW(TAG, "Timeout waiting for measurement interrupt");
    }
}
```

## 期待される出力

```
Stage 5: Interrupt Distance Measurement
VL53L3CX ToF Sensor
==================================
XSHUT pins initialized
Bottom ToF (GPIO7): ENABLED [DEFAULT - USB powered]
Front ToF (GPIO9): DISABLED (requires battery)
I2C master initialized successfully
SDA: GPIO3, SCL: GPIO4
INT pin initialized (GPIO6)
Initializing VL53L3CX sensor...
✓ Device booted
✓ Data initialized
✓ Product Type: 0xAA, Rev: 1.1
Using default measurement parameters (no configuration)
==================================
Starting distance measurements
Interrupt mode, 20 measurements
==================================
[01] Distance:  245 mm | Status: 0 | Signal: 15.32 Mcps
[02] Distance:  247 mm | Status: 0 | Signal: 15.28 Mcps
[03] Distance:  246 mm | Status: 0 | Signal: 15.30 Mcps
...
[20] Distance:  245 mm | Status: 0 | Signal: 15.31 Mcps
==================================
Measurements complete!
==================================
Test completed. Ready for Stage 6 (Dual sensor operation).
```

## ハードウェア接続

- **I2C SDA**: GPIO3
- **I2C SCL**: GPIO4
- **Bottom ToF XSHUT**: GPIO7 (HIGH: 有効) **[デフォルト - USB給電で動作]**
- **Bottom ToF INT**: GPIO6 (割り込みピン、active LOW)
- **Front ToF XSHUT**: GPIO9 (LOW: 無効、バッテリー必要)

### 重要：電源要件

**⚠️ センサー別の電源供給の違いに注意**

- **Bottom ToF (底面センサー)**: USB給電で動作 **[デフォルトテスト対象]**
  - 書き込み用USBケーブルからの電源で動作します
  - バッテリー不要でテスト可能

- **Front ToF (前面センサー)**: バッテリー電源が必要
  - USB給電のみでは動作しません
  - バッテリーを接続してください
  - バッテリー未接続時は、わずかな電圧でロジックが部分的に動作し、I2C通信は成功するが測定は失敗する現象が発生します

**推奨テスト手順:**
1. まず Bottom ToF (GPIO6 INT, GPIO7 XSHUT) でテスト（USB給電のみで動作）**← デフォルト設定**
2. Front ToF (GPIO8 INT, GPIO9 XSHUT) をテストする場合はバッテリーを接続

## ビルド方法

```bash
cd examples/stage5_interrupt_measurement
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## FreeRTOS セマフォについて

### バイナリセマフォ

```c
// セマフォ作成
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

// セマフォ待機（タスク側）
if (xSemaphoreTake(sem, timeout) == pdTRUE) {
    // セマフォ取得成功
}

// セマフォ通知（ISR側）
xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
```

### タスク通知との比較

Stage 5ではセマフォを使用していますが、タスク通知を使うこともできます：

```c
// タスク通知を使用する場合
static TaskHandle_t measurement_task_handle = NULL;

void IRAM_ATTR tof_int_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(measurement_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// メインループ
ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
```

タスク通知の方が軽量ですが、セマフォの方が汎用的です。

## パフォーマンス比較

| 項目 | ポーリング (Stage 4) | 割り込み (Stage 5) |
|------|---------------------|-------------------|
| CPU使用率 | 高 | 低 |
| 応答遅延 | ~1ms | <1μs |
| 消費電力 | 高 | 低 |
| コード複雑度 | 低 | 中 |
| リアルタイム性 | 中 | 高 |

## トラブルシューティング

### 割り込みが発生しない

```
Timeout waiting for measurement interrupt
```

確認事項:
1. INTピンの配線を確認
2. プルアップ抵抗が有効か確認
3. 割り込みエッジ設定（NEGEDGE）を確認
4. センサーが正常に動作しているか確認（Stage 4で確認）

### ISRがクラッシュする

```
Guru Meditation Error: Core 0 panic'ed (IllegalInstruction)
```

原因:
- ISRハンドラに`IRAM_ATTR`が付いていない
- ISRから呼び出している関数がIRAMにない

対処:
- 全てのISR関数に`IRAM_ATTR`を付ける
- ISRでは最小限の処理のみ行う

## 次のステップ

Stage 6 では、前方・底面の2つのセンサーを同時に使用します:
- I2Cアドレス変更シーケンス
- 複数センサー管理
- 個別割り込み処理
- センサー間の干渉回避
