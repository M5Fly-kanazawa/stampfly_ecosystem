# Basic Interrupt Measurement Example

シンプルな割り込み測定のサンプルプログラムです。

## 概要

底面ToFセンサーを使用して、割り込み方式で効率的に距離を測定します。1秒間隔で10回測定を行います。

## 機能

- 底面ToFセンサー（USB給電で動作）
- GPIO割り込み方式（CPU効率的）
- FreeRTOSセマフォ使用
- 測定回数: 10回
- 測定間隔: 1秒
- タイミングバジェット: 33ms
- 距離モード: MEDIUM (1-3m最適)

## ビルド＆実行

```bash
cd examples/basic_interrupt
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 期待される出力

```
I (xxx) BASIC_INTERRUPT: === Basic VL53L3CX Interrupt Example ===
I (xxx) BASIC_INTERRUPT: I2C initialized (SDA: GPIO3, SCL: GPIO4)
I (xxx) BASIC_INTERRUPT: Sensor power initialized (bottom sensor enabled)
I (xxx) BASIC_INTERRUPT: Interrupt initialized (GPIO6)
I (xxx) BASIC_INTERRUPT: VL53L3CX ready (Type: 0xAA, Rev: 1.1)
I (xxx) BASIC_INTERRUPT: Starting interrupt-based measurements...

[1] Distance:  245 mm, Status: 0, Signal: 15.32 Mcps
[2] Distance:  247 mm, Status: 0, Signal: 15.28 Mcps
[3] Distance:  246 mm, Status: 0, Signal: 15.30 Mcps
...
[10] Distance:  245 mm, Status: 0, Signal: 15.31 Mcps

I (xxx) BASIC_INTERRUPT: Measurements complete!
```

## ポーリング方式との違い

**ポーリング方式** ([basic_polling](../basic_polling/)):
- CPU が常にデータ準備を確認
- シンプルだが非効率

**割り込み方式** (このサンプル):
- データ準備時にGPIO割り込みが発生
- FreeRTOSセマフォで通知
- CPU効率的で低消費電力

## コードの理解

このサンプルは割り込みベースの測定を実装しています：

1. **セマフォ作成**: `xSemaphoreCreateBinary()`
2. **I2C初期化**: `init_i2c()`
3. **センサー電源ON**: `init_sensor_power()`
4. **割り込み設定**: `init_interrupt()` - GPIO6を立ち下がりエッジ検出
5. **センサー初期化**: `init_sensor()`
6. **測定開始**: `VL53LX_StartMeasurement()`
7. **割り込み待機**: `xSemaphoreTake(semaphore, timeout)` でデータ準備を待つ
8. **データ取得**: `VL53LX_GetMultiRangingData()`

### 割り込みの流れ

```
センサー測定完了 → GPIO6 LOW → ISR実行 → セマフォGive → タスク再開 → データ取得
```

## 自分のプロジェクトで使う

割り込みベースの測定を実装する場合：

```c
// 1. セマフォ作成
SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

// 2. 割り込み設定
static void IRAM_ATTR isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}
gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, isr_handler, NULL);

// 3. 測定開始
VL53LX_StartMeasurement(&tof_dev);

// 4. 割り込み駆動ループ
while (1) {
    if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
        VL53LX_MultiRangingData_t data;
        VL53LX_GetMultiRangingData(&tof_dev, &data);

        uint16_t distance = data.RangeData[0].RangeMilliMeter;
        // 距離を使った処理...

        VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
    }
}
```

## 次のステップ

- 連続測定やフィルタリングなど、より高度な例は [development/](../development/) にあります
- リアルタイム可視化には [development/stage7_teleplot_streaming](../development/stage7_teleplot_streaming/) をご覧ください
- Kalmanフィルタ実装は [development/stage8_filtered_streaming](../development/stage8_filtered_streaming/) を参照してください
