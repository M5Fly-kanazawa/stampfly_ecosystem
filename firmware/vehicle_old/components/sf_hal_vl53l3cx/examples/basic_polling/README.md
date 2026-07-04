# Basic Polling Measurement Example

シンプルなポーリング測定のサンプルプログラムです。

## 概要

底面ToFセンサーを使用して、ポーリング方式で距離を測定します。1秒間隔で10回測定を行います。

## 機能

- 底面ToFセンサー（USB給電で動作）
- ポーリング方式（割り込み不使用）
- 測定回数: 10回
- 測定間隔: 1秒
- タイミングバジェット: 33ms
- 距離モード: MEDIUM (1-3m最適)

## ビルド＆実行

```bash
cd examples/basic_polling
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 期待される出力

```
I (xxx) BASIC_POLLING: === Basic VL53L3CX Polling Example ===
I (xxx) BASIC_POLLING: I2C initialized (SDA: GPIO3, SCL: GPIO4)
I (xxx) BASIC_POLLING: Sensor power initialized (bottom sensor enabled)
I (xxx) BASIC_POLLING: VL53L3CX ready (Type: 0xAA, Rev: 1.1)
I (xxx) BASIC_POLLING: Starting measurements...

[1] Distance:  245 mm, Status: 0, Signal: 15.32 Mcps
[2] Distance:  247 mm, Status: 0, Signal: 15.28 Mcps
[3] Distance:  246 mm, Status: 0, Signal: 15.30 Mcps
...
[10] Distance:  245 mm, Status: 0, Signal: 15.31 Mcps

I (xxx) BASIC_POLLING: Measurements complete!
```

## コードの理解

このサンプルは最小限のコードで距離測定を実装しています：

1. **I2C初期化**: `init_i2c()`
2. **センサー電源ON**: `init_sensor_power()`
3. **センサー初期化**: `init_sensor()`
4. **測定設定**: `VL53LX_SetDistanceMode()`, `VL53LX_SetMeasurementTimingBudgetMicroSeconds()`
5. **測定開始**: `VL53LX_StartMeasurement()`
6. **ポーリングループ**: `VL53LX_GetMeasurementDataReady()` でデータ準備を確認
7. **データ取得**: `VL53LX_GetMultiRangingData()`

## 自分のプロジェクトで使う

このサンプルをベースに、自分のプロジェクトで距離測定を実装できます：

```c
// 1. I2C初期化
init_i2c();

// 2. センサー電源ON
init_sensor_power();

// 3. センサー初期化
VL53LX_platform_init(&tof_dev, i2c_bus_handle);
init_sensor();

// 4. 測定ループ
VL53LX_StartMeasurement(&tof_dev);
while (1) {
    uint8_t ready = 0;
    VL53LX_GetMeasurementDataReady(&tof_dev, &ready);
    if (ready) {
        VL53LX_MultiRangingData_t data;
        VL53LX_GetMultiRangingData(&tof_dev, &data);

        uint16_t distance = data.RangeData[0].RangeMilliMeter;
        // 距離を使った処理...

        VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
    }
}
```

## 次のステップ

- より効率的な測定には [basic_interrupt](../basic_interrupt/) をご覧ください
- 開発用の詳細なサンプルは [development/](../development/) にあります
