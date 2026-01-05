# StampFly ToF Driver API Reference

VL53L3CX ToFセンサードライバのAPI仕様書です。

## 目次

- [プラットフォーム層API](#プラットフォーム層api)
- [VL53LX Core API](#vl53lx-core-api)
- [Kalman Filter API](#kalman-filter-api)
- [使用例](#使用例)

---

## プラットフォーム層API

ESP-IDF I2C抽象化レイヤーのAPI（`vl53lx_platform.h`）

### VL53LX_platform_init()

デバイス構造体を初期化し、I2Cデバイスハンドルを設定します。

```c
VL53LX_Error VL53LX_platform_init(
    VL53LX_Dev_t *pdev,
    i2c_master_bus_handle_t i2c_bus_handle
);
```

**パラメータ:**
- `pdev`: VL53LXデバイス構造体へのポインタ（I2cDevAddrは事前に設定すること）
- `i2c_bus_handle`: ESP-IDF I2Cバスハンドル

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功
- その他: エラーコード

**使用例:**
```c
VL53LX_Dev_t dev;
dev.I2cDevAddr = 0x29;  // I2Cアドレスを設定
VL53LX_platform_init(&dev, i2c_bus_handle);
```

### VL53LX_platform_deinit()

デバイスリソースを解放します。

```c
VL53LX_Error VL53LX_platform_deinit(VL53LX_Dev_t *pdev);
```

---

## VL53LX Core API

VL53L3CXセンサーの制御API（`vl53lx_api.h`）

### 初期化API

#### VL53LX_WaitDeviceBooted()

センサーのブート完了を待機します。

```c
VL53LX_Error VL53LX_WaitDeviceBooted(VL53LX_Dev_t *pdev);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ

**戻り値:**
- `VL53LX_ERROR_NONE`: ブート完了
- その他: タイムアウトまたはエラー

#### VL53LX_DataInit()

デバイスを初期化します。

```c
VL53LX_Error VL53LX_DataInit(VL53LX_Dev_t *pdev);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ

**戻り値:**
- `VL53LX_ERROR_NONE`: 初期化成功
- その他: エラーコード

#### VL53LX_GetDeviceInfo()

デバイス情報を取得します。

```c
VL53LX_Error VL53LX_GetDeviceInfo(
    VL53LX_Dev_t *pdev,
    VL53LX_DeviceInfo_t *pDeviceInfo
);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ
- `pDeviceInfo`: デバイス情報格納先

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

**デバイス情報構造体:**
```c
typedef struct {
    uint8_t ProductType;           // 0xAA = VL53L3CX
    uint8_t ProductRevisionMajor;  // メジャーバージョン
    uint8_t ProductRevisionMinor;  // マイナーバージョン
    // ...
} VL53LX_DeviceInfo_t;
```

### 測定設定API

#### VL53LX_SetDistanceMode()

距離測定モードを設定します。

```c
VL53LX_Error VL53LX_SetDistanceMode(
    VL53LX_Dev_t *pdev,
    VL53LX_DistanceModes DistanceMode
);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ
- `DistanceMode`: 距離モード
  - `VL53LX_DISTANCEMODE_SHORT`: 短距離モード（1.3m最適）
  - `VL53LX_DISTANCEMODE_MEDIUM`: 中距離モード（3m最適）**推奨**
  - `VL53LX_DISTANCEMODE_LONG`: 長距離モード（4m最適）

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

#### VL53LX_SetMeasurementTimingBudgetMicroSeconds()

測定タイミングバジェットを設定します。

```c
VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(
    VL53LX_Dev_t *pdev,
    uint32_t MeasurementTimingBudgetMicroSeconds
);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ
- `MeasurementTimingBudgetMicroSeconds`: タイミングバジェット（マイクロ秒）
  - 範囲: 8000〜500000 (8ms〜500ms)
  - 推奨: 33000 (33ms、約30Hz測定レート)

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

**タイミングバジェットと測定レートの関係:**
- 8ms: 最高速だが精度低下
- 33ms: バランス良好（推奨）
- 100ms: 高精度
- 500ms: 最高精度

### 測定制御API

#### VL53LX_StartMeasurement()

距離測定を開始します。

```c
VL53LX_Error VL53LX_StartMeasurement(VL53LX_Dev_t *pdev);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ

**戻り値:**
- `VL53LX_ERROR_NONE`: 測定開始成功

#### VL53LX_StopMeasurement()

距離測定を停止します。

```c
VL53LX_Error VL53LX_StopMeasurement(VL53LX_Dev_t *pdev);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ

**戻り値:**
- `VL53LX_ERROR_NONE`: 停止成功

### データ取得API

#### VL53LX_GetMeasurementDataReady()

測定データの準備状態を確認します（ポーリング用）。

```c
VL53LX_Error VL53LX_GetMeasurementDataReady(
    VL53LX_Dev_t *pdev,
    uint8_t *pMeasurementDataReady
);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ
- `pMeasurementDataReady`: データ準備状態（0=未準備、1=準備完了）

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

**使用例（ポーリング）:**
```c
uint8_t ready = 0;
while (!ready) {
    VL53LX_GetMeasurementDataReady(&dev, &ready);
    vTaskDelay(pdMS_TO_TICKS(1));
}
```

#### VL53LX_GetMultiRangingData()

測定データを取得します。

```c
VL53LX_Error VL53LX_GetMultiRangingData(
    VL53LX_Dev_t *pdev,
    VL53LX_MultiRangingData_t *pMultiRangingData
);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ
- `pMultiRangingData`: 測定データ格納先

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

**測定データ構造体:**
```c
typedef struct {
    uint8_t NumberOfObjectsFound;  // 検出オブジェクト数
    VL53LX_RangingMeasurementData_t RangeData[4];  // 測定データ配列
    // ...
} VL53LX_MultiRangingData_t;

typedef struct {
    uint16_t RangeMilliMeter;           // 距離（mm）
    uint8_t RangeStatus;                // 測定ステータス
    FixPoint1616_t SignalRateRtnMegaCps; // 信号強度
    // ...
} VL53LX_RangingMeasurementData_t;
```

**RangeStatus値:**
- `0`: 有効な測定
- `1`: Sigma fail (信号品質低下)
- `2`: Signal fail (信号不足)
- `4`: Out of range (測定範囲外)
- `7`: Wrap around (位相ラップアラウンド)

**SignalRateRtnMegaCpsの変換:**
```c
float signal_mcps = data.RangeData[0].SignalRateRtnMegaCps / 65536.0;
```

**使用例:**
```c
VL53LX_MultiRangingData_t data;
VL53LX_GetMultiRangingData(&dev, &data);

uint16_t distance = data.RangeData[0].RangeMilliMeter;
uint8_t status = data.RangeData[0].RangeStatus;
float signal = data.RangeData[0].SignalRateRtnMegaCps / 65536.0;

printf("Distance: %d mm, Status: %d, Signal: %.2f Mcps\n",
       distance, status, signal);
```

#### VL53LX_ClearInterruptAndStartMeasurement()

割り込みをクリアして次の測定を開始します。

```c
VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_Dev_t *pdev);
```

**パラメータ:**
- `pdev`: デバイス構造体へのポインタ

**戻り値:**
- `VL53LX_ERROR_NONE`: 成功

**使用方法:**
データ取得後、必ずこの関数を呼び出して次の測定を開始してください。

---

## Kalman Filter API

1Dカルマンフィルタによる外れ値除去API（`vl53lx_outlier_filter.h`）

### VL53LX_FilterInit()

デフォルト設定でフィルタを初期化します。

```c
bool VL53LX_FilterInit(vl53lx_filter_t *filter);
```

**パラメータ:**
- `filter`: フィルタ構造体へのポインタ

**戻り値:**
- `true`: 初期化成功
- `false`: パラメータエラー

**デフォルト設定:**
- Q (process_noise): 1.0
- R (measurement_noise): 4.0
- max_change_rate: 500mm/sample
- status_check: 有効

### VL53LX_FilterInitWithConfig()

カスタム設定でフィルタを初期化します。

```c
bool VL53LX_FilterInitWithConfig(
    vl53lx_filter_t *filter,
    const vl53lx_filter_config_t *config
);
```

**パラメータ:**
- `filter`: フィルタ構造体へのポインタ
- `config`: フィルタ設定

**フィルタ設定構造体:**
```c
typedef struct {
    bool enable_status_check;       // ステータス検証を有効化
    bool enable_rate_limit;         // 変化率リミッターを有効化
    uint16_t max_change_rate_mm;    // 最大変化率（mm/sample）
    uint8_t valid_status_mask;      // 有効なステータスのビットマスク
    float kalman_process_noise;     // プロセスノイズQ
    float kalman_measurement_noise; // 測定ノイズR
} vl53lx_filter_config_t;
```

**設定例:**
```c
vl53lx_filter_config_t config = VL53LX_FilterGetDefaultConfig();

// スムーズな出力（高度測定など）
config.kalman_process_noise = 0.5f;
config.max_change_rate_mm = 200;

// 高速応答（障害物検出など）
config.kalman_process_noise = 2.0f;
config.max_change_rate_mm = 1000;

VL53LX_FilterInitWithConfig(&filter, &config);
```

### VL53LX_FilterGetDefaultConfig()

デフォルト設定を取得します。

```c
vl53lx_filter_config_t VL53LX_FilterGetDefaultConfig(void);
```

**戻り値:**
- デフォルト設定構造体

### VL53LX_FilterUpdate()

新しい測定値をフィルタに入力し、フィルタリング後の値を取得します。

```c
bool VL53LX_FilterUpdate(
    vl53lx_filter_t *filter,
    uint16_t distance_mm,
    uint8_t range_status,
    uint16_t *output_mm
);
```

**パラメータ:**
- `filter`: フィルタ構造体へのポインタ
- `distance_mm`: 生の測定距離（mm）
- `range_status`: センサーのRange Status
- `output_mm`: フィルタリング後の距離出力先

**戻り値:**
- `true`: 出力が有効
- `false`: 初期化前（出力無効）

**動作モード:**
- 測定値が有効な場合: カルマンフィルタで更新
- 測定値が無効な場合: 予測のみ（prediction-only mode）

**使用例:**
```c
vl53lx_filter_t filter;
VL53LX_FilterInit(&filter);

// 測定ループ内
VL53LX_MultiRangingData_t data;
VL53LX_GetMultiRangingData(&dev, &data);

uint16_t raw_distance = data.RangeData[0].RangeMilliMeter;
uint8_t range_status = data.RangeData[0].RangeStatus;
uint16_t filtered_distance;

if (VL53LX_FilterUpdate(&filter, raw_distance, range_status, &filtered_distance)) {
    printf("Raw: %d mm, Filtered: %d mm\n", raw_distance, filtered_distance);
}
```

### VL53LX_FilterReset()

フィルタをリセットします（状態をクリア）。

```c
void VL53LX_FilterReset(vl53lx_filter_t *filter);
```

**パラメータ:**
- `filter`: フィルタ構造体へのポインタ

**使用例:**
環境が大きく変化した場合（ドローンの離陸/着陸など）にリセットします。

### VL53LX_FilterDeinit()

フィルタを終了します。

```c
void VL53LX_FilterDeinit(vl53lx_filter_t *filter);
```

**パラメータ:**
- `filter`: フィルタ構造体へのポインタ

### VL53LX_FilterIsValidRangeStatus()

Range Statusが有効かどうかを判定します。

```c
bool VL53LX_FilterIsValidRangeStatus(uint8_t range_status);
```

**パラメータ:**
- `range_status`: センサーのRange Status

**戻り値:**
- `true`: Status 0（有効）
- `false`: その他（無効）

---

## 使用例

### 基本的なポーリング測定

```c
#include "vl53lx_platform.h"
#include "vl53lx_api.h"
#include "stampfly_tof_config.h"

void simple_polling_measurement(void) {
    // 1. I2C初期化
    i2c_master_bus_handle_t i2c_bus;
    // ... I2Cバス初期化コード ...

    // 2. センサー電源ON
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. デバイス初期化
    VL53LX_Dev_t dev;
    dev.I2cDevAddr = 0x29;
    VL53LX_platform_init(&dev, i2c_bus);
    VL53LX_WaitDeviceBooted(&dev);
    VL53LX_DataInit(&dev);

    // 4. 測定設定
    VL53LX_SetDistanceMode(&dev, VL53LX_DISTANCEMODE_MEDIUM);
    VL53LX_SetMeasurementTimingBudgetMicroSeconds(&dev, 33000);

    // 5. 測定開始
    VL53LX_StartMeasurement(&dev);

    // 6. 測定ループ
    for (int i = 0; i < 10; i++) {
        uint8_t ready = 0;

        // データ準備待ち
        while (!ready) {
            VL53LX_GetMeasurementDataReady(&dev, &ready);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // データ取得
        VL53LX_MultiRangingData_t data;
        VL53LX_GetMultiRangingData(&dev, &data);

        uint16_t distance = data.RangeData[0].RangeMilliMeter;
        uint8_t status = data.RangeData[0].RangeStatus;

        printf("[%d] Distance: %d mm, Status: %d\n", i+1, distance, status);

        // 次の測定開始
        VL53LX_ClearInterruptAndStartMeasurement(&dev);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 7. 測定停止
    VL53LX_StopMeasurement(&dev);
}
```

### 割り込みベース測定

```c
#include "freertos/semphr.h"

static SemaphoreHandle_t semaphore;

static void IRAM_ATTR int_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void interrupt_measurement(void) {
    // セマフォ作成
    semaphore = xSemaphoreCreateBinary();

    // 割り込み設定
    gpio_set_intr_type(STAMPFLY_TOF_BOTTOM_INT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, int_isr_handler, NULL);

    // デバイス初期化（前述と同様）
    // ...

    VL53LX_StartMeasurement(&dev);

    // 測定ループ
    while (1) {
        // 割り込み待機
        if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            VL53LX_MultiRangingData_t data;
            VL53LX_GetMultiRangingData(&dev, &data);

            uint16_t distance = data.RangeData[0].RangeMilliMeter;
            printf("Distance: %d mm\n", distance);

            VL53LX_ClearInterruptAndStartMeasurement(&dev);
        }
    }
}
```

### カルマンフィルタ付き測定

```c
#include "vl53lx_outlier_filter.h"

void filtered_measurement(void) {
    // フィルタ初期化
    vl53lx_filter_t filter;
    VL53LX_FilterInit(&filter);

    // デバイス初期化・測定開始（前述と同様）
    // ...

    while (1) {
        // データ取得（割り込みまたはポーリング）
        VL53LX_MultiRangingData_t data;
        VL53LX_GetMultiRangingData(&dev, &data);

        uint16_t raw_distance = data.RangeData[0].RangeMilliMeter;
        uint8_t range_status = data.RangeData[0].RangeStatus;
        uint16_t filtered_distance;

        // フィルタ適用
        if (VL53LX_FilterUpdate(&filter, raw_distance, range_status, &filtered_distance)) {
            printf("Raw: %d mm, Filtered: %d mm\n", raw_distance, filtered_distance);
        }

        VL53LX_ClearInterruptAndStartMeasurement(&dev);
    }
}
```

---

## エラーコード

主要なエラーコード（`vl53lx_error_codes.h`）：

| コード | 値 | 説明 |
|-------|---|------|
| VL53LX_ERROR_NONE | 0 | 成功 |
| VL53LX_ERROR_TIME_OUT | -1 | タイムアウト |
| VL53LX_ERROR_CONTROL_INTERFACE | -3 | I2C通信エラー |
| VL53LX_ERROR_INVALID_PARAMS | -4 | 無効なパラメータ |
| VL53LX_ERROR_NOT_SUPPORTED | -8 | サポートされていない操作 |
| VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL | -18 | バッファサイズ不足 |

---

## 参考リンク

- [basic_polling サンプル](../examples/basic_polling/)
- [basic_interrupt サンプル](../examples/basic_interrupt/)
- [stage8_filtered_streaming サンプル](../examples/development/stage8_filtered_streaming/)
- [メインREADME](../README.md)
