# Stage 4: VL53L3CX ポーリング距離測定

## 概要

このステージでは、ポーリング方式による基本的な距離測定を実装します。33msタイミングバジェットで、1秒間隔で20回の測定を行います。

## 測定シーケンス

```c
// 1. デバイス初期化
VL53LX_WaitDeviceBooted(&vl53lx_dev);
VL53LX_DataInit(&vl53lx_dev);

// 2. 測定パラメータ設定
VL53LX_SetDistanceMode(&vl53lx_dev, VL53LX_DISTANCEMODE_MEDIUM);
VL53LX_SetMeasurementTimingBudgetMicroSeconds(&vl53lx_dev, 33000);

// 3. 測定開始
VL53LX_StartMeasurement(&vl53lx_dev);

// 4. ポーリングループ
while (measurement_count < 20) {
    // データ準備完了を待機
    while (!data_ready) {
        VL53LX_GetMeasurementDataReady(&vl53lx_dev, &data_ready);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 測定データ取得
    VL53LX_GetMultiRangingData(&vl53lx_dev, &multi_ranging_data);

    // 距離表示
    if (multi_ranging_data.NumberOfObjectsFound > 0) {
        printf("Distance: %d mm\\n",
               multi_ranging_data.RangeData[0].RangeMilliMeter);
    }

    // 次測定開始
    VL53LX_ClearInterruptAndStartMeasurement(&vl53lx_dev);

    vTaskDelay(pdMS_TO_TICKS(1000));
}
```

## 期待される出力

```
Stage 4: Polling Distance Measurement
VL53L3CX ToF Sensor
==================================
XSHUT pins initialized
Bottom ToF (GPIO7): ENABLED [DEFAULT - USB powered]
Front ToF (GPIO9): DISABLED (requires battery)
I2C master initialized successfully
SDA: GPIO3, SCL: GPIO4
Initializing VL53L3CX sensor...
✓ Device booted
✓ Data initialized
✓ Product Type: 0xAA, Rev: 1.1
Configuring measurement parameters...
✓ Distance mode: MEDIUM
✓ Timing budget: 33 ms
==================================
Starting distance measurements
Polling mode, 20 measurements
==================================
[01] Distance:  245 mm | Status: 0 | Objects: 1
[02] Distance:  247 mm | Status: 0 | Objects: 1
[03] Distance:  246 mm | Status: 0 | Objects: 1
...
[20] Distance:  245 mm | Status: 0 | Objects: 1
==================================
Measurements complete!
==================================
```

## 測定パラメータ

### Distance Mode
- **MEDIUM**: 1-3m の範囲で最適
- その他のモード:
  - SHORT: 1.3m以下
  - LONG: 4m以上

### Timing Budget
- **33ms**: 最大30Hz測定レート
- より短い（8-15ms）: 低精度、高速
- より長い（50-100ms）: 高精度、低速

## VL53LX Multi-Ranging API

### VL53LX_MultiRangingData_t 構造体

```c
typedef struct {
    uint32_t TimeStamp;            // タイムスタンプ
    uint8_t StreamCount;           // ストリームカウント
    uint8_t NumberOfObjectsFound;  // 検出オブジェクト数 (0-4)
    VL53LX_TargetRangeData_t RangeData[VL53LX_MAX_RANGE_RESULTS];
    uint16_t EffectiveSpadRtnCount; // 有効SPAD数
} VL53LX_MultiRangingData_t;
```

### VL53LX_TargetRangeData_t 構造体

```c
typedef struct {
    int16_t RangeMilliMeter;  // 距離 (mm)
    uint8_t RangeStatus;      // ステータス (0=有効)
    uint8_t ExtendedRange;    // 拡張レンジフラグ
} VL53LX_TargetRangeData_t;
```

### Range Status コード

- **0**: 有効な測定
- **1**: シグマエラー
- **2**: シグナルエラー
- **4**: アウトオブレンジ
- **7**: ラップアラウンド
- **8**: タイムアウト

## ハードウェア接続

- **I2C SDA**: GPIO3
- **I2C SCL**: GPIO4
- **Bottom ToF XSHUT**: GPIO7 (HIGH: 有効) **[デフォルト - USB給電で動作]**
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
1. まず Bottom ToF (GPIO7) でテスト（USB給電のみで動作）**← デフォルト設定**
2. Front ToF (GPIO9) をテストする場合はバッテリーを接続

## ビルド方法

```bash
cd examples/stage4_polling_measurement
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## トラブルシューティング

### No objects detected

```
[01] No objects detected
```

原因:
1. センサーの視野内にオブジェクトがない
2. 距離が測定範囲外（MEDIUM: 1-3m）
3. XSHUT ピンの状態確認

### RangeStatus != 0

```
[01] Distance:  245 mm | Status: 2 | Objects: 1
```

Status 2 = シグナルエラー

対処:
- ターゲットの反射率を確認
- 照明条件を確認
- Distance Modeを変更

### 測定開始失敗

```
Start measurement failed (status: -1)
```

確認事項:
1. デバイス初期化が完了しているか
2. パラメータ設定が正しいか
3. I2C通信エラーがないか

## IPP (Image Processing Pipeline)

Stage 4では、IPPプラットフォーム層（`vl53lx_platform_ipp.c`）を追加しました:

- **VL53LX_ipp_hist_process_data()**: ヒストグラムデータ処理
- **VL53LX_ipp_hist_amb_dmax_calc()**: 環境光とDmax計算
- **VL53LX_ipp_xtalk_calibration_process_data()**: クロストークキャリブレーション

これらの関数は、VL53LX APIが測定データを処理する際に内部的に使用されます。

## 次のステップ

Stage 5 では、GPIO割り込みベースの測定を実装します:
- INT ピンの割り込み設定
- FreeRTOS セマフォによる同期
- より効率的なCPU使用
- 低消費電力動作
