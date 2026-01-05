# Stage 6: VL53L3CX 2センサー同時動作

## 概要

このステージでは、前方ToFと底面ToFの2つのセンサーを同時に使用する実装を示します。

## 重要：電源要件

**デフォルト設定**: 底面ToFのみ有効（USB給電で動作）

```c
#define ENABLE_FRONT_SENSOR  0  // 0: Bottom only (USB), 1: Both sensors (Battery required)
```

- **`ENABLE_FRONT_SENSOR = 0`**: 底面ToFのみ（USB給電のみで動作）**← デフォルト**
- **`ENABLE_FRONT_SENSOR = 1`**: 前方+底面ToF（バッテリー必要）

## I2Cアドレス変更シーケンス

同じI2Cバス上で2つのセンサーを使用するには、それぞれ異なるアドレスが必要です。

### 問題

両センサーのデフォルトI2Cアドレスは `0x29` です。

### 解決策

XSHUTピンを使用してセンサーを個別に制御し、I2Cアドレスを変更します。

### 手順

```
1. 両方のXSHUTをLOWにする
   → 両センサーがシャットダウン

2. 底面ToFのXSHUTをHIGHにする
   → 底面ToFが起動（アドレス: 0x29）

3. 底面ToFのI2Cアドレスを0x30に変更
   → VL53LX_SetDeviceAddress(&bottom_dev, 0x30 << 1)

4. 前方ToFのXSHUTをHIGHにする
   → 前方ToFが起動（アドレス: 0x29）

結果: 底面ToF=0x30, 前方ToF=0x29
```

### コード実装

```c
// Step 1: Shutdown both sensors
gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 0);
gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 0);
vTaskDelay(pdMS_TO_TICKS(10));

// Step 2: Enable bottom sensor (default 0x29)
gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);
vTaskDelay(pdMS_TO_TICKS(10));

// Step 3: Change bottom sensor address to 0x30
VL53LX_PlatformInit(&bottom_dev, i2c_bus_handle, 0x29);
VL53LX_SetDeviceAddress(&bottom_dev, 0x30 << 1);
bottom_dev.I2cDevAddr = 0x30;  // Update structure

// Step 4: Enable front sensor (default 0x29)
gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 1);
vTaskDelay(pdMS_TO_TICKS(10));
VL53LX_PlatformInit(&front_dev, i2c_bus_handle, 0x29);
```

**重要**: `VL53LX_SetDeviceAddress()`の引数は8ビットアドレス（7ビット << 1）です。

## 2つのデバイス構造体

```c
static VL53LX_Dev_t bottom_dev;  // 底面ToF (0x30)
static VL53LX_Dev_t front_dev;   // 前方ToF (0x29)
```

各センサーは独立した構造体を持ち、それぞれ異なるI2Cアドレスで通信します。

## 個別割り込み処理

```c
// 底面ToF: GPIO6
static void IRAM_ATTR bottom_int_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(bottom_semaphore, &xHigherPriorityTaskWoken);
}

// 前方ToF: GPIO8
static void IRAM_ATTR front_int_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(front_semaphore, &xHigherPriorityTaskWoken);
}
```

各センサーは独自の：
- セマフォ
- ISRハンドラ
- INTピン

を持ちます。

## 期待される出力（底面ToFのみ）

```
Stage 6: Dual Sensor Operation
VL53L3CX ToF Sensors
==================================
I2C master initialized successfully
SDA: GPIO3, SCL: GPIO4
Starting I2C address change sequence...
  1. Both sensors shutdown
  2. Bottom sensor enabled at default 0x29
  3. Bottom sensor address changed to 0x30
  4. Front sensor DISABLED (set ENABLE_FRONT_SENSOR=1 to enable)
I2C address change sequence complete
Bottom ToF: GPIO7 (0x30) [ENABLED - USB powered]
Front ToF: GPIO9 [DISABLED]
INT pins initialized
Bottom INT: GPIO6
Initializing BOTTOM sensor...
BOTTOM: ✓ Device booted
BOTTOM: ✓ Data initialized
BOTTOM: ✓ Product Type: 0xAA, Rev: 1.1
Using default measurement parameters
==================================
Starting dual sensor measurements
Interrupt mode, 20 measurements per sensor
Bottom sensor only (USB powered)
==================================
BOTTOM: Starting measurements...
BOTTOM [01]:  245 mm | Status: 0 | Signal: 15.32 Mcps
BOTTOM [02]:  247 mm | Status: 0 | Signal: 15.28 Mcps
...
BOTTOM [20]:  245 mm | Status: 0 | Signal: 15.31 Mcps
BOTTOM: Measurements complete!
==================================
All measurements complete!
==================================
```

## 期待される出力（両センサー有効時）

```c
#define ENABLE_FRONT_SENSOR  1  // 両方有効化
```

```
Stage 6: Dual Sensor Operation
VL53L3CX ToF Sensors
==================================
...
  4. Front sensor enabled at default 0x29
I2C address change sequence complete
Bottom ToF: GPIO7 (0x30) [ENABLED - USB powered]
Front ToF: GPIO9 (0x29) [ENABLED - Battery required]
INT pins initialized
Bottom INT: GPIO6
Front INT: GPIO8
Initializing BOTTOM sensor...
BOTTOM: ✓ Product Type: 0xAA, Rev: 1.1
Initializing FRONT sensor...
FRONT: ✓ Product Type: 0xAA, Rev: 1.1
==================================
Starting dual sensor measurements
Interrupt mode, 20 measurements per sensor
Both sensors active
==================================
BOTTOM: Starting measurements...
BOTTOM [01]:  245 mm | Status: 0 | Signal: 15.32 Mcps
...
BOTTOM: Measurements complete!
FRONT: Starting measurements...
FRONT [01]:  180 mm | Status: 0 | Signal: 18.45 Mcps
...
FRONT: Measurements complete!
==================================
```

## センサー構成

| センサー | XSHUT | INT | I2Cアドレス | 電源 |
|---------|-------|-----|-----------|------|
| 底面ToF | GPIO7 | GPIO6 | 0x30 | USB給電 |
| 前方ToF | GPIO9 | GPIO8 | 0x29 | バッテリー必要 |

## ビルド方法

```bash
cd examples/stage6_dual_sensor
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 前方ToFを有効にする方法

1. `main/main.c`を編集:
```c
#define ENABLE_FRONT_SENSOR  1  // 0→1に変更
```

2. **バッテリーを接続**

3. リビルド＆フラッシュ:
```bash
idf.py build flash monitor
```

## センサー間の干渉回避

### 空間的分離

StampFlyの物理配置により、前方ToFと底面ToFは異なる方向を向いているため、光学的な干渉は最小限です。

### 時間的分離（オプション）

さらに干渉を避けたい場合、測定を時間的に分離できます：

```c
// Option 1: Sequential measurement (current implementation)
measure_sensor(&bottom_dev, bottom_semaphore, "BOTTOM", 20);
vTaskDelay(pdMS_TO_TICKS(100));
measure_sensor(&front_dev, front_semaphore, "FRONT", 20);

// Option 2: Interleaved measurement
for (int i = 0; i < 20; i++) {
    measure_one(&bottom_dev, bottom_semaphore, "BOTTOM");
    vTaskDelay(pdMS_TO_TICKS(50));
    measure_one(&front_dev, front_semaphore, "FRONT");
    vTaskDelay(pdMS_TO_TICKS(50));
}

// Option 3: Simultaneous measurement (for advanced users)
// Use two FreeRTOS tasks with proper synchronization
```

## トラブルシューティング

### 両センサーが同じデータを返す

**原因**: I2Cアドレス変更が失敗している

**確認事項**:
1. アドレス変更シーケンスのログを確認
2. XSHUTピンの配線を確認
3. `VL53LX_SetDeviceAddress()`のステータスを確認

### 前方ToFの初期化が失敗する

**原因**: バッテリー未接続

**対処**:
1. バッテリーを接続
2. または `ENABLE_FRONT_SENSOR = 0` に設定

### I2C通信エラー

```
I2C read/write failed
```

**確認事項**:
1. I2Cアドレスが正しいか（0x29と0x30）
2. 両センサーのXSHUTが正しく設定されているか
3. I2Cバスのプルアップ抵抗

## パフォーマンス考察

### メモリ使用量

- 2つの`VL53LX_Dev_t`構造体（各約2KB）
- 2つのセマフォ
- 2つのISRハンドラ

合計: 追加約4-5KB

### CPU使用率

割り込みベースのため、両センサー使用時でもCPU使用率は低いままです。

## 実用的な使用例

### ドローン/ロボット

```c
// 底面ToF: 高度測定
float altitude = get_bottom_distance_mm() / 1000.0;  // meters

// 前方ToF: 障害物検出
float obstacle_distance = get_front_distance_mm() / 1000.0;  // meters

if (obstacle_distance < 0.5) {
    emergency_stop();
}
```

### データ構造化

```c
typedef struct {
    uint16_t distance_mm;
    uint8_t status;
    float signal_mcps;
    uint32_t timestamp_ms;
} tof_measurement_t;

tof_measurement_t bottom_data;
tof_measurement_t front_data;
```

## 次のステップ

Stage 6で2センサー管理の基本を習得しました。さらに発展させるには：

1. **リアルタイムタスク**: 2つのFreeRTOSタスクで同時測定
2. **データフュージョン**: 複数センサーデータの統合
3. **キャリブレーション**: センサー間のオフセット補正
4. **ロギング**: SDカードやFlashへのデータ記録

## まとめ

Stage 6では：
- ✅ I2Cアドレス変更シーケンス
- ✅ 2つのセンサーの個別管理
- ✅ 個別割り込み処理
- ✅ USB給電/バッテリー切り替え可能な設計

これでStampFly ToFドライバの完全な実装が完了しました！
