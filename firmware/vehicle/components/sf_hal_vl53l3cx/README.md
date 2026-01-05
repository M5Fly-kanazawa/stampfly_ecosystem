# StampFly ToF Sensor Driver (VL53L3CX)

ESP32-S3ベースのM5Stamp Fly用VL53L3CX Time-of-Flight (ToF)距離センサードライバ

## 概要

このプロジェクトは、StampFly（M5Stamp S3搭載クアッドコプター）に搭載されたVL53L3CX ToFセンサーを使用するためのESP-IDFコンポーネントとサンプルコードを提供します。

### 主な特徴

- ESP-IDF向けに最適化されたコンポーネント構成
- シンプルで分かりやすいサンプルコード（ポーリング/割り込み）
- 距離測定に必要な最小限のAPIのみを抽出
- 1Dカルマンフィルタによる外れ値除去
- タイミングバジェット33ms対応（約30Hz測定レート）
- 2センサー同時使用対応（前方/底面）

## ハードウェア仕様

### StampFly ToFセンサー配置

| センサー | XSHUT | INT | I2Cアドレス（デフォルト） |
|---------|-------|-----|------------------------|
| 前方ToF | GPIO9 | GPIO8 | 0x29 (7-bit) |
| 底面ToF | GPIO7 | GPIO6 | 0x29 (7-bit) → 0x30に変更 |

### 電源要件

**⚠️ 重要：センサー別の電源供給の違い**

- **底面ToF (Bottom ToF)**: USB給電で動作 **[デフォルトテスト対象]**
  - 書き込み用USBケーブルからの電源で動作します
  - バッテリー不要でテスト可能

- **前方ToF (Front ToF)**: バッテリー電源が必要
  - USB給電のみでは測定が動作しません
  - バッテリー未接続時、わずかな電圧でロジックが部分的に動作し、I2C通信は成功するが測定は失敗します

**推奨テスト手順:**
1. まず底面ToF (GPIO7) でテスト（USB給電のみで動作）
2. 前方ToF (GPIO9) をテストする場合はバッテリーを接続

### I2C接続

- SDA: GPIO3
- SCL: GPIO4
- 周波数: 400kHz（推奨）

## プロジェクト構成

```
stampfly_tof/
├── CMakeLists.txt              # コンポーネント定義
├── Kconfig                     # 設定オプション
├── include/                    # ヘッダーファイル
│   ├── stampfly_tof_config.h   # 設定ヘッダー
│   ├── vl53lx_outlier_filter.h # 1Dカルマンフィルタ
│   └── vl53lx/                 # VL53LX公式ヘッダー
├── src/                        # ソースファイル
│   ├── vl53lx_platform.c       # プラットフォーム層（ESP-IDF I2C抽象化）
│   ├── vl53lx_outlier_filter.c # 1Dカルマンフィルタ実装
│   └── vl53lx/                 # VL53LXコアドライバ（ST BareDriver 1.2.14）
├── examples/                   # サンプルプロジェクト
│   ├── basic_polling/          # ⭐ 基本ポーリング測定（初心者向け）
│   ├── basic_interrupt/        # ⭐ 基本割り込み測定（初心者向け）
│   └── development/            # 開発用詳細サンプル
│       ├── stage1_i2c_scan/    # I2Cバススキャン
│       ├── stage2_register_test/ # レジスタ読み書き
│       ├── stage3_device_init/ # デバイス初期化
│       ├── stage4_polling_measurement/ # ポーリング測定
│       ├── stage5_interrupt_measurement/ # 割り込み測定
│       ├── stage6_dual_sensor/ # 2センサー統合
│       ├── stage7_teleplot_streaming/ # Teleplotストリーミング
│       └── stage8_filtered_streaming/ # カルマンフィルタ付きストリーミング
├── docs/                       # メーカードキュメント
└── README.md                   # このファイル
```

## クイックスタート

### 1. サンプルを試す（推奨）

まずはシンプルなサンプルから始めましょう：

```bash
# 基本ポーリング測定
cd examples/basic_polling
idf.py set-target esp32s3
idf.py build flash monitor

# または、基本割り込み測定（より効率的）
cd examples/basic_interrupt
idf.py set-target esp32s3
idf.py build flash monitor
```

各サンプルの詳細は [examples/README.md](examples/README.md) を参照してください。

### 2. 自分のプロジェクトで使用する

このフォルダをあなたのESP-IDFプロジェクトの `components/` ディレクトリにコピーします。

```bash
# プロジェクトを作成
idf.py create-project my_tof_project
cd my_tof_project

# stampfly_tofコンポーネントをコピー
cp -r /path/to/stampfly_tof components/
```

`main/CMakeLists.txt`でstampfly_tofコンポーネントを要求：

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES stampfly_tof  # <- 追加
)
```

`main/main.c`で距離測定を実装：

```c
#include "vl53lx_platform.h"
#include "vl53lx_api.h"
#include "stampfly_tof_config.h"

void app_main(void) {
    // 1. I2C初期化
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_config, &i2c_bus);

    // 2. センサー電源ON（底面ToF）
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
    while (1) {
        uint8_t ready = 0;
        VL53LX_GetMeasurementDataReady(&dev, &ready);
        if (ready) {
            VL53LX_MultiRangingData_t data;
            VL53LX_GetMultiRangingData(&dev, &data);

            printf("Distance: %d mm\n", data.RangeData[0].RangeMilliMeter);

            VL53LX_ClearInterruptAndStartMeasurement(&dev);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

完全なコード例は [examples/basic_polling](examples/basic_polling/) または [examples/basic_interrupt](examples/basic_interrupt/) を参照してください。

ビルド＆実行：

```bash
idf.py build flash monitor
```

## サンプルプログラム

### 初心者向けサンプル

まずはこちらから始めてください：

- [basic_polling](examples/basic_polling/) - シンプルなポーリング測定
- [basic_interrupt](examples/basic_interrupt/) - 割り込みベース測定（より効率的）

### 開発用詳細サンプル

より詳細な実装や高度な機能を知りたい場合：

- [development/stage1-8](examples/development/) - 段階的な学習用サンプル
  - Stage 1: I2Cバススキャン
  - Stage 2: レジスタ読み書き
  - Stage 3: デバイス初期化
  - Stage 4: ポーリング測定（詳細版）
  - Stage 5: 割り込み測定（詳細版）
  - Stage 6: 2センサー同時使用
  - Stage 7: Teleplotリアルタイム可視化
  - Stage 8: カルマンフィルタ付きストリーミング

詳細は [examples/development/README.md](examples/development/README.md) を参照してください。

## 設定オプション

`idf.py menuconfig` から設定を変更できます。

```
Component config → StampFly ToF Sensor Configuration
```

主な設定項目：
- I2C SDA/SCL GPIO番号
- I2C周波数
- ToFセンサーXSHUT/INT GPIO番号
- タイミングバジェット（8～500ms）

## 主要機能

- ✅ ESP-IDF I2C master API対応
- ✅ VL53LX BareDriver 1.2.14統合
- ✅ シンプルなサンプルコード（ポーリング/割り込み）
- ✅ 2センサー同時使用対応
- ✅ 1Dカルマンフィルタ（外れ値除去）
- ✅ Teleplotリアルタイム可視化対応
- ✅ 詳細な開発用ステージサンプル（Stage 1-8）

## API仕様

詳細なAPI仕様は [docs/API.md](docs/API.md) を参照してください。

主要なAPI：
- **プラットフォーム層**: `VL53LX_platform_init()`, `VL53LX_platform_deinit()`
- **初期化**: `VL53LX_WaitDeviceBooted()`, `VL53LX_DataInit()`, `VL53LX_GetDeviceInfo()`
- **測定設定**: `VL53LX_SetDistanceMode()`, `VL53LX_SetMeasurementTimingBudgetMicroSeconds()`
- **測定制御**: `VL53LX_StartMeasurement()`, `VL53LX_StopMeasurement()`
- **データ取得**: `VL53LX_GetMeasurementDataReady()`, `VL53LX_GetMultiRangingData()`, `VL53LX_ClearInterruptAndStartMeasurement()`
- **カルマンフィルタ**: `VL53LX_FilterInit()`, `VL53LX_FilterUpdate()`, `VL53LX_FilterReset()`

## 参考ドキュメント

- **[API Reference](docs/API.md)** - API仕様書（関数リファレンス）
- [VL53L3CX Driver Documentation](docs/VL53L3CX_driver_doc.md) - ドライバ概要
- [M5StamFly Specifications](docs/M5StamFly_spec_ja.md) - ハードウェア仕様
- [VL53L3CX BareDriver (Manufacturer)](docs/VL53L3CX_BareDriver_1.2.14/) - STメーカードライバ

## ライセンス

このプロジェクトのオリジナル部分はMITライセンスです。
VL53L3CX Bare Driverは STMicroelectronics のライセンスに従います（GPL-2.0+ OR BSD-3-Clause）。

## 貢献

バグ報告や機能追加の提案は Issues からお願いします。
