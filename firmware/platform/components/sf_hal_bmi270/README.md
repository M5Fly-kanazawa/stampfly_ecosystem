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

# BMI270 Driver for ESP-IDF (ESP32-S3)

M5StampFly向けのBMI270 6軸IMU（ジャイロスコープ+加速度計）ドライバです。ESP-IDF v5.4.1ベースで開発されています。

## 特徴

- ✅ **簡単に使える**: 3つのシンプルなサンプルコード（ポーリング、割り込み、FIFO）
- ✅ **高性能**: FIFO使用時は1600Hz連続取得可能
- ✅ **低消費電力**: 割り込み駆動でCPU使用率最小化
- ✅ **実用的**: そのまま実アプリケーションに組み込み可能
- ✅ **完全ドキュメント化**: 日本語API仕様書と詳細サンプル

## クイックスタート

### 1. プロジェクトに組み込む

```bash
cd <your-esp-idf-project>/components/
git clone <this-repo-url> bmi270_driver
cd ..
idf.py build
```

### 2. サンプルを試す

```bash
# 環境セットアップ
source ~/esp/esp-idf/export.sh

# 最もシンプルなポーリングサンプルを実行
cd examples/basic_polling
idf.py set-target esp32s3
idf.py build flash monitor
```

## ハードウェア要件

### M5StampFly (推奨)

このドライバはM5StampFly向けに最適化されています：

| 信号 | ESP32-S3 GPIO | BMI270ピン |
|------|---------------|------------|
| MOSI | GPIO14 | SDx (SDI) |
| MISO | GPIO43 | SDO |
| SCK  | GPIO44 | SCx |
| CS   | GPIO46 | CSB |
| INT1 | GPIO11 | INT1 (オプション) |

### 他のESP32-S3ボード

ピン番号を変更すれば、任意のESP32-S3ボードで使用可能です：

```c
bmi270_config_t config = {
    .gpio_mosi = YOUR_MOSI_PIN,
    .gpio_miso = YOUR_MISO_PIN,
    .gpio_sclk = YOUR_SCLK_PIN,
    .gpio_cs = YOUR_CS_PIN,
    .spi_clock_hz = 10000000,
    .spi_host = SPI2_HOST,
    .gpio_other_cs = -1,  // 共有SPIバスがない場合
};
```

## サンプルコード

3つのレベルに応じたサンプルを用意しています：

### 1. Polling - ポーリング（最もシンプル）

```bash
cd examples/basic_polling
idf.py build flash monitor
```

**用途**: 学習、プロトタイピング、低速アプリケーション（〜200Hz）

**特徴**:
- コードが短く理解しやすい
- 実装が簡単
- CPU使用率は高め

[詳細 →](examples/basic_polling/README.md)

### 2. Interrupt - 割り込み（効率的）

```bash
cd examples/basic_interrupt
idf.py build flash monitor
```

**用途**: バッテリー駆動、省電力アプリケーション

**特徴**:
- 低CPU使用率
- データがある時だけ動作
- 即応性が高い

[詳細 →](examples/basic_interrupt/README.md)

### 3. FIFO - バッファ（高速・最高効率）

```bash
cd examples/basic_fifo
idf.py build flash monitor
```

**用途**: 高速モーショントラッキング、ドローン制御

**特徴**:
- 1600Hz連続取得可能
- CPU使用率最小
- データロス最小
- バッチ処理で効率的

[詳細 →](examples/basic_fifo/README.md)

## 基本的な使い方

### 最小サンプルコード

```c
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

void app_main(void)
{
    // デバイス初期化
    bmi270_dev_t dev = {0};
    bmi270_config_t config = {
        .gpio_mosi = 14,
        .gpio_miso = 43,
        .gpio_sclk = 44,
        .gpio_cs = 46,
        .spi_clock_hz = 10000000,
        .spi_host = SPI2_HOST,
        .gpio_other_cs = 12,  // M5StampFlyの場合（PMW3901共有SPI対策）
    };

    bmi270_spi_init(&dev, &config);
    bmi270_init(&dev);

    // センサー設定（100Hz, ±4g, ±1000°/s）
    bmi270_set_accel_config(&dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    bmi270_set_gyro_config(&dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);

    // データ読み取り
    while (1) {
        bmi270_gyro_t gyro;
        bmi270_accel_t accel;
        float temperature;

        bmi270_read_gyro_accel(&dev, &gyro, &accel);
        bmi270_read_temperature(&dev, &temperature);

        printf("Gyro: X=%.3f Y=%.3f Z=%.3f [rad/s]\n", gyro.x, gyro.y, gyro.z);
        printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n", accel.x, accel.y, accel.z);
        printf("Temp: %.2f [°C]\n", temperature);

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }
}
```

## API仕様

### 初期化

```c
// SPI通信初期化
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config);

// センサー初期化（コンフィグファイルアップロード、ACC/GYR有効化）
esp_err_t bmi270_init(bmi270_dev_t *dev);
```

### センサー設定

```c
// 加速度計設定
esp_err_t bmi270_set_accel_config(bmi270_dev_t *dev, bmi270_acc_odr_t odr, bmi270_filter_perf_t filter);

// ジャイロスコープ設定
esp_err_t bmi270_set_gyro_config(bmi270_dev_t *dev, bmi270_gyr_odr_t odr, bmi270_filter_perf_t filter);

// レンジ設定
esp_err_t bmi270_set_accel_range(bmi270_dev_t *dev, bmi270_acc_range_t range);
esp_err_t bmi270_set_gyro_range(bmi270_dev_t *dev, bmi270_gyr_range_t range);
```

### データ読み取り

```c
// ジャイロ（rad/s）＋加速度を一度に読み取り（推奨）
esp_err_t bmi270_read_gyro_accel(bmi270_dev_t *dev, bmi270_gyro_t *gyro, bmi270_accel_t *accel);

// 個別に読み取り（ジャイロはrad/s単位）
esp_err_t bmi270_read_gyro(bmi270_dev_t *dev, bmi270_gyro_t *data);
esp_err_t bmi270_read_accel(bmi270_dev_t *dev, bmi270_accel_t *data);

// ジャイロをdps単位で読み取り
esp_err_t bmi270_read_gyro_dps(bmi270_dev_t *dev, bmi270_gyro_t *data);
esp_err_t bmi270_read_gyro_accel_dps(bmi270_dev_t *dev, bmi270_gyro_t *gyro, bmi270_accel_t *accel);

// 温度センサー読み取り
esp_err_t bmi270_read_temperature(bmi270_dev_t *dev, float *temperature);

// 単位変換ユーティリティ
float bmi270_rad_to_dps(float rad_per_sec);
float bmi270_dps_to_rad(float deg_per_sec);
```

### 低レベルAPI

```c
// レジスタアクセス
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);

// バースト転送
esp_err_t bmi270_read_burst(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t length);
esp_err_t bmi270_write_burst(bmi270_dev_t *dev, uint8_t reg_addr, const uint8_t *data, uint16_t length);
```

詳細なAPI仕様は[docs/API.md](docs/API.md)を参照してください。

## ディレクトリ構造

```
stampfly_imu/
├── README.md                    # このファイル
├── docs/                        # ドキュメント
│   ├── API.md                  # API仕様書
│   ├── M5StamFly_spec_ja.md    # ハードウェア仕様
│   └── ...
├── components/bmi270_driver/    # ドライバ本体
│   ├── include/                # 公開ヘッダー
│   │   ├── bmi270_spi.h       # SPI通信API
│   │   ├── bmi270_init.h      # 初期化API
│   │   ├── bmi270_data.h      # データ読み取りAPI
│   │   └── ...
│   └── src/                    # 実装
└── examples/                    # サンプルコード
    ├── basic_polling/           # ポーリングサンプル
    ├── basic_interrupt/         # 割り込みサンプル
    ├── basic_fifo/              # FIFOサンプル
    └── development/             # 開発過程（学習用）
        ├── stage1_spi_basic/   # SPI基本通信
        ├── stage2_init/        # センサー初期化
        ├── stage3_polling/     # ポーリング読み取り
        ├── stage4_interrupt/   # 割り込み読み取り
        └── stage5_fifo/        # FIFO読み取り
```

## 開発過程を学ぶ

BMI270ドライバの開発過程を段階的に学びたい場合は、[`examples/development/`](examples/development/README.md)配下のステージ別サンプルを参照してください：

1. **Stage 1**: SPI基本通信（CHIP_ID読み取り）
2. **Stage 2**: センサー初期化（8KBコンフィグアップロード）
3. **Stage 3**: ポーリング読み取り
4. **Stage 4**: 割り込み読み取り
5. **Stage 5**: FIFO読み取り（4ステップ）

## よくある質問

### Q: サンプリング周波数を変更するには？

A: `bmi270_set_*_config()`でODRを設定します：

```c
// 200Hzに設定
bmi270_set_accel_config(&dev, BMI270_ACC_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
bmi270_set_gyro_config(&dev, BMI270_GYR_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
```

対応ODR: 25Hz, 50Hz, 100Hz, 200Hz, 400Hz, 800Hz, 1600Hz, 3200Hz

### Q: 測定レンジを変更するには？

A: 初期化後に`bmi270_set_*_range()`を使用：

```c
bmi270_set_accel_range(&dev, BMI270_ACC_RANGE_8G);   // ±8g
bmi270_set_gyro_range(&dev, BMI270_GYR_RANGE_2000);  // ±2000°/s
```

### Q: M5StampFly以外で使えますか？

A: はい。ESP32-S3であれば任意のボードで使用可能です。ピン番号を環境に合わせて変更してください。

### Q: 割り込みピン（INT1）は必須ですか？

A: いいえ。ポーリングサンプルは割り込みピンなしで動作します。高効率が必要な場合のみ使用してください。

## トラブルシューティング

### CHIP_IDが読めない (0x00が返る)

**原因**:
- SPI配線不良
- 電源供給不足
- 共有SPIバスの他デバイスCS未管理

**解決策**:
1. MISO配線を確認（最も多いミス）
2. 3.3V電源を確認
3. M5StampFlyの場合、`gpio_other_cs = 12`を設定

### 通信が不安定

**原因**: 共有SPIバスの他デバイスCSが浮いている

**解決策**:
```c
// M5StampFlyの場合（PMW3901と共有）
config.gpio_other_cs = 12;  // PMW3901 CSを管理
```

### データが更新されない

**原因**: センサーがスリープモード

**解決策**:
```c
// 必ず初期化してからデータ読み取り
bmi270_init(&dev);  // ACC/GYRを自動的に有効化
```

詳細は各サンプルのREADMEを参照してください。

## ライセンス

このプロジェクトのライセンスは未定です。

## 参考資料

- [BMI270 Datasheet (Bosch Sensortec)](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/)
- [M5StampFly Documentation](https://docs.m5stack.com/)

## 貢献

バグ報告、機能要望、プルリクエストは歓迎します。

## 開発者

このプロジェクトは**Claude Code** (Anthropic)との協働により開発されています。

**Generated with [Claude Code](https://claude.com/claude-code)**
