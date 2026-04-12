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

# BMI270 Driver API仕様書

BMI270 SPI Driverの完全なAPI仕様書です。

## 目次

- [初期化API](#初期化api)
- [センサー設定API](#センサー設定api)
- [データ読み取りAPI](#データ読み取りapi)
- [低レベルAPI](#低レベルapi)
- [型定義](#型定義)
- [定数](#定数)

---

## 初期化API

### `bmi270_spi_init()`

SPI通信を初期化し、BMI270との接続を確立します。

```c
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ（出力）
- `config`: SPI設定構造体ポインタ

**戻り値**:
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 無効な引数
- `ESP_FAIL`: SPI初期化失敗

**説明**:
- SPI2_HOSTを使用してSPI通信を初期化
- CHIP_ID (0x24)を読み取って接続確認
- 共有SPIバスの他デバイスCS管理（`gpio_other_cs`が設定されている場合）

**使用例**:
```c
bmi270_dev_t dev = {0};
bmi270_config_t config = {
    .gpio_mosi = 14,
    .gpio_miso = 43,
    .gpio_sclk = 44,
    .gpio_cs = 46,
    .spi_clock_hz = 10000000,  // 10 MHz
    .spi_host = SPI2_HOST,
    .gpio_other_cs = 12,  // PMW3901 CS (M5StampFly)
};

esp_err_t ret = bmi270_spi_init(&dev, &config);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI init failed");
}
```

---

### `bmi270_init()`

BMI270センサーを初期化します（コンフィグアップロード、ACC/GYR有効化）。

```c
esp_err_t bmi270_init(bmi270_dev_t *dev);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ

**戻り値**:
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_STATE`: 初期化失敗
- `ESP_ERR_TIMEOUT`: タイムアウト

**説明**:
- ソフトリセット実行
- 8KBコンフィグファイルをアップロード
- INTERNAL_STATUSポーリング（最大20ms）
- 初期化完了確認（message=0x01）
- ACC/GYR有効化（PWR_CTRL=0x0E）
- 動作モード切り替え（2µs遅延）

**使用例**:
```c
esp_err_t ret = bmi270_init(&dev);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BMI270 init failed");
}
ESP_LOGI(TAG, "BMI270 initialized (CHIP_ID: 0x%02X)", dev.chip_id);
```

**注意事項**:
- `bmi270_spi_init()`の後に呼び出すこと
- 初期化には約150ms必要
- 初期化後はACC/GYRが自動的に有効化される

---

## センサー設定API

### `bmi270_set_accel_config()`

加速度計の出力データレート（ODR）とフィルタ性能を設定します。

```c
esp_err_t bmi270_set_accel_config(bmi270_dev_t *dev,
                                   bmi270_acc_odr_t odr,
                                   bmi270_filter_perf_t filter);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `odr`: 出力データレート
- `filter`: フィルタ性能モード

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ書き込み失敗

**使用例**:
```c
// 100Hz, 高性能フィルタ
bmi270_set_accel_config(&dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);

// 1600Hz, 高性能フィルタ
bmi270_set_accel_config(&dev, BMI270_ACC_ODR_1600HZ, BMI270_FILTER_PERFORMANCE);
```

**ODR選択肢**:
| 定数 | 周波数 | 用途 |
|------|--------|------|
| `BMI270_ACC_ODR_25HZ` | 25 Hz | 低消費電力 |
| `BMI270_ACC_ODR_50HZ` | 50 Hz | 低消費電力 |
| `BMI270_ACC_ODR_100HZ` | 100 Hz | 標準（推奨） |
| `BMI270_ACC_ODR_200HZ` | 200 Hz | 高速 |
| `BMI270_ACC_ODR_400HZ` | 400 Hz | 高速 |
| `BMI270_ACC_ODR_800HZ` | 800 Hz | 非常に高速 |
| `BMI270_ACC_ODR_1600HZ` | 1600 Hz | 最高速 |
| `BMI270_ACC_ODR_3200HZ` | 3200 Hz | 最高速 |

**フィルタ選択肢**:
- `BMI270_FILTER_PERFORMANCE`: 高性能（推奨）
- `BMI270_FILTER_POWER`: 低消費電力

---

### `bmi270_set_gyro_config()`

ジャイロスコープの出力データレート（ODR）とフィルタ性能を設定します。

```c
esp_err_t bmi270_set_gyro_config(bmi270_dev_t *dev,
                                  bmi270_gyr_odr_t odr,
                                  bmi270_filter_perf_t filter);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `odr`: 出力データレート
- `filter`: フィルタ性能モード

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ書き込み失敗

**使用例**:
```c
// 100Hz, 高性能フィルタ
bmi270_set_gyro_config(&dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
```

**ODR選択肢**:
| 定数 | 周波数 | 用途 |
|------|--------|------|
| `BMI270_GYR_ODR_25HZ` | 25 Hz | 低消費電力 |
| `BMI270_GYR_ODR_50HZ` | 50 Hz | 低消費電力 |
| `BMI270_GYR_ODR_100HZ` | 100 Hz | 標準（推奨） |
| `BMI270_GYR_ODR_200HZ` | 200 Hz | 高速 |
| `BMI270_GYR_ODR_400HZ` | 400 Hz | 高速 |
| `BMI270_GYR_ODR_800HZ` | 800 Hz | 非常に高速 |
| `BMI270_GYR_ODR_1600HZ` | 1600 Hz | 最高速 |
| `BMI270_GYR_ODR_3200HZ` | 3200 Hz | 最高速 |

---

### `bmi270_set_accel_range()`

加速度計の測定レンジを設定します。

```c
esp_err_t bmi270_set_accel_range(bmi270_dev_t *dev, bmi270_acc_range_t range);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `range`: 測定レンジ

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ書き込み失敗

**使用例**:
```c
// ±8gに設定
bmi270_set_accel_range(&dev, BMI270_ACC_RANGE_8G);
```

**レンジ選択肢**:
| 定数 | レンジ | 感度 | 用途 |
|------|--------|------|------|
| `BMI270_ACC_RANGE_2G` | ±2g | 最高 | 静的・低加速度 |
| `BMI270_ACC_RANGE_4G` | ±4g | 高 | 標準（デフォルト） |
| `BMI270_ACC_RANGE_8G` | ±8g | 中 | 動的動作 |
| `BMI270_ACC_RANGE_16G` | ±16g | 低 | 衝撃・高加速度 |

**デフォルト**: ±4g

---

### `bmi270_set_gyro_range()`

ジャイロスコープの測定レンジを設定します。

```c
esp_err_t bmi270_set_gyro_range(bmi270_dev_t *dev, bmi270_gyr_range_t range);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `range`: 測定レンジ

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ書き込み失敗

**使用例**:
```c
// ±2000°/sに設定
bmi270_set_gyro_range(&dev, BMI270_GYR_RANGE_2000);
```

**レンジ選択肢**:
| 定数 | レンジ | 感度 | 用途 |
|------|--------|------|------|
| `BMI270_GYR_RANGE_125` | ±125°/s | 最高 | 微小回転 |
| `BMI270_GYR_RANGE_250` | ±250°/s | 非常に高 | 低速回転 |
| `BMI270_GYR_RANGE_500` | ±500°/s | 高 | 中速回転 |
| `BMI270_GYR_RANGE_1000` | ±1000°/s | 中 | 標準（デフォルト） |
| `BMI270_GYR_RANGE_2000` | ±2000°/s | 低 | 高速回転 |

**デフォルト**: ±1000°/s

**注意**: レンジは°/s（degrees per second）で表記されていますが、`bmi270_read_gyro()`および`bmi270_read_gyro_accel()`の出力はrad/s（radians per second）単位です。dps単位が必要な場合は、`bmi270_read_gyro_dps()`などのdps専用関数を使用してください。

---

## データ読み取りAPI

### `bmi270_read_gyro_accel()`

ジャイロスコープと加速度計のデータを同時に読み取ります（推奨）。

```c
esp_err_t bmi270_read_gyro_accel(bmi270_dev_t *dev,
                                  bmi270_gyro_t *gyro,
                                  bmi270_accel_t *accel);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `gyro`: ジャイロデータ構造体ポインタ（出力）
- `accel`: 加速度データ構造体ポインタ（出力）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**説明**:
- 12バイトをバースト読み取り（効率的）
- 生データ（int16_t）を物理値（float）に変換
- ジャイロ: rad/s (radians per second)
- 加速度: g (重力加速度)

**使用例**:
```c
bmi270_gyro_t gyro;
bmi270_accel_t accel;

esp_err_t ret = bmi270_read_gyro_accel(&dev, &gyro, &accel);
if (ret == ESP_OK) {
    printf("Gyro: X=%.3f Y=%.3f Z=%.3f [rad/s]\n", gyro.x, gyro.y, gyro.z);
    printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n", accel.x, accel.y, accel.z);
}
```

**パフォーマンス**:
- SPI転送: 1回（12バイト）
- 処理時間: 約50µs @ 10MHz SPI

---

### `bmi270_read_gyro()`

ジャイロスコープデータのみを読み取ります（rad/s単位）。

```c
esp_err_t bmi270_read_gyro(bmi270_dev_t *dev, bmi270_gyro_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `data`: ジャイロデータ構造体ポインタ（出力、rad/s単位）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**使用例**:
```c
bmi270_gyro_t gyro;
bmi270_read_gyro(&dev, &gyro);
printf("Gyro: X=%.3f Y=%.3f Z=%.3f [rad/s]\n", gyro.x, gyro.y, gyro.z);
```

---

### `bmi270_read_accel()`

加速度計データのみを読み取ります。

```c
esp_err_t bmi270_read_accel(bmi270_dev_t *dev, bmi270_accel_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `data`: 加速度データ構造体ポインタ（出力）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**使用例**:
```c
bmi270_accel_t accel;
bmi270_read_accel(&dev, &accel);
printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n", accel.x, accel.y, accel.z);
```

---

### `bmi270_read_gyro_dps()`

ジャイロスコープデータのみを読み取ります（dps単位）。

```c
esp_err_t bmi270_read_gyro_dps(bmi270_dev_t *dev, bmi270_gyro_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `data`: ジャイロデータ構造体ポインタ（出力、dps単位）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**説明**:
- `bmi270_read_gyro()`と同じデータを読み取りますが、単位がdps（degrees per second）です
- 従来のdps単位が必要な場合に使用します

**使用例**:
```c
bmi270_gyro_t gyro;
bmi270_read_gyro_dps(&dev, &gyro);
printf("Gyro: X=%.2f Y=%.2f Z=%.2f [dps]\n", gyro.x, gyro.y, gyro.z);
```

---

### `bmi270_read_gyro_accel_dps()`

ジャイロスコープ（dps単位）と加速度計のデータを同時に読み取ります。

```c
esp_err_t bmi270_read_gyro_accel_dps(bmi270_dev_t *dev,
                                      bmi270_gyro_t *gyro,
                                      bmi270_accel_t *accel);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `gyro`: ジャイロデータ構造体ポインタ（出力、dps単位）
- `accel`: 加速度データ構造体ポインタ（出力）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**説明**:
- `bmi270_read_gyro_accel()`と同じですが、ジャイロ単位がdps（degrees per second）です
- 従来のdps単位が必要な場合に使用します

**使用例**:
```c
bmi270_gyro_t gyro;
bmi270_accel_t accel;

esp_err_t ret = bmi270_read_gyro_accel_dps(&dev, &gyro, &accel);
if (ret == ESP_OK) {
    printf("Gyro: X=%.2f Y=%.2f Z=%.2f [dps]\n", gyro.x, gyro.y, gyro.z);
    printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n", accel.x, accel.y, accel.z);
}
```

---

### `bmi270_convert_gyro_raw()`

生データ（int16_t）を物理値（float, rad/s）に変換します。

```c
void bmi270_convert_gyro_raw(const bmi270_dev_t *dev,
                             const bmi270_raw_data_t *raw,
                             bmi270_gyro_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `raw`: 生データ構造体ポインタ
- `data`: 変換後データ構造体ポインタ（出力）

**使用例**:
```c
bmi270_raw_data_t raw_gyro = {gyr_x, gyr_y, gyr_z};
bmi270_gyro_t gyro;
bmi270_convert_gyro_raw(&dev, &raw_gyro, &gyro);
```

---

### `bmi270_convert_accel_raw()`

生データ（int16_t）を物理値（float, g）に変換します。

```c
void bmi270_convert_accel_raw(const bmi270_dev_t *dev,
                              const bmi270_raw_data_t *raw,
                              bmi270_accel_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `raw`: 生データ構造体ポインタ
- `data`: 変換後データ構造体ポインタ（出力）

**使用例**:
```c
bmi270_raw_data_t raw_accel = {acc_x, acc_y, acc_z};
bmi270_accel_t accel;
bmi270_convert_accel_raw(&dev, &raw_accel, &accel);
```

---

### `bmi270_convert_gyro_raw_dps()`

生データ（int16_t）を物理値（float, dps）に変換します。

```c
void bmi270_convert_gyro_raw_dps(const bmi270_dev_t *dev,
                                 const bmi270_raw_data_t *raw,
                                 bmi270_gyro_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `raw`: 生データ構造体ポインタ
- `data`: 変換後データ構造体ポインタ（出力、dps単位）

**説明**:
- `bmi270_convert_gyro_raw()`と同じですが、単位がdps（degrees per second）です
- 従来のdps単位が必要な場合に使用します

**使用例**:
```c
bmi270_raw_data_t raw_gyro = {gyr_x, gyr_y, gyr_z};
bmi270_gyro_t gyro;
bmi270_convert_gyro_raw_dps(&dev, &raw_gyro, &gyro);
```

---

### `bmi270_rad_to_dps()`

角速度をrad/sからdps（degrees per second）に変換します。

```c
float bmi270_rad_to_dps(float rad_per_sec);
```

**パラメータ**:
- `rad_per_sec`: 角速度 [rad/s]

**戻り値**:
- 角速度 [dps]

**説明**:
- 変換式: dps = rad/s × (180 / π)
- ユーティリティ関数（単位変換のみ）

**使用例**:
```c
bmi270_gyro_t gyro_rad;
bmi270_read_gyro(&dev, &gyro_rad);  // rad/s単位で読み取り

// dps単位に変換
float gyro_x_dps = bmi270_rad_to_dps(gyro_rad.x);
printf("Gyro X: %.2f [dps]\n", gyro_x_dps);
```

---

### `bmi270_dps_to_rad()`

角速度をdps（degrees per second）からrad/sに変換します。

```c
float bmi270_dps_to_rad(float dps);
```

**パラメータ**:
- `dps`: 角速度 [dps]

**戻り値**:
- 角速度 [rad/s]

**説明**:
- 変換式: rad/s = dps × (π / 180)
- ユーティリティ関数（単位変換のみ）

**使用例**:
```c
bmi270_gyro_t gyro_dps;
bmi270_read_gyro_dps(&dev, &gyro_dps);  // dps単位で読み取り

// rad/s単位に変換
float gyro_x_rad = bmi270_dps_to_rad(gyro_dps.x);
printf("Gyro X: %.3f [rad/s]\n", gyro_x_rad);
```

---

### `bmi270_read_temperature()`

BMI270内蔵の温度センサーからデータを読み取ります。

```c
esp_err_t bmi270_read_temperature(bmi270_dev_t *dev, float *temperature);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `temperature`: 温度値を格納する変数へのポインタ（出力、°C単位）

**戻り値**:
- `ESP_OK`: 成功
- その他: レジスタ読み取り失敗

**説明**:
- BMI270内蔵の温度センサーから温度を読み取ります
- 温度分解能: 約1/512 °C per LSB
- 測定範囲: -40°C ～ +85°C
- 変換式: `temperature [°C] = (raw_value / 512.0) + 23.0`
- 温度センサーは`bmi270_init()`で自動的に有効化されます

**使用例**:
```c
float temperature;
esp_err_t ret = bmi270_read_temperature(&dev, &temperature);
if (ret == ESP_OK) {
    printf("Temperature: %.2f [°C]\n", temperature);
}
```

**注意事項**:
- センサーの自己発熱により、周囲温度より数度高く測定される場合があります
- 高速ODR（1600Hz等）で連続動作させると、センサーの発熱が増加します
- 正確な環境温度測定には外部温度センサーの使用を推奨します

**推奨される使用方法**:
```c
// 温度は低頻度で読み取る（データ読み取りの10回に1回など）
if (sample_count % 10 == 0) {
    float temperature;
    bmi270_read_temperature(&dev, &temperature);
    printf("Temp: %.2f °C\n", temperature);
}
```

---

## 低レベルAPI

### `bmi270_read_register()`

1バイトレジスタ読み取り。

```c
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `reg_addr`: レジスタアドレス
- `data`: 読み取りデータポインタ（出力）

**戻り値**:
- `ESP_OK`: 成功
- その他: SPI転送失敗

**説明**:
- BMI270の3バイトReadプロトコルを自動処理
- TX: [0x80|addr] [Dummy] [Dummy]
- RX: [Echo] [Dummy] [DATA]

**使用例**:
```c
uint8_t chip_id;
bmi270_read_register(&dev, BMI270_REG_CHIP_ID, &chip_id);
ESP_LOGI(TAG, "CHIP_ID: 0x%02X", chip_id);  // 0x24のはず
```

---

### `bmi270_write_register()`

1バイトレジスタ書き込み。

```c
esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `reg_addr`: レジスタアドレス
- `data`: 書き込みデータ

**戻り値**:
- `ESP_OK`: 成功
- その他: SPI転送失敗

**説明**:
- BMI270の2バイトWriteプロトコル
- TX: [addr] [DATA]

**使用例**:
```c
// ACC/GYR有効化
bmi270_write_register(&dev, BMI270_REG_PWR_CTRL, 0x0E);
```

---

### `bmi270_read_burst()`

複数バイト連続読み取り（バースト転送）。

```c
esp_err_t bmi270_read_burst(bmi270_dev_t *dev,
                            uint8_t reg_addr,
                            uint8_t *data,
                            uint16_t length);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `reg_addr`: 開始レジスタアドレス
- `data`: 読み取りバッファポインタ（出力）
- `length`: 読み取りバイト数

**戻り値**:
- `ESP_OK`: 成功
- その他: SPI転送失敗

**説明**:
- 効率的な連続読み取り
- FIFOデータ読み取りなどに使用

**使用例**:
```c
uint8_t fifo_data[416];
bmi270_read_burst(&dev, BMI270_REG_FIFO_DATA, fifo_data, 416);
```

---

### `bmi270_write_burst()`

複数バイト連続書き込み（バースト転送）。

```c
esp_err_t bmi270_write_burst(bmi270_dev_t *dev,
                             uint8_t reg_addr,
                             const uint8_t *data,
                             uint16_t length);
```

**パラメータ**:
- `dev`: デバイス構造体ポインタ
- `reg_addr`: 開始レジスタアドレス
- `data`: 書き込みデータポインタ
- `length`: 書き込みバイト数

**戻り値**:
- `ESP_OK`: 成功
- その他: SPI転送失敗

**説明**:
- 効率的な連続書き込み
- コンフィグファイルアップロードなどに使用
- 最大連続長: 32バイト（推奨）

**使用例**:
```c
uint8_t config_chunk[32];
bmi270_write_burst(&dev, BMI270_REG_INIT_DATA, config_chunk, 32);
```

---

## 型定義

### `bmi270_dev_t`

デバイス構造体。

```c
typedef struct {
    spi_device_handle_t spi_handle;  // SPIデバイスハンドル
    uint8_t chip_id;                  // CHIP_ID (0x24)
    bmi270_gyr_range_t gyr_range;    // ジャイロレンジ
    bmi270_acc_range_t acc_range;    // 加速度レンジ
    uint32_t delay_us;                // SPI遅延時間
} bmi270_dev_t;
```

---

### `bmi270_config_t`

SPI設定構造体。

```c
typedef struct {
    int gpio_mosi;          // MOSI GPIOピン番号
    int gpio_miso;          // MISO GPIOピン番号
    int gpio_sclk;          // SCK GPIOピン番号
    int gpio_cs;            // CS GPIOピン番号
    int spi_clock_hz;       // SPI クロック周波数 (Hz)
    spi_host_device_t spi_host;  // SPIホスト (SPI2_HOST推奨)
    int gpio_other_cs;      // 共有SPIバスの他デバイスCS (-1=なし)
} bmi270_config_t;
```

**M5StampFly推奨設定**:
```c
bmi270_config_t config = {
    .gpio_mosi = 14,
    .gpio_miso = 43,
    .gpio_sclk = 44,
    .gpio_cs = 46,
    .spi_clock_hz = 10000000,  // 10 MHz
    .spi_host = SPI2_HOST,
    .gpio_other_cs = 12,       // PMW3901 CS
};
```

---

### `bmi270_gyro_t`

ジャイロスコープデータ（物理値）。

```c
typedef struct {
    float x;  // X軸角速度 [rad/s]
    float y;  // Y軸角速度 [rad/s]
    float z;  // Z軸角速度 [rad/s]
} bmi270_gyro_t;
```

**注意**:
- デフォルト単位はrad/s（radians per second）です
- `bmi270_read_gyro()`および`bmi270_read_gyro_accel()`はrad/s単位で値を返します
- dps（degrees per second）単位が必要な場合は、`bmi270_read_gyro_dps()`または`bmi270_read_gyro_accel_dps()`を使用してください

---

### `bmi270_accel_t`

加速度計データ（物理値）。

```c
typedef struct {
    float x;  // X軸加速度 [g]
    float y;  // Y軸加速度 [g]
    float z;  // Z軸加速度 [g]
} bmi270_accel_t;
```

---

### `bmi270_raw_data_t`

センサー生データ。

```c
typedef struct {
    int16_t x;  // X軸生データ
    int16_t y;  // Y軸生データ
    int16_t z;  // Z軸生データ
} bmi270_raw_data_t;
```

---

## 定数

### レジスタアドレス

```c
#define BMI270_REG_CHIP_ID          0x00  // CHIP_ID (0x24)
#define BMI270_REG_DATA_0           0x0C  // データレジスタ開始
#define BMI270_REG_ACC_X_LSB        0x0C  // 加速度X LSB
#define BMI270_REG_GYR_X_LSB        0x12  // ジャイロX LSB
#define BMI270_REG_PWR_CTRL         0x7D  // 電源制御
#define BMI270_REG_INIT_DATA        0x5E  // コンフィグアップロード
#define BMI270_REG_ACC_CONF         0x40  // 加速度設定
#define BMI270_REG_GYR_CONF         0x42  // ジャイロ設定
```

詳細は`bmi270_defs.h`を参照。

---

## 使用例

### 基本的な初期化とデータ読み取り

```c
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

void app_main(void)
{
    // 1. SPI初期化
    bmi270_dev_t dev = {0};
    bmi270_config_t config = {
        .gpio_mosi = 14,
        .gpio_miso = 43,
        .gpio_sclk = 44,
        .gpio_cs = 46,
        .spi_clock_hz = 10000000,
        .spi_host = SPI2_HOST,
        .gpio_other_cs = 12,
    };
    bmi270_spi_init(&dev, &config);

    // 2. センサー初期化
    bmi270_init(&dev);

    // 3. センサー設定
    bmi270_set_accel_config(&dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    bmi270_set_gyro_config(&dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);

    // 4. データ読み取りループ
    while (1) {
        bmi270_gyro_t gyro;
        bmi270_accel_t accel;
        bmi270_read_gyro_accel(&dev, &gyro, &accel);

        printf("Gyro: X=%.3f Y=%.3f Z=%.3f [rad/s]\n", gyro.x, gyro.y, gyro.z);
        printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n", accel.x, accel.y, accel.z);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## 関連資料

- [BMI270 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)
- [ESP-IDF SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/spi_master.html)
- [サンプルコード](../examples/)
