# ESP-IDF SPI Master ドライバを使用したBMI270通信実装ガイド

## 目次
1. [開発環境](#1-開発環境)
2. [ハードウェア接続](#2-ハードウェア接続)
3. [ESP-IDF SPI Master API概要](#3-esp-idf-spi-master-api概要)
4. [BMI270 SPI通信プロトコル](#4-bmi270-spi通信プロトコル)
5. [実装パターン](#5-実装パターン)
6. [トラブルシューティング](#6-トラブルシューティング)

---

## 1. 開発環境

### ESP-IDFバージョン
- **使用バージョン**: ESP-IDF v5.4.1
- **対象MCU**: ESP32-S3 (M5StampS3)
- **開発環境セットアップ**:
  ```bash
  source ~/esp/esp-idf/export.sh
  ```

### 必要なヘッダーファイル
```c
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
```

---

## 2. ハードウェア接続

### M5StampFly のピン配置

| 信号 | ESP32-S3 GPIO | BMI270ピン | 備考 |
|------|---------------|------------|------|
| MOSI | GPIO14 | SDx (SDI) | マスター→スレーブ |
| SCK  | GPIO44 | SCx | クロック |
| MISO | GPIO43 | SDO | スレーブ→マスター |
| CS   | GPIO46 | CSB | チップセレクト |

### ESP32-S3のSPIホスト
- **使用ホスト**: SPI2_HOST
- **理由**: SPI3_HOSTはフラッシュで使用される可能性があるため

### 電気的特性
- **電源電圧**: 3.3V (VDDIO)
- **最大クロック周波数**: 10 MHz (BMI270仕様)
- **SPIモード**: Mode 0 (CPOL=0, CPHA=0) または Mode 3 (CPOL=1, CPHA=1)

---

## 3. ESP-IDF SPI Master API概要

### 3.1 初期化の流れ

```
1. spi_bus_initialize()      ← バス全体の初期化
2. spi_bus_add_device()      ← デバイス（BMI270）を追加
3. トランザクション実行      ← データ送受信
4. spi_bus_remove_device()   ← デバイス削除（終了時）
5. spi_bus_free()            ← バス解放（終了時）
```

### 3.2 主要な構造体

#### `spi_bus_config_t` - バス設定
```c
typedef struct {
    int mosi_io_num;           // MOSIピン番号
    int miso_io_num;           // MISOピン番号
    int sclk_io_num;           // SCLKピン番号
    int quadwp_io_num;         // -1 (不使用)
    int quadhd_io_num;         // -1 (不使用)
    int max_transfer_sz;       // 最大転送サイズ (バイト)
    uint32_t flags;            // SPICOMMON_BUSFLAG_xxx
    int isr_cpu_id;            // 割り込みを処理するCPU
    intr_cpu_id_t intr_flags;  // 割り込みフラグ
} spi_bus_config_t;
```

#### `spi_device_interface_config_t` - デバイス設定
```c
typedef struct {
    uint8_t command_bits;      // コマンドフェーズのビット数
    uint8_t address_bits;      // アドレスフェーズのビット数
    uint8_t dummy_bits;        // ダミービット数
    uint8_t mode;              // SPIモード (0-3)
    uint16_t duty_cycle_pos;   // デューティサイクル (50% = 128)
    uint16_t cs_ena_pretrans;  // CS有効化の事前サイクル数
    uint8_t cs_ena_posttrans;  // CS有効化の事後サイクル数
    int clock_speed_hz;        // クロック周波数 (Hz)
    int input_delay_ns;        // 入力遅延補償 (ns)
    int spics_io_num;          // CSピン番号
    uint32_t flags;            // SPI_DEVICE_xxx フラグ
    int queue_size;            // トランザクションキューサイズ
    transaction_cb_t pre_cb;   // トランザクション前コールバック
    transaction_cb_t post_cb;  // トランザクション後コールバック
} spi_device_interface_config_t;
```

#### `spi_transaction_t` - トランザクション
```c
typedef struct {
    uint32_t flags;            // SPI_TRANS_xxx フラグ
    uint16_t cmd;              // コマンドデータ
    uint64_t addr;             // アドレスデータ
    size_t length;             // 総データ長 (ビット)
    size_t rxlength;           // 受信データ長 (ビット、0=length)
    void *user;                // ユーザーデータ
    union {
        const void *tx_buffer; // 送信バッファへのポインタ
        uint8_t tx_data[4];    // 4バイト以下ならここに直接
    };
    union {
        void *rx_buffer;       // 受信バッファへのポインタ
        uint8_t rx_data[4];    // 4バイト以下ならここに直接
    };
} spi_transaction_t;
```

### 3.3 トランザクション実行方法

| 関数 | 説明 | 用途 |
|------|------|------|
| `spi_device_transmit()` | 同期（ブロッキング） | 単発の読み書き |
| `spi_device_polling_transmit()` | ポーリング（ブロッキング） | 割り込みなし、高速 |
| `spi_device_queue_trans()` + `spi_device_get_trans_result()` | 非同期（キューイング） | 連続転送 |

### 3.4 DMA設定

| 設定値 | 説明 | 最大転送サイズ |
|--------|------|----------------|
| `SPI_DMA_DISABLED` | DMA無効 | 64バイト |
| `SPI_DMA_CH_AUTO` | DMA自動割り当て | 制限なし（RAM次第） |

**重要**: DMA使用時は `heap_caps_malloc(size, MALLOC_CAP_DMA)` でバッファを確保

---

## 4. BMI270 SPI通信プロトコル

### 4.1 読み取り操作の特殊性

BMI270のSPI読み取りは**3バイトトランザクション**です：

```
全二duplex通信:
送信: [CMD: R/W=1 + Addr] [Dummy] [Dummy]
受信: [Echo]                [Dummy] [DATA]  ← 3バイト目が有効データ
       ↑                     ↑       ↑
      破棄                  破棄   目的の値
```

#### シーケンス詳細

1. **Byte 1 (コマンド送信)**
   - bit 7: 1 = 読み取り
   - bit 6-0: レジスタアドレス
   - 例: `0x80` = CHIP_ID (0x00) 読み取り

2. **Byte 2 (ダミー受信)**
   - **破棄必須**（無効データ）

3. **Byte 3 (データ受信)**
   - **有効なレジスタ値**

#### タイミング図
```
CS   ‾‾|___________________|‾‾‾
        ↓                   ↓
SCK   __|‾|_|‾|_..._|‾|_|‾|_|__
        ↓                   ↓
MOSI  --[CMD]--[0x00]--[0x00]--
        ↓                   ↓
MISO  --[xx]--[Dummy]--[DATA]--
```

### 4.2 書き込み操作

書き込みは**2バイトトランザクション**：

```
送信: [CMD: R/W=0 + Addr] [DATA]
       ↑                    ↑
    書き込み指示        書き込む値
```

### 4.3 バースト読み取り

連続レジスタの読み取り（例: ACC X,Y,Z = 6バイト）:

```
送信: [CMD] [Dummy] [Dummy] [Dummy] ... [Dummy]
受信: [xx]  [Dummy] [D0]    [D1]    ... [DN]
                     ↑ ここから有効データ（N個）
```

**重要**: LSBレジスタから読み取り開始すると、対応するMSBがシャドウイング（ロック）される

### 4.4 バースト書き込み

大量データ書き込み（例: 8KBコンフィグファイル）:

```
送信: [CMD] [D0] [D1] [D2] ... [DN]
       ↑     ↑ データ開始（N個）
    書き込み
```

### 4.5 タイミング制約

| 項目 | 値 | 備考 |
|------|-----|------|
| 最大クロック周波数 | 10 MHz | VDDIO ≥ 1.62V時 |
| 書き込み後待機（通常モード） | 2 µs | t_IDLE_wr_act |
| 書き込み後待機（低電力モード） | 450 µs | t_IDLE_wacc_sum |
| 電源投入後待機 | 450 µs | 初回アクセス前 |
| 初期化完了待機（最大） | 20 ms | INTERNAL_STATUSポーリング |

---

## 5. 実装パターン

### 5.1 SPI初期化

```c
#include "driver/spi_master.h"
#include "driver/gpio.h"

// StampFly GPIO定義
#define BMI270_MOSI_PIN   14
#define BMI270_MISO_PIN   43
#define BMI270_SCLK_PIN   44
#define BMI270_CS_PIN     46

#define BMI270_SPI_CLOCK  10000000  // 10 MHz
#define BMI270_SPI_HOST   SPI2_HOST

static spi_device_handle_t bmi270_spi_handle;

esp_err_t bmi270_spi_init(void) {
    esp_err_t ret;

    // 1. SPIバス設定
    spi_bus_config_t bus_config = {
        .mosi_io_num = BMI270_MOSI_PIN,
        .miso_io_num = BMI270_MISO_PIN,
        .sclk_io_num = BMI270_SCLK_PIN,
        .quadwp_io_num = -1,  // 不使用
        .quadhd_io_num = -1,  // 不使用
        .max_transfer_sz = 8192 + 2,  // コンフィグファイル + ヘッダ
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    // バス初期化（DMA有効）
    ret = spi_bus_initialize(BMI270_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return ret;
    }

    // 2. BMI270デバイス設定
    spi_device_interface_config_t dev_config = {
        .mode = 0,  // SPI Mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = BMI270_SPI_CLOCK,
        .spics_io_num = BMI270_CS_PIN,
        .queue_size = 7,  // トランザクションキューサイズ
        .flags = 0,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
    };

    // デバイス追加
    ret = spi_bus_add_device(BMI270_SPI_HOST, &dev_config, &bmi270_spi_handle);
    return ret;
}
```

### 5.2 単一レジスタ読み取り（3バイトトランザクション）

```c
/**
 * @brief BMI270レジスタ読み取り（3バイト方式）
 *
 * @param reg_addr レジスタアドレス (0x00-0x7F)
 * @param data 読み取ったデータを格納するポインタ
 * @return esp_err_t ESP_OK=成功
 */
esp_err_t bmi270_read_register(uint8_t reg_addr, uint8_t *data) {
    esp_err_t ret;

    // 3バイトトランザクション用バッファ
    uint8_t tx_data[3] = {
        reg_addr | 0x80,  // Byte 1: Read command (bit7=1)
        0x00,             // Byte 2: Dummy (送信側)
        0x00              // Byte 3: Dummy (送信側)
    };
    uint8_t rx_data[3] = {0};

    spi_transaction_t trans = {
        .flags = 0,
        .length = 3 * 8,      // 3バイト = 24ビット
        .rxlength = 3 * 8,    // 3バイト受信
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    // 同期送受信
    ret = spi_device_polling_transmit(bmi270_spi_handle, &trans);

    if (ret == ESP_OK) {
        // rx_data[0] = コマンドエコー（破棄）
        // rx_data[1] = ダミー（破棄）
        *data = rx_data[2];  // ★ Byte 3が有効データ
    }

    return ret;
}
```

### 5.3 単一レジスタ書き込み（2バイトトランザクション）

```c
/**
 * @brief BMI270レジスタ書き込み（2バイト方式）
 *
 * @param reg_addr レジスタアドレス (0x00-0x7F)
 * @param data 書き込むデータ
 * @return esp_err_t ESP_OK=成功
 */
esp_err_t bmi270_write_register(uint8_t reg_addr, uint8_t data) {
    esp_err_t ret;

    // 2バイトトランザクション用バッファ
    uint8_t tx_data[2] = {
        reg_addr & 0x7F,  // Byte 1: Write command (bit7=0)
        data              // Byte 2: データ
    };

    spi_transaction_t trans = {
        .flags = 0,
        .length = 2 * 8,      // 2バイト = 16ビット
        .tx_buffer = tx_data,
        .rx_buffer = NULL,    // 受信不要
    };

    // 同期送信
    ret = spi_device_polling_transmit(bmi270_spi_handle, &trans);

    if (ret == ESP_OK) {
        // 書き込み後の待機時間（2µs、通常モード）
        esp_rom_delay_us(2);
    }

    return ret;
}
```

### 5.4 バースト読み取り（複数レジスタ）

```c
/**
 * @brief 連続レジスタ読み取り（バースト）
 *
 * @param reg_addr 開始レジスタアドレス
 * @param data 読み取りデータ格納バッファ
 * @param length 読み取るバイト数
 * @return esp_err_t ESP_OK=成功
 */
esp_err_t bmi270_read_burst(uint8_t reg_addr, uint8_t *data, size_t length) {
    esp_err_t ret;

    // 総バイト数 = 1 (CMD) + 1 (Dummy) + length (Data)
    size_t total_bytes = 2 + length;

    // DMA対応バッファ確保
    uint8_t *tx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);
    uint8_t *rx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);

    if (!tx_buffer || !rx_buffer) {
        heap_caps_free(tx_buffer);
        heap_caps_free(rx_buffer);
        return ESP_ERR_NO_MEM;
    }

    // 送信バッファ準備
    tx_buffer[0] = reg_addr | 0x80;  // Read command
    memset(&tx_buffer[1], 0x00, total_bytes - 1);  // Dummy bytes

    spi_transaction_t trans = {
        .flags = 0,
        .length = total_bytes * 8,
        .rxlength = total_bytes * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
    };

    ret = spi_device_polling_transmit(bmi270_spi_handle, &trans);

    if (ret == ESP_OK) {
        // rx_buffer[0] = コマンドエコー（破棄）
        // rx_buffer[1] = ダミー（破棄）
        // rx_buffer[2〜] = 有効データ
        memcpy(data, &rx_buffer[2], length);
    }

    heap_caps_free(tx_buffer);
    heap_caps_free(rx_buffer);

    return ret;
}
```

### 5.5 バースト書き込み（コンフィグファイル）

```c
/**
 * @brief 連続レジスタ書き込み（バースト）
 *
 * @param reg_addr 開始レジスタアドレス
 * @param data 書き込みデータ
 * @param length 書き込むバイト数
 * @return esp_err_t ESP_OK=成功
 */
esp_err_t bmi270_write_burst(uint8_t reg_addr, const uint8_t *data, size_t length) {
    esp_err_t ret;

    // 総バイト数 = 1 (CMD) + length (Data)
    size_t total_bytes = 1 + length;

    // DMA対応バッファ確保
    uint8_t *tx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);

    if (!tx_buffer) {
        return ESP_ERR_NO_MEM;
    }

    // 送信バッファ準備
    tx_buffer[0] = reg_addr & 0x7F;  // Write command
    memcpy(&tx_buffer[1], data, length);

    spi_transaction_t trans = {
        .flags = 0,
        .length = total_bytes * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL,
    };

    ret = spi_device_polling_transmit(bmi270_spi_handle, &trans);

    heap_caps_free(tx_buffer);

    if (ret == ESP_OK) {
        // 書き込み後の待機時間（低電力モード: 450µs）
        esp_rom_delay_us(450);
    }

    return ret;
}
```

### 5.6 CHIP_ID読み取り例

```c
#define BMI270_REG_CHIP_ID  0x00
#define BMI270_CHIP_ID      0x24

esp_err_t bmi270_check_chip_id(void) {
    uint8_t chip_id;
    esp_err_t ret;

    ret = bmi270_read_register(BMI270_REG_CHIP_ID, &chip_id);

    if (ret != ESP_OK) {
        ESP_LOGE("BMI270", "Failed to read CHIP_ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id == BMI270_CHIP_ID) {
        ESP_LOGI("BMI270", "CHIP_ID verified: 0x%02X ✓", chip_id);
        return ESP_OK;
    } else {
        ESP_LOGE("BMI270", "Invalid CHIP_ID: 0x%02X (expected 0x%02X)",
                 chip_id, BMI270_CHIP_ID);
        return ESP_ERR_INVALID_VERSION;
    }
}
```

### 5.7 6軸データ読み取り（シャドウイング対応）

```c
typedef struct {
    int16_t acc_x, acc_y, acc_z;   // 加速度 [LSB]
    int16_t gyr_x, gyr_y, gyr_z;   // 角速度 [LSB]
} bmi270_sensor_data_t;

#define BMI270_REG_ACC_X_LSB  0x0C  // ACC X軸 LSB

esp_err_t bmi270_read_sensor_data(bmi270_sensor_data_t *data) {
    uint8_t raw_data[12];  // ACC(6) + GYR(6) = 12バイト
    esp_err_t ret;

    // LSBから読み取り開始（シャドウイング）
    ret = bmi270_read_burst(BMI270_REG_ACC_X_LSB, raw_data, 12);

    if (ret == ESP_OK) {
        // リトルエンディアンで結合
        data->acc_x = (int16_t)(raw_data[0] | (raw_data[1] << 8));
        data->acc_y = (int16_t)(raw_data[2] | (raw_data[3] << 8));
        data->acc_z = (int16_t)(raw_data[4] | (raw_data[5] << 8));
        data->gyr_x = (int16_t)(raw_data[6] | (raw_data[7] << 8));
        data->gyr_y = (int16_t)(raw_data[8] | (raw_data[9] << 8));
        data->gyr_z = (int16_t)(raw_data[10] | (raw_data[11] << 8));
    }

    return ret;
}
```

---

## 6. トラブルシューティング

### 6.1 CHIP_IDが読めない（0x00または0xFFが返る）

**原因と対策**:

1. **SPIモードに切り替わっていない**
   - 電源投入後、最初のダミーリードを実行
   ```c
   uint8_t dummy;
   bmi270_read_register(BMI270_REG_CHIP_ID, &dummy);  // 1回目
   bmi270_read_register(BMI270_REG_CHIP_ID, &dummy);  // 2回目で正常
   ```

2. **配線ミス**
   - テスターで導通確認
   - オシロスコープで信号確認

3. **電源不足**
   - 3.3V電源の電圧降下確認
   - デカップリングコンデンサの追加

### 6.2 データが1バイトずれる

**原因**: 3バイトトランザクションのダミーバイトを考慮していない

**対策**: `rx_buffer[2]` からデータを取得（`rx_buffer[0]`, `rx_buffer[1]` は破棄）

### 6.3 初期化が失敗する（INTERNAL_STATUS != 0x01）

**原因と対策**:

1. **待機時間不足**
   - 書き込み後の待機時間を確認（450µs）
   - `esp_rom_delay_us()` を使用

2. **コンフィグファイルが不完全**
   - サイズ確認（8192バイト）
   - チェックサム検証（あれば）

3. **バースト書き込みの失敗**
   - DMAバッファ確保確認
   - 最大転送サイズ確認

### 6.4 DMAエラー

**原因**: バッファがDMA非対応領域に確保されている

**対策**:
```c
uint8_t *buffer = heap_caps_malloc(size, MALLOC_CAP_DMA);
```

### 6.5 クロック周波数が高すぎる

**症状**: データが不安定

**対策**: クロック周波数を下げる
```c
.clock_speed_hz = 5000000,  // 5 MHz（10 MHzから下げる）
```

### 6.6 CS信号の問題

**確認**: オシロスコープでCSの立ち上がり/立ち下がりを確認

**対策**: pre/post trans設定を調整
```c
.cs_ena_pretrans = 1,   // CS有効化を1サイクル早める
.cs_ena_posttrans = 1,  // CS無効化を1サイクル遅らせる
```

---

## 付録A: レジスタマップ（主要なもの）

| アドレス | 名称 | 説明 | R/W |
|---------|------|------|-----|
| 0x00 | CHIP_ID | チップID (0x24) | R |
| 0x0C-0x11 | ACC_X/Y/Z | 加速度データ (LSB, MSB) | R |
| 0x12-0x17 | GYR_X/Y/Z | 角速度データ (LSB, MSB) | R |
| 0x21 | INTERNAL_STATUS | 内部ステータス | R |
| 0x24-0x25 | FIFO_LENGTH | FIFOデータ量 | R |
| 0x26 | FIFO_DATA | FIFOデータ読み取り | R |
| 0x40 | ACC_CONF | 加速度設定 (ODR, Filter) | R/W |
| 0x42 | GYR_CONF | ジャイロ設定 (ODR, Filter) | R/W |
| 0x59 | INIT_CTRL | 初期化制御 | W |
| 0x5B-0x5C | INIT_ADDR | 初期化アドレス | W |
| 0x5E | INIT_DATA | コンフィグデータ | W |
| 0x7C | PWR_CONF | 電源設定 | R/W |
| 0x7D | PWR_CTRL | センサー有効化 | R/W |

---

## 付録B: ESP-IDF SPI関連リンク

- [ESP-IDF SPI Master Driver Documentation (v5.4.1)](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/spi_master.html)
- [ESP-IDF GitHub - SPI Examples](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master)
- [SPI Master Header File](https://github.com/espressif/esp-idf/blob/master/components/esp_driver_spi/include/driver/spi_master.h)

---

## 付録C: BMI270関連リンク

- [BMI270 Datasheet (Bosch Sensortec)](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)
- [BMI270 Sensor API (GitHub)](https://github.com/boschsensortec/BMI270_SensorAPI)
- [本プロジェクト: BMI270実装ガイド](./bmi270_doc_ja.md)

---

**Document Version**: 1.0
**Last Updated**: 2025-11-16
**Author**: Claude Code (Anthropic)
**Target**: ESP32-S3 (M5StampS3) + BMI270 IMU
