# ESP-IDF I2C Master API 調査レポート

## 調査日
2025-11-15

## 目的
StampFly ToFプロジェクトで新しいI2C master API (`driver/i2c_master.h`) への移行が失敗している原因を調査し、正しい実装方法を明確にする。

## 問題の症状
- 古いI2C API (`driver/i2c.h`): 4デバイス検出成功（0x10, 0x29, 0x40, 0x76）
- 新しいI2C master API: 全アドレスでタイムアウト（ESP_ERR_TIMEOUT）

## 新しいI2C Master API の概要

### 基本構造

```c
// 1. バスハンドルをグローバルに保持
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

// 2. バス設定構造体
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,      // クロックソース
    .i2c_port = I2C_NUM_0,                  // I2Cポート番号（-1で自動選択）
    .scl_io_num = GPIO_NUM_4,               // SCL GPIO番号
    .sda_io_num = GPIO_NUM_3,               // SDA GPIO番号
    .glitch_ignore_cnt = 7,                 // ノイズフィルタ（通常7）
    .flags.enable_internal_pullup = true,   // 内部プルアップ有効化
};

// 3. バス初期化
esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
```

### プローブ関数

```c
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle,
                           uint16_t address,
                           int xfer_timeout_ms);
```

**パラメータ:**
- `bus_handle`: 初期化済みのバスハンドル
- `address`: 7ビットI2Cアドレス（0x03～0x77）
- `xfer_timeout_ms`: タイムアウト（ms）
  - 正の値: タイムアウト時間（例：50ms）
  - -1: 無制限待機（非推奨）

**戻り値:**
- `ESP_OK`: デバイス検出成功（ACK受信）
- `ESP_ERR_NOT_FOUND`: デバイス未検出（NACK受信）
- `ESP_ERR_TIMEOUT`: バスビジー、ハードウェア故障、またはプルアップ抵抗の問題

## 古いAPIとの比較

### 初期化の違い

| 項目 | 古いAPI (`driver/i2c.h`) | 新しいAPI (`driver/i2c_master.h`) |
|------|--------------------------|-----------------------------------|
| 構造体 | `i2c_config_t` | `i2c_master_bus_config_t` |
| プルアップ | `sda_pullup_en`, `scl_pullup_en` (個別) | `flags.enable_internal_pullup` (一括) |
| 周波数設定 | `master.clk_speed` (バス初期化時) | デバイス追加時に設定 |
| 初期化関数 | `i2c_param_config()` + `i2c_driver_install()` | `i2c_new_master_bus()` |

### スキャン方法の違い

**古いAPI:**
```c
for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(STAMPFLY_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // デバイス検出
    }
}
```

**新しいAPI:**
```c
for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);

    if (ret == ESP_OK) {
        // デバイス検出
    }
}
```

## ESP-IDF公式サンプル (i2c_tools)

ESP-IDFの`examples/peripherals/i2c/i2c_tools`が参考実装として最適。

### バス初期化（i2cconfig）

```c
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,  // 自動選択
    .scl_io_num = i2c_gpio_scl,
    .sda_io_num = i2c_gpio_sda,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    }
};

ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus));
```

### デバイススキャン（i2cdetect）

```c
#define I2C_TOOL_TIMEOUT_VALUE_MS  50

for (int i = 0; i < 128; i += 16) {
    for (int j = 0; j < 16; j++) {
        address = i + j;
        esp_err_t ret = i2c_master_probe(s_i2c_bus, address,
                                        I2C_TOOL_TIMEOUT_VALUE_MS);
        if (ret == ESP_OK) {
            printf("%02x ", address);  // デバイス検出
        } else if (ret == ESP_ERR_TIMEOUT) {
            printf("UU ");  // タイムアウト（バスビジー）
        } else {
            printf("-- ");  // デバイスなし
        }
    }
}
```

## 既知の問題とバグ

### Issue #12159, #13213 (2023-2024)
- **症状**: `i2c_master_probe()`が全アドレスでESP_OKを返す
- **原因**: ドライバのバグ
- **状態**: 修正済み（commit `6e3e923`）
- **影響**: ESP-IDF v5.1～v5.2の一部バージョン

### 現在の問題との関連性
上記バグは「全てESP_OK」だが、現在の問題は「全てESP_ERR_TIMEOUT」なので、**別の原因**と考えられる。

## タイムアウトの原因候補

ESP-IDFドキュメントによると、`ESP_ERR_TIMEOUT`は以下の原因で発生する：
1. バスがビジー状態
2. ハードウェアクラッシュ
3. **プルアップ抵抗が接続されていない**

古いAPIで動作していたことから、ハードウェア自体は問題ないと推測される。

## 疑問点と調査項目

### 1. クロック周波数の設定場所
- 古いAPI: `master.clk_speed`でバス初期化時に設定
- 新しいAPI: デバイス追加時に`i2c_device_config_t::scl_speed_hz`で設定
- **疑問**: `i2c_master_probe()`を使う場合、デバイスを追加しないのでどこで周波数が設定される？

### 2. プルアップの設定方法
- 古いAPI: `sda_pullup_en`, `scl_pullup_en`で個別に設定
- 新しいAPI: `flags.enable_internal_pullup`で一括設定
- **疑問**: フラグの記述方法が正しいか？

### 3. バスハンドルのスコープ
- 公式サンプルでは静的グローバル変数 (`static i2c_master_bus_handle_t s_i2c_bus`)
- 現在のコードでも同様に実装済み

## 根本原因の特定

### ESP-IDF v5.4.1環境での調査結果

#### 使用中のESP-IDFバージョン
```bash
$ cd ~/esp/esp-idf && git describe --tags
v5.4.1
```
既知のi2c_master_probe()バグは修正済みのバージョン。

#### i2c_master_probe()の内部実装

ESP-IDF v5.4.1のソースコード（`components/esp_driver_i2c/i2c_master.c:1247`）より：

```c
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle, uint16_t address, int xfer_timeout_ms)
{
    ESP_RETURN_ON_FALSE(bus_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "i2c handle not initialized");
    TickType_t ticks_to_wait = (xfer_timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(xfer_timeout_ms);
    if (xSemaphoreTake(bus_handle->bus_lock_mux, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;  // ここでタイムアウト！
    }
    // ... 実際のI2C通信処理
}
```

**重大な発見:**
1. 関数内部で`pdMS_TO_TICKS(xfer_timeout_ms)`を実行している
2. つまり、呼び出し側は**ミリ秒単位の整数値**を渡す必要がある
3. `pdMS_TO_TICKS()`を二重に適用してはいけない

### 現在のコードの問題点

#### 問題1: タイムアウト値の二重変換 ⚠️

**現在のコード（誤り）:**
```c
esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(50));
```

この場合：
- `pdMS_TO_TICKS(50)` = 5 ticks（100Hzの場合）
- 関数内部で再度`pdMS_TO_TICKS(5)` = 0～1 tick
- 実質的に**タイムアウト時間が極端に短くなる**

**正しいコード:**
```c
esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);  // 50ミリ秒
```

この場合：
- 引数として50（ミリ秒）を渡す
- 関数内部で`pdMS_TO_TICKS(50)` = 50 ticks
- 正しく50ミリ秒のタイムアウトが設定される

#### 問題2: i2c_master_bus_config_tの構造体定義

ESP-IDF v5.4.1のヘッダーファイル（`components/esp_driver_i2c/include/driver/i2c_master.h:21`）より：

```c
typedef struct {
    i2c_port_num_t i2c_port;              // I2Cポート番号、-1で自動選択
    gpio_num_t sda_io_num;                // SDA GPIO番号
    gpio_num_t scl_io_num;                // SCL GPIO番号
    union {
        i2c_clock_source_t clk_source;    // クロックソース
    };
    uint8_t glitch_ignore_cnt;            // グリッチフィルタ（通常7）
    int intr_priority;                    // 割り込み優先度（0で自動選択）
    size_t trans_queue_depth;             // 転送キュー深さ（非同期転送用）
    struct {
        uint32_t enable_internal_pullup: 1;  // 内部プルアップ有効化
                                              // 注意: 高速周波数では不十分
                                              // 可能であれば外部プルアップ推奨
        uint32_t allow_pd: 1;                 // スリープモード対応
    } flags;
} i2c_master_bus_config_t;
```

**現在のコードで不足している設定:**
```c
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = STAMPFLY_I2C_PORT,
    .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
    .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    // ⚠️ intr_priorityが未設定（デフォルト0で自動選択）
    // ⚠️ trans_queue_depthが未設定（デフォルト0でキューなし）
};
```

通常、同期的なi2c_master_probe()を使う場合は`trans_queue_depth = 0`で問題ないはずだが、念のため確認が必要。

## デバッグ計画（優先度順）

### Step 1: タイムアウト値の修正 ⭐最優先⭐

**理由**: 二重変換により、実質的なタイムアウトが1ms未満になっている可能性が高い。

**修正内容:**
```c
// 修正前
esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(50));

// 修正後
esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);
```

**期待される結果**: この修正だけで問題が解決する可能性が非常に高い。

### Step 2: バス設定の完全な初期化

公式サンプル（i2c_tools）に合わせて、明示的に全フィールドを初期化する。

```c
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = STAMPFLY_I2C_PORT,
    .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
    .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,  // 明示的に0を設定（自動選択）
    .trans_queue_depth = 0,  // 同期モードではキュー不要
    .flags = {
        .enable_internal_pullup = true,
    },
};
```

### Step 3: エラーハンドリングの改善

タイムアウトと通信エラーを区別できるよう、エラーメッセージを詳細化する。

```c
esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
} else if (ret == ESP_ERR_NOT_FOUND) {
    // 正常：デバイスなし（NAck受信）
} else if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGW(TAG, "Timeout at address 0x%02X - check pull-ups", addr);
} else {
    ESP_LOGE(TAG, "Error at address 0x%02X: %s", addr, esp_err_to_name(ret));
}
```

### Step 4: 代替案 - 古いAPIへの一時的な復帰

Step 1で解決しない場合、新しいAPIに根本的な問題がある可能性も考慮。Stage 2以降の開発を進めるため、一時的に古いAPIに戻すことも検討。

```c
// 古いAPIは動作実績があるため、確実に動作する
#include "driver/i2c.h"  // 新しいAPIではなく旧APIを使用
```

deprecation warningは出るが、機能的には問題ない。

## 参考資料

1. **ESP-IDF公式ドキュメント**
   - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

2. **ESP-IDF公式サンプルコード**
   - https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_tools

3. **関連Issue**
   - Issue #12159: https://github.com/espressif/esp-idf/issues/12159
   - Issue #13213: https://github.com/espressif/esp-idf/issues/13213

4. **ソースコード**
   - https://github.com/espressif/esp-idf/blob/master/components/esp_driver_i2c/i2c_master.c

## 推奨される実装例（修正版）

### 完全なI2Cバススキャンコード

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stampfly_tof_config.h"

static const char *TAG = "I2C_SCAN";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

/**
 * @brief Initialize I2C master (新しいAPI - 推奨実装)
 */
static esp_err_t i2c_master_init(void)
{
    // バス設定（全フィールドを明示的に初期化）
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = STAMPFLY_I2C_PORT,        // または-1で自動選択
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,                    // 自動選択
        .trans_queue_depth = 0,                // 同期モードではキュー不要
        .flags = {
            .enable_internal_pullup = true,    // 内部プルアップ有効化
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d",
             STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);

    return ESP_OK;
}

/**
 * @brief Scan I2C bus for devices (修正版)
 */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    ESP_LOGI(TAG, "Scanning address range: 0x03 to 0x77");

    int devices_found = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        // ⭐重要⭐ タイムアウトはミリ秒単位の整数値で渡す
        // pdMS_TO_TICKS()は使わない！
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
            devices_found++;

            // VL53L3CXのデフォルトアドレスをチェック
            if (addr == VL53L3CX_DEFAULT_I2C_ADDR) {
                ESP_LOGI(TAG, "  -> VL53L3CX detected at default address!");
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // タイムアウト = バスビジーまたはプルアップ抵抗の問題
            ESP_LOGW(TAG, "Timeout at address 0x%02X", addr);
        }
        // ESP_ERR_NOT_FOUND（NACK）は正常なので何もしない
    }

    ESP_LOGI(TAG, "I2C scan completed. Devices found: %d", devices_found);

    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Please check:");
        ESP_LOGW(TAG, "  - I2C wiring (SDA, SCL)");
        ESP_LOGW(TAG, "  - Pull-up resistors (2-5kΩ recommended)");
        ESP_LOGW(TAG, "  - Sensor power supply");
        ESP_LOGW(TAG, "  - XSHUT pin levels");
    }
}
```

### 重要なポイントのまとめ

| 項目 | 誤った実装 | 正しい実装 |
|------|-----------|----------|
| タイムアウト指定 | `i2c_master_probe(h, addr, pdMS_TO_TICKS(50))` | `i2c_master_probe(h, addr, 50)` |
| flags初期化 | `.flags.enable_internal_pullup = true` | `.flags = { .enable_internal_pullup = true }` |
| バス設定 | 一部フィールドのみ | 全フィールド明示的に初期化 |

## まとめ

### 問題の本質

**タイムアウト値の二重変換**が根本原因である可能性が極めて高い：
- `i2c_master_probe()`関数は**内部で**`pdMS_TO_TICKS()`を実行する
- 呼び出し側で`pdMS_TO_TICKS(50)`を渡すと、実質0～1tickのタイムアウトになる
- これにより、全アドレスで即座にタイムアウトが発生していた

### 修正方法

**1行の修正で解決する可能性が高い:**
```c
// 修正前
i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(50))

// 修正後
i2c_master_probe(i2c_bus_handle, addr, 50)
```

### 新しいI2C Master APIの注意点

1. **タイムアウトパラメータ**: ミリ秒単位の整数値で指定（pdMS_TO_TICKSは不要）
2. **構造体初期化**: 全フィールドを明示的に初期化することを推奨
3. **プルアップ抵抗**: 内部プルアップは高速周波数では不十分、外部2-5kΩ推奨
4. **バスレベル設定**: クロック周波数はデバイス追加時に設定（バス初期化時ではない）
5. **エラー処理**: ESP_ERR_TIMEOUTはプルアップ抵抗の問題を示唆する

### 今後の開発指針

1. **まずStep 1の修正を適用**してテストする
2. 成功すれば、新しいAPIでStage 2以降を実装
3. 失敗する場合は、一時的に古いAPIに戻すことも検討
4. 古いAPIは動作実績があり、deprecation warning以外の問題はない

この調査により、新しいI2C master APIの正しい使用方法が明確になった。
