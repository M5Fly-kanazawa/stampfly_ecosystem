/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/getting-started-with-stampfly-ecosystem
 */

/**
 * @file board.cpp
 * @brief BSP implementation — owns I2C bus and (future) other shared HW
 *        BSP 実装 — I2C バスと (将来の) 他共有 HW を所有
 *
 * @design hardware_init.md §3 — sf_board の責務               [--]
 * @design hardware_init.md §4 — 起動シーケンス                [--]
 * @design hardware_init.md §5 — 失敗 3 段階分類               [--]
 */

#include "sf_board.hpp"

#include "esp_log.h"
#include "esp_event.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

namespace sf::internal::board {

namespace {

constexpr const char* TAG = "sf_board";

// ---------------------------------------------------------------------------
// StampFly hardware pin configuration
// StampFly のハードウェアピン構成
// ---------------------------------------------------------------------------
//
// I2C bus is shared by BMP280 (baro), BMM150 (mag), VL53L3CX (ToF),
// and INA3221 (power monitor). SDA=GPIO3, SCL=GPIO4 are fixed by the
// StampFly PCB design.
//
// I2C バスは BMP280 / BMM150 / VL53L3CX / INA3221 で共有する。
// SDA=GPIO3、SCL=GPIO4 は StampFly 基板で固定。

constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t     kI2cSda  = GPIO_NUM_3;
constexpr gpio_num_t     kI2cScl  = GPIO_NUM_4;

// ---------------------------------------------------------------------------
// Singleton state
// シングルトン状態
// ---------------------------------------------------------------------------
//
// File-scope statics are intentional: sf_board is the *one* place that owns
// these handles. Exposing them via getters (i2c_bus()) keeps the singleton
// pattern explicit but free of extern globals.
//
// ファイルスコープの static は意図的なもの。sf_board だけがハンドルを
// 所有し、getter (i2c_bus()) 経由で公開することでシングルトンを明示
// しつつ extern グローバルを避ける。

i2c_master_bus_handle_t g_i2c_bus = nullptr;
bool                    g_initialized = false;

// ---------------------------------------------------------------------------
// Step: default event loop
// ステップ: デフォルトイベントループ
// ---------------------------------------------------------------------------
//
// Required by WiFi / netif (used later by sf_comm and sf_telemetry). It is
// safe to create here even if WiFi is not used in a given build, because
// esp_event_loop_create_default() reserves only minimal resources.
//
// WiFi / netif (後で sf_comm と sf_telemetry が使う) で必要。WiFi を
// 使わないビルドでも esp_event_loop_create_default() は最小資源で済む
// ので、ここで生成しておいて問題ない。
esp_err_t init_event_loop()
{
    esp_err_t err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE) {
        // Already created (e.g. by an earlier sf_comm in legacy mode).
        // 既に生成済み (旧 sf_comm が先に呼んだ場合など)。問題なし。
        ESP_LOGI(TAG, "default event loop already exists");
        return ESP_OK;
    }
    return err;
}

// ---------------------------------------------------------------------------
// Step: I2C master bus
// ステップ: I2C マスターバス
// ---------------------------------------------------------------------------
//
// glitch_ignore_cnt = 7 and internal pullup are the StampFly proven
// settings (also used in examples/05_read_tof). Multiple devices share
// this bus; each HAL adds itself with i2c_master_bus_add_device().
//
// glitch_ignore_cnt = 7 と内部プルアップは StampFly で実績のある設定
// (examples/05_read_tof でも使用)。複数デバイスがこのバスを共有し、
// 各 HAL は i2c_master_bus_add_device() で自身を追加する。
esp_err_t init_i2c_bus()
{
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port                   = kI2cPort;
    cfg.sda_io_num                 = kI2cSda;
    cfg.scl_io_num                 = kI2cScl;
    cfg.clk_source                 = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt          = 7;
    cfg.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&cfg, &g_i2c_bus);
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// 公開 API
// ---------------------------------------------------------------------------

esp_err_t init()
{
    if (g_initialized) {
        // init() called twice. Treat as logic error and return without
        // re-initialising hardware (which would leak handles).
        // init() の二重呼び出し。ハンドルリークを避けるため再初期化しない。
        ESP_LOGW(TAG, "init() called twice — ignored");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Phase 1 boot: initializing shared HW");

    // Level 0: pre-kernel resources
    esp_err_t err = init_event_loop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "default event loop init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "  L0: default event loop ready");

    // Level 1: I2C master bus
    err = init_i2c_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "  L1: I2C bus ready (port=%d, SDA=%d, SCL=%d)",
             kI2cPort, kI2cSda, kI2cScl);

    // Level 2-3 (SPI host, LEDC timer) will be added in M2b when imu_task
    // and sf_actuator are migrated to board-managed handles.
    // Level 2-3 (SPI host、LEDC タイマー) は M2b で imu_task と sf_actuator
    // を board 経由に書き換えるときに追加する。

    g_initialized = true;
    ESP_LOGI(TAG, "Phase 1 complete: BSP ready");
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus()
{
    return g_i2c_bus;
}

bool sensor_present(SensorId /*id*/)
{
    // Reserved for M2b. Always returns false until the per-sensor
    // presence-tracking infrastructure is added.
    // M2b で実装予定。それまでは常に false を返す。
    return false;
}

}  // namespace sf::internal::board
