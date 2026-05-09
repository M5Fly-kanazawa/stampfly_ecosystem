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
#include "driver/spi_master.h"
#include "driver/ledc.h"
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
// IMU SPI bus (shared by BMI270 and PMW3901 OptFlow)
// IMU SPI バス (BMI270 と PMW3901 OptFlow で共有)
// ---------------------------------------------------------------------------
//
// ピン配置は M5StampFly 公式ハードウェアで固定。BMI270Wrapper の
// defaultStampFly() と同じ値を使う必要がある。BMI270 と PMW3901 が
// 同じバスを共有し、CS は別々 (BMI270=GPIO46, PMW3901=GPIO12)。
//
// Pin assignments are fixed by M5StampFly PCB. Must match
// BMI270Wrapper::Config::defaultStampFly(). BMI270 (CS=GPIO46) and
// PMW3901 (CS=GPIO12) share this bus with separate chip-select lines.
constexpr spi_host_device_t kImuSpiHost = SPI2_HOST;
constexpr gpio_num_t        kSpiMosi    = GPIO_NUM_14;
constexpr gpio_num_t        kSpiMiso    = GPIO_NUM_43;
constexpr gpio_num_t        kSpiSclk    = GPIO_NUM_44;

// BMI270 config_file は最大 8KB。max_transfer_sz は config_file_size + header
// で十分。ここでは BMI270 ドライバの BMI270_CONFIG_FILE_SIZE と整合する
// 12KB (8192 + alignment margin) にする。
//
// BMI270 firmware config blob is 8 KiB. max_transfer_sz must accommodate
// it plus a small header. 12 KiB matches sf_hal_bmi270's BMI270_CONFIG_FILE_SIZE.
constexpr int kSpiMaxTransferSize = 12288;

// ---------------------------------------------------------------------------
// Motor LEDC timer
// モータ LEDC タイマー
// ---------------------------------------------------------------------------
//
// MotorDriver と整合する設定。150kHz / 8bit は M5StampFly モータの
// 標準設定 (sf_actuator::actuator.cpp と同値)。
//
// Matches sf_actuator/MotorDriver: 150 kHz / 8-bit, low-speed mode for
// ESP32-S3 (high-speed mode does not exist on S3).
constexpr ledc_timer_t      kMotorTimer        = LEDC_TIMER_0;
constexpr ledc_mode_t       kMotorSpeedMode    = LEDC_LOW_SPEED_MODE;
constexpr int               kMotorPwmFreqHz    = 150000;
constexpr ledc_timer_bit_t  kMotorPwmResolution = LEDC_TIMER_8_BIT;

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

// ---------------------------------------------------------------------------
// Step: SPI bus (IMU + OptFlow)
// ステップ: SPI バス (IMU + OptFlow)
// ---------------------------------------------------------------------------
//
// sf_board が SPI bus を所有し、BMI270Wrapper と PMW3901Wrapper は
// spi_bus_add_device() のみ自身で行う。BMI270 ドライバ
// (sf_hal_bmi270/src/bmi270_spi.c) は spi_bus_initialize() の
// ESP_ERR_INVALID_STATE を許容するパッチが入っており、二重初期化を
// 検出して device add のみ進む。
//
// sf_board owns the SPI bus; BMI270Wrapper and PMW3901Wrapper only call
// spi_bus_add_device(). The BMI270 C driver tolerates ESP_ERR_INVALID_STATE
// from spi_bus_initialize() (already initialized) and proceeds to add_device.
esp_err_t init_spi_bus()
{
    spi_bus_config_t cfg = {};
    cfg.mosi_io_num     = kSpiMosi;
    cfg.miso_io_num     = kSpiMiso;
    cfg.sclk_io_num     = kSpiSclk;
    cfg.quadwp_io_num   = -1;
    cfg.quadhd_io_num   = -1;
    cfg.max_transfer_sz = kSpiMaxTransferSize;
    cfg.flags           = SPICOMMON_BUSFLAG_MASTER;
    return spi_bus_initialize(kImuSpiHost, &cfg, SPI_DMA_CH_AUTO);
}

// ---------------------------------------------------------------------------
// Step: LEDC motor timer
// ステップ: LEDC モータタイマー
// ---------------------------------------------------------------------------
//
// MotorDriver は ledc_channel_config() のみ自身で行う。ledc_timer_config()
// を MotorDriver 側でも呼ぶ可能性があるが、ESP-IDF は同じ timer 番号への
// 重複呼び出しを reconfiguration として扱うため衝突しない。それでも、
// 「物理的所有は sf_board」「channel 設定は MotorDriver」の責務分離を
// 明確化する目的で、ここで先に config しておく。
//
// MotorDriver only calls ledc_channel_config(). Even if it also calls
// ledc_timer_config() on the same timer number, ESP-IDF treats this as
// reconfiguration (no conflict). We pre-configure here to make ownership
// explicit per v3 design (R1).
esp_err_t init_motor_timer()
{
    ledc_timer_config_t cfg = {};
    cfg.speed_mode      = kMotorSpeedMode;
    cfg.duty_resolution = kMotorPwmResolution;
    cfg.timer_num       = kMotorTimer;
    cfg.freq_hz         = static_cast<uint32_t>(kMotorPwmFreqHz);
    cfg.clk_cfg         = LEDC_AUTO_CLK;
    cfg.deconfigure     = false;
    return ledc_timer_config(&cfg);
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

    // Level 2: SPI bus (shared by IMU and OptFlow)
    err = init_spi_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "  L2: SPI bus ready (host=%d, MOSI=%d, MISO=%d, SCLK=%d)",
             kImuSpiHost, kSpiMosi, kSpiMiso, kSpiSclk);

    // Level 3: LEDC motor timer
    err = init_motor_timer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC motor timer init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "  L3: LEDC motor timer ready (timer=%d, freq=%d Hz, %d-bit)",
             kMotorTimer, kMotorPwmFreqHz, kMotorPwmResolution);

    g_initialized = true;
    ESP_LOGI(TAG, "Phase 1 complete: BSP ready");
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus()
{
    return g_i2c_bus;
}

spi_host_device_t imu_spi()
{
    return kImuSpiHost;
}

ledc_timer_t motor_timer()
{
    return kMotorTimer;
}

ledc_mode_t motor_speed_mode()
{
    return kMotorSpeedMode;
}

bool sensor_present(SensorId /*id*/)
{
    // Reserved for M2b. Always returns false until the per-sensor
    // presence-tracking infrastructure is added.
    // M2b で実装予定。それまでは常に false を返す。
    return false;
}

}  // namespace sf::internal::board
