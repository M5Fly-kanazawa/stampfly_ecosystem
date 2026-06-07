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

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
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

// Optional-sensor presence flags, indexed by SensorId. Written once per sensor
// by its task setup (set_sensor_present), read by Failsafe / Telemetry. atomic
// so the cross-task read needs no lock. Size tracks the SensorId enum.
// Optional センサの存在フラグ。SensorId で索引。各センサの setup が1回書き込み
// (set_sensor_present)、Failsafe / Telemetry が読む。クロスタスク読み取りを
// ロック無しにするため atomic。サイズは SensorId enum に追従。
constexpr size_t kSensorSlots = 5;  // Mag, FrontToF, Flow, Baro, Power
std::atomic<bool> g_sensor_present[kSensorSlots] = {};

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
// Step: TCP/IP stack
// ステップ: TCP/IP スタック
// ---------------------------------------------------------------------------
//
// esp_netif_init() initialises the lwIP TCP/IP stack. WiFi (sf_comm) and
// UDP socket creation (sf_telemetry) both need this to be up before they
// run. Idempotent semantics: ESP_ERR_INVALID_STATE on re-call is benign.
//
// esp_netif_init() は lwIP TCP/IP スタックを初期化する。WiFi (sf_comm) と
// UDP ソケット生成 (sf_telemetry) の両方がこの上に乗る。再呼び出しは
// ESP_ERR_INVALID_STATE が返るが副作用なし (idempotent)。
esp_err_t init_netif()
{
    esp_err_t err = esp_netif_init();
    if (err == ESP_ERR_INVALID_STATE) {
        // Already initialised (legacy code path). Safe to ignore.
        // 既に初期化済み (旧コード経路)。無視して問題なし。
        ESP_LOGI(TAG, "TCP/IP stack already initialized");
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
// sf_board が SPI bus を所有し (R1)、BMI270Wrapper と PMW3901Wrapper は
// spi_bus_add_device() のみ自身で行う。両ドライバとも Config の
// skip_bus_init=true を受け取り、自前の spi_bus_initialize() を省く。
// (二重初期化の "握り潰し" ではなく、所有権を明示した省略。)
//
// sf_board owns the SPI bus (R1); BMI270Wrapper and PMW3901Wrapper only call
// spi_bus_add_device(). Both receive skip_bus_init=true in their Config and
// skip their own spi_bus_initialize() — an explicit ownership-driven skip,
// not a swallowed double-init.
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

// ---------------------------------------------------------------------------
// Critical-failure handling (R4, hardware_init.md §5)
// Critical 失敗の処理 (R4, hardware_init.md §5)
// ---------------------------------------------------------------------------
//
// A Critical shared-HW failure (bus/timer/infra) means the vehicle cannot fly.
// We must NOT continue: log the cause and halt (do NOT esp_restart, so the cause
// stays observable and a future LED pattern stays lit — §5). The halt loop uses
// vTaskDelay so the idle task keeps feeding the watchdog (no reboot). board::init
// runs in app_main before any flight task starts, so halting here stops the
// whole bring-up cleanly.
//
// 共有 HW の Critical 失敗 (バス/タイマ/インフラ) は飛行不能を意味する。続行禁止:
// 原因をログして停止する (esp_restart しない＝原因と将来の LED 表示を保つ, §5)。
// 停止ループは vTaskDelay を使い idle タスクが WDT を食わせ続ける (リブート無し)。
// board::init は飛行タスク開始前の app_main で走るため、ここで停止すれば
// ブリングアップ全体を綺麗に止められる。
enum class FatalReason {
    EventLoop,   ///< default event loop / イベントループ
    Netif,       ///< TCP/IP stack / TCP/IP スタック
    I2cBus,      ///< shared I2C master bus / 共有 I2C マスターバス
    SpiBus,      ///< shared SPI bus (IMU + Flow) / 共有 SPI バス
    LedcTimer,   ///< motor LEDC timer / モータ LEDC タイマー
};

const char* fatalReasonText(FatalReason r)
{
    switch (r) {
        case FatalReason::EventLoop: return "default event loop init";
        case FatalReason::Netif:     return "TCP/IP stack init";
        case FatalReason::I2cBus:    return "I2C master bus init";
        case FatalReason::SpiBus:    return "SPI bus init";
        case FatalReason::LedcTimer: return "LEDC motor timer init";
    }
    return "unknown";
}

[[noreturn]] void fatal(FatalReason reason, esp_err_t err)
{
    ESP_LOGE(TAG, "CRITICAL: %s failed: %s — vehicle cannot fly. Halting.",
             fatalReasonText(reason), esp_err_to_name(err));

    // LED error pattern (hardware_init.md §5): wired in Phase 6 when LED/notify
    // ownership is established. Until then the cause is reported on the log.
    // LED エラーパターン (hardware_init.md §5) は Phase 6 (LED/notify 所有確立時)
    // で配線する。それまでは原因をログで報告する。

    for (;;) {
        vTaskDelay(portMAX_DELAY);  // Halt; never returns. / 停止。戻らない。
    }
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

    // Level 0: pre-kernel resources (event loop + TCP/IP stack). All Critical:
    // every networked service depends on them — a failure halts via fatal().
    // Level 0: 全て Critical。全ネットワークサービスが依存するため失敗は fatal() で停止。
    esp_err_t err = init_event_loop();
    if (err != ESP_OK) {
        fatal(FatalReason::EventLoop, err);
    }
    err = init_netif();
    if (err != ESP_OK) {
        fatal(FatalReason::Netif, err);
    }
    ESP_LOGI(TAG, "  L0: event loop + TCP/IP stack ready");

    // Level 1: I2C master bus (Critical — every I2C HAL depends on it).
    // Level 1: I2C マスターバス (Critical — 全 I2C HAL が依存)。
    err = init_i2c_bus();
    if (err != ESP_OK) {
        fatal(FatalReason::I2cBus, err);
    }
    ESP_LOGI(TAG, "  L1: I2C bus ready (port=%d, SDA=%d, SCL=%d)",
             kI2cPort, kI2cSda, kI2cScl);

    // Level 2: SPI bus (shared by IMU and OptFlow; Critical — the IMU needs it).
    // Level 2: SPI バス (IMU + OptFlow 共有; Critical — IMU が依存)。
    err = init_spi_bus();
    if (err != ESP_OK) {
        fatal(FatalReason::SpiBus, err);
    }
    ESP_LOGI(TAG, "  L2: SPI bus ready (host=%d, MOSI=%d, MISO=%d, SCLK=%d)",
             kImuSpiHost, kSpiMosi, kSpiMiso, kSpiSclk);

    // Level 3: LEDC motor timer (Critical — no timer, no motor PWM).
    // Level 3: LEDC モータタイマー (Critical — タイマ無しでは PWM 出力不能)。
    err = init_motor_timer();
    if (err != ESP_OK) {
        fatal(FatalReason::LedcTimer, err);
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

spi_host_device_t flow_spi()
{
    // Flow shares the IMU's physical SPI bus (separate CS). Same host on purpose.
    // フローは IMU と物理 SPI バスを共有 (CS は別)。意図的に同一ホストを返す。
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

void set_sensor_present(SensorId id, bool present)
{
    const size_t idx = static_cast<size_t>(id);
    if (idx >= kSensorSlots) {
        return;  // Out of range — ignore (defensive). / 範囲外は無視 (防御的)。
    }
    g_sensor_present[idx].store(present, std::memory_order_relaxed);
}

bool sensor_present(SensorId id)
{
    const size_t idx = static_cast<size_t>(id);
    if (idx >= kSensorSlots) {
        return false;
    }
    return g_sensor_present[idx].load(std::memory_order_relaxed);
}

}  // namespace sf::internal::board
