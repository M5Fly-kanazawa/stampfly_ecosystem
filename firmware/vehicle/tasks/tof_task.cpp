/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tof_task.cpp
 * @brief ToF sensor reading task (30Hz)
 *        ToFセンサ読み取りタスク（30Hz）
 *
 * Reads VL53L3CX bottom ToF data via the shared I2C bus owned by
 * sf_board, and publishes distance measurements to sensor_tof topic.
 * The forward-facing ToF (optional, log only) is read in the same cycle,
 * AFTER the bottom sensor, and published to sensor_tof_front.
 *
 * sf_board が所有する共有 I2C バス経由で VL53L3CX 底面 ToF を
 * 読み取り、sensor_tof トピックに距離計測を発行する。
 * 前方 ToF（任意センサ・ログ専用）は同じ周期内で底面の「後」に読み、
 * sensor_tof_front トピックに発行する。
 *
 * @publisher  sensor_tof, sensor_tof_front
 *
 * @design architecture.md §6 — TofTask: Sensing(ToF)                  [OK]
 * @design detailed_design.md §8 — TofTask: 30Hz, priority 14         [OK]
 * @design hardware_init.md §3 — sf_board が i2c_bus を所有 (R1)      [OK]
 * @design topic_reference.md §3 — sensor_tof: Queue 2, 30Hz          [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - VL53L3CX init failure for **bottom** sensor → Critical for
 *     ALT/POS modes. Currently logged as warning and task aborts;
 *     escalation to abort() is pending the failure handling story
 *     (M5 で sensor_health Topic と一緒に整備)
 *   - VL53L3CX init failure for **front** sensor → Optional. Logged as
 *     a warning, XSHUT is held low, bottom sensing continues unchanged.
 *     前方センサの初期化失敗は Optional 扱い: 警告のみ、XSHUT を Low に保ち
 *     底面の測距は従来どおり継続する。
 *
 * Takeoff/Landing detection logic (TakeoffLandingMgr) runs in a
 * different layer and is wired in a later milestone (M4+) — this
 * task focuses on raw sensing only.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"   // front-ToF XSHUT control / 前方ToF XSHUT制御

#include "topics.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "vl53l3cx_wrapper.hpp"

static const char* TAG = "TofTask";

/// Cycle period [ticks]: 33ms ≈ 30Hz, matching VL53L3CX timing budget.
/// 周期 [tick]: 33ms ≈ 30Hz、VL53L3CX のタイミング予算と整合
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(33);

/// Throttle interval for read-failure warnings.
/// At 30Hz, log at most once every ~3 seconds during persistent failure.
/// 読み取り失敗ログの抑制間隔。30Hz で持続的失敗時は約 3 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 90;

/// Front sensor I2C address after re-addressing (bottom is 0x30). Both parts boot at
/// 0x29, so the front one is moved only AFTER the bottom one has left 0x29.
/// 前方センサの付け替え後 I2C アドレス（底面は 0x30）。両センサとも 0x29 で起動するため、
/// 底面が 0x29 を離れた「後」に前方を付け替える。
static constexpr uint8_t kFrontI2cAddress = stampfly::VL53L3CXWrapper::FRONT_I2C_ADDR;

/// VL53L3CX bottom sensor wrapper. File-scope static — owned by this
/// task only, never shared (per @publisher annotation above).
/// VL53L3CX 底面センサのラッパー。ファイルスコープ static で本タスクが
/// 所有し、外部共有しない (上記 @publisher アノテーション参照)。
static stampfly::VL53L3CXWrapper g_tof_bottom;

/// VL53L3CX front sensor wrapper (optional; used only when g_front_present is true).
/// VL53L3CX 前方センサのラッパー（任意。g_front_present が true のときのみ使用）。
static stampfly::VL53L3CXWrapper g_tof_front;
static bool g_front_present = false;

/// Convert wrapper distance reading to sf::TofData topic format.
/// Conversion: mm → m, range_status==0 means valid measurement.
/// ラッパーの距離値を sf::TofData トピック形式に変換する。
/// 変換: mm → m、range_status==0 を有効計測とする。
static sf::TofData buildTofTopic(const stampfly::DistanceData& src,
                                  uint32_t now_us)
{
    sf::TofData out{};
    out.timestamp = now_us;
    out.distance  = static_cast<float>(src.distance_mm) * 1e-3f;
    out.status    = src.range_status;
    out.valid     = (src.range_status == 0);
    return out;
}

/// Hold the front ToF in reset (XSHUT low) so it cannot answer at 0x29.
/// 前方 ToF をリセット保持（XSHUT Low）し、0x29 に応答させない。
static void holdFrontInReset()
{
    gpio_config_t xshut_front = {};
    xshut_front.pin_bit_mask = 1ULL << config::GPIO_TOF_XSHUT_FRONT;
    xshut_front.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&xshut_front);
    gpio_set_level(static_cast<gpio_num_t>(config::GPIO_TOF_XSHUT_FRONT), 0);
}

/// Bring up the optional front sensor. Call only after the bottom sensor has been
/// moved off 0x29. On failure XSHUT is returned to low and bottom sensing is untouched.
/// 任意の前方センサを起動する。底面が 0x29 を離れた後にのみ呼ぶこと。失敗時は
/// XSHUT を Low に戻し、底面の測距には影響しない。
static void initFrontTof()
{
    auto cfg = stampfly::VL53L3CXWrapper::Config::defaultFront(
        sf::internal::board::i2c_bus());
    cfg.xshut_pin = static_cast<gpio_num_t>(config::GPIO_TOF_XSHUT_FRONT);
    cfg.i2c_addr  = kFrontI2cAddress;

    bool ok = (g_tof_front.init(cfg) == ESP_OK) && (g_tof_front.startRanging() == ESP_OK);
    if (!ok) {
        ESP_LOGW(TAG, "VL53L3CX front not detected (optional sensor) — disabled");
        holdFrontInReset();
        return;
    }
    g_front_present = true;
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::FrontToF, true);
    ESP_LOGI(TAG, "VL53L3CX front ready (addr 0x%02X, ~30Hz)", kFrontI2cAddress);
}

/// Poll one sensor once: if a sample is ready, read it, publish it to `topic`, stamp the
/// BSP freshness for `sensor_id`, and rearm the sensor. Read failures are log-throttled
/// per sensor through `last_fail_log_cycle`.
/// センサ 1 個を 1 回ポーリングする。新サンプルがあれば読み、`topic` に発行し、
/// `sensor_id` の鮮度を BSP に記録し、センサを再アームする。読み取り失敗のログは
/// `last_fail_log_cycle` でセンサ毎に抑制する。
static void pollSensor(stampfly::VL53L3CXWrapper& sensor,
                       sf::Topic<sf::TofData, sf::Queue, 2>& topic,
                       sf::internal::board::SensorId sensor_id,
                       const char* label,
                       uint32_t cycle_count,
                       uint32_t& last_fail_log_cycle)
{
    // Step 1: poll for new data / ステップ 1: 新規データの有無をポーリング
    bool ready = false;
    esp_err_t err = sensor.isDataReady(ready);
    if (err != ESP_OK) {
        if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "%s isDataReady failed: %s", label, esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }
        return;
    }
    if (!ready) {
        // No new sample yet — retry on next cycle.
        // 新サンプル未到達 — 次サイクルに再試行
        return;
    }

    // Step 2: read distance and publish / ステップ 2: 距離を読んで発行
    stampfly::DistanceData reading{};
    err = sensor.getDistance(reading);
    if (err == ESP_OK) {
        uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
        topic.publish(buildTofTopic(reading, now_us));
        // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
        // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
        sf::internal::board::set_sensor_update(sensor_id, now_us);
    } else if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
        ESP_LOGW(TAG, "%s getDistance failed: %s", label, esp_err_to_name(err));
        last_fail_log_cycle = cycle_count;
    }

    // Required after each read to rearm the sensor for the next
    // measurement (per VL53L3CX driver contract).
    // 各読み取り後、次の計測用にセンサを再アームする必要がある
    // (VL53L3CX ドライバの契約)。
    sensor.clearInterruptAndStartMeasurement();
}

void TofTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "TofTask started");

    // -------------------------------------------------------------------
    // Two-sensor bring-up. Both VL53L3CX parts power up at the same I2C address
    // 0x29, so they must never be awake together while one is being re-addressed:
    //   1. Hold FRONT in reset, then bring up BOTTOM (0x29 -> 0x30).
    //   2. Only then release FRONT (0x29 -> 0x31), as an optional sensor.
    // 2 センサ起動手順。VL53L3CX は 2 個とも同じ I2C アドレス 0x29 で起動するため、
    // 片方の付け替え中にもう片方が起きていてはならない:
    //   1. 前方をリセット保持したまま底面を起動する（0x29 -> 0x30）。
    //   2. その後に前方を解除する（0x29 -> 0x31）。前方は任意センサ。
    // -------------------------------------------------------------------
    holdFrontInReset();

    // -------------------------------------------------------------------
    // Setup: borrow I2C bus from BSP and initialise the VL53L3CX driver
    // セットアップ: BSP から I2C バスを借用し VL53L3CX ドライバを初期化
    // -------------------------------------------------------------------
    auto cfg = stampfly::VL53L3CXWrapper::Config::defaultBottom(
        sf::internal::board::i2c_bus()
    );

    esp_err_t init_result = g_tof_bottom.init(cfg);
    if (init_result != ESP_OK) {
        ESP_LOGE(TAG, "VL53L3CX bottom init failed: %s — task aborting",
                 esp_err_to_name(init_result));
        sf::internal::board::set_sensor_present(
            sf::internal::board::SensorId::BottomToF, false);
        vTaskDelete(NULL);
        return;
    }

    // Begin continuous ranging (≈30Hz, matching timing_budget_ms=33).
    // 連続測距を開始 (≈30Hz、timing_budget_ms=33 に整合)。
    esp_err_t range_result = g_tof_bottom.startRanging();
    if (range_result != ESP_OK) {
        ESP_LOGE(TAG, "VL53L3CX startRanging failed: %s — task aborting",
                 esp_err_to_name(range_result));
        sf::internal::board::set_sensor_present(
            sf::internal::board::SensorId::BottomToF, false);
        vTaskDelete(NULL);
        return;
    }
    // ToF up and ranging — report presence for the sensor_health snapshot (R15).
    // ToF 起動・測距開始 — sensor_health 用に presence を報告 (R15)。
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::BottomToF, true);
    ESP_LOGI(TAG, "VL53L3CX bottom ready (continuous ranging at ~30Hz)");

    // Bottom has left 0x29, so the optional front sensor can now be released safely.
    // 底面が 0x29 を離れたので、任意の前方センサを安全に解除できる。
    initFrontTof();

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;
    uint32_t last_front_fail_log_cycle = 0;

    while (true) {
        ++cycle_count;

        // Bottom first, so its timing is unaffected by the front sensor's I2C traffic.
        // 底面を先に処理し、前方センサの I2C 通信が底面のタイミングに影響しないようにする。
        pollSensor(g_tof_bottom, sf::sensor_tof,
                   sf::internal::board::SensorId::BottomToF, "bottom",
                   cycle_count, last_fail_log_cycle);
        if (g_front_present) {
            pollSensor(g_tof_front, sf::sensor_tof_front,
                       sf::internal::board::SensorId::FrontToF, "front",
                       cycle_count, last_front_fail_log_cycle);
        }

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
