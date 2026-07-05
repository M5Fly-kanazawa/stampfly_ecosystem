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
 *
 * sf_board が所有する共有 I2C バス経由で VL53L3CX 底面 ToF を
 * 読み取り、sensor_tof トピックに距離計測を発行する。
 *
 * @publisher  sensor_tof
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
 *
 * Takeoff/Landing detection logic (TakeoffLandingMgr) runs in a
 * different layer and is wired in a later milestone (M4+) — this
 * task focuses on raw sensing only.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"   // front-ToF XSHUT hold-low / 前方ToF XSHUT固定

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

/// VL53L3CX bottom sensor wrapper. File-scope static — owned by this
/// task only, never shared (per @publisher annotation above).
/// VL53L3CX 底面センサのラッパー。ファイルスコープ static で本タスクが
/// 所有し、外部共有しない (上記 @publisher アノテーション参照)。
static stampfly::VL53L3CXWrapper g_tof_bottom;

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

void TofTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "TofTask started");

    // -------------------------------------------------------------------
    // Hold the (unsupported) FRONT ToF in reset before touching the bottom
    // sensor. Both VL53L3CX parts power up at the same I2C address 0x29; if
    // the front sensor is awake, the SetDeviceAddress below reaches BOTH and
    // the two sensors alias — ranging data becomes an interleaved mix. The
    // dual-sensor bring-up sequence (XSHUT-staggered re-addressing) belongs
    // here when front-ToF support lands.
    // 底面センサに触れる前に（未サポートの）前方 ToF をリセット保持する。VL53L3CX は
    // 2 個とも同じ I2C アドレス 0x29 で起動するため、前方が生きていると下の
    // SetDeviceAddress が両方に届いて 2 センサが混線し、測距データが交互に混ざる。
    // 前方 ToF 対応時は XSHUT を時差解除する 2 センサ起動手順をここに実装する。
    // -------------------------------------------------------------------
    gpio_config_t xshut_front = {};
    xshut_front.pin_bit_mask = 1ULL << config::GPIO_TOF_XSHUT_FRONT;
    xshut_front.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&xshut_front);
    gpio_set_level(static_cast<gpio_num_t>(config::GPIO_TOF_XSHUT_FRONT), 0);

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

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;

    while (true) {
        ++cycle_count;

        // Step 1: poll for new data
        // ステップ 1: 新規データの有無をポーリング
        bool ready = false;
        esp_err_t err = g_tof_bottom.isDataReady(ready);
        if (err != ESP_OK) {
            if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
                ESP_LOGW(TAG, "isDataReady failed: %s", esp_err_to_name(err));
                last_fail_log_cycle = cycle_count;
            }
            vTaskDelayUntil(&last_wake, kPeriodTicks);
            continue;
        }

        if (!ready) {
            // No new sample yet — sleep and retry on next cycle.
            // 新サンプル未到達 — スリープして次サイクルに再試行
            vTaskDelayUntil(&last_wake, kPeriodTicks);
            continue;
        }

        // Step 2: read distance and rearm interrupt
        // ステップ 2: 距離を読み、割り込みを再アーム
        stampfly::DistanceData reading{};
        err = g_tof_bottom.getDistance(reading);
        if (err == ESP_OK) {
            uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
            sf::sensor_tof.publish(buildTofTopic(reading, now_us));
            // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
            // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
            sf::internal::board::set_sensor_update(
                sf::internal::board::SensorId::BottomToF, now_us);
        } else if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "getDistance failed: %s", esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }

        // Required after each read to rearm the sensor for the next
        // measurement (per VL53L3CX driver contract).
        // 各読み取り後、次の計測用にセンサを再アームする必要がある
        // (VL53L3CX ドライバの契約)。
        g_tof_bottom.clearInterruptAndStartMeasurement();

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
