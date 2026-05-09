/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file mag_task.cpp
 * @brief Magnetometer sensor task (25Hz) — BMM150 over shared I2C
 *        地磁気センサタスク (25Hz) — 共有 I2C 経由の BMM150
 *
 * Reads BMM150 magnetometer (3-axis micro-Tesla) via the shared I2C bus
 * owned by sf_board, and publishes to sensor_mag topic.
 *
 * sf_board が所有する共有 I2C 経由で BMM150 (3 軸 µT) を読み、
 * sensor_mag トピックに発行する。
 *
 * @publisher  sensor_mag
 *
 * @design architecture.md §6 — MagTask: Sensing(Mag)                  [OK]
 * @design detailed_design.md §8 — MagTask: 25Hz, priority 18         [OK]
 * @design hardware_init.md §3 — sf_board が i2c_bus を所有 (R1)      [OK]
 * @design topic_reference.md §3 — sensor_mag: Queue 2, 25Hz          [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - BMM150 init failure → Optional. ヨー推定がドリフトしやすくなる
 *     が ACRO/STAB/ALT/POS は ESKF + IMU で動く。abort せず deletion
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "bmm150.hpp"

static const char* TAG = "MagTask";

/// Cycle period [ticks]: 40ms = 25Hz
/// 周期 [tick]: 40ms = 25Hz
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(40);

/// Read-failure log throttle: at 25Hz, ~5s between warnings
/// 読み取り失敗ログ抑制: 25Hz で約 5 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 125;

/// BMM150 wrapper — task-local file-scope static (no extern global).
/// BMM150 ラッパー — タスクローカルなファイルスコープ static
static stampfly::BMM150 g_mag;

/// Convert wrapper output to topic format.
/// ラッパー出力をトピック形式に変換。
static sf::MagData toTopic(const stampfly::MagData& src)
{
    sf::MagData out{};
    out.mag[0]    = src.x;
    out.mag[1]    = src.y;
    out.mag[2]    = src.z;
    out.timestamp = src.timestamp_us;
    return out;
}

void MagTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "MagTask started");

    // -------------------------------------------------------------------
    // Setup: borrow I2C bus from BSP and initialise BMM150
    // セットアップ: BSP から I2C バスを借用し BMM150 を初期化
    // -------------------------------------------------------------------
    stampfly::BMM150::Config cfg{};
    cfg.i2c_bus = sf::internal::board::i2c_bus();
    // data_rate / preset は Config の既定値 (ODR_10HZ, REGULAR) を使う。
    // タスク周期 25Hz に対しセンサ ODR が遅いため、data_ready=false が
    // 返るサイクルでは publish しない設計とする。
    //
    // Default data_rate=ODR_10HZ + preset=REGULAR. Sensor ODR is slower
    // than task period, so cycles with data_ready=false skip publish.

    esp_err_t err = g_mag.init(cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMM150 init failed: %s — Mag disabled (Optional)",
                 esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "BMM150 ready (25Hz)");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;

    while (true) {
        ++cycle_count;

        stampfly::MagData reading{};
        err = g_mag.read(reading);
        if (err == ESP_OK && reading.data_ready) {
            sf::sensor_mag.publish(toTopic(reading));
        } else if (err != ESP_OK &&
                   cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "BMM150 read failed: %s", esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }
        // data_ready=false は無声スキップ (sensor ODR < task rate の正常状況)。
        // Silently skip when data_ready=false (normal: ODR < task period).

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
