/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file mag_task.cpp
 * @brief Magnetometer sensor task (25Hz)
 *        地磁気センサタスク（25Hz）
 *
 * Reads BMM150 magnetometer data and publishes to sensor.mag topic.
 * BMM150地磁気データを読み取り、sensor.magトピックに発行する。
 *
 * @design architecture.md §6 — MagTask: Sensing(Mag)                  [--]
 * @design detailed_design.md §8 — MagTask: 25Hz, priority 18         [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
// TODO: #include "bmm150.hpp"
#include "config.hpp"

static const char* TAG = "MagTask";

// TODO: BMM150 driver instance

void MagTask(void* pvParameters)
{
    ESP_LOGI(TAG, "MagTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(40);  // 25Hz

    while (true) {
        sf::MagData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        // TODO: Wire BMM150 API
        // TODO: BMM150 APIを結合

        sf::sensor_mag.publish(data);
        vTaskDelayUntil(&last_wake, period);
    }
}
