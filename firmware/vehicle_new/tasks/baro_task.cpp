/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file baro_task.cpp
 * @brief Barometer sensor task (50Hz)
 *        気圧センサタスク（50Hz）
 *
 * Reads BMP280 barometer data and publishes to sensor.baro topic.
 * BMP280気圧データを読み取り、sensor.baroトピックに発行する。
 *
 * @design architecture.md §6 — BaroTask: Sensing(Baro)               [--]
 * @design detailed_design.md §8 — BaroTask: 50Hz, priority 16        [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
// TODO: #include "bmp280.hpp"
#include "config.hpp"

static const char* TAG = "BaroTask";

// TODO: BMP280 driver instance

void BaroTask(void* pvParameters)
{
    ESP_LOGI(TAG, "BaroTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        sf::BaroData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        // TODO: Wire BMP280 API
        // TODO: BMP280 APIを結合

        sf::sensor_baro.publish(data);
        vTaskDelayUntil(&last_wake, period);
    }
}
