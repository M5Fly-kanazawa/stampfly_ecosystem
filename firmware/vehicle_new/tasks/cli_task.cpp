/**
 * @file cli_task.cpp
 * @brief CLI + parameter access task (20Hz)
 *        CLI + パラメータアクセスタスク（20Hz）
 *
 * Handles USB Serial and TCP CLI commands.
 * Provides parameter read/write interface.
 *
 * USBシリアルおよびTCP CLIコマンドを処理する。
 * パラメータ読み書きインターフェースを提供する。
 *
 * @design architecture.md §6 — CLITask: CLI + Parameters              [--]
 * @design detailed_design.md §8 — CLITask: 20Hz, priority 5          [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.hpp"

static const char* TAG = "CLITask";

void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  // 20Hz

    while (true) {
        // TODO: Process USB Serial input
        // TODO: Process TCP CLI input
        // TODO: Handle parameter get/set commands
        // TODO: Handle system commands (status, reboot, etc.)

        vTaskDelayUntil(&last_wake, period);
    }
}
