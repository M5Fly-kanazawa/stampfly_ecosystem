/**
 * @file cli_task.cpp
 * @brief CLIタスク - USBシリアルコマンドライン処理
 *
 * Note: Binary logging moved to stampfly_logger component (400Hz via ESP Timer)
 */

#include "tasks_common.hpp"

static const char* TAG = "CLITask";

using namespace config;
using namespace globals;

void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    // Print initial prompt
    g_cli.print("\r\n=== StampFly RTOS Skeleton ===\r\n");
    g_cli.print("Type 'help' for available commands\r\n");
    g_cli.print("> ");

    TickType_t last_teleplot = xTaskGetTickCount();
    TickType_t last_csvlog = xTaskGetTickCount();
    const TickType_t teleplot_period = pdMS_TO_TICKS(50);  // 20Hz teleplot output
    const TickType_t csvlog_period = pdMS_TO_TICKS(50);    // 20Hz CSV log output

    while (true) {
        if (g_cli.isInitialized()) {
            g_cli.processInput();

            TickType_t now = xTaskGetTickCount();

            // Output teleplot data at fixed interval
            if (g_cli.isTeleplotEnabled() && (now - last_teleplot) >= teleplot_period) {
                g_cli.outputTeleplot();
                last_teleplot = now;
            }

            // Output CSV log data at fixed interval
            if (g_cli.isLogEnabled() && (now - last_csvlog) >= csvlog_period) {
                g_cli.outputCSVLog();
                last_csvlog = now;
            }

            // Binary logging now handled by stampfly_logger component at 400Hz
            // controlled via g_logger.start()/stop()
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling (teleplot/csvlog only)
    }
}
