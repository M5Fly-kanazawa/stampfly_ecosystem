/**
 * @file cli_task.cpp
 * @brief CLI Task - Serial CLI using custom LineEditor
 *
 * CLIタスク - カスタムLineEditorを使用した Serial CLI
 *
 * Features:
 * - Visible cursor (ANSI escape sequences)
 * - Command history (up/down arrows)
 * - Tab completion for registered commands
 * - Line editing (cursor movement, backspace, delete)
 *
 * Note: Uses custom LineEditor instead of ESP-IDF standard linenoise
 *       to ensure cursor visibility on all terminals.
 */

#include "tasks_common.hpp"
#include "serial_cli.hpp"

static const char* TAG = "CLITask";

using namespace config;
using namespace globals;

void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started, waiting for boot complete...");

    // Wait for boot sequence to finish before showing prompt
    // ブートシーケンス完了までプロンプト表示を待機
    while (!g_boot_complete) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Get SerialCLI singleton instance
    // SerialCLIシングルトンインスタンスを取得
    auto& cli = stampfly::SerialCLI::getInstance();

    // Initialize Serial CLI
    // Serial CLIを初期化
    esp_err_t ret = cli.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Serial CLI: %s", esp_err_to_name(ret));
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "Serial CLI initialized, starting REPL loop");

    // Run CLI loop (blocking)
    // CLIループを実行（ブロッキング）
    cli.run();

    // Should never reach here
    // ここには到達しないはず
    ESP_LOGW(TAG, "CLI loop exited unexpectedly");
    vTaskDelete(nullptr);
}
