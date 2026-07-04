/**
 * @file console.cpp
 * @brief Unified Console Implementation using ESP-IDF Console
 *
 * ESP-IDF Console を使用した統合コマンドコンソール実装
 */

#include "console.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "Console";

// External command registration functions
// 外部コマンド登録関数
extern "C" {
void register_system_commands();
void register_sensor_commands();
void register_motor_commands();
void register_control_commands();
void register_comm_commands();
void register_calib_commands();
void register_misc_commands();
void register_flight_commands();
void register_query_commands();
void register_wifi_bench_command();
}

namespace stampfly {

// =============================================================================
// Thread-Local Storage for Output Redirection
// =============================================================================
// Each thread (Serial REPL, WiFi client, etc.) can have its own output function
// 各スレッド（Serial REPL、WiFiクライアント等）が独自の出力関数を持てる

static __thread Console::OutputFunc t_output_func = nullptr;
static __thread void* t_output_ctx = nullptr;

// =============================================================================
// Console Implementation
// =============================================================================

Console& Console::getInstance()
{
    static Console instance;
    return instance;
}

esp_err_t Console::init()
{
    if (initialized_) {
        ESP_LOGW(TAG, "Console already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Console::init() called");

    // Note: esp_console_init() is NOT called here.
    // esp_console_init() はここでは呼ばない。
    // It will be called by SerialREPL via esp_console_new_repl_usb_cdc()
    // SerialREPL が esp_console_new_repl_usb_cdc() を通じて呼ぶ

    initialized_ = true;
    ESP_LOGI(TAG, "Console initialized (commands can be registered after REPL init)");

    return ESP_OK;
}

void Console::registerAllCommands()
{
    if (!initialized_) {
        ESP_LOGW(TAG, "Console not initialized, cannot register commands");
        return;
    }

    ESP_LOGI(TAG, "Registering all commands");

    // Note: Custom help command is registered in register_system_commands()
    // カスタムhelpコマンドは register_system_commands() で登録される
    // (Uses Console::print() for WiFi CLI compatibility)
    // (WiFi CLI互換のため Console::print() を使用)

    // Register command groups
    // コマンドグループを登録
    register_system_commands();
    register_sensor_commands();
    register_motor_commands();
    register_control_commands();
    register_comm_commands();
    register_calib_commands();
    register_misc_commands();
    register_flight_commands();
    register_query_commands();
    register_wifi_bench_command();

    ESP_LOGI(TAG, "All commands registered");
}

int Console::run(const char* cmdline)
{
    if (!initialized_) {
        ESP_LOGW(TAG, "Console not initialized");
        return -1;
    }

    if (cmdline == nullptr || strlen(cmdline) == 0) {
        return 0;  // Empty command is OK
    }

    int ret;
    esp_err_t err = esp_console_run(cmdline, &ret);

    if (err == ESP_ERR_NOT_FOUND) {
        print("Unknown command: %s\r\n", cmdline);
        print("Type 'help' for available commands.\r\n");
        return -1;
    } else if (err == ESP_ERR_INVALID_ARG) {
        // Command was empty or whitespace only
        return 0;
    } else if (err != ESP_OK) {
        print("Error executing command: %s\r\n", esp_err_to_name(err));
        return -1;
    }

    return ret;
}

void Console::setOutput(OutputFunc func, void* ctx)
{
    t_output_func = func;
    t_output_ctx = ctx;
}

void Console::clearOutput()
{
    t_output_func = nullptr;
    t_output_ctx = nullptr;
}

void Console::print(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprint(fmt, args);
    va_end(args);
}

void Console::vprint(const char* fmt, va_list args)
{
    char buf[PRINT_BUF_SIZE];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);

    if (len < 0) {
        return;  // Format error
    }

    // Truncate if too long
    // 長すぎる場合は切り詰める
    if (len >= static_cast<int>(sizeof(buf))) {
        len = sizeof(buf) - 1;
        buf[len] = '\0';
    }

    // Output to redirected function or default stdout
    // リダイレクト先関数またはデフォルトのstdoutに出力
    if (t_output_func != nullptr) {
        t_output_func(buf, t_output_ctx);
    } else {
        printf("%s", buf);
        fflush(stdout);
    }
}

}  // namespace stampfly
