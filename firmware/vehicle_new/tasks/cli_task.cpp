/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file cli_task.cpp
 * @brief CLI + parameter access task (USB-CDC serial console)
 *        CLI + パラメータアクセスタスク（USB-CDC シリアルコンソール）
 *
 * Owns the interactive serial console. Registers a small command set via the
 * ESP-IDF console registry — `param` (list/get/set/save), `status`, `reboot` — then
 * starts the USB-CDC REPL, which reads the serial line in its own task. Commands are
 * held in a static {name, help, func} table and registered in a loop (R6: registry
 * pattern, no extern global console pointer). The CLI is a ground/bring-up tool and
 * is not on the flight-critical path.
 *
 * 対話式シリアルコンソールを所有する。ESP-IDF コンソールレジストリに小さなコマンド集
 * （param(list/get/set/save)・status・reboot）を登録し、USB-CDC REPL を起動する。REPL は
 * 自前のタスクでシリアル行を読む。コマンドは静的な {name, help, func} テーブルに持ち、
 * ループで登録する（R6: レジストリパターン、extern グローバルのコンソールポインタを作らない）。
 * CLI は地上/ブリングアップ用ツールで、飛行クリティカル経路には載らない。
 *
 * @subscriber system_mode, sensor_power, sensor_health
 * @design architecture.md §6 — CLITask: CLI + Parameters              [OK]
 * @design architecture.md §3 — R6 CLI command registry pattern         [OK]
 * @design detailed_design.md §8 — CLITask                            [OK]
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_console.h"

#include "topics.hpp"
#include "params.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "CLITask";

namespace {

// =============================================================================
// Command handlers — signature int(int argc, char** argv) per esp_console.
// コマンドハンドラ — esp_console の int(int argc, char** argv) シグネチャ。
// =============================================================================

/// Find a registered parameter by name (linear scan over the params table).
/// 名前で登録パラメータを探す（params テーブルの線形走査）。
const sf::params::ParamEntry* findParam(const char* name)
{
    for (int i = 0, n = sf::params::count(); i < n; ++i) {
        const sf::params::ParamEntry* entry = sf::params::entry(i);
        if (entry != nullptr && std::strcmp(entry->name, name) == 0) {
            return entry;
        }
    }
    return nullptr;
}

/// Print one parameter's current value, formatted by its type.
/// パラメータ1件の現在値を型に応じて整形表示する。
void printParam(const sf::params::ParamEntry* entry)
{
    using sf::params::ParamType;
    switch (entry->type) {
    case ParamType::FLOAT: {
        float value = 0.0f;
        sf::params::get_float(entry->name, value);
        std::printf("%s = %.6g (float)\n", entry->name, value);
        break;
    }
    case ParamType::BOOL: {
        bool value = false;
        sf::params::get_bool(entry->name, value);
        std::printf("%s = %s (bool)\n", entry->name, value ? "true" : "false");
        break;
    }
    case ParamType::INT: {
        int32_t value = 0;
        sf::params::get_int(entry->name, value);
        std::printf("%s = %ld (int)\n", entry->name, static_cast<long>(value));
        break;
    }
    }
}

/// `param [list | get <name> | set <name> <value> | save]` — read/write parameters.
/// set_*() validates the range, runs the change callback and persists to NVS.
/// `param [list | get <name> | set <name> <value> | save]` — パラメータ読み書き。
/// set_*() は範囲検証・変更コールバック・NVS 保存を行う。
int cmd_param(int argc, char** argv)
{
    if (argc < 2 || std::strcmp(argv[1], "list") == 0) {
        sf::params::list();
        return 0;
    }
    if (std::strcmp(argv[1], "save") == 0) {
        sf::params::save();
        std::printf("params saved to NVS\n");
        return 0;
    }
    if (std::strcmp(argv[1], "get") == 0 && argc >= 3) {
        const sf::params::ParamEntry* entry = findParam(argv[2]);
        if (entry == nullptr) {
            std::printf("unknown param: %s\n", argv[2]);
            return 1;
        }
        printParam(entry);
        return 0;
    }
    if (std::strcmp(argv[1], "set") == 0 && argc >= 4) {
        const sf::params::ParamEntry* entry = findParam(argv[2]);
        if (entry == nullptr) {
            std::printf("unknown param: %s\n", argv[2]);
            return 1;
        }
        using sf::params::ParamType;
        bool ok = false;
        switch (entry->type) {
        case ParamType::FLOAT:
            ok = sf::params::set_float(entry->name, std::strtof(argv[3], nullptr));
            break;
        case ParamType::BOOL: {
            const bool value = (std::strcmp(argv[3], "1") == 0 ||
                                std::strcmp(argv[3], "true") == 0);
            ok = sf::params::set_bool(entry->name, value);
            break;
        }
        case ParamType::INT:
            ok = sf::params::set_int(
                entry->name, static_cast<int32_t>(std::strtol(argv[3], nullptr, 10)));
            break;
        }
        if (ok) {
            printParam(entry);
        } else {
            std::printf("set failed (out of range?): %s\n", entry->name);
        }
        return ok ? 0 : 1;
    }
    std::printf("usage: param [list | get <name> | set <name> <value> | save]\n");
    return 0;
}

/// `status` — flight state / mode / arm, battery, and sensor-health masks.
/// `status` — フライト状態/モード/ARM、電池、センサ健全性マスク。
int cmd_status(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    const sf::SystemMode mode = sf::system_mode.latest();
    const sf::PowerData power = sf::sensor_power.latest();
    const sf::SensorHealth health = sf::sensor_health.latest();
    std::printf("state   : %s\n",
                sf::flightStateName(static_cast<sf::FlightState>(mode.state)));
    std::printf("mode    : %s\n",
                sf::flightModeName(static_cast<sf::FlightMode>(mode.sub_mode)));
    std::printf("armed   : %s\n", mode.armed ? "yes" : "no");
    std::printf("battery : %.2f V, %.0f mA\n", power.voltage, power.current);
    std::printf("sensors : present=0x%02X healthy=0x%02X\n",
                health.present_mask, health.healthy_mask);
    return 0;
}

/// `reboot` — restart the flight controller.
/// `reboot` — フライトコントローラを再起動する。
int cmd_reboot(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("rebooting...\n");
    std::fflush(stdout);
    esp_restart();
    return 0;  // not reached / 到達しない
}

// =============================================================================
// Command registry (R6) — static {name, help, func} table registered in a loop.
// コマンドレジストリ（R6）— 静的な {name, help, func} テーブルをループ登録。
// =============================================================================

struct CliCommand {
    const char* name;
    const char* help;
    esp_console_cmd_func_t func;
};

const CliCommand kCommands[] = {
    {"param",  "param [list|get <name>|set <name> <value>|save]", &cmd_param},
    {"status", "Show flight state, battery and sensor health",    &cmd_status},
    {"reboot", "Reboot the flight controller",                    &cmd_reboot},
};

void registerCommands()
{
    for (const CliCommand& command : kCommands) {
        esp_console_cmd_t cmd = {};
        cmd.command  = command.name;
        cmd.help     = command.help;
        cmd.hint     = nullptr;
        cmd.func     = command.func;
        cmd.argtable = nullptr;
        esp_console_cmd_register(&cmd);
    }
}

}  // namespace

void CLITask(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "CLITask started");

    // Create the USB-CDC console REPL. esp_console_new_repl_usb_cdc() runs
    // esp_console_init() internally, so commands must be registered AFTER it.
    // USB-CDC コンソール REPL を生成する。esp_console_new_repl_usb_cdc() は内部で
    // esp_console_init() を呼ぶため、コマンド登録はこの後で行う。
    esp_console_repl_t* repl = nullptr;
    esp_console_repl_config_t repl_config = {};
    repl_config.max_history_len    = 16;
    repl_config.history_save_path  = nullptr;
    repl_config.task_stack_size    = config::STACK_CLI;
    repl_config.task_priority      = config::PRIORITY_CLI;
    repl_config.prompt             = "stampfly> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_usb_cdc_config_t cdc_config = {};

    esp_err_t err = esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "console REPL init failed: %s — CLI disabled",
                 esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    esp_console_register_help_command();  // built-in 'help' / 組み込みの 'help'
    registerCommands();                   // R6 registry / R6 レジストリ

    // Start the interactive REPL — esp_console reads the USB-CDC serial line in its
    // own task. On the SIL host this is inert (commands stay registered and remain
    // dispatchable via a scenario console feeder).
    // 対話 REPL を起動 — esp_console が自前タスクで USB-CDC シリアル行を読む。SIL ホスト
    // では inert（コマンドは登録済みのままで、シナリオのコンソールフィーダから dispatch 可）。
    esp_console_start_repl(repl);

    // The REPL task now owns interactive input; CLITask parks. It is the single owner
    // of the console (R6) and the hook point for a future TCP CLI.
    // 以降は REPL タスクが対話入力を所有し、CLITask は待機する。コンソールの唯一の所有者
    // （R6）であり、将来の TCP CLI の配線ポイント。
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
