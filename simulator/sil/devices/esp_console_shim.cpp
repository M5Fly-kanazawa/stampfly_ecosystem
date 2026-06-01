/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_console_shim.cpp
 * @brief Real (dispatching) ESP-IDF console registry for the emulator.
 *        エミュレータ用の実ディスパッチ ESP-IDF コンソールレジストリ。
 *
 * The header esp_console.h forwards esp_console_cmd_register / esp_console_run to
 * the shared registry here (ONE slot across translation units, like espnow_hub),
 * so a console line typed by the scenario console feeder actually dispatches to
 * the firmware's UNMODIFIED command handlers (version/status/help/...). Without
 * a scenario nothing is ever typed, so the registry fills at boot but no command
 * runs — the default run stays byte-identical.
 *
 * esp_console.h が register/run をここの共有レジストリへ転送する（TU 横断の単一スロット）。
 * シナリオのコンソールフィーダが打鍵した行が、無改変のファームコマンドハンドラへ実際に
 * ディスパッチされる。シナリオ無しなら何も打鍵されず＝既定実行は byte-identical。
 */

#include "esp_console.h"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {

struct Cmd {
    const char* name;                 // stable static string from the firmware
    esp_console_cmd_func_t func;
};

// Single shared registry (function-local static → one instance, filled at boot).
// 単一の共有レジストリ（関数ローカル static ＝1個、起動時に充填）。
std::vector<Cmd>& registry()
{
    static std::vector<Cmd> r;
    return r;
}

}  // namespace

extern "C" {

esp_err_t sil_console_register(const esp_console_cmd_t* c)
{
    if (c != nullptr && c->command != nullptr && c->func != nullptr) {
        registry().push_back(Cmd{c->command, c->func});
    }
    return ESP_OK;
}

// Tokenize the line and dispatch to the matching command handler, exactly like
// esp_console_run: ESP_OK when found (cmd_ret holds the handler's return),
// ESP_ERR_NOT_FOUND when no command matches (Console::run prints the message).
// 行をトークン分割してコマンドへディスパッチ（esp_console_run と同じ意味論）。
esp_err_t sil_console_dispatch(const char* cmdline, int* cmd_ret)
{
    if (cmd_ret != nullptr) *cmd_ret = 0;
    if (cmdline == nullptr) return ESP_OK;

    char buf[256];
    std::strncpy(buf, cmdline, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* argv[16];
    int argc = 0;
    for (char* tok = std::strtok(buf, " \t\r\n");
         tok != nullptr && argc < 16;
         tok = std::strtok(nullptr, " \t\r\n")) {
        argv[argc++] = tok;
    }
    if (argc == 0) return ESP_OK;  // empty/whitespace line

    for (const Cmd& cmd : registry()) {
        if (std::strcmp(cmd.name, argv[0]) == 0) {
            int r = cmd.func(argc, argv);
            if (cmd_ret != nullptr) *cmd_ret = r;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

}  // extern "C"
