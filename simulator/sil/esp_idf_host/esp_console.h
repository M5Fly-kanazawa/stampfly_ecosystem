/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_console.h
 * @brief Host shim for the ESP-IDF console/REPL (used by the old firmware CLI).
 *        ESP-IDF コンソール/REPL のホスト用シム（旧ファーム CLI が使用）。
 *
 * Inert on the host: command registration is accepted and the REPL never reads a
 * real serial line (the emulator is batch, not interactive). Commands can still
 * be invoked programmatically via esp_console_run() if a future harness wants to.
 * ホストでは inert。コマンド登録は受理、REPL は実シリアルを読まない（バッチ実行）。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_CONSOLE_USB_CDC 1

typedef int (*esp_console_cmd_func_t)(int argc, char** argv);

typedef struct {
    const char* command;
    const char* help;
    const char* hint;
    esp_console_cmd_func_t func;
    void* argtable;
} esp_console_cmd_t;

typedef struct {
    uint32_t max_cmdline_length;
    uint32_t max_cmdline_args;
    int      hint_color;
    int      hint_bold;
} esp_console_config_t;

#define ESP_CONSOLE_CONFIG_DEFAULT() { 256, 8, 0, 0 }

typedef struct esp_console_repl_s esp_console_repl_t;

typedef struct {
    uint32_t max_history_len;
    const char* history_save_path;
    uint32_t task_stack_size;
    uint32_t task_priority;
    const char* prompt;
    size_t max_cmdline_length;
} esp_console_repl_config_t;

typedef struct {
    int cdc_dev;
} esp_console_dev_usb_cdc_config_t;

static inline esp_err_t esp_console_init(const esp_console_config_t* c) { (void)c; return ESP_OK; }
static inline esp_err_t esp_console_deinit(void) { return ESP_OK; }
static inline esp_err_t esp_console_cmd_register(const esp_console_cmd_t* c) { (void)c; return ESP_OK; }
static inline esp_err_t esp_console_run(const char* cmdline, int* ret) { (void)cmdline; if (ret) *ret = 0; return ESP_OK; }
static inline esp_err_t esp_console_register_help_command(void) { return ESP_OK; }

static inline esp_err_t esp_console_new_repl_usb_cdc(const esp_console_dev_usb_cdc_config_t* dev,
                                                     const esp_console_repl_config_t* cfg,
                                                     esp_console_repl_t** out)
{ (void)dev; (void)cfg; if (out) *out = (esp_console_repl_t*)0x1; return ESP_OK; }
static inline esp_err_t esp_console_start_repl(esp_console_repl_t* repl) { (void)repl; return ESP_OK; }

#ifdef __cplusplus
}
#endif
