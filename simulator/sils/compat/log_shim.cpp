/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_shim.cpp
 * @brief The one variable behind esp_log_set_vprintf on the host
 *        ホストでの esp_log_set_vprintf の背後にある変数 1 個
 *
 * ESP_LOGx (compat/esp_log.h) hands each formatted line to the installed output function;
 * NULL means "write to stderr". The firmware's log sink (tasks/log_task.cpp) installs its own.
 * The variable lives here so that every translation unit, C or C++, sees the same one.
 *
 * ESP_LOGx（compat/esp_log.h）は整形した行を設定済みの出力関数へ渡す。NULL は「stderr へ書く」。
 * ファームのログシンク（tasks/log_task.cpp）が自前の関数を設定する。C でも C++ でも全翻訳単位が
 * 同じ変数を見るよう、ここに置く。
 */

#include "esp_log.h"

extern "C" {
vprintf_like_t esp_log_output_function = nullptr;
}
