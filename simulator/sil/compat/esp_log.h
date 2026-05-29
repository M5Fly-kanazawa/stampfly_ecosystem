/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_log.h (host shim)
 * @brief ESP_LOGx mapped to printf for host (PC) builds
 *        ホスト(PC)ビルド用にprintfへマップしたESP_LOGx
 *
 * Mirrors firmware/vehicle_new/test/esp_log.h so SIL and unit tests share
 * one logging shim.
 *
 * firmware/vehicle_new/test/esp_log.h と同内容。SILとユニットテストで
 * 1つのログシムを共有する。
 */

#pragma once

#include <cstdio>

#define ESP_LOGI(tag, fmt, ...) printf("[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERR]  %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...)
