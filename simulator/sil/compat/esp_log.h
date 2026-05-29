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

// Logs go to stderr so they never contaminate the CSV that the integrated SIL
// writes to stdout. Unit tests print results on stdout, logs on stderr.
// ログは stderr へ出す。統合SILが stdout に書く CSV を汚染しないため。
// ユニットテストは結果を stdout、ログを stderr に出す。
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[ERR]  %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...)
