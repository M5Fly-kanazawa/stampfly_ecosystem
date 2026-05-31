/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_log.h
 * @brief Host stub for ESP-IDF logging — redirects ESP_LOGx to stderr
 *        ESP-IDF ロギングのホスト用スタブ — ESP_LOGx を stderr に流す
 *
 * The SIL compiles the unmodified firmware on a PC. ESP-IDF logging is
 * redirected to stderr so it never pollutes stdout (which carries the
 * SIL's machine-readable results / CSV).
 *
 * SIL は本体ファームを無改変で PC 上にコンパイルする。ESP-IDF の
 * ロギングは stderr に流し、stdout（SIL の機械可読な結果・CSV）を
 * 汚さないようにする。
 */

#pragma once

#include <cstdio>

// Log levels map to stderr prints. ESP_LOGD/V are silenced by default.
// ログレベルは stderr 出力にマップ。ESP_LOGD/V は既定で無音。
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[ERR]  %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
