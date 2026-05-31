/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file nvs_flash.h
 * @brief Host stub for ESP-IDF NVS flash init
 *        ESP-IDF NVS フラッシュ初期化のホスト用スタブ
 *
 * On the SIL there is no flash partition; init/erase are no-ops that
 * always succeed. See nvs.h for the in-memory store.
 *
 * SIL にはフラッシュパーティションが無いので、init/erase は常に成功する
 * 何もしない実装。インメモリ保存は nvs.h を参照。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);

#ifdef __cplusplus
}  // extern "C"
#endif
