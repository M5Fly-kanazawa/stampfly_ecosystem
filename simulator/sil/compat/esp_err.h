/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_err.h (host shim)
 * @brief Minimal esp_err_t / error codes for host (PC) builds
 *        ホスト(PC)ビルド用の最小 esp_err_t・エラーコード
 *
 * Added for the M11 regression challenge: the legacy firmware/vehicle ESKF
 * (eskf.hpp/cpp) includes esp_err.h. This shim lets it compile UNMODIFIED on a
 * PC, preserving Code Identity for the legacy code too.
 *
 * M11回帰チャレンジ用。旧 firmware/vehicle の ESKF が esp_err.h を include する。
 * このシムで旧コードも無改変でPCコンパイルでき、旧コードのCode Identityを保つ。
 */

#pragma once

typedef int esp_err_t;

#define ESP_OK                 0
#define ESP_FAIL               -1
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
