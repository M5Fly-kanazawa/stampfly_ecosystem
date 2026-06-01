/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito — StampFly Ecosystem (SIL host — ESP-IDF host platform). */
/** @file esp_vfs_dev.h  Host shim for ESP-IDF VFS device registration (USB CDC console). */
#pragma once
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
// Register the USB CDC-ACM VFS device. Inert on host (no real USB serial).
static inline esp_err_t esp_vfs_dev_cdcacm_register(void) { return ESP_OK; }
#ifdef __cplusplus
}
#endif
