/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito — StampFly Ecosystem (SIL host — ESP-IDF host platform). */
/** @file esp_vfs_cdcacm.h  Host shim for the USB CDC-ACM VFS driver (console I/O). Inert. */
#pragma once
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
static inline esp_err_t esp_vfs_dev_cdcacm_register_b(void) { return ESP_OK; }  // placeholder
#ifdef __cplusplus
}
#endif
