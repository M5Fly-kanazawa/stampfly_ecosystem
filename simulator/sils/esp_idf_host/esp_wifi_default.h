/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_wifi_default.h
 * @brief Host shim for ESP-IDF's esp_wifi_default.h.
 *        ESP-IDF の esp_wifi_default.h のホスト用シム。
 *
 * On real ESP-IDF, esp_netif_create_default_wifi_sta() lives in this header (esp_wifi
 * component). The host bench already provides that inline in esp_netif.h, so this shim
 * just re-exposes it via that header — letting firmware include "esp_wifi_default.h"
 * (the canonical header) and compile identically on host and target.
 * 実 ESP-IDF では esp_netif_create_default_wifi_sta() は本ヘッダ(esp_wifi コンポーネント)に
 * ある。ホスト台は既に esp_netif.h でそのインラインを提供するため、本シムはそれを再公開
 * するだけ。ファームは正規ヘッダ "esp_wifi_default.h" を include でき、ホストでも実機でも
 * 同一にコンパイルできる。
 */

#pragma once

#include "esp_netif.h"  // provides the host inline esp_netif_create_default_wifi_sta()
