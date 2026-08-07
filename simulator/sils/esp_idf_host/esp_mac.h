/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_mac.h
 * @brief Host shim for the ESP-IDF MAC address API
 *        ESP-IDF MAC アドレス API のホスト用シム
 *
 * Provides esp_mac_type_t and esp_read_mac() so the firmware compiles and links
 * on a PC. The host returns a fixed, locally-administered MAC address; no real
 * eFuse base MAC exists.
 *
 * 本体ファームが参照する esp_mac_type_t・esp_read_mac() を提供し、PC 上で
 * コンパイル・リンクできるようにする。ホストは固定のローカル管理 MAC を返す。
 * 実機の eFuse ベース MAC は存在しない。
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * Build-time constants mirrored from sdkconfig
 * sdkconfig 由来のビルド時定数を再現
 * ===========================================================================*/

/* Number of universal MAC addresses provisioned in eFuse / eFuse の universal MAC 数 */
#define ESP_MAC_UNIVERSAL_MAC_ADDRESSES        4
#define ESP_MAC_UNIVERSAL_MAC_ADDRESSES_FOUR   4
#define ESP_MAC_UNIVERSAL_MAC_ADDRESSES_TWO    2

/* MAC-address universe (derivation base) selectors / MAC ユニバース選択子 */
#define ESP_MAC_ADDR_UNIVERSE_WIFI_STA   1  /* Wi-Fi STA universe / Wi-Fi STA 系列 */
#define ESP_MAC_ADDR_UNIVERSE_WIFI_AP    2  /* Wi-Fi AP universe / Wi-Fi AP 系列 */
#define ESP_MAC_ADDR_UNIVERSE_BT         3  /* Bluetooth universe / Bluetooth 系列 */
#define ESP_MAC_ADDR_UNIVERSE_ETH        4  /* Ethernet universe / Ethernet 系列 */

/* ===========================================================================
 * Enums / 列挙型
 * ===========================================================================*/

/* MAC address type to read / 読み出す MAC アドレスの種別 */
typedef enum {
    ESP_MAC_WIFI_STA = 0,   /* Wi-Fi station MAC / Wi-Fi STA の MAC */
    ESP_MAC_WIFI_SOFTAP,    /* Wi-Fi soft-AP MAC / Wi-Fi AP の MAC */
    ESP_MAC_BT,             /* Bluetooth MAC */
    ESP_MAC_ETH,            /* Ethernet MAC */
    ESP_MAC_IEEE802154,     /* 802.15.4 MAC */
    ESP_MAC_BASE,           /* base MAC / ベース MAC */
    ESP_MAC_EFUSE_FACTORY,  /* factory eFuse MAC / 工場出荷 eFuse MAC */
    ESP_MAC_EFUSE_CUSTOM,   /* custom eFuse MAC / カスタム eFuse MAC */
    ESP_MAC_EFUSE_EXT       /* extended eFuse MAC / 拡張 eFuse MAC */
} esp_mac_type_t;

/* ===========================================================================
 * Functions (stubs) / 関数（スタブ）
 * ===========================================================================*/

/* Read the MAC address of the given type / 指定種別の MAC アドレスを取得する */
static inline esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type)
{
    if (mac) {
        /* Fixed locally-administered host MAC; last byte varies by type so the
         * STA/AP MACs are not identical (mirrors real-device behaviour).
         * 固定のローカル管理 MAC。種別で末尾を変え STA/AP の MAC を区別する
         * （実機の挙動を模す）。 */
        static const uint8_t host_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        std::memcpy(mac, host_mac, 6);
        mac[5] = static_cast<uint8_t>(0x01 + static_cast<int>(type));
    }
    return ESP_OK;
}

/* Set the base MAC address / ベース MAC アドレスを設定する */
static inline esp_err_t esp_base_mac_addr_set(const uint8_t* mac)
{
    (void)mac;
    return ESP_OK;
}

/* Get the base MAC address / ベース MAC アドレスを取得する */
static inline esp_err_t esp_base_mac_addr_get(uint8_t* mac)
{
    return esp_read_mac(mac, ESP_MAC_BASE);
}

#ifdef __cplusplus
} /* extern "C" */
#endif
