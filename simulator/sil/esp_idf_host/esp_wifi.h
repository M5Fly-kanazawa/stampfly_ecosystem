/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_wifi.h
 * @brief Host shim for the ESP-IDF Wi-Fi driver API
 *        ESP-IDF Wi-Fi ドライバ API のホスト用シム
 *
 * Provides the wifi_*_t types, enums, and esp_wifi_* functions the firmware
 * references so it compiles and links on a PC. All stubs are inert: they
 * return ESP_OK and zero-fill any output buffers. The radio does not exist
 * on the host — the virtual transport is wired up separately (see esp_now.h).
 *
 * 本体ファームが参照する wifi_*_t 型・列挙・esp_wifi_* 関数を提供し、
 * PC 上でコンパイル・リンクできるようにする。スタブは全て無動作で、
 * ESP_OK を返し出力バッファをゼロ埋めする。ホストにラジオは存在しない。
 */

#pragma once

#include "esp_err.h"
#include <cstdint>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * Enums / 列挙型
 * ===========================================================================*/

/* Wi-Fi operating mode / Wi-Fi 動作モード */
typedef enum {
    WIFI_MODE_NULL = 0,  /* null mode / ヌルモード */
    WIFI_MODE_STA,       /* station / ステーション */
    WIFI_MODE_AP,        /* soft-AP / ソフト AP */
    WIFI_MODE_APSTA,     /* station + soft-AP / 両方 */
    WIFI_MODE_NAN,       /* NAN */
    WIFI_MODE_MAX
} wifi_mode_t;

/* Wi-Fi interface index / Wi-Fi インターフェイス番号 */
typedef enum {
    WIFI_IF_STA = 0,  /* station interface / STA インターフェイス */
    WIFI_IF_AP  = 1,  /* soft-AP interface / AP インターフェイス */
    WIFI_IF_NAN = 2,
    WIFI_IF_MAX
} wifi_interface_t;

/* Power-save mode / 省電力モード */
typedef enum {
    WIFI_PS_NONE = 0,      /* no power save / 省電力なし */
    WIFI_PS_MIN_MODEM,
    WIFI_PS_MAX_MODEM
} wifi_ps_type_t;

/* Config storage / 設定の保存先 */
typedef enum {
    WIFI_STORAGE_FLASH = 0,  /* persist in flash / フラッシュに保存 */
    WIFI_STORAGE_RAM         /* RAM only / RAM のみ */
} wifi_storage_t;

/* Secondary channel offset / セカンダリチャンネルのオフセット */
typedef enum {
    WIFI_SECOND_CHAN_NONE = 0,  /* no secondary channel / セカンダリなし */
    WIFI_SECOND_CHAN_ABOVE,
    WIFI_SECOND_CHAN_BELOW
} wifi_second_chan_t;

/* Authentication mode / 認証モード */
typedef enum {
    WIFI_AUTH_OPEN = 0,
    WIFI_AUTH_WEP,
    WIFI_AUTH_WPA_PSK,
    WIFI_AUTH_WPA2_PSK,
    WIFI_AUTH_WPA_WPA2_PSK,
    WIFI_AUTH_ENTERPRISE,
    WIFI_AUTH_WPA2_ENTERPRISE = WIFI_AUTH_ENTERPRISE,
    WIFI_AUTH_WPA3_PSK,
    WIFI_AUTH_WPA2_WPA3_PSK,
    WIFI_AUTH_WAPI_PSK,
    WIFI_AUTH_OWE,
    WIFI_AUTH_WPA3_ENT_192,
    WIFI_AUTH_MAX
} wifi_auth_mode_t;

/* ===========================================================================
 * Init config / 初期化設定
 * ===========================================================================*/

/* Wi-Fi stack init config. On host all fields are inert tuning knobs.
 * Wi-Fi スタック初期化設定。ホストでは全フィールドが無動作のチューニング値。 */
typedef struct {
    void*   osi_funcs;
    void*   wpa_crypto_funcs;
    int     static_rx_buf_num;
    int     dynamic_rx_buf_num;
    int     tx_buf_type;
    int     static_tx_buf_num;
    int     dynamic_tx_buf_num;
    int     rx_mgmt_buf_type;
    int     rx_mgmt_buf_num;
    int     cache_tx_buf_num;
    int     csi_enable;
    int     ampdu_rx_enable;
    int     ampdu_tx_enable;
    int     amsdu_tx_enable;
    int     nvs_enable;
    int     nano_enable;
    int     rx_ba_win;
    int     wifi_task_core_id;
    int     beacon_max_len;
    int     mgmt_sbuf_num;
    uint64_t feature_caps;
    bool    sta_disconnected_pm;
    int     espnow_max_encrypt_num;
    int     tx_hetb_queue_num;
    bool    dump_hesigb_enable;
    int     magic;
} wifi_init_config_t;

/* Default init config. Values are placeholders — host ignores them.
 * デフォルト初期化設定。値はプレースホルダでホストは無視する。 */
#define WIFI_INIT_CONFIG_MAGIC 0x1F2F3F4F
#define WIFI_INIT_CONFIG_DEFAULT() { \
    /* osi_funcs           */ nullptr, \
    /* wpa_crypto_funcs    */ nullptr, \
    /* static_rx_buf_num   */ 10,   \
    /* dynamic_rx_buf_num  */ 32,   \
    /* tx_buf_type         */ 1,    \
    /* static_tx_buf_num   */ 0,    \
    /* dynamic_tx_buf_num  */ 32,   \
    /* rx_mgmt_buf_type    */ 0,    \
    /* rx_mgmt_buf_num     */ 5,    \
    /* cache_tx_buf_num    */ 0,    \
    /* csi_enable          */ 1,    \
    /* ampdu_rx_enable     */ 1,    \
    /* ampdu_tx_enable     */ 1,    \
    /* amsdu_tx_enable     */ 0,    \
    /* nvs_enable          */ 1,    \
    /* nano_enable         */ 0,    \
    /* rx_ba_win           */ 6,    \
    /* wifi_task_core_id   */ 0,    \
    /* beacon_max_len      */ 752,  \
    /* mgmt_sbuf_num       */ 32,   \
    /* feature_caps        */ 0,    \
    /* sta_disconnected_pm */ false,\
    /* espnow_max_encrypt_num */ 7, \
    /* tx_hetb_queue_num   */ 0,    \
    /* dump_hesigb_enable  */ false,\
    /* magic               */ WIFI_INIT_CONFIG_MAGIC \
}

/* ===========================================================================
 * Connection config / 接続設定
 * ===========================================================================*/

/* Scan threshold for STA association / STA 接続のスキャン閾値 */
typedef struct {
    int8_t           rssi;      /* minimum RSSI / 最小 RSSI */
    wifi_auth_mode_t authmode;  /* minimum auth mode / 最小認証モード */
    int8_t           rssi_5g_adjustment;
} wifi_scan_threshold_t;

/* PMF (protected management frame) config / PMF 設定 */
typedef struct {
    bool capable;
    bool required;
} wifi_pmf_config_t;

/* Station configuration / ステーション設定 */
typedef struct {
    uint8_t               ssid[32];      /* target SSID / 接続先 SSID */
    uint8_t               password[64];  /* passphrase / パスフレーズ */
    uint8_t               scan_method;
    bool                  bssid_set;
    uint8_t               bssid[6];
    uint8_t               channel;
    uint16_t              listen_interval;
    int                   sort_method;
    wifi_scan_threshold_t threshold;
    wifi_pmf_config_t     pmf_cfg;
    uint32_t              rm_enabled        : 1;
    uint32_t              btm_enabled       : 1;
    uint32_t              mbo_enabled       : 1;
    uint32_t              ft_enabled        : 1;
    uint32_t              owe_enabled       : 1;
    uint32_t              transition_disable: 1;
    uint32_t              reserved          : 26;
    int                   sae_pwe_h2e;
    uint8_t               failure_retry_cnt;
} wifi_sta_config_t;

/* Soft-AP configuration / ソフト AP 設定 */
typedef struct {
    uint8_t          ssid[32];        /* AP SSID */
    uint8_t          password[64];    /* AP password / AP パスワード */
    uint8_t          ssid_len;        /* SSID length / SSID 長 */
    uint8_t          channel;         /* RF channel / RF チャンネル */
    wifi_auth_mode_t authmode;        /* auth mode / 認証モード */
    uint8_t          ssid_hidden;
    uint8_t          max_connection;  /* max clients / 最大接続数 */
    uint16_t         beacon_interval;
    uint8_t          csa_count;
    uint8_t          dtim_period;
    int              pairwise_cipher;
    bool             ftm_responder;
    wifi_pmf_config_t pmf_cfg;
    int              sae_pwe_h2e;
} wifi_ap_config_t;

/* Union holding either an AP or STA config / AP/STA 設定の共用体 */
typedef union {
    wifi_ap_config_t  ap;   /* soft-AP config / ソフト AP 設定 */
    wifi_sta_config_t sta;  /* station config / STA 設定 */
} wifi_config_t;

/* ===========================================================================
 * Functions (stubs) / 関数（スタブ）
 * ===========================================================================*/

/* Initialize the Wi-Fi driver / Wi-Fi ドライバを初期化する */
static inline esp_err_t esp_wifi_init(const wifi_init_config_t* config)
{
    (void)config;
    return ESP_OK;
}

/* Deinitialize the Wi-Fi driver / Wi-Fi ドライバを終了する */
static inline esp_err_t esp_wifi_deinit(void)
{
    return ESP_OK;
}

/* Set config storage backend / 設定の保存先を選ぶ */
static inline esp_err_t esp_wifi_set_storage(wifi_storage_t storage)
{
    (void)storage;
    return ESP_OK;
}

/* Set the operating mode / 動作モードを設定する */
static inline esp_err_t esp_wifi_set_mode(wifi_mode_t mode)
{
    (void)mode;
    return ESP_OK;
}

/* Get the operating mode / 動作モードを取得する */
static inline esp_err_t esp_wifi_get_mode(wifi_mode_t* mode)
{
    if (mode) {
        *mode = WIFI_MODE_STA;
    }
    return ESP_OK;
}

/* Set per-interface config / インターフェイス毎の設定を行う */
static inline esp_err_t esp_wifi_set_config(wifi_interface_t interface,
                                            wifi_config_t* conf)
{
    (void)interface;
    (void)conf;
    return ESP_OK;
}

/* Get per-interface config / インターフェイス毎の設定を取得する */
static inline esp_err_t esp_wifi_get_config(wifi_interface_t interface,
                                            wifi_config_t* conf)
{
    (void)interface;
    if (conf) {
        std::memset(conf, 0, sizeof(*conf));
    }
    return ESP_OK;
}

/* Set the primary/secondary RF channel / 主・副チャンネルを設定する */
static inline esp_err_t esp_wifi_set_channel(uint8_t primary,
                                             wifi_second_chan_t second)
{
    (void)primary;
    (void)second;
    return ESP_OK;
}

/* Get the current channel / 現在のチャンネルを取得する */
static inline esp_err_t esp_wifi_get_channel(uint8_t* primary,
                                             wifi_second_chan_t* second)
{
    if (primary) {
        *primary = 1;
    }
    if (second) {
        *second = WIFI_SECOND_CHAN_NONE;
    }
    return ESP_OK;
}

/* Set power-save mode / 省電力モードを設定する */
static inline esp_err_t esp_wifi_set_ps(wifi_ps_type_t type)
{
    (void)type;
    return ESP_OK;
}

/* Start the Wi-Fi driver / Wi-Fi ドライバを開始する */
static inline esp_err_t esp_wifi_start(void)
{
    return ESP_OK;
}

/* Stop the Wi-Fi driver / Wi-Fi ドライバを停止する */
static inline esp_err_t esp_wifi_stop(void)
{
    return ESP_OK;
}

/* Connect the station to the configured AP / STA を AP に接続する */
static inline esp_err_t esp_wifi_connect(void)
{
    return ESP_OK;
}

/* Disconnect the station / STA を切断する */
static inline esp_err_t esp_wifi_disconnect(void)
{
    return ESP_OK;
}

/* Read the MAC of the given interface / 指定インターフェイスの MAC を取得する */
static inline esp_err_t esp_wifi_get_mac(wifi_interface_t interface,
                                         uint8_t mac[6])
{
    (void)interface;
    if (mac) {
        /* A fixed, locally-administered host MAC (02:00:00:00:00:01).
         * 固定のローカル管理 MAC（02:00:00:00:00:01）を返す。 */
        static const uint8_t host_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
        std::memcpy(mac, host_mac, 6);
    }
    return ESP_OK;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

// --- SIL additions: WiFi event base + station events (old firmware comm) -----
#include "esp_event.h"
#ifndef WIFI_EVENT
#define WIFI_EVENT ((esp_event_base_t)"WIFI_EVENT")
#endif
enum {
    WIFI_EVENT_WIFI_READY = 0, WIFI_EVENT_SCAN_DONE = 1,
    WIFI_EVENT_STA_START = 2, WIFI_EVENT_STA_STOP = 3,
    WIFI_EVENT_STA_CONNECTED = 4, WIFI_EVENT_STA_DISCONNECTED = 5,
    WIFI_EVENT_AP_START = 12, WIFI_EVENT_AP_STOP = 13,   /* SoftAP lifecycle (values match ESP-IDF) */
};
typedef struct {
    uint8_t ssid[33]; uint8_t ssid_len; uint8_t bssid[6];
    uint8_t channel; int authmode; uint16_t aid;
} wifi_event_sta_connected_t;
typedef struct {
    uint8_t ssid[33]; uint8_t ssid_len; uint8_t bssid[6];
    uint8_t reason; int8_t rssi;
} wifi_event_sta_disconnected_t;
