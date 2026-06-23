/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_netif.h
 * @brief Host shim for ESP-IDF TCP/IP network interface (esp_netif)
 *        ESP-IDF TCP/IP ネットワークインターフェース (esp_netif) のホスト用シム
 *
 * Provides the esp_netif API surface plus the IP/WiFi event bases & ids the
 * firmware references, so it compiles and links unmodified on a PC. The
 * "default WiFi STA/AP" creators return a non-null dummy handle; IP queries
 * return a loopback-style address. No real networking occurs.
 *
 * 本体ファームが参照する esp_netif API、および IP/WiFi のイベントベース・ID を
 * 提供し、無改変で PC 上にコンパイル・リンクできるようにする。「デフォルト WiFi
 * STA/AP」生成関数は非 NULL のダミーハンドルを返し、IP 問い合わせはループバック
 * 風アドレスを返す。実際の通信は行わない。
 */

#pragma once

#include "esp_err.h"
#include "esp_event.h"

#include <cstdint>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// IPv4 address types & helper macros (mirror esp_netif_ip_addr.h / lwip)
// IPv4 アドレス型・補助マクロ（esp_netif_ip_addr.h / lwip を模倣）
// -----------------------------------------------------------------------------

// Packed 32-bit IPv4 address (network byte order in real ESP-IDF)
// パック化された 32bit IPv4 アドレス（実 ESP-IDF ではネットワークバイト順）
typedef struct esp_ip4_addr {
    uint32_t addr; // raw IPv4 address / 生の IPv4 アドレス
} esp_ip4_addr_t;

// Format string and accessor macros for printing an IPv4 address
// IPv4 アドレスを表示するための書式文字列とアクセサマクロ
#define IPSTR "%d.%d.%d.%d"
#define ESP_IP4_ADDR_BYTE(ipaddr, index) \
    (((uint8_t*)(&((ipaddr)->addr)))[index])
#define IP2STR(ipaddr)                 \
    ESP_IP4_ADDR_BYTE(ipaddr, 0),      \
    ESP_IP4_ADDR_BYTE(ipaddr, 1),      \
    ESP_IP4_ADDR_BYTE(ipaddr, 2),      \
    ESP_IP4_ADDR_BYTE(ipaddr, 3)

// Resolved IPv4 configuration of a netif (address / netmask / gateway)
// netif の確定済み IPv4 構成（アドレス / ネットマスク / ゲートウェイ）
typedef struct esp_netif_ip_info {
    esp_ip4_addr_t ip;      // interface IPv4 address / インターフェース IPv4 アドレス
    esp_ip4_addr_t netmask; // subnet mask / サブネットマスク
    esp_ip4_addr_t gw;      // gateway / ゲートウェイ
} esp_netif_ip_info_t;

// Opaque network interface handle
// 不透明なネットワークインターフェースハンドル
typedef struct esp_netif_obj esp_netif_t;

// -----------------------------------------------------------------------------
// IP_EVENT base & event ids (mirror esp_netif_types.h)
// IP_EVENT ベース・イベント ID（esp_netif_types.h を模倣）
// -----------------------------------------------------------------------------

// Event base token for IP-layer events.
// Real ESP-IDF declares this as `extern const char* IP_EVENT;` (defined in a
// .c file). Since this shim is header-only (no .cpp to provide the definition),
// it is supplied as a string-literal macro instead — it still works wherever the
// firmware passes IP_EVENT as an esp_event_base_t (const char*).
// IP レイヤイベント用のイベントベーストークン。
// 実 ESP-IDF は .c で定義される `extern const char* IP_EVENT;` だが、本シムは
// ヘッダのみ（定義を置く .cpp が無い）ため、文字列リテラルマクロで供給する。
// 本体が IP_EVENT を esp_event_base_t(const char*) として渡す箇所で機能する。
#define IP_EVENT ((esp_event_base_t)"IP_EVENT")

// IP event ids
// IP イベント ID
typedef enum {
    IP_EVENT_STA_GOT_IP = 0,   // station got IPv4 / station が IPv4 を取得
    IP_EVENT_STA_LOST_IP,      // station lost IPv4 / station が IPv4 を喪失
    IP_EVENT_AP_STAIPASSIGNED, // soft-AP assigned IPv4 to a client / soft-AP が割当
    IP_EVENT_GOT_IP6,          // got IPv6 link-local / IPv6 リンクローカル取得
    IP_EVENT_ETH_GOT_IP,       // ethernet got IPv4 / Ethernet が IPv4 を取得
    IP_EVENT_ETH_LOST_IP,      // ethernet lost IPv4 / Ethernet が IPv4 を喪失
} ip_event_t;

// Forward declaration of the netif object for the event payload
// イベントペイロード用に netif オブジェクトを前方宣言
struct esp_netif_obj;

// Payload for IP_EVENT_STA_GOT_IP / IP_EVENT_ETH_GOT_IP
// IP_EVENT_STA_GOT_IP / IP_EVENT_ETH_GOT_IP のペイロード
typedef struct {
    esp_netif_t* esp_netif;        // originating interface / 発信元インターフェース
    esp_netif_ip_info_t ip_info;   // resolved IPv4 config / 確定 IPv4 構成
    bool ip_changed;               // whether address changed / アドレス変化有無
} ip_event_got_ip_t;

// -----------------------------------------------------------------------------
// esp_netif behavioral flags / TCP/IP stack selectors (mirror esp_netif_types.h)
// esp_netif 動作フラグ / TCP-IP スタック選択子（esp_netif_types.h を模倣）
// -----------------------------------------------------------------------------

// Behavioral flags (subset referenced by firmware-adjacent config)
// 動作フラグ（ファーム周辺の構成が参照する部分集合）
typedef enum {
    ESP_NETIF_DHCP_CLIENT             = 1 << 0,
    ESP_NETIF_DHCP_SERVER             = 1 << 1,
    ESP_NETIF_FLAG_AUTOUP             = 1 << 2,
    ESP_NETIF_FLAG_GARP               = 1 << 3,
    ESP_NETIF_FLAG_EVENT_IP_MODIFIED  = 1 << 4,
    ESP_NETIF_FLAG_IS_PPP             = 1 << 5,
    ESP_NETIF_FLAG_IS_BRIDGE          = 1 << 6,
    ESP_NETIF_FLAG_MLDV6_REPORT       = 1 << 7,
} esp_netif_flags_t;

// TCP/IP stack backend selector
// TCP/IP スタックバックエンド選択子
typedef enum {
    ESP_NETIF_TCPIP_LWIP = 0,        // lwIP stack / lwIP スタック
    ESP_NETIF_TCPIP_OTHER,           // external/custom stack / 外部・独自スタック
} esp_netif_stack_type_t;

// Timing / reporting tuning hints (referenced by configs; inert here)
// タイミング / 通知のチューニングヒント（構成が参照; ここでは無効）
#define ESP_NETIF_USES_TCPIP_WITH_BSD_API 1
#define ESP_NETIF_REPORT_DATA_TRAFFIC     1
#define ESP_NETIF_IP_LOST_TIMER_INTERVAL  120

// -----------------------------------------------------------------------------
// Lifecycle & query API (inert host stubs)
// ライフサイクル・問い合わせ API（ホスト用無効スタブ）
// -----------------------------------------------------------------------------

// Singleton dummy interfaces handed out by the default creators
// デフォルト生成関数が払い出す単一のダミーインターフェース
struct esp_netif_obj {
    int placeholder; // unused / 未使用
};

static inline esp_netif_t* esp_netif_host_sta_singleton_(void)
{
    static struct esp_netif_obj sta_iface = {0};
    return &sta_iface;
}

static inline esp_netif_t* esp_netif_host_ap_singleton_(void)
{
    static struct esp_netif_obj ap_iface = {0};
    return &ap_iface;
}

// Initialize the underlying TCP/IP stack (inert on host)
// 下位 TCP/IP スタックを初期化（ホストでは無効動作）
static inline esp_err_t esp_netif_init(void)
{
    return ESP_OK;
}

// De-initialize the underlying TCP/IP stack
// 下位 TCP/IP スタックを終了
static inline esp_err_t esp_netif_deinit(void)
{
    return ESP_OK;
}

// Create the default WiFi STA interface (returns non-null dummy handle)
// デフォルトの WiFi STA インターフェースを生成（非 NULL のダミーを返す）
static inline esp_netif_t* esp_netif_create_default_wifi_sta(void)
{
    return esp_netif_host_sta_singleton_();
}

// Create the default WiFi soft-AP interface (returns non-null dummy handle)
// デフォルトの WiFi soft-AP インターフェースを生成（非 NULL のダミーを返す）
static inline esp_netif_t* esp_netif_create_default_wifi_ap(void)
{
    return esp_netif_host_ap_singleton_();
}

// Destroy the default WiFi STA interface
// デフォルトの WiFi STA インターフェースを破棄
static inline void esp_netif_destroy_default_wifi(void* esp_netif)
{
    (void)esp_netif;
}

// Look up a registered interface by its well-known key (e.g. "WIFI_STA_DEF")
// 既知キー（例 "WIFI_STA_DEF"）で登録済みインターフェースを取得
static inline esp_netif_t* esp_netif_get_handle_from_ifkey(const char* if_key)
{
    if (if_key != NULL && std::strstr(if_key, "AP") != NULL) {
        return esp_netif_host_ap_singleton_();
    }
    return esp_netif_host_sta_singleton_();
}

// Set the hostname advertised by the interface (inert on host)
// インターフェースが広告するホスト名を設定（ホストでは無効動作）
static inline esp_err_t esp_netif_set_hostname(esp_netif_t* esp_netif,
                                               const char* hostname)
{
    (void)esp_netif;
    (void)hostname;
    return ESP_OK;
}

// Get the hostname of the interface
// インターフェースのホスト名を取得
static inline esp_err_t esp_netif_get_hostname(esp_netif_t* esp_netif,
                                               const char** hostname)
{
    (void)esp_netif;
    if (hostname != NULL) {
        *hostname = "stampfly-host";
    }
    return ESP_OK;
}

// Query the resolved IPv4 configuration (returns a loopback-ish address)
// 確定 IPv4 構成を取得（ループバック風アドレスを返す）
static inline esp_err_t esp_netif_get_ip_info(esp_netif_t* esp_netif,
                                              esp_netif_ip_info_t* ip_info)
{
    (void)esp_netif;
    if (ip_info != NULL) {
        std::memset(ip_info, 0, sizeof(*ip_info));
        // 127.0.0.1 / 255.0.0.0 / 127.0.0.1 (host loopback-style placeholder)
        // 127.0.0.1 / 255.0.0.0 / 127.0.0.1（ホストのループバック風プレースホルダ）
        ip_info->ip.addr      = 0x0100007F; // 127.0.0.1 (LE byte order)
        ip_info->netmask.addr = 0x000000FF; // 255.0.0.0
        ip_info->gw.addr      = 0x0100007F; // 127.0.0.1
    }
    return ESP_OK;
}

// Set the static IPv4 configuration of the interface (inert on host).
// Used by sf_comm to re-address the SoftAP to the Tello subnet (192.168.10.1).
// インターフェースの静的 IPv4 構成を設定（ホストでは無効動作）。sf_comm が SoftAP を
// Tello サブネット（192.168.10.1）へ振り直すために使う。
static inline esp_err_t esp_netif_set_ip_info(esp_netif_t* esp_netif,
                                              const esp_netif_ip_info_t* ip_info)
{
    (void)esp_netif;
    (void)ip_info;
    return ESP_OK;
}

// Stop / start the DHCP server on the interface (inert on host). The real device
// must stop the server before set_ip_info and restart it afterwards.
// インターフェースの DHCP サーバを停止 / 開始（ホストでは無効動作）。実機は set_ip_info の
// 前に停止し、後で再開する必要がある。
static inline esp_err_t esp_netif_dhcps_stop(esp_netif_t* esp_netif)
{
    (void)esp_netif;
    return ESP_OK;
}

static inline esp_err_t esp_netif_dhcps_start(esp_netif_t* esp_netif)
{
    (void)esp_netif;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
