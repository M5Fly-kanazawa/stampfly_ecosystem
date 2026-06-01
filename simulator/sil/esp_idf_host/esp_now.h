/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_now.h
 * @brief Host shim for the ESP-NOW API
 *        ESP-NOW API のホスト用シム
 *
 * Provides the esp_now_* types and functions the firmware references so it
 * compiles and links on a PC. For E0 the transport is inert: register_recv_cb
 * and register_send_cb just stash the callback in a static slot, and send is a
 * no-op returning ESP_OK. The virtual-pilot wiring (delivering frames into the
 * stored recv callback) is added in E3.
 *
 * 本体ファームが参照する esp_now_* 型・関数を提供し、PC 上でコンパイル・
 * リンクできるようにする。E0 では転送は無動作で、register_recv_cb /
 * register_send_cb はコールバックを静的領域に保存するだけ、send は何もせず
 * ESP_OK を返す。仮想パイロット配線（保存した recv cb へフレームを渡す処理）
 * は E3 で追加する。
 */

#pragma once

#include "esp_err.h"
#include "esp_wifi.h"  /* wifi_interface_t (peer .ifidx) / ピアの .ifidx 用 */
#include <cstdint>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * Constants / 定数
 * ===========================================================================*/

/* Maximum ESP-NOW data payload, in bytes / ESP-NOW 最大ペイロード長（バイト） */
#define ESP_NOW_MAX_DATA_LEN     250

/* Encryption key length, in bytes / 暗号鍵の長さ（バイト） */
#define ESP_NOW_KEY_LEN          16

/* Ethernet / peer address length, in bytes / アドレス長（バイト） */
#define ESP_NOW_ETH_ALEN         6

/* Maximum number of peers / 最大ピア数 */
#define ESP_NOW_MAX_TOTAL_PEER_NUM    20
#define ESP_NOW_MAX_ENCRYPT_PEER_NUM  6

/* ===========================================================================
 * Enums / 列挙型
 * ===========================================================================*/

/* Send completion status / 送信完了ステータス */
typedef enum {
    ESP_NOW_SEND_SUCCESS = 0,  /* delivered / 配信成功 */
    ESP_NOW_SEND_FAIL          /* delivery failed / 配信失敗 */
} esp_now_send_status_t;

/* ===========================================================================
 * RX control / packet meta — minimal host stand-in
 * 受信制御 / パケットメタ — ホスト用の最小スタンドイン
 * ===========================================================================*/

/* Wi-Fi RX control info attached to received frames. The host fills zeros.
 * 受信フレームに付随する RX 制御情報。ホストはゼロ埋めする。 */
typedef struct {
    signed   rssi          : 8;   /* signal strength / 信号強度 */
    unsigned rate          : 5;
    unsigned                : 1;
    unsigned sig_mode      : 2;
    unsigned                : 16;
    unsigned mcs           : 7;
    unsigned cwb           : 1;
    unsigned                : 16;
    unsigned smoothing     : 1;
    unsigned not_sounding  : 1;
    unsigned                : 1;
    unsigned aggregation   : 1;
    unsigned stbc          : 2;
    unsigned fec_coding    : 1;
    unsigned sgi           : 1;
    signed   noise_floor   : 8;
    unsigned ampdu_cnt     : 8;
    unsigned channel       : 4;
    unsigned secondary_channel : 4;
    unsigned                : 8;
    unsigned timestamp     : 32;
    unsigned                : 32;
    unsigned                : 31;
    unsigned ant           : 1;
    unsigned sig_len       : 12;
    unsigned                : 12;
    unsigned rx_state      : 8;
} wifi_pkt_rx_ctrl_t;

/* ===========================================================================
 * Peer / callback info structs / ピア・コールバック情報構造体
 * ===========================================================================*/

/* Peer descriptor passed to esp_now_add_peer() / add_peer に渡すピア記述子 */
typedef struct {
    uint8_t          peer_addr[ESP_NOW_ETH_ALEN];  /* MAC address / MAC アドレス */
    uint8_t          lmk[ESP_NOW_KEY_LEN];         /* local master key / ローカル鍵 */
    uint8_t          channel;                      /* RF channel (0 = current) / RF チャンネル */
    wifi_interface_t ifidx;                        /* interface / インターフェイス */
    bool             encrypt;                      /* encryption enabled / 暗号化有効 */
    void*            priv;                          /* user pointer / ユーザポインタ */
} esp_now_peer_info_t;

/* Metadata delivered to the receive callback / 受信コールバックに渡すメタ情報 */
typedef struct {
    uint8_t*            src_addr;  /* sender MAC / 送信元 MAC */
    uint8_t*            des_addr;  /* destination MAC / 宛先 MAC */
    wifi_pkt_rx_ctrl_t* rx_ctrl;   /* RX control info / RX 制御情報 */
} esp_now_recv_info_t;

/* Metadata delivered to the send callback / 送信コールバックに渡すメタ情報 */
typedef struct {
    uint8_t des_addr[ESP_NOW_ETH_ALEN];  /* destination MAC / 宛先 MAC */
    uint8_t src_addr[ESP_NOW_ETH_ALEN];  /* source MAC / 送信元 MAC */
} esp_now_send_info_t;

/* Receive callback type / 受信コールバック型 */
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t* esp_now_info,
                                  const uint8_t* data, int data_len);

/* Send callback type / 送信コールバック型 */
typedef void (*esp_now_send_cb_t)(const esp_now_send_info_t* tx_info,
                                  esp_now_send_status_t status);

/* ===========================================================================
 * Shared callback hub (defined in devices/espnow_hub.cpp). The callbacks live
 * in ONE shared slot, not a per-TU static, so the virtual pilot in another
 * source file can deliver frames into the firmware's recv callback.
 * 共有コールバックハブ（devices/espnow_hub.cpp に実体）。TU ごとの static でなく
 * 単一スロットに保持し、別ファイルの仮想パイロットが本体へフレームを配信できる。
 * ===========================================================================*/

esp_err_t sil_espnow_set_recv_cb(esp_now_recv_cb_t cb);
esp_err_t sil_espnow_set_send_cb(esp_now_send_cb_t cb);
void      sil_espnow_deliver(const uint8_t* src_mac, const uint8_t* data, int len);
void      sil_espnow_report_send(const uint8_t* des_mac);

/* ===========================================================================
 * Functions (stubs) / 関数（スタブ）
 * ===========================================================================*/

/* Initialize ESP-NOW / ESP-NOW を初期化する */
static inline esp_err_t esp_now_init(void)
{
    return ESP_OK;
}

/* Deinitialize ESP-NOW / ESP-NOW を終了する */
static inline esp_err_t esp_now_deinit(void)
{
    sil_espnow_set_recv_cb(nullptr);
    sil_espnow_set_send_cb(nullptr);
    return ESP_OK;
}

/* Register the receive callback / 受信コールバックを登録する */
static inline esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb)
{
    return sil_espnow_set_recv_cb(cb);
}

/* Unregister the receive callback / 受信コールバックを解除する */
static inline esp_err_t esp_now_unregister_recv_cb(void)
{
    return sil_espnow_set_recv_cb(nullptr);
}

/* Register the send callback / 送信コールバックを登録する */
static inline esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb)
{
    return sil_espnow_set_send_cb(cb);
}

/* Unregister the send callback / 送信コールバックを解除する */
static inline esp_err_t esp_now_unregister_send_cb(void)
{
    return sil_espnow_set_send_cb(nullptr);
}

/* Add a peer / ピアを追加する */
static inline esp_err_t esp_now_add_peer(const esp_now_peer_info_t* peer)
{
    (void)peer;
    return ESP_OK;
}

/* Delete a peer / ピアを削除する */
static inline esp_err_t esp_now_del_peer(const uint8_t* peer_addr)
{
    (void)peer_addr;
    return ESP_OK;
}

/* Modify an existing peer / 既存ピアを変更する */
static inline esp_err_t esp_now_mod_peer(const esp_now_peer_info_t* peer)
{
    (void)peer;
    return ESP_OK;
}

/* Fetch a peer's info / ピア情報を取得する */
static inline esp_err_t esp_now_get_peer(const uint8_t* peer_addr,
                                         esp_now_peer_info_t* peer)
{
    (void)peer_addr;
    if (peer) {
        std::memset(peer, 0, sizeof(*peer));
    }
    return ESP_OK;
}

/* Test whether a peer exists / ピアが存在するか調べる */
static inline bool esp_now_is_peer_exist(const uint8_t* peer_addr)
{
    (void)peer_addr;
    /* No peers are registered on the host. / ホストではピアは未登録。 */
    return false;
}

/* Send a frame / フレームを送信する */
static inline esp_err_t esp_now_send(const uint8_t* peer_addr,
                                     const uint8_t* data, size_t len)
{
    /* The inert link drops the bytes (no RF), but we DO report a successful send
     * completion so any firmware path that waits on the send-done callback (e.g.
     * telemetry-to-controller) proceeds instead of stalling.
     * inert リンクはバイトを捨てる（RF 無し）が、送信完了は「成功」で通知し、送信
     * 完了コールバックを待つ本体経路（例: 機体→送信機テレメトリ）が止まらないようにする。 */
    (void)data;
    (void)len;
    sil_espnow_report_send(peer_addr);
    return ESP_OK;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
