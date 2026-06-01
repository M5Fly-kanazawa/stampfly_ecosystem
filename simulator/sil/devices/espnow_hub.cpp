/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file espnow_hub.cpp
 * @brief Shared ESP-NOW callback hub + virtual-pilot delivery point.
 *        ESP-NOW コールバック共有ハブ＋仮想パイロットの配信口。
 *
 * esp_now.h registers the firmware's receive/send callbacks here (ONE shared
 * slot, not a per-translation-unit static), so a virtual transmitter living in
 * a different source file can deliver real ESP-NOW frames into the firmware by
 * calling sil_espnow_deliver(). That is the only firmware-agnostic seam for the
 * radio: the bytes on the air are reproduced; the RF layer is not (RESET_PLAN
 * §11). The frame is handed to the firmware's recv callback exactly as the Wi-Fi
 * stack would, so the unmodified comm/decoder/command path runs.
 *
 * esp_now.h が本体の受信/送信コールバックをここ（TU ごとの static でなく単一の共有
 * スロット）に登録する。別ファイルの仮想送信機が sil_espnow_deliver() を呼べば実
 * ESP-NOW フレームを本体へ配信できる。電波上のバイト列のみ再現し RF 層は再現しない（§11）。
 */

#include "esp_now.h"

namespace {

// Single shared callback slots (the firmware registers exactly one of each).
// 単一の共有コールバックスロット（本体は各1個だけ登録する）。
esp_now_recv_cb_t g_recv_cb = nullptr;
esp_now_send_cb_t g_send_cb = nullptr;

}  // namespace

extern "C" {

esp_err_t sil_espnow_set_recv_cb(esp_now_recv_cb_t cb) { g_recv_cb = cb; return ESP_OK; }
esp_err_t sil_espnow_set_send_cb(esp_now_send_cb_t cb) { g_send_cb = cb; return ESP_OK; }

// Deliver one frame into the firmware's receive callback, as the Wi-Fi stack
// would on real hardware. Called by the virtual pilot (holding the run-token),
// so the callback runs cooperatively like a normal task step.
// 本体の受信コールバックへ1フレーム配信（実機の Wi-Fi スタック相当）。仮想パイロット
// が run-token を保持して呼ぶので、通常のタスクステップと同様に協調的に走る。
void sil_espnow_deliver(const uint8_t* src_mac, const uint8_t* data, int len)
{
    if (g_recv_cb == nullptr) return;
    static uint8_t s_src[6];
    static uint8_t s_des[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i) s_src[i] = src_mac[i];
    esp_now_recv_info_t info{};
    info.src_addr = s_src;
    info.des_addr = s_des;
    info.rx_ctrl  = nullptr;
    g_recv_cb(&info, data, len);
}

// Report a send completion to the firmware's send callback (delivery "succeeds"
// on the inert link). Lets any code that waits on the send-done signal proceed.
// 送信完了を本体の送信コールバックへ通知（inert リンクでは「成功」扱い）。
void sil_espnow_report_send(const uint8_t* des_mac)
{
    if (g_send_cb == nullptr) return;
    esp_now_send_info_t info{};
    for (int i = 0; i < 6; ++i) info.des_addr[i] = des_mac ? des_mac[i] : 0;
    g_send_cb(&info, ESP_NOW_SEND_SUCCESS);
}

}  // extern "C"
