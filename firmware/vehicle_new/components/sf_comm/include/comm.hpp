/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file comm.hpp
 * @brief Communication manager — ESP-NOW receive + WiFi STA bring-up
 *        通信マネージャー — ESP-NOW受信 + WiFi STA起動
 *
 * Manages wireless input from the transmitter:
 * - Owns WiFi initialization (netif + event loop + STA on fixed channel).
 * - Receives ESP-NOW control packets, decodes, and publishes to
 *   `command_setpoint`.
 * - Tracks freshness; failsafe components poll msSinceLastPacket().
 *
 * 送信機からの無線入力を管理する:
 * - WiFi 初期化を所有する（netif + イベントループ + 固定チャンネルの STA）。
 * - ESP-NOW 制御パケットを受信・デコードし `command_setpoint` に発行する。
 * - 新鮮度を追跡し、フェイルセーフは msSinceLastPacket() をポーリングする。
 *
 * @design architecture.md §6 — Communication subsystem                  [OK]
 * @design detailed_design.md §7 — sf_comm component                     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "esp_now.h"

namespace sf {

// Forward declaration of the ESP-NOW packet type (definition in comm.cpp).
// The on-air layout is the protocol SSOT ControlPacket (14 bytes).
// ESP-NOW パケット型の前方宣言（定義は comm.cpp）。on-air はプロトコル SSOT の
// ControlPacket（14バイト）。
struct ControlPacket;

/// Communication manager
/// 通信マネージャー
class Comm {
public:
    /// Initialize WiFi (STA) and ESP-NOW. Call after NVS is initialized.
    /// WiFi (STA) と ESP-NOW を初期化する。NVS 初期化後に呼ぶこと。
    void init();

    /// Diagnostic poll (the recv callback does the real work).
    /// 診断用ポーリング（実処理は受信コールバックが行う）。
    void update();

    /// True if a packet has arrived recently (within link timeout).
    /// 直近（リンクタイムアウト内）にパケット受信があれば true。
    bool isEspNowConnected() const { return espnow_connected_; }

    /// Milliseconds since the last valid packet (UINT32_MAX if never).
    /// 最終有効パケットからの経過時間 [ms]（未受信なら UINT32_MAX）。
    uint32_t timeSinceLastPacket() const;

    /// Alias used by failsafe code: identical to timeSinceLastPacket().
    /// フェイルセーフ用エイリアス: timeSinceLastPacket() と同一。
    uint32_t msSinceLastPacket() const { return timeSinceLastPacket(); }

private:
    /// Static ESP-NOW receive callback (runs in WiFi task context).
    /// 静的 ESP-NOW 受信コールバック（WiFi タスクコンテキストで実行）。
    static void onEspNowRecv(const esp_now_recv_info_t* info,
                             const uint8_t* data, int len);

    /// Decode a validated packet and publish to `command_setpoint`.
    /// 検証済みパケットをデコードし `command_setpoint` に発行する。
    void parseEspNowData(const ControlPacket& pkt);

    /// Clamp a float to the range [-1.0, 1.0].
    /// float を [-1.0, 1.0] に制限する。
    static float clampUnit(float v);

    /// Bring up netif + event loop + WiFi STA on the ESP-NOW channel.
    /// netif + イベントループ + ESP-NOW チャンネルの WiFi STA を起動する。
    void initWifi();

    /// Initialize ESP-NOW and register the receive callback.
    /// ESP-NOW を初期化し、受信コールバックを登録する。
    void initEspNow();

    /// Timeout that demarcates "connected" from "stale". 500ms matches the
    /// existing vehicle's link-loss threshold and is well above the 50Hz
    /// transmitter cadence.
    /// "接続中" と "古い" を分けるタイムアウト。既存 vehicle のリンク喪失
    /// 閾値と一致しており、50Hz 送信周期に対して十分余裕がある。
    static constexpr uint32_t kLinkTimeoutMs = 500;

    bool espnow_connected_ = false;          // Link status / リンク状態
    std::atomic<int64_t> last_packet_us_{0}; // esp_timer_get_time() at last
                                             //   valid recv / 最終有効受信時刻
};

// -----------------------------------------------------------------------------
// WiFi readiness signaling (used by sf_telemetry to avoid polling)
// WiFi 準備完了通知 (sf_telemetry が polling を避けるために使う)
// -----------------------------------------------------------------------------

/// Block until WiFi STA has obtained an IPv4 address, or until timeout.
///
/// 内部で IP_EVENT_STA_GOT_IP を listen する EventGroup を待つため、
/// busy-poll なしで効率的。Comm::init() が IP イベントハンドラを登録
/// しているため、本関数を呼ぶ前に Comm::init() が完了している必要がある。
///
/// Internally waits on an EventGroup whose bit is set by the
/// IP_EVENT_STA_GOT_IP handler registered in Comm::init(). No busy-polling.
/// Comm::init() must have completed before this function is called.
///
/// @param timeout_ms  Maximum wait [ms]. 0 means non-blocking peek.
/// @return true if IP is acquired within the timeout, false otherwise.
bool waitForWifiReady(uint32_t timeout_ms);

}  // namespace sf
