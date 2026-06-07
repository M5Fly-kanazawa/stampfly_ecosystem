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
 * Manages wireless input from the transmitter (HAL, responsibility #9 —
 * the physical layer only):
 * - Owns WiFi initialization (netif + event loop + STA on fixed channel).
 * - Receives ESP-NOW control packets, validates them, and forwards the raw
 *   wire fields as a RawControlInput fact to an injected sink (no normalization).
 * - Tracks freshness; failsafe components poll msSinceLastPacket().
 *
 * The Service-layer normalization/deadband/arbitration lives in sf_command
 * (responsibility #8). CommTask owns the CommandProcessor and INJECTS it as a
 * RawInputSink via setRawInputSink(); the receive path then hands each validated
 * packet straight to it, at packet time, with no added latency. sf_comm never
 * names sf_command — it only knows a function-pointer sink over the sf_core
 * RawControlInput type, so the physical layer never depends on the command layer
 * (dependency is injected by CommTask, which owns both).
 *
 * 送信機からの無線入力を管理する（HAL・責務#9 — 物理層のみ）:
 * - WiFi 初期化を所有する（netif + イベントループ + 固定チャンネルの STA）。
 * - ESP-NOW 制御パケットを受信・検証し、生の電波フィールドを RawControlInput という
 *   「事実」として、注入された sink へ転送する（正規化なし）。
 * - 新鮮度を追跡し、フェイルセーフは msSinceLastPacket() をポーリングする。
 *
 * 正規化・デッドバンド・調停という Service 層処理は sf_command（責務#8）にある。
 * CommTask が CommandProcessor を所有し setRawInputSink() で RawInputSink として
 * 注入する。受信経路は各検証済みパケットをパケット到着時にそのまま渡す（追加レイテンシ
 * なし）。sf_comm は sf_command を名指ししない — sf_core の RawControlInput 型に対する
 * 関数ポインタ sink を知るだけなので、物理層がコマンド層に依存しない（依存は両方を所有する
 * CommTask が注入する）。
 *
 * @design architecture.md §6 — Communication subsystem                  [OK]
 * @design detailed_design.md §7 — sf_comm component                     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "esp_now.h"

#include "data_types.hpp"

namespace sf {

/// Sink that consumes a validated raw input fact. CommTask points this at the
/// CommandProcessor; sf_comm calls it without knowing what it is (dependency
/// injection — keeps the HAL free of any command-layer dependency).
/// 検証済みの生入力（事実）を消費する sink。CommTask が CommandProcessor を指す。
/// sf_comm は中身を知らずに呼ぶ（依存性注入 — HAL をコマンド層依存から切り離す）。
using RawInputSink = void (*)(const RawControlInput&);

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

    /// Inject the sink that consumes each validated raw input. Call once before
    /// init() (i.e. before the radio can deliver a packet). CommTask points this
    /// at its CommandProcessor.
    /// 各検証済み生入力を消費する sink を注入する。init() の前（＝無線がパケットを
    /// 配達し得る前）に一度呼ぶこと。CommTask が自分の CommandProcessor を指す。
    void setRawInputSink(RawInputSink sink) { raw_sink_ = sink; }

private:
    /// Static ESP-NOW receive callback (runs in WiFi task context).
    /// 静的 ESP-NOW 受信コールバック（WiFi タスクコンテキストで実行）。
    static void onEspNowRecv(const esp_now_recv_info_t* info,
                             const uint8_t* data, int len);

    /// Build a RawControlInput from a validated packet and forward it to the
    /// injected sink (no normalization here). Updates the freshness timestamp.
    /// 検証済みパケットから RawControlInput を組み、注入された sink へ転送する
    /// （ここでは正規化しない）。新鮮度タイムスタンプも更新する。
    void forwardRawInput(const ControlPacket& pkt);

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

    /// Sink injected by CommTask; called with each validated raw input fact at
    /// packet time. Set once before init(), then only read by the recv path.
    /// CommTask が注入する sink。各検証済み生入力をパケット時に渡して呼ぶ。init() 前に
    /// 一度設定し、以後は受信経路が読むだけ。
    RawInputSink raw_sink_ = nullptr;
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
