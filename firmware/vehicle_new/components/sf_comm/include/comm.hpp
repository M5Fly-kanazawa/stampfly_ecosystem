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
    // -------------------------------------------------------------------------
    // Pairing (transmitter↔vehicle binding, crosstalk prevention)
    // ペアリング（送信機↔機体のバインド、混信対策）
    //
    // Mutual MAC learning handshake, following the legacy vehicle. While the
    // StateManager has us in PairingState::Pairing (read each update() from the
    // pairing_state topic), we broadcast a PairingPacket advertising our own MAC
    // every kPairingBroadcastMs. The controller learns it and unicasts a
    // ControlPacket back; the first one fixes the controller's src MAC as our peer
    // (saved to NVS). When paired, ControlPackets from any other MAC are dropped.
    //
    // 相互 MAC 学習ハンドシェイク（旧 vehicle 踏襲）。StateManager が PairingState::Pairing
    // にしている間（pairing_state トピックを update() ごとに読む）、自 MAC を広告する
    // PairingPacket を kPairingBroadcastMs ごとに broadcast する。コントローラがそれを学習し
    // ControlPacket をユニキャストで返す。最初の1通の src MAC を相手として確定（NVS保存）。
    // ペア後は他 MAC の ControlPacket を破棄する。
    //
    // @design requirements.md §2/§7 — Pairing                              [OK]
    // @design detailed_design.md §3 — Pairing state transitions            [OK]
    // -------------------------------------------------------------------------

    /// Read the latest PairingState (pairing_state topic) and act on edges:
    /// entering Pairing clears any existing bind (re-pair) and forces an immediate
    /// broadcast; while Pairing, broadcast a PairingPacket every kPairingBroadcastMs.
    /// 最新の PairingState（pairing_state トピック）を読み、エッジで動作する: Pairing 突入で
    /// 既存バインドを破棄（再ペア）し即時送出、Pairing 中は kPairingBroadcastMs ごとに送出。
    void servicePairing();

    /// Broadcast one PairingPacket {channel, own MAC, signature} to FF:FF:FF:FF:FF:FF.
    /// PairingPacket {channel, 自MAC, 署名} を1通 broadcast する。
    void sendPairingPacket();

    /// Handle a checksum-valid ControlPacket (runs in the ESP-NOW RX callback / WiFi
    /// task). Stays LIGHT: during Pairing it only RECORDS the controller's MAC as a
    /// pending bind (CommTask finalizes it); when paired it filters by peer MAC (drops
    /// crosstalk) and forwards. src_mac is the ESP-NOW sender MAC.
    /// チェックサム検証済み ControlPacket を処理（ESP-NOW 受信コールバック=WiFiタスク文脈で
    /// 実行）。軽量に保つ: Pairing 中は相手 MAC を「保留バインド」として控えるだけ（確定は
    /// CommTask）、ペア済みは相手 MAC でフィルタ（混信破棄）して転送。src_mac は送信元 MAC。
    void handleControlPacket(const ControlPacket& pkt, const uint8_t* src_mac);

    /// Finalize a bind captured by the RX callback: do the NVS save + unicast peer
    /// registration HERE in CommTask — NOT in the WiFi RX callback (a flash write there
    /// can trip the WiFi task watchdog → reset). Called from servicePairing().
    /// 受信コールバックが控えたバインドを確定する: NVS保存＋ユニキャスト peer 登録をここ
    /// CommTask で行う — WiFi 受信コールバック内ではしない（コールバック内のフラッシュ書込は
    /// WiFi タスクのウォッチドッグ発火→リセットを招く）。servicePairing() から呼ぶ。
    void finalizePendingBind();

    /// Register the controller as a unicast peer (so we can also send to it).
    /// コントローラをユニキャスト peer として登録する（こちらからも送れるように）。
    void addUnicastPeer(const uint8_t mac[6]);

    /// Publish the current bind status as a fact (pairing_complete topic).
    /// 現在のバインド状態を事実として発行する（pairing_complete トピック）。
    void publishBindStatus(bool bound, bool restored);

    /// NVS pairing persistence (namespace kPairingNvsNamespace, key kPairingNvsKey).
    /// NVS ペアリング永続化（namespace kPairingNvsNamespace, key kPairingNvsKey）。
    void loadPairingFromNvs();
    void savePairingToNvs(const uint8_t mac[6]);
    void clearPairingFromNvs();

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

    /// PairingPacket broadcast period while searching. 500ms matches the legacy
    /// vehicle (controller_comm.cpp tick) and the controller's channel-scan dwell.
    /// 探索中の PairingPacket 送出周期。500ms は旧 vehicle と一致し、コントローラの
    /// チャンネルスキャン滞留と整合する。
    static constexpr uint32_t kPairingBroadcastMs = 500;

    /// NVS location for the paired controller MAC (separate from sf_params).
    /// ペア済みコントローラ MAC の NVS 保存先（sf_params とは別 namespace）。
    static constexpr const char* kPairingNvsNamespace = "sf_pair";
    static constexpr const char* kPairingNvsKey       = "ctrl_mac";

    bool espnow_connected_ = false;          // Link status / リンク状態
    std::atomic<int64_t> last_packet_us_{0}; // esp_timer_get_time() at last
                                             //   valid recv / 最終有効受信時刻

    // Pairing state. own_mac_ is read once at init. pairing_active_ and paired_ are
    // written by CommTask (servicePairing) / the recv callback and read by both, so
    // they are atomic. controller_mac_ is written only when (re-)binding.
    // ペアリング状態。own_mac_ は init で一度読む。pairing_active_ と paired_ は CommTask
    // （servicePairing）/ 受信コールバックが書き両者が読むため atomic。controller_mac_ は
    // （再）バインド時のみ書く。
    uint8_t own_mac_[6] = {0};               // this vehicle's STA MAC / 自機 STA MAC
    uint8_t controller_mac_[6] = {0};        // bound transmitter MAC  / バインド済み送信機MAC
    std::atomic<bool> paired_{false};        // bound to a controller  / 相手にバインド済み
    std::atomic<bool> pairing_active_{false};// StateMgr has us searching / 探索中
    bool prev_pairing_active_ = false;       // for rising-edge detect  / 立ち上がり検出用
    int64_t last_pairing_bcast_us_ = 0;      // last PairingPacket send / 最終送出時刻

    // Pending bind: the RX callback (WiFi task) records the controller MAC + sets the
    // flag; CommTask (finalizePendingBind) does the heavy NVS/peer work. pending_mac_ is
    // written BEFORE the flag (release) and read AFTER it (acquire) — the flag is the
    // synchronization barrier.
    // 保留バインド: 受信コールバック(WiFiタスク)が相手 MAC を控えフラグを立て、CommTask
    // (finalizePendingBind)が重い NVS/peer 処理を行う。pending_mac_ はフラグより前に書き
    // (release)・後に読む(acquire)。フラグが同期バリア。
    std::atomic<bool> pending_bind_{false};  // a captured MAC awaits finalize / 確定待ち
    uint8_t pending_mac_[6] = {0};            // captured controller src MAC    / 控えた送信元MAC

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
