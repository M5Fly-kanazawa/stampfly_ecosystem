/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file comm.cpp
 * @brief Communication manager — ESP-NOW receive + WiFi STA bring-up
 *        通信マネージャー — ESP-NOW受信 + WiFi STA起動
 *
 * sf_comm OWNS WiFi initialization. It brings up netif + event loop +
 * WiFi (STA) on a fixed channel, then enables ESP-NOW reception. Other
 * components (sf_telemetry) assume WiFi is already up before binding
 * UDP sockets. Incoming control packets are validated and LATCHED as a
 * RawControlInput fact (no normalization); CommTask pulls it via
 * takeLatestInput() and feeds sf_command, which publishes command_setpoint
 * and pilot_request. Failsafe code polls msSinceLastPacket() to detect link
 * loss (this module never triggers failsafe directly).
 *
 * sf_comm が WiFi 初期化を所有する。netif + イベントループ + WiFi (STA)
 * を固定チャンネルで起動した後、ESP-NOW 受信を有効化する。他コンポーネント
 * (sf_telemetry) は UDP ソケットを bind する前に WiFi が起動済みである
 * ことを前提とする。受信した制御パケットは検証して RawControlInput という
 * 「事実」として保持する（正規化なし）。CommTask が takeLatestInput() で
 * 取り出して sf_command に渡し、sf_command が command_setpoint と pilot_request
 * を発行する。フェイルセーフは msSinceLastPacket() をポーリングしてリンク喪失を
 * 検出する（本モジュールはフェイルセーフを発火しない）。
 *
 * @design architecture.md §6 — Communication subsystem                  [OK]
 * @design detailed_design.md §7 — sf_comm component                     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#include "comm.hpp"
#include "data_types.hpp"
#include "topics.hpp"    // pairing_state / pairing_complete topics
#include "sf_board.hpp"  // borrow the BSP-owned default STA netif (R1)

#include <cstring>
#include <cstdio>        // snprintf (SoftAP SSID) / snprintf（SoftAP SSID 生成）

#include "params.hpp"    // wifi.mode (boot-time telemetry network mode)
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char* TAG = "comm";

namespace sf {

// =============================================================================
// Protocol — ESP-NOW ControlPacket (14 bytes, packed, little-endian) — the SSOT
// プロトコル — ESP-NOW ControlPacket（14バイト、パック、リトルエンディアン）= SSOT
//
// Layout / レイアウト (matches the struct below and protocol/spec/messages.yaml):
//   [0..2]  drone_mac  uint8×3  destination MAC lower 3 (match-any here)
//   [3..4]  throttle   uint16   12-bit ADC, 2048 = zero (spring-centred)
//   [5..6]  roll       uint16   12-bit ADC, 2048 = center
//   [7..8]  pitch      uint16   12-bit ADC, 2048 = center
//   [9..10] yaw        uint16   12-bit ADC, 2048 = center
//   [11]    flags      uint8    bit0=ARM,1=FLIP,2=MODE,3=ALT_MODE,4=POS_MODE
//   [12]    reserved   uint8    proactive_flag (ignored)
//   [13]    checksum   uint8    low 8 bits of the sum of bytes [0..12]
//
// The transmitter (firmware/controller, FIXED) sends THIS layout — the protocol
// Single Source of Truth. An earlier 12-byte CommanderPacket here had diverged from
// the SSOT (wrong size/scale/checksum) and rejected every controller packet
// (len != 14); it was removed. On-air stick values are 12-bit ADC (2048-centred),
// NOT 0..1000. Listening on the broadcast peer (FF:FF:FF:FF:FF:FF) is used.
// 送信機(firmware/controller, 固定)はこのレイアウト＝プロトコル SSOT を送る。以前の
// 12バイト CommanderPacket は SSOT から逸脱（サイズ/スケール/検査が違い）し全パケットを
// len!=14 で弾いていたため削除した。on-air は 12bit ADC(中央2048)、0..1000 ではない。
// ブロードキャストピア (FF:FF:FF:FF:FF:FF) でリスンする。
//
// @design protocol/spec/messages.yaml — ControlPacket (SSOT, 14 bytes)  [OK]
// =============================================================================

struct ControlPacket {
    uint8_t  drone_mac[3];  // bytes 0-2 : destination MAC lower 3 (match-any here)
    uint16_t throttle;      // bytes 3-4 : 12-bit ADC, 2048=zero (spring-centred)
    uint16_t roll;          // bytes 5-6 : 12-bit ADC, 2048=center
    uint16_t pitch;         // bytes 7-8 : 12-bit ADC, 2048=center
    uint16_t yaw;           // bytes 9-10: 12-bit ADC, 2048=center
    uint8_t  flags;         // byte  11  : bit0=ARM,1=FLIP,2=MODE,3=ALT_MODE,4=POS_MODE
    uint8_t  reserved;      // byte  12  : proactive_flag (ignored)
    uint8_t  checksum;      // byte  13  : low 8 bits of the sum of bytes 0-12
} __attribute__((packed));

static_assert(sizeof(ControlPacket) == 14,
              "ControlPacket must match the protocol SSOT (14 bytes)");

// =============================================================================
// Protocol — PairingPacket (11 bytes) — the SSOT (protocol/spec/messages.yaml)
// プロトコル — PairingPacket（11バイト）= SSOT
//
// Broadcast by the vehicle while in PairingState::Pairing to advertise its own
// MAC + channel. The controller scans CH1-13 for the signature, learns the MAC,
// and then unicasts ControlPackets back. Layout / レイアウト:
//   [0]     channel    uint8     WiFi channel (this vehicle's)
//   [1..6]  drone_mac  uint8×6   this vehicle's full STA MAC
//   [7..10] signature  uint8×4   0xAA 0x55 0x16 0x88 (identifies a StampFly)
//
// 機体が PairingState::Pairing の間 broadcast し、自 MAC + チャンネルを広告する。
// コントローラは CH1-13 を署名で走査して MAC を学習し、以降 ControlPacket をユニキャスト
// で返す。署名はコントローラ（固定）と一致させること（espnow_tdma.c on_data_recv）。
//
// @design protocol/spec/messages.yaml — PairingPacket (SSOT, 11 bytes)  [OK]
// =============================================================================

/// PairingPacket signature bytes (must match the controller, FIXED firmware).
/// PairingPacket 署名バイト（固定ファームのコントローラと一致させる）。
static constexpr uint8_t kPairingSignature[4] = {0xAA, 0x55, 0x16, 0x88};

/// PairingPacket wire size (1 channel + 6 MAC + 4 signature).
/// PairingPacket の電文長（channel 1 + MAC 6 + 署名 4）。
static constexpr size_t kPairingPacketSize = 11;

/// Broadcast MAC (FF:FF:FF:FF:FF:FF) — destination for the PairingPacket.
/// ブロードキャスト MAC（FF:FF:FF:FF:FF:FF）= PairingPacket の宛先。
static constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// -----------------------------------------------------------------------------
// Constants for normalization and link configuration
// 正規化およびリンク設定の定数
// -----------------------------------------------------------------------------

/// WiFi channel used by ESP-NOW. Must match the transmitter.
/// ESP-NOW で使う WiFi チャンネル。送信機と一致させる必要がある。
static constexpr uint8_t kWifiChannel = 1;

/// Hostname advertised on the WiFi interface.
/// WiFi インターフェースに通知するホスト名。
static constexpr const char* kHostname = "stampfly-vehicle";

/// Source ID stamped into RawControlInput.source (0 = ESP-NOW link). The stick
/// normalization, deadband and flags decoding all live in sf_command now — this
/// HAL component only reports WHERE the packet came from.
/// RawControlInput.source に刻む入力ソース ID（0 = ESP-NOW リンク）。スティック正規化・
/// デッドバンド・flags デコードは全て sf_command にある。本 HAL はパケットの「出どころ」を
/// 報告するだけ。
static constexpr uint8_t kCommandSourceEspNow = 0;

// -----------------------------------------------------------------------------
// WiFi readiness EventGroup (R16 — replaces sf_telemetry polling)
// WiFi 準備完了 EventGroup (R16 — sf_telemetry polling を廃止)
// -----------------------------------------------------------------------------
//
// Event handler registered in initWifi() sets WIFI_GOT_IP_BIT when the
// STA netif obtains an IPv4 address (IP_EVENT_STA_GOT_IP). sf_telemetry
// awaits this bit instead of busy-polling esp_netif_get_ip_info().
//
// initWifi() で登録するハンドラが IP_EVENT_STA_GOT_IP 受信時に
// WIFI_GOT_IP_BIT をセットする。sf_telemetry はこのビットを待つことで
// busy-poll を回避する。

constexpr EventBits_t kWifiGotIpBit = BIT0;
EventGroupHandle_t g_wifi_event_group = nullptr;

void onIpEvent(void* /*arg*/, esp_event_base_t /*event_base*/,
               int32_t event_id, void* /*event_data*/)
{
    if (event_id == IP_EVENT_STA_GOT_IP && g_wifi_event_group != nullptr) {
        xEventGroupSetBits(g_wifi_event_group, kWifiGotIpBit);
    }
}

// WIFI_EVENT handler: SoftAP start = network ready (the AP owns its address
// immediately, no DHCP wait); STA disconnect = auto-reconnect (router reboot,
// range drop — telemetry resumes by itself; ESP-NOW is unaffected either way).
// WIFI_EVENT ハンドラ: SoftAP 開始 = ネットワーク準備完了（AP は即時に自分の
// アドレスを持ち DHCP 待ちなし）。STA 切断 = 自動再接続（ルータ再起動・距離切れ —
// テレメトリは自力で復帰。ESP-NOW はいずれでも無影響）。
void onWifiEvent(void* /*arg*/, esp_event_base_t /*event_base*/,
                 int32_t event_id, void* /*event_data*/)
{
    if (event_id == WIFI_EVENT_AP_START && g_wifi_event_group != nullptr) {
        xEventGroupSetBits(g_wifi_event_group, kWifiGotIpBit);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();   // retry; paced by the WiFi stack / 再試行（ペースは WiFi スタック任せ）
    }
}

// -----------------------------------------------------------------------------
// Singleton pointer for the static C-style ESP-NOW recv callback to reach
// the Comm instance. Set at init() time; cleared is not required (Comm is a
// long-lived service object).
// 静的 C スタイルの ESP-NOW 受信コールバックから Comm インスタンスに到達する
// ためのシングルトンポインタ。init() で設定し、クリアは不要（Comm は長寿命）。
// -----------------------------------------------------------------------------

static Comm* g_comm_instance = nullptr;

// -----------------------------------------------------------------------------
// checksum8 — low 8 bits of the byte sum (protocol SSOT checksum, byte 13).
// checksum8 — バイト総和の下位8ビット（プロトコル SSOT のチェックサム、byte 13）。
//
// The transmitter computes byte[13] = (sum of bytes[0..12]) & 0xFF. We recompute
// over the same span and compare. Matches firmware/controller and the SSOT.
// 送信機は byte[13]=(bytes[0..12]の総和)&0xFF を計算する。同じ範囲で再計算して照合。
// -----------------------------------------------------------------------------
static uint8_t checksum8(const uint8_t* data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

// =============================================================================
// Public API
// 公開 API
// =============================================================================

// -----------------------------------------------------------------------------
// init — bring up WiFi (STA) and ESP-NOW, register receive callback.
// init — WiFi (STA) と ESP-NOW を起動し、受信コールバックを登録する。
//
// Prerequisites (assumed already done by app_main / sf_board):
//   - NVS:                 nvs_flash_init() in app_main()
//   - default event loop:  sf::internal::board::init() (Phase 1, L0)
//   - TCP/IP stack:        sf::internal::board::init() (Phase 1, L0)
//
// 前提条件 (app_main / sf_board が既に行っている):
//   - NVS:                 app_main() の nvs_flash_init()
//   - デフォルト event loop: sf::internal::board::init() の Phase 1 L0
//   - TCP/IP スタック:      sf::internal::board::init() の Phase 1 L0
//
// What this function does (in order) / 本関数の実行内容 (順序):
//   1. STA netif を BSP から借用し hostname を設定 (生成は board が実施)
//   2. esp_wifi_init / set storage / set mode(STA) / set channel / start
//   3. esp_now_init + register recv cb
//
// @design hardware_init.md §3 — sf_board が共有 HW 資源を所有 (R1: netif は board 生成・comm 借用)  [OK]
// -----------------------------------------------------------------------------
void Comm::init()
{
    ESP_LOGI(TAG, "Comm initializing (WiFi STA + ESP-NOW)...");

    // Publish singleton pointer so the static recv callback can reach us.
    // 静的受信コールバックが本インスタンスに到達できるようポインタを公開する。
    g_comm_instance = this;

    initWifi();
    initEspNow();

    // Read our own STA MAC once — it is advertised in every PairingPacket so the
    // controller can learn where to unicast ControlPackets (a real vehicle never
    // matches the controller's placeholder default MAC, so pairing is required).
    // 自機 STA MAC を一度読む — PairingPacket で広告し、コントローラが ControlPacket を
    // ユニキャストする宛先を学習できるようにする（実機はコントローラの既定 placeholder MAC
    // と一致しないため、飛ばすにはペアリングが必要）。
    esp_wifi_get_mac(WIFI_IF_STA, own_mac_);

    // Restore a previously paired controller MAC from NVS. If present we are
    // immediately Paired (register it as a unicast peer); otherwise we stay
    // NotPaired and the StateManager will auto-enter Pairing on the ground.
    // 以前ペアした相手 MAC を NVS から復元する。あれば即 Paired（ユニキャスト peer 登録）、
    // 無ければ NotPaired のままで、StateManager が地上で自動的に Pairing へ入る。
    loadPairingFromNvs();
    if (paired_.load(std::memory_order_acquire)) {
        addUnicastPeer(controller_mac_);
        publishBindStatus(true, /*restored=*/true);
        ESP_LOGI(TAG, "Pairing restored: %02X:%02X:%02X:%02X:%02X:%02X",
                 controller_mac_[0], controller_mac_[1], controller_mac_[2],
                 controller_mac_[3], controller_mac_[4], controller_mac_[5]);
    } else {
        publishBindStatus(false, /*restored=*/false);
        ESP_LOGI(TAG, "No stored pairing (awaiting pairing on the ground)");
    }

    last_packet_us_ = 0;  // 0 = "no packet ever received" / 未受信
    espnow_connected_ = false;

    ESP_LOGI(TAG, "Comm initialized (channel %u)", kWifiChannel);
}

// -----------------------------------------------------------------------------
// update — diagnostic poll (the recv callback does the real work).
// update — 診断用ポーリング（実処理は受信コールバックが行う）。
// -----------------------------------------------------------------------------
void Comm::update()
{
    // Connection-state heartbeat: mark "connected" if we have ever seen a
    // packet and it arrived recently. The actual failsafe lives elsewhere.
    // 接続状態ハートビート: パケットを一度でも受信し、最近到着していれば
    // "connected" とする。実際のフェイルセーフは別所に存在する。
    espnow_connected_ = (last_packet_us_ != 0) &&
                        (timeSinceLastPacket() < kLinkTimeoutMs);

    // Drive the pairing handshake from the StateManager's PairingState.
    // StateManager の PairingState に従ってペアリングのハンドシェイクを駆動する。
    servicePairing();
}

// -----------------------------------------------------------------------------
// timeSinceLastPacket — elapsed milliseconds since last valid packet.
// timeSinceLastPacket — 最終有効パケットからの経過時間 [ms]。
//
// Returns UINT32_MAX if no packet has ever been received.
// 一度も受信していない場合は UINT32_MAX を返す。
// -----------------------------------------------------------------------------
uint32_t Comm::timeSinceLastPacket() const
{
    const int64_t last_us = last_packet_us_.load(std::memory_order_acquire);
    if (last_us == 0) {
        return UINT32_MAX;
    }
    const int64_t now_us = esp_timer_get_time();
    const int64_t delta_us = now_us - last_us;
    return (delta_us < 0) ? 0u : static_cast<uint32_t>(delta_us / 1000);
}

// =============================================================================
// ESP-NOW Receive Path
// ESP-NOW 受信経路
// =============================================================================

// -----------------------------------------------------------------------------
// onEspNowRecv — static C callback (runs in WiFi task context).
// onEspNowRecv — 静的 C コールバック（WiFi タスクコンテキストで実行される）。
//
// Validates length and CRC, then latches the raw fields. Drops silently on any
// validation failure (educational note: a chatty log here would flood the
// console under noise). NO normalization or command publishing happens here —
// that is sf_command's job, driven by CommTask.
// 長さと CRC を検証し、生フィールドを保持する。検証失敗時は静かに破棄（ログを出すと
// ノイズ下でコンソールが溢れる）。正規化やコマンド発行はここで行わない — それは
// CommTask が駆動する sf_command の仕事。
// -----------------------------------------------------------------------------
void Comm::onEspNowRecv(const esp_now_recv_info_t* info,
                        const uint8_t* data, int len)
{
    // Guard against missing instance (init not yet called).
    // インスタンス未生成（init 未実行）に対するガード。
    if (g_comm_instance == nullptr) {
        return;
    }

    // Reject anything not exactly the expected packet size.
    // 想定サイズと異なるものは拒否する。
    if (data == nullptr || len != static_cast<int>(sizeof(ControlPacket))) {
        return;
    }

    // Validate the checksum: byte 13 = sum of bytes 0-12 (low 8 bits).
    // チェックサム検証: byte 13 = bytes 0-12 の総和（下位8ビット）。
    ControlPacket pkt;
    std::memcpy(&pkt, data, sizeof(pkt));
    if (pkt.checksum != checksum8(data, sizeof(ControlPacket) - 1)) {
        return;  // bad checksum / チェックサム不一致
    }

    // Reject the all-zero frame: it passes the sum checksum (0 == 0) but no real
    // controller emits it (axes raw 0 decodes to full −1.0 deflection on every
    // axis). A zeroed buffer from a buggy sender must not become a command.
    // 全ゼロフレームを拒否: 総和チェックサムは通る（0 == 0）が実コントローラは送らない
    // （axes raw 0 は全軸フル舵 −1.0 にデコードされる）。バグった送信元のゼロバッファを
    // 指令にしてはならない。
    bool all_zero = true;
    for (int i = 0; i < len; ++i) {
        if (data[i] != 0) { all_zero = false; break; }
    }
    if (all_zero) {
        return;
    }

    // Route by the ESP-NOW sender MAC: bind during pairing, else filter crosstalk.
    // ESP-NOW 送信元 MAC で振り分ける: ペアリング中はバインド、それ以外は混信をフィルタ。
    const uint8_t* src_mac = (info != nullptr) ? info->src_addr : nullptr;
    g_comm_instance->handleControlPacket(pkt, src_mac);
}

// -----------------------------------------------------------------------------
// forwardRawInput — build a RawControlInput and forward it to the injected sink.
// forwardRawInput — RawControlInput を組み、注入された sink へ転送する。
//
// No normalization, no deadband, no flags interpretation — this HAL component
// just packages the wire fields as a FACT and hands them to the sink (which
// CommTask points at sf_command, the Service that normalizes, deadbands, and
// decodes the switches). Processing AT PACKET TIME (here, in the recv context)
// keeps the proven low-latency command path: the marginally-stable POSITION_HOLD
// scenarios are sensitive to command-delivery phase, so a polled hand-off would
// shift their dynamics. sf_comm still does not depend on sf_command — it only
// calls a function-pointer sink (dependency injected by CommTask).
// 正規化・デッドバンド・flags 解釈は一切しない — 本 HAL は電波フィールドを「事実」として
// 包み、sink（CommTask が sf_command を指す。正規化・デッドバンド・スイッチデコードを行う
// Service）へ渡すだけ。パケット到着時（ここ＝受信文脈）に処理することで実証済みの低レイテンシ
// コマンド経路を保つ: 限界安定の POSITION_HOLD シナリオはコマンド配送の位相に敏感で、ポーリング
// 受け渡しでは動特性がずれてしまう。sf_comm は依然 sf_command に依存しない — 関数ポインタ sink を
// 呼ぶだけ（依存は CommTask が注入）。
// -----------------------------------------------------------------------------
void Comm::forwardRawInput(const ControlPacket& pkt)
{
    // Update freshness timestamp for failsafe polling (always, even with no sink).
    // フェイルセーフのポーリング用に新鮮度タイムスタンプを更新（sink 無しでも常時）。
    last_packet_us_.store(esp_timer_get_time(), std::memory_order_release);

    if (raw_sink_ == nullptr) {
        return;   // no consumer wired yet / 消費者未配線
    }

    RawControlInput raw{};
    raw.throttle  = pkt.throttle;
    raw.roll      = pkt.roll;
    raw.pitch     = pkt.pitch;
    raw.yaw       = pkt.yaw;
    raw.flags     = pkt.flags;
    raw.source    = kCommandSourceEspNow;
    raw.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    // Hand the fact to the Service layer (sf_command via the injected sink).
    // 事実を Service 層（注入された sink 経由で sf_command）へ渡す。
    raw_sink_(raw);
}

// =============================================================================
// Pairing — handshake, crosstalk filter, and NVS persistence
// ペアリング — ハンドシェイク、混信フィルタ、NVS 永続化
//
// Follows the legacy vehicle's mutual MAC learning (controller_comm.cpp). The
// StateManager owns PairingState and publishes it on pairing_state; this radio
// layer only EXECUTES (broadcast while Pairing, learn/save the peer MAC, filter
// crosstalk) and reports the bind status fact on pairing_complete.
// 旧 vehicle の相互 MAC 学習を踏襲（controller_comm.cpp）。PairingState は StateManager が
// 所有し pairing_state で発行する。本無線層は実行のみ（Pairing 中の送出、相手 MAC の学習/保存、
// 混信フィルタ）を行い、バインド状態の事実を pairing_complete で報告する。
// =============================================================================

// -----------------------------------------------------------------------------
// servicePairing — drive the handshake from the StateManager's PairingState.
// servicePairing — StateManager の PairingState に従ってハンドシェイクを駆動する。
//
// Called every update() (50Hz). Detects the rising edge into Pairing (a re-pair
// request: clear any existing bind so a new controller can take over), then while
// Pairing broadcasts a PairingPacket every kPairingBroadcastMs.
// update() ごと（50Hz）に呼ばれる。Pairing への立ち上がりエッジ（再ペア要求: 既存バインドを
// 破棄し新しい相手を受け入れる）を検出し、Pairing 中は kPairingBroadcastMs ごとに送出する。
// -----------------------------------------------------------------------------
void Comm::servicePairing()
{
    // Finalize any bind the RX callback captured (heavy NVS/peer work in task context).
    // 受信コールバックが控えたバインドを確定する（重い NVS/peer 処理をタスク文脈で）。
    finalizePendingBind();

    // Read the StateManager's decision. Default-init is NotPaired (state 0), so
    // before the StateManager ever publishes we stay inert (no broadcast).
    // StateManager の決定を読む。既定値は NotPaired(state 0) ゆえ、StateManager が
    // 一度も発行する前は不活性（送出しない）。
    const PairingStatus status = pairing_state.latest();
    const bool now_active =
        (status.state == static_cast<uint8_t>(PairingState::Pairing));

    // Rising edge into Pairing = (re-)pair request. Discard any existing bind so
    // a different controller can take over, and force an immediate broadcast.
    // Pairing への立ち上がり = （再）ペア要求。既存バインドを破棄して別の相手を受け入れ可能に
    // し、即時送出させる。
    if (now_active && !prev_pairing_active_) {
        if (paired_.load(std::memory_order_acquire)) {
            clearPairingFromNvs();
            esp_now_del_peer(controller_mac_);
            std::memset(controller_mac_, 0, sizeof(controller_mac_));
            paired_.store(false, std::memory_order_release);
            publishBindStatus(false, /*restored=*/false);
            ESP_LOGI(TAG, "Re-pairing: cleared existing bind");
        }
        last_pairing_bcast_us_ = 0;  // force immediate broadcast below / 即時送出
    }
    prev_pairing_active_ = now_active;
    pairing_active_.store(now_active, std::memory_order_release);

    if (!now_active) {
        return;
    }

    // While searching, broadcast a PairingPacket every kPairingBroadcastMs.
    // 探索中は kPairingBroadcastMs ごとに PairingPacket を broadcast する。
    const int64_t now_us = esp_timer_get_time();
    if (last_pairing_bcast_us_ == 0 ||
        (now_us - last_pairing_bcast_us_) >=
            static_cast<int64_t>(kPairingBroadcastMs) * 1000) {
        sendPairingPacket();
        last_pairing_bcast_us_ = now_us;
    }
}

// -----------------------------------------------------------------------------
// sendPairingPacket — broadcast {channel, own MAC, signature} (11 bytes).
// sendPairingPacket — {channel, 自MAC, 署名}（11バイト）を broadcast する。
// -----------------------------------------------------------------------------
void Comm::sendPairingPacket()
{
    uint8_t packet[kPairingPacketSize];
    // Advertise the ACTUAL radio channel, not the compile-time constant: in STA
    // mode joined to a router the channel follows the router, and the scanning
    // transmitter must lock onto where we really are.
    // 「実際の」無線チャネルを広告する（コンパイル時定数でなく）: ルータ接続の STA
    // モードではチャネルはルータに従うため、スキャン中の送信機は実在チャネルへ
    // ロックしなければならない。
    uint8_t primary = kWifiChannel;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    packet[0] = primary;                                        // byte 0   : channel
    std::memcpy(&packet[1], own_mac_, 6);                       // bytes 1-6: our MAC
    std::memcpy(&packet[7], kPairingSignature,                  // bytes 7-10: signature
                sizeof(kPairingSignature));

    const esp_err_t ret = esp_now_send(kBroadcastMac, packet, sizeof(packet));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Pairing broadcast failed: %s", esp_err_to_name(ret));
    }
}

// -----------------------------------------------------------------------------
// handleControlPacket — bind during Pairing, else filter crosstalk and forward.
// handleControlPacket — Pairing 中はバインド、それ以外は混信をフィルタして転送。
// -----------------------------------------------------------------------------
void Comm::handleControlPacket(const ControlPacket& pkt, const uint8_t* src_mac)
{
    const bool pairing = pairing_active_.load(std::memory_order_acquire);
    const bool bound   = paired_.load(std::memory_order_acquire);

    // Pairing and not yet bound: RECORD the first controller's MAC as a pending bind.
    // We do NOT save to NVS or add a peer here — this runs in the WiFi RX callback and a
    // flash write here can trip the WiFi task watchdog (→ reset). CommTask's
    // finalizePendingBind() does the heavy work. The binding packet is not forwarded.
    // ペアリング中かつ未バインド: 最初のコントローラの MAC を「保留バインド」として控える。
    // ここでは NVS 保存も peer 登録もしない — 本処理は WiFi 受信コールバックで走り、フラッシュ
    // 書込は WiFi タスクのウォッチドッグ発火(→リセット)を招く。重い処理は CommTask の
    // finalizePendingBind() が行う。バインド用パケットは転送しない。
    if (pairing && !bound) {
        if (src_mac != nullptr && !pending_bind_.load(std::memory_order_acquire)) {
            std::memcpy(pending_mac_, src_mac, 6);
            pending_bind_.store(true, std::memory_order_release);
        }
        return;
    }

    // Not bound (and not pairing): drop. Being bound to a transmitter is the
    // precondition for accepting flight commands — an unpaired vehicle must not
    // obey an arbitrary broadcast 14-byte packet (which carries the ARM bit).
    // Boot restores the bind from NVS; a fresh vehicle auto-enters Pairing on
    // the ground (StateTask) and binds there.
    // 未バインド（かつ非ペアリング中）: 破棄。送信機へのバインドは操縦コマンド受理の
    // 前提条件 — 未ペアの機体が任意のブロードキャスト 14 バイトパケット（ARM ビット
    // 入り）に従ってはならない。起動時は NVS からバインドを復元し、新品の機体は地上で
    // 自動的に Pairing へ入り（StateTask）そこでバインドする。
    if (!bound) {
        return;
    }

    // Bound: drop ControlPackets from any controller other than our peer. A null
    // src_mac (defensive: ESP-IDF always supplies one) must NOT bypass the filter.
    // バインド済み: 相手以外のコントローラからの ControlPacket は破棄する。src_mac が
    // null の場合（防御的: ESP-IDF は常に供給する）もフィルタを素通りさせない。
    if (src_mac == nullptr ||
        std::memcmp(src_mac, controller_mac_, 6) != 0) {
        return;  // crosstalk from another transmitter / 他送信機の混信
    }

    forwardRawInput(pkt);
}

// -----------------------------------------------------------------------------
// finalizePendingBind — complete a bind captured by the RX callback (CommTask ctx).
// finalizePendingBind — 受信コールバックが控えたバインドを確定する（CommTask 文脈）。
//
// The NVS write and peer registration run HERE (in CommTask), never in the WiFi RX
// callback, so a multi-millisecond flash write cannot stall the WiFi task.
// NVS 書込と peer 登録をここ(CommTask)で行い、WiFi 受信コールバックでは行わない。数ms の
// フラッシュ書込が WiFi タスクを止めないようにするため。
// -----------------------------------------------------------------------------
void Comm::finalizePendingBind()
{
    if (!pending_bind_.load(std::memory_order_acquire)) {
        return;
    }
    std::memcpy(controller_mac_, pending_mac_, 6);  // pending_mac_ published before flag
    savePairingToNvs(controller_mac_);
    addUnicastPeer(controller_mac_);
    paired_.store(true, std::memory_order_release);  // after mac is in place
    publishBindStatus(true, /*restored=*/false);
    pending_bind_.store(false, std::memory_order_release);
    ESP_LOGI(TAG, "Paired with %02X:%02X:%02X:%02X:%02X:%02X",
             controller_mac_[0], controller_mac_[1], controller_mac_[2],
             controller_mac_[3], controller_mac_[4], controller_mac_[5]);
}

// -----------------------------------------------------------------------------
// addUnicastPeer — register the controller as a unicast peer (for sending to it).
// addUnicastPeer — コントローラをユニキャスト peer として登録する（送信用）。
// -----------------------------------------------------------------------------
void Comm::addUnicastPeer(const uint8_t mac[6])
{
    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, mac, 6);
    // channel 0 = "use the radio's current channel" (esp_now API): correct on
    // the fixed channel AND when STA mode follows a router's channel.
    // channel 0 = 「無線の現在チャネルを使う」（esp_now API）: 固定チャネルでも、
    // STA モードでルータのチャネルに従う場合でも正しい。
    peer.channel = 0;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    esp_now_del_peer(mac);  // remove a stale entry first (ignore "not found")
    const esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Unicast peer add failed: %s", esp_err_to_name(ret));
    }
}

// -----------------------------------------------------------------------------
// publishBindStatus — publish the current bind status fact (pairing_complete).
// publishBindStatus — 現在のバインド状態の事実を発行する（pairing_complete）。
// -----------------------------------------------------------------------------
void Comm::publishBindStatus(bool bound, bool restored)
{
    PairingComplete pc{};
    std::memcpy(pc.controller_mac, controller_mac_, 6);
    pc.bound     = bound;
    pc.restored  = restored;
    // Stamp a NON-ZERO timestamp so a subscriber can use timestamp!=0 to mean
    // "comm has reported its bind status". On the SIL the virtual clock can read 0
    // at init (before the scheduler advances), which would otherwise collide with
    // the "never reported" sentinel and make StateManager miss a boot-restored bind.
    // 非ゼロの timestamp を刻み、購読側が timestamp!=0 で「comm が報告済み」と判定できる
    // ようにする。SIL の仮想クロックは init 時（スケジューラ進行前）に 0 を返しうるため、
    // そのままだと「未報告」番兵と衝突し StateManager が起動時復元バインドを取りこぼす。
    const uint32_t ts = static_cast<uint32_t>(esp_timer_get_time());
    pc.timestamp = (ts != 0) ? ts : 1;
    pairing_complete.publish(pc);
}

// -----------------------------------------------------------------------------
// NVS pairing persistence — namespace "sf_pair", key "ctrl_mac" (6-byte blob).
// NVS ペアリング永続化 — namespace "sf_pair", key "ctrl_mac"（6バイトブロブ）。
// -----------------------------------------------------------------------------
void Comm::loadPairingFromNvs()
{
    nvs_handle_t handle;
    if (nvs_open(kPairingNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return;  // namespace absent -> never paired / 未ペア
    }
    uint8_t mac[6] = {0};
    size_t len = sizeof(mac);
    const esp_err_t ret = nvs_get_blob(handle, kPairingNvsKey, mac, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != 6) {
        return;
    }

    // Sanity-check the stored MAC: all-zero / broadcast can only come from a
    // corrupted blob. Treating it as "bound" would make the crosstalk filter
    // discard every legitimate packet — vehicle unflyable until a re-pair.
    // 保存 MAC の妥当性検査: 全ゼロ/ブロードキャストは壊れた blob からしか生じない。
    // それを「バインド済み」と扱うと混信フィルタが正規パケットを全て破棄し、
    // 再ペアまで操縦不能になる。
    bool all_zero = true, all_ff = true;
    for (int i = 0; i < 6; ++i) {
        if (mac[i] != 0x00) all_zero = false;
        if (mac[i] != 0xFF) all_ff = false;
    }
    if (all_zero || all_ff) {
        ESP_LOGW(TAG, "Stored pairing MAC invalid (all-%s) — treating as unpaired",
                 all_zero ? "00" : "FF");
        return;
    }

    std::memcpy(controller_mac_, mac, 6);
    paired_.store(true, std::memory_order_release);
}

void Comm::savePairingToNvs(const uint8_t mac[6])
{
    nvs_handle_t handle;
    if (nvs_open(kPairingNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "NVS open (save pairing) failed");
        return;
    }
    // Check both steps: a failed save still flies this session (the bind is in
    // RAM) but silently comes back UNPAIRED after reboot — warn the pilot that
    // persistence did not happen.
    // 両ステップを検査: 保存に失敗しても今セッションは飛べる（バインドは RAM 上）が、
    // 再起動後に黙って未ペアに戻る — 永続化されなかったことを警告する。
    const esp_err_t set_err    = nvs_set_blob(handle, kPairingNvsKey, mac, 6);
    const esp_err_t commit_err = nvs_commit(handle);
    nvs_close(handle);
    if (set_err != ESP_OK || commit_err != ESP_OK) {
        ESP_LOGW(TAG, "Pairing NVS save failed (%s/%s) — bind will NOT survive reboot",
                 esp_err_to_name(set_err), esp_err_to_name(commit_err));
    }
}

void Comm::clearPairingFromNvs()
{
    nvs_handle_t handle;
    if (nvs_open(kPairingNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_erase_key(handle, kPairingNvsKey);
    nvs_commit(handle);
    nvs_close(handle);
}

// -----------------------------------------------------------------------------
// loadWifiCredsFromNvs — telemetry WiFi credentials, namespace "sf_wifi"
// (written by the CLI `wifi ssid/pass` commands).
// loadWifiCredsFromNvs — テレメトリ WiFi 資格情報、namespace "sf_wifi"
// （CLI `wifi ssid/pass` コマンドが書き込む）。
// -----------------------------------------------------------------------------
bool Comm::loadWifiCredsFromNvs(char* ssid, size_t ssid_len,
                                char* pass, size_t pass_len)
{
    ssid[0] = '\0';
    pass[0] = '\0';

    nvs_handle_t handle;
    if (nvs_open(kWifiNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return false;   // never configured / 未設定
    }
    size_t len = ssid_len;
    const esp_err_t ssid_err = nvs_get_str(handle, kWifiNvsKeySsid, ssid, &len);
    len = pass_len;
    nvs_get_str(handle, kWifiNvsKeyPass, pass, &len);   // optional (open AP) / 任意
    nvs_close(handle);

    return ssid_err == ESP_OK && ssid[0] != '\0';
}

// =============================================================================
// Initialization Helpers
// 初期化ヘルパ
// =============================================================================

// -----------------------------------------------------------------------------
// initWifi — bring up netif + event loop + WiFi STA on a fixed channel.
// initWifi — netif + イベントループ + 固定チャンネルの WiFi STA を起動する。
//
// We pick STA mode (not AP). The transmitter is a peer reachable on the
// same channel. Other agents (sf_telemetry) need the netif up to bind UDP.
// AP モードではなく STA を選ぶ。送信機は同一チャンネルのピア。
// sf_telemetry が UDP を bind するために netif が起動済みである必要がある。
// -----------------------------------------------------------------------------
void Comm::initWifi()
{
    // TCP/IP stack and default event loop are owned by sf_board (BSP),
    // initialized in sf::internal::board::init() (Phase 1, Level 0).
    // Per v3 design rule R1, this Comm component does not duplicate
    // those calls — it relies on the BSP having brought them up first.
    //
    // TCP/IP スタックとデフォルトイベントループは sf_board (BSP) が
    // sf::internal::board::init() の Phase 1 Level 0 で所有・初期化する。
    // v3 設計ルール R1 に従い、本 Comm コンポーネントはそれらの呼び出し
    // を二重化しない。BSP が先に立ち上げている前提で動く。

    // Borrow the default STA netif from the BSP (R1: sf_board owns esp_netif and
    // created it in board::init()). We only set the application-level hostname here;
    // we do NOT create the netif — that was the old ownership-scatter anti-pattern
    // (hardware_init.md §3/§4). esp_wifi_init/start below bind to this default STA netif.
    // BSP からデフォルト STA netif を借用する (R1: sf_board が esp_netif を所有し
    // board::init() で生成済み)。ここではアプリ層の hostname のみ設定し、netif は生成しない
    // — それが旧来の所有分散アンチパターン (hardware_init.md §3/§4)。下の esp_wifi_init/start
    // はこのデフォルト STA netif に bind される。
    esp_netif_t* sta_netif = sf::internal::board::sta_netif();
    if (sta_netif != nullptr) {
        esp_netif_set_hostname(sta_netif, kHostname);
    }

    // Initialize WiFi driver with default config.
    // WiFi ドライバをデフォルト設定で初期化する。
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    // Avoid writing transient state to flash on every config change.
    // 設定変更のたびに Flash へ書き込まないようにする。
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Disable power save for low ESP-NOW latency.
    // ESP-NOW のレイテンシを下げるため省電力を無効化する。
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // WiFi readiness EventGroup + handlers (R16: sf_telemetry blocks on the bit
    // instead of polling). STA: IP_EVENT_STA_GOT_IP. AP: WIFI_EVENT_AP_START
    // (the SoftAP has its address immediately). STA disconnect → auto-reconnect.
    // WiFi 準備完了 EventGroup とハンドラ（R16: sf_telemetry はポーリングせずビットで
    // ブロック）。STA: IP_EVENT_STA_GOT_IP。AP: WIFI_EVENT_AP_START（SoftAP は即時に
    // アドレスを持つ）。STA 切断時は自動再接続。
    if (g_wifi_event_group == nullptr) {
        g_wifi_event_group = xEventGroupCreate();
    }
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &onIpEvent, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &onWifiEvent, nullptr));

    // Telemetry network mode (boot-time parameter; ESP-NOW control works in
    // every mode). 0 = STA, 1 = SoftAP.
    // テレメトリのネットワークモード（起動時パラメータ。ESP-NOW 操縦は全モードで
    // 動く）。0 = STA、1 = SoftAP。
    int32_t wifi_mode = 0;
    sf::params::get_int("wifi.mode", wifi_mode);

    if (wifi_mode == 1) {
        startSoftAp();
    } else {
        startSta();
    }
}

// -----------------------------------------------------------------------------
// startSta — STA mode. With NVS credentials (CLI `wifi ssid/pass`), join the
// router: the radio then FOLLOWS the router's channel, and the pairing
// broadcast advertises the ACTUAL channel so the scanning transmitter locks to
// it (PairingPacket byte 0). Without credentials, behave as before: ESP-NOW
// only, radio pinned to the fixed channel, telemetry inert.
// startSta — STA モード。NVS 資格情報（CLI `wifi ssid/pass`）があればルータへ接続:
// 以後ラジオはルータのチャネルに「従い」、ペアリング broadcast は「実際の」チャネルを
// 広告するためスキャン中の送信機はそこへロックする（PairingPacket byte 0）。資格情報が
// 無ければ従来どおり: ESP-NOW のみ、固定チャネルに固定、テレメトリは無効。
// -----------------------------------------------------------------------------
void Comm::startSta()
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    char ssid[33] = {};
    char pass[65] = {};
    const bool have_creds = loadWifiCredsFromNvs(ssid, sizeof(ssid),
                                                 pass, sizeof(pass));
    if (have_creds) {
        wifi_config_t sta_cfg = {};
        std::strncpy(reinterpret_cast<char*>(sta_cfg.sta.ssid), ssid,
                     sizeof(sta_cfg.sta.ssid) - 1);
        std::strncpy(reinterpret_cast<char*>(sta_cfg.sta.password), pass,
                     sizeof(sta_cfg.sta.password) - 1);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    if (have_creds) {
        // Join the router — do NOT pin the channel; the AP decides it and the
        // pairing broadcast advertises the resulting channel to the transmitter.
        // ルータへ接続 — チャネルは固定しない。チャネルは AP が決め、ペアリング
        // broadcast がその結果を送信機へ広告する。
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi STA joining '%s' (telemetry; ESP-NOW follows the AP channel)",
                 ssid);
    } else {
        // ESP-NOW-only: pin the radio to the fixed channel (legacy behavior).
        // ESP-NOW のみ: ラジオを固定チャネルに固定（従来挙動）。
        ESP_ERROR_CHECK(esp_wifi_set_channel(kWifiChannel, WIFI_SECOND_CHAN_NONE));
        ESP_LOGI(TAG, "WiFi STA up on channel %u (no credentials — ESP-NOW only; "
                      "set via CLI `wifi ssid/pass` for telemetry)", kWifiChannel);
    }
}

// -----------------------------------------------------------------------------
// startSoftAp — SoftAP mode (APSTA): the AP serves telemetry clients on the
// ESP-NOW channel, the STA interface keeps carrying ESP-NOW exactly as before
// (peers are registered on WIFI_IF_STA) — zero impact on the control link, no
// infrastructure needed. SSID "StampFly-XXYY" from the MAC tail; default WPA2
// password (8+ chars) overridable via the same NVS `pass` slot.
// startSoftAp — SoftAP モード（APSTA）: AP が ESP-NOW チャネル上でテレメトリ
// クライアントに給仕し、STA インターフェースは従来どおり ESP-NOW を運ぶ（peer は
// WIFI_IF_STA に登録）— 操縦リンクへの影響ゼロ、インフラ不要。SSID は MAC 末尾から
// "StampFly-XXYY"。既定 WPA2 パスワード（8文字以上）は NVS の `pass` で上書き可。
// -----------------------------------------------------------------------------
void Comm::startSoftAp()
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    wifi_config_t ap_cfg = {};
    std::snprintf(reinterpret_cast<char*>(ap_cfg.ap.ssid), sizeof(ap_cfg.ap.ssid),
                  "StampFly-%02X%02X", mac[4], mac[5]);
    ap_cfg.ap.ssid_len = static_cast<uint8_t>(
        std::strlen(reinterpret_cast<char*>(ap_cfg.ap.ssid)));

    char ssid_unused[33] = {};
    char pass[65] = {};
    if (!loadWifiCredsFromNvs(ssid_unused, sizeof(ssid_unused),
                              pass, sizeof(pass)) ||
        std::strlen(pass) < 8) {
        std::strncpy(pass, kApDefaultPassword, sizeof(pass) - 1);
    }
    std::strncpy(reinterpret_cast<char*>(ap_cfg.ap.password), pass,
                 sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.channel        = kWifiChannel;   // AP on the ESP-NOW channel / ESP-NOW チャネル上
    ap_cfg.ap.max_connection = 2;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi SoftAP '%s' up on channel %u (telemetry; ESP-NOW unchanged)",
             reinterpret_cast<char*>(ap_cfg.ap.ssid), kWifiChannel);
}

// -----------------------------------------------------------------------------
// waitForWifiReady — block until STA has IP, or timeout
// WiFi 準備完了待機 — STA が IP を取得するまでブロック (タイムアウト付き)
// -----------------------------------------------------------------------------
//
// Replaces the polling loop previously used by sf_telemetry::waitForWifi().
// xEventGroupWaitBits is used so that the calling task sleeps efficiently
// rather than spinning. clearOnExit=false leaves the bit set so subsequent
// callers (or reconnects) can also detect readiness.
//
// 旧 sf_telemetry::waitForWifi() の polling ループを置換する。
// xEventGroupWaitBits を使うため呼び出しタスクは効率的にスリープする。
// clearOnExit=false でビットを保持し、再呼び出しや再接続でも検出可能。
bool waitForWifiReady(uint32_t timeout_ms)
{
    if (g_wifi_event_group == nullptr) {
        // initWifi() has not run yet; nothing to wait on.
        // initWifi() 未実行。待つべき対象がない。
        return false;
    }
    const TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    const EventBits_t bits = xEventGroupWaitBits(
        g_wifi_event_group,
        kWifiGotIpBit,
        pdFALSE,    // clearOnExit: keep bit set
        pdTRUE,     // waitForAllBits (we only have one bit)
        ticks);
    return (bits & kWifiGotIpBit) != 0;
}

// -----------------------------------------------------------------------------
// initEspNow — initialize ESP-NOW and register the receive callback.
// initEspNow — ESP-NOW を初期化し、受信コールバックを登録する。
//
// We also add a broadcast peer so that an unpaired transmitter can reach
// us during Phase 2a development. Pairing/encryption arrives in Phase 3.
// 未ペアリングの送信機が Phase 2a 開発中に到達できるようブロードキャスト
// ピアを追加する。ペアリング/暗号化は Phase 3 で導入する。
// -----------------------------------------------------------------------------
void Comm::initEspNow()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(&Comm::onEspNowRecv));

    // Add broadcast peer so we can also send (for future bidirectional use).
    // 将来の双方向通信のためブロードキャストピアを追加する。
    esp_now_peer_info_t peer{};
    std::memset(peer.peer_addr, 0xFF, sizeof(peer.peer_addr));
    peer.channel = 0;   // current radio channel (see addUnicastPeer) / 現在のチャネル
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(peer.peer_addr)) {
        const esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Broadcast peer add failed: %s",
                     esp_err_to_name(ret));
        }
    }

    ESP_LOGI(TAG, "ESP-NOW initialized (broadcast peer registered)");
}

}  // namespace sf
