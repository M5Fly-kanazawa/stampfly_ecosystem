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
 * UDP sockets. Incoming control packets are decoded and published to
 * the `command_setpoint` topic; failsafe code polls msSinceLastPacket()
 * to detect link loss (this module never triggers failsafe directly).
 *
 * sf_comm が WiFi 初期化を所有する。netif + イベントループ + WiFi (STA)
 * を固定チャンネルで起動した後、ESP-NOW 受信を有効化する。他コンポーネント
 * (sf_telemetry) は UDP ソケットを bind する前に WiFi が起動済みである
 * ことを前提とする。受信した制御パケットはデコードして `command_setpoint`
 * トピックに発行し、フェイルセーフは msSinceLastPacket() をポーリング
 * してリンク喪失を検出する（本モジュールはフェイルセーフを発火しない）。
 *
 * @design architecture.md §6 — Communication subsystem                  [OK]
 * @design detailed_design.md §7 — sf_comm component                     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#include "comm.hpp"
#include "topics.hpp"
#include "data_types.hpp"

#include <cstring>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char* TAG = "comm";

namespace sf {

// =============================================================================
// Protocol — ESP-NOW CommanderPacket (12 bytes, packed, little-endian)
// プロトコル — ESP-NOW CommanderPacket（12バイト、パック、リトルエンディアン）
//
// Layout / レイアウト:
//   [0..1]  throttle  uint16  0..65535 → [0.0, 1.0]
//   [2..3]  roll      int16   -32768..32767 → [-1.0, 1.0]
//   [4..5]  pitch     int16   -32768..32767 → [-1.0, 1.0]
//   [6..7]  yaw       int16   -32768..32767 → [-1.0, 1.0]
//   [8]     buttons   uint8   bit0=arm, bit1..7=mode/reserved
//   [9]     seq       uint8   monotonic sequence (wraps)
//   [10..11] crc16    uint16  CRC-16/CCITT-FALSE over bytes [0..9]
//
// Transmitter side will be matched in a separate task. Until then, listening
// on the broadcast peer (FF:FF:FF:FF:FF:FF) is acceptable for Phase 2a.
// 送信機側は別タスクで実装する。それまではブロードキャストピア
// (FF:FF:FF:FF:FF:FF) でリスンする運用を Phase 2a では許容する。
//
// @design protocol/spec/messages.yaml — ControlPacket (SSOT, 14 bytes)  [OK]
//
// The transmitter (firmware/controller, FIXED) sends THIS layout — the protocol
// Single Source of Truth. An earlier 12-byte CommanderPacket here had diverged from
// the SSOT (wrong size/scale/checksum) and would have rejected every controller
// packet (len != 12). On-air stick values are 12-bit ADC (2048-centred), NOT 0..1000.
// 送信機(firmware/controller, 固定)はこのレイアウト＝プロトコル SSOT を送る。以前の
// 12バイト CommanderPacket は SSOT から逸脱しており（サイズ/スケール/検査が違い）コント
// ローラの全パケットを len!=12 で弾いていた。on-air は 12bit ADC(中央2048)、0..1000 ではない。
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

/// 12-bit ADC stick centre and half-span. On-air channels are spring-centred at
/// 2048; (raw - centre) / half-span maps to [-1,1] (throttle clamped to [0,1]).
/// Matches firmware/controller and the proven legacy vehicle firmware.
/// 12bit ADC スティックの中央と半幅。on-air は 2048 中央、(raw-中央)/半幅 で [-1,1]
/// （スロットルは [0,1] にクランプ）。実コントローラ・実証済み旧ファームと一致。
static constexpr float kAdcCenter   = 2048.0f;
static constexpr float kAdcHalfSpan = 2048.0f;

/// ControlPacket.flags bit masks (protocol SSOT / controller_comm).
/// ControlPacket.flags のビットマスク（プロトコル SSOT / controller_comm）。
static constexpr uint8_t kFlagArm     = 0x01;  // bit0
static constexpr uint8_t kFlagFlip    = 0x02;  // bit1
static constexpr uint8_t kFlagMode    = 0x04;  // bit2
static constexpr uint8_t kFlagAltMode = 0x08;  // bit3: ALTITUDE_HOLD
static constexpr uint8_t kFlagPosMode = 0x10;  // bit4: POSITION_HOLD

/// Source ID published into CommandSetpoint.source (0 = ESP-NOW link)
/// CommandSetpoint.source に発行するソース ID（0 = ESP-NOW リンク）
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
//   1. STA netif の生成 (esp_netif_create_default_wifi_sta)
//   2. esp_wifi_init / set storage / set mode(STA) / set channel / start
//   3. esp_now_init + register recv cb
//
// @design hardware_init.md §3 — sf_board が共有 HW 資源を所有 (R1)  [--]
// -----------------------------------------------------------------------------
void Comm::init()
{
    ESP_LOGI(TAG, "Comm initializing (WiFi STA + ESP-NOW)...");

    // Publish singleton pointer so the static recv callback can reach us.
    // 静的受信コールバックが本インスタンスに到達できるようポインタを公開する。
    g_comm_instance = this;

    initWifi();
    initEspNow();

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
// Validates length and CRC, then forwards to instance method for decode +
// publish. Drops silently on any validation failure (educational note: a
// chatty log here would flood the console under noise).
// 長さと CRC を検証し、インスタンスメソッドに渡してデコード+発行する。
// 検証失敗時は静かに破棄（ログを出すとノイズ下でコンソールが溢れる）。
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

    (void)info;  // src MAC is unused in Phase 2a (broadcast peer accepted)
                 // src MAC は Phase 2a では未使用（ブロードキャスト許容）
    g_comm_instance->parseEspNowData(pkt);
}

// -----------------------------------------------------------------------------
// parseEspNowData — decode validated packet into CommandSetpoint and publish.
// parseEspNowData — 検証済みパケットを CommandSetpoint にデコードし発行する。
//
// Normalization (12-bit ADC, spring-centred at 2048; matches the controller):
//   throttle:        (raw - 2048) / 2048, clamped to [0, 1]   (2048 = zero)
//   roll/pitch/yaw:  (raw - 2048) / 2048, clamped to [-1, 1]   (2048 = center)
// flags (arm/mode) are decoded but routed in a separate change (a pilot-request
// topic → StateManager); StateManager is the sole arm/mode decision authority.
// 正規化（12bit ADC, 2048 中央, コントローラと一致）。flags(arm/mode) は別変更で
// pilot-request トピック→StateManager へ配線する（判断は StateManager が唯一行う）。
// -----------------------------------------------------------------------------
void Comm::parseEspNowData(const ControlPacket& pkt)
{
    // Build setpoint with normalized stick values.
    // 正規化されたスティック値でセットポイントを構築する。
    CommandSetpoint sp{};
    sp.throttle = clampUnit((static_cast<float>(pkt.throttle) - kAdcCenter) / kAdcHalfSpan);
    if (sp.throttle < 0.0f) sp.throttle = 0.0f;   // throttle is [0,1] (2048 = zero)
    sp.roll  = clampUnit((static_cast<float>(pkt.roll)  - kAdcCenter) / kAdcHalfSpan);
    sp.pitch = clampUnit((static_cast<float>(pkt.pitch) - kAdcCenter) / kAdcHalfSpan);
    sp.yaw   = clampUnit((static_cast<float>(pkt.yaw)   - kAdcCenter) / kAdcHalfSpan);
    sp.source    = kCommandSourceEspNow;
    sp.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    // Publish to Pub-Sub. Latest topic publish is callback-safe.
    // Pub-Sub に発行する。Latest トピックの publish はコールバック安全。
    sf::command_setpoint.publish(sp);

    // Update freshness timestamp for failsafe polling.
    // フェイルセーフのポーリング用に新鮮度タイムスタンプを更新する。
    last_packet_us_.store(esp_timer_get_time(), std::memory_order_release);

    // Forward the discrete pilot switches as FACTS on the pilot_request topic. sf_comm
    // does NOT decide arm/mode — it only reports which switches the controller sent.
    // StateManager (the sole authority) edge-detects ARM and derives the FlightMode
    // (R5: this reaches StateTask by topic, never a direct call). 離散スイッチを「事実」
    // として pilot_request に転送する。arm/mode の判断はしない（どのスイッチが来たかの報告のみ）。
    // 判断は StateManager（唯一の権限）が行う（R5: トピック経由・直接呼出禁止）。
    PilotRequest req{};
    req.arm      = (pkt.flags & kFlagArm)     != 0;
    req.acro     = (pkt.flags & kFlagMode)    != 0;
    req.alt_hold = (pkt.flags & kFlagAltMode) != 0;
    req.pos_hold = (pkt.flags & kFlagPosMode) != 0;
    req.source   = kCommandSourceEspNow;
    req.timestamp = sp.timestamp;
    sf::pilot_request.publish(req);
}

// -----------------------------------------------------------------------------
// clampUnit — clamp a float to the range [-1.0, 1.0].
// clampUnit — float を [-1.0, 1.0] に制限する。
// -----------------------------------------------------------------------------
float Comm::clampUnit(float v)
{
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
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

    // Create the default STA netif. Must be created before esp_wifi_init().
    // デフォルト STA netif を生成する。esp_wifi_init() より前に必要。
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Disable power save for low ESP-NOW latency.
    // ESP-NOW のレイテンシを下げるため省電力を無効化する。
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_start());

    // Pin radio to the ESP-NOW channel. Must be after esp_wifi_start().
    // ラジオを ESP-NOW チャンネルに固定する。esp_wifi_start() より後に呼ぶ。
    ESP_ERROR_CHECK(esp_wifi_set_channel(kWifiChannel, WIFI_SECOND_CHAN_NONE));

    // Register IP event handler so sf_telemetry can wait on WiFi readiness
    // without polling. Bit is set on IP_EVENT_STA_GOT_IP (R16).
    // sf_telemetry が polling せずに WiFi 準備を待てるよう IP イベント
    // ハンドラを登録する。IP_EVENT_STA_GOT_IP 受信時にビットをセット (R16)。
    if (g_wifi_event_group == nullptr) {
        g_wifi_event_group = xEventGroupCreate();
    }
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &onIpEvent, nullptr));

    ESP_LOGI(TAG, "WiFi STA up on channel %u (hostname '%s')",
             kWifiChannel, kHostname);
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
    peer.channel = kWifiChannel;
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
