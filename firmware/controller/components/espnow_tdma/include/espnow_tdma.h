/*
 * ESP-NOW TDMA通信モジュール - ESP-IDF版
 *
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 */
#ifndef ESPNOW_TDMA_H
#define ESPNOW_TDMA_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFiチャンネル設定（デフォルト値）
#define ESPNOW_CHANNEL_DEFAULT 1
#define ESPNOW_CHANNEL_MIN 1
#define ESPNOW_CHANNEL_MAX 13

// TDMA設定（デフォルト値）
#define TDMA_DEVICE_ID_DEFAULT 0  // デバイスID: 0=マスター, 1-9=スレーブ
#define TDMA_DEVICE_ID_MIN 0
#define TDMA_DEVICE_ID_MAX 9
#define TDMA_FRAME_US 20000       // 1フレーム = 20ms
#define TDMA_SLOT_US 2000        // 1スロット = 2ms
#define TDMA_NUM_SLOTS 10        // スロット数
#define TDMA_BEACON_ADVANCE_US 500  // ビーコン先行時間

// 制御パケットサイズ
#define CONTROL_PACKET_SIZE 14

// ランタイム設定値（NVSから読み込み可能）
// Runtime configuration (can be loaded from NVS)
extern uint8_t g_espnow_channel;
extern uint8_t g_tdma_device_id;

// ドローンMACアドレス (ペアリングで更新される)
extern uint8_t Drone_mac[6];

// 送信データバッファ (loop()からTDMAタスクへ)
extern uint8_t shared_senddata[CONTROL_PACKET_SIZE];

// TDMA同期状態
extern volatile bool first_beacon_received;
extern volatile int64_t last_beacon_time_us;

// 送信統計
extern volatile uint32_t send_success_count;
extern volatile uint32_t send_fail_count;
extern volatile uint32_t actual_send_freq_hz;

// ドローン接続状態
extern volatile bool drone_available;

/**
 * @brief チャンネル設定（espnow_init前に呼び出し）
 * @param channel WiFiチャンネル (1-13)
 */
void espnow_set_channel(uint8_t channel);

/**
 * @brief チャンネル取得
 * @return 現在のチャンネル
 */
uint8_t espnow_get_channel(void);

/**
 * @brief デバイスID設定（tdma_init前に呼び出し）
 * @param device_id デバイスID (0-9)
 */
void tdma_set_device_id(uint8_t device_id);

/**
 * @brief デバイスID取得
 * @return 現在のデバイスID
 */
uint8_t tdma_get_device_id(void);

/**
 * @brief ESP-NOW + WiFi初期化
 * @return ESP_OK: 成功
 */
esp_err_t espnow_init(void);

/**
 * @brief ブロードキャストピア(ビーコン用)初期化
 * @return ESP_OK: 成功
 */
esp_err_t beacon_peer_init(void);

/**
 * @brief ドローンピア初期化
 * @return ESP_OK: 成功
 */
esp_err_t drone_peer_init(void);

/**
 * @brief TDMAタイマーとタスク初期化
 * @return ESP_OK: 成功
 */
esp_err_t tdma_init(void);

/**
 * @brief TDMAタイマー開始
 * @return ESP_OK: 成功
 */
esp_err_t tdma_start(void);

// ============================================================================
// Pairing candidate table
// ペアリング候補表
//
// While pairing is in progress the controller no longer adopts the first
// PairingPacket it hears. Instead, every vehicle heard while scanning is
// upserted into this small fixed-size table (keyed by MAC) so main.cpp can
// show a selectable list on the LCD and let the user confirm which vehicle
// to adopt.
// ペアリング中は「最初に受信した1通」を無条件採用しない。走査中に聞こえた
// 全ての機体をこの固定長の表（MACをキーに更新）へ集め、main.cppがLCDに
// 選択可能な一覧として表示し、利用者がどの機体を採用するか確定できるように
// する。
// ============================================================================

// 候補表の最大件数（LCDの表示行数にも合わせた上限）
// Max tracked candidates (also bounds how many rows the LCD needs to show)
#define PAIRING_CANDIDATE_MAX 8

typedef struct {
    uint8_t  mac[6];        // 機体MACアドレス（フル6バイト） / vehicle MAC address (full 6 bytes)
    uint8_t  channel;       // 受信したチャンネル / channel the packet arrived on
    int8_t   rssi_dbm;      // 受信強度 (dBm)。ESP-IDF v5.xの esp_now_recv_info_t::rx_ctrl->rssi から取得
                             // received signal strength (dBm), read from esp_now_recv_info_t::rx_ctrl->rssi (ESP-IDF v5.x)
    uint32_t last_seen_ms;  // 最終受信時刻 (millis_now()基準) / time of the last packet from this MAC
} pairing_candidate_t;

/**
 * @brief ペアリングが必要か判定する
 * @param force_pairing 起動時にボタンで強制ペアリングが指示されたか
 * @return true: ペアリング（走査・選択UI）が必要
 */
bool pairing_is_needed(bool force_pairing);

/**
 * @brief 候補表を空にする（新しいペアリングセッションの開始時に呼ぶ）
 */
void pairing_candidates_reset(void);

/**
 * @brief ペアリング走査を開始/再開する（is_peeringを立て、チャンネルホップ用の
 *        内部タイマーをリセットする）。候補表はクリアしない
 * @note 候補表を空にしたい場合は別途 pairing_candidates_reset() を呼ぶこと
 */
void pairing_scan_start(void);

/**
 * @brief ペアリング走査を1ステップ進める。呼び出し元（main.cppのUIループ）が
 *        継続的に（例: 10〜20ms毎に）呼ぶことを前提とした非ブロッキング関数
 *        チャンネルのドウェル時間が経過していれば次チャンネルへホップし、
 *        ビープ間隔が経過していればビープを鳴らす
 */
void pairing_scan_service(void);

/**
 * @brief ペアリング走査を停止する（is_peeringを下げる）
 */
void pairing_scan_stop(void);

/**
 * @brief 現在の候補数を取得する
 */
uint8_t pairing_candidate_count(void);

/**
 * @brief 受信強度の強い順（同値は先着順で安定）でランクN番目の候補を取得する
 * @param rank 0が最も受信強度が強い候補
 * @param out 取得結果の格納先
 * @return true: 取得成功, false: rankが候補数の範囲外
 */
bool pairing_candidate_get_by_rank(uint8_t rank, pairing_candidate_t* out);

/**
 * @brief 指定候補を採用してチャンネル・MACを確定する
 *        Drone_mac / g_espnow_channel を更新し、WiFiチャンネルを適用し、
 *        走査を停止する（呼び出し後はdrone_peer_init()の呼び直しが必要）
 * @param candidate 採用する候補（pairing_candidate_get_by_rank()で取得したもの）
 */
void pairing_confirm(const pairing_candidate_t* candidate);

/**
 * @brief 確定した機体（drone_peer）宛にスティック中立・未武装の操縦電文を
 *        1通送信する。機体側の受理条件（自分宛の電文を発見）を満たすための
 *        呼びかけであり、機体を飛行させるものではない
 * @note 呼び出し前に drone_peer_init() でピア登録が済んでいること
 * @return ESP_OK: 送信要求成功
 */
esp_err_t pairing_send_probe(void);

/**
 * @brief 確定した機体からの応答受信フラグをクリアする
 *        （pairing_confirm()後、応答待ちを開始する直前に呼ぶ）
 */
void pairing_link_reset(void);

/**
 * @brief ペアリング成立（確定した機体が生きて応答している）を確認できたかを返す
 *
 * 2通りの経路のいずれかで立つ: (1) 確定した機体(drone_peer)宛の送信が
 * ESP-NOWのMAC層ACKで成功した（pairing_send_probe()の送信結果）。
 * firmware/vehicleは現状ESP-NOW経由で何も送り返さない（テレメトリはUDP）ため、
 * これが主経路になる。(2) 確定した機体（Drone_mac）から、まだペアリング
 * 広報中の形（PairingPacket形式）ではない電文を受信した。firmware/vehicle_old等、
 * ESP-NOW経由で何か送り返す相手向けの補助経路
 *
 * @return true: ペアリング成立が確認できた（上記いずれかの経路で確認）
 */
bool pairing_link_confirmed(void);

/**
 * @brief ピア情報をSPIFFSに保存
 * @return ESP_OK: 成功
 */
esp_err_t peer_info_save(void);

/**
 * @brief ピア情報をSPIFFSから読み込み
 * @return ESP_OK: 成功
 */
esp_err_t peer_info_load(void);

/**
 * @brief 送信データ更新 (mutex保護付き)
 * @param data 14バイトの制御データ
 * @return ESP_OK: 成功
 */
esp_err_t tdma_update_senddata(const uint8_t* data);

/**
 * @brief ドローンピアのMACアドレス取得
 * @return MACアドレスへのポインタ
 */
const uint8_t* get_drone_peer_addr(void);

/**
 * @brief ビーコンロスト判定
 * @return true: ビーコンロスト中
 */
bool is_beacon_lost(void);

#ifdef __cplusplus
}
#endif

#endif // ESPNOW_TDMA_H
