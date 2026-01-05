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

// WiFiチャンネル設定
#define ESPNOW_CHANNEL 1

// TDMA設定
#define TDMA_DEVICE_ID 0         // デバイスID: 0=マスター, 1-9=スレーブ
#define TDMA_FRAME_US 20000      // 1フレーム = 20ms
#define TDMA_SLOT_US 2000        // 1スロット = 2ms
#define TDMA_NUM_SLOTS 10        // スロット数
#define TDMA_BEACON_ADVANCE_US 500  // ビーコン先行時間

// 制御パケットサイズ
#define CONTROL_PACKET_SIZE 14

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

/**
 * @brief ペアリング処理
 * @param force_pairing 強制ペアリングフラグ
 * @return ESP_OK: 成功
 */
esp_err_t peering_process(bool force_pairing);

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
