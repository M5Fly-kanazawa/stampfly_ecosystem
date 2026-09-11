/*
 * ESP-NOW TDMA通信モジュール - ESP-IDF版
 *
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 */
#include "espnow_tdma.h"
#include <string.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <buzzer.h>

static const char* TAG = "ESPNOW";
static const char* TAG_TDMA = "TDMA";
static const char* TAG_BEACON = "BEACON";

// ランタイム設定値（デフォルト値で初期化）
// Runtime configuration (initialized with defaults)
uint8_t g_espnow_channel = ESPNOW_CHANNEL_DEFAULT;
uint8_t g_tdma_device_id = TDMA_DEVICE_ID_DEFAULT;

// ピア情報
static esp_now_peer_info_t drone_peer;
static esp_now_peer_info_t beacon_peer;

// ドローンMACアドレス (デフォルト値)
uint8_t Drone_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
static const uint8_t Beacon_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 共有送信データバッファ
uint8_t shared_senddata[CONTROL_PACKET_SIZE] = {0};
static SemaphoreHandle_t senddata_mutex = NULL;

// TDMA用タイマーとセマフォ
static esp_timer_handle_t tdma_timer = NULL;
static SemaphoreHandle_t beacon_sem = NULL;
static TaskHandle_t beacon_task_handle = NULL;
static TaskHandle_t tdma_send_task_handle = NULL;

// フレーム開始時刻 (64bit for overflow protection)
static volatile int64_t frame_start_time_us = 0;

// ビーコン同期状態
volatile bool first_beacon_received = false;
volatile int64_t last_beacon_time_us = 0;
static const uint32_t BEACON_TIMEOUT_US = 200000;  // 200ms = 10フレーム

// 送信統計
volatile uint32_t send_success_count = 0;
volatile uint32_t send_fail_count = 0;
volatile uint32_t actual_send_freq_hz = 0;

// 送信コールバック統計
static volatile uint32_t beacon_cb_success = 0;
static volatile uint32_t beacon_cb_fail = 0;
static volatile uint32_t drone_cb_success = 0;
static volatile uint32_t drone_cb_fail = 0;

// ドローン接続状態
volatile bool drone_available = true;
static volatile uint32_t drone_consecutive_failures = 0;
static const uint32_t DRONE_FAILURE_THRESHOLD = 10;
static volatile uint32_t drone_offline_time_ms = 0;
static const uint32_t DRONE_RETRY_INTERVAL_MS = 5000;

// Air time計測
static volatile int64_t send_start_time_us = 0;

// ペアリングフラグ
static volatile uint8_t is_peering = 0;

// millis()相当
static inline uint32_t millis_now(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ============================================================================
// Pairing candidate table state
// ペアリング候補表の状態
// ============================================================================

// 候補表本体（挿入順=先着順を保持したまま更新する。ランク付けは読み出し時に行う）
// Candidate storage (kept in first-seen/insertion order; ranking happens on read)
static pairing_candidate_t s_pairing_candidates[PAIRING_CANDIDATE_MAX];
static uint8_t s_pairing_candidate_count = 0;

// チャンネル走査のタイミング
// Channel-scan timing
static const uint32_t PAIRING_CHANNEL_DWELL_MS = 200;  // 1チャンネルあたりの滞留時間 / dwell time per channel
static const uint32_t PAIRING_BEEP_INTERVAL_MS = 500;  // 走査中ビープの間隔 / beep interval while scanning
static uint8_t  s_pairing_scan_channel = ESPNOW_CHANNEL_DEFAULT;
static uint32_t s_pairing_last_hop_ms = 0;
static uint32_t s_pairing_last_beep_ms = 0;

// 確定済み機体（Drone_mac）からの応答受信フラグ
// Reply-received flag for the adopted vehicle (Drone_mac)
static volatile bool s_pairing_link_confirmed = false;

// 候補表へ upsert する（既知MACなら更新、新規なら空きがあれば追加）
// Upsert into the candidate table (update if the MAC is known, else append if there is room)
static void pairing_candidate_upsert(const uint8_t* mac, uint8_t channel, int8_t rssi_dbm)
{
    uint32_t now = millis_now();

    for (uint8_t i = 0; i < s_pairing_candidate_count; i++) {
        if (memcmp(s_pairing_candidates[i].mac, mac, 6) == 0) {
            s_pairing_candidates[i].channel = channel;
            s_pairing_candidates[i].rssi_dbm = rssi_dbm;
            s_pairing_candidates[i].last_seen_ms = now;
            return;
        }
    }

    if (s_pairing_candidate_count < PAIRING_CANDIDATE_MAX) {
        pairing_candidate_t* c = &s_pairing_candidates[s_pairing_candidate_count];
        memcpy(c->mac, mac, 6);
        c->channel = channel;
        c->rssi_dbm = rssi_dbm;
        c->last_seen_ms = now;
        s_pairing_candidate_count++;
    }
    // 表が満杯の場合、新規の機体は候補表がリセットされるまで無視する
    // When the table is full, further new vehicles are ignored until the table is reset
}

// 送信コールバック
// Note: ESP-IDF v5.5+ changed callback signature to use esp_now_send_info_t
static void on_data_sent(const esp_now_send_info_t *send_info, esp_now_send_status_t status)
{
    const uint8_t *mac_addr = send_info->des_addr;
    bool is_beacon = (mac_addr[0] == 0xFF && mac_addr[5] == 0xFF);

    if (status == ESP_NOW_SEND_SUCCESS) {
        send_success_count++;
        if (is_beacon) {
            beacon_cb_success++;
        } else {
            drone_cb_success++;
            drone_consecutive_failures = 0;
            if (!drone_available) {
                drone_available = true;
                ESP_LOGI(TAG, "ドローン再接続");
            }

            // ペアリング応答待ち中の確定機体宛送信がACKされたら、リンク成立の
            // 証拠として扱う。firmware/vehicle は現状ESP-NOW経由で何も送り返さない
            // （テレメトリはUDP）ため、電文受信ではなくMAC層ACKで確認する
            // （on_data_recv側の一致チェックはfirmware/vehicle_old等、ESP-NOW経由で
            // 何か送り返す相手に対する補助的な確認経路として残す）
            // Treat an ACKed send to the adopted vehicle during the pairing
            // reply-wait as link confirmation. firmware/vehicle currently
            // sends nothing back over ESP-NOW (telemetry is UDP-only), so
            // this uses the MAC-layer ACK rather than an inbound packet
            // (the matching check in on_data_recv stays as a secondary path
            // for peers like firmware/vehicle_old that do send something
            // back over ESP-NOW)
            s_pairing_link_confirmed = true;
        }
    } else {
        send_fail_count++;
        if (is_beacon) {
            beacon_cb_fail++;
        } else {
            drone_cb_fail++;
            drone_consecutive_failures++;

            if (drone_available && drone_consecutive_failures >= DRONE_FAILURE_THRESHOLD) {
                drone_available = false;
                drone_offline_time_ms = millis_now();
                ESP_LOGW(TAG, "ドローンオフライン (連続失敗=%lu)", drone_consecutive_failures);
            }
        }
    }
}

// 受信コールバック
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (is_peering) {
        // ペアリングパケット形式:
        // Byte 0: チャンネル
        // Byte 1-6: MACアドレス
        // Byte 7-10: シグネチャ (AA 55 16 88)
        //
        // 「最初の1通」を無条件採用せず、候補表へ upsert するだけに留める。
        // 相手の確定は main.cpp の一覧選択UIで利用者が行う（pairing_confirm()）。
        // Do not adopt the first packet unconditionally — just upsert it into
        // the candidate table. The user confirms the actual peer via the
        // list-selection UI in main.cpp (pairing_confirm()).
        if (data_len >= 11 && data[7] == 0xAA && data[8] == 0x55 &&
            data[9] == 0x16 && data[10] == 0x88) {
            uint8_t recv_channel = data[0];
            if (recv_channel < ESPNOW_CHANNEL_MIN || recv_channel > ESPNOW_CHANNEL_MAX) {
                ESP_LOGW(TAG, "ペアリング受信: 無効なチャンネル=%d、無視", recv_channel);
                return;
            }
            int8_t rssi_dbm = recv_info->rx_ctrl->rssi;
            pairing_candidate_upsert(&data[1], recv_channel, rssi_dbm);
        }
    } else {
        // ビーコンパケット (2バイト: BE AC)
        if (data_len == 2 && data[0] == 0xBE && data[1] == 0xAC) {
            if (g_tdma_device_id != 0) {
                // スレーブ: ビーコン受信時刻をフレーム開始時刻として使用
                int64_t current_time = esp_timer_get_time();
                frame_start_time_us = current_time;
                last_beacon_time_us = current_time;

                if (!first_beacon_received) {
                    first_beacon_received = true;
                    ESP_LOGI(TAG_BEACON, "初回ビーコン受信");
                }
            }
        }

        // 確定済み機体からの電文（テレメトリ等）はペアリング成立の証拠とみなす。
        // ただし機体はバインド後も（StateManagerがPairing状態を抜けるまでの間）
        // PairingPacketの広報を続けることがあるため、その形（11バイト+署名）は
        // 除外する（バインド前から広報しているだけの電文を成立と誤認しないため）
        // Any packet from the adopted vehicle (telemetry included) counts as
        // pairing-link confirmation, EXCEPT a still-broadcasting PairingPacket
        // (the vehicle can keep advertising it for a while after binding, until
        // its StateManager leaves the Pairing state) — exclude that shape (11
        // bytes + signature) so a stray advertisement isn't mistaken for proof
        // of binding
        bool is_pairing_broadcast_shape =
            (data_len >= 11 && data[7] == 0xAA && data[8] == 0x55 &&
             data[9] == 0x16 && data[10] == 0x88);
        if (!is_pairing_broadcast_shape && memcmp(recv_info->src_addr, Drone_mac, 6) == 0) {
            s_pairing_link_confirmed = true;
        }
    }
}

// ビーコン送信タスク (マスターのみ)
static void beacon_task(void* parameter)
{
    uint8_t beacon_data[2] = {0xBE, 0xAC};

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!esp_now_is_peer_exist(beacon_peer.peer_addr)) {
            ESP_LOGW(TAG_BEACON, "ピア未登録、再登録中...");
            beacon_peer_init();
            continue;
        }

        esp_err_t result = esp_now_send(beacon_peer.peer_addr, beacon_data, sizeof(beacon_data));
        if (result != ESP_OK) {
            ESP_LOGD(TAG_BEACON, "ビーコン送信エラー: %d", result);
        }
    }
}

// TDMA送信タスク
static void tdma_send_task(void* parameter)
{
    uint8_t local_senddata[CONTROL_PACKET_SIZE];
    static const uint32_t WARMUP_FRAMES = 5;
    static uint32_t frame_counter = 0;

    ESP_LOGI(TAG_TDMA, "TDMA送信タスク開始");

    while (1) {
        if (xSemaphoreTake(beacon_sem, portMAX_DELAY) == pdTRUE) {
            frame_counter++;

            // ウォームアップ期間
            if (frame_counter <= WARMUP_FRAMES) {
                if (frame_counter == WARMUP_FRAMES) {
                    ESP_LOGI(TAG_TDMA, "ウォームアップ完了");
                }
                continue;
            }

            // 送信データをローカルにコピー
            if (xSemaphoreTake(senddata_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                memcpy(local_senddata, shared_senddata, CONTROL_PACKET_SIZE);
                xSemaphoreGive(senddata_mutex);
            } else {
                continue;  // Mutex タイムアウト
            }

            // ドローンがオフラインの場合は送信スキップ
            if (!drone_available) {
                uint32_t current_time_ms = millis_now();
                if (current_time_ms - drone_offline_time_ms >= DRONE_RETRY_INTERVAL_MS) {
                    drone_available = true;
                    drone_consecutive_failures = 0;
                    ESP_LOGI(TAG_TDMA, "ドローン送信再試行中...");
                } else {
                    continue;
                }
            }

            // ピア存在確認
            if (!esp_now_is_peer_exist(drone_peer.peer_addr)) {
                ESP_LOGW(TAG_TDMA, "ドローンピア未登録、再登録中...");
                drone_peer_init();
            }

            // スロット開始時刻を計算
            int64_t slot_0_start_us = frame_start_time_us + TDMA_BEACON_ADVANCE_US;
            int64_t slot_start_us = slot_0_start_us + (g_tdma_device_id * TDMA_SLOT_US);
            int64_t current_us = esp_timer_get_time();

            // スロット開始までウェイト
            if (slot_start_us > current_us) {
                int64_t wait_us = slot_start_us - current_us;
                if (wait_us > 20) {
                    esp_rom_delay_us(wait_us - 10);
                }
                while (esp_timer_get_time() < slot_start_us) {
                    // ビジーウェイト
                }
            }

            // 制御データ送信
            int64_t actual_send_time = esp_timer_get_time();
            send_start_time_us = actual_send_time;
            esp_err_t result = esp_now_send(drone_peer.peer_addr, local_senddata, CONTROL_PACKET_SIZE);

            // 送信周波数計測
            static int64_t last_send_time_us = 0;
            if (last_send_time_us > 0) {
                int64_t interval_us = actual_send_time - last_send_time_us;
                if (interval_us > 0) {
                    actual_send_freq_hz = 1000000 / interval_us;
                }
            }
            last_send_time_us = actual_send_time;

            if (result != ESP_OK) {
                ESP_LOGE(TAG_TDMA, "送信エラー: %d", result);
            }
        }
    }
}

// TDMAタイマーコールバック
static void IRAM_ATTR tdma_timer_callback(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // マスターのみフレーム開始時刻を記録（スレーブはビーコン受信で設定）
    // Only master updates frame start time (slaves use beacon reception)
    if (g_tdma_device_id == 0) {
        frame_start_time_us = esp_timer_get_time();
    }

    // マスター: ビーコン送信タスクに通知
    if (g_tdma_device_id == 0 && beacon_task_handle != NULL) {
        vTaskNotifyGiveFromISR(beacon_task_handle, &xHigherPriorityTaskWoken);
    }

    // セマフォを解放（マスター・スレーブ共通）
    // Release semaphore for both master and slave
    xSemaphoreGiveFromISR(beacon_sem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t espnow_init(void)
{
    esp_err_t ret;

    // ネットワークインターフェース初期化
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "netif初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "イベントループ作成失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // WiFi初期化
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) return ret;

    ret = esp_wifi_start();
    if (ret != ESP_OK) return ret;

    ret = esp_wifi_set_ps(WIFI_PS_NONE);  // 省電力OFF
    if (ret != ESP_OK) return ret;

    ret = esp_wifi_set_channel(g_espnow_channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) return ret;

    // ESP-NOW初期化
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // コールバック登録
    esp_now_register_send_cb(on_data_sent);
    esp_now_register_recv_cb(on_data_recv);

    ESP_LOGI(TAG, "ESP-NOW初期化完了 (CH=%d)", g_espnow_channel);
    return ESP_OK;
}

esp_err_t beacon_peer_init(void)
{
    memset(&beacon_peer, 0, sizeof(beacon_peer));
    memcpy(beacon_peer.peer_addr, Beacon_mac, 6);
    beacon_peer.channel = g_espnow_channel;
    beacon_peer.encrypt = false;
    beacon_peer.ifidx = WIFI_IF_STA;

    // 既存ピアを削除してから追加
    esp_now_del_peer(Beacon_mac);

    esp_err_t ret = esp_now_add_peer(&beacon_peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ビーコンピア追加失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ビーコンピア登録完了");
    return ESP_OK;
}

esp_err_t drone_peer_init(void)
{
    memset(&drone_peer, 0, sizeof(drone_peer));
    memcpy(drone_peer.peer_addr, Drone_mac, 6);
    drone_peer.channel = g_espnow_channel;
    drone_peer.encrypt = false;
    drone_peer.ifidx = WIFI_IF_STA;

    // 既存ピアを削除してから追加
    esp_now_del_peer(Drone_mac);

    esp_err_t ret = esp_now_add_peer(&drone_peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ドローンピア追加失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ドローンピア登録: MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             Drone_mac[0], Drone_mac[1], Drone_mac[2],
             Drone_mac[3], Drone_mac[4], Drone_mac[5]);
    return ESP_OK;
}

esp_err_t tdma_init(void)
{
    // セマフォ作成
    beacon_sem = xSemaphoreCreateBinary();
    if (beacon_sem == NULL) {
        ESP_LOGE(TAG_TDMA, "セマフォ作成失敗");
        return ESP_FAIL;
    }

    // Mutex作成
    senddata_mutex = xSemaphoreCreateMutex();
    if (senddata_mutex == NULL) {
        ESP_LOGE(TAG_TDMA, "Mutex作成失敗");
        return ESP_FAIL;
    }

    // 送信データ初期化
    memset(shared_senddata, 0, sizeof(shared_senddata));
    shared_senddata[0] = Drone_mac[3];
    shared_senddata[1] = Drone_mac[4];
    shared_senddata[2] = Drone_mac[5];
    // checksum
    shared_senddata[13] = 0;
    for (int i = 0; i < 13; i++) {
        shared_senddata[13] += shared_senddata[i];
    }

    // マスター: ビーコンタスク作成
    if (g_tdma_device_id == 0) {
        BaseType_t ret = xTaskCreatePinnedToCore(
            beacon_task,
            "BeaconTask",
            4096,
            NULL,
            configMAX_PRIORITIES - 1,
            &beacon_task_handle,
            1
        );
        if (ret != pdPASS) {
            ESP_LOGE(TAG_TDMA, "ビーコンタスク作成失敗");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG_TDMA, "ビーコンタスク作成完了");
    }

    // TDMA送信タスク作成
    BaseType_t ret = xTaskCreatePinnedToCore(
        tdma_send_task,
        "TDMASendTask",
        8192,
        NULL,
        configMAX_PRIORITIES - 1,
        &tdma_send_task_handle,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG_TDMA, "TDMA送信タスク作成失敗");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_TDMA, "TDMA送信タスク作成完了");

    // タイマー作成
    esp_timer_create_args_t timer_args = {
        .callback = tdma_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tdma_timer"
    };

    esp_err_t err = esp_timer_create(&timer_args, &tdma_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TDMA, "タイマー作成失敗: %s", esp_err_to_name(err));
        return err;
    }

    frame_start_time_us = esp_timer_get_time();

    ESP_LOGI(TAG_TDMA, "TDMA初期化完了 (ID=%d)", g_tdma_device_id);
    return ESP_OK;
}

esp_err_t tdma_start(void)
{
    esp_err_t err = esp_timer_start_periodic(tdma_timer, TDMA_FRAME_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TDMA, "タイマー開始失敗: %s", esp_err_to_name(err));
        return err;
    }

    if (g_tdma_device_id == 0) {
        ESP_LOGI(TAG_TDMA, "TDMAマスター開始 (period=%d us)", TDMA_FRAME_US);
    } else {
        ESP_LOGI(TAG_TDMA, "TDMAスレーブ開始 (ID=%d, period=%d us)", g_tdma_device_id, TDMA_FRAME_US);
    }
    return ESP_OK;
}

bool pairing_is_needed(bool force_pairing)
{
    bool mac_unset =
        (Drone_mac[0] == 0xFF && Drone_mac[1] == 0xFF && Drone_mac[2] == 0xFF &&
         Drone_mac[3] == 0xFF && Drone_mac[4] == 0xFF && Drone_mac[5] == 0xFF);
    return force_pairing || mac_unset;
}

void pairing_candidates_reset(void)
{
    s_pairing_candidate_count = 0;
}

void pairing_scan_start(void)
{
    ESP_LOGI(TAG, "ペアリング走査開始（チャンネルスキャン）...");
    // Pairing scan start (channel scanning)
    is_peering = 1;

    s_pairing_scan_channel = ESPNOW_CHANNEL_MIN;
    esp_wifi_set_channel(s_pairing_scan_channel, WIFI_SECOND_CHAN_NONE);

    uint32_t now = millis_now();
    s_pairing_last_hop_ms = now;
    s_pairing_last_beep_ms = now;
}

void pairing_scan_service(void)
{
    uint32_t now = millis_now();

    // チャンネルのドウェル時間が経過していれば次チャンネルへホップする
    // Hop to the next channel once the dwell time has elapsed
    if (now - s_pairing_last_hop_ms >= PAIRING_CHANNEL_DWELL_MS) {
        s_pairing_scan_channel++;
        if (s_pairing_scan_channel > ESPNOW_CHANNEL_MAX) {
            s_pairing_scan_channel = ESPNOW_CHANNEL_MIN;
        }
        esp_wifi_set_channel(s_pairing_scan_channel, WIFI_SECOND_CHAN_NONE);
        s_pairing_last_hop_ms = now;
    }

    // 走査中であることを示すビープ（約500ms間隔）
    // Beep at ~500ms intervals to indicate the scan is active
    if (now - s_pairing_last_beep_ms >= PAIRING_BEEP_INTERVAL_MS) {
        beep();
        s_pairing_last_beep_ms = now;
    }
}

void pairing_scan_stop(void)
{
    is_peering = 0;
}

uint8_t pairing_candidate_count(void)
{
    return s_pairing_candidate_count;
}

bool pairing_candidate_get_by_rank(uint8_t rank, pairing_candidate_t* out)
{
    if (out == NULL || rank >= s_pairing_candidate_count) {
        return false;
    }

    // 受信強度降順でランク付け（同値は先着順=挿入順のまま、安定な挿入ソート）
    // Rank by RSSI descending (ties keep first-seen/insertion order: stable insertion sort)
    uint8_t order[PAIRING_CANDIDATE_MAX];
    for (uint8_t i = 0; i < s_pairing_candidate_count; i++) {
        order[i] = i;
    }
    for (uint8_t i = 1; i < s_pairing_candidate_count; i++) {
        uint8_t key_index = order[i];
        int8_t key_rssi = s_pairing_candidates[key_index].rssi_dbm;
        int j = (int)i - 1;
        while (j >= 0 && s_pairing_candidates[order[j]].rssi_dbm < key_rssi) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key_index;
    }

    *out = s_pairing_candidates[order[rank]];
    return true;
}

void pairing_confirm(const pairing_candidate_t* candidate)
{
    memcpy(Drone_mac, candidate->mac, 6);
    g_espnow_channel = candidate->channel;
    esp_wifi_set_channel(g_espnow_channel, WIFI_SECOND_CHAN_NONE);
    pairing_scan_stop();

    ESP_LOGI(TAG, "ペアリング確定: CH=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             g_espnow_channel,
             Drone_mac[0], Drone_mac[1], Drone_mac[2],
             Drone_mac[3], Drone_mac[4], Drone_mac[5]);
}

esp_err_t pairing_send_probe(void)
{
    if (!esp_now_is_peer_exist(drone_peer.peer_addr)) {
        return ESP_ERR_INVALID_STATE;
    }

    // スティック中立・未武装の操縦電文を組み立てる（ControlPacketのレイアウトを
    // 手詰めする。espnow_tdma.cはCファイルでprotocol/espnow_protocol.hppのC++構造体を
    // 直接使えないため、tdma_init()の初期送信データ作成と同じ手順を踏む）
    // Build a neutral (centered stick, unarmed) control packet by hand (this is
    // a C file and cannot include the C++ ControlPacket struct from
    // protocol/espnow_protocol.hpp, so pack the bytes the same way
    // tdma_init() does for the initial send buffer)
    static const uint16_t STICK_CENTER_VALUE = 2048;  // 12bit ADC中央値=ニュートラル / 12-bit ADC midpoint = neutral
    uint8_t probe[CONTROL_PACKET_SIZE] = {0};
    probe[0] = Drone_mac[3];
    probe[1] = Drone_mac[4];
    probe[2] = Drone_mac[5];
    probe[3] = (uint8_t)(STICK_CENTER_VALUE & 0xFF);        // throttle (LSB)
    probe[4] = (uint8_t)((STICK_CENTER_VALUE >> 8) & 0xFF); // throttle (MSB)
    probe[5] = (uint8_t)(STICK_CENTER_VALUE & 0xFF);        // roll (LSB)
    probe[6] = (uint8_t)((STICK_CENTER_VALUE >> 8) & 0xFF); // roll (MSB)
    probe[7] = (uint8_t)(STICK_CENTER_VALUE & 0xFF);        // pitch (LSB)
    probe[8] = (uint8_t)((STICK_CENTER_VALUE >> 8) & 0xFF); // pitch (MSB)
    probe[9] = (uint8_t)(STICK_CENTER_VALUE & 0xFF);        // yaw (LSB)
    probe[10] = (uint8_t)((STICK_CENTER_VALUE >> 8) & 0xFF); // yaw (MSB)
    probe[11] = 0;  // flags: 全ビット0=未武装 / all bits 0 = unarmed
    probe[12] = 0;  // reserved
    probe[13] = 0;
    for (int i = 0; i < 13; i++) {
        probe[13] += probe[i];
    }

    return esp_now_send(drone_peer.peer_addr, probe, sizeof(probe));
}

void pairing_link_reset(void)
{
    s_pairing_link_confirmed = false;
}

bool pairing_link_confirmed(void)
{
    return s_pairing_link_confirmed;
}

esp_err_t peer_info_save(void)
{
    FILE* fp = fopen("/spiffs/peer_info.txt", "w");
    if (fp == NULL) {
        ESP_LOGE(TAG, "ファイルオープン失敗");
        return ESP_FAIL;
    }

    fprintf(fp, "%d,%02X,%02X,%02X,%02X,%02X,%02X",
            g_espnow_channel,
            Drone_mac[0], Drone_mac[1], Drone_mac[2],
            Drone_mac[3], Drone_mac[4], Drone_mac[5]);
    fclose(fp);

    ESP_LOGI(TAG, "ピア情報保存完了");
    return ESP_OK;
}

esp_err_t peer_info_load(void)
{
    // SPIFFS初期化
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            // 既にマウント済み
        } else {
            ESP_LOGE(TAG, "SPIFFS初期化失敗: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    FILE* fp = fopen("/spiffs/peer_info.txt", "r");
    if (fp == NULL) {
        ESP_LOGW(TAG, "ピア情報ファイルなし (初回起動)");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t saved_channel;
    int n = fscanf(fp, "%hhd,%hhX,%hhX,%hhX,%hhX,%hhX,%hhX",
                   &saved_channel,
                   &Drone_mac[0], &Drone_mac[1], &Drone_mac[2],
                   &Drone_mac[3], &Drone_mac[4], &Drone_mac[5]);
    fclose(fp);

    if (n != 7) {
        ESP_LOGW(TAG, "ピア情報パース失敗");
        return ESP_FAIL;
    }

    // チャンネルを適用（範囲チェック付き）
    // Apply channel (with range check)
    if (saved_channel >= ESPNOW_CHANNEL_MIN && saved_channel <= ESPNOW_CHANNEL_MAX) {
        g_espnow_channel = saved_channel;
    } else {
        ESP_LOGW(TAG, "保存されたチャンネル %d は無効、デフォルト使用", saved_channel);
    }

    ESP_LOGI(TAG, "ピア情報読込: CH=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             g_espnow_channel,
             Drone_mac[0], Drone_mac[1], Drone_mac[2],
             Drone_mac[3], Drone_mac[4], Drone_mac[5]);
    return ESP_OK;
}

esp_err_t tdma_update_senddata(const uint8_t* data)
{
    if (xSemaphoreTake(senddata_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        memcpy(shared_senddata, data, CONTROL_PACKET_SIZE);
        xSemaphoreGive(senddata_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

const uint8_t* get_drone_peer_addr(void)
{
    return drone_peer.peer_addr;
}

bool is_beacon_lost(void)
{
    if (g_tdma_device_id == 0 || !first_beacon_received) {
        return false;
    }
    int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
    return (time_since_beacon > BEACON_TIMEOUT_US);
}

// ============================================================================
// Channel / Device ID getter/setter
// チャンネル / デバイスID getter/setter
// ============================================================================

void espnow_set_channel(uint8_t channel)
{
    if (channel >= ESPNOW_CHANNEL_MIN && channel <= ESPNOW_CHANNEL_MAX) {
        g_espnow_channel = channel;
        ESP_LOGI(TAG, "チャンネル設定: %d", channel);
    } else {
        ESP_LOGW(TAG, "無効なチャンネル: %d (範囲: %d-%d)",
                 channel, ESPNOW_CHANNEL_MIN, ESPNOW_CHANNEL_MAX);
    }
}

uint8_t espnow_get_channel(void)
{
    return g_espnow_channel;
}

void tdma_set_device_id(uint8_t device_id)
{
    if (device_id >= TDMA_DEVICE_ID_MIN && device_id <= TDMA_DEVICE_ID_MAX) {
        g_tdma_device_id = device_id;
        ESP_LOGI(TAG_TDMA, "デバイスID設定: %d", device_id);
    } else {
        ESP_LOGW(TAG_TDMA, "無効なデバイスID: %d (範囲: %d-%d)",
                 device_id, TDMA_DEVICE_ID_MIN, TDMA_DEVICE_ID_MAX);
    }
}

uint8_t tdma_get_device_id(void)
{
    return g_tdma_device_id;
}
