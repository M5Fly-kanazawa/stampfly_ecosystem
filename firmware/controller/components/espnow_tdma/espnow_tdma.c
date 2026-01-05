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
static const uint32_t BEACON_TIMEOUT_US = 50000;  // 50ms = 5フレーム

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
static volatile uint8_t received_flag = 0;

// millis()相当
static inline uint32_t millis_now(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// 送信コールバック
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
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
    (void)recv_info;  // 未使用

    if (is_peering) {
        // ペアリングパケット (AA 55 16 88 ヘッダ)
        if (data_len >= 11 && data[7] == 0xAA && data[8] == 0x55 &&
            data[9] == 0x16 && data[10] == 0x88) {
            received_flag = 1;
            // MACアドレスを取得
            Drone_mac[0] = data[1];
            Drone_mac[1] = data[2];
            Drone_mac[2] = data[3];
            Drone_mac[3] = data[4];
            Drone_mac[4] = data[5];
            Drone_mac[5] = data[6];
            ESP_LOGI(TAG, "ペアリング受信: MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                     Drone_mac[0], Drone_mac[1], Drone_mac[2],
                     Drone_mac[3], Drone_mac[4], Drone_mac[5]);
        }
    } else {
        // ビーコンパケット (2バイト: BE AC)
        if (data_len == 2 && data[0] == 0xBE && data[1] == 0xAC) {
            if (TDMA_DEVICE_ID != 0) {
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
            int64_t slot_start_us = slot_0_start_us + (TDMA_DEVICE_ID * TDMA_SLOT_US);
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

    // フレーム開始時刻を記録
    frame_start_time_us = esp_timer_get_time();

    // マスター: ビーコン送信タスクに通知
    if (TDMA_DEVICE_ID == 0 && beacon_task_handle != NULL) {
        vTaskNotifyGiveFromISR(beacon_task_handle, &xHigherPriorityTaskWoken);
    }

    // セマフォを解放
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

    ret = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
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

    ESP_LOGI(TAG, "ESP-NOW初期化完了 (CH=%d)", ESPNOW_CHANNEL);
    return ESP_OK;
}

esp_err_t beacon_peer_init(void)
{
    memset(&beacon_peer, 0, sizeof(beacon_peer));
    memcpy(beacon_peer.peer_addr, Beacon_mac, 6);
    beacon_peer.channel = ESPNOW_CHANNEL;
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
    drone_peer.channel = ESPNOW_CHANNEL;
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
    if (TDMA_DEVICE_ID == 0) {
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

    ESP_LOGI(TAG_TDMA, "TDMA初期化完了 (ID=%d)", TDMA_DEVICE_ID);
    return ESP_OK;
}

esp_err_t tdma_start(void)
{
    esp_err_t err = esp_timer_start_periodic(tdma_timer, TDMA_FRAME_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TDMA, "タイマー開始失敗: %s", esp_err_to_name(err));
        return err;
    }

    if (TDMA_DEVICE_ID == 0) {
        ESP_LOGI(TAG_TDMA, "TDMAマスター開始 (period=%d us)", TDMA_FRAME_US);
    } else {
        ESP_LOGI(TAG_TDMA, "TDMAスレーブ開始 (ID=%d, period=%d us)", TDMA_DEVICE_ID, TDMA_FRAME_US);
    }
    return ESP_OK;
}

esp_err_t peering_process(bool force_pairing)
{
    bool need_pairing = force_pairing ||
        (Drone_mac[0] == 0xFF && Drone_mac[1] == 0xFF && Drone_mac[2] == 0xFF &&
         Drone_mac[3] == 0xFF && Drone_mac[4] == 0xFF && Drone_mac[5] == 0xFF);

    if (!need_pairing) {
        ESP_LOGI(TAG, "既存のペア情報を使用");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "ペアリングモード開始...");
    is_peering = 1;
    received_flag = 0;

    uint32_t beep_delay = 0;

    // ペアリング待機
    while (received_flag == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (millis_now() - beep_delay >= 500) {
            beep();
            beep_delay = millis_now();
        }
    }

    is_peering = 0;
    ESP_LOGI(TAG, "ペアリング完了: MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             Drone_mac[0], Drone_mac[1], Drone_mac[2],
             Drone_mac[3], Drone_mac[4], Drone_mac[5]);

    return ESP_OK;
}

esp_err_t peer_info_save(void)
{
    FILE* fp = fopen("/spiffs/peer_info.txt", "w");
    if (fp == NULL) {
        ESP_LOGE(TAG, "ファイルオープン失敗");
        return ESP_FAIL;
    }

    fprintf(fp, "%d,%02X,%02X,%02X,%02X,%02X,%02X",
            ESPNOW_CHANNEL,
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

    ESP_LOGI(TAG, "ピア情報読込: CH=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             ESPNOW_CHANNEL,
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
    if (TDMA_DEVICE_ID == 0 || !first_beacon_received) {
        return false;
    }
    int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
    return (time_since_beacon > BEACON_TIMEOUT_US);
}
