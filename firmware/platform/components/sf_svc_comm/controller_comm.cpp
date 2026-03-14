/**
 * @file controller_comm.cpp
 * @brief ESP-NOW Controller Communication Implementation
 *
 * for_tdmaブランチ互換のESP-NOW通信実装
 * - 制御パケット受信（14バイト）
 * - テレメトリ送信（22バイト）
 * - ペアリング機能（NVS保存/復元）
 */

#include "controller_comm.hpp"
#include "led_manager.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>

static const char* TAG = "ControllerComm";

namespace stampfly {

// NVS keys
static constexpr const char* NVS_NAMESPACE = "stampfly";
static constexpr const char* NVS_KEY_PAIRING = "ctrl_mac";
static constexpr const char* NVS_KEY_CHANNEL = "wifi_ch";

// WiFi STA NVS keys (multi-AP support)
static constexpr const char* NVS_KEY_STA_COUNT = "sta_count";      // u8
static constexpr const char* NVS_KEY_STA_AUTO  = "sta_auto";       // u8
static constexpr const char* NVS_KEY_STA_SSID_FMT = "sta_%d_ssid"; // string (index 0-4)
static constexpr const char* NVS_KEY_STA_PASS_FMT = "sta_%d_pass"; // string (index 0-4)
static constexpr const char* NVS_KEY_AP_SSID = "ap_ssid";

// Global instance pointer for callback access
static ControllerComm* s_instance = nullptr;

// ESP-NOW receive callback
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len)
{
    if (s_instance == nullptr) return;

    const uint8_t* mac = recv_info->src_addr;

    // ペアリングモード時の処理
    if (s_instance->isPairingMode()) {
        if (len == sizeof(ControlPacket)) {
            // 制御パケットを受信 -> ペアリング完了
            ESP_LOGI(TAG, "Pairing: Received control packet from %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            // MACアドレスを保存
            s_instance->onPairingComplete(mac);
        }
        return;
    }

    // 通常モード: 制御パケットの処理
    if (len == sizeof(ControlPacket)) {
        // チェックサム検証
        if (!ControllerComm::validateChecksum(data, len)) {
            ESP_LOGW(TAG, "Invalid checksum");
            return;
        }

        // パケット解析
        ControlPacket packet;
        memcpy(&packet, data, sizeof(ControlPacket));

        // コールバック呼び出し
        s_instance->onControlPacketReceived(packet, mac);
    }
}

// ESP-NOW send callback
// Note: ESP-IDF v5.5+ changed the callback signature to use wifi_tx_info_t
static void espnow_send_cb(const esp_now_send_info_t* send_info, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS && send_info != nullptr && send_info->des_addr != nullptr) {
        const uint8_t* mac_addr = send_info->des_addr;
        ESP_LOGD(TAG, "Send failed to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

// WiFi event handler for STA mode
// WiFi STAモード用イベントハンドラ
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    ControllerComm* self = static_cast<ControllerComm*>(arg);

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA started");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            self->onSTAConnected(event_data);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            self->onSTADisconnected(event_data);
            break;
    }
}

// IP event handler for STA mode
// WiFi STAモード用IPイベントハンドラ
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    ControllerComm* self = static_cast<ControllerComm*>(arg);

    if (event_id == IP_EVENT_STA_GOT_IP) {
        self->onSTAGotIP(event_data);
    }
}

esp_err_t ControllerComm::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ESP-NOW communication");
    ESP_LOGI(TAG, "  WiFi channel: %d", config.wifi_channel);

    config_ = config;
    s_instance = this;

    // ネットワークインターフェース作成（STAモード用）
    // Create network interface for STA mode
    sta_netif_ = esp_netif_create_default_wifi_sta();

    // ネットワークインターフェース作成（APモード用）
    esp_netif_create_default_wifi_ap();

    // WiFi初期化
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // WiFi/IPイベントハンドラ登録
    // Register WiFi/IP event handlers
    esp_event_handler_instance_t wifi_handler;
    esp_event_handler_instance_t ip_handler;

    ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, this, &wifi_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    wifi_event_handler_ = wifi_handler;

    ret = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &ip_event_handler, this, &ip_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    ip_event_handler_ = ip_handler;

    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // AP SSID生成（MACベースのユニークSSID）
    // Generate unique AP SSID based on MAC address
    char ap_ssid[33] = {};
    if (loadAPSSIDFromNVS(ap_ssid, sizeof(ap_ssid)) != ESP_OK) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(ap_ssid, sizeof(ap_ssid), "StampFly_%02X%02X", mac[4], mac[5]);
        saveAPSSIDToNVS(ap_ssid);
    }
    strncpy(ap_ssid_, ap_ssid, sizeof(ap_ssid_));

    // AP設定（テレメトリ用）
    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(ap_ssid);
    ap_config.ap.channel = config.wifi_channel;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
    }

    // 電力節約無効化（レイテンシ改善）
    esp_wifi_set_ps(WIFI_PS_NONE);

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // チャンネル設定
    ret = esp_wifi_set_channel(config.wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // STA設定リスト読み込み（NVSから）
    // Load STA configuration list from NVS
    loadSTAConfigsFromNVS();

    // 自動接続フラグ読み込み（デフォルトON）
    // Load auto-connect flag (default: ON)
    sta_auto_connect_ = loadSTAAutoConnectFromNVS();

    // 自動接続有効かつSTA設定済みなら接続試行
    // Auto-connect if enabled and STA configured
    if (sta_auto_connect_ && sta_config_count_ > 0) {
        ESP_LOGI(TAG, "Auto-connecting to STA (%d APs configured)...", sta_config_count_);
        connectSTA();  // 優先順位順に自動接続 / Auto-connect in priority order
    }

    // ESP-NOW初期化
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // コールバック登録
    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register recv callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_now_register_send_cb(espnow_send_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
        return ret;
    }

    // ペアリング情報をNVSから読み込み
    ret = loadPairingFromNVS();
    if (ret == ESP_OK && isPaired()) {
        ESP_LOGI(TAG, "Loaded pairing: %02X:%02X:%02X:%02X:%02X:%02X",
                 controller_mac_[0], controller_mac_[1], controller_mac_[2],
                 controller_mac_[3], controller_mac_[4], controller_mac_[5]);

        // ピア追加
        addControllerPeer();
    }

    initialized_ = true;
    ESP_LOGI(TAG, "ESP-NOW communication initialized");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SSID: %s", ap_ssid_);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "WiFi AP '%s' started on channel %d (http://192.168.4.1)",
             ap_ssid_, config.wifi_channel);

    return ESP_OK;
}

esp_err_t ControllerComm::start()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "ESP-NOW communication started");
    last_recv_time_ = 0;

    return ESP_OK;
}

esp_err_t ControllerComm::stop()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    connected_ = false;
    ESP_LOGI(TAG, "ESP-NOW communication stopped");

    return ESP_OK;
}

void ControllerComm::setControlCallback(ControlCallback callback)
{
    control_callback_ = callback;
}

esp_err_t ControllerComm::sendTelemetry(const TelemetryPacket& packet)
{
    if (!initialized_ || !isPaired()) {
        return ESP_ERR_INVALID_STATE;
    }

    // チェックサム計算
    TelemetryPacket pkt = packet;
    pkt.header = 0xAA;
    pkt.packet_type = 0x01;
    pkt.sequence = telemetry_sequence_++;

    uint8_t* data = reinterpret_cast<uint8_t*>(&pkt);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        sum += data[i];
    }
    pkt.checksum = sum;

    // 送信
    esp_err_t ret = esp_now_send(controller_mac_, data, sizeof(TelemetryPacket));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Telemetry send failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t ControllerComm::setChannel(int channel, bool save_to_nvs)
{
    if (channel < 1 || channel > 13) {
        ESP_LOGE(TAG, "Invalid channel %d (must be 1-13)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // STA接続中の警告
    // Warn if STA is connected - channel change may affect AP
    if (sta_connected_) {
        ESP_LOGW(TAG, "STA is connected - channel change may disconnect AP");
        ESP_LOGW(TAG, "STA will override this channel when reconnected");
    }

    ESP_LOGI(TAG, "Setting WiFi channel to %d", channel);

    // Update SoftAP config so the radio actually switches channel in APSTA mode
    // APSTAモードでは SoftAP の config チャンネルが無線チャンネルを決定するため更新が必要
    wifi_config_t ap_config = {};
    if (esp_wifi_get_config(WIFI_IF_AP, &ap_config) == ESP_OK) {
        ap_config.ap.channel = channel;
        esp_err_t ap_ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        if (ap_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to update AP channel config: %s", esp_err_to_name(ap_ret));
        }
    }

    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi channel: %s", esp_err_to_name(ret));
        return ret;
    }

    config_.wifi_channel = channel;

    // Update peer channel if paired
    if (paired_) {
        esp_now_peer_info_t peer = {};
        if (esp_now_get_peer(controller_mac_, &peer) == ESP_OK) {
            peer.channel = channel;
            esp_now_mod_peer(&peer);
        }
    }

    // Save to NVS if requested
    if (save_to_nvs) {
        ret = saveChannelToNVS();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save channel to NVS");
        }
    }

    return ESP_OK;
}

esp_err_t ControllerComm::saveChannelToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_CHANNEL, static_cast<uint8_t>(config_.wifi_channel));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Channel %d saved to NVS", config_.wifi_channel);
    }

    return ret;
}

// static
int ControllerComm::loadChannelFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return -1;  // NVS not initialized or namespace not found
    }

    uint8_t channel = 0;
    ret = nvs_get_u8(handle, NVS_KEY_CHANNEL, &channel);
    nvs_close(handle);

    if (ret == ESP_OK && channel >= 1 && channel <= 13) {
        ESP_LOGI(TAG, "Loaded channel %d from NVS", channel);
        return static_cast<int>(channel);
    }

    return -1;  // Key not found or invalid value
}

// static
esp_err_t ControllerComm::loadAPSSIDFromNVS(char* ssid, size_t max_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t len = max_len;
    ret = nvs_get_str(handle, NVS_KEY_AP_SSID, ssid, &len);
    nvs_close(handle);

    if (ret == ESP_OK && len > 0) {
        ESP_LOGI(TAG, "Loaded AP SSID '%s' from NVS", ssid);
    }

    return ret;
}

// static
esp_err_t ControllerComm::saveAPSSIDToNVS(const char* ssid)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(handle, NVS_KEY_AP_SSID, ssid);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AP SSID '%s' saved to NVS", ssid);
    }

    return ret;
}

void ControllerComm::enterPairingMode()
{
    ESP_LOGI(TAG, "Entering pairing mode on channel %d", config_.wifi_channel);
    pairing_mode_ = true;
    connected_ = false;
    pairing_broadcast_active_ = true;

    // ブロードキャストピア追加
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = config_.wifi_channel;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    // 既存のピアがあれば削除してから追加
    esp_now_del_peer(broadcast_mac);
    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
    }

    // Send first pairing packet immediately
    sendPairingPacket();
}

void ControllerComm::sendPairingPacket()
{
    // Pairing packet format for Simple_StampFly_Joy controller:
    // Byte 0: Channel
    // Byte 1-6: Drone MAC address
    // Byte 7-10: Signature 0xAA 0x55 0x16 0x88
    uint8_t pairing_packet[11];

    // Get own MAC address
    uint8_t my_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);

    // Build pairing packet
    pairing_packet[0] = static_cast<uint8_t>(config_.wifi_channel);
    memcpy(&pairing_packet[1], my_mac, 6);
    pairing_packet[7] = 0xAA;
    pairing_packet[8] = 0x55;
    pairing_packet[9] = 0x16;
    pairing_packet[10] = 0x88;

    ESP_LOGI(TAG, "Broadcasting pairing packet: MAC=%02X:%02X:%02X:%02X:%02X:%02X CH=%d",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5],
             config_.wifi_channel);

    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t ret = esp_now_send(broadcast_mac, pairing_packet, sizeof(pairing_packet));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Pairing broadcast failed: %s", esp_err_to_name(ret));
    }
}

void ControllerComm::exitPairingMode()
{
    ESP_LOGI(TAG, "Exiting pairing mode");
    pairing_mode_ = false;
    pairing_broadcast_active_ = false;

    // ブロードキャストピア削除
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_del_peer(broadcast_mac);
}

esp_err_t ControllerComm::savePairingToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_PAIRING, controller_mac_, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save pairing: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Pairing saved to NVS");
    return ret;
}

esp_err_t ControllerComm::loadPairingFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No NVS pairing data");
        return ret;
    }

    size_t len = 6;
    ret = nvs_get_blob(handle, NVS_KEY_PAIRING, controller_mac_, &len);
    nvs_close(handle);

    if (ret == ESP_OK && len == 6) {
        // 有効なMACアドレスかチェック（全ゼロでない）
        bool valid = false;
        for (int i = 0; i < 6; i++) {
            if (controller_mac_[i] != 0) {
                valid = true;
                break;
            }
        }
        if (valid) {
            paired_ = true;
            return ESP_OK;
        }
    }

    memset(controller_mac_, 0, 6);
    paired_ = false;
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ControllerComm::clearPairingFromNVS()
{
    ESP_LOGI(TAG, "Clearing pairing information from NVS");

    // ピア削除
    if (paired_) {
        esp_now_del_peer(controller_mac_);
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        nvs_erase_key(handle, NVS_KEY_PAIRING);
        nvs_commit(handle);
        nvs_close(handle);
    }

    memset(controller_mac_, 0, 6);
    paired_ = false;
    connected_ = false;

    return ESP_OK;
}

bool ControllerComm::validateChecksum(const uint8_t* data, size_t len)
{
    if (len < 2) return false;
    uint8_t sum = 0;
    for (size_t i = 0; i < len - 1; i++) {
        sum += data[i];
    }
    return sum == data[len - 1];
}

void ControllerComm::tick()
{
    if (!initialized_) return;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // ペアリングモード中は定期的にパケットを送信（500ms間隔）
    if (pairing_mode_ && pairing_broadcast_active_) {
        static uint32_t last_pairing_broadcast = 0;
        if (now - last_pairing_broadcast > 500) {
            sendPairingPacket();
            last_pairing_broadcast = now;
        }
    }

    // 接続状態の更新
    if (connected_ && last_recv_time_ > 0) {
        if (now - last_recv_time_ > config_.timeout_ms) {
            connected_ = false;
            ESP_LOGW(TAG, "Connection lost");
        }
    }
}

void ControllerComm::onControlPacketReceived(const ControlPacket& packet, const uint8_t* mac)
{
    // ペアリング済みの場合、MACアドレス確認
    if (paired_) {
        if (memcmp(mac, controller_mac_, 6) != 0) {
            // 別のコントローラからのパケットは無視
            return;
        }
    }

    last_recv_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (!connected_) {
        connected_ = true;
        ESP_LOGI(TAG, "Connected to controller");
    }

    // コールバック呼び出し
    if (control_callback_) {
        control_callback_(packet);
    }
}

void ControllerComm::onPairingComplete(const uint8_t* mac)
{
    // MACアドレス保存
    memcpy(controller_mac_, mac, 6);
    paired_ = true;
    pairing_mode_ = false;

    // NVSに保存
    savePairingToNVS();

    // ブロードキャストピア削除
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_del_peer(broadcast_mac);

    // コントローラをピアとして追加
    addControllerPeer();

    connected_ = true;
    last_recv_time_ = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // ペアリングLED表示を解除
    // Release pairing LED indicator
    LEDManager::getInstance().releaseChannel(
        LEDChannel::SYSTEM, LEDPriority::PAIRING);

    ESP_LOGI(TAG, "Pairing complete: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void ControllerComm::addControllerPeer()
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, controller_mac_, 6);
    peer.channel = config_.wifi_channel;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    // 既存のピアがあれば削除
    esp_now_del_peer(controller_mac_);

    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add controller peer: %s", esp_err_to_name(ret));
    }
}

// ============================================================================
// WiFi STA Mode Implementation (Multi-AP Support)
// WiFi STAモード実装（複数AP対応）
// ============================================================================

void ControllerComm::onSTAConnected(void* event_data)
{
    wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*)event_data;
    ESP_LOGI(TAG, "STA connected to SSID:%s, Channel:%d",
             event->ssid, event->channel);

    // チャンネル変更を検出・ログ
    // Detect and log channel change
    if (event->channel != config_.wifi_channel) {
        ESP_LOGW(TAG, "Channel changed: %d -> %d (AP follows STA)",
                 config_.wifi_channel, event->channel);
        config_.wifi_channel = event->channel;
    }
}

void ControllerComm::onSTADisconnected(void* event_data)
{
    wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
    const char* ssid = (current_sta_index_ >= 0 && current_sta_index_ < sta_config_count_) ?
                       sta_configs_[current_sta_index_].ssid : "unknown";

    ESP_LOGW(TAG, "STA disconnected from '%s' (reason:%d)", ssid, event->reason);
    sta_connected_ = false;
    is_connecting_ = false;

    // 自動再接続: 次のAPを試行
    // Auto-reconnect: try next AP
    if (sta_auto_connect_ && sta_config_count_ > 0) {
        connection_attempt_index_++;
        if (connection_attempt_index_ >= sta_config_count_) {
            connection_attempt_index_ = 0;  // 最初に戻る / Loop back
            ESP_LOGI(TAG, "All APs tried, retrying from first AP...");
            vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒待機 / Wait 5 sec
        }
        ESP_LOGI(TAG, "Trying next AP (index %d)...", connection_attempt_index_);
        connectSTA(connection_attempt_index_);
    }
}

void ControllerComm::onSTAGotIP(void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    const char* ssid = (current_sta_index_ >= 0 && current_sta_index_ < sta_config_count_) ?
                       sta_configs_[current_sta_index_].ssid : "unknown";

    ESP_LOGI(TAG, "STA got IP: " IPSTR " (connected to '%s')",
             IP2STR(&event->ip_info.ip), ssid);

    snprintf(sta_ip_addr_, sizeof(sta_ip_addr_),
             IPSTR, IP2STR(&event->ip_info.ip));
    sta_connected_ = true;
    is_connecting_ = false;

    // 接続成功したら、次回の試行は最初のAPから
    // On success, reset attempt index to start from first AP
    connection_attempt_index_ = 0;
}

esp_err_t ControllerComm::addSTAConfig(const char* ssid, const char* password)
{
    if (ssid == nullptr || password == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sta_config_count_ >= MAX_STA_CONFIGS) {
        ESP_LOGE(TAG, "STA config list full (max %d)", MAX_STA_CONFIGS);
        return ESP_ERR_NO_MEM;
    }

    // 重複チェック / Check for duplicates
    for (int i = 0; i < sta_config_count_; i++) {
        if (strcmp(sta_configs_[i].ssid, ssid) == 0) {
            ESP_LOGW(TAG, "SSID '%s' already exists at index %d", ssid, i);
            return ESP_ERR_INVALID_STATE;
        }
    }

    // 新しいAPを追加 / Add new AP
    STAConfig& cfg = sta_configs_[sta_config_count_];
    strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, password, sizeof(cfg.password) - 1);
    cfg.is_valid = true;

    sta_config_count_++;
    ESP_LOGI(TAG, "Added STA config #%d: SSID=%s", sta_config_count_, ssid);
    return ESP_OK;
}

esp_err_t ControllerComm::removeSTAConfig(int index)
{
    if (index < 0 || index >= sta_config_count_) {
        return ESP_ERR_INVALID_ARG;
    }

    // 配列を詰める / Shift array
    for (int i = index; i < sta_config_count_ - 1; i++) {
        sta_configs_[i] = sta_configs_[i + 1];
    }
    sta_config_count_--;

    // 現在接続中のindexを調整 / Adjust current index
    if (current_sta_index_ == index) {
        current_sta_index_ = -1;
        sta_connected_ = false;
    } else if (current_sta_index_ > index) {
        current_sta_index_--;
    }

    ESP_LOGI(TAG, "Removed STA config #%d", index);
    return ESP_OK;
}

esp_err_t ControllerComm::removeSTAConfig(const char* ssid)
{
    for (int i = 0; i < sta_config_count_; i++) {
        if (strcmp(sta_configs_[i].ssid, ssid) == 0) {
            return removeSTAConfig(i);
        }
    }
    ESP_LOGW(TAG, "SSID '%s' not found", ssid);
    return ESP_ERR_NOT_FOUND;
}

const ControllerComm::STAConfig* ControllerComm::getSTAConfig(int index) const
{
    if (index < 0 || index >= sta_config_count_) {
        return nullptr;
    }
    return &sta_configs_[index];
}

const char* ControllerComm::getCurrentSTASSID() const
{
    if (current_sta_index_ >= 0 && current_sta_index_ < sta_config_count_) {
        return sta_configs_[current_sta_index_].ssid;
    }
    return nullptr;
}

esp_err_t ControllerComm::connectSTA()
{
    // 優先順位順（index 0から）に接続試行
    // Try to connect in priority order (from index 0)
    if (sta_config_count_ == 0) {
        ESP_LOGE(TAG, "No STA configs available");
        return ESP_ERR_INVALID_STATE;
    }

    connection_attempt_index_ = 0;
    return connectSTA(0);
}

esp_err_t ControllerComm::connectSTA(int index)
{
    if (index < 0 || index >= sta_config_count_) {
        ESP_LOGE(TAG, "Invalid STA config index: %d", index);
        return ESP_ERR_INVALID_ARG;
    }

    if (is_connecting_) {
        ESP_LOGW(TAG, "Already connecting, skipping...");
        return ESP_ERR_INVALID_STATE;
    }

    const STAConfig& cfg = sta_configs_[index];
    if (!cfg.is_valid) {
        ESP_LOGE(TAG, "STA config #%d is invalid", index);
        return ESP_ERR_INVALID_STATE;
    }

    wifi_config_t sta_config = {};
    strncpy((char*)sta_config.sta.ssid, cfg.ssid, 32);
    strncpy((char*)sta_config.sta.password, cfg.password, 64);
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Connecting to SSID '%s' (priority %d/%d)...",
             cfg.ssid, index + 1, sta_config_count_);

    current_sta_index_ = index;
    connection_attempt_index_ = index;
    is_connecting_ = true;

    return esp_wifi_connect();
}

esp_err_t ControllerComm::disconnectSTA()
{
    sta_connected_ = false;
    current_sta_index_ = -1;
    is_connecting_ = false;
    return esp_wifi_disconnect();
}

esp_err_t ControllerComm::saveSTAConfigsToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    // AP設定数を保存 / Save AP count
    nvs_set_u8(handle, NVS_KEY_STA_COUNT, sta_config_count_);

    // 自動接続フラグを保存 / Save auto-connect flag
    nvs_set_u8(handle, NVS_KEY_STA_AUTO, sta_auto_connect_ ? 1 : 0);

    // 各AP設定を保存 / Save each AP config
    char key[32];
    for (int i = 0; i < sta_config_count_; i++) {
        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        nvs_set_str(handle, key, sta_configs_[i].ssid);

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        nvs_set_str(handle, key, sta_configs_[i].password);
    }

    // 削除されたAP設定のキーをクリア / Clear deleted AP config keys
    for (int i = sta_config_count_; i < MAX_STA_CONFIGS; i++) {
        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        nvs_erase_key(handle, key);

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        nvs_erase_key(handle, key);
    }

    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Saved %d STA configs to NVS", sta_config_count_);
    return ESP_OK;
}

esp_err_t ControllerComm::loadSTAConfigsFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No STA configs in NVS (first boot)");
        return ESP_ERR_NOT_FOUND;
    }

    // AP設定数を読み込み / Load AP count
    uint8_t count = 0;
    ret = nvs_get_u8(handle, NVS_KEY_STA_COUNT, &count);
    if (ret != ESP_OK || count > MAX_STA_CONFIGS) {
        nvs_close(handle);
        return ESP_ERR_INVALID_STATE;
    }

    sta_config_count_ = count;

    // 各AP設定を読み込み / Load each AP config
    char key[32];
    for (int i = 0; i < sta_config_count_; i++) {
        size_t len;

        snprintf(key, sizeof(key), NVS_KEY_STA_SSID_FMT, i);
        len = sizeof(sta_configs_[i].ssid);
        ret = nvs_get_str(handle, key, sta_configs_[i].ssid, &len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load SSID #%d", i);
            sta_configs_[i].is_valid = false;
            continue;
        }

        snprintf(key, sizeof(key), NVS_KEY_STA_PASS_FMT, i);
        len = sizeof(sta_configs_[i].password);
        ret = nvs_get_str(handle, key, sta_configs_[i].password, &len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load password #%d", i);
            sta_configs_[i].is_valid = false;
            continue;
        }

        sta_configs_[i].is_valid = true;
        ESP_LOGI(TAG, "Loaded STA config #%d: SSID=%s", i, sta_configs_[i].ssid);
    }

    nvs_close(handle);
    return ESP_OK;
}

bool ControllerComm::loadSTAAutoConnectFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return true;  // デフォルトON / Default: ON
    }

    uint8_t auto_conn = 1;  // デフォルトON / Default: ON
    nvs_get_u8(handle, NVS_KEY_STA_AUTO, &auto_conn);
    nvs_close(handle);

    return auto_conn == 1;
}

}  // namespace stampfly
