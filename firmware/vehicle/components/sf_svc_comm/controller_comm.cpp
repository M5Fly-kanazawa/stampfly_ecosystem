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
#include "esp_log.h"
#include "esp_wifi.h"
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
static void espnow_send_cb(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "Send failed to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
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

    // ネットワークインターフェース作成（APモード用）
    esp_netif_create_default_wifi_ap();

    // WiFi初期化
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // AP設定（テレメトリ用）
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "StampFly");
    ap_config.ap.ssid_len = 8;
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
    ESP_LOGI(TAG, "WiFi AP 'StampFly' started on channel %d (http://192.168.4.1)", config.wifi_channel);

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

esp_err_t ControllerComm::setChannel(int channel)
{
    if (channel < 1 || channel > 13) {
        ESP_LOGE(TAG, "Invalid channel %d (must be 1-13)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Setting WiFi channel to %d", channel);

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

    return ESP_OK;
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

}  // namespace stampfly
