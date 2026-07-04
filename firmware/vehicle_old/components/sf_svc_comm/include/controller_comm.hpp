/**
 * @file controller_comm.hpp
 * @brief ESP-NOW Controller Communication
 *
 * Control packet reception, telemetry transmission, pairing
 * for_tdmaブランチ互換
 */

#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace stampfly {

/**
 * @brief Control packet from controller (14 bytes)
 * Compatible with for_tdma branch
 */
struct ControlPacket {
    uint8_t drone_mac[3];   // byte 0-2: Destination MAC lower 3 bytes
    uint16_t throttle;      // byte 3-4: 0-1000
    uint16_t roll;          // byte 5-6: 0-1000 (500=center)
    uint16_t pitch;         // byte 7-8: 0-1000 (500=center)
    uint16_t yaw;           // byte 9-10: 0-1000 (500=center)
    uint8_t flags;          // byte 11: bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode, bit4:PosMode
    uint8_t reserved;       // byte 12: Reserved (proactive_flag, ignored)
    uint8_t checksum;       // byte 13: Sum of bytes 0-12
} __attribute__((packed));

// Control packet flags
constexpr uint8_t CTRL_FLAG_ARM      = 0x01;
constexpr uint8_t CTRL_FLAG_FLIP     = 0x02;
constexpr uint8_t CTRL_FLAG_MODE     = 0x04;
constexpr uint8_t CTRL_FLAG_ALT_MODE = 0x08;
constexpr uint8_t CTRL_FLAG_POS_MODE = 0x10;  // bit 4: Position hold mode

/**
 * @brief Telemetry packet to controller (22 bytes)
 */
struct TelemetryPacket {
    uint8_t header;           // byte 0: 0xAA
    uint8_t packet_type;      // byte 1: 0x01 = telemetry
    uint8_t sequence;         // byte 2: Sequence number
    uint16_t battery_mv;      // byte 3-4: Battery voltage [mV]
    int16_t altitude_cm;      // byte 5-6: Altitude [cm]
    int16_t velocity_x;       // byte 7-8: X velocity [mm/s]
    int16_t velocity_y;       // byte 9-10: Y velocity [mm/s]
    int16_t velocity_z;       // byte 11-12: Z velocity [mm/s]
    int16_t roll_deg10;       // byte 13-14: Roll [0.1 deg]
    int16_t pitch_deg10;      // byte 15-16: Pitch [0.1 deg]
    int16_t yaw_deg10;        // byte 17-18: Yaw [0.1 deg]
    uint8_t state;            // byte 19: FlightState
    uint8_t flags;            // byte 20: Warning flags
    uint8_t checksum;         // byte 21: Sum of bytes 0-20
} __attribute__((packed));

// Telemetry flags
constexpr uint8_t TELEM_FLAG_LOW_BATTERY  = 0x01;
constexpr uint8_t TELEM_FLAG_SENSOR_ERROR = 0x02;
constexpr uint8_t TELEM_FLAG_COMM_LOST    = 0x04;
constexpr uint8_t TELEM_FLAG_CALIBRATING  = 0x08;

using ControlCallback = std::function<void(const ControlPacket&)>;

class ControllerComm {
public:
    static constexpr int DEFAULT_WIFI_CHANNEL = 1;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 500;

    struct Config {
        int wifi_channel = DEFAULT_WIFI_CHANNEL;
        uint32_t timeout_ms = DEFAULT_TIMEOUT_MS;
    };

    ControllerComm() = default;
    ~ControllerComm() = default;

    /**
     * @brief Initialize ESP-NOW communication
     * @param config WiFi configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Start communication
     * @return ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop communication
     * @return ESP_OK on success
     */
    esp_err_t stop();

    /**
     * @brief Set control packet callback
     * @param callback Function to call on control packet reception
     */
    void setControlCallback(ControlCallback callback);

    /**
     * @brief Send telemetry packet
     * @param packet Telemetry data
     * @return ESP_OK on success
     */
    esp_err_t sendTelemetry(const TelemetryPacket& packet);

    /**
     * @brief Check if connected to controller
     * @return true if connected
     */
    bool isConnected() const { return connected_; }

    /**
     * @brief Check if paired to controller
     * @return true if paired
     */
    bool isPaired() const { return paired_; }

    /**
     * @brief Enter pairing mode
     */
    void enterPairingMode();

    /**
     * @brief Exit pairing mode
     */
    void exitPairingMode();

    /**
     * @brief Check if in pairing mode
     */
    bool isPairingMode() const { return pairing_mode_; }

    /**
     * @brief Set WiFi channel
     * @param channel WiFi channel (1-13)
     * @param save_to_nvs Save to NVS (default: false)
     * @return ESP_OK on success
     */
    esp_err_t setChannel(int channel, bool save_to_nvs = false);

    /**
     * @brief Get current WiFi channel
     * @return Current channel
     */
    int getChannel() const { return config_.wifi_channel; }

    /**
     * @brief Save current channel to NVS
     * @return ESP_OK on success
     */
    esp_err_t saveChannelToNVS();

    /**
     * @brief Load channel from NVS
     * @return Channel number (1-13) or -1 if not found
     */
    static int loadChannelFromNVS();

    /**
     * @brief Save pairing information to NVS
     * @return ESP_OK on success
     */
    esp_err_t savePairingToNVS();

    /**
     * @brief Load pairing information from NVS
     * @return ESP_OK on success
     */
    esp_err_t loadPairingFromNVS();

    /**
     * @brief Clear pairing information from NVS
     * @return ESP_OK on success
     */
    esp_err_t clearPairingFromNVS();

    /**
     * @brief Validate control packet checksum
     * @param data Raw packet data
     * @param len Packet length
     * @return true if checksum valid
     */
    static bool validateChecksum(const uint8_t* data, size_t len);

    /**
     * @brief Periodic tick for timeout checking
     */
    void tick();

    /**
     * @brief Get last receive timestamp
     * @return Tick count when last packet was received
     */
    uint32_t getLastReceiveTime() const { return last_recv_time_; }

    bool isInitialized() const { return initialized_; }

    // Internal callbacks (called from static ESP-NOW callback)
    void onControlPacketReceived(const ControlPacket& packet, const uint8_t* mac);
    void onPairingComplete(const uint8_t* mac);

    // Internal callbacks (called from static WiFi/IP event handlers)
    // WiFi/IPイベントハンドラから呼び出される内部コールバック
    void onSTAConnected(void* event_data);
    void onSTADisconnected(void* event_data);
    void onSTAGotIP(void* event_data);

    /**
     * @brief Load AP SSID from NVS
     * NVSからAP SSIDを読み込み
     * @param ssid Buffer to store SSID
     * @param max_len Buffer size
     * @return ESP_OK on success
     */
    static esp_err_t loadAPSSIDFromNVS(char* ssid, size_t max_len);

    /**
     * @brief Save AP SSID to NVS
     * AP SSIDをNVSに保存
     * @param ssid SSID string
     * @return ESP_OK on success
     */
    static esp_err_t saveAPSSIDToNVS(const char* ssid);

    /**
     * @brief Get current AP SSID
     * 現在のAP SSIDを取得
     * @return AP SSID string
     */
    const char* getAPSSID() const { return ap_ssid_; }

    /**
     * @brief Send pairing packet (called periodically during pairing mode)
     */
    void sendPairingPacket();

    /**
     * @brief Check if pairing broadcast is active
     */
    bool isPairingBroadcastActive() const { return pairing_broadcast_active_; }

    // WiFi STA (Station mode) management for multi-AP support
    // WiFi STAモード管理（複数AP対応）

    /**
     * @brief STA configuration structure
     * STA設定構造体
     */
    struct STAConfig {
        char ssid[32];
        char password[64];
        bool is_valid;
    };

    /**
     * @brief Add STA configuration to the list
     * STA設定をリストに追加
     * @param ssid SSID
     * @param password Password
     * @return ESP_OK on success, ESP_ERR_NO_MEM if list is full
     */
    esp_err_t addSTAConfig(const char* ssid, const char* password);

    /**
     * @brief Remove STA configuration by index
     * インデックスでSTA設定を削除
     * @param index Index (0-4)
     * @return ESP_OK on success
     */
    esp_err_t removeSTAConfig(int index);

    /**
     * @brief Remove STA configuration by SSID
     * SSIDでSTA設定を削除
     * @param ssid SSID
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found
     */
    esp_err_t removeSTAConfig(const char* ssid);

    /**
     * @brief Get number of saved STA configurations
     * 保存されているSTA設定数を取得
     * @return Number of configurations
     */
    int getSTAConfigCount() const { return sta_config_count_; }

    /**
     * @brief Get STA configuration by index
     * インデックスでSTA設定を取得
     * @param index Index (0-4)
     * @return Pointer to configuration, or nullptr if invalid
     */
    const STAConfig* getSTAConfig(int index) const;

    /**
     * @brief Connect to WiFi AP (auto-priority)
     * WiFi APに接続（優先順位順に自動試行）
     * @return ESP_OK on success
     */
    esp_err_t connectSTA();

    /**
     * @brief Connect to WiFi AP by index
     * インデックス指定でWiFi APに接続
     * @param index Index (0-4)
     * @return ESP_OK on success
     */
    esp_err_t connectSTA(int index);

    /**
     * @brief Disconnect from WiFi AP
     * WiFi APから切断
     * @return ESP_OK on success
     */
    esp_err_t disconnectSTA();

    /**
     * @brief Check if connected to WiFi AP
     * WiFi APに接続しているか確認
     * @return true if connected
     */
    bool isSTAConnected() const { return sta_connected_; }

    /**
     * @brief Get STA IP address
     * STA IPアドレスを取得
     * @return IP address string
     */
    const char* getSTAIPAddress() const { return sta_ip_addr_; }

    /**
     * @brief Get currently connected AP index
     * 現在接続中のAP indexを取得
     * @return Index, or -1 if not connected
     */
    int getCurrentSTAIndex() const { return current_sta_index_; }

    /**
     * @brief Get currently connected AP SSID
     * 現在接続中のAP SSIDを取得
     * @return SSID, or nullptr if not connected
     */
    const char* getCurrentSTASSID() const;

    /**
     * @brief Set auto-connect on boot
     * 起動時の自動接続を設定
     * @param enable true to enable
     */
    void setSTAAutoConnect(bool enable) { sta_auto_connect_ = enable; }

    /**
     * @brief Check if auto-connect is enabled
     * 自動接続が有効か確認
     * @return true if enabled
     */
    bool isSTAAutoConnect() const { return sta_auto_connect_; }

    /**
     * @brief Save STA configurations to NVS
     * STA設定をNVSに保存
     * @return ESP_OK on success
     */
    esp_err_t saveSTAConfigsToNVS();

    /**
     * @brief Load STA configurations from NVS
     * STA設定をNVSから読み込み
     * @return ESP_OK on success
     */
    esp_err_t loadSTAConfigsFromNVS();

    /**
     * @brief Load auto-connect flag from NVS
     * 自動接続フラグをNVSから読み込み
     * @return true if auto-connect enabled (default: true)
     */
    bool loadSTAAutoConnectFromNVS();

private:
    void addControllerPeer();

    bool initialized_ = false;
    bool connected_ = false;
    bool paired_ = false;
    bool pairing_mode_ = false;
    bool pairing_broadcast_active_ = false;
    Config config_;
    ControlCallback control_callback_;
    uint8_t controller_mac_[6] = {0};
    uint8_t telemetry_sequence_ = 0;
    uint32_t last_recv_time_ = 0;
    char ap_ssid_[33] = {};

    // WiFi STA mode members
    // WiFi STAモードのメンバー変数
    static constexpr int MAX_STA_CONFIGS = 5;  // Maximum 5 AP configurations / 最大5個のAP設定

    STAConfig sta_configs_[MAX_STA_CONFIGS] = {};
    int sta_config_count_ = 0;
    int current_sta_index_ = -1;  // Currently connected AP index (-1: not connected)

    bool sta_auto_connect_ = true;  // Auto-connect on boot (default: ON)
    bool sta_connected_ = false;
    char sta_ip_addr_[16] = {0};

    // Connection attempt management
    // 接続試行管理
    int connection_attempt_index_ = 0;  // Next AP index to try
    bool is_connecting_ = false;

    // Event handlers
    // イベントハンドラ
    void* sta_netif_ = nullptr;  // esp_netif_t* (void* to avoid header dependency)
    void* wifi_event_handler_ = nullptr;  // esp_event_handler_instance_t*
    void* ip_event_handler_ = nullptr;    // esp_event_handler_instance_t*
};

}  // namespace stampfly
