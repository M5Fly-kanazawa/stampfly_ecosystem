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
    uint8_t flags;          // byte 11: bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
    uint8_t reserved;       // byte 12: Reserved (proactive_flag, ignored)
    uint8_t checksum;       // byte 13: Sum of bytes 0-12
} __attribute__((packed));

// Control packet flags
constexpr uint8_t CTRL_FLAG_ARM      = 0x01;
constexpr uint8_t CTRL_FLAG_FLIP     = 0x02;
constexpr uint8_t CTRL_FLAG_MODE     = 0x04;
constexpr uint8_t CTRL_FLAG_ALT_MODE = 0x08;

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
     * @return ESP_OK on success
     */
    esp_err_t setChannel(int channel);

    /**
     * @brief Get current WiFi channel
     * @return Current channel
     */
    int getChannel() const { return config_.wifi_channel; }

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

    /**
     * @brief Send pairing packet (called periodically during pairing mode)
     */
    void sendPairingPacket();

    /**
     * @brief Check if pairing broadcast is active
     */
    bool isPairingBroadcastActive() const { return pairing_broadcast_active_; }

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
};

}  // namespace stampfly
