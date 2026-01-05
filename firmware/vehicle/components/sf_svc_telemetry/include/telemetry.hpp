/**
 * @file telemetry.hpp
 * @brief WiFi WebSocket Telemetry for StampFly
 *
 * Provides real-time telemetry streaming over WiFi WebSocket.
 * Uses the WiFi AP mode configured in stampfly_comm.
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

/**
 * @brief Telemetry packet structure (binary format)
 *
 * Version 2: Extended packet with IMU data and control inputs
 */
#pragma pack(push, 1)
struct TelemetryWSPacket {
    // Header (2 bytes)
    uint8_t  header;          // 0xAA
    uint8_t  packet_type;     // 0x20 = extended packet (v2)
    uint32_t timestamp_ms;    // ms since boot

    // Attitude - ESKF estimated (12 bytes)
    float roll;               // [rad]
    float pitch;              // [rad]
    float yaw;                // [rad]

    // Position - ESKF estimated (12 bytes)
    float pos_x;              // [m] NED
    float pos_y;              // [m]
    float pos_z;              // [m]

    // Velocity - ESKF estimated (12 bytes)
    float vel_x;              // [m/s]
    float vel_y;              // [m/s]
    float vel_z;              // [m/s]

    // Gyro - bias corrected (12 bytes) [NEW]
    float gyro_x;             // [rad/s]
    float gyro_y;             // [rad/s]
    float gyro_z;             // [rad/s]

    // Accel - bias corrected (12 bytes) [NEW]
    float accel_x;            // [m/s²]
    float accel_y;            // [m/s²]
    float accel_z;            // [m/s²]

    // Control inputs - normalized (16 bytes)
    float ctrl_throttle;      // [0-1]
    float ctrl_roll;          // [-1 to 1]
    float ctrl_pitch;         // [-1 to 1]
    float ctrl_yaw;           // [-1 to 1]

    // Magnetometer - raw sensor (12 bytes) [NEW v2.1]
    float mag_x;              // [uT]
    float mag_y;              // [uT]
    float mag_z;              // [uT]

    // Battery (4 bytes)
    float voltage;            // [V]

    // Status (2 bytes)
    uint8_t  flight_state;    // FlightState enum
    uint8_t  sensor_status;   // Sensor health flags

    // Heartbeat (4 bytes)
    uint32_t heartbeat;       // ESP32送信カウンタ

    uint8_t  checksum;        // XOR of all preceding bytes
    uint8_t  padding[3];      // 4バイトアライメント
};
#pragma pack(pop)

static_assert(sizeof(TelemetryWSPacket) == 108, "TelemetryWSPacket size mismatch");

/**
 * @brief Sensor status flags (bitfield)
 */
enum SensorStatusFlags : uint8_t {
    SENSOR_IMU_OK    = (1 << 0),
    SENSOR_MAG_OK    = (1 << 1),
    SENSOR_BARO_OK   = (1 << 2),
    SENSOR_TOF_OK    = (1 << 3),
    SENSOR_FLOW_OK   = (1 << 4),
};

/**
 * @brief WiFi WebSocket Telemetry Server
 *
 * Singleton class that manages WebSocket connections and broadcasts
 * telemetry data to all connected clients.
 */
class Telemetry {
public:
    static Telemetry& getInstance();

    // Delete copy/move
    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    struct Config {
        uint16_t port;
        uint32_t rate_hz;

        Config() : port(80), rate_hz(50) {}
    };

    /**
     * @brief Initialize telemetry server
     *
     * Starts HTTP server and registers WebSocket handler.
     * WiFi AP must be already initialized by stampfly_comm.
     *
     * @param config Server configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config());

    /**
     * @brief Stop telemetry server
     */
    esp_err_t stop();

    /**
     * @brief Get number of connected clients
     */
    int clientCount() const { return client_count_; }

    /**
     * @brief Check if any clients are connected
     */
    bool hasClients() const { return client_count_ > 0; }

    /**
     * @brief Broadcast binary data to all connected clients
     *
     * @param data Binary data
     * @param len Data length
     * @return Number of clients that received the data
     */
    int broadcast(const void* data, size_t len);

    /**
     * @brief Check if server is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get telemetry rate in Hz
     */
    uint32_t getRateHz() const { return config_.rate_hz; }

private:
    Telemetry() = default;

    // HTTP/WebSocket handlers
    static esp_err_t ws_handler(httpd_req_t* req);
    static esp_err_t http_get_handler(httpd_req_t* req);
    static esp_err_t http_get_threejs_handler(httpd_req_t* req);

    // Client management
    void addClient(int fd);
    void removeClient(int fd);

    bool initialized_ = false;
    httpd_handle_t server_ = nullptr;
    Config config_;
    int client_count_ = 0;

    // Connected client FDs (max 4 clients)
    static constexpr int MAX_CLIENTS = 4;
    int client_fds_[MAX_CLIENTS] = {-1, -1, -1, -1};
    SemaphoreHandle_t client_mutex_ = nullptr;
};

}  // namespace stampfly
