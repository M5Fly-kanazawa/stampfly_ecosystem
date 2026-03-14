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

    // ToF sensors (8 bytes) - 底面・前方距離
    float tof_bottom;         // [m] 底面ToF距離
    float tof_front;          // [m] 前方ToF距離

    // Status (2 bytes)
    uint8_t  flight_state;    // FlightState enum
    uint8_t  sensor_status;   // Sensor health flags

    // Heartbeat (4 bytes)
    uint32_t heartbeat;       // ESP32送信カウンタ

    uint8_t  checksum;        // XOR of all preceding bytes
    uint8_t  padding[3];      // 4バイトアライメント
};
#pragma pack(pop)

static_assert(sizeof(TelemetryWSPacket) == 116, "TelemetryWSPacket size mismatch");

/**
 * @brief Lightweight FFT packet structure (32 bytes)
 *
 * Minimal packet for high-rate FFT data capture.
 * Contains only gyro and accel data for vibration analysis.
 * 軽量FFTパケット - 振動解析用のジャイロ・加速度のみ
 */
#pragma pack(push, 1)
struct TelemetryFFTPacket {
    // Header (2 bytes)
    uint8_t  header;          // 0xBB (different from normal packet)
    uint8_t  packet_type;     // 0x30 = FFT packet
    uint32_t timestamp_ms;    // ms since boot

    // Gyro - bias corrected (12 bytes)
    float gyro_x;             // [rad/s]
    float gyro_y;             // [rad/s]
    float gyro_z;             // [rad/s]

    // Accel - bias corrected (12 bytes)
    float accel_x;            // [m/s²]
    float accel_y;            // [m/s²]
    float accel_z;            // [m/s²]

    // Footer (2 bytes)
    uint8_t  checksum;        // XOR of all preceding bytes
    uint8_t  padding;         // 4-byte alignment
};
#pragma pack(pop)

static_assert(sizeof(TelemetryFFTPacket) == 32, "TelemetryFFTPacket size mismatch");

/**
 * @brief Single FFT sample (56 bytes)
 *
 * Used within batch packets.
 * バッチパケット内の1サンプル
 *
 * Includes controller inputs and bias-corrected gyro for PID debugging.
 * PIDデバッグ用にコントローラ入力とバイアス補正済みジャイロを含む
 */
struct FFTSample {
    uint32_t timestamp_ms;    // ms since boot
    // Raw gyro (LPF filtered, no bias correction)
    // 生ジャイロ（LPFフィルタ済み、バイアス補正なし）
    float gyro_x;             // [rad/s]
    float gyro_y;             // [rad/s]
    float gyro_z;             // [rad/s]
    // Raw accel (LPF filtered, no bias correction)
    // 生加速度（LPFフィルタ済み、バイアス補正なし）
    float accel_x;            // [m/s²]
    float accel_y;            // [m/s²]
    float accel_z;            // [m/s²]
    // Bias-corrected gyro (what control loop sees)
    // バイアス補正済みジャイロ（制御ループが見る値）
    float gyro_corrected_x;   // [rad/s]
    float gyro_corrected_y;   // [rad/s]
    float gyro_corrected_z;   // [rad/s]
    // Controller inputs (normalized)
    // コントローラ入力（正規化済み）
    float ctrl_throttle;      // [0-1]
    float ctrl_roll;          // [-1 to 1]
    float ctrl_pitch;         // [-1 to 1]
    float ctrl_yaw;           // [-1 to 1]
};

static_assert(sizeof(FFTSample) == 56, "FFTSample size mismatch");

/**
 * @brief Batch FFT packet structure (232 bytes)
 *
 * Contains 4 samples in one WebSocket frame to overcome
 * per-frame overhead limitation (~155 frames/sec max).
 *
 * 4サンプルを1フレームにまとめて送信。
 * フレームあたりのオーバーヘッド制限(~155fps)を回避。
 *
 * Frame rate: 100Hz → Sample rate: 400Hz
 */
#pragma pack(push, 1)
struct TelemetryFFTBatchPacket {
    // Header (4 bytes)
    uint8_t  header;          // 0xBC (batch FFT packet)
    uint8_t  packet_type;     // 0x31
    uint8_t  sample_count;    // Number of samples (4)
    uint8_t  reserved;

    // Samples (56 bytes × 4 = 224 bytes)
    FFTSample samples[4];

    // Footer (4 bytes)
    uint8_t  checksum;        // XOR of all preceding bytes
    uint8_t  padding[3];      // 4-byte alignment
};
#pragma pack(pop)

static_assert(sizeof(TelemetryFFTBatchPacket) == 232, "TelemetryFFTBatchPacket size mismatch");

// Batch size constant
inline constexpr int FFT_BATCH_SIZE = 4;

/**
 * @brief Extended sample with ESKF estimates and sensor data (136 bytes)
 *
 * 400Hz unified telemetry format.
 * 400Hz統一テレメトリフォーマット
 *
 * Contains:
 * - Raw sensor data (for system identification)
 * - ESKF estimates (attitude, position, velocity, biases)
 * - Additional sensors (baro, ToF, optical flow)
 */
struct ExtendedSample {
    // --- Core sensor data (56 bytes, same layout as FFTSample) ---
    uint32_t timestamp_us;    // μs since boot (changed from ms for precision)

    // Raw gyro (LPF filtered, no bias correction)
    // 生ジャイロ（LPFフィルタ済み、バイアス補正なし）
    float gyro_x;             // [rad/s]
    float gyro_y;             // [rad/s]
    float gyro_z;             // [rad/s]

    // Raw accel (LPF filtered, no bias correction)
    // 生加速度（LPFフィルタ済み、バイアス補正なし）
    float accel_x;            // [m/s²]
    float accel_y;            // [m/s²]
    float accel_z;            // [m/s²]

    // Bias-corrected gyro (what control loop sees)
    // バイアス補正済みジャイロ（制御ループが見る値）
    float gyro_corrected_x;   // [rad/s]
    float gyro_corrected_y;   // [rad/s]
    float gyro_corrected_z;   // [rad/s]

    // Controller inputs (normalized)
    // コントローラ入力（正規化済み）
    float ctrl_throttle;      // [0-1]
    float ctrl_roll;          // [-1 to 1]
    float ctrl_pitch;         // [-1 to 1]
    float ctrl_yaw;           // [-1 to 1]

    // --- ESKF estimates (60 bytes) ---
    // Attitude quaternion [w, x, y, z]
    // 姿勢クォータニオン
    float quat_w;             // quaternion scalar
    float quat_x;             // quaternion vector x
    float quat_y;             // quaternion vector y
    float quat_z;             // quaternion vector z

    // Position NED [m]
    // 位置 NED座標系
    float pos_x;              // North [m]
    float pos_y;              // East [m]
    float pos_z;              // Down [m]

    // Velocity NED [m/s]
    // 速度 NED座標系
    float vel_x;              // [m/s]
    float vel_y;              // [m/s]
    float vel_z;              // [m/s]

    // Gyro bias estimate (scaled: value * 10000 rad/s)
    // ジャイロバイアス推定値（スケール: 値×10000 rad/s）
    int16_t gyro_bias_x;      // [0.0001 rad/s]
    int16_t gyro_bias_y;      // [0.0001 rad/s]
    int16_t gyro_bias_z;      // [0.0001 rad/s]

    // Accel bias estimate (scaled: value * 10000 m/s²)
    // 加速度バイアス推定値（スケール: 値×10000 m/s²）
    int16_t accel_bias_x;     // [0.0001 m/s²]
    int16_t accel_bias_y;     // [0.0001 m/s²]
    int16_t accel_bias_z;     // [0.0001 m/s²]

    // ESKF status flags
    // ESKF状態フラグ
    uint8_t eskf_status;      // bit0: converged, bit1-7: reserved

    uint8_t padding1[7];      // alignment to make ESKF section 60 bytes

    // --- Additional sensor data (20 bytes) ---
    // Barometer altitude [m]
    // 気圧高度
    float baro_altitude;      // [m] MSL altitude

    // ToF distance [m]
    // ToF距離
    float tof_bottom;         // [m] bottom sensor
    float tof_front;          // [m] front sensor

    // Optical flow (raw sensor output)
    // 光学フロー（センサ生出力）
    int16_t flow_x;           // [pixels/frame or sensor units]
    int16_t flow_y;           // [pixels/frame or sensor units]
    uint8_t flow_quality;     // [0-255] quality/confidence

    uint8_t padding2[3];      // alignment to 136 bytes total
};

static_assert(sizeof(ExtendedSample) == 136, "ExtendedSample size mismatch");

/**
 * @brief Extended batch packet (552 bytes)
 *
 * 400Hz unified telemetry: 4 samples per frame at 100fps.
 * 400Hz統一テレメトリ: 100fps × 4サンプル/フレーム
 *
 * Bandwidth: 552 bytes × 100 fps = 55.2 KB/s ≈ 442 kbps
 */
#pragma pack(push, 1)
struct TelemetryExtendedBatchPacket {
    // Header (4 bytes)
    uint8_t  header;          // 0xBD (extended batch packet)
    uint8_t  packet_type;     // 0x32
    uint8_t  sample_count;    // Number of samples (4)
    uint8_t  reserved;

    // Samples (136 bytes × 4 = 544 bytes)
    ExtendedSample samples[4];

    // Footer (4 bytes)
    uint8_t  checksum;        // XOR of all preceding bytes
    uint8_t  padding[3];      // 4-byte alignment
};
#pragma pack(pop)

static_assert(sizeof(TelemetryExtendedBatchPacket) == 552, "TelemetryExtendedBatchPacket size mismatch");

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
