/**
 * @file cli.hpp
 * @brief CLI Console (USB CDC)
 *
 * Command processing, sensor display, calibration
 *
 * Note: Uses static arrays instead of std::map/std::function
 * to avoid dynamic memory allocation issues on embedded systems.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "esp_err.h"

namespace stampfly {

// ==========================================================================
// V1 Packet Structure - DEPRECATED (commented out for future removal)
// ==========================================================================
#if 0
/**
 * @brief Binary log packet structure V1 (64 bytes) - Legacy
 *
 * Used for ESKF debugging - sensor data only
 */
#pragma pack(push, 1)
struct BinaryLogPacketV1 {
    uint8_t header[2];      // 0xAA, 0x55
    uint32_t timestamp_ms;  // ms since boot
    float accel_x;          // [m/s²]
    float accel_y;
    float accel_z;
    float gyro_x;           // [rad/s]
    float gyro_y;
    float gyro_z;
    float mag_x;            // [uT]
    float mag_y;
    float mag_z;
    float pressure;         // [Pa]
    float baro_alt;         // [m]
    float tof_bottom;       // [m]
    float tof_front;        // [m]
    int16_t flow_dx;        // raw delta
    int16_t flow_dy;
    uint8_t flow_squal;     // surface quality
    uint8_t checksum;       // XOR of bytes 2-62
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacketV1) == 64, "BinaryLogPacketV1 must be 64 bytes");
#endif

/**
 * @brief Binary log packet structure V2 (128 bytes)
 *
 * Includes sensor data + ESKF estimates
 * Header: 0xAA, 0x56 (different from V1 for detection)
 */
#pragma pack(push, 1)
struct BinaryLogPacketV2 {
    // Header (2 bytes)
    uint8_t header[2];      // 0xAA, 0x56 (V2)

    // Timestamp (4 bytes)
    uint32_t timestamp_ms;  // ms since boot

    // IMU data (24 bytes)
    float accel_x;          // [m/s²]
    float accel_y;
    float accel_z;
    float gyro_x;           // [rad/s]
    float gyro_y;
    float gyro_z;

    // Magnetometer (12 bytes)
    float mag_x;            // [uT]
    float mag_y;
    float mag_z;

    // Barometer (8 bytes)
    float pressure;         // [Pa]
    float baro_alt;         // [m]

    // ToF (8 bytes)
    float tof_bottom;       // [m]
    float tof_front;        // [m]

    // Optical Flow (5 bytes)
    int16_t flow_dx;        // raw delta
    int16_t flow_dy;
    uint8_t flow_squal;     // surface quality

    // === ESKF Estimates (52 bytes) ===
    // Position (12 bytes)
    float pos_x;            // [m] NED
    float pos_y;
    float pos_z;

    // Velocity (12 bytes)
    float vel_x;            // [m/s] NED
    float vel_y;
    float vel_z;

    // Attitude (12 bytes)
    float roll;             // [rad]
    float pitch;
    float yaw;

    // Biases (12 bytes)
    float gyro_bias_z;      // [rad/s] Yaw bias only (most important)
    float accel_bias_x;     // [m/s²]
    float accel_bias_y;

    // Status + metadata (17 bytes total to reach 128)
    uint8_t eskf_status;    // 0=not initialized, 1=running
    float baro_ref_alt;     // [m] barometer reference altitude (for PC replay)
    uint8_t reserved[11];   // padding to reach 128 bytes
    uint8_t checksum;       // XOR of bytes 2-126
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacketV2) == 128, "BinaryLogPacketV2 must be 128 bytes");

// V1 alias - DEPRECATED
// using BinaryLogPacket = BinaryLogPacketV1;

/**
 * @brief Command handler function pointer type
 */
using CommandHandlerFn = void (*)(int argc, char** argv, void* context);

class CLI {
public:
    static constexpr size_t MAX_CMD_LEN = 128;
    static constexpr size_t MAX_ARGS = 8;
    static constexpr size_t MAX_COMMANDS = 25;
    static constexpr size_t MAX_CMD_NAME_LEN = 16;
    static constexpr size_t MAX_HELP_LEN = 48;

    CLI() = default;
    ~CLI() = default;

    /**
     * @brief Initialize CLI
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Register command handler
     * @param name Command name
     * @param handler Command handler function pointer
     * @param help Help text
     * @param context Optional context pointer passed to handler
     */
    void registerCommand(const char* name, CommandHandlerFn handler,
                         const char* help = "", void* context = nullptr);

    /**
     * @brief Process input (call periodically)
     */
    void processInput();

    /**
     * @brief Print to CLI output
     * @param format Printf format string
     */
    void print(const char* format, ...);

    /**
     * @brief Register default commands
     */
    void registerDefaultCommands();

    bool isInitialized() const { return initialized_; }

    /**
     * @brief Print help (available commands)
     */
    void printHelp();

    /**
     * @brief Check if teleplot streaming is enabled
     */
    bool isTeleplotEnabled() const { return teleplot_enabled_; }

    /**
     * @brief Set teleplot streaming state
     */
    void setTeleplotEnabled(bool enabled) { teleplot_enabled_ = enabled; }

    /**
     * @brief Output teleplot data (call periodically from task)
     */
    void outputTeleplot();

    /**
     * @brief Check if CSV logging is enabled
     */
    bool isLogEnabled() const { return log_enabled_; }

    /**
     * @brief Set CSV logging state
     */
    void setLogEnabled(bool enabled) { log_enabled_ = enabled; }

    /**
     * @brief Output CSV log data (call periodically from task)
     */
    void outputCSVLog();

    /**
     * @brief Get log sample counter
     */
    uint32_t getLogCounter() const { return log_counter_; }

    /**
     * @brief Reset log sample counter
     */
    void resetLogCounter() { log_counter_ = 0; }

    // ==========================================================================
    // V1 Binary Log - DEPRECATED (commented out for future removal)
    // ==========================================================================
#if 0
    /**
     * @brief Check if binary logging is enabled
     */
    bool isBinlogEnabled() const { return binlog_enabled_; }

    /**
     * @brief Set binary logging state
     */
    void setBinlogEnabled(bool enabled) { binlog_enabled_ = enabled; }

    /**
     * @brief Output binary log packet V1 (call periodically from task at 100Hz)
     */
    void outputBinaryLog();
#endif

    /**
     * @brief Output binary log packet V2 with ESKF estimates (call periodically from task at 100Hz)
     */
    void outputBinaryLogV2();

    /**
     * @brief Check if V2 binary logging is enabled
     */
    bool isBinlogV2Enabled() const { return binlog_v2_enabled_; }

    /**
     * @brief Set V2 binary logging state
     */
    void setBinlogV2Enabled(bool enabled) { binlog_v2_enabled_ = enabled; }

    /**
     * @brief Get binary log sample counter
     */
    uint32_t getBinlogCounter() const { return binlog_counter_; }

    /**
     * @brief Reset binary log sample counter
     */
    void resetBinlogCounter() { binlog_counter_ = 0; }

    /**
     * @brief Set callback for binlog start event
     * @param callback Function to call when binlog is started
     *
     * Used for setting mag_ref at binlog start for PC/device sync
     */
    void setBinlogStartCallback(void (*callback)()) { binlog_start_callback_ = callback; }

    /**
     * @brief Call binlog start callback if set
     */
    void callBinlogStartCallback() {
        if (binlog_start_callback_) {
            binlog_start_callback_();
        }
    }

private:
    /**
     * @brief Static command entry (no dynamic allocation)
     */
    struct CommandEntry {
        char name[MAX_CMD_NAME_LEN];
        char help[MAX_HELP_LEN];
        CommandHandlerFn handler;
        void* context;
    };

    void parseAndExecute(const char* line);

    bool initialized_ = false;
    bool teleplot_enabled_ = false;
    bool log_enabled_ = false;
    // bool binlog_enabled_ = false;  // V1 - DEPRECATED
    bool binlog_v2_enabled_ = false;
    uint32_t log_counter_ = 0;
    uint32_t binlog_counter_ = 0;
    CommandEntry commands_[MAX_COMMANDS] = {};
    size_t command_count_ = 0;
    char input_buffer_[MAX_CMD_LEN] = {0};
    size_t input_pos_ = 0;
    void (*binlog_start_callback_)() = nullptr;
};

}  // namespace stampfly
