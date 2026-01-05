/**
 * @file logger.hpp
 * @brief High-precision binary logger component
 *
 * ESP Timerによる高精度タイミングとキューによるノンブロッキングログ出力
 * デフォルト400Hz、10-1000Hzの範囲で設定可能
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace stampfly {

/**
 * @brief Binary log packet structure V2 (128 bytes)
 *
 * センサデータ + ESKF推定値を含む
 * Header: 0xAA, 0x56
 */
#pragma pack(push, 1)
struct LogPacket {
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
    float gyro_bias_z;      // [rad/s] Yaw bias only
    float accel_bias_x;     // [m/s²]
    float accel_bias_y;

    // Status + metadata (17 bytes total to reach 128)
    uint8_t eskf_status;    // 0=not initialized, 1=running
    float baro_ref_alt;     // [m] barometer reference altitude
    uint8_t reserved[11];   // padding to reach 128 bytes
    uint8_t checksum;       // XOR of bytes 2-126
};
#pragma pack(pop)

static_assert(sizeof(LogPacket) == 128, "LogPacket must be 128 bytes");

/**
 * @brief High-precision binary logger
 *
 * ESKFタスクからノンブロッキングでデータを受け取り、
 * ESP Timerで指定された周波数でUSB出力する
 */
class Logger {
public:
    static constexpr uint32_t DEFAULT_FREQUENCY_HZ = 400;
    static constexpr uint32_t MIN_FREQUENCY_HZ = 10;
    static constexpr uint32_t MAX_FREQUENCY_HZ = 1000;
    static constexpr size_t DEFAULT_QUEUE_SIZE = 16;
    static constexpr size_t WRITER_TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t WRITER_TASK_PRIORITY = 5;

    Logger() = default;
    ~Logger();

    /**
     * @brief ロガー初期化
     * @param frequency_hz ログ出力周波数（10-1000Hz、デフォルト400Hz）
     * @param queue_size キューサイズ（デフォルト16パケット）
     * @return ESP_OK on success
     */
    esp_err_t init(uint32_t frequency_hz = DEFAULT_FREQUENCY_HZ,
                   size_t queue_size = DEFAULT_QUEUE_SIZE);

    /**
     * @brief ログ出力開始
     */
    void start();

    /**
     * @brief ログ出力停止
     */
    void stop();

    /**
     * @brief ログデータをキューに投入（ノンブロッキング）
     * @param packet ログパケット
     * @return true: 成功, false: キュー満杯
     *
     * ESKFタスクから呼び出される。ブロックしない。
     */
    bool pushData(const LogPacket& packet);

    /**
     * @brief 現在のログ周波数を取得
     */
    uint32_t getFrequency() const { return frequency_hz_; }

    /**
     * @brief ログ周波数を変更
     * @param frequency_hz 新しい周波数（10-1000Hz）
     * @return ESP_OK on success
     */
    esp_err_t setFrequency(uint32_t frequency_hz);

    /**
     * @brief ログカウンタを取得
     */
    uint32_t getLogCount() const { return log_count_; }

    /**
     * @brief ドロップカウンタを取得（キュー満杯で失われたパケット数）
     */
    uint32_t getDropCount() const { return drop_count_; }

    /**
     * @brief カウンタをリセット
     */
    void resetCounters() { log_count_ = 0; drop_count_ = 0; }

    /**
     * @brief ログ出力中か確認
     */
    bool isRunning() const { return running_; }

    /**
     * @brief 初期化済みか確認
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief 開始時コールバック設定
     * @param callback ログ開始時に呼ばれる関数
     */
    void setStartCallback(void (*callback)()) { start_callback_ = callback; }

private:
    static void timerCallback(void* arg);
    static void writerTask(void* arg);

    uint8_t calculateChecksum(const LogPacket& pkt);

    uint32_t frequency_hz_ = DEFAULT_FREQUENCY_HZ;
    bool initialized_ = false;
    volatile bool running_ = false;
    volatile uint32_t log_count_ = 0;
    volatile uint32_t drop_count_ = 0;

    esp_timer_handle_t timer_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t write_sem_ = nullptr;
    TaskHandle_t writer_task_ = nullptr;

    void (*start_callback_)() = nullptr;
};

}  // namespace stampfly
