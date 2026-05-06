/**
 * @file globals.hpp
 * @brief グローバル変数のextern宣言
 *
 * 全てのタスク・初期化関数から共有されるグローバル変数
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

// Sensor drivers (センサードライバ)
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "mag_calibration.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators
#include "motor_driver.hpp"
#include "led.hpp"
#include "led_manager.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// Estimation
#include "sensor_fusion.hpp"
#include "sensor_health.hpp"
#include "system_manager.hpp"
#include "filter.hpp"
#include "landing_handler.hpp"

// Communication
#include "controller_comm.hpp"
#include "logger.hpp"

// Math
#include "stampfly_math.hpp"

// Ring buffer template
#include "ring_buffer.hpp"

namespace globals {

// =============================================================================
// Sensors
// =============================================================================

extern stampfly::BMI270Wrapper g_imu;
extern stampfly::BMM150 g_mag;
extern stampfly::MagCalibrator g_mag_cal;
extern stampfly::BMP280 g_baro;
extern stampfly::VL53L3CXWrapper g_tof_bottom;
extern stampfly::VL53L3CXWrapper g_tof_front;
extern stampfly::PMW3901* g_optflow;
extern stampfly::PowerMonitor g_power;

// =============================================================================
// Actuators
// =============================================================================

extern stampfly::MotorDriver g_motor;
// g_led は削除済み - LEDManager を使用
extern stampfly::Buzzer g_buzzer;
extern stampfly::Button g_button;

// =============================================================================
// Estimators
// =============================================================================

extern sf::SensorFusion g_fusion;
extern stampfly::LandingHandler g_landing_handler;

// =============================================================================
// Filters
// =============================================================================

extern stampfly::LowPassFilter g_accel_lpf[3];
extern stampfly::LowPassFilter g_gyro_lpf[3];

// =============================================================================
// Communication
// =============================================================================

extern stampfly::ControllerComm g_comm;
extern stampfly::Logger g_logger;

// =============================================================================
// Barometer Reference
// =============================================================================

extern float g_baro_reference_altitude;
extern bool g_baro_reference_set;

// =============================================================================
// Sensor Ring Buffers (sized per sensor rate)
// センサーリングバッファ（各センサーのレートに応じたサイズ）
// =============================================================================

// Buffer sizes (each sized for ~0.5s at sensor rate)
// バッファサイズ（各センサーレートで約0.5秒分）
inline constexpr int IMU_BUFFER_SIZE  = 200;  // 400Hz × 0.5s
inline constexpr int MAG_BUFFER_SIZE  = 16;   // 25Hz × 0.64s
inline constexpr int BARO_BUFFER_SIZE = 32;   // 50Hz × 0.64s
inline constexpr int TOF_BUFFER_SIZE  = 16;   // 30Hz × 0.53s
inline constexpr int FLOW_BUFFER_SIZE = 64;   // 100Hz × 0.64s

// IMU group (same size, telemetry reads all with same index)
// IMUグループ（同サイズ、テレメトリは同じインデックスで読み出す）
extern RingBuffer<stampfly::math::Vector3, IMU_BUFFER_SIZE> g_accel_buf;
extern RingBuffer<stampfly::math::Vector3, IMU_BUFFER_SIZE> g_gyro_buf;
extern RingBuffer<stampfly::math::Vector3, IMU_BUFFER_SIZE> g_accel_raw_buf;
extern RingBuffer<stampfly::math::Vector3, IMU_BUFFER_SIZE> g_gyro_raw_buf;
// Note: IMU timestamp is stored via push(value, ts_us) in g_accel_raw_buf
// 注: IMUタイムスタンプは g_accel_raw_buf の push(value, ts_us) で格納

// Sensor last-read timestamps (volatile, set by each sensor task)
// 各センサータスクが最終取得時刻を記録
extern volatile uint32_t g_baro_last_timestamp_us;
extern volatile uint32_t g_tof_bottom_last_timestamp_us;
extern volatile uint32_t g_tof_front_last_timestamp_us;
extern volatile uint32_t g_mag_last_timestamp_us;
extern volatile uint32_t g_flow_last_timestamp_us;

// Independent sensor buffers
// 独立センサーバッファ
extern RingBuffer<stampfly::math::Vector3, MAG_BUFFER_SIZE> g_mag_buf;
extern bool g_mag_ref_set;

extern RingBuffer<float, BARO_BUFFER_SIZE> g_baro_buf;

extern RingBuffer<float, TOF_BUFFER_SIZE> g_tof_bottom_buf;
extern RingBuffer<float, TOF_BUFFER_SIZE> g_tof_front_buf;

struct OptFlowData {
    int16_t dx;
    int16_t dy;
    uint8_t squal;
};
extern RingBuffer<OptFlowData, FLOW_BUFFER_SIZE> g_flow_buf;

// =============================================================================
// Calibration Data
// =============================================================================

extern stampfly::math::Vector3 g_initial_gyro_bias;
extern stampfly::math::Vector3 g_initial_accel_bias;

// =============================================================================
// State Flags
// =============================================================================

extern volatile bool g_eskf_ready;
extern volatile bool g_boot_complete;

// Sensor health flags (for cross-task communication)
extern volatile bool g_imu_task_healthy;
extern volatile bool g_tof_task_healthy;
extern volatile bool g_mag_task_healthy;
extern volatile bool g_optflow_task_healthy;
extern volatile bool g_baro_task_healthy;

// Health monitor (centralized counting logic)
extern sf::HealthMonitor g_health;

// Data ready flags (新しいデータがバッファに追加されたことを示す)
extern volatile bool g_mag_data_ready;
extern volatile bool g_baro_data_ready;
extern volatile bool g_tof_bottom_data_ready;
extern volatile bool g_tof_front_data_ready;
extern volatile bool g_optflow_data_ready;

// =============================================================================
// Task Handles
// =============================================================================

extern TaskHandle_t g_imu_task_handle;
extern TaskHandle_t g_control_task_handle;
extern TaskHandle_t g_optflow_task_handle;
extern TaskHandle_t g_mag_task_handle;
extern TaskHandle_t g_baro_task_handle;
extern TaskHandle_t g_tof_task_handle;
extern TaskHandle_t g_power_task_handle;
extern TaskHandle_t g_led_task_handle;
extern TaskHandle_t g_button_task_handle;
extern TaskHandle_t g_comm_task_handle;
extern TaskHandle_t g_cli_task_handle;
extern TaskHandle_t g_telemetry_task_handle;

// =============================================================================
// Timer and Synchronization
// =============================================================================

extern esp_timer_handle_t g_imu_timer;
extern SemaphoreHandle_t g_imu_semaphore;
extern SemaphoreHandle_t g_control_semaphore;
extern SemaphoreHandle_t g_telemetry_imu_semaphore;  // For FFT mode sync with IMU

} // namespace globals

// =============================================================================
// CLI-accessible Pointers (global namespace for CLI component access)
// =============================================================================

extern stampfly::MagCalibrator* g_mag_calibrator;
extern stampfly::Logger* g_logger_ptr;
extern stampfly::ControllerComm* g_comm_ptr;
// g_led_ptr は削除済み - LEDManager を使用
extern stampfly::MotorDriver* g_motor_ptr;
extern stampfly::Buzzer* g_buzzer_ptr;
extern sf::SensorFusion* g_fusion_ptr;

// =============================================================================
// Shared Control Handler (used by both ESP-NOW and UDP callbacks)
// 共有制御ハンドラ（ESP-NOWとUDPコールバック両方で使用）
// =============================================================================

/**
 * @brief Handle control input from any source (ESP-NOW, UDP)
 *        任意のソース（ESP-NOW、UDP）からの制御入力を処理
 *
 * @param throttle Raw throttle value [0-4095]
 * @param roll Raw roll value [0-4095], center=2048
 * @param pitch Raw pitch value [0-4095], center=2048
 * @param yaw Raw yaw value [0-4095], center=2048
 * @param flags Control flags (ARM, FLIP, MODE, ALT_MODE)
 */
void handleControlInput(uint16_t throttle, uint16_t roll, uint16_t pitch,
                        uint16_t yaw, uint8_t flags);

// =============================================================================
// Debug Checkpoints (C linkage for external access)
// =============================================================================

extern "C" {
    extern volatile uint8_t g_imu_checkpoint;
    extern volatile uint32_t g_imu_last_loop;
    extern volatile uint8_t g_optflow_checkpoint;
    extern volatile uint32_t g_optflow_last_loop;
}
