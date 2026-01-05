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

// Sensor drivers
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

// Communication
#include "controller_comm.hpp"
#include "cli.hpp"
#include "logger.hpp"

// Math
#include "stampfly_math.hpp"

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
extern stampfly::AttitudeEstimator g_attitude_est;
extern stampfly::AltitudeEstimator g_altitude_est;

// =============================================================================
// Filters
// =============================================================================

extern stampfly::LowPassFilter g_accel_lpf[3];
extern stampfly::LowPassFilter g_gyro_lpf[3];

// =============================================================================
// Communication
// =============================================================================

extern stampfly::ControllerComm g_comm;
extern stampfly::CLI g_cli;
extern stampfly::Logger g_logger;

// =============================================================================
// Barometer Reference
// =============================================================================

extern float g_baro_reference_altitude;
extern bool g_baro_reference_set;

// =============================================================================
// Sensor Reference Buffers (for attitude initialization)
// =============================================================================

inline constexpr int REF_BUFFER_SIZE = 100;

// Accelerometer buffer (for roll/pitch initialization and ESKF)
extern stampfly::math::Vector3 g_accel_buffer[REF_BUFFER_SIZE];
extern int g_accel_buffer_index;
extern int g_accel_buffer_count;

// Gyroscope buffer (for ESKF)
extern stampfly::math::Vector3 g_gyro_buffer[REF_BUFFER_SIZE];
extern int g_gyro_buffer_index;
extern int g_gyro_buffer_count;

// Magnetometer buffer (for yaw=0 reference and ESKF)
extern stampfly::math::Vector3 g_mag_buffer[REF_BUFFER_SIZE];
extern int g_mag_buffer_index;
extern int g_mag_buffer_count;
extern bool g_mag_ref_set;

// Barometer buffer (for ESKF altitude)
extern float g_baro_buffer[REF_BUFFER_SIZE];
extern int g_baro_buffer_index;
extern int g_baro_buffer_count;

// ToF bottom buffer (for ESKF altitude)
extern float g_tof_bottom_buffer[REF_BUFFER_SIZE];
extern int g_tof_bottom_buffer_index;
extern int g_tof_bottom_buffer_count;

// ToF front buffer (for obstacle detection)
extern float g_tof_front_buffer[REF_BUFFER_SIZE];
extern int g_tof_front_buffer_index;
extern int g_tof_front_buffer_count;

// Optical flow buffer (for ESKF velocity)
struct OptFlowData {
    int16_t dx;
    int16_t dy;
    uint8_t squal;
};
extern OptFlowData g_optflow_buffer[REF_BUFFER_SIZE];
extern int g_optflow_buffer_index;
extern int g_optflow_buffer_count;

// =============================================================================
// Calibration Data
// =============================================================================

extern stampfly::math::Vector3 g_initial_gyro_bias;

// =============================================================================
// State Flags
// =============================================================================

extern volatile bool g_eskf_ready;

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
// Debug Checkpoints (C linkage for external access)
// =============================================================================

extern "C" {
    extern volatile uint8_t g_imu_checkpoint;
    extern volatile uint32_t g_imu_last_loop;
    extern volatile uint8_t g_optflow_checkpoint;
    extern volatile uint32_t g_optflow_last_loop;
}
