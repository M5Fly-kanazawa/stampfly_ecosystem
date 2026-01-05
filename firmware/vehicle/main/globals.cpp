/**
 * @file globals.cpp
 * @brief グローバル変数の実体定義
 */

#include "globals.hpp"

namespace globals {

// =============================================================================
// Sensors
// =============================================================================

stampfly::BMI270Wrapper g_imu;
stampfly::BMM150 g_mag;
stampfly::MagCalibrator g_mag_cal;
stampfly::BMP280 g_baro;
stampfly::VL53L3CXWrapper g_tof_bottom;
stampfly::VL53L3CXWrapper g_tof_front;
stampfly::PMW3901* g_optflow = nullptr;
stampfly::PowerMonitor g_power;

// =============================================================================
// Actuators
// =============================================================================

stampfly::MotorDriver g_motor;
// g_led は削除済み - LEDManager を使用
stampfly::Buzzer g_buzzer;
stampfly::Button g_button;

// =============================================================================
// Estimators
// =============================================================================

sf::SensorFusion g_fusion;
stampfly::AttitudeEstimator g_attitude_est;
stampfly::AltitudeEstimator g_altitude_est;

// =============================================================================
// Filters
// =============================================================================

stampfly::LowPassFilter g_accel_lpf[3];
stampfly::LowPassFilter g_gyro_lpf[3];

// =============================================================================
// Communication
// =============================================================================

stampfly::ControllerComm g_comm;
stampfly::CLI g_cli;
stampfly::Logger g_logger;

// =============================================================================
// Barometer Reference
// =============================================================================

float g_baro_reference_altitude = 0.0f;
bool g_baro_reference_set = false;

// =============================================================================
// Sensor Reference Buffers
// =============================================================================

// Accelerometer buffer (for roll/pitch initialization and ESKF)
stampfly::math::Vector3 g_accel_buffer[REF_BUFFER_SIZE];
int g_accel_buffer_index = 0;
int g_accel_buffer_count = 0;

// Gyroscope buffer (for ESKF)
stampfly::math::Vector3 g_gyro_buffer[REF_BUFFER_SIZE];
int g_gyro_buffer_index = 0;
int g_gyro_buffer_count = 0;

// Magnetometer buffer (for yaw=0 reference and ESKF)
stampfly::math::Vector3 g_mag_buffer[REF_BUFFER_SIZE];
int g_mag_buffer_index = 0;
int g_mag_buffer_count = 0;
bool g_mag_ref_set = false;

// Barometer buffer (for ESKF altitude)
float g_baro_buffer[REF_BUFFER_SIZE];
int g_baro_buffer_index = 0;
int g_baro_buffer_count = 0;

// ToF bottom buffer (for ESKF altitude)
float g_tof_bottom_buffer[REF_BUFFER_SIZE];
int g_tof_bottom_buffer_index = 0;
int g_tof_bottom_buffer_count = 0;

// ToF front buffer (for obstacle detection)
float g_tof_front_buffer[REF_BUFFER_SIZE];
int g_tof_front_buffer_index = 0;
int g_tof_front_buffer_count = 0;

// Optical flow buffer (for ESKF velocity)
OptFlowData g_optflow_buffer[REF_BUFFER_SIZE];
int g_optflow_buffer_index = 0;
int g_optflow_buffer_count = 0;

// =============================================================================
// Calibration Data
// =============================================================================

stampfly::math::Vector3 g_initial_gyro_bias = stampfly::math::Vector3::zero();

// =============================================================================
// State Flags
// =============================================================================

volatile bool g_eskf_ready = false;

volatile bool g_imu_task_healthy = false;
volatile bool g_tof_task_healthy = false;
volatile bool g_mag_task_healthy = false;
volatile bool g_optflow_task_healthy = false;
volatile bool g_baro_task_healthy = false;

sf::HealthMonitor g_health;

// Data ready flags (新しいデータがバッファに追加されたことを示す)
volatile bool g_mag_data_ready = false;
volatile bool g_baro_data_ready = false;
volatile bool g_tof_bottom_data_ready = false;
volatile bool g_tof_front_data_ready = false;
volatile bool g_optflow_data_ready = false;

// =============================================================================
// Task Handles
// =============================================================================

TaskHandle_t g_imu_task_handle = nullptr;
TaskHandle_t g_control_task_handle = nullptr;
TaskHandle_t g_optflow_task_handle = nullptr;
TaskHandle_t g_mag_task_handle = nullptr;
TaskHandle_t g_baro_task_handle = nullptr;
TaskHandle_t g_tof_task_handle = nullptr;
TaskHandle_t g_power_task_handle = nullptr;
TaskHandle_t g_led_task_handle = nullptr;
TaskHandle_t g_button_task_handle = nullptr;
TaskHandle_t g_comm_task_handle = nullptr;
TaskHandle_t g_cli_task_handle = nullptr;
TaskHandle_t g_telemetry_task_handle = nullptr;

// =============================================================================
// Timer and Synchronization
// =============================================================================

esp_timer_handle_t g_imu_timer = nullptr;
SemaphoreHandle_t g_imu_semaphore = nullptr;
SemaphoreHandle_t g_control_semaphore = nullptr;

} // namespace globals

// =============================================================================
// CLI-accessible Pointers (global namespace for CLI component access)
// =============================================================================

stampfly::MagCalibrator* g_mag_calibrator = nullptr;
stampfly::Logger* g_logger_ptr = nullptr;
stampfly::ControllerComm* g_comm_ptr = nullptr;
// g_led_ptr は削除済み - LEDManager を使用
stampfly::MotorDriver* g_motor_ptr = nullptr;
stampfly::Buzzer* g_buzzer_ptr = nullptr;
sf::SensorFusion* g_fusion_ptr = nullptr;

// =============================================================================
// Debug Checkpoints
// =============================================================================

extern "C" {
    volatile uint8_t g_imu_checkpoint = 0;
    volatile uint32_t g_imu_last_loop = 0;
    volatile uint8_t g_optflow_checkpoint = 0;
    volatile uint32_t g_optflow_last_loop = 0;
}
