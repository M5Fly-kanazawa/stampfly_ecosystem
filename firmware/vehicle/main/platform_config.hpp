/**
 * @file platform_config.hpp
 * @brief Platform hardware configuration for StampFly board
 *        StampFly ボードのプラットフォームハードウェア設定
 *
 * GPIO definitions, task scheduling, sensor driver settings,
 * communication settings, and other board-level constants.
 * Any application running on StampFly hardware uses these.
 *
 * GPIO定義、タスクスケジューリング、センサードライバ設定、
 * 通信設定、その他ボードレベルの定数。
 * StampFly ハードウェア上で動作する全アプリが使用。
 */

#pragma once

#include "freertos/FreeRTOS.h"

namespace config {

// =============================================================================
// GPIO Definitions
// =============================================================================

// SPI Bus
inline constexpr int GPIO_SPI_MOSI = 14;
inline constexpr int GPIO_SPI_MISO = 43;
inline constexpr int GPIO_SPI_SCK = 44;
inline constexpr int GPIO_IMU_CS = 46;
inline constexpr int GPIO_FLOW_CS = 12;

// I2C Bus
inline constexpr int GPIO_I2C_SDA = 3;
inline constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT
inline constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
inline constexpr int GPIO_TOF_XSHUT_FRONT = 9;

// Motors (LEDC PWM)
inline constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
inline constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
inline constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
inline constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
inline constexpr int GPIO_LED_MCU = 21;    // M5Stamp S3 内蔵LED
inline constexpr int GPIO_LED_BODY = 39;   // StampFly ボード上LED（2個直列）
inline constexpr int GPIO_LED = 39;        // 後方互換性のため残す（deprecated）
inline constexpr int GPIO_BUZZER = 40;
inline constexpr int GPIO_BUTTON = 0;

// =============================================================================
// Task Priorities
// =============================================================================

inline constexpr UBaseType_t PRIORITY_IMU_TASK = 24;
inline constexpr UBaseType_t PRIORITY_CONTROL_TASK = 23;
inline constexpr UBaseType_t PRIORITY_OPTFLOW_TASK = 20;
inline constexpr UBaseType_t PRIORITY_MAG_TASK = 18;
inline constexpr UBaseType_t PRIORITY_BARO_TASK = 16;
inline constexpr UBaseType_t PRIORITY_COMM_TASK = 15;
inline constexpr UBaseType_t PRIORITY_TOF_TASK = 14;
inline constexpr UBaseType_t PRIORITY_TELEMETRY_TASK = 13;
inline constexpr UBaseType_t PRIORITY_POWER_TASK = 12;
inline constexpr UBaseType_t PRIORITY_BUTTON_TASK = 10;
inline constexpr UBaseType_t PRIORITY_LED_TASK = 8;
inline constexpr UBaseType_t PRIORITY_CLI_TASK = 5;

// =============================================================================
// Task Stack Sizes
// =============================================================================

inline constexpr uint32_t STACK_SIZE_IMU = 16384;      // ESKF行列演算用
inline constexpr uint32_t STACK_SIZE_CONTROL = 8192;
inline constexpr uint32_t STACK_SIZE_OPTFLOW = 8192;
inline constexpr uint32_t STACK_SIZE_MAG = 8192;
inline constexpr uint32_t STACK_SIZE_BARO = 8192;
inline constexpr uint32_t STACK_SIZE_TOF = 8192;
inline constexpr uint32_t STACK_SIZE_POWER = 4096;
inline constexpr uint32_t STACK_SIZE_LED = 4096;
inline constexpr uint32_t STACK_SIZE_BUTTON = 4096;
inline constexpr uint32_t STACK_SIZE_COMM = 4096;
inline constexpr uint32_t STACK_SIZE_CLI = 8192;  // Increased for WiFi command output
inline constexpr uint32_t STACK_SIZE_TELEMETRY = 4096;

// =============================================================================
// Timing Constants
// =============================================================================

inline constexpr float IMU_DT = 0.0025f;          // 400Hz
inline constexpr float OPTFLOW_DT = 0.01f;        // 100Hz
inline constexpr float MAG_DT = 0.04f;            // 25Hz
inline constexpr float BARO_DT = 0.02f;           // 50Hz
inline constexpr float TOF_DT = 0.033f;           // 30Hz

// =============================================================================
// Sensor Thresholds
// =============================================================================

// Optical Flow
inline constexpr uint8_t FLOW_SQUAL_MIN = 0x19;   // 最小品質閾値
inline constexpr float FLOW_DISTANCE_MIN = 0.02f; // [m]
inline constexpr float FLOW_DISTANCE_MAX = 4.0f;  // [m]

// ToF
inline constexpr float TOF_DISTANCE_MIN = 0.01f;  // [m]
inline constexpr float TOF_DISTANCE_MAX = 4.0f;   // [m]

// =============================================================================
// LPF (Low Pass Filter) Settings
// =============================================================================

namespace lpf {
inline constexpr float ACCEL_CUTOFF_HZ = 50.0f;    // 加速度LPFカットオフ [Hz]
inline constexpr float GYRO_CUTOFF_HZ = 100.0f;    // ジャイロLPFカットオフ [Hz]
} // namespace lpf

// =============================================================================
// Sensor Driver Settings
// =============================================================================

namespace sensor {

// BMM150 (地磁気センサー)
// data_rate: 0=10Hz, 1=2Hz, 2=6Hz, 3=8Hz, 4=15Hz, 5=20Hz, 6=25Hz, 7=30Hz
inline constexpr int BMM150_DATA_RATE = 6;         // ODR_25HZ
// preset: 0=LOW_POWER, 1=REGULAR, 2=ENHANCED, 3=HIGH_ACCURACY
inline constexpr int BMM150_PRESET = 1;            // REGULAR

// BMP280 (気圧センサー)
// mode: 0=SLEEP, 1=FORCED, 3=NORMAL (2は無効値)
inline constexpr int BMP280_MODE = 3;              // NORMAL
// oversampling: 0=SKIP, 1=X1, 2=X2, 3=X4, 4=X8, 5=X16
inline constexpr int BMP280_PRESS_OVERSAMPLING = 3; // X4
inline constexpr int BMP280_TEMP_OVERSAMPLING = 2;  // X2
// standby: 0=0.5ms, 1=62.5ms, 2=125ms, 3=250ms, 4=500ms, 5=1000ms, 6=2000ms, 7=4000ms
inline constexpr int BMP280_STANDBY = 1;           // MS_62_5
// filter: 0=OFF, 1=COEF_2, 2=COEF_4, 3=COEF_8, 4=COEF_16
inline constexpr int BMP280_FILTER = 2;            // COEF_4

// INA3221 (電源モニター)
inline constexpr int POWER_BATTERY_CHANNEL = 1;    // バッテリー接続チャンネル
inline constexpr float POWER_SHUNT_RESISTOR = 0.1f; // シャント抵抗 [Ω]

} // namespace sensor

// =============================================================================
// Communication Settings
// =============================================================================

namespace comm {
inline constexpr int WIFI_CHANNEL = 1;             // ESP-NOW WiFiチャンネル
inline constexpr int TIMEOUT_MS = 500;             // 通信タイムアウト [ms]
} // namespace comm

namespace telemetry {
inline constexpr int PORT = 80;                    // WebSocketポート
inline constexpr int RATE_HZ = 50;                 // 送信レート [Hz]
} // namespace telemetry

namespace logger {
inline constexpr int RATE_HZ = 400;                // ログレート [Hz]
} // namespace logger

// =============================================================================
// Actuator Settings
// =============================================================================

namespace motor {
inline constexpr int PWM_FREQ_HZ = 150000;         // PWM周波数 [Hz]
inline constexpr int PWM_RESOLUTION_BITS = 8;      // PWM分解能 [bits]
} // namespace motor

namespace buzzer {
inline constexpr int LEDC_CHANNEL = 4;             // LEDCチャンネル
inline constexpr int LEDC_TIMER = 1;               // LEDCタイマー
} // namespace buzzer

namespace button {
inline constexpr int DEBOUNCE_MS = 50;             // デバウンス時間 [ms]
} // namespace button

namespace led {
// M5Stamp S3 内蔵LED
inline constexpr int NUM_LEDS_MCU = 1;

// StampFly ボード上LED（デイジーチェーン）
inline constexpr int NUM_LEDS_BODY = 2;

// LED インデックス（GPIO_LED_BODYのデイジーチェーン内）
inline constexpr int IDX_BODY_TOP = 0;     // 上面/表
inline constexpr int IDX_BODY_BOTTOM = 1;  // 下面/裏

// 後方互換性のため残す（deprecated）
inline constexpr int NUM_LEDS = 1;
} // namespace led

} // namespace config
