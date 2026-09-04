/**
 * @file workshop_api.hpp
 * @brief Workshop Simplified API for Students
 *        学生向けワークショップ簡易API
 *
 * This header provides a simple, Arduino-like interface to the
 * StampFly drone hardware. Students only need to implement:
 *   void setup();           // Called once at startup
 *   void loop_400Hz(float dt); // Called at 400Hz (every 2.5ms)
 *
 * All hardware complexity (FreeRTOS, SPI/I2C, sensor fusion) is
 * hidden behind the ws:: namespace functions.
 */

#pragma once

#include <cstdint>

// =============================================================================
// Student-implemented Functions
// 学生が実装する関数
// =============================================================================

/**
 * @brief Called once at startup (after all sensors are initialized)
 *        起動時に一度呼ばれる（全センサー初期化後）
 */
void setup();

/**
 * @brief Called at 400Hz (every 2.5ms)
 *        400Hzで呼ばれるメインループ
 * @param dt Time step in seconds (0.0025)
 */
void loop_400Hz(float dt);

// =============================================================================
// Workshop API Namespace
// ワークショップAPI名前空間
// =============================================================================

namespace ws {

// -----------------------------------------------------------------------------
// Communication Setup
// 通信設定
// -----------------------------------------------------------------------------

/**
 * @brief Set WiFi channel for ESP-NOW communication
 *        ESP-NOW通信のWiFiチャンネルを設定
 *
 * Use non-overlapping channels to avoid interference:
 * 混信を避けるため非重複チャンネルを使用:
 *   Channel 1, 6, or 11
 *
 * The channel is saved to flash memory and used on next boot.
 * チャンネルはフラッシュメモリに保存され、次回起動時に使用されます。
 *
 * @param channel WiFi channel (1, 6, or 11)
 */
void set_channel(int channel);

// -----------------------------------------------------------------------------
// Motor Control (Lesson 1)
// モーター制御
// -----------------------------------------------------------------------------

/**
 * @brief Set individual motor duty cycle
 *        個別モーターのデューティ比を設定
 * @param id Motor ID (1-4): 1=FR, 2=RR, 3=RL, 4=FL
 * @param duty Duty cycle (0.0 to 1.0)
 */
void motor_set_duty(int id, float duty);

/**
 * @brief Set all motors to the same duty cycle
 *        全モーターを同じデューティに設定
 * @param duty Duty cycle (0.0 to 1.0)
 */
void motor_set_all(float duty);

/**
 * @brief Stop all motors immediately
 *        全モーターを即座に停止
 */
void motor_stop_all();

/**
 * @brief Apply motor mixer (thrust + roll/pitch/yaw control)
 *        モーターミキサー適用（推力 + 姿勢制御）
 * @param thrust Thrust command (0.0 to 1.0)
 * @param roll   Roll torque (-1.0 to 1.0, positive = right wing down)
 * @param pitch  Pitch torque (-1.0 to 1.0, positive = nose up)
 * @param yaw    Yaw torque (-1.0 to 1.0, positive = clockwise from above)
 */
void motor_mixer(float thrust, float roll, float pitch, float yaw);

// -----------------------------------------------------------------------------
// Controller Input (Lesson 2)
// コントローラ入力
// -----------------------------------------------------------------------------

/**
 * @brief Get throttle stick value (0.0 to 1.0)
 *        スロットルスティック値を取得
 */
float rc_throttle();

/**
 * @brief Get roll stick value (-1.0 to 1.0)
 *        ロールスティック値を取得
 */
float rc_roll();

/**
 * @brief Get pitch stick value (-1.0 to 1.0)
 *        ピッチスティック値を取得
 */
float rc_pitch();

/**
 * @brief Get yaw stick value (-1.0 to 1.0)
 *        ヨースティック値を取得
 */
float rc_yaw();

/**
 * @brief ARM motors (enable motor output)
 *        モーターをARM（モーター出力を有効化）
 *
 * Motors will not spin until arm() is called.
 * モーターは arm() を呼ぶまで回りません。
 */
void arm();

/**
 * @brief DISARM motors (disable motor output and stop all motors)
 *        モーターをDISARM（モーター出力を無効化し全停止）
 */
void disarm();

/**
 * @brief Check if motors are armed
 *        モーターがARMされているか確認
 */
bool is_armed();

// -----------------------------------------------------------------------------
// Controller Buttons / Modes
// コントローラボタン・モード
// -----------------------------------------------------------------------------

/**
 * @brief Check if throttle/yaw stick button is pressed (raw state)
 *        スロットル/ヨースティックボタンの生押下状態を取得
 *
 * Note: This is also the ARM button. is_armed() returns the vehicle
 * flight state, while this returns the raw button press.
 * 注意: ARMボタンと同じ。is_armed()はVehicleの状態、これはボタンの生値。
 */
bool rc_throttle_yaw_button();

/**
 * @brief Check if roll/pitch stick button is pressed (raw state)
 *        ロール/ピッチスティックボタンの生押下状態を取得
 */
bool rc_roll_pitch_button();

/**
 * @brief Get stabilize/acro mode state (true = ACRO)
 *        スタビライズ/アクロモード状態を取得 (true = ACRO)
 */
bool rc_stabilize_acro_mode();

/**
 * @brief Get altitude hold mode state
 *        高度維持モード状態を取得
 */
bool rc_alt_mode();

/**
 * @brief Get position hold mode state
 *        位置保持モード状態を取得
 */
bool rc_pos_mode();

// -----------------------------------------------------------------------------
// LED Control (Lesson 3)
// LED制御
// -----------------------------------------------------------------------------

/**
 * @brief Disable system LED updates (flight state, battery, etc.)
 *        システムLED更新を無効化（飛行状態、バッテリー等）
 *
 * Call this in setup() so that ws::led_color() takes effect.
 * setup() 内で呼ぶと ws::led_color() の色が反映されるようになる。
 * Also enables loop_400Hz() to run during IDLE state.
 * IDLE状態でも loop_400Hz() が呼ばれるようになる。
 */
void disable_led_task();

/**
 * @brief Re-enable system LED updates
 *        システムLED更新を再有効化
 *
 * Restores automatic LED updates (flight state, battery, etc.)
 * after disable_led_task() was called.
 * disable_led_task() で無効化した後、自動LED更新を復元する。
 */
void enable_led_task();

/**
 * @brief Set LED color (RGB)
 *        LEDカラーを設定
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void led_color(uint8_t r, uint8_t g, uint8_t b);

// -----------------------------------------------------------------------------
// IMU Sensor (Lesson 4)
// IMUセンサー
// -----------------------------------------------------------------------------

/** @brief Get gyroscope X (roll rate) [rad/s] */
float gyro_x();

/** @brief Get gyroscope Y (pitch rate) [rad/s] */
float gyro_y();

/** @brief Get gyroscope Z (yaw rate) [rad/s] */
float gyro_z();

/** @brief Get accelerometer X (forward) [m/s^2] */
float accel_x();

/** @brief Get accelerometer Y (right) [m/s^2] */
float accel_y();

/** @brief Get accelerometer Z (down, NED) [m/s^2] */
float accel_z();

// -----------------------------------------------------------------------------
// Environmental / Distance Sensors (Lesson 10)
// 環境・距離センサ
// -----------------------------------------------------------------------------

/** @brief Get barometric altitude [m] (BMP280) */
float baro_altitude();

/** @brief Get barometric pressure [Pa] (BMP280) */
float baro_pressure();

/** @brief Get magnetometer X [uT] (BMM150) */
float mag_x();

/** @brief Get magnetometer Y [uT] (BMM150) */
float mag_y();

/** @brief Get magnetometer Z [uT] (BMM150) */
float mag_z();

/** @brief Get bottom ToF distance [m] (VL53L3CX, 0-2m) */
float tof_bottom();

/** @brief Get front ToF distance [m] (VL53L3CX, 0-2m, -1 if unavailable) */
float tof_front();

/** @brief Get optical flow velocity X [m/s] (PMW3901) */
float flow_vx();

/** @brief Get optical flow velocity Y [m/s] (PMW3901) */
float flow_vy();

/** @brief Get optical flow surface quality (0-255, higher=better) */
uint8_t flow_quality();

// -----------------------------------------------------------------------------
// Estimation (Lesson 9)
// 姿勢推定
// -----------------------------------------------------------------------------

/** @brief Get estimated roll angle [rad] */
float estimated_roll();

/** @brief Get estimated pitch angle [rad] */
float estimated_pitch();

/** @brief Get estimated yaw angle [rad] */
float estimated_yaw();

/** @brief Get estimated altitude [m] (positive = up) */
float estimated_altitude();

// -----------------------------------------------------------------------------
// Control Target Logging (Lesson 7: System Identification)
// 制御目標のロギング（レッスン7: システム同定）
// -----------------------------------------------------------------------------

/**
 * @brief Record the current inner-loop (rate) control target for the 400Hz
 *        Data Stream
 *        400Hz Data Stream 用に、現在の内側ループ（角速度）制御目標を記録する
 *
 * Call this once per loop_400Hz(), right after computing the target angular
 * rate your control law is chasing (e.g. right after
 * `roll_target = ws::rc_roll() * rate_max`). WorkshopControlTask copies the
 * value into the Data Stream's rate_ref[] field once per cycle so `sf sysid
 * fit` (Lesson 7) can reconstruct the plant input from
 * u = Kp * (rate_ref - gyro). This is a LOGGING HOOK ONLY — it does not
 * affect motor output, and flight works without calling it.
 *
 * loop_400Hz() 内で、制御則が追う角速度目標を計算した直後に一度呼ぶ（例:
 * `roll_target = ws::rc_roll() * rate_max` の直後）。WorkshopControlTask が
 * 周期に一度、その値を Data Stream の rate_ref[] へコピーする。これにより
 * `sf sysid fit`（レッスン7）が u = Kp * (rate_ref - gyro) でプラント入力を
 * 復元できる。これは「ロギング専用フック」であり、モータ出力には影響しない
 * — 呼ばなくても飛行自体は動く。
 *
 * @param roll  Roll rate target [rad/s], Body FRD (positive = right wing down)
 * @param pitch Pitch rate target [rad/s], Body FRD (positive = nose up)
 * @param yaw   Yaw rate target [rad/s], Body FRD (positive = clockwise from above)
 */
void set_rate_target(float roll, float pitch, float yaw);

/**
 * @brief Record the current outer-loop (tilt angle) control target for the
 *        400Hz Data Stream
 *        400Hz Data Stream 用に、現在の外側ループ（傾き角）制御目標を記録する
 *
 * Same logging-only contract as set_rate_target(): call once per
 * loop_400Hz() after computing the target tilt angle (only meaningful for a
 * cascade/angle controller — most lessons only use set_rate_target()). Read
 * into the Data Stream's angle_ref[] field; has no effect on control.
 *
 * set_rate_target() と同じ「ロギング専用」契約: loop_400Hz() 内で目標傾き角
 * を計算した直後に一度呼ぶ（カスケード/角度制御でのみ意味を持つ — ほとんどの
 * レッスンは set_rate_target() のみ使用）。Data Stream の angle_ref[] へ
 * 記録される。制御には影響しない。
 *
 * @param roll  Roll angle target [rad], Body FRD (positive = right wing down)
 * @param pitch Pitch angle target [rad], Body FRD (positive = nose up)
 */
void set_angle_target(float roll, float pitch);

// -----------------------------------------------------------------------------
// Utility
// ユーティリティ
// -----------------------------------------------------------------------------

/** @brief Check if system LED task is disabled */
bool is_led_task_disabled();

/** @brief Get elapsed time since boot [ms] */
uint32_t millis();

/** @brief Get battery voltage [V] */
float battery_voltage();

/** @brief Print formatted debug message to serial console */
void print(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

} // namespace ws
