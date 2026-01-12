/**
 * @file control_task.cpp
 * @brief 制御タスク (400Hz) - 角速度制御（Rate Control）
 *
 * コントローラ入力から目標角速度を計算し、PID制御でモーター出力を決定
 *
 * 物理単位モード (USE_PHYSICAL_UNITS=1):
 *   PID出力 [Nm] → ControlAllocator → モータ推力 [N] → Duty → モーター
 *
 * 電圧スケールモード (USE_PHYSICAL_UNITS=0):
 *   PID出力 [V] → レガシーミキサー → Duty → モーター
 */

#include "tasks_common.hpp"
#include "../rate_controller.hpp"
#include "../attitude_controller.hpp"
#include "control_allocation.hpp"
#include "motor_model.hpp"
#include "controller_comm.hpp"  // for CTRL_FLAG_MODE
#include "led_manager.hpp"      // for flight mode LED indication

static const char* TAG = "ControlTask";

// Mode switch debounce counter for noise immunity
// ノイズ耐性のためのモード切替デバウンスカウンタ
// FlightMode is defined in stampfly_state.hpp
static constexpr int MODE_SWITCH_DEBOUNCE_COUNT = 10;  // 10 cycles @ 400Hz = 25ms
static int g_mode_switch_counter = 0;
static stampfly::FlightMode g_pending_mode = stampfly::FlightMode::ACRO;

using namespace config;
using namespace globals;

// Debug: Yaw alert counter (1秒 = 400カウント @ 400Hz)
volatile int g_yaw_alert_counter = 0;

// 衝撃検出用カウンタ（加速度・角速度それぞれ）
static int g_impact_accel_count = 0;
static int g_impact_gyro_count = 0;

// =============================================================================
// Rate Controller Implementation
// =============================================================================

void RateController::init() {
    // デフォルト値で初期化
    roll_rate_max = rate_control::ROLL_RATE_MAX;
    pitch_rate_max = rate_control::PITCH_RATE_MAX;
    yaw_rate_max = rate_control::YAW_RATE_MAX;

    // Roll PID (軸別出力制限を使用)
    stampfly::PIDConfig roll_cfg;
    roll_cfg.Kp = rate_control::ROLL_RATE_KP;
    roll_cfg.Ti = rate_control::ROLL_RATE_TI;
    roll_cfg.Td = rate_control::ROLL_RATE_TD;
    roll_cfg.eta = rate_control::PID_ETA;
    roll_cfg.output_min = -rate_control::ROLL_OUTPUT_LIMIT;
    roll_cfg.output_max = rate_control::ROLL_OUTPUT_LIMIT;
    roll_cfg.derivative_on_measurement = true;
    roll_pid.init(roll_cfg);

    // Pitch PID (軸別出力制限を使用)
    stampfly::PIDConfig pitch_cfg;
    pitch_cfg.Kp = rate_control::PITCH_RATE_KP;
    pitch_cfg.Ti = rate_control::PITCH_RATE_TI;
    pitch_cfg.Td = rate_control::PITCH_RATE_TD;
    pitch_cfg.eta = rate_control::PID_ETA;
    pitch_cfg.output_min = -rate_control::PITCH_OUTPUT_LIMIT;
    pitch_cfg.output_max = rate_control::PITCH_OUTPUT_LIMIT;
    pitch_cfg.derivative_on_measurement = true;
    pitch_pid.init(pitch_cfg);

    // Yaw PID (軸別出力制限を使用)
    stampfly::PIDConfig yaw_cfg;
    yaw_cfg.Kp = rate_control::YAW_RATE_KP;
    yaw_cfg.Ti = rate_control::YAW_RATE_TI;
    yaw_cfg.Td = rate_control::YAW_RATE_TD;
    yaw_cfg.eta = rate_control::PID_ETA;
    yaw_cfg.output_min = -rate_control::YAW_OUTPUT_LIMIT;
    yaw_cfg.output_max = rate_control::YAW_OUTPUT_LIMIT;
    yaw_cfg.derivative_on_measurement = true;
    yaw_pid.init(yaw_cfg);

#if USE_PHYSICAL_UNITS
    // 物理単位モード: ControlAllocatorを初期化
    // Physical units mode: Initialize ControlAllocator
    allocator.init(stampfly::DEFAULT_QUAD_CONFIG);
    allocator.setMotorParams(stampfly::DEFAULT_MOTOR_PARAMS);
    ESP_LOGI(TAG, "Physical units mode enabled");
    ESP_LOGI(TAG, "  Kp units: [Nm/(rad/s)], Output units: [Nm]");
#else
    ESP_LOGI(TAG, "Voltage scale mode (legacy)");
    ESP_LOGI(TAG, "  Kp units: [V/(rad/s)], Output units: [V]");
#endif

    initialized = true;
    ESP_LOGI(TAG, "RateController initialized");
    ESP_LOGI(TAG, "  Sensitivity: R=%.1f P=%.1f Y=%.1f [rad/s]",
             roll_rate_max, pitch_rate_max, yaw_rate_max);
    ESP_LOGI(TAG, "  Roll  PID: Kp=%.2e Ti=%.3f Td=%.4f Limit=%.2e",
             roll_cfg.Kp, roll_cfg.Ti, roll_cfg.Td, roll_cfg.output_max);
    ESP_LOGI(TAG, "  Pitch PID: Kp=%.2e Ti=%.3f Td=%.4f Limit=%.2e",
             pitch_cfg.Kp, pitch_cfg.Ti, pitch_cfg.Td, pitch_cfg.output_max);
    ESP_LOGI(TAG, "  Yaw   PID: Kp=%.2e Ti=%.3f Td=%.4f Limit=%.2e",
             yaw_cfg.Kp, yaw_cfg.Ti, yaw_cfg.Td, yaw_cfg.output_max);
}

void RateController::reset() {
    roll_pid.reset();
    pitch_pid.reset();
    yaw_pid.reset();
}

// グローバルレートコントローラ（CLIからアクセス可能）
RateController g_rate_controller;

// CLIからアクセスするためのポインタ
RateController* g_rate_controller_ptr = &g_rate_controller;

// =============================================================================
// Attitude Controller Implementation (Outer Loop)
// 姿勢コントローラ実装（外側ループ）
// =============================================================================

void AttitudeController::init() {
    // デフォルト値で初期化
    // Initialize with default values
    max_roll_angle = attitude_control::MAX_ROLL_ANGLE;
    max_pitch_angle = attitude_control::MAX_PITCH_ANGLE;
    yaw_rate_max = rate_control::YAW_RATE_MAX;

    // Roll angle PID (outer loop)
    stampfly::PIDConfig roll_cfg;
    roll_cfg.Kp = attitude_control::ROLL_ANGLE_KP;
    roll_cfg.Ti = attitude_control::ROLL_ANGLE_TI;
    roll_cfg.Td = attitude_control::ROLL_ANGLE_TD;
    roll_cfg.eta = attitude_control::PID_ETA;
    roll_cfg.output_min = -attitude_control::MAX_RATE_SETPOINT;
    roll_cfg.output_max = attitude_control::MAX_RATE_SETPOINT;
    roll_cfg.derivative_on_measurement = true;
    roll_angle_pid.init(roll_cfg);

    // Pitch angle PID (outer loop)
    stampfly::PIDConfig pitch_cfg;
    pitch_cfg.Kp = attitude_control::PITCH_ANGLE_KP;
    pitch_cfg.Ti = attitude_control::PITCH_ANGLE_TI;
    pitch_cfg.Td = attitude_control::PITCH_ANGLE_TD;
    pitch_cfg.eta = attitude_control::PID_ETA;
    pitch_cfg.output_min = -attitude_control::MAX_RATE_SETPOINT;
    pitch_cfg.output_max = attitude_control::MAX_RATE_SETPOINT;
    pitch_cfg.derivative_on_measurement = true;
    pitch_angle_pid.init(pitch_cfg);

    initialized = true;
    ESP_LOGI(TAG, "AttitudeController initialized");
    ESP_LOGI(TAG, "  Max angles: Roll=%.1f° Pitch=%.1f°",
             max_roll_angle * 180.0f / 3.14159f,
             max_pitch_angle * 180.0f / 3.14159f);
    ESP_LOGI(TAG, "  Roll  Angle PID: Kp=%.2f Ti=%.2f Td=%.3f",
             roll_cfg.Kp, roll_cfg.Ti, roll_cfg.Td);
    ESP_LOGI(TAG, "  Pitch Angle PID: Kp=%.2f Ti=%.2f Td=%.3f",
             pitch_cfg.Kp, pitch_cfg.Ti, pitch_cfg.Td);
}

void AttitudeController::reset() {
    roll_angle_pid.reset();
    pitch_angle_pid.reset();
}

void AttitudeController::update(
    float stick_roll, float stick_pitch, float stick_yaw,
    float roll_current, float pitch_current,
    float dt,
    float& roll_rate_ref, float& pitch_rate_ref, float& yaw_rate_ref)
{
    // Convert stick to angle setpoints
    // スティック入力を角度セットポイントに変換
    float roll_angle_target = stick_roll * max_roll_angle;
    float pitch_angle_target = stick_pitch * max_pitch_angle;

    // Outer loop: angle PID → rate setpoint
    // 外側ループ：角度PID → レートセットポイント
    roll_rate_ref = roll_angle_pid.update(roll_angle_target, roll_current, dt);
    pitch_rate_ref = pitch_angle_pid.update(pitch_angle_target, pitch_current, dt);

    // Yaw: direct rate control (same as ACRO)
    // Yaw：直接レート制御（ACROと同じ）
    yaw_rate_ref = stick_yaw * yaw_rate_max;
}

// グローバル姿勢コントローラ
// Global attitude controller
AttitudeController g_attitude_controller;

/**
 * @brief Control Task - 400Hz (2.5ms period)
 *
 * 角速度制御ループ:
 * 1. コントローラ入力から目標角速度を計算
 * 2. IMUから現在の角速度を取得（バイアス補正済み）
 * 3. PID制御で制御出力を計算
 * 4. モーターミキサーで各モーター出力を決定
 *
 * Motor Layout (X-quad configuration, viewed from above)
 *
 *               Front
 *          FL (M4)   FR (M1)
 *             ╲   ▲   ╱
 *              ╲  │  ╱
 *               ╲ │ ╱
 *                ╲│╱
 *                 ╳         ← Center of drone
 *                ╱│╲
 *               ╱ │ ╲
 *              ╱  │  ╲
 *             ╱   │   ╲
 *          RL (M3)    RR (M2)
 *                Rear
 *
 * Motor rotation:
 *   M1 (FR): CCW (Counter-Clockwise)
 *   M2 (RR): CW  (Clockwise)
 *   M3 (RL): CCW (Counter-Clockwise)
 *   M4 (FL): CW  (Clockwise)
 */
void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started (400Hz via semaphore)");

    auto& state = stampfly::StampFlyState::getInstance();

    // コントローラ初期化
    // Initialize controllers
    g_rate_controller.init();
    g_attitude_controller.init();

    // 初期フライトモードをLEDに反映（デフォルト: ACRO=青）
    // Set initial flight mode LED (default: ACRO=blue)
    // Note: led_task will also monitor this, but set initial state here for immediate feedback
    stampfly::LEDManager::getInstance().onFlightModeChanged(
        state.getFlightMode() == stampfly::FlightMode::STABILIZE);

    // 前回のフライト状態（ARMED遷移時にPIDリセット用）
    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;

    while (true) {
        // Wait for control semaphore (given by IMU task after ESKF update)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Get current flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // ARMED遷移時にPIDをリセット（積分項クリア）
        if (flight_state == stampfly::FlightState::ARMED &&
            prev_flight_state != stampfly::FlightState::ARMED) {
            g_rate_controller.reset();
            g_attitude_controller.reset();
            ESP_LOGI(TAG, "PID reset on ARM");
        }
        prev_flight_state = flight_state;

        // Only run control when ARMED or FLYING
        if (flight_state != stampfly::FlightState::ARMED &&
            flight_state != stampfly::FlightState::FLYING) {
            // Ensure motors are stopped when not armed
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RR, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_RL, 0);
            g_motor.setMotor(stampfly::MotorDriver::MOTOR_FL, 0);
            continue;
        }

        // =====================================================================
        // 1. コントローラ入力取得
        // =====================================================================
        // throttle: 0.0 ~ 1.0
        // roll, pitch, yaw: -1.0 ~ +1.0
        float throttle, roll_cmd, pitch_cmd, yaw_cmd;
        state.getControlInput(throttle, roll_cmd, pitch_cmd, yaw_cmd);

        // =====================================================================
        // 2. フライトモード判定 & 目標角速度計算
        // =====================================================================
        // コントローラのMODEビットから取得（デバウンス処理付き）
        // Get from controller MODE bit (with debounce for noise immunity)
        //
        // Controller MODE bit definition:
        //   MODE = 0 (CTRL_FLAG_MODE not set) → ANGLECONTROL (STABILIZE)
        //   MODE = 1 (CTRL_FLAG_MODE set)     → RATECONTROL (ACRO)
        //
        uint8_t ctrl_flags = state.getControlFlags();
        stampfly::FlightMode requested_mode = (ctrl_flags & stampfly::CTRL_FLAG_MODE)
                                    ? stampfly::FlightMode::ACRO
                                    : stampfly::FlightMode::STABILIZE;

        // デバウンス処理：連続N回同じモードを検出したら切替
        // Debounce: switch only after N consecutive same mode readings
        if (requested_mode == g_pending_mode) {
            if (g_mode_switch_counter < MODE_SWITCH_DEBOUNCE_COUNT) {
                g_mode_switch_counter++;
            }
        } else {
            // 異なるモードが来たらカウンタリセット
            g_pending_mode = requested_mode;
            g_mode_switch_counter = 1;
        }

        // デバウンス閾値に達したらモード切替
        // LED更新はled_taskが担当（Disarm中も監視可能）
        if (g_mode_switch_counter >= MODE_SWITCH_DEBOUNCE_COUNT &&
            state.getFlightMode() != g_pending_mode) {
            state.setFlightMode(g_pending_mode);
            g_attitude_controller.reset();

            bool is_stabilize = (g_pending_mode == stampfly::FlightMode::STABILIZE);
            ESP_LOGI(TAG, "Mode changed to %s",
                     is_stabilize ? "STABILIZE" : "ACRO");
        }

        float roll_rate_target, pitch_rate_target, yaw_rate_target;

        if (state.getFlightMode() == stampfly::FlightMode::STABILIZE) {
            // STABILIZE: カスケード制御（姿勢 → レート）
            // STABILIZE: Cascade control (attitude → rate)

            // 現在の姿勢をESKFから取得
            // Get current attitude from ESKF
            float roll_current, pitch_current, yaw_current;
            state.getAttitudeEuler(roll_current, pitch_current, yaw_current);

            // 外側ループ：姿勢制御
            // Outer loop: attitude control
            constexpr float dt = IMU_DT;  // 2.5ms
            g_attitude_controller.update(
                roll_cmd, pitch_cmd, yaw_cmd,
                roll_current, pitch_current,
                dt,
                roll_rate_target, pitch_rate_target, yaw_rate_target
            );
        } else {
            // ACRO: 直接レート制御（既存）
            // ACRO: Direct rate control (existing)
            roll_rate_target = roll_cmd * g_rate_controller.roll_rate_max;
            pitch_rate_target = pitch_cmd * g_rate_controller.pitch_rate_max;
            yaw_rate_target = yaw_cmd * g_rate_controller.yaw_rate_max;
        }

        // =====================================================================
        // 3. 現在の角速度取得（バイアス補正済み）
        // =====================================================================
        stampfly::Vec3 accel, gyro;
        state.getIMUCorrected(accel, gyro);

        // =====================================================================
        // 衝撃・異常検出 - 加速度または角速度が閾値を超えたら自動Disarm
        // =====================================================================
        float accel_magnitude = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        float gyro_magnitude = std::sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);

        bool impact_detected = false;
        const char* impact_reason = nullptr;

        // 加速度チェック
        if (accel_magnitude > safety::IMPACT_ACCEL_THRESHOLD_MS2) {
            g_impact_accel_count++;
            if (g_impact_accel_count >= safety::IMPACT_COUNT_THRESHOLD) {
                impact_detected = true;
                impact_reason = "HIGH_ACCEL";
                ESP_LOGE(TAG, "IMPACT: accel=%.1f m/s^2 (%.1fG)",
                         accel_magnitude, accel_magnitude / 9.81f);
            }
        } else {
            g_impact_accel_count = 0;
        }

        // 角速度チェック
        if (gyro_magnitude > safety::IMPACT_GYRO_THRESHOLD_RPS) {
            g_impact_gyro_count++;
            if (g_impact_gyro_count >= safety::IMPACT_COUNT_THRESHOLD) {
                impact_detected = true;
                impact_reason = "HIGH_GYRO";
                ESP_LOGE(TAG, "IMPACT: gyro=%.1f rad/s (%.0f deg/s)",
                         gyro_magnitude, gyro_magnitude * 180.0f / 3.14159f);
            }
        } else {
            g_impact_gyro_count = 0;
        }

        // 衝撃検出時 → 自動Disarm
        if (impact_detected) {
            ESP_LOGE(TAG, "CRASH DETECTED [%s] - Auto DISARM", impact_reason);

            if (state.requestDisarm()) {
                g_motor.disarm();
                g_buzzer.errorTone();
                stampfly::LEDManager::getInstance().requestChannel(
                    stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CRITICAL_ERROR,
                    stampfly::LEDPattern::BLINK_FAST, 0xFF0000, 5000);  // 赤点滅5秒
                ESP_LOGE(TAG, "Motors DISARMED due to crash");
            }
            g_impact_accel_count = 0;
            g_impact_gyro_count = 0;
            continue;  // このサイクルはスキップ
        }

        float roll_rate_current = gyro.x;   // [rad/s]
        float pitch_rate_current = gyro.y;  // [rad/s]
        float yaw_rate_current = gyro.z;    // [rad/s]

        // デバッグ: ヨー関連の急変検出 → LED表示（1秒間維持）
        static float prev_yaw_cmd = 0.0f;
        static float prev_yaw_gyro = 0.0f;
        float yaw_cmd_diff = std::abs(yaw_cmd - prev_yaw_cmd);
        float yaw_gyro_diff = std::abs(yaw_rate_current - prev_yaw_gyro);
        auto& led_mgr = stampfly::LEDManager::getInstance();
        if (yaw_cmd_diff > 0.3f) {
            // 指令が急変 → 黄色点滅（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::STATUS,
                stampfly::LEDPriority::DEBUG_ALERT,
                stampfly::LEDPattern::BLINK_FAST, 0xFFFF00, 1000);
            g_yaw_alert_counter = 400;
        }
        if (yaw_gyro_diff > 1.0f) {
            // ジャイロが急変 → 黄色点灯（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::STATUS,
                stampfly::LEDPriority::DEBUG_ALERT,
                stampfly::LEDPattern::SOLID, 0xFFFF00, 1000);
            g_yaw_alert_counter = 400;
        }
        if (g_yaw_alert_counter > 0) {
            g_yaw_alert_counter--;
        }
        prev_yaw_cmd = yaw_cmd;
        prev_yaw_gyro = yaw_rate_current;

        // =====================================================================
        // 4. PID制御
        // =====================================================================
        constexpr float dt = IMU_DT;  // 2.5ms

        float roll_out = g_rate_controller.roll_pid.update(
            roll_rate_target, roll_rate_current, dt);
        float pitch_out = g_rate_controller.pitch_pid.update(
            pitch_rate_target, pitch_rate_current, dt);
        float yaw_out = g_rate_controller.yaw_pid.update(
            yaw_rate_target, yaw_rate_current, dt);

        // =====================================================================
        // 5. モーターミキサー / Control Allocation
        // =====================================================================
#if USE_PHYSICAL_UNITS
        // 物理単位モード: PID出力 [Nm] → ControlAllocator → モータDuty
        // Physical units mode: PID output [Nm] -> ControlAllocator -> Motor Duty

        // スロットル → 総推力 [N] (4モータ合計)
        // Throttle (0-1) -> Total thrust [N] (sum of 4 motors)
        // ホバー推力 0.343N (35g × 9.81) でthrottle=0.5程度を想定
        constexpr float MAX_TOTAL_THRUST = 4.0f * 0.15f;  // 4 × max_thrust_per_motor
        float total_thrust = throttle * MAX_TOTAL_THRUST;

        // 制御入力ベクトル: [総推力, ロールトルク, ピッチトルク, ヨートルク]
        // Control input vector: [total_thrust, roll_torque, pitch_torque, yaw_torque]
        float control[4] = {total_thrust, roll_out, pitch_out, yaw_out};

        // ミキシング: 制御入力 → モータ推力 [N]
        // Mixing: Control inputs -> Motor thrusts [N]
        float thrusts[4];
        g_rate_controller.allocator.mix(control, thrusts);

        // 推力 → Duty変換
        // Thrust -> Duty conversion
        float duties[4];
        g_rate_controller.allocator.thrustsToDuties(thrusts, duties);

        // モータ出力設定
        // Set motor outputs
        g_motor.setMotorDuties(duties);

#else
        // 電圧スケールモード（レガシー）: PID出力 [V] → ミキサー → モータDuty
        // Voltage scale mode (legacy): PID output [V] -> Mixer -> Motor Duty
        // X-quad mixer: setMixerOutput handles the motor mixing
        // thrust: 0.0 ~ 1.0
        // roll/pitch/yaw: ±3.7V (already limited by PID output limits)
        g_motor.setMixerOutput(throttle, roll_out, pitch_out, yaw_out);
#endif
    }
}
