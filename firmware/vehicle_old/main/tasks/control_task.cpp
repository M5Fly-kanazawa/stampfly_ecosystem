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
#include "../altitude_controller.hpp"
#include "../position_controller.hpp"
#include "control_allocation.hpp"
#include "motor_model.hpp"
#include "controller_comm.hpp"  // for CTRL_FLAG_MODE, CTRL_FLAG_ALT_MODE
#include "led_manager.hpp"      // for flight mode LED indication
#include "flight_command.hpp"   // for high-level flight commands
#include "command_queue.hpp"    // for command queue processing
#include "control_arbiter.hpp"  // for multi-source control input arbitration
#include "sensor_fusion.hpp"    // for g_fusion state access (altitude/velocity)
#include "filter.hpp"           // for NotchFilter
#include <algorithm>            // for std::clamp

// Trim values (defined in cli.cpp)
// トリム値（cli.cppで定義）
extern float g_trim_roll;
extern float g_trim_pitch;
extern float g_trim_yaw;

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

// グローバル高度コントローラ
// Global altitude controller
AltitudeController g_altitude_controller;

// グローバル位置コントローラ
// Global position controller
PositionController g_position_controller;

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
    g_altitude_controller.init();
    g_position_controller.init();

    // Gyro notch filter initialization
    // ジャイロノッチフィルタ初期化
    stampfly::NotchFilter gyro_notch[3];  // Roll, Pitch, Yaw
    if (config::gyro_notch::ENABLED) {
        constexpr float fs = 1.0f / IMU_DT;  // 400Hz
        for (int i = 0; i < 3; i++) {
            gyro_notch[i].init(fs, config::gyro_notch::CENTER_FREQ_HZ,
                               config::gyro_notch::Q);
        }
        ESP_LOGI(TAG, "Gyro notch filter: %.0fHz Q=%.1f",
                 config::gyro_notch::CENTER_FREQ_HZ, config::gyro_notch::Q);
    }

    // 初期フライトモードをLEDに反映（デフォルト: ACRO=青）
    // Set initial flight mode LED (default: ACRO=blue)
    // Note: led_task will also monitor this, but set initial state here for immediate feedback
    stampfly::LEDManager::getInstance().onFlightModeChanged(state.getFlightMode());

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
            g_altitude_controller.reset();
            g_position_controller.reset();

            // ALTITUDE_HOLD/POSITION_HOLDモードでARM → 現在高度をキャプチャ
            // Capture altitude when ARMing in ALTITUDE_HOLD or POSITION_HOLD mode
            stampfly::FlightMode arm_mode = state.getFlightMode();
            if (arm_mode == stampfly::FlightMode::ALTITUDE_HOLD ||
                arm_mode == stampfly::FlightMode::POSITION_HOLD) {
                auto fused_state = g_fusion.getState();
                float alt = -fused_state.position.z;
                g_altitude_controller.captureAltitude(alt);
                ESP_LOGI(TAG, "ALT capture on ARM: alt=%.2fm", alt);

                // POSITION_HOLD: also capture horizontal position
                // POSITION_HOLD: 水平位置もキャプチャ
                if (arm_mode == stampfly::FlightMode::POSITION_HOLD) {
                    g_position_controller.capturePosition(
                        fused_state.position.x, fused_state.position.y);
                    ESP_LOGI(TAG, "POS capture on ARM: x=%.2f y=%.2f",
                             fused_state.position.x, fused_state.position.y);
                }
            }

            ESP_LOGI(TAG, "PID reset on ARM");
        }

        // =====================================================================
        // モータードライバーARMの同期（WiFiコマンド対応）
        // Sync MotorDriver ARM state (for WiFi command support)
        // =====================================================================
        // StampFlyState が ARMED に遷移したら MotorDriver も ARM する
        // WiFi コマンドでの自動ARM に対応するため
        if (prev_flight_state != stampfly::FlightState::ARMED &&
            flight_state == stampfly::FlightState::ARMED) {
            if (!g_motor.isArmed()) {
                g_motor.arm();  // Enable motor driver
                ESP_LOGI(TAG, "MotorDriver auto-armed (WiFi command or controller)");
            }
        }
        // DISARM に遷移したら MotorDriver も DISARM + 全状態リセット
        // On DISARM: stop motors, reset controllers AND ESKF position/velocity
        // This prevents stale pos_z from being captured on immediate re-ARM.
        // Without this, DISARM at height → fall → re-ARM captures drifted pos_z
        // as altitude setpoint → drone shoots up to wrong altitude.
        else if ((prev_flight_state == stampfly::FlightState::ARMED ||
                  prev_flight_state == stampfly::FlightState::FLYING) &&
                 flight_state != stampfly::FlightState::ARMED &&
                 flight_state != stampfly::FlightState::FLYING) {
            if (g_motor.isArmed()) {
                g_motor.disarm();
                ESP_LOGI(TAG, "MotorDriver auto-disarmed");
            }
            g_altitude_controller.reset();
            g_position_controller.reset();
            g_fusion.resetForLanding();  // pos/vel=0 + freeze BA + P cleanup
            ESP_LOGI(TAG, "DISARM: controllers + ESKF reset");
        }

        prev_flight_state = flight_state;

        // =====================================================================
        // バッテリー電圧によるリアルタイムVbat更新（全モード共通）
        // Real-time Vbat update for accurate duty calculation (all modes)
        // =====================================================================
        {
            static int vbat_update_counter = 0;
            if (++vbat_update_counter >= 40) {  // 10Hz (400Hz / 40)
                vbat_update_counter = 0;
                float measured_vbat = state.getVoltage();
                if (measured_vbat > 2.5f && measured_vbat < 4.5f) {
                    stampfly::MotorParams params = g_rate_controller.allocator.getMotorParams();
                    params.Vbat = measured_vbat;
                    g_rate_controller.allocator.setMotorParams(params);
                }
            }
        }

        // =====================================================================
        // フライトモード判定（Disarm中も実行 - LED更新のため）
        // Flight mode detection (runs even when disarmed - for LED update)
        // =====================================================================
        // Controller flag definitions:
        //   CTRL_FLAG_POS_MODE set             → POSITION_HOLD
        //   CTRL_FLAG_ALT_MODE set (no POS)    → ALTITUDE_HOLD
        //   CTRL_FLAG_MODE set (no ALT/POS)    → ACRO
        //   Neither set                        → STABILIZE
        {
            uint8_t ctrl_flags = state.getControlFlags();
            stampfly::FlightMode requested_mode;
            if (ctrl_flags & stampfly::CTRL_FLAG_POS_MODE) {
                requested_mode = stampfly::FlightMode::POSITION_HOLD;
            } else if (ctrl_flags & stampfly::CTRL_FLAG_ALT_MODE) {
                requested_mode = stampfly::FlightMode::ALTITUDE_HOLD;
            } else if (ctrl_flags & stampfly::CTRL_FLAG_MODE) {
                requested_mode = stampfly::FlightMode::ACRO;
            } else {
                requested_mode = stampfly::FlightMode::STABILIZE;
            }

            // Sensor availability guard: downgrade to STABILIZE if required
            // sensors are disabled in ESKF config.
            // センサ可用性ガード: 必要なセンサが無効なら STABILIZE に降格
            {
                const auto& eskf = g_fusion.getESKF();
                bool has_altitude = eskf.isSensorEnabled(stampfly::ESKF::SENSOR_TOF) ||
                                    eskf.isSensorEnabled(stampfly::ESKF::SENSOR_BARO);
                bool has_position = eskf.isSensorEnabled(stampfly::ESKF::SENSOR_FLOW);

                if (requested_mode == stampfly::FlightMode::POSITION_HOLD &&
                    (!has_altitude || !has_position)) {
                    requested_mode = stampfly::FlightMode::STABILIZE;
                } else if (requested_mode == stampfly::FlightMode::ALTITUDE_HOLD &&
                           !has_altitude) {
                    requested_mode = stampfly::FlightMode::STABILIZE;
                }
            }

            // デバウンス処理：連続N回同じモードを検出したら切替
            if (requested_mode == g_pending_mode) {
                if (g_mode_switch_counter < MODE_SWITCH_DEBOUNCE_COUNT) {
                    g_mode_switch_counter++;
                }
            } else {
                g_pending_mode = requested_mode;
                g_mode_switch_counter = 1;
            }

            // デバウンス閾値に達したらモード切替
            if (g_mode_switch_counter >= MODE_SWITCH_DEBOUNCE_COUNT &&
                state.getFlightMode() != g_pending_mode) {
                stampfly::FlightMode prev_mode = state.getFlightMode();
                state.setFlightMode(g_pending_mode);
                g_attitude_controller.reset();

                // ALTITUDE_HOLD/POSITION_HOLD突入時: 現在高度をキャプチャ
                // Capture current altitude when entering ALTITUDE_HOLD or POSITION_HOLD
                if (g_pending_mode == stampfly::FlightMode::ALTITUDE_HOLD ||
                    g_pending_mode == stampfly::FlightMode::POSITION_HOLD) {
                    auto fused_state = g_fusion.getState();
                    float alt = -fused_state.position.z;  // NED -> altitude (positive up)
                    g_altitude_controller.captureAltitude(alt);
                    ESP_LOGI(TAG, "ALT capture: alt=%.2fm", alt);

                    // POSITION_HOLD突入時: 水平位置もキャプチャ
                    // Also capture horizontal position when entering POSITION_HOLD
                    if (g_pending_mode == stampfly::FlightMode::POSITION_HOLD) {
                        g_position_controller.capturePosition(
                            fused_state.position.x, fused_state.position.y);
                        ESP_LOGI(TAG, "POS capture: x=%.2f y=%.2f",
                                 fused_state.position.x, fused_state.position.y);
                    }
                }

                // POSITION_HOLD離脱時: 位置PIDリセット
                // Reset position PID when leaving POSITION_HOLD
                if (prev_mode == stampfly::FlightMode::POSITION_HOLD) {
                    g_position_controller.reset();
                    ESP_LOGI(TAG, "POSITION_HOLD: PID reset on exit");
                }

                // ALTITUDE_HOLD/POSITION_HOLD離脱時: 高度PIDリセット
                // Reset altitude PID when leaving altitude-holding modes
                if ((prev_mode == stampfly::FlightMode::ALTITUDE_HOLD ||
                     prev_mode == stampfly::FlightMode::POSITION_HOLD) &&
                    g_pending_mode != stampfly::FlightMode::ALTITUDE_HOLD &&
                    g_pending_mode != stampfly::FlightMode::POSITION_HOLD) {
                    g_altitude_controller.reset();
                    ESP_LOGI(TAG, "Altitude PID reset on exit");
                }

                const char* mode_name = "ACRO";
                if (g_pending_mode == stampfly::FlightMode::STABILIZE) mode_name = "STABILIZE";
                else if (g_pending_mode == stampfly::FlightMode::ALTITUDE_HOLD) mode_name = "ALTITUDE_HOLD";
                else if (g_pending_mode == stampfly::FlightMode::POSITION_HOLD) mode_name = "POSITION_HOLD";
                ESP_LOGI(TAG, "Mode changed to %s", mode_name);
            }
        }

        // =====================================================================
        // Command Queue Processing (10Hz)
        // コマンドキュー処理（10Hz）
        // =====================================================================
        // Process command queue every 100ms (40 cycles @ 400Hz)
        // コマンドキューを100msごとに処理（400Hzで40サイクルごと）
        static int queue_process_counter = 0;
        if (++queue_process_counter >= 40) {
            bool command_started = stampfly::CommandQueue::getInstance().processQueue();
            if (command_started) {
                ESP_LOGI(TAG, "Command started from queue");
            }
            queue_process_counter = 0;
        }

        // =====================================================================
        // High-Level Flight Commands Update
        // 高レベル飛行コマンド更新
        // =====================================================================
        // Update flight command service (runs at 400Hz)
        // フライトコマンドサービスを更新（400Hzで実行）
        stampfly::FlightCommandService::getInstance().update(IMU_DT);

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
        // 1. コントローラ入力取得（ControlArbiter経由）
        // Get control input from ControlArbiter (supports ESP-NOW, UDP, WebSocket)
        // =====================================================================
        // throttle: 0.0 ~ 1.0
        // roll, pitch, yaw: -1.0 ~ +1.0
        stampfly::ControlInput ctrl_input;
        float throttle, roll_cmd, pitch_cmd, yaw_cmd;

        if (stampfly::ControlArbiter::getInstance().getActiveControl(ctrl_input)) {
            // Active control input available from arbiter
            // アクティブな制御入力がアービターから取得できた
            throttle = ctrl_input.throttle;
            roll_cmd = ctrl_input.roll;
            pitch_cmd = ctrl_input.pitch;
            yaw_cmd = ctrl_input.yaw;

            // Debug log every 100 cycles (~250ms @ 400Hz) when throttle > 0
            static int ctrl_log_counter = 0;
            if (throttle > 0.01f && ++ctrl_log_counter >= 100) {
                ESP_LOGI(TAG, "← ControlArbiter: T=%.2f R=%.2f P=%.2f Y=%.2f (src=%d)",
                         throttle, roll_cmd, pitch_cmd, yaw_cmd, static_cast<int>(ctrl_input.source));
                ctrl_log_counter = 0;
            }
        } else {
            // No active control input (timeout or no source)
            // アクティブな制御入力なし（タイムアウトまたはソースなし）
            throttle = 0.0f;
            roll_cmd = 0.0f;
            pitch_cmd = 0.0f;
            yaw_cmd = 0.0f;

            // Log timeout every 400 cycles (~1s)
            static int timeout_log_counter = 0;
            if (++timeout_log_counter >= 400) {
                ESP_LOGW(TAG, "No active control input (timeout or no source)");
                timeout_log_counter = 0;
            }
        }

        // =====================================================================
        // 2. 目標角速度計算
        // =====================================================================
        float roll_rate_target, pitch_rate_target, yaw_rate_target;
        float angle_ref_roll = 0.0f, angle_ref_pitch = 0.0f;

        stampfly::FlightMode current_mode = state.getFlightMode();

        if (current_mode == stampfly::FlightMode::STABILIZE ||
            current_mode == stampfly::FlightMode::ALTITUDE_HOLD ||
            current_mode == stampfly::FlightMode::POSITION_HOLD) {
            // STABILIZE / ALTITUDE_HOLD / POSITION_HOLD: カスケード制御（姿勢 → レート）
            // Cascade control (attitude -> rate)

            // 現在の姿勢をESKFから取得
            // Get current attitude from ESKF
            float roll_current, pitch_current, yaw_current;
            state.getAttitudeEuler(roll_current, pitch_current, yaw_current);

            float roll_input, pitch_input;
            float yaw_trimmed = std::clamp(yaw_cmd + g_trim_yaw, -1.0f, 1.0f);

            if (current_mode == stampfly::FlightMode::POSITION_HOLD &&
                g_position_controller.position_captured) {
                // POSITION_HOLD: 位置制御からroll/pitch角度を取得
                // Get roll/pitch angles from position controller

                // Flow quality check for fallback
                // フロー品質チェック（フォールバック用）
                int16_t flow_dx, flow_dy;
                uint8_t flow_squal;
                state.getFlowRawData(flow_dx, flow_dy, flow_squal);
                float pos_variance = g_fusion.getPositionVariance();

                if (flow_squal < config::FLOW_SQUAL_MIN || pos_variance > 1.0f) {
                    // Flow quality degraded or ESKF position uncertain
                    // フロー品質低下またはESKF位置推定不確実 → 位置PIDリセット
                    g_position_controller.reset();

                    // Fallback to stick+trim (ALTITUDE_HOLD equivalent)
                    // ALTITUDE_HOLD相当にフォールバック
                    roll_input = std::clamp(roll_cmd + g_trim_roll, -1.0f, 1.0f);
                    pitch_input = std::clamp(pitch_cmd + g_trim_pitch, -1.0f, 1.0f);

                    static int fallback_log_counter = 0;
                    if (++fallback_log_counter >= 400) {
                        ESP_LOGW(TAG, "POS_HOLD fallback: squal=%u var=%.2f",
                                 flow_squal, pos_variance);
                        fallback_log_counter = 0;
                    }
                } else {
                    // Position control active
                    // 位置制御アクティブ
                    auto fused_state = g_fusion.getState();

                    // Stick → velocity command (body frame)
                    // スティック → 速度指令（body frame）
                    float vel_cmd_body_x = PositionController::stickToVelocity(pitch_cmd);
                    float vel_cmd_body_y = PositionController::stickToVelocity(roll_cmd);

                    // Position controller update → roll/pitch angle [rad]
                    // 位置コントローラ更新 → roll/pitch角度 [rad]
                    float pos_roll_angle, pos_pitch_angle;
                    constexpr float dt = IMU_DT;
                    g_position_controller.update(
                        vel_cmd_body_x, vel_cmd_body_y,
                        fused_state.position.x, fused_state.position.y,
                        fused_state.velocity.x, fused_state.velocity.y,
                        yaw_current, dt,
                        pos_roll_angle, pos_pitch_angle);

                    // Convert angle [rad] to normalized input [-1, 1] for AttitudeController
                    // 角度 [rad] をAttitudeControllerの正規化入力に変換
                    roll_input = pos_roll_angle / g_attitude_controller.max_roll_angle;
                    pitch_input = pos_pitch_angle / g_attitude_controller.max_pitch_angle;
                    roll_input = std::clamp(roll_input, -1.0f, 1.0f);
                    pitch_input = std::clamp(pitch_input, -1.0f, 1.0f);

                    // Debug log every 400 cycles (~1s @ 400Hz)
                    static int pos_log_counter = 0;
                    if (++pos_log_counter >= 400) {
                        ESP_LOGI(TAG, "POS_HOLD: sp=(%.2f,%.2f) pos=(%.2f,%.2f) r=%.1f° p=%.1f°",
                                 g_position_controller.pos_setpoint_x,
                                 g_position_controller.pos_setpoint_y,
                                 fused_state.position.x, fused_state.position.y,
                                 pos_roll_angle * 180.0f / 3.14159f,
                                 pos_pitch_angle * 180.0f / 3.14159f);
                        pos_log_counter = 0;
                    }
                }
            } else {
                // STABILIZE / ALTITUDE_HOLD: スティック + トリム
                // Stick + trim for non-position modes
                roll_input = std::clamp(roll_cmd + g_trim_roll, -1.0f, 1.0f);
                pitch_input = std::clamp(pitch_cmd + g_trim_pitch, -1.0f, 1.0f);
            }

            // 外側ループ：姿勢制御
            // Outer loop: attitude control
            constexpr float dt = IMU_DT;  // 2.5ms
            g_attitude_controller.update(
                roll_input, pitch_input, yaw_trimmed,
                roll_current, pitch_current,
                dt,
                roll_rate_target, pitch_rate_target, yaw_rate_target
            );

            // Record outer loop angle targets for telemetry
            // テレメトリ用に外側ループの角度目標を記録
            angle_ref_roll = roll_input * g_attitude_controller.max_roll_angle;
            angle_ref_pitch = pitch_input * g_attitude_controller.max_pitch_angle;
        } else {
            // ACRO: 直接レート制御（既存）
            // ACRO: Direct rate control (existing)
            roll_rate_target = roll_cmd * g_rate_controller.roll_rate_max;
            pitch_rate_target = pitch_cmd * g_rate_controller.pitch_rate_max;
            yaw_rate_target = yaw_cmd * g_rate_controller.yaw_rate_max;
        }

        // Publish control loop reference values for telemetry
        // テレメトリ用に制御ループ目標値を公開
        state.updateControlRef(angle_ref_roll, angle_ref_pitch,
                               roll_rate_target, pitch_rate_target, yaw_rate_target);

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
                g_motor.saveStatsToNVS();  // モーター統計保存
                g_motor.disarm();
                g_buzzer.errorTone();
                stampfly::LEDManager::getInstance().requestChannel(
                    stampfly::LEDChannel::BODY, stampfly::LEDPriority::CRITICAL_ERROR,
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

        // Apply notch filter to gyro feedback (before PID)
        // ノッチフィルタをジャイロフィードバックに適用（PID前）
        if (config::gyro_notch::ENABLED) {
            roll_rate_current = gyro_notch[0].apply(roll_rate_current);
            pitch_rate_current = gyro_notch[1].apply(pitch_rate_current);
            yaw_rate_current = gyro_notch[2].apply(yaw_rate_current);
        }

        // デバッグ: ヨー関連の急変検出 → LED表示（1秒間維持）
        static float prev_yaw_cmd = 0.0f;
        static float prev_yaw_gyro = 0.0f;
        float yaw_cmd_diff = std::abs(yaw_cmd - prev_yaw_cmd);
        float yaw_gyro_diff = std::abs(yaw_rate_current - prev_yaw_gyro);
        auto& led_mgr = stampfly::LEDManager::getInstance();
        if (yaw_cmd_diff > 0.3f) {
            // 指令が急変 → 黄色点滅（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::BODY,
                stampfly::LEDPriority::DEBUG_ALERT,
                stampfly::LEDPattern::BLINK_FAST, 0xFFFF00, 1000);
            g_yaw_alert_counter = 400;
        }
        if (yaw_gyro_diff > 1.0f) {
            // ジャイロが急変 → 黄色点灯（1秒間）- DEBUG_ALERT優先度、タイムアウト付き
            led_mgr.requestChannel(stampfly::LEDChannel::BODY,
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
        // Throttle -> Total thrust [N] (sum of 4 motors)
        constexpr float MAX_TOTAL_THRUST = 4.0f * 0.168f;  // 4 × max_thrust_per_motor (duty≤0.95)
        float total_thrust;

        if ((current_mode == stampfly::FlightMode::ALTITUDE_HOLD ||
             current_mode == stampfly::FlightMode::POSITION_HOLD) &&
            g_altitude_controller.altitude_captured) {
            // 高度制御: 閉ループスロットル
            // Altitude hold: closed-loop throttle
            auto fused_state = g_fusion.getState();
            float alt = -fused_state.position.z;       // NED -> altitude (positive up)
            float vel_z = -fused_state.velocity.z;     // NED -> velocity (positive up)

            uint16_t raw_throttle, raw_r, raw_p, raw_y;
            state.getRawControlInput(raw_throttle, raw_r, raw_p, raw_y);
            float climb_cmd = g_altitude_controller.stickToClimbRate(raw_throttle);

            float vbat = state.getVoltage();
            total_thrust = g_altitude_controller.update(climb_cmd, alt, vel_z, vbat, dt);

            // Debug log every 200 cycles (~500ms @ 400Hz)
            static int alt_log_counter = 0;
            if (++alt_log_counter >= 200) {
                ESP_LOGI(TAG, "ALT_HOLD: sp=%.2fm alt=%.2fm vz=%.2fm/s thrust=%.3fN hover=%.3fN vbat=%.2fV",
                         g_altitude_controller.altitude_setpoint, alt, vel_z,
                         total_thrust, g_altitude_controller.getHoverThrust(), vbat);
                alt_log_counter = 0;
            }
        } else {
            // 既存: オープンループスロットル
            // Existing: open-loop throttle
            // ホバー推力 0.343N (35g × 9.81) でthrottle=0.5程度を想定
            total_thrust = throttle * MAX_TOTAL_THRUST;
        }

        // 制御入力ベクトル: [総推力, ロールトルク, ピッチトルク, ヨートルク]
        // Control input vector: [total_thrust, roll_torque, pitch_torque, yaw_torque]
        float control[4] = {total_thrust, roll_out, pitch_out, yaw_out};
        state.setTotalThrust(total_thrust);

        // Update battery voltage for thrust→duty conversion (LPF, τ≈5s)
        // バッテリー電圧を更新（LPF適用、時定数約5秒）
        // Slow filter prevents positive feedback: motor current drop → Vbat drop
        // → duty increase → more current → oscillation
        {
            static float vbat_filtered = 3.7f;
            static bool vbat_initialized = false;
            float vbat_raw = state.getVoltage();
            if (!vbat_initialized && vbat_raw > 2.0f) {
                vbat_filtered = vbat_raw;
                vbat_initialized = true;
            }
            constexpr float VBAT_LPF_TAU = 5.0f;  // [s] time constant
            float alpha = dt / (VBAT_LPF_TAU + dt);
            vbat_filtered += alpha * (vbat_raw - vbat_filtered);
            g_rate_controller.allocator.setVbat(vbat_filtered);
        }

        // ミキシング: 制御入力 → モータ推力 [N]
        // Mixing: Control inputs -> Motor thrusts [N]
        float thrusts[4];
        g_rate_controller.allocator.mix(control, thrusts);

        // 推力 → Duty変換
        // Thrust -> Duty conversion
        float duties[4];
        g_rate_controller.allocator.thrustsToDuties(thrusts, duties);

        // Debug log every 100 cycles (~250ms @ 400Hz) when throttle > 0
        static int motor_log_counter = 0;
        if (total_thrust > 0.01f && ++motor_log_counter >= 100) {
            ESP_LOGI(TAG, "MOTOR: thrust=%.3fN -> duties[FR=%.2f RR=%.2f RL=%.2f FL=%.2f]",
                     total_thrust, duties[0], duties[1], duties[2], duties[3]);
            motor_log_counter = 0;
        }

        // モータ出力設定
        // Set motor outputs
        state.setMotorDuties(duties);
        g_motor.setMotorDuties(duties);

#else
        // 電圧スケールモード（レガシー）: PID出力 [V] → ミキサー → モータDuty
        // Voltage scale mode (legacy): PID output [V] -> Mixer -> Motor Duty
        // X-quad mixer: setMixerOutput handles the motor mixing
        // thrust: 0.0 ~ 1.0
        // roll/pitch/yaw: ±3.7V (already limited by PID output limits)

        // Debug log every 100 cycles (~250ms @ 400Hz) when throttle > 0
        static int motor_log_counter_legacy = 0;
        if (throttle > 0.01f && ++motor_log_counter_legacy >= 100) {
            ESP_LOGI(TAG, "MOTOR (legacy): throttle=%.2f roll=%.2f pitch=%.2f yaw=%.2f",
                     throttle, roll_out, pitch_out, yaw_out);
            motor_log_counter_legacy = 0;
        }

        g_motor.setMixerOutput(throttle, roll_out, pitch_out, yaw_out);
#endif
    }
}
