/**
 * @file altitude_controller.hpp
 * @brief Altitude Hold Controller with Hover Thrust Estimation
 *
 * 高度維持コントローラ（ホバー推力推定器付き）
 *
 * Control structure:
 *   stick -> deadzone -> climb_rate_cmd
 *                            |
 *   ESKF altitude -> [Alt PID] -> vel_target -> [Vel PID] -> thrust_correction
 *                     (outer)                    (inner)           |
 *                                                                  v
 *                 HoverEstimator(vbat) -> hover_thrust + correction = total_thrust
 *
 * The hover thrust estimator is swappable via function pointer,
 * allowing easy experimentation with different models.
 * ホバー推力推定式は関数ポインタで交換可能
 */

#pragma once

#include "pid.hpp"
#include "motor_model.hpp"
#include "config.hpp"
#include <algorithm>

// =====================================================================
// Hover Thrust Estimation Functions (swappable)
// ホバー推力推定関数（交換可能）
// =====================================================================

/// Type for hover thrust estimation function
/// ホバー推力推定関数の型
/// @param vbat Battery voltage [V]
/// @param mass Vehicle mass [kg]
/// @param correction Empirical correction factor
/// @return Estimated hover thrust [N]
using HoverThrustFn = float (*)(float vbat, float mass, float correction);

/// Default: constant model (voltage-independent)
/// デフォルト: 定数モデル（電圧非依存）
/// hover_thrust = mass * g * correction
inline float hoverThrustConstant(float vbat, float mass, float correction) {
    (void)vbat;
    return mass * 9.81f * correction;
}

/// Model-based: voltage-dependent (for future extension)
/// モデルベース: 電圧依存（将来拡張用）
inline float hoverThrustVoltageCorrected(float vbat, float mass, float correction) {
    float base = mass * 9.81f * correction;
    // Future: adjust based on experimental voltage-efficiency curve
    // 将来: 実験データから電圧-効率曲線で調整
    float voltage_factor = 1.0f;
    (void)vbat;  // Placeholder until experimental data is available
    return base * voltage_factor;
}

// =====================================================================
// Altitude Controller
// 高度コントローラ
// =====================================================================

struct HoverEstimatorConfig {
    float mass = config::altitude_control::MASS;
    float correction = config::altitude_control::HOVER_THRUST_CORRECTION;
    HoverThrustFn fn = hoverThrustConstant;
};

struct AltitudeController {
    stampfly::PID altitude_pid;     // Outer loop: altitude -> velocity target
    stampfly::PID velocity_pid;     // Inner loop: velocity -> thrust correction

    float altitude_setpoint = 0.0f; // Target altitude [m]
    bool altitude_captured = false;  // Altitude captured flag
    bool initialized = false;
    bool stick_unlocked_ = false;   // True after stick returns to center deadzone

    // Hover thrust estimation (swappable)
    // ホバー推力推定（交換可能）
    HoverEstimatorConfig hover_config;
    float current_vbat = 3.7f;      // Latest battery voltage [V]

    /**
     * @brief Initialize altitude controller with default parameters
     * デフォルトパラメータで高度コントローラを初期化
     */
    void init() {
        using namespace config::altitude_control;

        // Altitude PID (outer loop): altitude error [m] -> velocity target [m/s]
        // 高度PID（外ループ）: 高度誤差 → 速度目標
        stampfly::PIDConfig alt_cfg;
        alt_cfg.Kp = ALT_KP;
        alt_cfg.Ti = ALT_TI;
        alt_cfg.Td = ALT_TD;
        alt_cfg.eta = PID_ETA;
        alt_cfg.output_min = -ALT_OUTPUT_MAX;
        alt_cfg.output_max = ALT_OUTPUT_MAX;
        alt_cfg.derivative_on_measurement = true;
        altitude_pid.init(alt_cfg);

        // Velocity PID (inner loop): velocity error [m/s] -> thrust correction [N]
        // 速度PID（内ループ）: 速度誤差 → 推力補正
        stampfly::PIDConfig vel_cfg;
        vel_cfg.Kp = VEL_KP;
        vel_cfg.Ti = VEL_TI;
        vel_cfg.Td = VEL_TD;
        vel_cfg.eta = PID_ETA;
        vel_cfg.output_min = -VEL_OUTPUT_MAX;
        vel_cfg.output_max = VEL_OUTPUT_MAX;
        vel_cfg.derivative_on_measurement = true;
        velocity_pid.init(vel_cfg);

        initialized = true;
    }

    /**
     * @brief Reset PID states and altitude capture
     * PID内部状態と高度キャプチャをリセット
     */
    void reset() {
        altitude_pid.reset();
        velocity_pid.reset();
        altitude_captured = false;
        altitude_setpoint = 0.0f;
        stick_unlocked_ = false;
    }

    /**
     * @brief Capture current altitude as setpoint
     * 現在の高度をセットポイントとしてキャプチャ
     * @param alt Current altitude [m] (positive up)
     */
    void captureAltitude(float alt) {
        using namespace config::altitude_control;
        altitude_setpoint = std::clamp(alt, MIN_ALTITUDE, MAX_ALTITUDE);
        altitude_captured = true;
        stick_unlocked_ = false;
    }

    /// Swap the hover thrust estimation formula
    /// ホバー推力推定式を交換する
    void setHoverEstimator(HoverThrustFn fn) { hover_config.fn = fn; }

    /// Adjust hover thrust parameters
    /// ホバー推力パラメータを調整する
    void setHoverParams(float mass, float correction) {
        hover_config.mass = mass;
        hover_config.correction = correction;
    }

    /// Get current hover thrust (for debug/logging)
    /// 現在のホバー推力を取得（デバッグ・ログ用）
    float getHoverThrust() const {
        return hover_config.fn(current_vbat, hover_config.mass, hover_config.correction);
    }

    /**
     * @brief Main update: returns total thrust [N]
     * メイン更新関数: 推力 [N] を返す
     *
     * @param climb_rate_cmd Commanded climb rate [m/s] (positive up, 0 = hold)
     * @param altitude Current altitude [m] (positive up)
     * @param velocity_z Current vertical velocity [m/s] (positive up)
     * @param vbat Current battery voltage [V]
     * @param dt Control period [s]
     * @return Total thrust [N], clamped to [0, MAX_TOTAL_THRUST]
     */
    float update(float climb_rate_cmd, float altitude, float velocity_z,
                 float vbat, float dt) {
        using namespace config::altitude_control;

        current_vbat = vbat;

        // Update altitude setpoint from stick command
        // スティックコマンドからセットポイントを更新
        if (climb_rate_cmd != 0.0f) {
            altitude_setpoint += climb_rate_cmd * dt;
            altitude_setpoint = std::clamp(altitude_setpoint, MIN_ALTITUDE, MAX_ALTITUDE);
        }

        // Outer loop: altitude PID -> velocity target
        // 外ループ: 高度PID → 速度目標
        float velocity_target = altitude_pid.update(altitude_setpoint, altitude, dt);

        // Inner loop: velocity PID -> thrust correction
        // 内ループ: 速度PID → 推力補正
        float thrust_correction = velocity_pid.update(velocity_target, velocity_z, dt);

        // Feedforward: hover thrust
        // フィードフォワード: ホバー推力
        float hover_thrust = hover_config.fn(vbat, hover_config.mass, hover_config.correction);

        // Total thrust = hover + correction
        float total_thrust = hover_thrust + thrust_correction;
        total_thrust = std::clamp(total_thrust, 0.0f, MAX_TOTAL_THRUST);

        return total_thrust;
    }

    /**
     * @brief Convert throttle stick to climb/descent rate
     * スロットルスティックを上昇/下降速度に変換
     *
     * Stick center (2048) = hold altitude.
     * モード遷移直後はスティックがホバー位置（~60%）にあるため、
     * スティックがセンター付近のデッドゾーンに戻るまでコマンドをロック。
     * バネ復帰式スティックなら離すだけでロック解除される。
     *
     * @param raw_throttle Raw ADC value (0-4095, center=2048)
     * @return Climb rate [m/s] (positive=up, negative=down, 0=hold)
     */
    float stickToClimbRate(uint16_t raw_throttle) {
        using namespace config::altitude_control;

        // Normalize to [-1, 1] around center (2048)
        // 中央(2048)を基準に [-1, 1] に正規化
        float normalized = (static_cast<float>(raw_throttle) - 2048.0f) / 2048.0f;

        // After mode entry, require stick to return to deadzone before accepting commands
        // モード遷移後、スティックがデッドゾーンに戻るまでコマンドをロック
        if (!stick_unlocked_) {
            if (normalized > -STICK_DEADZONE && normalized < STICK_DEADZONE) {
                stick_unlocked_ = true;
            }
            return 0.0f;  // Hold altitude until stick returns to center
        }

        // Apply deadzone
        // デッドゾーン適用
        if (normalized > -STICK_DEADZONE && normalized < STICK_DEADZONE) {
            return 0.0f;  // Hold altitude
        }

        // Map to climb/descent rate
        // 上昇/下降速度にマッピング
        if (normalized >= STICK_DEADZONE) {
            // Above deadzone: climb
            float t = (normalized - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
            return t * MAX_CLIMB_RATE;
        } else {
            // Below deadzone: descend
            float t = (normalized + STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
            return t * MAX_DESCENT_RATE;  // t is negative here
        }
    }
};

// Global altitude controller (defined in control_task.cpp)
// グローバル高度コントローラ（control_task.cppで定義）
extern AltitudeController g_altitude_controller;
