/**
 * @file pid_controller.cpp
 * @brief Cascade PID controller — IController implementation
 *        カスケードPID制御器 — IController実装
 *
 * Cascade structure per flight mode:
 * カスケード構造（フライトモード別）:
 *
 *   ACRO:      setpoint → Rate PID → torque
 *   STABILIZE: setpoint → Attitude PID → Rate PID → torque
 *   ALT_HOLD:  setpoint → Alt PID → Vel PID → thrust correction
 *   POS_HOLD:  setpoint → Pos PID → Vel PID → angle correction
 *
 * @design requirements.md §4 — Component #6: replaceable control      [--]
 * @design detailed_design.md §4 — IController                        [--]
 */

#include "pid_controller.hpp"
#include "params.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "PID";

namespace sf {

void PidController::init()
{
    // Load gains from parameter system
    // パラメータシステムからゲインを読み込み
    loadParams();
    reset();
    ESP_LOGI(TAG, "PID cascade controller initialized");
}

void PidController::loadParams()
{
    // Rate control / レート制御
    params::get_float("rate.roll.kp", rate_roll_.kp);
    params::get_float("rate.roll.ti", rate_roll_.ti);
    params::get_float("rate.roll.td", rate_roll_.td);
    params::get_float("rate.pitch.kp", rate_pitch_.kp);
    params::get_float("rate.pitch.ti", rate_pitch_.ti);
    params::get_float("rate.pitch.td", rate_pitch_.td);
    params::get_float("rate.yaw.kp", rate_yaw_.kp);
    params::get_float("rate.yaw.ti", rate_yaw_.ti);
    params::get_float("rate.yaw.td", rate_yaw_.td);

    // Attitude control / 姿勢制御
    params::get_float("attitude.roll.kp", att_roll_.kp);
    params::get_float("attitude.roll.ti", att_roll_.ti);
    params::get_float("attitude.roll.td", att_roll_.td);
    params::get_float("attitude.pitch.kp", att_pitch_.kp);
    params::get_float("attitude.pitch.ti", att_pitch_.ti);
    params::get_float("attitude.pitch.td", att_pitch_.td);

    // Altitude control / 高度制御
    params::get_float("altitude.alt.kp", alt_pos_.kp);
    params::get_float("altitude.alt.ti", alt_pos_.ti);
    params::get_float("altitude.vel.kp", alt_vel_.kp);
    params::get_float("altitude.vel.ti", alt_vel_.ti);

    // Position control / 位置制御
    params::get_float("position.pos.kp", pos_x_.kp);
    params::get_float("position.pos.ti", pos_x_.ti);
    pos_y_ = pos_x_;  // Same gains for X and Y / XとY同じゲイン
    params::get_float("position.vel.kp", vel_x_.kp);
    params::get_float("position.vel.ti", vel_x_.ti);
    vel_y_ = vel_x_;
}

ControlOutput PidController::compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt)
{
    ControlOutput output = {};
    output.timestamp = state.timestamp;

    // Extract Euler angles from quaternion
    // クォータニオンからオイラー角を抽出
    math::Quat q(state.attitude[0], state.attitude[1],
                 state.attitude[2], state.attitude[3]);
    math::Vec3 euler = q.to_euler();

    // Rate setpoints (default: direct from stick for ACRO)
    // レートセットポイント（デフォルト: ACRO用にスティックから直接）
    float rate_sp_roll  = setpoint.roll * max_rate_;
    float rate_sp_pitch = setpoint.pitch * max_rate_;
    float rate_sp_yaw   = setpoint.yaw * max_yaw_rate_;

    // Thrust (direct throttle for non-altitude modes)
    // 推力（高度制御なしモードではスロットル直接）
    float thrust = setpoint.throttle * max_thrust_;

    // =========================================================================
    // Attitude control (STABILIZE and above)
    // 姿勢制御（STABILIZE以上）
    // =========================================================================
    if (current_mode_ >= FlightMode::STABILIZE) {
        float roll_sp  = setpoint.roll * max_angle_;
        float pitch_sp = setpoint.pitch * max_angle_;

        rate_sp_roll  = att_roll_.compute(roll_sp - euler.x, dt);
        rate_sp_pitch = att_pitch_.compute(pitch_sp - euler.y, dt);
    }

    // =========================================================================
    // Altitude control (ALT_HOLD and above)
    // 高度制御（ALT_HOLD以上）
    // =========================================================================
    if (current_mode_ >= FlightMode::ALT_HOLD) {
        // Stick → climb rate / スティック → 上昇率
        float climb_rate_sp = 0;
        if (fabsf(setpoint.throttle - 0.5f) > stick_deadzone_) {
            climb_rate_sp = (setpoint.throttle - 0.5f) * 2.0f * max_climb_rate_;
        }

        // Cascade: altitude error → velocity sp → thrust correction
        // カスケード: 高度誤差 → 速度目標 → 推力補正
        float vel_sp_z = alt_pos_.compute(alt_setpoint_ - (-state.position[2]), dt);
        if (climb_rate_sp != 0) vel_sp_z = climb_rate_sp;

        float thrust_correction = alt_vel_.compute(vel_sp_z - (-state.velocity[2]), dt);

        // Hover thrust + correction / ホバー推力 + 補正
        thrust = hover_thrust_ + thrust_correction;
    }

    // =========================================================================
    // Position control (POS_HOLD)
    // 位置制御（POS_HOLD）
    // =========================================================================
    if (current_mode_ >= FlightMode::POS_HOLD) {
        // TODO: Position → velocity → angle cascade
        // TODO: 位置 → 速度 → 角度 カスケード
    }

    // =========================================================================
    // Rate control (always active, innermost loop)
    // レート制御（常にアクティブ、最内ループ）
    //
    // Gyro feedback from state estimate (bias-corrected by ESKF)
    // 状態推定からのジャイロフィードバック（ESKFでバイアス補正済み）
    // =========================================================================
    math::Vec3 gyro_rate;
    // Approximate body rates from quaternion derivative
    // クォータニオン微分からボディレートを近似
    // For now, use the gyro bias as a proxy (actual gyro comes via topic)
    // 暫定: ジャイロバイアスをプロキシとして使用（実ジャイロはトピック経由）
    gyro_rate.x = 0;  // TODO: Get actual body rates
    gyro_rate.y = 0;
    gyro_rate.z = 0;

    output.torque[0] = rate_roll_.compute(rate_sp_roll - gyro_rate.x, dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch - gyro_rate.y, dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw - gyro_rate.z, dt);
    output.thrust = thrust;

    return output;
}

void PidController::reset()
{
    rate_roll_.reset();  rate_pitch_.reset();  rate_yaw_.reset();
    att_roll_.reset();   att_pitch_.reset();
    alt_pos_.reset();    alt_vel_.reset();
    pos_x_.reset();      pos_y_.reset();
    vel_x_.reset();      vel_y_.reset();
    ESP_LOGI(TAG, "PID controller reset");
}

void PidController::onModeChange(FlightMode new_mode)
{
    ESP_LOGI(TAG, "Mode: %s → %s",
             flightModeName(current_mode_), flightModeName(new_mode));

    // Reset outer loops when switching modes
    // モード切替時に外側ループをリセット
    if (new_mode != current_mode_) {
        if (current_mode_ >= FlightMode::STABILIZE || new_mode >= FlightMode::STABILIZE) {
            att_roll_.reset();
            att_pitch_.reset();
        }
        if (current_mode_ >= FlightMode::ALT_HOLD || new_mode >= FlightMode::ALT_HOLD) {
            alt_pos_.reset();
            alt_vel_.reset();
        }
        if (current_mode_ >= FlightMode::POS_HOLD || new_mode >= FlightMode::POS_HOLD) {
            pos_x_.reset(); pos_y_.reset();
            vel_x_.reset(); vel_y_.reset();
        }
    }

    current_mode_ = new_mode;
}

}  // namespace sf
