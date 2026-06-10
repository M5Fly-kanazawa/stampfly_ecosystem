/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

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
 * @design requirements.md §4 — Component #6: replaceable control      [OK]
 * @design detailed_design.md §4 — IController                        [OK]
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

void PidController::reloadParams()
{
    // Live tuning (ControllerCmd::ReloadParams): re-read gains and output
    // limits, KEEP the integrator state — a mid-flight gain change must not
    // kick the loops the way a full reset would.
    // ライブチューニング（ControllerCmd::ReloadParams）: ゲインと出力リミットを
    // 読み直し、積分器状態は「維持」する — 飛行中のゲイン変更が full reset の
    // ようにループを蹴ってはならない。
    loadParams();
    ESP_LOGI(TAG, "PID parameters reloaded (live)");
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
    params::get_float("position.vel.kp", vel_x_.kp);
    params::get_float("position.vel.ti", vel_x_.ti);
    pos_y_ = pos_x_;  // Same gains for X and Y / XとY同じゲイン
    vel_y_ = vel_x_;

    // Output limits — each loop is clamped to what its downstream stage can
    // physically deliver, so the conditional-integration anti-windup (pid.hpp)
    // sees the REAL saturation instead of the meaningless default 1.0.
    // 出力上限 — 各ループを下流段が物理的に出せる量でクランプし、条件付き積分の
    // アンチワインドアップ（pid.hpp）が無意味な既定 1.0 でなく実際の飽和を見るようにする。
    rate_roll_.output_limit  = max_roll_pitch_torque_;          // [Nm]
    rate_pitch_.output_limit = max_roll_pitch_torque_;          // [Nm]
    rate_yaw_.output_limit   = max_yaw_torque_;                 // [Nm]
    att_roll_.output_limit   = max_rate_;                       // [rad/s]
    att_pitch_.output_limit  = max_rate_;                       // [rad/s]
    alt_pos_.output_limit    = max_climb_rate_;                 // [m/s]
    alt_vel_.output_limit    = max_thrust_ - hover_thrust_;     // [N] thrust headroom
    pos_x_.output_limit      = max_pos_vel_;                    // [m/s]
    pos_y_.output_limit      = max_pos_vel_;                    // [m/s]
    vel_x_.output_limit      = gravity_ * max_pos_tilt_;        // [m/s²] = g·tilt limit
    vel_y_.output_limit      = gravity_ * max_pos_tilt_;        // [m/s²]
}

ControlOutput PidController::compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt)
{
    // Autonomous landing overrides the whole pipeline: the trigger conditions
    // (comm loss, battery emergency) mean the pilot setpoint is stale or must
    // be ignored. See computeLanding().
    // 自動着陸はパイプライン全体を上書きする: 発動条件（通信断・電池緊急）では
    // パイロット setpoint は stale か無視すべきもの。computeLanding() 参照。
    if (landing_) {
        return computeLanding(state, dt);
    }

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
    // Tilt setpoints, kept in scope for the Data Stream export below (in
    // POS_HOLD they are the position-cascade output, not the sticks).
    // 傾き目標。下の Data Stream 出力用にスコープを広げて保持（POS_HOLD では
    // スティックでなく位置カスケードの出力になる）。
    float roll_sp  = 0.0f;
    float pitch_sp = 0.0f;
    if (current_mode_ >= FlightMode::STABILIZE) {
        // Default: sticks command the tilt angle directly (STABILIZE).
        // 既定: スティックが傾き角を直接指令する（STABILIZE）。
        roll_sp  = setpoint.roll * max_angle_;
        pitch_sp = setpoint.pitch * max_angle_;

        // POS_HOLD: the position cascade OVERRIDES the stick tilt setpoints so the
        // craft holds its captured horizontal position instead of following sticks.
        // POS_HOLD: 位置カスケードがスティック傾き指令を上書きし、捕捉した水平位置を保持する。
        if (current_mode_ >= FlightMode::POS_HOLD) {
            computePositionHold(state, euler.z, dt, roll_sp, pitch_sp);
        }

        rate_sp_roll  = att_roll_.compute(roll_sp - euler.x, dt);
        rate_sp_pitch = att_pitch_.compute(pitch_sp - euler.y, dt);
    }

    // =========================================================================
    // Altitude control (ALT_HOLD and above)
    // 高度制御（ALT_HOLD以上）
    // =========================================================================
    if (current_mode_ >= FlightMode::ALT_HOLD) {
        const float altitude = -state.position[2];   // NED z-down → altitude up
        const float vel_up   = -state.velocity[2];   // vertical velocity, up positive

        // Capture the altitude target when entering ALT_HOLD, and keep tracking it
        // while the throttle stick is off-center (climb/descend), so releasing the
        // stick holds the altitude actually reached.
        // ALT_HOLD 進入時に高度目標を捕捉し、スロットルが中央外（上昇/下降）の間は追従。
        // スティックを戻すと到達した高度を保持する。
        if (capture_alt_) { alt_setpoint_ = altitude; capture_alt_ = false; }

        // Stick → climb rate / スティック → 上昇率
        float climb_rate_sp = 0;
        if (fabsf(setpoint.throttle - 0.5f) > stick_deadzone_) {
            climb_rate_sp = (setpoint.throttle - 0.5f) * 2.0f * max_climb_rate_;
            alt_setpoint_ = altitude;   // track while moving / 移動中は追従
        }

        // Cascade: altitude error → velocity sp → thrust correction. With the
        // stick centered, the position loop holds alt_setpoint (closed-loop).
        // カスケード: 高度誤差 → 速度目標 → 推力補正。中央では位置ループが alt_setpoint
        // を閉ループで保持する。
        float vel_sp_z = alt_pos_.compute(alt_setpoint_ - altitude, dt);
        if (climb_rate_sp != 0) vel_sp_z = climb_rate_sp;

        float thrust_correction = alt_vel_.compute(vel_sp_z - vel_up, dt);

        // Hover thrust + correction, clamped to the physical thrust range. The
        // mixer would silently clip negative/excess thrust at the duty stage
        // anyway; clamping here keeps the published control_output honest.
        // ホバー推力 + 補正。物理推力範囲にクランプする。ミキサーは duty 段で負/過大
        // 推力を黙ってクリップするが、ここでクランプして control_output を正直に保つ。
        thrust = hover_thrust_ + thrust_correction;
        if (thrust < 0.0f)         thrust = 0.0f;
        if (thrust > max_thrust_)  thrust = max_thrust_;
    }

    // Position control (POS_HOLD) is applied inside the attitude block above —
    // computePositionHold() turns the position/velocity error into the tilt
    // setpoints the attitude loop tracks. See the helper below.
    // 位置制御（POS_HOLD）は上の姿勢ブロック内で適用される（computePositionHold が
    // 位置/速度誤差を姿勢ループが追従する傾き指令に変換）。下のヘルパ参照。

    // =========================================================================
    // Rate control (always active, innermost loop)
    // レート制御（常にアクティブ、最内ループ）
    //
    // Gyro feedback from state estimate (bias-corrected by ESKF)
    // 状態推定からのジャイロフィードバック（ESKFでバイアス補正済み）
    // =========================================================================
    // Body angular rate from the state estimate (bias-corrected by the ESKF, FRD).
    // This closes the rate inner loop; it was previously hardcoded to 0 (open loop).
    // 状態推定からの機体角速度（ESKF でバイアス補正済み、FRD）。これでレート内ループが
    // 閉じる。以前は 0 固定＝開ループだった。
    math::Vec3 gyro_rate;
    gyro_rate.x = state.angular_rate[0];
    gyro_rate.y = state.angular_rate[1];
    gyro_rate.z = state.angular_rate[2];

    output.torque[0] = rate_roll_.compute(rate_sp_roll - gyro_rate.x, dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch - gyro_rate.y, dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw - gyro_rate.z, dt);
    output.thrust = thrust;

    // Export the cascade setpoints for the Data Stream (rate-loop reference at
    // the control rate is required for identification/tuning analysis).
    // カスケード目標値を Data Stream 用に出力（制御周期のレート目標は同定・
    // チューニング解析に必須）。
    output.rate_ref[0] = rate_sp_roll;
    output.rate_ref[1] = rate_sp_pitch;
    output.rate_ref[2] = rate_sp_yaw;
    output.angle_ref[0] = roll_sp;    // POS_HOLD: cascade output / ACRO: 0
    output.angle_ref[1] = pitch_sp;

    return output;
}

void PidController::computePositionHold(const StateEstimate& state, float yaw,
                                        float dt, float& roll_sp, float& pitch_sp)
{
    // Capture the hold target (NED north/east) when entering POS_HOLD.
    // POS_HOLD 進入時に保持目標（NED 北/東）を捕捉する。
    if (capture_pos_) {
        pos_setpoint_x_ = state.position[0];
        pos_setpoint_y_ = state.position[1];
        capture_pos_ = false;
    }

    // Outer loop (NED): position error → desired horizontal velocity.
    // 外ループ（NED）: 位置誤差 → 目標水平速度。
    const float vx_sp = pos_x_.compute(pos_setpoint_x_ - state.position[0], dt);
    const float vy_sp = pos_y_.compute(pos_setpoint_y_ - state.position[1], dt);

    // Inner loop (NED): velocity error → desired horizontal acceleration.
    // 内ループ（NED）: 速度誤差 → 目標水平加速度。
    const float ax_ned = vel_x_.compute(vx_sp - state.velocity[0], dt);
    const float ay_ned = vel_y_.compute(vy_sp - state.velocity[1], dt);

    // Rotate the desired NED acceleration into the body frame (yaw only).
    // 目標 NED 加速度を機体座標へ回転（ヨーのみ）。
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float ax_body =  cy * ax_ned + sy * ay_ned;   // forward (FRD X) / 前方
    const float ay_body = -sy * ax_ned + cy * ay_ned;   // right   (FRD Y) / 右

    // Map acceleration to tilt (a ≈ g·tilt). Accelerate forward → pitch nose down
    // (negative pitch); accelerate right → roll right (positive roll). Clamp to the
    // POS_HOLD tilt limit so the outer loop cannot command an aggressive attitude.
    // 加速度を傾きへ写像（a≈g·tilt）。前進=ノーズダウン(負pitch)、右=右ロール(正roll)。
    // 外ループが過激な姿勢を指令しないよう POS_HOLD 傾き上限でクランプ。
    auto clampTilt = [this](float t) {
        if (t >  max_pos_tilt_) return  max_pos_tilt_;
        if (t < -max_pos_tilt_) return -max_pos_tilt_;
        return t;
    };
    pitch_sp = clampTilt(-ax_body / gravity_);
    roll_sp  = clampTilt( ay_body / gravity_);
}

// -----------------------------------------------------------------------------
// computeLanding — autonomous landing: level attitude + fixed descent rate.
// Structure: STABILIZE attitude loop with zero tilt setpoints and zero yaw rate
// (no dependence on the stale sticks), plus the ALT_HOLD velocity loop tracking
// a constant downward velocity. Touchdown is detected by the ToF landing
// detector (TakeoffLandingMgr) which drives LANDING → IDLE_GROUND → disarm.
// computeLanding — 自動着陸: 水平姿勢＋固定降下率。
// 構成: 傾き指令ゼロ・ヨーレートゼロの STABILIZE 姿勢ループ（stale なスティックに
// 依存しない）＋ 一定の下向き速度を追従する ALT_HOLD の速度ループ。接地は ToF の
// 着陸検出（TakeoffLandingMgr）が捉え、LANDING → IDLE_GROUND → disarm が進む。
// -----------------------------------------------------------------------------
ControlOutput PidController::computeLanding(const StateEstimate& state, float dt)
{
    ControlOutput output = {};
    output.timestamp = state.timestamp;

    // Level-attitude cascade: tilt setpoints = 0 → rate setpoints.
    // 水平姿勢カスケード: 傾き指令 0 → レート指令。
    math::Quat q(state.attitude[0], state.attitude[1],
                 state.attitude[2], state.attitude[3]);
    math::Vec3 euler = q.to_euler();
    const float rate_sp_roll  = att_roll_.compute(0.0f - euler.x, dt);
    const float rate_sp_pitch = att_pitch_.compute(0.0f - euler.y, dt);
    const float rate_sp_yaw   = 0.0f;   // hold heading / 方位保持

    // Vertical: track a constant descent (up-positive velocity = −descent rate).
    // 鉛直: 一定降下を追従（上正の速度 = −降下率）。
    const float vel_up = -state.velocity[2];            // NED z-down → up-positive
    const float thrust_correction =
        alt_vel_.compute(-landing_descent_rate_ - vel_up, dt);
    float thrust = hover_thrust_ + thrust_correction;
    if (thrust < 0.0f)        thrust = 0.0f;
    if (thrust > max_thrust_) thrust = max_thrust_;

    // Innermost rate loop (same as the normal path).
    // 最内レートループ（通常経路と同じ）。
    output.torque[0] = rate_roll_.compute(rate_sp_roll - state.angular_rate[0], dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch - state.angular_rate[1], dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw - state.angular_rate[2], dt);
    output.thrust = thrust;

    // Data Stream export (landing: level attitude target, attitude-loop rates).
    // Data Stream 出力（着陸中: 水平姿勢目標、姿勢ループ由来のレート）。
    output.rate_ref[0] = rate_sp_roll;
    output.rate_ref[1] = rate_sp_pitch;
    output.rate_ref[2] = rate_sp_yaw;
    output.angle_ref[0] = 0.0f;
    output.angle_ref[1] = 0.0f;
    return output;
}

void PidController::onLanding()
{
    if (landing_) {
        return;   // already landing (idempotent) / 既に着陸中（冪等）
    }
    ESP_LOGW(TAG, "Autonomous landing engaged (%.1f m/s descent)",
             static_cast<double>(landing_descent_rate_));
    landing_ = true;

    // Fresh start for the loops the landing path uses: the attitude loops may
    // carry integrator state from a different mode, and the vertical loop may
    // have wound up against a saturated climb.
    // 着陸経路が使うループを仕切り直す: 姿勢ループは別モードの積分状態を、鉛直ループは
    // 飽和上昇に対する巻き上がりを抱えている可能性がある。
    att_roll_.reset();
    att_pitch_.reset();
    alt_vel_.reset();
}

void PidController::reset()
{
    rate_roll_.reset();  rate_pitch_.reset();  rate_yaw_.reset();
    att_roll_.reset();   att_pitch_.reset();
    alt_pos_.reset();    alt_vel_.reset();
    pos_x_.reset();      pos_y_.reset();
    vel_x_.reset();      vel_y_.reset();
    landing_ = false;    // landing override ends with the flight / 着陸上書きは飛行と共に終了
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
        // Capture the current altitude as the hold target when entering ALT_HOLD.
        // ALT_HOLD 進入時、現在高度を保持目標として捕捉する。
        if (new_mode >= FlightMode::ALT_HOLD && current_mode_ < FlightMode::ALT_HOLD) {
            capture_alt_ = true;
        }
        if (current_mode_ >= FlightMode::POS_HOLD || new_mode >= FlightMode::POS_HOLD) {
            pos_x_.reset(); pos_y_.reset();
            vel_x_.reset(); vel_y_.reset();
        }
        // Capture the current horizontal position as the hold target on entry.
        // POS_HOLD 進入時、現在の水平位置を保持目標として捕捉する。
        if (new_mode >= FlightMode::POS_HOLD && current_mode_ < FlightMode::POS_HOLD) {
            capture_pos_ = true;
        }
    }

    current_mode_ = new_mode;
}

}  // namespace sf
