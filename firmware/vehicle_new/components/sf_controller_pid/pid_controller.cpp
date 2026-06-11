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

    // Heading hold / ヘディングホールド
    params::get_float("attitude.yawhold.kp", yaw_hold_kp_);
    params::get_float("attitude.yawhold.rate_max", yaw_hold_rate_max_);

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

    // Output limits — each loop is clamped to what its downstream stage may
    // deliver, so the conditional-integration anti-windup (pid.hpp) sees the
    // REAL saturation instead of the meaningless default 1.0. The values follow
    // the flight-proven legacy vehicle/ limits (see pid_controller.hpp).
    // 出力上限 — 各ループを下流段に渡してよい量でクランプし、条件付き積分の
    // アンチワインドアップ（pid.hpp）が無意味な既定 1.0 でなく実際の飽和を見るように
    // する。値は旧 vehicle/ の飛行実績リミットに従う（pid_controller.hpp 参照）。
    rate_roll_.output_limit  = max_roll_pitch_torque_;          // [Nm]
    rate_pitch_.output_limit = max_roll_pitch_torque_;          // [Nm]
    rate_yaw_.output_limit   = max_yaw_torque_;                 // [Nm]
    att_roll_.output_limit   = max_att_rate_sp_;                // [rad/s]
    att_pitch_.output_limit  = max_att_rate_sp_;                // [rad/s]
    alt_pos_.output_limit    = max_climb_rate_;                 // [m/s]
    alt_vel_.output_limit    = max_thrust_correction_;          // [N]
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

    // Guidance cancel-on-stick-movement: any stick departing from its engage
    // snapshot hands control back to the pilot instantly (the pilot always wins).
    // 誘導のスティック動作解除: どれかのスティックが設定時スナップショットから動いたら
    // 即座にパイロットへ返す（パイロット優先）。
    if (guidance_active_) {
        const float dr = fabsf(setpoint.roll     - stick_snapshot_[0]);
        const float dp = fabsf(setpoint.pitch    - stick_snapshot_[1]);
        const float dy = fabsf(setpoint.yaw      - stick_snapshot_[2]);
        const float dt_ = fabsf(setpoint.throttle - stick_snapshot_[3]);
        if (dr > stick_move_cancel_ || dp > stick_move_cancel_ ||
            dy > stick_move_cancel_ || dt_ > stick_move_cancel_) {
            guidance_active_ = false;
            capture_pos_ = true;   // hold where we are now / いまの位置で保持し直す
            capture_alt_ = true;
            ESP_LOGW(TAG, "Guidance cancelled by pilot stick");
        }
    }

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

        // Auto-takeoff (ALT/POS modes): sticks are ignored — level attitude and
        // heading hold, exactly like the landing path's mirror image. POS_HOLD
        // additionally holds the launch point via the cascade below.
        // 自動離陸（ALT/POS モード）: スティック無視 — 水平姿勢＋方位保持（着陸経路の
        // 鏡像）。POS_HOLD はさらに下のカスケードで発進点を保持する。
        if (current_mode_ >= FlightMode::ALT_HOLD &&
            phase_ == VerticalPhase::TakeoffClimb) {
            roll_sp  = 0.0f;
            pitch_sp = 0.0f;
            rate_sp_yaw = 0.0f;
        }

        // Guidance: walk the POS_HOLD setpoints toward the target and steer yaw
        // with a rate-limited P loop. The cascade below then tracks the walking
        // setpoint — the proven hold loops are untouched, only their target moves.
        // 誘導: POS_HOLD 設定点を目標へ歩かせ、yaw はレート制限付き P で向ける。
        // 下のカスケードは歩く設定点を追従する — 実績の保持ループは無改変で、
        // 目標だけが動く。
        if (guidance_active_ && current_mode_ >= FlightMode::POS_HOLD &&
            phase_ == VerticalPhase::Airborne) {
            const float step = guide_speed_ * dt;
            auto seek = [step](float current, float target) {
                const float d = target - current;
                if (d >  step) return current + step;
                if (d < -step) return current - step;
                return target;
            };
            pos_setpoint_x_ = seek(pos_setpoint_x_, guide_pos_[0]);
            pos_setpoint_y_ = seek(pos_setpoint_y_, guide_pos_[1]);
            alt_setpoint_   = seek(alt_setpoint_, -guide_pos_[2]);  // NED z → alt
            capture_pos_ = false;   // setpoints are guidance-owned now / 設定点は誘導所有
            capture_alt_ = false;

            // Yaw: shortest-path error, P with a turn-rate limit.
            // ヨー: 最短経路誤差の P、回頭率制限付き。
            float yaw_err = guide_yaw_ - euler.z;
            while (yaw_err >  3.14159265f) yaw_err -= 6.2831853f;
            while (yaw_err < -3.14159265f) yaw_err += 6.2831853f;
            float yaw_cmd = guide_yaw_kp_ * yaw_err;
            if (yaw_cmd >  guide_yaw_rate_max_) yaw_cmd =  guide_yaw_rate_max_;
            if (yaw_cmd < -guide_yaw_rate_max_) yaw_cmd = -guide_yaw_rate_max_;
            rate_sp_yaw = yaw_cmd;
        }

        // Heading hold: yaw stick neutral → hold the heading captured at stick
        // release (rate-limited P on the estimator yaw; see pid_controller.hpp).
        // Skipped while guidance owns yaw, during auto-takeoff (its own law
        // already commands zero yaw rate), and on the ground — in STABILIZE the
        // throttle floor is the airborne test, in ALT_HOLD+ the vertical phase.
        // Any yaw stick input releases the hold instantly (pilot always wins);
        // the target re-captures at the next stick release.
        // ヘディングホールド: ヨースティック中立 → 離した瞬間に捕捉した方位を保持
        // （推定ヨー角のレート制限付き P。pid_controller.hpp 参照）。誘導がヨーを所有
        // している間・自動離陸中（その則が既にヨーレート 0 を指令）・地上ではスキップ —
        // STABILIZE ではスロットル床値が、ALT_HOLD 以上では鉛直フェーズが空中判定。
        // ヨースティック入力で即解除（パイロット優先）し、次の中立で目標を再捕捉する。
        if (yaw_hold_kp_ > 0.0f && !guidance_active_ &&
            !(current_mode_ >= FlightMode::ALT_HOLD &&
              phase_ != VerticalPhase::Airborne)) {
            const bool stick_neutral =
                fabsf(setpoint.yaw) < kYawHoldStickDeadband;
            const bool airborne =
                (current_mode_ >= FlightMode::ALT_HOLD) ||
                (setpoint.throttle > kYawHoldThrottleFloor);
            if (stick_neutral && airborne) {
                if (!yaw_hold_active_) {
                    yaw_hold_target_ = euler.z;   // capture on engage edge / 係合エッジで捕捉
                    yaw_hold_active_ = true;
                }
                // Shortest-path heading error, P with a turn-rate limit (the
                // same shape as the guidance yaw law above).
                // 最短経路の方位誤差、回頭率制限付き P（上の誘導ヨー則と同形）。
                float hold_err = yaw_hold_target_ - euler.z;
                while (hold_err >  3.14159265f) hold_err -= 6.2831853f;
                while (hold_err < -3.14159265f) hold_err += 6.2831853f;
                float hold_cmd = yaw_hold_kp_ * hold_err;
                if (hold_cmd >  yaw_hold_rate_max_) hold_cmd =  yaw_hold_rate_max_;
                if (hold_cmd < -yaw_hold_rate_max_) hold_cmd = -yaw_hold_rate_max_;
                rate_sp_yaw = hold_cmd;
            } else {
                yaw_hold_active_ = false;
            }
        }

        // POS_HOLD: the position cascade OVERRIDES the stick tilt setpoints so the
        // craft holds its captured horizontal position instead of following sticks.
        // Not while Grounded — the hold target is captured at (auto-)takeoff.
        // POS_HOLD: 位置カスケードがスティック傾き指令を上書きし、捕捉した水平位置を保持する。
        // Grounded 中は走らせない — 保持目標は（自動）離陸時に捕捉する。
        if (current_mode_ >= FlightMode::POS_HOLD &&
            phase_ != VerticalPhase::Grounded) {
            computePositionHold(state, euler.z, dt, roll_sp, pitch_sp);
        }

        rate_sp_roll  = att_roll_.compute(roll_sp, euler.x, dt);
        rate_sp_pitch = att_pitch_.compute(pitch_sp, euler.y, dt);
    }

    // =========================================================================
    // Altitude control (ALT_HOLD and above)
    // 高度制御（ALT_HOLD以上）
    // =========================================================================
    if (current_mode_ >= FlightMode::ALT_HOLD) {
        const float altitude = -state.position[2];   // NED z-down → altitude up
        const float vel_up   = -state.velocity[2];   // vertical velocity, up positive

        if (phase_ == VerticalPhase::Grounded) {
            // Armed on the ground: props stopped. Without this gate the vertical
            // loop would command hover thrust the instant the craft ARMs in
            // ALT/POS mode. Flight starts via the auto-takeoff verb (onTakeoff).
            // 地上 ARM 中: プロペラ停止。このゲートがないと ALT/POS で ARM した瞬間に
            // 鉛直ループがホバー推力を指令してしまう。飛行開始は自動離陸 verb から。
            thrust = 0.0f;
        } else if (phase_ == VerticalPhase::TakeoffClimb) {
            // Auto-takeoff: track a fixed climb rate (altitude loop bypassed —
            // there is no meaningful altitude target yet). The ToF airborne
            // detection ends this phase via TakeoffComplete.
            // 自動離陸: 固定上昇率を追従（高度ループはバイパス — まだ意味のある高度
            // 目標がない）。ToF の空中検知が TakeoffComplete でこのフェーズを終える。
            float thrust_correction =
                alt_vel_.compute(takeoff_climb_rate_, vel_up, dt);
            thrust = hover_thrust_ + thrust_correction;
            if (thrust < 0.0f)         thrust = 0.0f;
            if (thrust > max_thrust_)  thrust = max_thrust_;
        } else {
            // Airborne: normal ALT_HOLD law.
            // 空中: 通常の ALT_HOLD 則。
            // Capture the altitude target when entering ALT_HOLD, and keep tracking
            // it while the throttle stick is off-center (climb/descend), so releasing
            // the stick holds the altitude actually reached.
            // ALT_HOLD 進入時に高度目標を捕捉し、スロットルが中央外（上昇/下降）の間は
            // 追従。スティックを戻すと到達した高度を保持する。
            if (capture_alt_) { alt_setpoint_ = altitude; capture_alt_ = false; }

            // Stick → climb rate. Suppressed while guidance owns the altitude
            // target: in an API flight the throttle stick rests at the BOTTOM,
            // which must not read as a permanent descend command.
            // スティック → 上昇率。誘導が高度目標を所有する間は無効化: API 飛行では
            // スロットルスティックは下端のままで、それが恒常的な降下指令になっては
            // ならない。
            float climb_rate_sp = 0;
            if (!guidance_active_ &&
                fabsf(setpoint.throttle - 0.5f) > stick_deadzone_) {
                climb_rate_sp = (setpoint.throttle - 0.5f) * 2.0f * max_climb_rate_;
                alt_setpoint_ = altitude;   // track while moving / 移動中は追従
            }

            // Cascade: altitude error → velocity sp → thrust correction. With the
            // stick centered, the position loop holds alt_setpoint (closed-loop).
            // カスケード: 高度誤差 → 速度目標 → 推力補正。中央では位置ループが
            // alt_setpoint を閉ループで保持する。
            float vel_sp_z = alt_pos_.compute(alt_setpoint_, altitude, dt);
            if (climb_rate_sp != 0) vel_sp_z = climb_rate_sp;

            float thrust_correction = alt_vel_.compute(vel_sp_z, vel_up, dt);

            // Hover thrust + correction, clamped to the physical thrust range. The
            // mixer would silently clip negative/excess thrust at the duty stage
            // anyway; clamping here keeps the published control_output honest.
            // ホバー推力 + 補正。物理推力範囲にクランプする。ミキサーは duty 段で負/
            // 過大推力を黙ってクリップするが、ここでクランプして出力を正直に保つ。
            thrust = hover_thrust_ + thrust_correction;
            if (thrust < 0.0f)         thrust = 0.0f;
            if (thrust > max_thrust_)  thrust = max_thrust_;
        }
    }

    // Position control (POS_HOLD) is applied inside the attitude block above —
    // computePositionHold() turns the position/velocity error into the tilt
    // setpoints the attitude loop tracks. See the helper below.
    // 位置制御（POS_HOLD）は上の姿勢ブロック内で適用される（computePositionHold が
    // 位置/速度誤差を姿勢ループが追従する傾き指令に変換）。下のヘルパ参照。

    // =========================================================================
    // Sysid excitation: add the identification signal to ONE axis' rate
    // setpoint, after every outer loop has produced its setpoint and before
    // the rate loop consumes it — so the Data Stream's rate_ref (exported
    // below) carries the excitation exactly as the rate loop saw it.
    // 同定励振: 全外側ループが目標を作った後・レートループが消費する前に、1軸の
    // レート目標へ信号を加算する — Data Stream の rate_ref（下で出力）には
    // レートループが見たとおりの励振が乗る。
    // =========================================================================
    if (excite_active_) {
        if (phase_ != VerticalPhase::Airborne || landing_) {
            excite_active_ = false;   // safety: flight phase ended / 飛行終了で停止
        } else {
            float sig = 0.0f;
            if (excite_waveform_ == 2) {
                // Stepped sine at a fixed frequency (autotune measurement point).
                // 固定周波数のステップドサイン（自動チューンの測定点）。
                excite_phase_ += 2.0f * 3.14159265f * excite_freq_ * dt;
                sig = excite_amp_ * sinf(excite_phase_);
            } else if (excite_waveform_ == 1) {
                // Log chirp f0→f1 over the duration: phase(t) = 2π·f0·(k^t−1)/ln(k).
                // 対数チャープ: f0→f1。
                const float k = powf(kChirpF1 / kChirpF0, 1.0f / excite_dur_);
                const float lnk = logf(k);
                const float ph = 2.0f * 3.14159265f * kChirpF0 *
                                 (powf(k, excite_t_) - 1.0f) / lnk;
                sig = excite_amp_ * sinf(ph);
            } else {
                // Doublet train: alternating ± with a fixed half period.
                // ダブレット列: 固定半周期の交互±。
                const int half = static_cast<int>(excite_t_ / kDoubletHalfS);
                sig = ((half % 2) == 0) ? excite_amp_ : -excite_amp_;
            }
            if      (excite_axis_ == 0) rate_sp_roll  += sig;
            else if (excite_axis_ == 1) rate_sp_pitch += sig;
            else                        rate_sp_yaw   += sig;

            excite_t_ += dt;
            if (excite_t_ >= excite_dur_) {
                excite_active_ = false;
                if (excite_waveform_ == 2) {
                    // Stash the I/Q sums for this frequency point; ControlTask
                    // fetches and publishes (core components must not touch
                    // topics — smoke-test builds have no FreeRTOS).
                    // この周波数点の I/Q 和を保持。取得・発行は ControlTask
                    // （コア部品はトピック禁制 — smoke ビルドは FreeRTOS なし）。
                    sysid_pending_.w  = 2.0f * 3.14159265f * excite_freq_;
                    sysid_pending_.ur = iq_ur_; sysid_pending_.ui = iq_ui_;
                    sysid_pending_.yr = iq_yr_; sysid_pending_.yi = iq_yi_;
                    sysid_pending_.samples = iq_n_;
                    sysid_pending_.seq = ++sysid_seq_;
                    sysid_pending_.timestamp = state.timestamp;
                    sysid_pending_valid_ = true;
                }
                ESP_LOGI(TAG, "Sysid excitation done");
            }
        }
    }

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

    output.torque[0] = rate_roll_.compute(rate_sp_roll, gyro_rate.x, dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch, gyro_rate.y, dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw, gyro_rate.z, dt);
    output.thrust = thrust;

    // Stepped-sine I/Q accumulation (after the settle transient): correlate the
    // ACTUAL rate-loop output torque u and the gyro y with the excitation phase.
    // ステップドサインの I/Q 蓄積（整定過渡後）: 「実際の」レートループ出力トルク u と
    // ジャイロ y を励振位相と相関する。
    if (excite_active_ && excite_waveform_ == 2 && excite_t_ > excite_settle_s_) {
        const float u_ax = output.torque[excite_axis_];
        const float y_ax = (excite_axis_ == 0) ? gyro_rate.x
                         : (excite_axis_ == 1) ? gyro_rate.y : gyro_rate.z;
        const float c = cosf(excite_phase_);
        const float sn = sinf(excite_phase_);
        iq_ur_ += u_ax * c;  iq_ui_ -= u_ax * sn;
        iq_yr_ += y_ax * c;  iq_yi_ -= y_ax * sn;
        iq_n_++;
    }

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
    const float vx_sp = pos_x_.compute(pos_setpoint_x_, state.position[0], dt);
    const float vy_sp = pos_y_.compute(pos_setpoint_y_, state.position[1], dt);

    // Inner loop (NED): velocity error → desired horizontal acceleration.
    // 内ループ（NED）: 速度誤差 → 目標水平加速度。
    const float ax_ned = vel_x_.compute(vx_sp, state.velocity[0], dt);
    const float ay_ned = vel_y_.compute(vy_sp, state.velocity[1], dt);

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
    const float rate_sp_roll  = att_roll_.compute(0.0f, euler.x, dt);
    const float rate_sp_pitch = att_pitch_.compute(0.0f, euler.y, dt);
    const float rate_sp_yaw   = 0.0f;   // hold heading / 方位保持

    // Vertical: track a constant descent (up-positive velocity = −descent rate).
    // 鉛直: 一定降下を追従（上正の速度 = −降下率）。
    const float vel_up = -state.velocity[2];            // NED z-down → up-positive
    const float thrust_correction =
        alt_vel_.compute(-landing_descent_rate_, vel_up, dt);
    float thrust = hover_thrust_ + thrust_correction;
    if (thrust < 0.0f)        thrust = 0.0f;
    if (thrust > max_thrust_) thrust = max_thrust_;

    // Innermost rate loop (same as the normal path).
    // 最内レートループ（通常経路と同じ）。
    output.torque[0] = rate_roll_.compute(rate_sp_roll, state.angular_rate[0], dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch, state.angular_rate[1], dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw, state.angular_rate[2], dt);
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

void PidController::onTakeoff()
{
    if (phase_ != VerticalPhase::Grounded) {
        return;   // already airborne or climbing (idempotent) / 既に上昇中か空中（冪等）
    }
    ESP_LOGI(TAG, "Auto-takeoff engaged (%.1f m/s climb)",
             static_cast<double>(takeoff_climb_rate_));
    phase_ = VerticalPhase::TakeoffClimb;

    // Fresh vertical loop (it may hold a Grounded-phase zero-output history) and a
    // launch-point capture for POS_HOLD (the cascade starts at the next compute).
    // 鉛直ループを仕切り直し（Grounded フェーズのゼロ出力履歴を持ちうる）、POS_HOLD 用に
    // 発進点を捕捉（カスケードは次の compute から動く）。
    alt_vel_.reset();
    capture_pos_ = true;
}

void PidController::onTakeoffComplete()
{
    if (phase_ == VerticalPhase::Airborne) {
        return;   // idempotent / 冪等
    }
    if (phase_ == VerticalPhase::TakeoffClimb) {
        ESP_LOGI(TAG, "Auto-takeoff complete — normal mode law engaged");
    }
    phase_ = VerticalPhase::Airborne;

    // ALT_HOLD captures the altitude actually reached; the vertical-loop integral
    // is kept (it has spooled to near-hover thrust — a smooth handoff).
    // ALT_HOLD は実際に到達した高度を捕捉。鉛直ループの積分は維持（ほぼホバー推力まで
    // 巻き上がっており、滑らかな引き継ぎになる）。
    capture_alt_ = true;
}

void PidController::setGuidanceTarget(const GuidanceTarget& target,
                                      const CommandSetpoint& current_sticks)
{
    // Guidance is a POS_HOLD-only feature (the position cascade is what tracks
    // the walking setpoint). Reject elsewhere so a stray API target cannot
    // disturb a manual mode.
    // 誘導は POS_HOLD 専用（歩く設定点を追うのは位置カスケード）。他モードでは拒否し、
    // 迷い込んだ API 目標が手動モードを乱さないようにする。
    if (current_mode_ < FlightMode::POS_HOLD || target.mode == 0) {
        ESP_LOGW(TAG, "Guidance target ignored (mode=%s)",
                 flightModeName(current_mode_));
        return;
    }
    guide_pos_[0] = target.position[0];
    guide_pos_[1] = target.position[1];
    guide_pos_[2] = target.position[2];
    guide_yaw_    = target.yaw;
    if (target.speed > 0.05f && target.speed <= 2.0f) {
        guide_speed_ = target.speed;
    }
    // Stick snapshot: guidance is cancelled by stick MOVEMENT, not position —
    // in an API flight the throttle stick rests at the bottom, which must not
    // read as a descend command or an instant cancel.
    // スティックスナップショット: 解除は「位置」でなく「動き」で判定 — API 飛行では
    // スロットルスティックは下端のままであり、それが降下指令や即時解除になってはならない。
    stick_snapshot_[0] = current_sticks.roll;
    stick_snapshot_[1] = current_sticks.pitch;
    stick_snapshot_[2] = current_sticks.yaw;
    stick_snapshot_[3] = current_sticks.throttle;
    guidance_active_ = true;
    ESP_LOGI(TAG, "Guidance target: NED [%.2f %.2f %.2f] yaw %.2f speed %.2f",
             static_cast<double>(guide_pos_[0]), static_cast<double>(guide_pos_[1]),
             static_cast<double>(guide_pos_[2]), static_cast<double>(guide_yaw_),
             static_cast<double>(guide_speed_));
}

void PidController::startExcitation(const SysidCommand& cmd)
{
    if (phase_ != VerticalPhase::Airborne || landing_) {
        ESP_LOGW(TAG, "Sysid excitation rejected: not airborne");
        return;
    }
    excite_axis_     = (cmd.axis <= 2) ? cmd.axis : 0;
    excite_waveform_ = (cmd.waveform <= 2) ? cmd.waveform : 1;
    excite_freq_     = cmd.frequency;
    if (excite_waveform_ == 2 &&
        (excite_freq_ < 0.5f || excite_freq_ > 50.0f)) {
        ESP_LOGW(TAG, "Sysid sine rejected: bad frequency %.1f Hz",
                 static_cast<double>(excite_freq_));
        return;
    }
    // Stepped sine: skip 2 excitation cycles of transient, then accumulate.
    // ステップドサイン: 2 周期分の過渡を捨ててから蓄積。
    excite_phase_    = 0.0f;
    excite_settle_s_ = (excite_waveform_ == 2) ? (2.0f / excite_freq_) : 0.0f;
    iq_ur_ = iq_ui_ = iq_yr_ = iq_yi_ = 0.0f;
    iq_n_  = 0;
    excite_amp_ = cmd.amplitude;
    if (excite_amp_ < 0.0f)           excite_amp_ = 0.0f;
    if (excite_amp_ > kExciteAmpMax)  excite_amp_ = kExciteAmpMax;
    excite_dur_ = cmd.duration;
    if (excite_dur_ <= 0.0f)          excite_dur_ = 1.0f;
    if (excite_dur_ > kExciteDurMax)  excite_dur_ = kExciteDurMax;
    excite_t_      = 0.0f;
    excite_active_ = true;
    ESP_LOGI(TAG, "Sysid excitation start: axis=%u %s amp=%.2f rad/s dur=%.1f s",
             static_cast<unsigned>(excite_axis_),
             excite_waveform_ == 2 ? "sine" :
             (excite_waveform_ == 1 ? "chirp" : "doublet"),
             static_cast<double>(excite_amp_), static_cast<double>(excite_dur_));
}

bool PidController::fetchSysidResult(SysidFreqResult& out)
{
    if (!sysid_pending_valid_) {
        return false;
    }
    out = sysid_pending_;
    sysid_pending_valid_ = false;
    return true;
}

void PidController::reset()
{
    rate_roll_.reset();  rate_pitch_.reset();  rate_yaw_.reset();
    att_roll_.reset();   att_pitch_.reset();
    alt_pos_.reset();    alt_vel_.reset();
    pos_x_.reset();      pos_y_.reset();
    vel_x_.reset();      vel_y_.reset();
    landing_ = false;    // landing override ends with the flight / 着陸上書きは飛行と共に終了
    phase_   = VerticalPhase::Grounded;  // next flight starts grounded / 次の飛行は接地から
    guidance_active_ = false;            // guidance dies with the flight / 誘導も飛行と共に終了
    excite_active_   = false;            // so does the excitation / 励振も同様
    yaw_hold_active_ = false;            // heading hold too / ヘディングホールドも同様
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
        // Heading-hold target is mode-local: re-capture under the new mode's law.
        // ヘディングホールド目標はモード局所 — 新モードの則で再捕捉する。
        yaw_hold_active_ = false;
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
            guidance_active_ = false;   // mode change revokes guidance / モード切替で誘導解除
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
