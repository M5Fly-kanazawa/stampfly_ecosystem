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
 * @design detailed_design.md §3 注4/注5/注6 — takeoff/landing law    [OK]
 * @design architecture.md INV-1/INV-2 — one attitude pipeline; pilot [OK]
 *         keeps attitude (level only on a dead link)
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

    // Attitude trim (equilibrium tilt, all modes) / 姿勢トリム（平衡傾き・全モード）
    params::get_float("attitude.roll.trim",  roll_trim_);
    params::get_float("attitude.pitch.trim", pitch_trim_);
    int32_t trim_learn = 1;  // onboard trim-learning enable / オンボード学習の有効化
    params::get_int("attitude.trim.learn", trim_learn);
    trim_learn_enable_ = (trim_learn != 0);

    // Altitude control / 高度制御
    params::get_float("altitude.alt.kp", alt_pos_.kp);
    params::get_float("altitude.alt.ti", alt_pos_.ti);
    params::get_float("altitude.vel.kp", alt_vel_.kp);
    params::get_float("altitude.vel.ti", alt_vel_.ti);
    // Manual ALT_HOLD stick rates (separately tunable) / 手動 ALT_HOLD スティック速度（別々に調整可）
    params::get_float("altitude.climb_rate",   max_climb_rate_);
    params::get_float("altitude.descent_rate", max_descent_rate_);

    // Hover thrust correction: hover_thrust = mg × corr. Tunable so a motor/prop
    // change (e.g. fresh, stronger motors) can be matched WITHOUT a rebuild — drop
    // corr when the craft over-climbs on auto-takeoff. mg = 0.037 kg · 9.80665.
    // ホバー推力補正: hover_thrust = mg × corr。モータ/プロペラ交換（新品=強い等）に
    // 再ビルドなしで合わせられる — 自動離陸で過上昇するなら corr を下げる。
    constexpr float kMassG = 0.037f * 9.80665f;   // mg [N]
    float hover_corr = 1.12f;
    params::get_float("hover.thrust_corr", hover_corr);
    hover_thrust_ = kMassG * hover_corr;

    // Position control / 位置制御
    params::get_float("position.pos.kp", pos_x_.kp);
    params::get_float("position.pos.ti", pos_x_.ti);
    params::get_float("position.vel.kp", vel_x_.kp);
    params::get_float("position.vel.ti", vel_x_.ti);
    pos_y_ = pos_x_;  // Same gains for X and Y / XとY同じゲイン
    vel_y_ = vel_x_;
    // POS_HOLD stick reposition speed (deflect to move, release to hold) / POS_HOLD スティック再配置速度
    params::get_float("position.stick_vel", stick_reposition_vel_);

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
    // NOTE: autonomous landing is NOT a separate path — it is VerticalPhase::Landing,
    // handled inline in the attitude and vertical blocks below so it shares the ONE
    // pipeline (INV-1) and keeps pilot attitude while the link is live (INV-2).
    // 注: 自動着陸は別経路でない — VerticalPhase::Landing として下の姿勢/鉛直ブロックで
    // インライン処理し、単一パイプライン（INV-1）を共有し、リンク生存中は姿勢を保つ（INV-2）。
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
    // Run the attitude cascade for STABILIZE and above, AND during Landing in any
    // mode — an autonomous landing must be attitude-stabilized even if it started
    // from ACRO (a comm-loss landing). INV-1: one attitude path for every phase.
    // 姿勢カスケードは STABILIZE 以上、加えて任意モードの Landing 中も走らせる — 自動着陸は
    // ACRO から始まっても（通信途絶着陸）姿勢安定化が必要。INV-1: 全フェーズ単一姿勢経路。
    if (current_mode_ >= FlightMode::STABILIZE ||
        phase_ == VerticalPhase::Landing) {
        // Default: sticks command the tilt angle directly (STABILIZE).
        // 既定: スティックが傾き角を直接指令する（STABILIZE）。
        roll_sp  = setpoint.roll * max_angle_;
        pitch_sp = setpoint.pitch * max_angle_;

        // Auto-takeoff is auto-VERTICAL ONLY: the controller climbs to the target
        // altitude (the altitude block below owns thrust), but the PILOT keeps FULL
        // attitude control throughout — roll/pitch tilt the craft and yaw turns it,
        // exactly as in normal flight (centered sticks → level → straight-up climb;
        // hands-on → the pilot steers while climbing; tilting only costs ~cosθ of lift,
        // which the altitude loop compensates). Earlier TakeoffClimb forced level attitude
        // (roll_sp=pitch_sp=yaw=0), which DEAD-STICKED roll/pitch whenever the craft stayed
        // in that phase — the flight-test bug (2026-06-14). Never lock out attitude: the
        // pilot must be able to steer in any state (user decision). POS_HOLD still holds the
        // launch point via the position cascade below (that override is by design, and
        // pilot intent in POS_HOLD is "hold", not "tilt").
        // 自動離陸は「鉛直のみ自動」: 制御器は目標高度まで上昇する（推力は下の高度ブロックが
        // 所有）が、パイロットは終始「完全な姿勢制御」を保つ — roll/pitch で傾け yaw で回頭、
        // 通常飛行と全く同じ（中立→水平→真上に上昇／倒せば上昇中も操縦／傾きは ~cosθ の揚力損
        // のみで高度ループが補償）。旧来は TakeoffClimb で水平強制（roll_sp=pitch_sp=yaw=0）し、
        // そのフェーズに留まるとロール/ピッチが死んだ（実機バグ 2026-06-14）。どの状態でも姿勢を
        // 奪わない（ユーザー判断）。POS_HOLD は下の位置カスケードで発進点を保持する（設計どおり。
        // POS_HOLD のパイロット意図は「保持」であって「傾ける」ではない）。

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
        // Active in flight AND during the auto-takeoff climb (attitude is the pilot's in
        // every airborne phase) — skipped only while guidance owns yaw and while Grounded.
        // In STABILIZE the throttle floor is the airborne test; in ALT_HOLD+ being off the
        // Grounded phase is. Any yaw stick input releases the hold instantly (pilot always
        // wins); the target re-captures at the next stick release.
        // ヘディングホールド: ヨースティック中立 → 離した瞬間に捕捉した方位を保持
        // （推定ヨー角のレート制限付き P。pid_controller.hpp 参照）。飛行中＋自動離陸の上昇中も
        // 有効（空中の全フェーズで姿勢はパイロットのもの）— 誘導がヨーを所有している間・地上では
        // スキップ。STABILIZE ではスロットル床値が、ALT_HOLD 以上では Grounded でないことが
        // 空中判定。ヨースティック入力で即解除（パイロット優先）し、次の中立で再捕捉する。
        if (yaw_hold_kp_ > 0.0f && !guidance_active_ &&
            !(current_mode_ >= FlightMode::ALT_HOLD &&
              phase_ == VerticalPhase::Grounded)) {
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

        // POS_HOLD: the position cascade OVERRIDES the stick tilt setpoints — a centred
        // stick HOLDS the captured position, and a deflected roll/pitch stick REPOSITIONS
        // it (commands a horizontal velocity; releasing re-captures the new position).
        // See computePositionHold. Not while Grounded — the hold target is captured at
        // (auto-)takeoff.
        // POS_HOLD: 位置カスケードがスティック傾き指令を上書きする — 中立は捕捉位置を保持し、
        // roll/pitch を倒すと「再配置」する（水平速度を指令、離すと新位置を再捕捉）。
        // computePositionHold 参照。Grounded 中は走らせない — 保持目標は（自動）離陸時に捕捉。
        // Not during Landing: the autonomous descent uses direct stick tilt (or the
        // level gate below), not the position cascade — keep the landing law uniform
        // across modes (a POS_HOLD landing steers like STABILIZE).
        // Landing 中は走らせない: 自動降下は位置カスケードでなく直接スティック傾き（下の
        // 水平ゲート）を使い、着陸則をモード間で統一する（POS_HOLD 着陸も STABILIZE 同様に操縦）。
        if (current_mode_ >= FlightMode::POS_HOLD &&
            phase_ != VerticalPhase::Grounded &&
            phase_ != VerticalPhase::Landing) {
            computePositionHold(state, setpoint, euler.z, dt, roll_sp, pitch_sp);
        }

        // Landing — the SINGLE level gate (INV-2). The pilot keeps roll/pitch/yaw while
        // the command link is LIVE (a pilot-commanded landing is steerable, same vertical-
        // only-auto principle as takeoff). If the link is STALE (comm-loss failsafe — no
        // pilot), force LEVEL: stale sticks must not steer. Link-liveness = setpoint
        // freshness, so no failsafe flag is threaded and it self-levels if the link drops
        // mid-descent. roll_sp/pitch_sp already hold the stick tilt; zero them when stale.
        // Landing — 単一の水平ゲート（INV-2）。リンク生存中はパイロットが roll/pitch/yaw を保つ
        // （パイロット指令の着陸は操縦可、離陸と同じ鉛直のみ自動の思想）。リンク途絶（通信途絶
        // フェイルセーフ＝パイロット不在）なら水平に強制: stale なスティックで操縦させない。生存
        // 判定は設定点の新鮮さゆえフラグ配線不要、降下中にリンクが切れても自動で水平化する。
        if (phase_ == VerticalPhase::Landing) {
            const bool link_live =
                (state.timestamp - setpoint.timestamp) < kLandingLinkStaleUs;
            if (!link_live) {
                roll_sp = 0.0f;
                pitch_sp = 0.0f;
                rate_sp_yaw = 0.0f;
            }
        }

        // @design architecture.md INV-1 — attitude trim at the single cascade
        //        confluence: equilibrium tilt added to the angle SETPOINT for EVERY
        //        mode (after the POS_HOLD override and the Landing level gate, before
        //        the angle→rate loop). The angle loop then holds this tilt, cancelling
        //        steady horizontal drift with no extra thrust, and POS_HOLD's position
        //        loop no longer has to carry the equilibrium tilt. On a stale-link
        //        landing the level gate zeros the pilot tilt but trim still applies,
        //        so the descent holds the equilibrium-level (stable) not a false
        //        geometric zero (which would drift by the trim amount). Flight-identified. [OK]
        // @design architecture.md INV-1 — 単一合流点での姿勢トリム: 平衡傾きを全モードの
        //        角度「目標」に加算（POS_HOLD 上書きと Landing 水平ゲートの後、角度→レート
        //        ループの前）。角度ループがこの傾きを保ち、定常水平ドリフトを推力を余分に
        //        食わず打ち消す。POS_HOLD の位置ループは平衡傾きを担わずに済む。リンク途絶
        //        着陸では水平ゲートがパイロット傾きを 0 にするがトリムは残るため、降下は
        //        平衡水平（安定）を保ち見かけの幾何ゼロ（トリム分ドリフトする）にしない。飛行で同定。
        // Always-on learning: nudge roll_trim_/pitch_trim_ toward the equilibrium from
        // the hover drift (hover-gated) BEFORE applying them, so the craft self-trims
        // while flying. / 常時学習: 適用前にホバードリフトから roll_trim_/pitch_trim_ を
        // 平衡へ寄せる（ホバー限定）→ 飛行中に自己トリム。
        learnTrim(state, setpoint, euler.z, dt);
        roll_sp  += roll_trim_;
        pitch_sp += pitch_trim_;

        rate_sp_roll  = att_roll_.compute(roll_sp, euler.x, dt);
        rate_sp_pitch = att_pitch_.compute(pitch_sp, euler.y, dt);
    }

    // =========================================================================
    // Vertical channel — phase-switched (INV-1: the phase changes ONLY this channel)
    // 鉛直チャネル — フェーズで分岐（INV-1: フェーズが変えるのはこのチャネルのみ）
    // =========================================================================
    if (phase_ == VerticalPhase::Landing) {
        // Autonomous landing descent — mode-INDEPENDENT (a comm-loss landing can start
        // from ACRO/STABILIZE, which have no altitude loop). Track a constant downward
        // velocity; the touchdown detector (TakeoffLandingMgr, INV-3) ends LANDING at the
        // ground. Attitude was handled by the one pipeline above (pilot or level gate).
        // 自動着陸の降下 — モード非依存（ACRO/STABILIZE からの通信途絶着陸は高度ループを持た
        // ない）。一定の下向き速度を追従し、接地は TakeoffLandingMgr が検出（INV-3）して LANDING
        // を終える。姿勢は上の単一パイプライン（パイロット or 水平ゲート）で処理済み。
        const float altitude = -state.position[2];   // NED z-down → altitude up
        const float vel_up   = -state.velocity[2];   // up-positive / 上正
        const float thrust_correction =
            alt_vel_.compute(-landing_descent_rate_, vel_up, dt);
        thrust = hover_thrust_ + thrust_correction;

        // Near-ground settle assist: a constant-velocity descent is BALANCED by ground
        // effect near the floor (more rotor lift on reduced thrust), so the craft lingers
        // and the descent stalls (pilot report 2026-06-14). Once the altitude estimate is
        // near the ground, ramp the thrust CEILING down from hover toward
        // kLandingSettleThrustFrac·hover over kLandingSettleRampS. The velocity loop still
        // modulates within the ceiling (so a normal descent touches down gently before the
        // ceiling bites), but if the craft is STILL floating the falling ceiling drops below
        // what ground effect can support and the craft positively SETTLES — bounded linger,
        // no thrust-threshold guesswork, and no free-fall (the floor is a fraction of hover,
        // not 0). This only shapes the vertical channel (INV-1); attitude is untouched.
        // 近地面の着地アシスト: 定速降下は接地近傍で地面効果（低推力でローター揚力増）と釣り合い、
        // 機体が粘って降下停滞する（パイロット報告）。高度推定が接地近傍に来たら、推力の上限を
        // hover から kLandingSettleThrustFrac·hover へ kLandingSettleRampS かけて絞る。速度ループ
        // は上限内で調整（通常降下は上限が効く前に穏やかに接地）だが、まだ浮いていれば下がる上限が
        // 地面効果の支えを下回り機体は確実に沈む — 粘りは有界、推力閾値の当て推量も自由落下もない
        // （床は hover の一定割合で 0 でない）。鉛直チャネルのみ整形（INV-1）、姿勢は不変。
        float thrust_ceiling = max_thrust_;
        if (altitude < kLandingSettleAltM) {
            landing_settle_t_ += dt;
            const float r = (landing_settle_t_ < kLandingSettleRampS)
                                ? (landing_settle_t_ / kLandingSettleRampS) : 1.0f;
            const float frac = 1.0f - (1.0f - kLandingSettleThrustFrac) * r;  // hover→frac·hover
            thrust_ceiling = hover_thrust_ * frac;
        }
        if (thrust < 0.0f)            thrust = 0.0f;
        if (thrust > thrust_ceiling)  thrust = thrust_ceiling;
    } else if (current_mode_ >= FlightMode::ALT_HOLD) {
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
            // Auto-takeoff: the altitude cascade climbs toward alt_setpoint_ (the
            // target set at onTakeoff), velocity-limited to takeoff_climb_rate_ so the
            // ascent is gentle and the position loop DECELERATES near the target —
            // the craft captures the target altitude with no overshoot. When it settles
            // there (takeoff_reached_), the controller reports completion and the state
            // machine moves TAKEOFF→FLYING. The ToF 0.15m airborne edge is the ESKF
            // vertical handoff only (ImuTask), independent of this.
            // 自動離陸: 高度カスケードが alt_setpoint_（onTakeoff で設定した目標）へ上昇し、
            // 速度を takeoff_climb_rate_ に制限する — 上昇は穏やかで、目標近傍で位置ループが
            // 減速し、機体はオーバーシュートなく目標高度を捕捉する。そこに整定したら
            // （takeoff_reached_）制御器が完了を報告し、状態機械が TAKEOFF→FLYING を進める。
            // ToF 0.15m 空中エッジは ESKF 鉛直ハンドオフ専用（ImuTask）でこれとは独立。
            float vel_sp_z = alt_pos_.compute(alt_setpoint_, altitude, dt);
            // Clamp the climb/correction to the gentle takeoff rate (both directions):
            // the position loop decelerates as the target nears and corrects a small
            // overshoot DOWN to the target — capturing 0.5m exactly, not the peak. The
            // overshoot comes from the brief blind window below the ToF handoff (the
            // ESKF holds vertical velocity at 0 until ~0.15m), so we must let the loop
            // settle back, not clamp descent to zero.
            // 上昇/補正を穏やかな離陸率（両方向）にクランプ: 位置ループが目標近傍で減速し、
            // 小さな行き過ぎを目標まで下方修正する — ピークでなく 0.5m を正確に捕捉する。
            // 行き過ぎは ToF ハンドオフ未満の短いブラインド窓（ESKF は ~0.15m まで鉛直速度を
            // 0 に保持）に由来するため、降下を 0 にクランプせずループを戻らせる必要がある。
            if (vel_sp_z >  takeoff_climb_rate_) vel_sp_z =  takeoff_climb_rate_;
            if (vel_sp_z < -takeoff_climb_rate_) vel_sp_z = -takeoff_climb_rate_;
            float thrust_correction = alt_vel_.compute(vel_sp_z, vel_up, dt);
            thrust = hover_thrust_ + thrust_correction;
            if (thrust < 0.0f)         thrust = 0.0f;
            if (thrust > max_thrust_)  thrust = max_thrust_;

            // Takeoff-complete detection — robust ONE-SIDED reach + timeout backstop.
            // reached: the craft has CLIMBED to within kTakeoffCaptureBandM of the target
            // (altitude >= target - band), sustained briefly → fires on the way UP, immune
            // to a steady-state hover offset and to vertical-velocity noise (the old
            // two-sided band + low-velocity settle never fired on hardware, dead-sticking
            // roll/pitch — see pid_controller.hpp). The timeout guarantees we always leave
            // TakeoffClimb so the pilot regains attitude control even if "reached" never trips.
            // 離陸完了検出 — ロバストな片側到達＋タイムアウト・バックストップ。到達: 機体が目標の
            // kTakeoffCaptureBandM 以内まで上昇（altitude>=target-band）を短時間持続 → 上昇途中で
            // 発火し、定常ホバー偏差・鉛直速度ノイズに非依存（旧両側バンド+低速整定は実機で発火せず
            // ロール/ピッチを0固定した。pid_controller.hpp 参照）。タイムアウトは、到達が発火しなく
            // ても必ず TakeoffClimb を抜けてパイロットが姿勢制御を取り戻すことを保証する。
            ++takeoff_elapsed_cycles_;
            if (altitude >= takeoff_target_alt_ - kTakeoffCaptureBandM) {
                if (++takeoff_settle_cycles_ >= kTakeoffSettleCycles) {
                    takeoff_reached_ = true;
                }
            } else {
                takeoff_settle_cycles_ = 0;
            }
            if (takeoff_elapsed_cycles_ >= kTakeoffMaxCycles) {
                takeoff_reached_ = true;   // timeout backstop — never stay dead-sticked
            }
        } else {
            // Airborne: normal ALT_HOLD law.
            // 空中: 通常の ALT_HOLD 則。
            // Capture the altitude target when entering ALT_HOLD, and keep tracking
            // it while the throttle stick is off-center (climb/descend), so releasing
            // the stick holds the altitude actually reached.
            // ALT_HOLD 進入時に高度目標を捕捉し、スロットルが中央外（上昇/下降）の間は
            // 追従。スティックを戻すと到達した高度を保持する。
            if (capture_alt_) { alt_setpoint_ = altitude; capture_alt_ = false; }

            // Throttle stick → vertical command (symmetric, spring-centred throttle).
            // throttle_axis ∈ [-1,+1] with centre(2048)=0: centre = HOLD, up = climb (up
            // to max_climb_rate_), down = descend (up to max_descent_rate_) — the stick
            // raises/lowers the TARGET altitude (flight-proven legacy vehicle scheme).
            // スロットルスティック → 鉛直指令（対称、バネ中央スロットル）。throttle_axis ∈
            // [-1,+1]、中央(2048)=0: 中央=ホールド、上=上昇（max_climb_rate_ まで）、
            // 下=降下（max_descent_rate_ まで）— スティックで目標高度を上下する（旧 vehicle
            // 実績方式）。
            const float ta = setpoint.throttle_axis;

            // Re-center gate: after an (auto-)takeoff or an in-flight switch INTO ALT/POS
            // the stick may rest off-center (e.g. STABILIZE hover throttle is up), which
            // would jump the altitude. The gate suppresses the command until the stick
            // first returns to the center deadzone (= the spring rest), then opens — the
            // legacy "release the spring stick to unlock" behavior. Guidance/API own the
            // target via the walking setpoint and never reach this stick path.
            // 再センターゲート: （自動）離陸後や飛行中の ALT/POS 進入直後はスティックが中央
            // から外れていることがあり（例: STABILIZE のホバースロットルは上）、高度がジャンプ
            // する。ゲートはスティックが初めて中央デッドゾーン（=バネ静止）に戻るまで指令を抑え、
            // その後開く — 旧来の「バネ式は離せば解除」。誘導/API は歩く設定点で目標を所有し、
            // このスティック経路に達しない。
            if (!throttle_recentered_ && fabsf(ta) < stick_deadzone_) {
                throttle_recentered_ = true;
            }

            float climb_rate_sp = 0;
            if (throttle_recentered_ && !guidance_active_ &&
                fabsf(ta) > stick_deadzone_) {
                // Rescale beyond the deadzone to [0..1], then to the climb or descent
                // rate (separate limits). / デッドゾーン外を [0..1] に再スケールし上昇/降下
                // 速度（別リミット）へ。
                const float mag = (fabsf(ta) - stick_deadzone_) / (1.0f - stick_deadzone_);
                climb_rate_sp = (ta > 0.0f) ? (mag * max_climb_rate_)
                                            : (-mag * max_descent_rate_);
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
        if (phase_ != VerticalPhase::Airborne) {   // Landing is a phase, so this catches it too
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
                    // off-tone gyro power = the disturbance/noise floor at this frequency
                    // オフ音ジャイロ電力 = この周波数の外乱/雑音床
                    sysid_pending_.off_power = iq_yr_off_ * iq_yr_off_
                                             + iq_yi_off_ * iq_yi_off_;
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
        float y_ax = (excite_axis_ == 0) ? gyro_rate.x
                   : (excite_axis_ == 1) ? gyro_rate.y : gyro_rate.z;
        // Detrend: subtract a slow running mean of the rate so the near-DC disturbance
        // (CW/CCW trim) does not leak into the low-frequency lock-in. The LPF cutoff
        // (~0.5 Hz) is below the lowest tone (1.5 Hz), so only drift is removed. SEED the
        // mean to the current rate on the FIRST accumulated sample of each point (iq_n_==0)
        // so it starts converged — otherwise y_dc_ carries a STALE DC from the previous
        // tone/axis/run and biases this point's low-freq lock-in for ~0.3 s.
        // 除トレンド: 近DC外乱の低周波漏れを防ぐ。各点の最初の蓄積サンプルで現在のDCに再シードし収束済で
        // 開始（前の音/軸/実行の古いDCの持ち越しで低周波が偏るのを防ぐ）。
        constexpr float kDetrendAlpha = 0.008f;   // ~0.5 Hz cutoff at 400 Hz
        if (iq_n_ == 0) y_dc_ = y_ax;             // re-seed per point (no cross-tone/axis carry)
        y_dc_ += kDetrendAlpha * (y_ax - y_dc_);
        y_ax -= y_dc_;
        const float c = cosf(excite_phase_);
        const float sn = sinf(excite_phase_);
        iq_ur_ += u_ax * c;  iq_ui_ -= u_ax * sn;
        iq_yr_ += y_ax * c;  iq_yi_ -= y_ax * sn;
        // Off-tone lock-in at a nearby UNexcited frequency = the disturbance/noise FLOOR at
        // this frequency. coh = on/(on+off) (computed by the autotune) then down-weights
        // disturbance-dominated tones — the onboard SNR gate. The off-tone is f + max(27%,
        // 2 Hz) so it stays WELL-separated even at the lowest tones (2 Hz → 4 Hz, not 2.5);
        // otherwise on-tone energy leaks into off_power and wrongly DEPRESSES coh on clean
        // low-freq points (which would needlessly reject good roll/pitch data). Top tone
        // 35→44.5 Hz stays well below the 200 Hz Nyquist (no aliasing).
        // オフ音は f+max(27%,2Hz)で最低音でも十分離す（2Hz→4Hz）。漏れで clean 点の coh を誤って
        // 下げ、良好な roll/pitch を不要に棄却するのを防ぐ。最高音 44.5Hz は Nyquist 200Hz 未満。
        const float f_off = excite_freq_ + fmaxf(0.27f * excite_freq_, 2.0f);
        excite_phase_off_ += 2.0f * 3.14159265f * f_off * dt;
        const float co = cosf(excite_phase_off_);
        const float sno = sinf(excite_phase_off_);
        iq_yr_off_ += y_ax * co;  iq_yi_off_ -= y_ax * sno;
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

// -----------------------------------------------------------------------------
// learnTrim — always-on onboard attitude-trim learning (hover-gated)
// learnTrim — 常時オンボード姿勢トリム学習（ホバー限定）
//
// @design architecture.md INV-1/INV-2 — slow bias on the single trim, no parallel
//         path; pilot keeps full attitude authority                       [OK]
// -----------------------------------------------------------------------------
void PidController::learnTrim(const StateEstimate& state, const CommandSetpoint& setpoint,
                             float yaw, float dt)
{
    // Disabled by param (attitude.trim.learn = 0): no observation, no persist — use
    // when the optical flow is unreliable, or to tune the trim by hand only.
    // param (attitude.trim.learn = 0) で無効化: 観測も保存もしない — オプティカルフローが
    // 不安定なとき、または手動のみでトリムを詰めるときに使う。
    if (!trim_learn_enable_) return;

    // Persist the learned trim to NVS on the landing edge (Airborne OR Landing ->
    // Grounded — the autonomous descent passes through the Landing phase, so accept it
    // too, else a landing that goes Airborne->Landing->Grounded never persists).
    // Touchdown (Grounded) is the safe moment: thrust is gated, so the one-shot flash
    // write cannot disturb flight. NOTE: this set_float+save runs in ControlTask; it is
    // a single event at touchdown (not per-cycle), so the ~37ms flash stall lands after
    // touchdown and is harmless. A stricter R5/R7-clean design would route it via a
    // DISARM topic to a dedicated persister (future TODO).
    // 学習トリムを着陸エッジ（Airborne または Landing -> Grounded。自動降下は Landing
    // フェーズを通るので両方受理。さもないと Airborne->Landing->Grounded の着陸が永続
    // しない）で NVS 保存。接地（Grounded）は安全な瞬間: 推力はゲート済みゆえ単発フラッシュ
    // 書込が飛行を乱さない。注: この set_float+save は ControlTask で走るが接地時の単発
    // （毎サイクルでない）ゆえ ~37ms フラッシュ停止は接地後で無害。厳密な R5/R7 準拠は
    // DISARM トピック経由で専用永続化タスクに回す（将来 TODO）。
    if ((trim_prev_phase_ == VerticalPhase::Airborne ||
         trim_prev_phase_ == VerticalPhase::Landing) &&
        phase_ == VerticalPhase::Grounded) {
        params::set_float("attitude.roll.trim",  roll_trim_);
        params::set_float("attitude.pitch.trim", pitch_trim_);
        params::save();
    }
    trim_prev_phase_ = phase_;

    // Gate: learn only in a hands-near-neutral hover in STABILIZE/ALT_HOLD (NOT
    // POS_HOLD — its position loop already cancels drift) while airborne and not under
    // guidance. Deliberate translation/turn (a stick out of the deadband) pauses
    // learning so a commanded move is not mistaken for trim error.
    // ゲート: STABILIZE/ALT_HOLD のスティックほぼ中立ホバーでのみ学習（POS_HOLD 除外＝
    // 位置ループが既にドリフトを打ち消す）、空中かつ誘導なし。意図的な移動/旋回
    // （スティックが不感帯外）は学習を止め、指令移動をトリム誤差と誤認しない。
    // ALT_HOLD also requires a NEUTRAL vertical stick: an active climb/descent
    // (throttle_axis off-centre) tilts the craft to track the rate against wind and
    // would corrupt the horizontal-drift observation. (STABILIZE uses direct throttle,
    // which does not couple into the horizontal axes the same way — left ungated.)
    // ALT_HOLD は鉛直スティックも中立を要求: 上昇/下降中（throttle_axis が中央外）は風に
    // 抗してレート追従するため機体が傾き、水平ドリフト観測を汚す。（STABILIZE は直接
    // スロットルで水平軸へのカップリングが異なるためゲートしない。）
    const bool hovering =
        (current_mode_ == FlightMode::STABILIZE || current_mode_ == FlightMode::ALT_HOLD) &&
        phase_ == VerticalPhase::Airborne &&
        !guidance_active_ &&
        fabsf(setpoint.roll)  < kTrimLearnStickDead &&
        fabsf(setpoint.pitch) < kTrimLearnStickDead &&
        fabsf(setpoint.yaw)   < kYawHoldStickDeadband &&
        (current_mode_ != FlightMode::ALT_HOLD ||
         fabsf(setpoint.throttle_axis) < kTrimLearnStickDead);
    if (!hovering) { trim_learn_init_ = false; return; }

    // First hover sample: seed the velocity reference and skip one step (no accel yet).
    // 初回ホバーサンプル: 速度基準を仕込み 1 ステップ飛ばす（加速度未確定）。
    if (!trim_learn_init_) {
        trim_vel_prev_[0] = state.velocity[0];
        trim_vel_prev_[1] = state.velocity[1];
        trim_accel_lpf_[0] = 0.0f;
        trim_accel_lpf_[1] = 0.0f;
        trim_learn_init_ = true;
        return;
    }
    if (dt <= 0.0f) return;

    // Horizontal accel = d/dt of the ESKF NED velocity (no accel state in
    // StateEstimate), rotated to the body FRD frame by yaw (same rotation as
    // computePositionHold), then EMA-smoothed to reject differentiation noise.
    // 水平加速度 = ESKF NED 速度の微分（StateEstimate に加速度状態なし）を yaw で機体
    // FRD へ回転（computePositionHold と同一）、微分ノイズ除去に EMA 平滑。
    const float a_north = (state.velocity[0] - trim_vel_prev_[0]) / dt;
    const float a_east  = (state.velocity[1] - trim_vel_prev_[1]) / dt;
    trim_vel_prev_[0] = state.velocity[0];
    trim_vel_prev_[1] = state.velocity[1];

    const float cy = cosf(yaw), sy = sinf(yaw);
    const float ax_body =  cy * a_north + sy * a_east;   // forward (FRD X) / 前方
    const float ay_body = -sy * a_north + cy * a_east;   // right   (FRD Y) / 右

    const float alpha = 1.0f - expf(-2.0f * 3.14159265f * kTrimLearnAccelHz * dt);
    trim_accel_lpf_[0] += alpha * (ax_body - trim_accel_lpf_[0]);
    trim_accel_lpf_[1] += alpha * (ay_body - trim_accel_lpf_[1]);

    // Integrate the smoothed drift into the trim with time constant kTrimLearnTau.
    // Signs match sf trim analyze (SIL-verified): forward drift -> +pitch (nose up
    // brakes it), right drift -> -roll. First-order: trim -> equilibrium as 1/tau.
    // 平滑ドリフトを時定数 kTrimLearnTau でトリムに積分。符号は sf trim analyze（SIL
    // 検証済み）と一致: 前方ドリフト -> +pitch（機首上げで制動）、右 -> -roll。1次系。
    const float k = dt / kTrimLearnTau;
    pitch_trim_ += k * (trim_accel_lpf_[0] / gravity_);
    roll_trim_  -= k * (trim_accel_lpf_[1] / gravity_);

    // Clamp to the param range (+/-kTrimMax). / param 範囲（±kTrimMax）にクランプ。
    roll_trim_  = fminf(fmaxf(roll_trim_,  -kTrimMax), kTrimMax);
    pitch_trim_ = fminf(fmaxf(pitch_trim_, -kTrimMax), kTrimMax);
}

void PidController::computePositionHold(const StateEstimate& state,
                                        const CommandSetpoint& setpoint, float yaw,
                                        float dt, float& roll_sp, float& pitch_sp)
{
    const float cy = cosf(yaw), sy = sinf(yaw);

    // Stick repositioning (deflect to move, release to hold). Roll/pitch sticks command a
    // horizontal velocity in the BODY frame; the sign matches STABILIZE tilt (roll
    // right → move right, pitch forward → move forward), so the craft moves the way
    // the same stick would tilt it by hand. Sticks are deadbanded upstream, so a
    // centred stick is exactly 0 = "hold".
    // スティック再配置（倒して動かし、離して保持）。roll/pitch スティックが機体座標の水平
    // 速度を指令。符号は STABILIZE の傾き方向と一致（右ロール→右、前ピッチ→前）させ、手で
    // 傾けるのと同じ向きに動く。スティックは上流で不感帯処理済ゆえ中央は厳密に 0 =「保持」。
    const float v_fwd   = -setpoint.pitch * stick_reposition_vel_;   // forward (FRD x) [m/s]
    const float v_right =  setpoint.roll  * stick_reposition_vel_;   // right   (FRD y) [m/s]
    const bool  repositioning = (v_fwd != 0.0f || v_right != 0.0f);

    float vx_sp, vy_sp;   // desired NED velocity (N, E) fed to the velocity loop
    if (repositioning) {
        // The pilot drives: bypass the position loop and feed the stick velocity
        // (body FRD → NED) straight to the velocity loop. Reset the position-loop
        // integrators (unused here, must not wind up) and keep the hold target pinned
        // to the current position, so it "sticks" wherever the craft is when released.
        // パイロットが操縦: 位置ループを迂回し、スティック速度（機体 FRD→NED）を速度ループへ
        // 直接渡す。位置ループ積分はリセット（未使用・巻き上がり防止）し、保持目標は現在位置に
        // 固定し続ける → 離した瞬間の位置で止まる。
        vx_sp = cy * v_fwd - sy * v_right;   // NED N
        vy_sp = sy * v_fwd + cy * v_right;   // NED E
        pos_x_.reset();
        pos_y_.reset();
        pos_setpoint_x_ = state.position[0];
        pos_setpoint_y_ = state.position[1];
        reposition_active_ = true;
    } else {
        // Hold. On the repositioning→neutral edge, capture where we stopped as the new
        // target (the craft's momentum may carry it slightly past — the position loop
        // pulls it back). capture_pos_ is also set on POS_HOLD entry (onModeChange /
        // onTakeoff) and after a guidance cancel.
        // 保持。再配置→中立のエッジで、止まった位置を新目標として捕捉する（慣性で少し行き過ぎ
        // ても位置ループが引き戻す）。capture_pos_ は POS_HOLD 進入（onModeChange / onTakeoff）
        // と誘導解除後にも立つ。
        if (reposition_active_) {
            capture_pos_ = true;
            reposition_active_ = false;
        }
        if (capture_pos_) {
            pos_setpoint_x_ = state.position[0];
            pos_setpoint_y_ = state.position[1];
            capture_pos_ = false;
        }
        // Outer loop (NED): position error → desired horizontal velocity.
        // 外ループ（NED）: 位置誤差 → 目標水平速度。
        vx_sp = pos_x_.compute(pos_setpoint_x_, state.position[0], dt);
        vy_sp = pos_y_.compute(pos_setpoint_y_, state.position[1], dt);
    }

    // Inner loop (NED): velocity error → desired horizontal acceleration.
    // 内ループ（NED）: 速度誤差 → 目標水平加速度。
    const float ax_ned = vel_x_.compute(vx_sp, state.velocity[0], dt);
    const float ay_ned = vel_y_.compute(vy_sp, state.velocity[1], dt);

    // Rotate the desired NED acceleration into the body frame (yaw only).
    // 目標 NED 加速度を機体座標へ回転（ヨーのみ）。
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

void PidController::onLanding()
{
    if (phase_ == VerticalPhase::Landing) {
        return;   // already landing (idempotent) / 既に着陸中（冪等）
    }
    ESP_LOGW(TAG, "Autonomous landing engaged (%.1f m/s descent)",
             static_cast<double>(landing_descent_rate_));
    // Landing is a VerticalPhase (INV-1) — compute() then descends and keeps pilot
    // attitude (or levels on a stale link). NOT a separate control path.
    // Landing は VerticalPhase（INV-1）— 以後 compute() が降下しつつ姿勢を保つ（リンク
    // 途絶なら水平化）。別の制御経路ではない。
    phase_ = VerticalPhase::Landing;
    landing_settle_t_ = 0.0f;   // near-ground settle ramp starts fresh / 着地アシストのランプを初期化

    // Fresh start for the loops the landing law uses: the attitude loops may carry
    // integrator state from a different mode, and the vertical loop may have wound up
    // against a saturated climb.
    // 着陸則が使うループを仕切り直す: 姿勢ループは別モードの積分状態を、鉛直ループは
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
    ESP_LOGI(TAG, "Auto-takeoff engaged (%.1f m/s climb → %.2f m)",
             static_cast<double>(takeoff_climb_rate_),
             static_cast<double>(takeoff_target_alt_));
    phase_ = VerticalPhase::TakeoffClimb;

    // Fresh vertical loops (they may hold a Grounded-phase zero-output history), set
    // the climb target, and capture the launch point for POS_HOLD (the cascade starts
    // at the next compute). The throttle re-center gate is closed: the pilot must pass
    // the stick through center before it commands altitude (Case A — anti-jump).
    // The takeoff-complete signal starts fresh.
    // 鉛直ループを仕切り直し（Grounded フェーズのゼロ出力履歴を持ちうる）、上昇目標を設定し、
    // POS_HOLD 用に発進点を捕捉（カスケードは次の compute から動く）。スロットル再センター
    // ゲートは閉: パイロットがスティックを中央に通すまで高度を指令しない（Case A — ジャンプ
    // 防止）。離陸完了信号は初期化する。
    alt_pos_.reset();
    alt_vel_.reset();
    alt_setpoint_ = takeoff_target_alt_;   // climb toward the target / 目標へ上昇
    capture_pos_  = true;
    throttle_recentered_    = false;
    takeoff_reached_        = false;
    takeoff_settle_cycles_  = 0;
    takeoff_elapsed_cycles_ = 0;
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

    // ALT_HOLD holds the TARGET altitude that TakeoffClimb already captured
    // (alt_setpoint_ == takeoff_target_alt_) — NOT the instantaneous altitude, so any
    // residual climb momentum cannot bias the hold height (decision ②: capture at the
    // target value, not the overshoot). The vertical-loop integral is kept (it spooled
    // to near-hover thrust — a smooth handoff). The takeoff-complete signal is cleared
    // now that the state machine has consumed it.
    // ALT_HOLD は TakeoffClimb が既に捕捉した「目標」高度（alt_setpoint_ ==
    // takeoff_target_alt_）を保持する — 瞬時高度ではない。残留する上昇運動量が保持高度を
    // バイアスしない（確定②: 行き過ぎでなく目標値で捕捉）。鉛直ループの積分は維持
    // （ほぼホバー推力まで巻き上がり、滑らかな引き継ぎ）。状態機械が消費済みの離陸完了
    // 信号はここでクリアする。
    takeoff_reached_        = false;
    takeoff_settle_cycles_  = 0;
    takeoff_elapsed_cycles_ = 0;
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
    if (phase_ != VerticalPhase::Airborne) {   // Landing/Takeoff/Grounded are not Airborne
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
    // Reset the off-tone accumulator/phase per point. (y_dc_ for the detrend is re-seeded
    // to the current rate on the first accumulated sample — see the I/Q block — so it
    // never carries a stale DC across tones/axes/runs.)
    // オフ音蓄積/位相は点ごとにリセット。（除トレンドの y_dc_ は最初の蓄積サンプルで現DCに再シード。）
    excite_phase_off_ = 0.0f;
    iq_yr_off_ = iq_yi_off_ = 0.0f;
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
    phase_   = VerticalPhase::Grounded;  // next flight starts grounded; clears Landing too / 次の飛行は接地から（Landing も解除）
    landing_settle_t_ = 0.0f;            // near-ground settle ramp / 着地アシストのランプ
    guidance_active_ = false;            // guidance dies with the flight / 誘導も飛行と共に終了
    reposition_active_ = false;          // stick repositioning state clears / スティック再配置状態クリア
    excite_active_   = false;            // so does the excitation / 励振も同様
    yaw_hold_active_ = false;            // heading hold too / ヘディングホールドも同様
    throttle_recentered_    = false;     // re-center gate re-arms for the next takeoff / 次の離陸用にゲート再武装
    takeoff_reached_        = false;      // takeoff-complete signal clears / 離陸完了信号クリア
    takeoff_settle_cycles_  = 0;
    takeoff_elapsed_cycles_ = 0;
    trim_learn_init_ = false;                    // re-seed the trim-learner filter / トリム学習フィルタ再初期化
    trim_prev_phase_ = VerticalPhase::Grounded;  // landing-edge sync / 着陸エッジ整合
    // NOTE: roll_trim_/pitch_trim_ are NOT cleared — the learned/loaded trim persists
    // across resets (config, not integrator state). / roll_trim_/pitch_trim_ はクリア
    // しない — 学習/読込トリムは reset を跨いで保持（積分器状態でなく構成値）。
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
        // Entering ALT_HOLD/POS_HOLD in flight (Case B): capture the current altitude
        // as the hold target, and close the throttle re-center gate so the stick must
        // pass through center before it commands climb/descent — the stick is at an
        // arbitrary position at the switch and must not jump the altitude (decision ②).
        // 飛行中の ALT_HOLD/POS_HOLD 進入（Case B）: 現在高度を保持目標として捕捉し、
        // スロットル再センターゲートを閉じる — スティックは切替時に任意位置にあり、
        // 高度をジャンプさせないため中央を通すまで上昇/下降を指令させない（確定②）。
        if (new_mode >= FlightMode::ALT_HOLD && current_mode_ < FlightMode::ALT_HOLD) {
            capture_alt_         = true;
            throttle_recentered_ = false;
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
