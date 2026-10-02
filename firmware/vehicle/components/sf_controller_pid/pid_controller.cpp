/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
 * @design architecture.md INV-1 — alt_vel_ integral time scheduled  [OK]
 *         by vertical phase only (climb/hover); see applyAltVelTiForPhase()
 * @design analysis/scripts/alt_dob_design/README.md §5 — accel-based [OK]
 *         disturbance observer (DOB), altitude vel loop, opt-in via
 *         altitude.dob.fc (0=off); Airborne-only, INV-1 vertical channel
 *         only; see computeDobCorrection()/resetDobStates()
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
    // ようにループの出力に急変を与えてはならない。
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
    // Yaw torque cap (NT-Kanazawa saturation treatment; see params.cpp)
    // ヨートルク上限（NT金沢飽和の治療。params.cpp 参照）
    params::get_float("rate.yaw.max_torque", max_yaw_torque_);

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
    // alt_vel_.ti is phase-scheduled (climb vs. hover), NOT set directly here —
    // load both candidates into the cache and let applyAltVelTiForPhase() (below)
    // pick per phase_. Position loop (alt_pos_) ti is not scheduled: the
    // disturbance enters the inner (thrust) loop, so strengthening position
    // integral does not help it (see plan doc, not repeated here).
    // alt_vel_.ti はフェーズ別スケジュール（climb/hover）— ここでは直接設定せず、
    // 両候補をキャッシュへロードし、下の applyAltVelTiForPhase() が phase_ に応じて
    // 選択する。位置ループ(alt_pos_)の ti はスケジュールしない（外乱は内側の推力
    // ループに入るため位置積分を強めても効かない）。
    params::get_float("altitude.vel.ti", alt_vel_ti_climb_);
    params::get_float("altitude.vel.ti_hover", alt_vel_ti_hover_);
    // Manual ALT_HOLD stick rates (separately tunable) / 手動 ALT_HOLD スティック速度（別々に調整可）
    params::get_float("altitude.climb_rate",   max_climb_rate_);
    params::get_float("altitude.descent_rate", max_descent_rate_);

    // Acceleration-based disturbance observer (DOB) for the Airborne altitude
    // vel loop. fc<=0 (default) disables it entirely (opt-in via `param set
    // altitude.dob.fc <Hz>`); range-guard [kDobFcMin,kDobFcMax] outside the
    // sim-validated band. See computeDobCorrection() and
    // analysis/scripts/alt_dob_design/README.md §5 for the design.
    // 高度速度ループ(Airborne)用の加速度ベース外乱オブザーバ(DOB)。fc<=0（既定）
    // で完全無効（`param set altitude.dob.fc <Hz>` で opt-in）。シム検証済み帯域
    // [kDobFcMin,kDobFcMax] 外は範囲ガード。computeDobCorrection() と
    // analysis/scripts/alt_dob_design/README.md §5 参照。
    float dob_fc = 0.0f;
    params::get_float("altitude.dob.fc", dob_fc);
    dob_enabled_ = (dob_fc > 0.0f);
    if (dob_enabled_) {
        if (dob_fc < kDobFcMin || dob_fc > kDobFcMax) {
            ESP_LOGW(TAG, "altitude.dob.fc %.2f out of [%.2f, %.2f] Hz, clamping",
                     static_cast<double>(dob_fc),
                     static_cast<double>(kDobFcMin), static_cast<double>(kDobFcMax));
            dob_fc = fminf(fmaxf(dob_fc, kDobFcMin), kDobFcMax);
        }
        computeDobQCoeffs(dob_fc);
    }

    // Hover thrust correction: hover_thrust = mg × corr. Tunable so a motor/prop
    // change (e.g. fresh, stronger motors) can be matched WITHOUT a rebuild — drop
    // corr when the craft over-climbs on auto-takeoff. mg = 0.037 kg · 9.80665.
    // ホバー推力補正: hover_thrust = mg × corr。モータ/プロペラ交換（新品=強い等）に
    // 再ビルドなしで合わせられる — 自動離陸で過上昇するなら corr を下げる。
    float hover_corr = 1.12f;
    params::get_float("hover.thrust_corr", hover_corr);
    hover_thrust_ = kMassG * hover_corr;   // kMassG is a class constant (see header)
    // Onboard hover-thrust learning enable (1 = learn the true hover thrust in flight, 0 =
    // manual hover.thrust_corr only). See learnHoverThrust().
    // オンボード・ホバー推力学習の有効化（1 = 飛行中に真のホバー推力を学習, 0 = 手動 corr のみ）。
    int32_t hover_learn = 1;
    params::get_int("hover.thrust.learn", hover_learn);
    hover_learn_enable_ = (hover_learn != 0);

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

    // Flip maneuver (docs/plans/flip-maneuver-plan.md §3.4/§5.3 final
    // recommendation). max_thrust_n is NOT a `flip.*` NVS param — it mirrors
    // this controller's own max_thrust_ so FlipSequencer's thrust ratios
    // convert to newtons without depending on PidController (see
    // FlipSequencer::Config's doc).
    // 宙返りマニューバ（plan §3.4/§5.3 最終推奨）。max_thrust_n は `flip.*` の
    // NVS パラメータではない — 本制御器自身の max_thrust_ を複写し、
    // FlipSequencer の推力比率が PidController に依存せず [N] に変換できる
    // ようにする（FlipSequencer::Config のドキュメント参照）。
    // Master switch (default OFF). Read here so `param set flip.enable` applies
    // live through notifyControllerReload, like every other flip.* param.
    // マスタースイッチ（既定OFF）。他の flip.* と同様、`param set flip.enable` が
    // notifyControllerReload 経由で即時反映されるようここで読む。
    int32_t flip_enable = 0;
    params::get_int("flip.enable", flip_enable);
    flip_enabled_ = (flip_enable != 0);
    params::get_float("flip.rate_roll_dps",     flip_.config.rate_roll_dps);
    params::get_float("flip.rate_pitch_dps",    flip_.config.rate_pitch_dps);
    params::get_float("flip.rate_ramp_rps2",    flip_.config.rate_ramp_rps2);
    int32_t flip_profile_mode = 0;
    params::get_int("flip.profile_mode", flip_profile_mode);
    flip_.config.profile_mode = static_cast<int>(flip_profile_mode);
    params::get_float("flip.ff_gain",           flip_.config.ff_gain);
    params::get_float("flip.boost_ms",          flip_.config.boost_ms);
    params::get_float("flip.thrust_boost_ratio",   flip_.config.thrust_boost_ratio);
    params::get_float("flip.thrust_spin_hi_ratio", flip_.config.thrust_spin_hi_ratio);
    params::get_float("flip.thrust_lo_n",       flip_.config.thrust_lo_n);
    params::get_float("flip.angle_a_deg",       flip_.config.angle_a_deg);
    params::get_float("flip.motor_lag_ms",      flip_.config.motor_lag_ms);
    params::get_float("flip.brake_margin",      flip_.config.brake_margin);
    params::get_float("flip.recover_boost_tilt_deg", flip_.config.recover_boost_tilt_deg);
    params::get_float("flip.recover_rate_limit_dps", flip_.config.recover_rate_limit_dps);
    params::get_float("flip.spin_yaw_torque_limit_nm", flip_.config.spin_yaw_torque_limit_nm);
    params::get_float("flip.spin_torque_limit_nm", flip_.config.spin_torque_limit_nm);
    params::get_float("flip.brake_ramp_rps2",   flip_.config.brake_ramp_rps2);
    params::get_float("flip.settle_ms",         flip_.config.settle_ms);
    params::get_float("flip.settle_tilt_deg",   flip_.config.settle_tilt_deg);
    params::get_float("flip.handoff_rate_dps",  flip_.config.handoff_rate_dps);
    params::get_float("flip.handoff_force_deg", flip_.config.handoff_force_deg);
    params::get_float("flip.brake_timeout_ms",  flip_.config.brake_timeout_ms);
    params::get_float("flip.recover_timeout_ms", flip_.config.recover_timeout_ms);
    params::get_float("flip.spin_timeout_ms",   flip_.config.spin_timeout_ms);
    params::get_float("flip.gyro_abort_dps",    flip_.config.gyro_abort_dps);
    params::get_float("flip.min_height_m",      flip_.config.min_height_m);
    params::get_float("flip.min_voltage_v",     flip_.config.min_voltage_v);
    params::get_float("flip.max_tilt_deg",      flip_.config.max_tilt_deg);
    params::get_float("flip.max_rate_dps",      flip_.config.max_rate_dps);
    params::get_float("flip.max_hvel_mps",      flip_.config.max_hvel_mps);
    params::get_float("flip.max_vvel_mps",      flip_.config.max_vvel_mps);
    params::get_float("flip.cooldown_ms",       flip_.config.cooldown_ms);
    flip_.config.max_thrust_n = max_thrust_;

    // Re-apply the phase schedule so a mid-flight param reload cannot leave
    // alt_vel_.ti stale relative to the (unchanged) phase_.
    // 飛行中の param 再読込でも alt_vel_.ti が（不変の）phase_ に対して古いままに
    // ならないよう、フェーズスケジュールを再適用する。
    applyAltVelTiForPhase();
    // DOB states depend on the (possibly just-changed) fc/coefficients and on
    // hover_thrust_ (just computed above) — always re-seed on a param load, the
    // same equilibrium-init discipline as every phase transition (see
    // resetDobStates() doc).
    // DOB状態は（変わったかもしれない）fc/係数と（直前で計算した）hover_thrust_
    // に依存する — param 再読込では常に再シードする。全フェーズ遷移と同じ
    // 平衡初期化の作法（resetDobStates() のコメント参照）。
    resetDobStates(hover_thrust_);
}

// Schedule the vertical-velocity loop integral time by phase: strong integral
// (short Ti) only in Airborne hover to reject the low-freq battery-sag disturbance;
// gentle Ti in TakeoffClimb/Landing/Grounded to bound windup and capture overshoot.
// Output is continuous across the switch (PI out = P + integral; Ti changes only the
// integral increment, td=0). @design architecture.md INV-1 — vertical channel only [OK]
// 鉛直速度ループの積分時間をフェーズ別に適用。ホバーのみ強い積分で低周波外乱を除去、
// 離陸/着陸は穏やかな積分で巻き上がり・捕捉オーバーシュートを抑える。Ti 切替は積分値を
// 触らず増分のみ変えるため出力連続。
void PidController::applyAltVelTiForPhase()
{
    alt_vel_.ti = (phase_ == VerticalPhase::Airborne)
                      ? alt_vel_ti_hover_ : alt_vel_ti_climb_;
}

// Hold the attitude (roll/pitch) and rate (roll/pitch/yaw) integrators while Grounded.
// WHY: on the floor the craft cannot follow its setpoints, so any residual tilt-estimate
// error (~0.1-0.3 deg) is integrated by the attitude PID into a rate setpoint the rate
// loop can never reach, which winds the rate integrator up to its torque limit (real log
// ground_spin_stab, STABILIZE ARMED_GROUND 34 s: roll rate-I saturated at 5.2 mNm, motor
// duty spread 0.458 while thrust was ~0.1). Mixer has no priority, so that torque swamps
// the thrust and the craft leaves the floor with a wound-up integrator (tens of degrees of
// roll at liftoff). Holding keeps the P/D action and the zero state set by reset() at ARM.
// Same pipeline and law in every phase (INV-1): only the integrator update is gated.
// Grounded ends at TakeoffComplete (STABILIZE/ACRO: ToF airborne; ALT/POS: onTakeoff()).
// 地上（Grounded）の間は姿勢（roll/pitch）・レート（roll/pitch/yaw）の積分器を保持する。
// 理由: 床の上では機体は設定点に追従できず、傾き推定の残差（~0.1-0.3 deg）が姿勢PIDで
// 積分され、レートループが到達できないレート設定点となり、レート積分器をトルク上限まで
// 巻き上げる（実ログ ground_spin_stab、STABILIZE ARMED_GROUND 34 s: ロールのレート積分が
// 5.2 mNm で飽和、推力 ~0.1 のままモータ duty 差 0.458）。ミキサーに優先順位はなく、そのトルクが
// 推力を圧倒し、巻き上がった積分器を持ったまま離床する（離床時にロール数十度）。保持しても
// P/D 動作と ARM 時の reset() によるゼロ状態は保たれる。全フェーズで同一パイプライン・
// 同一制御則（INV-1）: 変わるのは積分器の更新可否のみ。Grounded は TakeoffComplete で終わる
// （STABILIZE/ACRO: ToF 空中検知、ALT/POS: onTakeoff()）。
void PidController::applyGroundIntegratorHold()
{
    const bool integrate = (phase_ != VerticalPhase::Grounded);
    att_roll_.integration_enabled   = integrate;
    att_pitch_.integration_enabled  = integrate;
    rate_roll_.integration_enabled  = integrate;
    rate_pitch_.integration_enabled = integrate;
    rate_yaw_.integration_enabled   = integrate;
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

    // Hold the attitude/rate integrators while Grounded (floor-constrained windup).
    // Grounded の間は姿勢/レート積分器を保持（床拘束による巻き上がり防止）。
    applyGroundIntegratorHold();

    // Flip maneuver (docs/plans/flip-maneuver-plan.md §3/§4/§7).
    //
    // Deferred start: onFlip(dir) cannot capture "current" yaw/height itself
    // (it receives no StateEstimate — matching onTakeoff()/onLanding()), so it
    // only latches flip_start_pending_; the actual flip_.start() happens HERE,
    // on the first compute() cycle after onFlip() (same one-cycle-deferred-
    // capture idiom as capture_alt_/capture_pos_ elsewhere in this file).
    //
    // Readiness (C2-C8) is re-evaluated every cycle regardless of mode/phase
    // into flip_ready_/flip_reason_ — isFlipReady() is const and reads that
    // cache (same reason isTakeoffComplete()/isGuidanceActive() read cached
    // flags instead of computing on demand). The natural gates (height, tilt,
    // rate, velocity, cooldown) keep it false outside a stable Airborne hover
    // without this controller needing to know FlightState.
    //
    // If a flip is active(), the sequencer is advanced exactly once per cycle
    // and its output is consumed below at the attitude confluence and the
    // vertical channel INSTEAD OF the normal cascade — the sequencer only
    // switches the setpoint SOURCE, never the control law (INV-1).
    //
    // 宙返りマニューバ（plan §3/§4/§7）。
    //
    // 遅延開始: onFlip(dir) は自ら「現在の」ヨー/高度を取り込めない
    // （StateEstimate を受け取らない — onTakeoff()/onLanding()と同型）ため
    // flip_start_pending_ を立てるだけ。実際の flip_.start() は「ここ」、
    // onFlip() 後の最初の compute() 周期で行う（本ファイル他所の
    // capture_alt_/capture_pos_ と同じ1周期遅延取り込みの作法）。
    //
    // 実行条件（C2-C8）はモード/フェーズに関わらず毎周期
    // flip_ready_/flip_reason_ へ再評価する — isFlipReady() は const で
    // このキャッシュを読む（isTakeoffComplete()/isGuidanceActive() が
    // オンデマンド計算でなくキャッシュ済みフラグを読むのと同じ理由）。
    // 判定自体（高度・傾き・角速度・速度・クールダウン）が、この制御器が
    // FlightState を知らなくても、安定した空中ホバー以外では自然に false に
    // 保つ。
    //
    // フリップが active() なら、列生成器を1周期に1回だけ進め、その出力を
    // 下の姿勢合流点と鉛直チャネルで「通常カスケードの代わりに」使う —
    // 列生成器は設定点の「出所」だけを切り替え、制御則は変えない（INV-1）。
    if (flip_start_pending_) {
        flip_.start(flip_pending_dir_, euler.z, -state.position[2]);
        flip_start_pending_ = false;
    }
    updateFlipReadiness(state, dt);

    FlipSequencer::Output flip_out{};
    const bool flip_active = flip_.active();
    if (flip_active) {
        flip_out = flip_.update(buildFlipInput(state, dt));
        trackFlipPhaseTransition(flip_.phase());
    } else if (flip_settle_remaining_s_ > 0.0f) {
        flip_settle_remaining_s_ -= dt;   // post-flip settle window (Config::settle_ms) / 宙返り後の整定窓
    }
    const bool flip_settling = !flip_active && flip_settle_remaining_s_ > 0.0f;

    // Attitude-loop output cap: the flip's own (Config::recover_rate_limit_dps)
    // while a flip is active, normal flight's otherwise. Assigned every cycle, so
    // there is no state to forget to restore on any exit from FLIP.
    // 姿勢ループの出力上限: 宙返り中は宙返り専用（Config::recover_rate_limit_dps）、
    // それ以外は通常飛行の値。毎周期代入するので、FLIP のどの出口でも戻し忘れる
    // 状態がない。
    constexpr float kFlipDegToRad = 3.14159265358979f / 180.0f;
    const float att_rate_limit = flip_active
        ? flip_.config.recover_rate_limit_dps * kFlipDegToRad : max_att_rate_sp_;
    att_roll_.output_limit  = att_rate_limit;
    att_pitch_.output_limit = att_rate_limit;

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
    // 姿勢カスケードは STABILIZE 以上、加えて任意モードの Landing 中も動かす — 自動着陸は
    // ACRO から始まっても（通信途絶着陸）姿勢安定化が必要。INV-1: 全フェーズ単一姿勢経路。
    //
    // FLIP takes the WHOLE block's place instead of running inside it: Spin/
    // Brake bypass the attitude PID entirely (not just override its result) so
    // its integrators cannot wind up against the meaningless mid-flip Euler-
    // angle error, and Boost/Recover reuse att_roll_/att_pitch_ through a
    // dedicated small helper instead of the stick/POS_HOLD/trim/yaw-hold
    // machinery below (the pilot/guidance/trim setpoint sources are all
    // ignored during FLIP — flip-maneuver-plan.md §4.1).
    // FLIP はこのブロック「全体」の代わりになる（内部で動くのではない）:
    // Spin/Brake は姿勢 PID を完全に迂回する（結果の上書きではない）ため、
    // フリップ中の無意味なオイラー角誤差で積分器が巻き上がらない。
    // Boost/Recover は att_roll_/att_pitch_ を専用の小さなヘルパ経由で
    // 再利用し、下のスティック/POS_HOLD/トリム/ヘディングホールド機構は
    // 使わない（パイロット/誘導/トリムの設定点はいずれも FLIP 中は無視 —
    // plan §4.1）。
    if (flip_active) {
        computeFlipAttitude(state, flip_out, dt, rate_sp_roll, rate_sp_pitch,
                            rate_sp_yaw, roll_sp, pitch_sp);
    } else if (current_mode_ >= FlightMode::STABILIZE ||
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
        // そのフェーズに留まるとロール/ピッチが応答しなくなった（実機バグ 2026-06-14）。どの状態でも姿勢を
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
            if (guide_mode_ == 2) {
                // Velocity guidance (Tello `rc`). R16 staleness auto-release: if the
                // client stopped sending, decay the velocities to 0 so the craft
                // re-captures and HOLDS (a stopped rc must not run away). The
                // horizontal velocity is injected in computePositionHold (it reuses
                // the stick-reposition path) and the climb rate in the vertical block;
                // here we only drive yaw RATE directly (cw+).
                // 速度誘導（Tello `rc`）。R16 鮮度オートリリース: クライアントが送信を止めたら
                // 速度を 0 に減衰し再捕捉・保持（止まった rc で速度指令が増大し続けてはならない）。水平速度は
                // computePositionHold（スティック再配置経路の再利用）、上昇率は鉛直ブロックで注入。
                // ここでは yaw レート（cw+）のみ直接駆動する。
                if (state.timestamp - guide_stamp_ > kLandingLinkStaleUs) {
                    guide_vx_ = guide_vy_ = guide_vz_ = guide_vyaw_ = 0.0f;
                }
                rate_sp_yaw = guide_vyaw_;
            } else {
                // Position guidance (mode 1): walk the setpoints toward the target.
                // 位置誘導（mode 1）: 設定点を目標へ歩かせる。
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

                // Yaw: shortest-path error, P with a turn-rate limit (shared
                // formula — see shortestPathYawRate()).
                // ヨー: 最短経路誤差の P、回頭率制限付き（共有の式 —
                // shortestPathYawRate() 参照）。
                rate_sp_yaw = shortestPathYawRate(guide_yaw_, euler.z,
                                                  guide_yaw_kp_, guide_yaw_rate_max_);
            }
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
                // Shortest-path heading error, P with a turn-rate limit
                // (shared formula — see shortestPathYawRate()). This exact
                // gain pair is reused, unmodified, by the FLIP Boost/Recover
                // heading hold in computeFlipAttitude().
                // 最短経路の方位誤差、回頭率制限付き P（共有の式 —
                // shortestPathYawRate() 参照）。この同じゲイン対は
                // computeFlipAttitude() の FLIP Boost/Recover ヘディング
                // ホールドでも無改変で再利用する。
                rate_sp_yaw = shortestPathYawRate(yaw_hold_target_, euler.z,
                                                  yaw_hold_kp_, yaw_hold_rate_max_);
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
        // computePositionHold 参照。Grounded 中は動かさない — 保持目標は（自動）離陸時に捕捉。
        // Not during Landing: the autonomous descent uses direct stick tilt (or the
        // level gate below), not the position cascade — keep the landing law uniform
        // across modes (a POS_HOLD landing steers like STABILIZE).
        // Landing 中は動かさない: 自動降下は位置カスケードでなく直接スティック傾き（下の
        // 水平判定）を使い、着陸則をモード間で統一する（POS_HOLD 着陸も STABILIZE 同様に操縦）。
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
        // Landing — 単一の水平判定（INV-2）。リンク生存中はパイロットが roll/pitch/yaw を保つ
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
        //        角度「目標」に加算（POS_HOLD 上書きと Landing 水平判定の後、角度→レート
        //        ループの前）。角度ループがこの傾きを保ち、定常水平ドリフトを推力を余分に
        //        食わず打ち消す。POS_HOLD の位置ループは平衡傾きを担わずに済む。リンク途絶
        //        着陸では水平判定がパイロット傾きを 0 にするがトリムは残るため、降下は
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
        // を終える。姿勢は上の単一パイプライン（パイロット or 水平判定）で処理済み。
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
            // 地上 ARM 中: プロペラ停止。この判定がないと ALT/POS で ARM した瞬間に
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
            // 成立し、定常ホバー偏差・鉛直速度ノイズに非依存（旧両側バンド+低速整定は実機で成立せず
            // ロール/ピッチを0固定した。pid_controller.hpp 参照）。タイムアウトは、到達が成立しなく
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
        } else if (flip_active) {
            // FLIP overrides the vertical channel with the sequencer's own
            // thrust schedule and SUSPENDS the altitude cascade entirely —
            // not just its output. Running alt_pos_/alt_vel_/DOB/hover-learn
            // against the huge, transient altitude error a flip produces
            // would wind up their integrators and corrupt the learned
            // hover_thrust_ (flip-maneuver-plan.md §3.3: "高度カスケードは休止").
            // FLIP は鉛直チャネルを列生成器自身の推力スケジュールで上書きし、
            // 高度カスケードを完全に「休止」する — 出力の上書きだけではない。
            // フリップが生む巨大で過渡的な高度誤差に対して
            // alt_pos_/alt_vel_/DOB/ホバー推力学習を動かすと、それらの積分器が
            // 巻き上がり学習済み hover_thrust_ を汚染する（plan §3.3:
            // 「高度カスケードは休止」）。
            thrust = flip_out.thrust_n;
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
            // スロットル再センターロック: （自動）離陸後や飛行中の ALT/POS 進入直後はスティックが中央
            // から外れていることがあり（例: STABILIZE のホバースロットルは上）、高度がジャンプ
            // する。ロックはスティックが初めて中央デッドゾーン（=バネ静止）に戻るまで指令を抑え、
            // 戻ると解除する — 旧来の「バネ式は離せば解除」。誘導/API は歩く設定点で目標を所有し、
            // このスティック経路に達しない。
            if (!throttle_recentered_ && fabsf(ta) < stick_deadzone_) {
                throttle_recentered_ = true;
            }

            float climb_rate_sp = 0;
            if (guidance_active_ && guide_mode_ == 2) {
                // Velocity guidance (Tello `rc` channel c): command the climb rate
                // directly; track the altitude while moving so releasing (vz→0, or the
                // R16 staleness decay) HOLDS the altitude reached. Same shape as the
                // stick path below — one vertical law (INV-1).
                // 速度誘導（Tello `rc` ch c）: 上昇率を直接指令し、移動中は高度を追従 → 離す
                // （vz→0 or R16 鮮度減衰）と到達高度を保持。下のスティック経路と同形 — 単一鉛直則（INV-1）。
                climb_rate_sp = guide_vz_;
                if (climb_rate_sp != 0.0f) alt_setpoint_ = altitude;
            } else if (throttle_recentered_ && !guidance_active_ &&
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

            // Acceleration-based disturbance observer (DOB, opt-in, this branch is
            // ALREADY Airborne-only — INV-1: a phase may change only the vertical
            // channel). Subtracts an estimate of the external vertical force
            // disturbance built from measured specific force, reacting faster than
            // the vel-loop integrator alone (bypasses the ESKF velocity lag). See
            // computeDobCorrection(); design analysis/scripts/alt_dob_design/
            // README.md §5. Applied AFTER the PI output above and BEFORE the
            // physical clamp below, so learnHoverThrust() (after the clamp) still
            // sees the PI-ONLY correction — the DOB's own washout removes its DC,
            // so it never competes with the hover-thrust learner or the vel-loop
            // integrator for DC ownership (band separation, README §3).
            // 加速度ベース外乱オブザーバ（DOB, opt-in。このブランチは既にAirborne
            // 限定 — INV-1: フェーズが変えるのは鉛直チャネルのみ）。実測した加速度計の測定値から
            // 外乱力の推定を差し引き、速度ループ積分器単体より速く反応する（ESKF
            // 速度の遅れを回避）。computeDobCorrection() 参照、設計根拠 README §5。
            // 上のPI出力の後・下の物理クランプの前に適用 — クランプ後の
            // learnHoverThrust() は「PI単体」の補正を見続ける（DOBのDCは自身の
            // ウォッシュアウトが除くため、ホバー推力学習や速度ループ積分器とDC
            // 所有権を争わない。帯域分離、README §3）。
            if (dob_enabled_) {
                thrust -= computeDobCorrection(state, dt);
            }

            if (thrust < 0.0f)         thrust = 0.0f;
            if (thrust > max_thrust_)  thrust = max_thrust_;

            // Feed the DOB's internal actuation model with the FINAL commanded
            // thrust (DOB correction included) — the correct internal-model-
            // control structure: the model must see what the rotors are actually
            // being told to do, not the pre-DOB PI output.
            // DOB内部アクチュエーションモデルへ「最終」指令推力（DOB補正込み）を
            // 与える — 内部モデル制御として正しい構造（モデルはローターへの
            // 実際の指令を見るべきで、DOB適用前のPI出力ではない）。
            if (dob_enabled_) {
                dob_delay_ring_[dob_delay_idx_] = thrust;
                dob_delay_idx_ = (dob_delay_idx_ + 1) % kDobDelaySamples;
            }

            // Always-on hover-thrust learning: slowly fold the steady velocity-loop output
            // into hover_thrust_ so the feed-forward tracks the true hover thrust (robust to
            // motor wear / battery sag). Runs AFTER the thrust output above, so it has zero
            // same-cycle effect — purely a slow background adapter. INV-1: vertical only.
            // 常時ホバー推力学習: 速度ループ定常出力をゆっくり hover_thrust_ に畳み込み FF を
            // 真のホバー推力へ追従（モータ劣化/電圧低下にロバスト）。上の推力出力の後に呼ぶので
            // 同サイクル影響ゼロ（純粋な低速バックグラウンド適応）。INV-1: 鉛直のみ。
            learnHoverThrust(thrust_correction, vel_up, climb_rate_sp, dt);
        }
    }

    // FLIP vertical-channel override, unconditional catch-all: the button can
    // start a flip from ANY FLYING mode (flip-maneuver-plan.md §9 decision 3),
    // including ACRO/STABILIZE, which the if/else chain above never touches
    // (thrust would otherwise stay at the raw-throttle default from the top
    // of this function). The ALT_HOLD/POS_HOLD Airborne branch above already
    // set thrust = flip_out.thrust_n itself, so this is a no-op there.
    // FLIP 鉛直チャネル上書き、無条件の catch-all: ボタンは任意の FLYING
    // モードからフリップを起動できる（plan §9 決定3）。ACRO/STABILIZE は上の
    // if/else 連鎖が一切触れない（さもなければ推力はこの関数冒頭の生スロットル
    // 既定値のまま）。ALT_HOLD/POS_HOLD の Airborne 分岐は既に自分で
    // thrust = flip_out.thrust_n を設定済みのため、ここでは no-op。
    if (flip_active) {
        thrust = flip_out.thrust_n;
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
                    // オフ周波数点ジャイロ電力 = この周波数の外乱/雑音床
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

    // While the flip spins (rate-only phases), let the flip-axis rate loops use
    // the mixer's geometric torque headroom (flip.spin_torque_limit_nm) instead
    // of the flight-proven normal clamp: the collective sits at mid-scale
    // (thrust_spin_hi_ratio 0.5) precisely so that +-torque headroom exists,
    // and the heavier pitch axis (Iyy) needs it to brake inside 360 deg (SILS
    // api_flip 2026-09-16: 383 deg overshoot at the normal clamp). Restored
    // every cycle so nothing leaks into normal flight.
    // 回転中（レートのみのフェーズ）は回転軸のレートループにミキサーの幾何学的な
    // トルク余裕（flip.spin_torque_limit_nm）を使わせ、通常飛行の実績上限を
    // 使わない: 集合推力を中点（thrust_spin_hi_ratio 0.5）にしたのは ± のトルク
    // 余裕を作るためで、慣性の大きいピッチ軸は 360° 以内で減速するのにそれが要る
    // （SILS api_flip 2026-09-16: 通常上限では 383° まで行き過ぎ）。毎周期戻すので
    // 通常飛行には漏れない。
    const bool flip_spinning = flip_active && !flip_out.attitude_loop;
    const float spin_limit = flip_spinning ? flip_.config.spin_torque_limit_nm
                                           : max_roll_pitch_torque_;
    rate_roll_.output_limit  = spin_limit;
    rate_pitch_.output_limit = spin_limit;
    output.torque[0] = rate_roll_.compute(rate_sp_roll, gyro_rate.x, dt);
    output.torque[1] = rate_pitch_.compute(rate_sp_pitch, gyro_rate.y, dt);
    output.torque[2] = rate_yaw_.compute(rate_sp_yaw, gyro_rate.z, dt);
    // Flip feedforward: the sequencer provides the torque its commanded angular
    // acceleration needs (zero outside Spin/Brake or when flip.ff_gain = 0); the same
    // single rate loop adds it to its PID output and applies the same torque limit
    // (INV-1: no parallel loop).
    // 宙返りのフィードフォワード: 列生成器が指令角加速度に要るトルクを提供し（Spin/Brake
    // 以外または flip.ff_gain = 0 ではゼロ）、同じ唯一のレートループが PID 出力に足して
    // 同じトルク上限を掛ける（INV-1: 並列ループなし）。
    if (flip_spinning) {
        output.torque[0] = fmaxf(-spin_limit, fminf(spin_limit, output.torque[0] + flip_out.torque_ff[0]));
        output.torque[1] = fmaxf(-spin_limit, fminf(spin_limit, output.torque[1] + flip_out.torque_ff[1]));
    }
    // While the flip spins (rate-only phases), cap the yaw torque: yaw needs
    // tau/(4*kappa) of thrust per motor (0.075 N at the 1.23 mNm clamp — as much
    // as the whole per-motor collective), so a small yaw-rate error saturating
    // the yaw loop would eat the mixer headroom the flip axis needs. SILS
    // api_flip (2026-09-16): a -15 deg/s yaw transient during the pitch spin-up
    // drove the yaw loop to its clamp, the per-motor clamp then cut the pitch
    // torque to 1/10 and the brake failed. A few degrees of yaw drift over the
    // 0.3 s spin is harmless; the heading hold in Recover restores it.
    // 回転中（レートのみのフェーズ）はヨートルクに上限を設ける: ヨーは 1 モータ
    // あたり τ/(4κ) の推力差が要り（クランプ 1.23 mN·m で 0.075 N — 集合推力の
    // 1 モータ分に匹敵）、小さなヨーレート誤差でヨーループが飽和すると回転軸に
    // 必要なミキサーの余裕を食い尽くす。SILS api_flip（2026-09-16）: ピッチ加速中の
    // −15 °/s のヨー過渡でヨーループが上限に張り付き、モータごとのクランプで
    // ピッチトルクが 1/10 になり減速に失敗した。0.3 s の回転中の数度のヨーずれは
    // 無害で、Recover のヘディング保持が戻す。
    if (flip_spinning || flip_settling) {
        const float yaw_limit = flip_.config.spin_yaw_torque_limit_nm;
        if (output.torque[2] >  yaw_limit) output.torque[2] =  yaw_limit;
        if (output.torque[2] < -yaw_limit) output.torque[2] = -yaw_limit;
    }
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
        // 開始（前の周波数点/軸/実行の古いDCの持ち越しで低周波が偏るのを防ぐ）。
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
        // オフ周波数点は f+max(27%,2Hz)で最低周波数点でも十分離す（2Hz→4Hz）。漏れで clean 点の coh を誤って
        // 下げ、良好な roll/pitch を不要に棄却するのを防ぐ。最高周波数点 44.5Hz は Nyquist 200Hz 未満。
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

    // Persist the learned hover thrust on the landing edge (every cycle, all phases — the
    // learn step above runs only in Airborne, but touchdown is seen as Grounded here).
    // 学習したホバー推力を着陸エッジで保存（毎サイクル全フェーズ。上の学習は Airborne のみだが
    // 接地はここで Grounded として検出される）。
    persistHoverThrust();

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
    // しない）で NVS 保存。接地（Grounded）は安全な瞬間: 推力は判定済みゆえ単発フラッシュ
    // 書込が飛行を乱さない。注: この set_float+save は ControlTask で実行されるが接地時の単発
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
    // 判定: STABILIZE/ALT_HOLD のスティックほぼ中立ホバーでのみ学習（POS_HOLD 除外＝
    // 位置ループが既にドリフトを打ち消す）、空中かつ誘導なし。意図的な移動/旋回
    // （スティックが不感帯外）は学習を止め、指令移動をトリム誤差と誤認しない。
    // ALT_HOLD also requires a NEUTRAL vertical stick: an active climb/descent
    // (throttle_axis off-centre) tilts the craft to track the rate against wind and
    // would corrupt the horizontal-drift observation. (STABILIZE uses direct throttle,
    // which does not couple into the horizontal axes the same way — left ungated.)
    // ALT_HOLD は鉛直スティックも中立を要求: 上昇/下降中（throttle_axis が中央外）は風に
    // 抗してレート追従するため機体が傾き、水平ドリフト観測を汚す。（STABILIZE は直接
    // スロットルで水平軸へのカップリングが異なるため判定しない。）
    // A FLIP (and its settle window) is a commanded maneuver, not a hover: the
    // velocity swing through the flip would be integrated as a phantom drift.
    // Freezing here also drops trim_learn_init_, so the velocity reference is
    // re-seeded after the window instead of differencing across the flip.
    // FLIP（とその整定窓）は指令マニューバでありホバーではない: 宙返り中の速度変化を
    // 架空のドリフトとして積分してしまう。ここで止めると trim_learn_init_ も落ちるので、
    // 速度基準は宙返りをまたいで差分せず、窓の後で取り直される。
    const bool flip_or_settling = flip_.active() || flip_settle_remaining_s_ > 0.0f;
    const bool hovering =
        (current_mode_ == FlightMode::STABILIZE || current_mode_ == FlightMode::ALT_HOLD) &&
        phase_ == VerticalPhase::Airborne &&
        !guidance_active_ && !flip_or_settling &&
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
    // Signs match sf trim analyze (SILS-verified): forward drift -> +pitch (nose up
    // brakes it), right drift -> -roll. First-order: trim -> equilibrium as 1/tau.
    // 平滑ドリフトを時定数 kTrimLearnTau でトリムに積分。符号は sf trim analyze（SILS
    // 検証済み）と一致: 前方ドリフト -> +pitch（機首上げで制動）、右 -> -roll。1次系。
    const float k = dt / kTrimLearnTau;
    pitch_trim_ += k * (trim_accel_lpf_[0] / gravity_);
    roll_trim_  -= k * (trim_accel_lpf_[1] / gravity_);

    // Clamp to the param range (+/-kTrimMax). / param 範囲（±kTrimMax）にクランプ。
    roll_trim_  = fminf(fmaxf(roll_trim_,  -kTrimMax), kTrimMax);
    pitch_trim_ = fminf(fmaxf(pitch_trim_, -kTrimMax), kTrimMax);
}

// -----------------------------------------------------------------------------
// learnHoverThrust — always-on onboard hover-thrust learning (steady-hover-gated)
// learnHoverThrust — 常時オンボード・ホバー推力学習（定常ホバー限定）
//
// Makes altitude hold ROBUST to thrust degradation (motor wear over flight time, battery
// sag) WITHOUT hand-tuning hover.thrust_corr per flight: at a true steady hover the velocity
// loop's output is the residual (true_hover_thrust − hover_thrust_ feed-forward). Folding
// that residual slowly into hover_thrust_ makes the feed-forward track the true hover thrust;
// the velocity integral then unwinds and the correction re-centres near 0, restoring the full
// ±max_thrust_correction_ authority for the climb and disturbances. Total thrust is unchanged
// at the moment of transfer (no altitude bump): hover_thrust_ rises by δ while the loop drops
// the correction by δ. The vertical analogue of learnTrim().
//
// 高度保持を推力劣化（飛行時間によるモータ劣化・電圧低下）にロバスト化し、hover.thrust_corr の
// フライト毎手調整を不要にする: 真の定常ホバーでは速度ループ出力が残差（真のホバー推力 −
// hover_thrust_ FF）。これをゆっくり hover_thrust_ に畳み込むと FF が真のホバー推力へ追従し、
// 速度積分が解け補正が0付近へ戻り、±max_thrust_correction_ の全権限が離陸・外乱に復活する。
// 移し替えの瞬間は総推力不変（高度に段差なし）: hover_thrust_ が δ 増え補正が δ 減る。
//
// @design architecture.md INV-1 — vertical channel only; one pipeline    [OK]
// -----------------------------------------------------------------------------
void PidController::learnHoverThrust(float thrust_correction, float vz_up,
                                    float climb_rate_sp, float dt)
{
    // Disabled by param (hover.thrust.learn = 0): no learning — use the manual
    // hover.thrust_corr only (e.g. while diagnosing the vertical loop). The landing-edge
    // NVS persist is a separate every-cycle step (persistHoverThrust), since touchdown is
    // seen in the Grounded phase where THIS function is not called.
    // param (hover.thrust.learn = 0) で無効化: 学習せず手動 corr のみ。着陸エッジの NVS 保存は
    // 別の毎サイクル処理（persistHoverThrust）— 接地は Grounded フェーズで起き本関数は呼ばれない。
    if (!hover_learn_enable_ || dt <= 0.0f) return;

    // Gate: learn ONLY in a true steady hover with the altitude loop active — Airborne,
    // ALT_HOLD/POS_HOLD, throttle neutral (no commanded climb/descent), and the craft
    // actually still (|vz| small). The |vz| gate pauses learning through the altitude bob so
    // a transient correction is not mistaken for a hover-thrust error.
    // 判定: 高度ループが動く真の定常ホバーでのみ学習 — Airborne・ALT/POS・スロットル中立
    // （上昇/下降指令なし）・実際に静止（|vz| 小）。|vz| 判定が上下動中の学習を止め、過渡の
    // 補正をホバー推力誤差と誤認しない。
    // Same FLIP/settle-window freeze as learnTrim(): the post-flip altitude
    // recovery is a transient, not a hover-thrust error.
    // learnTrim() と同じ FLIP/整定窓の停止: 宙返り後の高度回復は過渡でありホバー推力の誤差ではない。
    const bool flip_or_settling = flip_.active() || flip_settle_remaining_s_ > 0.0f;
    const bool steady_hover =
        phase_ == VerticalPhase::Airborne &&
        (current_mode_ == FlightMode::ALT_HOLD || current_mode_ == FlightMode::POS_HOLD) &&
        climb_rate_sp == 0.0f && !flip_or_settling &&
        fabsf(vz_up) < kHoverLearnVzDead;
    if (!steady_hover) return;

    // First-order transfer of the steady velocity-loop output into the feed-forward with
    // time constant kHoverLearnTau. Clamp to the hover.thrust_corr range so a bad observation
    // cannot run the feed-forward away.
    // 速度ループ定常出力を時定数 kHoverLearnTau で1次系的に FF へ移す。誤観測で FF が発散
    // しないよう hover.thrust_corr 範囲にクランプ。
    hover_thrust_ += (dt / kHoverLearnTau) * thrust_correction;
    hover_thrust_ = fminf(fmaxf(hover_thrust_, kHoverCorrMin * kMassG), kHoverCorrMax * kMassG);
}

// -----------------------------------------------------------------------------
// persistHoverThrust — save the learned hover thrust to NVS on the landing edge.
// Called EVERY cycle (not from learnHoverThrust, which only runs in Airborne) because
// touchdown is seen in the Grounded phase. Mirrors learnTrim's landing-edge persist:
// touchdown (Grounded) is the safe one-shot flash window — thrust is gated, so the ~37ms
// flash stall lands after touchdown and cannot disturb flight.
// persistHoverThrust — 学習したホバー推力を着陸エッジで NVS 保存。毎サイクル呼ぶ
// （learnHoverThrust は Airborne のみゆえ）。接地は Grounded フェーズで起きる。learnTrim の
// 着陸エッジ保存と同じ安全な接地時単発窓（推力判定済み）。
// -----------------------------------------------------------------------------
void PidController::persistHoverThrust()
{
    if (hover_learn_enable_ &&
        (hover_prev_phase_ == VerticalPhase::Airborne ||
         hover_prev_phase_ == VerticalPhase::Landing) &&
        phase_ == VerticalPhase::Grounded) {
        params::set_float("hover.thrust_corr", hover_thrust_ / kMassG);
        params::save();
    }
    hover_prev_phase_ = phase_;
}

// -----------------------------------------------------------------------------
// computeDobQCoeffs — 2nd-order Butterworth low-pass biquad coefficients (RBJ
// Audio EQ Cookbook LPF recipe, Q=1/sqrt(2), bilinear transform at the nominal
// kDobRateHz — see that constant's doc in pid_controller.hpp). Called from
// loadParams() whenever altitude.dob.fc changes; filter STATES (dob_q_w1_/w2_)
// are untouched here — see resetDobStates().
//
// computeDobQCoeffs — 2次バターワースLPFのbiquad係数（RBJ Audio EQ Cookbook の
// LPF式、Q=1/√2、ノミナル kDobRateHz で双一次変換 — 定数の解説は
// pid_controller.hpp 参照）。altitude.dob.fc 変更時に loadParams() から呼ぶ。
// フィルタ「状態」（dob_q_w1_/w2_）はここでは触らない — resetDobStates() 参照。
// -----------------------------------------------------------------------------
void PidController::computeDobQCoeffs(float fc)
{
    constexpr float kButterworthQ = 0.70710678f;   // 1/sqrt(2) — maximally flat (Butterworth)
    const float w0    = 2.0f * 3.14159265f * fc / kDobRateHz;
    const float cosw0 = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * kButterworthQ);

    const float a0 = 1.0f + alpha;
    dob_q_b0_ = ((1.0f - cosw0) * 0.5f) / a0;
    dob_q_b1_ = (1.0f - cosw0) / a0;
    dob_q_b2_ = dob_q_b0_;
    dob_q_a1_ = (-2.0f * cosw0) / a0;
    dob_q_a2_ = (1.0f - alpha) / a0;
}

// -----------------------------------------------------------------------------
// dobBiquad — Q-filter single-sample update, Direct Form II (2 delay states).
// dobBiquad — Qフィルタ1サンプル更新、Direct Form II（状態2つ）。
// -----------------------------------------------------------------------------
float PidController::dobBiquad(float x)
{
    const float w = x - dob_q_a1_ * dob_q_w1_ - dob_q_a2_ * dob_q_w2_;
    const float y = dob_q_b0_ * w + dob_q_b1_ * dob_q_w1_ + dob_q_b2_ * dob_q_w2_;
    dob_q_w2_ = dob_q_w1_;
    dob_q_w1_ = w;
    return y;
}

// -----------------------------------------------------------------------------
// dobWashout — washout single-sample update: 1st-order high-pass in
// backward-difference/DC-blocker form, fixed coefficient kDobWashoutAlpha
// (kDobWashoutHz at the nominal kDobRateHz).
// dobWashout — ウォッシュアウト1サンプル更新: 後退差分/DCブロッカー形の1次HP、
// 固定係数 kDobWashoutAlpha（ノミナル kDobRateHz での kDobWashoutHz）。
// -----------------------------------------------------------------------------
float PidController::dobWashout(float x, float alpha)
{
    const float y = alpha * (dob_wo_y_prev_ + x - dob_wo_x_prev_);
    dob_wo_x_prev_ = x;
    dob_wo_y_prev_ = y;
    return y;
}

// -----------------------------------------------------------------------------
// computeDobCorrection — acceleration-based disturbance observer (DOB) for the
// Airborne altitude vertical-velocity loop (opt-in, param altitude.dob.fc).
// Caller (compute()) gates this to dob_enabled_ && Airborne.
//
// Compares a nominal actuation model (pure delay + 1st-order lag, driven by
// the PAST commanded thrust so the model's own delay closes on itself)
// against the measured vertical specific force. The residual — force the
// model does not explain — is external disturbance (battery-sag thrust
// droop, gust). It is low-pass filtered (Q, 2nd-order Butterworth at the
// param fc) then high-pass filtered (washout, fixed 0.03 Hz) so the DOB owns
// only the MID band and the velocity-loop integrator + hover-thrust learner
// keep DC ownership (band separation; see
// analysis/scripts/alt_dob_design/README.md §3/§5).
//
// computeDobCorrection — 高度鉛直速度ループ用の加速度ベース外乱オブザーバ
// （DOB、opt-in、param altitude.dob.fc）。呼び出し側（compute()）が
// dob_enabled_ && Airborne で判定する。
//
// ノミナルなアクチュエーションモデル（純遅れ+1次遅れ、モデル自身の遅れ分
// 過去の指令推力で駆動し内部で遅れを閉じる）と実測の鉛直の加速度計測定値を比較する。
// モデルで説明できない残差（外乱：電池電圧低下による推力低下・突風）を2次バター
// ワースLPF（Q, paramのfc）→1次HP（ウォッシュアウト, 固定0.03Hz）に通し、
// DOBは中域のみを担当、DC所有権は速度ループ積分器＋ホバー推力学習に残す
// （帯域分離、README §3/§5）。
//
// @design analysis/scripts/alt_dob_design/README.md §5 — DOB algorithm  [OK]
// @design architecture.md INV-1 — vertical channel only; caller gates
//         Airborne-only                                                [OK]
// -----------------------------------------------------------------------------
float PidController::computeDobCorrection(const StateEstimate& state, float dt)
{
    if (dt <= 0.0f) {
        return dob_d_hat_;   // no elapsed time — hold the last value / 経過時間ゼロ→前回値保持
    }

    // Nominal actuation model: the thrust commanded kDobModelDelayS ago, run
    // through a 1st-order lag (motor time constant) — the model's PREDICTED
    // vertical thrust force absent any external disturbance.
    // ノミナルなアクチュエーションモデル: kDobModelDelayS 前の指令推力を1次遅れ
    // （モータ時定数）に通した「外乱なしなら出ているはずの推力」の予測。
    const float delayed_u    = dob_delay_ring_[dob_delay_idx_];
    const float model_alpha  = 1.0f - expf(-dt / kDobModelLagS);
    dob_model_state_ += model_alpha * (delayed_u - dob_model_state_);

    // Measured upward specific force (body→NED rotation, third row — same
    // convention as math::Quat::to_dcm; NED z is down, so negate).
    // 実測の上向きの加速度計測定値（機体→NED回転第3行、math::Quat::to_dcm と同一規約。
    // NED z は下向きなので負にする）。
    const float qw = state.attitude[0], qx = state.attitude[1];
    const float qy = state.attitude[2], qz = state.attitude[3];
    const float r31 = 2.0f * (qx * qz - qw * qy);
    const float r32 = 2.0f * (qy * qz + qw * qx);
    const float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);   // R33
    const float f_up = -(r31 * state.specific_force[0] +
                          r32 * state.specific_force[1] +
                          cos_tilt * state.specific_force[2]);

    // Specific-force validity guard: an estimator that does not populate
    // specific_force (e.g. sf_estimator_complementary zero-inits it) yields
    // f_up = 0; in real flight f_up sits near +g (≈9.8). Below the guard the
    // measurement is implausible (no data, or a free-fall-like transient), so
    // HOLD the last d_hat instead of slamming the filters with garbage — with
    // the un-primed startup value 0 this makes the DOB a clean no-op.
    // 加速度計の測定値の妥当性ガード: specific_force を埋めない推定器（例: 相補フィルタは
    // ゼロ初期化のまま）では f_up=0 になる。実飛行の f_up は +g（≈9.8）近傍。
    // ガード未満は非妥当な計測（データなし or 自由落下級の過渡）なので、ゴミを
    // フィルタへ送らず前回 d_hat を保持 — 未プライム時の初期値0なら DOB は
    // 完全な no-op になる。
    if (f_up < kDobMinFupMs2) {
        return dob_d_hat_;
    }

    // Residual [N] = measured vertical thrust force − model-predicted force
    // (projected onto vertical via cos_tilt) = external disturbance.
    // 残差[N] = 実測鉛直推力 − モデル予測力（cos_tiltで鉛直投影）= 外乱。
    const float residual = kMassKg * f_up - dob_model_state_ * cos_tilt;

    // Stage 1 — PRIME: average the residual over the first kDobPrimeCycles
    // (d_hat stays 0), then preset Q and washout to that average's steady
    // state — the DOB engages from equilibrium with no artificial step. The
    // residual carries a standing DC (thrust-calibration deficit k_T≈0.95,
    // README §2) AND Airborne entry usually lands mid-transient; see the
    // "engage conditioning" doc in pid_controller.hpp (SILS-measured collapse
    // with an instantaneous-sample prime, 2026-07-18).
    // 第1段 — プライム: 最初の kDobPrimeCycles で残差を平均（この間 d_hat=0）し、
    // その平均の定常状態へ Q・ウォッシュアウトをプリセット — 人工ステップなしの
    // 平衡からエンゲージ。残差には定在DC（推力較正欠損 k_T≈0.95、README §2）が
    // あり、さらに Airborne 進入はたいてい過渡の最中に起きる。瞬時値プライムでの
    // SILS実測墜落（2026-07-18）含め pid_controller.hpp「エンゲージ整形」解説参照。
    if (dob_prime_count_ < kDobPrimeCycles) {
        dob_prime_accum_ += residual;
        ++dob_prime_count_;
        if (dob_prime_count_ == kDobPrimeCycles) {
            const float res_avg = dob_prime_accum_ / static_cast<float>(kDobPrimeCycles);
            // DF2 biquad DC steady state: w = x/(1+a1+a2) → output = x (DC gain 1).
            // DF2 biquad のDC定常: w = x/(1+a1+a2) → 出力 = x（DCゲイン1）。
            const float w_ss = res_avg / (1.0f + dob_q_a1_ + dob_q_a2_);
            dob_q_w1_ = w_ss;
            dob_q_w2_ = w_ss;
            // Washout in equilibrium with that DC: prev input = avg, output 0.
            // そのDCと平衡なウォッシュアウト: prev入力=平均、出力0。
            dob_wo_x_prev_ = res_avg;
            dob_wo_y_prev_ = 0.0f;
        }
        dob_d_hat_ = 0.0f;
        return dob_d_hat_;
    }

    // Stage 2 — FAST-SETTLE washout during the engage window, normal after.
    // 第2段 — エンゲージ窓中は高速整定ウォッシュアウト、以後は通常。
    const bool engaging = (dob_engage_count_ < kDobEngageRampCycles);
    const float wo_alpha = engaging ? kDobWashoutFastAlpha : kDobWashoutAlpha;

    const float q_out = dobBiquad(residual);
    const float d_raw = dobWashout(q_out, wo_alpha);

    // Stage 3 — RAMP the applied correction 0→1 across the engage window.
    // 第3段 — 適用補正をエンゲージ窓で 0→1 にランプ。
    float ramp = 1.0f;
    if (engaging) {
        ramp = static_cast<float>(dob_engage_count_) /
               static_cast<float>(kDobEngageRampCycles);
        ++dob_engage_count_;
    }
    dob_d_hat_ = ramp * fminf(fmaxf(d_raw, -kDobClampN), kDobClampN);
    return dob_d_hat_;
}

// -----------------------------------------------------------------------------
// resetDobStates — equilibrium (steady-state) re-initialization of every DOB
// filter state to current_thrust. Sim-validated as NECESSARY, not cosmetic
// (analysis/scripts/alt_dob_design/README.md §4-1): a cold start otherwise
// injects a multi-second thrust transient into every Airborne (re-)entry.
// Called from loadParams() (fc change) and every phase_ transition helper
// that calls applyAltVelTiForPhase() — the same INV-1 vertical-channel-only
// scope — via a single shared helper, never a one-off inline assignment
// (architectural-invariants discipline).
//
// resetDobStates — DOB全フィルタ状態を current_thrust へ平衡（定常）再初期化。
// シム検証で必須と確定（意匠でない。README §4-1）: 冷開始だと Airborne
// (再)進入のたびに数秒スケールの推力過渡が注入される。loadParams()（fc変更）
// と applyAltVelTiForPhase() を呼ぶ全フェーズ遷移ヘルパ（同じINV-1鉛直
// チャネル限定スコープ）から、単一の共有ヘルパ経由で呼ぶ — 場当たりの
// 個別代入は行わない（アーキテクチャ不変条件の作法）。
// -----------------------------------------------------------------------------
void PidController::resetDobStates(float current_thrust)
{
    // Delay buffer + actuation model: pre-fill with the current thrust so the
    // model starts already "caught up" — no artificial startup transient.
    // 遅延バッファ+アクチュエーションモデル: 現在推力で充填し「追いついた」
    // 状態で開始 — 人工的な起動過渡なし。
    for (int i = 0; i < kDobDelaySamples; ++i) {
        dob_delay_ring_[i] = current_thrust;
    }
    dob_delay_idx_   = 0;
    dob_model_state_ = current_thrust;

    // Q-filter / washout: no specific-force reading is available at any reset
    // call site (none pass a StateEstimate), so the measurement-side states
    // cannot be equilibrium-seeded HERE. They are zeroed as placeholders and
    // the engage-conditioning counters restart — the next Airborne samples in
    // computeDobCorrection() re-run PRIME (0.25 s residual average) →
    // FAST-SETTLE → RAMP (see the "engage conditioning" doc in
    // pid_controller.hpp).
    // Qフィルタ/ウォッシュアウト: どのリセット呼び出し箇所も加速度計の実測値
    // （StateEstimate）を渡さないため、計測側の状態は「ここでは」平衡シード
    // できない。プレースホルダとして0にし、エンゲージ整形カウンタを再スタート —
    // 次の Airborne サンプル列で computeDobCorrection() がプライム（0.25s残差
    // 平均）→高速整定→ランプを再実行する（pid_controller.hpp「エンゲージ整形」
    // 解説参照）。
    dob_q_w1_ = 0.0f;
    dob_q_w2_ = 0.0f;
    dob_wo_x_prev_ = 0.0f;
    dob_wo_y_prev_ = 0.0f;
    dob_d_hat_     = 0.0f;
    dob_prime_count_  = 0;
    dob_prime_accum_  = 0.0f;
    dob_engage_count_ = 0;
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
    // Velocity source: the Tello `rc` velocity guidance (mode 2) when engaged, else
    // the pilot stick. If the pilot moves a stick, the cancel-on-stick test upstream
    // (compute()) has already cleared guidance_active_, so reaching here with velocity
    // guidance active means the pilot is hands-off — one path, pilot always wins (INV-2).
    // 速度源: 係合中は Tello `rc` 速度誘導（mode 2）、それ以外はパイロットスティック。パイロットが
    // スティックを動かせば上流（compute()）のスティック解除判定が既に guidance_active_ を落として
    // いるため、ここに速度誘導 active で来る＝パイロットは手放し — 単一経路・パイロット優先（INV-2）。
    float v_fwd, v_right;
    if (guidance_active_ && guide_mode_ == 2) {
        v_fwd   = guide_vx_;   // body forward (FRD x) [m/s] — Tello rc b / 機体前後
        v_right = guide_vy_;   // body right   (FRD y) [m/s] — Tello rc a / 機体左右
    } else {
        v_fwd   = -setpoint.pitch * stick_reposition_vel_;   // forward (FRD x) [m/s]
        v_right =  setpoint.roll  * stick_reposition_vel_;   // right   (FRD y) [m/s]
    }
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
        // During the post-flip settle window the target TRACKS the craft: the
        // flip leaves up to ~1.4 m/s of horizontal velocity, and capturing the
        // position at FlipComplete made the craft overshoot 1 m and fly back at
        // the position loop's pace (SILS 2026-09-17). Tracking lets the velocity
        // loop bleed the speed off and the hold point becomes wherever the craft
        // stops — the same behaviour as the factory Tello.
        // 宙返り後の整定窓では目標が機体に追従する: 宙返りで最大約 1.4 m/s の水平速度が
        // 残り、FlipComplete の位置を取り込むと 1 m 行き過ぎてから位置ループの速さで
        // 戻っていた（SILS 2026-09-17）。追従させると速度ループが速度を抜き、止まった
        // 場所が保持点になる — 工場出荷の Tello と同じ振る舞い。
        const bool flip_settling = flip_settle_remaining_s_ > 0.0f && !flip_.active();
        if (capture_pos_ || flip_settling) {
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
    // In the post-flip settle window the cap shrinks to settle_tilt_deg so the
    // position loop cannot starve the mixer while the flip's horizontal
    // velocity is still being bled off (FlipSequencer::Config::settle_ms).
    // 宙返り後の整定窓では上限を settle_tilt_deg に縮め、宙返りで付いた水平速度が
    // 抜ける間に位置ループがミキサーの余裕を食い尽くさないようにする。
    constexpr float kDegToRad = 3.14159265358979f / 180.0f;
    const float settle_tilt = flip_.config.settle_tilt_deg * kDegToRad;
    const bool settling = flip_settle_remaining_s_ > 0.0f && !flip_.active();
    const float tilt_limit = (settling && settle_tilt < max_pos_tilt_) ? settle_tilt : max_pos_tilt_;
    auto clampTilt = [tilt_limit](float t) {
        if (t >  tilt_limit) return  tilt_limit;
        if (t < -tilt_limit) return -tilt_limit;
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
    applyAltVelTiForPhase();    // → climb ti (gentle, bounds windup) / climb ti へ（穏やか）
    resetDobStates(hover_thrust_);  // DOB is Airborne-only; re-seed for the next entry / DOBはAirborne限定、次回進入用に再シード
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
    applyAltVelTiForPhase();    // → climb ti (gentle, bounds capture overshoot) / climb ti へ（穏やか）
    resetDobStates(hover_thrust_);  // DOB is Airborne-only; re-seed ahead of the climb / DOBはAirborne限定、上昇に備え再シード

    // Fresh vertical loops (they may hold a Grounded-phase zero-output history), set
    // the climb target, and capture the launch point for POS_HOLD (the cascade starts
    // at the next compute). The throttle re-center gate is closed: the pilot must pass
    // the stick through center before it commands altitude (Case A — anti-jump).
    // The takeoff-complete signal starts fresh.
    // 鉛直ループを仕切り直し（Grounded フェーズのゼロ出力履歴を持ちうる）、上昇目標を設定し、
    // POS_HOLD 用に発進点を捕捉（カスケードは次の compute から動く）。スロットル再センター
    // 判定は閉: パイロットがスティックを中央に通すまで高度を指令しない（Case A — ジャンプ
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
    applyAltVelTiForPhase();    // → hover ti (strong, rejects battery-sag disturbance) / hover ti へ（強い積分）
    resetDobStates(hover_thrust_);  // fresh equilibrium-init DOB start for this Airborne session (sim-validated, README §4-1) / この空中セッション用にDOBを平衡初期化で新規開始（シム検証済み、README §4-1）

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
    // FLIP owns every setpoint channel for its short duration — a guidance
    // target arriving mid-flip must not fight it (flip-maneuver-plan.md §4.1:
    // "誘導目標・同定励振は FLIP 中は無視"). Rejected here rather than
    // silently overwritten-and-ignored, so a caller can tell the difference.
    // FLIP は短時間の間、全ての設定点チャネルを所有する — フリップ中に届いた
    // 誘導目標がそれと競合してはならない（plan §4.1: 「誘導目標・同定励振は
    // FLIP 中は無視」）。無言で上書き＆無視にせず、ここで拒否して呼び出し側が
    // 判別できるようにする。
    if (flip_.active()) {
        ESP_LOGW(TAG, "Guidance target rejected: flip in progress");
        return;
    }
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

    // Were we ALREADY engaged in velocity (rc) mode before this call? Capture it
    // BEFORE overwriting guide_mode_ — it decides whether to re-snapshot the sticks.
    // 本呼び出し前に既に速度（rc）モードで係合していたか? guide_mode_ を上書きする前に捕捉
    // — スティックを取り直すかの判定に使う。
    const bool velocity_refresh =
        (target.mode == 2) && guidance_active_ && (guide_mode_ == 2);

    if (target.mode == 2) {
        // Velocity guidance (Tello `rc`): store the body-frame velocity command;
        // it is injected into computePositionHold / the climb-rate path. The
        // timestamp drives the R16 staleness auto-release (see compute()).
        // 速度誘導（Tello `rc`）: 機体系速度指令を保持し computePositionHold / 上昇率経路へ注入。
        // 時刻は R16 鮮度オートリリース（compute() 参照）を駆動する。
        guide_mode_  = 2;
        guide_vx_    = target.vx;
        guide_vy_    = target.vy;
        guide_vz_    = target.vz;
        guide_vyaw_  = target.vyaw;
        guide_stamp_ = target.timestamp;
    } else {
        guide_mode_   = 1;
        guide_pos_[0] = target.position[0];
        guide_pos_[1] = target.position[1];
        guide_pos_[2] = target.position[2];
        guide_yaw_    = target.yaw;
        if (target.speed > 0.05f && target.speed <= 2.0f) {
            guide_speed_ = target.speed;
        }
    }

    // Stick snapshot: guidance is cancelled by stick MOVEMENT, not position —
    // in an API flight the throttle stick rests at the bottom, which must not
    // read as a descend command or an instant cancel. Take it only on the INITIAL
    // engage (or a mode switch): a continuous rc stream refreshes the target every
    // datagram, and re-snapshotting each time would mask a slow pilot stick creep
    // below the cancel threshold (the absolute-departure test must compare against
    // the original engage pose).
    // スティックスナップショット: 解除は「位置」でなく「動き」で判定 — API 飛行では
    // スロットルスティックは下端のままであり、それが降下指令や即時解除になってはならない。
    // 取得は「最初の係合」（またはモード切替）時のみ: rc は毎データグラムで目標を更新するため、
    // 毎回取り直すとパイロットのゆっくりしたスティック・クリープを解除閾値以下でマスクしてしまう
    // （絶対変位の判定は元の係合姿勢と比較する必要がある）。
    if (!velocity_refresh) {
        stick_snapshot_[0] = current_sticks.roll;
        stick_snapshot_[1] = current_sticks.pitch;
        stick_snapshot_[2] = current_sticks.yaw;
        stick_snapshot_[3] = current_sticks.throttle;
    }
    guidance_active_ = true;
    if (target.mode == 2) {
        ESP_LOGD(TAG, "Guidance velocity: body [%.2f %.2f %.2f] yawrate %.2f",
                 static_cast<double>(guide_vx_), static_cast<double>(guide_vy_),
                 static_cast<double>(guide_vz_), static_cast<double>(guide_vyaw_));
    } else {
        ESP_LOGI(TAG, "Guidance target: NED [%.2f %.2f %.2f] yaw %.2f speed %.2f",
                 static_cast<double>(guide_pos_[0]), static_cast<double>(guide_pos_[1]),
                 static_cast<double>(guide_pos_[2]), static_cast<double>(guide_yaw_),
                 static_cast<double>(guide_speed_));
    }
}

void PidController::startExcitation(const SysidCommand& cmd)
{
    // Same FLIP-owns-every-channel rejection as setGuidanceTarget() above —
    // flip-maneuver-plan.md §4.1.
    // setGuidanceTarget() 上と同じ「FLIP が全チャネルを所有」の拒否 —
    // plan §4.1。
    if (flip_.active()) {
        ESP_LOGW(TAG, "Sysid excitation rejected: flip in progress");
        return;
    }
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
    // オフ周波数点蓄積/位相は点ごとにリセット。（除トレンドの y_dc_ は最初の蓄積サンプルで現DCに再シード。）
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
    applyAltVelTiForPhase();             // → climb ti (Grounded uses the gentle schedule) / climb ti へ
    resetDobStates(hover_thrust_);       // DOB re-seeds for the next flight / DOBは次の飛行用に再シード
    landing_settle_t_ = 0.0f;            // near-ground settle ramp / 着地アシストのランプ
    guidance_active_ = false;            // guidance dies with the flight / 誘導も飛行と共に終了
    reposition_active_ = false;          // stick repositioning state clears / スティック再配置状態クリア
    excite_active_   = false;            // so does the excitation / 励振も同様
    yaw_hold_active_ = false;            // heading hold too / ヘディングホールドも同様
    throttle_recentered_    = false;     // re-center gate re-arms for the next takeoff / 次の離陸用にロックを再び掛ける
    takeoff_reached_        = false;      // takeoff-complete signal clears / 離陸完了信号クリア
    takeoff_settle_cycles_  = 0;
    takeoff_elapsed_cycles_ = 0;
    trim_learn_init_ = false;                    // re-seed the trim-learner filter / トリム学習フィルタ再初期化
    trim_prev_phase_ = VerticalPhase::Grounded;  // landing-edge sync / 着陸エッジ整合
    // NOTE: roll_trim_/pitch_trim_ are NOT cleared — the learned/loaded trim persists
    // across resets (config, not integrator state). / roll_trim_/pitch_trim_ はクリア
    // しない — 学習/読込トリムは reset を跨いで保持（積分器状態でなく構成値）。
    flip_.reset();                              // idle + cooldown handling, keeps the last result_ / idle化+クールダウン処理、最終result_は保持
    flip_start_pending_ = false;
    flip_prev_phase_    = FlipSequencer::Phase::Idle;
    flip_ready_         = false;
    flip_reason_        = FlipBlockReason::NotFlying;
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
        // スロットル再センターロックを掛ける — スティックは切替時に任意位置にあり、
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

// -----------------------------------------------------------------------------
// shortestPathYawRate — shared heading-rate P law (target - current, wrapped
// to [-pi,pi], clamped to +-rate_max). One formula for three call sites:
// the guidance position-mode (1) yaw seek and the normal yaw-hold block
// above (both refactored to call this — identical math, no behavior change),
// and the FLIP Boost/Recover heading hold below (computeFlipAttitude()).
// shortestPathYawRate — 共有のヘディングレート P 則（target-current を
// [-pi,pi] に折返し、±rate_max にクランプ）。3箇所が1つの式を共有する:
// 誘導位置モード(1)のヨーシークと上の通常ヘディングホールド（どちらも
// この呼び出しへリファクタ済み — 数式は同一、挙動変化なし）、そして下の
// FLIP Boost/Recover ヘディングホールド（computeFlipAttitude()）。
// -----------------------------------------------------------------------------
float PidController::shortestPathYawRate(float target_yaw, float current_yaw,
                                         float kp, float rate_max) const
{
    float error = target_yaw - current_yaw;
    while (error >  3.14159265f) error -= 6.2831853f;
    while (error < -3.14159265f) error += 6.2831853f;
    float rate = kp * error;
    if (rate >  rate_max) rate =  rate_max;
    if (rate < -rate_max) rate = -rate_max;
    return rate;
}

// =============================================================================
// Flip maneuver (docs/plans/flip-maneuver-plan.md §3/§4/§7)
// 宙返りマニューバ（plan §3/§4/§7）
// =============================================================================

// -----------------------------------------------------------------------------
// onFlip — FLYING->FLIP entry (ControllerCmd::Flip). Latches the direction for
// the deferred flip_.start() in compute() (see compute()'s doc on
// flip_start_pending_) and resets the rate-loop integrators so Boost starts
// clean.
//
// Defensive re-check, not a hard gate: state_task already verified
// isFlipReady() before issuing ControllerCmd::Flip, but up to one control
// cycle may have passed. This proceeds regardless of a stale disagreement —
// once FlightState::FLIP is entered the ONLY documented exit is flip_done
// (detailed_design.md §3.1 FLIP row: FLIP->FLYING via flip_done, FLIP-> any
// other state only through IMPACT/DISARM/emergency). Refusing to start here
// would strand the state machine in FLIP with no way back. A stale-condition
// start is still bounded and safe: the in-flight abort paths (spin_timeout /
// gyro_abort / recover_timeout) catch a maneuver that goes wrong once running.
//
// onFlip — FLYING→FLIP突入（ControllerCmd::Flip）。方向を、compute() 内の
// 遅延 flip_.start() 用に保持し（compute() の flip_start_pending_ ドキュメント
// 参照）、Boost がきれいに始まるようレートループ積分器をリセットする。
//
// 防御的再評価であり、ハードゲートではない: state_task は
// ControllerCmd::Flip 発行前に isFlipReady() を確認済みだが、最大1制御周期分
// 経過している可能性がある。判定が古くなっていても開始を続行する —
// FlightState::FLIP に入った後の唯一の文書化された脱出は flip_done
// （detailed_design.md §3.1 FLIP行: FLIP→FLYING は flip_done 経由、
// FLIP→他状態は IMPACT/DISARM/emergency のみ）。ここで開始を拒否すると
// 状態機械が戻る手段なく FLIP に取り残される。古い判定のまま開始しても、
// 実行中の打ち切り経路（spin_timeout / gyro_abort / recover_timeout）が
// 走り出してから異常を捕捉するため安全な範囲に収まる。
// -----------------------------------------------------------------------------
void PidController::onFlip(FlipDirection dir)
{
    FlipBlockReason reason;
    if (!isFlipReady(reason)) {
        ESP_LOGW(TAG, "Flip %s starting despite a stale readiness check (%s)",
                 flipDirectionName(dir), flipBlockReasonName(reason));
    }
    // Belt-and-suspenders for the same stale-readiness race: C8 already keeps
    // flip_ready_ false while excitation/guidance is active, but force both
    // off HERE too so a flip that starts anyway (see the doc above) cannot
    // have either still injecting into the setpoints it now owns exclusively.
    // 同じ「判定の古さ」による競合への保険: C8 は励振/誘導が有効な間
    // flip_ready_ を false に保つが、（上のドキュメントどおり）それでも開始した
    // 場合に備え、ここでも両方を強制停止し、フリップが今から独占する設定点へ
    // どちらかが注入し続けることがないようにする。
    excite_active_   = false;
    guidance_active_ = false;
    rate_roll_.reset();
    rate_pitch_.reset();
    rate_yaw_.reset();
    flip_pending_dir_   = dir;
    flip_start_pending_ = true;
    ESP_LOGI(TAG, "Flip engaged: %s", flipDirectionName(dir));
}

// -----------------------------------------------------------------------------
// onFlipComplete — FLIP->FLYING exit (ControllerCmd::FlipComplete, plan §4.1
// onExit). Re-captures the altitude/position targets at the CURRENT state
// (via the existing capture_alt_/capture_pos_ deferred-capture flags — the
// same mechanism onTakeoff()/onModeChange() already use) and resets the
// altitude/position integrators: both horizontal position and altitude moved
// during the flip (plan §3.6: horizontal drift ~0.1m over 0.5s; vertical is
// actively driven by Recover), so holding the PRE-flip targets would command
// a walk-back instead of a clean hold where the flip actually ended.
//
// flip_.reset() clears phase_ back to Idle (isFlipActive()/isFlipDone()
// both go false from here) but deliberately keeps the last result_ — see
// FlipSequencer::reset()'s doc: the Tello API polls ControllerStatus AFTER
// state_task's FLIP->FLYING and needs flip_result to answer `flip <dir>`.
//
// onFlipComplete — FLIP→FLYING退出（ControllerCmd::FlipComplete, plan §4.1
// onExit）。高度・位置目標を「現在」の状態で取り直し（既存の
// capture_alt_/capture_pos_ 遅延取り込みフラグ経由 — onTakeoff()/
// onModeChange() が既に使うのと同じ仕組み）、高度・位置積分器をリセットする:
// フリップ中は水平位置・高度とも動く（plan §3.6: 水平ドリフト0.5sで約0.1m、
// 鉛直は Recover が能動駆動）ため、フリップ「前」の目標を保持すると
// 「戻る」指令になり、フリップが実際に終わった場所での綺麗な保持にならない。
//
// flip_.reset() は phase_ を Idle に戻す（ここから isFlipActive()/
// isFlipDone() は両方 false）が、最後の result_ は意図的に保持する —
// FlipSequencer::reset() のドキュメント参照: Tello API は state_task の
// FLIP→FLYING の「後」に ControllerStatus をポーリングし、`flip <dir>` に
// 答えるため flip_result を必要とする。
// -----------------------------------------------------------------------------
void PidController::onFlipComplete()
{
    // Altitude target = the PRE-flip altitude (plan §4.1 onExit), not the
    // current one: Recover ends near the lowest point, so re-capturing there
    // would ratchet the hover down by ~0.1 m per flip (SILS 2026-09-16).
    // 高度目標は「宙返り前」の高度（plan §4.1 onExit）。現在高度で取り直すと
    // Recover の終わりは最低点付近なので宙返りごとに約 0.1 m ずつ下がる（SILS
    // 2026-09-16）。
    alt_setpoint_ = flip_.startHeightM();
    capture_alt_ = false;
    capture_pos_ = true;
    // Keep the altitude and horizontal-velocity integrators: they hold the
    // hover-thrust and tilt trim learned BEFORE the flip (the cascade was
    // paused, not disturbed, during it). Resetting them made the hover sag
    // 0.1-0.15 m for several seconds after each flip and, with a 1.0 m
    // minimum height, refused the next flip as "too low" (SILS 2026-09-17
    // perturbation runs). Only the outer position P loops start afresh
    // from the re-captured position.
    // 高度と水平速度の積分項は保持する: 宙返り前に学習したホバー推力と傾きの
    // 補正を持っており、宙返り中は休止していただけで乱れていない。リセットすると
    // 宙返りのたびにホバーが 0.1〜0.15 m 沈んで数秒続き、最低高度 1.0 m のもとで
    // 次の宙返りが「too low」で拒否された（SILS 2026-09-17 摂動族）。取り直した
    // 位置から始め直すのは外側の位置 P ループだけ。
    pos_x_.reset();
    pos_y_.reset();
    rate_yaw_.reset();
    flip_settle_remaining_s_ = flip_.config.settle_ms * 0.001f;
    flip_.reset();
    flip_prev_phase_ = FlipSequencer::Phase::Idle;
    ESP_LOGI(TAG, "Flip complete — normal mode law resumed");
}

// -----------------------------------------------------------------------------
// isFlipReady — reads the cache updateFlipReadiness() (called every
// compute() cycle) last wrote. const, so it cannot itself read the
// StateEstimate needed to evaluate C2-C7 — same reason isTakeoffComplete()/
// isGuidanceActive() read a cached flag instead of computing on demand.
// isFlipReady — updateFlipReadiness()（毎 compute() 周期で呼ぶ）が最後に
// 書いたキャッシュを読む。const のため C2-C7 の評価に要る StateEstimate を
// 自ら読めない — isTakeoffComplete()/isGuidanceActive() がオンデマンド計算
// でなくキャッシュ済みフラグを読むのと同じ理由。
// -----------------------------------------------------------------------------
bool PidController::isFlipReady(FlipBlockReason& reason) const
{
    reason = flip_reason_;
    return flip_ready_;
}

// -----------------------------------------------------------------------------
// buildFlipInput — assemble one cycle's FlipSequencer::Input from the state
// estimate. Sign conventions MATCH the ones already used elsewhere in this
// file (see the Landing/ALT_HOLD vertical blocks in compute()): NED
// position/velocity z is down-positive, so altitude and up-velocity are both
// negated.
//
// estimator_ok is a coarse proxy (sensor_mask != 0, i.e. the ESKF has not
// frozen every state) — StateEstimate does not yet carry a dedicated
// estimator-health fact reachable from a core (topic-free) component; a
// cleaner signal is expected once the parallel estimator work lands (see this
// task's final report). battery_v/tof_valid are exact: both are injected by
// ControlTask into its LOCAL copy of StateEstimate before calling compute()
// (see data_types.hpp's doc on those two fields and control_task.cpp).
//
// buildFlipInput — state から1周期分の FlipSequencer::Input を組み立てる。
// 符号規約は本ファイル他所（compute() の Landing/ALT_HOLD 鉛直ブロック）と
// 一致させる: NED の位置/速度の z は下向き正のため、高度と上向き速度は
// どちらも符号反転する。
//
// estimator_ok は粗い代理指標（sensor_mask != 0、すなわち ESKF が全状態を
// 凍結していない）— StateEstimate はまだコア（トピック禁制）部品から届く
// 専用の推定器健全性の事実を持たない。並行実装中の推定器側の作業が
// 着地すればより正確な信号に置き換わる見込み（本タスクの最終報告参照）。
// battery_v/tof_valid は正確 — どちらも ControlTask が compute() 呼び出し前に
// StateEstimate の「ローカルコピー」へ注入する（data_types.hpp のこの2
// フィールドのドキュメントと control_task.cpp 参照）。
// -----------------------------------------------------------------------------
FlipSequencer::Input PidController::buildFlipInput(const StateEstimate& state, float dt) const
{
    FlipSequencer::Input input{};
    for (int i = 0; i < 3; ++i) input.gyro[i] = state.angular_rate[i];
    for (int i = 0; i < 4; ++i) input.quat[i] = state.attitude[i];
    input.height_m = -state.position[2];                   // NED z-down -> altitude up
    input.vertical_velocity_up_mps = -state.velocity[2];   // NED z-down -> up-positive
    input.horizontal_speed_mps =
        sqrtf(state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1]);
    input.battery_v    = state.battery_voltage;
    input.estimator_ok = (state.sensor_mask != 0);
    input.tof_valid    = state.tof_valid;
    input.dt = dt;
    return input;
}

// -----------------------------------------------------------------------------
// updateFlipReadiness — re-evaluate C2-C8 into the flip_ready_/flip_reason_
// cache isFlipReady() (const) reads. Called once per compute() cycle,
// regardless of mode/phase — see compute()'s doc for why no explicit
// FlightState/Airborne check is needed here.
// updateFlipReadiness — C2-C8 を再評価し、isFlipReady()（const）が読む
// flip_ready_/flip_reason_ キャッシュを更新する。モード/フェーズに関わらず
// compute() 周期ごとに1回呼ぶ — 明示的な FlightState/Airborne 判定が不要な
// 理由は compute() のドキュメント参照。
// -----------------------------------------------------------------------------
void PidController::updateFlipReadiness(const StateEstimate& state, float dt)
{
    // Master switch first: the one gate both the FLIP button (state_task) and
    // the API `flip` verb (api_task) read via controller_status.flip_ready, so
    // disabling here rejects both entry points (reason: Disabled).
    // マスタースイッチを最初に判定: FLIP ボタン（state_task）と API `flip`
    // （api_task）はどちらも controller_status.flip_ready を読むため、ここで無効に
    // すれば両経路が拒否される（理由: Disabled）。
    if (!flip_enabled_) {
        flip_ready_  = false;
        flip_reason_ = FlipBlockReason::Disabled;
        return;
    }

    const FlipSequencer::Input input = buildFlipInput(state, dt);
    FlipBlockReason reason = FlipBlockReason::None;
    bool ready = flip_.ready(input, reason);

    // C8 (plan §3.1): identification excitation or guidance already driving a
    // setpoint through this same pipeline — reject rather than let two
    // sources fight for the same channels. FlipSequencer has no knowledge of
    // sysid/guidance (it only knows its own phases), so this is judged here,
    // not inside ready() (plan §4.1/§4.2 INV-3: detection vs. judgment).
    // C8（plan §3.1）: 同定励振または誘導が既にこの同じパイプラインへ設定点を
    // 流している — 2つの出所が同一チャネルを取り合うのを避けるため拒否する。
    // FlipSequencer は sysid/誘導を知らない（自身のフェーズのみ知る）ため、
    // ready() の中でなくここで判定する（plan §4.1/§4.2 INV-3: 検出と判断の
    // 分離）。
    // An ENGAGED guidance target (API "up"/"forward" already reached, POS_HOLD
    // holding it) is not a conflict: the API flies through guidance, so a flip
    // right after a move is the normal Tello use. onFlip() releases the target
    // and FlipComplete re-captures the position. Only the sysid excitation is
    // a competing setpoint SOURCE; an in-progress API move cannot overlap a
    // flip because ApiTask processes commands one at a time.
    // 係合中の誘導目標（API の up/forward 到達後に POS_HOLD が保持）は競合ではない:
    // API は誘導で飛ぶので、移動直後の宙返りが Tello の通常の使い方。onFlip() が目標を
    // 解放し FlipComplete が位置を取り直す。競合する設定点の出所は同定励振だけ。
    // 進行中の API 移動は ApiTask が 1 件ずつ処理するため宙返りと重ならない。
    if (ready && excite_active_) {
        ready = false;
        reason = FlipBlockReason::SourceConflict;
    }
    // Busy: a flip is already active() — StateManager rejects a second FLIP
    // request while state==FLIP (detailed_design.md §3.1), so this mainly
    // guards the brief window before that transition is observed.
    // Busy: フリップが既に active() — StateManager は state==FLIP 中の
    // 2件目の FLIP 要求を拒否する（detailed_design.md §3.1）ため、これは
    // 主にその遷移が観測されるまでの短い窓を守る防御。
    if (ready && flip_.active()) {
        ready = false;
        reason = FlipBlockReason::Busy;
    }

    flip_ready_  = ready;
    flip_reason_ = reason;
}

// -----------------------------------------------------------------------------
// computeFlipAttitude — route roll/pitch/yaw setpoints for one FLIP cycle.
// See flip_sequencer.hpp's Output doc and pid_controller.hpp's doc on this
// method for the two-branch shape (Spin/Brake bypass the attitude PID
// entirely; Boost/Recover/Done reuse it for a level hold).
// computeFlipAttitude — 1 FLIP周期分の roll/pitch/yaw 設定点を配線する。
// 2分岐の形（Spin/Brake は姿勢PIDを完全迂回、Boost/Recover/Done は水平保持に
// 再利用）は flip_sequencer.hpp の Output ドキュメントと pid_controller.hpp
// の本メソッドのドキュメント参照。
// -----------------------------------------------------------------------------
void PidController::computeFlipAttitude(const StateEstimate& state,
                                        const FlipSequencer::Output& flip_out,
                                        float dt,
                                        float& rate_sp_roll, float& rate_sp_pitch,
                                        float& rate_sp_yaw, float& roll_sp,
                                        float& pitch_sp)
{
    if (!flip_out.attitude_loop) {
        // Spin/Brake: drive the EXISTING rate PID directly with the
        // sequencer's ramped rate setpoint — the same open-loop-setpoint path
        // ACRO already uses (INV-1: no parallel control law). roll_sp/
        // pitch_sp stay 0 for the Data Stream's angle_ref export (unused by
        // any control law in this branch).
        // Spin/Brake: 既存のレートPIDを列生成器のランプ済みレート設定点で
        // 直接駆動する — ACROと同じ開ループ設定点経路（INV-1: 並列の制御則
        // なし）。roll_sp/pitch_sp は Data Stream の angle_ref 出力用に0の
        // まま（この分岐ではどの制御則も使わない）。
        rate_sp_roll  = flip_out.rate_sp[0];
        rate_sp_pitch = flip_out.rate_sp[1];
        rate_sp_yaw   = flip_out.rate_sp[2];
        roll_sp  = 0.0f;
        pitch_sp = 0.0f;
        return;
    }

    // Boost/Recover/Done: reuse the EXISTING attitude PID for a level hold
    // (roll_sp=pitch_sp=0). The measurement is the quaternion-based tilt error
    // (FlipSequencer::levelError), NOT Euler roll/pitch: after an aborted flip
    // Recover must right the craft from ANY attitude, and Euler angles wrap at
    // 180 deg and jump at the pitch singularity. For small tilts the two are
    // identical, so the normal Boost/Done level hold is unchanged.
    // Yaw: the shared heading-hold P law (same gains normal flight's heading
    // hold uses) keeps the yaw captured at start(), but only while the craft is
    // within recover_boost_tilt_deg of level — the Euler yaw of a tilted or
    // inverted craft is meaningless (it jumps 180 deg through a pitch flip), and
    // chasing it would spin the craft while it is still being righted.
    // Boost/Recover/Done: 既存の姿勢PIDを水平保持（roll_sp=pitch_sp=0）に
    // 再利用する。計測値はオイラーの roll/pitch ではなくクォータニオンベースの
    // 傾き誤差（FlipSequencer::levelError）: 打ち切り後の Recover は「どの姿勢
    // からでも」機体を立て直す必要があり、オイラー角は 180° で折り返し、ピッチ
    // 特異点で跳ぶ。小さな傾きでは両者は同一なので、通常の Boost/Done の
    // 水平保持は変わらない。
    // ヨー: 共有のヘディングホールド P 則（通常飛行のヘディングホールドと同じ
    // ゲイン）で start() 時のヨーを保つが、水平から recover_boost_tilt_deg 以内の
    // ときだけ — 傾いた/反転した機体のオイラーヨーは無意味（ピッチ宙返りで
    // 180° 跳ぶ）で、それを追うと立て直し中の機体を回してしまう。
    float roll_meas = 0.0f;
    float pitch_meas = 0.0f;
    FlipSequencer::levelError(state.attitude, roll_meas, pitch_meas);

    roll_sp  = 0.0f;
    pitch_sp = 0.0f;
    rate_sp_roll  = att_roll_.compute(roll_sp, roll_meas, dt);
    rate_sp_pitch = att_pitch_.compute(pitch_sp, pitch_meas, dt);

    const bool near_level = FlipSequencer::tiltRad(state.attitude) <=
                            flip_.config.recover_boost_tilt_deg * (3.14159265f / 180.0f);
    if (flip_out.hold_yaw && near_level) {
        const math::Quat q(state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]);
        rate_sp_yaw = shortestPathYawRate(flip_.startYawRad(), q.to_euler().z,
                                          yaw_hold_kp_, yaw_hold_rate_max_);
    } else {
        rate_sp_yaw = 0.0f;
    }
}

// -----------------------------------------------------------------------------
// trackFlipPhaseTransition — edge-detect Spin/Recover entry and reset the
// rate-loop integrators exactly there (flip-maneuver-plan.md §3.3: "積分項は
// P2開始時とP4開始時にリセットする"), in addition to the reset onFlip()
// already does before Boost. Spin's reset clears any windup Boost's brief
// level-hold left behind before the accel ramp begins; Recover's reset
// clears deceleration-torque windup before the attitude PID re-engages.
// trackFlipPhaseTransition — Spin/Recover 突入をエッジ検出し、そこで正確に
// レートループ積分器をリセットする（plan §3.3: 「積分項はP2開始時とP4開始時に
// リセットする」）。onFlip() が Boost 前に既に行うリセットに加えて行う。
// Spin のリセットは、加速ランプが始まる前に Boost の短い水平保持が残した
// 巻き上がりを消す。Recover のリセットは、姿勢PID再係合前に減速トルクの
// 巻き上がりを消す。
// -----------------------------------------------------------------------------
void PidController::trackFlipPhaseTransition(FlipSequencer::Phase now)
{
    if (now == flip_prev_phase_) {
        return;
    }
    if (now == FlipSequencer::Phase::Spin || now == FlipSequencer::Phase::Recover) {
        rate_roll_.reset();
        rate_pitch_.reset();
        rate_yaw_.reset();
    }
    if (now == FlipSequencer::Phase::Recover) {
        // The attitude PID was idle through Spin/Brake; start its integrator and
        // derivative from zero against the (possibly large) tilt error.
        // 姿勢PIDは Spin/Brake の間は休止していた。（大きいことがある）傾き誤差に
        // 対し積分・微分をゼロから始める。
        att_roll_.reset();
        att_pitch_.reset();
    }
    flip_prev_phase_ = now;
}

}  // namespace sf
