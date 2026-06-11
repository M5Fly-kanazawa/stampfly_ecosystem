/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file pid_controller.hpp
 * @brief Cascade PID controller — IController implementation
 *        カスケードPIDコントローラ — IController実装
 *
 * @design requirements.md §4 — Component #6: replaceable control      [OK]
 * @design detailed_design.md §4 — IController                        [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid.hpp"
#include "sf_math.hpp"

namespace sf {

class PidController : public IController {
public:
    void init();

    ControlOutput compute(
        const StateEstimate& state,
        const CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(FlightMode new_mode) override;
    void onLanding() override;
    void onTakeoff() override;
    void onTakeoffComplete() override;
    void setGuidanceTarget(const GuidanceTarget& target,
                           const CommandSetpoint& current_sticks) override;
    void startExcitation(const SysidCommand& cmd) override;
    void reloadParams() override;

private:
    /// Load PID gains from parameter system / パラメータからゲインを読み込み
    void loadParams();

    /// Autonomous-landing control: level attitude + fixed descent rate.
    /// 自動着陸制御: 水平姿勢＋固定降下率。
    ControlOutput computeLanding(const StateEstimate& state, float dt);

    /// POS_HOLD cascade: position/velocity error → tilt setpoints (roll/pitch).
    /// Writes roll_sp/pitch_sp [rad], overriding the stick values in the caller.
    /// POS_HOLD カスケード: 位置/速度誤差 → 傾き指令（roll/pitch）。roll_sp/pitch_sp[rad]
    /// を書き込み、呼び出し側のスティック値を上書きする。
    void computePositionHold(const StateEstimate& state, float yaw, float dt,
                             float& roll_sp, float& pitch_sp);

    FlightMode current_mode_ = FlightMode::STABILIZE;

    // Vertical flight phase for the ALT_HOLD/POS_HOLD laws. Raw throttle→thrust
    // makes no sense in these modes, so the phase gates the vertical loop:
    //   Grounded     — armed on the ground: thrust forced to ZERO (props stopped;
    //                  without this gate ALT_HOLD would command hover thrust at ARM)
    //   TakeoffClimb — auto-takeoff (ControllerCmd::Takeoff): fixed climb rate +
    //                  level attitude (POS: hold the launch point) until airborne
    //   Airborne     — normal mode law (ALT_HOLD captures the current altitude on
    //                  entry). STABILIZE/ACRO ignore the phase (manual throttle).
    // ALT_HOLD/POS_HOLD 則のための鉛直飛行フェーズ。これらのモードでは生スロットル→
    // 推力は意味を持たないため、フェーズが鉛直ループをゲートする:
    //   Grounded     — 地上で ARM 中: 推力を強制ゼロ（プロペラ停止。このゲートが
    //                  ないと ALT_HOLD は ARM した瞬間にホバー推力を指令してしまう）
    //   TakeoffClimb — 自動離陸（ControllerCmd::Takeoff）: 空中検知まで固定上昇率＋
    //                  水平姿勢（POS は発進点保持）
    //   Airborne     — 通常のモード則（ALT_HOLD は突入時に現在高度を捕捉）。
    //                  STABILIZE/ACRO はフェーズを無視（手動スロットル）。
    enum class VerticalPhase : uint8_t { Grounded, TakeoffClimb, Airborne };
    VerticalPhase phase_ = VerticalPhase::Grounded;

    // Rate control PIDs (innermost loop) / レート制御PID（最内ループ）
    PID rate_roll_, rate_pitch_, rate_yaw_;

    // Attitude control PIDs (outer loop) / 姿勢制御PID（外ループ）
    PID att_roll_, att_pitch_;

    // Altitude control PIDs / 高度制御PID
    PID alt_pos_, alt_vel_;

    // Position control PIDs / 位置制御PID
    PID pos_x_, pos_y_;
    PID vel_x_, vel_y_;

    // Constants / 定数
    float max_rate_       = 1.0f;    // [rad/s] ACRO stick → rate sp scale (legacy ROLL/PITCH_RATE_MAX)
    float max_yaw_rate_   = 5.0f;    // [rad/s] max yaw rate (legacy YAW_RATE_MAX)
    float max_angle_      = 0.5236f; // [rad] max tilt (30 deg)
    // Attitude-loop output limit: how hard the angle loop may command the rate
    // loop. Distinct from max_rate_ (the ACRO stick scale): a 30° tilt error with
    // kp=5 wants 2.6 rad/s, so clamping at the 1.0 rad/s stick scale would cripple
    // STABILIZE recovery. Legacy vehicle/ MAX_RATE_SETPOINT = 3.0 rad/s.
    // 姿勢ループ出力上限: 角度ループがレートループに指令できる上限。max_rate_
    // （ACRO スティックスケール）とは別物 — 30° 誤差×kp=5 は 2.6 rad/s を要求するため
    // 1.0 rad/s で切ると STABILIZE の復元が利かない。旧 vehicle/ の
    // MAX_RATE_SETPOINT = 3.0 rad/s を踏襲。
    float max_att_rate_sp_ = 3.0f;   // [rad/s] attitude-loop output limit
    // Thrust output is PHYSICAL total thrust [N]: the B^-1 mixer (actuator.cpp)
    // allocates it across the motors and converts to duty via the motor curve.
    // max_thrust_ = 4 × max-per-motor (0.168 N) = 0.672 N (T/W ≈ 1.85).
    // hover_thrust_ = mg × 1.12: the 1.12 is the legacy vehicle/'s FLIGHT-MEASURED
    // correction (HOVER_THRUST_CORRECTION, stable 1.11–1.13 across logs) — the
    // motor curve over-promises thrust by ~12% on real hardware, and both
    // firmwares share that curve. In SIL the plant inverts the curve exactly, so
    // the +12% bias is absorbed by the velocity-loop integrator (within its
    // ±0.15 N limit).
    // スラスト出力は物理の総推力 [N]: B^-1 ミキサーが各モータに配分しモータ曲線で duty に。
    // max_thrust_ = 4×最大/モータ(0.168N) = 0.672N（T/W≈1.85）。
    // hover_thrust_ = mg×1.12: 1.12 は旧 vehicle/ の「飛行実測」補正
    // （HOVER_THRUST_CORRECTION、ログ全体で 1.11–1.13 と安定）— モータ曲線は実機で
    // 推力を約12%過大に見積もり、曲線は両ファーム共通。SIL ではプラントが曲線を厳密に
    // 逆変換するため +12% の偏りは速度ループ積分（±0.15N 上限内）が吸収する。
    float max_thrust_     = 0.672f;  // [N] total (4 × 0.168 N per motor)
    float hover_thrust_   = 0.407f;  // [N] mg × 1.12 = 0.037·9.80665·1.12
    float max_climb_rate_ = 0.5f;    // [m/s] (= legacy ALT_OUTPUT_MAX)
    // Vertical-velocity-loop output limit [N]. Legacy vehicle/ VEL_OUTPUT_MAX:
    // the proven alt gains were tuned against this saturation, and it also bounds
    // how far the hover-thrust bias can pull the integrator.
    // 鉛直速度ループ出力上限 [N]。旧 vehicle/ の VEL_OUTPUT_MAX。実績高度ゲインは
    // この飽和と組で調整されており、ホバー推力偏りによる積分の引き込みも抑える。
    float max_thrust_correction_ = 0.15f;  // [N] (= legacy VEL_OUTPUT_MAX)
    float stick_deadzone_ = 0.1f;
    float alt_setpoint_   = 0;       // [m] captured altitude (ALT_HOLD target)
    bool  capture_alt_    = false;   // capture alt_setpoint on the next ALT_HOLD compute
    float gravity_        = math::kGravity;  // [m/s²] accel→tilt mapping in POS_HOLD (SSOT: sf::math)
    float max_pos_tilt_   = 0.1745f; // [rad] POS_HOLD tilt limit (10 deg; matches the
                                     // proven firmware/vehicle margin so altitude holds
                                     // without 1/cosθ thrust compensation, and the small
                                     // tilt keeps |a|≈g so accel-attitude stays valid)
    float pos_setpoint_x_ = 0;       // [m] captured position N (POS_HOLD target, NED)
    float pos_setpoint_y_ = 0;       // [m] captured position E (POS_HOLD target, NED)
    bool  capture_pos_    = false;   // capture pos_setpoint on the next POS_HOLD compute

    // Guidance (Tello-style API / future Navigator, R11). While active the
    // POS_HOLD setpoints WALK toward guide_pos_ at guide_speed_ (the cascade
    // tracks the walking setpoint — same proven loops, just a moving target),
    // the altitude target follows guide_pos_[2], and yaw turns to guide_yaw_
    // with a rate-limited P loop. Pilot stick MOVEMENT (departure from the
    // snapshot taken when the target was set) cancels guidance instantly.
    // 誘導（Tello 風 API / 将来の Navigator, R11）。アクティブ中は POS_HOLD の設定点が
    // guide_pos_ へ guide_speed_ で「歩き」（カスケードは歩く設定点を追従 — 実績ループ
    // のまま目標だけ動く）、高度目標は guide_pos_[2] に従い、yaw はレート制限付き P で
    // guide_yaw_ に向く。スティックの「動き」（目標設定時のスナップショットからの逸脱）
    // で誘導は即時解除される。
    // Rate-loop sysid excitation (see IController::startExcitation). The signal
    // adds to ONE axis' rate setpoint; everything else flies normally. Safety:
    // amplitude clamped to ±1.5 rad/s, duration to 10 s, active only Airborne,
    // killed by reset()/mode change/landing.
    // レートループ同定励振。1軸のレート目標にのみ加算し、他は通常飛行。安全:
    // 振幅±1.5 rad/s・時間10 s にクランプ、Airborne 中のみ、reset()/モード切替/
    // 着陸で停止。
    bool    excite_active_   = false;
    uint8_t excite_axis_     = 0;
    uint8_t excite_waveform_ = 0;
    float   excite_amp_      = 0;      // [rad/s]
    float   excite_dur_      = 0;      // [s]
    float   excite_t_        = 0;      // [s] elapsed / 経過
    // Stepped-sine (waveform=2, autotune) state: excitation phase, settle gate
    // and the I/Q correlation sums of (u = actual axis torque, y = gyro).
    // ステップドサイン（waveform=2, 自動チューン）状態: 励振位相・整定ゲート・
    // （u=実トルク, y=ジャイロ）の I/Q 相関和。
    float   excite_freq_     = 0;      // [Hz]
    float   excite_phase_    = 0;      // [rad] accumulated / 積算位相
    float   excite_settle_s_ = 0;      // [s] transient to skip / 捨てる整定時間
    float   iq_ur_ = 0, iq_ui_ = 0, iq_yr_ = 0, iq_yi_ = 0;
    uint32_t iq_n_ = 0;
    uint32_t sysid_seq_ = 0;           // result sequence / 結果シーケンス
    static constexpr float kExciteAmpMax = 1.5f;   // [rad/s]
    static constexpr float kExciteDurMax = 10.0f;  // [s]
    static constexpr float kChirpF0      = 1.0f;   // [Hz] chirp start
    static constexpr float kChirpF1      = 25.0f;  // [Hz] chirp end
    static constexpr float kDoubletHalfS = 0.3f;   // [s] doublet half period

    bool  guidance_active_   = false;
    float guide_pos_[3]      = {0, 0, 0};  // [m] NED target / NED 目標
    float guide_yaw_         = 0;          // [rad] target yaw / 目標ヨー
    float guide_speed_       = 0.3f;       // [m/s] setpoint walk speed / 設定点速度
    float stick_snapshot_[4] = {0, 0, 0, 0};  // r,p,y,thr at engage / 設定時スティック
    float guide_yaw_kp_      = 2.0f;       // [1/s] yaw P gain / ヨー P ゲイン
    float guide_yaw_rate_max_ = 1.0f;      // [rad/s] yaw turn rate limit / ヨー回頭率上限
    float stick_move_cancel_ = 0.15f;      // stick departure to cancel / 解除閾値
    float max_pos_vel_    = 1.0f;    // [m/s] POS_HOLD horizontal velocity setpoint limit

    // Rate-loop output limits for the PID anti-windup (see loadParams). Each PID
    // clamps its output and gates its integrator at ±output_limit, so the limit
    // must be on the order of what the plant can deliver — with the default 1.0
    // the rate integrators could wind up to ~130× the available torque.
    // Values are the legacy vehicle/'s FLIGHT-PROVEN limits (ROLL/PITCH/YAW
    // _OUTPUT_LIMIT): the proven rate gains were tuned against these saturations.
    // They sit below the geometric maxima (roll/pitch 2·0.168 N·0.023 m ≈
    // 7.7e-3 Nm, yaw 2·0.168 N·κ ≈ 3.3e-3 Nm), leaving thrust headroom — full
    // differential torque would starve the collective.
    // レートループ出力上限（PID アンチワインドアップ用、loadParams 参照）。各 PID は
    // 出力と積分器を ±output_limit でゲートするため、上限はプラントが出せる量の
    // オーダーと一致させる必要がある — 既定 1.0 のままだと積分器は実トルクの約130倍
    // まで巻き上がる。値は旧 vehicle/ の「飛行実績」上限（*_OUTPUT_LIMIT）: 実績
    // レートゲインはこの飽和と組で調整されている。幾何最大値（ロール/ピッチ
    // 2·0.168N·0.023m≈7.7e-3 Nm、ヨー 2·0.168N·κ≈3.3e-3 Nm）より低く、総推力の
    // 余裕を残す — 差動トルクを使い切ると総推力が枯渇するため。
    float max_roll_pitch_torque_ = 5.2e-3f;  // [Nm] legacy ROLL/PITCH_OUTPUT_LIMIT
    float max_yaw_torque_        = 2.2e-3f;  // [Nm] legacy YAW_OUTPUT_LIMIT

    // Autonomous landing (ControllerCmd::Landing). While landing_ is set the
    // controller ignores the pilot setpoint entirely — the trigger conditions
    // (comm loss, battery emergency) mean the sticks are stale or unreliable.
    // Cleared by reset() (the next ARM). 0.3 m/s ≈ gentle indoor descent; the
    // ToF landing detector (TakeoffLandingMgr) ends the state at touchdown.
    // 自動着陸（ControllerCmd::Landing）。landing_ 中はパイロット setpoint を完全に
    // 無視する — 発動条件（通信断・電池緊急）はスティックが stale か信頼できない状況。
    // reset()（次の ARM）で解除。0.3 m/s は屋内の穏やかな降下率で、接地は ToF の
    // 着陸検出（TakeoffLandingMgr）が状態を終わらせる。
    bool  landing_              = false;
    float landing_descent_rate_ = 0.3f;   // [m/s] downward / 下向き降下率

    // Auto-takeoff climb rate (TakeoffClimb phase). Mirror of the landing descent
    // rate: gentle, vertical-velocity-loop tracked, indoor-safe. The phase ends at
    // the ToF airborne detection (TakeoffLandingMgr), typically well under 1 s.
    // 自動離陸の上昇率（TakeoffClimb フェーズ）。着陸降下率の鏡像: 穏やかで、鉛直速度
    // ループが追従し屋内でも安全。フェーズは ToF の空中検知（TakeoffLandingMgr）で
    // 終わり、通常 1 秒未満。
    float takeoff_climb_rate_ = 0.3f;     // [m/s] upward / 上向き上昇率
};

}  // namespace sf
