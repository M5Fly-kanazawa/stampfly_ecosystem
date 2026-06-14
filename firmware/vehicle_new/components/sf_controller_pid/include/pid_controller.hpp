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
    bool isTakeoffComplete() const override { return takeoff_reached_; }
    void setGuidanceTarget(const GuidanceTarget& target,
                           const CommandSetpoint& current_sticks) override;
    bool isGuidanceActive() const override { return guidance_active_; }
    void startExcitation(const SysidCommand& cmd) override;

    /// One-shot fetch of a completed stepped-sine point (autotune). Returns
    /// true once per completed excitation; the TASK layer publishes it (core
    /// components must not touch topics — they also build in the smoke tests
    /// without FreeRTOS).
    /// 完了したステップドサイン点の一回限り取得（自動チューン）。励振完了ごとに
    /// 一度だけ true。publish は「タスク層」が行う（コア部品はトピックに触れない —
    /// FreeRTOS なしの smoke テストでもビルドされるため）。
    bool fetchSysidResult(SysidFreqResult& out);
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
    //   TakeoffClimb — auto-takeoff (ControllerCmd::Takeoff): the altitude cascade
    //                  climbs toward takeoff_target_alt_ (0.5m), velocity-limited to
    //                  takeoff_climb_rate_, with level attitude (POS: hold the launch
    //                  point). The cascade decelerates near the target, so the craft
    //                  CAPTURES the target altitude (no overshoot); isTakeoffComplete()
    //                  reports it and the state machine moves TAKEOFF→FLYING. The ToF
    //                  0.15m airborne edge is the ESKF vertical handoff only (ImuTask),
    //                  independent of this.
    //   Airborne     — normal mode law. ALT_HOLD holds the target captured at takeoff,
    //                  or the current altitude on an in-flight switch into ALT/POS
    //                  (onModeChange). STABILIZE/ACRO ignore the phase (manual throttle).
    // ALT_HOLD/POS_HOLD 則のための鉛直飛行フェーズ。これらのモードでは生スロットル→
    // 推力は意味を持たないため、フェーズが鉛直ループをゲートする:
    //   Grounded     — 地上で ARM 中: 推力を強制ゼロ（プロペラ停止。このゲートが
    //                  ないと ALT_HOLD は ARM した瞬間にホバー推力を指令してしまう）
    //   TakeoffClimb — 自動離陸（ControllerCmd::Takeoff）: 高度カスケードが
    //                  takeoff_target_alt_(0.5m) へ向かい、速度は takeoff_climb_rate_ に
    //                  制限、水平姿勢（POS は発進点保持）。目標近傍で減速するため機体は
    //                  目標高度を「捕捉」（オーバーシュートなし）。isTakeoffComplete() が
    //                  報告し、状態機械が TAKEOFF→FLYING を進める。ToF 0.15m 空中エッジは
    //                  ESKF 鉛直ハンドオフ専用（ImuTask）でこれとは独立。
    //   Airborne     — 通常のモード則。ALT_HOLD は離陸で捕捉した目標、または飛行中の
    //                  ALT/POS 切替では現在高度（onModeChange）を保持。STABILIZE/ACRO は
    //                  フェーズを無視（手動スロットル）。
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
    // ALT_HOLD manual stick rates (param altitude.climb_rate / .descent_rate, separately
    // tunable). Throttle stick is spring-centred (centre=hold); up commands a climb up to
    // max_climb_rate_, down a descent up to max_descent_rate_. Distinct from the AUTO rates
    // (takeoff_climb_rate_ / landing_descent_rate_). Legacy vehicle's MAX_CLIMB/DESCENT_RATE.
    // ALT_HOLD の手動スティック速度（param altitude.climb_rate / .descent_rate、別々に調整可）。
    // スロットルはバネ中央（中央=ホールド）、上=max_climb_rate_ まで上昇・下=max_descent_rate_
    // まで降下。自動レート（takeoff_climb_rate_ / landing_descent_rate_）とは別物。
    float max_climb_rate_   = 0.5f;  // [m/s] up   (= legacy MAX_CLIMB_RATE / ALT_OUTPUT_MAX)
    float max_descent_rate_ = 0.5f;  // [m/s] down (= legacy MAX_DESCENT_RATE)
    // Vertical-velocity-loop output limit [N]. Legacy vehicle/ VEL_OUTPUT_MAX:
    // the proven alt gains were tuned against this saturation, and it also bounds
    // how far the hover-thrust bias can pull the integrator.
    // 鉛直速度ループ出力上限 [N]。旧 vehicle/ の VEL_OUTPUT_MAX。実績高度ゲインは
    // この飽和と組で調整されており、ホバー推力偏りによる積分の引き込みも抑える。
    float max_thrust_correction_ = 0.15f;  // [N] (= legacy VEL_OUTPUT_MAX)
    float stick_deadzone_ = 0.1f;
    float alt_setpoint_   = 0;       // [m] captured altitude (ALT_HOLD target)
    bool  capture_alt_    = false;   // capture alt_setpoint on the next ALT_HOLD compute

    // Throttle re-center gate (2026-06-14 redesign). After an (auto-)takeoff or an
    // in-flight switch INTO ALT/POS, the throttle stick may rest off-center; treating
    // it immediately as a climb/descend command would jump the altitude. The gate
    // stays CLOSED (throttle ignored for altitude) until the stick is first seen within
    // the center deadzone, then OPENS so the stick commands climb/descent normally.
    // Closed by onTakeoff (Case A: ground ARM) and onModeChange into ALT/POS (Case B:
    // in-flight switch) and reset(). RC-only: guidance/API drives altitude via the
    // walking setpoint, not the throttle path, so the gate never blocks an API flight.
    // No timeout — re-centering the throttle is the pilot's natural next action.
    // スロットル再センターゲート（2026-06-14 再設計）。（自動）離陸後や飛行中の ALT/POS
    // 進入直後はスロットルが中央から外れていることがあり、それを即座に上昇/下降指令と
    // みなすと高度がジャンプする。ゲートはスティックが初めて中央デッドゾーン内に入るまで
    // 「閉」（高度に対しスロットル無視）で、その後「開」いて通常どおり上昇/下降を指令する。
    // onTakeoff（Case A: 地上 ARM）・ALT/POS への onModeChange（Case B: 飛行中切替）・
    // reset() で閉じる。RC 限定: 誘導/API は歩く設定点で高度を動かしスロットル経路を使わない
    // ため、API 飛行をゲートが妨げることはない。タイムアウトなし — スロットルを中央へ戻すのは
    // パイロットの自然な次動作。
    bool  throttle_recentered_ = false;
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
    SysidFreqResult sysid_pending_{};  // completed point awaiting fetch / 取得待ちの完了点
    bool     sysid_pending_valid_ = false;
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

    // Heading hold (STABILIZE and above). While the yaw stick is neutral, hold
    // the heading captured at stick release with a rate-limited P loop on the
    // estimator yaw (= bias-corrected gyro integral — no magnetometer needed).
    // Rationale: the rate loop nulls yaw RATE, not angle, so a yaw disturbance
    // torque random-walks the heading between pilot corrections. Flight-log
    // replay (2026-06-11, kp=3): hands-off heading excursion 12.3°→5.7° mean,
    // and the heading RETURNS to target instead of drifting. Pilot yaw input
    // wins instantly; guidance owns yaw in API flights; kp=0 disables.
    // ヘディングホールド（STABILIZE 以上）。ヨースティック中立の間、離した瞬間に捕捉
    // した方位を推定ヨー角（=バイアス補正済みジャイロ積分 — 地磁気不要）のレート制限
    // 付き P ループで保持する。理由: レートループはヨー「角速度」しか戻さないため、
    // ヨー外乱トルクで方位は操縦修正の合間にランダムウォークする。フライトログ再生
    // （2026-06-11, kp=3）: 手放し方位ずれ平均 12.3°→5.7°、かつ方位は流れず目標へ
    // 戻る。パイロットのヨー入力が常に優先。API 飛行では誘導がヨーを所有。kp=0 で無効。
    bool  yaw_hold_active_ = false;
    float yaw_hold_target_ = 0;        // [rad] captured heading / 捕捉方位
    float yaw_hold_kp_     = 3.0f;     // [1/s] P gain (param attitude.yawhold.kp)
    float yaw_hold_rate_max_ = 2.0f;   // [rad/s] correction rate limit (param attitude.yawhold.rate_max)
    // Engage gates: the stick deadband mirrors guidance's cancel threshold scale,
    // and the throttle floor keeps the hold OFF on the ground in STABILIZE
    // (ALT_HOLD+ gates on phase_ == Airborne instead — throttle is a climb
    // command there, not thrust).
    // 係合ゲート: スティック不感帯と、STABILIZE で地上では保持しないためのスロットル
    // 床値（ALT_HOLD 以上は phase_ == Airborne でゲート — そこではスロットルは上昇
    // 指令であり推力ではない）。
    static constexpr float kYawHoldStickDeadband = 0.03f;
    static constexpr float kYawHoldThrottleFloor = 0.25f;

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
    // rate: gentle, vertical-velocity-loop tracked, indoor-safe. It is the velocity
    // CLAMP on the altitude cascade's climb toward takeoff_target_alt_ — the position
    // loop decelerates below this rate as the target nears, capturing it smoothly.
    // 自動離陸の上昇率（TakeoffClimb フェーズ）。着陸降下率の鏡像: 穏やかで、鉛直速度
    // ループが追従し屋内でも安全。高度カスケードが takeoff_target_alt_ へ上昇する際の
    // 速度クランプで、目標が近づくと位置ループがこの率以下に減速し滑らかに捕捉する。
    float takeoff_climb_rate_ = 0.3f;     // [m/s] upward / 上向き上昇率

    // Auto-takeoff target altitude (TakeoffClimb phase, ALT/POS). The default hover
    // height after an ARM-triggered auto-takeoff — high enough to clear ground effect
    // (0.15m ToF airborne is the ESKF handoff, NOT this control target). SSOT for the
    // takeoff height: a core component cannot depend on main/config.hpp, so it lives
    // here alongside the other vertical constants (hover_thrust_, takeoff_climb_rate_);
    // the network-API takeoff inherits it by holding the post-takeoff altitude.
    // 自動離陸の目標高度（TakeoffClimb フェーズ, ALT/POS）。ARM 起動の自動離陸後の既定
    // ホバー高度 — 地面効果を抜ける高さ（0.15m ToF 空中検知は ESKF ハンドオフであって
    // この制御目標ではない）。離陸高度の SSOT: コア部品は main/config.hpp に依存できないため、
    // 他の鉛直定数（hover_thrust_, takeoff_climb_rate_）と並べてここに置く。ネットワーク
    // API の離陸は離陸後の高度を保持することでこれを継承する。
    float takeoff_target_alt_ = 0.5f;     // [m] / 目標高度

    // Auto-takeoff capture: TakeoffClimb → "reached" when the altitude settles within
    // kTakeoffCaptureBandM of the target at under kTakeoffCaptureVelMps vertical speed,
    // sustained kTakeoffSettleCycles control cycles (rejects a transient near-miss).
    // takeoff_reached_ is the controller-side TAKEOFF→FLYING signal (isTakeoffComplete).
    // 自動離陸の捕捉: TakeoffClimb は、高度が目標の kTakeoffCaptureBandM 以内・鉛直速度
    // kTakeoffCaptureVelMps 未満に整定し kTakeoffSettleCycles 周期持続したとき「到達」。
    // takeoff_reached_ が制御器側の TAKEOFF→FLYING 信号（isTakeoffComplete）。
    static constexpr float    kTakeoffCaptureBandM  = 0.05f;  // [m] ±5 cm of target
    static constexpr float    kTakeoffCaptureVelMps = 0.10f;  // [m/s] near-hover
    static constexpr uint16_t kTakeoffSettleCycles  = 20;     // 20 × 2.5ms = 50 ms
    bool     takeoff_reached_       = false;
    uint16_t takeoff_settle_cycles_ = 0;
};

}  // namespace sf
