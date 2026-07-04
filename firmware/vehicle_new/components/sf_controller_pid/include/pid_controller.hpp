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
 * @design detailed_design.md §3 注4/注5/注6 — takeoff/landing law    [OK]
 * @design architecture.md INV-1 — one attitude pipeline, all phases  [OK]
 * @design architecture.md INV-2 — pilot keeps attitude; level only   [OK]
 *         on a dead link (setpoint staleness, R16)
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

    /// POS_HOLD cascade: position/velocity error → tilt setpoints (roll/pitch).
    /// Writes roll_sp/pitch_sp [rad], overriding the stick values in the caller.
    /// Roll/pitch sticks REPOSITION the hold point (deflect to move, release to hold): a
    /// deflected stick commands a horizontal velocity; releasing it re-captures the
    /// current position as the new hold target.
    /// POS_HOLD カスケード: 位置/速度誤差 → 傾き指令（roll/pitch）。roll_sp/pitch_sp[rad]
    /// を書き込み、呼び出し側のスティック値を上書きする。roll/pitch スティックは保持点を
    /// 「再配置」する（倒して動かし、離して保持）: 倒すと水平速度を指令、離すと現在位置を
    /// 新しい保持目標として再捕捉する。
    void computePositionHold(const StateEstimate& state, const CommandSetpoint& setpoint,
                             float yaw, float dt, float& roll_sp, float& pitch_sp);

    FlightMode current_mode_ = FlightMode::STABILIZE;

    // Vertical flight phase. INV-1 (architecture.md): ALL phases share the ONE
    // attitude+rate pipeline in compute() — a phase may change ONLY the vertical
    // channel (thrust / climb / descent) and its own exit condition, never the
    // attitude law. There is deliberately NO separate per-phase control function.
    // The phase gates the vertical loop (raw throttle→thrust makes no sense in
    // ALT/POS, and Landing must descend regardless of mode):
    //   Grounded     — armed on the ground: thrust forced to ZERO (props stopped;
    //                  without this gate ALT_HOLD would command hover thrust at ARM)
    //   TakeoffClimb — auto-takeoff (ControllerCmd::Takeoff): the altitude cascade
    //                  climbs toward takeoff_target_alt_ (0.5m), velocity-limited to
    //                  takeoff_climb_rate_. Attitude is the PILOT's (INV-2) — only the
    //                  vertical channel is automated. The cascade decelerates near the
    //                  target, so the craft CAPTURES it (no overshoot); isTakeoffComplete()
    //                  reports it and the state machine moves TAKEOFF→FLYING. The ToF
    //                  0.15m airborne edge is the ESKF vertical handoff only (ImuTask).
    //   Airborne     — normal mode law. ALT_HOLD holds the target captured at takeoff,
    //                  or the current altitude on an in-flight switch into ALT/POS
    //                  (onModeChange). STABILIZE/ACRO ignore the phase (manual throttle).
    //   Landing      — autonomous landing (ControllerCmd::Landing): the vertical channel
    //                  descends at landing_descent_rate_ regardless of flight mode (a
    //                  comm-loss landing can start from ACRO/STABILIZE, which have no
    //                  altitude loop). Attitude stays the PILOT's WHILE THE LINK IS LIVE
    //                  (a pilot-commanded landing is steerable, INV-2); on a stale link
    //                  (comm-loss failsafe, no pilot) the single level gate forces
    //                  roll/pitch/yaw to zero. Touchdown is detected by TakeoffLandingMgr
    //                  (INV-3), which drives LANDING → IDLE_GROUND → disarm.
    // 鉛直飛行フェーズ。INV-1: 全フェーズが compute() の単一姿勢+レートパイプラインを共有
    // し、フェーズが変えてよいのは鉛直チャネル（推力/上昇/降下）と自身の脱出条件のみ。姿勢則は
    // 変えない。フェーズ別の制御関数は意図的に持たない。フェーズは鉛直ループをゲートする:
    //   Grounded     — 地上 ARM 中: 推力を強制ゼロ（プロペラ停止）
    //   TakeoffClimb — 自動離陸: 高度カスケードが目標(0.5m)へ速度制限上昇。姿勢はパイロット
    //                  （INV-2, 鉛直のみ自動）。目標近傍で減速し捕捉、isTakeoffComplete() が報告。
    //   Airborne     — 通常のモード則（捕捉した目標 or 飛行中切替で現在高度を保持）。
    //   Landing      — 自動着陸: 鉛直チャネルが landing_descent_rate_ で降下（モード非依存=
    //                  ACRO/STABILIZE からの通信途絶着陸にも対応）。リンク生存中は姿勢は
    //                  パイロット（操縦可, INV-2）、リンク途絶（パイロット不在）時のみ単一の
    //                  水平ゲートで roll/pitch/yaw を 0 にする。接地は TakeoffLandingMgr が検出
    //                  （INV-3）し LANDING→IDLE_GROUND→disarm へ。
    enum class VerticalPhase : uint8_t { Grounded, TakeoffClimb, Airborne, Landing };
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

    // POS_HOLD stick repositioning (deflect to move, release to hold). A deflected roll/pitch
    // stick commands a horizontal velocity (body frame) into the velocity loop instead
    // of the position-hold output; releasing the stick re-captures the current position
    // as the new hold target, so the craft stops where the pilot let go. Velocity scale
    // is param position.stick_vel. Sticks are deadbanded upstream (command.cpp), so a
    // centred stick reads exactly 0 = "hold". reposition_active_ tracks the moving state
    // so the falling edge (stick → neutral) triggers the re-capture.
    // POS_HOLD スティック再配置（倒して動かし、離して保持）。roll/pitch スティックを倒すと
    // 位置保持出力でなく水平速度（機体座標）を速度ループに指令し、離すと現在位置を新しい
    // 保持目標として再捕捉する → パイロットが離した位置で止まる。速度スケールは param
    // position.stick_vel。スティックは上流（command.cpp）で不感帯処理済ゆえ中央は厳密に
    // 0 =「保持」。reposition_active_ は移動状態を追跡し、立下りエッジ（中立復帰）で再捕捉する。
    float stick_reposition_vel_ = 0.4f;  // [m/s] max stick reposition speed (param position.stick_vel)
    bool  reposition_active_    = false; // pilot is repositioning via stick

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
    // Coherence/SNR gate: accumulate the gyro I/Q at an OFF-tone (a nearby UNexcited
    // frequency) to measure the disturbance/noise FLOOR, and a slow running mean of the
    // rate to DETREND the near-DC disturbance (CW/CCW trim) before the lock-in. The
    // per-point coh = on/(on+off) then down-weights disturbed frequencies in the fit.
    // コヒーレンス/SNRゲート: オフ音(非励振の近傍周波数)でジャイロI/Qを積算し雑音床を測る＋レートの
    // 遅い走査平均で近DC外乱(CW/CCWトリム)を除トレンドしてからロックインする。
    float   excite_phase_off_ = 0;     // [rad] off-tone phase / オフ音位相
    float   iq_yr_off_ = 0, iq_yi_off_ = 0;   // off-tone gyro I/Q (noise floor)
    float   y_dc_ = 0;                  // slow running mean of the rate (detrend) / レートの遅い平均
    uint32_t sysid_seq_ = 0;           // result sequence / 結果シーケンス
    SysidFreqResult sysid_pending_{};  // completed point awaiting fetch / 取得待ちの完了点
    bool     sysid_pending_valid_ = false;
    static constexpr float kExciteAmpMax = 1.5f;   // [rad/s]
    static constexpr float kExciteDurMax = 10.0f;  // [s]
    static constexpr float kChirpF0      = 1.0f;   // [Hz] chirp start
    static constexpr float kChirpF1      = 25.0f;  // [Hz] chirp end
    static constexpr float kDoubletHalfS = 0.3f;   // [s] doublet half period

    bool  guidance_active_   = false;
    uint8_t guide_mode_      = 0;          // 1=position seek, 2=velocity(rc) / 1=位置シーク,2=速度
    float guide_pos_[3]      = {0, 0, 0};  // [m] NED target (mode 1) / NED 目標
    float guide_yaw_         = 0;          // [rad] target yaw (mode 1) / 目標ヨー
    float guide_speed_       = 0.3f;       // [m/s] setpoint walk speed (mode 1) / 設定点速度
    // Velocity command (mode 2 — Tello `rc`). Body frame: x fwd, y right, z up;
    // yaw cw+. Injected into the SAME reposition path the pilot sticks use
    // (computePositionHold) and the ALT_HOLD climb-rate path — no parallel law (INV-1).
    // guide_stamp_ is the last rc timestamp; if it goes stale (>kLandingLinkStaleUs)
    // the velocities decay to 0 so a stopped client cannot run away (R16).
    // 速度指令（mode 2 — Tello `rc`）。機体系 x前/y右/z上, yaw cw+。パイロットスティックと
    // 同じ再配置経路（computePositionHold）＋ALT_HOLD 上昇率経路に注入 — 並列制御則なし（INV-1）。
    // guide_stamp_ は最終 rc 時刻。古く（>kLandingLinkStaleUs）なれば速度を 0 に減衰し暴走防止（R16）。
    float guide_vx_          = 0;          // [m/s] body forward (mode 2) / 機体前後
    float guide_vy_          = 0;          // [m/s] body right   (mode 2) / 機体左右
    float guide_vz_          = 0;          // [m/s] climb rate up+ (mode 2) / 上昇率
    float guide_vyaw_        = 0;          // [rad/s] yaw rate cw+ (mode 2) / ヨーレート
    uint32_t guide_stamp_    = 0;          // [us] last velocity-target stamp / 最終速度目標時刻
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

    // Attitude trim (equilibrium tilt) applied to the angle-loop SETPOINT at the
    // attitude confluence — one injection for every mode (STABILIZE / ALT_HOLD /
    // POS_HOLD / Landing), after the POS_HOLD override and the Landing level gate.
    // The angle loop drives the craft to this tilt, cancelling steady horizontal
    // drift (CG offset / sensor-level bias) with no extra thrust. Flight-identified
    // (sf trim analyze); the true equilibrium tilt is unknowable on the ground.
    // 姿勢トリム（平衡傾き）。姿勢合流点で角度ループの「目標」に加算 — 全モード
    // （STABILIZE / ALT_HOLD / POS_HOLD / Landing）で1点、POS_HOLD 上書きと Landing
    // 水平ゲートの後。角度ループが機体をこの傾きへ駆動し、CG オフセットやセンサ水平
    // バイアス由来の定常水平ドリフトを推力を余分に食わず打ち消す。飛行で同定（sf trim
    // analyze）— 真の平衡傾きは地上で知り得ない。
    float roll_trim_  = 0.0f;  // [rad] roll equilibrium tilt (param attitude.roll.trim)
    float pitch_trim_ = 0.0f;  // [rad] pitch equilibrium tilt (param attitude.pitch.trim)

    // --- Always-on onboard trim learning (hover-gated) ---
    // Slowly drives roll_trim_/pitch_trim_ to cancel steady horizontal drift while
    // hovering, so the pilot need not run sf trim analyze by hand. StateEstimate has
    // NO acceleration state, so the observation is the ESKF velocity differentiated to
    // a body-frame accel and EMA-smoothed; integrated with time constant kTrimLearnTau;
    // clamped to kTrimMax (= the param range); persisted to NVS on the landing edge.
    // See learnTrim(). Signs match sf trim analyze (SIL-verified).
    // 常時オンボード・トリム学習（ホバー限定）。ホバー中に roll_trim_/pitch_trim_ を
    // ゆっくり動かし定常水平ドリフトを打ち消す（手動 sf trim analyze 不要）。StateEstimate
    // に加速度状態が無いので、観測は ESKF 速度の微分による機体加速度を EMA 平滑、時定数
    // kTrimLearnTau で積分、kTrimMax（=param 範囲）にクランプ、着陸エッジで NVS 保存。
    // learnTrim() 参照。符号は sf trim analyze（SIL 検証済み）と一致。
    static constexpr float kTrimMax            = 0.1f;   // [rad] trim clamp (= param range)
    static constexpr float kTrimLearnTau       = 20.0f;  // [s] learning time constant
    static constexpr float kTrimLearnAccelHz   = 0.3f;   // [Hz] accel EMA cutoff (rejects ~99% of the 400Hz differentiation noise, passes <0.1Hz drift)
    static constexpr float kTrimLearnStickDead = 0.08f;  // [-] roll/pitch neutral band
    bool  trim_learn_enable_ = true;             // param attitude.trim.learn (0 = off, manual only)
    bool  trim_learn_init_   = false;            // first-hover-sample guard
    float trim_vel_prev_[2]  = {0.0f, 0.0f};     // [m/s] prev NED horiz velocity (N,E)
    float trim_accel_lpf_[2] = {0.0f, 0.0f};     // [m/s^2] EMA body accel (fwd,right)
    VerticalPhase trim_prev_phase_ = VerticalPhase::Grounded;  // landing-edge detect
    void learnTrim(const StateEstimate& state, const CommandSetpoint& setpoint,
                   float yaw, float dt);

    // --- Always-on onboard hover-thrust learning (steady-hover-gated) ---
    // Robustness to thrust variation (motor wear over flight time, battery sag): hover
    // thrust is otherwise a FIXED feed-forward (mg·corr) and only the bounded velocity-loop
    // correction (±max_thrust_correction_) adapts, so a motor that weakens beyond the
    // correction's reach makes the craft sink / under-climb — then hover.thrust_corr must be
    // hand-tuned per flight (a band-aid). This learner slowly transfers the STEADY velocity-
    // loop output into hover_thrust_, so the feed-forward tracks the TRUE hover thrust and
    // the correction re-centres (full ±limit authority restored for climb/disturbance). The
    // vertical analogue of learnTrim() — a "hover thrust estimator" (cf. PX4/ArduPilot).
    // Persisted to hover.thrust_corr on the landing edge, so degradation self-compensates
    // across flights WITHOUT per-flight tuning. See learnHoverThrust().
    // 常時オンボード・ホバー推力学習（定常ホバー限定）。推力変動（飛行時間によるモータ劣化・
    // 電圧サグ）へのロバスト化: ホバー推力は固定FF（mg·corr）で適応するのは制限付き補正
    // （±max_thrust_correction_）のみゆえ、補正の届く範囲を超えてモータが弱ると機体が沈む／
    // 上昇不足（→ hover.thrust_corr のフライト毎手調整＝場当たり）。本学習は定常ホバーの速度
    // ループ定常出力をゆっくり hover_thrust_ に移し、FF を真のホバー推力へ追従させ補正を
    // 中心化（離陸／外乱に ±上限の全権限が復活）。learnTrim() の鉛直版（"hover thrust
    // estimator"）。着陸エッジで hover.thrust_corr に保存し、劣化を手調整なしで自己補償。
    // @design architecture.md INV-1 — vertical channel only; one pipeline  [OK]
    static constexpr float kMassG            = 0.037f * 9.80665f;  // mg [N]
    static constexpr float kHoverLearnTau    = 20.0f;   // [s] learning time constant
    static constexpr float kHoverLearnVzDead = 0.30f;   // [m/s] reject only FAST transients; the slow tau averages the bob, and a gentle sink (FF too low) MUST be learnable
    static constexpr float kHoverCorrMin     = 0.5f;    // hover.thrust_corr clamp (= params.cpp range)
    static constexpr float kHoverCorrMax     = 2.0f;
    bool  hover_learn_enable_ = true;            // param hover.thrust.learn (0 = off, manual corr only)
    VerticalPhase hover_prev_phase_ = VerticalPhase::Grounded;  // landing-edge detect for NVS persist
    void learnHoverThrust(float thrust_correction, float vz_up, float climb_rate_sp, float dt);
    void persistHoverThrust();   // NVS save on the landing edge (called every cycle)

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

    // Autonomous landing descent rate (VerticalPhase::Landing — set by onLanding,
    // cleared by reset() at the next ARM). 0.3 m/s ≈ gentle indoor descent; the
    // touchdown detector (TakeoffLandingMgr) ends the state at the ground (INV-3).
    // Landing is NOT a separate control path — it is a vertical phase in compute()
    // (INV-1); attitude stays the pilot's while the link is live (INV-2).
    // 自動着陸の降下率（VerticalPhase::Landing — onLanding で設定、reset()=次の ARM で
    // 解除）。0.3 m/s は屋内の穏やかな降下率で、接地は TakeoffLandingMgr が検出（INV-3）。
    // Landing は別経路でなく compute() の鉛直フェーズ（INV-1）、姿勢はリンク生存中パイロット（INV-2）。
    float landing_descent_rate_ = 0.3f;   // [m/s] downward / 下向き降下率

    // Near-ground settle assist (Landing). A constant-velocity descent is balanced by
    // ground effect near the floor (more rotor lift on reduced thrust), so the craft
    // lingers and stalls. Below kLandingSettleAltM the thrust CEILING ramps from hover
    // toward kLandingSettleThrustFrac·hover over kLandingSettleRampS, so a craft still
    // floating is positively settled within a bounded time — gentle (the velocity loop
    // works under the ceiling) yet decisive (the ceiling falls below ground-effect lift).
    // landing_settle_t_ accumulates time spent below the altitude trigger (reset by
    // onLanding / reset()). See pid_controller.cpp Landing branch + detailed_design §3 注8.
    // 近地面の着地アシスト（Landing）。定速降下は接地近傍で地面効果と釣り合い機体が粘る。
    // kLandingSettleAltM 未満で推力上限を hover→kLandingSettleThrustFrac·hover へ
    // kLandingSettleRampS かけて絞り、まだ浮く機体を有界時間で確実に沈める（速度ループは上限内
    // で動くので穏やか、上限が地面効果揚力を下回るので確実）。landing_settle_t_ は高度トリガ未満で
    // 過ごした時間を積算（onLanding / reset() でクリア）。
    static constexpr float kLandingSettleAltM      = 0.15f;  // [m] altitude to start the settle ramp / アシスト開始高度
    static constexpr float kLandingSettleThrustFrac = 0.70f; // [-] ceiling floor = frac·hover / 上限の床
    static constexpr float kLandingSettleRampS     = 1.0f;   // [s] ramp duration hover→floor / ランプ時間
    float landing_settle_t_ = 0.0f;       // [s] time below the settle altitude / アシスト高度未満の経過時間

    // Landing steerability gate: the pilot keeps roll/pitch/yaw during a landing
    // ONLY while the command link is live. Link-liveness = setpoint freshness
    // (R16): if (state.timestamp − setpoint.timestamp) exceeds this, the link is
    // assumed lost (comm-loss failsafe, no pilot) and the descent is forced LEVEL.
    // Matches the Failsafe comm-loss timeout so the two agree. No separate flag is
    // threaded from the StateManager — freshness alone tells us if the pilot is there,
    // and it also self-levels if the link drops mid-descent.
    // 着陸の操縦ゲート: リンク生存中のみパイロットが roll/pitch/yaw を保つ。生存判定は設定点
    // の新鮮さ（R16）: (state.timestamp − setpoint.timestamp) がこれを超えたらリンク途絶
    // （通信途絶フェイルセーフ＝パイロット不在）とみなし水平降下に強制。Failsafe の通信途絶
    // タイムアウトと一致させ両者を整合。StateManager からフラグを配線しない — 新鮮さだけで
    // パイロットの在否が分かり、降下中にリンクが切れても自動で水平化する。
    static constexpr uint32_t kLandingLinkStaleUs = 500000;  // 500 ms / R16 timeout

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

    // Auto-takeoff complete (TakeoffClimb → Airborne, the controller-side TAKEOFF→FLYING
    // signal isTakeoffComplete). ROBUST one-sided "reached" + timeout backstop:
    //   reached  = altitude has climbed to within kTakeoffCaptureBandM of the target
    //              (one-sided, altitude >= target - band) sustained kTakeoffSettleCycles.
    //   backstop = after kTakeoffMaxCycles in TakeoffClimb, complete unconditionally.
    // The earlier two-sided band (|target-alt|<band) + low-velocity-settle never fired on
    // hardware: a steady-state offset (real hover altitude ≠ setpoint) or vertical-velocity
    // noise kept it out of the tight window, so the craft stayed in TakeoffClimb with the
    // roll/pitch sticks dead-sticked level (pid_controller.cpp lines ~190). The one-sided
    // reach is offset-immune (it fires on the way UP), and the timeout guarantees we always
    // leave TakeoffClimb so the pilot regains attitude control. Airborne ALT_HOLD then holds
    // alt_setpoint_ = target (decision ②) and arrests any climb overshoot.
    // 自動離陸完了（TakeoffClimb→Airborne、制御器側 TAKEOFF→FLYING 信号 isTakeoffComplete）。
    // ロバストな「片側到達」＋タイムアウト・バックストップ:
    //   到達 = 高度が目標の kTakeoffCaptureBandM 以内まで上昇（片側, altitude>=target-band）を
    //          kTakeoffSettleCycles 持続。backstop = TakeoffClimb で kTakeoffMaxCycles 経過で無条件完了。
    // 旧来の両側バンド+低速整定は実機で発火しなかった: 定常偏差（実保持高度≠setpoint）や鉛直速度
    // ノイズで窓に入らず、機体が TakeoffClimb に留まりロール/ピッチが0固定された（compute 行~190）。
    // 片側到達は偏差非依存（上昇途中で発火）、タイムアウトは必ず TakeoffClimb を抜けることを保証し
    // パイロットが姿勢制御を取り戻す。Airborne ALT_HOLD は alt_setpoint_=target を保持（決定②）し
    // 行き過ぎを吸収する。
    static constexpr float    kTakeoffCaptureBandM = 0.05f;  // [m] reach within 5 cm below target
    static constexpr uint16_t kTakeoffSettleCycles = 20;     // 20 × 2.5ms = 50 ms sustained
    static constexpr uint16_t kTakeoffMaxCycles    = 1200;   // 3 s @ 400Hz timeout backstop
    bool     takeoff_reached_        = false;
    uint16_t takeoff_settle_cycles_  = 0;
    uint16_t takeoff_elapsed_cycles_ = 0;   // time in TakeoffClimb, for the timeout backstop
};

}  // namespace sf
