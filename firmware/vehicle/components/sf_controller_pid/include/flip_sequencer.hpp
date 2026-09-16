/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file flip_sequencer.hpp
 * @brief Angle-scheduled flip (宙返り) setpoint sequencer
 *        角度スケジュール型フリップの設定点列生成器
 *
 * Generates rate/attitude setpoints and a vertical-channel thrust command for
 * one 360-degree flip about the roll or pitch body axis. Holds NO control law
 * of its own (INV-1): during Spin/Brake it drives the EXISTING rate PID
 * directly (the same open-loop-setpoint path ACRO already uses), and during
 * Boost/Recover it hands roll/pitch back to the EXISTING attitude PID. It
 * only decides WHICH setpoint the existing pipeline should track, and for
 * how long — never a new control law.
 *
 * 1軸（ロール or ピッチ）まわりの360度フリップの設定点（レート/姿勢）と鉛直
 * チャネルの推力指令を生成する。自身の制御則は持たない（INV-1）: Spin/Brake
 * では既存のレートPIDを直接駆動し（ACROと同じ開ループ設定点経路）、
 * Boost/Recoverでは roll/pitch を既存の姿勢PIDに戻す。既存パイプラインが
 * 追う「設定点」と「どれだけの時間」だけを決め、新しい制御則は作らない。
 *
 * Phase state machine (flip-maneuver-plan.md §3.2/§7):
 *
 *   Boost -> Spin -> Brake -> Recover -> Done
 *            |  (spin_timeout: skip straight to Recover, no braking needed
 *            |   if rotation never picked up)
 *            +----------------------------------------------------------+
 *                                                                        v
 *                                                                    Recover
 *
 * @design docs/plans/flip-maneuver-plan.md §3.2 — phase table              [OK]
 * @design docs/plans/flip-maneuver-plan.md §3.4/§5.3 — parameter defaults  [OK]
 * @design docs/plans/flip-maneuver-plan.md §3.5 — abort/safety            [OK]
 * @design docs/plans/flip-maneuver-plan.md §7 — ramp command, final tune  [OK]
 * @design firmware/vehicle/docs/detailed_design.md §3 — FLIP row          [OK]
 * @design firmware/vehicle/docs/architecture.md INV-1 — one attitude+rate [OK]
 *         pipeline, no parallel control law
 */

#pragma once

#include "data_types.hpp"   // FlipDirection / FlipBlockReason / FlipResult
#include "sf_math.hpp"

namespace sf {

/// Angle-scheduled flip setpoint sequencer.
/// 角度スケジュール型フリップ列生成器。
class FlipSequencer {
public:
    // =========================================================================
    // Tunable parameters — all `flip.<name>` params (sf_core params.cpp).
    // PidController::loadParams() copies the live param values into this
    // struct's public fields every reload; FlipSequencer itself never touches
    // the param system, so it stays host-testable in isolation (see
    // test/test_flip_sequencer.cpp — no ESP-IDF, no NVS).
    // 調整可能パラメータ — 全て `flip.<name>` パラメータ（sf_core params.cpp）。
    // PidController::loadParams() が再読込のたびライブ値をこの構造体の公開
    // フィールドへコピーする。FlipSequencer 自身は param システムに一切触れない
    // ため、単体で host テスト可能（test/test_flip_sequencer.cpp — ESP-IDF・
    // NVS 不要）。
    // =========================================================================
    struct Config {
        // Rate-command peak, per rotation axis [deg/s] (param flip.rate_roll_dps
        // / flip.rate_pitch_dps). Under the ramp command below, the MEASURED
        // peak stays <=1700 deg/s (plan §5.3 v3 sweep); pitch is lower because
        // Iyy is 1.45x Ixx and needs more deceleration margin.
        // 回転軸別のレート指令ピーク [deg/s]（param flip.rate_roll_dps /
        // flip.rate_pitch_dps）。下のランプ指令下で「計測」ピークは1700deg/s
        // 以下（plan §5.3 v3掃引）。ピッチが低いのは Iyy が Ixx の1.45倍で
        // 減速余裕を優先するため。
        float rate_roll_dps  = 1500.0f;
        float rate_pitch_dps = 1400.0f;

        // Rate-setpoint ramp — the angular ACCELERATION OF THE COMMAND itself
        // (not a control gain), used for both the Spin ramp-up and the Brake
        // ramp-down [rad/s^2] (param flip.rate_ramp_rps2). A ramp beats a step
        // here: less mixer saturation and less altitude loss for the same
        // rotation (plan §5.3/§7 item 1 — this replaces the plan's earlier
        // brake_gain/k_b formula with a single alpha_cmd used both ways).
        // レート設定点のランプ — 「指令自体」の角加速度（制御ゲインではない）。
        // Spin の立上げと Brake の立下げ両方に使う [rad/s^2]
        // （param flip.rate_ramp_rps2）。同じ回転でもステップよりランプの方が
        // ミキサー飽和・高度損失とも少ない（plan §5.3/§7-1 —
        // brake_gain/k_b 式を、両方向に使う単一の alpha_cmd に置き換えた）。
        float rate_ramp_rps2 = 300.0f;

        // Boost duration [ms] (param flip.boost_ms): pre-spin climb so the
        // craft has upward momentum to spend while inverted. Altitude loss
        // flattens out past ~150ms (plan §5.3).
        // Boost 継続時間 [ms]（param flip.boost_ms）: 反転前に上昇速度を付け、
        // 反転中にその運動量を使い切る。約150ms以上で高度損失が頭打ち
        // （plan §5.3）。
        float boost_ms = 150.0f;

        // Thrust ratios, both relative to max_thrust_n (below) — NOT the
        // voltage-dependent ELECTRICAL thrust limit the plan's §5.3 numbers
        // used, because the controller has no per-voltage thrust-ceiling
        // model yet (flip-maneuver-plan.md §3.4 note). Using max_thrust_n
        // keeps the sequencer self-contained; a future voltage-aware ceiling
        // would only need to change what max_thrust_n is fed.
        // 推力比率。両方とも max_thrust_n（下記）に対する比率 — plan §5.3 の
        // 数値が使う「電圧依存の電気的上限」ではない（制御器はまだ電圧別推力
        // 上限モデルを持たないため。plan §3.4 注記）。max_thrust_n 基準にする
        // ことで列生成器は自己完結し、将来の電圧依存上限化は max_thrust_n に
        // 与える値を変えるだけで済む。
        float thrust_boost_ratio   = 0.9f;    // Boost / Recover
        float thrust_spin_hi_ratio = 0.5f;    // Spin accel/decel windows, Brake (0.5 = per-motor mid-scale: full +-torque headroom; 0.75 clamped the pitch brake in SILS 2026-09-16 / モータ中点で上下のトルク余裕が最大。0.75 では SILS でピッチの減速がクランプされた)

        // Reduced thrust during the inverted coast window (phi_a..phi_b) [N]
        // (param flip.thrust_lo_n) — a small POSITIVE value, not zero, so the
        // motors never fully stop (plan §5.3: 0 increases mixer saturation
        // and widens the attitude residual on the heavier pitch axis).
        // 反転中の惰性区間（φ_a〜φ_b）の減推力 [N]（param flip.thrust_lo_n）—
        // ゼロでなく小さな正値（plan §5.3: 0はミキサー飽和を増やし、慣性の
        // 大きいピッチ軸で姿勢残差も広げる）。
        float thrust_lo_n = 0.03f;

        // The controller's own max_thrust_ [N] — mirrored here by
        // PidController::loadParams() (NOT a `flip.*` NVS parameter) so the
        // ratios above convert to absolute newtons WITHOUT FlipSequencer
        // depending on PidController. Default matches PidController's own
        // max_thrust_ default, for host tests that never call loadParams().
        // 制御器自身の max_thrust_ [N] — PidController::loadParams() が
        // ここへ複写する（`flip.*` の NVS パラメータではない）。上の比率を
        // FlipSequencer が PidController に依存せず絶対値[N]へ変換できるように
        // する。既定値は loadParams() を呼ばない host テスト用に
        // PidController 自身の max_thrust_ 既定と一致させてある。
        float max_thrust_n = 0.672f;

        // Angle at which the accel window ends and the inverted coast window
        // begins [deg] (param flip.angle_a_deg): past this the thrust
        // vector's upward component is small, so spending torque headroom on
        // collective thrust stops paying off.
        // 加速窓が終わり反転惰性窓が始まる角度 [deg]（param flip.angle_a_deg）:
        // これを過ぎると推力ベクトルの上向き成分が乏しく、集合推力にトルク
        // 余裕を割く価値が薄れる。
        float angle_a_deg = 60.0f;

        // Motor response lag used in the brake-angle lookahead below [ms]
        // (param flip.motor_lag_ms) — matches the mixer/motor time constant
        // used elsewhere in this firmware.
        // 減速角の先読み計算に使うモータ応答遅れ [ms]（param flip.motor_lag_ms）
        // — ファーム他所のミキサー/モータ時定数と一致。
        float motor_lag_ms = 16.0f;

        // Brake-angle safety factor (param flip.brake_margin): the stopping
        // angle is computed with brake_margin * rate_ramp_rps2 as the achievable
        // deceleration, so braking starts EARLIER than the ideal ramp needs.
        // SILS api_flip (2026-09-16) showed the pitch axis (Iyy 1.45x roll)
        // still turning at ~950 deg/s when it reached 360 deg with margin 1.0,
        // overshooting to -31 deg and kicking the craft sideways under boost.
        // 減速角の安全係数（param flip.brake_margin）: 達成できる減速度を
        // brake_margin × rate_ramp_rps2 として制動角を計算し、理想のランプより
        // 早めに減速を始める。SILS api_flip（2026-09-16）で、係数 1.0 だとピッチ軸
        // （慣性がロールの 1.45 倍）が 360° 到達時にまだ約 950 °/s で回っており、
        // −31° まで行き過ぎて増強推力で横に蹴り出された。
        float brake_margin = 1.0f;

        // Recover thrust is boosted only while the true tilt is below this
        // angle (param flip.recover_boost_tilt_deg); when still tilted, the
        // collective stays at thrust_spin_hi so the thrust vector does not
        // throw the craft sideways while the attitude loop is still levelling.
        // Recover の増強推力は傾きがこの角度未満のときだけ（param
        // flip.recover_boost_tilt_deg）。まだ傾いている間は集合推力を
        // thrust_spin_hi に留め、姿勢ループが水平に戻す途中で推力ベクトルが
        // 機体を横へ押し出さないようにする。
        float recover_boost_tilt_deg = 20.0f;

        // Yaw-torque cap applied by PidController during the rate-only phases
        // (Spin/Brake) [N*m] (param flip.spin_yaw_torque_limit_nm). Yaw needs
        // tau/(4*kappa) of per-motor thrust, so an unconstrained yaw loop can
        // consume the mixer headroom the flip axis needs (SILS 2026-09-16:
        // pitch flip brake failed for this reason). Consumed by PidController,
        // not by the sequencer itself.
        // レートのみのフェーズ（Spin/Brake）で PidController が掛けるヨートルク
        // 上限 [N·m]（param flip.spin_yaw_torque_limit_nm）。ヨーは 1 モータあたり
        // τ/(4κ) の推力差を要するため、無制限のヨーループは回転軸に必要な
        // ミキサーの余裕を食う（SILS 2026-09-16: これでピッチの減速が失敗）。
        // 列生成器自身ではなく PidController が使う。
        float spin_yaw_torque_limit_nm = 0.3e-3f;

        // Flip-axis rate-loop torque limit during Spin/Brake [N*m] (param
        // flip.spin_torque_limit_nm), applied by PidController in place of the
        // normal-flight clamp (rate.roll/pitch.max_torque 5.2e-3, a flight-
        // proven value below the mixer's geometric maximum of about 7.7e-3 at
        // mid-scale collective). The flip needs the extra authority on the
        // heavier pitch axis to brake inside 360 deg (SILS 2026-09-16).
        // Spin/Brake 中の回転軸レートループのトルク上限 [N·m]（param
        // flip.spin_torque_limit_nm）。PidController が通常飛行の上限
        // （rate.roll/pitch.max_torque 5.2e-3、飛行実績値で、中点集合推力での
        // ミキサー幾何学的上限 約 7.7e-3 より低い）の代わりに掛ける。慣性の
        // 大きいピッチ軸が 360° 以内で減速するのに必要（SILS 2026-09-16）。
        float spin_torque_limit_nm = 7.0e-3f;

        // Brake ramp-down of the rate command [rad/s^2] (param
        // flip.brake_ramp_rps2), separate from the accel ramp above: braking
        // may use the full spin_torque_limit_nm authority, and a faster brake
        // shortens the inverted time. Also the deceleration assumed by the
        // brake-angle lookahead. SILS 2026-09-16: with the 300 rad/s^2 ramp the
        // pitch axis reached 383 deg before stopping.
        // レート指令の減速ランプ [rad/s²]（param flip.brake_ramp_rps2）。加速
        // ランプとは別: 減速は spin_torque_limit_nm の権限をフルに使ってよく、
        // 速い減速は反転時間を短くする。減速角の先読みで仮定する減速度でもある。
        // SILS 2026-09-16: 300 rad/s² のランプではピッチ軸が 383° まで回った。
        float brake_ramp_rps2 = 500.0f;

        // Post-flip settle window (param flip.settle_ms / flip.settle_tilt_deg):
        // for settle_ms after FlipComplete the position loop's tilt command is
        // capped at settle_tilt_deg and the yaw torque at
        // spin_yaw_torque_limit_nm, so the outer loops cannot starve the mixer
        // (and the altitude loop) while the craft still carries the flip's
        // horizontal velocity. SILS 2026-09-16: without it the position loop
        // demanded the full 10 deg and the yaw loop its full clamp right after a
        // pitch flip, motors saturated, and the craft descended at 0.7 m/s.
        // 宙返り直後の整定窓（param flip.settle_ms / flip.settle_tilt_deg）:
        // FlipComplete から settle_ms の間、位置ループの傾き指令を settle_tilt_deg、
        // ヨートルクを spin_yaw_torque_limit_nm に抑え、宙返りで付いた水平速度が
        // 残る間に外側ループがミキサー（と高度ループ）の余裕を食い尽くさない
        // ようにする。SILS 2026-09-16: これが無いとピッチ宙返り直後に位置ループが
        // 10° 一杯、ヨーループが上限一杯を要求してモータが飽和し 0.7 m/s で降下した。
        float settle_ms       = 1500.0f;
        float settle_tilt_deg = 5.0f;

        // Attitude-loop handoff conditions (plan §3.2/§3.3): the craft leaves
        // Brake for Recover once it has turned past handoff_min_deg AND its
        // MEASURED rate has decayed below handoff_rate_dps, or unconditionally
        // past handoff_force_deg (a safety backstop if the rate never decays).
        // 姿勢ループへの引き渡し条件（plan §3.2/§3.3）: handoff_min_deg を
        // 過ぎ、かつ「計測」レートが handoff_rate_dps 未満に落ちたら
        // Brake→Recover。または handoff_force_deg を過ぎたら無条件
        // （レートが落ちない場合の安全弁）。
        float handoff_min_deg   = 290.0f;
        float handoff_rate_dps  = 300.0f;
        float handoff_force_deg = 350.0f;

        // Abort timeouts and limits (plan §3.5).
        // 打ち切りのタイムアウト・上限（plan §3.5）。
        float recover_timeout_ms = 800.0f;   // Recover never reaches vz>=0        / Recover が vz>=0 に到達しない
        float spin_timeout_ms    = 700.0f;   // Spin never reaches phi_brake       / Spin が phi_brake に到達しない
        float gyro_abort_dps     = 1800.0f;  // measured |omega| nears the 2000dps gyro range / ジャイロ計測範囲への接近

        // Execution-condition gates C2-C7 (flip-maneuver-plan.md §3.1),
        // checked every cycle by ready(). C1 (FlightState::FLYING) is judged
        // by StateManager and C8 (source conflict) by PidController — neither
        // is evaluated here (plan §4.1/§4.2 INV-3: detection vs. judgment).
        // 実行条件 C2-C7（plan §3.1）、ready() が毎周期判定。C1
        // （FlightState::FLYING）は StateManager、C8（他ソースとの競合）は
        // PidController が判定し、ここでは扱わない（plan §4.1/§4.2 INV-3:
        // 検出と判断の分離）。
        float min_height_m  = 1.0f;
        float min_voltage_v = 3.6f;
        float max_tilt_deg  = 15.0f;
        float max_rate_dps  = 60.0f;
        float max_hvel_mps  = 0.3f;
        float max_vvel_mps  = 0.2f;
        float cooldown_ms   = 2000.0f;
    };

    /// Phase of the flip state machine (flip-maneuver-plan.md §3.2/§7).
    /// フリップ状態機械のフェーズ（plan §3.2/§7）。
    enum class Phase : uint8_t {
        Idle,     // not engaged / 未係合
        Boost,    // pre-spin climb, level attitude / 反転前の上昇、水平姿勢
        Spin,     // rate-loop-only spin-up + inverted coast / レートループのみで加速+反転惰性
        Brake,    // rate-loop-only deceleration / レートループのみで減速
        Recover,  // attitude-loop level hold + climb / 姿勢ループで水平保持+上昇
        Done,     // terminal — call reset() to arm the next flip / 終端 — 次のフリップは reset() で再係合
    };

    /// One cycle's estimator input (flip-maneuver-plan.md §3.1/§3.2/§3.6).
    /// 1周期分の推定器入力（plan §3.1/§3.2/§3.6）。
    struct Input {
        float gyro[3];   // [rad/s] body FRD, bias-corrected / 機体FRD、バイアス補正後
        float quat[4];   // [w,x,y,z] estimator attitude / 推定姿勢
        float height_m;                  // [m] above ground / 対地高度
        float vertical_velocity_up_mps;  // [m/s] up-positive / 上向き正
        float horizontal_speed_mps;      // [m/s] |horizontal velocity| / 水平速度の大きさ
        float battery_v;                 // [V] live pack voltage / 実電源電圧
        bool  estimator_ok;              // estimator-health fact / 推定器健全性の事実
        bool  tof_valid;                 // ToF observation currently usable / ToF観測が使用可能
        float dt;                        // [s] this cycle's time step / この周期のタイムステップ
    };

    /// One cycle's setpoint output (flip-maneuver-plan.md §3.2/§3.3/§4.1).
    /// 1周期分の設定点出力（plan §3.2/§3.3/§4.1）。
    struct Output {
        // true: route roll/pitch through the EXISTING attitude PID
        // (Boost/Recover/Done). false: drive the EXISTING rate PID directly —
        // the same open-loop-setpoint path ACRO already uses (Spin/Brake).
        // true: roll/pitch を「既存の」姿勢PIDへ通す（Boost/Recover/Done）。
        // false: 「既存の」レートPIDを直接駆動する。ACROと同じ開ループ設定点
        // 経路（Spin/Brake）。
        bool  attitude_loop;
        float rate_sp[3];  // [rad/s] R,P,Y — meaningful only if !attitude_loop / attitude_loop=false時のみ有効
        float roll_sp;     // [rad] — meaningful only if attitude_loop (always 0) / attitude_loop=true時のみ有効（常に0）
        float pitch_sp;    // [rad] — meaningful only if attitude_loop (always 0) / 同上
        bool  hold_yaw;    // true: hold the yaw captured at start() / true: start() 時のヨーを保持
        float thrust_n;    // [N] vertical-channel override, every phase / 鉛直チャネル上書き、全フェーズ共通
    };

    /// Begin a flip in `dir`, capturing the yaw/height to return to.
    /// dir 方向のフリップを開始し、復帰先のヨー/高度を取り込む。
    void start(FlipDirection dir, float yaw_now, float height_now);
    /// Altitude captured at start() [m] — the altitude target to restore on
    /// FlipComplete (plan §4.1 onExit: 高度目標＝取り込み値).
    /// start() で取り込んだ高度 [m] — FlipComplete で戻す高度目標（plan §4.1）。
    float startHeightM() const { return start_height_m_; }

    /// Advance one control cycle and return this cycle's setpoints. Only
    /// meaningful while active(); the caller must not call this from Idle.
    /// 1制御周期進め、この周期の設定点を返す。active() の間のみ意味を持ち、
    /// 呼び出し側は Idle から呼んではならない。
    Output update(const Input& input);

    /// True from start() until reset() — Phase::Done included, so the caller
    /// keeps routing through the sequencer's (held, steady) output for the
    /// short window between reaching Done and the state machine consuming
    /// flip_done (see reset()'s doc for why reset(), not Done, is the edge).
    /// start() から reset() まで true — Phase::Done を含む。Done 到達から
    /// 状態機械が flip_done を消費するまでの短い窓でも、呼び出し側は
    /// 列生成器の（保持された定常）出力を使い続けることになる（reset() の
    /// ドキュメント参照 — Done でなく reset() が境界である理由）。
    bool active() const { return phase_ != Phase::Idle; }

    /// True once the terminal Phase::Done is reached; `result` is the
    /// outcome. `result` is filled EVERY call (even before Done, and even
    /// after reset() returns this to Idle) so a caller publishing it as
    /// telemetry (ControllerStatus.flip_result) always sees the last known
    /// outcome, not a transient None right after Done clears.
    /// 終端 Phase::Done に達したら true。`result` は毎回書き込む（Done 到達
    /// 前でも、reset() で Idle に戻った後でも）— テレメトリ
    /// （ControllerStatus.flip_result）として発行する側が、Done 直後の
    /// 一過性 None ではなく常に最後の結果を見られるようにするため。
    bool done(FlipResult& result) const;

    /// Current phase (telemetry/tests). / 現在のフェーズ（テレメトリ/テスト用）。
    Phase phase() const { return phase_; }

    /// Rotation angle accumulated since start() [deg] (telemetry/tests).
    /// start() からの積算回転角 [deg]（テレメトリ/テスト用）。
    float rotationAngleDeg() const;

    /// Yaw captured at start() [rad] (used by PidController's Boost/Recover
    /// heading hold — see pid_controller.cpp computeFlipAttitude()).
    /// start() で取り込んだヨー [rad]（PidController の Boost/Recover
    /// ヘディングホールドが使う — pid_controller.cpp の
    /// computeFlipAttitude() 参照）。
    float startYawRad() const { return start_yaw_; }

    /// Return to Idle and arm the C7 cooldown (plan §3.1) if a flip was in
    /// progress or had just finished. Deliberately does NOT clear the last
    /// `result_` — flip_result must stay readable (e.g. by the Tello API,
    /// which polls ControllerStatus after state_task's FLIP->FLYING) until
    /// the NEXT start() begins a new attempt. Safe to call from Idle (the
    /// cooldown is untouched then).
    /// Idle へ戻し、フリップが進行中/完了直後なら C7 のクールダウンを起動する
    /// （plan §3.1）。意図的に最後の `result_` はクリアしない — flip_result は
    /// （state_task の FLIP→FLYING 後に ControllerStatus をポーリングする
    /// Tello API 等のために）次の start() が新しい試行を始めるまで読み出せる
    /// 必要がある。Idle からの呼び出しも安全（その場合クールダウンは変更しない）。
    void reset();

    /// Evaluate execution conditions C2-C7 (plan §3.1). Advances the C7
    /// cooldown timer by input.dt, so call this every cycle while FLYING —
    /// even while a flip that started earlier is still active() (harmless;
    /// active() alone already implies not-ready via FlipBlockReason::Busy,
    /// judged by the caller, see PidController::isFlipReady()).
    /// 実行条件 C2-C7（plan §3.1）を評価する。input.dt で C7 クールダウンを
    /// 進めるため、FLYING 中は毎周期呼ぶこと — 既に進行中の（先に始まった）
    /// フリップの間に呼んでも無害（active() 自体が
    /// FlipBlockReason::Busy を通じて不成立を意味する。判定は呼び出し側、
    /// PidController::isFlipReady() 参照）。
    bool ready(const Input& input, FlipBlockReason& reason) const;

    Config config;   // live parameters — see the Config doc above / ライブパラメータ — 上記 Config 参照

private:
    Phase         phase_     = Phase::Idle;
    FlipDirection direction_ = FlipDirection::Right;
    int   axis_ = 0;      // 0=roll, 1=pitch / 0=ロール, 1=ピッチ
    float sign_ = 1.0f;   // +1 or -1, flip-maneuver-plan.md §3.2 table

    float start_yaw_    = 0.0f;   // [rad] captured at start() / start()時に取り込み
    float start_height_m_ = 0.0f; // [m] captured at start(), telemetry only / start()時に取り込み（テレメトリ用）
    float phi_rad_    = 0.0f;     // [rad] accumulated rotation angle, always >=0 / 積算回転角、常に0以上
    float rate_cmd_   = 0.0f;     // [rad/s] ramped rate setpoint, signed / ランプ済みレート設定点（符号付き）
    float phase_elapsed_s_ = 0.0f;
    FlipResult result_ = FlipResult::None;

    // C7 cooldown countdown. `mutable` because ready() is logically a const
    // query (plan: "毎周期評価"), but must still advance this timer as a
    // side effect every time it is called — a textbook mutable-timer case,
    // scoped to this one field only (everything else stays non-mutable).
    // C7 クールダウンのカウントダウン。`mutable` なのは、ready() が論理的には
    // const な問い合わせ（plan: 「毎周期評価」）でありながら、呼ばれるたびに
    // このタイマだけは副作用として進める必要があるため — 教科書的な
    // mutable タイマの用例で、この1フィールドだけに限定する（他は
    // 非mutableのまま）。
    mutable float cooldown_remaining_s_ = 0.0f;

    // Fixed safety margin between the brake-entry angle and the point the
    // accel window's high thrust resumes (flip-maneuver-plan.md §3.2:
    // "phi_b = phi_brake - 20 deg"). Kept out of Config because it is a
    // fixed geometric margin, not a tunable knob — see plan §3.2.
    // ブレーキ開始角と加速窓の高推力再開点との安全マージン（plan §3.2:
    // 「φ_b=φ_brake-20°」）。固定の幾何マージンであり調整対象でないため
    // Config に含めない — plan §3.2 参照。
    static constexpr float kBrakePrepMarginDeg = 20.0f;
    static constexpr float kDegToRad = 3.14159265358979f / 180.0f;
    static constexpr float kRadToDeg = 180.0f / 3.14159265358979f;

    void  enterPhase(Phase next);
    void  accumulatePhi(const Input& input);
    float rampRate(float current, float target, float dt, float ramp_rps2) const;
    float axisRateDps() const;
    float brakeAngleDeg(float measured_rate_dps) const;
    float spinThrustN(float phi_deg, float phi_brake_deg) const;
    float recoverThrustN(const Input& input) const;
    bool  withinSteadyBounds(const Input& input) const;
    Output outputAttitudeLevel(float thrust_n) const;
    Output outputRotating(float rate_cmd, float thrust_n) const;
    Output updateBoost(const Input& input);
    Output updateSpin(const Input& input);
    Output updateBrake(const Input& input);
    Output updateRecover(const Input& input);
    Output updateDone() const;
};

}  // namespace sf
