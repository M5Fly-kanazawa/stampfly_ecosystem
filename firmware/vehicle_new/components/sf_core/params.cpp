/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file params.cpp
 * @brief Parameter system and topic instance implementation
 *        パラメータシステムおよびトピックインスタンス実装
 *
 * @design detailed_design.md §6 — Parameter system                    [OK]
 * @design requirements.md §3 — Parameter management                   [OK]
 */

#include "topics.hpp"
#include "params.hpp"
#include "esp_log.h"
#include "esp_timer.h"   // reload-callback timestamps / 再読込コールバックの時刻印
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cstdio>   // snprintf (NVS key derivation) / snprintf（NVSキー導出）
#include <cmath>

static const char* TAG = "Params";

namespace sf {

// =============================================================================
// Topic instances (defined here, declared extern in topics.hpp)
// トピックインスタンス（ここで定義、topics.hppでextern宣言）
// =============================================================================

Topic<ImuData,         RingBuffer, 8>  sensor_imu;
Topic<TofData,         Queue, 2>       sensor_tof;
Topic<FlowData,        Queue, 2>       sensor_flow;
Topic<MagData,         Queue, 2>       sensor_mag;
Topic<BaroData,        Queue, 2>       sensor_baro;
Topic<PowerData,       Latest, 1>      sensor_power;
Topic<SensorSnapshot,  Latest, 1>      sensor_snapshot;
Topic<StateEstimate,   Latest, 1>      estimate_state;
Topic<CommandSetpoint, Latest, 1>      command_setpoint;
Topic<PilotRequest,    Latest, 1>      pilot_request;
Topic<ButtonEvent,     Queue, 4>       button_event;
Topic<ControlOutput,   Latest, 1>      control_output;
Topic<ControllerStatus, Latest, 1>     controller_status;
Topic<MotorOutput,     Latest, 1>      actuator_motor;
Topic<LogStreamSample, RingBuffer, 32> log_stream;
Topic<FlowData,        RingBuffer, 8>  log_flow;
Topic<SystemMode,      Latest, 1>      system_mode;
Topic<SystemAlert,     Queue, 4>       system_alert;
Topic<SystemStatus,    Latest, 1>      system_status;
Topic<PairingStatus,   Latest, 1>      pairing_state;
Topic<PairingComplete, Latest, 1>      pairing_complete;
Topic<UiCommand,       Queue, 4>       ui_command;
Topic<MotorTest,       Latest, 1>      motor_test;
Topic<MagCalCommand,   Queue,  2>      mag_command;
Topic<ApiCommand,      Queue,  4>      api_command;
Topic<SysidCommand,    Queue,  2>      sysid_command;
Topic<SysidFreqResult, Latest, 1>      sysid_result;
Topic<MagCalStatus,    Latest, 1>      mag_cal_status;
Topic<EstimatorCommand,  Queue, 4>     estimator_command;
Topic<ControllerCommand, Queue, 4>     controller_command;
Topic<NotifyCommand,     Queue, 8>     notify_command;
Topic<SensorHealth,      Latest, 1>    sensor_health;
Topic<GuidanceTarget,    Latest, 1>    command_target;
Topic<NavigationPath,    Queue, 4>     nav_path;

void topics_init()
{
    sensor_imu.init();
    sensor_tof.init();
    sensor_flow.init();
    sensor_mag.init();
    sensor_baro.init();
    sensor_power.init();
    sensor_snapshot.init();
    estimate_state.init();
    command_setpoint.init();
    pilot_request.init();
    button_event.init();
    control_output.init();
    controller_status.init();
    actuator_motor.init();
    log_stream.init();
    log_flow.init();
    system_mode.init();
    system_alert.init();
    system_status.init();
    pairing_state.init();
    pairing_complete.init();
    ui_command.init();
    motor_test.init();
    mag_command.init();
    api_command.init();
    sysid_command.init();
    sysid_result.init();
    mag_cal_status.init();
    estimator_command.init();
    controller_command.init();
    notify_command.init();
    sensor_health.init();
    command_target.init();
    nav_path.init();
}

// =============================================================================
// Parameter System Implementation
// パラメータシステム実装
//
// @design detailed_design.md §6 — Parameter table (SSOT = params.cpp) [OK]
// =============================================================================

// Explicit parameter variable definitions
// 明示的なパラメータ変数定義
namespace param_vars {
    // Rate control
    // Rate gains are PHYSICAL [Nm/(rad/s)] (the mixer is a B^-1 allocation,
    // actuator.cpp). Values are the FLIGHT-PROVEN legacy vehicle/ gains
    // (config.hpp rate_control, physical-units mode) — directly transferable
    // because both firmwares share the same loop structure (Tustin PID with
    // D-on-M, η=0.125), the same B^-1 mixer geometry (d=0.023 m, κ=0.00971)
    // and the same motor curve, so the plant seen by the rate loop is identical.
    // Earlier SIL-derived near-P values (kp = I/τ_resp, ti=20) are superseded.
    // レートゲインは物理 [Nm/(rad/s)]（ミキサーは B^-1 配分）。値は旧 vehicle/ の
    // 「飛行実績ゲイン」（config.hpp rate_control 物理単位モード）— 両ファームは
    // ループ構造（Tustin PID・測定値微分・η=0.125）、ミキサー幾何（d=0.023m,
    // κ=0.00971）、モータ曲線が同一で、レートループから見たプラントが同じため
    // そのまま移植できる。以前の SIL 由来 near-P 値（kp=I/τ_resp, ti=20）は置換。
    float rate_roll_kp    = 1.365e-3f; // legacy ROLL_RATE_KP
    float rate_roll_ti    = 0.7f;      // legacy ROLL_RATE_TI
    float rate_roll_td    = 0.01f;
    float rate_pitch_kp   = 1.995e-3f; // legacy PITCH_RATE_KP
    float rate_pitch_ti   = 0.7f;      // legacy PITCH_RATE_TI
    float rate_pitch_td   = 0.01f;
    float rate_yaw_kp     = 5.31e-3f;  // legacy YAW_RATE_KP (3x: yaw authority)
    float rate_yaw_ti     = 1.6f;      // legacy YAW_RATE_TI
    float rate_yaw_td     = 0.01f;

    // Scheduled autotune (solo pilot, hands-free): a single operator cannot type
    // `autotune` mid-flight, so SET these on the GROUND, then arm and fly. After the
    // craft has been FLYING for sched_delay seconds, the rate-loop autotune runs
    // automatically on sched_axis (a beep cues the pilot to hold a steady hover).
    // One-shot per flight; -1 = OFF. Disable by setting axis back to -1.
    // スケジュール autotune（ソロ操縦・ハンズフリー）: 飛行中に `autotune` を打てないため、
    // 地上で設定→離陸。FLYING 到達から sched_delay 秒後に sched_axis のレート autotune が
    // 自動起動（ブザーで合図、定位置ホバー保持を促す）。1飛行1回・-1=OFF。
    int32_t autotune_sched_axis  = -1;     // -1=off, 0=roll, 1=pitch, 2=yaw
    float   autotune_sched_delay = 20.0f;  // [s] FLYING dwell before firing

    // Autotune system-identification result, per axis. Written by the onboard autotune
    // whenever the plant FIT succeeds (even if the gain design is then rejected, e.g.
    // a thin-margin yaw) — so the identified model is retained for analysis. NOT applied
    // to control (read-back only). Persisted with `param save`. Identified plant per axis:
    //   G(s) = b * e^{-L s} / (s (T s + 1));  b = gain, tau = T [s], delay = L [s],
    //   resid = fit residual (lower = better). 0 = not yet identified.
    // autotune システム同定結果（軸ごと）。プラントのフィット成功時に必ず記録（ゲイン設計が
    // 棄却される軸=余裕の薄い yaw 等でも同定結果は残す）。制御には未使用（読み出し専用）。
    // `param save` で永続。同定プラント: G(s)=b·e^{-Ls}/(s(Ts+1))、tau=T[s]、delay=L[s]、
    // resid=フィット残差（小さいほど良）。0=未同定。
    float autotune_roll_b    = 0.0f, autotune_roll_tau    = 0.0f,
          autotune_roll_delay  = 0.0f, autotune_roll_resid  = 0.0f;
    float autotune_pitch_b   = 0.0f, autotune_pitch_tau   = 0.0f,
          autotune_pitch_delay = 0.0f, autotune_pitch_resid = 0.0f;
    float autotune_yaw_b     = 0.0f, autotune_yaw_tau     = 0.0f,
          autotune_yaw_delay   = 0.0f, autotune_yaw_resid   = 0.0f;

    // Autotune design-margin result, per axis. Written by the onboard autotune right
    // after the loop-shaping design (tunePid) succeeds — BEFORE the GM-floor / gain-range
    // gates — so the margins are kept even when the design is then REJECTED (e.g. a thin
    // yaw GM): you can read WHY it was rejected. Read-back only, persisted with `param save`.
    //   wc = achieved crossover [rad/s], pm = phase margin [deg], gm = gain margin [dB]
    //   (gm = 99 means no −180° crossing in the sweep, i.e. effectively infinite/safe).
    //   0 = not yet designed.
    // autotune 設計余裕結果（軸ごと）。ループ整形設計(tunePid)成功直後＝GM下限/ゲイン範囲ゲートの
    // 前に記録するため、設計が棄却される軸(余裕の薄い yaw 等)でも余裕が残り「なぜ棄却されたか」が
    // 読める。読み出し専用・`param save` で永続。wc=交差[rad/s]、pm=位相余裕[deg]、gm=ゲイン余裕[dB]
    // （gm=99 は掃引中に −180°交差なし＝実質無限大/安全）。0=未設計。
    float autotune_roll_wc  = 0.0f, autotune_roll_pm  = 0.0f, autotune_roll_gm  = 0.0f;
    float autotune_pitch_wc = 0.0f, autotune_pitch_pm = 0.0f, autotune_pitch_gm = 0.0f;
    float autotune_yaw_wc   = 0.0f, autotune_yaw_pm   = 0.0f, autotune_yaw_gm   = 0.0f;

    // Estimator selection (RESET_PLAN P2: replaceable estimation). The IMU task's
    // factory reads this: 0 = ESKF (15-state), 1 = complementary filter. The SIL
    // bench swaps estimators via this parameter alone — no code change.
    // 推定器の選択（P2: 差し替え可能）。IMU タスクのファクトリが読む: 0=ESKF, 1=相補。
    int32_t estimator_type = 0;

    // Telemetry WiFi mode (boot-time, sf_comm initWifi): 0 = STA — join the
    // router whose SSID/password are stored in NVS via the CLI `wifi` command
    // (unconfigured → ESP-NOW-only, telemetry inert); 1 = SoftAP — the vehicle
    // serves "StampFly-XXXX" on the ESP-NOW channel (no infrastructure needed).
    // ESP-NOW control works in every mode.
    // テレメトリ WiFi モード（起動時, sf_comm initWifi）: 0 = STA — CLI `wifi`
    // コマンドで NVS に保存した SSID/パスワードのルータへ接続（未設定なら
    // ESP-NOW のみ・テレメトリ無効）; 1 = SoftAP — 機体が ESP-NOW チャネル上で
    // "StampFly-XXXX" を提供（インフラ不要）。ESP-NOW 操縦は全モードで動く。
    int32_t wifi_mode = 0;

    // Blackbox SPIFFS logger enable (0 = OFF default, 1 = ON). DEFAULT OFF because the
    // SPIFFS write done while ARMED triggers a flash erase that disables the flash
    // cache and STALLS BOTH CORES ~37ms every ~0.5s — the control loop freezes and the
    // craft drifts (a periodic yaw "kick"). WiFi telemetry (sf log wifi) already covers
    // analysis. Enable only when the onboard log is truly needed and the periodic stall
    // is acceptable. Proper fix (future): buffer in RAM, write on DISARM only.
    // Blackbox SPIFFS ロガー有効化（0=既定OFF, 1=ON）。既定OFF — ARM 中の SPIFFS 書き込みは
    // フラッシュ消去でフラッシュキャッシュを無効化し両コアを ~0.5 秒ごとに ~37ms 停止させる
    // （制御ループ凍結→機体ドリフト＝周期的ヨーキック）。解析は WiFi テレメトリで足りる。
    // 本当に必要かつ周期ストールを許容できる時のみ ON。恒久対策(将来)=RAM 緩衝し DISARM で書込。
    int32_t log_blackbox_enable = 0;

    // Attitude control
    float att_roll_kp     = 5.0f;
    float att_roll_ti     = 4.0f;
    float att_roll_td     = 0.04f;
    float att_pitch_kp    = 5.0f;
    float att_pitch_ti    = 4.0f;
    float att_pitch_td    = 0.04f;

    // Heading hold (STABILIZE+, yaw stick neutral): P gain [1/s] on the estimator
    // yaw and the correction turn-rate limit [rad/s]. kp=0 disables the hold.
    // Defaults from the 2026-06-11 flight-log replay (excursion 12.3°→5.7° mean).
    // ヘディングホールド（STABILIZE 以上・ヨースティック中立時）: 推定ヨー角への
    // P ゲイン [1/s] と補正回頭率上限 [rad/s]。kp=0 で無効。既定値は 2026-06-11 の
    // フライトログ再生で決定（方位ずれ平均 12.3°→5.7°）。
    float att_yawhold_kp       = 3.0f;
    float att_yawhold_rate_max = 2.0f;

    // Altitude control — cascade alt → vertical-velocity → thrust [N].
    // Values are the FLIGHT-PROVEN legacy vehicle/ gains (config.hpp
    // altitude_control, "PI-v1": alt 0.6/7.0 → vel 0.1/2.5). The legacy velocity
    // loop also output physical thrust [N] (VEL_OUTPUT_MAX 0.15 N), so the units
    // match and the gains transfer 1:1. The earlier, stronger SIL-tuned values
    // (1.5/8 → 0.3/2) are superseded by the hardware-proven set.
    // 高度制御 — カスケード 高度→鉛直速度→推力[N]。値は旧 vehicle/ の飛行実績ゲイン
    // （config.hpp altitude_control「PI-v1」: alt 0.6/7.0 → vel 0.1/2.5）。旧の速度
    // ループも物理推力[N]出力（VEL_OUTPUT_MAX 0.15N）で単位が一致し、そのまま移植
    // できる。以前の強めの SIL 調整値（1.5/8 → 0.3/2）は実機実績値で置換。
    float alt_alt_kp      = 0.6f;
    float alt_alt_ti      = 7.0f;
    float alt_vel_kp      = 0.1f;
    float alt_vel_ti      = 2.5f;
    // ALT_HOLD manual stick rates (separately tunable). The throttle stick is
    // spring-centred (centre = hold); push up → climb at climb_rate, push down →
    // descend at descent_rate. Mirrors the flight-proven legacy vehicle's
    // MAX_CLIMB_RATE / MAX_DESCENT_RATE (separate constants).
    // ALT_HOLD の手動スティック速度（別々にチューニング可）。スロットルはバネ中央
    // （中央=ホールド）、上=climb_rate で上昇・下=descent_rate で降下。旧 vehicle の
    // MAX_CLIMB_RATE / MAX_DESCENT_RATE（別定数）を踏襲。
    float alt_climb_rate   = 0.5f;   // [m/s]
    float alt_descent_rate = 0.5f;   // [m/s]

    // Hover thrust correction (HOVER_THRUST_CORRECTION): hover_thrust = mg × corr.
    // The idealized motor curve over-promises thrust, so worn hardware needs corr
    // ≈ 1.12 (flight-measured) to actually hover. FRESH/stronger motors produce
    // MORE thrust per duty → corr must DROP (else auto-takeoff over-climbs and the
    // rate loop runs hot). Tune from a hover log: corr_new = 1.12 × (duty_new/duty_old).
    // ホバー推力補正: hover_thrust = mg × corr。理想モータ曲線は推力を過大評価するため、
    // 摩耗ハードは corr≈1.12（飛行実測）でホバー。新品/強いモータは同 duty で推力が大きい
    // → corr を下げる（さもないと自動離陸が過上昇しレートループが過敏化）。ホバーログから
    // corr_new = 1.12 ×（新duty/旧duty）で調整。
    float hover_thrust_corr = 1.12f;

    // Position control
    float pos_pos_kp      = 1.0f;
    float pos_pos_ti      = 5.0f;
    float pos_vel_kp      = 0.8f;   // tight hold: enabled by the accel-comp estimator fix
    float pos_vel_ti      = 2.0f;   // (the old 0.3 was the limit with the contaminated est)

    // ESKF process noise
    float eskf_gyro_noise   = 0.009655f;
    float eskf_accel_noise  = 0.3f;
    float eskf_gyro_bias    = 0.000013f;
    float eskf_accel_bias   = 0.0001f;

    // Gyro-bias deviation limit around the boot-calibration nominal [rad/s]
    // (PX4-style bias limiting — bounds how far any sensor anomaly can drag the
    // bias that feeds the rate loop; see EskfConfig::bg_deviation_max).
    // 起動校正ノミナルまわりのジャイロバイアス偏差上限 [rad/s]（PX4 流バイアス制限 —
    // センサ異常がレートループ用バイアスを引きずれる距離を有界化。
    // EskfConfig::bg_deviation_max 参照）。
    float eskf_bg_dev_max   = 0.03f;

    // ESKF observation noise
    // tof_noise lowered 0.03→0.01: the flight-log offline replay showed the ToF innovation
    // NIS ≪ 1 at 0.03 (over-conservative — ToF tracks within <1 cm), so 0.01 trusts ToF more
    // for tighter vertical tracking (altlog REPORT §4).
    // tof_noise を 0.03→0.01: 実機ログ再生で ToF イノベ NIS≪1（0.03 は保守的すぎ、ToF は
    // <1cm で追従）→ 0.01 で ToF を信用し鉛直追従を締める。
    float eskf_tof_noise      = 0.01f;
    float eskf_flow_noise     = 0.30f;
    float eskf_baro_noise     = 0.1f;
    float eskf_mag_noise      = 1.0f;
    // Accel-attitude observation noise σ [m/s²]. History: 0.06→0.8 cured the χ² latch-up
    // (chi2_latchup_finding). Then 0.8→1.2 from the flight-log offline replay: at 0.8 the
    // χ² REJECTION on real data is ~10 % (over-rejecting the x-axis 11.9 Hz airframe
    // vibration), it hits the ideal ~5 % at 1.2, and over-rejects-the-other-way (1.3 %, more
    // accel-bias drift) at 2.0. 1.2 is the data-optimum; pair it with eskf_accel_att_lpf
    // (the SIL n2 vibration is isotropic and could not show this — the real x-axis mode is
    // the driver; see altlog REPORT §3–4).
    // 0.06→0.8 で χ² ラッチアップ解消、さらに 0.8→1.2 を実機ログ再生で確定: 0.8 は実データで
    // χ² 棄却 ~10%（x軸 11.9Hz 機体振動を過剰棄却）、1.2 で理想 ~5%、2.0 で過小棄却＋バイアス
    // ドリフト増。1.2 がデータ最適。eskf_accel_att_lpf と併用（SIL の n2 振動は等方的で実機の
    // x軸モードを欠くため SIL では出ない）。
    float eskf_accel_att      = 1.2f;
    // Accel-attitude LPF cutoff [Hz] (0 = off). 30 Hz cleans the airframe vibration from the
    // gravity reference; the offline sweep cut accel-bias drift 0.28→0.18 (12 Hz notch did
    // NOT help — broadband). Applied to the attitude update only, NOT predict.
    // accel 姿勢 LPF カットオフ[Hz]（0=無効）。30Hz で重力基準から機体振動を清浄化、掃引で
    // バイアスドリフト 0.28→0.18（12Hz ノッチは広帯域ゆえ無効）。姿勢更新のみ、predict には不適用。
    float eskf_accel_att_lpf  = 30.0f;

    // ESKF sensor enable
    bool eskf_use_tof   = true;
    bool eskf_use_flow  = true;
    bool eskf_use_baro  = false;
    bool eskf_use_mag   = false;

    // ESKF gates
    float eskf_mahalanobis  = 15.0f;
    float eskf_tof_innov    = 0.5f;
    float eskf_baro_innov   = 0.5f;
    float eskf_flow_clamp   = 0.3f;
    int32_t eskf_flow_squal = 10;   // min PMW3901 SQUAL to fuse flow (L-1)

    // ESKF accel-attitude (proven firmware/vehicle values). Registered here as the
    // single source of truth so they are NOT silently taken from the struct defaults.
    // 加速度-姿勢（実証済み firmware/vehicle 値）。SSOT として登録し struct 既定値に
    // 暗黙依存しないようにする。
    float eskf_att_k_adaptive = 10.0f;   // adaptive R: R *= (1 + k|a-g|²)
    float eskf_att_chi2_gate  = 7.81f;   // χ²(3, 0.95) accel-attitude outlier gate
    float eskf_att_corr_clamp = 0.05f;   // [rad] per-update roll/pitch correction clamp

    // ESKF acceleration-compensated accel-attitude (POS_HOLD). The accelerometer measures
    // specific force f = a_kin − g; during a horizontal maneuver the kinematic term a_kin
    // is mistaken for a tilt and the attitude sticks at the "apparent gravity" angle
    // atan(a/g), so POS_HOLD flies away. An α-β tracker on the flow velocity estimates
    // a_kin (state = velocity + acceleration; β small so the SUSTAINED drift acceleration
    // is captured, not washed out like a naive derivative), and the accel-attitude update
    // subtracts R^T·a_kin → the residual is the TRUE attitude error.
    // ESKF 運動加速度補償の accel-attitude（POS_HOLD）。加速度計は比力 f=a_kin−g を測り、水平
    // マニューバ中は運動加速度 a_kin を傾きと誤認し姿勢が「見かけの重力」角 atan(a/g) に張付き
    // POS_HOLD が飛び去る。フロー速度の α-β トラッカで a_kin を推定（状態=速度+加速度、β 小で
    // 持続ドリフト加速度を単純微分のように washout せず捕捉）、accel-attitude が R^T·a_kin を
    // 差し引き残差を真の姿勢誤差にする。
    bool  eskf_accel_comp_enable = true;  // on (adopted; SIL clean+N0 all 4 axes hold)
    float eskf_accel_comp_alpha  = 0.2f;  // α-β velocity gain
    float eskf_accel_comp_beta   = 0.02f; // α-β acceleration gain (small = capture DC drift)
    float eskf_accel_comp_max    = 5.0f;  // [m/s²] physical clamp on a_kin

    // Safety
    float safety_accel_g     = 3.0f;
    float safety_gyro_dps    = 800.0f;
    float safety_comm_timeout = 500.0f;
    float safety_low_v       = 3.4f;
    float safety_usb_v       = 3.3f;

    // Calibration — boot gyro/accel bias calibration on/off (ImuTask seeds the
    // estimator at rest before flight). Default on.
    // キャリブレーション — 起動時バイアス校正の ON/OFF（ImuTask が飛行前に静止で推定器へ
    // 種付け）。既定 ON。
    bool calibration_enable = true;
}

namespace params {

using namespace param_vars;

// -----------------------------------------------------------------------------
// Live-reload callbacks — set on the table rows below. A param set publishes a
// ReloadParams verb on the owning task's command topic; the OWNER re-reads its
// parameters in its own context (thread-safe immediate application; the
// callback itself never touches another task's objects).
// ライブ再読込コールバック — 下のテーブル行に設定。param set が所有タスクの
// コマンドトピックへ ReloadParams verb を発行し、「所有者」が自分の文脈で
// パラメータを読み直す（スレッド安全な即時反映。コールバック自身は他タスクの
// オブジェクトに決して触らない）。
//
// @design detailed_design.md §5 — parameter change → immediate apply     [OK]
// -----------------------------------------------------------------------------

static void notifyControllerReload()
{
    controller_command.publish(
        {static_cast<uint8_t>(ControllerCmd::ReloadParams), 0,
         static_cast<uint32_t>(esp_timer_get_time())});
}

static void notifyEstimatorReload()
{
    estimator_command.publish(
        {static_cast<uint8_t>(EstimatorCmd::ReloadParams),
         static_cast<uint32_t>(esp_timer_get_time()), 0});
}

/// Parameter table — the single source of truth (SSOT). Each row binds a name to
/// a param_vars variable with its default/min/max/callback. To add a parameter,
/// add a variable to param_vars (above) and a row here.
/// パラメータテーブル — 唯一の真実源 (SSOT)。各行が名前を param_vars 変数に
/// 既定/最小/最大/コールバック付きで結ぶ。追加は param_vars に変数を、ここに行を。
static const ParamEntry table[] = {
    // Rate control — PHYSICAL gains [Nm/(rad/s)] for the B^-1 mixer (actuator.cpp).
    // kp = I/τ_resp (τ_resp=0.05s); ti large = near-P inner loop. See the variable
    // declarations above for the rationale. Max 0.01 = ~25× headroom over kp.
    // レート制御 — B^-1 ミキサー用の物理ゲイン [Nm/(rad/s)]。kp = 慣性/τ_resp。
    {"rate.roll.kp",    ParamType::FLOAT, &rate_roll_kp,   1.365e-3f, 0.0f,  0.01f,  &notifyControllerReload},
    {"rate.roll.ti",    ParamType::FLOAT, &rate_roll_ti,   0.7f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.roll.td",    ParamType::FLOAT, &rate_roll_td,   0.01f,     0.0f,  1.0f,   &notifyControllerReload},
    {"rate.pitch.kp",   ParamType::FLOAT, &rate_pitch_kp,  1.995e-3f, 0.0f,  0.01f,  &notifyControllerReload},
    {"rate.pitch.ti",   ParamType::FLOAT, &rate_pitch_ti,  0.7f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.pitch.td",   ParamType::FLOAT, &rate_pitch_td,  0.01f,     0.0f,  1.0f,   &notifyControllerReload},
    {"rate.yaw.kp",     ParamType::FLOAT, &rate_yaw_kp,    5.31e-3f,  0.0f,  0.01f,  &notifyControllerReload},
    {"rate.yaw.ti",     ParamType::FLOAT, &rate_yaw_ti,    1.6f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.yaw.td",     ParamType::FLOAT, &rate_yaw_td,    0.01f,     0.0f,  1.0f,   &notifyControllerReload},
    {"autotune.sched.axis",  ParamType::INT,   &autotune_sched_axis,  -1.0f, -1.0f,  2.0f,   nullptr},
    {"autotune.sched.delay", ParamType::FLOAT, &autotune_sched_delay, 20.0f,  3.0f, 120.0f,  nullptr},
    // Autotune sysid results (written by autotune, read-back only). Wide ranges = result store.
    {"autotune.roll.b",      ParamType::FLOAT, &autotune_roll_b,      0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.roll.tau",    ParamType::FLOAT, &autotune_roll_tau,    0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.roll.delay",  ParamType::FLOAT, &autotune_roll_delay,  0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.roll.resid",  ParamType::FLOAT, &autotune_roll_resid,  0.0f,  0.0f, 1.0e6f, nullptr},
    {"autotune.pitch.b",     ParamType::FLOAT, &autotune_pitch_b,     0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.pitch.tau",   ParamType::FLOAT, &autotune_pitch_tau,   0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.pitch.delay", ParamType::FLOAT, &autotune_pitch_delay, 0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.pitch.resid", ParamType::FLOAT, &autotune_pitch_resid, 0.0f,  0.0f, 1.0e6f, nullptr},
    {"autotune.yaw.b",       ParamType::FLOAT, &autotune_yaw_b,       0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.yaw.tau",     ParamType::FLOAT, &autotune_yaw_tau,     0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.yaw.delay",   ParamType::FLOAT, &autotune_yaw_delay,   0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.yaw.resid",   ParamType::FLOAT, &autotune_yaw_resid,   0.0f,  0.0f, 1.0e6f, nullptr},
    // Autotune design margins (written by autotune, read-back only). wc[rad/s] pm[deg] gm[dB].
    {"autotune.roll.wc",     ParamType::FLOAT, &autotune_roll_wc,     0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.roll.pm",     ParamType::FLOAT, &autotune_roll_pm,     0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.roll.gm",     ParamType::FLOAT, &autotune_roll_gm,     0.0f, -200.0f, 200.0f, nullptr},
    {"autotune.pitch.wc",    ParamType::FLOAT, &autotune_pitch_wc,    0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.pitch.pm",    ParamType::FLOAT, &autotune_pitch_pm,    0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.pitch.gm",    ParamType::FLOAT, &autotune_pitch_gm,    0.0f, -200.0f, 200.0f, nullptr},
    {"autotune.yaw.wc",      ParamType::FLOAT, &autotune_yaw_wc,      0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.yaw.pm",      ParamType::FLOAT, &autotune_yaw_pm,      0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.yaw.gm",      ParamType::FLOAT, &autotune_yaw_gm,      0.0f, -200.0f, 200.0f, nullptr},

    // Estimator selection (0 = ESKF, 1 = complementary) — RESET_PLAN P2.
    {"estimator.type",  ParamType::INT,   &estimator_type, 0.0f,      0.0f,  1.0f,   nullptr},

    // Telemetry WiFi mode (0 = STA, 1 = SoftAP) — boot-time, no live reload
    // (the radio cannot be re-homed mid-flight).
    // テレメトリ WiFi モード（0=STA, 1=SoftAP）— 起動時のみ。ライブ再読込なし
    // （無線は飛行中に載せ替えられない）。
    {"wifi.mode",       ParamType::INT,   &wifi_mode,      0.0f,      0.0f,  1.0f,   nullptr},
    {"log.blackbox.enable", ParamType::INT, &log_blackbox_enable, 0.0f, 0.0f, 1.0f, nullptr},

    // Attitude control
    {"attitude.roll.kp",  ParamType::FLOAT, &att_roll_kp,  5.0f,  0.0f,  50.0f,  &notifyControllerReload},
    {"attitude.roll.ti",  ParamType::FLOAT, &att_roll_ti,  4.0f,  0.01f, 100.0f, &notifyControllerReload},
    {"attitude.roll.td",  ParamType::FLOAT, &att_roll_td,  0.04f, 0.0f,  1.0f,   &notifyControllerReload},
    {"attitude.pitch.kp", ParamType::FLOAT, &att_pitch_kp, 5.0f,  0.0f,  50.0f,  &notifyControllerReload},
    {"attitude.pitch.ti", ParamType::FLOAT, &att_pitch_ti, 4.0f,  0.01f, 100.0f, &notifyControllerReload},
    {"attitude.pitch.td", ParamType::FLOAT, &att_pitch_td, 0.04f, 0.0f,  1.0f,   &notifyControllerReload},

    // Heading hold (kp=0 disables / kp=0 で無効)
    {"attitude.yawhold.kp",       ParamType::FLOAT, &att_yawhold_kp,       3.0f, 0.0f, 10.0f, &notifyControllerReload},
    {"attitude.yawhold.rate_max", ParamType::FLOAT, &att_yawhold_rate_max, 2.0f, 0.1f, 5.0f,  &notifyControllerReload},

    // Altitude control (SIL-validated; see the variable defaults above)
    {"altitude.alt.kp",   ParamType::FLOAT, &alt_alt_kp,  0.6f,  0.0f, 10.0f,  &notifyControllerReload},
    {"altitude.alt.ti",   ParamType::FLOAT, &alt_alt_ti,  7.0f,  0.1f, 100.0f, &notifyControllerReload},
    {"altitude.vel.kp",   ParamType::FLOAT, &alt_vel_kp,  0.1f,  0.0f, 10.0f,  &notifyControllerReload},
    {"altitude.vel.ti",   ParamType::FLOAT, &alt_vel_ti,  2.5f,  0.1f, 100.0f, &notifyControllerReload},
    {"altitude.climb_rate",   ParamType::FLOAT, &alt_climb_rate,   0.5f, 0.05f, 2.0f, &notifyControllerReload},
    {"altitude.descent_rate", ParamType::FLOAT, &alt_descent_rate, 0.5f, 0.05f, 2.0f, &notifyControllerReload},
    {"hover.thrust_corr",     ParamType::FLOAT, &hover_thrust_corr, 1.12f, 0.5f, 2.0f, &notifyControllerReload},

    // Position control
    {"position.pos.kp",   ParamType::FLOAT, &pos_pos_kp,  1.0f,  0.0f, 10.0f,  &notifyControllerReload},
    {"position.pos.ti",   ParamType::FLOAT, &pos_pos_ti,  5.0f,  0.1f, 100.0f, &notifyControllerReload},
    {"position.vel.kp",   ParamType::FLOAT, &pos_vel_kp,  0.8f,  0.0f, 10.0f,  &notifyControllerReload},
    {"position.vel.ti",   ParamType::FLOAT, &pos_vel_ti,  2.0f,  0.1f, 100.0f, &notifyControllerReload},

    // ESKF process noise
    {"eskf.process.gyro_noise",  ParamType::FLOAT, &eskf_gyro_noise,  0.009655f, 0.001f, 1.0f,  &notifyEstimatorReload},
    {"eskf.process.accel_noise", ParamType::FLOAT, &eskf_accel_noise, 0.3f,      0.01f,  10.0f, &notifyEstimatorReload},
    {"eskf.process.gyro_bias",   ParamType::FLOAT, &eskf_gyro_bias,   0.000013f, 1e-7f,  0.01f, &notifyEstimatorReload},
    {"eskf.process.accel_bias",  ParamType::FLOAT, &eskf_accel_bias,  0.0001f,   1e-7f,  0.01f, &notifyEstimatorReload},
    // Min lowered to 0 so eskf.bias.gyro_dev_max=0 FREEZES the gyro bias at the boot
    // still-calibration nominal (no in-flight ESKF update reaches the rate loop) — a
    // diagnostic toggle. Restore 0.03 for normal random-walk tracking.
    // 最小を0に下げ、=0 で起動静止校正値にジャイロバイアスを凍結（飛行中ESKF更新を
    // レートループへ反映しない）。診断用トグル。通常は0.03へ戻す。
    {"eskf.bias.gyro_dev_max",   ParamType::FLOAT, &eskf_bg_dev_max,  0.03f,     0.0f, 1.0f,  &notifyEstimatorReload},

    // ESKF observation noise
    {"eskf.obs.tof_noise",       ParamType::FLOAT, &eskf_tof_noise,     0.01f, 0.001f, 1.0f,  &notifyEstimatorReload},
    {"eskf.obs.flow_noise",      ParamType::FLOAT, &eskf_flow_noise,    0.30f, 0.01f,  5.0f,  &notifyEstimatorReload},
    {"eskf.obs.baro_noise",      ParamType::FLOAT, &eskf_baro_noise,    0.1f,  0.01f,  5.0f,  &notifyEstimatorReload},
    {"eskf.obs.mag_noise",       ParamType::FLOAT, &eskf_mag_noise,     1.0f,  0.01f,  10.0f, &notifyEstimatorReload},
    {"eskf.obs.accel_att_noise", ParamType::FLOAT, &eskf_accel_att,     1.2f,  0.001f, 2.0f,  &notifyEstimatorReload},
    {"eskf.obs.accel_att_lpf",   ParamType::FLOAT, &eskf_accel_att_lpf, 30.0f, 0.0f,   200.0f,&notifyEstimatorReload},

    // ESKF sensor enable
    {"eskf.use_tof",  ParamType::BOOL, &eskf_use_tof,  1.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_flow", ParamType::BOOL, &eskf_use_flow, 1.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_baro", ParamType::BOOL, &eskf_use_baro, 0.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_mag",  ParamType::BOOL, &eskf_use_mag,  0.0f, 0.0f, 1.0f, &notifyEstimatorReload},

    // ESKF gates
    {"eskf.gate.mahalanobis", ParamType::FLOAT, &eskf_mahalanobis, 15.0f, 1.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.gate.tof_innov",   ParamType::FLOAT, &eskf_tof_innov,   0.5f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.baro_innov",  ParamType::FLOAT, &eskf_baro_innov,  0.5f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.flow_clamp",  ParamType::FLOAT, &eskf_flow_clamp,  0.3f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.flow_squal",  ParamType::INT,   &eskf_flow_squal,  10.0f, 0.0f,  255.0f, &notifyEstimatorReload},

    // ESKF accel-attitude (SSOT — proven firmware/vehicle values)
    {"eskf.att.k_adaptive",   ParamType::FLOAT, &eskf_att_k_adaptive, 10.0f, 0.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.att.chi2_gate",    ParamType::FLOAT, &eskf_att_chi2_gate,  7.81f, 0.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.att.corr_clamp",   ParamType::FLOAT, &eskf_att_corr_clamp, 0.05f, 0.001f, 1.0f,  &notifyEstimatorReload},

    // ESKF acceleration-compensated accel-attitude (POS_HOLD; α-β flow-acceleration tracker)
    {"eskf.accel_comp.enable", ParamType::BOOL,  &eskf_accel_comp_enable, 1.0f,  0.0f,  1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.alpha",  ParamType::FLOAT, &eskf_accel_comp_alpha,  0.2f,  0.01f, 1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.beta",   ParamType::FLOAT, &eskf_accel_comp_beta,   0.02f, 0.0f,  1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.max",    ParamType::FLOAT, &eskf_accel_comp_max,    5.0f,  0.5f,  20.0f, &notifyEstimatorReload},

    // Safety
    {"safety.impact.accel_g",  ParamType::FLOAT, &safety_accel_g,     3.0f,   1.0f,   10.0f,   nullptr},
    {"safety.impact.gyro_dps", ParamType::FLOAT, &safety_gyro_dps,    800.0f, 100.0f, 2000.0f, nullptr},
    {"safety.comm.timeout_ms", ParamType::FLOAT, &safety_comm_timeout, 500.0f, 100.0f, 5000.0f, nullptr},
    {"safety.battery.low_v",   ParamType::FLOAT, &safety_low_v,       3.4f,   3.0f,   4.2f,    nullptr},
    {"safety.battery.usb_v",   ParamType::FLOAT, &safety_usb_v,       3.3f,   2.5f,   3.5f,    nullptr},

    // Calibration — boot gyro/accel bias calibration on/off
    {"calibration.enable",     ParamType::BOOL,  &calibration_enable, 1.0f,   0.0f,   1.0f,    nullptr},
};

static constexpr int TABLE_SIZE = sizeof(table) / sizeof(table[0]);
static const char* NVS_NAMESPACE = "sf_params";

// =============================================================================
// NVS key derivation
// NVS キー導出
//
// NVS limits key names to 15 characters; 31 of the 54 parameter names exceed
// that (e.g. "eskf.process.accel_noise" = 24), so storing under the raw name
// fails with ESP_ERR_NVS_KEY_TOO_LONG. We derive a fixed-length 9-character
// key "p" + 8-hex FNV-1a hash of the name. Hash keys are stable across table
// reordering (unlike index-based keys, which would silently load the wrong
// value after an insertion). init() verifies there are no hash collisions.
// NVS のキー名は15文字まで。54個中31個のパラメータ名がこれを超え（例:
// "eskf.process.accel_noise" は24文字）、生の名前では ESP_ERR_NVS_KEY_TOO_LONG で
// 保存に失敗する。そこで名前の FNV-1a ハッシュから固定長9文字のキー
// "p"+16進8桁 を導出する。ハッシュキーはテーブルの並べ替えに不変（インデックス
// ベースだと行挿入後に黙って別の値を読んでしまう）。init() で衝突がないことを検証。
// =============================================================================

static uint32_t fnv1aHash(const char* s)
{
    uint32_t hash = 2166136261u;            // FNV offset basis
    while (*s) {
        hash ^= static_cast<uint8_t>(*s++);
        hash *= 16777619u;                  // FNV prime
    }
    return hash;
}

static void nvsKeyFor(const char* name, char out[16])
{
    snprintf(out, 16, "p%08lx",
             static_cast<unsigned long>(fnv1aHash(name)));
}

// =============================================================================
// Find parameter by name
// 名前でパラメータを検索する
// =============================================================================

static const ParamEntry* find(const char* name)
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (strcmp(table[i].name, name) == 0) {
            return &table[i];
        }
    }
    return nullptr;
}

// =============================================================================
// Public API Implementation
// 公開API実装
// =============================================================================

void init()
{
    // One-time NVS-key collision check: two names hashing to the same key would
    // silently alias their stored values. With 54 names on a 32-bit hash the
    // probability is ~3e-7, but verify anyway — this is the kind of failure that
    // is invisible until a parameter "mysteriously" loads someone else's value.
    // NVS キー衝突の一回限り検査: 2つの名前が同じキーにハッシュされると保存値が
    // 黙って混線する。54名×32bitハッシュで確率は ~3e-7 だが念のため検証 — これは
    // パラメータが「謎に」他人の値を読むまで見えない種類の故障。
    for (int i = 0; i < TABLE_SIZE; i++) {
        for (int j = i + 1; j < TABLE_SIZE; j++) {
            if (fnv1aHash(table[i].name) == fnv1aHash(table[j].name)) {
                ESP_LOGE(TAG, "NVS key collision: '%s' vs '%s' — rename one!",
                         table[i].name, table[j].name);
            }
        }
    }

    load();
    ESP_LOGI(TAG, "Parameter system initialized (%d params)", TABLE_SIZE);
}

bool get_float(const char* name, float& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) return false;
    out = *static_cast<float*>(e->value_ptr);
    return true;
}

bool get_bool(const char* name, bool& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;
    out = *static_cast<bool*>(e->value_ptr);
    return true;
}

bool get_int(const char* name, int32_t& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;
    out = *static_cast<int32_t*>(e->value_ptr);
    return true;
}

bool set_float(const char* name, float value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) {
        ESP_LOGW(TAG, "set_float: '%s' not found", name);
        return false;
    }

    // Validate range
    // 範囲を検証
    if (value < e->min_val || value > e->max_val) {
        ESP_LOGW(TAG, "set_float: '%s' = %f out of range [%f, %f]",
                 name, value, e->min_val, e->max_val);
        return false;
    }

    *static_cast<float*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %f", name, value);

    // Call callback if registered
    // コールバックが登録されていれば呼ぶ
    if (e->callback) {
        e->callback();
    }

    return true;
}

bool set_bool(const char* name, bool value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;

    *static_cast<bool*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %s", name, value ? "true" : "false");

    if (e->callback) {
        e->callback();
    }
    return true;
}

bool set_int(const char* name, int32_t value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;

    if (value < static_cast<int32_t>(e->min_val) ||
        value > static_cast<int32_t>(e->max_val)) {
        ESP_LOGW(TAG, "set_int: '%s' = %ld out of range", name, (long)value);
        return false;
    }

    *static_cast<int32_t*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %ld", name, (long)value);

    if (e->callback) {
        e->callback();
    }
    return true;
}

void save()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    int saved = 0;
    int failed = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        char key[16];
        nvsKeyFor(e.name, key);
        esp_err_t set_err = ESP_OK;

        if (e.type == ParamType::FLOAT) {
            float val = *static_cast<float*>(e.value_ptr);
            // Store float as uint32_t bit pattern
            // floatをuint32_tビットパターンとして保存
            uint32_t raw;
            memcpy(&raw, &val, sizeof(float));
            set_err = nvs_set_u32(handle, key, raw);
        } else if (e.type == ParamType::BOOL) {
            bool val = *static_cast<bool*>(e.value_ptr);
            set_err = nvs_set_u8(handle, key, val ? 1 : 0);
        } else if (e.type == ParamType::INT) {
            set_err = nvs_set_i32(handle, key,
                                  *static_cast<int32_t*>(e.value_ptr));
        }

        // Count failures instead of silently claiming success — the old code
        // ignored every nvs_set_* error and logged a bogus "Saved N parameters".
        // 失敗を数える（黙って成功を装わない）— 旧実装は nvs_set_* のエラーを全て
        // 無視し、偽の「Saved N parameters」を出していた。
        if (set_err == ESP_OK) {
            saved++;
        } else {
            failed++;
            ESP_LOGE(TAG, "save: '%s' (key %s) failed: %s",
                     e.name, key, esp_err_to_name(set_err));
        }
    }

    esp_err_t commit_err = nvs_commit(handle);
    nvs_close(handle);
    if (commit_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(commit_err));
    } else if (failed > 0) {
        ESP_LOGW(TAG, "Saved %d parameters to NVS (%d FAILED)", saved, failed);
    } else {
        ESP_LOGI(TAG, "Saved %d parameters to NVS", saved);
    }
}

void load()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // NVS not initialized or no data — use defaults
        // NVS未初期化またはデータなし — デフォルトを使用
        ESP_LOGI(TAG, "No saved parameters, using defaults");
        return;
    }

    int loaded = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        char key[16];
        nvsKeyFor(e.name, key);

        if (e.type == ParamType::FLOAT) {
            uint32_t raw;
            if (nvs_get_u32(handle, key, &raw) == ESP_OK) {
                float val;
                memcpy(&val, &raw, sizeof(float));
                // Validate range before applying
                // 適用前に範囲を検証
                if (val >= e.min_val && val <= e.max_val && !std::isnan(val)) {
                    *static_cast<float*>(e.value_ptr) = val;
                    loaded++;
                }
            }
        } else if (e.type == ParamType::BOOL) {
            uint8_t raw;
            if (nvs_get_u8(handle, key, &raw) == ESP_OK) {
                *static_cast<bool*>(e.value_ptr) = (raw != 0);
                loaded++;
            }
        } else if (e.type == ParamType::INT) {
            int32_t raw;
            if (nvs_get_i32(handle, key, &raw) == ESP_OK) {
                if (raw >= static_cast<int32_t>(e.min_val) &&
                    raw <= static_cast<int32_t>(e.max_val)) {
                    *static_cast<int32_t*>(e.value_ptr) = raw;
                    loaded++;
                }
            }
        }
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded %d parameters from NVS", loaded);
}

void reset_all()
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            *static_cast<float*>(e.value_ptr) = e.default_val;
        } else if (e.type == ParamType::BOOL) {
            *static_cast<bool*>(e.value_ptr) = (e.default_val != 0.0f);
        } else if (e.type == ParamType::INT) {
            *static_cast<int32_t*>(e.value_ptr) = static_cast<int32_t>(e.default_val);
        }
    }

    // Fire each DISTINCT change callback once so the owning tasks re-read the
    // restored defaults live (same path as `param set`). Firing per-row would
    // flood the small command queues with dozens of identical ReloadParams verbs.
    // 「異なる」変更コールバックを1回ずつ発火し、所有タスクに復元後の既定値を
    // ライブで読み直させる（`param set` と同じ経路）。行ごとに発火すると小さな
    // コマンドキューが同一の ReloadParams で溢れる。
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (table[i].callback == nullptr) continue;
        bool seen = false;
        for (int j = 0; j < i; j++) {
            if (table[j].callback == table[i].callback) { seen = true; break; }
        }
        if (!seen) table[i].callback();
    }

    ESP_LOGI(TAG, "All %d parameters reset to defaults", TABLE_SIZE);
}

void list()
{
    ESP_LOGI(TAG, "=== Parameters (%d) ===", TABLE_SIZE);
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            ESP_LOGI(TAG, "  %-30s = %f  [%f, %f]",
                     e.name, *static_cast<float*>(e.value_ptr),
                     e.min_val, e.max_val);
        } else if (e.type == ParamType::BOOL) {
            ESP_LOGI(TAG, "  %-30s = %s",
                     e.name, *static_cast<bool*>(e.value_ptr) ? "true" : "false");
        } else if (e.type == ParamType::INT) {
            ESP_LOGI(TAG, "  %-30s = %ld  [%ld, %ld]",
                     e.name, (long)*static_cast<int32_t*>(e.value_ptr),
                     (long)e.min_val, (long)e.max_val);
        }
    }
}

int count()
{
    return TABLE_SIZE;
}

const ParamEntry* entry(int index)
{
    if (index < 0 || index >= TABLE_SIZE) return nullptr;
    return &table[index];
}

}  // namespace params
}  // namespace sf
