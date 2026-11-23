/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file flight_limits.hpp
 * @brief Safety limits shared by the state machine, controller and API (named constants)
 *        状態機械・制御器・API が共有する安全上限（名前付き定数）
 *
 * Components cannot include main/config.hpp (sf_state and sf_controller_pid are core
 * components), so the limits that several of them must agree on live here in sf_core,
 * the one component they all already depend on. One definition, no copies.
 *
 * sf_state・sf_controller_pid などのコア部品は main/config.hpp を include できない。
 * 複数の部品が一致して守るべき上限は、全部品が既に依存している sf_core のここに置く。
 * 定義は 1 か所で、複製しない。
 *
 * Every value below is a CANDIDATE (not yet validated on a real flight); the basis
 * is written next to each one.
 * 以下の値はすべて候補値（実機飛行での検証はまだ）。根拠は各値の横に書く。
 *
 * @design architecture.md §4 — ToF silence handling (state machine, R16)   [OK]
 * @design detailed_design.md §3.1 — ToF-silence rows of the normative table [OK]
 * @design coding_and_education.md §2 — no magic numbers                    [OK]
 */

#pragma once

#include <cstdint>

#include "sf_math.hpp"   // sf::math::kGravity (the one gravity constant) / 重力加速度の唯一の定義
#include "thrust_excitation_table.hpp"   // generated waveform tables of the thrust excitation / 推力励振の波形表（生成物）

namespace sf {
namespace limits {

/// Altitude ceiling for the auto-takeoff target, for API altitude inputs, for the ALT_HOLD /
/// POS_HOLD stick climb and the rc velocity guidance in the controller (altitudeHoldClimbRate()) [m].
/// CANDIDATE. Basis: an auto-takeoff that ran into the ceiling of the room
/// and the ToF's valid range of ~1.6 m (flight logs: 95-99 % valid up to 1.7 m, falling
/// above). 1.2 m keeps the craft inside the range where ToF altitude can be trusted and
/// bounds the time and the speed of the descent without a ToF: its thrust is at most kNoTofThrustCapRatio x the support thrust, so the craft sinks
/// with a constant downward acceleration of about 0.05 g (not at a regulated speed), reaching the floor from the ceiling in about
/// sqrt(2 x 1.2 / 0.49) = 2.2 s at about 1.1 m/s (SILS: touchdown 2.3-2.9 s after the silence, 0.8-1.2 m/s), far inside kBlindLandingTimeoutUs (10 s).
/// 自動離陸の目標、API の高度入力、制御器の ALT_HOLD・POS_HOLD のスティック上昇と rc 速度誘導
/// （altitudeHoldClimbRate()）の上限 [m]。候補値。根拠: 自動離陸で
/// 部屋の天井に衝突した例と ToF の有効距離 約 1.6 m（飛行ログで 1.7 m まで 95-99 % が有効、
/// それ以上で低下）。1.2 m なら ToF 高度が信頼できる範囲に収まり、ToF なしの降下の時間と速さも
/// 抑えられる: その推力は高々 kNoTofThrustCapRatio × 支持推力なので、機体は速度を調整されずに、約 0.05 g の一定の下向き加速度で沈み、
/// 上限の高さから床まで約 sqrt(2 × 1.2 / 0.49) = 2.2 s、約 1.1 m/s で着く（SILS: 出力停止の 2.3〜2.9 s 後に接地、0.8〜1.2 m/s）。
/// kBlindLandingTimeoutUs（10 s）よりずっと短い。
inline constexpr float kAltitudeCeilingM = 1.2f;
/// Yaw turn rate limit [rad/s] and P gain [1/s] of the guidance yaw law (PidController: the guidance
/// position mode seeks the target yaw with rate = gain x wrapped error, clamped to the limit). One definition for the controller, the
/// API's `rc` yaw rate (full stick), and the `cw` / `ccw` guidance and wait (rotate_guidance.hpp). Neither is a parameter, a
/// setter or an autotune output: they change only by editing these constants, and rotate_guidance.hpp checks its lead angles against them at compile time.
/// 誘導ヨー則のヨー回頭率の上限 [rad/s] と P ゲイン [1/s]（PidController: 誘導位置モードは 率 = ゲイン × 折り返した誤差 で
/// 目標ヨーを追い、上限でクランプする）。制御器・API の `rc` のヨーレート（満舵）・`cw` / `ccw` の誘導と待ち時間
/// （rotate_guidance.hpp）の定義は 1 か所。どちらもパラメータ・setter・オートチューンの出力ではなく、この定数を書き換えたときだけ変わる。
/// rotate_guidance.hpp は先行角をこれらに対してコンパイル時に検査する。
inline constexpr float kGuidanceYawRateMaxRadPerSec = 1.0f;
inline constexpr float kGuidanceYawKpPerSec         = 2.0f;
/// Target altitude of the auto-takeoff [m]: the height every takeoff ends at and then holds, whether the ARM of the transmitter started it
/// (PidController: the climb target) or the API `takeoff` (ApiTask: the altitude of the guidance target it seeds). High enough to clear the
/// ground effect (the ToF airborne edge of 0.15 m is the ESKF vertical handoff, not this target). Must not exceed kAltitudeCeilingM.
/// 自動離陸の目標高度 [m]: 送信機の ARM が始めた離陸（PidController: 上昇目標）も API の `takeoff`（ApiTask: 初期化する誘導目標の高度）も、
/// 離陸はこの高さで終わってそのまま保持する。地面効果を抜けるのに十分な高さ（ToF の空中判定 0.15 m は ESKF の鉛直ハンドオフ用で、この目標ではない）。
/// kAltitudeCeilingM を超えてはならない。
inline constexpr float kAutoTakeoffTargetAltitudeM = 0.5f;
static_assert(kAutoTakeoffTargetAltitudeM <= kAltitudeCeilingM,
              "auto-takeoff target must not exceed the altitude ceiling");
/// Margin above kAltitudeCeilingM [m] beyond which the controller forbids any climb command and brings the target back
/// to the ceiling (auto-takeoff, ALT_HOLD, POS_HOLD). CANDIDATE. Basis: the ESKF altitude error with a working ToF is a
/// few cm (innovation gate 0.5 m, ToF noise 0.02 m), so 0.15 m is the same as the ground/air threshold of the
/// takeoff/landing manager: beyond it the altitude is not a measurement error but a real overshoot.
/// kAltitudeCeilingM を超える余裕 [m]。これを超えたら制御器は上昇の指令を一切禁止し、目標を上限へ戻す（自動離陸・
/// ALT_HOLD・POS_HOLD）。候補値。根拠: ToF が動作しているときの ESKF の高度誤差は数 cm（イノベーション判定 0.5 m、
/// ToF ノイズ 0.02 m）なので、離着陸マネージャの接地/空中のしきい値と同じ 0.15 m を超えたら、測定誤差でなく本当の行き過ぎ。
inline constexpr float kAltitudeCeilingMarginM = 0.15f;
/// Auto-takeoff velocity loop (PidController::takeoffClimbRate): the part of the climb that the accelerometer integral shows and the ESKF does not, beyond this [m/s],
/// is added to the ESKF climb rate. CANDIDATE. Basis: SILS, an ordinary takeoff (corr 1.12, noise off): the integral exceeds the pinned / lagging ESKF by up to 0.5 m/s in the
/// first second, so 0.6 keeps it on the ESKF signal (alt_recenter_gate and landing_dead_zone, whose margins are small, are unchanged), while the failing takeoff
/// (`hover.thrust_corr` 1.2, 1.2-1.8 m/s) is seen.
/// 自動離陸の速度ループ（PidController::takeoffClimbRate）: 加速度積分が示し ESKF が示さない上昇のうち、この値 [m/s] を超えた分を ESKF の上昇率に足す。候補値。
/// 根拠: SILS の普通の離陸（corr 1.12、ノイズなし）では、固定された・遅れた ESKF を積分が最初の 1 s に最大 0.5 m/s 超えるので、0.6 なら ESKF の信号のまま
/// （余裕の小さい alt_recenter_gate と landing_dead_zone は変わらない）。失敗する離陸（`hover.thrust_corr` 1.2、1.2〜1.8 m/s）は見える。
inline constexpr float kTakeoffVelocityDisagreeMps = 0.6f;   // [m/s]
/// The full brake: while the craft is still rising and would stop more than kAltitudeCeilingBrakeRampM above kAltitudeCeilingM + kAltitudeCeilingBrakeStartM even if it
/// braked now, the thrust is at most this fraction of the SUPPORT thrust [-] (the thrust that holds the craft: blind_landing.hpp; a brake that does not wait for the velocity
/// loop, whose correction is limited, and does not trust the hover-thrust estimate); with less predicted excess the fraction is between 1 and this value (kAltitudeCeilingBrakeRampM). CANDIDATE.
/// Basis: SILS tof_silent_est_high with noise n2 (the estimate is 1.35 x the true hover): the auto-takeoff reached 1.5 m/s and the velocity loop's thrust bottomed at 0.89 x the weight, so the craft went
/// on to 1.69 m (ceiling 1.2 m, ToF range 1.6 m); 0.5 x the support thrust is half the weight, a deceleration of 4.9 m/s^2.
/// 完全なブレーキ: まだ上昇中で、今ブレーキをかけても kAltitudeCeilingM + kAltitudeCeilingBrakeStartM より kAltitudeCeilingBrakeRampM を超えて上で止まる見込みの間は、推力は
/// 「支持推力」（機体を支える推力: blind_landing.hpp）のこの割合 [-] 以下（補正が制限されている速度ループを待たず、ホバー推力の見積もりを信用しないブレーキ）。予測の超過がそれより小さい間は、
/// 割合は 1 とこの値の間（kAltitudeCeilingBrakeRampM）。候補値。根拠: SILS の tof_silent_est_high のノイズ n2（見積もりが真のホバーの 1.35 倍）では、自動離陸が
/// 1.5 m/s に達し、速度ループの推力は重さの 0.89 倍で底を打ち、機体は 1.69 m まで行った（上限 1.2 m、ToF の有効距離 1.6 m）。
/// 支持推力の 0.5 倍は重さの半分で、減速度は 4.9 m/s^2。
inline constexpr float kAltitudeCeilingBrakeFrac = 0.5f;
/// The predicted stop height above kAltitudeCeilingM [m] at which the brake starts. CANDIDATE. Basis: SILS (`hover.thrust_corr` 1.2, learning off):
/// with the brake at the hard-side margin (kAltitudeCeilingMarginM, 0.15 m) the true peak of a 0.5 m takeoff was 1.35-1.40 m; the hard side
/// (limitClimbAboveCeiling) stays at the margin. A stick climb or a hover whose predicted stop height stays below 1.25 m is not touched; above it the brake acts in proportion
/// (kAltitudeCeilingBrakeRampM), so a hover at 1.24-1.26 m in noise sees a gentle reduction of the thrust, not a switch.
/// ブレーキを始める、止まる高さの kAltitudeCeilingM に対する余裕 [m]。候補値。根拠: SILS（`hover.thrust_corr` 1.2、学習なし）で、ブレーキを
/// 厳しい側の余裕（kAltitudeCeilingMarginM、0.15 m）で始めると、0.5 m 離陸の真値のピークは 1.35〜1.40 m だった。厳しい側
/// （limitClimbAboveCeiling）は余裕のまま。止まる高さの予測が 1.25 m 未満にとどまるスティック上昇やホバーには触れない。それより上では、ブレーキは比例して働く
/// （kAltitudeCeilingBrakeRampM）ので、雑音の中の 1.24〜1.26 m のホバーは、切り替えでなく穏やかな推力の低下を受ける。
inline constexpr float kAltitudeCeilingBrakeStartM = 0.05f;
/// Derating of the deceleration that the brake is assumed to give when the stop height is predicted [-]. CANDIDATE. Basis: the thrust follows the command
/// with a delay of about 80 ms (kSupportThrustLagS) and the attitude loop takes a part of the thrust; 0.7 x 4.9 = 3.4 m/s^2. Final build, SILS (the auto-takeoff and the API takeoff, `hover.thrust_corr` 0.9 / 1.0 / 1.2 x noise n0 / n2 x 3 seeds,
/// 36 runs per value): the true peak height is 1.279 m at 0.5, 1.282 m at 0.7 and 1.291 m at 0.9 (the hard limit is the ceiling + margin, 1.35 m): the result hardly depends on it, so the physical estimate is kept.
/// 止まる高さを予測するときに、ブレーキが出すと仮定する減速度に掛ける余裕 [-]。候補値。根拠: 推力は指令に約 80 ms 遅れ（kSupportThrustLagS）、
/// 姿勢ループが推力の一部を使う。0.7 × 4.9 = 3.4 m/s^2。SILS（自動離陸と API の離陸、`hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ n0 / n2 × 3 seed、
/// 1 値あたり 36 本）: 真値のピークの高さは 0.5 で 1.279 m、0.7 で 1.282 m、0.9 で 1.291 m（硬い側の限界は上限 + 余裕の 1.35 m）。結果はほとんどこれに依らないので、物理的な見積もりのまま。
inline constexpr float kAltitudeCeilingBrakeDecelDerate = 0.7f;
/// Predicted excess of the stop height [m] over which the brake limit falls from 1 x the support thrust to kAltitudeCeilingBrakeFrac x it (linear). CANDIDATE. Basis: SILS, the 1.2 m
/// hover after an API takeoff and `up 200` (`hover.thrust_corr` 0.9 / 1.0 / 1.2 x noise n0 / n2 x 2 seeds, 12 runs per value): the number of times
/// the brake went to more than half of its depth per run, with the hard brake (ramp 0): 136 / 134 (0.9, n0), 18 / 18 (1.0, n0), 9 / 12 (0.9, n2), 38 / 38 (1.2, n2: the takeoff overshoot); ramp 0.03 m: 0 / 0, 0 / 0, 0 / 0, 9 / 9; ramp 0.06 m: 0 / 0,
/// 0 / 0, 0 / 0, 3 / 10; ramp 0.1 m (chosen): 0 / 0, 0 / 0, 0 / 0, 3 / 3. The worst true peak of the same 12 runs: 1.288 / 1.279 / 1.290 / 1.296 m (the hard limit is the ceiling + margin, 1.35 m), so
/// the ramp costs at most 0.01 m of overshoot.
/// 止まる高さの予測の超過 [m] のうち、ブレーキの上限が 支持推力の 1 倍から kAltitudeCeilingBrakeFrac 倍まで（直線的に）下がる幅。候補値。根拠: SILS の、API 離陸と `up 200` の後の
/// 1.2 m のホバー（`hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ n0 / n2 × 2 seed、1 値あたり 12 本）で、ブレーキが深さの半分より深くなった回数（1 回の実行あたり）:
/// 硬いブレーキ（幅 0）: 136 / 134（0.9、n0）、18 / 18（1.0、n0）、9 / 12（0.9、n2）、38 / 38（1.2、n2: 離陸の行き過ぎ）。幅 0.03 m: 0 / 0、0 / 0、0 / 0、9 / 9。幅 0.06 m: 0 / 0、0 / 0、0 / 0、
/// 3 / 10。幅 0.1 m（採用）: 0 / 0、0 / 0、0 / 0、3 / 3。同じ 12 本の真値のピークの最悪: 1.288 / 1.279 / 1.290 / 1.296 m（硬い側の限界は上限 + 余裕の 1.35 m）。つまり幅の代償は行き過ぎ最大 0.01 m。
inline constexpr float kAltitudeCeilingBrakeRampM = 0.1f;   // [m]

/// Upper bound of the base thrust of the closed-loop landing (ToF alive), as a multiple of the calibrated support thrust [-] (PidController::landingBaseThrust). CANDIDATE. Basis: SILS, the 1.2 m
/// hover and the API `land` (`hover.thrust_corr` 1.1 / 1.2 x noise n2 x 3 seeds): the craft climbed after the LANDING began by 0.17-0.24 / 0.47-0.57 m with no bound
/// (the estimate is 1.17 / 1.24 x the support thrust); with the bound 1.05 (= 1 / kNoTofThrustCapRatio) 0-0.03 / 0.00-0.01 m, with 1.08 (chosen) 0.03-0.09 / 0.00-0.04 m, with 1.10 0.08-0.11 / 0.05-0.11 m,
/// with 1.15 0.17-0.26 / 0.17-0.29 m. An ordinary landing must not change: in landing_dead_zone the estimate is 1.057 x the support thrust, and a bound of 1.05 (0.4 % less thrust) turned its soft contact
/// (the dead-zone path, under 3 G) into a 3.2 G impact (contact at 0.27 m/s instead of about 0.2 m/s), so 1.08 is the smallest value among those tried that leaves it untouched.
/// ToF 生存中の閉ループ着陸の基準推力の上限（較正した支持推力の倍数 [-]。PidController::landingBaseThrust）。候補値。根拠: SILS の、1.2 m のホバーと API の `land`
/// （`hover.thrust_corr` 1.1 / 1.2 × ノイズ n2 × 3 seed）で、上限なしでは LANDING が始まった後に機体が 0.17〜0.24 / 0.47〜0.57 m 上昇した（見積もりは支持推力の 1.17 / 1.24 倍）。
/// 上限 1.05（= 1 / kNoTofThrustCapRatio）なら 0〜0.03 / 0.00〜0.01 m、1.08（採用）なら 0.03〜0.09 / 0.00〜0.04 m、1.10 なら 0.08〜0.11 / 0.05〜0.11 m、1.15 なら 0.17〜0.26 / 0.17〜0.29 m。
/// 普通の着陸は変わってはならない: landing_dead_zone の見積もりは支持推力の 1.057 倍で、上限 1.05（推力 0.4 % 減）にすると、その柔らかい接触（不感帯の経路、3 G 未満）が 3.2 G の衝撃
/// （接触が約 0.2 m/s でなく 0.27 m/s）に変わったので、1.08 は試した値のうちそれに触れない最小の値。
inline constexpr float kLandingBaseOverSupportMax = 1.08f;   // [-]
/// Rise brake of the closed-loop landing (PidController::brakeRiseInLanding): at a climb rate of 0 or less the thrust passes unchanged; while the craft rises in the LANDING (climb rate above 0), the thrust is limited to a fraction of
/// the support reference that is 1 just above a climb rate of 0 and falls linearly to kLandingRiseBrakeFrac over kLandingRiseBrakeRampMps. So the limit steps in at a climb rate of 0 for a thrust above the support reference
/// (it is cut to 1 x the support reference), and is continuous above 0. CANDIDATES, one value each measured. Basis: the fraction at the end of the ramp is the one of the altitude ceiling's full brake (kAltitudeCeilingBrakeFrac),
/// whose ramp form removed the on-off switching of a hard brake without it (20-136 switches per run); the ramp width 0.2 m/s is a rough estimate not tied to a measurement of the climb-rate noise (a hover or a slow
/// descent sees a limit of 0.9-1.0 x the support reference for a climb rate of up to 0.04 m/s). SILS (a `land` 6 s after an API takeoff, `hover.thrust_corr` 0.9 / 1.0 / 1.2, accelerometer offset -0.6..+0.6 m/s^2,
/// noise off / n2, 120 runs): re-rise above 0.10 m in 22 runs without the brake and without the bound of landingBaseThrust() before the calibration, 0 runs with them; worst descent speed 2.75 -> 0.63 m/s; the
/// ordinary landing (`landing_dead_zone`) is unchanged. Other values of the ramp and of the fraction were NOT swept.
/// 閉ループ着陸の上昇ブレーキ（PidController::brakeRiseInLanding）: 上昇率 0 以下では推力をそのまま通し、LANDING 中に機体が上昇している間（上昇率が 0 より大きい）は、推力を支持推力の基準の割合以下に制限する。
/// 割合は上昇率 0 の直上で 1、kLandingRiseBrakeRampMps で直線的に kLandingRiseBrakeFrac まで下がる。したがって、支持推力の基準より高い推力は、上昇率 0 で段差を持って
/// 上限（支持推力の基準の 1 倍）まで切られ、0 より上では連続。候補値で、それぞれ 1 値だけ測った。根拠: ランプの終わりの割合は高度上限の全ブレーキ（kAltitudeCeilingBrakeFrac）と同じで、そのランプの形は、ランプの無い硬いブレーキ
/// （1 回の実行で 20〜136 回の入り切り）の切り替えをなくした。ランプの幅 0.2 m/s は、上昇率の雑音の測定には結び付けていない大まかな見当（上昇率 0.04 m/s までなら、ホバーや遅い降下が受ける制限は支持推力の基準の 0.9〜1.0 倍）。
/// SILS（API 離陸の 6 s 後の `land`、`hover.thrust_corr` 0.9 / 1.0 / 1.2、加速度計のオフセット −0.6〜+0.6 m/s^2、ノイズなし / n2、120 本）: 再上昇 0.10 m 超は、このブレーキも較正前の
/// landingBaseThrust() の上限も無いと 22 本、有ると 0 本。最大の降下速度は 2.75 → 0.63 m/s。普通の着陸（`landing_dead_zone`）は変化なし。ランプと割合の他の値は未測定。
inline constexpr float kLandingRiseBrakeFrac     = kAltitudeCeilingBrakeFrac;   // [-]
inline constexpr float kLandingRiseBrakeRampMps  = 0.2f;                        // [m/s]

/// Downward-ToF silence threshold [us]: no NEW sample for this long = sensor stopped.
/// CANDIDATE. Basis: the ToF runs at ~30 Hz and the largest interval between samples
/// measured in flight logs is 67-99 ms, so 0.3 s is three times or more that, while still
/// short enough to react within ~0.3 m of climb at 0.3 m/s. The comparison is on the
/// SIGNED time difference (sensor_freshness.hpp), so a timestamp slightly newer than
/// "now" counts as fresh.
/// 下向き ToF の出力停止しきい値 [us]: この時間、新しいサンプルが来なければセンサ停止。候補値。
/// 根拠: ToF は約 30 Hz で、飛行ログで測ったサンプル間隔の最大が 67〜99 ms なので 0.3 s は
/// その 3 倍以上。一方、0.3 m/s の上昇で約 0.3 m 以内に反応できる短さ。比較は符号付きの
/// 時刻差（sensor_freshness.hpp）で行い、「現在」より少し新しい時刻印は fresh とみなす。
inline constexpr uint32_t kTofSilenceUs = 300000;   // 0.3 s

/// Re-initialisation of the downward ToF while the craft is NOT armed (tof_silence_policy.hpp
/// decideToFReinit, executed by TofTask). CANDIDATES.
///  - kTofReinitSilenceUs: the silence (no new sample, or no sensor at all) that must have lasted before the
///    first attempt. 2 s = about 60 missing 30 Hz samples and 6.7 x kTofSilenceUs: the silence episode (and with it
///    the ARM refusal) is declared by Failsafe within 0.4 s, so it is long established before the sensor is reset,
///    and a bus glitch of a few samples never resets a healthy sensor.
///  - kTofReinitRetryIntervalUs: the time between two attempts for as long as the sensor stays dead. 10 s = at most
///    6 attempts per minute: a failing attempt on a sensor that does not answer costs one NACKed transaction and
///    the 20 ms XSHUT pulse (bus busy well under 1 ms), a working sensor needs one attempt, and the log carries at
///    most 6 attempt lines per minute (kTofReinitFailLogEveryAttempts throttles the failure line to 1 per minute).
///  - kTofReinitAttemptTimeoutUs: an attempt that took longer than this is rejected as failed (the sensor is put back
///    in reset). The driver cannot be interrupted, so this bounds ACCEPTANCE; the real upper bound of one attempt is
///    the driver's own: XSHUT pulse 20 ms + boot polling at most 500 ms (VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS)
///    + one 100 ms I2C transaction timeout + the ordinary bring-up of ~0.1 s = about 0.72 s, below this value.
/// 機体が ARM 中でないときの下向き ToF の再初期化（tof_silence_policy.hpp の decideToFReinit、実行は TofTask）。
/// 候補値。
///  - kTofReinitSilenceUs: 最初の試行までに続いていなければならない出力停止（新サンプル無し、またはセンサ自体が無い）。
///    2 s = 30 Hz のサンプル約 60 個の欠落で kTofSilenceUs の 6.7 倍: 出力停止のエピソード（ARM の拒否も）は
///    Failsafe が 0.4 s 以内に宣言するので、センサをリセットするずっと前から成立しており、数サンプルのバスの
///    乱れで正常なセンサをリセットすることはない。
///  - kTofReinitRetryIntervalUs: センサが動かない間の試行の間隔。10 s = 毎分最大 6 回: 応答しないセンサへの
///    失敗する試行は NACK された通信 1 回と 20 ms の XSHUT パルスだけ（バスの使用は 1 ms を十分下回る）、
///    動くセンサなら 1 回で済み、ログに出る試行の行は毎分最大 6 行（失敗の行は
///    kTofReinitFailLogEveryAttempts で毎分 1 行に抑制）。
///  - kTofReinitAttemptTimeoutUs: これより長くかかった試行は失敗として却下する（センサはリセットに戻す）。
///    ドライバは中断できないので、これは「受理」の上限で、1 回の試行の実際の上限はドライバ自身のもの:
///    XSHUT パルス 20 ms + 起動待ちのポーリング最大 500 ms（VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS）
///    + I2C 通信 1 回のタイムアウト 100 ms + 通常の立ち上げ約 0.1 s = 約 0.72 s で、この値より小さい。
inline constexpr uint32_t kTofReinitSilenceUs          = 2000000;    // 2 s
inline constexpr uint32_t kTofReinitRetryIntervalUs    = 10000000;   // 10 s
inline constexpr uint32_t kTofReinitAttemptTimeoutUs   = 1000000;    // 1 s
inline constexpr uint32_t kTofReinitFailLogEveryAttempts = 6;        // 6 x 10 s = 1 failure line per minute / 毎分 1 行

/// ToF-silence EPISODE: it starts at the loss (no new sample for kTofSilenceUs, or only invalid
/// samples while flying for kTofInvalidPersistUs) and ends only when the estimator has RE-ACQUIRED
/// the ToF: kTofReacquireSamples consecutive samples adopted with an innovation (|ToF height -
/// estimated height|) of at most kTofReacquireInnovationM, and the ToF alive (a continuous stream
/// of samples) for at least kTofAliveMinUs since the first sample after the loss. A sample that
/// merely ARRIVES proves nothing: the estimator may still hold an altitude that is metres off.
/// CANDIDATES. Basis: the ToF runs at ~30 Hz, so 10 samples are ~0.33 s of agreement; the 0.10 m
/// innovation is below the 0.15 m ground/airborne threshold of the takeoff/landing manager and
/// far below the 0.5 m innovation gate, and a few times the ToF noise; 1 s of life is longer than
/// a sensor that flickers on and off (0.5 s alive, 0.4 s silent) can reach. Decided in SILS
/// (recover_*.scn), NOT validated on a real flight.
/// ToF 出力停止の「エピソード」: 喪失（kTofSilenceUs の間新サンプル無し、または飛行中に無効サンプル
/// だけが kTofInvalidPersistUs 続く）で始まり、推定器が ToF を「取り込み直した」ときだけ終わる:
/// イノベーション（|ToF の高さ − 推定高度|）が kTofReacquireInnovationM 以下で採用されたサンプルが
/// kTofReacquireSamples 個連続し、かつ喪失後の最初のサンプルから kTofAliveMinUs 以上 ToF が生存
/// （サンプルが途切れず届く）していること。サンプルが「届いた」だけでは何も保証されない: 推定器は
/// 数 m ずれた高度を保持したままかもしれない。
/// 候補値。根拠: ToF は約 30 Hz なので 10 サンプルは約 0.33 s の一致。イノベーション 0.10 m は
/// 離着陸マネージャの接地/空中のしきい値 0.15 m より小さく、イノベーション判定 0.5 m よりずっと小さく、
/// ToF ノイズの数倍。生存 1 s は、点滅するセンサ（0.5 s 生存と 0.4 s 停止の繰り返し）が到達できない長さ。
/// SILS（recover_*.scn）で決めた値で、実機飛行での検証はまだ。
inline constexpr uint16_t kTofReacquireSamples     = 10;
inline constexpr float    kTofReacquireInnovationM = 0.10f;      // [m]
inline constexpr uint32_t kTofAliveMinUs           = 1000000;    // 1 s

/// Only INVALID samples for this long while the craft is in the air = the ToF cannot be used (e.g. a
/// floor material it cannot range), treated as silence. What counts as "in the air" is decided per
/// phase (tof_health_tracker.hpp classifyInvalidSample()): never on the ground (an invalid sample below
/// the minimum range is normal there), in TAKEOFF only after kTofInvalidTakeoffRiseM / kTofInvalidTakeoffMaxUs,
/// in LANDING only while the last valid height was at least kTofInvalidLandingMinHeightM, never in FLIP.
/// CANDIDATE. Basis: a single out-of-range sample lasts 33 ms (30 Hz), so 15 samples in a row are not noise;
/// the longest run of invalid samples measured in flight (TAKEOFF / FLYING) in the real logs is 298 ms.
/// Real logs (front_tof_poshold_yaw360, flip_test2): on the floor the sensor reports status 254 with distance
/// 0 until the craft has risen about 3 cm, so a takeoff starts with an invalid run of 186-344 ms after the
/// TAKEOFF state is entered (the motors spool up while the craft is still on the floor).
/// 空中にいる間に無効サンプルだけがこの時間続いたら、ToF は使えない（例: 測距できない床材）として出力停止と
/// 同じに扱う。「空中」の意味はフェーズごとに決める（tof_health_tracker.hpp の classifyInvalidSample()）:
/// 地上では数えない（最小レンジ未満の無効サンプルが正常）、TAKEOFF は kTofInvalidTakeoffRiseM /
/// kTofInvalidTakeoffMaxUs の後だけ、LANDING は最後の有効な高さが kTofInvalidLandingMinHeightM 以上の間だけ、
/// FLIP は数えない。候補値。根拠: 範囲外の単発は 33 ms（30 Hz）なので、15 サンプル連続はノイズではない。実機ログで
/// 測った飛行中（TAKEOFF・FLYING）の無効サンプルの最長連続は 298 ms。実機ログ（front_tof_poshold_yaw360、
/// flip_test2）: 床の上ではセンサは status 254・距離 0 を返し、機体が約 3 cm 上がるまで続く。つまり離陸は、
/// TAKEOFF に入ってから 186〜344 ms の無効の連続で始まる（機体がまだ床にいる間にモータが立ち上がる）。
inline constexpr uint32_t kTofInvalidPersistUs = 500000;   // 0.5 s

/// In TAKEOFF the invalid samples count toward the stuck verdict only after the craft has risen this far
/// by the accelerometer integral [m], or after kTofInvalidTakeoffMaxUs from the start of TAKEOFF, whichever comes first;
/// then kTofInvalidTakeoffPersistUs of them (not kTofInvalidPersistUs) make the verdict. Both are conditions that can only bring
/// the verdict EARLIER: an accelerometer offset error that fakes a rise on the floor, or a craft that the floor still holds (weak motors), at worst ends
/// in a ToF-silent LANDING on the floor, which is harmless (the thrust of that descent is below the support thrust: kNoTofThrustCapRatio). CANDIDATES.
/// Basis: (1) the real sensor returns its first valid sample at 0.031-0.032 m (3 real logs: 186 ms and 344 ms after
/// the TAKEOFF entry); in SILS (dead zone 31 mm, rest at 21 mm) the first valid sample comes when the estimated rise
/// is 0.011-0.015 m (395 ms after the entry), so 0.05 m is more than three times the dead-zone exit, and the
/// accelerometer integral errs by (offset error) x t^2 / 2 = 0.1 m/s^2 x (0.5 s)^2 / 2 = 1 cm in the first 0.5 s.
/// (2) What the persistence costs: with a ToF that stays invalid on the floor (the pinned estimator keeps the
/// auto-takeoff at full climb) the craft rises until the verdict, and then the blind descent has to stop the climb.
/// SILS, tof_invalid_alt_hover (ARM with every sample invalid), peak truth height after the ARM, noise off / n2 seeds 1-3 /
/// hover.thrust_corr 0.9 / 1.2: persistence 0.5 s: 0.39 / 0.99, 1.06, 1.24 / 0.26 / 0.91 m (the 1.2 m ceiling is
/// reached); 0.3 s: 0.26 / 0.55-0.74 / 0.18 / 0.55 m; 0.15 s (chosen): 0.20 / 0.38-0.41 / 0.15 / 0.28 m; 0.1 s: 0.16 /
/// 0.25-0.38 / 0.12 / 0.28 m. (Rejected alternative, counting 0.5 s from the TAKEOFF entry: about 0.16 m, but it cuts a slow
/// takeoff short.) 0.15 s is five samples at 30 Hz; the real logs show no invalid sample in the air after the first valid
/// one except 34 ms in a FLIP. Rise threshold (with 0.15 s): 0.02 / 0.03 / 0.05 m give a peak of 0.12 / 0.16 / 0.20 m
/// noise off and 0.25-0.38 / 0.25-0.38 / 0.38-0.41 m with n2: a lower threshold stops the climb up to 0.13 m earlier, but
/// 0.05 m keeps the margin over the 1 cm error of the accelerometer integral.
/// TAKEOFF では、加速度積分の推定でこの高さ [m] だけ上昇した後、または TAKEOFF の開始から kTofInvalidTakeoffMaxUs 経った後の
/// どちらか早い方から、無効サンプルを継続判定に数える。判定は kTofInvalidPersistUs でなく kTofInvalidTakeoffPersistUs
/// 続いたとき。どちらの条件も、判定を「早める」向きにしか働かない: 加速度計のオフセット誤差が床の上で偽の上昇を作っても、床がまだ支えている
/// 機体（モータが弱い）でも、せいぜい床の上で ToF なしの LANDING に入るだけで、害は無い（その降下の推力は支持推力未満: kNoTofThrustCapRatio）。候補値。
/// 根拠: (1) 実機のセンサが最初の有効サンプルを返すのは 0.031〜0.032 m（実機ログ 3 本。TAKEOFF 突入の
/// 186 ms 後と 344 ms 後）。SILS（不感帯 31 mm、静止 21 mm）では、最初の有効サンプルは上昇量の推定が 0.011〜0.015 m
/// のとき（突入の 395 ms 後）に来るので、0.05 m は不感帯を出る量の 3 倍以上。加速度積分の誤差は（オフセット誤差）× t² / 2
/// = 0.1 m/s² × (0.5 s)² / 2 = 1 cm（最初の 0.5 s）。(2) 継続時間の代償: ToF が床の上で無効のままだと（固定された推定が
/// 自動離陸を全速の上昇のままにする）、判定までは機体が上がり、その後 ToF なしの降下が上昇を止める。SILS の
/// tof_invalid_alt_hover（全サンプルが無効のまま ARM）、ARM 後の真値の最大高度、ノイズなし / n2 の seed 1〜3 /
/// hover.thrust_corr 0.9 / 1.2: 継続 0.5 s: 0.39 / 0.99、1.06、1.24 / 0.26 / 0.91 m（1.2 m の上限に届く）、0.3 s: 0.26 /
/// 0.55〜0.74 / 0.18 / 0.55 m、0.15 s（採用）: 0.20 / 0.38〜0.41 / 0.15 / 0.28 m、0.1 s: 0.16 / 0.25〜0.38 / 0.12 /
/// 0.28 m。（不採用の規則: TAKEOFF 突入から 0.5 s 数える。約 0.16 m だが、離陸の遅い機体を打ち切る。）0.15 s は 30 Hz で
/// 5 サンプル。実機ログには、最初の有効サンプルの後に空中で無効サンプルが出る例が FLIP 中の 34 ms の 1 回しかない。
/// 上昇量の閾値（継続 0.15 s）: 0.02 / 0.03 / 0.05 m で最大高度はノイズなしで 0.12 / 0.16 / 0.20 m、n2 で 0.25〜0.38 /
/// 0.25〜0.38 / 0.38〜0.41 m: 閾値が低いと上昇を最大 0.13 m 早く止められるが、0.05 m は加速度積分の
/// 誤差 1 cm に対する余裕を保つ。
inline constexpr float    kTofInvalidTakeoffRiseM  = 0.05f;      // [m]
/// Ground threshold of the takeoff/landing manager [m] (TakeoffLandingConfig::ground_tof_m takes its default from here). The two users differ at the boundary: the manager's ground flag
/// needs a valid reading STRICTLY BELOW it (takeoff_landing.cpp, "distance < ground_tof_m"), while TakeoffFloorWatch counts a reading AT OR BELOW it as in the band ("distance > band" ends the
/// run). A reading of exactly 0.05 m (never seen: the ToF resolves 1 mm and the noise is 0.5 cm) is therefore floor evidence for the watch but not "on the ground" for the manager.
/// CANDIDATE. Basis: the real sensor's first valid distance is 0.031 m and the craft's own height on the floor is a few cm, so 5 cm is 1.6 times the dead zone.
/// 離着陸マネージャの地上の閾値 [m]（TakeoffLandingConfig::ground_tof_m の既定値はここから取る）。境界での扱いは 2 つの使い手で違う: マネージャの接地フラグは、これ「未満」の
/// 有効な読みを要し（takeoff_landing.cpp の「distance < ground_tof_m」）、TakeoffFloorWatch は、これ「以下」の読みを帯の中として数える（「distance > band」で区間が終わる）。
/// ちょうど 0.05 m の読み（現れない: ToF の分解能は 1 mm、雑音は 0.5 cm）は、監視器には床の証拠だが、マネージャには「接地」でない。
/// 候補値。根拠: 実機のセンサが有効になる距離は 0.031 m で、機体が床の上にいるときの高さは数 cm なので、5 cm は不感帯の 1.6 倍。
inline constexpr float    kTakeoffGroundBandM       = 0.05f;     // [m]
/// Airborne threshold of the takeoff/landing manager [m]: a valid ToF reading above it means off the ground (TakeoffLandingConfig::airborne_tof_m takes its default from here). Between
/// kTakeoffGroundBandM and this the manager's ground flag holds its last value (hysteresis). TakeoffFloorWatch uses it to release the floor evidence it has latched at once (a valid reading above it proves the craft left the floor; readings above kTakeoffGroundBandM release it after kAutoTakeoffFloorHoldUs).
/// 離着陸マネージャの空中の閾値 [m]: これを超える有効な ToF の読みは離陸を意味する（TakeoffLandingConfig::airborne_tof_m の既定値はここから取る）。kTakeoffGroundBandM とこの値の間では、
/// マネージャの接地フラグは直前の値を保つ（ヒステリシス）。TakeoffFloorWatch は、ラッチした床の証拠をこれですぐ解く（これを超える有効な読みは、機体が床を離れた証拠。kTakeoffGroundBandM を超える読みが kAutoTakeoffFloorHoldUs 続いても解く）。
inline constexpr float    kTakeoffAirborneBandM     = 0.15f;     // [m]

/// Auto-takeoff (ALT_HOLD / POS_HOLD / API takeoff) that CANNOT lift is aborted (DISARM, the motors stop), but only on POSITIVE evidence that the craft is
/// still on the floor, never because evidence of flight is missing: the healthy downward ToF (not silent) reads a valid height of at most
/// kTakeoffGroundBandM, and that reading has not risen by more than kAutoTakeoffFloorRiseM for kAutoTakeoffFloorHoldUs (TakeoffFloorWatch,
/// takeoff_policy.hpp), kAutoTakeoffNoLiftAbortUs after TAKEOFF began. An invalid sample, a silent ToF, or a reading above the band is NOT that evidence
/// (an accelerometer cannot tell a craft that lifts slowly from one that the floor holds: a rule built on it aborts slow takeoffs in the air and does not
/// stop takeoffs without a ToF). CANDIDATES. Basis (SILS, motor health 0.85-1.0 at `hover.thrust_corr` 1.0, noise n0 / n2): a craft that cannot lift
/// (health 0.85) reads the SILS floor at 21 mm, below the 31 mm dead zone, so its samples are INVALID and the ToF-invalid rule (kTofInvalidTakeoffMaxUs, above) ends its takeoff by a
/// LANDING instead; this abort is for a craft that rests on a floor the ToF reads validly (the real sensor above 31 mm: a mat, a tilt) and keeps full correction on it for ever. 5 s is more
/// than five times the time an ordinary takeoff needs to leave the band (0.4-0.9 s) and longer than the 3 s of the controller's own backstop, which therefore only
/// declares the takeoff complete when the evidence is absent (takeoff_policy.hpp). The band is the ground threshold of the takeoff/landing manager (kTakeoffGroundBandM). The rise margin
/// 0.02 m is four times the ToF noise (0.5 cm), and the hold 1 s is thirty ToF samples. On the REAL sensor the rise margin never fires inside the band: the lowest valid reading is 0.031 m, so
/// the readings of the band span 0.031-0.050 = 0.019 m, less than 0.02 m; what ends a run there is leaving the band (a reading above kTakeoffGroundBandM), and the margin matters only for a
/// sensor that reads closer than 0.031 m (the SILS floor without the dead zone reads 0.021 m). And the real floor reads INVALID (status 254, the first 0.19-0.34 s of the TAKEOFF in the three real logs), so on a real
/// floor this evidence is seldom there at all (it needs a floor the ToF reads validly: a mat, a tilt). Once the evidence has held, it is latched: a single stray reading above the band cannot let the takeoff complete on the floor. The latch is released by the craft leaving the floor, with the weight of evidence the abort has: valid
/// readings above kTakeoffGroundBandM for kAutoTakeoffFloorHoldUs, or one valid reading above kTakeoffAirborneBandM (a craft that hangs in the 5-15 cm ground effect is not held for ever).
/// 浮けない自動離陸（ALT_HOLD / POS_HOLD / API の takeoff）は中止して DISARM（モータ停止）する。ただし、機体がまだ床の上にいるという「積極的な」証拠があるときだけで、飛んでいる証拠が
/// 「無い」ことでは中止しない: 健全な（出力停止でない）下向き ToF が kTakeoffGroundBandM 以下の有効な高さを読み、その読みが kAutoTakeoffFloorHoldUs の間、
/// kAutoTakeoffFloorRiseM を超えて増えておらず（TakeoffFloorWatch、takeoff_policy.hpp）、かつ離陸の開始から kAutoTakeoffNoLiftAbortUs 経ったとき。無効サンプル、
/// 出力停止の ToF、帯より上の読みは、その証拠ではない（加速度計は、ゆっくり浮く機体と床が支えている機体を区別できない: それに基づく規則は、空中で遅い離陸を
/// 打ち切り、ToF なしの離陸では止まらない）。候補値。根拠（SILS、`hover.thrust_corr` 1.0、モータの健全度 0.85〜1.0、ノイズ n0 / n2）: 浮けない機体（健全度 0.85）は SILS の床を 21 mm と読み、
/// 不感帯 31 mm より下なので、サンプルは「無効」で、離陸は上の ToF 無効の規則（kTofInvalidTakeoffMaxUs）が LANDING で終わらせる。この中止の対象は、ToF が有効に読める床（実センサの 31 mm 超: マット、傾き）に
/// 載ったまま最大の補正を出し続ける機体。5 s は、普通の離陸が帯を出る時間（0.4〜0.9 s）の 5 倍以上で、制御器自身の時間切れの備え（3 s）より長い。そのため制御器の備えは、
/// 証拠が無いときだけ離陸完了を判定する（takeoff_policy.hpp）。帯は離着陸マネージャの地上の閾値（kTakeoffGroundBandM）。上昇の余裕 0.02 m は ToF ノイズ（0.5 cm）の 4 倍、保持 1 s は ToF の 30 サンプル。
/// 「実センサ」では、上昇の余裕は帯の中では働かない: 有効な最小の読みは 0.031 m なので、帯の読みの幅は 0.031〜0.050 = 0.019 m で 0.02 m に届かない。そこで区間を終わらせるのは帯を出ること
/// （kTakeoffGroundBandM より上の読み）で、余裕が効くのは 0.031 m より近くを読むセンサだけ（不感帯の無い SILS の床は 0.021 m）。そして実機の床は「無効」を返す（status 254。実機ログ 3 本の TAKEOFF の最初の
/// 0.19〜0.34 s）ので、実機の床ではこの証拠そのものがほとんど立たない（ToF が有効に読める床: マット、傾き、が要る）。証拠が一度立ったらラッチする:
/// 帯の外へ 1 つ外れた読みで、床の上で離陸が完了することはない。ラッチは、機体が床を離れたことで解ける（中止と同じ重さの証拠: kTakeoffGroundBandM を超える有効な読みが kAutoTakeoffFloorHoldUs 続く、
/// または kTakeoffAirborneBandM を超える有効な読みが 1 つ。5〜15 cm の地面効果に留まる機体は、永久には保持されない）。
inline constexpr uint32_t kAutoTakeoffNoLiftAbortUs = 5000000;   // 5 s
inline constexpr float    kAutoTakeoffFloorRiseM    = 0.02f;     // [m]
inline constexpr uint32_t kAutoTakeoffFloorHoldUs   = 1000000;   // 1 s
inline constexpr uint32_t kTofInvalidTakeoffMaxUs  = 2000000;    // 2 s
inline constexpr uint32_t kTofInvalidTakeoffPersistUs = 150000;  // 0.15 s

/// In LANDING the invalid samples count toward the stuck verdict only while the last valid height was at least
/// this [m]; nearer the floor the sensor's dead zone makes an invalid sample normal again. CANDIDATE. Basis: the
/// real sensor's first valid distance is 0.031 m and the craft touches down at about 0.02-0.03 m, so 0.1 m is
/// three times the dead zone, with a margin for the estimator and the ToF noise.
/// LANDING では、最後の有効な高さがこの値 [m] 以上の間だけ、無効サンプルを継続判定に数える。床の近くでは
/// センサの不感帯のため無効サンプルが再び正常になる。候補値。根拠: 実機のセンサが有効になる距離は 0.031 m、
/// 接地は約 0.02〜0.03 m なので、0.1 m は不感帯の 3 倍で、推定器と ToF ノイズへの余裕がある。
inline constexpr float    kTofInvalidLandingMinHeightM = 0.10f;  // [m]

/// Touchdown in the ToF DEAD ZONE (the downward ToF is alive but reads "no target" below about 0.031 m, so the last
/// centimetres of a landing are invalid samples): during the LANDING descent, a valid reading at or below
/// kLandingDeadZoneBandM followed by only invalid samples for kLandingDeadZoneHoldMs means the craft is on the floor.
/// CANDIDATE. Basis: the one real landing (front_tof_poshold_yaw360, 0.28 m/s) read valid down to 0.039 m, one more
/// valid 0.031 m and then invalid for good; the only gap inside the band before that last valid reading was ONE invalid sample
/// (66 ms between two valid readings), so 0.2 s (6 samples at 33 Hz) is about 3 times the longest gap; at 0.28 m/s the craft
/// would sink 0.06 m in that time, more than it has left (it starts at 0.03-0.04 m). The band equals
/// kTofInvalidLandingMinHeightM: below it an invalid sample is normal (tof_health_tracker.hpp), above it is a fault.
/// ToF の「不感帯」での接地（下向き ToF は動作しているが、約 0.031 m より近いと「目標なし」を返すので、着陸の最後の数 cm
/// は無効サンプルになる）: LANDING の降下中に、kLandingDeadZoneBandM 以下の有効な読みの後、無効サンプルだけが
/// kLandingDeadZoneHoldMs 続いたら、機体は床の上にあるとする。候補値。根拠: 実機の唯一の着陸（
/// front_tof_poshold_yaw360、0.28 m/s）は 0.039 m まで有効、さらに 0.031 m が 1 回有効で、以後ずっと無効。最後の有効な
/// 読みより前に、帯の中で有効な読みの間に挟まった途切れは無効 1 サンプル（有効な読みの間隔 66 ms）だけだったので、
/// 0.2 s（33 Hz で 6 サンプル）は最長の途切れの約 3 倍。0.28 m/s ではその間に 0.06 m 沈み、残りの高さ（0.03〜0.04 m）を超える。帯は
/// kTofInvalidLandingMinHeightM と同じ値: それより下では無効サンプルが正常で（tof_health_tracker.hpp）、上では異常。
inline constexpr float    kLandingDeadZoneBandM  = kTofInvalidLandingMinHeightM;   // [m]
inline constexpr uint32_t kLandingDeadZoneHoldMs = 200;                            // 0.2 s

/// At rest while the ground flag of the takeoff/landing manager is true. While the flag is true ImuTask holds the vertical state at zero (applyVerticalGroundHandoff), and the flag stays true up to
/// kTakeoffAirborneBandM after the craft has been below kTakeoffGroundBandM, so the estimated vertical velocity there is a held zero, not a measurement of rest; the ToF readings are the evidence
/// (TakeoffLandingMgr::atRest()). Two values, both CANDIDATES. (1) kLandingRestBandM [m]: how far the valid readings of the present run may spread. Basis: the velocity gate of the same detector
/// (TakeoffLandingConfig::landing_vel_mps 0.05 m/s) over its shortest hold (stall_hold_ms 0.4 s) is 0.02 m, which equals kAutoTakeoffFloorRiseM (four times the ToF noise, 0.5 cm). A real craft
/// that holds still keeps the readings of 0.8 s within 0.005 m (real log flip_test1, 0.045-0.050 m, 123.8-124.6 s). (2) kLandingRestHoldMs: how long the stillness must last where the readings
/// are at or above kTakeoffGroundBandM (the flag cannot be the evidence there; on the floor the base hold applies). The band alone does not reject every craft that moves: in the 56 SILS cases where a LANDING that began in
/// the first 0.6 s of a takeoff ended in the air (the held estimate read "at rest" while the craft climbed or sank at up to 0.45 m/s), the ToF spread in the first 0.4 s was 0.011-0.068 m (median 0.034 m), and
/// 15 of the 56 (4 distinct trajectories, the craft turning round at the top of the climb, the SILS ToF flattening the turn) stayed within 0.02 m. A hold of 0.5 s still let 3 of the 56 stop in the air (0.074 m), 0.7 s let none, and
/// 1 s is the weight of evidence the takeoff already asks of the floor (kAutoTakeoffFloorHoldUs): about 1.4 times the least that works. A craft that really hangs in the ground effect keeps its readings, so the
/// longer hold costs it only time.
/// 接地フラグが true の間の静止。フラグが true の間、ImuTask は鉛直状態を 0 に固定し（applyVerticalGroundHandoff）、フラグは機体が kTakeoffGroundBandM を下回った後 kTakeoffAirborneBandM まで true のままなので、
/// そこでの推定の鉛直速度は固定された 0 であり、静止の測定ではない。証拠は ToF の読み（TakeoffLandingMgr::atRest()）。値は 2 つ、いずれも候補値。(1) kLandingRestBandM [m]: 今の区間の有効な読みが広がってよい幅。
/// 根拠: 同じ検出器の速度ゲート（TakeoffLandingConfig::landing_vel_mps 0.05 m/s）に最短の保持時間（stall_hold_ms 0.4 s）を掛けた 0.02 m で、kAutoTakeoffFloorRiseM（ToF ノイズ 0.5 cm の 4 倍）と同じ。
/// 静止する実機は 0.8 s の読みを 0.005 m の中に保つ（実機ログ flip_test1、0.045〜0.050 m、123.8〜124.6 s）。(2) kLandingRestHoldMs: 読みが kTakeoffGroundBandM 以上にある間（そこでは接地フラグは証拠になれない。床の上では基本の保持時間のまま）、
/// 静止が続くべき時間。幅だけでは、動く機体を全部は除けない: 離陸の最初の 0.6 s の間に始まった LANDING が空中で終わった SILS の 56 通り（固定された推定が「静止」を報告する間に、機体は最大 0.45 m/s で上昇または降下していた）で、
/// 最初の 0.4 s の ToF の広がりは 0.011〜0.068 m（中央値 0.034 m）、56 通りのうち 15 通り（軌道としては 4 通り。上昇の頂点で向きを変える機体を、SILS の ToF が平らに読む）は 0.02 m に収まった。保持時間 0.5 s では 56 通りのうち 3 通りが
/// 空中（0.074 m）で止まり、0.7 s では 0 通り。1 s は、離陸が床の証拠に既に求める重さ（kAutoTakeoffFloorHoldUs）で、働く最小の値の約 1.4 倍。地面効果で本当に浮いて止まる機体は読みを保つので、保持時間が長くなっても失うのは時間だけ。
inline constexpr float    kLandingRestBandM      = 0.02f;                          // [m]
inline constexpr uint32_t kLandingRestHoldMs     = 1000;                           // 1 s

/// failsafe_status is published by Failsafe every 100 ms. A reader that finds it older than this
/// treats the ToF as silent: a status nobody refreshes (Failsafe dead / PowerTask stalled) must
/// never leave the last "not silent" in force (every ToF protection reads that flag).
/// CANDIDATE. Basis: 10 publish periods (a few missed ticks are jitter), and short enough that a stalled Failsafe is noticed before a blind climb gets far.
/// failsafe_status は Failsafe が 100 ms ごとに発行する。これより古いと読んだ側は ToF を出力停止と
/// 扱う: 誰も更新しない状態（Failsafe の停止・PowerTask の停滞）で最後の「出力停止でない」が効き続けて
/// はならない（ToF の保護は全部このフラグを読む）。候補値。根拠: 発行周期の 10 倍（数回の取りこぼしは揺れ）で、Failsafe が停滞しても、ToF なしの上昇が進む前に気付ける短さ。
inline constexpr uint32_t kFailsafeStatusStaleUs = 1000000;   // 1 s

/// Descent without the downward ToF (the autonomous landing while the ToF is silent): a thrust that CANNOT climb.
/// The vertical velocity of that descent is an accelerometer integral (vertical_velocity_estimator.hpp), whose error
/// grows with the time since its last reset (offset error x time: real logs, 3 s: -0.25 .. -0.39 m/s), so a loop
/// that raises the thrust to correct "too fast a descent" climbs when the estimate is wrong (SILS: an accelerometer
/// offset of 0.1 m/s^2 made the craft rise again and touch down at 1.43 m/s, 0.2 m/s^2: 2.69 m and 5.06 m/s). The
/// structure is therefore:
///   thrust <= kNoTofThrustCapRatio x support thrust, always. The hover-thrust ESTIMATE is not a reference of the
///             ceiling: only the support thrust (the thrust that held the craft, calibrated by the accelerometer).
///             Whatever the velocity estimate says, the thrust stays below the weight, so the craft cannot rise.
///   thrust >= kBlindDescentThrustMinFrac x support thrust (the floor).
///   While the velocity estimate is trusted (kVerticalVelocityTrustS after its last reset): thrust = support + PI(target
///             - velocity), bounded to [floor, ceiling]. After that: the constant thrust = the ceiling (a constant
///             small downward acceleration), then the late fall (below) multiplies it.
///   support  : the commanded thrust that held the craft = thrust x g / upward specific force, the means over a window
///             that opens when the craft has left the floor (blind_landing.hpp, SupportThrustFilter): the floor
///             carries the weight only before the take-off, so the quotient is the hover thrust even while
///             accelerating. Without such a window (before the take-off, or the window is still short): min(thrust
///             of the flight so far, kBlindUncalibratedFrac x hover estimate), and the ceiling follows it, so
///             the error of the hover estimate is absorbed on the side that does not climb.
/// The PI gains are fractions of the hover-thrust estimate, so they do not depend on the airframe mass.
/// CANDIDATE values decided in SILS (detailed_design.md §3.1), NOT validated on a real flight.
/// ToF なしの降下（ToF が出力停止している間の自動着陸）: 「上昇できない」推力。この降下の鉛直速度は加速度計の積分
/// （vertical_velocity_estimator.hpp）で、誤差は最後のリセットからの時間とともに増える（オフセット誤差 × 時間。
/// 実機ログで 3 s: −0.25〜−0.39 m/s）。「降下が速すぎる」を直すために推力を上げるループは、推定が誤っていると上昇
/// する（SILS: 加速度計のオフセット 0.1 m/s^2 で機体が再上昇して 1.43 m/s で接地、0.2 m/s^2 では 2.69 m、5.06 m/s）。
/// そこで構造を次のようにする:
///   推力 ≦ kNoTofThrustCapRatio × 支持推力（常に）。上限の基準は hover 推力の「見積もり」ではなく、支持推力
///             （機体を支えていた指令推力を加速度計で較正した値）だけ。速度推定が何であれ推力は重さを下回り、
///             機体は上昇できない。
///   推力 ≧ kBlindDescentThrustMinFrac × 支持推力（下限）。
///   速度推定を信用できる間（最後のリセットから kVerticalVelocityTrustS）: 推力 = 支持推力 + PI(目標 − 速度) を
///             [下限, 上限] に制限。それを過ぎたら: 推力は上限の一定値（小さな一定の下向き加速度）で、
///             その後に後半の低下（下）を掛ける。
///   支持推力: 機体を支えていた指令推力 = 推力 × g ÷ 上向きの加速度計の測定値の、機体が床を離れた後に開く窓の
///             平均（blind_landing.hpp の SupportThrustFilter）。床が重さを支えるのは離陸前だけなので、
///             加速中でも商はホバー推力に一致する。窓が無い（離陸前、または窓がまだ短い）ときは
///             min(ここまでの飛行の推力, kBlindUncalibratedFrac × hover 推力の見積もり) で、上限もそれに従う
///             （見積もりの誤差を、上昇しない側で吸収する）。
/// PI のゲインは hover 推力の見積もりに対する割合なので、機体の質量に依らない。
/// 候補値で、SILS で決めた（detailed_design.md §3.1）。実機飛行での検証はまだ。
/// Thrust ceiling / support thrust [-]. CANDIDATE. Basis (margin = 1 - ratio = 5 % of the weight = 0.49 m/s^2 of upward reading error):
///   - the support thrust is thrust x g / reading, so a reading that is off by e m/s^2 makes it off by e / g: the mean of the bias-corrected
///     reading minus g in two real hovers is -0.05 and -0.09 m/s^2 (0.5 and 0.9 %, support thrust too HIGH); the +-0.2 m/s^2 injected in
///     the SILS matrix is 2 %;
///   - the spread of the 2 s window of near-hover samples in the real hover (front_tof_poshold_yaw360, 5.6 s): +-1.0 % of the long mean
///     (a 0.3 s window: +-4.9 %, 0.5 s: +-3.2 %, 1 s: +-2.5 %: that is why kSupportWindowS is 2 s); the replay of that log through the
///     calibration gives +0.7 % at the moment it is calibrated (4.1 s after the take-off began) and -1.1 .. +1.5 % afterwards;
///   - SILS sweep of this ratio over hover 0.5 m / 1.2 m, ALT_HOLD climb and early climb with the offset -0.2 / 0 / +0.2 m/s^2 injected at
///     the silence (noise n0 and n2, 3 seeds): no climb after the descent started for 0.95 and 0.96. With noise off, +0.2 m/s^2 injected from the
///     take-off and an ALT_HOLD climb, 0.97 climbed 0.07 m again and 0.98 climbed 0.54 m; at 1.00 the 1.2 m hover with +0.2 m/s^2 rose 0.05 m
///     (the ceiling equals the weight plus the offset error). Touchdown speed from 1.2 m with noise n2 falls with the ratio (0.95: 1.17 m/s,
///     0.97: 1.02, 0.98: 0.93, 0.99: 0.84, 1.00: 0.72): 0.95 (chosen) keeps a margin of about 3x the two measured errors above taken together
///     (+-1 % window, 0.5-0.9 % bias) and costs about 0.3 m/s of touchdown speed against 0.99.
/// 推力の上限 ÷ 支持推力 [-]。候補値。根拠（余裕 = 1 − 比 = 重さの 5 % = 上向きの読みの誤差 0.49 m/s^2 分）:
///   - 支持推力は 推力 × g ÷ 読み なので、読みが e m/s^2 ずれていれば e / g だけずれる: 実機のホバー 2 本の、バイアス補正済みの読みの平均 − g は
///     −0.05 と −0.09 m/s^2（0.5 と 0.9 %、支持推力は「高い」側）。SILS の試験で注入した ±0.2 m/s^2 は 2 %;
///   - 実機のホバー（front_tof_poshold_yaw360、5.6 s）でのホバー近傍のサンプルの 2 s 窓の散らばりは、長い平均の ±1.0 %（0.3 s 窓は
///     ±4.9 %、0.5 s は ±3.2 %、1 s は ±2.5 %: kSupportWindowS を 2 s にした理由）。そのログを較正に通した再生では、較正済みになった時点
///     （離陸の開始の 4.1 s 後）で +0.7 %、その後は −1.1〜+1.5 %;
///   - この比の SILS 掃引（0.5 m・1.2 m のホバー、ALT_HOLD の上昇、上昇初期。出力停止時に −0.2 / 0 / +0.2 m/s^2 を注入。ノイズ n0・n2、3 seed）:
///     降下が始まった後の上昇は 0.95 と 0.96 で無し。ノイズなし・+0.2 m/s^2 を離陸から注入・ALT_HOLD の上昇の組では、0.97 が 0.07 m、0.98 が 0.54 m
///     再上昇した。1.00 では 1.2 m のホバー + 0.2 m/s^2 が 0.05 m 再上昇した（上限が重さ + オフセット誤差に等しい）。ノイズ n2 の 1.2 m からの接地速度は
///     比とともに下がる（0.95: 1.17 m/s、0.97: 1.02、0.98: 0.93、0.99: 0.84、1.00: 0.72）: 0.95（採用）は上の 2 つの実測誤差（窓の ±1 %、バイアス
///     0.5〜0.9 %）を合わせた値の約 3 倍の余裕を保ち、0.99 に比べて接地速度が約 0.3 m/s 高くなる。
inline constexpr float kNoTofThrustCapRatio   = 0.95f;   // [-]   thrust ceiling = ratio x support thrust (< 1: cannot climb) / 推力の上限 = 比 × 支持推力（1 未満: 上昇できない）
/// Largest value the parameter safety.blind_descent.cap_ratio accepts [-]. It must stay below 1: the guarantee that the descent without a ToF cannot climb
/// is "thrust ceiling = ratio x support thrust < weight". A stored or set value above this is rejected (params.cpp). CANDIDATE.
/// safety.blind_descent.cap_ratio パラメータが受け付ける最大値 [-]。1 未満でなければならない: ToF なしの降下が上昇できないという保証は
/// 「推力の上限 = 比 × 支持推力 < 重さ」だから。これを超える値は設定も保存からの読み込みも受け付けない（params.cpp）。候補値。
inline constexpr float kNoTofThrustCapRatioMax = 0.99f;   // [-]
static_assert(kNoTofThrustCapRatio <= kNoTofThrustCapRatioMax && kNoTofThrustCapRatioMax < 1.0f,
              "the default must be within the accepted range and the range must stay below 1 (cannot climb)");
/// Support thrust before ANY airborne sample exists (the craft has not left the floor, or fewer than kSupportClimbMinS of airborne samples), as a
/// fraction of the hover-thrust ESTIMATE: that [-]. CANDIDATE. Basis: the ceiling is then 0.95 x 0.75 = 0.71 x the estimate, below the weight while the
/// estimate is below 1 / 0.71 = 1.40 x the true hover thrust (SILS with `hover.thrust_corr` 1.2: the estimate is 1.29 x the true one; a value of 0.9
/// climbs 2.6-6 m there). This is only the start value: once kSupportClimbMinS of airborne samples exist, the early estimate (below)
/// replaces it WITHOUT being cut by it (a cut by it makes the ceiling 0.63-0.71 x the thrust that holds the craft in the two front_tof logs, a fall at 0.29-0.37 g).
/// 空中のサンプルが「まだ 1 つも無い」とき（機体が床を離れていない、または空中のサンプルが kSupportClimbMinS 未満）の支持推力。hover 推力の
/// 「見積もり」に対する割合 [-]。候補値。根拠: 上限は 0.95 × 0.75 = 見積もりの 0.71 倍で、見積もりが真のホバー推力の 1 / 0.71 = 1.40 倍未満の間は
/// 重さを下回る（SILS の `hover.thrust_corr` 1.2 では見積もりが真値の 1.29 倍。0.9 ではそこで 2.6〜6 m 上昇した）。
/// これは開始値にすぎない: 空中のサンプルが kSupportClimbMinS 分たまったら、下の早い推定が、これで切られずに置き換える（切ると、上限は実機ログ
/// front_tof の 2 本で機体を支える推力の 0.63〜0.71 倍になり、0.29〜0.37 g で落ちる）。
inline constexpr float kBlindUncalibratedFrac = 0.75f;
inline constexpr float kBlindThrustFilterTauS = 1.0f;    // [s]   time constant of the thrust and of the accelerometer reading means once the window is longer than it (before: plain means) / 推力と加速度計の読みの平均の時定数（窓がこれより長くなってから。それまでは単純平均）
inline constexpr float kBlindMinUpwardSpecificForce = 2.0f;  // [m/s^2] below this the accelerometer reading is not used (estimator without specific_force, or free fall) / これ未満の加速度計の読みは使わない（specific_force を埋めない推定器、または自由落下）
inline constexpr float kSupportLeaveRiseM      = 0.10f;  // [m]     the craft has left the floor when the estimated rise since rest reaches this / 静止からの上昇量の推定がこれに達したら、機体は床を離れた
inline constexpr float kSupportLeaveExcessMps2 = 1.0f;   // [m/s^2] ... or when the low-passed upward reading exceeds g by this (the floor reads exactly g) / または、ろ波した上向きの読みが g をこれ超えたら（床の上は g ちょうど）
inline constexpr float kSupportLeaveLpfTauS    = 0.1f;   // [s]     low-pass time constant of that excess / その超過分のろ波の時定数
/// The commanded thrust is passed through a lag of this time constant before it is averaged against the accelerometer
/// (the thrust that acts on the craft trails the command). CANDIDATE. Basis: the flight-measured actuation of this craft is a
/// pure delay of 62 ms plus a first-order lag of 20 ms (the DOB model, analysis/scripts/alt_dob_design/README.md §2); one
/// first-order lag of 80 ms follows a slow command the same way (82 ms in total). Without it the mean command over a take-off
/// (the command falls from 0.43 N to 0.26 N within 0.4 s in SILS) leads the delivered thrust and the quotient overestimated the
/// support thrust by 12 % (SILS, estimate +20 %).
/// 指令推力は、加速度計と突き合わせて平均する前に、この時定数の遅れを通す（機体に働く推力は指令に遅れる）。候補値。根拠:
/// この機体の飛行で測った作動は、純遅れ 62 ms + 1 次遅れ 20 ms（DOB のモデル、analysis/scripts/alt_dob_design/README.md §2）。
/// 80 ms の 1 次遅れ 1 つが、ゆっくりした指令に同じように追従する（合計 82 ms）。これが無いと、離陸中の指令の平均
/// （SILS で指令が 0.4 s のうちに 0.43 N から 0.26 N へ下がる）が実際に出た推力より先行し、商が支持推力を 12 % 過大に
/// 見積もった（SILS、見積もり +20 %）。
inline constexpr float kSupportThrustLagS      = 0.08f;  // [s]
/// Only the samples whose low-passed reading is within this fraction of g of g enter the support-thrust means (the thrust is then
/// the weight, so the quotient is the hover thrust whatever the thrust-against-command curve is). CANDIDATE. Basis: SILS, where the
/// quotient thrust x g / reading is 12 % above the hover thrust at +0.2 g and about 5 % at +0.1 g (the delivered thrust is
/// concave in the command); 0.05 keeps that error near 2 %. A real hover reads g within 0.03 g (mean of the bias-corrected reading
/// minus g: -0.05 / -0.09 m/s^2 in two logs).
/// ろ波した読みが g のこの割合以内のサンプルだけを、支持推力の平均に入れる（推力は重さに等しく、推力と指令の曲線が何で
/// あれ、商はホバー推力になる）。候補値。根拠: SILS では、商（推力 × g ÷ 読み）が +0.2 g でホバー推力の 12 %、+0.1 g で
/// 約 5 % 高い（出る推力は指令に対し上に凸）。0.05 でその誤差は約 2 % に収まる。実機のホバーは g を 0.03 g 以内で読む
/// （バイアス補正済みの読みの平均 − g は 2 本のログで −0.05 / −0.09 m/s^2）。
inline constexpr float kSupportHoverBandFrac   = 0.05f;  // [-]
/// A sample enters the support-thrust means only after the low-passed reading has stayed in that band this long [s]: the low-pass
/// (kSupportLeaveLpfTauS) still sits near g for a moment after the craft has left the floor, and the climb must not leak in.
/// 標本が支持推力の平均に入るのは、ろ波した読みがその帯の中にこの時間 [s] 居続けた後だけ: 機体が床を離れた直後は
/// ローパスがまだ g の近くにあり、上昇のサンプルが紛れ込んではならない。
inline constexpr float kSupportBandSettleS     = 0.25f;  // [s]
/// Time of near-hover samples that must be averaged before the support thrust counts as calibrated [s]. CANDIDATE. Basis: real hovers
/// (front_tof_poshold_yaw360, 5.6 s): the window mean of thrust x g / reading departs from the long mean by up to +-4.9 % for 0.3 s,
/// +-3.2 % for 0.5 s, +-2.5 % for 1 s and +-1 % for 2 s; the thrust ceiling's margin is 5 %.
/// 支持推力が較正済みとみなされるまでに平均しなければならないホバー近傍のサンプルの時間 [s]。候補値。根拠: 実機のホバー
/// （front_tof_poshold_yaw360、5.6 s）で、窓の平均（推力 × g ÷ 読み）が長い平均から外れるのは 0.3 s で最大 ±4.9 %、
/// 0.5 s で ±3.2 %、1 s で ±2.5 %、2 s で ±1 %。推力の上限の余裕は 5 %。
inline constexpr float kSupportWindowS         = 2.0f;
/// Early estimate of the support thrust before a hover has been flown (the silence comes in the take-off climb): the quotient thrust x g /
/// reading over ALL the airborne samples, x kSupportClimbDerate, once kSupportClimbMinS of them are there. It is used as it is (bounded only by
/// kBlindCalibratedMaxFrac x the hover estimate): the lowered estimate (kBlindUncalibratedFrac) does not cut it. CANDIDATE. Basis:
///  - Real logs: the quotient is the weight times 0.98-1.03 in the real hovers and climbs (front_tof_poshold_yaw360 / yaw4x90) and 0.97-1.12 in flip_test2 (the thrust
///    falls while the average is taken), 1.11-1.12 in a SILS climb of +0.2 g (the delivered thrust is concave in the command). The ceiling (0.95 x support) over the thrust that
///    held the craft, on the replay of the 3 real logs before the calibration: derate 0.85: 0.79-0.91; 0.90: 0.83-0.96; 0.92: 0.85-0.98; 0.91: 0.84-0.98; 0.94: 0.87-1.00 (flip_test2 reaches the weight).
///    Target: 0.84 or more, never the weight. The target is 0.84, not 0.85, so that the re-take-off from the floor is zero: 0.91 is the smallest value with no worse cell, and the lowest ratio
///    of the real logs at 0.91 is 0.842. The rejected alternative, 0.85 with the cut by the lowered estimate, gives a real ceiling of 0.63-0.71 x the thrust that holds the craft in
///    the two front_tof logs (0.79-0.91 in flip_test2).
///  - SILS early-climb matrices (the silence 0.8-3.0 s after the take-off began; accelerometer offset -0.4..+0.6 m/s^2 injected at the ARM or at the silence; `hover.thrust_corr`
///    1.0-1.3; noise off / n2; 340 runs per value): with 0.92 four cells are WORSE than the descent from the lowered estimate (kBlindUncalibratedFrac) by more than 0.1 m/s of touchdown speed
///    (up to 1.1 m/s) or 0.05 m of rise: offset +0.6 m/s^2 (outside the 0.49 m/s^2 margin of kNoTofThrustCapRatio), `hover.thrust_corr` 1.2-1.3 and no noise, where the quotient reads 1.14 x the
///    weight (the concave thrust and the offset add), so the ceiling is 0.996 x the weight and the craft HOVERS at 0.6-0.7 m until the late fall begins at 5 s and then falls (2.02 m/s). A craft whose ceiling
///    is within 1.5 % of its weight hovers instead of descending, so a tiny change of the value swaps which cells are worse. Over the same 340 runs: derate 0.915: 2 such cells; 0.91 / 0.905 / 0.90: none.
///    0.91 is the smallest value with no worse cell (the invariant "a landing that has begun never lifts the craft from the floor again" needs none), and it is adopted.
///  - Price of the lower value: more strong touchdowns (above 1.5 m/s). Early-climb matrix (200 runs; the lowered estimate: 111): 18 (0.92), 21 (0.915), 33 (0.91), 42 (0.905), 49 (0.90), 84 (0.88). ToF-silent
///    matrix (6 scenes x 9 offsets x 3 `hover.thrust_corr` x n0 / n2 x 3 seeds, 870 valid runs): 38 (0.92) -> 67 (0.91), the largest increase 0.5 m/s. The real-log ceiling over the thrust that held the craft
///    scales with the value: its lowest (yaw4x90) is 0.851 / 0.842 / 0.837 at 0.92 / 0.91 / 0.905. The count of strong touchdowns is to be lowered by the barometer (a planned step), not by a value here.
/// ホバーを飛ぶ前（出力停止が離陸の上昇中に来る）の支持推力の早い推定: 空中の「全」サンプルでの商（推力 × g ÷ 読み）× kSupportClimbDerate を、
/// それが kSupportClimbMinS 分たまったら使う。そのまま使う（上限は hover 推力の見積もりの kBlindCalibratedMaxFrac 倍だけ）: 低くした見積もり
/// （kBlindUncalibratedFrac）では切らない。候補値。根拠:
///  - 実機ログ: 商は重さの、実機のホバーと上昇（front_tof_poshold_yaw360 / yaw4x90）で 0.98〜1.03 倍、flip_test2（平均を取る間に推力が下がる）で 0.97〜1.12 倍、
///    SILS の +0.2 g の上昇で 1.11〜1.12 倍（出る推力が指令に対し上に凸）。実機ログ 3 本の再生で、較正前の「上限 ÷ 機体を支えた推力」は、補正 0.85: 0.79〜0.91、
///    0.90: 0.83〜0.96、0.92: 0.85〜0.98、0.91: 0.84〜0.98、0.94: 0.87〜1.00（flip_test2 で重さに届く）。目標: 0.84 以上で、重さは超えない。目標は 0.85 でなく 0.84: 床からの再離陸を 0 にするため。0.91 は悪い組が無い最小の値で、0.91 での実機ログの最低の比は 0.842。
///    不採用の案（0.85 と低くした見積もりでの切り捨て）は、
///    front_tof の 2 本のログで上限を機体を支える推力の 0.63〜0.71 倍にする（flip_test2 は 0.79〜0.91 倍）。
///  - SILS の上昇初期の評価表（離陸の開始の 0.8〜3.0 s 後に出力停止。加速度計のオフセット −0.4〜+0.6 m/s^2 を ARM または出力停止で注入、`hover.thrust_corr` 1.0〜1.3、ノイズなし / n2、
///    1 値あたり 340 本）: 0.92 では、低くした見積もり（kBlindUncalibratedFrac）からの降下より接地速度が 0.1 m/s 超（最大 1.1 m/s）または上昇が 0.05 m 超「悪い」組が 4 つある: オフセット +0.6 m/s^2
///    （kNoTofThrustCapRatio の余裕 0.49 m/s^2 の外）、`hover.thrust_corr` 1.2〜1.3、ノイズなしで、商が重さの 1.14 倍を読み（上に凸な推力とオフセットが足し合わさる）、上限は重さの 0.996 倍になって、
///    機体は 5 s に後半の低下が始まるまで 0.6〜0.7 m に「ホバー」し、その後に落ちる（2.02 m/s）。上限が重さの 1.5 % 以内の機体は降りずにホバーするので、値のわずかな差で、悪い組の顔ぶれが入れ替わる。
///    同じ 340 本で、補正 0.915 では 2 組、0.91・0.905・0.90 では 0 組。0.91 は悪い組が無い最小の値（「始まった着陸が、機体を床からまた持ち上げない」という不変条件はそれを要する）で、これを採る。
///  - 値を下げる代償: 強い接地（1.5 m/s 超）が増える。上昇初期の評価表（200 本。低くした見積もりは 111 本）で 18（0.92）、21（0.915）、33（0.91）、42（0.905）、49（0.90）、84（0.88）。ToF なしの評価表
///    （6 場面 × オフセット 9 通り × `hover.thrust_corr` 3 通り × n0 / n2 × 3 seed）の成立した 870 本で 38（0.92）→ 67（0.91）、増え幅の最大は 0.5 m/s。実機ログの「上限 ÷ 機体を支えた推力」は補正に比例する:
///    最低（yaw4x90）は 0.92 / 0.91 / 0.905 で 0.851 / 0.842 / 0.837。強い接地の件数は、ここの値でなく気圧計（予定の段階）で下げる。
inline constexpr float kSupportClimbDerate     = 0.91f;  // [-]
inline constexpr float kSupportClimbMinS       = 0.15f;  // [s]
inline constexpr float kBlindCalibratedMinFrac = 0.5f;   // [-]   accelerometer-calibrated support thrust is bounded below by frac * hover thrust estimate / 加速度計で較正した支持推力の下限（hover 推力の見積もりに対する割合）
inline constexpr float kBlindCalibratedMaxFrac = 1.5f;   // [-]   ... and above by frac * hover thrust estimate (a bad reading cannot start the descent at an absurd thrust) / 同じく上限（誤った読みで降下が異常な推力から始まらない）
inline constexpr float kBlindDescentRateMps   = 0.4f;   // [m/s] target descent speed (down) / 目標の降下速度（下向き）
inline constexpr float kBlindDescentKp        = 1.0f;   // [1/(m/s)] P gain, fraction of the hover thrust estimate per m/s of velocity error / P ゲイン（速度誤差 1 m/s あたりの hover 推力の見積もりに対する割合）
inline constexpr float kBlindDescentTiS       = 2.0f;   // [s]   integral time of the PI / PI の積分時間
inline constexpr float kBlindDescentCorrectionLimitFrac = 1.0f;  // [-] PI output (and integral) limit, fraction of the hover thrust estimate / PI の出力（と積分）の上限（hover 推力の見積もりに対する割合）
inline constexpr float kBlindDescentThrustMinFrac = 0.25f;  // [-]  thrust floor, fraction of the support thrust (the attitude loop needs some thrust to act; a climb of 1.3 m/s is stopped by it in 0.2 s) / 推力の下限（支持推力に対する割合。姿勢ループが働くには推力が要る。1.3 m/s の上昇はこれで 0.2 s で止まる）
/// The accelerometer-integral velocity is used by the PI only this long after its last reset [s] (the last time the
/// craft rested on the ground, or the ESKF velocity was followed with the ToF healthy). CANDIDATE. Basis: real flight
/// logs (front_tof_poshold_yaw360 / yaw4x90, flip_test2; the estimator fed with the real IMU after a ToF loss in the
/// hover): the velocity error is -0.21 m/s after 1 s, -0.25 .. -0.39 m/s after 3 s, -0.44 .. -0.71 m/s after 5 s
/// (flip_test2: -0.34 m/s after 2 s). 2 s keeps the error near 0.3 m/s, below the 0.4 m/s target, and is long enough
/// to remove a climb the craft had when the ToF stopped (0.2 s at the thrust floor). SILS sweep 1 / 2 / 3 / 5 s: the hover and ALT_HOLD-climb cells are
/// identical (the craft lands before the PI phase ends); the early-climb cells differ only at 1 s (rise 0.93-1.00 m, touchdown 2.0 m/s, against
/// 0.74-0.76 m and 1.9 m/s), so 2 s is the shortest value that costs nothing.
/// 加速度積分の速度を PI が使うのは、その最後のリセット（機体が最後に地上で静止した、または ToF が健全で ESKF の
/// 速度に従った時点）からこの時間 [s] 以内だけ。候補値。根拠: 実機ログ（front_tof_poshold_yaw360 / yaw4x90、
/// flip_test2。ホバー中の ToF 喪失の後に実際の IMU を推定器へ通した）で、速度誤差は 1 s で −0.21 m/s、3 s で
/// −0.25〜−0.39 m/s、5 s で −0.44〜−0.71 m/s（flip_test2 は 2 s で −0.34 m/s）。2 s なら誤差は 0.3 m/s 付近で
/// 目標 0.4 m/s より小さく、ToF が止まった時点の上昇を止めるのに足りる長さ（推力の下限で 0.2 s）。SILS の 1 / 2 / 3 / 5 s の掃引: ホバーと
/// ALT_HOLD の上昇の場面は同一（PI の区間が終わる前に着地する）。上昇初期の場面だけ 1 s で違う（上昇 0.93〜1.00 m、接地 2.0 m/s。2 s 以上は
/// 0.74〜0.76 m、1.9 m/s）ので、2 s が代償なしの最短値。
inline constexpr float kVerticalVelocityTrustS = 2.0f;   // [s]

/// Late fall (the last safety of the descent without a ToF): the velocity comes from an integral, whose error
/// grows with time, so after kBlindLandingLateStartS of that descent the thrust is multiplied by a factor that falls
/// at kBlindLandingLateRampPerS and reaches ZERO at kBlindLandingLateStartS + 1 / kBlindLandingLateRampPerS = 8.33 s,
/// whatever the velocity estimate says. The time is counted over the blind descents of one LANDING (a ToF that
/// returns and stops again does not restart it). A normal descent from the 1.2 m ceiling (a constant downward acceleration of about 0.05 g) ends in
/// about 2-3 s, so this never acts then.
/// ToF なしの降下の最後の安全策（後半の低下）: 速度は積分から来て、その誤差は時間とともに増えるので、この降下が
/// kBlindLandingLateStartS 続いたら、推力に係数を掛ける。係数は kBlindLandingLateRampPerS で下がり、速度推定が
/// 何であれ kBlindLandingLateStartS + 1 / kBlindLandingLateRampPerS = 8.33 s で「ゼロ」になる。時間は 1 回の LANDING
/// の中の ToF なしの降下を通して数える（ToF が戻ってまた止まっても数え直さない）。1.2 m の天井からの
/// 通常の降下（約 0.05 g の一定の下向き加速度）は約 2〜3 s で終わるので、そのときはここは働かない。
inline constexpr float kBlindLandingLateStartS = 5.0f;    // [s]   the late fall starts after this time of the descent without a ToF / 後半の低下は、ToF なしの降下がこの時間続いた後に始まる
inline constexpr float kBlindLandingLateRampPerS = 0.3f;  // [1/s] fall rate of the thrust factor: zero 3.33 s after the start / 推力の係数の低下率: 開始の 3.33 s 後にゼロ

/// The vertical velocity estimate follows the ESKF only after the ESKF vertical state has been free
/// (ToF healthy and consumed, not pinned by the on-ground hold) for this long [s]. CANDIDATE. Basis: at
/// the ground-to-air edge the estimator resets velocity to 0 (imu_task, handoff) although the craft is
/// climbing, and the ToF innovations need about a second to pull it back (SILS tof_silent_takeoff_est_high:
/// ESKF 0.56 m/s while the truth is 1.27 m/s 0.4 s after the edge; hover noise +-0.1-0.2 m/s). The
/// 80-run matrix is insensitive to 0.5 / 1 / 2 s (worst noise-n2 touchdown speed 0.99 / 0.96 / 1.00 m/s).
/// 鉛直速度の推定が ESKF に従うのは、ESKF の鉛直状態が自由（ToF が健全で取り込み済み、接地ホールドで固定されて
/// いない）になってからこの時間 [s] 経ったあと。候補値。根拠: 接地→空中のエッジで推定器は、機体が上昇中でも
/// 速度を 0 にリセットし（imu_task のハンドオフ）、ToF のイノベーションが引き戻すのに約 1 s かかる（SILS
/// tof_silent_takeoff_est_high: エッジの 0.4 s 後に、真値 1.27 m/s に対し ESKF は 0.56 m/s。ホバーでのノイズは
/// ±0.1〜0.2 m/s）。80 本の試験は 0.5 / 1 / 2 s に鈍感（ノイズ n2 の接地速度の最悪 0.99 / 0.96 / 1.00 m/s）。
inline constexpr float kVerticalVelocityHandoffSettleS = 1.0f;   // [s]
/// Time constant [s] of the complementary observer that follows the ESKF vertical velocity while the
/// ToF is healthy. CANDIDATE. Basis: the ESKF velocity is noisy and wrong for a moment after a transient, the
/// accelerometer integral drifts with its offset error; a long time constant keeps the accelerometer's fast
/// part (the descent starts within 0.3 s) and bounds the observer's drift to (offset error x tau). SILS sweep
/// over the 80-run matrix (5 scenarios x hover.thrust_corr 0.9-1.2 x noise off / n2 x 3 seeds), worst touchdown
/// speed with noise n2 and the number of runs that ended neither by touchdown nor by impact: tau 1 s: 1.29 m/s,
/// 2 runs; 2 s: 1.15 m/s, 0; 4 s: 0.96 m/s, 0 (chosen); 8 s: 0.83 m/s, 1 run. 4 s is on the flat part.
/// ToF が健全な間 ESKF の鉛直速度に従う相補オブザーバの時定数 [s]。候補値。根拠: ESKF の速度はノイズが大きく、
/// 過渡の直後は一時的に間違う。加速度の積分はオフセット誤差でドリフトする。長い時定数は加速度計の速い成分
/// （降下は 0.3 s 以内に始まる）を残し、オブザーバのドリフトを（オフセット誤差 × tau）に抑える。80 本の試験
/// （5 場面 × hover.thrust_corr 0.9〜1.2 × ノイズなし / n2 の 3 seed）での SILS 掃引、ノイズ n2 の接地速度の
/// 最悪と、接地でも衝撃でも終わらなかった本数: tau 1 s: 1.29 m/s、2 本。2 s: 1.15 m/s、0 本。4 s: 0.96 m/s、
/// 0 本（採用）。8 s: 0.83 m/s、1 本。4 s は平らな範囲の上。
inline constexpr float kVerticalVelocityEskfTauS = 4.0f;   // [s]
/// Time constant [s] of the offset low-pass at rest on the ground (rule 1). CANDIDATE. Basis: rest
/// lasts at least the spool-up of an ARM (0.3 s in SILS), the reading at rest is quiet (props stopped), so
/// a short filter that still averages the 400 Hz white noise is enough.
/// 地上で静止しているときのオフセットのローパスの時定数 [s]（規則 1）。候補値。根拠: 静止は少なくとも
/// ARM のスプールアップ（SILS で 0.3 s）続き、静止中の読みは静か（プロペラ停止）なので、400 Hz の
/// 白色ノイズを平均する短いフィルタで足りる。
inline constexpr float kVerticalVelocityRestTauS = 0.2f;   // [s]
/// Commanded thrust below this fraction of the hover-thrust estimate cannot lift the craft: on the ground (phase
/// Grounded) it is the "at rest" condition of the vertical velocity estimate (rule 1). With a manual throttle above it
/// (STABILIZE / ACRO take-off) the craft may be leaving the floor and the integral takes over from zero. CANDIDATE. Basis:
/// the true hover thrust is 0.8-1.1 x the estimate in the SILS matrix (`hover.thrust_corr` 0.9-1.2), so 0.8 is below any
/// weight; the ALT_HOLD / POS_HOLD ground phase commands zero thrust.
/// この割合より低い指令推力は機体を持ち上げられない: 地上（フェーズ Grounded）では、鉛直速度の推定の「静止」の条件
/// （規則 1）。これより大きい手動スロットル（STABILIZE / ACRO の離陸）では機体が床を離れているかもしれず、積分が
/// 0 から引き継ぐ。候補値。根拠: SILS の試験（`hover.thrust_corr` 0.9〜1.2）で真のホバー推力は見積もりの 0.8〜1.1 倍
/// なので、0.8 はどの重さよりも低い。ALT_HOLD / POS_HOLD の地上フェーズは推力ゼロを指令する。
inline constexpr float kVerticalVelocityRestThrustFrac = 0.8f;   // [-]
/// Release level of the contact rule of the vertical velocity estimate, fraction of the contact threshold: the
/// velocity is set to zero when the upward reading falls back below it after a contact (the spike is over).
/// 鉛直速度の推定の接触の規則の解除の水準（接触の閾値に対する割合）: 接触の後、上向きの読みがこれを下回ったら（突起が
/// 終わったら）速度を 0 にする。
inline constexpr float kVerticalVelocityContactReleaseFrac = 0.75f;   // [-]
/// Limit of the learned accelerometer offset B [m/s^2] (a bad estimate cannot bias the integral by
/// more than this; real offsets are 0.1-0.4 m/s^2, see sensor_noise.hpp).
/// 学習した加速度計のオフセット B の制限 [m/s^2]（誤った推定でも積分をこれ以上偏らせない。実際のオフセットは
/// 0.1〜0.4 m/s^2、sensor_noise.hpp 参照）。
inline constexpr float kVerticalVelocityOffsetLimit = 1.0f;   // [m/s^2]
/// Magnitude limit of the vertical velocity estimate [m/s] (a runaway guard; far above any real value).
/// 鉛直速度推定の大きさの制限 [m/s]（発散防止。実際の値よりずっと大きい）。
inline constexpr float kVerticalVelocityLimitMps = 20.0f;

/// Touchdown while the ToF is silent: an IMPACT followed by STILLNESS. The impact alone is not
/// evidence (real flight logs show vibration up to 1.56 g in hover, 2.28 g in aggressive flight); a
/// craft that is held up by the floor is not accelerating, is not descending, and is not being
/// carried by the thrust. So the touchdown latches when, after an impact (|accel| above
/// kBlindTouchdownAccelG for kBlindTouchdownSamples consecutive IMU samples, 400 Hz), ALL of these hold
/// for kBlindTouchdownStillS:
///   - the low-passed [tau kBlindTouchdownStillLpfTauS] |accel| is within kBlindTouchdownStillToleranceG
///     of 1 g (the craft is not accelerating),
///   - the commanded thrust is below kBlindTouchdownThrustFrac of the support thrust (it could not
///     hold the craft up),
///   - the vertical velocity estimate is within kBlindTouchdownVelocityMps of zero.
/// CANDIDATE. Basis: three real flight logs (front_tof_poshold_yaw360, front_tof_poshold_yaw4x90,
/// flip_test2; detailed_design.md §3.1 has the table). In-flight runs of >= 2 consecutive samples above
/// the impact threshold (TAKEOFF/FLYING, FLIP and the final crash excluded): 1.4 g: 16 / 11 / 44; 1.6 g: 1 / 0 / 6;
/// 1.8 g: 1 / 0 / 1; 2.0 g: 0 / 0 / 0 (chosen; max in flight 1.58 g hover, 2.28 g aggressive). The one real touchdown
/// (yaw360, closed-loop landing at 0.28 m/s) shows 3 consecutive samples above 2 g, peak 3.39 g, so 2.0 g x 2
/// is seen with margin. With the stillness conditions replayed on the logs there is no false latch in flight and the
/// real touchdown latches 0.45 s after the impact. In SILS it latches 0.38-0.91 s (median 0.44 s) after the contact in all 80 runs of
/// the matrix with the 3 G impact cut disabled (none in the air). The 3 G impact cut (Failsafe, 2 samples) still fires
/// FIRST in 79 of 80 normal runs: the real touchdown above peaks at 3.39 g and the SILS contact at 6-9 g, so the
/// detector only ends landings that touch down softly (below 3 g). Stillness values: |LPF(|a|) - 1 g| < 0.15 g
/// is 1.5-1.8x the largest deviation of the filtered reading in the two real hovers (0.085 and 0.097 g, std 0.02 g); 0.25 s = 2.5 time constants of the filter after the
/// impact has decayed; thrust ratio 0.85 and |v| < 0.15 m/s exclude a descent at the support thrust (ratio ~1,
/// v = -0.4 m/s) with a margin. NOT validated on a real blind landing.
/// ToF 出力停止中の接地: 「衝撃」の後の「静止」。衝撃だけでは証拠にならない（実機ログではホバーで 1.56 g、
/// 激しい飛行で 2.28 g までの振動がある）。床に支えられた機体は、加速せず、降下せず、推力で運ばれてもいない。
/// そこで接地は、衝撃（|加速度| が kBlindTouchdownAccelG を kBlindTouchdownSamples 回連続の IMU サンプル
/// （400 Hz）で超える）の後、次の「全部」が kBlindTouchdownStillS 続いたときにラッチする:
///   - ろ波した[時定数 kBlindTouchdownStillLpfTauS]|加速度| が 1 g の ±kBlindTouchdownStillToleranceG 以内
///     （機体が加速していない）、
///   - 指令推力が支持推力の kBlindTouchdownThrustFrac 未満（機体を支えられない推力）、
///   - 鉛直速度の推定が 0 の ±kBlindTouchdownVelocityMps 以内。
/// 候補値。根拠: 実機ログ 3 本（front_tof_poshold_yaw360、front_tof_poshold_yaw4x90、flip_test2。表は
/// detailed_design.md §3.1）。衝撃の閾値を超えるサンプルが 2 個以上連続した、飛行中（TAKEOFF/FLYING。FLIP と最後の
/// 墜落を除く）の回数: 1.4 g: 16 / 11 / 44、1.6 g: 1 / 0 / 6、1.8 g: 1 / 0 / 1、2.0 g: 0 / 0 / 0（採用。飛行中の最大は
/// ホバー 1.58 g、激しい飛行 2.28 g）。本物の接地 1 回（yaw360、閉ループの着陸、0.28 m/s）は 2 g 超が 3 サンプル連続、
/// 最大 3.39 g なので、2.0 g × 2 は余裕をもって見える。静止の条件を含めてログに再生すると、飛行中の誤ラッチは 0、
/// 本物の接地は衝撃の 0.45 s 後にラッチする。SILS では、3 G の衝撃カットを無効にした 80 本すべてで接触の 0.38〜0.91 s 後（中央値 0.44 s）にラッチする
/// （空中でのラッチは 0）。3 G の衝撃カット（Failsafe、2 サンプル）は通常の設定の 80 本中 79 本で「先に」
/// 働く: 上の本物の接地は最大 3.39 g、SILS の接触は 6〜9 g なので、この検出器が終わらせるのは 3 g 未満の柔らかい
/// 接地だけ。静止の値: |ろ波した |加速度| − 1 g| < 0.15 g は、実機のホバー 2 回でろ波した読みの最大のずれ
/// （0.085 g と 0.097 g、標準偏差 0.02 g）の 1.5〜1.8 倍。0.25 s は衝撃が減衰した後のフィルタの時定数の 2.5 倍。推力比 0.85 と |v| < 0.15 m/s は、支持推力での降下
/// （比 ≈ 1、v = −0.4 m/s）を余裕をもって除く。実機での ToF なしの着陸の検証はまだ。
inline constexpr float    kBlindTouchdownAccelG    = 2.0f;                                      // [g]    impact threshold; also the default of safety.blind_touchdown.accel_g (params.cpp) / 衝撃の閾値。safety.blind_touchdown.accel_g の既定値でもある（params.cpp）
inline constexpr float    kBlindTouchdownAccelMps2 = kBlindTouchdownAccelG * sf::math::kGravity;    // [m/s^2] same as above in m/s^2 / 上と同じ値を m/s^2 で
inline constexpr uint16_t kBlindTouchdownSamples   = 2;       // consecutive IMU samples (400 Hz) / 連続する IMU サンプルの数（400 Hz）
inline constexpr float    kBlindTouchdownStillToleranceG = 0.15f;  // [g]    | low-passed |accel| - 1 g | below this = not accelerating / ろ波した |加速度| が 1 g からこれ未満のずれ = 加速していない
inline constexpr float    kBlindTouchdownStillLpfTauS = 0.1f;   // [s]    low-pass time constant of |accel| for the stillness test / 静止の判定に使う |加速度| のろ波の時定数
inline constexpr float    kBlindTouchdownStillS    = 0.25f;   // [s]    the stillness must last this long / 静止がこの時間続くこと
inline constexpr float    kBlindTouchdownThrustFrac = 1.0f;   // [-]    commanded thrust / support thrust below this = cannot hold the craft up (always true in the descent without a ToF: the ceiling is kNoTofThrustCapRatio < 1) / 指令推力 ÷ 支持推力がこれ未満 = 機体を支えられない（ToF なしの降下では常に真: 上限は kNoTofThrustCapRatio < 1）
inline constexpr float    kBlindTouchdownVelocityMps = 0.15f;  // [m/s]  |vertical velocity estimate| below this = not moving / |鉛直速度の推定| がこれ未満 = 動いていない

/// Touchdown by the FLOOR HOLDING THE CRAFT UP (the second path, OR with the impact-then-stillness one above; no impact needed): the commanded
/// thrust is below kBlindFloorThrustFrac of the support thrust AND the low-passed |accel| is within kBlindFloorToleranceG of 1 g, for
/// kBlindFloorS. Physics: an accelerometer reads the thrust per mass (specific force). In the air, with a thrust below 0.7 of the weight, it
/// reads below 0.7 g (+- drag); reading 1 g with that thrust can only be the floor carrying the rest of the weight. A false latch needs the
/// support thrust to exceed the weight by 1 - 0.1 = 0.9 g / 0.7 g: 29 %. The calibrated support thrust is within about 2 % of the weight
/// (kNoTofThrustCapRatio); the uncalibrated one is at most 0.75 x the hover estimate, so it needs an estimate 1.7 x the true hover (SILS with
/// `hover.thrust_corr` 1.2: 1.29-1.45). CANDIDATE.
/// 「床が機体を支えている」ことによる接地（衝撃のあとの静止との OR の、2 つ目の経路。衝撃は要らない）: 指令推力が支持推力の
/// kBlindFloorThrustFrac 未満、かつ ろ波した |加速度| が 1 g の ±kBlindFloorToleranceG 以内、が kBlindFloorS 続くこと。物理: 加速度計は
/// 「推力 ÷ 質量」を読む。空中で推力が重さの 0.7 倍未満なら、読みは 0.7 g 未満（抗力で多少ずれる）で、その推力で 1 g を読むのは、
/// 残りの重さを床が支えているときだけ。誤ラッチには、支持推力が重さを 0.9 g ÷ 0.7 g − 1 = 29 % 超えて高いことが要る。較正した
/// 支持推力は重さの約 2 % 以内（kNoTofThrustCapRatio）、較正なしは見積もりの 0.75 倍以下なので、見積もりが真のホバーの 1.7 倍
/// 要る（SILS の `hover.thrust_corr` 1.2 では 1.29〜1.45 倍）。候補値。
inline constexpr float    kBlindFloorThrustFrac = 0.7f;   // [-]   commanded thrust / support thrust below this / 指令推力 ÷ 支持推力がこれ未満
inline constexpr float    kBlindFloorToleranceG = 0.1f;   // [g]   | low-passed |accel| - 1 g | below this / ろ波した |加速度| が 1 g からこれ未満のずれ
inline constexpr float    kBlindFloorS          = 0.5f;   // [s]   both must hold this long / 両方がこの時間続くこと

/// Longest LANDING time allowed while the downward ToF is silent [us], counted from the moment the state ENTERED
/// LANDING (not from the silence, so a ToF that returns and stops again does not extend it); once it has passed and the
/// ToF is silent the landing is declared complete (motors off). CANDIDATE. Basis: since the thrust of the descent without
/// a ToF is at most kNoTofThrustCapRatio x the support thrust, the craft is always sinking: from the 1.2 m ceiling in about
/// 2-3 s (a constant downward acceleration of about 0.05 g, not a regulated speed; SILS: 2.3-2.9 s after the silence); the late
/// fall brings the thrust to ZERO at 8.33 s of that descent, so at 10 s the craft is on the ground or falling
/// (10 s = 8.33 s + 1.67 s, the fall time from 13 m: sqrt(2 * 13 / 9.8) = 1.63 s): cutting the motors then stops nothing
/// that was holding it up.
/// 下向き ToF の出力停止中に LANDING で許す最長時間 [us]。状態が LANDING に「入った時刻」から数える（出力停止の
/// 時刻からではない。ToF が戻ってまた止まっても延びない）。これを過ぎて ToF が出力停止中なら着陸完了（モータ停止）
/// とする。候補値。根拠: ToF なしの降下の推力は高々 kNoTofThrustCapRatio × 支持推力なので、機体は常に沈む:
/// 1.2 m の天井から約 2〜3 s（約 0.05 g の一定の下向き加速度で、調整された速度ではない。SILS: 出力停止の 2.3〜2.9 s 後）。後半の低下はこの降下の 8.33 s で
/// 推力を「ゼロ」にするので、10 s では機体は接地しているか落下中（10 s = 8.33 s + 1.67 s。13 m からの
/// 落下時間 sqrt(2 × 13 / 9.8) = 1.63 s）で、そこでモータを切っても機体を支えていたものを止めることにならない。
inline constexpr uint32_t kBlindLandingTimeoutUs = 10000000;   // 10 s
/// Minimum time of the descent without a ToF before the time limit may cut the motors [us]: the landing is completed by time only when the state has
/// been LANDING for kBlindLandingTimeoutUs AND the ToF has been continuously silent in it for this long. CANDIDATE. Basis: a LANDING with a live
/// ToF can last longer than kBlindLandingTimeoutUs (a slow descent, ground effect); if the ToF then dies, the limit alone would stop the motors in the air at once.
/// 5.5 s: the longest descent without a ToF measured, from the silence to the first contact, is 4.7 s (SILS, the 972-run ToF-silent matrix, a descent whose thrust ceiling is not limited by the support thrust;
/// per case: 0.5 m hover 1.0-2.6 s, 1.2 m hover 1.4-2.8 s, API takeoff 0.5-2.4 s, auto-takeoff 0.5-3.6 s, ALT_HOLD stick climb 1.2-4.7 s, the longest with noise n0 and an accelerometer offset). With the ceiling limited by the
/// support thrust the longest is 4.0 s. So 5.5 s covers every one of them with 0.8 s to spare (1.5 s with the limited ceiling), and it is short of the 8.33 s at which the late fall brings the thrust to zero: the thrust
/// has been falling for only 0.5 s (to 85 %) when the limit is reached, kBlindLandingLateStartS = 5 s being the start of that fall. A value of 4 s would cut the motors up to 0.7 s before the contact of the longest descents.
/// 時間切れがモータを切ってよくなるまでの、ToF なしの降下の最低時間 [us]: 状態が kBlindLandingTimeoutUs の間 LANDING であり、「かつ」その中で ToF が
/// この時間連続して出力停止しているときだけ、時間で着陸完了とする。候補値。根拠: ToF が動作している LANDING は kBlindLandingTimeoutUs より長く続くことが
/// ある（ゆっくりした降下、地面効果）。その後に ToF が停止すると、上限だけでは空中で即座にモータを止めてしまう。
/// 5.5 s: ToF なしの降下を、出力停止から最初の接触まで測った最長は 4.7 s（SILS の 972 本の ToF なしの評価表、推力の上限を支持推力で抑えない降下。
/// 場合別: 0.5 m ホバー 1.0〜2.6 s、1.2 m ホバー 1.4〜2.8 s、API 離陸 0.5〜2.4 s、自動離陸 0.5〜3.6 s、ALT_HOLD のスティック上昇 1.2〜4.7 s。最長はノイズ n0 で加速度計のオフセットがある場合）。
/// 推力の上限を支持推力で抑える降下では最長 4.0 s。よって 5.5 s は全てを 0.8 s の余裕（抑える降下では 1.5 s）で覆い、後半の低下が推力を
/// ゼロにする 8.33 s より手前: 上限に達した時点で推力が下がり始めて 0.5 s（85 %まで）しか経っていない（後半の低下の開始は kBlindLandingLateStartS = 5 s）。4 s では、最長の降下で
/// 接触の最大 0.7 s 前にモータを切ってしまう。
inline constexpr uint32_t kBlindLandingMinSilentUs = 5500000;   // 5.5 s

// ---------------------------------------------------------------------------------------------------------------------
// Floor-relative barometric height (barometric_height_estimator.hpp). Observation only: nothing in the control path reads it.
// 床基準の気圧高度（barometric_height_estimator.hpp）。観測専用: 制御の経路は何も読まない。
// ---------------------------------------------------------------------------------------------------------------------

/// Time constant of the floor-zero average, tracked while the craft is disarmed on the floor [s]. CANDIDATE. Basis: 17 real logs (analysis/scripts/baro_offset_study.py):
/// the BMP280 pressure altitude on the floor has a noise of 0.04-0.2 m rms (about 13 samples/s) and a drift of up to 0.04 m/s in the first minute after the power-on;
/// 2 s averages about 26 samples (noise 1-4 cm) and lags a drift of 0.04 m/s by 8 cm at most, and the zero is replaced by the learned offset in flight anyway.
/// 床の上で DISARM の間に追跡する、床の基準（ゼロ）の平均の時定数 [s]。候補値。根拠: 実機ログ 17 本（analysis/scripts/baro_offset_study.py）: 床の上の BMP280 の気圧高度は
/// ノイズが rms 0.04〜0.2 m（約 13 サンプル/s）、電源投入後の最初の 1 分は最大 0.04 m/s のドリフト。2 s は約 26 サンプルの平均（ノイズ 1〜4 cm）で、0.04 m/s のドリフトに対する遅れは高々 8 cm。しかもゼロは飛行中に学習したオフセットで置き換わる。
inline constexpr float kBaroZeroTauS = 2.0f;
/// Time constant of the learned offset (barometric altitude above the floor zero minus the ToF height) [s]. CANDIDATE. Basis: see the table of analysis/scripts/baro_offset_study.py (pooled error after the ToF goes silent).
/// 学習するオフセット（床のゼロからの気圧高度 − ToF の高さ）の時定数 [s]。候補値。根拠: analysis/scripts/baro_offset_study.py の表（ToF 出力停止後の誤差、全ログ合算）。
inline constexpr float kBaroLearnTauS = 5.0f;
/// The offset is learned only while the ToF height is inside this window [m]. CANDIDATE. Basis: below 0.15 m is the ground/air threshold of the takeoff/landing manager (ground effect, the sensor's dead zone); 1.5 m is below the 1.6 m ToF range (95-99 % valid up to 1.7 m).
/// ToF の高さがこの範囲にある間だけオフセットを学習する [m]。候補値。根拠: 0.15 m 未満は離着陸マネージャの接地/空中のしきい値（地面効果、センサの不感帯）、1.5 m は ToF の有効距離 1.6 m 未満（1.7 m まで 95〜99 % が有効）。
inline constexpr float kBaroLearnMinHeightM = 0.15f;
inline constexpr float kBaroLearnMaxHeightM = 1.5f;
/// The offset is learned only while the tilt of the craft is below this [rad]. CANDIDATE. Basis: the beam of the downward ToF leaves the floor beyond about 30 degrees (the field of view is 27 degrees) and the tilt-corrected height is no longer a height; 0.52 rad = 30 degrees.
/// 機体の傾きがこれ未満の間だけオフセットを学習する [rad]。候補値。根拠: 約 30 度を超えると下向き ToF のビームが床から外れ（視野 27 度）、傾き補正した高さがもう高さでなくなる。0.52 rad = 30 度。
inline constexpr float kBaroLearnMaxTiltRad = 0.52f;
/// A sample whose error (barometric height - ToF height - offset) exceeds this is not learned [m]. CANDIDATE. Basis: the error of the in-flight barometric height minus the ToF height was 1.5 m at most in 17 real logs, mostly 0.5-1 m; 2 m only removes a ToF that jumps (a reflection).
/// 誤差（気圧高度 − ToF の高さ − オフセット）がこれを超えるサンプルは学習しない [m]。候補値。根拠: 飛行中の気圧高度 − ToF の高さの誤差は実機ログ 17 本で最大 1.5 m、多くは 0.5〜1 m。2 m は、跳ぶ ToF（反射）だけを除く。
inline constexpr float kBaroLearnGateM = 2.0f;
/// The newest ToF sample must be no older than this to be paired with a barometer sample [us]. CANDIDATE. Basis: the ToF runs at about 30 Hz (33 ms period); 0.1 s allows two lost samples.
/// 気圧計のサンプルと組にする ToF の最新サンプルは、これより古くてはならない [us]。候補値。根拠: ToF は約 30 Hz（周期 33 ms）。0.1 s は 2 サンプルの欠落を許す。
inline constexpr uint32_t kBaroLearnMaxTofAgeUs = 100000;

// ---------------------------------------------------------------------------------------------------------------------
// Thrust excitation (thrust_excitation.hpp): a small signal added to the total thrust while the altitude loop stays closed,
// to measure the dead time and lag from thrust to vertical acceleration, the lag of the downward ToF and the accelerometer
// noise in flight. Every value is a CANDIDATE (not yet flown); the basis is written next to it. The waveform tables
// (frequencies, phases, ramp lengths) are generated from protocol/spec/thrust_excitation.yaml into thrust_excitation_table.hpp.
// 推力励振（thrust_excitation.hpp）: 高さのループを閉じたまま総推力へ小さな信号を足し、推力 → 鉛直の加速度のむだ時間と遅れ、
// 下向き ToF の遅れ、飛行中の加速度計の雑音を測る。値は全て候補値（まだ飛ばしていない）。根拠は値の横に書く。波形の表（周波数・
// 位相・立ち上げの長さ）は protocol/spec/thrust_excitation.yaml から thrust_excitation_table.hpp に生成する。
// @design detailed_design.md §3 — thrust excitation [OK]
// ---------------------------------------------------------------------------------------------------------------------

/// Largest amplitude of a thrust excitation, in percent of the hover thrust [%]. A larger request is rejected, not clipped (a
/// mistyped number must be noticed). CANDIDATE. Basis: 10 % of the hover thrust (about 0.4 N) is 0.04 N, a quarter of the velocity-loop correction limit
/// (0.15 N) and about 0.1 g of acceleration; the duty margin at 3.6 V (hover duty 0.73-0.79) covers it.
/// 推力励振の振幅の上限（ホバー推力に対する割合 [%]）。超える要求は切り詰めずに拒否する（打ち間違いに気づくため）。候補値。根拠: ホバー推力（約 0.4 N）の
/// 10 % は 0.04 N で、速度ループの補正の上限（0.15 N）の 4 分の 1、加速度では約 0.1 g。3.6 V のホバー duty（0.73〜0.79）の余裕で収まる。
inline constexpr float kThrustExciteAmplitudeMaxPercent = 10.0f;
/// Shortest and longest nominal length of a thrust excitation [s]. CANDIDATE. Basis: the longest is 3 periods of the 10 s multisine (the frequency resolution
/// that the 20 s record of the 0.4 Hz tone needs); the shortest holds the two 0.5 s ramps of the multisine and one second of signal.
/// 推力励振の公称の長さの下限・上限 [s]。候補値。根拠: 上限は 10 s の多重正弦波の 3 周期（0.4 Hz の音に必要な周波数分解能の 20 s 以上の記録）。下限は多重正弦波の
/// 0.5 s の立ち上げ・立ち下げ 2 つと 1 s の信号が入る長さ。
inline constexpr float kThrustExciteDurationMinS = 2.0f;
inline constexpr float kThrustExciteDurationMaxS = 30.0f;
/// Upper bound of (amplitude [%] x half period [s]) of a step [%.s], and the shortest half period [s]. CANDIDATE. Basis: a rough closed-loop
/// calculation (position PI 0.45, velocity PI 0.1, motor lag 16 ms) moves the height by 0.052 m for +-5 % held 0.5 s, 0.114 m for 1.0 s and 0.160 m for 1.5 s;
/// 4.0 %.s (5 % for 0.8 s) stays below the 0.15 m abort line with the 0.06-0.08 m of hover wander. A half period below 0.2 s is not a step response of this loop.
/// 階段の（振幅 [%] × 半周期 [s]）の上限 [%·s] と、最短の半周期 [s]。候補値。根拠: 粗い閉ループの試算（位置 PI 0.45・速度 PI 0.1・モータの遅れ 16 ms）で、
/// ±5 % を 0.5 s 保持すると高さが 0.052 m、1.0 s で 0.114 m、1.5 s で 0.160 m 動く。4.0 %·s（5 % なら 0.8 s）なら、ホバーの揺れ 0.06〜0.08 m を足しても
/// 打ち切りの 0.15 m に収まる。0.2 s 未満の半周期は、このループの段応答にならない。
inline constexpr float kThrustExciteStepAmplitudeTimeMaxPercentSeconds = 4.0f;
inline constexpr float kThrustExciteStepHalfPeriodMinS = 0.2f;
/// Height window to START a thrust excitation [m] and to keep running [m]. CANDIDATE. Basis: the standard takeoff holds 0.5 m and the measured hold wanders 0.42-0.64 m,
/// so the start window opens at 0.35 m; the run window opens 0.1 m lower (0.25 m: clear of the ToF near-range dead zone and the ground effect). The top (0.9 m start,
/// 1.05 m run) stays 0.2 m below the start of the ceiling brake (about 1.25 m = kAltitudeCeilingM + kAltitudeCeilingBrakeStartM) so the brake is never engaged by the excitation.
/// 推力励振を始められる高さの窓 [m] と、実行中の窓 [m]。候補値。根拠: 標準の離陸は 0.5 m で、保持の実測は 0.42〜0.64 m の揺れなので、開始の窓は 0.35 m から。実行中の窓は
/// それより 0.1 m 広い（0.25 m: ToF の近距離の不感帯と地面効果から離す）。上端（開始 0.9 m、実行中 1.05 m）は、天井ブレーキの始まり（約 1.25 m =
/// kAltitudeCeilingM + kAltitudeCeilingBrakeStartM）から 0.2 m 以上下で、励振がブレーキを働かせない。
inline constexpr float kThrustExciteStartHeightMinM = 0.35f;
inline constexpr float kThrustExciteStartHeightMaxM = 0.9f;
inline constexpr float kThrustExciteRunHeightMinM   = 0.25f;
inline constexpr float kThrustExciteRunHeightMaxM   = 1.05f;
/// Largest distance of the height from its target while running [m]; beyond it the run is aborted. CANDIDATE. Basis: the hover itself wanders 0.06-0.08 m (peak to peak)
/// and a 10 % multisine moves the height by up to 0.033 m in a rough closed-loop calculation; 0.15 m is the same line as the ground/air margin of the takeoff manager.
/// 実行中の、高さの目標からのずれの上限 [m]。超えたら打ち切る。候補値。根拠: ホバー自体が 0.06〜0.08 m（ピーク間）揺れ、10 % の多重正弦波は粗い閉ループの試算で最大 0.033 m 動かす。
/// 0.15 m は離着陸マネージャの接地/空中の余裕と同じ線。
inline constexpr float kThrustExciteAltitudeErrorAbortM = 0.15f;
/// Battery voltage needed to START [V], and the voltage below which a running excitation is aborted [V]. CANDIDATE. Basis: the start value keeps the duty margin for the extra
/// 10 % (hover duty at 3.6 V is already 0.73-0.79); the abort value is 0.2 V above the 3.4 V warning (safety.battery.low_v) so the run ends before the warning, and the emergency
/// (3.0 V, landing in every mode) is far away.
/// 開始に要る電池の電圧 [V] と、実行中にこれを下回ったら打ち切る電圧 [V]。候補値。根拠: 開始の値は、増える 10 % の分の duty の余裕を保つ（3.6 V のホバー duty は既に 0.73〜0.79）。
/// 打ち切りの値は、3.4 V の警告（safety.battery.low_v）より 0.2 V 上で、警告の前に終わる。緊急（3.0 V、全モードで着陸）はずっと先。
inline constexpr float kThrustExciteBatteryStartMinV = 3.8f;
inline constexpr float kThrustExciteBatteryAbortV    = 3.6f;
/// Time the battery voltage must stay below kThrustExciteBatteryAbortV before the abort [s]. CANDIDATE. Basis: the INA3221 reading updates only about every 0.4 s
/// (averaging of 64 samples), so one low reading is one update; 0.5 s needs two.
/// 電池の電圧が kThrustExciteBatteryAbortV を下回り続けてから打ち切るまでの時間 [s]。候補値。根拠: INA3221 の値は約 0.4 s ごとにしか更新されない（64 標本の平均）ので、
/// 低い読みは 1 回の更新。0.5 s なら 2 回要る。
inline constexpr float kThrustExciteBatteryAbortHoldS = 0.5f;
/// Tilt [rad] allowed to START and above which a running excitation is aborted. CANDIDATE. Basis: the abort value 0.2 rad (11.5 degrees) lies just beyond the POS_HOLD tilt limit
/// (0.1745 rad = 10 degrees, also the pass line of the rate-excitation scenarios), which only a disturbance can exceed; the start value is half of it.
/// 開始で許す傾き [rad] と、実行中にこれを超えたら打ち切る傾き [rad]。候補値。根拠: 打ち切りの 0.2 rad（11.5 度）は、POS_HOLD の傾きの上限（0.1745 rad = 10 度。レート励振の
/// シナリオの合格線でもある）の少し外で、外乱でしか届かない値。開始の値はその半分。
inline constexpr float kThrustExciteStartTiltMaxRad = 0.1f;
inline constexpr float kThrustExciteTiltAbortRad    = 0.2f;
/// Conditions to leave the wait (Standby) and begin: altitude error [m], vertical speed [m/s] and the time all hold [s]; and the wait that is given up [s]. CANDIDATE.
/// Basis: the hover wander is 0.06-0.08 m peak to peak, so an error of 0.05 m and a speed of 0.1 m/s are the quiet part of it; 0.5 s of quiet is ten ToF samples; 3 s is
/// about six of the 0.5 s windows.
/// 待機（Standby）を終えて始める条件: 高さのずれ [m]・鉛直速度 [m/s]・全てが続く時間 [s]。待機を諦める時間 [s]。候補値。根拠: ホバーの揺れは 0.06〜0.08 m（ピーク間）なので、
/// ずれ 0.05 m・速度 0.1 m/s はその静かな部分。静かな 0.5 s は ToF の 10 標本。3 s は 0.5 s の窓が 6 回。
inline constexpr float kThrustExciteStartAltitudeErrorMaxM = 0.05f;
inline constexpr float kThrustExciteStartVerticalSpeedMaxMps = 0.1f;
inline constexpr float kThrustExciteStartSettleHoldS = 0.5f;
inline constexpr float kThrustExciteStartSettleTimeoutS = 3.0f;
/// Time the ToF must be invalid or silent before a running excitation is aborted [s]. CANDIDATE. Basis: the ToF samples at about 28.5 Hz (35 ms), so 0.1 s is about three
/// samples: one lost sample does not abort, three do.
/// 実行中の励振を打ち切るまでに ToF が無効・出力停止である時間 [s]。候補値。根拠: ToF は約 28.5 Hz（35 ms）なので、0.1 s は約 3 サンプル。1 つ抜けても打ち切らず、3 つで打ち切る。
inline constexpr float kThrustExciteToFLostAbortS = 0.1f;
/// Time after the end of an excitation during which the hover-thrust learner and the support-thrust window stay frozen [s]. CANDIDATE. Basis: the learning time constant is
/// 20 s, but the response of the height to the last tone decays within about 1 s (closed-loop settling of the 0.4 Hz tone).
/// 励振が終わってから、ホバー推力の学習と支持推力の窓を止めたままにする時間 [s]。候補値。根拠: 学習の時定数は 20 s だが、最後の音に対する高さの応答は約 1 s で減衰する
/// （0.4 Hz の音の閉ループの整定）。
inline constexpr float kThrustExciteLearningHoldOffS = 1.0f;
/// Largest speed [m/s] of the horizontal motion that still counts as a still guidance when the excitation starts. CANDIDATE. Basis: the position hold wanders by a few cm
/// with speeds below 0.1 m/s (the POS_HOLD hold accuracy is +-6-7 cm); a guidance that is moving the craft is faster.
/// 励振の開始のとき、誘導が静止中とみなす水平の速さの上限 [m/s]。候補値。根拠: 位置保持は 0.1 m/s 未満の速さで数 cm 揺れる（POS_HOLD の保持精度は ±6〜7 cm）。
/// 誘導が機体を動かしているなら、これより速い。
inline constexpr float kThrustExciteStartHorizontalSpeedMaxMps = 0.1f;
/// Time that an API command waits for the controller to accept or reject a thrust excitation [ms]. CANDIDATE. Basis: ControlTask consumes the queue every control cycle (2.5 ms)
/// and publishes the status in the same cycle; 500 ms is two orders of magnitude more, and shorter than the 1 s margin of the other waits.
/// API の命令が、制御器が推力励振を受理・拒否するのを待つ時間 [ms]。候補値。根拠: ControlTask は制御周期（2.5 ms）ごとにキューを取り出し、同じ周期で状態を発行する。500 ms は
/// その 2 桁上で、他の待機の余裕 1 s より短い。
inline constexpr uint32_t kThrustExciteAckTimeoutMs = 500;
static_assert(kThrustExciteRunHeightMaxM < kAltitudeCeilingM + kAltitudeCeilingBrakeStartM,
              "the excitation must stay below the start of the ceiling brake");
static_assert(kThrustExciteStartHeightMaxM < kThrustExciteRunHeightMaxM &&
              kThrustExciteRunHeightMinM < kThrustExciteStartHeightMinM,
              "the start window lies inside the run window");
static_assert(kThrustExciteBatteryAbortV < kThrustExciteBatteryStartMinV,
              "the abort voltage lies below the start voltage");

}  // namespace limits
}  // namespace sf
