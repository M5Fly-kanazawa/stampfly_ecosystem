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

namespace sf {
namespace limits {

/// Altitude ceiling for the auto-takeoff target, for API altitude inputs and, since 2026-11-09, for the ALT_HOLD /
/// POS_HOLD stick climb and the rc velocity guidance in the controller (altitudeHoldClimbRate()) [m].
/// CANDIDATE. Basis: 2026-11 accident (auto-takeoff ran into the ceiling of the room)
/// and the ToF's valid range of ~1.6 m (flight logs: 95-99 % valid up to 1.7 m, falling
/// above). 1.2 m keeps the craft inside the range where ToF altitude can be trusted and
/// bounds the blind-landing time below (1.2 m / 0.3 m/s = 4 s). Was 2.0 m in api_task.
/// 自動離陸の目標、API の高度入力、および 2026-11-09 からは制御器の ALT_HOLD・POS_HOLD のスティック上昇と rc 速度誘導
/// （altitudeHoldClimbRate()）の上限 [m]。候補値。根拠: 2026-11 の事故（自動離陸で
/// 部屋の天井に衝突）と ToF の有効距離 約 1.6 m（飛行ログで 1.7 m まで 95-99 % が有効、
/// それ以上で低下）。1.2 m なら ToF 高度が信頼できる範囲に収まり、下の ToF なし着陸の時間も
/// 抑えられる（1.2 m ÷ 0.3 m/s = 4 s）。api_task では従来 2.0 m だった。
inline constexpr float kAltitudeCeilingM = 1.2f;
/// Margin above kAltitudeCeilingM [m] beyond which the controller forbids any climb command and brings the target back
/// to the ceiling (auto-takeoff, ALT_HOLD, POS_HOLD). CANDIDATE. Basis: the ESKF altitude error with a working ToF is a
/// few cm (innovation gate 0.5 m, ToF noise 0.02 m), so 0.15 m is the same as the ground/air threshold of the
/// takeoff/landing manager: beyond it the altitude is not a measurement error but a real overshoot.
/// kAltitudeCeilingM を超える余裕 [m]。これを超えたら制御器は上昇の指令を一切禁止し、目標を上限へ戻す（自動離陸・
/// ALT_HOLD・POS_HOLD）。候補値。根拠: ToF が生きているときの ESKF の高度誤差は数 cm（イノベーション判定 0.5 m、
/// ToF ノイズ 0.02 m）なので、離着陸マネージャの接地/空中のしきい値と同じ 0.15 m を超えたら、測定誤差でなく本当の行き過ぎ。
inline constexpr float kAltitudeCeilingMarginM = 0.15f;
/// Above kAltitudeCeilingM + kAltitudeCeilingMarginM, while the craft is still rising, the thrust is at most this fraction of the hover-thrust
/// ESTIMATE [-] (a brake that does not wait for the velocity loop, whose correction is limited). CANDIDATE. Basis: SILS tof_silent_est_high with
/// noise n2 (the estimate is 1.35 x the true hover): the auto-takeoff reaches 1.5 m/s and the velocity loop's thrust bottoms at 0.89 x the weight, so
/// the craft went on to 1.69 m (ceiling 1.2 m, ToF range 1.6 m); 0.5 x the estimate is below the weight whatever the estimate error up to +100 %.
/// kAltitudeCeilingM + kAltitudeCeilingMarginM を超えて、まだ上昇中の間は、推力は hover 推力の「見積もり」のこの割合 [-] 以下
/// （補正が制限されている速度ループを待たないブレーキ）。候補値。根拠: SILS の tof_silent_est_high のノイズ n2（見積もりが真のホバーの
/// 1.35 倍）では、自動離陸が 1.5 m/s に達し、速度ループの推力は重さの 0.89 倍で底を打ち、機体は 1.69 m まで行った（上限 1.2 m、ToF の
/// 有効距離 1.6 m）。見積もりの 0.5 倍は、見積もり誤差が +100 % までなら重さを下回る。
inline constexpr float kAltitudeCeilingBrakeFrac = 0.5f;

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
/// by the accelerometer integral [m], or after kTofInvalidTakeoffMaxUs from the start of TAKEOFF, whichever
/// comes first; then kTofInvalidTakeoffPersistUs of them (not kTofInvalidPersistUs) make the verdict. CANDIDATES.
/// Basis: (1) the real sensor returns its first valid sample at 0.031-0.032 m (3 real logs: 186 ms and 344 ms after
/// the TAKEOFF entry); in SILS (dead zone 31 mm, rest at 21 mm) the first valid sample comes when the estimated rise
/// is 0.011-0.015 m (395 ms after the entry), so 0.05 m is more than three times the dead-zone exit, and the
/// accelerometer integral errs by (offset error) x t^2 / 2 = 0.1 m/s^2 x (0.5 s)^2 / 2 = 1 cm in the first 0.5 s.
/// (2) What the persistence costs: with a ToF that stays invalid on the floor (the pinned estimator keeps the
/// auto-takeoff at full climb) the craft rises until the verdict, and then the blind descent has to stop the climb.
/// SILS, tof_invalid_alt_hover (ARM with every sample invalid), peak truth height after the ARM, noise off / n2 seeds 1-3 /
/// hover.thrust_corr 0.9 / 1.2: persistence 0.5 s: 0.39 / 0.99, 1.06, 1.24 / 0.26 / 0.91 m (the 1.2 m ceiling is
/// reached); 0.3 s: 0.26 / 0.55-0.74 / 0.18 / 0.55 m; 0.15 s (chosen): 0.20 / 0.38-0.41 / 0.15 / 0.28 m; 0.1 s: 0.16 /
/// 0.25-0.38 / 0.12 / 0.28 m. (The previous rule, counting 0.5 s from the TAKEOFF entry: about 0.16 m, but it cut a slow
/// takeoff short.) 0.15 s is five samples at 30 Hz; the real logs show no invalid sample in the air after the first valid
/// one except 34 ms in a FLIP. Rise threshold (with 0.15 s): 0.02 / 0.03 / 0.05 m give a peak of 0.12 / 0.16 / 0.20 m
/// noise off and 0.25-0.38 / 0.25-0.38 / 0.38-0.41 m with n2: a lower threshold stops the climb up to 0.13 m earlier, but
/// 0.05 m (the owner's example) keeps the margin over the 1 cm error of the accelerometer integral.
/// TAKEOFF では、加速度積分の推定でこの高さ [m] だけ上昇した後、または TAKEOFF の開始から kTofInvalidTakeoffMaxUs 経った
/// 後のどちらか早い方から、無効サンプルを継続判定に数える。判定は kTofInvalidPersistUs でなく kTofInvalidTakeoffPersistUs
/// 続いたとき。候補値。根拠: (1) 実機のセンサが最初の有効サンプルを返すのは 0.031〜0.032 m（実機ログ 3 本。TAKEOFF 突入の
/// 186 ms 後と 344 ms 後）。SILS（不感帯 31 mm、静止 21 mm）では、最初の有効サンプルは上昇量の推定が 0.011〜0.015 m
/// のとき（突入の 395 ms 後）に来るので、0.05 m は不感帯を出る量の 3 倍以上。加速度積分の誤差は（オフセット誤差）× t² / 2
/// = 0.1 m/s² × (0.5 s)² / 2 = 1 cm（最初の 0.5 s）。(2) 継続時間の代償: ToF が床の上で無効のままだと（固定された推定が
/// 自動離陸を全速の上昇のままにする）、判定までは機体が上がり、その後 ToF なしの降下が上昇を止める。SILS の
/// tof_invalid_alt_hover（全サンプルが無効のまま ARM）、ARM 後の真値の最大高度、ノイズなし / n2 の seed 1〜3 /
/// hover.thrust_corr 0.9 / 1.2: 継続 0.5 s: 0.39 / 0.99、1.06、1.24 / 0.26 / 0.91 m（1.2 m の上限に届く）、0.3 s: 0.26 /
/// 0.55〜0.74 / 0.18 / 0.55 m、0.15 s（採用）: 0.20 / 0.38〜0.41 / 0.15 / 0.28 m、0.1 s: 0.16 / 0.25〜0.38 / 0.12 /
/// 0.28 m。（以前の規則: TAKEOFF 突入から 0.5 s 数える。約 0.16 m だが、離陸の遅い機体を打ち切った。）0.15 s は 30 Hz で
/// 5 サンプル。実機ログには、最初の有効サンプルの後に空中で無効サンプルが出る例が FLIP 中の 34 ms の 1 回しかない。
/// 上昇量の閾値（継続 0.15 s）: 0.02 / 0.03 / 0.05 m で最大高度はノイズなしで 0.12 / 0.16 / 0.20 m、n2 で 0.25〜0.38 /
/// 0.25〜0.38 / 0.38〜0.41 m: 閾値が低いと上昇を最大 0.13 m 早く止められるが、0.05 m（持ち主の例示）は加速度積分の
/// 誤差 1 cm に対する余裕を保つ。
inline constexpr float    kTofInvalidTakeoffRiseM  = 0.05f;      // [m]
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
/// ToF の「不感帯」での接地（下向き ToF は生きているが、約 0.031 m より近いと「目標なし」を返すので、着陸の最後の数 cm
/// は無効サンプルになる）: LANDING の降下中に、kLandingDeadZoneBandM 以下の有効な読みの後、無効サンプルだけが
/// kLandingDeadZoneHoldMs 続いたら、機体は床の上にあるとする。候補値。根拠: 実機の唯一の着陸（
/// front_tof_poshold_yaw360、0.28 m/s）は 0.039 m まで有効、さらに 0.031 m が 1 回有効で、以後ずっと無効。最後の有効な
/// 読みより前に、帯の中で有効な読みの間に挟まった途切れは無効 1 サンプル（有効な読みの間隔 66 ms）だけだったので、
/// 0.2 s（33 Hz で 6 サンプル）は最長の途切れの約 3 倍。0.28 m/s ではその間に 0.06 m 沈み、残りの高さ（0.03〜0.04 m）を超える。帯は
/// kTofInvalidLandingMinHeightM と同じ値: それより下では無効サンプルが正常で（tof_health_tracker.hpp）、上では異常。
inline constexpr float    kLandingDeadZoneBandM  = kTofInvalidLandingMinHeightM;   // [m]
inline constexpr uint32_t kLandingDeadZoneHoldMs = 200;                            // 0.2 s

/// failsafe_status is published by Failsafe every 100 ms. A reader that finds it older than this
/// treats the ToF as silent: a status nobody refreshes (Failsafe dead / PowerTask stalled) must
/// never leave the last "not silent" in force (every ToF protection reads that flag).
/// CANDIDATE. Basis: 10 publish periods (a few missed ticks are jitter), and short enough that a stalled Failsafe is noticed before a blind climb gets far.
/// failsafe_status は Failsafe が 100 ms ごとに発行する。これより古いと読んだ側は ToF を出力停止と
/// 扱う: 誰も更新しない状態（Failsafe の停止・PowerTask の停滞）で最後の「出力停止でない」が効き続けて
/// はならない（ToF の保護は全部このフラグを読む）。候補値。根拠: 発行周期の 10 倍（数回の取りこぼしはふらつき）で、Failsafe が停滞しても、ToF なしの上昇が進む前に気付ける短さ。
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
///   thrust >= kBlindDescentThrustMinFrac x support thrust (the floor, as before).
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
///   推力 ≧ kBlindDescentThrustMinFrac × 支持推力（下限。従来どおり）。
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
///   - この比の SILS 掃引（0.5 m・1.2 m のホバー、ALT_HOLD の上昇、上昇初期。沈黙時に −0.2 / 0 / +0.2 m/s^2 を注入。ノイズ n0・n2、3 seed）:
///     降下が始まった後の上昇は 0.95 と 0.96 で無し。ノイズなし・+0.2 m/s^2 を離陸から注入・ALT_HOLD の上昇の組では、0.97 が 0.07 m、0.98 が 0.54 m
///     再上昇した。1.00 では 1.2 m のホバー + 0.2 m/s^2 が 0.05 m 再上昇した（上限が重さ + オフセット誤差に等しい）。ノイズ n2 の 1.2 m からの接地速度は
///     比とともに下がる（0.95: 1.17 m/s、0.97: 1.02、0.98: 0.93、0.99: 0.84、1.00: 0.72）: 0.95（採用）は上の 2 つの実測誤差（窓の ±1 %、バイアス
///     0.5〜0.9 %）を合わせた値の約 3 倍の余裕を保ち、0.99 に比べて接地速度が約 0.3 m/s 高くなる。
inline constexpr float kNoTofThrustCapRatio   = 0.95f;   // [-]   thrust ceiling = ratio x support thrust (< 1: cannot climb) / 推力の上限 = 比 × 支持推力（1 未満: 上昇できない）
/// Support thrust without a calibration window, as a fraction of the hover-thrust ESTIMATE: at most this [-]. CANDIDATE. Basis: the
/// ceiling is then 0.95 x 0.75 = 0.71 x the estimate, below the weight while the estimate is below 1 / 0.71 = 1.40 x the true hover
/// thrust (SILS with `hover.thrust_corr` 1.2: the estimate is 1.29 x the true one; the first choice 0.9 climbed 2.6-6 m there). The price is
/// a descent that is faster when the estimate is right: 0.95 x 0.75 = 0.71 of the weight, 2.8 m/s^2 downward.
/// 較正の窓が無いときの支持推力。hover 推力の「見積もり」に対する割合の上限 [-]。候補値。根拠: 上限は 0.95 × 0.75 = 見積もりの
/// 0.71 倍で、見積もりが真のホバー推力の 1 / 0.71 = 1.40 倍未満の間は重さを下回る（SILS の `hover.thrust_corr` 1.2 では見積もりが
/// 真値の 1.29 倍。最初に選んだ 0.9 ではそこで 2.6〜6 m 上昇した）。代償は、見積もりが正しいときの降下が速いこと: 重さの 0.95 × 0.75 =
/// 0.71、下向き 2.8 m/s^2。
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
/// reading over ALL the airborne samples, x kSupportClimbDerate, once kSupportClimbMinS of them are there; the support thrust is the
/// smaller of that and the lowered estimate. CANDIDATE. Basis: in a climb of +0.2 g the quotient is 11-12 % above the hover thrust in SILS
/// (n0 and n2; the delivered thrust is concave in the command), 1 % in the one real take-off log (yaw360, +0.03 g); 0.85 puts the result
/// 2-5 % below the truth in SILS, so the thrust ceiling stays below the weight. Without it the lowered estimate alone (0.71 x the estimate) climbed
/// 4-8 m when the estimate was 1.45 x the true hover (SILS n2, `hover.thrust_corr` 1.2).
/// ホバーを飛ぶ前（沈黙が離陸の上昇中に来る）の支持推力の早い推定: 空中の「全」サンプルでの商（推力 × g ÷ 読み）× kSupportClimbDerate を、
/// それが kSupportClimbMinS 分たまったら使い、支持推力は、これと低くした見積もりの小さいほう。候補値。根拠: +0.2 g の上昇では、
/// 商は SILS でホバー推力の 11〜12 % 高い（n0・n2 とも。出る推力は指令に対し上に凸）、実機の離陸ログ 1 本（yaw360、+0.03 g）では 1 %。
/// 0.85 で結果は SILS の真値の 2〜5 % 下になり、推力の上限は重さを下回る。これが無いと、低くした見積もり（見積もりの 0.71 倍）だけでは、
/// 見積もりが真のホバーの 1.45 倍のとき（SILS の n2、`hover.thrust_corr` 1.2）に 4〜8 m 上昇した。
inline constexpr float kSupportClimbDerate     = 0.85f;  // [-]
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
/// returns and stops again does not restart it). A normal descent from the 1.2 m ceiling at kBlindDescentRateMps ends in
/// about 3.5 s, so this never acts then.
/// ToF なしの降下の最後の安全策（後半の低下）: 速度は積分から来て、その誤差は時間とともに増えるので、この降下が
/// kBlindLandingLateStartS 続いたら、推力に係数を掛ける。係数は kBlindLandingLateRampPerS で下がり、速度推定が
/// 何であれ kBlindLandingLateStartS + 1 / kBlindLandingLateRampPerS = 8.33 s で「ゼロ」になる。時間は 1 回の LANDING
/// の中の ToF なしの降下を通して数える（ToF が戻ってまた止まっても数え直さない）。1.2 m の天井から
/// kBlindDescentRateMps で降りる通常の降下は約 3.5 s で終わるので、そのときはここは働かない。
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
/// 鉛直速度推定の大きさの制限 [m/s]（暴走防止。実際の値よりずっと大きい）。
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
/// 「推力 ÷ 質量」（比力）を読む。空中で推力が重さの 0.7 倍未満なら、読みは 0.7 g 未満（抗力で多少ずれる）で、その推力で 1 g を読むのは、
/// 残りの重さを床が支えているときだけ。誤ラッチには、支持推力が重さを 0.9 g ÷ 0.7 g − 1 = 29 % 超えて高いことが要る。較正した
/// 支持推力は重さの約 2 % 以内（kNoTofThrustCapRatio）、較正なしは見積もりの 0.75 倍以下なので、見積もりが真のホバーの 1.7 倍
/// 要る（SILS の `hover.thrust_corr` 1.2 では 1.29〜1.45 倍）。候補値。
inline constexpr float    kBlindFloorThrustFrac = 0.7f;   // [-]   commanded thrust / support thrust below this / 指令推力 ÷ 支持推力がこれ未満
inline constexpr float    kBlindFloorToleranceG = 0.1f;   // [g]   | low-passed |accel| - 1 g | below this / ろ波した |加速度| が 1 g からこれ未満のずれ
inline constexpr float    kBlindFloorS          = 0.5f;   // [s]   both must hold this long / 両方がこの時間続くこと

/// Longest LANDING time allowed while the downward ToF is silent [us], counted from the moment the state ENTERED
/// LANDING (not from the silence, so a ToF that returns and stops again does not extend it); once it has passed and the
/// ToF is silent the landing is declared complete (motors off). CANDIDATE. Basis: since the thrust of the descent without
/// a ToF is at most kNoTofThrustCapRatio x the support thrust, the craft is always sinking: from the 1.2 m ceiling at
/// the target speed in about 3.5 s, and from 1.2 m under the constant ceiling thrust (0.05 g) in about 3 s; the late
/// fall brings the thrust to ZERO at 8.33 s of that descent, so at 10 s the craft is on the ground or falling
/// (10 s = 8.33 s + 1.67 s, the fall time from 13 m: sqrt(2 * 13 / 9.8) = 1.63 s): cutting the motors then stops nothing
/// that was holding it up.
/// 下向き ToF の出力停止中に LANDING で許す最長時間 [us]。状態が LANDING に「入った時刻」から数える（出力停止の
/// 時刻からではない。ToF が戻ってまた止まっても延びない）。これを過ぎて ToF が出力停止中なら着陸完了（モータ停止）
/// とする。候補値。根拠: ToF なしの降下の推力は高々 kNoTofThrustCapRatio × 支持推力なので、機体は常に沈む:
/// 1.2 m の天井から目標速度で約 3.5 s、上限の一定推力（0.05 g）では約 3 s。後半の低下はこの降下の 8.33 s で
/// 推力を「ゼロ」にするので、10 s では機体は接地しているか落下中（10 s = 8.33 s + 1.67 s。13 m からの
/// 落下時間 sqrt(2 × 13 / 9.8) = 1.63 s）で、そこでモータを切っても機体を支えていたものを止めることにならない。
inline constexpr uint32_t kBlindLandingTimeoutUs = 10000000;   // 10 s

}  // namespace limits
}  // namespace sf
