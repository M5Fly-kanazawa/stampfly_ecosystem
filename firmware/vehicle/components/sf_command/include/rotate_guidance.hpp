/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file rotate_guidance.hpp
 * @brief Guidance yaw target and wait rules of the `cw` / `ccw` commands (pure functions)
 *        `cw` / `ccw` コマンドの誘導ヨー目標と待ち時間の規則（純粋関数）
 *
 * The controller turns toward the guidance yaw target by the SHORTEST path: it wraps the error
 * (target - heading) into [-pi, +pi], so a target 180 degrees or more away is reached the other
 * way round, or not at all (a 360-degree turn has error 0). The API therefore never hands the
 * controller the final heading of a large turn at once. It keeps the target a bounded LEAD ahead of the
 * measured heading, in the commanded direction, and moves it toward the final heading as the
 * craft turns; the error the controller sees stays below 180 degrees for every angle, and
 * a turn of at most the lead is the single step it always was. All angles go through
 * this one rule (no special case for large angles). Headings here are UNWRAPPED: a running
 * angle that keeps counting past +-pi, so "how far is left to turn" is a plain difference.
 * Pure functions of the numbers only (no ESP-IDF, no FreeRTOS), checked on the host in
 * test/test_rotate_guidance.cpp.
 * 制御器は誘導ヨー目標へ「最短経路」で回る: 誤差（目標 - 向き）を [-pi, +pi] に折り返すため、
 * 180 度以上先の目標は逆回りで到達するか、到達しない（360 度の回頭は誤差 0）。そこで API は
 * 大きな回頭の最終の向きを一度に渡さない。目標を、測った向きから指令の向きへ有限の「先行量」
 * だけ先に置き、機体が回るにつれて最終の向きへ進める。制御器が見る誤差はどの角度でも 180 度
 * 未満に収まり、先行量以下の回頭はこれまでどおり 1 回の設定で済む。全ての角度がこの 1 つの規則を
 * 通る（大角度だけの特別扱いは無い）。ここでの向きは「アンラップ」した値: ±pi を越えて数え続ける
 * 角度で、「あと何度回るか」が単なる差になる。
 *
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI   [OK]
 * @design detailed_design.md §3 注13 — API の引数の範囲と `cw` / `ccw` の回頭  [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "flight_limits.hpp"   // sf::limits::kGuidanceYaw* (shared with the controller)
#include "sf_math.hpp"

namespace sf {
namespace rotate_guidance {

/// Yaw rate limit [rad/s] and P gain [1/s] of the guidance yaw law: the SAME constants the
/// controller uses (flight_limits.hpp; neither can change at run time). The turn takes at least
/// angle / the limit.
/// 誘導ヨー則のヨーレート上限 [rad/s] と P ゲイン [1/s]: 制御器が使うのと「同じ」定数
/// （flight_limits.hpp。どちらも実行時には変わらない）。回頭には少なくとも 角度 / 上限 の時間がかかる。
constexpr float kYawRateLimitRadPerSec = sf::limits::kGuidanceYawRateMaxRadPerSec;
constexpr float kYawGainPerSec         = sf::limits::kGuidanceYawKpPerSec;

/// Largest lead of the guidance target over the measured heading [rad] = 120 degrees.
/// Basis: (1) well below 180 degrees (60 degrees of margin for the estimate and the poll
/// period, over which the heading moves at most 0.05 rad), so the controller's shortest-path
/// direction is always the commanded one; (2) above 90 degrees plus the largest residual
/// the previous turn may leave (kYawReachedToleranceRad), so a turn of up to 90 degrees is
/// handed over as one step, unchanged.
/// 誘導目標の、測った向きに対する先行量の上限 [rad] = 120 度。根拠: (1) 180 度を十分下回る
/// （推定値とポーリング周期のための 60 度の余裕。周期中に向きは高々 0.05 rad しか動かない）ため、
/// 制御器の最短経路の向きは常に指令の向きになる。(2) 90 度に、直前の回頭が残しうる最大の残差
/// （kYawReachedToleranceRad）を足した値より大きく、90 度までの回頭は従来どおり 1 回の設定で渡す。
constexpr float kYawLeadMaxRad = 2.0f * sf::math::kPi / 3.0f;

/// The target is moved forward again when its lead has shrunk to this [rad] = 60 degrees.
/// Basis: above rate limit / P gain = 1.0 / 2.0 = 0.5 rad (the error at which the controller's
/// yaw P law reaches its rate limit), so the craft keeps the full rate limit while the target
/// is moved in a few large steps (6 per full turn) rather than on every poll.
/// 目標の先行量がこの値 [rad] = 60 度まで縮んだら、目標を再び前へ進める。根拠: 速度上限 / P ゲイン
/// = 1.0 / 2.0 = 0.5 rad（制御器のヨー P 則が速度上限に達する誤差）より大きく、機体は目標が
/// 1 周あたり 6 回の大きな段で進む間も全速（速度上限）を保つ。ポーリング毎には動かさない。
constexpr float kYawRefreshLeadRad = sf::math::kPi / 3.0f;

// The refresh lead keeps the full rate limit only while the gain times it reaches the limit; a
// lower gain still turns the right way (the error stays 60-120 degrees, never 180), only slower
// (test_rotate_guidance, parametrised gain), and the timeout margin would then have to be revisited.
// 再設定の先行角が速度上限を保つのは、ゲイン × 先行角 が上限に届く間だけ。ゲインが低くても向きは
// 正しい（誤差は 60〜120 度で 180 度に届かない）が遅くなる（test_rotate_guidance、ゲインを変えた試験）。
// その場合は待ち時間の余裕を見直す必要がある。
static_assert(kYawRefreshLeadRad * kYawGainPerSec >= kYawRateLimitRadPerSec,
              "the refresh lead must keep the yaw P law at its rate limit");

/// The turn is complete when the measured heading is within this [rad] (~10 degrees) of the
/// final heading.
/// 測った向きが最終の向きのこの範囲 [rad]（約 10 度）に入ったら回頭は完了。
constexpr float kYawReachedToleranceRad = 0.17f;

/// Time allowed beyond the turn itself [ms]: the approach to the final heading (the P law's
/// time constant is 1 / 2.0 = 0.5 s) and a yaw response slower than the rate limit when the
/// motors are near their limit. The margin a 90-degree turn (1.6 s at the rate limit) always
/// had under an 8 s timeout, kept for every angle.
/// 回頭そのものに加えて許す時間 [ms]: 最終の向きへの接近（P 則の時定数は 1 / 2.0 = 0.5 s）と、
/// モータが上限に近いときの速度上限より遅いヨー応答。90 度の回頭（速度上限で 1.6 s）が 8 s の
/// タイムアウトのもとで常に持っていた余裕を、全ての角度で保つ。
constexpr uint32_t kRotateMarginMs = 6000;

/// Heading [rad] that continues a running unwrapped heading with a new wrapped measurement:
/// the new value is taken as the nearest equivalent to the running one.
/// アンラップした向きを、折り返された新しい測定値で更新した向き [rad]: 新しい値を、
/// 継続中の値に最も近い等価な角度として採る。
inline float unwrapHeading(float previous_unwrapped_rad, float measured_wrapped_rad)
{
    return previous_unwrapped_rad +
           sf::math::wrapAngleRad(measured_wrapped_rad - previous_unwrapped_rad);
}

/// Final heading [rad, unwrapped around the measured heading] of a turn of `delta_rad` (cw = +)
/// composed on the previous TARGET heading `previous_target_rad` (wrapped, as ApiTask keeps it): the
/// previous target is at most a small residual from the heading, so the turn is counted from where
/// the craft is, without carrying the residual of the estimate into the next command.
/// 直前の「目標」の向き `previous_target_rad`（ApiTask が持つ折り返した値）に `delta_rad`（cw = +）の
/// 回頭を合成した最終の向き [rad、測った向きのまわりでアンラップ]。直前の目標は向きから小さな残差
/// しか離れていないので、回頭は機体のいる向きから数え、推定値の残差を次のコマンドへ持ち越さない。
inline float composeFinalHeading(float measured_heading_rad, float previous_target_rad, float delta_rad)
{
    return measured_heading_rad +
           sf::math::wrapAngleRad(previous_target_rad - measured_heading_rad) + delta_rad;
}

/// Guidance yaw target [rad, unwrapped] for the turn to `final_heading_rad` (see the file
/// comment). Returns the final heading once it is within kYawLeadMaxRad of the measured
/// heading; otherwise a target kYawLeadMaxRad ahead of the heading toward it, kept unchanged
/// (`current_target_rad`) until its lead has shrunk to kYawRefreshLeadRad.
/// `final_heading_rad` までの回頭の誘導ヨー目標 [rad、アンラップ]（ファイル冒頭参照）。
/// 最終の向きが測った向きの kYawLeadMaxRad 以内に入ったら最終の向きを返す。それまでは、
/// 向きから最終の向きへ kYawLeadMaxRad 先の目標を返し、先行量が kYawRefreshLeadRad に縮むまで
/// 目標（`current_target_rad`）を変えない。
inline float nextGuidanceYaw(float final_heading_rad, float measured_heading_rad,
                             float current_target_rad)
{
    const float remaining = final_heading_rad - measured_heading_rad;
    if (std::fabs(remaining) <= kYawLeadMaxRad) {
        return final_heading_rad;
    }
    const float direction = remaining > 0.0f ? 1.0f : -1.0f;
    const float lead = (current_target_rad - measured_heading_rad) * direction;
    if (lead >= kYawRefreshLeadRad) {
        return current_target_rad;
    }
    return measured_heading_rad + direction * kYawLeadMaxRad;
}

/// True when the measured heading is within kYawReachedToleranceRad of the final heading
/// (both unwrapped, so a 360-degree turn is not complete before the craft has gone round).
/// 測った向きが最終の向きの kYawReachedToleranceRad 以内なら true（どちらもアンラップ済みで、
/// 360 度の回頭は機体が 1 周するまで完了しない）。
inline bool isYawTurnComplete(float final_heading_rad, float measured_heading_rad)
{
    return std::fabs(final_heading_rad - measured_heading_rad) < kYawReachedToleranceRad;
}

/// Wait [ms] for a turn of `angle_rad` (magnitude): the time at the yaw rate limit plus
/// kRotateMarginMs.
/// `angle_rad`（大きさ）の回頭の待ち時間 [ms]: ヨーレート上限での所要時間 + kRotateMarginMs。
inline uint32_t rotateTimeoutMs(float angle_rad)
{
    const float turn_ms = std::fabs(angle_rad) / kYawRateLimitRadPerSec * 1000.0f;
    return static_cast<uint32_t>(turn_ms) + kRotateMarginMs;
}

}  // namespace rotate_guidance
}  // namespace sf
