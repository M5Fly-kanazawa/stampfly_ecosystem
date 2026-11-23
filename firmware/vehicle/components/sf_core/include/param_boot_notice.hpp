/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file param_boot_notice.hpp
 * @brief Decisions for the boot-time notices on saved parameters that the defaults have left behind (pure functions):
 *        the yaw-rate derivative time, and the altitude-control gains
 *        既定値に取り残された保存済みパラメータに対する起動時の注意の判定（純粋関数）:
 *        ヨー角速度の微分時間と、高さの制御のゲイン
 *
 * The default of `rate.yaw.td` changed from 0.01 to 0 (2026-11-22), but a craft whose NVS
 * already holds 0.01 keeps it (there is no migration by version). params::init() logs one
 * warning line after loading when this function says so; the value is never rewritten.
 * `rate.yaw.td` の既定値は 0.01 から 0 に変わった（2026-11-22）が、NVS に既に 0.01 を保存した
 * 機体はそのままになる（版による移行の仕組みは無い）。params::init() は読み込みの後、この関数が
 * 真を返したときに警告を 1 行ログに出す。値は書き換えない。
 *
 * The same applies to the altitude-control gains (2026-11-23): see needsAltitudeGainNotice() below.
 * 高さの制御のゲイン（2026-11-23）にも同じ仕組みを使う。下の needsAltitudeGainNotice() を参照。
 *
 * @design detailed_design.md §4 — Yaw rate PID: why it has no derivative  [OK]
 * @design detailed_design.md §4 — Altitude control gains (counterpart of the merge-off ToF)  [OK]
 */

#pragma once

#include <cmath>

namespace sf {
namespace params {

/// Yaw-rate derivative time [s] above which the boot notice is logged. The default is 0 (no
/// derivative term), so any positive value is a leftover or a deliberate setting.
/// 起動時の注意を出すヨー角速度の微分時間 [s] のしきい値。既定値は 0（微分項なし）なので、
/// 正の値は古い保存値か、利用者が意図した設定である。
inline constexpr float kYawRateDerivativeNoticeAboveS = 0.0f;

/// True when `rate.yaw.td` (the value after the NVS load) is finite and above the threshold.
/// A NaN or an infinity never reaches here (the load refuses it), and gives false.
/// NVS 読み込み後の `rate.yaw.td` が有限で、しきい値より大きいとき true。NaN・無限大は
/// 読み込みで拒否されるのでここへ来ず、来ても false を返す。
inline bool needsYawRateDerivativeNotice(float yaw_rate_td_s)
{
    return std::isfinite(yaw_rate_td_s) && yaw_rate_td_s > kYawRateDerivativeNoticeAboveS;
}

// -----------------------------------------------------------------------------
// Altitude-control gains (2026-11-23)
// 高さの制御のゲイン（2026-11-23）
// -----------------------------------------------------------------------------
// The defaults of altitude.alt.kp / altitude.vel.kp / altitude.vel.ti_hover changed from 0.45 / 0.1 / 2.5 to
// 0.85 / 0.12 / 2.0, the counterpart of switching the downward ToF histogram merge off. A value saved in NVS
// overrides the default, and the on-board learning calls params::save() (which writes EVERY parameter) when a
// landing ends on the ground, so a craft that has landed that way holds the gains that were live then. The old set is on the safe side
// (more phase margin against the faster sensor) but is not the intended behaviour. The values are never rewritten.
// altitude.alt.kp / altitude.vel.kp / altitude.vel.ti_hover の既定値は、下向き ToF のヒストグラム併合を切ったことと
// 対で、0.45 / 0.1 / 2.5 から 0.85 / 0.12 / 2.0 に変わった。NVS の保存値は既定値を上書きし、オンボード学習は着陸のたびに
// params::save()（全パラメータを書く）を呼ぶので、そうして着陸した機体はそのときのゲインを保持している。古い組は
// 安全側（速くなったセンサに対して位相余裕が大きい）だが、意図した挙動ではない。値は書き換えない。

/// Superseded defaults (through 2026-11-22). / 置き換えられた旧既定値（2026-11-22 まで）。
inline constexpr float kSupersededAltitudePositionKp = 0.45f;
inline constexpr float kSupersededAltitudeVelocityKp = 0.1f;
inline constexpr float kSupersededAltitudeVelocityTiHoverS = 2.5f;

/// Tolerance for "equals a superseded default". NVS stores the exact bit pattern of a float, so an old default
/// reads back identical; the tolerance only absorbs a different spelling of the same literal.
/// 「旧既定値と等しい」の許容幅。NVS は float のビット列をそのまま保存するので、旧既定値は同じ値で読み戻される。
/// 許容幅は、同じ数値の書き方の違いを吸収するだけ。
inline constexpr float kSupersededMatchToleranceAbs = 1.0e-6f;

/// True when `value` equals `superseded` within the tolerance. / `value` が許容幅の内で `superseded` と等しいとき true。
inline bool equalsSuperseded(float value, float superseded)
{
    return std::isfinite(value) && std::fabs(value - superseded) <= kSupersededMatchToleranceAbs;
}

/// True when at least one of the three altitude gains (the values after the NVS load) still equals its superseded
/// default. "At least one", not "all": the gains are a set, so a half-updated set is also worth a notice.
/// 高さの 3 つのゲイン（NVS 読み込み後の値）のうち 1 つでも、旧既定値と等しいとき true。「全て」でなく「1 つでも」:
/// ゲインは組なので、半端に更新された組にも注意を出す価値がある。
inline bool needsAltitudeGainNotice(float alt_kp, float vel_kp, float vel_ti_hover_s)
{
    return equalsSuperseded(alt_kp, kSupersededAltitudePositionKp) ||
           equalsSuperseded(vel_kp, kSupersededAltitudeVelocityKp) ||
           equalsSuperseded(vel_ti_hover_s, kSupersededAltitudeVelocityTiHoverS);
}

}  // namespace params
}  // namespace sf
