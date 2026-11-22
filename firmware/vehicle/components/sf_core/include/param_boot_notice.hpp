/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file param_boot_notice.hpp
 * @brief Decision for the boot-time notice on a saved yaw-rate derivative time (pure function)
 *        保存済みのヨー角速度の微分時間に対する起動時の注意の判定（純粋関数）
 *
 * The default of `rate.yaw.td` changed from 0.01 to 0 (2026-11-22), but a craft whose NVS
 * already holds 0.01 keeps it (there is no migration by version). params::init() logs one
 * warning line after loading when this function says so; the value is never rewritten.
 * `rate.yaw.td` の既定値は 0.01 から 0 に変わった（2026-11-22）が、NVS に既に 0.01 を保存した
 * 機体はそのままになる（版による移行の仕組みは無い）。params::init() は読み込みの後、この関数が
 * 真を返したときに警告を 1 行ログに出す。値は書き換えない。
 *
 * @design detailed_design.md §4 — Yaw rate PID: why it has no derivative  [OK]
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

}  // namespace params
}  // namespace sf
