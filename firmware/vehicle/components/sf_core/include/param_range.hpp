/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file param_range.hpp
 * @brief Range check of a float parameter value (pure function)
 *        float パラメータ値の範囲検査（純粋関数）
 *
 * @design detailed_design.md §6 — Parameter system                    [OK]
 */

#pragma once

#include <cmath>

namespace sf {
namespace params {

/// True when `value` may be stored in a float parameter whose range is [min_val, max_val]:
/// the value is finite AND inside the range. A NaN is outside every range, but the comparisons
/// `value < min_val || value > max_val` are both false for it, so a rejection written that way
/// lets it through; this form decides on the finite test first and on the positive range test.
/// `value` を、範囲 [min_val, max_val] の float パラメータに保存してよいとき true:
/// 値が有限で、かつ範囲内。NaN はどの範囲にも入らないが、`value < min_val || value > max_val`
/// の比較は NaN に対して両方とも偽になり、その書き方の拒否は NaN を通してしまう。この形は
/// 有限かどうかを先に判定し、範囲は「入っている」ことを肯定形で判定する。
inline bool isFloatInRange(float value, float min_val, float max_val)
{
    return std::isfinite(value) && value >= min_val && value <= max_val;
}

}  // namespace params
}  // namespace sf
