/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file nonfinite_guard.hpp
 * @brief Detection of non-finite numbers (NaN, +/-infinity) along the control path, and the
 *        motor-output guard that turns them into zero
 *        制御経路で有限でない数（NaN・±無限大）を検出する部品と、それをゼロにするモータ出力の保護
 *
 * The limits of the control path are comparisons (`if (x > max) x = max`). A comparison with NaN is
 * false, so NaN passes every limit unchanged; once it enters a PID integrator every later output is
 * NaN. Converting NaN to an integer (the PWM duty) is undefined behaviour in C++. Two parts here:
 *   NonFiniteMonitor  : counts, per control boundary (NonFiniteStage), the cycles in which a value was
 *                       not finite, and remembers the first boundary and time. It only counts: it never
 *                       changes a value and never stops the flight.
 *   guardMotorDuties  : the one place that changes a value. A motor whose command is not finite is
 *                       driven with duty 0; finite duties are not touched (bit for bit).
 *
 * 制御経路の上下限は比較式（`if (x > max) x = max`）である。NaN との比較は常に偽なので、NaN はどの上下限も
 * そのまま通り抜け、PID の積分器に入ると以後の出力がすべて NaN になる。NaN の整数への変換（PWM の duty）は
 * C++ の未定義動作である。ここには 2 つの部品がある:
 *   NonFiniteMonitor  : 制御の境界（NonFiniteStage）ごとに、有限でない値があった周期を数え、最初の境界と
 *                       時刻を覚える。数えるだけで、値を書き換えず、飛行を止めない。
 *   guardMotorDuties  : 値を書き換える唯一の場所。指令が有限でないモータは duty 0 で駆動する。有限な duty には
 *                       触れない（ビット単位で不変）。
 *
 * Pure logic (no FreeRTOS, no ESP-IDF): runs in the host test (test_nonfinite_guard.cpp).
 * Cost per control cycle: one std::isfinite per checked value, 35 in all (see detailed_design.md §8).
 * 純粋なロジック（FreeRTOS・ESP-IDF なし）: ホスト試験（test_nonfinite_guard.cpp）で動く。
 * 1 制御周期あたりの費用: 検査する値 1 個につき std::isfinite 1 回、合計 35 回（detailed_design.md §8）。
 *
 * @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "data_types.hpp"

namespace sf {

/// Bit i of the result is set when values[i] is not finite. At most 32 values.
/// 結果のビット i は values[i] が有限でないときに立つ。値は最大 32 個。
inline uint32_t nonFiniteMask(const float* values, size_t count)
{
    uint32_t mask = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) {
            mask |= (1u << i);
        }
    }
    return mask;
}

/// Number of set bits of a mask (the number of non-finite values it stands for).
/// マスクで立っているビットの数（そのマスクが表す有限でない値の数）。
inline uint32_t countMaskBits(uint32_t mask)
{
    uint32_t bits = 0;
    for (; mask != 0; mask &= (mask - 1)) {
        ++bits;
    }
    return bits;
}

/// Number of values that are not finite.
/// 有限でない値の数。
inline uint32_t countNonFinite(const float* values, size_t count)
{
    return countMaskBits(nonFiniteMask(values, count));
}

/// The display name of a boundary ("none" when stage is kNonFiniteNoStage or out of range).
/// 境界の表示名（kNonFiniteNoStage または範囲外なら "none"）。
inline const char* nonFiniteStageName(uint8_t stage)
{
    switch (static_cast<NonFiniteStage>(stage)) {
        case NonFiniteStage::EstimateInput:   return "estimate input";
        case NonFiniteStage::SetpointInput:   return "setpoint input";
        case NonFiniteStage::ControlReference: return "control reference";
        case NonFiniteStage::ControlOutput:   return "control output";
        case NonFiniteStage::MixerOutput:     return "mixer output";
        case NonFiniteStage::MotorCommand:    return "motor command";
    }
    return "none";
}

/// Counts the control cycles whose values were not finite, per boundary of the control path.
/// 制御経路の境界ごとに、有限でない値があった制御周期を数える。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
class NonFiniteMonitor {
public:
    NonFiniteMonitor() { stats_.first_stage = kNonFiniteNoStage; }

    /// Call once at the start of each ARMED control cycle. now_us is the time stamped on the first
    /// detection. Boundaries are checked only in armed cycles, so the counts are "during ARM".
    /// ARM 中の制御周期の最初に 1 回呼ぶ。now_us は最初の検出に付ける時刻。境界の検査は ARM 中の周期だけなので、
    /// 数え上げは「ARM 中」のもの。
    void beginCycle(uint32_t now_us)
    {
        cycle_us_  = now_us;
        cycle_hit_ = false;
        ++stats_.checked_cycles;
    }

    /// Record the check of one boundary: non_finite_count is how many of its values were not finite.
    /// Call the boundaries in the order of the control path (input to motor) so that the first one
    /// recorded in a cycle is the most upstream. Returns true when something was not finite.
    /// 1 つの境界の検査結果を記録する: non_finite_count はその境界の値のうち有限でなかった数。
    /// 制御経路の順（入力からモータへ）に呼ぶこと。そうすれば 1 周期で最初に記録される境界が最も上流になる。
    /// 有限でない値があれば true を返す。
    bool record(NonFiniteStage stage, uint32_t non_finite_count)
    {
        if (non_finite_count == 0) {
            return false;
        }
        const size_t index = static_cast<size_t>(stage);
        ++stats_.stage_count[index];
        if (!cycle_hit_) {
            cycle_hit_ = true;
            ++stats_.detected_cycles;
            if (stats_.first_stage == kNonFiniteNoStage) {
                stats_.first_stage = static_cast<uint8_t>(stage);
                stats_.first_us    = cycle_us_;
            }
        }
        return true;
    }

    /// True once this cycle has recorded a detection.
    /// この周期で検出を記録していれば true。
    bool detectedThisCycle() const { return cycle_hit_; }

    /// The counts so far. The caller stamps timestamp before publishing.
    /// ここまでの数え上げ。発行の前に呼び出し側が timestamp を入れる。
    const NonFiniteStatus& result() const { return stats_; }

private:
    NonFiniteStatus stats_      = {};
    uint32_t        cycle_us_   = 0;
    bool            cycle_hit_  = false;   // this cycle has recorded a detection / この周期は検出を記録済み
};

/// Number of motors. / モータの数。
static constexpr size_t kMotorCount = 4;

/// The motor-output guard. upstream_mask has bit i set when motor i's value was already seen to be
/// not finite before the limits (a limit turns +infinity into a finite maximum, which would hide it).
/// A motor whose bit is set, or whose final duty is not finite, gets duty 0. Finite duties are left
/// untouched. Returns the mask of the motors that were set to 0.
/// モータ出力の保護。upstream_mask はモータ i の値が上下限の前にすでに有限でないと分かっていればビット i が
/// 立つ（上下限は +無限大を有限の最大値にしてしまい、検出を隠すため）。ビットが立つモータ、または最終の duty が
/// 有限でないモータは duty 0 にする。有限な duty には触れない。0 にしたモータのマスクを返す。
inline uint32_t guardMotorDuties(float duties[kMotorCount], uint32_t upstream_mask)
{
    const uint32_t replaced = upstream_mask | nonFiniteMask(duties, kMotorCount);
    for (size_t i = 0; i < kMotorCount; ++i) {
        if ((replaced >> i) & 1u) {
            duties[i] = 0.0f;
        }
    }
    return replaced;
}

}  // namespace sf
