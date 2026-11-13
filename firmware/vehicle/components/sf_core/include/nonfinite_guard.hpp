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
 * Cost per control cycle: one std::isfinite per checked value, 44 in all (see detailed_design.md §8).
 * 純粋なロジック（FreeRTOS・ESP-IDF なし）: ホスト試験（test_nonfinite_guard.cpp）で動く。
 * 1 制御周期あたりの費用: 検査する値 1 個につき std::isfinite 1 回、合計 44 回（detailed_design.md §8）。
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

/// Check the estimate the controller reads (attitude 4, position 3, velocity 3, angular rate 3 = 13 values) and
/// record it as EstimateInput. Only counts; no value is changed.
/// 制御器が読む推定値（姿勢 4・位置 3・速度 3・角速度 3 = 13 個）を検査し、EstimateInput として記録する。
/// 数えるだけで、値は変えない。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
inline void checkEstimateInput(NonFiniteMonitor& monitor, const StateEstimate& state)
{
    const uint32_t estimate_bad = countNonFinite(state.attitude, 4)
                                + countNonFinite(state.position, 3)
                                + countNonFinite(state.velocity, 3)
                                + countNonFinite(state.angular_rate, 3);
    monitor.record(NonFiniteStage::EstimateInput, estimate_bad);
}

/// Check the targets the controller is given and record them as ONE boundary, SetpointInput: the pilot setpoint
/// (throttle, throttle axis, roll, pitch, yaw = 5 values) and the guidance target of the API / Navigator (position 3,
/// yaw, speed, velocity 3, yaw rate = 9 values). The guidance target is read as it stands in the topic, every armed
/// cycle: a non-finite value that stays there is counted in every cycle it stays. Only counts; no value is changed.
/// 制御器に与える目標を検査し、1 つの境界 SetpointInput として記録する: パイロットの目標値（スロットル・スロットル軸・
/// ロール・ピッチ・ヨー = 5 個）と、API / ナビゲータの誘導目標（位置 3・ヨー・速度・速度指令 3・ヨーレート = 9 個）。
/// 誘導目標はトピックにある値のまま、ARM 中の毎周期読む: 有限でない値が残っている間は、その周期ごとに数える。
/// 数えるだけで、値は変えない。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
inline void checkCommandInputs(NonFiniteMonitor& monitor, const CommandSetpoint& setpoint,
                               const GuidanceTarget& guidance)
{
    const float setpoint_values[] = {setpoint.throttle, setpoint.throttle_axis, setpoint.roll,
                                     setpoint.pitch, setpoint.yaw};
    const float guidance_values[] = {guidance.position[0], guidance.position[1], guidance.position[2],
                                     guidance.yaw, guidance.speed, guidance.vx, guidance.vy, guidance.vz,
                                     guidance.vyaw};
    const uint32_t command_bad = countNonFinite(setpoint_values, sizeof(setpoint_values) / sizeof(float))
                               + countNonFinite(guidance_values, sizeof(guidance_values) / sizeof(float));
    monitor.record(NonFiniteStage::SetpointInput, command_bad);
}

/// Check every input of the controller: the estimate, the pilot setpoint and the guidance target.
/// 制御器の入力すべてを検査する: 推定値・パイロットの目標値・誘導目標。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
inline void checkControllerInputs(NonFiniteMonitor& monitor, const StateEstimate& state,
                                  const CommandSetpoint& setpoint, const GuidanceTarget& guidance)
{
    checkEstimateInput(monitor, state);
    checkCommandInputs(monitor, setpoint, guidance);
}

/// The motor path of a control law that produces motor duties directly (the workshop): record the duties as MixerOutput
/// (the values the law asked for, before any limit), turn every motor whose value is not finite into duty 0, and record the
/// motors set to 0 as MotorCommand. The same two records and the same guard as Actuator::update().
/// モータ duty を直接出す制御則（ワークショップ）のモータ経路: duty を MixerOutput（制御則が求めた、上下限の前の値）として
/// 記録し、有限でない値のモータを duty 0 にし、0 にしたモータを MotorCommand として記録する。Actuator::update() と
/// 同じ 2 つの記録と同じ保護。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
inline void recordAndGuardMotorDuties(NonFiniteMonitor& monitor, float duties[kMotorCount])
{
    const uint32_t requested_mask = nonFiniteMask(duties, kMotorCount);
    monitor.record(NonFiniteStage::MixerOutput, countMaskBits(requested_mask));
    const uint32_t zeroed_mask = guardMotorDuties(duties, requested_mask);
    monitor.record(NonFiniteStage::MotorCommand, countMaskBits(zeroed_mask));
}

}  // namespace sf
