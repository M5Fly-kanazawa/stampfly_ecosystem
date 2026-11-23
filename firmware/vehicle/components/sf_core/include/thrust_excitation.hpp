/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file thrust_excitation.hpp
 * @brief Thrust excitation: the waveform, the start / abort judgement and the state machine
 *        推力励振: 波形・開始と打ち切りの判断・状態機械
 *
 * A small signal is added to the total thrust command while the altitude loop stays closed, so that a flight
 * log shows the dead time and lag from thrust to vertical acceleration, the lag of the downward ToF and the
 * accelerometer noise in flight. This header has no ESP-IDF / FreeRTOS dependency (the ToF, the battery and the
 * flight phase are detected elsewhere and arrive as plain facts), so the host tests cover every cell:
 *   - decideThrustExcitationRequest(): the start conditions (reason codes),
 *   - decideThrustExcitationAbort():   the abort conditions of a running excitation,
 *   - ThrustExcitation:                 the state machine and the waveform (one step per 400 Hz control cycle).
 * The design is in detailed_design.md §3 ("thrust excitation").
 *
 * 高さのループを閉じたまま総推力の指令へ小さな信号を足し、飛行記録に、推力 → 鉛直の加速度のむだ時間と遅れ、
 * 下向き ToF の遅れ、飛行中の加速度計の雑音を残す。このヘッダは ESP-IDF / FreeRTOS に依存しない
 * （ToF・電池・飛行フェーズの検出は別の層が行い、素の事実として届く）ので、ホスト試験で全てのセルを検査できる:
 *   - decideThrustExcitationRequest(): 開始の条件（理由の符号）
 *   - decideThrustExcitationAbort():   実行中の励振の打ち切り条件
 *   - ThrustExcitation:                 状態機械と波形（400 Hz の制御周期ごとに 1 歩）
 * 設計は detailed_design.md §3（「推力励振」）。
 *
 * Time is counted in control cycles (the control loop runs at a fixed 400 Hz, kThrustExciteSampleRateHz), so the
 * waveform is exact and repeatable: the multisine is read from a quarter-wave table by integer phase indices, no
 * floating-point phase accumulates, and the PC regenerates the same signal from the same table.
 * 時間は制御周期で数える（制御ループは 400 Hz 固定。kThrustExciteSampleRateHz）。波形は正確で再現できる:
 * 多重正弦波は整数の位相の添字で 4 分の 1 波の表を読み、浮動小数点の位相は積算せず、PC が同じ表から同じ信号を再生成する。
 *
 * @design architecture.md INV-1 — only the vertical (thrust) channel is touched      [OK]
 * @design architecture.md INV-3 — detection elsewhere, judgement here (pure functions) [OK]
 * @design architecture.md INV-4 — every (state x event) cell is in detailed_design.md §3 [OK]
 * @design architecture.md INV-5 — the signal is added to thrust [N], never to duty     [OK]
 * @design coding_and_education.md §2 — no magic numbers, bilingual comments           [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "data_types.hpp"
#include "flight_limits.hpp"
#include "sf_math.hpp"

namespace sf {

/// Facts the judgement reads. The controller fills them from its own state and from what ControlTask injected into the
/// StateEstimate (INV-3: detection stays in the existing layers, this struct carries only the result).
/// 判断が読む事実。制御器が、自分の状態と、ControlTask が StateEstimate へ注入したものから埋める
/// （INV-3: 検出は既存の層に残り、この構造体は結果だけを運ぶ）。
struct ThrustExcitationFacts {
    bool  airborne;                // vertical phase is Airborne / 鉛直フェーズが Airborne
    bool  altitude_mode;           // flight mode is ALT_HOLD or POS_HOLD / モードが ALT_HOLD か POS_HOLD
    bool  flip_active;             // a flip is running / フリップが動いている
    bool  rate_excitation_active;  // a rate-axis excitation is running / 角速度の軸の励振が動いている
    float altitude_m;              // [m] height above the floor / 床からの高さ
    float altitude_target_m;       // [m] height target of the altitude loop / 高さのループの目標
    float vertical_speed_mps;      // [m/s] vertical speed, up positive / 鉛直速度（上が正）
    float tilt_rad;                // [rad] larger of |roll| and |pitch| / |ロール| と |ピッチ| の大きい方
    float battery_voltage;         // [V]
    bool  tof_usable;              // downward ToF valid and not silent / 下向き ToF が有効で、出力停止でない
    bool  guidance_still;          // guidance target reached, no rc velocity command, no stick reposition / 誘導の目標に着いている・rc の速度指令なし・スティックの再配置なし
    bool  stick_moved;             // a stick left its snapshot taken at the request / スティックが要求のときのスナップショットから動いた
};

/// Name of a reason code, as written in the API reply and in the log line (one short lower-case word).
/// 理由の符号の名前。API の応答とログの 1 行に書く（小文字の短い語）。
inline const char* thrustExcitationReasonName(ThrustExcitationReason reason)
{
    switch (reason) {
        case ThrustExcitationReason::None:             return "none";
        case ThrustExcitationReason::NotAirborne:      return "not_airborne";
        case ThrustExcitationReason::ModeNotAllowed:   return "mode";
        case ThrustExcitationReason::Busy:             return "busy";
        case ThrustExcitationReason::AmplitudeLimit:   return "amplitude";
        case ThrustExcitationReason::DurationLimit:    return "duration";
        case ThrustExcitationReason::StepLimit:        return "step_limit";
        case ThrustExcitationReason::BatteryLow:       return "battery";
        case ThrustExcitationReason::HeightOutOfRange: return "height";
        case ThrustExcitationReason::ToFInvalid:       return "tof";
        case ThrustExcitationReason::TiltTooLarge:     return "tilt";
        case ThrustExcitationReason::NotSettled:       return "not_settled";
        case ThrustExcitationReason::HeightWindow:     return "height_window";
        case ThrustExcitationReason::HeightError:      return "height_error";
        case ThrustExcitationReason::StickMoved:       return "stick";
        case ThrustExcitationReason::Cancelled:        return "cancelled";
        case ThrustExcitationReason::FlightEnded:      return "flight_ended";
        case ThrustExcitationReason::Preempted:        return "preempted";
    }
    return "unknown";
}

/// True when `value` is finite and inside (0, limit]. / value が有限で (0, limit] にあるとき真。
inline bool isPositiveWithin(float value, float limit)
{
    return std::isfinite(value) && value > 0.0f && value <= limit;
}

/// Nominal length of a request [s]: the Multisine length, or 2 x count x half period for Steps.
/// 要求の公称の長さ [s]: 多重正弦波は長さ、階段は 2 × 回数 × 半周期。
inline float thrustExcitationNominalLengthS(const ThrustExcitationCommand& command)
{
    if (command.kind == static_cast<uint8_t>(ThrustExcitationKind::Steps)) {
        return 2.0f * static_cast<float>(command.count) * command.half_period_s;
    }
    return command.duration_s;
}

/// Judge the numbers of a request (amplitude, length, steps). Reason None = they are inside the limits.
/// 要求の数（振幅・長さ・階段）を判断する。理由 None = 上限の中。
inline ThrustExcitationReason decideThrustExcitationRanges(const ThrustExcitationCommand& command)
{
    if (!isPositiveWithin(command.amplitude_percent, limits::kThrustExciteAmplitudeMaxPercent)) {
        return ThrustExcitationReason::AmplitudeLimit;
    }
    const bool is_steps = command.kind == static_cast<uint8_t>(ThrustExcitationKind::Steps);
    if (is_steps) {
        const bool half_ok = std::isfinite(command.half_period_s) &&
                             command.half_period_s >= limits::kThrustExciteStepHalfPeriodMinS &&
                             command.count >= 1;
        const bool product_ok = half_ok && command.amplitude_percent * command.half_period_s <=
                                           limits::kThrustExciteStepAmplitudeTimeMaxPercentSeconds;
        if (!product_ok) {
            return ThrustExcitationReason::StepLimit;
        }
    }
    const float length_s = thrustExcitationNominalLengthS(command);
    const float length_min_s = is_steps ? 0.0f : limits::kThrustExciteDurationMinS;
    if (!std::isfinite(length_s) || length_s < length_min_s || length_s > limits::kThrustExciteDurationMaxS) {
        return ThrustExcitationReason::DurationLimit;
    }
    return ThrustExcitationReason::None;
}

/// Judge a request: the start conditions. Reason None = accepted (the run still waits for the craft to be still).
/// 要求を判断する: 開始の条件。理由 None = 受理（実行は機体が静止するのをさらに待つ）。
/// @design detailed_design.md §3 thrust excitation — start conditions [OK]
inline ThrustExcitationReason decideThrustExcitationRequest(const ThrustExcitationCommand& command,
                                                            const ThrustExcitationFacts& facts,
                                                            bool already_active)
{
    if (!facts.airborne) return ThrustExcitationReason::NotAirborne;
    if (!facts.altitude_mode) return ThrustExcitationReason::ModeNotAllowed;
    if (already_active || facts.flip_active || facts.rate_excitation_active) return ThrustExcitationReason::Busy;
    const ThrustExcitationReason range_reason = decideThrustExcitationRanges(command);
    if (range_reason != ThrustExcitationReason::None) return range_reason;
    if (!(facts.battery_voltage >= limits::kThrustExciteBatteryStartMinV)) return ThrustExcitationReason::BatteryLow;
    if (!(facts.altitude_m >= limits::kThrustExciteStartHeightMinM &&
          facts.altitude_m <= limits::kThrustExciteStartHeightMaxM)) {
        return ThrustExcitationReason::HeightOutOfRange;
    }
    if (!facts.tof_usable) return ThrustExcitationReason::ToFInvalid;
    if (!(facts.tilt_rad <= limits::kThrustExciteStartTiltMaxRad)) return ThrustExcitationReason::TiltTooLarge;
    return ThrustExcitationReason::None;
}

/// True while the craft is still enough to begin: small height error, small vertical speed, small tilt, a valid ToF
/// and a still guidance. / 始めてよいほど機体が静止している間、真。
inline bool isThrustExcitationStill(const ThrustExcitationFacts& facts)
{
    const float error_m = std::fabs(facts.altitude_target_m - facts.altitude_m);
    return error_m <= limits::kThrustExciteStartAltitudeErrorMaxM &&
           std::fabs(facts.vertical_speed_mps) <= limits::kThrustExciteStartVerticalSpeedMaxMps &&
           facts.tilt_rad <= limits::kThrustExciteStartTiltMaxRad &&
           facts.tof_usable && facts.guidance_still;
}

/// Judge a waiting or running excitation: the abort conditions. Reason None = go on. `tof_lost_s` and `battery_low_s` are the
/// times the ToF has been unusable and the voltage has been below the abort voltage (the caller counts them).
/// 待機中・実行中の励振を判断する: 打ち切り条件。理由 None = 続ける。tof_lost_s・battery_low_s は、ToF が使えなかった時間と
/// 電圧が打ち切りの電圧を下回っていた時間（呼び出し側が数える）。
/// @design detailed_design.md §3 thrust excitation — abort conditions [OK]
inline ThrustExcitationReason decideThrustExcitationAbort(const ThrustExcitationFacts& facts,
                                                          float tof_lost_s, float battery_low_s)
{
    if (!facts.airborne) return ThrustExcitationReason::FlightEnded;
    if (facts.flip_active) return ThrustExcitationReason::Preempted;
    if (!facts.altitude_mode) return ThrustExcitationReason::ModeNotAllowed;
    if (facts.stick_moved) return ThrustExcitationReason::StickMoved;
    if (!(facts.altitude_m >= limits::kThrustExciteRunHeightMinM &&
          facts.altitude_m <= limits::kThrustExciteRunHeightMaxM)) {
        return ThrustExcitationReason::HeightWindow;
    }
    if (!(std::fabs(facts.altitude_target_m - facts.altitude_m) <= limits::kThrustExciteAltitudeErrorAbortM)) {
        return ThrustExcitationReason::HeightError;
    }
    if (tof_lost_s >= limits::kThrustExciteToFLostAbortS) return ThrustExcitationReason::ToFInvalid;
    if (!(facts.tilt_rad <= limits::kThrustExciteTiltAbortRad)) return ThrustExcitationReason::TiltTooLarge;
    if (battery_low_s >= limits::kThrustExciteBatteryAbortHoldS) return ThrustExcitationReason::BatteryLow;
    return ThrustExcitationReason::None;
}

/// sin(2*pi*index/period) from the quarter-wave table, for an integer index in [0, period).
/// 整数の添字 index（[0, period)）に対する sin(2π·index/period)。4 分の 1 波の表から読む。
inline float thrustExciteSine(uint32_t index)
{
    constexpr uint32_t kQuarter = limits::kThrustExcitePeriodCycles / 4u;
    const uint32_t quadrant = index / kQuarter;
    const uint32_t offset = index % kQuarter;
    switch (quadrant) {
        case 0:  return limits::kThrustExciteQuarterSine[offset];
        case 1:  return limits::kThrustExciteQuarterSine[kQuarter - offset];
        case 2:  return -limits::kThrustExciteQuarterSine[offset];
        default: return -limits::kThrustExciteQuarterSine[kQuarter - offset];
    }
}

/// The multisine at control cycle `cycle` (any value; the pattern repeats every kThrustExcitePeriodCycles), normalised so that
/// its peak is 1. / 制御周期 cycle の多重正弦波（任意の値。kThrustExcitePeriodCycles ごとに繰り返す）。ピークが 1 になるよう正規化。
inline float thrustExciteMultisine(uint32_t cycle)
{
    constexpr uint32_t kPeriod = limits::kThrustExcitePeriodCycles;
    const uint32_t cycle_in_period = cycle % kPeriod;
    float sum = 0.0f;
    for (uint32_t tone = 0; tone < limits::kThrustExciteToneCount; ++tone) {
        const uint32_t phase = limits::kThrustExciteToneFrequencyIndex[tone] * cycle_in_period +
                               limits::kThrustExciteTonePhaseIndex[tone];
        sum += thrustExciteSine(phase % kPeriod);
    }
    return sum * limits::kThrustExciteMultisineScale;
}

/// Half-cosine rise: 0 at fraction 0, 1 at fraction 1. / 半余弦の立ち上がり: 割合 0 で 0、割合 1 で 1。
inline float thrustExciteHalfCosineRise(uint32_t step, uint32_t steps)
{
    return 0.5f * (1.0f - std::cos(math::kPi * static_cast<float>(step) / static_cast<float>(steps)));
}

/**
 * @brief State machine and waveform of the thrust excitation.
 *        推力励振の状態機械と波形。
 *
 * request() accepts or rejects; step() is called once per control cycle while isStepping() and returns the thrust [N] to add
 * (0 when idle). cancel() ramps a run out; reset() stops at once (DISARM). Nothing here allocates or logs.
 * request() が受理・拒否し、step() は isStepping() の間、制御周期ごとに 1 回呼ばれ、足す推力 [N] を返す（アイドルでは 0）。
 * cancel() は実行を立ち下げて止め、reset() は即時に止める（DISARM）。ここは確保もログも行わない。
 *
 * @design detailed_design.md §3 — thrust excitation state machine [OK]
 */
class ThrustExcitation {
public:
    /// Accept or reject a request. On acceptance the state becomes Standby. Returns the reason (None = accepted).
    /// 要求を受理・拒否する。受理で状態は Standby になる。理由を返す（None = 受理）。
    ThrustExcitationReason request(const ThrustExcitationCommand& command, const ThrustExcitationFacts& facts,
                                   float hover_thrust_n)
    {
        const ThrustExcitationReason reason = decideThrustExcitationRequest(command, facts, isActive());
        if (reason != ThrustExcitationReason::None) {
            if (reason != ThrustExcitationReason::Busy) {   // a refused second request must not erase the first one's status / 拒否された 2 つ目の要求が 1 つ目の状態を消さない
                recordFinish(ThrustExcitationResult::Rejected, reason);
            }
            return reason;
        }
        acceptRequest(command, hover_thrust_n);
        return reason;
    }

    /// Cancel: a waiting excitation ends at once, a running one ramps out. / 取り消し: 待機中は即終了、実行中は立ち下げる。
    void cancel()
    {
        abortWith(ThrustExcitationReason::Cancelled);
    }

    /// Stop at once without a ramp (DISARM, reset). / 立ち下げずに即時に止める（DISARM・reset）。
    void reset()
    {
        if (state_ != ThrustExcitationState::None) {
            result_ = ThrustExcitationResult::Aborted;
            reason_ = ThrustExcitationReason::FlightEnded;
        }
        state_ = ThrustExcitationState::None;
        holdoff_cycles_ = 0;
        stepping_ = false;
    }

    /// One control cycle. Returns the thrust [N] to add to the command (0 when idle).
    /// 制御周期 1 回分。指令へ足す推力 [N] を返す（アイドルでは 0）。
    float step(const ThrustExcitationFacts& facts)
    {
        if (state_ == ThrustExcitationState::None) {
            tickHoldoff();
            return 0.0f;
        }
        ++elapsed_cycles_;
        if (state_ == ThrustExcitationState::Standby) {
            stepStandby(facts);
            return 0.0f;
        }
        return stepRun(facts);
    }

    /// True while step() must be called (a request is in progress, or the learning hold-off is counting down).
    /// step() を呼ぶべき間、真（要求が進行中、または学習の停止の残り時間を数えている）。
    bool isStepping() const { return stepping_; }
    /// True while a request is waiting or running. / 要求が待機中・実行中の間、真。
    bool isActive() const { return state_ != ThrustExcitationState::None; }
    /// True while the hover-thrust learner and the support-thrust window must not update (running, and for a while after).
    /// ホバー推力の学習と支持推力の窓が更新してはならない間、真（実行中と、その後しばらく）。
    bool suppressesLearning() const { return stepping_; }

    ThrustExcitationState  state() const { return state_; }
    ThrustExcitationResult result() const { return result_; }
    ThrustExcitationReason reason() const { return reason_; }
    ThrustExcitationKind   kind() const { return kind_; }
    /// Amplitude of the accepted request in 1/1000 of the hover thrust. / 受理した要求の振幅（ホバー推力の千分率）。
    uint16_t amplitudePermille() const { return amplitude_permille_; }
    /// Milliseconds since the request was accepted (frozen after the end). / 受理からの経過 [ms]（終了後は止まる）。
    uint32_t elapsedMs() const
    {
        return elapsed_cycles_ * 1000u / limits::kThrustExciteSampleRateHz;
    }

private:
    /// Convert seconds to whole control cycles. / 秒を制御周期の整数に変換する。
    static uint32_t cyclesOf(float seconds)
    {
        return static_cast<uint32_t>(seconds * static_cast<float>(limits::kThrustExciteSampleRateHz) + 0.5f);
    }

    /// Store the request and enter Standby. / 要求を保持して Standby に入る。
    void acceptRequest(const ThrustExcitationCommand& command, float hover_thrust_n)
    {
        kind_ = static_cast<ThrustExcitationKind>(command.kind);
        amplitude_ratio_ = command.amplitude_percent * 0.01f;
        amplitude_permille_ = static_cast<uint16_t>(command.amplitude_percent * 10.0f + 0.5f);
        hover_thrust_n_ = hover_thrust_n;
        const bool is_steps = kind_ == ThrustExcitationKind::Steps;
        half_cycles_ = is_steps ? cyclesOf(command.half_period_s) : 0u;
        body_cycles_ = is_steps ? 2u * command.count * half_cycles_ : cyclesOf(command.duration_s);
        ramp_up_cycles_ = is_steps ? limits::kThrustExciteStepsRampUpCycles : limits::kThrustExciteMultisineRampUpCycles;
        ramp_down_cycles_ = is_steps ? limits::kThrustExciteStepsRampDownCycles
                                     : limits::kThrustExciteMultisineRampDownCycles;
        total_cycles_ = is_steps ? body_cycles_ + ramp_down_cycles_ : body_cycles_;
        state_ = ThrustExcitationState::Standby;
        result_ = ThrustExcitationResult::Running;
        reason_ = ThrustExcitationReason::None;
        cycle_ = 0;
        standby_cycles_ = 0;
        still_cycles_ = 0;
        tof_lost_cycles_ = 0;
        battery_low_cycles_ = 0;
        elapsed_cycles_ = 0;
        aborting_ = false;
        abort_cycle_ = 0;
        stepping_ = true;
    }

    /// Record the end of a request. The learning hold-off starts after a run (not after a refusal). / 要求の終了を記録する。
    void recordFinish(ThrustExcitationResult result, ThrustExcitationReason reason)
    {
        const bool was_running = state_ != ThrustExcitationState::None && state_ != ThrustExcitationState::Standby;
        state_ = ThrustExcitationState::None;
        result_ = result;
        reason_ = reason;
        aborting_ = false;
        holdoff_cycles_ = was_running ? cyclesOf(limits::kThrustExciteLearningHoldOffS) : 0u;
        stepping_ = holdoff_cycles_ > 0u;
    }

    /// Count the learning hold-off down. / 学習の停止の残り時間を数える。
    void tickHoldoff()
    {
        if (holdoff_cycles_ > 0u) {
            --holdoff_cycles_;
        }
        stepping_ = holdoff_cycles_ > 0u;
    }

    /// Stop with a reason: a waiting excitation ends at once; a running one starts the abort ramp (once).
    /// 理由つきで止める: 待機中は即終了。実行中は打ち切りの立ち下げを始める（1 回だけ）。
    void abortWith(ThrustExcitationReason reason)
    {
        if (state_ == ThrustExcitationState::None || aborting_) return;
        if (state_ == ThrustExcitationState::Standby) {
            recordFinish(ThrustExcitationResult::Aborted, reason);
            return;
        }
        aborting_ = true;
        abort_cycle_ = 0;
        reason_ = reason;
        state_ = ThrustExcitationState::RampDown;
    }

    /// Standby: wait for a still craft; begin after kThrustExciteStartSettleHoldS, give up after kThrustExciteStartSettleTimeoutS.
    /// Standby: 機体の静止を待つ。kThrustExciteStartSettleHoldS 静止が続けば始め、kThrustExciteStartSettleTimeoutS で諦める。
    void stepStandby(const ThrustExcitationFacts& facts)
    {
        const ThrustExcitationReason reason = decideThrustExcitationAbort(facts, 0.0f, 0.0f);
        const bool cannot_wait = reason == ThrustExcitationReason::FlightEnded ||
                                 reason == ThrustExcitationReason::Preempted ||
                                 reason == ThrustExcitationReason::ModeNotAllowed ||
                                 reason == ThrustExcitationReason::StickMoved;
        if (cannot_wait) {
            recordFinish(ThrustExcitationResult::Aborted, reason);
            return;
        }
        ++standby_cycles_;
        still_cycles_ = isThrustExcitationStill(facts) ? still_cycles_ + 1u : 0u;
        if (still_cycles_ >= cyclesOf(limits::kThrustExciteStartSettleHoldS)) {
            state_ = ThrustExcitationState::RampUp;
            cycle_ = 0;
        } else if (standby_cycles_ >= cyclesOf(limits::kThrustExciteStartSettleTimeoutS)) {
            recordFinish(ThrustExcitationResult::Rejected, ThrustExcitationReason::NotSettled);
        }
    }

    /// Count the cycles during which the ToF is unusable and the battery is low, for the abort judgement.
    /// 打ち切りの判断のために、ToF が使えない周期と電池が低い周期を数える。
    void updateDebounce(const ThrustExcitationFacts& facts)
    {
        tof_lost_cycles_ = facts.tof_usable ? 0u : tof_lost_cycles_ + 1u;
        const bool battery_low = facts.battery_voltage < limits::kThrustExciteBatteryAbortV;
        battery_low_cycles_ = battery_low ? battery_low_cycles_ + 1u : 0u;
    }

    /// Envelope of the nominal waveform at cycle `cycle`: the rise at the start, the fall at the end. 0..1.
    /// 公称の波形の包絡線（周期 cycle）: 始めの立ち上げと終わりの立ち下げ。0..1。
    float envelope(uint32_t cycle) const
    {
        if (cycle < ramp_up_cycles_) {
            return thrustExciteHalfCosineRise(cycle, ramp_up_cycles_);
        }
        const uint32_t fall_start = total_cycles_ - ramp_down_cycles_;
        if (cycle >= fall_start) {
            const uint32_t steps_into_fall = cycle - fall_start + 1u;
            return 1.0f - thrustExciteHalfCosineRise(steps_into_fall, ramp_down_cycles_);
        }
        return 1.0f;
    }

    /// Waveform (peak 1) at cycle `cycle`. / 周期 cycle の波形（ピーク 1）。
    float waveform(uint32_t cycle) const
    {
        if (kind_ == ThrustExcitationKind::Multisine) {
            return thrustExciteMultisine(cycle);
        }
        const uint32_t held = cycle < body_cycles_ ? cycle : body_cycles_ - 1u;   // the last level is held through the fall / 最後の段は立ち下げの間も保つ
        return ((held / half_cycles_) % 2u == 0u) ? 1.0f : -1.0f;
    }

    /// State label of the cycle: RampUp, Running or RampDown (an abort is always RampDown).
    /// その周期の状態の名前: RampUp・Running・RampDown（打ち切りは常に RampDown）。
    ThrustExcitationState stateAt(uint32_t cycle) const
    {
        if (aborting_ || cycle >= total_cycles_ - ramp_down_cycles_) return ThrustExcitationState::RampDown;
        return cycle < ramp_up_cycles_ ? ThrustExcitationState::RampUp : ThrustExcitationState::Running;
    }

    /// Running: judge the abort, advance one cycle and return the thrust [N] to add.
    /// 実行中: 打ち切りを判断し、1 周期進め、足す推力 [N] を返す。
    float stepRun(const ThrustExcitationFacts& facts)
    {
        updateDebounce(facts);
        const float rate_hz = static_cast<float>(limits::kThrustExciteSampleRateHz);   // divide, not multiply by 1/rate: 40 / 400 is exactly 0.1 / 割る（1/rate を掛けない）: 40 / 400 はちょうど 0.1
        const ThrustExcitationReason reason = decideThrustExcitationAbort(
            facts, static_cast<float>(tof_lost_cycles_) / rate_hz, static_cast<float>(battery_low_cycles_) / rate_hz);
        if (reason != ThrustExcitationReason::None) {
            abortWith(reason);
        }
        float gain = envelope(cycle_);
        if (aborting_) {
            const uint32_t abort_cycles = limits::kThrustExciteAbortRampCycles;
            gain *= 1.0f - thrustExciteHalfCosineRise(abort_cycle_, abort_cycles);
            ++abort_cycle_;
        }
        const float value_n = amplitude_ratio_ * hover_thrust_n_ * waveform(cycle_) * gain;
        advanceCycle();
        return value_n;
    }

    /// Move to the next cycle and end the run when its length or the abort ramp is over.
    /// 次の周期へ進み、長さまたは打ち切りの立ち下げが終わったら実行を終える。
    void advanceCycle()
    {
        ++cycle_;
        const bool abort_done = aborting_ && abort_cycle_ > limits::kThrustExciteAbortRampCycles;
        if (abort_done) {
            recordFinish(ThrustExcitationResult::Aborted, reason_);
        } else if (cycle_ >= total_cycles_) {
            const bool was_aborting = aborting_;
            recordFinish(was_aborting ? ThrustExcitationResult::Aborted : ThrustExcitationResult::Completed,
                         was_aborting ? reason_ : ThrustExcitationReason::None);
        } else {
            state_ = stateAt(cycle_);
        }
    }

    ThrustExcitationState  state_ = ThrustExcitationState::None;
    ThrustExcitationResult result_ = ThrustExcitationResult::None;
    ThrustExcitationReason reason_ = ThrustExcitationReason::None;
    ThrustExcitationKind   kind_ = ThrustExcitationKind::Cancel;
    float    amplitude_ratio_ = 0.0f;       // ratio of the hover thrust / ホバー推力に対する割合
    uint16_t amplitude_permille_ = 0;       // the same in 1/1000 / 同じ値（千分率）
    float    hover_thrust_n_ = 0.0f;        // [N] hover thrust at the request / 要求のときのホバー推力
    uint32_t half_cycles_ = 0;              // cycles of one step / 1 段の周期数
    uint32_t body_cycles_ = 0;              // nominal length in cycles / 公称の長さ（周期数）
    uint32_t total_cycles_ = 0;             // body plus the appended fall of the steps / 本体 + 階段の追加の立ち下げ
    uint32_t ramp_up_cycles_ = 0;
    uint32_t ramp_down_cycles_ = 0;
    uint32_t cycle_ = 0;                    // cycles since the run began / 実行の開始からの周期数
    uint32_t standby_cycles_ = 0;           // cycles spent waiting / 待機した周期数
    uint32_t still_cycles_ = 0;             // consecutive still cycles / 静止が続いた周期数
    uint32_t tof_lost_cycles_ = 0;
    uint32_t battery_low_cycles_ = 0;
    uint32_t elapsed_cycles_ = 0;
    uint32_t holdoff_cycles_ = 0;           // learning hold-off left / 学習の停止の残り
    uint32_t abort_cycle_ = 0;              // cycles into the abort ramp / 打ち切りの立ち下げの進み
    bool     aborting_ = false;
    bool     stepping_ = false;
};

}  // namespace sf
