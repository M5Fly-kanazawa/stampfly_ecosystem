/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_policy.hpp
 * @brief How a TAKEOFF ends, and what a lost link does during it (pure decisions)
 *        TAKEOFF がどう終わるか、その間に通信が途絶えたらどうなるか（純粋な判断関数）
 *
 * Principle (8th pass): the ABSENCE of evidence is never treated as evidence that the craft is on the floor. The accelerometer alone cannot tell a craft that
 * lifts slowly from one that the floor still holds, so no safety decision here is gated on "the craft has left the floor" as seen by the accelerometer. The
 * only evidence of being on the floor that this header acts on is POSITIVE and comes from a healthy ToF: a valid reading near the ground that does not rise
 * (TakeoffFloorWatch). 8 回目の方針: 証拠が「無い」ことを、床にいる証拠として扱わない。加速度計だけでは、ゆっくり浮く機体と床がまだ支えている機体を区別
 * できないので、ここでの安全判断は、加速度計が見た「床を離れた」を門にしない。ここで使う「床にいる」証拠は、健全な ToF が示す「積極的な」もの
 * （地面の近くの有効な読みで、増えていない: TakeoffFloorWatch）だけ。
 *
 * Three decisions, all pure functions of their facts so that every cell of the normative table (detailed_design.md §3.1, INV-4) can be checked on the host
 * (test/test_takeoff_policy.cpp). StateTask / StateManager execute the result (INV-3): this header only DECIDES.
 *  1. decideTakeoff(): TAKEOFF -> FLYING (complete), TAKEOFF -> IDLE_GROUND (abort, DISARM) or stay.
 *       manual (ACRO / STABILIZE): complete when the ToF-based airborne verdict says so (while a ToF-silence episode is in progress the verdict is "armed = not
 *         on the ground", ImuTask: nothing proves ground contact). A craft that the floor still holds may therefore show FLYING (the pilot has the throttle).
 *       auto (ALT_HOLD / POS_HOLD, ARM-triggered or API): complete when the controller reports the target captured (or its 3 s backstop), UNLESS the ToF
 *         positively shows the craft on the floor (floor_stays): then it stays in TAKEOFF, and kAutoTakeoffNoLiftAbortUs after the TAKEOFF began it is aborted by DISARM
 *         (the motors stop; not "back to ARMED_GROUND", which would start the auto-takeoff again). A ToF that is silent or invalid is not that evidence: the ToF-invalid rule
 *         (tof_health_tracker.hpp) ends such a takeoff by a LANDING instead.
 *  2. decideCommLossLanding(): once the grace after COMM_LOST has elapsed, TAKEOFF and FLYING both go to LANDING (it used to be FLYING only, so a loss during a
 *     TAKEOFF that never completed kept the craft climbing for ever).
 *  3. TakeoffFloorWatch: the detection of the positive evidence.
 *
 * 3 つの判断で、いずれも事実の純関数なので、規範表（detailed_design.md §3.1、INV-4）の全セルをホストで検証できる（test/test_takeoff_policy.cpp）。結果の実行は
 * StateTask / StateManager（INV-3）で、このヘッダは「判断」だけをする。
 *  1. decideTakeoff(): TAKEOFF → FLYING（完了）、TAKEOFF → IDLE_GROUND（中止、DISARM）、または留まる。
 *       手動（ACRO / STABILIZE）: ToF による空中の判定が立ったら完了（ToF 出力停止のエピソード中、この判定は「ARM 中 = 接地していない」で、ImuTask が立てる:
 *         接地を示す証拠が無いので）。床がまだ支えている機体も FLYING と表示されうる（スロットルはパイロットが握っている）。
 *       自動（ALT_HOLD / POS_HOLD。ARM 起動または API）: 制御器が目標の捕捉（または 3 s の備え）を報告したら完了。ただし ToF が機体は床の上にいると「積極的に」示して
 *         いる（floor_stays）ときは TAKEOFF に留め、TAKEOFF の開始から kAutoTakeoffNoLiftAbortUs 経ったら DISARM で中止する（モータ停止。「ARMED_GROUND へ戻す」にしない:
 *         自動離陸をまた始めるため）。出力停止の ToF や無効サンプルはその証拠ではない: そういう離陸は ToF 無効の規則（tof_health_tracker.hpp）が LANDING で終わらせる。
 *  2. decideCommLossLanding(): COMM_LOST の猶予が過ぎたら、TAKEOFF も FLYING も LANDING にする（以前は FLYING だけで、完了しない TAKEOFF の間に通信が途絶えると
 *     機体は上昇し続けた）。
 *  3. TakeoffFloorWatch: 積極的な証拠の検出。
 *
 * @design architecture.md INV-3 — decision here, execution in StateTask / StateManager          [OK]
 * @design architecture.md INV-4 — every (state x input) cell is in the table                    [OK]
 * @design detailed_design.md §3.1 — TAKEOFF -> FLYING / IDLE_GROUND rows, comm-loss row        [OK]
 * @design coding_and_education.md §2 — Bilingual comments                                       [OK]
 */

#pragma once

#include <cstdint>

#include "flight_limits.hpp"
#include "flight_state.hpp"

namespace sf {

/// What StateTask does with the TAKEOFF state this cycle. この周期に StateTask が TAKEOFF 状態にすること。
enum class TakeoffAction : uint8_t {
    Continue,      // stay in TAKEOFF / TAKEOFF に留まる
    Complete,      // TAKEOFF -> FLYING (notifyTakeoffComplete) / TAKEOFF → FLYING
    AbortDisarm,   // TAKEOFF -> IDLE_GROUND by DISARM (the craft cannot lift) / DISARM で TAKEOFF → IDLE_GROUND（浮けない）
};

/// The facts decideTakeoff() needs. decideTakeoff() が要る事実。
struct TakeoffFacts {
    bool     auto_takeoff;        ///< ALT_HOLD / POS_HOLD (ARM-triggered or API) / 自動離陸（ALT_HOLD・POS_HOLD）
    bool     airborne;            ///< system_status.airborne (ToF-based; "armed = not on the ground" while the ToF is silent) / ToF による空中の判定
    bool     controller_reached;  ///< controller_status.takeoff_reached (target captured, or its backstop) / 目標を捕捉（または備え）
    bool     floor_stays;         ///< TakeoffFloorWatch::stays(): the healthy ToF positively shows the craft on the floor / ToF が床の上と積極的に示す
    uint32_t elapsed_us;          ///< time since the TAKEOFF began [us] / TAKEOFF の開始からの時間 [us]
};

/// The decision for the TAKEOFF state (rules in the file comment). TAKEOFF 状態に対する判断（規則はファイルの説明）。
/// @design detailed_design.md §3.1 — TAKEOFF -> FLYING (manual: ToF airborne; auto: captured, no floor evidence) / TAKEOFF -> IDLE_GROUND (floor evidence 5 s) [OK]
inline constexpr TakeoffAction decideTakeoff(const TakeoffFacts& facts)
{
    if (!facts.auto_takeoff) {
        return facts.airborne ? TakeoffAction::Complete : TakeoffAction::Continue;
    }
    if (facts.floor_stays && facts.elapsed_us >= limits::kAutoTakeoffNoLiftAbortUs) {
        return TakeoffAction::AbortDisarm;
    }
    if (facts.controller_reached && !facts.floor_stays) {
        return TakeoffAction::Complete;
    }
    return TakeoffAction::Continue;
}

/// True when the grace after COMM_LOST has elapsed and the state must go to LANDING: TAKEOFF and FLYING, in every flight mode. Other states do not
/// (LANDING is already it, FLIP defers to FLYING, the ground ends the loss). COMM_LOST の猶予が過ぎ、状態を LANDING にすべきなら true:
/// TAKEOFF と FLYING、全モード。他の状態は違う（LANDING は既にそれ、FLIP は FLYING まで保留、地上は喪失を終わらせる）。
/// @design detailed_design.md §3.1 — comm loss: hover 3 s -> LANDING (also from TAKEOFF) [OK]
inline constexpr bool decideCommLossLanding(FlightState state, bool grace_elapsed)
{
    return grace_elapsed && (state == FlightState::FLYING || state == FlightState::TAKEOFF);
}

/// Detects the POSITIVE evidence that an auto-takeoff is still on the floor: the healthy ToF reads a valid height at or below kTakeoffGroundBandM, and that reading
/// has not risen by more than kAutoTakeoffFloorRiseM above the lowest of the run, for kAutoTakeoffFloorHoldUs. A reading that rises restarts the run; a
/// reading above the band, or a ToF that is not healthy, ends it; an INVALID sample is neutral (it neither proves nor disproves: the real floor reads invalid
/// in the dead zone). ImuTask owns it (INV-3: detection there, decision in StateTask): ONE call per NEW ToF sample while in TAKEOFF, reset() otherwise; the fact goes out as tof_health.floor_stays.
/// 自動離陸がまだ床の上にいるという「積極的な」証拠を検出する: 健全な ToF が kTakeoffGroundBandM 以下の有効な高さを読み、その読みが、連続区間の最低値より
/// kAutoTakeoffFloorRiseM を超えて増えていないことが kAutoTakeoffFloorHoldUs 続く。読みが増えたら区間を数え直し、帯より上の読みや健全でない ToF は区間を終わらせ、
/// 「無効」サンプルは中立（証明も反証もしない: 実際の床は不感帯で無効を返す）。ImuTask が持つ（INV-3: 検出はそこ、判断は StateTask）: TAKEOFF の間は新しい ToF サンプルごとに 1 回呼び、それ以外では reset() する。事実は tof_health.floor_stays として発行する。
class TakeoffFloorWatch {
public:
    /// Forget everything (at the TAKEOFF entry). 全てを忘れる（TAKEOFF の突入で）。
    void reset()
    {
        run_active_ = false;
        run_since_us_ = 0;
        lowest_m_ = 0.0f;
    }

    /// One NEW ToF sample. 新しい ToF サンプル 1 つ。
    /// @param now_us      Time of the sample [us]          サンプルの時刻 [us]
    /// @param tof_healthy The ToF is not silent           ToF が出力停止でない
    /// @param valid       The sample is a valid range      サンプルが有効な測距
    /// @param distance_m  Measured height [m]              測った高さ [m]
    void onSample(uint32_t now_us, bool tof_healthy, bool valid, float distance_m)
    {
        if (!tof_healthy) {
            run_active_ = false;
            return;
        }
        if (!valid) {
            return;   // neutral: an invalid sample is no evidence either way / 中立: 無効サンプルはどちらの証拠でもない
        }
        if (distance_m > limits::kTakeoffGroundBandM) {
            run_active_ = false;   // above the band: lifted / 帯より上: 浮いた
            return;
        }
        if (!run_active_ || distance_m > lowest_m_ + limits::kAutoTakeoffFloorRiseM) {
            run_active_   = true;   // a new run: the first reading, or the reading has risen / 新しい区間: 最初の読み、または読みが増えた
            run_since_us_ = now_us;
            lowest_m_     = distance_m;
            return;
        }
        if (distance_m < lowest_m_) {
            lowest_m_ = distance_m;
        }
    }

    /// The evidence has held for kAutoTakeoffFloorHoldUs and the ToF is healthy now. 証拠が kAutoTakeoffFloorHoldUs 続き、かつ今 ToF が健全。
    bool stays(uint32_t now_us, bool tof_healthy) const
    {
        const uint32_t held_us = static_cast<uint32_t>(now_us - run_since_us_);   // unsigned difference: correct across a wrap / 符号なしの差: 巻き戻りをまたいでも正しい
        return tof_healthy && run_active_ && held_us >= limits::kAutoTakeoffFloorHoldUs;
    }

private:
    bool     run_active_   = false;   // a run of in-band readings is in progress / 帯の中の読みの連続区間が進行中
    uint32_t run_since_us_ = 0;       // start of the run [us] / 区間の開始 [us]
    float    lowest_m_     = 0.0f;    // lowest reading of the run [m] / 区間の最低の読み [m]
};

}  // namespace sf
