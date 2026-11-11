/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_policy.hpp
 * @brief How a TAKEOFF starts and ends, and what a lost link does during it (pure decisions)
 *        TAKEOFF がどう始まりどう終わるか、その間に通信が途絶えたらどうなるか（純粋な判断関数）
 *
 * Principle: the ABSENCE of evidence is never treated as evidence that the craft is on the floor. The accelerometer alone cannot tell a craft that
 * lifts slowly from one that the floor still holds, so no safety decision here is gated on "the craft has left the floor" as seen by the accelerometer. The
 * only evidence of being on the floor that this header acts on is POSITIVE and comes from a healthy ToF: a valid reading near the ground that does not rise
 * (TakeoffFloorWatch). 方針: 証拠が「無い」ことを、床にいる証拠として扱わない。加速度計だけでは、ゆっくり浮く機体と床がまだ支えている機体を区別
 * できないので、ここでの安全判断は、加速度計が見た「床を離れた」を判定条件にしない。ここで使う「床にいる」証拠は、健全な ToF が示す「積極的な」もの
 * （地面の近くの有効な読みで、増えていない: TakeoffFloorWatch）だけ。
 *
 * Six decisions, all pure functions of their facts so that every cell of the normative table (detailed_design.md §3.1, INV-4) can be checked on the host
 * (test/test_takeoff_policy.cpp). StateTask / StateManager execute the result (INV-3): this header only DECIDES.
 *  1. decideTakeoff(): TAKEOFF -> FLYING (complete), TAKEOFF -> IDLE_GROUND (abort, DISARM), TAKEOFF -> LANDING (the latch that does not release) or stay.
 *       manual (ACRO / STABILIZE): complete when the ToF-based airborne verdict says so (while a ToF-silence episode is in progress the verdict is "armed = not
 *         on the ground", ImuTask: nothing proves ground contact). A craft that the floor still holds may therefore show FLYING (the pilot has the throttle).
 *       auto (ALT_HOLD / POS_HOLD, ARM-triggered or API): complete when the controller reports the target captured (or its 3 s backstop), UNLESS the ToF
 *         positively shows the craft on the floor (floor_stays), or has shown it (floor_seen, the latch: the evidence held once and has not been released since,
 *         so that one stray reading outside the band cannot let a takeoff complete on the floor): then it stays in TAKEOFF,
 *         and (floor_stays only) kAutoTakeoffNoLiftAbortUs after the TAKEOFF began it is aborted by DISARM
 *         (the motors stop; not "back to ARMED_GROUND", which would start the auto-takeoff again). The latch is released by the craft leaving the floor, with the same weight of
 *         evidence that the abort needs to be reached: a valid reading above kTakeoffGroundBandM for kAutoTakeoffFloorHoldUs (1 s), or one valid reading above
 *         kTakeoffAirborneBandM (TakeoffFloorWatch). A craft that hangs in the 5-15 cm ground effect is therefore not held for ever. A latch that does NOT release
 *         (the readings straddle kTakeoffGroundBandM, each run shorter than kAutoTakeoffFloorHoldUs, so neither the abort nor the release is reached) ends the TAKEOFF by a LANDING
 *         once kAutoTakeoffNoLiftAbortUs has passed, the evidence is not there NOW (floor_seen without floor_stays) and the craft is not lifting off now (no run of valid readings above the ground band in progress,
 *         floor_leaving: a slow lift is not landed in the middle of leaving, the latch is released after kAutoTakeoffFloorHoldUs of it): a craft that the takeoff thrust (the hover estimate plus the correction) has not
 *         lifted for kAutoTakeoffNoLiftAbortUs is not lifted by the landing thrust, which starts from the hover estimate and descends (SILS, detailed_design.md §3 note 8 (3)). A ToF that is silent or invalid is not
 *         that evidence: the ToF-invalid rule (tof_health_tracker.hpp) ends such a takeoff by a LANDING instead. The DISARM of the pilot in TAKEOFF is the motor cut, and an API `land`
 *         that arrives in TAKEOFF is held by ApiTask until the TAKEOFF has been left (StateManager::notifyLandingRequest ignores TAKEOFF).
 *  2. decideCommLossLanding(): once the grace after COMM_LOST has elapsed, TAKEOFF and FLYING both go to LANDING (covering FLYING alone would let a loss during a
 *     TAKEOFF that never completes keep the craft climbing for ever).
 *  3. TakeoffFloorWatch: the detection of the positive evidence.
 *  4. landingVerdictCounts(): whether a touchdown verdict of ImuTask ends the LANDING in force (only a verdict judged in that very LANDING does).
 *  5. decideArmedGroundTakeoff() and LiftoffWatch: how ARMED_GROUND ends in TAKEOFF. ALT_HOLD / POS_HOLD: the spool dwell has elapsed. ACRO / STABILIZE: the throttle is past the
 *     takeoff threshold, OR the healthy ToF shows the craft off the floor (valid readings above kTakeoffGroundBandM, uninterrupted for kAutoTakeoffFloorHoldUs): a craft that rises with
 *     the throttle at or below the threshold (a gust, a fresh battery, ground effect) is in the air, and ARMED_GROUND is a state of the floor (it has no comm-loss landing, no battery
 *     landing, and a silent ToF cuts its motors). The evidence is POSITIVE too: a silent, invalid or in-band ToF never starts a TAKEOFF, the throttle alone decides then.
 *  6. decideBatteryEmergency(): what the battery EMERGENCY (the voltage is below the critical threshold) does in each state. FLYING lands; FLIP defers the landing until FLYING is back; in TAKEOFF the craft
 *     lands only when it has been shown to be OFF the floor (TakeoffFloorWatch::leaving(): the healthy ToF reads valid heights above kTakeoffGroundBandM now), or when the ToF is silent (the blind descent
 *     of the LANDING cannot lift a craft, so nothing is lost on the floor and the craft is not dropped in the air); otherwise the motors are cut at once: a LANDING that starts on the floor can float the
 *     craft up (detailed_design.md §3.1), and a ToF that is healthy but reads invalid (the real floor reads invalid in the dead zone) or reads in the band is no evidence of the air. The ground and LANDING only log.
 *
 * 6 つの判断で、いずれも事実の純関数なので、規範表（detailed_design.md §3.1、INV-4）の全セルをホストで検証できる（test/test_takeoff_policy.cpp）。結果の実行は
 * StateTask / StateManager（INV-3）で、このヘッダは「判断」だけをする。
 *  1. decideTakeoff(): TAKEOFF → FLYING（完了）、TAKEOFF → IDLE_GROUND（中止、DISARM）、TAKEOFF → LANDING（解けないラッチ）、または留まる。
 *       手動（ACRO / STABILIZE）: ToF による空中の判定が立ったら完了（ToF 出力停止のエピソード中、この判定は「ARM 中 = 接地していない」で、ImuTask が立てる:
 *         接地を示す証拠が無いので）。床がまだ支えている機体も FLYING と表示されうる（スロットルはパイロットが握っている）。
 *       自動（ALT_HOLD / POS_HOLD。ARM 起動または API）: 制御器が目標の捕捉（または 3 s の備え）を報告したら完了。ただし ToF が機体は床の上にいると「積極的に」示して
 *         いる（floor_stays）、または示した（floor_seen: ラッチ。証拠が一度立ち、以後解けていない。帯の外へ外れた 1 つの読みで、床の上で離陸が完了しないように）ときは
 *         TAKEOFF に留め、（floor_stays のときだけ）TAKEOFF の開始から kAutoTakeoffNoLiftAbortUs 経ったら DISARM で中止する（モータ停止。「ARMED_GROUND へ戻す」にしない:
 *         自動離陸をまた始めるため）。ラッチは、機体が床を離れたことで解ける。中止に至るのと同じ重さの証拠を要する: kTakeoffGroundBandM を超える有効な読みが
 *         kAutoTakeoffFloorHoldUs（1 s）続く、または kTakeoffAirborneBandM を超える有効な読みが 1 つ（TakeoffFloorWatch）。5〜15 cm の地面効果で止まる機体は、したがって永久には保持されない。
 *         解けないラッチ（読みが kTakeoffGroundBandM をまたぎ、どの区間も kAutoTakeoffFloorHoldUs より短いので、中止にも解除にも届かない）は、kAutoTakeoffNoLiftAbortUs が過ぎ、かつ
 *         証拠が「今」無く（floor_stays が偽で floor_seen が真）、機体が今は浮きつつない（帯より上の有効な読みの区間が進行中でない: floor_leaving。ゆっくり浮く機体を、離れつつある途中で着陸させない。
 *         ラッチは、その状態が kAutoTakeoffFloorHoldUs 続くと解ける）とき、TAKEOFF を LANDING で終わらせる: 離陸の推力（ホバー推力の見積もり + 補正）が kAutoTakeoffNoLiftAbortUs の間浮かせなかった機体は、
 *         ホバー推力の見積もりから始まって降りる着陸の推力でも浮かない（SILS。detailed_design.md §3 注8 (3)）。出力停止の ToF や無効サンプルはその証拠ではない:
 *         そういう離陸は ToF 無効の規則（tof_health_tracker.hpp）が LANDING で終わらせる。TAKEOFF でのパイロットの DISARM はモータ停止で、
 *         TAKEOFF に届いた API の `land` は、TAKEOFF を抜けるまで ApiTask が保留する（StateManager::notifyLandingRequest は TAKEOFF を無視する）。
 *  2. decideCommLossLanding(): COMM_LOST の猶予が過ぎたら、TAKEOFF も FLYING も LANDING にする（FLYING だけを対象にすると、完了しない TAKEOFF の間に通信が途絶えたとき
 *     機体が上昇し続ける）。
 *  3. TakeoffFloorWatch: 積極的な証拠の検出。
 *  4. landingVerdictCounts(): ImuTask の接地の判定が、有効な LANDING を終わらせるか（その LANDING の中で出された判定だけが終わらせる）。
 *  5. decideArmedGroundTakeoff() と LiftoffWatch: ARMED_GROUND がどう TAKEOFF に至るか。ALT_HOLD / POS_HOLD: スプールの待ちが過ぎた。ACRO / STABILIZE: スロットルが離陸の閾値を超えた、
 *     または健全な ToF が機体は床を離れたと示す（kTakeoffGroundBandM を超える有効な読みが、途切れずに kAutoTakeoffFloorHoldUs 続く）。スロットルが閾値以下のまま浮く機体（突風・新しい電池・
 *     地面効果）は空中にいて、ARMED_GROUND は床の状態（通信断の着陸も電池の着陸も無く、ToF が出力停止するとモータを止める）。証拠はここでも「積極的な」もの:
 *     出力停止・無効・帯の中の ToF は TAKEOFF を始めさせず、そのときはスロットルだけが決める。
 *  6. decideBatteryEmergency(): 電池の EMERGENCY（電圧が危険閾値未満）が各状態で何をするか。FLYING は着陸し、FLIP は FLYING に戻るまで着陸を保留する。TAKEOFF は、機体が床を「離れた」と示されている
 *     （TakeoffFloorWatch::leaving(): 健全な ToF が kTakeoffGroundBandM を超える有効な高さを今読んでいる）か、ToF が出力停止のときだけ着陸する（LANDING の ToF なしの降下は機体を浮かせられないので、床の上で
 *     失うものも、空中で機体を落とすこともない）。それ以外はすぐモータを止める: 床の上で始めた LANDING は機体を浮かせうる（detailed_design.md §3.1）うえ、無効サンプルを返す ToF（実際の床は不感帯で
 *     無効を返す）や帯の中を読む ToF は、空中にいる証拠ではない。地上と LANDING はログ出力だけ。
 *
 * @design architecture.md INV-3 — decision here, execution in StateTask / StateManager          [OK]
 * @design architecture.md INV-4 — every (state x input) cell is in the table                    [OK]
 * @design detailed_design.md §3.1 — TAKEOFF -> FLYING / IDLE_GROUND / LANDING rows, "TAKEOFF の完了とラッチのセル" table, comm-loss row [OK]
 * @design detailed_design.md §3.1 — ARMED_GROUND -> TAKEOFF rows (spool / throttle / lift-off evidence), "ARMED_GROUND の離陸のセル" table [OK]
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
    Land,          // TAKEOFF -> LANDING (the floor evidence is latched, not there now, and the abort time has passed) / TAKEOFF → LANDING（床の証拠がラッチされ、今は無く、中止の時刻を過ぎた）
};

/// The facts decideTakeoff() needs. decideTakeoff() が要る事実。
struct TakeoffFacts {
    bool     auto_takeoff;        ///< ALT_HOLD / POS_HOLD (ARM-triggered or API) / 自動離陸（ALT_HOLD・POS_HOLD）
    bool     airborne;            ///< system_status.airborne (ToF-based; "armed = not on the ground" while the ToF is silent) / ToF による空中の判定
    bool     controller_reached;  ///< controller_status.takeoff_reached (target captured, or its backstop) / 目標を捕捉（または備え）
    bool     floor_stays;         ///< TakeoffFloorWatch::stays(): the healthy ToF positively shows the craft on the floor NOW (the abort needs this) / ToF が床の上と「今」積極的に示す（中止はこれを要する）
    bool     floor_seen;          ///< TakeoffFloorWatch::seen(): the evidence held once in this TAKEOFF and the craft has not been shown to leave the floor since (the latch; blocks the completion) / 証拠がこの TAKEOFF で一度立ち、以後、機体が床を離れた証拠が来ていない（ラッチ。完了を止める）
    bool     floor_leaving;       ///< TakeoffFloorWatch::leaving(): the healthy ToF reads above the ground band NOW and that run continues (the craft is lifting off; the latch is released after the hold time of it); blocks the LANDING of a latched TAKEOFF / 健全な ToF が接地の帯より上を「今」読み、その区間が続いている（機体は浮きつつある。その保持時間でラッチが解ける）。ラッチ中の TAKEOFF の LANDING を止める
    uint32_t elapsed_us;          ///< time since the TAKEOFF began [us] / TAKEOFF の開始からの時間 [us]
};

/// The decision for the TAKEOFF state (rules in the file comment). TAKEOFF 状態に対する判断（規則はファイルの説明）。
/// @design detailed_design.md §3.1 — TAKEOFF -> FLYING (manual: ToF airborne; auto: captured, no floor evidence, none latched) / TAKEOFF -> IDLE_GROUND (floor evidence 5 s) / TAKEOFF -> LANDING (latch, no evidence now, 5 s); the cells: "TAKEOFF の完了とラッチのセル" [OK]
inline constexpr TakeoffAction decideTakeoff(const TakeoffFacts& facts)
{
    if (!facts.auto_takeoff) {
        return facts.airborne ? TakeoffAction::Complete : TakeoffAction::Continue;
    }
    if (facts.floor_stays && facts.elapsed_us >= limits::kAutoTakeoffNoLiftAbortUs) {
        return TakeoffAction::AbortDisarm;
    }
    if (facts.controller_reached && !facts.floor_stays && !facts.floor_seen) {
        return TakeoffAction::Complete;   // the abort needs 1 s of evidence NOW, so the completion must not be let through by one reading that leaves the band: the latch (released by the craft leaving the floor) / 中止は「今」の 1 s の証拠を要するので、帯を外れた 1 つの読みで完了を通してはならない: ラッチ（機体が床を離れると解ける）
    }
    if (facts.floor_seen && !facts.floor_stays && !facts.floor_leaving && facts.elapsed_us >= limits::kAutoTakeoffNoLiftAbortUs) {
        return TakeoffAction::Land;   // a latch that does not release (the readings straddle the band) and the craft is not lifting off now: the TAKEOFF must end; the landing thrust does not lift a craft that the takeoff thrust did not lift / 解けないラッチ（読みが帯をまたぐ）で、機体が今は浮きつつない: TAKEOFF は終わらなければならない。離陸の推力で浮かなかった機体は、着陸の推力でも浮かない
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

/// What StateManager does with a battery EMERGENCY alert. 電池の EMERGENCY アラートに StateManager がすること。
enum class BatteryEmergencyAction : uint8_t {
    LogOnly,     // no transition: LANDING already, or on the ground (the alert is re-raised while the voltage stays low) / 遷移なし: 既に LANDING、または地上（電圧が低いままなら再発報される）
    Defer,       // FLIP: set the pending flag; the landing follows FLIP -> FLYING / FLIP: 保留フラグを立てる。着陸は FLIP → FLYING の後
    Land,        // -> LANDING (the craft is, or may be, in the air) / LANDING（機体は空中にいる、またはいる可能性がある）
    CutMotors,   // TAKEOFF without evidence of the air -> IDLE_GROUND by DISARM / 空中の証拠が無い TAKEOFF → DISARM で IDLE_GROUND
};

/// The facts decideBatteryEmergency() needs. decideBatteryEmergency() が要る事実。
struct BatteryEmergencyFacts {
    FlightState state;          ///< the flight state / 飛行状態
    bool        tof_silent;     ///< the downward-ToF silence verdict (failsafe_status.tof_silent) / 下向き ToF の出力停止の判定
    bool        floor_leaving;  ///< TakeoffFloorWatch::leaving() of a healthy ToF: the craft is off the floor now (tof_health.floor_leaving) / 健全な ToF による TakeoffFloorWatch::leaving(): 機体は今、床を離れている
};

/// The response to a battery EMERGENCY (rules in the file comment, item 6). Evidence of the air is POSITIVE (the healthy ToF reads valid heights above the ground band now); its absence is not evidence of
/// the floor either, so a silent ToF lands: the blind descent starts below the supporting thrust and cannot lift the craft, on the floor or in the air. A ToF that is healthy but has shown no height above the band
/// (invalid in the dead zone of the real floor, or in the band) cuts the motors: a LANDING there can float a craft that is on the floor (detailed_design.md §3.1).
/// 電池の EMERGENCY への応答（規則はファイルの説明の項目 6）。空中の証拠は「積極的な」もの（健全な ToF が接地の帯より上の有効な高さを今読む）。その不在は床の証拠でもないので、出力停止の ToF は着陸する:
/// ToF なしの降下は支持推力より下から始まり、床の上でも空中でも機体を浮かせられない。健全だが帯より上の高さを示していない ToF（実際の床の不感帯で無効、または帯の中）はモータを止める:
/// そこでの LANDING は、床の上の機体を浮かせうる（detailed_design.md §3.1）。
/// @design detailed_design.md §3.1 — battery EMERGENCY rows (TAKEOFF: evidence of the air -> LANDING, else DISARM; FLYING -> LANDING; FLIP deferred; LANDING / ground log only) [OK]
inline constexpr BatteryEmergencyAction decideBatteryEmergency(const BatteryEmergencyFacts& facts)
{
    switch (facts.state) {
    case FlightState::FLIP:
        return BatteryEmergencyAction::Defer;
    case FlightState::FLYING:
        return BatteryEmergencyAction::Land;
    case FlightState::TAKEOFF:
        return (facts.floor_leaving || facts.tof_silent) ? BatteryEmergencyAction::Land : BatteryEmergencyAction::CutMotors;
    default:
        return BatteryEmergencyAction::LogOnly;   // LANDING, ARMED_GROUND, IDLE_GROUND, IDLE_HELD, INIT / LANDING・ARMED_GROUND・IDLE_GROUND・IDLE_HELD・INIT
    }
}

/// True when the touchdown verdict of ImuTask (system_status.landing) ends the LANDING in force: only a verdict that ImuTask made WHILE this LANDING was in force counts. ImuTask stamps every status with the
/// SystemMode.state_since_us it had read (system_status.verdict_state_since_us); the stamp must equal the one of the present state. A verdict left over from before (the ground flag keeps its value between
/// kTakeoffGroundBandM and kTakeoffAirborneBandM) is stamped with the earlier state, so no cycle is left in which it could stop the motors about 0.1 m above the floor in the cycle the LANDING began.
/// The equality needs no clock: a stamp is the state's entry time, and a state entered again has a new one.
/// ImuTask の接地の判定（system_status.landing）が有効な LANDING を終わらせるなら true: ImuTask が、この LANDING が有効だった「間」に出した判定だけを数える。ImuTask は全ての status に、読んだ
/// SystemMode.state_since_us を刻む（system_status.verdict_state_since_us）。その刻印が今の状態のものと等しくなければならない。以前から残った判定（接地フラグは kTakeoffGroundBandM と
/// kTakeoffAirborneBandM の間で値を保つ）には前の状態の刻印が付くので、LANDING が始まった周期に、床の約 0.1 m 上でモータを止めてしまう周期は残らない。等しいかだけで比べるので時計は要らない
/// （刻印は状態に入った時刻で、もう一度入った状態は新しい時刻を持つ）。
/// @design detailed_design.md §3 注7 — a landing verdict of before the LANDING is no evidence of a touchdown in it (TakeoffLandingMgr::noteLandingDescent + this stamp) [OK]
inline constexpr bool landingVerdictCounts(bool verdict_landing, uint32_t verdict_state_since_us, uint32_t state_since_us)
{
    return verdict_landing && verdict_state_since_us == state_since_us;
}

/// Detects the POSITIVE evidence that an auto-takeoff is still on the floor: the healthy ToF reads a valid height at or below kTakeoffGroundBandM (the boundary belongs to the band; the
/// manager's ground flag needs STRICTLY below, flight_limits.hpp), and that reading has not risen by more than kAutoTakeoffFloorRiseM above the lowest of the run, for kAutoTakeoffFloorHoldUs. A reading that rises restarts the run; a
/// reading above the band, or a ToF that is not healthy, ends it; an INVALID sample is neutral (it neither proves nor disproves: the real floor reads invalid
/// in the dead zone). Two outputs: stays() is the evidence NOW (it is what the abort needs); seen() is the LATCH of it, set when the evidence has held, so that the evidence ending by one stray reading outside the band does not
/// let the takeoff complete on the floor. The latch is RELEASED by the craft leaving the floor, with the weight of evidence that the abort has: valid readings above kTakeoffGroundBandM that last kAutoTakeoffFloorHoldUs (a
/// run of the other side, ended by an in-band reading or a silent ToF; an invalid sample is neutral there too), or one valid reading above kTakeoffAirborneBandM. ImuTask owns it (INV-3: detection there, decision in StateTask):
/// ONE call per NEW ToF sample while in TAKEOFF, reset() for every NEW ToF sample taken outside TAKEOFF; the facts go out as tof_health.floor_stays / floor_seen.
/// 自動離陸がまだ床の上にいるという「積極的な」証拠を検出する: 健全な ToF が kTakeoffGroundBandM 以下（境界は帯に含む。マネージャの接地フラグは「未満」を要する。flight_limits.hpp）の有効な高さを読み、その読みが、連続区間の最低値より
/// kAutoTakeoffFloorRiseM を超えて増えていないことが kAutoTakeoffFloorHoldUs 続く。読みが増えたら区間を数え直し、帯より上の読みや健全でない ToF は区間を終わらせ、
/// 「無効」サンプルは中立（証明も反証もしない: 実際の床は不感帯で無効を返す）。出力は 2 つ: stays() は「今」の証拠（中止が要るのはこちら）、seen() はそのラッチで、証拠が立ったら立つ。
/// 証拠が帯の外へ外れた 1 つの読みで途切れても、床の上で離陸が完了しないようにするため。ラッチは、機体が床を離れたことで解ける。中止と同じ重さの証拠を要する: kTakeoffGroundBandM を超える有効な読みが
/// kAutoTakeoffFloorHoldUs 続く（反対側の区間。帯の中の読みか出力停止の ToF で終わり、無効サンプルはここでも中立）、または kTakeoffAirborneBandM を超える有効な読みが 1 つ。ImuTask が持つ（INV-3: 検出はそこ、判断は StateTask）:
/// TAKEOFF の間は新しい ToF サンプルごとに 1 回呼び、TAKEOFF の外では新しい ToF サンプルごとに reset() する。事実は tof_health.floor_stays / floor_seen として発行する。
/// @design detailed_design.md §3.1 — the evidence, its latch and the release of the latch ("TAKEOFF の完了とラッチのセル") [OK]
class TakeoffFloorWatch {
public:
    /// Forget everything: both runs and the latch. ImuTask calls it for EVERY ToF sample taken outside TAKEOFF (the same cadence as onSample() inside it), so a TAKEOFF always starts clean.
    /// 全てを忘れる（2 つの区間もラッチも）。ImuTask は TAKEOFF の「外」で取った ToF サンプルごとに呼ぶ（中の onSample() と同じ頻度）ので、TAKEOFF は必ず白紙から始まる。
    void reset()
    {
        run_active_ = false;
        run_since_us_ = 0;
        lowest_m_ = 0.0f;
        above_active_ = false;
        above_since_us_ = 0;
        seen_ = false;
    }

    /// One NEW ToF sample. 新しい ToF サンプル 1 つ。
    /// @param now_us      Time of the sample [us]          サンプルの時刻 [us]
    /// @param tof_healthy The ToF is not silent           ToF が出力停止でない
    /// @param valid       The sample is a valid range      サンプルが有効な測距
    /// @param distance_m  Measured height [m]              測った高さ [m]
    void onSample(uint32_t now_us, bool tof_healthy, bool valid, float distance_m)
    {
        if (!tof_healthy) {
            run_active_   = false;   // the latch stays: a silent ToF proves neither the floor nor the lift-off / ラッチは残す: 出力停止の ToF は床も離陸も証明しない
            above_active_ = false;
            return;
        }
        latchIfStays(now_us);   // the evidence that has held until now is latched whatever this sample is (an invalid sample too: the clock runs) / 今までに続いた証拠は、このサンプルが何であれラッチする（無効サンプルでも: 時計は進む）
        if (!valid) {
            return;   // neutral: an invalid sample is no evidence either way / 中立: 無効サンプルはどちらの証拠でもない
        }
        if (distance_m > limits::kTakeoffGroundBandM) {
            run_active_ = false;   // above the band: not (now) on the floor / 帯より上: （今は）床の上ではない
            trackAboveRun(now_us, distance_m);
            return;
        }
        above_active_ = false;   // an in-band reading ends the run above the band / 帯の中の読みは、帯より上の区間を終わらせる
        trackRun(now_us, distance_m);
        latchIfStays(now_us);
    }

    /// The evidence has held for kAutoTakeoffFloorHoldUs and the ToF is healthy now. 証拠が kAutoTakeoffFloorHoldUs 続き、かつ今 ToF が健全。
    bool stays(uint32_t now_us, bool tof_healthy) const
    {
        const uint32_t held_us = static_cast<uint32_t>(now_us - run_since_us_);   // unsigned difference: correct across a wrap / 符号なしの差: 巻き戻りをまたいでも正しい
        return tof_healthy && run_active_ && held_us >= limits::kAutoTakeoffFloorHoldUs;
    }

    /// The craft is lifting off NOW: the last valid readings are above kTakeoffGroundBandM and that run is in progress (a run above the band ends with an in-band reading, a silent ToF or reset(); an invalid sample is neutral).
    /// 機体は「今」浮きつつある: 直近の有効な読みが kTakeoffGroundBandM より上で、その区間が進行中（帯より上の区間は、帯の中の読み・出力停止の ToF・reset() で終わる。無効サンプルは中立）。
    bool leaving() const { return above_active_; }

    /// The latch: the evidence has held in this TAKEOFF and the craft has not been shown to leave the floor since (see the class comment). 証拠がこの TAKEOFF で一度立ち、以後、機体が床を離れた証拠が来ていない（クラスの説明）。
    bool seen() const { return seen_; }

private:
    /// Latch the evidence the moment stays() is true. 証拠が stays() になった時点でラッチする。
    void latchIfStays(uint32_t now_us)
    {
        if (stays(now_us, true)) {
            seen_ = true;
        }
    }

    /// A valid in-band reading: start a run, or follow it (a rise beyond kAutoTakeoffFloorRiseM above the lowest starts a new one). 帯の中の有効な読み: 区間を始める、または追う（最低値より kAutoTakeoffFloorRiseM を超える上昇は新しい区間）。
    void trackRun(uint32_t now_us, float distance_m)
    {
        if (!run_active_ || distance_m > lowest_m_ + limits::kAutoTakeoffFloorRiseM) {
            run_active_   = true;   // a new run: the first reading, or the reading has risen / 新しい区間: 最初の読み、または読みが増えた
            run_since_us_ = now_us;
            lowest_m_     = distance_m;
        } else if (distance_m < lowest_m_) {
            lowest_m_ = distance_m;
        }
    }

    /// A valid reading above kTakeoffGroundBandM: the craft is leaving the floor. One above kTakeoffAirborneBandM releases the latch at once; readings above the ground band last for kAutoTakeoffFloorHoldUs release it too.
    /// 帯（kTakeoffGroundBandM）より上の有効な読み: 機体は床を離れつつある。kTakeoffAirborneBandM を超える 1 つが、ラッチをすぐ解く。帯より上の読みが kAutoTakeoffFloorHoldUs 続いても解く。
    /// A single reading above kTakeoffAirborneBandM releasing the latch is an INTENDED decision. If one stray reflection on the floor does it and the TAKEOFF completes, the craft is then under the ordinary altitude
    /// control, which only climbs toward the target; that does not stand in the way of ending a craft that cannot lift (its readings stay on the floor, and the floor evidence is built again by them).
    /// 15 cm を超える有効な読み 1 つでラッチが解けるのは、意図した判断。床の上の 1 回の誤反射でそうなって TAKEOFF が完了しても、その後は通常の高度制御で目標高度へ上がるだけで、浮けない機体の
    /// 中止を妨げない（浮けない機体の読みは床の上のままで、床の証拠はその読みで立て直される）。
    /// @design detailed_design.md §3.1 — the release of the latch by one reading above kTakeoffAirborneBandM ("TAKEOFF の完了とラッチのセル") [OK]
    void trackAboveRun(uint32_t now_us, float distance_m)
    {
        if (distance_m > limits::kTakeoffAirborneBandM) {
            seen_ = false;   // a valid reading above the airborne band: the craft has left the floor / 空中の帯より上の有効な読み: 機体は床を離れた
        }
        if (!above_active_) {
            above_active_   = true;
            above_since_us_ = now_us;
        }
        const uint32_t above_us = static_cast<uint32_t>(now_us - above_since_us_);   // unsigned difference: correct across a wrap / 符号なしの差: 巻き戻りをまたいでも正しい
        if (above_us >= limits::kAutoTakeoffFloorHoldUs) {
            seen_ = false;   // 1 s above the ground band: as much evidence of leaving as the abort needs of staying / 帯より上に 1 s: 中止が「留まる」に要るのと同じ重さの「離れた」証拠
        }
    }

    bool     run_active_    = false;   // a run of in-band readings is in progress / 帯の中の読みの連続区間が進行中
    uint32_t run_since_us_  = 0;       // start of the run [us] / 区間の開始 [us]
    float    lowest_m_      = 0.0f;    // lowest reading of the run [m] / 区間の最低の読み [m]
    bool     above_active_  = false;   // a run of valid readings above the ground band is in progress / 帯より上の有効な読みの連続区間が進行中
    uint32_t above_since_us_ = 0;      // start of that run [us] / その区間の開始 [us]
    bool     seen_          = false;   // the latch (see seen()) / ラッチ（seen() 参照）
};


/// The facts decideArmedGroundTakeoff() needs. decideArmedGroundTakeoff() が要る事実。
struct ArmedGroundFacts {
    bool auto_takeoff;              ///< ALT_HOLD / POS_HOLD: ARM itself starts the takeoff / ARM 自体が離陸を始める
    bool spool_elapsed;             ///< the spool/settle dwell of ARMED_GROUND has elapsed / ARMED_GROUND のスプール・整定の待ちが過ぎた
    bool throttle_past_threshold;   ///< the throttle is above the manual takeoff threshold / スロットルが手動離陸の閾値より上
    bool lifted_off;                ///< LiftoffWatch::lifted() of a healthy ToF: the craft is off the floor / 健全な ToF による LiftoffWatch::lifted(): 機体は床を離れた
};

/// True when ARMED_GROUND must become TAKEOFF this cycle. Auto modes: the spool dwell has elapsed. Manual modes: the throttle is past the threshold, or the healthy ToF shows
/// the craft off the floor (a craft that rises at or below the threshold is in the air: it must not stay in a state that has no comm-loss or battery landing and cuts the motors
/// when the ToF falls silent). The ToF evidence only ADDS to the throttle: without it (silent, invalid, in the band) the throttle decides alone.
/// この周期に ARMED_GROUND を TAKEOFF にすべきなら true。自動のモード: スプールの待ちが過ぎた。手動のモード: スロットルが閾値を超えた、または健全な ToF が機体は床を離れたと示す
/// （閾値以下で浮く機体は空中にいる。通信断の着陸も電池の着陸も無く、ToF が出力停止するとモータを止める状態に留まってはならない）。ToF の証拠はスロットルに「足す」だけ:
/// 無い（出力停止・無効・帯の中）ときはスロットルだけが決める。
/// @design detailed_design.md §3.1 — ARMED_GROUND -> TAKEOFF (auto: spool; manual: throttle, or the ToF shows the craft off the floor); the cells: "ARMED_GROUND の離陸のセル" [OK]
inline constexpr bool decideArmedGroundTakeoff(const ArmedGroundFacts& facts)
{
    if (facts.auto_takeoff) {
        return facts.spool_elapsed;
    }
    return facts.throttle_past_threshold || facts.lifted_off;
}

/// Detects, in ARMED_GROUND, the POSITIVE evidence that the craft is off the floor: the healthy ToF reads valid heights above kTakeoffGroundBandM for kAutoTakeoffFloorHoldUs without a break (the same
/// weight of evidence as the one that releases the latch of TakeoffFloorWatch, whose run it is). Unlike that run, an INVALID sample ends it too: the real floor reads invalid in the dead zone, so
/// two stray readings a second apart with invalid samples between them would otherwise prove a lift-off, and here the evidence starts a TAKEOFF (there it only releases a latch). A flying craft reads
/// valid heights on every sample (three real flight logs: no invalid sample in flight). An in-band reading, an invalid sample, a silent ToF or reset() ends the run. lifted() is a LATCH, so that the 30 Hz ToF
/// samples and the StateTask cycle need no common clock; reset() clears it. ImuTask owns it (INV-3): ONE call per NEW ToF sample while in ARMED_GROUND, reset() for every NEW ToF sample taken
/// outside it; the fact goes out as tof_health.lifted_off.
/// ARMED_GROUND で、機体が床を離れたという「積極的な」証拠を検出する: 健全な ToF が kTakeoffGroundBandM を超える有効な高さを、途切れずに kAutoTakeoffFloorHoldUs 読む（TakeoffFloorWatch のラッチを
/// 解くのと同じ重さの証拠で、その区間と同じもの）。その区間と違い、「無効」サンプルも区間を終わらせる: 実際の床は不感帯で無効を返すので、無効サンプルを挟んで 1 s 離れた 2 つの外れ値が
/// 浮上の証拠になってしまう（そちらはラッチを解くだけ、こちらは TAKEOFF を始める）。飛んでいる機体は毎サンプル有効な高さを読む（実機ログ: 飛行中の無効サンプルは無い）。帯の中の読み・無効サンプル・
/// 出力停止の ToF・reset() が区間を終わらせる。lifted() はラッチで、30 Hz の ToF サンプルと StateTask の周期が共通の時計を要らないようにする。reset() で消える。ImuTask が持つ（INV-3）:
/// ARMED_GROUND の間は新しい ToF サンプルごとに 1 回呼び、その外では新しい ToF サンプルごとに reset() する。事実は tof_health.lifted_off として発行する。
/// @design detailed_design.md §3.1 — the lift-off evidence of ARMED_GROUND ("ARMED_GROUND の離陸のセル") [OK]
class LiftoffWatch {
public:
    /// Forget the run and the latch. ImuTask calls it for EVERY ToF sample taken outside ARMED_GROUND, so an ARMED_GROUND always starts clean.
    /// 区間とラッチを忘れる。ImuTask は ARMED_GROUND の「外」で取った ToF サンプルごとに呼ぶので、ARMED_GROUND は必ず白紙から始まる。
    void reset()
    {
        run_active_   = false;
        run_since_us_ = 0;
        lifted_       = false;
    }

    /// One NEW ToF sample. 新しい ToF サンプル 1 つ。
    /// @param now_us      Time of the sample [us]          サンプルの時刻 [us]
    /// @param tof_healthy The ToF is not silent           ToF が出力停止でない
    /// @param valid       The sample is a valid range      サンプルが有効な測距
    /// @param distance_m  Measured height [m]              測った高さ [m]
    void onSample(uint32_t now_us, bool tof_healthy, bool valid, float distance_m)
    {
        if (!tof_healthy || !valid || distance_m <= limits::kTakeoffGroundBandM) {
            run_active_ = false;   // no evidence in this sample: the run is broken (the latch stays) / このサンプルに証拠は無い: 区間は途切れる（ラッチは残る）
            return;
        }
        if (!run_active_) {
            run_active_   = true;
            run_since_us_ = now_us;
        }
        const uint32_t held_us = static_cast<uint32_t>(now_us - run_since_us_);   // unsigned difference: correct across a wrap / 符号なしの差: 巻き戻りをまたいでも正しい
        if (held_us >= limits::kAutoTakeoffFloorHoldUs) {
            lifted_ = true;
        }
    }

    /// The craft has been shown off the floor since the last reset(). 直近の reset() 以後、機体が床を離れたと示された。
    bool lifted() const { return lifted_; }

private:
    bool     run_active_   = false;   // a run of valid readings above the ground band is in progress / 帯より上の有効な読みの連続区間が進行中
    uint32_t run_since_us_ = 0;       // start of the run [us] / 区間の開始 [us]
    bool     lifted_       = false;   // the latch (see lifted()) / ラッチ（lifted() 参照）
};

}  // namespace sf
