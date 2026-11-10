/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tof_silence_policy.hpp
 * @brief What the state machine does when the downward ToF goes silent (pure decision)
 *        下向き ToF が出力停止したとき状態機械が何をするか（純粋な判断関数）
 *
 * "Silent" = a ToF-silence EPISODE is in progress (failsafe_status.tof_silent). It starts when the
 * sensor produced no NEW sample for sf::limits::kTofSilenceUs, or when only INVALID samples arrived
 * for kTofInvalidPersistUs while flying (a floor the ToF cannot range), and it ends only when the
 * estimator has RE-ACQUIRED the ToF (tof_health_tracker.hpp): samples that merely arrive prove
 * nothing. Failsafe judges it (sf::SilenceWatch, sensor_freshness.hpp), publishes the level on
 * failsafe_status and raises AlertType::TOF_SILENT, exactly like the comm-loss failsafe;
 * StateManager executes the response decided here. A single "out of range" sample is not an
 * episode (handled later together with the barometer) and never reaches this function. Readers
 * pass the status through sf::FailsafeStatusGuard: a status nobody refreshes reads as silent.
 *
 * Why this matters: the ground-contact flag (TakeoffLandingMgr::on_ground_) is released by ToF
 * samples only. When the ToF stops, the flag freezes; if it froze "on the ground" the estimator
 * kept pinning position/velocity to zero while the craft climbed, and the auto-takeoff kept
 * integrating thrust (2026-11 accident, craft hit the ceiling). Since then ImuTask no longer
 * trusts the frozen flag while the ToF is silent, the landing descends blind, and the response
 * depends on whether the flight mode needs altitude:
 *  - ALT_HOLD / POS_HOLD / auto-takeoff NEED the altitude  -> land (blind descent).
 *  - ACRO / STABILIZE do not (the pilot owns the vertical axis) -> NEVER force a landing:
 *    warn (LED + tone) and let the pilot fly on; the next ARM is refused while silent.
 *
 * The decision is a pure function of (state, mode, silent, blind-landing time) so every cell of
 * the normative table (detailed_design.md §3.1, INV-4) can be checked on the host
 * (test/test_tof_silence.cpp). StateManager executes the result (sole transition owner, INV-3);
 * this header only DECIDES.
 *
 * 「出力停止」= ToF 出力停止の「エピソード」が進行中（failsafe_status.tof_silent）。センサが
 * sf::limits::kTofSilenceUs の間新しいサンプルを出さなかったとき、または飛行中に「無効」サンプルだけが
 * kTofInvalidPersistUs 続いたとき（ToF が測れない床）に始まり、推定器が ToF を「取り込み直した」
 * （tof_health_tracker.hpp）ときだけ終わる: サンプルが「届いた」だけでは何も保証されない。判定は
 * Failsafe が行い（sf::SilenceWatch、sensor_freshness.hpp）、レベルを failsafe_status に発行し、
 * 通信断のフェイルセーフと同じ形で AlertType::TOF_SILENT を発報する。ここで決めた応答を
 * StateManager が実行する。単発の「範囲外」サンプルはエピソードではなく（後段で気圧計と一緒に扱う）、
 * この関数には届かない。読む側は sf::FailsafeStatusGuard を通す: 誰も更新しない状態は出力停止と読む。
 *
 * なぜ重要か: 接地フラグ（TakeoffLandingMgr::on_ground_）は ToF サンプルだけで解除される。ToF が
 * 止まるとフラグは最後の値のまま固定され、「接地」のまま固定されると、機体が上昇しても推定器が位置・速度を 0 に固定
 * し続け、自動離陸は推力を積分し続けた（2026-11 の事故。天井に衝突）。以後、ToF の出力停止中は
 * ImuTask が固定されたままのフラグを信用せず、着陸は ToF なしで降り、応答は飛行モードが高度を必要とするか
 * で分かれる:
 *  - ALT_HOLD・POS_HOLD・自動離陸は高度を必要とする → 着陸（ToF なしの降下）。
 *  - ACRO・STABILIZE は必要としない（鉛直軸はパイロットのもの）→ 強制着陸は「しない」:
 *    警告（LED + 音）を出し、パイロットは飛び続けられる。出力停止中は次の ARM を拒否する。
 *
 * 判断は (状態, モード, 出力停止, ToF なしの着陸時間) の純関数なので、規範表（detailed_design.md §3.1、
 * INV-4）の全セルをホストで検証できる（test/test_tof_silence.cpp）。結果の実行は StateManager
 * （遷移の唯一の実行者、INV-3）が行い、このヘッダは「判断」だけをする。
 *
 * @design architecture.md §4 — ToF silence: Failsafe detects, StateManager transitions (R16) [OK]
 * @design architecture.md INV-3 — decision here, execution in StateManager                   [OK]
 * @design architecture.md INV-4 — every (state x mode x input) cell is in the table          [OK]
 * @design detailed_design.md §3.1 — "ToF silent" rows                                        [OK]
 * @design coding_and_education.md §2 — Bilingual comments                                    [OK]
 */

#pragma once

#include <cstdint>

#include "flight_limits.hpp"
#include "flight_state.hpp"

namespace sf {

/// Action the state machine takes on the current ToF-silence verdict.
/// 現在の ToF 出力停止の判定に対して状態機械が取る動作。
enum class ToFSilenceAction : uint8_t {
    None,              // nothing to do / 何もしない
    Disarm,            // ARMED_GROUND -> IDLE_GROUND (never launch blind) / 見えないまま離陸しない
    StartLanding,      // TAKEOFF or FLYING (altitude mode) -> LANDING (blind descent) / ToF なしの降下で着陸
    WarnOnly,          // ACRO/STABILIZE flight: LED + tone, NO forced landing / 警告のみ（強制着陸しない）
    CompleteLanding,   // LANDING -> IDLE_GROUND after the blind-landing time / ToF なしの着陸の時間切れ
};

/// True for the modes whose control law needs the altitude estimate (ALT_HOLD, POS_HOLD). ACRO
/// and STABILIZE give the pilot the vertical axis directly. Same ordering the StateManager uses
/// for the auto-land rule (mode >= ALT_HOLD).
/// 制御則が高度推定を必要とするモード（ALT_HOLD・POS_HOLD）なら true。ACRO・STABILIZE は鉛直軸を
/// パイロットが直接握る。StateManager の自動着陸規則（mode >= ALT_HOLD）と同じ順序。
inline constexpr bool flightModeNeedsAltitude(FlightMode mode)
{
    return mode >= FlightMode::ALT_HOLD;
}

/// True when a request to switch INTO `requested` must be refused because the downward ToF is
/// silent: the altitude modes would fly on an altitude nobody can see. Leaving them (to ACRO or
/// STABILIZE) is never refused.
/// 下向き ToF が出力停止しているために、`requested` への切り替え要求を拒否すべきなら true:
/// 高度モードは誰も見られない高度で飛ぶことになる。そこから出る方向（ACRO・STABILIZE へ）は拒否しない。
inline constexpr bool toFSilenceRefusesMode(FlightMode requested, bool silent)
{
    return silent && flightModeNeedsAltitude(requested);
}

/// How a request to switch INTO `requested` is answered while a ToF-silence episode is in progress.
/// 下向き ToF の出力停止エピソード中に、`requested` への切り替え要求にどう答えるか。
enum class ToFModeRefusal : uint8_t {
    None,       // not refused / 拒否しない
    Keep,       // refused on the GROUND: the pilot's switch edge persists and applies by itself
                // when the episode ends (changing the mode while parked is the safest time)
                // 地上で拒否: パイロットのスイッチのエッジは持続し、エピソードが終わると自動で
                // 適用される（設置中のモード変更が最も安全）
    Consume,    // refused IN FLIGHT: the edge is CONSUMED. The mode must not switch by itself at the
                // instant the ToF recovers (the pilot may have moved on); the pilot flips the switch
                // again. Shown in the log and on CLI `status` (ModeBlock::TofSwitchConsumed).
                // 飛行中に拒否: エッジは「消費」する。ToF が復帰した瞬間にモードがパイロットの操作なしに切り替わっては
                // ならない（パイロットはもう別のことをしているかもしれない）。パイロットがスイッチを
                // 入れ直す。ログと CLI `status` に出す（ModeBlock::TofSwitchConsumed）。
};

/// The refusal for (state, requested mode, silent). Only altitude modes are ever refused.
/// (状態、要求されたモード、出力停止) に対する拒否。拒否されるのは高度モードだけ。
inline constexpr ToFModeRefusal decideToFModeRefusal(FlightState state, FlightMode requested, bool silent)
{
    if (!toFSilenceRefusesMode(requested, silent)) {
        return ToFModeRefusal::None;
    }
    return (state == FlightState::FLYING) ? ToFModeRefusal::Consume : ToFModeRefusal::Keep;
}

/// True when ARM must be refused: during an episode in EVERY mode, ACRO included (the craft may be
/// flown manually, but not launched while its altitude sensor is unvouched for).
/// ARM を拒否すべきなら true: エピソード中は ACRO を含む「全モード」（手動で飛ばせても、高度センサが
/// 保証されないまま離陸させない）。
inline constexpr bool toFSilenceRefusesArm(bool silent)
{
    return silent;
}

/// Decide the action for (state, mode, silent). `blind_landing_elapsed_us` is how long the state has been
/// LANDING, counted from the moment it entered LANDING (not from the silence: a ToF that returns and
/// stops again does not extend it; 0 when not applicable). `silent_elapsed_us` is how long the ToF has been
/// continuously silent in this LANDING (the descent without a ToF; 0 when not applicable): the landing is
/// completed by time only after BOTH have passed their limits (kBlindLandingTimeoutUs, kBlindLandingMinSilentUs).
/// (状態, モード, 出力停止) に対する動作を判断する。`blind_landing_elapsed_us` は状態が LANDING に
/// 入った時刻から数えた LANDING の経過時間（出力停止の時刻からではない: ToF が戻ってまた止まっても
/// 延びない。該当しなければ 0）。`silent_elapsed_us` はこの LANDING の中で ToF が連続して出力停止している時間
/// （ToF なしの降下。該当しなければ 0）: 時間による着陸完了は、「両方」が上限（kBlindLandingTimeoutUs、
/// kBlindLandingMinSilentUs）を過ぎた後でだけ。
///
/// Rows (detailed_design.md §3.1):
///  - ARMED_GROUND : Disarm (any mode).
///  - TAKEOFF, FLYING : StartLanding in ALT_HOLD / POS_HOLD (and the auto-takeoff, which only
///              exists in those modes); WarnOnly in ACRO / STABILIZE.
///  - LANDING : CompleteLanding once blind_landing_elapsed_us >= kBlindLandingTimeoutUs AND
///              silent_elapsed_us >= kBlindLandingMinSilentUs, because without ToF no touchdown can be detected
///              by height (the second condition: a ToF that dies after a long LANDING gets the minimum time of a
///              descent without a ToF before the motors are cut in the air).
///  - FLIP : None. Interrupting the maneuver would only add a second sequence owner. The response
///           is applied AT ONCE when FLIP returns to FLYING: StateManager::notifyFlipComplete()
///           looks at the standing level and decides on the spot, it does not wait for the next
///           re-raised alert (which cost up to ~1 s of ALT_HOLD flown on the estimate).
///  - INIT / IDLE_GROUND / IDLE_HELD : None here. The ARM refusal (ArmBlock::TofSilent) is the
///           pre-arm gate in StateManager::evaluateArmBlock, and the refusal of the altitude
///           modes is StateManager::requestModeChange (ModeBlock::TofSilent).
/// 行（detailed_design.md §3.1）:
///  - ARMED_GROUND: Disarm（モードを問わない）。
///  - TAKEOFF・FLYING: ALT_HOLD・POS_HOLD（と、それらにしかない自動離陸）では StartLanding、
///             ACRO・STABILIZE では WarnOnly。
///  - LANDING: blind_landing_elapsed_us >= kBlindLandingTimeoutUs かつ silent_elapsed_us >= kBlindLandingMinSilentUs で
///             CompleteLanding。ToF が無いと高さで接地を検出できないため（2 つ目の条件: 長い LANDING の後に ToF が止まった
///             ときも、ToF なしの降下の最低時間を与えてから、空中でモータを切る）。
///  - FLIP: None。マニューバを中断すると、シーケンスの所有者が 2 つになるだけ。応答は FLIP が
///          FLYING に戻った「その場で」適用する: StateManager::notifyFlipComplete() が継続状態
///          （レベル）を見てその場で判断し、次に再発報されるアラートを待たない（待つと、推定だけで
///          飛ぶ ALT_HOLD が最大約 1 s 続いた）。
///  - INIT・IDLE_GROUND・IDLE_HELD: ここでは None。ARM 拒否（ArmBlock::TofSilent）は
///          StateManager::evaluateArmBlock の ARM 前判定が、高度モードの拒否は
///          StateManager::requestModeChange（ModeBlock::TofSilent）が担う。
inline constexpr ToFSilenceAction decideToFSilenceAction(FlightState state, FlightMode mode,
                                                         bool silent,
                                                         uint32_t blind_landing_elapsed_us,
                                                         uint32_t silent_elapsed_us)
{
    if (!silent) {
        return ToFSilenceAction::None;
    }
    switch (state) {
        case FlightState::ARMED_GROUND:
            return ToFSilenceAction::Disarm;
        case FlightState::TAKEOFF:
        case FlightState::FLYING:
            return flightModeNeedsAltitude(mode) ? ToFSilenceAction::StartLanding
                                                 : ToFSilenceAction::WarnOnly;
        case FlightState::LANDING:
            return (blind_landing_elapsed_us >= limits::kBlindLandingTimeoutUs &&
                    silent_elapsed_us >= limits::kBlindLandingMinSilentUs)
                       ? ToFSilenceAction::CompleteLanding
                       : ToFSilenceAction::None;
        default:
            return ToFSilenceAction::None;
    }
}

/// Time since the state ENTERED LANDING, whether the downward ToF is alive or silent: the clock of the landing time limit
/// (kBlindLandingTimeoutUs). It restarts only when LANDING has ended, so a ToF that returns and stops again does not extend it.
/// It also counts the time the ToF has been continuously silent in this LANDING (the descent without a ToF): the minimum time that a
/// ToF that dies after a long LANDING gets before the motors are cut (kBlindLandingMinSilentUs).
/// 状態が LANDING に「入った」時刻からの経過時間（下向き ToF が動作していても出力停止でも）: 着陸の時間切れ
/// （kBlindLandingTimeoutUs）の時計。LANDING が終わったときだけ数え直すので、ToF が戻ってまた止まっても延びない。
/// この LANDING の中で ToF が連続して出力停止している時間（ToF なしの降下）も数える: 長い LANDING の後に ToF が止まったときに、
/// モータを切る前に与える最低時間（kBlindLandingMinSilentUs）。
class LandingClock {
public:
    /// Observe the state once per update. 更新ごとに 1 回、状態を観測する。
    /// @param landing Whether the state is LANDING now      今 LANDING か
    /// @param silent  Whether the downward ToF is silent now 今 下向き ToF が出力停止か
    /// @param now_us  Current time [us]                    現在時刻 [us]
    void update(bool landing, bool silent, uint32_t now_us)
    {
        if (landing && !running_) {
            running_  = true;
            since_us_ = now_us;
        } else if (!landing) {
            running_ = false;
        }
        elapsed_us_ = running_ ? (now_us - since_us_) : 0u;   // unsigned difference: correct across a wrap / 符号なしの差: 巻き戻りをまたいでも正しい
        if (running_ && silent && !silent_running_) {
            silent_running_  = true;
            silent_since_us_ = now_us;
        } else if (!(running_ && silent)) {
            silent_running_ = false;
        }
        silent_elapsed_us_ = silent_running_ ? (now_us - silent_since_us_) : 0u;
    }
    uint32_t elapsedUs() const { return elapsed_us_; }   ///< Time in LANDING [us], 0 when not landing / LANDING の経過 [us]。着陸中でなければ 0
    uint32_t silentElapsedUs() const { return silent_elapsed_us_; }   ///< Continuous silence in this LANDING [us], 0 when the ToF is alive / この LANDING での連続した出力停止 [us]。ToF が動作していれば 0

private:
    bool     running_    = false;
    uint32_t since_us_   = 0;
    uint32_t elapsed_us_ = 0;
    bool     silent_running_    = false;
    uint32_t silent_since_us_   = 0;
    uint32_t silent_elapsed_us_ = 0;
};

}  // namespace sf
