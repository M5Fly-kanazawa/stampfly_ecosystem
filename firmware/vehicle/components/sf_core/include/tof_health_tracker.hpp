/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tof_health_tracker.hpp
 * @brief Has the estimator re-acquired the downward ToF? (pure fact tracker)
 *        推定器は下向き ToF を取り込み直したか？（純粋な事実の追跡器）
 *
 * A ToF sample that merely ARRIVES proves nothing. After a silence the estimator may still hold an
 * altitude that is metres off (its position/velocity were pinned or integrated blind), and the
 * innovation gate rejects the very samples that would correct it. So "the ToF is usable again" is
 * judged on what the ESTIMATOR did with the samples, not on their arrival:
 *   altitude_reliable = kTofReacquireSamples consecutive samples adopted with a small innovation
 *                       AND the ToF alive (a continuous stream) for kTofAliveMinUs since the first
 *                       sample after the loss.
 * The tracker also reports two facts the estimator side must act on / publish:
 *  - needsReacquire(valid): the first VALID sample after a loss must be preceded by the
 *    estimator's re-acquisition (inflate the vertical covariance and lift the innovation gate for
 *    that sample, the same mechanism as the end of a flip: IEstimator::reacquireAltitude()).
 *  - invalidStuck(): only invalid samples for kTofInvalidPersistUs while the craft is in the air (a
 *    floor the ToF cannot range). Samples keep arriving, so SilenceWatch cannot see it; it is a loss
 *    like a silence. What an invalid sample MEANS depends on the phase (classifyInvalidSample()):
 *    on the ground (IDLE_GROUND, ARMED_GROUND, DISARM) it is NORMAL (below the minimum range) and counts
 *    as agreement; FLYING counts it; TAKEOFF counts it only after the craft has risen
 *    kTofInvalidTakeoffRiseM by the accelerometer integral, or kTofInvalidTakeoffMaxUs after the takeoff
 *    started (until then the craft is still in the dead zone of the sensor), and then needs only the shorter
 *    kTofInvalidTakeoffPersistUs (a craft that has left the dead zone and sees nothing is climbing blind);
 *    LANDING counts it only
 *    while the last valid height was at least kTofInvalidLandingMinHeightM (near the floor the dead
 *    zone is normal again); FLIP never counts it (the sensor looks sideways). A sample that is
 *    neither agreement nor counted is ignored: it does not make the estimator "reliable".
 *
 * It decides nothing: ImuTask publishes the facts on tof_health, Failsafe judges the verdict
 * (failsafe_status.tof_silent), StateManager decides what to do (tof_silence_policy.hpp), R16/INV-3.
 * Call tick() EVERY IMU cycle in every flight state (SilenceWatch needs continuous observation)
 * and onSample() for every consumed ToF sample. Not thread-safe: ImuTask only.
 *
 * Pure logic: no ESP-IDF / FreeRTOS dependency, host unit-testable (test/test_tof_health.cpp).
 *
 * ToF のサンプルが「届いた」だけでは何も保証されない。出力停止の後、推定器は数 m ずれた高度を
 * 保持しているかもしれず（位置・速度は固定されたか、ToF なしで積分された）、イノベーション判定は
 * それを直すはずのサンプルをまさに棄却する。そこで「ToF が再び使える」は、サンプルの到着ではなく
 * 推定器がそれをどう扱ったかで判断する:
 *   altitude_reliable = 小さなイノベーションで採用されたサンプルが kTofReacquireSamples 個連続し、
 *                       かつ喪失後の最初のサンプルから kTofAliveMinUs の間 ToF が生存（サンプルが
 *                       途切れず届く）している。
 * 追跡器は、推定器側が動く・発行する事実も 2 つ報告する:
 *  - needsReacquire(valid): 喪失後の最初の「有効」サンプルの前に、推定器の取り込み直し（鉛直の
 *    共分散を広げ、そのサンプルだけイノベーション判定を外す。宙返りの終わりと同じ仕組み:
 *    IEstimator::reacquireAltitude()）が要る。
 *  - invalidStuck(): 機体が空中にいる間に無効サンプルだけが kTofInvalidPersistUs 続く（ToF が測れない
 *    床）。サンプルは届き続けるので SilenceWatch には見えない。出力停止と同じ「喪失」。無効サンプルの
 *    「意味」はフェーズで決まる（classifyInvalidSample()）: 地上（IDLE_GROUND・ARMED_GROUND・DISARM 中）
 *    では「正常」（最小レンジ未満）で、一致と数える。FLYING は数える。TAKEOFF は、加速度積分の推定で
 *    kTofInvalidTakeoffRiseM 上昇した後か、離陸の開始から kTofInvalidTakeoffMaxUs 経った後にだけ数え
 *    （それまで機体はセンサの不感帯にいる）、数えたあとは短い kTofInvalidTakeoffPersistUs で足りる
 *    （不感帯を出たのに有効な読みが無い機体は ToF なしで上昇している）。LANDING は、最後の有効な高さが kTofInvalidLandingMinHeightM
 *    以上だったときだけ数える（床の近くでは不感帯が再び正常）。FLIP は数えない（センサが横を向く）。
 *    一致でも継続でもない無効サンプルは無視する: 推定器を「信頼できる」にしない。
 *
 * 追跡器は何も判断しない: ImuTask が事実を tof_health に発行し、Failsafe が判定
 * （failsafe_status.tof_silent）を出し、StateManager が何をするかを決める
 * （tof_silence_policy.hpp、R16/INV-3）。tick() は飛行状態に関わらず IMU の「毎周期」呼び
 * （SilenceWatch は連続観測が要る）、onSample() は消費した ToF サンプルごとに呼ぶ。
 * スレッドセーフではない: ImuTask 専用。
 *
 * 純粋ロジック: ESP-IDF・FreeRTOS に依存せず、ホスト単体テスト可能（test/test_tof_health.cpp）。
 *
 * @design architecture.md §4 — ToF silence: Failsafe detects, StateManager transitions (R16) [OK]
 * @design architecture.md INV-3 — facts here, decision in tof_silence_policy.hpp              [OK]
 * @design detailed_design.md §3.1 — ToF silent rows,注 12                                   [OK]
 * @design coding_and_education.md §2 — Bilingual comments                                     [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "data_types.hpp"   // ToFSilenceReason
#include "flight_limits.hpp"
#include "sensor_freshness.hpp"

namespace sf {

/// Flight phase as the tracker needs it (ImuTask maps FlightState to it). / トラッカーが必要とする飛行
/// フェーズ（ImuTask が FlightState から写す）。
enum class ToFFlightPhase : uint8_t {
    Ground  = 0,   ///< resting: INIT, IDLE_GROUND, IDLE_HELD, ARMED_GROUND (and DISARM) / 静止中
    Takeoff = 1,   ///< TAKEOFF
    Flying  = 2,   ///< FLYING
    Landing = 3,   ///< LANDING
    Flip    = 4,   ///< FLIP
};

/// One consumed ToF sample as the tracker needs it. / トラッカーが必要とする、消費した ToF サンプル 1 つ。
struct ToFSampleFacts {
    uint32_t       now_us;              // [us] clock of the judging task / 判定するタスクの時計
    bool           valid;               // the sample is a valid range / 有効な測距値
    bool           adopted;             // the estimator took the sample (not gated out) / 推定器が採用した（棄却されなかった）
    float          innovation_m;        // |ToF height - estimated height| before the update [m] / 更新前の |ToF の高さ − 推定高度|
    float          distance_m;          // measured distance [m] (read only when valid) / 測距値 [m]（有効なときだけ読む）
    ToFFlightPhase phase;               // flight phase of the craft / 機体の飛行フェーズ
    float          rise_m;              // estimated rise since the craft last rested [m] (accelerometer integral) / 最後の静止からの上昇量の推定 [m]（加速度積分）
    uint32_t       takeoff_elapsed_us;  // [us] time since the TAKEOFF state was entered (Takeoff only) / TAKEOFF に入ってからの時間（Takeoff のみ）
};

/// What an INVALID sample means for the judgement. / 無効サンプルが判定にとって何を意味するか。
enum class InvalidSampleMeaning : uint8_t {
    Agrees,    ///< normal here: agrees with the estimator (ground, below the minimum range) / ここでは正常: 推定器と一致（地上、最小レンジ未満）
    Counts,    ///< evidence of a ToF that cannot range: counts toward the stuck verdict / 測れない ToF の証拠: 継続判定に数える
    Ignored,   ///< expected here but proves nothing: neither agreement nor evidence / ここでは想定内だが何も保証しない: 一致でも証拠でもない
};

/// Meaning of an invalid sample by phase (pure; every row is checked in test/test_tof_health.cpp).
/// 無効サンプルの意味をフェーズで決める（純関数。全行を test/test_tof_health.cpp で検査する）。
/// @param last_valid_height_m Height of the last valid sample [m] / 最後の有効サンプルの高さ [m]
inline constexpr InvalidSampleMeaning classifyInvalidSample(const ToFSampleFacts& sample, float last_valid_height_m)
{
    switch (sample.phase) {
    case ToFFlightPhase::Ground:
        return InvalidSampleMeaning::Agrees;
    case ToFFlightPhase::Takeoff: {
        // The time limit is a backstop for a rise that an accelerometer offset hides. Both conditions only bring the verdict EARLIER, so a
        // craft that the floor still holds (weak motors) may end in a ToF-silent LANDING on the floor: harmless (the descent thrust is below the support thrust).
        // 時間の上限は、加速度計のオフセットが隠す上昇のための備え。どちらの条件も判定を「早める」向きにしか働かないので、床がまだ支えている機体
        // （モータが弱い）は、床の上で ToF なしの LANDING に入ることがある: 害は無い（降下の推力は支持推力未満）。
        const bool left_dead_zone = sample.rise_m >= limits::kTofInvalidTakeoffRiseM ||
                                    sample.takeoff_elapsed_us >= limits::kTofInvalidTakeoffMaxUs;
        return left_dead_zone ? InvalidSampleMeaning::Counts : InvalidSampleMeaning::Ignored;
    }
    case ToFFlightPhase::Flying:
        return InvalidSampleMeaning::Counts;
    case ToFFlightPhase::Landing:
        return (last_valid_height_m >= limits::kTofInvalidLandingMinHeightM) ? InvalidSampleMeaning::Counts
                                                                            : InvalidSampleMeaning::Ignored;
    case ToFFlightPhase::Flip:
        break;
    }
    return InvalidSampleMeaning::Ignored;
}

class ToFHealthTracker {
public:
    /// Observe the clock EVERY IMU cycle (all flight states). Detects a silence by the saturating
    /// SilenceWatch (also > 35.8 min) and starts a new loss episode.
    /// 時計を IMU の「毎周期」（全飛行状態）観測する。飽和する SilenceWatch で出力停止を検出
    /// （35.8 分超も）し、新しい喪失エピソードを始める。
    void tick(uint32_t now_us)
    {
        watch_.observe(now_us, last_stamp_us_);
        if (!watch_.hasSeenSample() || watch_.isSilent(limits::kTofSilenceUs)) {
            beginLoss();
        }
    }

    /// Call BEFORE feeding a sample to the estimator: true when this is the first VALID sample
    /// after a loss, i.e. the estimator must re-acquire the ToF first (consumed: true once).
    /// サンプルを推定器に渡す「前」に呼ぶ: 喪失後の最初の「有効」サンプルなら true、つまり推定器が
    /// 先に ToF を取り込み直す必要がある（消費される: 1 回だけ true）。
    bool needsReacquire(bool sample_valid)
    {
        if (!sample_valid || !reacquire_pending_) {
            return false;
        }
        reacquire_pending_ = false;
        return true;
    }

    /// Call AFTER the estimator processed the sample.
    /// 推定器がサンプルを処理した「後」に呼ぶ。
    void onSample(uint32_t stamp_us, const ToFSampleFacts& sample)
    {
        last_stamp_us_ = stamp_us;
        if (!alive_) {
            alive_ = true;
            alive_since_us_ = sample.now_us;
            agreement_streak_ = 0;
        }
        if (sample.valid) {
            onValidSample(sample);
        } else {
            onInvalidSample(sample);
        }
        if (!reliable_ && !stuck_ && agreement_streak_ >= limits::kTofReacquireSamples &&
            timestampAgeUs(sample.now_us, alive_since_us_) >= static_cast<int32_t>(limits::kTofAliveMinUs)) {
            reliable_ = true;
        }
    }

    /// The disarm edge: a stuck verdict is a flight fact and ends with the flight. The reliability
    /// and the pending re-acquisition survive (they are about the sensor, not the flight).
    /// DISARM のエッジ: 無効サンプルの継続は飛行の事実で、飛行とともに終わる。信頼度と取り込み直しの
    /// 保留は残る（飛行でなくセンサについてのものだから）。
    void onDisarm()
    {
        stuck_ = false;
        invalid_run_active_ = false;
    }

    bool altitudeReliable() const { return reliable_ && !stuck_; }
    bool invalidStuck() const { return stuck_; }

private:
    // A loss: the estimator's altitude is no longer vouched for. Idempotent while the loss lasts.
    // 喪失: 推定器の高度はもう保証されない。喪失が続く間は何度呼んでも同じ。
    void beginLoss()
    {
        reliable_ = false;
        alive_ = false;
        agreement_streak_ = 0;
        reacquire_pending_ = true;
    }

    void onValidSample(const ToFSampleFacts& sample)
    {
        invalid_run_active_ = false;
        last_valid_height_m_ = sample.distance_m;
        if (stuck_) {
            stuck_ = false;   // a valid sample ends the stuck verdict, but the altitude must be re-acquired
            beginLoss();
            alive_ = true;
            alive_since_us_ = sample.now_us;
        }
        const bool agrees = sample.adopted && (std::fabs(sample.innovation_m) <= limits::kTofReacquireInnovationM);
        agreement_streak_ = agrees ? bumpStreak(agreement_streak_) : 0;
    }

    void onInvalidSample(const ToFSampleFacts& sample)
    {
        switch (classifyInvalidSample(sample, last_valid_height_m_)) {
        case InvalidSampleMeaning::Agrees:
            // On the ground an invalid sample (below the minimum range) is normal: it agrees with
            // the estimator, which holds the ground at height 0. A stuck verdict is kept until a
            // valid sample or the disarm (a landing mid-episode must not clear it).
            // 地上の無効サンプル（最小レンジ未満）は正常: 地面を高さ 0 に保持する推定器と一致する。
            // 継続判定は、有効サンプルか DISARM まで保つ（エピソードの途中の着陸で消してはならない）。
            invalid_run_active_ = false;
            agreement_streak_ = stuck_ ? 0 : bumpStreak(agreement_streak_);
            break;
        case InvalidSampleMeaning::Ignored:
            invalid_run_active_ = false;   // the run restarts when the evidence starts / 証拠が始まったら数え直す
            break;
        case InvalidSampleMeaning::Counts:
            countInvalidRun(sample);
            break;
        }
    }

    // Evidence of a ToF that cannot range: after kTofInvalidPersistUs of it (kTofInvalidTakeoffPersistUs in
    // TAKEOFF, where a craft that has left the dead zone but sees nothing is already climbing fast and blind) the
    // verdict is "stuck", which is a loss (the re-acquisition is requested for the first valid sample that follows).
    // 測れない ToF の証拠: kTofInvalidPersistUs（TAKEOFF では kTofInvalidTakeoffPersistUs。不感帯を出たのに何も見えない
    // 機体は、すでに速く上昇しているのに ToF の読みが無い）続いたら「継続」判定で、これは喪失（その後の最初の有効サンプルに
    // 取り込み直しを求める）。
    void countInvalidRun(const ToFSampleFacts& sample)
    {
        agreement_streak_ = 0;
        if (!invalid_run_active_) {
            invalid_run_active_ = true;
            invalid_since_us_ = sample.now_us;
        }
        const uint32_t persist_us = (sample.phase == ToFFlightPhase::Takeoff) ? limits::kTofInvalidTakeoffPersistUs
                                                                              : limits::kTofInvalidPersistUs;
        if (timestampAgeUs(sample.now_us, invalid_since_us_) >= static_cast<int32_t>(persist_us)) {
            stuck_ = true;
            reliable_ = false;
            reacquire_pending_ = true;   // the first valid sample after it must come after the re-acquisition / その後の最初の有効サンプルは取り込み直しの後
        }
    }

    static constexpr uint16_t bumpStreak(uint16_t streak)
    {
        return (streak < UINT16_MAX) ? static_cast<uint16_t>(streak + 1) : streak;
    }

    SilenceWatch watch_;
    uint32_t last_stamp_us_    = 0;
    uint32_t alive_since_us_   = 0;
    uint32_t invalid_since_us_ = 0;
    float    last_valid_height_m_ = 0.0f;   // [m] height of the last valid sample / 最後の有効サンプルの高さ
    uint16_t agreement_streak_ = 0;
    bool     alive_              = false;
    bool     reliable_           = false;
    bool     stuck_              = false;
    bool     invalid_run_active_ = false;
    bool     reacquire_pending_  = true;   // starts as a loss (never seen a sample) / 起動時は喪失（サンプル未到達）
};

/// The verdict Failsafe publishes: WHY the downward ToF is in a silence episode (None = no episode).
/// Priority: no new sample (also when the sensor never produced one) > only invalid samples while
/// flying > samples are back but the estimator has not re-acquired the ToF. A pure function of three
/// facts so every cell can be checked on the host (test/test_tof_health.cpp).
/// Failsafe が発行する判定: 下向き ToF が出力停止のエピソード中である「理由」（None = エピソード
/// なし）。優先順位: 新サンプル無し（センサが一度もサンプルを出していない場合も）> 飛行中に無効
/// サンプルだけ > サンプルは戻ったが推定器がまだ ToF を取り込み直していない。3 つの事実の純関数で、
/// 全セルをホストで検査できる（test/test_tof_health.cpp）。
inline constexpr ToFSilenceReason judgeToFSilenceReason(bool no_new_sample, bool invalid_stuck,
                                                        bool altitude_reliable)
{
    if (no_new_sample) {
        return ToFSilenceReason::NoSample;
    }
    if (invalid_stuck) {
        return ToFSilenceReason::InvalidStuck;
    }
    return altitude_reliable ? ToFSilenceReason::None : ToFSilenceReason::Reacquiring;
}

}  // namespace sf
