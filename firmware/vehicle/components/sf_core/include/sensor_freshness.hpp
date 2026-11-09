/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sensor_freshness.hpp
 * @brief Silence / freshness judgement for cross-task timestamps (saturating watch)
 *        タスク間の時刻印に対する出力停止・鮮度の判定（飽和する監視器）
 *
 * Problem 1 (read-order inversion): a timestamp written by ANOTHER task can be a few
 * microseconds NEWER than the "now" this task read. An unsigned subtraction (now - ts)
 * then wraps to ~4.29e9 and a healthy source is reported stale (false positive). The age
 * of a NEW sample must therefore be taken signed, and a negative age means "just arrived".
 *
 * Problem 2 (long silence): the stamps are uint32 microseconds (wrap every 71.6 min). Any
 * stateless difference of two uint32 values is ambiguous beyond 2^31 us (35.8 min): a signed
 * difference turns negative ("newer than now" = healthy) and an unsigned one aliases back to
 * a small age after 71.6 min. A source that stays silent longer than that would be reported
 * healthy again — the dangerous direction. A stateless function cannot fix this.
 *
 * Therefore the judgement is made by SilenceWatch, a small stateful observer: every cycle
 * the caller passes (now, latest stamp). When the stamp CHANGED, a new sample arrived and
 * the silent time restarts from that sample's (signed, clamped >= 0) age. When it did NOT
 * change, the silent time grows by the time elapsed since the previous observation and
 * SATURATES at UINT32_MAX (71 min) — it never wraps, so "silent" stays silent however long
 * the silence lasts. Only the short, locally measured interval between two observations is
 * a modular difference, and that is valid below 35.8 min (callers observe at 1-400 Hz).
 *
 * Problem 3 (a gap between observations): if the CALLER stops observing for 35.8 min or more
 * (e.g. the judge only runs while armed), the elapsed time between two observations is no
 * longer a valid small modular difference. The watch therefore treats a gap that looks like
 * negative time beyond kClockSkewToleranceUs, and a stamp that looks that far in the future,
 * as "silent" (saturated), also for the very first observation. Gaps of 71.6 min or more alias
 * to a small value and CANNOT be detected: callers MUST observe every cycle regardless of the
 * flight state (arm/disarm), never only while the judgement is needed.
 *
 * Pure logic: no ESP-IDF / FreeRTOS dependency, so it is host unit-testable
 * (test/test_sensor_freshness.cpp, test/test_tof_silence.cpp).
 *
 * 問題1（読み出し順序の逆転）: 別タスクが書いた時刻印は、このタスクが読んだ「現在時刻」より
 * 数 us 新しいことがある。符号なしの引き算 (now - ts) は ~4.29e9 に桁あふれし、正常な発行元を
 * stale と誤判定する（偽陽性）。新しいサンプルの経過は符号付きで取り、負は「今届いた」とみなす。
 *
 * 問題2（長時間の出力停止）: 時刻印は uint32 のマイクロ秒（71.6 分で巻き戻る）。uint32 2 つの
 * 状態を持たない差は 2^31 us（35.8 分）を超えると曖昧になる: 符号付きは負（「now より新しい」
 * ＝健全）に戻り、符号なしは 71.6 分で小さな経過に戻る。それより長く出力停止した発行元が再び
 * 健全と判定される — 危険な側の誤り。状態を持たない関数では直せない。
 *
 * そこで判定は、状態を持つ小さな監視器 SilenceWatch が行う。呼び出し側は毎周期 (現在時刻, 最新の
 * 時刻印) を渡す。時刻印が「変化」していれば新サンプルが届いたので、出力停止の時間はそのサンプルの
 * 経過（符号付き、0 以上に丸める）から再開する。変化していなければ、前回の観測からの経過時間だけ
 * 増え、UINT32_MAX（71 分）で「飽和」する — 巻き戻らないので、どれだけ長く止まっても「停止」の
 * ままである。モジュラ差になるのは、2 回の観測の間の短い自前計測の間隔だけで、これは 35.8 分
 * 未満なら正しい（呼び出しは 1〜400 Hz）。
 *
 * 問題3（観測の空き）: 「呼び出し側」が 35.8 分以上観測をやめる（例: 判定を ARM 中しか
 * 動かさない）と、2 回の観測の間の経過は小さなモジュラ差として有効でなくなる。そこで監視器は、
 * 時間が kClockSkewToleranceUs を超えて負に見える観測の空きと、それほど遠い未来に見える時刻印を
 * 「停止」（飽和）と扱う。最初の観測でも同じ。71.6 分以上の空きは小さな値に戻り検出できない:
 * 呼び出し側は、判定が要るときだけでなく、飛行状態（ARM/DISARM）に関わらず毎周期観測すること。
 *
 * 純粋ロジック: ESP-IDF・FreeRTOS に依存せず、ホスト単体テスト可能
 * （test/test_sensor_freshness.cpp、test/test_tof_silence.cpp）。
 *
 * @design architecture.md §3 — R15 sensor_health 1 Hz                    [OK]
 * @design architecture.md §4 — R16 timeout judged on the subscriber side [OK]
 * @design detailed_design.md §2 — sensor_health publisher: sf_board / 各 task [OK]
 * @design coding_and_education.md §2 — Bilingual comments                 [OK]
 */

#pragma once

#include <cstdint>

namespace sf {

/// Signed age of a timestamp [us]: positive = ts is older than now, negative = ts is
/// newer than now (cross-task read-order inversion). ONLY valid while |age| < 2^31 us
/// (35.8 min): use it for a freshly arrived sample, never to judge a long silence
/// (that is SilenceWatch's job).
/// 時刻の符号付き経過 [us]: 正 = ts が now より古い、負 = ts が now より新しい
/// （タスク間の読み出し順序の逆転）。|age| < 2^31 us（35.8 分）の間だけ有効:
/// 届いたばかりのサンプルに使い、長時間の出力停止の判定には使わない（SilenceWatch の仕事）。
inline constexpr int32_t timestampAgeUs(uint32_t now_us, uint32_t timestamp_us)
{
    return static_cast<int32_t>(now_us - timestamp_us);
}

/// Observer that turns (now, latest stamp) pairs into a saturating "silent for" time.
/// (現在時刻, 最新の時刻印) の組を、飽和する「出力停止の継続時間」に変える監視器。
///
/// Usage: call observe() EVERY cycle, in every flight state (a gap of 35.8 min or more between
/// observations reads as silent, 71.6 min or more cannot be detected), then ask isSilent().
/// Not thread-safe: observe() and isSilent() belong to one task (the judging task).
/// 使い方: observe() を飛行状態に関わらず「毎周期」呼び（観測の空きが 35.8 分以上なら停止と読み、
/// 71.6 分以上は検出できない）、isSilent() で問い合わせる。
/// スレッドセーフではない: observe() と isSilent() は判定するタスク 1 つに属する。
///
/// @design architecture.md §4 — R16 timeout judged on the subscriber side [OK]
class SilenceWatch {
public:
    /// "Silent for" value of a source that never produced a sample; also the saturation cap.
    /// 一度もサンプルを出していない発行元の「停止継続時間」。飽和の上限でもある。
    static constexpr uint32_t kSaturatedUs = UINT32_MAX;

    /// Cross-task / clock jitter tolerated as "just now" [us] (1 s). A stamp this far ahead of
    /// now, or an observation clock this far behind the previous one, is jitter; anything
    /// further is the 2^31 us ambiguity of a long gap and reads as silent.
    /// 「たった今」として許容するタスク間・時計のふらつき [us]（1 s）。now よりこれだけ先の
    /// 時刻印、前回の観測よりこれだけ戻った観測時刻はふらつき。それ以上は長い空きによる
    /// 2^31 us の曖昧さで、停止と読む。
    static constexpr int32_t kClockSkewToleranceUs = 1000000;

    /// Feed the latest stamp. timestamp_us == 0 means "never published" (-> silent) until a
    /// non-zero stamp has been seen once; after that a 0 is an ordinary sample (the stamp
    /// legitimately wraps through 0 every 2^32 us).
    /// 最新の時刻印を与える。timestamp_us == 0 は、0 以外の時刻印を一度見るまでは「未発行」
    /// （→ 出力停止）。見た後の 0 は通常のサンプル（時刻印は 2^32 us ごとに 0 を通って巻き戻る）。
    void observe(uint32_t now_us, uint32_t timestamp_us)
    {
        const int32_t since_last_observe_us = timestampAgeUs(now_us, last_now_us_);
        if (timestamp_us == 0 && !seen_stamp_) {
            silent_for_us_ = kSaturatedUs;
        } else if (!observed_ || timestamp_us != last_timestamp_us_) {
            silent_for_us_ = ageOfNewSampleUs(now_us, timestamp_us);
        } else {
            silent_for_us_ = silentAfterGapUs(silent_for_us_, since_last_observe_us);
        }
        seen_stamp_ = seen_stamp_ || (timestamp_us != 0);
        last_timestamp_us_ = timestamp_us;
        // Never move the reference backwards within the jitter tolerance, so a jittery clock
        // cannot count time twice. A larger backward step is a long gap: re-base on it.
        // 基準時刻はふらつき許容の範囲では後戻りさせない。ふらつく時計で同じ時間を二重に数えない
        // ため。それより大きく戻って見えるのは長い空きなので、そこへ基準を取り直す。
        if (!observed_ || since_last_observe_us > 0 || since_last_observe_us < -kClockSkewToleranceUs) {
            last_now_us_ = now_us;
        }
        observed_ = true;
    }

    /// True when no new sample has arrived for window_us or longer (age == window is already
    /// silent: strict "<" for fresh), or when no sample has ever arrived / observe() was never
    /// called. Judges only WHEN samples arrive, never WHAT they say: a sensor that keeps
    /// delivering "out of range" samples is NOT silent (a separate judgement,
    /// detailed_design.md §3.1).
    /// window_us 以上新サンプルが無ければ true（経過 == window はすでに停止: fresh は厳密な "<"）。
    /// 一度もサンプルが無い・observe() が一度も呼ばれていない場合も true。判定するのはサンプルが
    /// 「いつ」届くかだけで、「何を」言っているかは見ない: 「範囲外」のサンプルを出し続けるセンサは
    /// 出力停止ではない（別の判定。detailed_design.md §3.1）。
    bool isSilent(uint32_t window_us) const { return silent_for_us_ >= window_us; }

    /// Complement of isSilent(). / isSilent() の補集合。
    bool isFresh(uint32_t window_us) const { return !isSilent(window_us); }

    /// Silent time as of the last observe() [us], saturated at kSaturatedUs.
    /// 直近の observe() 時点の出力停止の継続時間 [us]（kSaturatedUs で飽和）。
    uint32_t silentForUs() const { return silent_for_us_; }

    /// True once a non-zero stamp has been observed (the source has produced a sample).
    /// 0 以外の時刻印を観測済みなら true（発行元がサンプルを出したことがある）。
    bool hasSeenSample() const { return seen_stamp_; }

private:
    static constexpr uint32_t saturatingAdd(uint32_t a, uint32_t b)
    {
        return (b > kSaturatedUs - a) ? kSaturatedUs : (a + b);
    }

    // Silent time of a NEW sample: its signed age, 0 within the jitter tolerance ahead of now,
    // saturated when the stamp looks further ahead (an age of 35.8 min or more wrapped negative).
    // 新サンプルの停止時間: 符号付きの経過。now よりふらつき許容以内の先なら 0、それより先に
    // 見える（35.8 分以上の経過が負に巻き戻った）なら飽和。
    static constexpr uint32_t ageOfNewSampleUs(uint32_t now_us, uint32_t timestamp_us)
    {
        const int32_t age_us = timestampAgeUs(now_us, timestamp_us);
        if (age_us >= 0) {
            return static_cast<uint32_t>(age_us);
        }
        return (age_us >= -kClockSkewToleranceUs) ? 0u : kSaturatedUs;
    }

    // Silent time when the stamp did NOT change: add the time since the previous observation.
    // Zero or within-tolerance backward time adds 0; a larger backward step is a gap of 35.8 min
    // or more -> saturate.
    // 時刻印が変化しなかったときの停止時間: 前回の観測からの時間を足す。0 またはふらつき許容内の
    // 後戻りは 0 を足し、それより大きな後戻りは 35.8 分以上の空きなので飽和。
    static constexpr uint32_t silentAfterGapUs(uint32_t silent_for_us, int32_t since_last_observe_us)
    {
        if (since_last_observe_us < -kClockSkewToleranceUs) {
            return kSaturatedUs;
        }
        const uint32_t elapsed_us = (since_last_observe_us < 0) ? 0u : static_cast<uint32_t>(since_last_observe_us);
        return saturatingAdd(silent_for_us, elapsed_us);
    }

    uint32_t silent_for_us_     = kSaturatedUs;   // starts "silent" / 起動時は「停止」
    uint32_t last_timestamp_us_ = 0;
    uint32_t last_now_us_       = 0;
    bool     observed_          = false;
    bool     seen_stamp_        = false;   // a non-zero stamp was seen / 0 以外の時刻印を観測済み
};

}  // namespace sf
