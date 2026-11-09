/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file failsafe_status_guard.hpp
 * @brief Reader-side guard: a failsafe_status nobody refreshes reads as "ToF silent"
 *        読む側の保護: 誰も更新しない failsafe_status は「ToF 出力停止」と読む
 *
 * failsafe_status is a LEVEL topic: the last value stays readable forever. If Failsafe stops
 * publishing (PowerTask stalled or dead), the last "not silent" would stay in force and every ToF
 * protection (ground-contact trust, blind landing, ARM / mode refusal, LED) would be off for good —
 * the dangerous direction, exactly what the silence handling exists to prevent. Each reading task
 * therefore passes the value through its own FailsafeStatusGuard every cycle: the stamp of the
 * status is watched by a SilenceWatch (sensor_freshness.hpp), and a status older than
 * sf::limits::kFailsafeStatusStaleUs (or never published) reads as tof_silent = true with reason
 * StatusStale. The guard owns a SilenceWatch, which must be observed every cycle by ONE task, so
 * every reader task has its own instance.
 *
 * Pure logic (takes the status by value, no topic access): host unit-testable
 * (test/test_tof_health.cpp).
 *
 * failsafe_status はレベル型のトピックで、最後の値が読み続けられる。Failsafe が発行をやめる
 * （PowerTask の停滞・停止）と、最後の「出力停止でない」が効き続け、ToF の保護の全部（接地フラグの
 * 信用・ToF なしの着陸・ARM／モードの拒否・LED）が永久に無効になる — 出力停止の処理が防ぐべき
 * まさに危険な側。そこで読む各タスクは、毎周期、値を自分の FailsafeStatusGuard に通す: 状態の
 * 時刻印を SilenceWatch（sensor_freshness.hpp）で監視し、sf::limits::kFailsafeStatusStaleUs より
 * 古い（または一度も発行されない）状態は、tof_silent = true・理由 StatusStale と読む。ガードは
 * 1 つのタスクが毎周期観測すべき SilenceWatch を持つので、読むタスクごとに自分のインスタンスを持つ。
 *
 * 純粋ロジック（状態を値で受け取り、トピックには触れない）: ホスト単体テスト可能
 * （test/test_tof_health.cpp）。
 *
 * @design architecture.md §4 — R16 timeout judged on the subscriber side [OK]
 * @design detailed_design.md §3.1 — ToF silent rows, 注 12              [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include <cstdint>

#include "data_types.hpp"
#include "flight_limits.hpp"
#include "sensor_freshness.hpp"

namespace sf {

/// Stateless staleness test for DISPLAY only (the CLI `status` line): the stamp of failsafe_status is
/// older than kFailsafeStatusStaleUs, never published (0), or so far ahead that it is a clock wrap. It needs
/// no observation every cycle, so any number of CLI tasks may call it; it is NOT a safety judgement
/// (those use FailsafeStatusGuard, whose SilenceWatch also catches gaps of 35.8 min and more).
/// 表示「だけ」のための状態を持たない鮮度の判定（CLI の `status` の行）: failsafe_status の時刻印が
/// kFailsafeStatusStaleUs より古い、一度も発行されない（0）、またはクロックの巻き戻りと見えるほど先。毎周期の観測が
/// 要らないので、CLI タスクがいくつ呼んでもよい。安全の判定「ではない」（それは FailsafeStatusGuard で、その
/// SilenceWatch は 35.8 分以上の空きも捕まえる）。
inline constexpr bool isFailsafeStatusStaleForDisplay(uint32_t now_us, uint32_t stamp_us)
{
    if (stamp_us == 0) {
        return true;
    }
    const int32_t age_us = timestampAgeUs(now_us, stamp_us);
    return age_us >= static_cast<int32_t>(limits::kFailsafeStatusStaleUs) ||
           age_us < -SilenceWatch::kClockSkewToleranceUs;
}

class FailsafeStatusGuard {
public:
    /// Observe the latest failsafe_status EVERY cycle and return it, or a "silent" status when it
    /// is stale. The returned value is what the reading task must use.
    /// 最新の failsafe_status を「毎周期」観測して返す。古ければ「出力停止」の状態を返す。
    /// 返す値が、読むタスクが使うべきもの。
    FailsafeStatus guard(uint32_t now_us, const FailsafeStatus& latest)
    {
        watch_.observe(now_us, latest.timestamp);
        if (!watch_.hasSeenSample() || watch_.isSilent(limits::kFailsafeStatusStaleUs)) {
            FailsafeStatus stale = latest;
            stale.tof_silent = true;
            stale.tof_reason = static_cast<uint8_t>(ToFSilenceReason::StatusStale);
            return stale;
        }
        return latest;
    }

private:
    SilenceWatch watch_;
};

}  // namespace sf
