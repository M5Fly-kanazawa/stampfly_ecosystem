/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file nonfinite_scope.hpp
 * @brief Task-side wrapper of sf::NonFiniteMonitor — owns the monitor, publishes the counts
 *        sf::NonFiniteMonitor のタスク側ラッパ — 監視器を持ち、数え上げを発行する
 *
 * ControlTask owns one scope. It calls beginCycle() right after it wakes (with whether the craft is
 * armed) and endCycle() at the end of the cycle. The checks of the boundaries (record) happen in
 * between, only in an armed cycle. The counts go to `nonfinite_status` once a second of armed cycles,
 * at every cycle that detected something, and once more when the craft stops being armed (so the
 * last cycles before DISARM are not left unpublished); `status` on the CLI prints them. The publish
 * is a Latest-topic copy — nothing that waits — and nothing here logs: the 400 Hz task never writes
 * the console for a detection.
 *
 * ControlTask が 1 つ持つ。起床の直後に beginCycle()（ARM 中かを渡す）、周期の終わりに endCycle() を呼ぶ。
 * 境界の検査（record）はその間で、ARM 中の周期だけ行う。数え上げは、ARM 中の 1 秒（400 周期）ごと、何かを検出した
 * 周期ごと、ARM でなくなった時に 1 回（DISARM 直前の周期が未発行で残らないように）、`nonfinite_status` へ
 * 発行し、CLI の `status` が表示する。発行は Latest トピックのコピー — 待ちを伴うものはない。ここでは何も
 * ログに出さない: 400 Hz のタスクは検出でコンソールへ書かない。
 *
 * @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
 * @publisher nonfinite_status (ControlTask)
 */

#pragma once

#include <cstdint>

#include "config.hpp"
#include "data_types.hpp"
#include "nonfinite_guard.hpp"
#include "topics.hpp"

namespace sf {
namespace tasks {

/// ControlTask's non-finite monitor plus its publish.
/// ControlTask の有限性の監視器と、その発行。
class NonFiniteScope {
public:
    explicit NonFiniteScope(Topic<NonFiniteStatus, Latest, 1>& topic) : topic_(topic) {}

    /// Call right after the task wakes. armed = the arm state the task already read this cycle.
    /// タスクが起きた直後に呼ぶ。armed = タスクがこの周期に既に読んだ ARM 状態。
    void beginCycle(uint32_t now_us, bool armed)
    {
        const bool was_armed = armed_;
        armed_ = armed;
        if (armed) {
            monitor_.beginCycle(now_us);
        } else if (was_armed) {
            publish(now_us);   // the last armed cycles / ARM 中の最後の周期たち
        }
        last_us_ = now_us;
    }

    /// The monitor the boundaries are recorded into. Valid to record only in an armed cycle.
    /// 境界を記録する監視器。記録してよいのは ARM 中の周期だけ。
    NonFiniteMonitor& monitor() { return monitor_; }

    /// Call at the end of an armed cycle: publish on a detection and once a second.
    /// ARM 中の周期の終わりに呼ぶ: 検出があったときと、毎秒発行する。
    void endCycle()
    {
        const bool due = (++cycles_since_publish_ >= kPublishEveryCycles);
        if (monitor_.detectedThisCycle() || due) {
            publish(last_us_);
        }
    }

private:
    /// Microseconds per second. / 1 秒あたりのマイクロ秒。
    static constexpr uint32_t kMicrosecondsPerSecond = 1000000u;

    /// Publish period: once a second.
    /// 発行の周期: 毎秒 1 回。
    static constexpr uint32_t kPublishEveryCycles = kMicrosecondsPerSecond / config::IMU_PERIOD_US;

    void publish(uint32_t now_us)
    {
        cycles_since_publish_ = 0;
        NonFiniteStatus status = monitor_.result();
        status.timestamp       = now_us;
        topic_.publish(status);
    }

    NonFiniteMonitor                   monitor_;
    Topic<NonFiniteStatus, Latest, 1>& topic_;
    uint32_t                           cycles_since_publish_ = 0;
    uint32_t                           last_us_              = 0;
    bool                               armed_                = false;
};

}  // namespace tasks
}  // namespace sf
