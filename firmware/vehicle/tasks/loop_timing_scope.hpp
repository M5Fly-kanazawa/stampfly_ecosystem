/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file loop_timing_scope.hpp
 * @brief Task-side wrapper of sf::LoopTimingMonitor — reads the clock, publishes once a second
 *        sf::LoopTimingMonitor のタスク側ラッパ — 時計を読み、毎秒発行する
 *
 * ImuTask and ControlTask each own one scope. The task calls beginCycle() right after it wakes
 * and endCycle() at the top of the loop, just before it waits again (so every path of the
 * cycle, `continue` included, is measured). Once a second (400 cycles) the counts go to the
 * task's own topic (imu_loop_timing / control_loop_timing); `status` on the CLI prints them.
 * The publish is one Latest-topic copy per second — nothing that waits.
 *
 * ImuTask と ControlTask が 1 つずつ持つ。タスクは実行開始の直後に beginCycle()、ループ先頭（次に
 * 待つ直前）で endCycle() を呼ぶ（`continue` を含む周期のどの経路も測れる）。毎秒（400 周期）、
 * 数え上げをそのタスク自身のトピック（imu_loop_timing / control_loop_timing）へ発行し、CLI の
 * `status` が表示する。発行は 1 秒に Latest トピックのコピー 1 回 — 待ちを伴うものはない。
 *
 * @design detailed_design.md §8 — Loop timing monitor (imu_loop_timing / control_loop_timing)  [OK]
 * @publisher imu_loop_timing (ImuTask), control_loop_timing (ControlTask)
 */

#pragma once

#include <cstdint>

#include "esp_timer.h"

#include "config.hpp"
#include "data_types.hpp"
#include "loop_timing.hpp"
#include "topics.hpp"

namespace sf {
namespace tasks {

/// One 400 Hz task's timing monitor plus its once-a-second publish.
/// 400 Hz タスク 1 本の周期監視器と、毎秒の発行。
class LoopTimingScope {
public:
    explicit LoopTimingScope(Topic<LoopTiming, Latest, 1>& topic)
        : monitor_(config::IMU_PERIOD_US), topic_(topic) {}

    /// Call right after the task wakes (now_us = the wake time the task already read, armed = the
    /// arm state the task already read this cycle).
    /// タスクが実行を開始した直後に呼ぶ（now_us = タスクが既に読んだ実行開始の時刻、armed = タスクがこの周期に
    /// 既に読んだ ARM 状態）。
    void beginCycle(uint32_t now_us, bool armed)
    {
        monitor_.onWake(now_us, armed);
        if (++cycles_since_publish_ >= kPublishEveryCycles) {
            cycles_since_publish_ = 0;
            LoopTiming timing     = monitor_.result();
            timing.timestamp      = now_us;
            topic_.publish(timing);
        }
    }

    /// Call at the top of the loop, just before waiting for the next wake.
    /// ループ先頭、次の実行開始を待つ直前に呼ぶ。
    void endCycle()
    {
        monitor_.onDone(static_cast<uint32_t>(esp_timer_get_time()));
    }

private:
    /// Microseconds per second. / 1 秒あたりのマイクロ秒。
    static constexpr uint32_t kMicrosecondsPerSecond = 1000000u;

    /// Publish period: once a second.
    /// 発行の周期: 毎秒 1 回。
    static constexpr uint32_t kPublishEveryCycles = kMicrosecondsPerSecond / config::IMU_PERIOD_US;

    LoopTimingMonitor            monitor_;
    Topic<LoopTiming, Latest, 1>& topic_;
    uint32_t                     cycles_since_publish_ = 0;
};

}  // namespace tasks
}  // namespace sf
