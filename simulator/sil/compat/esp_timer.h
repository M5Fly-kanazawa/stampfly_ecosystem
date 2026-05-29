/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_timer.h (host shim)
 * @brief esp_timer_get_time() backed by std::chrono on host
 *        ホストでstd::chronoを裏に持つesp_timer_get_time()
 *
 * Used by failsafe.cpp for alert timestamps. Returns microseconds since the
 * first call (monotonic). SIL code that needs determinism should derive time
 * from the simulation step counter, not this clock.
 *
 * failsafe.cpp がアラートのタイムスタンプに使用。初回呼び出しからの
 * マイクロ秒（単調）を返す。決定論が要るSILコードはこの時計ではなく
 * シミュレーションのステップカウンタから時刻を導くこと。
 */

#pragma once

#include <cstdint>
#include <chrono>

/// Microseconds since first call (monotonic)
/// 初回呼び出しからのマイクロ秒（単調）
inline int64_t esp_timer_get_time(void)
{
    using namespace std::chrono;
    static const auto t0 = steady_clock::now();
    return duration_cast<microseconds>(steady_clock::now() - t0).count();
}
