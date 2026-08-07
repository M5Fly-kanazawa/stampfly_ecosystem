/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_realtime.cpp
 * @brief Wall-clock pacing implementation — see emu_realtime.hpp.
 *        壁時計ペーシングの実装 — emu_realtime.hpp 参照。
 */

#include "emu_realtime.hpp"

#include <chrono>
#include <cstdlib>
#include <unistd.h>   // usleep

namespace {

using Clock = std::chrono::steady_clock;

// Cached once (env vars never change mid-run). -1 = not yet checked.
// 一度だけ判定してキャッシュ（実行中に env は変わらない）。-1=未判定。
int g_enabled = -1;

// Wall-clock instant paired with virtual t=0, latched on the FIRST call (the
// scheduler always calls on_advance(0) once before the run loop starts — see
// scheduler.cpp Scheduler::run()). 仮想 t=0 に対応する壁時計時刻。初回呼び出しで
// ラッチ（scheduler.cpp の run() は while ループの前に on_advance(0) を必ず1回呼ぶ）。
Clock::time_point g_epoch{};
bool g_epoch_set = false;

}  // namespace

extern "C" {

bool sils_realtime_enabled(void)
{
    if (g_enabled < 0) {
        g_enabled = (std::getenv("SILS_EMU_REALTIME") != nullptr) ? 1 : 0;
    }
    return g_enabled != 0;
}

void sils_realtime_pace(int64_t now_us)
{
    if (!sils_realtime_enabled()) return;   // default path: untouched

    if (!g_epoch_set) {
        g_epoch = Clock::now();
        g_epoch_set = true;
        return;   // nothing to pace against yet (this IS t=0)
    }

    const auto wall_elapsed = Clock::now() - g_epoch;
    const auto virt_elapsed = std::chrono::microseconds(now_us);

    // Ahead of the wall clock → sleep off the surplus. Behind → do nothing and
    // let the loop run flat out (no catch-up beyond "as fast as possible");
    // this matches real hardware, which never runs faster than real time but
    // also never owes itself time back.
    // 壁時計より進みすぎ→余剰分だけ眠る。遅れ→何もせず全速（それ以上の
    // 「追いつき」はしない）— 実機は実時間より速くは進まないが、自分に
    // 時間を貸し借りもしない、というのと同じ。
    if (virt_elapsed > wall_elapsed) {
        const auto surplus_us =
            std::chrono::duration_cast<std::chrono::microseconds>(virt_elapsed - wall_elapsed).count();
        if (surplus_us > 0) {
            usleep(static_cast<useconds_t>(surplus_us));
        }
    }
}

}  // extern "C"
