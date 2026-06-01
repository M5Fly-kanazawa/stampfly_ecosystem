/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_main.cpp
 * @brief StampFly emulator host entry — runs the REAL firmware app_main on host.
 *        StampFly エミュレータのホスト入口 — 実ファームの app_main をホストで走らせる。
 *
 * The firmware is compiled UNMODIFIED against the ESP-IDF host platform
 * (simulator/sil/esp_idf_host + compat) and the StampFly virtual board. This
 * entry just calls app_main() — which does BSP init and creates all 14 tasks
 * via xTaskCreatePinnedToCore (registered with the cooperative scheduler) — then
 * runs the scheduler for the requested duration. Firmware-agnostic: any ESP-IDF
 * firmware exposing app_main() links here (E5 runs firmware/vehicle the same way).
 *
 * ファームは ESP-IDF host platform ＋ StampFly 仮想ボードに対して無改変でコンパイル
 * される。この入口は app_main() を呼ぶだけ（BSP init ＋ 14タスク生成＝協調スケジューラ
 * に登録）→ 指定時間スケジューラを回す。ファーム非依存（E5 で firmware/vehicle も同様に）。
 *
 * @design simulator/sil/RESET_PLAN.md §5-7 — run the real firmware unmodified  [--]
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "scheduler.hpp"

// The firmware entry point (C linkage), defined in firmware/.../main/main.cpp.
// ファームのエントリ（C リンケージ）。firmware/.../main/main.cpp に定義。
extern "C" void app_main(void);

int main(int argc, char** argv)
{
    // Simulated wall-clock duration to run [us] (default 2 s).
    // 走らせるシミュレーション時間 [us]（既定 2 秒）。
    const int64_t duration_us =
        (argc > 1) ? static_cast<int64_t>(std::atoll(argv[1])) : 2'000'000;

    std::printf("[emu] === StampFly emulator: vehicle_new app_main on host ===\n");

    // E0: no Plant yet — sensors read inert virtual devices (zeros). The on_advance
    // hook (physics) is wired in E1. An empty hook keeps the scheduler loop safe.
    // E0: まだ Plant 無し。物理 on_advance フックは E1 で配線。空フックで安全に回す。
    sil::rtos::Scheduler::instance().set_on_advance([](int64_t /*now_us*/) {});

    // BSP init + create all 14 tasks (each xTaskCreatePinnedToCore registers a task).
    // BSP 初期化 ＋ 14タスク生成（各 xTaskCreate がタスクを登録）。
    app_main();

    std::printf("[emu] app_main returned; running scheduler for %lld us\n",
                static_cast<long long>(duration_us));

    // Drive the deterministic cooperative scheduler.
    // 決定論的協調スケジューラを駆動。
    sil::rtos::Scheduler::instance().run(duration_us);

    std::printf("[emu] scheduler stopped — emulator run complete\n");
    return 0;
}
