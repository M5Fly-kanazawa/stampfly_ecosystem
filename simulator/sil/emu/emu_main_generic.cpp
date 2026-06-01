/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_main_generic.cpp
 * @brief Firmware-AGNOSTIC emulator host entry — brings up the Plant + virtual
 *        board, then runs ANY firmware's app_main() on the host scheduler.
 *        ファーム非依存のエミュレータ入口 — Plant＋仮想ボードを起こし、任意の
 *        ファームの app_main() をホストスケジューラで走らせる。
 *
 * Unlike emu_main.cpp (which prints a vehicle_new-specific estimator check via
 * that firmware's topics), this entry references NO firmware types — it only
 * touches the SIL infrastructure (Plant, virtual board, scheduler). Used by the
 * old firmware (firmware/vehicle) target to prove the emulator is firmware-
 * agnostic: a second, independent firmware runs on the same libsf_emu.
 *
 * emu_main.cpp と違いファーム型を一切参照せず、SIL基盤のみに触れる。旧ファーム
 * （firmware/vehicle）ターゲットが使い、同じ基盤で2本目が走る＝ファーム非依存を実証。
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>    // fcntl, O_NONBLOCK — make host stdin non-blocking
#include <unistd.h>   // STDIN_FILENO

#include "scheduler.hpp"
#include "plant.hpp"
#include "virtual_board.hpp"
#include "scenario.hpp"          // E6: deterministic scripted-input timeline
#include "console_feeder.hpp"    // E6: scripted console bytes -> firmware stdin
#include "emu_record.hpp"        // E6: virtual-time-stamped input/event log
#include "emu_trajectory.hpp"    // review-video trajectory recorder (SIL_EMU_TRAJ)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void);

// Optional firmware-specific virtual pilot (ESP-NOW transmitter). Defined by the
// per-firmware emu target (e.g. virtual_pilot_vehicle.cpp); weak so emulators
// without a pilot link to nullptr and simply run no pilot.
// 任意のファーム固有仮想パイロット。ターゲット側で定義（weak）。未定義なら nullptr。
extern "C" __attribute__((weak)) void sil_virtual_pilot_task(void*);

namespace {

sil::Plant g_plant;
int64_t    g_last_step_us = 0;
float      g_peak_alt = 0.0f;   // highest truth altitude over the whole run / 実行中の最高高度
constexpr float kGroundZ = 0.013f;

void on_advance(int64_t now_us)
{
    if (now_us > g_last_step_us) {
        sil_board_step_plant((float)(now_us - g_last_step_us) * 1e-6f);
        g_last_step_us = now_us;
        // Track the peak altitude so a no-runaway gate can bound the whole flight
        // path, not just the final sample (a transient climb that descends would
        // otherwise slip past a final-only check).
        // 最高高度を追跡し、no-runaway ゲートが最終サンプルだけでなく飛行経路全体を
        // 有界判定できるようにする（一時的に上昇して戻る軌跡を最終値だけでは見逃す）。
        const float alt = -g_plant.truth().pos_ned.z;
        if (alt > g_peak_alt) g_peak_alt = alt;
        // Record a review-video trajectory row (no-op unless SIL_EMU_TRAJ was set).
        // レビュー動画用に軌跡を1行記録（SIL_EMU_TRAJ 未設定なら no-op）。
        sil_emu_traj_sample((double)now_us * 1e-6, &g_plant);
    }
}

// app_main runs AS a scheduler task — exactly like the real ESP-IDF "main" task.
// This matters: firmwares call vTaskDelay() at the top of app_main (the old
// firmware delays 3 s for USB), which is only valid from a task context. It
// creates the other tasks, then returns → the main task deletes itself.
// app_main を「main タスク」として走らせる（実 ESP-IDF と同じ）。app_main 冒頭の
// vTaskDelay はタスク文脈でのみ有効。他タスクを生成し、戻ったら自タスクを削除。
void app_main_task(void* /*arg*/)
{
    app_main();
    vTaskDelete(nullptr);
}

}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    const int64_t duration_us =
        (argc > 2) ? (int64_t)std::atoll(argv[2]) : 1'000'000;

    // Unbuffered so the last firmware log before any crash is not lost when
    // output is redirected to a file (block-buffered otherwise).
    // 非バッファ化: ファイルリダイレクト時もクラッシュ直前のログを失わない。
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    // Replace the host stdin with a NON-BLOCKING, never-written pipe. The
    // firmware's serial-CLI task does read(STDIN_FILENO,...) to wait for USB-CDC
    // console bytes. We want that read to return EAGAIN immediately (not block,
    // not EOF): blocking would stall the cooperative scheduler in the kernel
    // (hang detector aborts); EOF (e.g. stdin=/dev/null) makes the firmware log
    // "read()==0" every poll. An open empty pipe yields EAGAIN forever, so the
    // firmware's own EAGAIN path silently yields (vTaskDelay) — a faithful USB
    // console with no operator typing, with no log spam. No firmware change.
    // 非ブロッキングの空パイプを stdin に被せ、read() を常に EAGAIN にする。ブロック
    // （ハング）も EOF（毎回ログ）も避け、firmware の EAGAIN ポーリング yield が働く。
    int cli_pipe[2];
    if (pipe(cli_pipe) == 0) {
        fcntl(cli_pipe[0], F_SETFL, O_NONBLOCK);  // read end: non-blocking
        fcntl(cli_pipe[1], F_SETFL, O_NONBLOCK);  // write end: non-blocking (feeder)
        dup2(cli_pipe[0], STDIN_FILENO);
        // The write end stays open for the whole run (never closed → read() gets
        // EAGAIN, not EOF). A scenario "key" event writes scripted bytes into it
        // via the console feeder; with no scenario nothing is written (unchanged).
        // 書き込み端は実行中ずっと開いたまま（→ read は EOF でなく EAGAIN）。シナリオの
        // key 事象がフィーダ経由でここへ台本バイトを書く。シナリオ無しなら何も書かない。
        sil_console_set_fd(cli_pipe[1]);
    }

    // E6: open the deterministic input/event recorder if requested. The path
    // comes from the SIL_EMU_EVENTS env var (set by the sf CLI / scenario runs).
    // When unset, the recorder stays closed and every record call is a no-op, so
    // the default run is byte-identical to before this feature.
    // E6: SIL_EMU_EVENTS が指すパスへ決定論レコーダをオープン（sf CLI/シナリオ実行が設定）。
    // 未設定なら閉じたまま＝record は no-op ＝ 既定実行は本機能前と byte-identical。
    sil_emu_record_open(std::getenv("SIL_EMU_EVENTS"));

    // Open the review-video trajectory if requested (SIL_EMU_TRAJ → CSV path). Unset
    // → recorder stays closed and every sample is a no-op (run unchanged).
    // レビュー動画の軌跡を要求時に開く（SIL_EMU_TRAJ → CSV パス）。未設定なら閉じたまま。
    sil_emu_traj_open(std::getenv("SIL_EMU_TRAJ"));

    std::printf("[emu] === StampFly emulator (firmware-agnostic entry) ===\n");

    // E6: load a scripted input scenario if given (argv[3]). A parse error aborts
    // BEFORE the scheduler starts (no firmware singletons exist yet → safe return).
    // E6: 指定があれば入力シナリオ（argv[3]）をロード。パースエラーはスケジューラ起動前に
    // 中断（ファームのシングルトン未生成ゆえ安全に return）。
    const char* scenario_path = (argc > 3) ? argv[3] : nullptr;
    if (sil_scenario_load(scenario_path) < 0) {
        std::fprintf(stderr, "[emu] scenario load failed — aborting before run\n");
        sil_emu_record_close();
        sil_emu_traj_close();
        return 2;
    }

    std::printf("[emu] (1) plant.init ...\n");
    if (!g_plant.init(model_path)) {
        std::fprintf(stderr, "[emu] plant init failed (model: %s)\n", model_path);
        return 1;
    }
    g_plant.setStartHeight(kGroundZ);
    sil_board_attach_plant(&g_plant);
    sil::rtos::Scheduler::instance().set_on_advance(on_advance);

    // Spawn app_main as the "main" task, then run the scheduler. app_main creates
    // the other tasks from inside this task (as on real ESP-IDF).
    // app_main を main タスクとして起動し、スケジューラを回す。
    TaskHandle_t h = nullptr;
    xTaskCreatePinnedToCore(app_main_task, "main", 16384, nullptr, 1, &h, 0);

    // Input source (mutually exclusive, same spawn site/priority as before so the
    // no-scenario path is byte-identical):
    //  - a loaded scenario → the deterministic scenario driver task,
    //  - else the firmware's weak virtual pilot (legacy default, unchanged).
    // 入力源（排他、spawn 位置/優先度は従来同一＝シナリオ無しは byte-identical）:
    // シナリオがあればドライバ、無ければ従来の weak 仮想パイロット。
    TaskHandle_t ph = nullptr;
    if (sil_scenario_active()) {
        xTaskCreatePinnedToCore(sil_scenario_driver_task, "scn_driver", 8192, nullptr, 1, &ph, 0);
    } else if (sil_virtual_pilot_task != nullptr) {
        xTaskCreatePinnedToCore(sil_virtual_pilot_task, "pilot", 8192, nullptr, 1, &ph, 0);
    }

    std::printf("[emu] running scheduler for %lld us\n", (long long)duration_us);
    sil::rtos::Scheduler::instance().run(duration_us);

    std::printf("[emu] scheduler stopped — run complete. truth alt=%.3f m  peak alt=%.3f m\n",
                -g_plant.truth().pos_ned.z, g_peak_alt);

    // The run is complete and the scheduler has joined every task thread. We now
    // exit WITHOUT running static destructors: the firmware's singletons (state,
    // system, logger, services) are designed to live for the whole power-on life
    // of the MCU and are never destructed on real hardware (the program never
    // returns). Destructing them here, in arbitrary host order at process exit,
    // double-touches mutexes/semaphores and aborts — an artifact of the host, not
    // a firmware defect. _Exit models "the MCU was powered off": clean, faithful.
    // 実機では静的シングルトンは破棄されない（プログラムは戻らない）。ホスト終了時の
    // 任意順の破棄は mutex 二重操作で abort する（ホスト固有の人工物）。_Exit で
    // 「電源断」を忠実に再現し、破棄を走らせずクリーンに終了する。
    sil_emu_record_close();   // flush/close the events log (lines were flushed)
    sil_emu_traj_close();     // flush/close the review-video trajectory (if open)
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(0);
}
