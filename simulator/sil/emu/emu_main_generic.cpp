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
#include <cstring>    // strcmp — parse the SIL_EMU_NOISE level
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

// Build the Plant config from the environment. Sensor noise stays OFF unless
// SIL_EMU_NOISE selects a level: n0 (static white + bias + RW), n1 (n0 + broadband
// throttle vibration), or n2 (n1 + band-limited vibration + ToF/baro observation
// noise). Seed is overridable via SIL_EMU_SEED. Unset/off reproduces the previous
// behaviour byte-for-byte (clean path unchanged).
// 環境変数から Plant 設定を作る。SIL_EMU_NOISE で準位選択: n0（静的）/ n1（n0＋広帯域
// スロットル振動）/ n2（n1＋帯域制限振動＋ToF/baro 観測ノイズ）。シードは SIL_EMU_SEED で
// 上書き可。未設定/off は従来と byte-identical。
sil::Plant::Config plant_config_from_env()
{
    sil::Plant::Config cfg;   // defaults: noise OFF (clean path unchanged)
    const char* noise = std::getenv("SIL_EMU_NOISE");
    if (!noise) return cfg;
    const bool n0 = (std::strcmp(noise, "n0") == 0);
    const bool n1 = (std::strcmp(noise, "n1") == 0);
    const bool n2 = (std::strcmp(noise, "n2") == 0);
    if (n0 || n1 || n2) {
        cfg.noise.enable        = true;
        cfg.noise.vib_enable    = (n1 || n2);   // throttle-dependent vibration
        cfg.noise.vib_bandlimit = n2;           // n2: band-limit the vibration spectrum
        cfg.noise.obs_enable    = n2;           // n2: ToF/baro observation noise
        if (const char* seed = std::getenv("SIL_EMU_SEED")) {
            cfg.noise.seed = (uint32_t)std::atoi(seed);
        }
        std::printf("[emu] sensor noise: %s ON (seed=%u%s%s)\n", noise, cfg.noise.seed,
                    (n1 || n2) ? ", vibration" : "",
                    n2 ? " (band-limited) + ToF/baro obs" : "");
    }
    return cfg;
}

// Apply the SIL_EMU_GROUND_EFFECT knob on top of the noise config. "1"/"on" enables a
// default-strength ground effect; a numeric value sets the gain directly. OFF by default
// (clean path byte-identical). Models the near-floor lift boost so the touchdown "float"
// is reproduced and the firmware's stalled-descent land detector can be verified.
// SIL_EMU_GROUND_EFFECT ノブをノイズ設定に重ねて適用。"1"/"on" で既定強度、数値で gain 直接指定。
// 既定 OFF（クリーン経路バイト一致）。接地近傍の揚力ブーストを模し着陸「フロート」を再現して、
// ファームの降下停滞接地検出器を検証可能にする。
void apply_ground_effect_from_env(sil::Plant::Config& cfg)
{
    const char* ge = std::getenv("SIL_EMU_GROUND_EFFECT");
    if (!ge) return;
    float gain = 0.30f;   // default strength when just enabled / 有効化時の既定強度
    if (std::strcmp(ge, "1") != 0 && std::strcmp(ge, "on") != 0) {
        gain = static_cast<float>(std::atof(ge));   // explicit numeric gain / 明示 gain
    }
    if (gain <= 0.0f) return;
    cfg.ge_gain = gain;
    std::printf("[emu] ground effect ON (gain=%.2f, height=%.3fm)\n",
                cfg.ge_gain, cfg.ge_height);
}

// SIL_EMU_TURBULENCE = the turbulence force amplitude [N] (deterministic 1–3 Hz lateral
// disturbance) for the wobble-minimization benchmark. OFF by default. e.g. 0.03.
// SIL_EMU_TURBULENCE = 乱流力振幅[N]（決定論的1–3Hz横外乱）。既定 OFF。
void apply_turbulence_from_env(sil::Plant::Config& cfg)
{
    const char* tb = std::getenv("SIL_EMU_TURBULENCE");
    if (!tb) return;
    float amp = static_cast<float>(std::atof(tb));
    if (amp <= 0.0f) return;
    cfg.turbulence_n = amp;
    std::printf("[emu] turbulence ON (amplitude=%.3f N, 1-3 Hz)\n", cfg.turbulence_n);
}

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
    sil::Plant::Config plant_cfg = plant_config_from_env();
    apply_ground_effect_from_env(plant_cfg);   // opt-in ground effect (default OFF)
    apply_turbulence_from_env(plant_cfg);       // opt-in turbulence disturbance (default OFF)
    if (!g_plant.init(model_path, plant_cfg)) {
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
