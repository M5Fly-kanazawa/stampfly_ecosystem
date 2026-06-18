/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_main.cpp
 * @brief StampFly emulator host entry — runs the REAL firmware app_main on host,
 *        with the MuJoCo Plant wired to the virtual board (E1).
 *        StampFly エミュレータのホスト入口 — 実 app_main をホストで走らせ、MuJoCo
 *        Plant を仮想ボードに接続（E1）。
 *
 * E0: app_main + 14 tasks link and run against inert virtual devices.
 * E1: the BMI270 SPI device + LEDC motors are backed by the MuJoCo Plant, so the
 *     REAL BMI270 driver feeds the REAL estimator with Plant-sourced IMU, and the
 *     control output drives the motors back into the physics — a real closed
 *     hardware loop through the unmodified firmware.
 *
 * @design simulator/sil/RESET_PLAN.md §5-7 — run the real firmware unmodified  [--]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>    // strcmp — parse the SIL_EMU_NOISE level
#include <fcntl.h>    // fcntl, O_NONBLOCK — non-blocking host stdin for the CLI
#include <unistd.h>   // STDIN_FILENO
#include <fstream>    // SIL_EMU_PARAMS_FILE — general param overrides (GUI)
#include <string>
#include <sstream>

#include "scheduler.hpp"
#include "plant.hpp"
#include "virtual_board.hpp"
#include "topics.hpp"
#include "data_types.hpp"
#include "params.hpp"           // P2-3 contrast: toggle calibration.enable via env
#include "scenario.hpp"          // P8: deterministic *.scn scripted-input driver
#include "scenario_inject.hpp"   // pairing NVS seed (boot Paired unless SIL_EMU_UNPAIRED)
#include "console_feeder.hpp"    // P8: scripted console bytes → firmware stdin
#include "emu_record.hpp"        // P8: virtual-time-stamped input/event log
#include "emu_trajectory.hpp"    // P8: review-video trajectory recorder (SIL_EMU_TRAJ)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void);

namespace {

sil::Plant g_plant;
int64_t    g_last_step_us = 0;

constexpr float kGroundZ = 0.013f;   // body rest height on the ground (ENU up)

// Scheduler advance hook: step the physics by the elapsed virtual time, pushing
// the latched motor duties into the Plant (the BMI270 device then reads the new
// IMU on the firmware's next SPI read).
// スケジューラ advance フック: 経過仮想時間ぶん物理を進める。
void on_advance(int64_t now_us)
{
    if (now_us > g_last_step_us) {
        const float dt = (float)(now_us - g_last_step_us) * 1e-6f;
        sil_board_step_plant(dt);
        g_last_step_us = now_us;
        // Record a review-video trajectory row (no-op unless SIL_EMU_TRAJ was set).
        // レビュー動画用に軌跡を1行記録（SIL_EMU_TRAJ 未設定なら no-op）。
        sil_emu_traj_sample((double)now_us * 1e-6, &g_plant);
    }
}

// Build the Plant config from the environment. Sensor noise stays OFF unless
// SIL_EMU_NOISE selects a level: n0 (static white + bias + RW), n1 (n0 + broadband
// throttle vibration), or n2 (n1 + band-limited vibration + ToF/baro observation
// noise). Seed is overridable via SIL_EMU_SEED. Unset/off → byte-identical.
// 環境変数から Plant 設定を作る。SIL_EMU_NOISE で準位選択: n0（静的）/ n1（n0＋広帯域
// スロットル振動）/ n2（n1＋帯域制限振動＋ToF/baro 観測ノイズ）。未設定/off は byte-identical。
sil::Plant::Config plant_config_from_env()
{
    sil::Plant::Config cfg;   // defaults: noise OFF (clean path unchanged)

    // Battery sag/discharge model ON for the closed-loop emulator: the full firmware
    // runs power_task and reads the live INA3221 voltage to compensate thrust→duty,
    // so the dynamic supply is consistent end-to-end (Model Identity). Override with
    // SIL_EMU_BATTERY=off for an ideal constant supply (debugging / A-B contrast).
    // 閉ループ emu では電池サグモデル ON（full firmware が power_task 稼働＋実 INA3221 電圧で
    // thrust→duty 補償）。動的電源が端から端まで整合（Model Identity）。SIL_EMU_BATTERY=off で
    // 理想定電圧（デバッグ/A-B 対照）に切替。
    cfg.batt_model_enable = true;
    if (const char* batt = std::getenv("SIL_EMU_BATTERY")) {
        if (std::strcmp(batt, "off") == 0) cfg.batt_model_enable = false;
    }

    // Opt-in physics knobs (default OFF). SIL_EMU_GROUND_EFFECT = near-floor lift gain;
    // SIL_EMU_TURBULENCE = 1-3 Hz lateral turbulence force [N] (wobble-minimization study).
    // These must be read BEFORE the noise early-return below so they apply with or without noise.
    // オプトイン物理ノブ（既定OFF）。noise の早期 return より前に読む（ノイズ有無に依らず適用）。
    if (const char* ge = std::getenv("SIL_EMU_GROUND_EFFECT")) {
        float gain = (std::strcmp(ge, "1") == 0 || std::strcmp(ge, "on") == 0) ? 0.30f
                                                                               : (float)std::atof(ge);
        if (gain > 0.0f) { cfg.ge_gain = gain;
            std::printf("[emu] ground effect ON (gain=%.2f)\n", cfg.ge_gain); }
    }
    if (const char* tb = std::getenv("SIL_EMU_TURBULENCE")) {
        float amp = (float)std::atof(tb);
        if (amp > 0.0f) { cfg.turbulence_n = amp;
            std::printf("[emu] turbulence ON (amplitude=%.3f N, 1-3 Hz)\n", cfg.turbulence_n); }
    }
    // SIL_EMU_THRUST_EFF overrides the plant real-vs-ideal thrust efficiency (default
    // 0.893 = 1/1.12, the worn-motor baseline matching the firmware hover correction).
    // Raise it to model FRESH/stronger motors (e.g. 1.196 ≈ 0.893×1.34 → 1.34× thrust
    // per duty), reproducing a fresh-airframe over-thrust (lower hover duty + auto-
    // takeoff over-climb). OFF by default.
    // SIL_EMU_THRUST_EFF はプラント実/理想推力効率（既定0.893=1/1.12）を上書き。値を上げて
    // 新品/強いモータ（例 1.196≈0.893×1.34 で同 duty 1.34 倍推力）を模擬し、新機体の過剰推力
    // （低ホバー duty＋自動離陸過上昇）を再現。既定 OFF。
    if (const char* te = std::getenv("SIL_EMU_THRUST_EFF")) {
        float eff = (float)std::atof(te);
        if (eff > 0.0f) { cfg.thrust_efficiency = eff;
            std::printf("[emu] thrust efficiency override = %.3f (default 0.893)\n", cfg.thrust_efficiency); }
    }

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

}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    const int64_t duration_us =
        (argc > 2) ? (int64_t)std::atoll(argv[2]) : 1'000'000;   // 1 s default

    std::setvbuf(stdout, nullptr, _IONBF, 0);   // unbuffered: keep logs across _Exit
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    std::printf("[emu] === StampFly emulator: vehicle_new app_main on host ===\n");

    // Non-blocking, never-written pipe as stdin so the firmware's CLI read() gets
    // EAGAIN (not block, not EOF). A scenario "key" event writes scripted bytes here
    // via the console feeder. Same pattern as emu_main_generic.cpp.
    // 非ブロッキングの空パイプを stdin に被せ CLI read() を EAGAIN にする。シナリオの key
    // 事象がフィーダ経由で書き込む。emu_main_generic.cpp と同じ手法。
    int cli_pipe[2];
    if (pipe(cli_pipe) == 0) {
        fcntl(cli_pipe[0], F_SETFL, O_NONBLOCK);
        fcntl(cli_pipe[1], F_SETFL, O_NONBLOCK);
        dup2(cli_pipe[0], STDIN_FILENO);
        sil_console_set_fd(cli_pipe[1]);
    }

    // P8: open the deterministic event log + review-video trajectory if requested
    // (env from the sf CLI). Unset → both stay closed and every call is a no-op.
    // P8: 要求時に決定論イベントログ＋レビュー動画軌跡を開く。未設定なら no-op。
    sil_emu_record_open(std::getenv("SIL_EMU_EVENTS"));
    sil_emu_traj_open(std::getenv("SIL_EMU_TRAJ"));

    // P8: load a scripted input scenario (argv[3]) BEFORE the scheduler starts. A
    // parse error aborts before any firmware singleton exists (safe return).
    // P8: 入力シナリオ（argv[3]）をスケジューラ起動前にロード。パースエラーは安全に中断。
    const char* scenario_path = (argc > 3) ? argv[3] : nullptr;
    if (sil_scenario_load(scenario_path) < 0) {
        std::fprintf(stderr, "[emu] scenario load failed — aborting before run\n");
        sil_emu_record_close();
        sil_emu_traj_close();
        return 2;
    }

    // Bring up the MuJoCo Plant and connect it to the virtual board (E1).
    // MuJoCo Plant を起こし、仮想ボードに接続（E1）。
    if (!g_plant.init(model_path, plant_config_from_env())) {
        std::fprintf(stderr, "[emu] plant init failed (model: %s)\n", model_path);
        return 1;
    }
    g_plant.setStartHeight(kGroundZ);
    sil_board_attach_plant(&g_plant);

    sil::rtos::Scheduler::instance().set_on_advance(on_advance);

    // Seed the pairing NVS BEFORE app_main() so comm::init() boots the vehicle PAIRED
    // to the injector's transmitter MAC. Real hardware boots unpaired and auto-enters
    // Pairing (which blocks ARM); the flight scenarios inject RC without a pairing
    // handshake, so we pre-bind them (a vehicle that "was paired before"). The pairing
    // scenario sets SIL_EMU_UNPAIRED to skip this and exercise the real handshake.
    // ペアリング NVS を app_main() の前に seed し、comm::init() が機体をインジェクタの送信機
    // MAC にペア済みで起動させる。実機は未ペア起動→自動 Pairing（ARM を阻む）だが、飛行
    // シナリオはペアリングなしで RC を注入するため事前バインドする。ペアリングシナリオは
    // SIL_EMU_UNPAIRED を設定してこれをスキップし実ハンドシェイクを試験する。
    sil::seed_pairing_nvs();

    // BSP init + create all 14 tasks (the real firmware startup, unmodified).
    // BSP 初期化＋14タスク生成（実ファーム起動、無改変）。
    app_main();
    std::printf("[emu] app_main returned; running scheduler for %lld us\n",
                (long long)duration_us);

    // P8: spawn the scenario driver task — it injects the scripted ESP-NOW
    // ControlPackets (via the real recv seam) at their virtual times. Only when a
    // scenario was loaded; the no-scenario path is unchanged.
    // P8: シナリオドライバを起動 — 台本の ESP-NOW ControlPacket を実受信シーム経由で
    // 仮想時刻に注入する。シナリオ読込時のみ。無シナリオ経路は不変。
    if (sil_scenario_active()) {
        TaskHandle_t ph = nullptr;
        xTaskCreatePinnedToCore(sil_scenario_driver_task, "scn_driver", 8192, nullptr, 1, &ph, 0);
    }

    // P2-3 contrast: SIL_EMU_NO_CALIB disables the firmware boot calibration so the
    // estimator runs with the raw injected bias — the "without calibration" half of the
    // contrast test. Set AFTER app_main (params already loaded from the empty SIL NVS,
    // which leaves the table defaults) and BEFORE the scheduler runs (ImuTask setup
    // reads calibration.enable). Unset → calibration stays on (default), path unchanged.
    // P2-3 対照: SIL_EMU_NO_CALIB でファーム起動校正を無効化し、推定器を生バイアスのまま
    // 走らせる（対照試験の「校正なし」側）。app_main 後（params は空 SIL NVS から読まれ table
    // 既定が残る）かつ scheduler 実行前（ImuTask setup が calibration.enable を読む）に設定。
    // 未設定なら校正は ON のまま（既定）で経路不変。
    if (std::getenv("SIL_EMU_NO_CALIB")) {
        sf::params::set_bool("calibration.enable", false);
        std::printf("[emu] SIL_EMU_NO_CALIB set — boot calibration DISABLED\n");
    }

    // χ² latch-up investigation sweep hooks: override the accel-attitude robustness
    // params before the estimator reads them (same timing window as NO_CALIB above).
    // SIL_EMU_CHI2_GATE = accel χ² gate, SIL_EMU_KADAPT = adaptive-R k, SIL_EMU_ACCEL_ATT
    // = accel-attitude noise. Unset → table defaults, path unchanged (byte-identical).
    // χ²ラッチアップ調査の掃引フック: 推定器が読む前に accel 姿勢ロバスト性 param を上書き。
    if (const char* v = std::getenv("SIL_EMU_CHI2_GATE")) {
        sf::params::set_float("eskf.att.chi2_gate", std::atof(v));
        std::printf("[emu] SIL_EMU_CHI2_GATE=%s — accel χ² gate overridden\n", v);
    }
    if (const char* v = std::getenv("SIL_EMU_KADAPT")) {
        sf::params::set_float("eskf.att.k_adaptive", std::atof(v));
        std::printf("[emu] SIL_EMU_KADAPT=%s — adaptive-R k overridden\n", v);
    }
    if (const char* v = std::getenv("SIL_EMU_ACCEL_ATT")) {
        sf::params::set_float("eskf.obs.accel_att_noise", std::atof(v));
        std::printf("[emu] SIL_EMU_ACCEL_ATT=%s — accel-att noise overridden\n", v);
    }

    // General parameter overrides for the SIL GUI: SIL_EMU_PARAMS_FILE points to a text
    // file of "<param.name> <value>" lines (one per line, '#' comments allowed). Each line
    // is applied through the type-correct setter — the param's type is looked up in the
    // SSOT table (params::entry), so a float/bool/int param is set with the right call and
    // range-validated. Same timing window as the env overrides above (after app_main loads
    // the table defaults, before the scheduler reads them). Unset → table defaults, path
    // unchanged. This is how the GUI's parameter panel feeds a run without a rebuild.
    // SIL GUI 用の汎用パラメータ上書き: SIL_EMU_PARAMS_FILE は "<param名> <値>" 行のテキスト
    // ファイル（1行1個、'#' コメント可）。各行を型に正しいセッタで適用（型は SSOT テーブル
    // params::entry で引く）→ float/bool/int を正しい呼び出しで範囲検証付き設定。上の env
    // 上書きと同じタイミング窓。未設定なら既定のまま。GUI のパラメータパネルが再ビルド無しで
    // 走行に値を渡す経路。
    if (const char* path = std::getenv("SIL_EMU_PARAMS_FILE")) {
        std::ifstream pf(path);
        if (!pf) {
            std::printf("[emu] SIL_EMU_PARAMS_FILE=%s — cannot open, skipped\n", path);
        } else {
            std::string line;
            int applied = 0;
            while (std::getline(pf, line)) {
                size_t hash = line.find('#');
                if (hash != std::string::npos) line.erase(hash);   // strip comment
                std::istringstream ls(line);
                std::string name; double value;
                if (!(ls >> name >> value)) continue;              // blank / malformed
                // Look up the param's type in the SSOT table.
                // パラメータの型を SSOT テーブルで引く。
                bool found = false;
                sf::params::ParamType type = sf::params::ParamType::FLOAT;
                for (int i = 0; i < sf::params::count(); ++i) {
                    const sf::params::ParamEntry* e = sf::params::entry(i);
                    if (e && name == e->name) { type = e->type; found = true; break; }
                }
                if (!found) {
                    std::printf("[emu] PARAMS_FILE: unknown param '%s' — skipped\n", name.c_str());
                    continue;
                }
                bool ok = false;
                switch (type) {
                    case sf::params::ParamType::FLOAT: ok = sf::params::set_float(name.c_str(), (float)value); break;
                    case sf::params::ParamType::BOOL:  ok = sf::params::set_bool(name.c_str(), value != 0.0);  break;
                    case sf::params::ParamType::INT:   ok = sf::params::set_int(name.c_str(), (int32_t)value); break;
                }
                if (ok) ++applied;
                else std::printf("[emu] PARAMS_FILE: '%s'=%g rejected (out of range?)\n", name.c_str(), value);
            }
            std::printf("[emu] SIL_EMU_PARAMS_FILE=%s — %d param(s) overridden\n", path, applied);
        }
    }

    sil::rtos::Scheduler::instance().run(duration_us);

    sil_emu_record_close();   // flush/close the events log
    sil_emu_traj_close();     // flush/close the review-video trajectory (if open)

    // --- post-run validation: did the real estimator track the Plant? ---------
    // 実行後の検証: 実推定器が Plant を追従したか。
    sf::ImuData imu = sf::sensor_imu.latest();
    sf::StateEstimate est = sf::estimate_state.latest();
    sil::Plant::Truth truth = g_plant.truth();
    std::printf("[emu] scheduler stopped — emulator run complete\n");
    std::printf("[emu] IMU (body-FRD, via real BMI270 driver): "
                "accel=[%.3f %.3f %.3f] m/s^2  gyro=[%.4f %.4f %.4f] rad/s\n",
                imu.accel[0], imu.accel[1], imu.accel[2],
                imu.gyro[0], imu.gyro[1], imu.gyro[2]);
    std::printf("[emu] estimate quat=[%.4f %.4f %.4f %.4f]  truth alt=%.3f m\n",
                est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3],
                -truth.pos_ned.z);
    // Confirm the real comm decoded the controller's SSOT ControlPacket: the last
    // command_setpoint should reflect the injected sticks (non-zero when the
    // scenario commanded throttle/attitude). Before the 14-byte alignment fix the
    // comm rejected every packet and this stayed all-zero. 受信確認: 実 comm が SSOT
    // ControlPacket を復号したか。注入スティックが反映されれば成功（整合前は全て0）。
    sf::CommandSetpoint cmd = sf::command_setpoint.latest();
    std::printf("[emu] last command_setpoint: throttle=%.3f roll=%.3f pitch=%.3f yaw=%.3f (src=%u)\n",
                cmd.throttle, cmd.roll, cmd.pitch, cmd.yaw, cmd.source);
    sf::SystemMode sm = sf::system_mode.latest();
    std::printf("[emu] final state=%u sub_mode=%u armed=%d | imu.ts=%u est.ts=%u\n",
                sm.state, sm.sub_mode, (int)sm.armed,
                sf::sensor_imu.latest().timestamp, est.timestamp);
    return 0;
}
