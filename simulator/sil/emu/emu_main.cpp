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

#include "scheduler.hpp"
#include "plant.hpp"
#include "virtual_board.hpp"
#include "topics.hpp"
#include "data_types.hpp"

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
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    const int64_t duration_us =
        (argc > 2) ? (int64_t)std::atoll(argv[2]) : 1'000'000;   // 1 s default

    std::printf("[emu] === StampFly emulator: vehicle_new app_main on host ===\n");

    // Bring up the MuJoCo Plant and connect it to the virtual board (E1).
    // MuJoCo Plant を起こし、仮想ボードに接続（E1）。
    if (!g_plant.init(model_path)) {
        std::fprintf(stderr, "[emu] plant init failed (model: %s)\n", model_path);
        return 1;
    }
    g_plant.setStartHeight(kGroundZ);
    sil_board_attach_plant(&g_plant);

    sil::rtos::Scheduler::instance().set_on_advance(on_advance);

    // BSP init + create all 14 tasks (the real firmware startup, unmodified).
    // BSP 初期化＋14タスク生成（実ファーム起動、無改変）。
    app_main();
    std::printf("[emu] app_main returned; running scheduler for %lld us\n",
                (long long)duration_us);

    sil::rtos::Scheduler::instance().run(duration_us);

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
    return 0;
}
