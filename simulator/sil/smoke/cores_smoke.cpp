/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file cores_smoke.cpp
 * @brief P1.0 smoke test — the real firmware estimator and controller run on
 *        the host through their interfaces (IEstimator / IController).
 *        P1.0 スモークテスト — 本体ファームの推定器・制御器を、ホスト上で
 *        インターフェース（IEstimator / IController）経由で動かす。
 *
 * This proves the algorithm cores (ESKF, cascade PID) and the parameter
 * system compile and run on a PC, called through the same abstract
 * interfaces that the firmware tasks use on hardware. It deliberately goes
 * THROUGH the interfaces (the SIL reset's requirement), not by reaching into
 * the concrete math cores directly.
 *
 * これは算法コア（ESKF・カスケード PID）とパラメータシステムが PC 上で
 * コンパイル・実行でき、しかも実機のタスクが使うのと同じ抽象インターフェース
 * 経由で呼べることを示す。具象な数値コアを直に叩くのではなく、必ず
 * インターフェースを通す（SIL 仕切り直しの要件）。
 */

#include <cstdio>

#include "data_types.hpp"
#include "estimator.hpp"
#include "controller.hpp"
#include "eskf_estimator.hpp"
#include "pid_controller.hpp"
#include "params.hpp"
#include "flight_state.hpp"

/// Run one predict step through IEstimator and print the state estimate.
/// IEstimator 経由で 1 回 predict し、推定値を表示する。
static sf::StateEstimate run_estimator_once()
{
    // Construct the concrete ESKF, then use it only via the interface pointer.
    // 具象 ESKF を生成し、以降はインターフェースのポインタ経由でのみ使う。
    static sf::EskfEstimator eskf;
    eskf.init();
    sf::IEstimator* estimator = &eskf;

    // Feed one IMU sample: at rest the accelerometer reads +g on the body z
    // (NED down). dt is the 400Hz control period.
    // IMU を 1 サンプル投入: 静止時、加速度計は機体 z（NED 下向き）に +g を
    // 読む。dt は 400Hz の制御周期。
    sf::ImuData imu = {};
    imu.accel[2] = 9.81f;
    estimator->predict(imu, 0.0025f);

    return estimator->getState();
}

/// Run one compute step through IController and print the control output.
/// IController 経由で 1 回 compute し、制御出力を表示する。
static sf::ControlOutput run_controller_once(const sf::StateEstimate& state)
{
    static sf::PidController pid;
    pid.init();
    sf::IController* controller = &pid;

    // Neutral sticks, mid throttle.
    // スティック中立、スロットル中央。
    sf::CommandSetpoint setpoint = {};
    setpoint.throttle = 0.5f;

    return controller->compute(state, setpoint, 0.0025f);
}

int main()
{
    // Load parameters (NVS stub is empty → compiled-in defaults are used).
    // パラメータを読み込む（NVS スタブは空 → コンパイル時デフォルトを使う）。
    sf::params::init();
    printf("[cores_smoke] params loaded: %d entries\n", sf::params::count());

    sf::StateEstimate state = run_estimator_once();
    printf("[cores_smoke] estimate: quat=(%.3f,%.3f,%.3f,%.3f) pos_z=%.3f\n",
           state.attitude[0], state.attitude[1], state.attitude[2],
           state.attitude[3], state.position[2]);

    sf::ControlOutput control = run_controller_once(state);
    printf("[cores_smoke] control: thrust=%.4f torque=(%.5f,%.5f,%.5f)\n",
           control.thrust, control.torque[0], control.torque[1], control.torque[2]);

    printf("[cores_smoke] OK — firmware estimator + controller run on host via interfaces\n");
    return 0;
}
