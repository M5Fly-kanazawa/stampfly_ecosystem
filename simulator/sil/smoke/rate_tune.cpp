/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file rate_tune.cpp
 * @brief SIL rate-loop step test — characterize/recalibrate the yaw-rate gains.
 *        SIL レートループのステップ試験 — ヨーレートゲインの特性把握/再較正。
 *
 * The closed-loop hover surfaced that a 1 rad/s yaw command tracked at only
 * 0.05 rad/s: the mixer scales yaw by kappa (0.00971) so the yaw gain needs to
 * be ~1/kappa larger than roll/pitch. This harness commands a yaw-rate step and
 * reports tracking so gains can be swept (CLAUDE.md: control params need SIL
 * numerical backing). Gains are injected at runtime via the parameter system
 * before the controller loads them, so a sweep is just repeated runs.
 *
 * 閉ループホバーで 1 rad/s のヨー指令が 0.05 rad/s しか追従しないと判明。ミキサーが
 * ヨーを kappa(0.00971) で縮小するため、ヨーゲインはロール/ピッチの ~1/kappa 倍要る。
 * 本ハーネスはヨーレートのステップを与えて追従を報告し、ゲインを掃引できる
 * （CLAUDE.md: 制御パラメータは SIL の数値裏付けが必須）。ゲインはコントローラが
 * 読む前にパラメータ系へ実行時注入するので、掃引は実行の繰り返しだけ。
 *
 * Usage: rate_tune <model.xml> <yaw_stick> <rate.yaw.kp> <rate.yaw.ti>
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "scheduler.hpp"
#include "topics.hpp"
#include "params.hpp"
#include "data_types.hpp"
#include "flight_state.hpp"
#include "config.hpp"
#include "plant.hpp"
#include "plant_bridge.hpp"

void ImuTask(void*);
void ControlTask(void*);
void StateTask(void*);
// ControlTask self-registers its handle (sf::tasks::control_handle(), R3).
// ControlTask が自分のハンドルを登録する（sf::tasks::control_handle(), R3）。

using sil::rtos::Scheduler;

namespace {
constexpr int64_t kStepUs     = 500000;    // command the step at 0.5 s
constexpr float   kMaxYawRate = 5.0f;      // config: max_yaw_rate_

sil::Plant g_plant;
float g_hover_duty = 0.0f;
// Physical hover throttle: thrust [N] = throttle · max_thrust_(0.672), so for
// hover thrust mg → throttle = mg / 0.672 (B^-1 mixer consumes physical units).
// 物理ホバー throttle: 推力[N] = throttle·max_thrust_(0.672) なので mg/0.672。
const float g_hover_throttle = (0.037f * 9.81f) / 0.672f;
int64_t g_last_step_us = 0;
float g_yaw_stick = 0.2f;
int64_t g_sim_us = 3000000;     // run duration (argv[5])
int64_t g_ss_window_us = 2000000;  // steady-state window starts here (= g_sim_us − 1s)

// Tracking samples taken in the steady window [2.0, 3.0] s.
double g_ss_sum = 0.0;     // mean steady-state yaw rate
int    g_ss_n = 0;
float  g_peak_yaw = 0.0f;  // overshoot peak
float  g_max_tilt = 0.0f;  // hover must hold while yawing
float  g_max_alt_err = 0.0f;

void physics(int64_t now_us)
{
    static bool armed = false;
    if (!armed) {
        armed = true;
        sf::SystemMode mode = {};
        mode.state = static_cast<uint8_t>(sf::FlightState::FLYING);
        mode.armed = true;
        mode.timestamp = static_cast<uint32_t>(now_us);
        sf::system_mode.publish(mode);

        sf::CommandSetpoint sp = {};
        sp.throttle = g_hover_throttle;       // hover, no yaw yet
        sp.timestamp = static_cast<uint32_t>(now_us);
        sf::command_setpoint.publish(sp);
    }

    static bool stepped = false;
    if (!stepped && now_us >= kStepUs) {
        stepped = true;
        sf::CommandSetpoint sp = {};
        sp.throttle = g_hover_throttle;
        sp.yaw = g_yaw_stick;             // step the yaw-rate command
        sp.timestamp = static_cast<uint32_t>(now_us);
        sf::command_setpoint.publish(sp);
    }

    sf::MotorOutput cmd = sf::actuator_motor.latest();
    g_plant.setDuty(cmd);
    if (now_us > g_last_step_us) {
        g_plant.step(static_cast<float>(now_us - g_last_step_us) * 1e-6f);
        g_last_step_us = now_us;
    }
    sil::bridge::current_imu = g_plant.imu();

    sil::Plant::Truth t = g_plant.truth();
    if (t.omega_frd.z > g_peak_yaw) g_peak_yaw = t.omega_frd.z;
    float tilt = std::acos(std::fmin(1.0f, std::fmax(-1.0f,
                    t.q_nb.inv_rotate(sf::math::Vec3{0, 0, 1}).z))) * 180.0f / 3.14159265f;
    if (tilt > g_max_tilt) g_max_tilt = tilt;
    float ae = std::fabs(-t.pos_ned.z - 0.5f);
    if (ae > g_max_alt_err) g_max_alt_err = ae;
    if (now_us >= g_ss_window_us) { g_ss_sum += t.omega_frd.z; ++g_ss_n; }
}
}  // namespace

int main(int argc, char** argv)
{
    const char* model_path = (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    g_yaw_stick   = (argc > 2) ? std::atof(argv[2]) : 0.2f;
    float yaw_kp  = (argc > 3) ? std::atof(argv[3]) : 5.31e-3f;
    float yaw_ti  = (argc > 4) ? std::atof(argv[4]) : 1.6f;
    g_sim_us = (argc > 5) ? static_cast<int64_t>(std::atof(argv[5]) * 1e6) : 3000000;
    float yaw_td  = (argc > 6) ? std::atof(argv[6]) : 0.01f;
    g_ss_window_us = g_sim_us - 1000000;  // mean over the final second

    sf::params::init();
    sf::topics_init();

    // Inject the gains under test BEFORE the controller's loadParams() runs.
    // 試験ゲインをコントローラの loadParams() 前に注入する。
    sf::params::set_float("rate.yaw.kp", yaw_kp);
    sf::params::set_float("rate.yaw.ti", yaw_ti);
    sf::params::set_float("rate.yaw.td", yaw_td);

    if (!g_plant.init(model_path)) {
        fprintf(stderr, "[rate_tune] plant init failed\n");
        return 1;
    }
    g_hover_duty = g_plant.hoverDuty();

    sf::MotorOutput m0{};
    for (int i = 0; i < 4; ++i) m0.duty[i] = g_hover_duty;
    g_plant.setDuty(m0);
    g_plant.primeMotors();
    sf::actuator_motor.publish(m0);
    sil::bridge::current_imu = g_plant.imu();
    sil::bridge::has_plant = true;

    Scheduler& scheduler = Scheduler::instance();
    scheduler.set_on_advance(physics);

    TaskHandle_t hs = nullptr, hc = nullptr, hi = nullptr;
    xTaskCreatePinnedToCore(StateTask,   "StateTask",   config::STACK_STATE,
                            nullptr, config::PRIORITY_STATE,   &hs, 1);
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", config::STACK_CONTROL,
                            nullptr, config::PRIORITY_CONTROL, &hc, 1);
    // ControlTask registers its own handle in setup (sf::tasks::control_handle()).
    // ControlTask は setup で自分のハンドルを登録する。
    xTaskCreatePinnedToCore(ImuTask,     "ImuTask",     config::STACK_IMU,
                            nullptr, config::PRIORITY_IMU,     &hi, 1);

    scheduler.run(g_sim_us);

    float cmd_rate = g_yaw_stick * kMaxYawRate;
    float ss = (g_ss_n > 0) ? static_cast<float>(g_ss_sum / g_ss_n) : 0.0f;
    float overshoot = (cmd_rate > 1e-6f) ? (g_peak_yaw - cmd_rate) / cmd_rate * 100.0f : 0.0f;
    float err_pct = (cmd_rate > 1e-6f) ? (ss - cmd_rate) / cmd_rate * 100.0f : 0.0f;

    printf("kp=%.4g ti=%.3g | cmd=%.3f ss=%.3f rad/s (err %.1f%%) "
           "peak=%.3f (overshoot %.1f%%) | maxtilt=%.2fdeg maxalt=%.3fm\n",
           yaw_kp, yaw_ti, cmd_rate, ss, err_pct, g_peak_yaw, overshoot,
           g_max_tilt, g_max_alt_err);
    return 0;
}
