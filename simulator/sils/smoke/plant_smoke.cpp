/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file plant_smoke.cpp
 * @brief A/4 smoke — the SILS Plant hovers and synthesizes correct FRD sensors.
 *        A/4 スモーク — SILS Plant がホバーし、正しい FRD センサを合成する。
 *
 * Two checks, all against physics truth (no firmware, no real data):
 *   (1) HOVER: 4 motors at hover duty → altitude held; synthetic IMU reads
 *       [0,0,-9.81] (driver-normalized), gyro ~0; ToF ~0.5, baro ~0.5, flow ~0;
 *       attitude level.
 *   (2) YAW REACTION SIGN: spin the CCW diagonal pair (M1 FR, M3 RL) up and the
 *       CW pair (M2 RR, M4 FL) down (total thrust ~mg) → the body yaws nose-right,
 *       i.e. truth.omega_frd.z > 0, and the synthetic gyro agrees with truth.
 *
 * 2つの検査、すべて物理真値に対して（ファーム無し・実データ無し）:
 *   (1) ホバー: 4モータをホバー duty に → 高度維持、合成 IMU が [0,0,-9.81]
 *       （ドライバ正規化）、ジャイロ ~0、ToF ~0.5、baro ~0.5、flow ~0、水平。
 *   (2) ヨー反トルク符号: CCW 対角ペア(M1 FR, M3 RL)を上げ CW ペア(M2 RR, M4 FL)を
 *       下げる（総推力 ~mg） → 機体は機首右へヨー、すなわち truth.omega_frd.z > 0、
 *       合成ジャイロが真値と一致。
 */

#include <cmath>
#include <cstdio>

#include "plant.hpp"

using sf::math::Vec3;

namespace {
int g_failures = 0;

void check(bool ok, const char* name)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}
}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sils/models/stampfly.xml";

    sils::Plant::Config cfg;  // documented motor curve: Am/Bm/Cm/Ct, v_batt 3.7
    const float mg = cfg.mass * cfg.g;

    // =========================================================================
    // (1) HOVER
    // =========================================================================
    printf("[plant_smoke] --- (1) hover ---\n");
    sils::Plant plant;
    if (!plant.init(model_path, cfg)) {
        fprintf(stderr, "[plant_smoke] plant init failed (model: %s)\n", model_path);
        return 1;
    }

    // Hover duty from the real motor curve (inverts V=Am·ω²+Bm·ω+Cm, T=Ct·ω²).
    // 実モータ曲線から導くホバー duty（V=Am·ω²+Bm·ω+Cm, T=Ct·ω² の逆算）。
    const float hover_duty = plant.hoverDuty();
    printf("[plant_smoke] mg=%.4f N, hover_duty=%.4f, thrust/motor=%.4f N\n",
           mg, hover_duty, plant.dutyToThrust(hover_duty));

    sf::MotorOutput hover{};
    for (int i = 0; i < 4; ++i) hover.duty[i] = hover_duty;
    plant.setDuty(hover);
    plant.primeMotors();  // start at hover speed (skip spool-up lag)

    float start_z = plant.truth().pos_ned.z;
    for (int s = 0; s < 800; ++s) plant.step(0.0025f);  // 2 s

    sils::Plant::Truth t = plant.truth();
    sf::ImuData  imu  = plant.imu();
    sf::TofData  tof  = plant.tof();
    sf::BaroData baro = plant.baro();
    sf::FlowData flow = plant.flow();
    float end_z = t.pos_ned.z;

    printf("[plant_smoke] altitude: start=%.3f end=%.3f m (truth alt=%.3f)\n",
           -start_z, -end_z, -t.pos_ned.z);
    printf("[plant_smoke] imu accel(FRD)=[%.3f %.3f %.3f] gyro=[%.4f %.4f %.4f]\n",
           imu.accel[0], imu.accel[1], imu.accel[2], imu.gyro[0], imu.gyro[1], imu.gyro[2]);
    printf("[plant_smoke] tof=%.3f m  baro=%.3f m  flow=[%d %d]\n",
           tof.distance, baro.altitude, flow.dx, flow.dy);

    check(std::fabs(end_z - start_z) < 0.02f, "hover holds altitude (|dz| < 2 cm)");
    check(std::fabs(imu.accel[0]) < 0.2f && std::fabs(imu.accel[1]) < 0.2f &&
          std::fabs(imu.accel[2] + 9.81f) < 0.2f, "imu accel(FRD) = [0,0,-9.81]");
    check(std::sqrt(imu.gyro[0]*imu.gyro[0] + imu.gyro[1]*imu.gyro[1] +
                    imu.gyro[2]*imu.gyro[2]) < 0.05f, "gyro ~ 0 at hover");
    check(std::fabs(tof.distance - 0.5f) < 0.02f && tof.valid, "tof distance ~ 0.5 m");
    check(std::fabs(baro.altitude - 0.5f) < 0.02f, "baro altitude ~ 0.5 m");
    check(std::abs(flow.dx) <= 1 && std::abs(flow.dy) <= 1, "flow ~ 0 at stationary hover");

    Vec3 down_body = t.q_nb.inv_rotate(Vec3{0, 0, 1});
    check(std::fabs(down_body.z - 1.0f) < 0.02f, "attitude level (NED-down → body-down)");

    // =========================================================================
    // (2) YAW REACTION TORQUE SIGN
    // CCW pair (M1 FR=duty[0], M3 RL=duty[2]) up, CW pair (M2 RR=duty[1],
    // M4 FL=duty[3]) down, keeping total thrust ~ mg. Expect +yaw (nose-right):
    // truth.omega_frd.z > 0, and the synthetic gyro.z agrees.
    // =========================================================================
    printf("[plant_smoke] --- (2) yaw reaction sign ---\n");
    sils::Plant plant2;
    if (!plant2.init(model_path, cfg)) {
        fprintf(stderr, "[plant_smoke] plant2 init failed\n");
        return 1;
    }

    // thrust_ccw = mg/4·(1+r), thrust_cw = mg/4·(1-r); total stays mg.
    // duty = sqrt(thrust/k). 推力比 r で対角ペアを上下させ、総推力は mg のまま。
    const float r = 0.1f;
    const float duty_ccw = hover_duty * std::sqrt(1.0f + r);
    const float duty_cw  = hover_duty * std::sqrt(1.0f - r);
    sf::MotorOutput yawcmd{};
    yawcmd.duty[0] = duty_ccw;  // M1 FR (CCW)
    yawcmd.duty[2] = duty_ccw;  // M3 RL (CCW)
    yawcmd.duty[1] = duty_cw;   // M2 RR (CW)
    yawcmd.duty[3] = duty_cw;   // M4 FL (CW)
    plant2.setDuty(yawcmd);
    plant2.primeMotors();

    for (int s = 0; s < 100; ++s) plant2.step(0.0025f);  // 0.25 s

    sils::Plant::Truth t2 = plant2.truth();
    sf::ImuData imu2 = plant2.imu();
    printf("[plant_smoke] yaw test: omega_frd=[%.3f %.3f %.3f] gyro=[%.3f %.3f %.3f]\n",
           t2.omega_frd.x, t2.omega_frd.y, t2.omega_frd.z,
           imu2.gyro[0], imu2.gyro[1], imu2.gyro[2]);

    check(t2.omega_frd.z > 0.05f, "CCW-pair-up → +yaw (nose-right, omega_frd.z > 0)");
    check(std::fabs(t2.omega_frd.x) < 0.2f && std::fabs(t2.omega_frd.y) < 0.2f,
          "no roll/pitch from a balanced diagonal split");
    check(std::fabs(imu2.gyro[2] - t2.omega_frd.z) < 1e-3f,
          "synthetic gyro.z == truth omega_frd.z (gyro synthesis correct)");

    printf("[plant_smoke] %s (%d failure%s)\n",
           g_failures == 0 ? "OK" : "FAILED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 2;
}
