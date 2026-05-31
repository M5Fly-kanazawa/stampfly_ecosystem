/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file physics_smoke.cpp
 * @brief P1.2 smoke test — StampFly model hovers; built-in sensors agree with
 *        the StampFly frame convention via the frames module.
 *        P1.2 スモーク — StampFly モデルがホバーし、内蔵センサが frames 経由で
 *        StampFly 規約と一致する。
 *
 * Applies hover thrust (mg/4 per motor) to the StampFly MJCF and checks:
 *   - the body holds altitude and stays level (balanced thrust = weight),
 *   - MuJoCo's built-in accelerometer, mapped FLU→FRD, reads [0,0,-9.81]
 *     (the driver-normalized convention: gravity -9.8 on body-down at hover),
 *   - the gyro reads ~0, and the truth quaternion maps to a level q_nb.
 *
 * ホバー推力（mg/4）を StampFly MJCF に与え、機体が高度を保ち水平を保つこと、
 * 内蔵加速度計を FLU→FRD で写すと [0,0,-9.81]（ドライバ正規化規約）になること、
 * ジャイロ ~0、真値クォータニオンが水平の q_nb に写ること、を確認する。
 */

#include <cmath>
#include <cstdio>

#include <mujoco/mujoco.h>

#include "frames.hpp"

using sil::frames::Vec3;
using sil::frames::Quat;
namespace F = sil::frames;

namespace {
constexpr int kSteps = 800;          // 2 s at 2.5 ms / 2.5ms で 2 秒
int g_failures = 0;

void check(bool ok, const char* name)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

// Read a named sensor's first 3 (or 4) components from mjData.
// 名前付きセンサの先頭3(または4)成分を mjData から読む。
const double* sensor_ptr(const mjModel* m, const mjData* d, const char* name)
{
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    return (id < 0) ? nullptr : d->sensordata + m->sensor_adr[id];
}
}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) {
        fprintf(stderr, "[physics_smoke] load failed: %s\n", error);
        return 1;
    }
    mjData* d = mj_makeData(m);

    // Hover thrust per motor = m·g / 4, from the model's drone mass.
    // モータ毎のホバー推力 = m·g / 4（モデルの機体質量から）。
    int body_id = mj_name2id(m, mjOBJ_BODY, "drone");
    double mass = m->body_mass[body_id];
    double hover_total = mass * 9.81;
    double hover_per_motor = hover_total / 4.0;
    printf("[physics_smoke] mass=%.4f kg, hover total=%.4f N, per motor=%.4f N\n",
           mass, hover_total, hover_per_motor);

    // Compute sensors at the initial state before reading them (mj_step has
    // not run yet, so sensordata would otherwise be zero).
    // 初期状態のセンサを計算してから読む（mj_step 前は sensordata が 0 のため）。
    mj_forward(m, d);
    const double* p0 = sensor_ptr(m, d, "body_pos");
    double start_z = p0 ? p0[2] : 0.0;

    for (int step = 0; step < kSteps; ++step) {
        for (int i = 0; i < 4 && i < m->nu; ++i) d->ctrl[i] = hover_per_motor;
        mj_step(m, d);
    }

    // Read final state + built-in sensors.
    // 最終状態＋内蔵センサを読む。
    const double* pos   = sensor_ptr(m, d, "body_pos");
    const double* accel = sensor_ptr(m, d, "imu_accel");   // FLU (site) frame
    const double* gyro  = sensor_ptr(m, d, "imu_gyro");    // FLU
    const double* quat  = sensor_ptr(m, d, "body_quat");   // body(FLU)->world(ENU)

    double end_z = pos ? pos[2] : 0.0;
    printf("[physics_smoke] altitude: start=%.3f m -> end=%.3f m\n", start_z, end_z);

    // Map MuJoCo built-in sensors (FLU) to StampFly FRD via the frames module.
    // 内蔵センサ(FLU) を frames で StampFly FRD へ写す。
    Vec3 accel_frd = F::flu_to_frd(Vec3{(float)accel[0], (float)accel[1], (float)accel[2]});
    Vec3 gyro_frd  = F::flu_to_frd(Vec3{(float)gyro[0],  (float)gyro[1],  (float)gyro[2]});
    Quat q_nb = F::mujoco_quat_to_qnb(Quat{(float)quat[0], (float)quat[1], (float)quat[2], (float)quat[3]});

    printf("[physics_smoke] accel(FRD)=[%.3f %.3f %.3f]  gyro(FRD)=[%.4f %.4f %.4f]\n",
           accel_frd.x, accel_frd.y, accel_frd.z, gyro_frd.x, gyro_frd.y, gyro_frd.z);

    // 1. Hover: altitude held within 2 cm.
    // 1. ホバー: 高度が 2 cm 以内に保たれる。
    check(std::fabs(end_z - start_z) < 0.02, "hover holds altitude (|dz| < 2 cm)");

    // 2. Accelerometer at hover (FLU->FRD) = [0,0,-9.81] (level, gravity -9.8 down).
    // 2. ホバー時の加速度計(FLU->FRD) = [0,0,-9.81]（水平・重力 -9.8 が下）。
    check(std::fabs(accel_frd.x) < 0.2 && std::fabs(accel_frd.y) < 0.2 &&
          std::fabs(accel_frd.z + 9.81f) < 0.2, "accel(FRD) = [0,0,-9.81] at hover");

    // 3. Gyro ~ 0 (no rotation at balanced hover).
    // 3. ジャイロ ~ 0（均衡ホバーで無回転）。
    check(gyro_frd.norm() < 0.05f, "gyro ~ 0 at hover");

    // 4. Level: body Z (down) component of q_nb is aligned (no roll/pitch).
    //    A level attitude rotates NED down [0,0,1] to body down ~[0,0,1].
    // 4. 水平: q_nb の機体Z(下) が一致（ロール/ピッチ無し）。
    Vec3 down_body = q_nb.inv_rotate(Vec3{0, 0, 1});
    check(std::fabs(down_body.z - 1.0f) < 0.02f, "attitude level (NED-down maps to body-down)");

    printf("[physics_smoke] %s (%d failure%s)\n",
           g_failures == 0 ? "OK" : "FAILED",
           g_failures, g_failures == 1 ? "" : "s");

    mj_deleteData(d);
    mj_deleteModel(m);
    return g_failures == 0 ? 0 : 2;
}
