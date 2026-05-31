/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file frames_test.cpp
 * @brief Canonical unit tests for the SIL frames module.
 *        SIL frames モジュールの正準単体テスト。
 *
 * These tests ARE the verification of the MuJoCo↔StampFly mapping
 * (coordinate_frames.md §5). If they pass, the single coordinate transform in
 * the SIL is correct.
 *
 * これらのテストが MuJoCo↔StampFly 対応の検証（coordinate_frames.md §5）。
 * 通れば、SIL で唯一の座標変換が正しい。
 */

#include <cmath>
#include <cstdio>

#include "frames.hpp"

using sil::frames::Vec3;
using sil::frames::Quat;
namespace F = sil::frames;

namespace {
int g_failures = 0;

void check(bool ok, const char* name)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

bool close(float a, float b, float tol = 1e-4f) { return std::fabs(a - b) < tol; }
bool vclose(const Vec3& a, const Vec3& b, float tol = 1e-4f)
{
    return close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol);
}
// Quaternions q and −q are the same rotation; compare up to sign.
// q と −q は同じ回転。符号を無視して比較。
bool qclose_rot(const Quat& a, const Quat& b, float tol = 1e-4f)
{
    bool same = close(a.w, b.w, tol) && close(a.x, b.x, tol) &&
                close(a.y, b.y, tol) && close(a.z, b.z, tol);
    bool neg  = close(a.w, -b.w, tol) && close(a.x, -b.x, tol) &&
                close(a.y, -b.y, tol) && close(a.z, -b.z, tol);
    return same || neg;
}
}  // namespace

int main()
{
    printf("[frames_test] canonical coordinate-frame cases\n");

    const Quat identity{1, 0, 0, 0};

    // 1. Level rest: driver-normalized accelerometer reads gravity as −9.8 on
    //    body-Z (down); gyro is zero.
    // 1. 水平静止: 加速度計のドライバ出力は body-Z(下) に重力 −9.8、ジャイロ 0。
    {
        Vec3 a = F::accel_body_frd(Vec3{0, 0, 0}, identity);
        check(vclose(a, Vec3{0, 0, -9.81f}), "level rest accel = [0,0,-9.81] (FRD)");
        Vec3 g = F::gyro_body_frd(Vec3{0, 0, 0});
        check(vclose(g, Vec3{0, 0, 0}), "level rest gyro = [0,0,0]");
    }

    // 2. Attitude: level & north-facing MuJoCo quaternion → q_nb = identity.
    //    MuJoCo level/north (FLU→ENU) = +90° yaw about Z = [0.7071,0,0,0.7071].
    // 2. 姿勢: 水平・北向きの MuJoCo クォータニオン → q_nb = 単位元。
    {
        const float s = 0.70710678f;
        Quat q_mj_level_north{s, 0, 0, s};
        Quat q_nb = F::mujoco_quat_to_qnb(q_mj_level_north);
        check(qclose_rot(q_nb, identity), "level/north MuJoCo quat -> q_nb = identity");
    }

    // 3. World ENU↔NED mapping and self-inverse.
    // 3. 世界 ENU↔NED の対応と自己逆。
    {
        Vec3 e{1, 2, 3};
        check(vclose(F::enu_to_ned(e), Vec3{2, 1, -3}), "enu_to_ned([1,2,3]) = [2,1,-3]");
        check(vclose(F::ned_to_enu(F::enu_to_ned(e)), e), "ned_to_enu(enu_to_ned(v)) = v");
    }

    // 4. Body FLU↔FRD mapping and self-inverse.
    // 4. 機体 FLU↔FRD の対応と自己逆。
    {
        Vec3 v{1, 2, 3};
        check(vclose(F::flu_to_frd(v), Vec3{1, -2, -3}), "flu_to_frd([1,2,3]) = [1,-2,-3]");
        check(vclose(F::frd_to_flu(F::flu_to_frd(v)), v), "frd_to_flu(flu_to_frd(v)) = v");
    }

    // 5. Cross-check vs MuJoCo built-in accelerometer: at rest MuJoCo's
    //    <accelerometer> reads [0,0,+9.81] in FLU; FLU→FRD must equal the
    //    driver output [0,0,-9.81]. (No sign fix — the new convention matches.)
    // 5. MuJoCo 内蔵加速度計との照合: 静止で内蔵は FLU で [0,0,+9.81]、
    //    FLU→FRD がドライバ出力 [0,0,-9.81] と一致（符号合わせ不要）。
    {
        Vec3 mj_accel_flu{0, 0, 9.81f};
        Vec3 from_mujoco = F::flu_to_frd(mj_accel_flu);
        Vec3 driver = F::accel_body_frd(Vec3{0, 0, 0}, identity);
        check(vclose(from_mujoco, driver), "MuJoCo <accelerometer> (FLU->FRD) == driver output");
    }

    // 6. Gravity magnitude is preserved under any attitude (rotation invariant).
    // 6. 任意姿勢で重力の大きさが保存（回転不変）。
    {
        // A few non-trivial attitudes via rotation vectors [rad].
        const Vec3 rotvecs[] = {{0.3f, 0, 0}, {0, -0.4f, 0}, {0, 0, 1.2f}, {0.2f, 0.5f, -0.3f}};
        bool all_ok = true;
        for (const Vec3& rv : rotvecs) {
            Quat q_nb = Quat::from_rotvec(rv);
            Vec3 a = F::accel_body_frd(Vec3{0, 0, 0}, q_nb);
            if (!close(a.norm(), 9.81f, 1e-3f)) all_ok = false;
        }
        check(all_ok, "|accel| = 9.81 for arbitrary attitude (rotation preserves norm)");
    }

    // 7. Attitude round-trip: q_nb -> MuJoCo -> q_nb.
    // 7. 姿勢往復: q_nb -> MuJoCo -> q_nb。
    {
        Quat q_nb_in = Quat::from_rotvec(Vec3{0.2f, -0.5f, 0.9f});
        q_nb_in.normalize();
        Quat q_mj = F::qnb_to_mujoco_quat(q_nb_in);
        Quat q_nb_out = F::mujoco_quat_to_qnb(q_mj);
        check(qclose_rot(q_nb_in, q_nb_out), "q_nb -> MuJoCo -> q_nb round-trip");
    }

    // 8. Velocity: a due-north velocity in ENU (+Y) maps to NED north (+X).
    // 8. 速度: ENU で真北(+Y)の速度が NED の北(+X)になる。
    {
        Vec3 v_enu_north{0, 1.5f, 0};
        check(vclose(F::enu_to_ned(v_enu_north), Vec3{1.5f, 0, 0}),
              "north velocity ENU(+Y) -> NED(+X)");
    }

    // 9. Gyro axis: pure roll rate about body-forward (X) is preserved; a left
    //    (FLU +Y) rate becomes a negative right (FRD +Y) rate.
    // 9. ジャイロ軸: 前方(X)周りのロールレートは保存、左(FLU +Y)レートは
    //    右(FRD +Y)で負になる。
    {
        check(vclose(F::gyro_body_frd(Vec3{2, 0, 0}), Vec3{2, 0, 0}), "roll rate (X) preserved");
        check(vclose(F::gyro_body_frd(Vec3{0, 1, 0}), Vec3{0, -1, 0}), "FLU +Y rate -> FRD -Y");
    }

    printf("[frames_test] %s (%d failure%s)\n",
           g_failures == 0 ? "ALL PASS" : "FAILED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
