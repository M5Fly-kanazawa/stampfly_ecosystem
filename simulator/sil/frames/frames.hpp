/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file frames.hpp
 * @brief The ONLY coordinate transform in the SIL — MuJoCo ↔ StampFly.
 *        SIL で唯一の座標変換 — MuJoCo ↔ StampFly。
 *
 * Spec & rationale: simulator/sil/docs/coordinate_frames.md. Nowhere else in
 * the SIL performs a coordinate transform (the old SIL broke with 4 scattered
 * frame inconsistencies). Because the firmware drivers are normalized (each
 * HAL returns body-FRD physical quantities, 論点2), this module only needs the
 * MuJoCo↔StampFly WORLD/BODY rotations — no per-sensor chip remap.
 *
 * 仕様と理由: simulator/sil/docs/coordinate_frames.md。SIL の他のどこでも
 * 座標変換しない（旧 SIL は座標系不整合4箇所で崩れた）。ファームのドライバが
 * 正規化されている（各 HAL が機体 FRD の物理量を返す, 論点2）ため、本モジュールは
 * MuJoCo↔StampFly の世界/機体の回転だけでよい — チップごとの remap は不要。
 *
 * Conventions:
 *   StampFly: World = NED (Z down, g_ned=[0,0,+9.81]); Body = FRD (X-fwd,Y-right,Z-down);
 *             attitude q_nb = body→NED.
 *   MuJoCo:   World = ENU (Z up, gravity [0,0,-9.81]); Body = FLU (X-fwd,Y-left,Z-up);
 *             framequat = body(FLU)→world(ENU).
 */

#pragma once

#include "sf_math.hpp"

namespace sil {
namespace frames {

using sf::math::Vec3;
using sf::math::Quat;

// Gravitational acceleration vector in NED [m/s^2] (+Z is down).
// NED の重力加速度ベクトル [m/s^2]（+Z が下）。
inline Vec3 gravity_ned() { return {0.0f, 0.0f, 9.81f}; }

// =============================================================================
// World: ENU (MuJoCo) ↔ NED (StampFly). Swap X/Y, negate Z. Self-inverse.
// 世界: ENU(MuJoCo) ↔ NED(StampFly)。X/Y 入替・Z 反転。自分が逆。
// =============================================================================
inline Vec3 enu_to_ned(const Vec3& e) { return {e.y, e.x, -e.z}; }
inline Vec3 ned_to_enu(const Vec3& n) { return {n.y, n.x, -n.z}; }

// =============================================================================
// Body: FLU (MuJoCo) ↔ FRD (StampFly). Negate Y/Z (180° about body X). Self-inverse.
// 機体: FLU(MuJoCo) ↔ FRD(StampFly)。Y/Z 反転（body X 周り 180°）。自分が逆。
// =============================================================================
inline Vec3 flu_to_frd(const Vec3& v) { return {v.x, -v.y, -v.z}; }
inline Vec3 frd_to_flu(const Vec3& v) { return {v.x, -v.y, -v.z}; }

// Fixed orientation quaternions (see coordinate_frames.md §3.3).
// 固定の向きクォータニオン（coordinate_frames.md §3.3）。
inline Quat q_enu_to_ned() {
    const float s = 0.70710678118654752f;  // 1/√2
    return {0.0f, s, s, 0.0f};             // 180° about (1,1,0)/√2
}
inline Quat q_frd_to_flu() {
    return {0.0f, 1.0f, 0.0f, 0.0f};       // 180° about X (FRD↔FLU are the same rotation)
}

// =============================================================================
// Attitude: MuJoCo framequat (body FLU→world ENU) → StampFly q_nb (body FRD→NED).
//   q_nb = q_enu_to_ned ⊗ q_mj ⊗ q_frd_to_flu     (verified: level/north ⇒ identity)
// 姿勢: MuJoCo framequat → StampFly q_nb。
// =============================================================================
inline Quat mujoco_quat_to_qnb(const Quat& q_mj) {
    Quat q = q_enu_to_ned() * q_mj * q_frd_to_flu();
    q.normalize();
    return q;
}

// Inverse: StampFly q_nb → MuJoCo framequat (for placing a body in the MJCF).
// 逆変換: StampFly q_nb → MuJoCo framequat。
inline Quat qnb_to_mujoco_quat(const Quat& q_nb) {
    Quat q = q_enu_to_ned().conj() * q_nb * q_frd_to_flu().conj();
    q.normalize();
    return q;
}

// =============================================================================
// Sensor synthesis helpers (body FRD, driver-normalized convention, 論点2).
// 合成センサ補助（機体 FRD・ドライバ正規化規約, 論点2）。
// =============================================================================

// Accelerometer the driver returns in body FRD — gravity reads −9.8 at rest.
//   out = R_bn·(a_world_ned − g_ned) = q_nb.inv_rotate(a_world_ned − g_ned)
// 加速度計のドライバ出力（機体 FRD・水平静止で重力 −9.8）。
inline Vec3 accel_body_frd(const Vec3& a_world_ned, const Quat& q_nb) {
    return q_nb.inv_rotate(a_world_ned - gravity_ned());
}

// Gyro: MuJoCo body angular velocity (FLU site) → StampFly body rates (FRD).
// ジャイロ: MuJoCo の機体角速度(FLU site) → StampFly 機体レート(FRD)。
inline Vec3 gyro_body_frd(const Vec3& omega_flu) { return flu_to_frd(omega_flu); }

}  // namespace frames
}  // namespace sil
