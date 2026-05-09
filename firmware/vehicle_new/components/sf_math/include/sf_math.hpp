/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sf_math.hpp
 * @brief Minimal math library — Vector3, Quaternion, Matrix operations
 *        最小数学ライブラリ — Vector3、Quaternion、行列演算
 *
 * Header-only. Designed for ESKF and control algorithms on ESP32.
 * ヘッダーのみ。ESP32上のESKFおよび制御アルゴリズム向け設計。
 *
 * @design detailed_design.md §7 — sf_math                            [--]
 */

#pragma once

#include <cmath>
#include <cstring>

namespace sf {
namespace math {

// =============================================================================
// Vector3 — 3D vector
// 3次元ベクトル
// =============================================================================

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }

    float norm() const { return sqrtf(x*x + y*y + z*z); }
    float norm_sq() const { return x*x + y*y + z*z; }

    /// Cross product / 外積
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }

    /// Dot product / 内積
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
};

// =============================================================================
// Quaternion — unit quaternion for rotation
// 単位クォータニオン（回転表現）
// =============================================================================

struct Quat {
    float w = 1, x = 0, y = 0, z = 0;

    Quat() = default;
    Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    /// Quaternion multiplication / クォータニオン乗算
    Quat operator*(const Quat& q) const {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }

    /// Normalize to unit quaternion / 単位クォータニオンに正規化
    void normalize() {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n > 1e-10f) { w/=n; x/=n; y/=n; z/=n; }
    }

    /// Conjugate (inverse for unit quat) / 共役（単位quatの逆）
    Quat conj() const { return {w, -x, -y, -z}; }

    /// Rotate vector by this quaternion: v' = q * v * q^-1
    /// クォータニオンでベクトルを回転: v' = q * v * q^-1
    Vec3 rotate(const Vec3& v) const {
        Quat qv(0, v.x, v.y, v.z);
        Quat result = (*this) * qv * conj();
        return {result.x, result.y, result.z};
    }

    /// Inverse rotate: v' = q^-1 * v * q (body→NED to NED→body)
    /// 逆回転: v' = q^-1 * v * q（body→NEDからNED→body）
    Vec3 inv_rotate(const Vec3& v) const {
        return conj().rotate(v);
    }

    /// Build rotation matrix (3x3) from quaternion
    /// クォータニオンから回転行列（3x3）を構築
    void to_dcm(float R[3][3]) const {
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;

        R[0][0] = 1 - 2*(yy+zz); R[0][1] = 2*(xy-wz);     R[0][2] = 2*(xz+wy);
        R[1][0] = 2*(xy+wz);     R[1][1] = 1 - 2*(xx+zz); R[1][2] = 2*(yz-wx);
        R[2][0] = 2*(xz-wy);     R[2][1] = 2*(yz+wx);     R[2][2] = 1 - 2*(xx+yy);
    }

    /// Create from rotation vector (small angle: δθ)
    /// 回転ベクトル（微小角: δθ）から生成
    static Quat from_rotvec(const Vec3& rv) {
        float angle = rv.norm();
        if (angle < 1e-8f) return {1, 0, 0, 0};
        float half = angle * 0.5f;
        float s = sinf(half) / angle;
        return {cosf(half), rv.x*s, rv.y*s, rv.z*s};
    }

    /// Get Euler angles (roll, pitch, yaw) from quaternion
    /// クォータニオンからオイラー角（roll, pitch, yaw）を取得
    Vec3 to_euler() const {
        float roll  = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y));
        float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2*(w*y - z*x))));
        float yaw   = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));
        return {roll, pitch, yaw};
    }
};

// =============================================================================
// Fixed-size matrix operations for ESKF (15x15)
// ESKF用固定サイズ行列演算（15x15）
// =============================================================================

constexpr int N = 15;  // ESKF state dimension / ESKF状態次元

/// Symmetric matrix stored as flat array
/// フラット配列で格納される対称行列
struct SymMat15 {
    float data[N][N] = {};

    float& operator()(int i, int j) { return data[i][j]; }
    float  operator()(int i, int j) const { return data[i][j]; }

    /// Set to zero / ゼロに設定
    void zero() { memset(data, 0, sizeof(data)); }

    /// Set diagonal / 対角を設定
    void set_diag(int i, float val) { data[i][i] = val; }

    /// Enforce symmetry / 対称性を強制
    void symmetrize() {
        for (int i = 0; i < N; i++)
            for (int j = i+1; j < N; j++)
                data[i][j] = data[j][i] = 0.5f * (data[i][j] + data[j][i]);
    }
};

}  // namespace math
}  // namespace sf
