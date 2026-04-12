/**
 * @file quad_model.hpp
 * @brief Simplified quadrotor dynamics model for SIL simulation
 *        SILシミュレーション用簡易クアッドロータダイナミクスモデル
 *
 * 6-DOF rigid body simulation with motor dynamics.
 * StampFly physical parameters (mass=37g, arm=23mm).
 *
 * モータダイナミクス付き6自由度剛体シミュレーション。
 * StampFly物理パラメータ（質量=37g、アーム長=23mm）。
 *
 * @design requirements.md §10 — SIL simulator                        [--]
 */

#pragma once

#include "sf_math.hpp"
#include <cstdio>

namespace sf {
namespace sim {

using namespace math;

/// Quadrotor physical parameters / クアッドロータ物理パラメータ
struct QuadParams {
    float mass       = 0.037f;   // [kg] StampFly mass / 質量
    float arm_length = 0.023f;   // [m] Motor-to-center / モーター中心距離
    float Ixx        = 5.0e-6f;  // [kg*m²] Roll inertia / ロール慣性
    float Iyy        = 5.0e-6f;  // [kg*m²] Pitch inertia / ピッチ慣性
    float Izz        = 9.0e-6f;  // [kg*m²] Yaw inertia / ヨー慣性
    float k_thrust   = 0.168f;   // [N] Max thrust per motor / モーター最大推力
    float k_torque   = 0.00971f; // [m] Torque-to-thrust ratio / トルク推力比
    float gravity    = 9.81f;    // [m/s²]
    float motor_tc   = 0.02f;    // [s] Motor time constant / モーター時定数
    float drag_coeff = 0.01f;    // [N/(m/s)] Linear drag / 線形抗力
};

/// Quadrotor state / クアッドロータ状態
struct QuadState {
    Vec3 position = {};     // [m] NED / 位置
    Vec3 velocity = {};     // [m/s] NED / 速度
    Quat attitude = {};     // Quaternion / 姿勢
    Vec3 angular_rate = {}; // [rad/s] body frame / ボディ角速度
    float motor_speed[4] = {}; // [0..1] Current motor speed / モーター速度
};

/// Simulated sensor output / シミュレーテッドセンサ出力
struct SimSensors {
    Vec3 accel;       // [m/s²] body frame / ボディ加速度
    Vec3 gyro;        // [rad/s] body frame / ボディ角速度
    float tof_bottom; // [m] distance to ground / 地面距離
    float baro_alt;   // [m] barometric altitude / 気圧高度
    Vec3 mag;         // [uT] magnetic field / 磁場
};

/// Quadrotor dynamics model / クアッドロータダイナミクスモデル
class QuadModel {
public:
    /// Initialize with default parameters / デフォルトパラメータで初期化
    void init(const QuadParams& params = {})
    {
        params_ = params;
        state_ = {};
        state_.attitude = {1, 0, 0, 0};  // Identity
        // Start 0.05m above ground / 地面から0.05m上で開始
        state_.position.z = -0.05f;  // NED: up is negative
    }

    /// Step simulation forward / シミュレーションを1ステップ進める
    ///
    /// @param motor_cmd Motor commands [0..1] x4 / モーターコマンド
    /// @param dt Time step [s] / タイムステップ
    void step(const float motor_cmd[4], float dt)
    {
        // Motor dynamics (first-order lag) / モーターダイナミクス（1次遅れ）
        for (int i = 0; i < 4; i++) {
            float cmd = motor_cmd[i];
            if (cmd < 0) cmd = 0;
            if (cmd > 1) cmd = 1;
            float alpha = dt / (params_.motor_tc + dt);
            state_.motor_speed[i] += alpha * (cmd - state_.motor_speed[i]);
        }

        // Compute thrust per motor / モーター毎の推力を計算
        float thrust[4];
        for (int i = 0; i < 4; i++) {
            thrust[i] = state_.motor_speed[i] * params_.k_thrust;
        }

        // Total thrust in body Z (up in body = negative Z in NED body)
        // ボディZ軸の全推力
        float total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];

        // Torques from X-quad mixer / X-quadミキサーからのトルク
        // M1(FR,CCW) M2(RR,CW) M3(RL,CCW) M4(FL,CW)
        float L = params_.arm_length;
        float tau_roll  = L * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]);
        float tau_pitch = L * ( thrust[0] - thrust[1] - thrust[2] + thrust[3]);
        float tau_yaw   = params_.k_torque * (thrust[0] - thrust[1] + thrust[2] - thrust[3]);

        // Angular acceleration (Euler's equation simplified)
        // 角加速度（オイラーの方程式を簡略化）
        Vec3 angular_accel;
        angular_accel.x = tau_roll  / params_.Ixx;
        angular_accel.y = tau_pitch / params_.Iyy;
        angular_accel.z = tau_yaw   / params_.Izz;

        // Update angular rate / 角速度を更新
        state_.angular_rate += angular_accel * dt;

        // Update attitude / 姿勢を更新
        Quat dq = Quat::from_rotvec(state_.angular_rate * dt);
        state_.attitude = state_.attitude * dq;
        state_.attitude.normalize();

        // Thrust vector in NED / NED座標系での推力ベクトル
        Vec3 thrust_body(0, 0, -total_thrust);  // Body up
        Vec3 thrust_ned = state_.attitude.rotate(thrust_body);

        // Gravity / 重力
        Vec3 gravity_ned(0, 0, params_.mass * params_.gravity);

        // Drag / 抗力
        Vec3 drag = state_.velocity * (-params_.drag_coeff);

        // Linear acceleration / 線形加速度
        Vec3 accel = (thrust_ned + gravity_ned + drag) * (1.0f / params_.mass);

        // Update velocity and position / 速度と位置を更新
        state_.velocity += accel * dt;
        state_.position += state_.velocity * dt;

        // Ground constraint / 地面制約
        if (state_.position.z > -0.01f) {
            state_.position.z = -0.01f;
            if (state_.velocity.z > 0) state_.velocity.z = 0;
        }
    }

    /// Generate simulated sensor readings / シミュレーテッドセンサ値を生成
    SimSensors getSensors() const
    {
        SimSensors s;

        // Accelerometer: specific force in body frame
        // 加速度計: ボディフレームでの比力
        Vec3 gravity_ned(0, 0, params_.gravity);
        Vec3 gravity_body = state_.attitude.inv_rotate(gravity_ned);
        // In free flight: accel = -gravity (in body), on ground accel = 0-(-g) = g
        // Specific force = total_accel - gravity
        s.accel.x = -gravity_body.x;
        s.accel.y = -gravity_body.y;
        s.accel.z = -gravity_body.z;

        // Gyroscope: angular rate in body frame
        // ジャイロ: ボディ角速度
        s.gyro = state_.angular_rate;

        // ToF: distance to ground (NED: z negative = up)
        // ToF: 地面までの距離
        s.tof_bottom = -state_.position.z;  // Convert NED to positive height
        if (s.tof_bottom < 0.01f) s.tof_bottom = 0.01f;
        if (s.tof_bottom > 4.0f) s.tof_bottom = 4.0f;

        // Barometer altitude / 気圧高度
        s.baro_alt = -state_.position.z;

        // Magnetometer (fixed NED reference rotated to body)
        // 地磁気（固定NED参照をボディに回転）
        Vec3 mag_ned(20.0f, 0.0f, 40.0f);  // Japan approx
        s.mag = state_.attitude.inv_rotate(mag_ned);

        return s;
    }

    /// Get current state / 現在の状態を取得
    const QuadState& getState() const { return state_; }

    /// Print state summary / 状態サマリーを表示
    void printState() const
    {
        Vec3 euler = state_.attitude.to_euler();
        printf("pos=[%6.3f %6.3f %6.3f] vel=[%5.2f %5.2f %5.2f] "
               "att=[%5.1f %5.1f %5.1f] deg\n",
               state_.position.x, state_.position.y, state_.position.z,
               state_.velocity.x, state_.velocity.y, state_.velocity.z,
               euler.x * 57.3f, euler.y * 57.3f, euler.z * 57.3f);
    }

private:
    QuadParams params_;
    QuadState state_;
};

}  // namespace sim
}  // namespace sf
