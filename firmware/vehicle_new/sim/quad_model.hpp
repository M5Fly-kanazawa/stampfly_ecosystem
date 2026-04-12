/**
 * @file quad_model.hpp
 * @brief Quadrotor dynamics + sensor model for SIL simulation
 *        SILシミュレーション用クアッドロータ力学 + センサモデル
 *
 * 6-DOF rigid body with:
 * - Quadratic thrust model (thrust ∝ duty²)
 * - Euler equation with gyroscopic coupling
 * - Correct specific force (accelerometer model)
 * - Sensor noise: Gaussian + bias + throttle-dependent vibration
 *
 * @design requirements.md §10 — SIL simulator                        [--]
 * @design noise_and_vibration_model.md §4-5 — Sensor/vibration model  [--]
 */

#pragma once

#include "sf_math.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace sf {
namespace sim {

using namespace math;

// =============================================================================
// Random number generation (simple, portable)
// 乱数生成（シンプル、ポータブル）
// =============================================================================

/// Generate Gaussian random number (Box-Muller transform)
/// ガウス乱数を生成（Box-Muller変換）
inline float randn()
{
    float u1 = (static_cast<float>(rand()) + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = static_cast<float>(rand()) / (RAND_MAX + 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

// =============================================================================
// Physical Parameters / 物理パラメータ
// =============================================================================

struct QuadParams {
    float mass       = 0.037f;    // [kg] StampFly mass
    float arm_length = 0.023f;    // [m] motor-to-center
    float Ixx        = 5.0e-6f;  // [kg·m²] roll inertia
    float Iyy        = 5.0e-6f;  // [kg·m²] pitch inertia
    float Izz        = 9.0e-6f;  // [kg·m²] yaw inertia
    float k_thrust   = 0.168f;   // [N] max thrust per motor at duty=1
    float k_torque   = 0.00971f; // [m] torque/thrust ratio
    float gravity    = 9.81f;    // [m/s²]
    float motor_tc   = 0.02f;    // [s] motor time constant
    float drag_coeff = 0.01f;    // [N/(m/s)] linear drag
};

// =============================================================================
// Sensor Noise Parameters / センサノイズパラメータ
//
// @design noise_and_vibration_model.md §2-4
// =============================================================================

/// Sensor noise parameters derived from 72 real flight logs
/// 実飛行ログ72ファイルから導出したセンサノイズパラメータ
///
/// Static: 35 pre-flight segments (102s), median values
/// Flight: 32 trimmed segments (168s), median values
/// See: noise_analysis_v3.txt for full statistical report
struct SensorNoiseParams {
    // IMU static noise (from flight log pre-flight segments)
    // IMU静的ノイズ（飛行前静止区間から）
    float gyro_noise_density   = 0.000098f;  // [rad/s/√Hz] measured static
    float accel_noise_density  = 0.000664f;  // [m/s²/√Hz] measured static

    // IMU startup bias (from bias statistics across 72 logs)
    // IMU起動時バイアス（72ログのバイアス統計から）
    float gyro_bias_init_std   = 0.006f;     // [rad/s] log median ~0.004
    float accel_bias_init_std  = 0.13f;      // [m/s²] log median ~0.13

    // IMU bias random walk / バイアスランダムウォーク
    float gyro_bias_rw         = 0.000013f;  // [rad/s/√s]
    float accel_bias_rw        = 0.0001f;    // [m/s²/√s]

    // Throttle-dependent vibration (from flight segments)
    // スロットル依存振動（飛行区間から）
    // At hover duty ~0.6: σ_accel ≈ 1.54 m/s², σ_gyro ≈ 0.76 rad/s
    // K = σ / duty² ≈ 1.54 / 0.36 ≈ 4.3 (accel), 0.76 / 0.36 ≈ 2.1 (gyro)
    float vib_accel_k          = 4.3f;       // σ_accel = K × duty² [m/s²]
    float vib_gyro_k           = 2.1f;       // σ_gyro = K × duty² [rad/s]

    // ToF noise (from flight segments, median σ = 0.033m)
    // ToFノイズ（飛行区間から、中央値σ = 0.033m）
    float tof_noise_base       = 0.010f;     // [m] base noise
    float tof_noise_scale      = 0.015f;     // [m/m] distance-proportional

    // Baro noise / 気圧ノイズ
    float baro_noise_std       = 0.05f;      // [m] altitude noise
    float baro_drift_rate      = 0.001f;     // [m/√s] slow drift
};

// =============================================================================
// Quadrotor State / クアッドロータ状態
// =============================================================================

struct QuadState {
    Vec3 position = {};        // [m] NED
    Vec3 velocity = {};        // [m/s] NED
    Quat attitude = {};        // quaternion
    Vec3 angular_rate = {};    // [rad/s] body frame
    float motor_speed[4] = {}; // [0..1] current motor speed
    float avg_duty = 0;        // average motor duty (for noise scaling)
};

// =============================================================================
// Simulated Sensor Output / シミュレーテッドセンサ出力
// =============================================================================

struct SimSensors {
    Vec3 accel;        // [m/s²] body frame (specific force + noise)
    Vec3 gyro;         // [rad/s] body frame (angular rate + noise)
    float tof_bottom;  // [m] distance to ground
    float baro_alt;    // [m] barometric altitude
    Vec3 mag;          // [uT] magnetic field in body frame
};

// =============================================================================
// Quadrotor Dynamics + Sensor Model
// クアッドロータ力学 + センサモデル
// =============================================================================

class QuadModel {
public:
    void init(const QuadParams& params = {},
              const SensorNoiseParams& noise = {})
    {
        params_ = params;
        noise_params_ = noise;
        state_ = {};
        state_.attitude = {1, 0, 0, 0};
        state_.position.z = 0.0f;  // On ground (NED z=0)

        // Initialize random biases (unique per boot)
        // ランダムバイアスを初期化（起動毎に異なる）
        for (int i = 0; i < 3; i++) {
            gyro_bias_[i]  = randn() * noise_params_.gyro_bias_init_std;
            accel_bias_[i] = randn() * noise_params_.accel_bias_init_std;
        }
        baro_drift_ = 0;

        // Store true acceleration for specific force calculation
        // 比力計算のために真の加速度を保存
        true_accel_ned_ = {0, 0, 0};
    }

    /// Step simulation / シミュレーションを1ステップ進める
    void step(const float motor_cmd[4], float dt)
    {
        // Motor dynamics (1st-order lag) / モーターダイナミクス（1次遅れ）
        float avg = 0;
        for (int i = 0; i < 4; i++) {
            float cmd = fmaxf(0, fminf(1.0f, motor_cmd[i]));
            float alpha = dt / (params_.motor_tc + dt);
            state_.motor_speed[i] += alpha * (cmd - state_.motor_speed[i]);
            avg += state_.motor_speed[i];
        }
        state_.avg_duty = avg * 0.25f;

        // Quadratic thrust: T = k × duty²
        // 二次推力: T = k × duty²
        // @design noise_and_vibration_model.md §5 — Quadratic thrust
        float thrust[4];
        for (int i = 0; i < 4; i++) {
            float spd = state_.motor_speed[i];
            thrust[i] = spd * spd * params_.k_thrust;
        }

        float total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];

        // Torques (X-quad mixer) / トルク（X-quadミキサー）
        float L = params_.arm_length;
        float tau_roll  = L * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]);
        float tau_pitch = L * ( thrust[0] - thrust[1] - thrust[2] + thrust[3]);
        float tau_yaw   = params_.k_torque *
                          (thrust[0] - thrust[1] + thrust[2] - thrust[3]);

        // Euler equation with gyroscopic coupling
        // ジャイロスコピック項付きオイラー方程式
        // @design noise_and_vibration_model.md §5 — Euler gyroscopic
        Vec3 w = state_.angular_rate;
        Vec3 angular_accel;
        angular_accel.x = (tau_roll  - (params_.Izz - params_.Iyy) * w.y * w.z)
                          / params_.Ixx;
        angular_accel.y = (tau_pitch - (params_.Ixx - params_.Izz) * w.z * w.x)
                          / params_.Iyy;
        angular_accel.z = (tau_yaw   - (params_.Iyy - params_.Ixx) * w.x * w.y)
                          / params_.Izz;

        state_.angular_rate += angular_accel * dt;

        // Attitude integration / 姿勢積分
        Quat dq = Quat::from_rotvec(state_.angular_rate * dt);
        state_.attitude = state_.attitude * dq;
        state_.attitude.normalize();

        // Forces in NED / NED座標系での力
        Vec3 thrust_body(0, 0, -total_thrust);
        Vec3 thrust_ned = state_.attitude.rotate(thrust_body);
        Vec3 gravity_ned(0, 0, params_.mass * params_.gravity);
        Vec3 drag = state_.velocity * (-params_.drag_coeff);

        // True NED acceleration (without gravity, for specific force)
        // 真のNED加速度（重力なし、比力計算用）
        Vec3 non_grav_force = thrust_ned + drag;
        true_accel_ned_ = non_grav_force * (1.0f / params_.mass);

        // Total acceleration / 全加速度
        Vec3 total_accel = (non_grav_force + gravity_ned) * (1.0f / params_.mass);

        state_.velocity += total_accel * dt;
        state_.position += state_.velocity * dt;

        // Ground constraint / 地面制約
        // Ground level is z = 0 in NED (z positive = down)
        // 地面レベルはNEDでz=0（z正=下向き）
        const float ground_z = 0.0f;  // Ground at z=0
        if (state_.position.z > ground_z) {
            // Penetrated ground / 地面を貫通
            state_.position.z = ground_z;

            if (state_.velocity.z > 0) {
                // Moving downward → stop / 下向きに移動 → 停止
                state_.velocity.z = 0;
            }

            if (total_accel.z > 0) {
                // Net force pushes into ground → ground reaction cancels it
                // 正味力が地面方向 → 垂直抗力が相殺
                true_accel_ned_.z = 0;
            }
            // If total_accel.z <= 0 → lifting off (thrust > weight)
            // total_accel.z <= 0 → 離陸中（推力 > 重量）
            // → position.z will go negative (up) next step

            // Strong damping on ground (physical contact prevents rotation)
            // 地上では強い減衰（物理的接触が回転を防止）
            state_.angular_rate = state_.angular_rate * 0.5f;
            // Also clamp attitude to near-level on ground
            // 地上では姿勢をほぼ水平にクランプ
            Vec3 euler = state_.attitude.to_euler();
            if (fabsf(euler.x) > 0.1f || fabsf(euler.y) > 0.1f) {
                state_.attitude = Quat(1, 0, 0, 0);  // Reset to level
                state_.angular_rate = {0, 0, 0};
            }
        }
    }

    /// Generate sensor readings with noise
    /// ノイズ付きセンサ値を生成
    ///
    /// @design noise_and_vibration_model.md §4 — Sensor noise models
    SimSensors getSensors(float dt) const
    {
        SimSensors s;
        float fs = 1.0f / dt;  // Sampling frequency
        float duty = state_.avg_duty;

        // =================================================================
        // Accelerometer: specific force = R_nb × a_nongrav
        // 加速度計: 比力 = R_nb × 非重力加速度
        //
        // Specific force is what the accelerometer measures:
        // the total acceleration minus gravity, in body frame.
        //
        // 比力は加速度計が測定するもの:
        // 全加速度から重力を引いたもの、ボディフレームで。
        //
        // At rest: accel = R_nb × (0 - g_ned) = R_nb × {0,0,-g}
        //   → body_z ≈ -9.81 (pointing up in body)
        //
        // In flight with thrust T upward:
        //   accel = R_nb × (T/m + drag/m)
        //   → includes thrust contribution
        //
        // @design noise_and_vibration_model.md §5 — Correct specific force
        // =================================================================

        // Dynamic acceleration in body frame
        // ボディフレームでの動的な加速度
        //
        // Accelerometer output = R_nb × (a_total - g)
        // where a_total includes gravity: a_total = thrust/m + drag/m + g
        // So: output = R_nb × (thrust/m + drag/m + g - g) = R_nb × (thrust/m + drag/m)
        //
        // BUT the real vehicle firmware (imu_task.cpp) outputs:
        //   body_z = +g at rest (NED down-positive convention after BMI270 axis swap)
        // This means the accelerometer output convention is:
        //   output = R_nb × (g - a_freefall) = -specific_force
        //   At rest: output = R_nb × g = [0, 0, +g] (body z-down)
        //   In free fall: output = [0, 0, 0]
        //
        // Match the real vehicle convention:
        // 実機の慣例に合わせる:
        Vec3 g_ned(0, 0, params_.gravity);
        Vec3 total_accel_ned = true_accel_ned_ + g_ned;  // thrust/m + drag/m + g
        Vec3 accel_body = state_.attitude.inv_rotate(total_accel_ned);
        // At rest: accel_body = inv_rotate([0,0,+g]) = [0,0,+g]
        Vec3 specific_force = accel_body;

        // Add noise / ノイズを追加
        float accel_static_sigma = noise_params_.accel_noise_density * sqrtf(fs);
        float accel_vib_sigma = noise_params_.vib_accel_k * duty * duty;
        float accel_sigma = sqrtf(accel_static_sigma * accel_static_sigma
                                + accel_vib_sigma * accel_vib_sigma);

        s.accel.x = specific_force.x + randn() * accel_sigma + accel_bias_[0];
        s.accel.y = specific_force.y + randn() * accel_sigma + accel_bias_[1];
        s.accel.z = specific_force.z + randn() * accel_sigma + accel_bias_[2];

        // =================================================================
        // Gyroscope: angular rate + noise + bias
        // ジャイロ: 角速度 + ノイズ + バイアス
        // =================================================================

        float gyro_static_sigma = noise_params_.gyro_noise_density * sqrtf(fs);
        float gyro_vib_sigma = noise_params_.vib_gyro_k * (M_PI / 180.0f)
                               * duty * duty;
        float gyro_sigma = sqrtf(gyro_static_sigma * gyro_static_sigma
                               + gyro_vib_sigma * gyro_vib_sigma);

        s.gyro.x = state_.angular_rate.x + randn() * gyro_sigma + gyro_bias_[0];
        s.gyro.y = state_.angular_rate.y + randn() * gyro_sigma + gyro_bias_[1];
        s.gyro.z = state_.angular_rate.z + randn() * gyro_sigma + gyro_bias_[2];

        // =================================================================
        // ToF: distance to ground with noise
        // ToF: 地面距離 + ノイズ
        // =================================================================

        float true_height = -state_.position.z;
        float tof_sigma = noise_params_.tof_noise_base
                        + noise_params_.tof_noise_scale * true_height;
        s.tof_bottom = true_height + randn() * tof_sigma;
        if (s.tof_bottom < 0.01f) s.tof_bottom = 0.01f;
        if (s.tof_bottom > 4.0f) s.tof_bottom = 4.0f;

        // =================================================================
        // Barometer: altitude with noise and drift
        // 気圧: 高度 + ノイズ + ドリフト
        // =================================================================

        s.baro_alt = true_height + randn() * noise_params_.baro_noise_std
                   + baro_drift_;

        // =================================================================
        // Magnetometer (ideal, noise TODO)
        // 地磁気（理想、ノイズTODO）
        // =================================================================

        Vec3 mag_ned(20.0f, 0.0f, 40.0f);
        s.mag = state_.attitude.inv_rotate(mag_ned);

        return s;
    }

    /// Update bias random walk (call once per step)
    /// バイアスランダムウォークを更新（ステップ毎に1回呼ぶ）
    void updateBiases(float dt)
    {
        for (int i = 0; i < 3; i++) {
            gyro_bias_[i]  += randn() * noise_params_.gyro_bias_rw * sqrtf(dt);
            accel_bias_[i] += randn() * noise_params_.accel_bias_rw * sqrtf(dt);
        }
        baro_drift_ += randn() * noise_params_.baro_drift_rate * sqrtf(dt);
    }

    const QuadState& getState() const { return state_; }

    void printState() const
    {
        Vec3 euler = state_.attitude.to_euler();
        printf("pos=[%6.3f %6.3f %6.3f] vel=[%5.2f %5.2f %5.2f] "
               "att=[%5.1f %5.1f %5.1f]deg duty=%.2f\n",
               state_.position.x, state_.position.y, state_.position.z,
               state_.velocity.x, state_.velocity.y, state_.velocity.z,
               euler.x * 57.3f, euler.y * 57.3f, euler.z * 57.3f,
               state_.avg_duty);
    }

private:
    QuadParams params_;
    SensorNoiseParams noise_params_;
    QuadState state_;

    // Sensor biases (persist across steps)
    // センサバイアス（ステップ間で永続）
    float gyro_bias_[3] = {};
    float accel_bias_[3] = {};
    float baro_drift_ = 0;

    // True non-gravitational acceleration in NED (for specific force)
    // NED座標の真の非重力加速度（比力計算用）
    Vec3 true_accel_ned_ = {};
};

}  // namespace sim
}  // namespace sf
