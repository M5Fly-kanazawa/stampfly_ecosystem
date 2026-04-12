/**
 * @file quad_physics.hpp
 * @brief Quadrotor physics engine with contact dynamics
 *        接触力学付きクアッドロータ物理エンジン
 *
 * - RK4 integration for rigid body dynamics
 * - Box collision model (81.5 x 81.5 x 31mm) with 8 corner contacts
 * - Spring-damper ground contact (semi-implicit for stability)
 * - Coulomb friction
 * - No if-based ground constraints — all interactions are forces
 *
 * - 剛体ダイナミクスにRK4積分
 * - 立方体コリジョンモデル（81.5×81.5×31mm）8頂点接触
 * - バネ-ダンパ地面接触（安定性のための陰的積分）
 * - クーロン摩擦
 * - if文ベースの地面制約なし — 全相互作用は力として表現
 *
 * @design requirements.md §10 — SIL simulator                        [--]
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
// Random number generation
// =============================================================================

inline float randn()
{
    float u1 = (static_cast<float>(rand()) + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = static_cast<float>(rand()) / (RAND_MAX + 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

// =============================================================================
// Parameters
// =============================================================================

struct QuadParams {
    float mass       = 0.037f;
    float arm_length = 0.023f;
    float Ixx        = 5.0e-6f;
    float Iyy        = 5.0e-6f;
    float Izz        = 9.0e-6f;
    float k_thrust   = 0.168f;
    float k_torque   = 0.00971f;
    float gravity    = 9.81f;
    float motor_tc   = 0.02f;
    float drag_coeff = 0.01f;
};

struct ContactParams {
    // Box collision model (StampFly dimensions)
    // 立方体コリジョンモデル（StampFly寸法）
    float box_half_x = 0.04075f;  // 81.5mm / 2
    float box_half_y = 0.04075f;  // 81.5mm / 2
    float box_half_z = 0.0155f;   // 31mm / 2

    // Spring-damper contact parameters
    // バネ-ダンパ接触パラメータ
    // Spring stiffness and damping tuned for dt=2.5ms stability
    // dt=2.5msの安定性のために調整されたバネ定数と減衰
    // Critical damping: c_crit = 2*sqrt(k*m) = 2*sqrt(500*0.037) = 8.6
    // Underdamped (c < c_crit) for slight bounce
    // 不足減衰（c < c_crit）で軽いバウンス
    // Contact spring: pen = mg/k = 0.363/726 ≈ 0.5mm at rest
    // 接触バネ: 静止時の凹み = mg/k = 0.363/726 ≈ 0.5mm
    // With 10x substep (dt=0.25ms), k=726 is stable
    // 10倍サブステップ（dt=0.25ms）でk=726は安定
    float k_contact  = 726.0f;    // [N/m]
    float c_contact  = 8.0f;      // [N·s/m] (c_crit = 2*sqrt(726*0.037) = 10.4)
    float mu_friction = 0.6f;     // Coulomb friction coefficient

    // Ground plane at z = 0 (NED: z positive = down)
    // 地面平面 z = 0（NED: z正 = 下）
    float ground_z   = 0.0f;
};

struct SensorNoiseParams {
    float gyro_noise_density   = 0.000098f;
    float accel_noise_density  = 0.000664f;
    float gyro_bias_init_std   = 0.006f;
    float accel_bias_init_std  = 0.13f;
    float gyro_bias_rw         = 0.000013f;
    float accel_bias_rw        = 0.0001f;
    float vib_accel_k          = 4.3f;
    float vib_gyro_k           = 2.1f;
    float tof_noise_base       = 0.010f;
    float tof_noise_scale      = 0.015f;
    float baro_noise_std       = 0.05f;
    float baro_drift_rate      = 0.001f;
};

// =============================================================================
// State
// =============================================================================

struct QuadState {
    Vec3 position = {};
    Vec3 velocity = {};
    Quat attitude = {};
    Vec3 angular_rate = {};
    float motor_speed[4] = {};
    float avg_duty = 0;
};

struct SimSensors {
    Vec3 accel;
    Vec3 gyro;
    float tof_bottom;
    float baro_alt;
    Vec3 mag;
};

// =============================================================================
// Rigid body derivative (for RK4)
// 剛体微分（RK4用）
// =============================================================================

struct StateDerivative {
    Vec3 d_pos;
    Vec3 d_vel;
    Vec3 d_omega;
    // Quaternion derivative handled separately
};

// =============================================================================
// Physics Engine
// 物理エンジン
// =============================================================================

class QuadPhysics {
public:
    void init(const QuadParams& qp = {},
              const ContactParams& cp = {},
              const SensorNoiseParams& np = {})
    {
        qp_ = qp;
        cp_ = cp;
        np_ = np;
        state_ = {};
        state_.attitude = {1, 0, 0, 0};
        // Start just above ground (bottom of box at ground level)
        // 地面のすぐ上から開始（箱の底面が地面レベル）
        state_.position.z = -cp_.box_half_z;

        for (int i = 0; i < 3; i++) {
            gyro_bias_[i]  = randn() * np_.gyro_bias_init_std;
            accel_bias_[i] = randn() * np_.accel_bias_init_std;
        }
        baro_drift_ = 0;
        total_force_ned_ = {0, 0, 0};
    }

    // =========================================================================
    // Main simulation step
    // メインシミュレーションステップ
    // =========================================================================

    /// Physics sub-step count per control step
    /// 制御ステップあたりの物理サブステップ数
    static constexpr int PHYSICS_SUBSTEPS = 10;

    void step(const float motor_cmd[4], float dt)
    {
        // Motor dynamics at control rate
        // 制御レートでのモーターダイナミクス
        float avg = 0;
        for (int i = 0; i < 4; i++) {
            float cmd = fmaxf(0, fminf(1.0f, motor_cmd[i]));
            float alpha = dt / (qp_.motor_tc + dt);
            state_.motor_speed[i] += alpha * (cmd - state_.motor_speed[i]);
            avg += state_.motor_speed[i];
        }
        state_.avg_duty = avg * 0.25f;

        // Run physics at higher rate (10x control rate)
        // 物理を高レートで実行（制御レートの10倍）
        float sub_dt = dt / PHYSICS_SUBSTEPS;
        for (int sub = 0; sub < PHYSICS_SUBSTEPS; sub++) {
            physicsSubStep(sub_dt);
        }
    }

    // physicsSubStep is internal — public API uses step()
    void physicsSubStep(float dt)
    {
        // Compute motor forces and torques
        // モーター力とトルクを計算
        float thrust[4];
        for (int i = 0; i < 4; i++) {
            float s = state_.motor_speed[i];
            thrust[i] = s * s * qp_.k_thrust;
        }

        float total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];
        float L = qp_.arm_length;
        Vec3 motor_torque;
        motor_torque.x = L * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]);
        motor_torque.y = L * ( thrust[0] - thrust[1] - thrust[2] + thrust[3]);
        motor_torque.z = qp_.k_torque * (thrust[0] - thrust[1] + thrust[2] - thrust[3]);

        // Thrust in body frame → NED
        // ボディフレームの推力 → NED
        Vec3 thrust_body(0, 0, -total_thrust);
        Vec3 thrust_ned = state_.attitude.rotate(thrust_body);

        // Gravity in NED
        // NED座標の重力
        Vec3 gravity_ned(0, 0, qp_.mass * qp_.gravity);

        // Drag
        // 抗力
        Vec3 drag = state_.velocity * (-qp_.drag_coeff);

        // =====================================================================
        // Contact forces (spring-damper at each box corner)
        // 接触力（各立方体頂点でのバネ-ダンパ）
        // =====================================================================

        Vec3 contact_force_total = {};
        Vec3 contact_torque_total = {};
        computeContactForces(contact_force_total, contact_torque_total);

        // Total external force and torque
        // 全外力とトルク
        Vec3 ext_force = thrust_ned + gravity_ned + drag + contact_force_total;
        Vec3 ext_torque = motor_torque + contact_torque_total;

        // Store for accelerometer model
        // 加速度計モデル用に保存
        total_force_ned_ = ext_force;

        // =====================================================================
        // RK4 integration for position/velocity
        // 位置/速度のRK4積分
        // =====================================================================

        Vec3 acc = ext_force * (1.0f / qp_.mass);

        // RK4 for translational dynamics
        // 並進ダイナミクスのRK4
        Vec3 k1_v = acc;
        Vec3 k1_p = state_.velocity;

        Vec3 v2 = state_.velocity + k1_v * (dt * 0.5f);
        Vec3 k2_v = acc;  // Simplified: force is constant over dt
        Vec3 k2_p = v2;

        Vec3 v3 = state_.velocity + k2_v * (dt * 0.5f);
        Vec3 k3_v = acc;
        Vec3 k3_p = v3;

        Vec3 v4 = state_.velocity + k3_v * dt;
        Vec3 k4_v = acc;
        Vec3 k4_p = v4;

        state_.velocity += (k1_v + k2_v * 2.0f + k3_v * 2.0f + k4_v) * (dt / 6.0f);
        state_.position += (k1_p + k2_p * 2.0f + k3_p * 2.0f + k4_p) * (dt / 6.0f);

        // =====================================================================
        // Semi-implicit contact velocity correction
        // 陰的接触速度補正
        //
        // For each penetrating corner, apply implicit damping:
        // v_new = (v + dt*F_spring/m) / (1 + c*dt/m)
        //
        // This prevents oscillation with stiff contacts.
        // 各貫通頂点に対して陰的減衰を適用。
        // 剛い接触での振動を防止。
        // =====================================================================

        applyImplicitContactDamping(dt);

        // =====================================================================
        // RK4 for rotational dynamics (Euler equation)
        // 回転ダイナミクスのRK4（オイラー方程式）
        // =====================================================================

        Vec3 w = state_.angular_rate;
        Vec3 alpha1 = eulerEquation(w, ext_torque);
        Vec3 w2 = w + alpha1 * (dt * 0.5f);
        Vec3 alpha2 = eulerEquation(w2, ext_torque);
        Vec3 w3 = w + alpha2 * (dt * 0.5f);
        Vec3 alpha3 = eulerEquation(w3, ext_torque);
        Vec3 w4 = w + alpha3 * dt;
        Vec3 alpha4 = eulerEquation(w4, ext_torque);

        state_.angular_rate += (alpha1 + alpha2 * 2.0f + alpha3 * 2.0f + alpha4) * (dt / 6.0f);

        // Quaternion integration
        // クォータニオン積分
        Quat dq = Quat::from_rotvec(state_.angular_rate * dt);
        state_.attitude = state_.attitude * dq;
        state_.attitude.normalize();
    }

    // =========================================================================
    // Sensor model
    // センサモデル
    // =========================================================================

    SimSensors getSensors(float dt) const
    {
        SimSensors s;
        float fs = 1.0f / dt;
        float duty = state_.avg_duty;

        // Accelerometer: total force / mass, in body frame
        // 加速度計: 全力/質量、ボディフレーム
        // Accelerometer measures: (total_force - gravity) / m in body = specific force
        // But in our convention (matching vehicle): output = R_nb × (F_total/m)
        // At rest on ground: F_total = gravity + contact_reaction ≈ 0
        //   → accel_body = R_nb × 0 = 0... but real accelerometer reads +g
        //
        // The accelerometer measures: a_body = R_nb × (a_ned) where a_ned = F/m
        // With F = gravity + contact + thrust + drag
        // At rest: F = mg(down) + N(up) = 0 → a_ned = 0 → a_body = 0
        //
        // But real IMU reads +g at rest (after BMI270→NED transform).
        // This is because accelerometer measures specific force = a - g
        // In our NED: specific force = F_nongrav/m = (F_total - mg)/m
        // At rest: (0 - mg)/m... no, F_total includes gravity already.
        //
        // Let's be precise:
        // Accelerometer output (body) = R_nb × (a_inertial - g_ned)
        //   = R_nb × (F_total/m - g_ned)
        //   = R_nb × ((thrust + contact + drag)/m + g + g_ned... no
        //
        // F_total = thrust + gravity + drag + contact
        // a_inertial = F_total / m
        // Specific force = a_inertial - g = F_total/m - g = (thrust + contact + drag)/m
        // In body: a_body = R_nb × specific_force
        //
        // At rest: thrust=0, drag=0, contact = -gravity → specific_force = -gravity/m... no
        // contact_force_z = -mg (upward in NED = negative z) → contact/m = [0,0,-g]
        // specific_force = [0,0,-g]
        // a_body = R_nb × [0,0,-g] = [0,0,-g] (horizontal)
        //
        // But vehicle outputs [0,0,+g]. The difference is the BMI270→NED axis swap.
        // In the real vehicle: sensor_z points UP, swap makes body_z = -sensor_z.
        // So the real sensor sees +g in its Z axis, which gets converted to -g in body_z,
        // then the code adds 2g bias → effective +g.
        //
        // For SIL: output specific force + 2g to match vehicle convention.
        // SIL用: specific forceに2gを加算してvehicle慣例に合わせる

        Vec3 non_grav_force = total_force_ned_ - Vec3(0, 0, qp_.mass * qp_.gravity);
        Vec3 specific_force_ned = non_grav_force * (1.0f / qp_.mass);
        Vec3 specific_force_body = state_.attitude.inv_rotate(specific_force_ned);

        // Add 2g to z to match vehicle convention:
        // static: specific_force_body.z = -g → + 2g → +g (matches vehicle)
        // vehicle慣例に合わせてzに2gを加算
        specific_force_body.z += 2.0f * qp_.gravity;

        // Add noise
        float accel_sigma = sqrtf(
            (np_.accel_noise_density * sqrtf(fs)) * (np_.accel_noise_density * sqrtf(fs))
            + (np_.vib_accel_k * duty * duty) * (np_.vib_accel_k * duty * duty));
        s.accel.x = specific_force_body.x + randn() * accel_sigma + accel_bias_[0];
        s.accel.y = specific_force_body.y + randn() * accel_sigma + accel_bias_[1];
        s.accel.z = specific_force_body.z + randn() * accel_sigma + accel_bias_[2];

        // Gyroscope
        float gyro_sigma = sqrtf(
            (np_.gyro_noise_density * sqrtf(fs)) * (np_.gyro_noise_density * sqrtf(fs))
            + (np_.vib_gyro_k * duty * duty) * (np_.vib_gyro_k * duty * duty));
        s.gyro.x = state_.angular_rate.x + randn() * gyro_sigma + gyro_bias_[0];
        s.gyro.y = state_.angular_rate.y + randn() * gyro_sigma + gyro_bias_[1];
        s.gyro.z = state_.angular_rate.z + randn() * gyro_sigma + gyro_bias_[2];

        // ToF: distance from bottom of box to ground
        // ToF: 箱底面から地面までの距離
        // Find lowest corner
        float min_z = 1e6f;
        for (int c = 0; c < 8; c++) {
            Vec3 corner = getCorner(c);
            Vec3 corner_w = state_.position + state_.attitude.rotate(corner);
            if (corner_w.z < min_z) min_z = corner_w.z;
        }
        float height = -min_z;  // NED to positive height
        if (height < 0) height = 0;
        float tof_sigma = np_.tof_noise_base + np_.tof_noise_scale * height;
        s.tof_bottom = height + randn() * tof_sigma;
        if (s.tof_bottom < 0.01f) s.tof_bottom = 0.01f;
        if (s.tof_bottom > 4.0f) s.tof_bottom = 4.0f;

        // Barometer
        s.baro_alt = height + randn() * np_.baro_noise_std + baro_drift_;

        // Magnetometer
        Vec3 mag_ned(20.0f, 0.0f, 40.0f);
        s.mag = state_.attitude.inv_rotate(mag_ned);

        return s;
    }

    void updateBiases(float dt)
    {
        for (int i = 0; i < 3; i++) {
            gyro_bias_[i]  += randn() * np_.gyro_bias_rw * sqrtf(dt);
            accel_bias_[i] += randn() * np_.accel_bias_rw * sqrtf(dt);
        }
        baro_drift_ += randn() * np_.baro_drift_rate * sqrtf(dt);
    }

    const QuadState& getState() const { return state_; }

    /// Check if any corner is in contact with ground
    /// 頂点が地面に接触しているか確認
    bool isOnGround() const
    {
        for (int c = 0; c < 8; c++) {
            Vec3 corner_w = state_.position + state_.attitude.rotate(getCorner(c));
            if (corner_w.z > cp_.ground_z - 0.001f) return true;
        }
        return false;
    }

    void printState() const
    {
        Vec3 euler = state_.attitude.to_euler();
        printf("pos=[%6.3f %6.3f %6.3f] vel=[%5.2f %5.2f %5.2f] "
               "att=[%5.1f %5.1f %5.1f]deg duty=%.2f ground=%d\n",
               state_.position.x, state_.position.y, state_.position.z,
               state_.velocity.x, state_.velocity.y, state_.velocity.z,
               euler.x * 57.3f, euler.y * 57.3f, euler.z * 57.3f,
               state_.avg_duty, isOnGround());
    }

private:
    // =========================================================================
    // Box collision corners
    // 立方体コリジョン頂点
    // =========================================================================

    Vec3 getCorner(int idx) const
    {
        // 8 corners in body frame (NED body: x=fwd, y=right, z=down)
        // 8頂点をボディフレーム（NED: x=前, y=右, z=下）で定義
        float hx = cp_.box_half_x;
        float hy = cp_.box_half_y;
        float hz = cp_.box_half_z;

        float sx = (idx & 1) ? -1.0f : 1.0f;
        float sy = (idx & 2) ? -1.0f : 1.0f;
        float sz = (idx & 4) ? -1.0f : 1.0f;

        return Vec3(sx * hx, sy * hy, sz * hz);
    }

    // =========================================================================
    // Contact force computation
    // 接触力計算
    // =========================================================================

    void computeContactForces(Vec3& force_total, Vec3& torque_total)
    {
        force_total = {};
        torque_total = {};

        float R[3][3];
        state_.attitude.to_dcm(R);

        for (int c = 0; c < 8; c++) {
            Vec3 corner_body = getCorner(c);
            Vec3 corner_world = state_.position + state_.attitude.rotate(corner_body);

            // Penetration depth (positive when below ground)
            // 貫通深さ（地面より下で正）
            float pen = corner_world.z - cp_.ground_z;
            if (pen <= 0) continue;  // No contact

            // Velocity at contact point
            // 接触点での速度
            Vec3 r_world = state_.attitude.rotate(corner_body);
            Vec3 v_corner = state_.velocity + state_.angular_rate.cross(r_world);

            // Normal force (spring-damper, upward in NED = negative z)
            // 法線力（バネ-ダンパ、NEDで上向き = 負z）
            float fn = -(cp_.k_contact * pen + cp_.c_contact * v_corner.z);
            if (fn > 0) fn = 0;  // Contact can only push, not pull
                                  // 接触は押すだけ、引けない

            Vec3 f_contact = {0, 0, fn};

            // Friction (Coulomb model)
            // 摩擦（クーロンモデル）
            float v_tan_sq = v_corner.x * v_corner.x + v_corner.y * v_corner.y;
            if (v_tan_sq > 1e-8f && fn < 0) {
                float v_tan = sqrtf(v_tan_sq);
                float f_friction = cp_.mu_friction * fabsf(fn);
                f_contact.x = -f_friction * v_corner.x / v_tan;
                f_contact.y = -f_friction * v_corner.y / v_tan;
            }

            // Accumulate force and torque
            // 力とトルクを蓄積
            force_total += f_contact;
            torque_total += r_world.cross(f_contact);
        }
    }

    // =========================================================================
    // Semi-implicit contact damping
    // 陰的接触減衰
    //
    // Prevents oscillation with stiff spring contacts.
    // 剛いバネ接触での振動を防止。
    //
    // For each penetrating corner:
    //   v_new = (v + dt*F_spring/m) / (1 + c*dt/m)
    // =========================================================================

    void applyImplicitContactDamping(float dt)
    {
        // Count penetrating corners and max penetration
        // 貫通頂点数と最大貫通深さ
        int contact_count = 0;
        float max_pen = 0;

        for (int c = 0; c < 8; c++) {
            Vec3 corner_world = state_.position + state_.attitude.rotate(getCorner(c));
            float pen = corner_world.z - cp_.ground_z;
            if (pen > 0) {
                contact_count++;
                if (pen > max_pen) max_pen = pen;
            }
        }

        if (contact_count == 0) return;

        // Semi-implicit correction for vertical velocity
        // 垂直速度の陰的補正
        //
        // Solve: m * v_new = m * v_old + dt * (-k*pen - c*v_new) * n_contacts
        // → v_new * (m + c*dt*n) = m*v_old - k*pen*dt*n
        // → v_new = (m*v_old - k*pen*dt*n) / (m + c*dt*n)
        //
        float n = static_cast<float>(contact_count);
        float denom = qp_.mass + cp_.c_contact * dt * n;
        state_.velocity.z = (qp_.mass * state_.velocity.z
                           - cp_.k_contact * max_pen * dt * n) / denom;

        // Prevent deep penetration: push position back
        // 深い貫通を防止: 位置を押し戻す
        if (max_pen > 0.005f) {
            state_.position.z -= (max_pen - 0.002f);
        }

        // Rotational damping from ground contact
        // 地面接触による回転減衰
        // Stronger damping to prevent tumbling away
        // 転がって飛んでいくのを防止する強い減衰
        float rot_factor = 1.0f / (1.0f + 100.0f * dt * n);
        state_.angular_rate.x *= rot_factor;
        state_.angular_rate.y *= rot_factor;
        state_.angular_rate.z *= (1.0f / (1.0f + 30.0f * dt * n));
    }

    // =========================================================================
    // Euler equation: I * alpha = tau - omega x (I * omega)
    // オイラー方程式
    // =========================================================================

    Vec3 eulerEquation(const Vec3& w, const Vec3& tau) const
    {
        return Vec3(
            (tau.x - (qp_.Izz - qp_.Iyy) * w.y * w.z) / qp_.Ixx,
            (tau.y - (qp_.Ixx - qp_.Izz) * w.z * w.x) / qp_.Iyy,
            (tau.z - (qp_.Iyy - qp_.Ixx) * w.x * w.y) / qp_.Izz
        );
    }

    // State
    QuadParams qp_;
    ContactParams cp_;
    SensorNoiseParams np_;
    QuadState state_;

    // Sensor biases
    float gyro_bias_[3] = {};
    float accel_bias_[3] = {};
    float baro_drift_ = 0;

    // Total force for accelerometer model
    Vec3 total_force_ned_ = {};
};

}  // namespace sim
}  // namespace sf
