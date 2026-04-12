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
    float c_contact  = 5.0f;      // [N·s/m] (c_crit=10.4, ratio=0.48 → ~3cm bounce from 0.5m)
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

    // =========================================================================
    // Physics sub-step: RK4 for free dynamics + semi-implicit contact
    // 物理サブステップ: 自由ダイナミクスのRK4 + 陰的接触
    //
    // Structure:
    // 1. Compute non-contact forces (thrust, gravity, drag, motor torque)
    // 2. RK4 integrate translational and rotational dynamics (no contact)
    // 3. Compute contact from post-RK4 state
    // 4. Semi-implicit contact correction (both linear and angular)
    //
    // 構造:
    // 1. 非接触力を計算（推力、重力、抗力、モータートルク）
    // 2. 並進・回転ダイナミクスをRK4積分（接触なし）
    // 3. RK4後の状態から接触を計算
    // 4. 陰的接触補正（並進・回転の両方）
    // =========================================================================

    void physicsSubStep(float dt)
    {
        // ==== Step 1: Non-contact forces ====
        // ==== ステップ1: 非接触力 ====

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

        Vec3 thrust_body(0, 0, -total_thrust);
        Vec3 thrust_ned = state_.attitude.rotate(thrust_body);
        Vec3 gravity_ned(0, 0, qp_.mass * qp_.gravity);
        Vec3 drag = state_.velocity * (-qp_.drag_coeff);

        // Non-contact force and torque only
        // 非接触力とトルクのみ
        Vec3 free_force = thrust_ned + gravity_ned + drag;
        Vec3 free_acc = free_force * (1.0f / qp_.mass);

        // ==== Step 2: RK4 for free dynamics (no contact) ====
        // ==== ステップ2: 自由ダイナミクスのRK4（接触なし） ====

        // Translational RK4
        // 並進RK4
        Vec3 k1_v = free_acc;
        Vec3 k1_p = state_.velocity;
        Vec3 k2_p = state_.velocity + k1_v * (dt * 0.5f);
        Vec3 k3_p = state_.velocity + k1_v * (dt * 0.5f);
        Vec3 k4_p = state_.velocity + k1_v * dt;

        state_.velocity += k1_v * dt;  // For constant force, RK4 = Euler
        state_.position += (k1_p + k2_p * 2.0f + k3_p * 2.0f + k4_p) * (dt / 6.0f);

        // Rotational RK4 (Euler equation, non-contact torque only)
        // 回転RK4（オイラー方程式、非接触トルクのみ）
        Vec3 w = state_.angular_rate;
        Vec3 a1 = eulerEquation(w, motor_torque);
        Vec3 w2 = w + a1 * (dt * 0.5f);
        Vec3 a2 = eulerEquation(w2, motor_torque);
        Vec3 w3 = w + a2 * (dt * 0.5f);
        Vec3 a3 = eulerEquation(w3, motor_torque);
        Vec3 w4 = w + a3 * dt;
        Vec3 a4 = eulerEquation(w4, motor_torque);

        state_.angular_rate += (a1 + a2 * 2.0f + a3 * 2.0f + a4) * (dt / 6.0f);

        // Quaternion integration
        // クォータニオン積分
        Quat dq = Quat::from_rotvec(state_.angular_rate * dt);
        state_.attitude = state_.attitude * dq;
        state_.attitude.normalize();

        // ==== Step 3 & 4: Semi-implicit contact solver ====
        // ==== ステップ3 & 4: 陰的接触ソルバー ====
        //
        // For each penetrating corner, compute spring force from position
        // and solve velocity implicitly with damper:
        //
        //   v_new = (v_rk4 + dt * F_spring / m) / (1 + c * dt / m)
        //
        // This is the standard semi-implicit Euler for stiff systems:
        // - Spring (position-dependent): explicit from current position
        // - Damper (velocity-dependent): implicit in new velocity
        //
        // 各貫通頂点に対して、位置からバネ力を計算し、
        // ダンパで速度を陰的に解く:
        //
        //   v_new = (v_rk4 + dt * F_spring / m) / (1 + c * dt / m)
        //
        // スティッフ系の標準的な陰的オイラー法:
        // - バネ（位置依存）: 現在位置から陽的
        // - ダンパ（速度依存）: 新速度で陰的

        solveContact(dt);

        // Store total force for accelerometer model
        // 加速度計モデル用に全力を保存
        // Includes contact force contribution estimated from velocity change
        // 速度変化から推定した接触力の寄与を含む
        total_force_ned_ = free_force + contact_force_cache_;
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

    // =========================================================================
    // SDIRK2 implicit contact solver with Newton iteration
    // ニュートン反復付きSDIRK2陰的接触ソルバー
    //
    // SDIRK2 Butcher tableau (γ = 1 - 1/√2 ≈ 0.2929):
    //   γ   | γ   0          (A-stable, L-stable, 2nd order)
    //   1   | 1-γ  γ
    //   ----+--------
    //       | 1-γ  γ
    //
    // For each penetrating corner, solve the 2D stiff system:
    //   d(pen)/dt = v_n
    //   d(v_n)/dt = F(pen, v_n) / m_eff
    //
    // where F = -k*pen - c*v_n  (spring-damper, active when pen > 0)
    //
    // Newton iteration for implicit stage:
    //   Residual: r_i = k_i - f(y + h*γ*k_i + known)
    //   Jacobian: J = I - h*γ * ∂f/∂y
    //   Update:   k_i -= J^{-1} * r_i
    // =========================================================================

    void solveContact(float dt)
    {
        static constexpr float GAMMA = 0.29289321881f;  // 1 - 1/√2
        static constexpr int MAX_NEWTON = 5;
        static constexpr float TOL = 1e-8f;

        contact_force_cache_ = {};
        Vec3 contact_torque_sum = {};

        for (int ci = 0; ci < 8; ci++) {
            Vec3 corner_body = getCorner(ci);
            Vec3 r_world = state_.attitude.rotate(corner_body);
            Vec3 corner_world = state_.position + r_world;

            float pen0 = corner_world.z - cp_.ground_z;
            if (pen0 <= 0) continue;

            Vec3 v_corner = state_.velocity + state_.angular_rate.cross(r_world);
            float vn0 = v_corner.z;

            float k = cp_.k_contact;
            float c = cp_.c_contact;
            float m = qp_.mass;
            float hg = dt * GAMMA;

            // ---- Contact force function ----
            // f(pen, vn) = [vn, -(k*pen + c*vn)/m]  when pen > 0
            auto f_contact = [&](float p, float v, float& fp, float& fv) {
                fp = v;
                if (p > 0) {
                    float fn = -(k * p + c * v);
                    if (fn > 0) fn = 0;  // Push only
                    fv = fn / m;
                } else {
                    fv = 0;
                }
            };

            // ---- SDIRK2 Stage 1: k1 = f(y + h*γ*k1) ----
            float k1p = vn0;
            float k1v = (pen0 > 0) ? -(k * pen0 + c * vn0) / m : 0;

            for (int it = 0; it < MAX_NEWTON; it++) {
                float p1 = pen0 + hg * k1p;
                float v1 = vn0  + hg * k1v;
                float fp, fv;
                f_contact(p1, v1, fp, fv);

                float rp = k1p - fp;
                float rv = k1v - fv;
                if (fabsf(rp) < TOL && fabsf(rv) < TOL) break;

                // J = I - hg * df/dy
                // df/dy = [[0,1],[-k/m,-c/m]] when p>0
                bool active = (p1 > 0);
                float j11 = 1.0f;
                float j12 = -hg;
                float j21 = active ? hg * k / m : 0;
                float j22 = 1.0f + (active ? hg * c / m : 0);
                float det = j11 * j22 - j12 * j21;
                if (fabsf(det) < 1e-12f) break;

                k1p -= ( j22 * rp - j12 * rv) / det;
                k1v -= (-j21 * rp + j11 * rv) / det;
            }

            // ---- SDIRK2 Stage 2: k2 = f(y + h*(1-γ)*k1 + h*γ*k2) ----
            float k2p = k1p;
            float k2v = k1v;

            for (int it = 0; it < MAX_NEWTON; it++) {
                float p2 = pen0 + dt * ((1.0f - GAMMA) * k1p + GAMMA * k2p);
                float v2 = vn0  + dt * ((1.0f - GAMMA) * k1v + GAMMA * k2v);
                float fp, fv;
                f_contact(p2, v2, fp, fv);

                float rp = k2p - fp;
                float rv = k2v - fv;
                if (fabsf(rp) < TOL && fabsf(rv) < TOL) break;

                bool active = (p2 > 0);
                float j11 = 1.0f;
                float j12 = -hg;
                float j21 = active ? hg * k / m : 0;
                float j22 = 1.0f + (active ? hg * c / m : 0);
                float det = j11 * j22 - j12 * j21;
                if (fabsf(det) < 1e-12f) break;

                k2p -= ( j22 * rp - j12 * rv) / det;
                k2v -= (-j21 * rp + j11 * rv) / det;
            }

            // ---- Update: y_{n+1} = y_n + h*((1-γ)*k1 + γ*k2) ----
            float dp = dt * ((1.0f - GAMMA) * k1p + GAMMA * k2p);
            float dv = dt * ((1.0f - GAMMA) * k1v + GAMMA * k2v);

            state_.position.z += dp;
            state_.velocity.z += dv;

            // Compute contact force for torque and accelerometer
            // トルクと加速度計用に接触力を計算
            float pen_new = pen0 + dp;
            float vn_new = vn0 + dv;
            Vec3 f_vec = {};
            if (pen_new > 0) {
                float fn = -(k * pen_new + c * vn_new);
                if (fn > 0) fn = 0;
                f_vec.z = fn;
            }

            // Friction (explicit, tangential)
            // 摩擦（陽的、接線方向）
            float vt_sq = v_corner.x * v_corner.x + v_corner.y * v_corner.y;
            if (vt_sq > 1e-8f && f_vec.z < 0) {
                float vt = sqrtf(vt_sq);
                float ff = cp_.mu_friction * fabsf(f_vec.z);
                f_vec.x = -ff * v_corner.x / vt;
                f_vec.y = -ff * v_corner.y / vt;
                state_.velocity.x += f_vec.x * dt / m;
                state_.velocity.y += f_vec.y * dt / m;
            }

            // Contact torque: τ = r × F
            contact_torque_sum += r_world.cross(f_vec);
            contact_force_cache_ += f_vec;
        }

        // Apply contact torque to angular velocity
        // 接触トルクを角速度に適用
        if (contact_torque_sum.norm_sq() > 1e-12f) {
            Vec3 alpha_c;
            alpha_c.x = contact_torque_sum.x / qp_.Ixx;
            alpha_c.y = contact_torque_sum.y / qp_.Iyy;
            alpha_c.z = contact_torque_sum.z / qp_.Izz;

            // Contact torque intensity for angular damping
            // 角減衰のための接触トルク強度
            float intensity = contact_force_cache_.norm() /
                              (qp_.mass * qp_.gravity + 1e-6f);
            float damp = 1.0f / (1.0f + intensity * 20.0f * dt);

            state_.angular_rate.x = state_.angular_rate.x * damp + alpha_c.x * dt;
            state_.angular_rate.y = state_.angular_rate.y * damp + alpha_c.y * dt;
            state_.angular_rate.z = state_.angular_rate.z * damp + alpha_c.z * dt;
        }
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
    Vec3 contact_force_cache_ = {};
};

}  // namespace sim
}  // namespace sf
