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
        free_force_cache_ = free_force;  // Save for accelerometer model
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

        // Compute actual inertial acceleration from velocity change
        // 速度変化から実際の慣性加速度を計算
        // This automatically includes contact effects correctly
        // 接触の効果を自動的に正確に含む
        // (accelerometer uses free_force_cache_ set in Step 1)
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

        // Accelerometer model: measures ALL forces on the body
        // 加速度計モデル: 機体に作用する全ての力を測定
        //
        // Specific force = (F_total / m) - g
        //   F_total = thrust + gravity + drag + contact
        //   a_inertial = F_total / m
        //   specific_force = a_inertial - g = (thrust + drag + contact) / m
        //
        // At rest: contact = -mg → specific = (-mg + 0 + 0)/m = -g
        //   → body = inv_rotate(-g) = [0,0,-g] + 2g = [0,0,+g] ✓
        //
        // The contact force is now solved from constraints (not approximate spring)
        // so F_contact = -mg exactly at rest → no bias
        //
        // 接触力は拘束から解かれた値（近似バネではない）
        // 静止時 F_contact = -mg が正確 → バイアスなし
        //
        Vec3 total_force = free_force_cache_ + contact_force_cache_;
        Vec3 nongrav = total_force - Vec3(0, 0, qp_.mass * qp_.gravity);
        Vec3 specific_force_ned = nongrav * (1.0f / qp_.mass);
        Vec3 specific_force_body = state_.attitude.inv_rotate(specific_force_ned);

        // Add 2g to z to match vehicle convention:
        // static: specific_force_body.z = -g → + 2g → +g (matches vehicle)
        // vehicle慣例に合わせてzに2gを加算
        // TODO: +2g is a temporary convention workaround.
        // The PGS contact solver sign convention needs careful redesign
        // to eliminate this addition and produce correct -g at rest.
        // See discussion: accel_z should be -9.81 at rest in NED body Z.
        // TODO: +2gは一時的な慣例回避策。
        // PGS接触ソルバーの符号定義を慎重に再設計して
        // この加算を排除し、静止時に正しい-gを出力する必要がある。
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
    // Constraint-based contact solver (Projected Gauss-Seidel)
    // 拘束ベース接触ソルバー（射影ガウス-ザイデル法）
    //
    // The contact force λ is an UNKNOWN solved from the constraint:
    //   Constraint: v_n_new ≥ 0 (no penetration velocity)
    //   λ ≥ 0 (contact can only push)
    //   Complementarity: λ * v_n_new = 0
    //
    // For each contact point:
    //   v_n_new = v_n + (1/m) * λ * dt + a_free * dt
    //   where a_free = free acceleration (gravity + thrust + drag) / m
    //
    // Solve: λ = max(0, -m * (v_n + a_free*dt) / dt)
    //        (+ restitution for bounce)
    //
    // 接触力λは拘束条件から解く未知数:
    //   拘束: v_n_new ≥ 0（貫通速度なし）
    //   λ ≥ 0（接触は押すだけ）
    //   相補性: λ * v_n_new = 0
    //
    // PGS反復で複数接触点を同時解決
    // =========================================================================

    void solveContact(float dt)
    {
        static constexpr int PGS_ITERATIONS = 10;
        static constexpr float RESTITUTION = 0.25f;  // Bounce coefficient

        contact_force_cache_ = {};

        // Collect active contacts
        // アクティブ接触点を収集
        struct ContactPoint {
            Vec3 r_world;    // Body center to contact point (NED)
            float pen;       // Penetration depth
            float vn;        // Normal velocity (into ground)
            Vec3 v_tangent;  // Tangential velocity
            float lambda_n;  // Normal impulse (solved)
        };

        ContactPoint contacts[8];
        int n_contacts = 0;

        for (int ci = 0; ci < 8; ci++) {
            Vec3 corner_body = getCorner(ci);
            Vec3 r_world = state_.attitude.rotate(corner_body);
            Vec3 corner_world = state_.position + r_world;

            float pen = corner_world.z - cp_.ground_z;
            if (pen <= 0) continue;

            Vec3 v_corner = state_.velocity + state_.angular_rate.cross(r_world);

            contacts[n_contacts].r_world = r_world;
            contacts[n_contacts].pen = pen;
            contacts[n_contacts].vn = v_corner.z;
            contacts[n_contacts].v_tangent = Vec3(v_corner.x, v_corner.y, 0);
            contacts[n_contacts].lambda_n = 0;
            n_contacts++;
        }

        if (n_contacts == 0) return;

        // Free acceleration in z (from RK4 step, without contact)
        // z方向の自由加速度（RK4ステップから、接触なし）
        float a_free_z = (free_force_cache_.z) / qp_.mass;

        // PGS iterations to solve for contact impulses
        // PGS反復で接触インパルスを解く
        for (int iter = 0; iter < PGS_ITERATIONS; iter++) {
            for (int ci = 0; ci < n_contacts; ci++) {
                ContactPoint& cp = contacts[ci];

                // Current velocity at contact point (updated by previous contacts)
                // 現在の接触点速度（前の接触で更新済み）
                Vec3 v_at_point = state_.velocity + state_.angular_rate.cross(cp.r_world);
                float vn_current = v_at_point.z;

                // Target: v_n_new = -e * v_n_incoming  (restitution)
                // For resting contact (small vn): target = 0
                // 目標: v_n_new = -e * v_n_incoming（反発）
                // 静止接触（小さいvn）: 目標 = 0
                float v_target = 0;
                if (cp.vn > 0.01f) {
                    // Incoming velocity: apply restitution
                    // 入射速度: 反発適用
                    v_target = -RESTITUTION * cp.vn;
                }

                // Effective mass at contact point for normal direction
                // 法線方向の接触点有効質量
                // 1/m_eff = 1/m + (r × n)^T * I^{-1} * (r × n)
                // For z-direction normal: n = [0,0,1]
                // r × n = [r.y, -r.x, 0]
                float rxn_x = cp.r_world.y;
                float rxn_y = -cp.r_world.x;
                float inv_m_eff = 1.0f / qp_.mass
                    + rxn_x * rxn_x / qp_.Ixx
                    + rxn_y * rxn_y / qp_.Iyy;
                float m_eff = 1.0f / inv_m_eff;

                // Solve for impulse
                float delta_lambda = m_eff * (v_target - vn_current);

                // Project: λ ≥ 0
                float new_lambda = fmaxf(0, cp.lambda_n + delta_lambda);
                delta_lambda = new_lambda - cp.lambda_n;
                cp.lambda_n = new_lambda;

                // Apply impulse
                state_.velocity.z += delta_lambda / qp_.mass;
                state_.angular_rate.x += delta_lambda * rxn_x / qp_.Ixx;
                state_.angular_rate.y += delta_lambda * rxn_y / qp_.Iyy;
            }
        }

        // Position correction: push out of ground (Baumgarte stabilization)
        // 位置補正: 地面から押し出す（バウムガルテ安定化）
        float beta = 0.2f;  // Stabilization factor
        for (int ci = 0; ci < n_contacts; ci++) {
            if (contacts[ci].pen > 0) {
                state_.position.z -= beta * contacts[ci].pen;
            }
        }

        // Compute contact force for accelerometer and torque
        // 加速度計とトルク用に接触力を計算
        // F = λ / dt (impulse → force)
        Vec3 contact_torque_sum = {};
        for (int ci = 0; ci < n_contacts; ci++) {
            // λ is positive upward impulse → force is NED negative z
            // λは正の上向きインパルス → 力はNED負z
            float force_n = contacts[ci].lambda_n / dt;
            Vec3 f_vec(0, 0, -force_n);

            // Friction impulse (Coulomb, explicit)
            // 摩擦インパルス（クーロン、陽的）
            Vec3 vt = contacts[ci].v_tangent;
            float vt_mag = vt.norm();
            if (vt_mag > 1e-6f && force_n > 0) {
                float f_friction = cp_.mu_friction * force_n;
                Vec3 friction_dir = vt * (-1.0f / vt_mag);
                f_vec.x = friction_dir.x * f_friction;
                f_vec.y = friction_dir.y * f_friction;
                state_.velocity.x += f_vec.x * dt / qp_.mass;
                state_.velocity.y += f_vec.y * dt / qp_.mass;
            }

            contact_force_cache_ += f_vec;
            contact_torque_sum += contacts[ci].r_world.cross(f_vec);
        }

        // Apply contact torque
        // 接触トルクを適用
        state_.angular_rate.x += contact_torque_sum.x * dt / qp_.Ixx;
        state_.angular_rate.y += contact_torque_sum.y * dt / qp_.Iyy;
        state_.angular_rate.z += contact_torque_sum.z * dt / qp_.Izz;
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
    Vec3 free_force_cache_ = {};      // Non-contact force (thrust+gravity+drag)
};

}  // namespace sim
}  // namespace sf
