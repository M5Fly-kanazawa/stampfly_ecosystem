/**
 * @file quad_physics.hpp
 * @brief Quadrotor physics engine with contact dynamics
 *        接触力学付きクアッドロータ物理エンジン
 *
 * - RK4 integration for rigid body dynamics
 * - Box collision model (81.5 x 81.5 x 31mm) with 8 corner contacts
 * - Constraint-based ground contact (PGS solver + Baumgarte stabilization)
 * - Coulomb friction
 * - No if-based ground constraints — all interactions are forces
 *
 * - 剛体ダイナミクスにRK4積分
 * - 立方体コリジョンモデル（81.5×81.5×31mm）8頂点接触
 * - 拘束ベース地面接触（PGSソルバー + バウムガルテ安定化）
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
        a_inertial_ned_ = {};
        vel_before_step_ = {};
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

        // Save velocity before physics for inertial acceleration calculation
        // 慣性加速度計算のため物理前の速度を保存
        vel_before_step_ = state_.velocity;

        // Run physics at higher rate (10x control rate)
        // 物理を高レートで実行（制御レートの10倍）
        float sub_dt = dt / PHYSICS_SUBSTEPS;
        for (int sub = 0; sub < PHYSICS_SUBSTEPS; sub++) {
            physicsSubStep(sub_dt);
        }

        // Inertial acceleration = velocity change / dt
        // 慣性加速度 = 速度変化 / dt
        // This is the TRUE acceleration of the body, averaged over the step.
        // ステップ全体で平均化された、機体の真の加速度。
        a_inertial_ned_ = (state_.velocity - vel_before_step_) * (1.0f / dt);
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

        // Accelerometer model
        // 加速度計モデル
        //
        // a_inertial = Δv / Δt  (inertial acceleration, zero at rest)
        // accel_output = a_inertial - g_body  (what the accelerometer outputs)
        //
        // a_inertial = Δv / Δt（慣性加速度、静止時ゼロ）
        // accel_output = a_inertial - g_body（加速度計の出力）
        //
        // At rest (level): a_inertial = 0
        //   g_body = inv_rotate([0, 0, +g]) = [0, 0, +g]
        //   accel_output_body = 0 - [0, 0, +g] = [0, 0, -g]  → -9.81 ✓
        //
        // 静止時（水平）: a_inertial = 0
        //   g_body = inv_rotate([0, 0, +g]) = [0, 0, +g]
        //   accel_output_body = 0 - [0, 0, +g] = [0, 0, -g]  → -9.81 ✓
        //
        Vec3 g_ned(0, 0, qp_.gravity);
        Vec3 accel_output_ned = a_inertial_ned_ - g_ned;
        Vec3 accel_output_body = state_.attitude.inv_rotate(accel_output_ned);

        // Add noise
        // ノイズを追加
        float accel_sigma = sqrtf(
            (np_.accel_noise_density * sqrtf(fs)) * (np_.accel_noise_density * sqrtf(fs))
            + (np_.vib_accel_k * duty * duty) * (np_.vib_accel_k * duty * duty));
        s.accel.x = accel_output_body.x + randn() * accel_sigma + accel_bias_[0];
        s.accel.y = accel_output_body.y + randn() * accel_sigma + accel_bias_[1];
        s.accel.z = accel_output_body.z + randn() * accel_sigma + accel_bias_[2];

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
        // =================================================================
        // Per-contact-point PGS with full angular coupling
        // 各接触点PGS（角運動量結合付き）
        //
        // Based on Erin Catto's sequential impulse method (Box2D/GDC2006).
        // Each contact impulse at offset r from COM produces:
        //   Δv     = λ * n / m           (linear velocity change)
        //   Δω     = I⁻¹ * (r × λn)     (angular velocity change)
        //
        // The restoring torque on a tilted body is NOT a separate mechanism —
        // it emerges naturally from applying normal impulses at offset points.
        //
        // Catto法（Box2D/GDC2006）に基づく逐次インパルス法。
        // オフセットrの接触点でのインパルスは線形・角速度を同時に変化させる。
        // 傾いた物体の復元トルクは別メカニズムではなく、
        // オフセット接触点への法線インパルス適用から自然に発生する。
        // =================================================================

        static constexpr int PGS_ITERATIONS = 20;
        static constexpr float RESTITUTION = 0.25f;
        static constexpr float BAUMGARTE = 0.2f;    // Position correction rate
        static constexpr float SLOP = 0.0005f;      // 0.5mm allowed penetration

        contact_force_cache_ = {};

        // --- Collect active contacts ---
        // --- アクティブ接触点を収集 ---
        struct ContactPoint {
            Vec3 r_world;      // COM to contact point (world frame)
            int corner_idx;    // Corner index (for warm starting)
            float pen;         // Penetration depth (positive = below ground)
            float vn_init;     // Initial normal velocity (positive = away)
            float m_eff;       // Effective mass at this contact
            float bias;        // Velocity bias (Baumgarte + restitution)
            float rxn_x;      // (r × n).x component
            float rxn_y;      // (r × n).y component
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
            float vn = -v_corner.z;  // Normal velocity (positive = away)

            // r × n where n = [0,0,-1]
            // r × n（n = [0,0,-1]）
            float rxn_x = -r_world.y;
            float rxn_y =  r_world.x;

            // Effective mass: 1/m_eff = 1/m + (r×n)^T I^{-1} (r×n)
            // 有効質量: 1/m_eff = 1/m + (r×n)^T I^{-1} (r×n)
            float inv_m_eff = 1.0f / qp_.mass
                + rxn_x * rxn_x / qp_.Ixx
                + rxn_y * rxn_y / qp_.Iyy;
            float m_eff = 1.0f / inv_m_eff;

            // Velocity bias: Baumgarte position correction + restitution
            // 速度バイアス: バウムガルテ位置補正 + 反発
            float bias = 0;
            float pen_excess = pen - SLOP;
            if (pen_excess > 0) {
                bias += BAUMGARTE * pen_excess / dt;
            }
            if (vn < -0.01f) {
                // Impact: add restitution bounce velocity
                // 衝突: 反発跳ね返り速度を追加
                bias += -RESTITUTION * vn;  // positive (away from ground)
            }

            auto& cp = contacts[n_contacts];
            cp.r_world = r_world;
            cp.corner_idx = ci;
            cp.pen = pen;
            cp.vn_init = vn;
            cp.m_eff = m_eff;
            cp.bias = bias;
            cp.rxn_x = rxn_x;
            cp.rxn_y = rxn_y;
            n_contacts++;
        }

        if (n_contacts == 0) {
            // No contact: clear warm start lambdas
            // 接触なし: ウォームスタート用λをクリア
            for (int i = 0; i < 8; i++) warm_lambda_[i] = 0;
            return;
        }

        // --- Warm start: initialize from previous frame's lambdas ---
        // --- ウォームスタート: 前フレームのλで初期化 ---
        // Scale by 0.8 to prevent over-accumulation during transitions
        // 遷移中の過蓄積を防ぐため0.8でスケーリング
        // --- Jacobi iteration (simultaneous update) ---
        // --- ヤコビ反復（同時更新）---
        //
        // Unlike Gauss-Seidel, Jacobi computes ALL deltas from current state,
        // then applies ALL at once. This eliminates ordering-dependent asymmetry
        // that GS creates for symmetric contact configurations.
        //
        // For 4 symmetric corners on flat ground, the contact matrix has a zero
        // eigenvalue (degenerate angular mode). GS cannot converge for this mode,
        // but Jacobi preserves the initial symmetry (zero) exactly.
        //
        // ガウス-ザイデルと異なり、ヤコビ法は全接触のdeltaを現在の状態から計算し、
        // 一括で適用する。対称接触構成でGSが生む順序依存の非対称性を排除。
        //
        // 4対称頂点の接触行列には固有値0（縮退角モード）がある。
        // GSはこのモードで収束不可能だが、ヤコビ法は初期対称性を正確に保存する。
        // =================================================================

        float lambda_acc[8] = {};

        for (int iter = 0; iter < PGS_ITERATIONS; iter++) {
            // Phase 1: Compute all deltas from current state
            // フェーズ1: 現在の状態から全deltaを計算
            float delta[8] = {};
            for (int ci = 0; ci < n_contacts; ci++) {
                const auto& cp = contacts[ci];

                Vec3 v_at_point = state_.velocity
                    + state_.angular_rate.cross(cp.r_world);
                float vn = -v_at_point.z;

                float lambda_new = fmaxf(0,
                    lambda_acc[ci] + cp.m_eff * (cp.bias - vn));
                delta[ci] = lambda_new - lambda_acc[ci];
                lambda_acc[ci] = lambda_new;
            }

            // Phase 2: Apply all deltas simultaneously
            // フェーズ2: 全deltaを同時に適用
            for (int ci = 0; ci < n_contacts; ci++) {
                state_.velocity.z -= delta[ci] / qp_.mass;
                state_.angular_rate.x += delta[ci] * contacts[ci].rxn_x / qp_.Ixx;
                state_.angular_rate.y += delta[ci] * contacts[ci].rxn_y / qp_.Iyy;
            }
        }

        // --- Friction (Coulomb, explicit) ---
        // --- 摩擦（クーロン、陽的）---
        for (int ci = 0; ci < n_contacts; ci++) {
            float fn = lambda_acc[ci] / dt;  // Normal force at contact
            Vec3 v_at_point = state_.velocity
                + state_.angular_rate.cross(contacts[ci].r_world);
            Vec3 vt(v_at_point.x, v_at_point.y, 0);
            float vt_mag = vt.norm();

            if (vt_mag > 1e-6f && fn > 0) {
                float f_friction = fminf(cp_.mu_friction * fn,
                                         qp_.mass * vt_mag / dt);
                Vec3 friction_impulse = vt * (-f_friction * dt / vt_mag);
                state_.velocity.x += friction_impulse.x / qp_.mass;
                state_.velocity.y += friction_impulse.y / qp_.mass;
            }
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

    // Accelerometer model: velocity-based inertial acceleration
    // 加速度計モデル: 速度ベースの慣性加速度
    Vec3 vel_before_step_ = {};       // Velocity before physics step
    Vec3 a_inertial_ned_ = {};        // Inertial acceleration (Δv/Δt)

    // Contact solver state
    // 接触ソルバー状態
    Vec3 contact_force_cache_ = {};
    Vec3 free_force_cache_ = {};        // Non-contact force (thrust+gravity+drag)
    float warm_lambda_[8] = {};         // Warm start: previous frame's impulses
};

}  // namespace sim
}  // namespace sf
