/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file plant.cpp
 * @brief SIL physics plant implementation (motor model + truth + synthetic sensors).
 *        SIL 物理プラント実装（モータモデル＋真値＋合成センサ）。
 *
 * Sign/formula provenance: simulator/sil/docs/coordinate_frames.md §4 and the
 * firmware ESKF observation models. All frame conversions go through frames.hpp.
 *
 * 符号・式の出典: simulator/sil/docs/coordinate_frames.md §4 とファーム ESKF の
 * 観測モデル。全ての座標変換は frames.hpp を経由する。
 */

#include "plant.hpp"

#include <cmath>
#include <cstdio>

namespace sil {

using sf::math::Vec3;
using sf::math::Quat;

namespace {

// Quintic smootherstep S(u)=6u⁵−15u⁴+10u³ on u∈[0,1] and its first/second derivatives
// (w.r.t. u). It is C²: S=S'=S''=0 at u=0 and S=1,S'=S''=0 at u=1, so chaining phases at
// these endpoints keeps position, velocity AND acceleration continuous — the handling
// trajectory (and therefore the analytic IMU specific force) has no jump.
// 五次 smootherstep S(u)=6u⁵−15u⁴+10u³（u∈[0,1]）と一階・二階微分（u について）。C²:
// u=0 で S=S'=S''=0、u=1 で S=1,S'=S''=0 ゆえ、この端点で相を連結すると位置・速度・加速度が
// 連続 ― ハンドリング軌道（と解析 IMU 比力）に飛びが無い。
inline float smoothstep(float u)   { return ((6.0f * u - 15.0f) * u + 10.0f) * u * u * u; }
inline float smoothstep_d1(float u){ return 30.0f * u * u * (u - 1.0f) * (u - 1.0f); }
inline float smoothstep_d2(float u){ return 60.0f * u * (2.0f * u - 3.0f) * u + 60.0f * u; }

// Rotation vector (axis·angle, [rad]) of a unit quaternion — the inverse of
// Quat::from_rotvec. Shortest-path: a negative scalar part is flipped so the angle is in
// [0,π]. Returns 0 for the identity. 単位クォータニオンの回転ベクトル（axis·angle[rad]）—
// Quat::from_rotvec の逆。最短経路（負のスカラー部は反転し角を[0,π]に）。恒等は 0。
inline Vec3 rotvec_of(Quat q)
{
    if (q.w < 0.0f) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }  // shortest path
    const float vn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (vn < 1e-9f) return {0.0f, 0.0f, 0.0f};                           // ~identity
    const float angle = 2.0f * std::atan2(vn, q.w);                      // ∈ [0,π]
    const float s = angle / vn;
    return {q.x * s, q.y * s, q.z * s};
}

}  // namespace

Plant::~Plant()
{
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
}

// -----------------------------------------------------------------------------
// init — load MJCF, cache ids, zero the controls, compute the initial sensors.
// init — MJCF を読み、id をキャッシュ、制御をゼロ化、初期センサを計算。
// -----------------------------------------------------------------------------
bool Plant::init(const char* model_path, const Config& cfg)
{
    cfg_ = cfg;

    char error[1024] = {0};
    m_ = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m_) {
        fprintf(stderr, "[plant] model load failed: %s\n", error);
        return false;
    }
    d_ = mj_makeData(m_);

    body_id_   = mj_name2id(m_, mjOBJ_BODY,   "drone");
    accel_sid_ = mj_name2id(m_, mjOBJ_SENSOR, "imu_accel");
    gyro_sid_  = mj_name2id(m_, mjOBJ_SENSOR, "imu_gyro");
    quat_sid_  = mj_name2id(m_, mjOBJ_SENSOR, "body_quat");
    pos_sid_   = mj_name2id(m_, mjOBJ_SENSOR, "body_pos");
    vel_sid_   = mj_name2id(m_, mjOBJ_SENSOR, "body_vel");

    // Seed the sensor-noise model (default OFF). Deterministic per cfg.noise.seed.
    // センサノイズモデルをシード（既定 OFF）。cfg.noise.seed で決定論的。
    noise_.init(cfg_.noise);

    // Motor transport delay: convert cfg.motor_delay_ms to an integer number of
    // substeps at this model's fixed timestep h (rounded to the nearest substep —
    // at 4 kHz that is ±0.125 ms, well under the ms-scale delay being modeled), and
    // size each motor's ring buffer to hold exactly that many past duty-target
    // samples, zero-initialized (matches the all-motors-off boot state). 0 → empty
    // vectors, so substep() takes the explicit bypass branch (byte-identical to the
    // pre-delay code).
    // モータ輸送遅れ: cfg.motor_delay_ms を本モデルの固定刻み h での substep 整数個数へ
    // 変換（最近傍丸め — 4kHz では±0.125ms、ms オーダーのむだ時間に対し十分小さい）し、
    // 各モータのリングバッファをその個数ぶんゼロ初期化で確保（全モータ停止のブート状態と
    // 整合）。0＝空ベクタ＝substep() は明示バイパス分岐を通る（遅延導入前とバイト一致）。
    const double h = m_->opt.timestep;
    delay_n_ = (cfg_.motor_delay_ms > 0.0f)
                   ? (int)std::lround((double)cfg_.motor_delay_ms * 1e-3 / h)
                   : 0;
    delay_head_ = 0;
    for (int i = 0; i < 4; ++i) delay_buf_[i].assign((size_t)delay_n_, 0.0f);

    // Initialize the battery supply voltage. With the sag model OFF this stays the
    // fixed nominal (cfg_.v_batt) forever; with it ON, start from the initial SoC.
    // 電池電源電圧を初期化。サグモデル OFF なら固定公称（cfg_.v_batt）のまま、ON なら
    // 初期 SoC から開始する。
    if (cfg_.batt_model_enable) {
        batt_charge_mah_ = cfg_.batt_initial_frac * cfg_.batt_capacity_mah;
        // At rest only the avionics draw flows, so the initial sag is tiny.
        // 静止時はアビオ電流のみ流れるため初期サグは僅か。
        v_batt_ = ocvFromCharge(batt_charge_mah_) - cfg_.avionics_current_a * cfg_.batt_r_int;
    } else {
        v_batt_ = cfg_.v_batt;
    }

    // Start with motors off and compute sensordata for the initial pose.
    // モータ停止で開始し、初期姿勢のセンサ値を計算する。
    for (int i = 0; i < m_->nu; ++i) d_->ctrl[i] = 0.0;
    mj_forward(m_, d_);

    return body_id_ >= 0 && accel_sid_ >= 0 && gyro_sid_ >= 0 &&
           quat_sid_ >= 0 && pos_sid_ >= 0 && vel_sid_ >= 0;
}

void Plant::setDuty(const sf::MotorOutput& cmd)
{
    // Index 1:1: duty[0..3] = M1FR/M2RR/M3RL/M4FL = MJCF rotor1..4. No reshuffle.
    // 添字 1:1: duty[0..3] = M1FR/M2RR/M3RL/M4FL = MJCF rotor1..4。並べ替えなし。
    for (int i = 0; i < 4; ++i) motor_target_[i] = cmd.duty[i];
}

void Plant::primeMotors()
{
    for (int i = 0; i < 4; ++i) {
        motor_duty_[i] = motor_target_[i];
        // Also pre-fill the transport-delay ring buffer (if enabled) with the target
        // duty, so a delayed path does not ramp up from the zero-filled boot buffer —
        // consistent with skipping the first-order spool-up lag just above.
        // 輸送遅れリングバッファ（有効時）も目標 duty で埋める — 上で一次遅れの
        // スプールアップを省略しているのと整合させる。
        for (size_t k = 0; k < delay_buf_[i].size(); ++k) delay_buf_[i][k] = motor_target_[i];
    }
}

void Plant::setWind(const sf::math::Vec3& force_ned)
{
    cfg_.wind_force_ned = force_ned;
}

void Plant::setHealth(int motor, float gain)
{
    if (motor < 0 || motor > 3) return;
    cfg_.health[motor] = gain < 0.0f ? 0.0f : (gain > 1.0f ? 1.0f : gain);
}

void Plant::setImuBias(const sf::math::Vec3& accel_bias, const sf::math::Vec3& gyro_bias)
{
    cfg_.imu_bias_accel = accel_bias;
    cfg_.imu_bias_gyro  = gyro_bias;
}

void Plant::setStartHeight(float z)
{
    // Free-joint state: position (0,0,z) ENU, level orientation, zero velocity.
    // フリージョイント状態: 位置(0,0,z) ENU・水平姿勢・速度ゼロ。
    d_->qpos[0] = 0.0; d_->qpos[1] = 0.0; d_->qpos[2] = z;
    d_->qpos[3] = 1.0; d_->qpos[4] = 0.0; d_->qpos[5] = 0.0; d_->qpos[6] = 0.0;
    for (int i = 0; i < 6; ++i) d_->qvel[i] = 0.0;
    ground_rest_z_enu_ = z;   // remember the ground rest height for handling placement
    mj_forward(m_, d_);
}

// -----------------------------------------------------------------------------
// startHandling — capture the (crashed) pose and arm the kinematic lift→right→
// carry→place trajectory. See plant.hpp for the maneuver description.
// startHandling — （墜落した）姿勢を捕え、持上げ→正立→運搬→設置のキネマティック軌道を
// 起動する。動作の説明は plant.hpp。
// -----------------------------------------------------------------------------
void Plant::startHandling(float carry_alt_m, float place_x_ned, float place_y_ned,
                          float lift_s, float carry_s, float place_s)
{
    Truth t = truth();                              // capture the current (crash) pose
    handle_p0_ned_      = t.pos_ned;
    handle_q0_nb_       = t.q_nb;
    // Right-to-level rotation vector (FRD): r such that q0 ⊗ from_rotvec(r) = identity,
    // i.e. r = rotvec(q0⁻¹) — the same SLERP levels a tilted OR inverted crash.
    // 水平へ起こす回転ベクトル（FRD）: q0 ⊗ from_rotvec(r)=恒等 となる r、すなわち
    // r=rotvec(q0⁻¹) ― 同じ SLERP が傾いた墜落も反転した墜落も水平へ起こす。
    handle_rv_flip_frd_ = rotvec_of(handle_q0_nb_.conj());

    handle_carry_z_ned_  = -carry_alt_m;            // NED z (up = −z)
    handle_place_x_ned_  = place_x_ned;
    handle_place_y_ned_  = place_y_ned;
    handle_ground_z_ned_ = -ground_rest_z_enu_;     // place back on the ground rest height
    handle_lift_s_  = lift_s  > 1e-3f ? lift_s  : 1e-3f;
    handle_carry_s_ = carry_s > 1e-3f ? carry_s : 1e-3f;
    handle_place_s_ = place_s > 1e-3f ? place_s : 1e-3f;
    handle_t_ = 0.0f;
    handling_active_ = true;
}

// -----------------------------------------------------------------------------
// handlingSubstep — advance the prescribed handling trajectory by h [s]. Evaluates the
// three smootherstep phases (lift / carry / place) for position + a SLERP-to-level for
// attitude, writes qpos/qvel to MuJoCo (mj_forward refreshes the position sensors), and
// stores the analytic IMU specific force + gyro. NO mj_step — the body follows the hand,
// not the dynamics.
// handlingSubstep — 規定ハンドリング軌道を h[s] 進める。3つの smootherstep 相（lift/carry/
// place）の位置と水平への SLERP 姿勢を評価し、qpos/qvel を MuJoCo に書込み（mj_forward が位置
// センサを更新）、解析 IMU 比力＋ジャイロを保存。mj_step なし ― 機体は動力学でなく手に従う。
// -----------------------------------------------------------------------------
void Plant::handlingSubstep(float h)
{
    const float t      = handle_t_;
    const float total  = handle_lift_s_ + handle_carry_s_ + handle_place_s_;
    const float t_flip = handle_lift_s_ + handle_carry_s_;   // righting spans lift+carry

    // --- Position / velocity / acceleration in NED via the active phase's smootherstep -
    // --- 有効相の smootherstep で NED の位置/速度/加速度 ---
    Vec3 pos = handle_p0_ned_, vel{}, acc{};
    if (t < handle_lift_s_) {
        // LIFT: rise straight up from the crash z to the carry altitude.
        // LIFT: 墜落 z から carry 高度へ真上に上げる。
        const float T = handle_lift_s_, u = t / T;
        const float dz = handle_carry_z_ned_ - handle_p0_ned_.z;
        pos.z = handle_p0_ned_.z + dz * smoothstep(u);
        vel.z = dz * smoothstep_d1(u) / T;
        acc.z = dz * smoothstep_d2(u) / (T * T);
    } else if (t < t_flip) {
        // CARRY: translate horizontally to the placement spot, hold the carry altitude.
        // CARRY: 設置点へ水平並進、carry 高度を保持。
        const float T = handle_carry_s_, u = (t - handle_lift_s_) / T;
        const float dx = handle_place_x_ned_ - handle_p0_ned_.x;
        const float dy = handle_place_y_ned_ - handle_p0_ned_.y;
        pos.x = handle_p0_ned_.x + dx * smoothstep(u);
        pos.y = handle_p0_ned_.y + dy * smoothstep(u);
        pos.z = handle_carry_z_ned_;
        vel.x = dx * smoothstep_d1(u) / T;
        vel.y = dy * smoothstep_d1(u) / T;
        acc.x = dx * smoothstep_d2(u) / (T * T);
        acc.y = dy * smoothstep_d2(u) / (T * T);
    } else {
        // PLACE: lower straight down onto the ground at the placement spot.
        // PLACE: 設置点で真下に地面へ降ろす。
        const float T = handle_place_s_;
        float u = (t - t_flip) / T;
        if (u > 1.0f) u = 1.0f;
        const float dz = handle_ground_z_ned_ - handle_carry_z_ned_;
        pos.x = handle_place_x_ned_;
        pos.y = handle_place_y_ned_;
        pos.z = handle_carry_z_ned_ + dz * smoothstep(u);
        vel.z = dz * smoothstep_d1(u) / T;
        acc.z = dz * smoothstep_d2(u) / (T * T);
    }

    // --- Attitude: SLERP from the crash pose to level over [0, t_flip] -----------------
    // q_nb(σ) = q0 ⊗ from_rotvec(σ·rv); body-frame (FRD) angular rate ω = σ̇·rv.
    // --- 姿勢: [0,t_flip] で墜落姿勢から水平へ SLERP。機体(FRD)角速度 ω=σ̇·rv。 ---
    float sigma = 1.0f, sigma_dot = 0.0f;
    if (t < t_flip) {
        const float u = t / t_flip;
        sigma     = smoothstep(u);
        sigma_dot = smoothstep_d1(u) / t_flip;
    }
    Quat q_nb = handle_q0_nb_ * Quat::from_rotvec(handle_rv_flip_frd_ * sigma);
    q_nb.normalize();
    Vec3 omega_frd = handle_rv_flip_frd_ * sigma_dot;

    // --- Write the prescribed kinematic state into MuJoCo, refresh the sensors ----------
    // --- 規定キネマティック状態を MuJoCo に書込み、センサを更新 ---
    Vec3 pos_enu = frames::ned_to_enu(pos);
    Vec3 vel_enu = frames::ned_to_enu(vel);
    Vec3 omega_flu = frames::frd_to_flu(omega_frd);   // MuJoCo free-joint ω is body FLU
    Quat q_mj = frames::qnb_to_mujoco_quat(q_nb);
    d_->qpos[0] = pos_enu.x; d_->qpos[1] = pos_enu.y; d_->qpos[2] = pos_enu.z;
    d_->qpos[3] = q_mj.w;    d_->qpos[4] = q_mj.x;    d_->qpos[5] = q_mj.y; d_->qpos[6] = q_mj.z;
    d_->qvel[0] = vel_enu.x; d_->qvel[1] = vel_enu.y; d_->qvel[2] = vel_enu.z;
    d_->qvel[3] = omega_flu.x; d_->qvel[4] = omega_flu.y; d_->qvel[5] = omega_flu.z;
    mj_forward(m_, d_);

    // --- Analytic IMU: specific force from the trajectory's acceleration + gyro ----------
    // accel_body_frd(a,q) = q.inv_rotate(a − g_ned): at rest (a=0, level) → [0,0,−9.81].
    // --- 解析 IMU: 軌道の加速度から比力＋ジャイロ ---
    handle_accel_frd_ = frames::accel_body_frd(acc, q_nb);
    handle_gyro_frd_  = omega_frd;

    // Advance the noise clock (motors off while handled) so the seeded model stays in
    // step; no-op when noise is disabled. ノイズ時計を進める（手持ち中モータ停止）。無効時 no-op。
    noise_.setThrottle(0.0f);
    noise_.advance(h);

    handle_t_ += h;
    if (handle_t_ >= total) {
        // Done: snap to the exact placed-level-rest pose with zero velocity so the handoff
        // back to dynamics is seamless (the craft sits still on the ground, ready to re-arm).
        // 完了: 設置・水平・静止の正確な姿勢へスナップし速度ゼロ ― 動力学への引継ぎが滑らか
        //（機体は地面に静止、再 ARM 可能）。
        Vec3 rest_enu = frames::ned_to_enu(
            {handle_place_x_ned_, handle_place_y_ned_, handle_ground_z_ned_});
        d_->qpos[0] = rest_enu.x; d_->qpos[1] = rest_enu.y; d_->qpos[2] = rest_enu.z;
        d_->qpos[3] = 1.0; d_->qpos[4] = 0.0; d_->qpos[5] = 0.0; d_->qpos[6] = 0.0;
        for (int i = 0; i < 6; ++i) d_->qvel[i] = 0.0;
        mj_forward(m_, d_);
        handle_accel_frd_ = {0.0f, 0.0f, -9.81f};
        handle_gyro_frd_  = {0.0f, 0.0f, 0.0f};
        handling_active_ = false;
    }
}

// -----------------------------------------------------------------------------
// dutyToThrust — normalized duty [0,1] → steady-state thrust [N] via the real
// motor + propeller curve (docs/architecture/stampfly-parameters.md §3):
//   V = duty·v_batt ; solve V = Am·ω² + Bm·ω + Cm for ω ; T = Ct·ω².
// dutyToThrust — 正規化 duty[0,1] → 実モータ＋プロペラ曲線で定常推力 [N]。
// -----------------------------------------------------------------------------
// solveOmega — positive root of Am·ω² + Bm·ω + (Cm − V) = 0 (0 if V ≤ Cm).
// solveOmega — Am·ω²+Bm·ω+(Cm−V)=0 の正の根（V≤Cm なら 0）。
float Plant::solveOmega(float V) const
{
    if (V <= cfg_.motor_Cm) return 0.0f;           // below the offset → no spin
    const float a = cfg_.motor_Am;
    const float b = cfg_.motor_Bm;
    const float disc = b * b + 4.0f * a * (V - cfg_.motor_Cm);  // > 0 since V > Cm
    const float omega = (-b + std::sqrt(disc)) / (2.0f * a);
    return omega > 0.0f ? omega : 0.0f;
}

float Plant::dutyToThrust(float duty) const
{
    if (duty <= 0.0f) return 0.0f;
    const float omega = solveOmega(duty * v_batt_);  // V = duty·v_batt (current supply)
    // T = Ct·ω², scaled by the real-world thrust efficiency (see Config::thrust_efficiency:
    // the firmware's HOVER_THRUST_CORRECTION 1.12 compensates this deficit on real hw).
    // T = Ct·ω² に実機推力効率を乗ずる（ファームの 1.12 補償が打ち消す実機の欠損）。
    return cfg_.thrust_efficiency * cfg_.Ct * omega * omega;   // thrust T [N]
}

// ocvFromCharge — remaining charge [mAh] → open-circuit voltage [V] via a simplified
// 1S LiPo discharge curve (ported from vpython/sensors/power_monitor.py): mostly
// linear in SoC with slightly steeper ends, clamped to [empty, full].
// ocvFromCharge — 残容量 [mAh] → 開回路電圧 [V]（簡略 1S LiPo 放電曲線、vpython 移植）。
// SoC にほぼ線形で両端だけ急峻、[empty, full] にクランプ。
float Plant::ocvFromCharge(float charge_mah) const
{
    float pct = charge_mah / cfg_.batt_capacity_mah;   // state of charge [0..1]
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    float adj;
    if      (pct > 0.95f) adj = 0.9f + (pct - 0.9f) * 2.0f;  // steep near full
    else if (pct < 0.05f) adj = pct * 2.0f;                  // steep near empty
    else                  adj = pct;

    float v = cfg_.batt_empty_v + adj * (cfg_.batt_full_v - cfg_.batt_empty_v);
    if (v < cfg_.batt_empty_v) v = cfg_.batt_empty_v;
    if (v > cfg_.batt_full_v)  v = cfg_.batt_full_v;
    return v;
}

// updateBattery — Coulomb-count the consumed charge and recompute the terminal
// voltage v_batt_ = OCV(SoC) − I·R_int. No-op when the model is disabled.
// updateBattery — 消費容量をクーロンカウントし端子電圧 v_batt_=OCV(SoC)−I·R_int を再計算。
// モデル無効時は何もしない。
void Plant::updateBattery(float i_total_a, float h)
{
    if (!cfg_.batt_model_enable) return;            // constant v_batt_ (= cfg_.v_batt)

    // Coulomb counting: charge[mAh] -= I[A]·h[s] / 3.6  (A·s → mAh).
    // クーロンカウント: charge[mAh] -= I[A]·h[s]/3.6（A·s→mAh）。
    batt_charge_mah_ -= i_total_a * h / 3.6f;
    if (batt_charge_mah_ < 0.0f) batt_charge_mah_ = 0.0f;

    // Terminal voltage = open-circuit voltage minus the internal-resistance drop.
    // 端子電圧 = 開回路電圧 − 内部抵抗による電圧降下。
    v_batt_ = ocvFromCharge(batt_charge_mah_) - i_total_a * cfg_.batt_r_int;
}

// -----------------------------------------------------------------------------
// hoverDuty — per-motor duty that gives mg/4 of thrust (inverts the motor curve).
// hoverDuty — mg/4 の推力を出す各モータ duty（モータ曲線の逆算）。
// -----------------------------------------------------------------------------
float Plant::hoverDuty() const
{
    const float thrust = cfg_.mass * cfg_.g / 4.0f;       // per-motor hover thrust [N]
    // Invert T = efficiency·Ct·ω² so the OUTPUT thrust (after efficiency) is mg/4.
    // 効率込みの T=efficiency·Ct·ω² を逆算し、出力推力が mg/4 になる ω を得る。
    const float omega = std::sqrt(thrust / (cfg_.Ct * cfg_.thrust_efficiency));  // ω = √(T/(Ct·η))
    const float V = cfg_.motor_Am * omega * omega + cfg_.motor_Bm * omega + cfg_.motor_Cm;
    return V / v_batt_;                                    // duty = V / v_batt (current supply)
}

// -----------------------------------------------------------------------------
// step — advance the physics by `dt` of VIRTUAL time in fixed model-timestep
// substeps, carrying the remainder so the physics clock tracks the caller's
// virtual clock 1:1.
//
// Why an accumulator: mj_step always advances exactly one model timestep (it
// takes no dt). If the caller invokes step() at an irregular rate (the SIL
// scheduler jumps its virtual clock to the next wake/timer), stepping once per
// call would decouple physics time from virtual time. The accumulator runs the
// integer number of substeps that fit the elapsed `dt` and keeps the remainder,
// so physics time = virtual time. The model timestep is 0.25 ms (4000 Hz) = 10×
// the 400 Hz controller, so the inter-sample plant dynamics under the held motor
// command are integrated (a physics step == control period would leave none).
//
// step — 物理を「仮想時間 dt」ぶん、固定モデル timestep の substep で進め、端数を
// 繰り越して物理時計を呼び出し側の仮想時計と 1:1 に保つ。mj_step は dt を取らず常に
// モデル timestep を1回進めるため、不規則な呼び出し（SIL スケジューラの仮想時計ジャンプ）
// では1呼び出し1ステップだと物理時間が仮想時間から乖離する。累積器で dt に収まる整数回
// の substep を回し端数を残す。モデル timestep は 0.25ms(4000Hz)=400Hz 制御の10倍。
// -----------------------------------------------------------------------------
void Plant::step(float dt)
{
    last_dt_ = dt;  // flow integrates over the elapsed call interval (unchanged)

    // Accumulate the elapsed virtual time and run as many fixed substeps as fit.
    // The model timestep (double) is the substep length h; the remainder carries.
    // 経過仮想時間を累積し、収まる回数だけ固定 substep を回す。端数は次回へ繰り越す。
    const double h = m_->opt.timestep;
    step_accum_ += (double)dt;
    while (step_accum_ >= h) {
        // While a handling maneuver runs, the body follows the PRESCRIBED kinematic
        // trajectory (handlingSubstep) instead of the motor/contact dynamics (substep).
        // ハンドリング動作中は機体が規定キネマティック軌道（handlingSubstep）に従い、
        // モータ/接触の動力学（substep）に従わない。
        if (handling_active_) handlingSubstep((float)h);
        else                  substep((float)h);
        step_accum_ -= h;
    }
}

// -----------------------------------------------------------------------------
// substep — one fixed-timestep physics step of length h:
//   motor lag → thrust → reaction yaw torque + wind → mj_step → advance noise.
//
// MuJoCo <motor gear="0 0 1 0 0 0"> applies thrust along each rotor site's +Z
// (FLU up); the off-center sites give the roll/pitch arm moment for free. Only the
// aerodynamic yaw reaction torque and wind are applied manually via xfrc_applied.
//
// substep — 長さ h の固定刻み物理 1 ステップ。MuJoCo の <motor gear="0 0 1 0 0 0"> は
// 各ロータ site の +Z（FLU 上）に推力を加え、中心からずれた site が腕モーメントを生む。
// ヨー反トルクと風だけ xfrc_applied で手動付与する。
// -----------------------------------------------------------------------------
void Plant::substep(float h)
{
    // First-order motor lag toward the commanded target, then quadratic thrust.
    // Each motor's terminal voltage is duty·v_batt_ (this substep's supply); ω and
    // thrust follow the motor curve, and the electrical current I = (V − Km·ω)/Rm
    // is accumulated to drive the battery sag (computed with this substep's v_batt_,
    // then v_batt_ is updated for the next substep — an explicit 1-substep lag at 4kHz).
    // 指令目標への一次遅れ、続いて二次推力。各モータ端子電圧は duty·v_batt_（本 substep の
    // 電源）。ω・推力はモータ曲線、電気電流 I=(V−Km·ω)/Rm を積算して電池サグを駆動（本
    // substep の v_batt_ で計算し、その後 v_batt_ を更新＝4kHz で陽的に1 substep 遅れ）。
    const float alpha = 1.0f - std::exp(-h / cfg_.motor_tau);
    const float v_supply = v_batt_;          // this substep's supply voltage

    // Motor transport delay (duty-path): the "L" of the identified G(s)=b·e^(−Ls)/
    // (s(Ts+1)) (Config::motor_delay_ms). delay_n_==0 → duty_cmd IS motor_target_ (no
    // buffer touched at all — exact bypass, byte-identical to the pre-delay code).
    // delay_n_>0 → push this substep's commanded target into each motor's ring
    // buffer and read back the value written delay_n_ substeps ago as the input to
    // the first-order lag below.
    // モータ輸送遅れ（duty経路）: 同定モデル G(s) の L に相当（Config::motor_delay_ms）。
    // delay_n_==0 は duty_cmd がそのまま motor_target_（バッファ不触＝遅延導入前と完全に
    // バイト一致）。delay_n_>0 は本 substep の目標を各モータのリングバッファへ push し、
    // delay_n_ substep 前の値を下の一次遅れの入力として読み出す。
    float duty_cmd[4];
    if (delay_n_ == 0) {
        for (int i = 0; i < 4; ++i) duty_cmd[i] = motor_target_[i];
    } else {
        for (int i = 0; i < 4; ++i) {
            duty_cmd[i] = delay_buf_[i][delay_head_];       // read: delay_n_ substeps old
            delay_buf_[i][delay_head_] = motor_target_[i];  // write: this substep's target
        }
        delay_head_ = (delay_head_ + 1) % delay_n_;
    }
    // Ground-effect lift gain at the current body height (ENU z = qpos[2]). Computed once
    // per substep and applied to every motor's thrust — near the floor the rotors make more
    // lift for the same ω, so the craft holds altitude on reduced thrust (the touchdown float).
    // 現在の機体高さ（ENU z = qpos[2]）での地面効果揚力ゲイン。substep ごとに1回計算し全モータ
    // 推力へ適用 — 床近傍では同じ ω でも揚力が増し、機体は低推力で高度を保つ（着陸フロート）。
    const float ge_mult = groundEffectMultiplier(static_cast<float>(d_->qpos[2]));
    float thrust[4];
    float i_total = cfg_.avionics_current_a; // battery current [A] (avionics baseline)
    for (int i = 0; i < 4; ++i) {
        motor_duty_[i] += (duty_cmd[i] - motor_duty_[i]) * alpha;
        const float v_motor = motor_duty_[i] * v_supply;
        const float omega   = solveOmega(v_motor);
        thrust[i] = cfg_.thrust_efficiency * cfg_.Ct * omega * omega * cfg_.health[i] * ge_mult;
        if (i < m_->nu) d_->ctrl[i] = thrust[i];

        // Motor electrical current (DC model: V = I·Rm + Km·ω). Clamp ≥ 0.
        // モータ電気電流（DC モデル: V = I·Rm + Km·ω）。0 以上にクランプ。
        const float i_motor = (v_motor - cfg_.motor_Km * omega) / cfg_.motor_Rm;
        i_total += (i_motor > 0.0f) ? i_motor : 0.0f;
    }

    // Update the battery supply (Coulomb count + IR sag) for the next substep.
    // 次 substep に向けて電池電源を更新（クーロンカウント＋IR サグ）。
    updateBattery(i_total, h);

    // Net reaction yaw torque about body +Z in FLU (Z up). CCW props (M1 FR, M3 RL)
    // give −Z_FLU reaction; CW props (M2 RR, M4 FL) give +Z_FLU.
    // 機体 +Z（FLU・Z上）まわりの正味反トルク。CCW プロペラ(M1 FR, M3 RL)は −Z_FLU、
    // CW プロペラ(M2 RR, M4 FL)は +Z_FLU の反作用。
    const float tau_yaw_flu_z =
        (-thrust[0] - thrust[2] + thrust[1] + thrust[3]) * cfg_.kappa;

    // Express the body-Z(FLU) torque in the world (ENU) using the truth quaternion
    // (framequat: body FLU → world ENU). At level this is just world +Z (up).
    // 真値クォータニオン（framequat: 機体 FLU→世界 ENU）で機体Z(FLU)トルクを世界(ENU)へ。
    // 水平なら世界 +Z（上）になる。
    const double* q = sensor(quat_sid_);
    Quat q_mj{(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
    // Turbulence body TORQUE (roll/pitch/yaw, 3–6 Hz) — excites the RATE loop directly so the
    // rate-loop methods (D-term, notch, retune) can be evaluated. The 1–3 Hz horizontal FORCE
    // (below) is position/attitude-loop territory; the rate-loop wobble (3–6 Hz peaking) needs
    // this torque. Amplitude scales with turbulence_n (× a 0.03 m effective arm). 0 = off.
    // 乱流ボディトルク（3–6Hz）— レートループを直接励起し、レート手法（D項/ノッチ/再整形）を
    // 評価可能にする。1–3Hz水平力は位置/姿勢ループ域、レートふらつき（3–6Hzピーキング）には本トルク。
    Vec3 tau_body = {0.0f, 0.0f, tau_yaw_flu_z};
    if (cfg_.turbulence_n > 0.0f) {
        const float t = turb_t_, k = 2.0f * 3.14159265f, Q = cfg_.turbulence_n * 0.03f;
        tau_body.x += Q * (std::sin(k * 3.7f * t) + 0.6f * std::sin(k * 5.3f * t + 1.2f));
        tau_body.y += Q * (0.8f * std::sin(k * 4.1f * t + 0.6f) + std::sin(k * 5.9f * t + 2.1f));
        tau_body.z += 0.5f * Q * std::sin(k * 2.9f * t + 0.4f);
    }
    Vec3 torque_world = q_mj.rotate(tau_body);

    // Wind force NED → world ENU, plus deterministic band-limited turbulence (1–3 Hz
    // horizontal sinusoid sum) to excite the attitude-wobble band for the wobble study.
    // Incommensurate frequencies → non-repeating-ish but fully repeatable; 0 = off.
    // 風力 NED→世界 ENU ＋ 決定論的帯域制限乱流（1–3Hz水平正弦和）でふらつき帯域を励起。
    Vec3 wind_ned = cfg_.wind_force_ned;
    if (cfg_.turbulence_n > 0.0f) {
        turb_t_ += h;
        const float t = turb_t_, k = 2.0f * 3.14159265f, A = cfg_.turbulence_n;
        wind_ned.x += A * (std::sin(k * 1.3f * t) + 0.7f * std::sin(k * 2.1f * t + 1.0f)
                                                   + 0.5f * std::sin(k * 3.3f * t + 2.0f));
        wind_ned.y += A * (0.8f * std::sin(k * 0.9f * t + 0.3f) + std::sin(k * 1.7f * t + 1.5f)
                                                                + 0.6f * std::sin(k * 2.6f * t + 0.7f));
    }
    Vec3 wind_enu = frames::ned_to_enu(wind_ned);

    // xfrc_applied[6·body + 0..5] = [Fx,Fy,Fz, Tx,Ty,Tz] in the world frame.
    // MuJoCo does NOT auto-clear it, so overwrite every step.
    // xfrc_applied[6·body + 0..5] = 世界座標の [Fx,Fy,Fz, Tx,Ty,Tz]。MuJoCo は自動で
    // クリアしないので毎ステップ上書きする。
    double* f = d_->xfrc_applied + 6 * body_id_;
    f[0] = wind_enu.x;     f[1] = wind_enu.y;     f[2] = wind_enu.z;
    f[3] = torque_world.x; f[4] = torque_world.y; f[5] = torque_world.z;

    mj_step(m_, d_);

    // Feed the current throttle (mean of the four actual motor duties) to the noise
    // model so the N1 throttle-dependent vibration σ tracks the commanded thrust. No
    // effect for N0 (vib disabled). 現在のスロットル（4モータ実 duty の平均）をノイズ
    // モデルへ渡し、N1 のスロットル依存振動 σ を推力に追従させる（N0 では無効）。
    const float mean_duty =
        0.25f * (motor_duty_[0] + motor_duty_[1] + motor_duty_[2] + motor_duty_[3]);
    noise_.setThrottle(mean_duty);

    // Advance the IMU noise one substep (bias random walk + fresh white sample) so
    // the next imu() reads this step's noise. No-op when noise is disabled.
    // IMU ノイズを1 substep 進める（バイアスRW＋新しい白色サンプル）。無効時は無操作。
    noise_.advance(h);
}

// -----------------------------------------------------------------------------
// truth — MuJoCo state → StampFly NED/FRD, via frames only.
// truth — MuJoCo 状態 → StampFly NED/FRD（frames だけで変換）。
// -----------------------------------------------------------------------------
Plant::Truth Plant::truth() const
{
    const double* p = sensor(pos_sid_);   // ENU world position
    const double* v = sensor(vel_sid_);   // ENU world linear velocity
    const double* q = sensor(quat_sid_);  // framequat FLU → ENU
    const double* w = sensor(gyro_sid_);  // FLU body angular rate

    Truth t;
    t.pos_ned   = frames::enu_to_ned({(float)p[0], (float)p[1], (float)p[2]});
    t.vel_ned   = frames::enu_to_ned({(float)v[0], (float)v[1], (float)v[2]});
    t.q_nb      = frames::mujoco_quat_to_qnb({(float)q[0], (float)q[1], (float)q[2], (float)q[3]});
    t.omega_frd = frames::gyro_body_frd({(float)w[0], (float)w[1], (float)w[2]});
    return t;
}

// -----------------------------------------------------------------------------
// imu — synthetic IMU in body-FRD (driver-normalized: gravity −9.8 at rest).
//
// Primary path uses MuJoCo's built-in <accelerometer> (specific force a−g in the
// FLU site frame), mapped to FRD by frames::flu_to_frd → [0,0,−9.81] at rest.
// This matches the driver-normalized convention the firmware ESKF expects.
//
// imu — 機体 FRD の合成 IMU（ドライバ正規化: 静止で重力 −9.8）。
// 主経路は MuJoCo 内蔵 <accelerometer>（FLU site での比力 a−g）を frames::flu_to_frd で
// FRD に写す → 静止で [0,0,−9.81]。ファーム ESKF が期待するドライバ正規化規約と一致。
// -----------------------------------------------------------------------------
sf::ImuData Plant::imu() const
{
    Vec3 accel_frd, gyro_frd;
    if (handling_active_) {
        // Handling: the body is kinematically prescribed (not integrated), so MuJoCo's
        // accelerometer would read the free-dynamics acceleration, not the hand's. Use the
        // specific force / gyro computed analytically from the prescribed trajectory.
        // ハンドリング中: 機体はキネマティックに規定（積分でない）ゆえ MuJoCo 加速度計は
        // 手でなく自由動力学の加速度を読む。規定軌道から解析した比力/ジャイロを使う。
        accel_frd = handle_accel_frd_;
        gyro_frd  = handle_gyro_frd_;
    } else {
        const double* a = sensor(accel_sid_);  // FLU specific force (a − g)
        const double* w = sensor(gyro_sid_);   // FLU body angular rate
        accel_frd = frames::flu_to_frd({(float)a[0], (float)a[1], (float)a[2]});
        gyro_frd  = frames::gyro_body_frd({(float)w[0], (float)w[1], (float)w[2]});
    }

    sf::ImuData out{};
    out.accel[0] = accel_frd.x; out.accel[1] = accel_frd.y; out.accel[2] = accel_frd.z;
    out.gyro[0]  = gyro_frd.x;  out.gyro[1]  = gyro_frd.y;  out.gyro[2]  = gyro_frd.z;
    // Layer the N0 sensor noise (bias + white) on the clean sample. No-op if disabled
    // → the clean path stays byte-identical. RESET_PLAN §13 P5.
    // クリーンサンプルに N0 ノイズ（バイアス＋白色）を載せる。無効時は無操作＝クリーン経路は不変。
    noise_.applyAccel(out.accel);
    noise_.applyGyro(out.gyro);
    // Add the deterministic raw IMU bias last (a constant pre-calibration MEMS offset,
    // body FRD). Default zero → clean path unchanged; nonzero is what the firmware boot
    // calibration measures and removes (P2-3 contrast test).
    // 最後に決定論的な生 IMU バイアスを加算（校正前 MEMS オフセットの定数、機体 FRD）。
    // 既定ゼロ＝クリーン経路は不変。非ゼロをファーム起動校正が測定・除去する（P2-3 対照試験）。
    out.accel[0] += cfg_.imu_bias_accel.x;
    out.accel[1] += cfg_.imu_bias_accel.y;
    out.accel[2] += cfg_.imu_bias_accel.z;
    out.gyro[0]  += cfg_.imu_bias_gyro.x;
    out.gyro[1]  += cfg_.imu_bias_gyro.y;
    out.gyro[2]  += cfg_.imu_bias_gyro.z;
    out.temperature = 25.0f;
    out.timestamp = (uint32_t)(d_->time * 1e6);
    return out;
}

// -----------------------------------------------------------------------------
// tof — downward range. ESKF model: height = distance·cosR·cosP, observe −height.
// So distance = −pos_z / (cosR·cosP). At level hover (alt 0.5) → 0.5 m.
// tof — 下向き距離。ESKF: height = distance·cosR·cosP、観測 −height。よって
// distance = −pos_z/(cosR·cosP)。水平ホバー(高度0.5)で 0.5 m。
// -----------------------------------------------------------------------------
sf::TofData Plant::tof() const
{
    Truth t = truth();
    Vec3 e = t.q_nb.to_euler();             // x=roll, y=pitch, z=yaw
    float denom = std::cos(e.x) * std::cos(e.y);
    float dist = (std::fabs(denom) > 1e-3f) ? (-t.pos_ned.z / denom) : -t.pos_ned.z;

    sf::TofData out{};
    out.distance = dist;
    // Validity reflects whether a real target is in range (from the TRUE distance), NOT
    // the noisy reading — a real VL53 sees the surface and reports VALID even with noise.
    // 妥当性は真の距離（実標的が範囲内か）で決める。ノイズで揺れた値ではない。
    out.valid = (dist > 0.0f && dist < 4.0f);   // VL53L3CX range gate (true distance)

    // N2 observation noise, applied ONLY above the VL53 reliable close-range minimum
    // (kTofMinRange). The body rests at ~13 mm, which is BELOW the sensor's reliable
    // range; adding σ-class noise there drives the reading negative/invalid and makes
    // the boot ToF gate (and the takeoff/landing state machine) chatter. Above ~5 cm the
    // craft is airborne where the noise actually matters for altitude hold. No-op unless
    // N2 obs noise is enabled; then it propagates through the VL53 model to the firmware.
    // N2 観測ノイズは VL53 の信頼近接下限（kTofMinRange）より上でのみ付与。静止高 ~13mm は
    // 信頼範囲より近く、そこへ σ 級ノイズを載せると負/無効になり起動 ToF 判定と離着陸状態機械が
    // チャタリングする。~5cm 超は空中で高度保持にノイズが効く領域。N2 ON 時のみ、VL53 経由で伝播。
    constexpr float kTofMinRange = 0.05f;       // VL53 reliable close-range minimum [m]
    if (dist > kTofMinRange) {
        noise_.applyTof(out.distance);
        if (out.distance < kTofMinRange) out.distance = kTofMinRange;  // stay in valid range
    }
    out.status = 0;
    out.timestamp = (uint32_t)(d_->time * 1e6);
    return out;
}

// -----------------------------------------------------------------------------
// baro — pressure-altitude = −pos_z; ISA pressure from altitude.
// baro — 気圧高度 = −pos_z、高度から ISA 気圧。
// -----------------------------------------------------------------------------
sf::BaroData Plant::baro() const
{
    Truth t = truth();
    float alt = -t.pos_ned.z;
    // N2 observation noise on the altitude [m], added BEFORE the ISA pressure mapping so
    // it propagates through the BMP280 pressure encode/decode to the firmware. No-op
    // unless N2 obs noise is on. (Shipping config has USE_BAROMETER=false, so the ESKF
    // does not fuse baro — the noise is faithful but currently affects telemetry only.)
    // N2 観測ノイズを高度[m]に付与（ISA 気圧変換の前＝BMP280 の気圧エンコード/デコード経由で
    // ファームへ伝播）。出荷 config は USE_BAROMETER=false ゆえ ESKF は baro 未融合（ノイズは
    // 忠実だが現状テレメトリのみに影響）。
    noise_.applyBaro(alt);

    sf::BaroData out{};
    out.altitude = alt;
    out.pressure = 101325.0f * std::pow(1.0f - 2.25577e-5f * alt, 5.25588f);
    out.temperature = 25.0f;
    out.timestamp = (uint32_t)(d_->time * 1e6);
    return out;
}

// -----------------------------------------------------------------------------
// flow — PMW3901 optical flow, raw counts, NO body remap (dx=fwd, dy=right).
//
// The plant must be the exact inverse of the ESKF's rotation removal
// (eskf_core.cpp:535-536): the ESKF computes trans_x = flow_x − gyro_y and
// trans_y = flow_y + gyro_x, so synthesis ADDS the opposite-signed gyro term:
//   dx ∝ (vx_body/height + omega_frd.y) ,  dy ∝ (vy_body/height − omega_frd.x).
//
// flow — PMW3901 オプティカルフロー、生カウント、機体 remap なし（dx=前, dy=右）。
// ESKF の回転除去（eskf_core.cpp:535-536）の厳密な逆: ESKF は
// trans_x = flow_x − gyro_y, trans_y = flow_y + gyro_x。よって合成は逆符号の
// ジャイロ項を足す: dx ∝ (vx/h + ωy), dy ∝ (vy/h − ωx)。
// -----------------------------------------------------------------------------
sf::FlowData Plant::flow() const
{
    Truth t = truth();
    Vec3 v_body = t.q_nb.inv_rotate(t.vel_ned);   // NED velocity → body FRD
    float height = -t.pos_ned.z;
    if (height < 1e-3f) height = 1e-3f;

    float rpp = cfg_.flow_rad_per_pixel;
    float fdx = (v_body.x / height + t.omega_frd.y) * last_dt_ / rpp;
    float fdy = (v_body.y / height - t.omega_frd.x) * last_dt_ / rpp;

    sf::FlowData out{};
    out.dx = (int16_t)std::lround(fdx);
    out.dy = (int16_t)std::lround(fdy);
    out.squal = 100;  // strong surface / 良好な表面
    out.timestamp = (uint32_t)(d_->time * 1e6);
    return out;
}

// -----------------------------------------------------------------------------
// mag — body magnetic field from a fixed NED reference (default OFF in the ESKF).
// mag — 固定 NED 基準磁場から機体磁場（ESKF では既定 OFF）。
// -----------------------------------------------------------------------------
sf::MagData Plant::mag() const
{
    Truth t = truth();
    Vec3 ref_ned{20.0f, 0.0f, 40.0f};        // reference field NED [µT]
    Vec3 b = t.q_nb.inv_rotate(ref_ned);

    sf::MagData out{};
    out.mag[0] = b.x; out.mag[1] = b.y; out.mag[2] = b.z;
    out.timestamp = (uint32_t)(d_->time * 1e6);
    return out;
}

}  // namespace sil
