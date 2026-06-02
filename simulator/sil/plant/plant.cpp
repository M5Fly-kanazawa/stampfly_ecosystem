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
    for (int i = 0; i < 4; ++i) motor_duty_[i] = motor_target_[i];
}

void Plant::setWind(const sf::math::Vec3& force_ned)
{
    cfg_.wind_force_ned = force_ned;
}

void Plant::setStartHeight(float z)
{
    // Free-joint state: position (0,0,z) ENU, level orientation, zero velocity.
    // フリージョイント状態: 位置(0,0,z) ENU・水平姿勢・速度ゼロ。
    d_->qpos[0] = 0.0; d_->qpos[1] = 0.0; d_->qpos[2] = z;
    d_->qpos[3] = 1.0; d_->qpos[4] = 0.0; d_->qpos[5] = 0.0; d_->qpos[6] = 0.0;
    for (int i = 0; i < 6; ++i) d_->qvel[i] = 0.0;
    mj_forward(m_, d_);
}

// -----------------------------------------------------------------------------
// dutyToThrust — normalized duty [0,1] → steady-state thrust [N] via the real
// motor + propeller curve (docs/architecture/stampfly-parameters.md §3):
//   V = duty·v_batt ; solve V = Am·ω² + Bm·ω + Cm for ω ; T = Ct·ω².
// dutyToThrust — 正規化 duty[0,1] → 実モータ＋プロペラ曲線で定常推力 [N]。
// -----------------------------------------------------------------------------
float Plant::dutyToThrust(float duty) const
{
    if (duty <= 0.0f) return 0.0f;
    const float V = duty * cfg_.v_batt;            // terminal voltage [V]
    if (V <= cfg_.motor_Cm) return 0.0f;           // below the offset → no spin

    // Positive root of Am·ω² + Bm·ω + (Cm − V) = 0.
    // Am·ω² + Bm·ω + (Cm − V) = 0 の正の根。
    const float a = cfg_.motor_Am;
    const float b = cfg_.motor_Bm;
    const float disc = b * b + 4.0f * a * (V - cfg_.motor_Cm);  // > 0 since V > Cm
    const float omega = (-b + std::sqrt(disc)) / (2.0f * a);
    if (omega <= 0.0f) return 0.0f;
    // T = Ct·ω², scaled by the real-world thrust efficiency (see Config::thrust_efficiency:
    // the firmware's HOVER_THRUST_CORRECTION 1.12 compensates this deficit on real hw).
    // T = Ct·ω² に実機推力効率を乗ずる（ファームの 1.12 補償が打ち消す実機の欠損）。
    return cfg_.thrust_efficiency * cfg_.Ct * omega * omega;   // thrust T [N]
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
    return V / cfg_.v_batt;                                // duty = V / v_batt
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
        substep((float)h);
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
    // 指令目標への一次遅れ、続いて二次推力。
    const float alpha = 1.0f - std::exp(-h / cfg_.motor_tau);
    float thrust[4];
    for (int i = 0; i < 4; ++i) {
        motor_duty_[i] += (motor_target_[i] - motor_duty_[i]) * alpha;
        thrust[i] = dutyToThrust(motor_duty_[i]) * cfg_.health[i];
        if (i < m_->nu) d_->ctrl[i] = thrust[i];
    }

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
    Vec3 torque_world = q_mj.rotate({0.0f, 0.0f, tau_yaw_flu_z});

    // Wind force NED → world ENU.
    // 風力 NED → 世界 ENU。
    Vec3 wind_enu = frames::ned_to_enu(cfg_.wind_force_ned);

    // xfrc_applied[6·body + 0..5] = [Fx,Fy,Fz, Tx,Ty,Tz] in the world frame.
    // MuJoCo does NOT auto-clear it, so overwrite every step.
    // xfrc_applied[6·body + 0..5] = 世界座標の [Fx,Fy,Fz, Tx,Ty,Tz]。MuJoCo は自動で
    // クリアしないので毎ステップ上書きする。
    double* f = d_->xfrc_applied + 6 * body_id_;
    f[0] = wind_enu.x;     f[1] = wind_enu.y;     f[2] = wind_enu.z;
    f[3] = torque_world.x; f[4] = torque_world.y; f[5] = torque_world.z;

    mj_step(m_, d_);

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
    const double* a = sensor(accel_sid_);  // FLU specific force (a − g)
    const double* w = sensor(gyro_sid_);   // FLU body angular rate

    Vec3 accel_frd = frames::flu_to_frd({(float)a[0], (float)a[1], (float)a[2]});
    Vec3 gyro_frd  = frames::gyro_body_frd({(float)w[0], (float)w[1], (float)w[2]});

    sf::ImuData out{};
    out.accel[0] = accel_frd.x; out.accel[1] = accel_frd.y; out.accel[2] = accel_frd.z;
    out.gyro[0]  = gyro_frd.x;  out.gyro[1]  = gyro_frd.y;  out.gyro[2]  = gyro_frd.z;
    // Layer the N0 sensor noise (bias + white) on the clean sample. No-op if disabled
    // → the clean path stays byte-identical. RESET_PLAN §13 P5.
    // クリーンサンプルに N0 ノイズ（バイアス＋白色）を載せる。無効時は無操作＝クリーン経路は不変。
    noise_.applyAccel(out.accel);
    noise_.applyGyro(out.gyro);
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
    out.status = 0;
    out.valid = (dist > 0.0f && dist < 4.0f);  // VL53L3CX range gate
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
