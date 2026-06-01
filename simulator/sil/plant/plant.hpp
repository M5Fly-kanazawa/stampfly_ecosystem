/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file plant.hpp
 * @brief SIL physics plant — MuJoCo rigid body + self-written motor/sensor/wind.
 *        SIL 物理プラント — MuJoCo 剛体＋自作のモータ/センサ/風。
 *
 * This is the algorithm-independent SIL bench (RESET_PLAN §5 ①). It owns the
 * MuJoCo model, turns per-motor duty (from the firmware's actuator_motor topic)
 * into thrust + yaw reaction torque + wind, steps the physics, and synthesizes
 * the StampFly sensors (IMU/ToF/baro/flow/mag) in body-FRD via the frames module.
 *
 * これはアルゴリズム非依存の SIL ベンチ（RESET_PLAN §5 ①）。MuJoCo モデルを所有し、
 * 各モータ duty（ファームの actuator_motor トピック由来）を推力＋ヨー反トルク＋風に
 * 変換して物理を進め、StampFly センサ（IMU/ToF/baro/flow/mag）を frames 経由で
 * 機体 FRD に合成する。
 *
 * Design philosophy (RESET_PLAN §6): the motor/sensor models are SELF-WRITTEN;
 * MuJoCo's built-in <accelerometer>/<gyro> are used as the synthesis basis and as
 * a trusted answer-check. Coordinate transforms happen ONLY in frames.hpp.
 *
 * 設計方針（RESET_PLAN §6）: モータ/センサモデルは自作。MuJoCo 内蔵の
 * <accelerometer>/<gyro> を合成の土台かつ信頼できる答え合わせに使う。座標変換は
 * frames.hpp の中だけで行う。
 *
 * @design simulator/sil/docs/coordinate_frames.md §4.1-4.5                 [OK]
 * @design simulator/sil/RESET_PLAN.md §5-6 — algorithm-independent bench   [--]
 */

#pragma once

#include <cstdint>

#include <mujoco/mujoco.h>

#include "sf_math.hpp"
#include "frames.hpp"
#include "data_types.hpp"
#include "sensor_noise.hpp"

namespace sil {

/// SIL physics plant: MuJoCo + self-written motor/sensor/wind models.
/// SIL 物理プラント: MuJoCo ＋自作のモータ/センサ/風モデル。
class Plant {
public:
    /// Tunable plant parameters (StampFly physical values + plant-only models).
    ///
    /// Motor + propeller model derived from the documented electrical/mechanical
    /// parameters (docs/architecture/stampfly-parameters.md §3), NOT a fitted
    /// k·duty². The per-motor command is a normalized duty in [0,1] (what the
    /// firmware mixer outputs); the duty is converted to thrust through the real
    /// motor curve: V = duty·v_batt, solve V = Am·ω² + Bm·ω + Cm for the prop
    /// speed ω, then thrust T = Ct·ω² and reaction torque Q = kappa·T.
    ///
    /// モータ＋プロペラモデルは文書の電気・機械パラメータ
    /// （docs/architecture/stampfly-parameters.md §3）から導出（フィットの k·duty²
    /// ではない）。各モータ指令は正規化 duty[0,1]（ファームのミキサー出力）。duty を
    /// 実モータ曲線で推力に変換: V = duty·v_batt、V = Am·ω² + Bm·ω + Cm を ω について
    /// 解き、推力 T = Ct·ω²、反トルク Q = kappa·T。
    struct Config {
        float v_batt   = 3.7f;      ///< 1S LiPo nominal terminal voltage [V]
        float motor_Am = 5.39e-8f;  ///< V/(rad/s)²  (V = Am·ω² + Bm·ω + Cm)
        float motor_Bm = 6.33e-4f;  ///< V/(rad/s)
        float motor_Cm = 1.53e-2f;  ///< V (offset)
        float Ct       = 1.00e-8f;  ///< thrust coeff N/(rad/s)²  (T = Ct·ω²)
        float kappa    = 9.71e-3f;  ///< Cq/Ct [m]  (reaction torque Q = kappa·T)
        float motor_tau = 0.02f;    ///< first-order motor lag time constant [s]
        float mass     = 0.037f;    ///< body mass [kg]
        float g        = 9.81f;     ///< gravity [m/s²]
        float health[4] = {1.0f, 1.0f, 1.0f, 1.0f}; ///< per-motor thrust gain (1=healthy)
        sf::math::Vec3 wind_force_ned = {0.0f, 0.0f, 0.0f}; ///< external wind force NED [N]
        float flow_rad_per_pixel = 0.00222f; ///< PMW3901 rad per count (matches ESKF)
        SensorNoise::Config noise; ///< IMU sensor noise (N0, default OFF). RESET_PLAN §13
    };

    /// Ground-truth state in StampFly conventions (NED world, FRD body).
    /// StampFly 規約での真値状態（NED 世界・FRD 機体）。
    struct Truth {
        sf::math::Vec3 pos_ned;    ///< position NED [m]
        sf::math::Vec3 vel_ned;    ///< velocity NED [m/s]
        sf::math::Vec3 omega_frd;  ///< body angular rate FRD [rad/s]
        sf::math::Quat q_nb;       ///< attitude body→NED
    };

    Plant() = default;
    ~Plant();
    Plant(const Plant&) = delete;
    Plant& operator=(const Plant&) = delete;

    /// Load the MJCF model and cache sensor/body ids. Returns false on failure.
    /// MJCF モデルを読み、センサ/ボディ id をキャッシュ。失敗時 false。
    bool init(const char* model_path, const Config& cfg);

    /// Convenience overload using the default Config. / 既定 Config を使う簡易版。
    bool init(const char* model_path) { return init(model_path, Config{}); }

    /// Set the commanded per-motor duty target (index 0..3 = M1FR/M2RR/M3RL/M4FL,
    /// matching MotorOutput.duty and the MJCF rotor order, 1:1, no reshuffle).
    /// 指令の各モータ duty 目標を設定（添字 0..3 = M1FR/M2RR/M3RL/M4FL、MotorOutput.duty
    /// と MJCF ロータ順に 1:1 対応、並べ替えなし）。
    void setDuty(const sf::MotorOutput& cmd);

    /// Snap actual motor speed to the commanded target (skip spool-up lag). Used to
    /// warm-start a test at hover so the first-order lag does not cause an initial dip.
    /// 実モータ回転を指令目標に即時一致させる（スプールアップ遅れを飛ばす）。ホバーから
    /// テストを暖機起動し、一次遅れによる初期沈下を避けるために使う。
    void primeMotors();

    /// Set the external wind force in NED [N] (default zero).
    /// NED の外乱風力 [N] を設定（既定ゼロ）。
    void setWind(const sf::math::Vec3& force_ned);

    /// Place the body at rest, level, at height z [m] (MuJoCo ENU up). Used to
    /// start a flight on the ground (z ≈ box half-height) for a takeoff demo.
    /// 機体を高さ z [m]（MuJoCo ENU 上）で水平・静止配置する。地上から離陸する
    /// デモのため、地上スタート（z ≈ 箱の半分の高さ）に使う。
    void setStartHeight(float z);

    /// Per-motor duty [0,1] that produces hover thrust (mg/4) via the motor curve.
    /// Inverts the model: T = mg/4 → ω = √(T/Ct) → V = Am·ω²+Bm·ω+Cm → duty = V/v_batt.
    /// モータ曲線でホバー推力（mg/4）を出す各モータ duty[0,1]。モデルを逆算する。
    float hoverDuty() const;

    /// Convert a single motor duty [0,1] to steady-state thrust [N] via the curve.
    /// 1モータの duty[0,1] を曲線で定常推力 [N] に変換する。
    float dutyToThrust(float duty) const;

    /// Battery terminal voltage [V] as the INA3221 power monitor would measure it.
    /// In SIL the "battery" is the Plant supply (fixed nominal today; a future
    /// sag/discharge model would surface here automatically). The firmware's
    /// thrust→duty stage divides by this, so it must match the v_batt the motor
    /// curve uses for the closed loop to be consistent.
    /// INA3221 電源モニタが測る電池端子電圧 [V]。SIL では「電池」= Plant 電源
    /// （今は固定公称値、将来の電圧降下モデルがあればここに現れる）。ファームの
    /// thrust→duty 段はこれで割るため、モータ曲線が使う v_batt と一致させる。
    float batteryVoltage() const { return cfg_.v_batt; }

    /// Advance the physics by dt: motor lag → thrust → reaction torque + wind → mj_step.
    /// 物理を dt 進める: モータ遅れ → 推力 → 反トルク＋風 → mj_step。
    void step(float dt);

    /// Ground-truth state (NED/FRD), converted from MuJoCo via frames only.
    /// 真値状態（NED/FRD）。MuJoCo から frames だけで変換。
    Truth truth() const;

    // Synthetic sensors (body-FRD, driver-normalized convention). 合成センサ。
    sf::ImuData  imu()  const;  ///< accel [m/s²] (−9.8 at rest) + gyro [rad/s], FRD
    sf::TofData  tof()  const;  ///< downward distance [m]
    sf::BaroData baro() const;  ///< pressure-altitude [m] (= −pos_z)
    sf::FlowData flow() const;  ///< PMW3901 dx/dy [counts] over the last step
    sf::MagData  mag()  const;  ///< body magnetic field [µT] (default ref, OFF by default)

    const mjModel* model() const { return m_; }  ///< for the optional viewer / ビューア用
    mjData*        data()        { return d_; }   ///< for the optional viewer / ビューア用

private:
    /// Pointer to a named sensor's data inside d_->sensordata.
    /// 名前付きセンサの d_->sensordata 内の先頭ポインタ。
    const double* sensor(int sid) const { return d_->sensordata + m_->sensor_adr[sid]; }

    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;
    Config cfg_;
    SensorNoise noise_;  ///< IMU noise model (seeded; advanced each step). §13 P5

    float motor_duty_[4]   = {0.0f, 0.0f, 0.0f, 0.0f}; ///< actual (lagged) duty
    float motor_target_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; ///< commanded target duty
    float last_dt_ = 0.0025f;  ///< last step dt (for flow count integration)

    int body_id_  = -1;
    int accel_sid_ = -1, gyro_sid_ = -1, quat_sid_ = -1, pos_sid_ = -1, vel_sid_ = -1;
};

}  // namespace sil
