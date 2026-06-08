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
        /// Real-world thrust efficiency vs the IDEALIZED curve above (motor/prop losses
        /// + battery sag the firmware's vbat reading doesn't capture). The firmware's
        /// HOVER_THRUST_CORRECTION = 1.12 (config.hpp:533, measured from flight-log
        /// throttle) compensates exactly this deficit, so it commands mass·g·1.12 for
        /// hover. WITHOUT this factor the Plant uses the loss-free datasheet curve and
        /// produces 1.12× the firmware's intended thrust → the hover command over-thrusts
        /// (~1 m/s² climb), drifts past the ToF ceiling, the ESKF goes blind and the craft
        /// never holds. With efficiency = 1/1.12 the firmware's hover command yields actual
        /// mg, matching the real hardware the firmware flew on (Model Identity).
        /// 実機の推力効率（理想曲線比、モータ/プロップ損失＋ファームが測れない電池サグ）。
        /// ファームの HOVER_THRUST_CORRECTION=1.12 がこの欠損を補償するため hover に
        /// mass·g·1.12 を指令する。この係数が無いと Plant は損失ゼロの理想曲線で 1.12 倍の
        /// 過推力を出し hover 指令が暴走上昇する。1/1.12 で実機同様に hover 指令＝mg となる。
        float thrust_efficiency = 1.0f / 1.12f;  ///< ≈0.893 (matches firmware HOVER_THRUST_CORRECTION)
        float kappa    = 9.71e-3f;  ///< Cq/Ct [m]  (reaction torque Q = kappa·T)
        float motor_tau = 0.02f;    ///< first-order motor lag time constant [s]

        // --- Battery (1S LiPo) sag/discharge model — Model fidelity ---
        // When enabled, the supply voltage is DYNAMIC: v_batt(t) = OCV(SoC) − I·R_int,
        // with the battery current I summed from each motor's electrical draw
        // I_i = (V_motor,i − Km·ω_i)/Rm (motor electrical model, params doc §3) plus a
        // quiescent avionics draw. SoC is depleted by Coulomb counting. DISABLED by
        // default → constant v_batt (the physics/smoke tests probe the motor curve at a
        // fixed nominal voltage). The closed-loop emulator (emu_vehicle_new) enables it
        // so the firmware's live-voltage thrust→duty compensation has a real sag to track.
        // 電池(1S LiPo)サグ/放電モデル。有効時は電源電圧が動的: v_batt=OCV(SoC)−I·R_int。
        // 電池電流 I は各モータ電気電流 I_i=(V_motor,i−Km·ω_i)/Rm（params文書§3）＋アビオ静止
        // 電流の総和。SoC はクーロンカウントで減少。既定 OFF（物理/smoke テストは固定公称で
        // モータ曲線を検証）。閉ループ emu が有効化し、ファームの実電圧 thrust→duty 補償に
        // 追従すべき実サグを与える。
        bool  batt_model_enable  = false;
        float batt_capacity_mah  = 300.0f;   ///< 1S LiPo capacity [mAh] (juida2026 EVIDENCE)
        float batt_r_int         = 0.1f;     ///< battery internal resistance [Ω] (vpython model)
        float batt_full_v        = 4.2f;     ///< OCV at full charge [V]
        float batt_empty_v       = 3.3f;     ///< OCV at empty [V]
        float batt_initial_frac  = 1.0f;     ///< initial state of charge [0..1] (1 = full)
        float motor_Rm           = 0.34f;    ///< winding resistance [Ω] (params doc, LCR)
        float motor_Km           = 6.12e-4f; ///< back-EMF constant [V/(rad/s)] (params doc)
        float avionics_current_a = 0.1f;     ///< MCU + sensors quiescent draw [A]
        float mass     = 0.037f;    ///< body mass [kg]
        float g        = 9.81f;     ///< gravity [m/s²]
        float health[4] = {1.0f, 1.0f, 1.0f, 1.0f}; ///< per-motor thrust gain (1=healthy)
        sf::math::Vec3 wind_force_ned = {0.0f, 0.0f, 0.0f}; ///< external wind force NED [N]
        /// Deterministic RAW IMU bias (body FRD), modeling the pre-calibration MEMS
        /// offset. Added as a constant to the synthetic IMU output. Default zero. The
        /// firmware boot calibration measures and removes it (P2-3 contrast test).
        /// 決定論的な生 IMU バイアス（機体 FRD）。校正前の MEMS オフセットを模擬。合成 IMU
        /// 出力に定数として加算。既定ゼロ。ファームの起動校正が測定・除去する（P2-3 対照試験）。
        sf::math::Vec3 imu_bias_accel = {0.0f, 0.0f, 0.0f}; ///< accel raw bias [m/s²]
        sf::math::Vec3 imu_bias_gyro  = {0.0f, 0.0f, 0.0f}; ///< gyro  raw bias [rad/s]
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

    /// Set per-motor thrust health gain at runtime (1=healthy, 0=dead). Used by the
    /// P7 disturbance scenarios to degrade a motor mid-flight and watch the mixer
    /// compensate. Out-of-range motor index is ignored; gain is clamped to [0,1].
    /// 各モータの推力健全度ゲインを実行時設定（1=正常, 0=停止）。P7 外乱シナリオが飛行中に
    /// モータを劣化させ、ミキサーの補償を見るのに使う。範囲外 index は無視、gain は [0,1] にクランプ。
    void setHealth(int motor, float gain);

    /// Set the deterministic raw IMU bias (body FRD) at runtime — a constant added to
    /// the synthetic accel [m/s²] / gyro [rad/s]. Models a pre-calibration MEMS offset
    /// so the firmware boot calibration has something real to measure and remove.
    /// 決定論的な生 IMU バイアス（機体 FRD）を実行時設定 — 合成 accel[m/s²]/gyro[rad/s] に
    /// 定数加算。校正前 MEMS オフセットを模擬し、ファーム起動校正に測定・除去対象を与える。
    void setImuBias(const sf::math::Vec3& accel_bias, const sf::math::Vec3& gyro_bias);

    /// Place the body at rest, level, at height z [m] (MuJoCo ENU up). Used to
    /// start a flight on the ground (z ≈ box half-height) for a takeoff demo.
    /// 機体を高さ z [m]（MuJoCo ENU 上）で水平・静止配置する。地上から離陸する
    /// デモのため、地上スタート（z ≈ 箱の半分の高さ）に使う。
    void setStartHeight(float z);

    /// Begin a PHYSICAL HANDLING maneuver: a hand lifts the (crashed) craft, rights it
    /// to level, carries it to a placement spot and sets it back down on the ground —
    /// the SIL stand-in for "pick the drone up and put it back" of the re-fly workflow.
    /// While it runs the Plant PRESCRIBES a smooth kinematic trajectory (NO dynamics, NO
    /// teleport): the body pose moves continuously through three smootherstep phases —
    ///   lift  (rise straight up from the crash pose to carry_alt, start righting),
    ///   carry (translate to place_x/place_y at carry_alt, finish righting to level),
    ///   place (lower straight down onto the ground at place_x/place_y, stay level).
    /// The synthetic IMU specific force is computed ANALYTICALLY from the trajectory's
    /// acceleration (frames::accel_body_frd) and the gyro from its angular velocity, so
    /// the firmware sees a physically consistent lift/right/carry/place — the ToF rises
    /// above the airborne threshold (→ IDLE_HELD) and drops back on placement (→
    /// IDLE_GROUND → re-calibration). Smootherstep gives zero velocity AND acceleration
    /// at every phase boundary (C²), so the path — and the IMU — has no discontinuity.
    /// place_x/place_y are NED [m]; carry_alt is height [m] up; the three durations [s]
    /// each must be > 0. Capturing the start pose from truth() means an inverted/tilted
    /// crash is righted by the same SLERP-to-level.
    ///
    /// 物理ハンドリング動作を開始: 手が（墜落した）機体を持上げ→正立へ起こし→設置点へ運搬→
    /// 地面に戻す ― 再飛行ワークフローの「ドローンを拾って置き直す」の SIL 代理。実行中は Plant が
    /// 滑らかなキネマティック軌道を規定する（動力学なし・teleport なし）: 機体姿勢が3つの
    /// smootherstep 相を連続に動く ― lift（墜落姿勢から carry_alt まで真上に上げ、起こし開始）、
    /// carry（carry_alt で place_x/place_y へ並進、水平へ起こし完了）、place（place_x/place_y で
    /// 真下に地面へ降ろし、水平維持）。合成 IMU 比力は軌道の加速度から解析的に（frames::
    /// accel_body_frd）、ジャイロは角速度から計算するので、ファームは物理的に整合した
    /// 持上げ/正立/運搬/設置を見る ― ToF が空中閾値を超え（→IDLE_HELD）、設置で戻る（→IDLE_GROUND
    /// →再校正）。smootherstep は各相境界で速度と加速度が共にゼロ（C²）ゆえ軌道（と IMU）に
    /// 不連続が無い。place_x/place_y は NED[m]、carry_alt は高さ[m]、3つの所要時間[s]は各 >0。
    /// 開始姿勢を truth() から捕える＝反転/傾いた墜落も同じ「水平への SLERP」で起こされる。
    void startHandling(float carry_alt_m, float place_x_ned, float place_y_ned,
                       float lift_s, float carry_s, float place_s);

    /// True while a handling maneuver is in progress (the kinematic override is active
    /// and the synthetic IMU is the analytic handling value, not the MuJoCo dynamics).
    /// ハンドリング動作の実行中に true（キネマティック上書きが有効で、合成 IMU は MuJoCo
    /// 動力学でなく解析ハンドリング値）。
    bool handlingActive() const { return handling_active_; }

    /// Per-motor duty [0,1] that produces hover thrust (mg/4) via the motor curve.
    /// Inverts the model: T = mg/4 → ω = √(T/Ct) → V = Am·ω²+Bm·ω+Cm → duty = V/v_batt.
    /// モータ曲線でホバー推力（mg/4）を出す各モータ duty[0,1]。モデルを逆算する。
    float hoverDuty() const;

    /// Convert a single motor duty [0,1] to steady-state thrust [N] via the curve.
    /// 1モータの duty[0,1] を曲線で定常推力 [N] に変換する。
    float dutyToThrust(float duty) const;

    /// Battery terminal voltage [V] as the INA3221 power monitor would measure it.
    /// With the battery model OFF this is the fixed nominal (cfg_.v_batt); with it
    /// ON it is the dynamic terminal voltage OCV(SoC) − I·R_int updated each substep.
    /// The motor curve (duty→thrust) and the firmware (thrust→duty) both use this
    /// value, so the closed loop stays consistent (Model Identity).
    /// INA3221 電源モニタが測る電池端子電圧 [V]。電池モデル OFF なら固定公称（cfg_.v_batt）、
    /// ON なら substep ごとに更新される動的端子電圧 OCV(SoC)−I·R_int。モータ曲線(duty→推力)
    /// とファーム(推力→duty)が同じ値を使うので閉ループが整合（Model Identity）。
    float batteryVoltage() const { return v_batt_; }

    /// Advance the physics by dt of VIRTUAL time, integrating in fixed
    /// model-timestep (m_->opt.timestep) substeps with a carried remainder, so
    /// the physics clock tracks the caller's virtual clock 1:1. mj_step always
    /// advances exactly one model timestep (it takes no dt), and the controller
    /// samples at 400 Hz while the plant integrates at the finer model timestep
    /// (≥10× oversampling) — the inter-sample plant dynamics are preserved.
    /// 物理を「仮想時間 dt」ぶん進める。固定モデル timestep（m_->opt.timestep）刻みの
    /// substep で積分し端数を繰り越す → 物理時計が呼び出し側の仮想時計と 1:1 で一致。
    /// mj_step は常にモデル timestep を1回進める（dt を取らない）。制御は 400Hz で
    /// サンプルし、プラントはより細かいモデル timestep（10倍以上）で積分する。
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
    /// One fixed-timestep physics substep of length h [s]: motor lag → thrust →
    /// reaction torque + wind → mj_step → advance noise. h is the model timestep.
    /// 長さ h [s] の固定刻み物理 substep 1回: モータ遅れ → 推力 → 反トルク＋風 →
    /// mj_step → ノイズ前進。h はモデル timestep。
    void substep(float h);

    /// One fixed-timestep step of the PRESCRIBED handling trajectory (length h [s]):
    /// advance the handling clock, evaluate the smootherstep pose/velocity/acceleration,
    /// write qpos/qvel into MuJoCo, mj_forward to refresh the position sensors, and store
    /// the analytic IMU specific force + gyro. Replaces substep() while handling is active
    /// (no dynamics integration). 規定ハンドリング軌道の固定刻み 1 ステップ（長さ h[s]）:
    /// ハンドリング時計を進め、smootherstep の姿勢/速度/加速度を評価し、qpos/qvel を MuJoCo に
    /// 書込み、mj_forward で位置センサを更新、解析 IMU 比力＋ジャイロを保存。ハンドリング中は
    /// substep() を置き換える（動力学積分なし）。
    void handlingSubstep(float h);

    /// Motor curve inverse: terminal motor voltage V → steady-state prop speed ω
    /// (positive root of Am·ω² + Bm·ω + (Cm − V) = 0; 0 if V ≤ Cm).
    /// モータ曲線の逆: 端子電圧 V → 定常回転 ω（Am·ω²+Bm·ω+(Cm−V)=0 の正根、V≤Cm なら0）。
    float solveOmega(float V) const;

    /// Open-circuit voltage [V] from remaining charge [mAh] (1S LiPo discharge
    /// curve, ported from simulator/vpython/sensors/power_monitor.py).
    /// 残容量 [mAh] → 開回路電圧 [V]（1S LiPo 放電曲線、vpython 実装から移植）。
    float ocvFromCharge(float charge_mah) const;

    /// Update the battery: Coulomb-count the charge and recompute the terminal
    /// voltage v_batt_ = OCV(SoC) − I·R_int. No-op when the model is disabled.
    /// 電池更新: クーロンカウントで容量を減らし端子電圧 v_batt_=OCV(SoC)−I·R_int を再計算。
    /// モデル無効時は何もしない。
    void updateBattery(float i_total_a, float h);

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
    double step_accum_ = 0.0;  ///< virtual-time accumulator [s] for fixed substeps

    float v_batt_ = 3.7f;          ///< current supply (terminal) voltage [V]
    float batt_charge_mah_ = 0.0f; ///< remaining battery charge [mAh] (model on)

    // --- Physical handling maneuver (kinematic override) state ---
    // 物理ハンドリング動作（キネマティック上書き）の状態。
    bool  handling_active_ = false;     ///< kinematic override in progress
    float ground_rest_z_enu_ = 0.013f;  ///< body rest height [m] ENU (set by setStartHeight)
    float handle_t_ = 0.0f;             ///< elapsed handling time [s]
    float handle_lift_s_  = 0.0f;       ///< lift  phase duration [s]
    float handle_carry_s_ = 0.0f;       ///< carry phase duration [s]
    float handle_place_s_ = 0.0f;       ///< place phase duration [s]
    sf::math::Vec3 handle_p0_ned_{};    ///< captured crash position NED [m]
    float handle_carry_z_ned_ = 0.0f;   ///< carry altitude as NED z [m] (= −carry_alt)
    float handle_place_x_ned_ = 0.0f;   ///< placement target x NED [m]
    float handle_place_y_ned_ = 0.0f;   ///< placement target y NED [m]
    float handle_ground_z_ned_ = 0.0f;  ///< ground placement NED z [m] (= −ground_rest)
    sf::math::Quat  handle_q0_nb_{};     ///< captured crash attitude (body→NED)
    sf::math::Vec3  handle_rv_flip_frd_{};  ///< right-to-level rot-vec (FRD, θ·n̂)
    sf::math::Vec3  handle_accel_frd_{0.0f, 0.0f, -9.81f};  ///< analytic specific force FRD
    sf::math::Vec3  handle_gyro_frd_{};  ///< analytic body angular rate FRD

    int body_id_  = -1;
    int accel_sid_ = -1, gyro_sid_ = -1, quat_sid_ = -1, pos_sid_ = -1, vel_sid_ = -1;
};

}  // namespace sil
