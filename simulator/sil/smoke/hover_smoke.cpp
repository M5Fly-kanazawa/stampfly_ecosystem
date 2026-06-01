/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file hover_smoke.cpp
 * @brief P1.2(c)+ closed loop — a full flight in the SIL: takeoff → hover → yaw
 *        spin → yaw stop → landing. Gate G3 + the review-video bundle.
 *        SIL でのフル飛行: 離陸 → ホバ → ヨー旋回 → ヨー停止 → 着陸。ゲート G3 ＋
 *        レビュー動画バンドル。
 *
 * The real firmware loop (ImuTask → estimate_state → ControlTask → B^-1 mixer →
 * actuator_motor) runs on the host RTOS emulator at 400Hz; the MuJoCo Plant
 * closes the loop. The flight is a CLOSED-LOOP altitude schedule in ALT_HOLD:
 * the stick commands a climb/hold/descent rate and the cascade altitude loop
 * (alt → vertical-velocity → thrust) holds the captured altitude — so altitude
 * no longer sags while yawing (the mixer redistributes duty for yaw torque, and
 * the nonlinear thrust curve drops net lift; the loop catches it). The Plant's
 * barometer is injected on sensor_baro so the estimator has a non-drifting
 * altitude + vertical velocity. It runs identically for any estimator (selected
 * by estimator.type) — algorithm-independent (RESET_PLAN P2).
 *
 * 実ファームループをホスト RTOS エミュレータで 400Hz 稼働、MuJoCo Plant がループを
 * 閉じる。飛行は ALT_HOLD の閉ループ高度スケジュール: スティックは上昇/保持/下降の
 * レートを指令し、カスケード高度ループ（高度→鉛直速度→推力）が捕捉高度を保持する。
 * これでヨー旋回中も高度が下がらない（ミキサーがヨートルク用に duty を再配分し、非線形な
 * 推力曲線で正味揚力が落ちるのをループが補償）。Plant の気圧を sensor_baro に注入し、
 * 推定器にドリフトしない高度＋鉛直速度を与える。どの推定器でも同一に動く（P2）。
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "scheduler.hpp"
#include "topics.hpp"
#include "params.hpp"
#include "data_types.hpp"
#include "flight_state.hpp"
#include "config.hpp"
#include "plant.hpp"
#include "plant_bridge.hpp"

void ImuTask(void*);
void ControlTask(void*);
void StateTask(void*);
extern TaskHandle_t g_control_task_handle;

using sil::rtos::Scheduler;
using sf::math::Vec3;

namespace {
constexpr int64_t kSimDurationUs = 5600000;   // 5.6 s
constexpr float kGroundZ = 0.013f;            // body rest height on the ground (ENU up)
constexpr float kG2AttRmseMaxDeg = 6.0f;      // G2: estimate-vs-truth attitude RMSE bound

// Closed-loop ALT_HOLD schedule. The throttle stick commands a vertical RATE:
// climb_rate = (throttle − 0.5)·2·max_climb_rate(0.5), so throttle 0.9 → +0.4 m/s,
// 0.5 → hold, 0.1 → −0.4 m/s (|throttle−0.5|>deadzone(0.1) to move). The altitude
// loop holds the captured altitude when the stick is centered. On the ground and
// after touchdown the mode is STABILIZE with throttle 0 (motors idle). Phases [s].
// 閉ループ ALT_HOLD スケジュール。スロットルスティックは鉛直レートを指令:
// climb_rate=(throttle−0.5)·2·max_climb_rate(0.5)。0.9→+0.4 m/s、0.5→保持、0.1→−0.4 m/s
// （移動は |throttle−0.5|>不感帯(0.1)）。中央で高度ループが捕捉高度を保持。地上と着地後は
// STABILIZE・スロットル0（モータ待機）。
constexpr float T_GROUND = 0.4f;   // STABILIZE, sit on the ground / 地上待機
constexpr float T_CLIMB  = 1.6f;   // ALT_HOLD climb +0.4 m/s for 1.2 s → ~0.48 m
constexpr float T_HOVER  = 2.2f;   // ALT_HOLD hold / 高度保持
constexpr float T_YAW    = 3.4f;   // ALT_HOLD hold + yaw spin (1.2 s) / ヨー旋回
constexpr float T_STOP   = 4.0f;   // ALT_HOLD hold, yaw stop / ヨー停止
constexpr float T_LAND   = 5.1f;   // ALT_HOLD descend −0.4 m/s (1.1 s) / 着陸下降
                                   // after T_LAND: STABILIZE throttle 0, rest

constexpr float THR_CLIMB   = 0.9f;    // climb  +0.4 m/s (stick up)
constexpr float THR_HOLD    = 0.5f;    // hold captured altitude (stick centered)
constexpr float THR_DESCEND = 0.1f;    // descend −0.4 m/s (stick down)
constexpr float YAW_STICK   = 0.2f;    // · max_yaw_rate(5) = 1.0 rad/s

constexpr int64_t kBaroPeriodUs = 20000;  // inject barometer at 50 Hz (BMP280 rate)

sil::Plant g_plant;
int64_t g_last_step_us = 0;
int64_t g_last_baro_us = 0;
FILE* g_traj = nullptr;
int g_rec_count = 0;
constexpr int kRecDiv = 8;

// Metrics for the G3 verdict.
float g_max_alt = 0.0f;
float g_max_tilt = 0.0f;
float g_yaw_during_spin = 0.0f;   // peak |yaw rate| in the spin phase
float g_yaw_after_stop = 0.0f;    // |yaw rate| at the end of the stop phase
float g_final_alt = 0.0f;
float g_yaw_cmd_now = 0.0f;
// Altitude-hold band over the centered-stick window [T_HOVER, T_LAND): proves the
// altitude no longer sags while yawing. min/max truth altitude in that window.
// 中央スティック窓 [T_HOVER, T_LAND) の高度保持バンド: ヨー中に高度が下がらない証拠。
float g_hold_alt_min = 1e9f;
float g_hold_alt_max = -1e9f;

// G2 attitude-tracking accumulators (estimate vs truth, settled airborne window).
// G2 姿勢追従の累積（推定 vs 真値、整定後の飛行窓）。
double g_att_err2_sum = 0.0;   // Σ error² [deg²]
long   g_att_err_n    = 0;     // sample count

float tiltDeg(const sf::math::Quat& q_nb)
{
    // Guard a degenerate quaternion: at t=0 the estimator has not published yet, so
    // estimate_state holds the zero-initialized [0,0,0,0] — treat that as level (0°)
    // instead of the arccos(0)=90° artifact it would otherwise produce.
    // 退化クォータニオンのガード: t=0 は推定器が未発行で estimate_state が [0,0,0,0]。
    // arccos(0)=90° のアーティファクトを避け、水平(0°)として扱う。
    const float n2 = q_nb.w * q_nb.w + q_nb.x * q_nb.x +
                     q_nb.y * q_nb.y + q_nb.z * q_nb.z;
    if (n2 < 1e-6f) return 0.0f;

    float c = q_nb.inv_rotate(Vec3{0, 0, 1}).z;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * 180.0f / 3.14159265f;
}

// Signed roll & pitch [deg] from the attitude quaternion. Unlike the tilt magnitude
// (arccos, always ≥0, which folds near-zero estimation noise onto the positive side
// as pulses), roll/pitch keep their sign — the natural, physically meaningful
// attitude for the graph. Same degenerate-quaternion guard as tiltDeg (t=0 = level).
// 姿勢クォータニオンから符号付きロール・ピッチ[deg]。傾斜角の大きさ（arccos、常に≥0で
// ゼロ近傍ノイズを正側へ折り返してパルス化）と違い符号を保つ — グラフに自然で物理的に
// 意味のある姿勢。tiltDeg と同じ退化ガード（t=0 の推定は水平扱い）。
void rollPitchDeg(const sf::math::Quat& q_nb, float& roll_deg, float& pitch_deg)
{
    const float n2 = q_nb.w * q_nb.w + q_nb.x * q_nb.x +
                     q_nb.y * q_nb.y + q_nb.z * q_nb.z;
    if (n2 < 1e-6f) { roll_deg = 0.0f; pitch_deg = 0.0f; return; }
    const sf::math::Vec3 e = q_nb.to_euler();   // [rad] (x=roll, y=pitch, z=yaw)
    roll_deg  = e.x * (180.0f / 3.14159265f);
    pitch_deg = e.y * (180.0f / 3.14159265f);
}

// Flight schedule: throttle + yaw command + flight mode as a function of time.
// alt_hold=true selects ALT_HOLD (closed-loop altitude); false is STABILIZE with
// the motors idle (ground / touchdown).
// 飛行スケジュール: 時刻に対するスロットル＋ヨー指令＋フライトモード。
// alt_hold=true で ALT_HOLD（閉ループ高度）、false で STABILIZE・モータ待機（地上/着地）。
void schedule(float t, float& throttle, float& yaw, bool& alt_hold)
{
    yaw = 0.0f;
    alt_hold = true;
    if (t < T_GROUND)      { throttle = 0.0f;        alt_hold = false; }  // ground
    else if (t < T_CLIMB)    throttle = THR_CLIMB;                        // takeoff
    else if (t < T_HOVER)    throttle = THR_HOLD;                         // settle to hold
    else if (t < T_YAW)    { throttle = THR_HOLD; yaw = YAW_STICK; }      // yaw spin
    else if (t < T_STOP)     throttle = THR_HOLD;                         // yaw stop
    else if (t < T_LAND)     throttle = THR_DESCEND;                      // descend
    else                   { throttle = 0.0f;        alt_hold = false; }  // touchdown
}

void physics(int64_t now_us)
{
    const float t = now_us * 1e-6f;
    float throttle, yaw;
    bool alt_hold;
    schedule(t, throttle, yaw, alt_hold);
    g_yaw_cmd_now = yaw * 5.0f;  // commanded yaw rate (· max_yaw_rate) for the graph

    // Arm + drive the firmware via topics (FLYING throughout). sub_mode selects the
    // controller cascade: ALT_HOLD for the closed-loop altitude flight, STABILIZE
    // (motors idle at throttle 0) while resting on the ground.
    // トピックでファームを駆動（終始 FLYING）。sub_mode が制御カスケードを選択:
    // 飛行中は ALT_HOLD（閉ループ高度）、地上待機は STABILIZE（スロットル0で待機）。
    sf::SystemMode mode = {};
    mode.state = static_cast<uint8_t>(sf::FlightState::FLYING);
    mode.sub_mode = static_cast<uint8_t>(
        alt_hold ? sf::FlightMode::ALT_HOLD : sf::FlightMode::STABILIZE);
    mode.armed = true;
    mode.timestamp = static_cast<uint32_t>(now_us);
    sf::system_mode.publish(mode);

    sf::CommandSetpoint sp = {};
    sp.throttle = throttle;
    sp.roll = sp.pitch = 0.0f;
    sp.yaw = yaw;
    sp.timestamp = static_cast<uint32_t>(now_us);
    sf::command_setpoint.publish(sp);

    // Apply the firmware's motor command, advance physics, feed back the IMU.
    // ファームのモータ指令を与え、物理を進め、IMU を返す。
    sf::MotorOutput cmd = sf::actuator_motor.latest();
    g_plant.setDuty(cmd);
    if (now_us > g_last_step_us) {
        g_plant.step(static_cast<float>(now_us - g_last_step_us) * 1e-6f);
        g_last_step_us = now_us;
    }
    sil::bridge::current_imu = g_plant.imu();

    // Inject the Plant barometer on sensor_baro at 50 Hz (the real baro_task rate).
    // ALT_HOLD needs a non-drifting altitude + vertical velocity; both estimators
    // get them from this (ESKF: Kalman update; complementary: blend). In the real
    // firmware baro_task publishes this; the SIL bench stands in for it.
    // Plant の気圧を sensor_baro に 50Hz（実 baro_task レート）で注入。ALT_HOLD には
    // ドリフトしない高度＋鉛直速度が要り、両推定器ともこれから得る。実機では baro_task が
    // publish する役を SIL ベンチが代行する。
    if (now_us - g_last_baro_us >= kBaroPeriodUs) {
        sf::sensor_baro.publish(g_plant.baro());
        g_last_baro_us = now_us;
    }

    sil::Plant::Truth tr = g_plant.truth();
    const float alt = -tr.pos_ned.z;
    const float tilt = tiltDeg(tr.q_nb);
    if (alt > g_max_alt) g_max_alt = alt;
    if (tilt > g_max_tilt) g_max_tilt = tilt;
    if (t >= T_YAW - 0.6f && t < T_YAW &&
        std::fabs(tr.omega_frd.z) > g_yaw_during_spin)
        g_yaw_during_spin = std::fabs(tr.omega_frd.z);
    if (t >= T_STOP - 0.1f && t < T_STOP) g_yaw_after_stop = std::fabs(tr.omega_frd.z);
    // Altitude-hold band over the centered-stick window (hover → yaw → stop), BEFORE
    // the commanded descent (T_STOP): this is where the old open-loop schedule sagged
    // while yawing. Start a bit after T_HOVER so the climb→hold settling is excluded.
    // 中央スティック窓（ホバ→ヨー→停止、指令下降 T_STOP の前）の高度バンド。旧開放ループは
    // ここで沈んだ。登り→保持の整定を除くため T_HOVER から少し後を窓の始点にする。
    if (t >= T_HOVER + 0.3f && t < T_STOP) {
        if (alt < g_hold_alt_min) g_hold_alt_min = alt;
        if (alt > g_hold_alt_max) g_hold_alt_max = alt;
    }
    g_final_alt = alt;

    // G2 — estimate tracking: angle between the estimated and true attitude
    // quaternions [deg], accumulated over the settled airborne window [1 s, T_LAND).
    // The startup gyro/accel bias is where ESKF (estimates BG/BA) pulls ahead of the
    // complementary filter (no bias state).
    // G2 — 推定追従: 推定姿勢と真値姿勢クォータニオン間の角度[deg]を整定後の飛行窓で累積。
    // 起動バイアスにより ESKF（BG/BA を推定）が相補（バイアス状態なし）より優位になる箇所。
    sf::StateEstimate est = sf::estimate_state.latest();
    sf::math::Quat qe(est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3]);
    const float qe_n2 = qe.w * qe.w + qe.x * qe.x + qe.y * qe.y + qe.z * qe.z;
    if (qe_n2 > 1e-6f && t >= 1.0f && t < T_LAND) {
        // Roll/pitch tracking error [deg] (gravity-observable attitude — the channel
        // the gyro/accel bias perturbs). Uses the same signed rollPitchDeg as the
        // recorder, so estimate and truth share one convention (yaw has no absolute
        // reference, so it is excluded).
        // ロール/ピッチ追従誤差[deg]（重力で観測できる姿勢＝バイアスが効く軸）。記録と同じ
        // 符号付き rollPitchDeg を使い推定と真値の規約を一致させる（ヨーは絶対基準が無く除外）。
        float r_t, p_t, r_e, p_e;
        rollPitchDeg(tr.q_nb, r_t, p_t);
        rollPitchDeg(qe, r_e, p_e);
        const double er = (double)r_e - r_t, ep = (double)p_e - p_t;
        g_att_err2_sum += er * er + ep * ep;
        g_att_err_n += 2;   // two components (roll, pitch)
    }

    if (g_traj != nullptr && (g_rec_count++ % kRecDiv) == 0) {
        const double* q = g_plant.data()->qpos;
        // Signed roll/pitch for truth and estimate (deg, 4 decimals — the angles are
        // ~0.1°, so coarse rounding would itself look like pulses).
        // 真値・推定の符号付きロール/ピッチ（deg、小数4桁 — 角度が ~0.1° なので粗い丸めは
        // それ自体がパルスに見える）。
        float roll, pitch, roll_e, pitch_e;
        rollPitchDeg(tr.q_nb, roll, pitch);
        rollPitchDeg(qe, roll_e, pitch_e);
        std::fprintf(g_traj,
                "%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t, q[0], q[1], q[2], q[3], q[4], q[5], q[6],
                alt, roll, pitch, tr.omega_frd.z, g_yaw_cmd_now,
                -est.position[2], roll_e, pitch_e,
                cmd.duty[0], cmd.duty[1], cmd.duty[2], cmd.duty[3]);
    }
}
}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    const char* out_dir = (argc > 2) ? argv[2] : nullptr;
    const int est_type = (argc > 3) ? std::atoi(argv[3]) : 0;  // 0=ESKF, 1=complementary
    const char* milestone = (argc > 4) ? argv[4] : "P1";       // review-bundle label
    const char* noise_lvl = (argc > 5) ? argv[5] : "off";      // "off" | "n0" (RESET_PLAN §13)
    const unsigned noise_seed =
        (argc > 6) ? (unsigned)std::strtoul(argv[6], nullptr, 10) : 12345u;
    const bool noise_on = (std::strcmp(noise_lvl, "off") != 0);

    if (out_dir != nullptr) {
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/trajectory.csv", out_dir);
        g_traj = std::fopen(path, "w");
        if (g_traj != nullptr) {
            std::fprintf(g_traj,
                "t,px,py,pz,qw,qx,qy,qz,alt,roll,pitch,yawrate,yawcmd,"
                "alt_est,roll_est,pitch_est,m0,m1,m2,m3\n");
        }
    }

    sf::params::init();
    sf::topics_init();

    // Algorithm-independence (RESET_PLAN P2): pick the estimator by a parameter
    // only; the bench, the flight, and the firmware tasks are unchanged.
    // アルゴリズム非依存（P2）: 推定器を param だけで選ぶ。ベンチ・飛行・ファームは不変。
    sf::params::set_int("estimator.type", est_type);
    printf("[flight] estimator.type=%d (%s)\n",
           est_type, est_type == 1 ? "complementary filter" : "ESKF");

    // SIL gain-sweep hook: override ALT_HOLD gains from the environment so we can
    // tune without recompiling, then bake the winners into params.cpp. No env set →
    // params.cpp defaults are used. (Temporary scaffolding for the P1 ALT_HOLD tune.)
    // SIL ゲインスイープ用フック: 環境変数で ALT_HOLD ゲインを上書きし、再コンパイルなしで
    // 調整 → 勝者を params.cpp に焼き込む。未設定なら params.cpp の既定値。（一時的）
    auto env_set = [](const char* env, const char* param) {
        const char* s = std::getenv(env);
        if (s != nullptr) {
            sf::params::set_float(param, std::atof(s));
            printf("[flight] override %s = %s\n", param, s);
        }
    };
    env_set("ALT_AK", "altitude.alt.kp");
    env_set("ALT_AT", "altitude.alt.ti");
    env_set("ALT_VK", "altitude.vel.kp");
    env_set("ALT_VT", "altitude.vel.ti");

    // Sensor suite for this bench: the harness injects the barometer (below), so the
    // ESKF altitude/velocity come from baro+IMU — the same channel the complementary
    // filter uses. ToF/flow are not injected here, so disable them (otherwise the
    // unobservable horizontal states drift on accel-only). This is a per-run SIL
    // config via params (like estimator.type); the firmware defaults are unchanged.
    // 本ベンチのセンサ構成: ハーネスが気圧を注入するので、ESKF の高度/速度も baro+IMU から
    // 得る（相補フィルタと同じ系統）。ToF/flow は注入しないので無効化（さもないと観測不能な
    // 水平状態が加速度のみでドリフト）。estimator.type と同様の実行時 SIL 設定で、ファームの
    // 既定値は変えない。
    sf::params::set_bool("eskf.use_baro", true);
    sf::params::set_bool("eskf.use_tof", false);
    sf::params::set_bool("eskf.use_flow", false);

    // Plant config: enable the N0 sensor-noise model when requested (seeded for
    // determinism). Default (no argv) → clean sensors, identical to P1–P4.
    // Plant 構成: 要求時に N0 センサノイズを有効化（シード付き＝決定論）。既定はクリーン。
    sil::Plant::Config plant_cfg;
    if (noise_on) {
        plant_cfg.noise.enable = true;
        plant_cfg.noise.seed = noise_seed;
        // Per-component env overrides (sweep noise terms without recompiling, like the
        // ALT_* gain hooks). Unset → the N0 defaults in SensorNoise::Config.
        // 成分別の env 上書き（再コンパイルせずノイズ成分を掃引）。未設定なら N0 既定。
        auto envf = [](const char* e, float& v) {
            const char* s = std::getenv(e);
            if (s != nullptr) v = (float)std::atof(s);
        };
        envf("N_GYRO_DENS",  plant_cfg.noise.gyro_density);
        envf("N_ACCEL_DENS", plant_cfg.noise.accel_density);
        envf("N_GYRO_BIAS",  plant_cfg.noise.gyro_bias_sigma);
        envf("N_ACCEL_BIAS", plant_cfg.noise.accel_bias_sigma);
        envf("N_GYRO_RW",    plant_cfg.noise.gyro_bias_rw);
        envf("N_ACCEL_RW",   plant_cfg.noise.accel_bias_rw);
    }
    printf("[flight] noise=%s seed=%u\n", noise_on ? noise_lvl : "off", noise_seed);
    if (!g_plant.init(model_path, plant_cfg)) {
        fprintf(stderr, "[flight] plant init failed (model: %s)\n", model_path);
        return 1;
    }
    g_plant.setStartHeight(kGroundZ);  // start on the ground for a real takeoff
    sil::bridge::current_imu = g_plant.imu();
    sil::bridge::has_plant = true;

    Scheduler& scheduler = Scheduler::instance();
    scheduler.set_on_advance(physics);

    TaskHandle_t h_state = nullptr, h_control = nullptr, h_imu = nullptr;
    xTaskCreatePinnedToCore(StateTask,   "StateTask",   config::STACK_STATE,
                            nullptr, config::PRIORITY_STATE,   &h_state,   1);
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", config::STACK_CONTROL,
                            nullptr, config::PRIORITY_CONTROL, &h_control, 1);
    g_control_task_handle = h_control;
    xTaskCreatePinnedToCore(ImuTask,     "ImuTask",     config::STACK_IMU,
                            nullptr, config::PRIORITY_IMU,     &h_imu,     1);

    scheduler.run(kSimDurationUs);

    // ---- G2 + G3 verdict (full flight, from physics truth) ----
    const float hold_band = g_hold_alt_max - g_hold_alt_min;  // altitude sag over hold
    const double att_rmse =
        g_att_err_n > 0 ? std::sqrt(g_att_err2_sum / (double)g_att_err_n) : 0.0;
    printf("[flight] max_alt=%.3f m  max_tilt=%.2f deg  yaw_spin=%.2f rad/s  "
           "yaw_after_stop=%.3f  final_alt=%.3f m  hold_band=%.3f m  att_rmse=%.3f deg\n",
           g_max_alt, g_max_tilt, g_yaw_during_spin, g_yaw_after_stop, g_final_alt,
           hold_band, att_rmse);

    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok) ++failures;
    };
    check(g_max_alt > 0.35f, "took off (max alt > 0.35 m)");
    check(g_max_tilt < 15.0f, "attitude stayed bounded (max tilt < 15 deg)");
    check(g_yaw_during_spin > 0.8f, "yaw spin tracked (> 0.8 rad/s)");
    check(g_yaw_after_stop < 0.2f, "yaw stopped (< 0.2 rad/s)");
    check(hold_band < 0.08f, "altitude held through yaw (band < 8 cm)");
    check(g_final_alt < 0.08f, "landed (final alt < 8 cm)");
    check(att_rmse < kG2AttRmseMaxDeg, "estimate tracked truth (G2 att RMSE < bound)");

    printf("[flight] %s — G2+G3 takeoff->hover->yaw->stop->landing\n",
           failures == 0 ? "OK" : "FAILED");

    if (g_traj != nullptr) { std::fclose(g_traj); g_traj = nullptr; }
    if (out_dir != nullptr) {
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/results.json", out_dir);
        FILE* rf = std::fopen(path, "w");
        if (rf != nullptr) {
            std::fprintf(rf,
                "{\n  \"gate\": \"G2+G3\",\n  \"milestone\": \"%s\",\n  \"pass\": %s,\n"
                "  \"flight\": \"takeoff-hover-yaw-stop-landing\",\n"
                "  \"duration_s\": %.1f,\n  \"estimator\": \"%s\",\n"
                "  \"noise\": \"%s\",\n  \"noise_seed\": %u,\n"
                "  \"metrics\": {\n"
                "    \"max_alt_m\": %.3f,\n    \"max_tilt_deg\": %.3f,\n"
                "    \"yaw_spin_rad_s\": %.3f,\n    \"yaw_after_stop_rad_s\": %.3f,\n"
                "    \"alt_hold_band_m\": %.3f,\n    \"final_alt_m\": %.3f,\n"
                "    \"g2_att_rmse_deg\": %.3f\n  },\n"
                "  \"checks\": [\n"
                "    {\"name\": \"took_off\",         \"pass\": %s},\n"
                "    {\"name\": \"attitude_bounded\",  \"pass\": %s},\n"
                "    {\"name\": \"yaw_spin\",          \"pass\": %s},\n"
                "    {\"name\": \"yaw_stopped\",       \"pass\": %s},\n"
                "    {\"name\": \"alt_held\",          \"pass\": %s},\n"
                "    {\"name\": \"landed\",            \"pass\": %s},\n"
                "    {\"name\": \"estimate_tracked\",  \"pass\": %s}\n  ]\n}\n",
                milestone,
                failures == 0 ? "true" : "false", kSimDurationUs * 1e-6,
                est_type == 1 ? "complementary" : "eskf",
                noise_on ? noise_lvl : "off", noise_seed,
                g_max_alt, g_max_tilt, g_yaw_during_spin, g_yaw_after_stop,
                hold_band, g_final_alt, att_rmse,
                g_max_alt > 0.35f ? "true" : "false",
                g_max_tilt < 15.0f ? "true" : "false",
                g_yaw_during_spin > 0.8f ? "true" : "false",
                g_yaw_after_stop < 0.2f ? "true" : "false",
                hold_band < 0.08f ? "true" : "false",
                g_final_alt < 0.08f ? "true" : "false",
                att_rmse < kG2AttRmseMaxDeg ? "true" : "false");
            std::fclose(rf);
            printf("[flight] bundle written to %s/\n", out_dir);
        }
    }
    return failures == 0 ? 0 : 2;
}
