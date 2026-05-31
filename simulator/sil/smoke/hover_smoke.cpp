/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file hover_smoke.cpp
 * @brief P1.2(c) closed loop — vehicle_new hovers in the SIL (gate G3).
 *        P1.2(c) 閉ループ — vehicle_new が SIL でホバーする（ゲート G3）。
 *
 * The real firmware loop (ImuTask → estimate_state → ControlTask → mixer →
 * actuator_motor) runs on the host RTOS emulator at 400Hz. The MuJoCo Plant
 * closes the loop: each IMU cycle it reads the per-motor duty from
 * actuator_motor, advances physics, and feeds the synthetic IMU back to the
 * firmware via the sim BMI270 wrapper (plant_bridge). Algorithm-independent:
 * the bench never names ESKF or PID — only sensors in, motor duty out, physics
 * truth for the verdict.
 *
 * 実ファームループ（ImuTask → estimate_state → ControlTask → ミキサー →
 * actuator_motor）をホスト RTOS エミュレータ上で 400Hz で走らせる。MuJoCo Plant が
 * ループを閉じる: IMU サイクルごとに actuator_motor の各モータ duty を読み、物理を
 * 進め、合成 IMU を sim BMI270 ラッパ（plant_bridge）経由でファームへ返す。算法
 * 非依存: ベンチは ESKF も PID も名指さない — センサ入力・モータ出力・物理真値だけ。
 *
 * G3 gate: starting at hover (motors primed), the craft holds attitude and stays
 * bounded in altitude and horizontal position over the run.
 * G3 ゲート: ホバー（モータ prime）から始め、姿勢を保ち、高度と水平位置が有界。
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "scheduler.hpp"
#include "topics.hpp"
#include "params.hpp"
#include "data_types.hpp"
#include "flight_state.hpp"
#include "config.hpp"
#include "plant.hpp"
#include "plant_bridge.hpp"

// Firmware task functions (unmodified) + the IMU→Control notification handle.
void ImuTask(void*);
void ControlTask(void*);
void StateTask(void*);
extern TaskHandle_t g_control_task_handle;

using sil::rtos::Scheduler;
using sf::math::Vec3;

namespace {
constexpr int64_t kSimDurationUs = 3000000;  // 3 s
constexpr float kStartAlt = 0.5f;            // model starts the body at 0.5 m

sil::Plant g_plant;
float g_hover_duty = 0.0f;       // Plant motor duty at hover (for priming)
// Controller throttle that yields hover thrust. The controller output is now
// PHYSICAL (thrust [N] = throttle · max_thrust_) and the B^-1 mixer allocates it,
// so hover throttle = mg / max_thrust_total (0.363 / 0.672 ≈ 0.54).
// コントローラ出力は物理量（推力[N] = throttle·max_thrust_）で B^-1 が配分するため、
// ホバー throttle = mg / 最大総推力（0.363/0.672 ≈ 0.54）。
constexpr float kMaxThrustN = 0.672f;                    // = controller max_thrust_
const float g_hover_throttle = (0.037f * 9.81f) / kMaxThrustN;
int64_t g_last_step_us = 0;

// At this time we command a yaw rate to prove the rate loop actively tracks
// (yaw is decoupled from altitude/horizontal position). Expected steady-state
// rate = setpoint.yaw · max_yaw_rate = 0.2 · 5.0 = 1.0 rad/s.
// この時刻でヨーレートを指令し、レート内ループが能動追従することを実証する
// （ヨーは高度・水平位置と非干渉）。定常レート = 0.2·5.0 = 1.0 rad/s。
constexpr int64_t kYawCmdUs = 1500000;   // 1.5 s
constexpr float   kYawStick = 0.2f;
constexpr float   kYawExpected = 1.0f;   // 0.2 · max_yaw_rate(5.0)

// Tracked bounds over the whole run (filled in physics()).
float g_max_alt_err = 0.0f;
float g_max_tilt_deg = 0.0f;
float g_max_horiz = 0.0f;
float g_final_yaw_rate = 0.0f;

// Optional trajectory recording for the review video (set when an out dir is
// given). One CSV row every kRecDiv physics steps (400Hz / 8 = 50 fps).
// レビュー動画用の任意の軌跡記録（出力ディレクトリ指定時）。kRecDiv ステップ毎に
// 1 行（400Hz/8 = 50 fps）。
FILE* g_traj = nullptr;
int   g_rec_count = 0;
constexpr int kRecDiv = 8;
float g_yaw_cmd_now = 0.0f;  // current commanded yaw rate (for the graph)

float tiltDeg(const sf::math::Quat& q_nb)
{
    // Angle between NED-down and the body's down axis (0 = perfectly level).
    // NED 下方と機体の下方軸のなす角（0 = 完全水平）。
    Vec3 down_body = q_nb.inv_rotate(Vec3{0, 0, 1});
    float c = down_body.z;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * 180.0f / 3.14159265f;
}

// on_advance hook: arm once, then each IMU cycle apply the firmware's motor
// command to the Plant, step physics, and publish the synthetic IMU.
// on_advance フック: 一度 ARM し、以降 IMU サイクルごとにファームのモータ指令を
// Plant に与え、物理を進め、合成 IMU を発行する。
void physics(int64_t now_us)
{
    static bool armed = false;
    if (!armed) {
        armed = true;
        sf::SystemMode mode = {};
        mode.state = static_cast<uint8_t>(sf::FlightState::FLYING);
        mode.armed = true;
        mode.timestamp = static_cast<uint32_t>(now_us);
        sf::system_mode.publish(mode);

        // STABILIZE (controller default): thrust = throttle·max_thrust_ (=1.0) is
        // used directly as duty, so throttle = hover duty holds altitude.
        // STABILIZE（既定）: thrust = throttle·max_thrust_(=1.0) を duty として使うので
        // throttle = ホバー duty で高度を保つ。
        sf::CommandSetpoint sp = {};
        sp.throttle = g_hover_throttle;
        sp.roll = sp.pitch = sp.yaw = 0.0f;
        sp.timestamp = static_cast<uint32_t>(now_us);
        sf::command_setpoint.publish(sp);
    }

    // At kYawCmdUs, command a yaw rate. The rate inner loop (closed by
    // angular_rate) should drive omega_frd.z to ~1 rad/s while hover holds.
    // kYawCmdUs でヨーレートを指令。レート内ループ（angular_rate で閉じた）が
    // omega_frd.z を ~1 rad/s に駆動し、ホバーは保たれるはず。
    static bool yaw_cmd = false;
    if (!yaw_cmd && now_us >= kYawCmdUs) {
        yaw_cmd = true;
        g_yaw_cmd_now = kYawExpected;
        sf::CommandSetpoint sp = {};
        sp.throttle = g_hover_throttle;
        sp.roll = sp.pitch = 0.0f;
        sp.yaw = kYawStick;
        sp.timestamp = static_cast<uint32_t>(now_us);
        sf::command_setpoint.publish(sp);
    }

    // Apply the latest firmware motor command, advance physics to now_us.
    // 最新のファームモータ指令を与え、物理を now_us まで進める。
    sf::MotorOutput cmd = sf::actuator_motor.latest();
    g_plant.setDuty(cmd);
    if (now_us > g_last_step_us) {
        g_plant.step(static_cast<float>(now_us - g_last_step_us) * 1e-6f);
        g_last_step_us = now_us;
    }

    // Feed the synthetic IMU back to the firmware (via the sim BMI270 wrapper).
    // 合成 IMU をファームへ返す（sim BMI270 ラッパ経由）。
    sil::bridge::current_imu = g_plant.imu();

    // Track physical-truth bounds for the G3 verdict.
    // G3 判定のため物理真値の有界性を記録。
    sil::Plant::Truth t = g_plant.truth();
    float alt_err = std::fabs(-t.pos_ned.z - kStartAlt);
    float horiz = std::sqrt(t.pos_ned.x * t.pos_ned.x + t.pos_ned.y * t.pos_ned.y);
    float tilt = tiltDeg(t.q_nb);
    if (alt_err > g_max_alt_err) g_max_alt_err = alt_err;
    if (horiz > g_max_horiz) g_max_horiz = horiz;
    if (tilt > g_max_tilt_deg) g_max_tilt_deg = tilt;
    g_final_yaw_rate = t.omega_frd.z;  // tracked yaw rate (settles after the cmd)

    // Record a trajectory frame for the review video (downsampled to ~50 fps).
    // The estimate is the firmware's own ESKF output (estimate_state topic) so
    // the video can overlay estimate vs physics truth.
    // レビュー動画用に軌跡フレームを記録（~50fps にダウンサンプル）。推定はファーム
    // 自身の ESKF 出力（estimate_state トピック）なので推定 vs 物理真値を重ねられる。
    if (g_traj != nullptr && (g_rec_count++ % kRecDiv) == 0) {
        const double* q = g_plant.data()->qpos;  // MuJoCo [x,y,z, qw,qx,qy,qz]
        sf::StateEstimate est = sf::estimate_state.latest();
        sf::math::Quat qe(est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3]);
        float est_alt = -est.position[2];
        float est_tilt = tiltDeg(qe);
        fprintf(g_traj,
                "%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,"
                "%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f\n",
                now_us * 1e-6,
                q[0], q[1], q[2], q[3], q[4], q[5], q[6],
                -t.pos_ned.z, tilt, t.omega_frd.z, g_yaw_cmd_now,
                est_alt, est_tilt,
                cmd.duty[0], cmd.duty[1], cmd.duty[2], cmd.duty[3]);
    }
}
}  // namespace

int main(int argc, char** argv)
{
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    const char* out_dir = (argc > 2) ? argv[2] : nullptr;  // record bundle if given
    const int est_type = (argc > 3) ? std::atoi(argv[3]) : 0;  // 0=ESKF, 1=complementary

    // Open the trajectory file for the review video (header = column names).
    // レビュー動画用の軌跡ファイルを開く（ヘッダ = 列名）。
    if (out_dir != nullptr) {
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/trajectory.csv", out_dir);
        g_traj = std::fopen(path, "w");
        if (g_traj != nullptr) {
            std::fprintf(g_traj,
                "t,px,py,pz,qw,qx,qy,qz,alt,tilt,yawrate,yawcmd,"
                "alt_est,tilt_est,m0,m1,m2,m3\n");
        }
    }

    sf::params::init();
    sf::topics_init();

    // Algorithm-independence (RESET_PLAN P2): select the estimator by a parameter
    // only. The bench (physics, loop, sensors, gates) is byte-for-byte the same;
    // the firmware's IMU-task factory reads estimator.type and swaps ESKF ↔
    // complementary. Nothing in this harness or the firmware tasks changes.
    // アルゴリズム非依存（P2）: 推定器を param だけで選ぶ。ベンチ（物理・ループ・
    // センサ・ゲート）は完全に同一で、ファームの IMU タスクのファクトリが
    // estimator.type を読んで ESKF ↔ 相補を差し替える。
    sf::params::set_int("estimator.type", est_type);
    printf("[hover_smoke] estimator.type=%d (%s)\n",
           est_type, est_type == 1 ? "complementary filter" : "ESKF");

    if (!g_plant.init(model_path)) {
        fprintf(stderr, "[hover_smoke] plant init failed (model: %s)\n", model_path);
        return 1;
    }
    g_hover_duty = g_plant.hoverDuty();
    printf("[hover_smoke] hover_duty=%.4f, thrust/motor=%.4f N (mg/4=%.4f)\n",
           g_hover_duty, g_plant.dutyToThrust(g_hover_duty), 0.037f * 9.81f / 4.0f);

    // Prime motors at hover and seed actuator_motor + the IMU bridge so the very
    // first physics step already sees hover thrust (no spool-up dip).
    // モータをホバーで prime し、actuator_motor と IMU bridge を種付けして、最初の
    // 物理ステップから既にホバー推力にする（スプールアップの落下なし）。
    sf::MotorOutput m0{};
    for (int i = 0; i < 4; ++i) m0.duty[i] = g_hover_duty;
    g_plant.setDuty(m0);
    g_plant.primeMotors();
    sf::actuator_motor.publish(m0);
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

    // ---- G3 verdict (all from physics truth) ----
    sil::Plant::Truth t = g_plant.truth();
    float final_alt = -t.pos_ned.z;
    float final_tilt = tiltDeg(t.q_nb);
    sf::MotorOutput m = sf::actuator_motor.latest();

    printf("[hover_smoke] final: alt=%.3f m  tilt=%.2f deg  horiz=%.3f m  "
           "motors=[%.3f %.3f %.3f %.3f]\n",
           final_alt, final_tilt,
           std::sqrt(t.pos_ned.x * t.pos_ned.x + t.pos_ned.y * t.pos_ned.y),
           m.duty[0], m.duty[1], m.duty[2], m.duty[3]);
    printf("[hover_smoke] bounds over %.1f s: max|alt-0.5|=%.3f m  max tilt=%.2f deg  "
           "max horiz=%.3f m\n",
           kSimDurationUs * 1e-6, g_max_alt_err, g_max_tilt_deg, g_max_horiz);
    // G3 gate: attitude stays near level, altitude + horizontal position bounded,
    // motors within [0,1] and not collapsed to zero.
    // G3 ゲート: 姿勢は水平近く、高度・水平位置は有界、モータは [0,1] で 0 潰れなし。
    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok) ++failures;
    };
    check(g_max_tilt_deg < 10.0f, "attitude stays level (max tilt < 10 deg)");
    check(g_max_alt_err < 0.10f, "altitude bounded (max |alt-0.5| < 10 cm)");
    check(g_max_horiz < 0.10f, "horizontal position bounded (< 10 cm)");
    check(m.duty[0] > 0.05f && m.duty[0] < 0.99f, "motors active, not saturated");

    // Rate loop tracks a commanded yaw rate. This was 0.05 rad/s before the SIL
    // gain recalibration (rate.yaw.kp 5.31e-3 → 1.0, ti 1.6 → 20): the simplified
    // mixer attenuates yaw by kappa, so the yaw loop needs a high-gain near-P
    // controller (integrator plant). Proves the rate loop is doing real work.
    // レート内ループが指令ヨーレートを追従する。SIL のゲイン再較正（rate.yaw.kp
    // 5.31e-3→1.0, ti 1.6→20）前は 0.05 rad/s だった。簡易ミキサーが κ でヨーを
    // 減衰するため高ゲインのほぼ P 制御（積分器プラント）が要る。レート内ループが
    // 実際に働いている証拠。
    printf("[hover_smoke] yaw-rate tracking: commanded=%.2f rad/s, measured=%.2f rad/s\n",
           kYawExpected, g_final_yaw_rate);
    check(std::fabs(g_final_yaw_rate - kYawExpected) < 0.20f,
          "rate loop tracks yaw command (omega_z → 1 rad/s)");

    printf("[hover_smoke] %s — G3 closed-loop hover\n",
           failures == 0 ? "OK" : "FAILED");

    // Write the machine-readable bundle (results.json) alongside the trajectory.
    // This is the single source of truth for the milestone gate (RESET_PLAN §8).
    // 機械可読バンドル（results.json）を軌跡と並べて書く。マイルストーンゲートの
    // 唯一の正（RESET_PLAN §8）。
    if (g_traj != nullptr) {
        std::fclose(g_traj);
        g_traj = nullptr;
    }
    if (out_dir != nullptr) {
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/results.json", out_dir);
        FILE* rf = std::fopen(path, "w");
        if (rf != nullptr) {
            std::fprintf(rf,
                "{\n"
                "  \"gate\": \"G3\",\n"
                "  \"milestone\": \"P1\",\n"
                "  \"pass\": %s,\n"
                "  \"duration_s\": %.1f,\n"
                "  \"hover_duty\": %.4f,\n"
                "  \"metrics\": {\n"
                "    \"max_tilt_deg\": %.3f,\n"
                "    \"max_alt_err_m\": %.4f,\n"
                "    \"max_horiz_m\": %.4f,\n"
                "    \"yaw_cmd_rad_s\": %.3f,\n"
                "    \"yaw_meas_rad_s\": %.3f\n"
                "  },\n"
                "  \"checks\": [\n"
                "    {\"name\": \"attitude_level\",   \"pass\": %s, \"limit_deg\": 10.0},\n"
                "    {\"name\": \"altitude_bounded\", \"pass\": %s, \"limit_m\": 0.10},\n"
                "    {\"name\": \"horiz_bounded\",    \"pass\": %s, \"limit_m\": 0.10},\n"
                "    {\"name\": \"motors_active\",    \"pass\": %s},\n"
                "    {\"name\": \"yaw_tracks\",       \"pass\": %s, \"tol_rad_s\": 0.20}\n"
                "  ]\n"
                "}\n",
                failures == 0 ? "true" : "false",
                kSimDurationUs * 1e-6, g_hover_duty,
                g_max_tilt_deg, g_max_alt_err, g_max_horiz,
                kYawExpected, g_final_yaw_rate,
                g_max_tilt_deg < 10.0f ? "true" : "false",
                g_max_alt_err < 0.10f ? "true" : "false",
                g_max_horiz < 0.10f ? "true" : "false",
                (m.duty[0] > 0.05f && m.duty[0] < 0.99f) ? "true" : "false",
                std::fabs(g_final_yaw_rate - kYawExpected) < 0.20f ? "true" : "false");
            std::fclose(rf);
            printf("[hover_smoke] bundle written to %s/ (trajectory.csv + results.json)\n",
                   out_dir);
        }
    }
    return failures == 0 ? 0 : 2;
}
