/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file integrated_sil.cpp
 * @brief Integrated SIL — physics + ESKF + PID + StateManager + Failsafe
 *        統合SIL — 物理 + ESKF + PID + StateManager + Failsafe
 *
 * Unlike the legacy sil_main.cpp (which faked the flight state with a bare
 * `bool armed` + a hardcoded T_ARM time), this harness drives the UNMODIFIED
 * firmware StateManager and Failsafe through the simulation loop. The flight
 * state is the single source of truth for motor gating and control mode, and
 * failsafe alerts flow through the exact same handleAlert path as the target.
 *
 * 旧 sil_main.cpp は `bool armed` + ハードコード T_ARM で状態を模擬していたが、
 * 本ハーネスは無改変のファーム StateManager / Failsafe をループで駆動する。
 * フライト状態がモーター制御ゲートと制御モードの唯一の真実であり、failsafe
 * アラートは実機と全く同じ handleAlert 経路を流れる。
 *
 * Scenarios (--scenario <name>):
 *   nominal | comm_lost | impact | low_battery | eskf_diverged
 *
 * Build & run: make integrated_sil && ./integrated_sil [--scenario <name>]
 *   stdout = CSV state log, stderr = transition / debug log
 *
 * @design development_roadmap.md §2 — (B) integrated SIL, StateManager-driven [--]
 * @design architecture.md §2 — Sole transition owner                          [--]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>

// Silence library logs in THIS translation unit (matches sil_main.cpp).
// state_manager.cpp / failsafe.cpp are separate TUs and log to stderr.
// このTUのライブラリログを抑制（sil_main.cpp と同じ）。
// state_manager.cpp / failsafe.cpp は別TUで stderr にログ出力する。
#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"

#include "topics.hpp"
#include "state_manager.hpp"
#include "failsafe.hpp"
#include "scenario.hpp"
#include "battery_model.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;
using namespace sf::sil;

// =============================================================================
// Mixer: thrust/torque → motor duty (matches sf_actuator / sil_main)
// ミキサー: 推力/トルク → モーターduty（sf_actuator / sil_main と一致）
// =============================================================================

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    const float L = 0.023f;
    const float kappa = 0.00971f;

    float t = thrust_cmd * 0.25f;
    float r = torque[0] / (4.0f * L);
    float p = torque[1] / (4.0f * L);
    float y = torque[2] / (4.0f * kappa);

    float t1 = t - r + p + y;  // M1 FR CCW
    float t2 = t - r - p - y;  // M2 RR CW
    float t3 = t + r - p + y;  // M3 RL CCW
    float t4 = t + r + p - y;  // M4 FL CW

    for (int i = 0; i < 4; i++) {
        float ti = (i == 0) ? t1 : (i == 1) ? t2 : (i == 2) ? t3 : t4;
        if (ti < 0) ti = 0;
        duty[i] = sqrtf(ti / k_thrust);
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// ESKF divergence detector (SIL-side; the firmware lacks an isHealthy() API)
// ESKF発散検出（SIL側; ファームに isHealthy() が無いため）
// =============================================================================

static bool eskfDiverged(EskfCore& eskf)
{
    Vec3 p = eskf.getPosition();
    Vec3 v = eskf.getVelocity();
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return true;
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return true;
    if (p.x * p.x + p.y * p.y + p.z * p.z > 2500.0f) return true;  // |pos| > 50 m
    return false;
}

// =============================================================================
// Noise-stage selector — the diagnostic ladder's noise axis
// ノイズ段階セレクタ — 診断梯子のノイズ軸
//
// N0: perfectly clean sensors (isolate ESKF/linearization structure)
// N1: white noise + bias, no throttle-coupled vibration
// N2: full vibration model (calibrated from hover02 log) = default
// N3/N4 (colored/FFT-profile) reserved → treated as N2 for now.
//
// N0: 完全クリーン（ESKF・線形化の構造を分離）
// N1: 白色ノイズ+バイアス、スロットル結合振動なし
// N2: フル振動モデル（hover02ログ校正）= 既定。 N3/N4 は予約（暫定N2扱い）
// =============================================================================

static SensorNoiseParams makeNoise(int stage)
{
    SensorNoiseParams np;  // default = N2 (full)
    if (stage <= 0) {
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.gyro_bias_rw = 0; np.accel_bias_rw = 0;
        for (int i = 0; i < 3; i++) { np.vib_accel_k[i] = 0; np.vib_gyro_k[i] = 0; }
        np.tof_noise_base = 0; np.tof_noise_scale = 0;
        np.baro_noise_std = 0; np.baro_drift_rate = 0;
    } else if (stage == 1) {
        for (int i = 0; i < 3; i++) { np.vib_accel_k[i] = 0; np.vib_gyro_k[i] = 0; }
    }
    return np;
}

// =============================================================================
// Nominal flight progression — issues StateManager events from flight context
// 正常フライト進行 — フライト状況から StateManager イベントを発行
// =============================================================================

static void advanceNominal(StateManager& sm, Scenario scn,
                           float t, float height, float climb,
                           bool& mission_complete)
{
    switch (sm.getState()) {
        case FlightState::IDLE_GROUND:
            // Arm once. After a completed flight we stay grounded (no re-takeoff).
            // ARM は一度だけ。飛行完了後は着地したまま（再離陸しない）。
            if (t >= 4.0f && !mission_complete) sm.requestArm();   // → ARMED_GROUND
            break;
        case FlightState::ARMED_GROUND:
            if (t >= 4.2f) sm.notifyTakeoff();           // → TAKEOFF
            break;
        case FlightState::TAKEOFF:
            if (height > 0.03f && climb > 0.05f)
                sm.notifyTakeoffComplete();              // → FLYING
            break;
        case FlightState::FLYING:
            // Auto-land at t≥10s, except scenarios that drive landing via their
            // own injected alert (comm_lost → LANDING, impact → emergency IDLE).
            // t≥10s で自動着陸。ただし専用アラートで着陸を駆動するシナリオ
            // （comm_lost→LANDING, impact→緊急IDLE）は除く。
            if (scn != Scenario::COMM_LOST && scn != Scenario::IMPACT && t >= 10.0f)
                sm.notifyLandingRequest();               // → LANDING
            break;
        case FlightState::LANDING:
            if (height < 0.02f && climb < 0.05f) {
                sm.notifyLandingComplete();              // → IDLE_GROUND
                mission_complete = true;
            }
            break;
        default:
            break;
    }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv)
{
    // Fixed seed: the SIL must be deterministic so scenarios are reproducible
    // and regressions are detectable. (Legacy sil_main used time()-based seed.)
    // 固定シード: SILは決定論的でなければならない（シナリオ再現・回帰検出のため）。
    // 旧 sil_main は time() ベースのシードだった。
    srand(1);

    // --- Parse CLI: --scenario <name>, --feedback {truth|eskf}, --gust ---
    // CLI解析: --scenario <name>, --feedback {truth|eskf}, --gust
    Scenario scn = Scenario::NOMINAL;
    bool feedback_eskf = false;  // false=true attitude (M3 baseline), true=ESKF attitude (L4 closed loop)
    bool use_gust = false;       // inject body-frame gusts during FLYING / FLYING中に突風注入
    int  noise_stage = 2;        // N0..N2 (N2=full default; N3/N4 reserved) / ノイズ段階
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scn = parseScenario(argv[++i]);
        } else if (std::strcmp(argv[i], "--feedback") == 0 && i + 1 < argc) {
            feedback_eskf = (std::strcmp(argv[++i], "eskf") == 0);
        } else if (std::strcmp(argv[i], "--gust") == 0) {
            use_gust = true;
        } else if (std::strcmp(argv[i], "--noise-stage") == 0 && i + 1 < argc) {
            const char* ns = argv[++i];                        // accept "N2" or "2"
            noise_stage = (ns[0] == 'N' || ns[0] == 'n') ? atoi(ns + 1) : atoi(ns);
        }
    }

    fprintf(stderr, "=== StampFly Integrated SIL (scenario=%s, feedback=%s, gust=%s, noise=N%d) ===\n",
            scenarioName(scn), feedback_eskf ? "eskf" : "truth", use_gust ? "on" : "off", noise_stage);

    // --- Topics, StateManager, Failsafe ---
    topics_init();
    StateManager g_sm;
    g_sm.init();
    Failsafe fs;
    fs.init();

    // --- Synthetic battery ---
    BatteryModel battery;
    if (scn == Scenario::LOW_BATTERY) battery.reset(3.5f, 0.20f);  // sags to <3.0V while armed
    else                              battery.reset(4.0f, 0.01f);  // nearly constant

    // --- Physics ---
    QuadPhysics quad;
    QuadParams qp;
    ContactParams cp;
    SensorNoiseParams np = makeNoise(noise_stage);
    quad.init(qp, cp, np);

    // --- ESKF (initialized after calibration, like the firmware) ---
    EskfCore eskf;
    EskfConfig cfg;
    cfg.use_tof = true;
    cfg.use_baro = false;
    cfg.use_mag = false;
    cfg.use_flow = false;
    cfg.accel_noise = 0.3f;
    cfg.gyro_noise = 0.009655f;
    cfg.tof_noise = 0.01f;
    eskf.init(cfg);

    // --- PID cascade (same gains as sil_main) ---
    PID rate_r, rate_p, rate_y;
    PID att_r, att_p;
    PID alt_p, alt_v;
    rate_r.kp = 1.365e-3f; rate_r.ti = 0.7f; rate_r.td = 0.01f;
    rate_r.output_limit = 5.2e-3f; rate_r.eta = 0.125f;
    rate_p = rate_r; rate_p.kp = 1.995e-3f;
    rate_y.kp = 5.31e-3f; rate_y.ti = 1.6f; rate_y.output_limit = 2.2e-3f;
    att_r.kp = 5.0f; att_r.ti = 4.0f; att_r.output_limit = 3.0f;
    att_p = att_r;
    alt_p.kp = 0.6f; alt_p.ti = 7.0f; alt_p.output_limit = 0.5f;
    alt_v.kp = 0.1f; alt_v.ti = 2.5f; alt_v.output_limit = 0.15f;

    // --- Simulation parameters ---
    const float dt = 0.0025f;
    const int   steps = static_cast<int>(16.0f / dt);  // 16 s (enough for land)
    const int   tof_div = 13;                          // ~30 Hz
    const float hover_thrust = qp.mass * qp.gravity;
    const float target_alt = 0.5f;

    // Phase timing (sensor init + calibration), matches firmware startup
    // フェーズタイミング（センサ初期化 + キャリブ）、ファーム起動に対応
    const float T_PHASE1_END = 1.0f;
    const float T_PHASE2_END = 3.0f;
    const int   CAL_SAMPLES = 400;

    // --- LPF (matches sil_main ESKF path) ---
    const float alpha_eskf = 0.32f;
    Vec3 gyro_eskf_lpf = {}, accel_eskf_lpf = {};
    bool lpf_initialized = false;

    // --- State tracking ---
    bool eskf_ready = false;
    bool was_airborne = false;
    bool mission_complete = false;  // set true after first full land / 初回着陸完了で true

    // --- Gust schedule (body-frame force[N]/torque[Nm], injected only while FLYING) ---
    // --- 突風スケジュール（body座標 力[N]/トルク[Nm]、FLYING中のみ注入）---
    // Values from flight_scenario_test.cpp (wind on asymmetric body), retimed to
    // this harness's FLYING window (~t=4.5..10s). Enabled with --gust.
    // 値は flight_scenario_test.cpp 由来（非対称体への風）、本ハーネスの
    // FLYING期間(~t=4.5..10s)に再配置。--gust で有効化。
    struct Gust { float t, dur; Vec3 force, torque; const char* name; };
    const Gust gusts[] = {
        {5.0f, 0.5f,  {0.0f,   -0.012f, 0.0f},  {6e-5f,  0.0f,   0.0f}, "side wind"},
        {6.5f, 0.2f,  {0.015f, 0.01f,   0.0f},  {-5e-5f, 7e-5f,  0.0f}, "strong burst"},
        {7.5f, 1.5f,  {0.003f, -0.003f, 0.0f},  {2e-5f,  -2e-5f, 0.0f}, "turbulence"},
        {9.2f, 0.15f, {0.02f,  0.0f,    0.005f},{0.0f,   8e-5f,  0.0f}, "sharp gust"},
    };
    const int n_gusts = 4;
    Vec3 cal_accel_sum = {}, cal_gyro_sum = {};
    int cal_count = 0;

    // --- Scenario injection flags (one-shot) ---
    bool alert_injected = false;   // comm_lost / eskf_diverged
    bool impact_injected = false;  // impact

    // --- Sticky last-alert for CSV ---
    uint8_t last_alert_type = 0;

    // --- CSV header (stdout) ---
    printf("time,true_x,true_y,true_z,true_vz,"
           "roll,pitch,yaw,"
           "eskf_x,eskf_y,eskf_z,eskf_vz,"
           "eskf_roll,eskf_pitch,eskf_yaw,"
           "thrust,m1,m2,m3,m4,"
           "flight_state,flight_mode,armed,alert_type,"
           "norm_ratio,innov_norm,p_att_x,p_att_y\n");

    // =========================================================================
    // Simulation loop
    // =========================================================================
    for (int step = 0; step < steps; step++) {
        float t = step * dt;
        float zero_cmd[4] = {0, 0, 0, 0};

        // --- Sensors + LPF ---
        SimSensors sens = quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 accel_raw(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro_raw(sens.gyro.x, sens.gyro.y, sens.gyro.z);
        if (!lpf_initialized) {
            gyro_eskf_lpf = gyro_raw;
            accel_eskf_lpf = accel_raw;
            lpf_initialized = true;
        }
        gyro_eskf_lpf = gyro_eskf_lpf * (1.0f - alpha_eskf) + gyro_raw * alpha_eskf;
        accel_eskf_lpf = accel_eskf_lpf * (1.0f - alpha_eskf) + accel_raw * alpha_eskf;
        Vec3 accel = accel_eskf_lpf;
        Vec3 gyro = gyro_eskf_lpf;

        // --- Publish sensors so Failsafe can read them (every step) ---
        // --- Failsafe が読めるようセンサを発行（毎ステップ）---
        FlightState st_now = g_sm.getState();
        bool impact_now = (scn == Scenario::IMPACT && isAirborne(st_now) &&
                           t >= 8.0f && !impact_injected);
        ImuData imu = {};
        if (impact_now) {
            imu.accel[2] = 8.0f * 9.80665f;  // 8 G impact
            impact_injected = true;
            fprintf(stderr, "t=%.2f scenario: inject 8G impact\n", t);
        } else {
            imu.accel[0] = accel_raw.x; imu.accel[1] = accel_raw.y; imu.accel[2] = accel_raw.z;
        }
        imu.gyro[0] = gyro_raw.x; imu.gyro[1] = gyro_raw.y; imu.gyro[2] = gyro_raw.z;
        sensor_imu.publish(imu);

        battery.step(dt, g_sm.isArmed());
        PowerData pw = {};
        pw.voltage = battery.voltage();
        sensor_power.publish(pw);

        // --- Phase 1: sensor init (t < 1s), no control, no ESKF ---
        if (t < T_PHASE1_END) {
            quad.step(zero_cmd, dt);
            continue;
        }

        // --- Phase 2: calibration (1s ≤ t < 3s) ---
        if (t < T_PHASE2_END) {
            quad.step(zero_cmd, dt);
            if (cal_count < CAL_SAMPLES) {
                cal_accel_sum += accel;
                cal_gyro_sum += gyro;
                cal_count++;
            }
            if (cal_count == CAL_SAMPLES && !eskf_ready) {
                Vec3 accel_avg = cal_accel_sum * (1.0f / cal_count);
                Vec3 gyro_avg = cal_gyro_sum * (1.0f / cal_count);
                Vec3 accel_bias(accel_avg.x, accel_avg.y, accel_avg.z - (-qp.gravity));
                eskf.setGyroBias(gyro_avg);
                eskf.setAccelBias(accel_bias);
                eskf.setAttitudeFromGravity(accel_avg);
                eskf.shrinkBiasCovariance(0.01f);
                eskf.setFreezeAccelBias(true);
                eskf_ready = true;
                g_sm.forceIdle();  // INIT → IDLE_GROUND (firmware does this on startup complete)
                fprintf(stderr, "=== ESKF ready, state → IDLE_GROUND ===\n");
            }
            continue;
        }

        // --- ESKF predict + observe ---
        if (eskf_ready) {
            eskf.predict(accel, gyro, dt);
            eskf.updateAccelAttitude(accel);
            if (step % tof_div == 0) {
                eskf.updateToF(sens.tof_bottom);
                eskf.updateToFVelocity(sens.tof_bottom, tof_div * dt);
            }
        }

        auto true_st = quad.getState();
        Vec3 eskf_pos = eskf.getPosition();
        Vec3 eskf_vel = eskf.getVelocity();
        Vec3 eskf_euler = eskf.getAttitude().to_euler();
        float height = -true_st.position.z;
        float climb = -true_st.velocity.z;

        // --- Scenario alert injection (comm_lost / eskf_diverged) ---
        if (scn == Scenario::COMM_LOST && g_sm.getState() == FlightState::FLYING &&
            t >= 8.0f && !alert_injected) {
            SystemAlert a = {};
            a.type = static_cast<uint8_t>(AlertType::COMM_LOST);
            a.severity = static_cast<uint8_t>(AlertSeverity::CRITICAL);
            system_alert.publish(a);
            alert_injected = true;
            fprintf(stderr, "t=%.2f scenario: inject COMM_LOST\n", t);
        }
        if (scn == Scenario::ESKF_DIVERGED && g_sm.getState() == FlightState::FLYING &&
            t >= 8.0f && !alert_injected) {
            SystemAlert a = {};
            a.type = static_cast<uint8_t>(AlertType::ESKF_DIVERGED);
            a.severity = static_cast<uint8_t>(AlertSeverity::WARNING);
            system_alert.publish(a);
            alert_injected = true;
            fprintf(stderr, "t=%.2f scenario: inject ESKF_DIVERGED\n", t);
        }

        // --- Failsafe checks (~50 Hz) ---
        if (step % 8 == 0) fs.update();

        // --- True ESKF divergence guard (real-vehicle-valid) ---
        if (eskf_ready && eskfDiverged(eskf)) {
            SystemAlert a = {};
            a.type = static_cast<uint8_t>(AlertType::ESKF_DIVERGED);
            a.severity = static_cast<uint8_t>(AlertSeverity::WARNING);
            system_alert.publish(a);
        }

        // --- Drain alert queue into StateManager (same as state_task.cpp) ---
        FlightState before_alerts = g_sm.getState();
        SystemAlert alert;
        while (system_alert.read(alert)) {
            last_alert_type = alert.type;
            g_sm.handleAlert(alert);
        }
        // An emergency landing (airborne → IDLE_GROUND via alert) ends the
        // mission, so we must not auto-rearm afterwards.
        // 緊急着陸（アラートで airborne → IDLE_GROUND）はミッション終了。
        // 以後の自動再ARMを防ぐ。
        if (isAirborne(before_alerts) && g_sm.getState() == FlightState::IDLE_GROUND)
            mission_complete = true;

        // --- Advance nominal flight progression ---
        advanceNominal(g_sm, scn, t, height, climb, mission_complete);

        // --- Position/velocity estimate selection by flight state ---
        FlightState st = g_sm.getState();
        bool airborne = isAirborne(st);  // TAKEOFF / FLYING / LANDING
        Vec3 est_pos, est_vel;
        if (airborne) {
            if (!was_airborne) { eskf.resetPositionVelocity(); was_airborne = true; }
            est_pos = eskf_pos;
            est_vel = eskf_vel;
        } else {
            est_pos = true_st.position;
            est_vel = true_st.velocity;
            eskf.resetPositionVelocity();
            was_airborne = false;
        }
        // Attitude fed to control: truth (M3 baseline) or ESKF estimate (closed loop).
        // With --feedback eskf this becomes a true L4 closed loop (estimate in the loop).
        // 制御に渡す姿勢: 真値(M3基準) または ESKF推定(閉ループ)。
        // --feedback eskf で真のL4閉ループ（推定値をループ内に）になる。
        Vec3 est_euler = feedback_eskf ? eskf_euler : true_st.attitude.to_euler();

        // --- Control (motors only when airborne) ---
        float thrust = 0;
        float torque[3] = {0, 0, 0};
        if (airborne) {
            float alt_target = (st == FlightState::LANDING) ? 0.0f : target_alt;
            float est_height = -est_pos.z;
            float est_climb = -est_vel.z;
            float vel_sp = alt_p.compute(alt_target - est_height, dt);
            float thrust_corr = alt_v.compute(vel_sp - est_climb, dt);
            thrust = hover_thrust + thrust_corr;
            float rate_sp_r = att_r.compute(0 - est_euler.x, dt);
            float rate_sp_p = att_p.compute(0 - est_euler.y, dt);
            torque[0] = rate_r.compute(rate_sp_r - true_st.angular_rate.x, dt);
            torque[1] = rate_p.compute(rate_sp_p - true_st.angular_rate.y, dt);
            torque[2] = rate_y.compute(0 - true_st.angular_rate.z, dt);
        }

        // --- Gust injection (FLYING only, if enabled) ---
        // --- 突風注入（FLYING中のみ、有効時）---
        Vec3 gust_f = {}, gust_tq = {};
        if (use_gust && st == FlightState::FLYING) {
            for (int gi = 0; gi < n_gusts; gi++) {
                if (t >= gusts[gi].t && t < gusts[gi].t + gusts[gi].dur) {
                    gust_f += gusts[gi].force;
                    gust_tq += gusts[gi].torque;
                }
            }
        }
        quad.setExternalForceBody(gust_f, gust_tq);

        // --- Mixer + physics ---
        float motor_duty[4] = {0, 0, 0, 0};
        if (airborne) mixer(thrust, torque, motor_duty, qp.k_thrust);
        quad.step(airborne ? motor_duty : zero_cmd, dt);

        // --- Debug (stderr, 1 Hz) ---
        if (step % 400 == 0) {
            fprintf(stderr, "t=%.1f state=%-12s h=%.3f T=%.3f\n",
                    t, flightStateName(st), height, thrust);
        }

        // --- CSV (stdout, 50 Hz) ---
        if (step % 8 == 0) {
            Vec3 true_euler = true_st.attitude.to_euler();
            // ESKF diagnostics (the diagnostic ladder's observability axis):
            // norm gate ratio, accel innovation magnitude, attitude covariance.
            // ESKF診断（診断梯子の可観測性軸）: ノルムゲート比・加速度イノベーション・姿勢共分散
            Vec3 accel_bc = accel - eskf.getAccelBias();
            float norm_ratio = fabsf(accel_bc.norm() - cfg.gravity) / cfg.gravity;
            Vec3 g_ned_vec(0.0f, 0.0f, -cfg.gravity);
            Vec3 g_exp = eskf.getAttitude().inv_rotate(g_ned_vec);
            float innov_norm = (accel_bc - g_exp).norm();
            float p_att_x = eskf.getPDiag(ATT_X);
            float p_att_y = eskf.getPDiag(ATT_Y);
            printf("%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.3f,%.3f,%.3f,%.3f,"
                   "%d,%d,%d,%d,"
                   "%.5f,%.4f,%.6f,%.6f\n",
                   t,
                   true_st.position.x, true_st.position.y,
                   true_st.position.z, true_st.velocity.z,
                   true_euler.x * 57.3f, true_euler.y * 57.3f, true_euler.z * 57.3f,
                   eskf_pos.x, eskf_pos.y, eskf_pos.z, eskf_vel.z,
                   eskf_euler.x * 57.3f, eskf_euler.y * 57.3f, eskf_euler.z * 57.3f,
                   thrust,
                   motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3],
                   static_cast<int>(st), static_cast<int>(g_sm.getMode()),
                   g_sm.isArmed() ? 1 : 0, static_cast<int>(last_alert_type),
                   norm_ratio, innov_norm, p_att_x, p_att_y);
        }
    }

    fprintf(stderr, "\n=== Final state: %s ===\n", flightStateName(g_sm.getState()));
    return 0;
}
