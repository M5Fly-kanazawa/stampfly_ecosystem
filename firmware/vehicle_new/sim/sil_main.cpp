/**
 * @file sil_main.cpp
 * @brief SIL Simulation — fly StampFly on PC with visualization
 *        SILシミュレーション — PC上でStampFlyを飛ばして可視化
 *
 * Reproduces the firmware startup sequence:
 *   Phase1: Sensor init (1s)
 *   Phase2: Sensor stabilization + calibration (2s)
 *   Phase3: ESKF init from calibration data (instant)
 *   IDLE:   ESKF running, pos/vel held, wait for ARM
 *   ARM:    Motors start
 *   TAKEOFF: resetPosVel + unfreeze accel bias
 *   FLYING:  Full ESKF closed-loop control
 *
 * ファームの起動シーケンスを再現:
 *   Phase1: センサ初期化（1秒）
 *   Phase2: センサ安定化 + キャリブレーション（2秒）
 *   Phase3: キャリブレーションデータからESKF初期化
 *   IDLE:   ESKF動作中、pos/vel保持、ARM待ち
 *   ARM:    モーター始動
 *   離陸:   resetPosVel + accelBias解凍
 *   飛行:   ESKF完全閉ループ制御
 *
 * Build & run: make run
 * Save CSV:    make csv
 * Plot:        make plot
 *
 * @design requirements.md §10 — SIL simulator                        [--]
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;

// =============================================================================
// Mixer: thrust/torque → motor duty (matches sf_actuator)
// ミキサー: 推力/トルク → モーターduty（sf_actuatorと一致）
// =============================================================================

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    float L = 0.023f;
    float kappa = 0.00971f;

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
// Simulation timeline (matches firmware startup)
// シミュレーションタイムライン（ファーム起動に対応）
// =============================================================================

// Phase timing / フェーズタイミング
static constexpr float T_PHASE1_END  = 1.0f;   // Sensor init complete
static constexpr float T_PHASE2_END  = 3.0f;   // Calibration complete, ESKF init
static constexpr float T_ARM         = 4.0f;    // ARM command (like stick input)
static constexpr float T_SIM_END     = 14.0f;   // Total simulation time

// Calibration parameters / キャリブレーションパラメータ
static constexpr int   CAL_SAMPLES   = 400;     // 1s at 400Hz (Phase2)
static constexpr float TAKEOFF_ALT   = 0.03f;   // Takeoff detection threshold [m]

// =============================================================================
// Main Simulation
// =============================================================================

int main()
{
    srand(static_cast<unsigned>(time(nullptr)));

    fprintf(stderr, "=== StampFly SIL Simulation ===\n");
    fprintf(stderr, "Startup sequence: Phase1(0-%.0fs) → Phase2(%.0f-%.0fs) → "
            "IDLE(%.0fs) → ARM(%.0fs) → FLY\n\n",
            T_PHASE1_END, T_PHASE1_END, T_PHASE2_END, T_PHASE2_END, T_ARM);

    // --- Initialize physics engine ---
    // --- 物理エンジンを初期化 ---
    QuadPhysics quad;
    QuadParams qp;
    ContactParams cp;
    SensorNoiseParams np;  // Full noise from 72-log analysis
    quad.init(qp, cp, np);

    // --- Initialize ESKF (but don't enable yet — matches firmware Phase1-2) ---
    // --- ESKF初期化（まだ有効化しない — ファームPhase1-2に対応）---
    EskfCore eskf;
    EskfConfig cfg;
    cfg.use_tof = true;
    cfg.use_baro = false;
    cfg.use_mag = false;
    cfg.use_flow = false;
    cfg.accel_noise = 0.3f;
    cfg.gyro_noise = 0.009655f;
    cfg.tof_noise = 0.01f;
    // accel_chi2_gate = 7.81 (default, chi²(3, 0.95))
    eskf.init(cfg);

    // --- Initialize PIDs ---
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
    const int steps = static_cast<int>(T_SIM_END / dt);
    const int tof_div = 13;  // ~30Hz (400/13 ≈ 30.8Hz, matches real ToF rate)
    const float hover_thrust = qp.mass * qp.gravity;
    const float target_alt = 0.5f;

    fprintf(stderr, "Duration: %.1fs at %.0fHz (%d steps)\n",
            T_SIM_END, 1/dt, steps);
    fprintf(stderr, "Target alt: %.2fm, Hover thrust: %.4fN\n\n",
            target_alt, hover_thrust);

    // --- State tracking ---
    bool eskf_ready = false;        // ESKF initialized and running
    bool armed = false;             // Motors enabled
    bool has_taken_off = false;     // Takeoff detected (one-shot)
    bool is_flying = false;         // Airborne (pos/vel estimation active)

    // --- IMU LPF (matches firmware notch+LPF pipeline) ---
    // --- IMU LPF（ファームのnotch+LPFパイプラインに対応）---
    // ESKF path: 30Hz LPF (heavy filtering for attitude estimation)
    // Rate control path: 80Hz LPF (lighter for responsiveness)
    // ESKF: 30Hz LPF（姿勢推定用の強いフィルタ）
    // レート制御: 80Hz LPF（応答性のための軽いフィルタ）
    const float alpha_eskf = 0.32f;   // ~30Hz cutoff at 400Hz
    const float alpha_rate = 0.67f;   // ~80Hz cutoff at 400Hz
    Vec3 gyro_eskf_lpf = {}, accel_eskf_lpf = {};
    Vec3 gyro_rate_lpf = {};
    bool lpf_initialized = false;

    // Calibration accumulators / キャリブレーション累積
    Vec3 cal_accel_sum = {}, cal_gyro_sum = {};
    int cal_count = 0;

    // --- CSV header (stdout) ---
    printf("time,true_x,true_y,true_z,true_vz,"
           "roll,pitch,yaw,"
           "eskf_x,eskf_y,eskf_z,eskf_vz,"
           "eskf_roll,eskf_pitch,eskf_yaw,"
           "thrust,m1,m2,m3,m4,"
           "accel_z,gyro_x,"
           "raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,"
           "flt_ax,flt_ay,flt_az,flt_gx,flt_gy,flt_gz,"
           "duty_avg\n");

    // =================================================================
    // Simulation loop
    // シミュレーションループ
    // =================================================================

    for (int step = 0; step < steps; step++) {
        float t = step * dt;
        float zero_cmd[4] = {0, 0, 0, 0};

        // --- Sensors (always available after Phase1) ---
        // --- センサ（Phase1後は常に利用可能）---
        SimSensors sens = quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 accel_raw(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro_raw(sens.gyro.x, sens.gyro.y, sens.gyro.z);

        // Apply LPF to IMU (matches firmware filter pipeline)
        // IMUにLPFを適用（ファームのフィルタパイプラインに対応）
        if (!lpf_initialized) {
            gyro_eskf_lpf = gyro_raw;
            accel_eskf_lpf = accel_raw;
            gyro_rate_lpf = gyro_raw;
            lpf_initialized = true;
        }
        gyro_eskf_lpf = gyro_eskf_lpf * (1.0f - alpha_eskf) + gyro_raw * alpha_eskf;
        accel_eskf_lpf = accel_eskf_lpf * (1.0f - alpha_eskf) + accel_raw * alpha_eskf;
        gyro_rate_lpf = gyro_rate_lpf * (1.0f - alpha_rate) + gyro_raw * alpha_rate;
        Vec3 accel = accel_eskf_lpf;   // ESKF uses 30Hz filtered
        Vec3 gyro = gyro_eskf_lpf;     // ESKF uses 30Hz filtered

        // =============================================================
        // Phase 1: Sensor init (t < 1s)
        // フェーズ1: センサ初期化
        // Physics runs but no control, no ESKF
        // 物理は動作するが制御なし、ESKFなし
        // =============================================================
        if (t < T_PHASE1_END) {
            quad.step(zero_cmd, dt);
            // No ESKF, no CSV output during init
            continue;
        }

        // =============================================================
        // Phase 2: Calibration (1s ≤ t < 3s)
        // フェーズ2: キャリブレーション
        // Collect stable sensor data for bias estimation
        // バイアス推定のため安定したセンサデータを収集
        // =============================================================
        if (t < T_PHASE2_END) {
            quad.step(zero_cmd, dt);

            // Accumulate calibration samples
            // キャリブレーションサンプルを蓄積
            if (cal_count < CAL_SAMPLES) {
                cal_accel_sum += accel;
                cal_gyro_sum += gyro;
                cal_count++;
            }

            // At end of Phase 2: initialize ESKF
            // Phase2の終了時: ESKFを初期化
            if (cal_count == CAL_SAMPLES && !eskf_ready) {
                Vec3 accel_avg = cal_accel_sum * (1.0f / cal_count);
                Vec3 gyro_avg = cal_gyro_sum * (1.0f / cal_count);

                fprintf(stderr, "=== Phase 2: Calibration Complete ===\n");
                fprintf(stderr, "Gyro avg:  [%.4f %.4f %.4f] rad/s\n",
                        gyro_avg.x, gyro_avg.y, gyro_avg.z);
                fprintf(stderr, "Accel avg: [%.4f %.4f %.4f] m/s²\n",
                        accel_avg.x, accel_avg.y, accel_avg.z);

                // Compute biases / バイアスを計算
                Vec3 accel_bias(accel_avg.x,
                                accel_avg.y,
                                accel_avg.z - (-qp.gravity));

                fprintf(stderr, "Accel bias: [%.4f %.4f %.4f] m/s²\n",
                        accel_bias.x, accel_bias.y, accel_bias.z);

                // Initialize ESKF (matches firmware Phase 3)
                // ESKF初期化（ファームPhase3に対応）
                eskf.setGyroBias(gyro_avg);
                eskf.setAccelBias(accel_bias);

                // Set initial attitude from gravity vector
                // 重力ベクトルから初期姿勢を設定
                eskf.setAttitudeFromGravity(accel_avg);

                // Shrink bias covariance by 100x (trust calibration)
                // バイアス共分散を1/100に縮小（キャリブレーションを信頼）
                eskf.shrinkBiasCovariance(0.01f);

                // Freeze accel bias during ground phase
                // 地上フェーズ中はaccelバイアスをフリーズ
                eskf.setFreezeAccelBias(true);

                eskf_ready = true;
                fprintf(stderr, "=== Phase 3: ESKF Ready ===\n\n");
            }

            continue;
        }

        // =============================================================
        // IDLE / ARMED / FLYING phases (t ≥ 3s)
        // IDLE / ARMED / FLYING フェーズ
        // =============================================================

        // ARM at T_ARM / ARM時刻
        if (t >= T_ARM && !armed) {
            armed = true;
            fprintf(stderr, "t=%.1f ARM\n", t);
        }

        // --- ESKF predict + observe (runs every step after init) ---
        // --- ESKF予測 + 観測（初期化後は毎ステップ実行）---
        // === Incremental ESKF debug ===
        // === 段階的ESKFデバッグ ===
        // Step 1: predict only (expect drift)
        // Step 2: + updateAccelAttitude (expect attitude stabilization)
        // Step 3: + updateToF (expect altitude tracking)
        // Step 4: + updateToFVelocity (expect velocity tracking)
        if (eskf_ready) {
            eskf.predict(accel, gyro, dt);
            eskf.updateAccelAttitude(accel);
            if (step % tof_div == 0) {
                eskf.updateToF(sens.tof_bottom);
                eskf.updateToFVelocity(sens.tof_bottom, tof_div * dt);
            }
        }

        // --- State estimation selection ---
        // --- 状態推定の選択 ---
        auto true_st = quad.getState();
        Vec3 eskf_pos = eskf.getPosition();
        Vec3 eskf_vel = eskf.getVelocity();
        Vec3 eskf_euler = eskf.getAttitude().to_euler();

        // State transition: ground → airborne (ESKF for pos/vel)
        // 状態遷移: 地上 → 空中（ESKFでpos/vel）
        float true_height = -true_st.position.z;
        bool airborne = true_height > 0.03f && armed && (t > T_ARM + 0.2f);

        Vec3 est_pos, est_vel, est_euler;

        if (airborne) {
            if (!is_flying) {
                is_flying = true;
                eskf.resetPositionVelocity();
                fprintf(stderr, "t=%.2f AIRBORNE (h=%.1fcm)\n",
                        t, true_height * 100);
            }
            // ESKF for pos/vel, true attitude for control
            // ESKFでpos/vel推定、制御には真値姿勢を使用
            // ESKF attitude runs open-loop (not fed back to control)
            // ESKF姿勢はオープンループ推定（制御にフィードバックしない）
            est_pos = eskf_pos;
            est_vel = eskf_vel;
            est_euler = true_st.attitude.to_euler();
        } else {
            est_pos = true_st.position;
            est_vel = true_st.velocity;
            est_euler = true_st.attitude.to_euler();
            eskf.resetPositionVelocity();
        }

        // --- Control ---
        float thrust = 0;
        float torque[3] = {0, 0, 0};

        if (armed) {
            float est_height = -est_pos.z;
            float est_climb = -est_vel.z;

            float vel_sp = alt_p.compute(target_alt - est_height, dt);
            float thrust_corr = alt_v.compute(vel_sp - est_climb, dt);
            thrust = hover_thrust + thrust_corr;

            float rate_sp_r = att_r.compute(0 - est_euler.x, dt);
            float rate_sp_p = att_p.compute(0 - est_euler.y, dt);

            // Rate control uses true angular rate for SIL validation
            // SIL検証用に真値角速度を使用
            // TODO: switch to gyro - bias once ESKF is validated
            torque[0] = rate_r.compute(rate_sp_r - true_st.angular_rate.x, dt);
            torque[1] = rate_p.compute(rate_sp_p - true_st.angular_rate.y, dt);
            torque[2] = rate_y.compute(0 - true_st.angular_rate.z, dt);
        }

        // --- Mixer ---
        float motor_duty[4] = {0, 0, 0, 0};
        if (armed) {
            mixer(thrust, torque, motor_duty, qp.k_thrust);
        }

        // --- Physics step ---
        quad.step(armed ? motor_duty : zero_cmd, dt);

        // --- Debug output ---
        if (step % 400 == 0 && t >= T_PHASE2_END) {
            Vec3 true_euler_dbg = true_st.attitude.to_euler();
            fprintf(stderr, "t=%.1f h_true=%.3f h_eskf=%.3f T=%.3f d=%.3f "
                    "att=[%.1f %.1f]° true=[%.1f %.1f]° err=[%.1f %.1f]° %s\n",
                    t, -true_st.position.z, -eskf_pos.z, thrust, motor_duty[0],
                    eskf_euler.x * 57.3f, eskf_euler.y * 57.3f,
                    true_euler_dbg.x * 57.3f, true_euler_dbg.y * 57.3f,
                    (eskf_euler.x - true_euler_dbg.x) * 57.3f,
                    (eskf_euler.y - true_euler_dbg.y) * 57.3f,
                    is_flying ? "FLY" : (armed ? "ARM" : "IDLE"));
        }

        // --- CSV output (50Hz, after Phase2) ---
        if (step % 8 == 0 && t >= T_PHASE2_END) {
            Vec3 true_euler = true_st.attitude.to_euler();

            printf("%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.3f,%.3f,%.3f,%.3f,"
                   "%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.3f\n",
                   t,
                   true_st.position.x, true_st.position.y,
                   true_st.position.z, true_st.velocity.z,
                   true_euler.x*57.3f, true_euler.y*57.3f, true_euler.z*57.3f,
                   eskf_pos.x, eskf_pos.y, eskf_pos.z, eskf_vel.z,
                   eskf_euler.x*57.3f, eskf_euler.y*57.3f, eskf_euler.z*57.3f,
                   thrust,
                   motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3],
                   sens.accel.z, sens.gyro.x,
                   accel_raw.x, accel_raw.y, accel_raw.z,
                   gyro_raw.x, gyro_raw.y, gyro_raw.z,
                   accel.x, accel.y, accel.z,
                   gyro.x, gyro.y, gyro.z,
                   true_st.avg_duty);
        }
    }

    // --- Final state ---
    fprintf(stderr, "\n=== Final State ===\n");
    quad.printState();
    auto fp = eskf.getPosition();
    fprintf(stderr, "ESKF pos=[%.3f %.3f %.3f]\n", fp.x, fp.y, fp.z);

    return 0;
}
