/**
 * @file sil_main.cpp
 * @brief SIL Simulation — fly StampFly on PC with visualization
 *        SILシミュレーション — PC上でStampFlyを飛ばして可視化
 *
 * Full pipeline: Physics → Sensors(+noise) → ESKF → PID → Mixer → Motors
 * Output: CSV to stdout for plotting with plot_flight.py
 *
 * Build & run: make run
 * Save CSV:    make csv
 * Plot:        make plot
 *
 * @design requirements.md §10 — SIL simulator                        [--]
 * @design noise_and_vibration_model.md — Noise/vibration model        [--]
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
#include "quad_model.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;

// =============================================================================
// Mixer: thrust/torque → motor duty (matches sf_actuator)
// ミキサー: 推力/トルク → モーターduty（sf_actuatorと一致）
//
// X-quad: M1(FR,CCW) M2(RR,CW) M3(RL,CCW) M4(FL,CW)
// With quadratic thrust: duty = sqrt(thrust_normalized)
// =============================================================================

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    // Solve for individual motor thrusts
    // 個別モーター推力を求める
    float L = 0.023f;
    float kappa = 0.00971f;

    // Thrust allocation (same signs as quad_model.hpp)
    // 推力配分（quad_model.hppと同じ符号）
    float t = thrust_cmd * 0.25f;
    float r = torque[0] / (4.0f * L);
    float p = torque[1] / (4.0f * L);
    float y = torque[2] / (4.0f * kappa);

    float t1 = t - r + p + y;  // M1 FR CCW
    float t2 = t - r - p - y;  // M2 RR CW
    float t3 = t + r - p + y;  // M3 RL CCW
    float t4 = t + r + p - y;  // M4 FL CW

    // Thrust → duty: inverse of quadratic model (duty = sqrt(T/k))
    // 推力 → duty: 二次モデルの逆（duty = sqrt(T/k)）
    for (int i = 0; i < 4; i++) {
        float ti = (i == 0) ? t1 : (i == 1) ? t2 : (i == 2) ? t3 : t4;
        if (ti < 0) ti = 0;
        duty[i] = sqrtf(ti / k_thrust);
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// Main Simulation
// =============================================================================

int main()
{
    srand(static_cast<unsigned>(time(nullptr)));

    fprintf(stderr, "=== StampFly SIL Simulation ===\n");

    // --- Initialize physics model ---
    QuadModel quad;
    QuadParams qp;
    SensorNoiseParams np;
    // Full noise model (Phase 2: static + bias + vibration)
    // フルノイズモデル（Phase 2: 静的 + バイアス + 振動）
    // All parameters from real flight log analysis (72 logs)
    // 全パラメータは実飛行ログ解析（72ログ）から導出
    quad.init(qp, np);

    // --- Initialize ESKF ---
    EskfCore eskf;
    EskfConfig cfg;
    cfg.use_tof = true;
    cfg.use_baro = false;
    cfg.use_mag = false;
    cfg.use_flow = false;
    cfg.accel_noise = 0.3f;      // Vehicle-tuned value
    cfg.gyro_noise = 0.009655f;  // Vehicle-tuned value
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
    const float sim_time = 10.0f;
    const int steps = static_cast<int>(sim_time / dt);
    const int tof_div = 13;  // ~30Hz
    const float hover_thrust = qp.mass * qp.gravity;
    const float target_alt = 0.5f;

    fprintf(stderr, "Duration: %.1fs at %.0fHz (%d steps)\n", sim_time, 1/dt, steps);
    fprintf(stderr, "Target alt: %.2fm, Hover thrust: %.4fN\n\n", target_alt, hover_thrust);

    // --- Startup calibration: collect bias from static sensors ---
    // --- 起動キャリブレーション: 静的センサからバイアスを収集 ---
    fprintf(stderr, "Calibrating (100 samples)...\n");
    Vec3 accel_sum = {}, gyro_sum = {};
    for (int i = 0; i < 100; i++) {
        float zero_cmd[4] = {0, 0, 0, 0};
        quad.step(zero_cmd, dt);
        quad.updateBiases(dt);
        SimSensors cal = quad.getSensors(dt);
        accel_sum += Vec3(cal.accel.x, cal.accel.y, cal.accel.z);
        gyro_sum += Vec3(cal.gyro.x, cal.gyro.y, cal.gyro.z);
    }

    // Set ESKF initial biases from calibration
    // キャリブレーションからESKF初期バイアスを設定
    // Gyro bias = average gyro reading (should be near zero + bias)
    // Accel bias: static accel should be [0, 0, -g] in body
    // So accel_bias = measured - expected = measured - [0, 0, -g]
    Vec3 gyro_avg = gyro_sum * (1.0f / 100.0f);
    Vec3 accel_avg = accel_sum * (1.0f / 100.0f);

    fprintf(stderr, "Gyro bias:  [%.4f %.4f %.4f] rad/s\n",
            gyro_avg.x, gyro_avg.y, gyro_avg.z);
    fprintf(stderr, "Accel avg:  [%.4f %.4f %.4f] m/s²\n",
            accel_avg.x, accel_avg.y, accel_avg.z);
    Vec3 accel_bias(accel_avg.x, accel_avg.y, accel_avg.z + qp.gravity);
    fprintf(stderr, "Accel bias: [%.4f %.4f %.4f] m/s² (avg - [0,0,-g])\n",
            accel_bias.x, accel_bias.y, accel_bias.z);

    // Set calibrated biases to ESKF
    // キャリブレーション値をESKFに設定
    eskf.setGyroBias(gyro_avg);
    eskf.setAccelBias(accel_bias);

    // --- CSV header (stdout) ---
    printf("time,true_x,true_y,true_z,true_vz,"
           "roll,pitch,yaw,"
           "eskf_x,eskf_y,eskf_z,eskf_vz,"
           "eskf_roll,eskf_pitch,eskf_yaw,"
           "thrust,m1,m2,m3,m4,"
           "accel_z,gyro_x\n");

    // =================================================================
    // Simulation loop
    // シミュレーションループ
    // =================================================================

    for (int step = 0; step < steps; step++) {
        float t = step * dt;

        bool armed = (t >= 1.0f);

        // --- Sensors ---
        SimSensors sens = quad.getSensors(dt);
        quad.updateBiases(dt);

        // --- ESKF prediction and updates ---
        Vec3 accel(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro(sens.gyro.x, sens.gyro.y, sens.gyro.z);
        eskf.predict(accel, gyro, dt);
        eskf.updateAccelAttitude(accel);
        if (step % tof_div == 0) {
            eskf.updateToF(sens.tof_bottom);
        }

        // --- Get states ---
        auto true_st = quad.getState();
        Vec3 eskf_pos = eskf.getPosition();
        Vec3 eskf_vel = eskf.getVelocity();
        Vec3 eskf_euler = eskf.getAttitude().to_euler();

        // ESKF-controlled flight: use ESKF for position/velocity
        // ESKF制御飛行: 位置/速度にESKFを使用
        // Rate control still uses true angular rate (gyro bias not corrected yet)
        // レート制御は引き続き真の角速度を使用（ジャイロバイアス未補正）
        Vec3 est_pos = eskf_pos;
        Vec3 est_vel = eskf_vel;
        Vec3 est_euler = eskf_euler;

        // --- Control ---
        float thrust = 0;
        float torque[3] = {0, 0, 0};

        if (armed) {
            bool on_ground = true_st.position.z >= -0.01f;

            // Takeoff: open-loop thrust until well airborne (>10cm)
            // 離陸: 十分に空中（>10cm）になるまでオープンループ推力
            float true_height = -true_st.position.z;
            bool takeoff_phase = true_height < 0.10f;

            if (takeoff_phase) {
                thrust = hover_thrust * 1.2f;  // 20% above hover for takeoff
            } else {
                // Altitude cascade (airborne) / 高度カスケード（空中）
                float est_height = -est_pos.z;
                float est_climb = -est_vel.z;
                float vel_sp = alt_p.compute(target_alt - est_height, dt);
                float thrust_corr = alt_v.compute(vel_sp - est_climb, dt);
                thrust = hover_thrust + thrust_corr;
            }
            if (!takeoff_phase) {
                // Attitude cascade (stable flight only, not during takeoff)
                // 姿勢カスケード（安定飛行時のみ、離陸中は除く）
                float rate_sp_r = att_r.compute(0 - est_euler.x, dt);
                float rate_sp_p = att_p.compute(0 - est_euler.y, dt);

                torque[0] = rate_r.compute(rate_sp_r - true_st.angular_rate.x, dt);
                torque[1] = rate_p.compute(rate_sp_p - true_st.angular_rate.y, dt);
                torque[2] = rate_y.compute(0 - true_st.angular_rate.z, dt);
            }
            // During takeoff: torque = 0 (thrust only, motors balanced)
            // 離陸中: トルク = 0（推力のみ、モーター均等）
            // On ground: torque = 0, thrust goes equally to all motors
            // 地上: トルク=0、推力は全モーター均等
        }

        // --- Mixer ---
        float motor_duty[4] = {0, 0, 0, 0};
        if (armed) {
            mixer(thrust, torque, motor_duty, qp.k_thrust);
        }

        // Debug: print periodically
        if (step >= 400 && step % 100 == 0) {
            auto dst = quad.getState();
            fprintf(stderr, "t=%.1f true_z=%.4f eskf_z=%.4f T=%.3f d=%.3f\n",
                    t, dst.position.z, est_pos.z, thrust, motor_duty[0]);
        }

        // Debug: physics state at ARM
        if (step >= 400 && step < 410) {
            auto ds = quad.getState();
            fprintf(stderr, "s=%d pos_z=%.6f vel_z=%.6f m=[%.3f %.3f %.3f %.3f] avg=%.3f\n",
                    step, ds.position.z, ds.velocity.z,
                    motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3],
                    ds.avg_duty);
        }

        // --- Physics step ---
        quad.step(motor_duty, dt);

        // --- CSV output (50Hz) ---
        if (step % 8 == 0) {
            auto st = quad.getState();
            Vec3 true_euler = st.attitude.to_euler();

            printf("%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.3f,%.3f,%.3f,%.3f,"
                   "%.4f,%.4f\n",
                   t,
                   st.position.x, st.position.y, st.position.z, st.velocity.z,
                   true_euler.x*57.3f, true_euler.y*57.3f, true_euler.z*57.3f,
                   eskf_pos.x, eskf_pos.y, eskf_pos.z, eskf_vel.z,
                   eskf_euler.x*57.3f, eskf_euler.y*57.3f, eskf_euler.z*57.3f,
                   thrust,
                   motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3],
                   sens.accel.z, sens.gyro.x);
        }
    }

    // --- Final state ---
    fprintf(stderr, "\n=== Final State ===\n");
    quad.printState();
    auto fp = eskf.getPosition();
    fprintf(stderr, "ESKF pos=[%.3f %.3f %.3f]\n", fp.x, fp.y, fp.z);

    return 0;
}
