/**
 * @file control_test.cpp
 * @brief Progressive control validation — rate → attitude → noise → ESKF
 *        段階的制御検証 — レート → 姿勢 → ノイズ → ESKF
 *
 * 4 test levels:
 *   Level 1: Rate control with true angular rate (ω → 0)
 *   Level 2: Attitude control with true euler (θ → 0)
 *   Level 3: Same as L2 but with noisy sensors
 *   Level 4: Same as L3 but using ESKF estimates
 *
 * Each level applies periodic disturbance torque impulses.
 * Output: CSV per level for comparison plotting.
 *
 * Build: g++ -std=c++17 -O2 -I. -I../test -I../components/sf_math/include
 *        -I../components/sf_estimator_eskf/include
 *        -I../components/sf_controller_pid/include
 *        -I../components/sf_core/include -I../components/sf_estimator/include
 *        -I../components/sf_controller/include -I../components/sf_state/include
 *        -o control_test control_test.cpp
 *        ../components/sf_estimator_eskf/eskf_core.cpp -lm
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

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
// Mixer (same as sil_main)
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

    float t1 = t - r + p + y;
    float t2 = t - r - p - y;
    float t3 = t + r - p + y;
    float t4 = t + r + p - y;

    for (int i = 0; i < 4; i++) {
        float ti = (i == 0) ? t1 : (i == 1) ? t2 : (i == 2) ? t3 : t4;
        if (ti < 0) ti = 0;
        duty[i] = sqrtf(ti / k_thrust);
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// Test levels
// =============================================================================

enum TestLevel {
    LEVEL_RATE_TRUE = 1,    // Rate control, true angular rate
    LEVEL_ATT_TRUE  = 2,    // Attitude control, true euler + true rate
    LEVEL_ATT_NOISY = 3,    // Attitude control, noisy sensors
    LEVEL_ATT_ESKF  = 4,    // Attitude control, ESKF estimates
};

struct TestResult {
    float max_roll_err;
    float max_pitch_err;
    float rms_roll_err;
    float rms_pitch_err;
    float settle_time;  // time to return within 2deg after disturbance
};

static TestResult run_test(TestLevel level, FILE* csv)
{
    // --- Physics ---
    QuadPhysics quad;
    QuadParams qp;
    ContactParams cp;
    SensorNoiseParams np;

    bool use_noise = (level >= LEVEL_ATT_NOISY);
    if (!use_noise) {
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.gyro_bias_rw = 0; np.accel_bias_rw = 0;
        np.vib_accel_k[0] = np.vib_accel_k[1] = np.vib_accel_k[2] = 0;
        np.vib_gyro_k[0] = np.vib_gyro_k[1] = np.vib_gyro_k[2] = 0;
        np.tof_noise_base = 0; np.tof_noise_scale = 0;
    }
    quad.init(qp, cp, np);

    // Start airborne at h=0.5m with motors at hover speed
    // 空中スタート h=0.5m、モーターはホバー速度で初期化
    auto& init_state = const_cast<QuadState&>(quad.getState());
    init_state.position.z = -0.5f;
    float hover_duty = sqrtf(qp.mass * qp.gravity / (4.0f * qp.k_thrust));
    for (int i = 0; i < 4; i++) init_state.motor_speed[i] = hover_duty;

    // --- ESKF (Level 4 only) ---
    EskfCore eskf;
    if (level == LEVEL_ATT_ESKF) {
        EskfConfig cfg;
        cfg.use_tof = true;
        cfg.use_baro = false;
        cfg.use_mag = false;
        cfg.use_flow = false;
        cfg.accel_noise = 0.3f;
        cfg.gyro_noise = 0.009655f;
        cfg.tof_noise = 0.01f;
        eskf.init(cfg);
    }

    // --- PIDs ---
    PID rate_r, rate_p, rate_y;
    PID att_r, att_p;

    rate_r.kp = 1.365e-3f; rate_r.ti = 0.7f; rate_r.td = 0.01f;
    rate_r.output_limit = 5.2e-3f; rate_r.eta = 0.125f;
    rate_p = rate_r; rate_p.kp = 1.995e-3f;
    rate_y.kp = 5.31e-3f; rate_y.ti = 1.6f; rate_y.output_limit = 2.2e-3f;

    att_r.kp = 5.0f; att_r.ti = 4.0f; att_r.output_limit = 3.0f;
    att_p = att_r;

    // --- LPF for noisy levels ---
    const float alpha_eskf = 0.32f;
    const float alpha_rate = 0.67f;
    Vec3 gyro_eskf_lpf = {}, accel_eskf_lpf = {};
    Vec3 gyro_rate_lpf = {};
    bool lpf_init = false;

    // --- Simulation parameters ---
    const float dt = 0.0025f;
    const float sim_time = 6.0f;
    const int steps = static_cast<int>(sim_time / dt);
    const float hover_thrust = qp.mass * qp.gravity;
    const int tof_div = 13;

    // --- Calibration for ESKF (Level 4) ---
    if (level == LEVEL_ATT_ESKF) {
        Vec3 cal_accel_sum = {}, cal_gyro_sum = {};
        for (int i = 0; i < 400; i++) {
            float zero[4] = {0,0,0,0};
            quad.step(zero, dt);
            quad.updateBiases(dt);
            SimSensors s = quad.getSensors(dt);
            cal_accel_sum += Vec3(s.accel.x, s.accel.y, s.accel.z);
            cal_gyro_sum += Vec3(s.gyro.x, s.gyro.y, s.gyro.z);
        }
        Vec3 accel_avg = cal_accel_sum * (1.0f/400);
        Vec3 gyro_avg = cal_gyro_sum * (1.0f/400);
        eskf.setGyroBias(gyro_avg);
        eskf.setAccelBias(Vec3(accel_avg.x, accel_avg.y, accel_avg.z + qp.gravity));
        eskf.setAttitudeFromGravity(accel_avg);
        eskf.shrinkBiasCovariance(0.01f);
        eskf.setFreezeAccelBias(true);
    }

    // --- Gust disturbance schedule ---
    // Wind force + torque applied to body frame via physics engine
    // ボディ座標系の風力+トルクを物理エンジンに印加
    //
    // Model: lateral wind hits drone body above CG → force + torque
    // モデル: 横風が重心より上に当たる → 並進力 + 回転トルク
    //   Force: drag-like lateral push (~10mN ≈ 3% of weight)
    //   Torque: force × moment arm (~10mm above CG)
    //     5e-5 Nm on Ixx=5e-6 → α = 10 rad/s², in 200ms: Δω ≈ 2 rad/s
    struct Gust {
        float t, duration;
        Vec3 force;    // Body-frame force [N]
        Vec3 torque;   // Body-frame torque [Nm]
    };
    Gust gusts[] = {
        {1.0f, 0.2f,  {0.01f,  0.0f,  0.0f}, {0.0f,  5e-5f,  0.0f}},  // +X wind → pitch
        {2.5f, 0.2f,  {0.0f, -0.015f, 0.0f}, {7e-5f, 0.0f,   0.0f}},  // -Y wind → roll
        {4.0f, 0.3f,  {0.008f, 0.008f, 0.0f},{-4e-5f,4e-5f,  0.0f}},  // Combined
    };
    const int n_gusts = 3;

    // --- CSV header ---
    fprintf(csv, "time,true_roll,true_pitch,true_rate_x,true_rate_y,"
                 "est_roll,est_pitch,est_rate_x,est_rate_y,"
                 "torque_x,torque_y,disturbance\n");

    // --- Stats ---
    float sum_roll_err2 = 0, sum_pitch_err2 = 0;
    float max_roll_err = 0, max_pitch_err = 0;
    int stat_count = 0;

    // --- Main loop ---
    for (int step = 0; step < steps; step++) {
        float t = step * dt;

        // Sensors
        SimSensors sens = quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 accel_raw(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro_raw(sens.gyro.x, sens.gyro.y, sens.gyro.z);

        // LPF
        if (!lpf_init) {
            gyro_eskf_lpf = gyro_raw; accel_eskf_lpf = accel_raw;
            gyro_rate_lpf = gyro_raw; lpf_init = true;
        }
        gyro_eskf_lpf = gyro_eskf_lpf * (1-alpha_eskf) + gyro_raw * alpha_eskf;
        accel_eskf_lpf = accel_eskf_lpf * (1-alpha_eskf) + accel_raw * alpha_eskf;
        gyro_rate_lpf = gyro_rate_lpf * (1-alpha_rate) + gyro_raw * alpha_rate;

        // ESKF predict + observe (Level 4)
        if (level == LEVEL_ATT_ESKF) {
            eskf.predict(accel_eskf_lpf, gyro_eskf_lpf, dt);
            eskf.updateAccelAttitude(accel_eskf_lpf);
            if (step % tof_div == 0) {
                eskf.updateToF(sens.tof_bottom);
                eskf.updateToFVelocity(sens.tof_bottom, tof_div * dt);
            }
        }

        // True state
        auto true_st = quad.getState();
        Vec3 true_euler = true_st.attitude.to_euler();

        // Select estimates based on level
        float est_roll, est_pitch, est_rate_x, est_rate_y, est_rate_z;

        switch (level) {
        case LEVEL_RATE_TRUE:
            // Rate control only — no attitude loop
            est_roll = 0; est_pitch = 0;  // not used
            est_rate_x = true_st.angular_rate.x;
            est_rate_y = true_st.angular_rate.y;
            est_rate_z = true_st.angular_rate.z;
            break;
        case LEVEL_ATT_TRUE:
            est_roll = true_euler.x;
            est_pitch = true_euler.y;
            est_rate_x = true_st.angular_rate.x;
            est_rate_y = true_st.angular_rate.y;
            est_rate_z = true_st.angular_rate.z;
            break;
        case LEVEL_ATT_NOISY:
            est_roll = true_euler.x;   // attitude still true
            est_pitch = true_euler.y;
            // Rate from filtered gyro - true bias (known in sim)
            est_rate_x = gyro_rate_lpf.x;
            est_rate_y = gyro_rate_lpf.y;
            est_rate_z = gyro_rate_lpf.z;
            break;
        case LEVEL_ATT_ESKF: {
            Vec3 eskf_euler = eskf.getAttitude().to_euler();
            Vec3 eskf_gbias = eskf.getGyroBias();
            est_roll = eskf_euler.x;
            est_pitch = eskf_euler.y;
            est_rate_x = gyro_rate_lpf.x - eskf_gbias.x;
            est_rate_y = gyro_rate_lpf.y - eskf_gbias.y;
            est_rate_z = gyro_rate_lpf.z - eskf_gbias.z;
            break;
        }
        }

        // --- Control ---
        float torque[3] = {0, 0, 0};

        if (level == LEVEL_RATE_TRUE) {
            // Rate-only: command ω → 0
            torque[0] = rate_r.compute(0 - est_rate_x, dt);
            torque[1] = rate_p.compute(0 - est_rate_y, dt);
            torque[2] = rate_y.compute(0 - est_rate_z, dt);
        } else {
            // Attitude + rate cascade
            float rate_sp_r = att_r.compute(0 - est_roll, dt);
            float rate_sp_p = att_p.compute(0 - est_pitch, dt);
            torque[0] = rate_r.compute(rate_sp_r - est_rate_x, dt);
            torque[1] = rate_p.compute(rate_sp_p - est_rate_y, dt);
            torque[2] = rate_y.compute(0 - est_rate_z, dt);
        }

        // --- Gust disturbance (body-frame force on physics) ---
        // --- 突風外乱（ボディ座標系の力を物理エンジンに印加）---
        float dist_flag = 0;
        Vec3 gust_force = {}, gust_torque = {};
        for (int gi = 0; gi < n_gusts; gi++) {
            if (t >= gusts[gi].t && t < gusts[gi].t + gusts[gi].duration) {
                gust_force += gusts[gi].force;
                gust_torque += gusts[gi].torque;
                dist_flag = 1;
            }
        }
        quad.setExternalForceBody(gust_force, gust_torque);

        // --- Mixer + step ---
        float motor_duty[4];
        mixer(hover_thrust, torque, motor_duty, qp.k_thrust);

        // Debug: print single-step trace at first disturbance
        if (step >= 400 && step < 420 && level == LEVEL_RATE_TRUE) {
            auto st = quad.getState();
            fprintf(stderr, "s=%d t=%.4f ω=%.4f τ_cmd=%.6f d=[%.4f %.4f %.4f %.4f] "
                    "ms=[%.4f %.4f %.4f %.4f]\n",
                    step, t, st.angular_rate.x, torque[0],
                    motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3],
                    st.motor_speed[0], st.motor_speed[1], st.motor_speed[2], st.motor_speed[3]);
        }

        quad.step(motor_duty, dt);

        // --- Stats (after t=0.5s settling) ---
        if (t > 0.5f) {
            float roll_err = true_euler.x * 57.3f;
            float pitch_err = true_euler.y * 57.3f;
            sum_roll_err2 += roll_err * roll_err;
            sum_pitch_err2 += pitch_err * pitch_err;
            if (fabsf(roll_err) > max_roll_err) max_roll_err = fabsf(roll_err);
            if (fabsf(pitch_err) > max_pitch_err) max_pitch_err = fabsf(pitch_err);
            stat_count++;
        }

        // --- CSV (50Hz) ---
        if (step % 8 == 0) {
            fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.0f\n",
                    t,
                    true_euler.x * 57.3f, true_euler.y * 57.3f,
                    true_st.angular_rate.x * 57.3f, true_st.angular_rate.y * 57.3f,
                    est_roll * 57.3f, est_pitch * 57.3f,
                    est_rate_x * 57.3f, est_rate_y * 57.3f,
                    torque[0], torque[1], dist_flag);
        }
    }

    TestResult res;
    res.max_roll_err = max_roll_err;
    res.max_pitch_err = max_pitch_err;
    res.rms_roll_err = sqrtf(sum_roll_err2 / stat_count);
    res.rms_pitch_err = sqrtf(sum_pitch_err2 / stat_count);
    return res;
}

// =============================================================================
// Main
// =============================================================================

int main()
{
    const char* names[] = {
        "L1: Rate ctrl (true rate)",
        "L2: Att ctrl (true state)",
        "L3: Att ctrl (noisy gyro)",
        "L4: Att ctrl (ESKF)",
    };
    const char* files[] = {
        "control_L1_rate.csv",
        "control_L2_att_true.csv",
        "control_L3_att_noisy.csv",
        "control_L4_att_eskf.csv",
    };
    TestLevel levels[] = {
        LEVEL_RATE_TRUE, LEVEL_ATT_TRUE, LEVEL_ATT_NOISY, LEVEL_ATT_ESKF
    };

    fprintf(stderr, "=== Progressive Control Validation ===\n\n");
    fprintf(stderr, "Gusts: +X 10mN@1.0s(200ms), -Y 15mN@2.5s(200ms), "
                    "combined@4.0s(300ms)\n\n");

    fprintf(stderr, "%-30s %10s %10s %10s %10s\n",
            "Test", "max_roll", "max_pitch", "rms_roll", "rms_pitch");
    fprintf(stderr, "%s\n", "-----------------------------------------------------------------------");

    for (int i = 0; i < 4; i++) {
        srand(42);  // Deterministic noise
        FILE* csv = fopen(files[i], "w");
        TestResult res = run_test(levels[i], csv);
        fclose(csv);

        fprintf(stderr, "%-30s %9.2f° %9.2f° %9.2f° %9.2f°\n",
                names[i],
                res.max_roll_err, res.max_pitch_err,
                res.rms_roll_err, res.rms_pitch_err);
    }

    fprintf(stderr, "\nCSV files saved for plotting.\n");
    return 0;
}
