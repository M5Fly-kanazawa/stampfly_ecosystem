/**
 * @file flight_scenario_test.cpp
 * @brief Full flight scenario: ground → takeoff → hover → gusts → land
 *        フル飛行シナリオ: 地上 → 離陸 → ホバー → 突風 → 着陸
 *
 * Tests L1-L4 with realistic full sequence including ground contact,
 * motor spinup, takeoff transition, and sustained hover with gusts.
 *
 * L1: True rate + true attitude
 * L2: True rate + true attitude (same, baseline)
 * L3: Noisy sensors, true attitude for control
 * L4: Noisy sensors, ESKF for all estimation
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

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
// Mixer
// =============================================================================

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    float L = 0.023f, kappa = 0.00971f;
    float t = thrust_cmd * 0.25f;
    float r = torque[0] / (4.0f * L);
    float p = torque[1] / (4.0f * L);
    float y = torque[2] / (4.0f * kappa);
    float t1 = t - r + p + y, t2 = t - r - p - y;
    float t3 = t + r - p + y, t4 = t + r + p - y;
    for (int i = 0; i < 4; i++) {
        float ti = (i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if (ti < 0) ti = 0;
        duty[i] = sqrtf(ti / k_thrust);
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// Test levels
// =============================================================================

enum Level { L1=1, L2=2, L3=3, L4=4 };

static void run_scenario(Level level, FILE* csv)
{
    // --- Physics ---
    QuadPhysics quad;
    QuadParams qp;
    ContactParams cp;
    SensorNoiseParams np;

    bool use_noise = (level >= L3);
    if (!use_noise) {
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.gyro_bias_rw = 0; np.accel_bias_rw = 0;
        np.vib_accel_k[0]=np.vib_accel_k[1]=np.vib_accel_k[2]=0;
        np.vib_gyro_k[0]=np.vib_gyro_k[1]=np.vib_gyro_k[2]=0;
        np.tof_noise_base = 0; np.tof_noise_scale = 0;
    }
    quad.init(qp, cp, np);  // Starts on ground

    // --- ESKF ---
    EskfCore eskf;
    bool eskf_ready = false;

    // --- PIDs ---
    PID rate_r, rate_p, rate_y, att_r, att_p, alt_p, alt_v;

    // Tuned PIDs (from parameter sweep)
    // チューニング済みPID（パラメータスイープ結果）
    rate_r.kp = 1.365e-3f; rate_r.ti = 0.7f; rate_r.td = 0.008f;
    rate_r.output_limit = 5.2e-3f; rate_r.eta = 0.125f;
    rate_p = rate_r; rate_p.kp = 1.995e-3f;
    rate_y.kp = 5.31e-3f; rate_y.ti = 1.6f; rate_y.output_limit = 2.2e-3f;

    att_r.kp = 14.0f; att_r.ti = 4.0f; att_r.output_limit = 3.0f;
    att_p = att_r;

    alt_p.kp = 1.2f; alt_p.ti = 3.0f; alt_p.output_limit = 0.5f;
    alt_v.kp = 0.20f; alt_v.ti = 1.0f; alt_v.output_limit = 0.25f;

    // --- LPF ---
    const float alpha_eskf = 0.20f, alpha_rate = 0.67f;
    Vec3 gyro_eskf_lpf={}, accel_eskf_lpf={}, gyro_rate_lpf={};
    bool lpf_init = false;

    // --- Simulation ---
    const float dt = 0.0025f;
    const float sim_time = 16.0f;
    const int steps = static_cast<int>(sim_time / dt);
    const float hover_thrust = qp.mass * qp.gravity;
    const int tof_div = 13;

    // --- Flight phases ---
    // t=0-1:     Ground idle (sensors warm up)
    // t=1-2:     Calibration (collect bias)
    // t=2:       ARM + ESKF init
    // t=2-3.5:   Climb to 50cm
    // t=3.5-11:  Hover with gusts
    // t=11:      Descend command (target → 0)
    // t=~12.5:   Touchdown + motor off
    // t=12.5-16: Ground settling
    const float T_CAL_START = 1.0f;
    const float T_ARM = 2.0f;
    const float T_LAND_CMD = 11.0f;
    const float CRUISE_ALT = 0.5f;
    bool landed = false;

    // Calibration
    Vec3 cal_accel_sum={}, cal_gyro_sum={};
    int cal_count = 0;
    const int CAL_SAMPLES = 400;

    // --- Gust schedule ---
    // Realistic gusts: body-frame force + torque (wind on asymmetric body)
    // 現実的な突風: ボディ座標系の力+トルク（非対称体への風）
    struct Gust { float t, dur; Vec3 force, torque; const char* name; };
    Gust gusts[] = {
        // Gentle gust during climb (tests transition robustness)
        // 上昇中の軽い突風（遷移のロバスト性テスト）
        {2.5f, 0.3f, {0.005f,0,0}, {0,3e-5f,0}, "climb gust"},

        // Moderate side wind during hover
        // ホバー中の中程度の横風
        {4.0f, 0.5f, {0,-0.012f,0}, {6e-5f,0,0}, "side wind"},

        // Strong gust burst
        // 強い突風バースト
        {6.5f, 0.2f, {0.015f,0.01f,0}, {-5e-5f,7e-5f,0}, "strong burst"},

        // Sustained mild turbulence (long duration, small amplitude)
        // 持続的な軽い乱流（長時間、小振幅）
        {8.5f, 1.5f, {0.003f,-0.003f,0}, {2e-5f,-2e-5f,0}, "turbulence"},

        // Final sharp gust
        // 最後の鋭い突風
        {10.5f, 0.15f, {0.02f,0,0.005f}, {0,8e-5f,0}, "sharp gust"},
    };
    const int n_gusts = 5;

    bool armed = false;

    // --- CSV header ---
    fprintf(csv, "time,phase,true_roll,true_pitch,true_alt,"
                 "est_roll,est_pitch,est_alt,"
                 "thrust,gust\n");

    // =================================================================
    // Main loop
    // =================================================================
    for (int step = 0; step < steps; step++) {
        float t = step * dt;
        float zero_cmd[4] = {0,0,0,0};

        // Sensors
        SimSensors sens = quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 accel_raw(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro_raw(sens.gyro.x, sens.gyro.y, sens.gyro.z);

        // LPF
        if (!lpf_init) {
            gyro_eskf_lpf=gyro_raw; accel_eskf_lpf=accel_raw;
            gyro_rate_lpf=gyro_raw; lpf_init=true;
        }
        gyro_eskf_lpf = gyro_eskf_lpf*(1-alpha_eskf) + gyro_raw*alpha_eskf;
        accel_eskf_lpf = accel_eskf_lpf*(1-alpha_eskf) + accel_raw*alpha_eskf;
        gyro_rate_lpf = gyro_rate_lpf*(1-alpha_rate) + gyro_raw*alpha_rate;

        // --- Phase: Idle (t < 1) ---
        if (t < T_CAL_START) {
            quad.step(zero_cmd, dt);
            continue;
        }

        // --- Phase: Calibration (1 ≤ t < 2) ---
        if (t < T_ARM) {
            quad.step(zero_cmd, dt);
            if (cal_count < CAL_SAMPLES) {
                cal_accel_sum += accel_raw;
                cal_gyro_sum += gyro_raw;
                cal_count++;
            }
            if (cal_count == CAL_SAMPLES && !eskf_ready && level == L4) {
                Vec3 accel_avg = cal_accel_sum * (1.0f/cal_count);
                Vec3 gyro_avg = cal_gyro_sum * (1.0f/cal_count);
                EskfConfig cfg;
                cfg.use_tof = true; cfg.use_baro = false;
                cfg.use_mag = false; cfg.use_flow = false;
                cfg.accel_noise = 0.3f;
                cfg.gyro_noise = 0.03f;
                cfg.tof_noise = 0.01f;
                eskf.init(cfg);
                eskf.setGyroBias(gyro_avg);
                eskf.setAccelBias(Vec3(accel_avg.x, accel_avg.y,
                                       accel_avg.z + qp.gravity));
                eskf.setAttitudeFromGravity(accel_avg);
                eskf.shrinkBiasCovariance(0.01f);
                eskf.setFreezeAccelBias(true);
                eskf_ready = true;
            }
            continue;
        }

        // --- ARM ---
        if (!armed) {
            armed = true;
            fprintf(stderr, "  t=%.1f ARM (level %d)\n", t, level);
        }

        // --- ESKF predict + observe (L4 only) ---
        if (level == L4 && eskf_ready) {
            eskf.predict(accel_eskf_lpf, gyro_eskf_lpf, dt);
            eskf.updateAccelAttitude(accel_eskf_lpf);
            if (step % tof_div == 0) {
                eskf.updateToF(sens.tof_bottom);
                eskf.updateToFVelocity(sens.tof_bottom, tof_div*dt);
            }
        }

        // --- True state ---
        auto true_st = quad.getState();
        Vec3 true_euler = true_st.attitude.to_euler();
        float true_height = -true_st.position.z;

        // --- Estimates based on level ---
        float est_roll, est_pitch, est_rate_x, est_rate_y, est_rate_z;
        float est_height, est_climb;

        if (level <= L2) {
            // True everything
            est_roll = true_euler.x; est_pitch = true_euler.y;
            est_rate_x = true_st.angular_rate.x;
            est_rate_y = true_st.angular_rate.y;
            est_rate_z = true_st.angular_rate.z;
            est_height = true_height;
            est_climb = -true_st.velocity.z;
        } else if (level == L3) {
            // True attitude, noisy rate, true altitude
            est_roll = true_euler.x; est_pitch = true_euler.y;
            est_rate_x = gyro_rate_lpf.x;
            est_rate_y = gyro_rate_lpf.y;
            est_rate_z = gyro_rate_lpf.z;
            est_height = true_height;
            est_climb = -true_st.velocity.z;
        } else {
            // ESKF for everything
            Vec3 eskf_euler = eskf.getAttitude().to_euler();
            Vec3 eskf_pos = eskf.getPosition();
            Vec3 eskf_vel = eskf.getVelocity();
            Vec3 gb = eskf.getGyroBias();
            est_roll = eskf_euler.x; est_pitch = eskf_euler.y;
            est_rate_x = gyro_rate_lpf.x - gb.x;
            est_rate_y = gyro_rate_lpf.y - gb.y;
            est_rate_z = gyro_rate_lpf.z - gb.z;
            est_height = -eskf_pos.z;
            est_climb = -eskf_vel.z;

            // Ground phase: hold ESKF pos/vel until airborne
            // 地上フェーズ: 空中になるまでESKF pos/velを保持
            if (true_height < 0.03f) {
                eskf.resetPositionVelocity();
                est_height = true_height;
                est_climb = -true_st.velocity.z;
            }
        }

        // --- Target altitude (landing ramp) ---
        // --- 目標高度（着陸ランプ）---
        float target_alt;
        if (t < T_LAND_CMD) {
            target_alt = CRUISE_ALT;
        } else {
            // Linear ramp down: 50cm → 0cm over 2s
            // 線形降下: 50cm → 0cm を 2秒で
            float ramp = 1.0f - (t - T_LAND_CMD) / 2.0f;
            if (ramp < 0) ramp = 0;
            target_alt = CRUISE_ALT * ramp;
        }

        // Landing detection: on ground + low thrust → disarm
        // 着陸検出: 地上 + 低推力 → ディスアーム
        if (t > T_LAND_CMD + 2.5f && true_height < 0.02f && !landed) {
            landed = true;
            fprintf(stderr, "  t=%.1f LANDED (level %d)\n", t, level);
        }

        // --- Control ---
        float thrust = 0;
        float torque[3] = {0,0,0};

        if (!landed) {
            // Altitude + attitude cascade
            float vel_sp = alt_p.compute(target_alt - est_height, dt);
            float thrust_corr = alt_v.compute(vel_sp - est_climb, dt);
            thrust = hover_thrust + thrust_corr;

            float rate_sp_r = att_r.compute(0 - est_roll, dt);
            float rate_sp_p = att_p.compute(0 - est_pitch, dt);
            torque[0] = rate_r.compute(rate_sp_r - est_rate_x, dt);
            torque[1] = rate_p.compute(rate_sp_p - est_rate_y, dt);
            torque[2] = rate_y.compute(0 - est_rate_z, dt);
        }

        // --- Gust ---
        float gust_flag = 0;
        Vec3 gf={}, gt={};
        for (int gi = 0; gi < n_gusts; gi++) {
            if (t >= gusts[gi].t && t < gusts[gi].t + gusts[gi].dur) {
                gf += gusts[gi].force; gt += gusts[gi].torque;
                gust_flag = 1;
            }
        }
        quad.setExternalForceBody(gf, gt);

        // --- Mixer + physics ---
        float motor_duty[4] = {0,0,0,0};
        if (!landed) {
            mixer(thrust, torque, motor_duty, qp.k_thrust);
        }
        quad.step(motor_duty, dt);

        // --- CSV (50Hz) ---
        if (step % 8 == 0) {
            // 0=idle/cal, 1=ground, 2=climb, 3=hover, 4=descend, 5=landed
            int phase = landed ? 5 :
                        (t < T_ARM) ? 0 :
                        (true_height < 0.03f) ? 1 :
                        (t < T_ARM + 1.5f) ? 2 :
                        (t >= T_LAND_CMD) ? 4 : 3;
            fprintf(csv, "%.3f,%d,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.4f,%.0f\n",
                    t, phase,
                    true_euler.x*57.3f, true_euler.y*57.3f, true_height,
                    est_roll*57.3f, est_pitch*57.3f, est_height,
                    thrust, gust_flag);
        }
    }

    // Final state
    auto fs = quad.getState();
    fprintf(stderr, "  Final: alt=%.3fm roll=%.1f° pitch=%.1f°\n",
            -fs.position.z, fs.attitude.to_euler().x*57.3f,
            fs.attitude.to_euler().y*57.3f);
}

// =============================================================================
// Main
// =============================================================================

int main()
{
    const char* names[] = {
        "L1: Rate+Att (true)",
        "L2: Rate+Att (true)",
        "L3: Noisy gyro",
        "L4: Full ESKF",
    };
    const char* files[] = {
        "scenario_L1.csv", "scenario_L2.csv",
        "scenario_L3.csv", "scenario_L4.csv",
    };
    Level levels[] = { L1, L2, L3, L4 };

    fprintf(stderr, "=== Full Flight Scenario: Ground → Takeoff → Hover → Gusts → Land ===\n");
    fprintf(stderr, "Timeline: idle(0-1) → cal(1-2) → ARM(2) → climb → hover+gusts → descend(11) → land\n");
    fprintf(stderr, "Gusts: climb(2.5), side_wind(4), burst(6.5), turbulence(8.5), sharp(10.5)\n\n");

    for (int i = 0; i < 4; i++) {
        srand(42);
        fprintf(stderr, "%s:\n", names[i]);
        FILE* csv = fopen(files[i], "w");
        run_scenario(levels[i], csv);
        fclose(csv);
    }

    // Summary from CSVs
    fprintf(stderr, "\n%-25s %8s %8s %8s %8s %8s\n",
            "Test", "max_r", "max_p", "rms_r", "rms_p", "alt_rms");
    fprintf(stderr, "----------------------------------------------------------------------\n");

    for (int i = 0; i < 4; i++) {
        FILE* f = fopen(files[i], "r");
        char hdr[512]; fgets(hdr, sizeof(hdr), f);
        float max_r=0, max_p=0, sum_r2=0, sum_p2=0, sum_alt2=0;
        int n=0;
        float t_f, phase, tr, tp, ta, er, ep, ea, th, gf;
        while (fscanf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                       &t_f,&phase,&tr,&tp,&ta,&er,&ep,&ea,&th,&gf) == 10) {
            if (t_f < 3.5f) continue;  // Stats after settling
            if (fabsf(tr) > max_r) max_r = fabsf(tr);
            if (fabsf(tp) > max_p) max_p = fabsf(tp);
            sum_r2 += tr*tr; sum_p2 += tp*tp;
            sum_alt2 += (ta - 0.5f)*(ta - 0.5f);
            n++;
        }
        fclose(f);
        fprintf(stderr, "%-25s %7.2f° %7.2f° %7.2f° %7.2f° %7.1fmm\n",
                names[i], max_r, max_p,
                sqrtf(sum_r2/n), sqrtf(sum_p2/n),
                sqrtf(sum_alt2/n)*1000);
    }

    return 0;
}
