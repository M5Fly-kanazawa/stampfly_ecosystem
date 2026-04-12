/**
 * @file sil_main.cpp
 * @brief Software-in-the-Loop simulation — fly StampFly on PC
 *        SILシミュレーション — PC上でStampFlyを飛ばす
 *
 * Runs the full pipeline (ESKF + PID + Mixer) with a physics model.
 * No ESP32 hardware needed — pure C++ on host PC.
 *
 * 物理モデル付きで完全なパイプライン（ESKF + PID + ミキサー）を実行。
 * ESP32ハードウェア不要 — ホストPC上の純粋C++。
 *
 * Build: make -f Makefile.sil
 *
 * @design requirements.md §10 — SIL simulator                        [--]
 * @design architecture.md §5 — Main pipeline                         [--]
 */

#include <cstdio>
#include <cmath>

// ESP logging stubs / ESPロギングスタブ
#define ESP_LOGI(tag, fmt, ...) // silent in SIL
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
// X-quad mixer (same as sf_actuator)
// X-quadミキサー（sf_actuatorと同じ）
// =============================================================================

static void mixer(float thrust, const float torque[3], float duty[4])
{
    // M1(FR,CCW) = T - Roll + Pitch + Yaw
    // M2(RR,CW)  = T - Roll - Pitch - Yaw
    // M3(RL,CCW) = T + Roll - Pitch + Yaw
    // M4(FL,CW)  = T + Roll + Pitch - Yaw
    float max_thrust = 0.168f * 4;  // 4 motors max

    float t_norm = thrust / max_thrust;  // Normalize to [0..1]
    float r = torque[0] * 100.0f;  // Scale torque to duty contribution
    float p = torque[1] * 100.0f;
    float y = torque[2] * 100.0f;

    duty[0] = t_norm - r + p + y;
    duty[1] = t_norm - r - p - y;
    duty[2] = t_norm + r - p + y;
    duty[3] = t_norm + r + p - y;

    // Clamp [0..1] / クランプ
    for (int i = 0; i < 4; i++) {
        if (duty[i] < 0) duty[i] = 0;
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// SIL Main
// =============================================================================

int main()
{
    printf("=== StampFly SIL Simulation ===\n\n");

    // Initialize physics model / 物理モデルを初期化
    QuadModel quad;
    QuadParams params;
    quad.init(params);

    // Initialize ESKF / ESKFを初期化
    EskfCore eskf;
    EskfConfig eskf_cfg;
    eskf_cfg.use_tof = true;
    eskf_cfg.use_baro = false;
    eskf_cfg.use_mag = false;
    eskf_cfg.use_flow = false;
    eskf.init(eskf_cfg);

    // Initialize PID controllers / PIDコントローラを初期化
    PID rate_roll, rate_pitch, rate_yaw;
    PID att_roll, att_pitch;
    PID alt_pos, alt_vel;

    // Rate PID gains / レートPIDゲイン
    rate_roll.kp = 1.365e-3f; rate_roll.ti = 0.7f; rate_roll.td = 0.01f;
    rate_roll.output_limit = 5.2e-3f;
    rate_pitch = rate_roll;
    rate_pitch.kp = 1.995e-3f;
    rate_yaw.kp = 5.31e-3f; rate_yaw.ti = 1.6f; rate_yaw.output_limit = 2.2e-3f;

    // Attitude PID / 姿勢PID
    att_roll.kp = 5.0f; att_roll.ti = 4.0f; att_roll.output_limit = 3.0f;
    att_pitch = att_roll;

    // Altitude PID / 高度PID
    alt_pos.kp = 0.6f; alt_pos.ti = 7.0f; alt_pos.output_limit = 0.5f;
    alt_vel.kp = 0.1f; alt_vel.ti = 2.5f; alt_vel.output_limit = 0.15f;

    // Simulation parameters / シミュレーションパラメータ
    const float dt = 0.0025f;       // 400Hz
    const float sim_time = 10.0f;   // 10 seconds
    const int steps = static_cast<int>(sim_time / dt);
    const int tof_divider = 13;     // ~30Hz
    const float hover_thrust = params.mass * params.gravity;  // 0.363N
    const float target_alt = 0.5f;  // Target: 0.5m hover

    printf("Simulation: %.1fs at %.0fHz (%d steps)\n", sim_time, 1.0f/dt, steps);
    printf("Target altitude: %.2fm\n", target_alt);
    printf("Hover thrust: %.4fN\n\n", hover_thrust);

    // CSV header / CSVヘッダー
    printf("time,pos_x,pos_y,pos_z,vel_z,roll,pitch,yaw,"
           "eskf_z,eskf_vz,thrust,m1,m2,m3,m4\n");

    // =========================================================================
    // Simulation loop / シミュレーションループ
    // =========================================================================

    for (int step = 0; step < steps; step++) {
        float t = step * dt;

        // --- Arm at t=1s, start altitude hold ---
        // --- t=1sでARM、高度制御開始 ---
        bool armed = (t >= 1.0f);

        // --- Get simulated sensors ---
        // --- シミュレーテッドセンサを取得 ---
        SimSensors sensors = quad.getSensors();

        // --- ESKF predict ---
        // --- ESKF予測 ---
        Vec3 accel(sensors.accel.x, sensors.accel.y, sensors.accel.z);
        Vec3 gyro(sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
        eskf.predict(accel, gyro, dt);

        // --- ESKF accel attitude update ---
        // --- ESKF加速度姿勢更新 ---
        eskf.updateAccelAttitude(accel);

        // --- ESKF ToF update (30Hz) ---
        // --- ESKF ToF更新（30Hz） ---
        if (step % tof_divider == 0) {
            eskf.updateToF(sensors.tof_bottom);
        }

        // --- Get ESKF state ---
        // --- ESKF状態を取得 ---
        Vec3 est_pos = eskf.getPosition();
        Vec3 est_vel = eskf.getVelocity();
        Quat est_att = eskf.getAttitude();
        Vec3 est_euler = est_att.to_euler();

        // --- Control ---
        // --- 制御 ---
        float thrust = 0;
        float torque[3] = {0, 0, 0};

        if (armed) {
            // Altitude cascade / 高度カスケード
            float alt_error = target_alt - (-est_pos.z);
            float vel_sp = alt_pos.compute(alt_error, dt);
            float thrust_corr = alt_vel.compute(vel_sp - (-est_vel.z), dt);
            thrust = hover_thrust + thrust_corr;

            // Attitude cascade (hold level) / 姿勢カスケード（水平保持）
            float rate_sp_roll  = att_roll.compute(0 - est_euler.x, dt);
            float rate_sp_pitch = att_pitch.compute(0 - est_euler.y, dt);

            // Rate control / レート制御
            torque[0] = rate_roll.compute(rate_sp_roll - sensors.gyro.x, dt);
            torque[1] = rate_pitch.compute(rate_sp_pitch - sensors.gyro.y, dt);
            torque[2] = rate_yaw.compute(0 - sensors.gyro.z, dt);
        }

        // --- Mixer ---
        // --- ミキサー ---
        float motor_duty[4] = {0, 0, 0, 0};
        if (armed) {
            mixer(thrust, torque, motor_duty);
        }

        // --- Physics step ---
        // --- 物理ステップ ---
        quad.step(motor_duty, dt);

        // --- Output (every 20ms = 50Hz) ---
        // --- 出力（20msごと = 50Hz） ---
        if (step % 8 == 0) {
            auto state = quad.getState();
            Vec3 euler = state.attitude.to_euler();
            printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
                   "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f\n",
                   t,
                   state.position.x, state.position.y, state.position.z,
                   state.velocity.z,
                   euler.x * 57.3f, euler.y * 57.3f, euler.z * 57.3f,
                   est_pos.z, est_vel.z,
                   thrust,
                   motor_duty[0], motor_duty[1], motor_duty[2], motor_duty[3]);
        }
    }

    // Final state / 最終状態
    printf("\n=== Final State ===\n");
    quad.printState();

    auto final_pos = eskf.getPosition();
    printf("ESKF pos=[%.3f %.3f %.3f]\n", final_pos.x, final_pos.y, final_pos.z);

    return 0;
}
