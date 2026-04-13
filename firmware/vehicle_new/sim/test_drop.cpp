/**
 * @file test_drop.cpp
 * @brief Drop test — verify contact dynamics (bounce, settle, flip)
 *        落下テスト — 接触ダイナミクス検証（バウンド、静止、転倒）
 *
 * Scenarios:
 * 1. Level drop from 0.5m → bounce → settle
 * 2. Tilted drop → bounce → flip → settle
 * 3. Drop with motors on → hover recovery
 */

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>

#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

#include "sf_math.hpp"
#include "quad_physics.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;

void run_scenario(const char* name, QuadPhysics& quad, float sim_time, float dt)
{
    int steps = static_cast<int>(sim_time / dt);
    float zero_cmd[4] = {0, 0, 0, 0};

    printf("\n=== %s ===\n", name);
    printf("time,pos_x,pos_y,pos_z,vel_z,roll,pitch,yaw,on_ground\n");

    for (int i = 0; i < steps; i++) {
        float t = i * dt;

        quad.step(zero_cmd, dt);

        if (i % 2 == 0) {  // 5ms interval (200Hz output)
            auto st = quad.getState();
            Vec3 euler = st.attitude.to_euler();
            printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%d\n",
                   t, st.position.x, st.position.y, st.position.z,
                   st.velocity.z,
                   euler.x * 57.3f, euler.y * 57.3f, euler.z * 57.3f,
                   quad.isOnGround());
        }
    }

    auto final_st = quad.getState();
    Vec3 final_euler = final_st.attitude.to_euler();
    printf("Final: pos=[%.4f %.4f %.4f] vel=[%.3f %.3f %.3f] att=[%.1f %.1f %.1f] ground=%d\n",
           final_st.position.x, final_st.position.y, final_st.position.z,
           final_st.velocity.x, final_st.velocity.y, final_st.velocity.z,
           final_euler.x*57.3f, final_euler.y*57.3f, final_euler.z*57.3f,
           quad.isOnGround());
}

int main()
{
    srand(42);  // Fixed seed for reproducibility
    float dt = 0.0025f;

    // =========================================================================
    // Scenario 1: Level drop from 0.5m
    // シナリオ1: 0.5mからの水平落下
    // =========================================================================
    {
        QuadPhysics quad;
        QuadParams qp;
        ContactParams cp;
        SensorNoiseParams np = {};  // No noise for physics test
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.vib_accel_k[0] = np.vib_accel_k[1] = np.vib_accel_k[2] = 0;
        np.vib_gyro_k[0] = np.vib_gyro_k[1] = np.vib_gyro_k[2] = 0;

        quad.init(qp, cp, np);
        // Set initial height to 0.5m (NED: z = -0.5)
        // 初期高度0.5m
        auto& state = const_cast<QuadState&>(quad.getState());
        state.position.z = -0.5f;

        run_scenario("Level drop from 0.5m", quad, 1.0f, dt);
    }

    // =========================================================================
    // Scenario 2: Tilted drop (30° roll) from 0.3m
    // シナリオ2: 傾いた落下（ロール30°）0.3mから
    // =========================================================================
    {
        QuadPhysics quad;
        QuadParams qp;
        ContactParams cp;
        SensorNoiseParams np = {};
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.vib_accel_k[0] = np.vib_accel_k[1] = np.vib_accel_k[2] = 0;
        np.vib_gyro_k[0] = np.vib_gyro_k[1] = np.vib_gyro_k[2] = 0;

        quad.init(qp, cp, np);
        auto& state = const_cast<QuadState&>(quad.getState());
        state.position.z = -0.3f;
        // 30 degree roll
        state.attitude = Quat::from_rotvec(Vec3(30.0f * M_PI / 180.0f, 0, 0));

        run_scenario("Tilted drop (30deg roll) from 0.3m", quad, 1.0f, dt);
    }

    // =========================================================================
    // Scenario 3: Upside-down drop from 0.2m
    // シナリオ3: 裏返し落下 0.2mから
    // =========================================================================
    {
        QuadPhysics quad;
        QuadParams qp;
        ContactParams cp;
        SensorNoiseParams np = {};
        np.gyro_noise_density = 0; np.accel_noise_density = 0;
        np.gyro_bias_init_std = 0; np.accel_bias_init_std = 0;
        np.vib_accel_k[0] = np.vib_accel_k[1] = np.vib_accel_k[2] = 0;
        np.vib_gyro_k[0] = np.vib_gyro_k[1] = np.vib_gyro_k[2] = 0;

        quad.init(qp, cp, np);
        auto& state = const_cast<QuadState&>(quad.getState());
        state.position.z = -0.2f;
        // 180 degree roll (upside down)
        state.attitude = Quat::from_rotvec(Vec3(M_PI, 0, 0));

        run_scenario("Upside-down drop from 0.2m", quad, 1.0f, dt);
    }

    return 0;
}
