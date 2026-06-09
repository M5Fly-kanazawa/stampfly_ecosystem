/**
 * @file test_main.cpp
 * @brief Unit test runner for vehicle_new algorithm layer
 *        vehicle_newアルゴリズム層の単体テストランナー
 *
 * Runs on host PC (not ESP32). Tests sf_math, ESKF core, PID.
 * ホストPC上で実行（ESP32ではない）。sf_math、ESKFコア、PIDをテスト。
 *
 * Build: g++ -std=c++17 -I../components/sf_math/include
 *            -I../components/sf_estimator_eskf/include
 *            -I../components/sf_controller_pid/include
 *            -I../components/sf_core/include
 *            -I../components/sf_estimator/include
 *            -I../components/sf_controller/include
 *            -I../components/sf_state/include
 *            test_main.cpp ../components/sf_estimator_eskf/eskf_core.cpp
 *            -o test_vehicle_new -lm
 *
 * @design requirements.md §10 — Unit testing on PC                    [--]
 * @design coding_and_education.md §2 — Bilingual comments             [--]
 */

#include <cstdio>
#include <cmath>
#include <cassert>

// Stub ESP logging for PC build
// PC用のESPロギングスタブ
#define ESP_LOGI(tag, fmt, ...) printf("[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERR]  %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) // silent

#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"

// =============================================================================
// Test framework (minimal)
// テストフレームワーク（最小）
// =============================================================================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-40s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_NEAR(a, b, tol) \
    if (fabsf((a) - (b)) > (tol)) { \
        printf("FAIL: %s:%d: %.6f != %.6f (tol=%.6f)\n", \
               __FILE__, __LINE__, (float)(a), (float)(b), (float)(tol)); \
        throw 1; \
    }

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

// =============================================================================
// sf_math tests
// sf_math テスト
// =============================================================================

TEST(vec3_add)
{
    sf::math::Vec3 a(1, 2, 3);
    sf::math::Vec3 b(4, 5, 6);
    auto c = a + b;
    ASSERT_NEAR(c.x, 5.0f, 1e-6f);
    ASSERT_NEAR(c.y, 7.0f, 1e-6f);
    ASSERT_NEAR(c.z, 9.0f, 1e-6f);
}

TEST(vec3_cross)
{
    sf::math::Vec3 a(1, 0, 0);
    sf::math::Vec3 b(0, 1, 0);
    auto c = a.cross(b);
    ASSERT_NEAR(c.x, 0.0f, 1e-6f);
    ASSERT_NEAR(c.y, 0.0f, 1e-6f);
    ASSERT_NEAR(c.z, 1.0f, 1e-6f);
}

TEST(vec3_norm)
{
    sf::math::Vec3 v(3, 4, 0);
    ASSERT_NEAR(v.norm(), 5.0f, 1e-6f);
}

TEST(quat_identity_rotate)
{
    sf::math::Quat q;  // Identity (1,0,0,0)
    sf::math::Vec3 v(1, 2, 3);
    auto r = q.rotate(v);
    ASSERT_NEAR(r.x, 1.0f, 1e-5f);
    ASSERT_NEAR(r.y, 2.0f, 1e-5f);
    ASSERT_NEAR(r.z, 3.0f, 1e-5f);
}

TEST(quat_90deg_z_rotate)
{
    // 90 degree rotation around Z axis
    // Z軸周りの90度回転
    float half = M_PI / 4.0f;
    sf::math::Quat q(cosf(half), 0, 0, sinf(half));
    sf::math::Vec3 v(1, 0, 0);  // X axis
    auto r = q.rotate(v);
    ASSERT_NEAR(r.x, 0.0f, 1e-5f);
    ASSERT_NEAR(r.y, 1.0f, 1e-5f);
    ASSERT_NEAR(r.z, 0.0f, 1e-5f);
}

TEST(quat_normalize)
{
    sf::math::Quat q(2, 0, 0, 0);
    q.normalize();
    ASSERT_NEAR(q.w, 1.0f, 1e-6f);
}

TEST(quat_from_rotvec)
{
    // Small rotation around Z
    // Z軸周りの微小回転
    sf::math::Vec3 rv(0, 0, 0.1f);
    auto q = sf::math::Quat::from_rotvec(rv);
    ASSERT_NEAR(q.w * q.w + q.z * q.z, 1.0f, 1e-5f);
    ASSERT_TRUE(q.w > 0.99f);
}

TEST(quat_to_euler_identity)
{
    sf::math::Quat q;  // Identity
    auto e = q.to_euler();
    ASSERT_NEAR(e.x, 0.0f, 1e-6f);  // roll
    ASSERT_NEAR(e.y, 0.0f, 1e-6f);  // pitch
    ASSERT_NEAR(e.z, 0.0f, 1e-6f);  // yaw
}

TEST(quat_dcm_identity)
{
    sf::math::Quat q;
    float R[3][3];
    q.to_dcm(R);
    ASSERT_NEAR(R[0][0], 1.0f, 1e-6f);
    ASSERT_NEAR(R[1][1], 1.0f, 1e-6f);
    ASSERT_NEAR(R[2][2], 1.0f, 1e-6f);
    ASSERT_NEAR(R[0][1], 0.0f, 1e-6f);
}

// =============================================================================
// ESKF Core tests
// ESKFコア テスト
// =============================================================================

TEST(eskf_init_identity)
{
    sf::EskfCore eskf;
    sf::EskfConfig cfg;
    eskf.init(cfg);

    auto q = eskf.getAttitude();
    ASSERT_NEAR(q.w, 1.0f, 1e-6f);
    ASSERT_NEAR(q.x, 0.0f, 1e-6f);

    auto p = eskf.getPosition();
    ASSERT_NEAR(p.x, 0.0f, 1e-6f);
    ASSERT_NEAR(p.y, 0.0f, 1e-6f);
    ASSERT_NEAR(p.z, 0.0f, 1e-6f);
}

TEST(eskf_predict_stationary)
{
    // Stationary IMU: accel = [0, 0, -9.81], gyro = [0, 0, 0]
    // 静止IMU: accel = [0, 0, -9.81], gyro = [0, 0, 0]
    sf::EskfCore eskf;
    sf::EskfConfig cfg;
    eskf.init(cfg);

    sf::math::Vec3 accel(0, 0, -9.81f);
    sf::math::Vec3 gyro(0, 0, 0);

    // Predict 100 steps (0.25s at 400Hz)
    // 100ステップ予測（400Hzで0.25秒）
    for (int i = 0; i < 100; i++) {
        eskf.predict(accel, gyro, 0.0025f);
    }

    // Position should remain near zero (gravity cancels)
    // 位置はほぼゼロのまま（重力が相殺）
    auto p = eskf.getPosition();
    ASSERT_NEAR(p.x, 0.0f, 0.01f);
    ASSERT_NEAR(p.y, 0.0f, 0.01f);
    ASSERT_NEAR(p.z, 0.0f, 0.1f);  // Z may drift slightly

    // Attitude should remain identity
    // 姿勢は単位クォータニオンのまま
    auto q = eskf.getAttitude();
    ASSERT_NEAR(q.w, 1.0f, 0.01f);
}

TEST(eskf_predict_freefall)
{
    // Free fall: accel = [0, 0, 0], gyro = [0, 0, 0]
    // 自由落下: accel = [0, 0, 0], gyro = [0, 0, 0]
    sf::EskfCore eskf;
    sf::EskfConfig cfg;
    eskf.init(cfg);

    sf::math::Vec3 accel(0, 0, 0);
    sf::math::Vec3 gyro(0, 0, 0);

    // Predict 400 steps (1s at 400Hz)
    // 400ステップ予測（400Hzで1秒）
    for (int i = 0; i < 400; i++) {
        eskf.predict(accel, gyro, 0.0025f);
    }

    // Should fall: z ≈ 0.5*g*t² = 0.5*9.81*1 ≈ 4.9m
    // 落下するはず: z ≈ 0.5*g*t² = 0.5*9.81*1 ≈ 4.9m
    auto p = eskf.getPosition();
    ASSERT_NEAR(p.z, 4.9f, 0.5f);  // NED: z positive is down
}

TEST(eskf_tof_update)
{
    sf::EskfCore eskf;
    sf::EskfConfig cfg;
    cfg.use_tof = true;
    eskf.init(cfg);

    // Set position to z = -0.5 (0.5m up in NED)
    // Then observe ToF at 0.5m → should correct
    eskf.updateToF(0.5f);

    auto p = eskf.getPosition();
    // Position should move toward -0.5 (0.5m up)
    // 位置は-0.5（0.5m上方）に向かうはず
    ASSERT_TRUE(p.z < 0.0f);
}

TEST(eskf_reset_position)
{
    sf::EskfCore eskf;
    sf::EskfConfig cfg;
    eskf.init(cfg);

    // Predict to move position
    sf::math::Vec3 accel(1, 0, -9.81f);
    sf::math::Vec3 gyro(0, 0, 0);
    for (int i = 0; i < 100; i++) {
        eskf.predict(accel, gyro, 0.0025f);
    }

    // Reset position
    eskf.resetPositionVelocity();
    auto p = eskf.getPosition();
    ASSERT_NEAR(p.x, 0.0f, 1e-6f);
    ASSERT_NEAR(p.y, 0.0f, 1e-6f);
    ASSERT_NEAR(p.z, 0.0f, 1e-6f);
}

// =============================================================================
// PID tests
// PIDテスト
// =============================================================================

TEST(pid_proportional)
{
    sf::PID pid;
    pid.kp = 2.0f;
    pid.ti = 1000.0f;  // Effectively no integral
    pid.td = 0;
    pid.output_limit = 100.0f;

    float out = pid.compute(1.0f, 0.01f);
    ASSERT_NEAR(out, 2.0f, 0.01f);
}

TEST(pid_integral)
{
    sf::PID pid;
    pid.kp = 1.0f;
    pid.ti = 1.0f;
    pid.td = 0;
    pid.output_limit = 100.0f;

    // Accumulate integral over 10 steps
    // 10ステップで積分を蓄積
    for (int i = 0; i < 10; i++) {
        pid.compute(1.0f, 0.1f);
    }

    // Trapezoidal (Tustin) integration: each step adds kp/ti·(e+e_prev)·dt/2.
    // The first step sees prev_error=0 and adds only 0.05; steps 2..10 add 0.1
    // each → 0.05 + 9×0.1 = 0.95 (rectangular integration would give 1.0).
    // 台形（Tustin）積分: 各ステップで kp/ti·(e+e_prev)·dt/2 を加算。初回は
    // prev_error=0 で 0.05 のみ、2〜10 回目は各 0.1 → 0.05 + 9×0.1 = 0.95
    // （矩形積分なら 1.0）。
    ASSERT_NEAR(pid.integral, 0.95f, 0.01f);
}

TEST(pid_reset)
{
    sf::PID pid;
    pid.kp = 1.0f;
    pid.ti = 1.0f;
    pid.output_limit = 100.0f;

    pid.compute(1.0f, 0.1f);
    ASSERT_TRUE(pid.integral != 0.0f);

    pid.reset();
    ASSERT_NEAR(pid.integral, 0.0f, 1e-6f);
    ASSERT_NEAR(pid.prev_error, 0.0f, 1e-6f);
}

TEST(pid_output_limit)
{
    sf::PID pid;
    pid.kp = 100.0f;
    pid.ti = 1000.0f;
    pid.td = 0;
    pid.output_limit = 5.0f;

    float out = pid.compute(1.0f, 0.01f);
    ASSERT_NEAR(out, 5.0f, 1e-6f);  // Clamped to limit
}

// =============================================================================
// Main
// =============================================================================

int main()
{
    printf("=== vehicle_new Unit Tests ===\n\n");

    printf("[sf_math]\n");
    run_vec3_add();
    run_vec3_cross();
    run_vec3_norm();
    run_quat_identity_rotate();
    run_quat_90deg_z_rotate();
    run_quat_normalize();
    run_quat_from_rotvec();
    run_quat_to_euler_identity();
    run_quat_dcm_identity();

    printf("\n[ESKF]\n");
    run_eskf_init_identity();
    run_eskf_predict_stationary();
    run_eskf_predict_freefall();
    run_eskf_tof_update();
    run_eskf_reset_position();

    printf("\n[PID]\n");
    run_pid_proportional();
    run_pid_integral();
    run_pid_reset();
    run_pid_output_limit();

    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           tests_passed, tests_run, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
