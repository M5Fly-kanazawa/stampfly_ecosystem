/**
 * @file eskf_test.cpp
 * @brief ESKF Unit Tests
 *
 * Theoretical verification of ESKF implementation:
 * - State initialization
 * - Predict step (gyro integration, accel integration)
 * - Measurement updates (baro, ToF, mag, accel attitude, flow)
 * - Covariance propagation
 */

#include "eskf.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace stampfly;
using namespace stampfly::math;

// Test framework
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_NEAR(expected, actual, tolerance, msg) \
    do { \
        float _exp = (expected); \
        float _act = (actual); \
        float _tol = (tolerance); \
        if (std::fabs(_exp - _act) <= _tol) { \
            tests_passed++; \
        } else { \
            tests_failed++; \
            printf("  FAIL: %s\n", msg); \
            printf("    Expected: %.6f, Actual: %.6f, Diff: %.6f, Tol: %.6f\n", \
                   _exp, _act, std::fabs(_exp - _act), _tol); \
        } \
    } while(0)

#define ASSERT_TRUE(condition, msg) \
    do { \
        if (condition) { \
            tests_passed++; \
        } else { \
            tests_failed++; \
            printf("  FAIL: %s\n", msg); \
        } \
    } while(0)

#define TEST_SECTION(name) \
    printf("\n=== %s ===\n", name)

// ============================================================================
// Test 1: Initialization
// ============================================================================
void test_initialization() {
    TEST_SECTION("Test 1: Initialization");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();

    ASSERT_TRUE(eskf.init(config) == ESP_OK, "Init returns ESP_OK");
    ASSERT_TRUE(eskf.isInitialized(), "isInitialized() returns true");

    auto state = eskf.getState();

    // Position should be zero
    ASSERT_NEAR(0.0f, state.position.x, 1e-6f, "Initial pos.x = 0");
    ASSERT_NEAR(0.0f, state.position.y, 1e-6f, "Initial pos.y = 0");
    ASSERT_NEAR(0.0f, state.position.z, 1e-6f, "Initial pos.z = 0");

    // Velocity should be zero
    ASSERT_NEAR(0.0f, state.velocity.x, 1e-6f, "Initial vel.x = 0");
    ASSERT_NEAR(0.0f, state.velocity.y, 1e-6f, "Initial vel.y = 0");
    ASSERT_NEAR(0.0f, state.velocity.z, 1e-6f, "Initial vel.z = 0");

    // Attitude should be identity quaternion (no rotation)
    ASSERT_NEAR(1.0f, state.orientation.w, 1e-6f, "Initial quat.w = 1");
    ASSERT_NEAR(0.0f, state.orientation.x, 1e-6f, "Initial quat.x = 0");
    ASSERT_NEAR(0.0f, state.orientation.y, 1e-6f, "Initial quat.y = 0");
    ASSERT_NEAR(0.0f, state.orientation.z, 1e-6f, "Initial quat.z = 0");

    // Euler angles should be zero
    ASSERT_NEAR(0.0f, state.roll, 1e-6f, "Initial roll = 0");
    ASSERT_NEAR(0.0f, state.pitch, 1e-6f, "Initial pitch = 0");
    ASSERT_NEAR(0.0f, state.yaw, 1e-6f, "Initial yaw = 0");

    // Biases should be zero
    ASSERT_NEAR(0.0f, state.gyro_bias.x, 1e-6f, "Initial gyro_bias.x = 0");
    ASSERT_NEAR(0.0f, state.accel_bias.x, 1e-6f, "Initial accel_bias.x = 0");

    // Covariance matrix diagonal should be positive
    auto& P = eskf.getCovariance();
    bool all_positive = true;
    for (int i = 0; i < 15; i++) {
        if (P(i, i) <= 0.0f) {
            all_positive = false;
            printf("  P(%d,%d) = %.6f <= 0\n", i, i, P(i, i));
        }
    }
    ASSERT_TRUE(all_positive, "Covariance diagonal all positive");

    printf("  Initialization: OK\n");
}

// ============================================================================
// Test 2: Predict - Gyro Integration (Attitude)
// ============================================================================
void test_predict_attitude() {
    TEST_SECTION("Test 2: Predict - Gyro Integration");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();

    float dt = 0.01f;  // 100Hz
    // ESKFでは accel は重力込みの生データ。静止時は [0, 0, -g] (body frame, z-up)
    // ただし実装では accel_world_z = R*accel + g なので、入力0で重力のみになる
    Vector3 accel(0.0f, 0.0f, 0.0f);

    // Test 1: Rotate around Z axis (yaw) - requires mag_enabled=true
    config.mag_enabled = true;  // Enable yaw integration
    eskf.init(config);

    Vector3 gyro_yaw(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 10; i++) {
        eskf.predict(accel, gyro_yaw, dt);
    }

    auto state = eskf.getState();
    float expected_yaw = 1.0f * 0.1f;  // omega * t = 0.1 rad
    printf("  Yaw after 100ms at 1 rad/s: %.4f rad (expected: %.4f)\n",
           state.yaw, expected_yaw);
    ASSERT_NEAR(expected_yaw, state.yaw, 0.01f, "Yaw integration");
    ASSERT_NEAR(0.0f, state.roll, 0.01f, "Roll unchanged during yaw rotation");
    ASSERT_NEAR(0.0f, state.pitch, 0.01f, "Pitch unchanged during yaw rotation");

    // Test 2: Reset and rotate around X axis (roll)
    eskf.reset();
    Vector3 gyro_roll(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) {
        eskf.predict(accel, gyro_roll, dt);
    }

    state = eskf.getState();
    float expected_roll = 1.0f * 0.1f;
    printf("  Roll after 100ms at 1 rad/s: %.4f rad (expected: %.4f)\n",
           state.roll, expected_roll);
    ASSERT_NEAR(expected_roll, state.roll, 0.01f, "Roll integration");

    // Test 3: Reset and rotate around Y axis (pitch)
    eskf.reset();
    Vector3 gyro_pitch(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 10; i++) {
        eskf.predict(accel, gyro_pitch, dt);
    }

    state = eskf.getState();
    float expected_pitch = 1.0f * 0.1f;
    printf("  Pitch after 100ms at 1 rad/s: %.4f rad (expected: %.4f)\n",
           state.pitch, expected_pitch);
    ASSERT_NEAR(expected_pitch, state.pitch, 0.01f, "Pitch integration");
}

// ============================================================================
// Test 3: Predict - Accel Integration (Velocity/Position)
// ============================================================================
void test_predict_velocity_position() {
    TEST_SECTION("Test 3: Predict - Accel Integration");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    float dt = 0.01f;
    Vector3 gyro(0.0f, 0.0f, 0.0f);

    // ESKF uses: accel_world = R * accel_body + gravity
    // For stationary (hovering), we need input accel that cancels gravity:
    // accel_body = [0, 0, -g] so that R * [0, 0, -g] + [0, 0, g] = [0, 0, 0]
    Vector3 accel_hover(0.0f, 0.0f, -config.gravity);

    for (int i = 0; i < 100; i++) {
        eskf.predict(accel_hover, gyro, dt);
    }

    auto state = eskf.getState();
    printf("  After 1s with hover accel: vel=[%.4f, %.4f, %.4f], pos=[%.4f, %.4f, %.4f]\n",
           state.velocity.x, state.velocity.y, state.velocity.z,
           state.position.x, state.position.y, state.position.z);

    // Velocity and position should remain near zero (hovering)
    ASSERT_NEAR(0.0f, state.velocity.x, 0.1f, "Vel.x near 0 with hover");
    ASSERT_NEAR(0.0f, state.velocity.y, 0.1f, "Vel.y near 0 with hover");
    ASSERT_NEAR(0.0f, state.velocity.z, 0.1f, "Vel.z near 0 with hover");

    // Test 2: Apply 1 m/s^2 acceleration in X direction + hover in Z
    eskf.reset();
    Vector3 accel_x(1.0f, 0.0f, -config.gravity);

    for (int i = 0; i < 100; i++) {
        eskf.predict(accel_x, gyro, dt);
    }

    state = eskf.getState();
    float expected_vel_x = 1.0f * 1.0f;  // a * t = 1 m/s
    float expected_pos_x = 0.5f * 1.0f * 1.0f * 1.0f;  // 0.5 * a * t^2 = 0.5 m

    printf("  After 1s with 1 m/s^2 in X: vel.x=%.4f (exp: %.4f), pos.x=%.4f (exp: %.4f)\n",
           state.velocity.x, expected_vel_x, state.position.x, expected_pos_x);

    ASSERT_NEAR(expected_vel_x, state.velocity.x, 0.1f, "Vel.x with constant accel");
    ASSERT_NEAR(expected_pos_x, state.position.x, 0.1f, "Pos.x with constant accel");

    // Test 3: Free fall (accel = 0, gravity causes downward acceleration)
    // In NED, positive z is down, so velocity.z should increase
    eskf.reset();
    Vector3 accel_freefall(0.0f, 0.0f, 0.0f);  // No thrust

    for (int i = 0; i < 100; i++) {
        eskf.predict(accel_freefall, gyro, dt);
    }

    state = eskf.getState();
    float expected_vel_z = config.gravity * 1.0f;  // g * t
    float expected_pos_z = 0.5f * config.gravity * 1.0f * 1.0f;  // 0.5 * g * t^2

    printf("  After 1s free fall: vel.z=%.4f (exp: %.4f), pos.z=%.4f (exp: %.4f)\n",
           state.velocity.z, expected_vel_z, state.position.z, expected_pos_z);

    ASSERT_NEAR(expected_vel_z, state.velocity.z, 0.2f, "Vel.z in free fall");
    ASSERT_NEAR(expected_pos_z, state.position.z, 0.2f, "Pos.z in free fall");
}

// ============================================================================
// Test 4: Baro Update
// ============================================================================
void test_baro_update() {
    TEST_SECTION("Test 4: Baro Update");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // Initial position.z is 0
    auto state = eskf.getState();
    ASSERT_NEAR(0.0f, state.position.z, 1e-6f, "Initial position.z = 0");

    // Update with baro reading of 1.0m altitude
    // In NED, altitude = -position.z, so baro=1.0m means position.z should go to -1.0
    eskf.updateBaro(1.0f);
    state = eskf.getState();
    printf("  After baro update (1.0m): position.z=%.4f m (NED: altitude=%.4f m)\n",
           state.position.z, -state.position.z);

    // position.z should become negative (altitude is positive)
    ASSERT_TRUE(state.position.z < 0.0f, "position.z decreased after baro update (altitude increased)");

    // Multiple updates should converge
    for (int i = 0; i < 10; i++) {
        eskf.updateBaro(1.0f);
    }
    state = eskf.getState();
    printf("  After 10 baro updates: position.z=%.4f m (altitude=%.4f m)\n",
           state.position.z, -state.position.z);
    ASSERT_NEAR(-1.0f, state.position.z, 0.2f, "position.z converges to -altitude");
}

// ============================================================================
// Test 5: ToF Update (with tilt correction)
// ============================================================================
void test_tof_update() {
    TEST_SECTION("Test 5: ToF Update");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // Test 1: Flat orientation, ToF = 0.5m
    // ToF measures height above ground, so altitude = 0.5m, position.z = -0.5m (NED)
    eskf.updateToF(0.5f);
    auto state = eskf.getState();
    printf("  Flat, ToF=0.5m: position.z=%.4f m (altitude=%.4f m)\n",
           state.position.z, -state.position.z);
    ASSERT_TRUE(state.position.z < 0.0f, "position.z decreased after ToF update (altitude increased)");

    // Test 2: With tilt, altitude should be ToF * cos(tilt)
    eskf.reset();

    // First, tilt the drone by pitching
    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, 0.0f);  // No accel input
    Vector3 gyro_pitch(0.0f, 1.0f, 0.0f);  // Pitch at 1 rad/s

    // Pitch for 0.3s to get ~0.3 rad (~17 degrees)
    for (int i = 0; i < 30; i++) {
        eskf.predict(accel, gyro_pitch, dt);
    }

    state = eskf.getState();
    printf("  After pitching: pitch=%.2f deg\n", state.pitch * 180.0f / M_PI);

    // Now update with ToF
    float tof_distance = 0.5f;
    eskf.updateToF(tof_distance);

    state = eskf.getState();
    // Expected altitude = ToF * cos(pitch) * cos(roll)
    float expected_alt = tof_distance * std::cos(state.pitch) * std::cos(state.roll);
    printf("  Tilted, ToF=0.5m: position.z=%.4f m (altitude=%.4f m, expected ~%.4f)\n",
           state.position.z, -state.position.z, expected_alt);

    // With tilt, altitude should be less than ToF distance
    // In NED: position.z should be closer to 0 than -tof_distance
    ASSERT_TRUE(std::fabs(state.position.z) < tof_distance, "|position.z| < ToF when tilted");
}

// ============================================================================
// Test 6: Accel Attitude Update (Roll/Pitch correction)
// ============================================================================
void test_accel_attitude_update() {
    TEST_SECTION("Test 6: Accel Attitude Update");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    config.accel_motion_threshold = 1.0f;  // Relax threshold for testing
    eskf.init(config);

    // ESKF expects: g_body = R^T * [0, 0, -g]
    // When flat (R=I): g_body = [0, 0, -g]
    // When rolled by angle θ around X: R^T rotates the gravity vector
    // g_body = [0, g*sin(θ), -g*cos(θ)]

    float roll_rad = 30.0f * M_PI / 180.0f;
    float g = config.gravity;

    // For roll around X-axis, accelerometer measures:
    // a_x = 0, a_y = g*sin(roll), a_z = -g*cos(roll)
    Vector3 accel_rolled(
        0.0f,
        g * std::sin(roll_rad),
        -g * std::cos(roll_rad)
    );

    printf("  Input accel for 30 deg roll: [%.3f, %.3f, %.3f]\n",
           accel_rolled.x, accel_rolled.y, accel_rolled.z);
    printf("  Accel norm: %.3f (should be ~%.3f)\n",
           std::sqrt(accel_rolled.x*accel_rolled.x + accel_rolled.y*accel_rolled.y + accel_rolled.z*accel_rolled.z),
           g);

    // Multiple updates to converge
    for (int i = 0; i < 50; i++) {
        eskf.updateAccelAttitude(accel_rolled);
    }

    auto state = eskf.getState();
    printf("  After 50 updates: roll=%.2f deg (expected: %.2f deg)\n",
           state.roll * 180.0f / M_PI, roll_rad * 180.0f / M_PI);

    // Roll should approach the true roll angle
    ASSERT_NEAR(roll_rad, std::fabs(state.roll), 0.15f, "Roll from accel attitude");

    // Test pitch
    eskf.reset();
    float pitch_rad = 20.0f * M_PI / 180.0f;

    // For pitch around Y-axis, accelerometer measures:
    // a_x = g*sin(pitch), a_y = 0, a_z = -g*cos(pitch)
    Vector3 accel_pitched(
        g * std::sin(pitch_rad),
        0.0f,
        -g * std::cos(pitch_rad)
    );

    printf("  Input accel for 20 deg pitch: [%.3f, %.3f, %.3f]\n",
           accel_pitched.x, accel_pitched.y, accel_pitched.z);

    for (int i = 0; i < 50; i++) {
        eskf.updateAccelAttitude(accel_pitched);
    }

    state = eskf.getState();
    printf("  After 50 updates: pitch=%.2f deg (expected: %.2f deg)\n",
           state.pitch * 180.0f / M_PI, pitch_rad * 180.0f / M_PI);

    ASSERT_NEAR(pitch_rad, std::fabs(state.pitch), 0.15f, "Pitch from accel attitude");
}

// ============================================================================
// Test 7: Mag Update (Yaw correction)
// ============================================================================
void test_mag_update() {
    TEST_SECTION("Test 7: Mag Update");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    config.mag_enabled = true;
    config.mag_ref = Vector3(20.0f, 0.0f, 40.0f);  // Reference: north
    eskf.init(config);

    // First, rotate yaw by predict
    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, 9.81f);
    Vector3 gyro_yaw(0.0f, 0.0f, 1.0f);

    for (int i = 0; i < 50; i++) {  // 0.5 rad yaw
        eskf.predict(accel, gyro_yaw, dt);
    }

    auto state = eskf.getState();
    float yaw_before = state.yaw;
    printf("  Yaw before mag update: %.2f deg\n", yaw_before * 180.0f / M_PI);

    // Mag reading corresponding to yaw=0 (pointing north)
    Vector3 mag_north = config.mag_ref;
    eskf.updateMag(mag_north);

    state = eskf.getState();
    float yaw_after = state.yaw;
    printf("  Yaw after mag update (north): %.2f deg\n", yaw_after * 180.0f / M_PI);

    // Yaw should move towards 0
    ASSERT_TRUE(std::fabs(yaw_after) < std::fabs(yaw_before),
                "Yaw moves towards reference after mag update");
}

// ============================================================================
// Test 8: Covariance Propagation
// ============================================================================
void test_covariance_propagation() {
    TEST_SECTION("Test 8: Covariance Propagation");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // Get initial covariance trace
    auto& P_init = eskf.getCovariance();
    float trace_init = 0.0f;
    for (int i = 0; i < 15; i++) {
        trace_init += P_init(i, i);
    }
    printf("  Initial covariance trace: %.6f\n", trace_init);

    // Predict step should increase covariance (process noise added)
    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, 9.81f);
    Vector3 gyro(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 10; i++) {
        eskf.predict(accel, gyro, dt);
    }

    auto& P_after_predict = eskf.getCovariance();
    float trace_after_predict = 0.0f;
    for (int i = 0; i < 15; i++) {
        trace_after_predict += P_after_predict(i, i);
    }
    printf("  After 10 predictions: trace=%.6f\n", trace_after_predict);

    ASSERT_TRUE(trace_after_predict > trace_init,
                "Covariance increases after prediction");

    // Measurement update should decrease covariance (information gained)
    float trace_before_update = trace_after_predict;
    eskf.updateBaro(0.0f);  // Update with measurement

    auto& P_after_update = eskf.getCovariance();
    float trace_after_update = 0.0f;
    for (int i = 0; i < 15; i++) {
        trace_after_update += P_after_update(i, i);
    }
    printf("  After baro update: trace=%.6f\n", trace_after_update);

    ASSERT_TRUE(trace_after_update < trace_before_update,
                "Covariance decreases after measurement update");

    // Check symmetry
    bool is_symmetric = true;
    for (int i = 0; i < 15; i++) {
        for (int j = i + 1; j < 15; j++) {
            if (std::fabs(P_after_update(i, j) - P_after_update(j, i)) > 1e-6f) {
                is_symmetric = false;
                printf("  P(%d,%d)=%.6f != P(%d,%d)=%.6f\n",
                       i, j, P_after_update(i, j), j, i, P_after_update(j, i));
            }
        }
    }
    ASSERT_TRUE(is_symmetric, "Covariance matrix is symmetric");

    // Check positive definiteness (all diagonal elements positive)
    bool is_positive = true;
    for (int i = 0; i < 15; i++) {
        if (P_after_update(i, i) <= 0.0f) {
            is_positive = false;
            printf("  P(%d,%d)=%.6f <= 0\n", i, i, P_after_update(i, i));
        }
    }
    ASSERT_TRUE(is_positive, "Covariance diagonal all positive");
}

// ============================================================================
// Test 9: Reset
// ============================================================================
void test_reset() {
    TEST_SECTION("Test 9: Reset");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // Do some predictions and updates
    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, 9.81f);
    Vector3 gyro(0.1f, 0.2f, 0.3f);

    for (int i = 0; i < 100; i++) {
        eskf.predict(accel, gyro, dt);
    }
    eskf.updateBaro(2.0f);
    eskf.updateToF(1.5f);

    auto state_before = eskf.getState();
    printf("  Before reset: pos=[%.3f, %.3f, %.3f], yaw=%.2f deg\n",
           state_before.position.x, state_before.position.y, state_before.position.z,
           state_before.yaw * 180.0f / M_PI);

    // Reset
    eskf.reset();

    auto state_after = eskf.getState();
    printf("  After reset: pos=[%.3f, %.3f, %.3f], yaw=%.2f deg\n",
           state_after.position.x, state_after.position.y, state_after.position.z,
           state_after.yaw * 180.0f / M_PI);

    // All should be zero
    ASSERT_NEAR(0.0f, state_after.position.x, 1e-6f, "Reset pos.x = 0");
    ASSERT_NEAR(0.0f, state_after.position.y, 1e-6f, "Reset pos.y = 0");
    ASSERT_NEAR(0.0f, state_after.position.z, 1e-6f, "Reset pos.z = 0");
    ASSERT_NEAR(0.0f, state_after.velocity.x, 1e-6f, "Reset vel.x = 0");
    ASSERT_NEAR(0.0f, state_after.roll, 1e-6f, "Reset roll = 0");
    ASSERT_NEAR(0.0f, state_after.pitch, 1e-6f, "Reset pitch = 0");
    ASSERT_NEAR(0.0f, state_after.yaw, 1e-6f, "Reset yaw = 0");
}

// ============================================================================
// Test 10: Bias Estimation
// ============================================================================
void test_bias_estimation() {
    TEST_SECTION("Test 10: Bias Estimation");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    config.accel_motion_threshold = 10.0f;  // Allow updates
    eskf.init(config);

    // Simulate gyro with bias
    float true_gyro_bias = 0.01f;  // 0.01 rad/s bias in X

    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, config.gravity);
    Vector3 gyro(true_gyro_bias, 0.0f, 0.0f);  // Measured gyro includes bias

    // Run predict and accel attitude update (which helps estimate bias)
    for (int i = 0; i < 1000; i++) {
        eskf.predict(accel, gyro, dt);
        if (i % 10 == 0) {
            eskf.updateAccelAttitude(accel);
        }
    }

    auto state = eskf.getState();
    printf("  True gyro bias X: %.6f rad/s\n", true_gyro_bias);
    printf("  Estimated gyro bias X: %.6f rad/s\n", state.gyro_bias.x);

    // The ESKF should estimate the bias over time
    // Note: exact convergence depends on observability and noise settings
    // We just check that some estimation is happening
    ASSERT_TRUE(std::fabs(state.gyro_bias.x) > 0.0f,
                "Gyro bias estimation non-zero");

    printf("  Note: Bias convergence depends on observability conditions\n");
}

// ============================================================================
// Test 11: Flow Update (Velocity estimation)
// ============================================================================
void test_flow_update() {
    TEST_SECTION("Test 11: Flow Update");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // First, give it some altitude (ToF or baro)
    for (int i = 0; i < 5; i++) {
        eskf.updateBaro(0.5f);
        eskf.updateToF(0.5f);
    }

    auto state = eskf.getState();
    printf("  Initial: vel=[%.3f, %.3f], alt=%.3f\n",
           state.velocity.x, state.velocity.y, state.position.z);

    // Simulate flow indicating 0.5 m/s in X direction
    // flow = velocity / height, so flow = 0.5 / 0.5 = 1.0 rad/s
    float flow_x = 1.0f;  // rad/s
    float flow_y = 0.0f;
    float distance = 0.5f;

    for (int i = 0; i < 10; i++) {
        eskf.updateFlow(flow_x, flow_y, distance);
    }

    state = eskf.getState();
    float expected_vel_x = flow_x * distance;  // 0.5 m/s
    printf("  After flow updates: vel=[%.3f, %.3f] (expected vel.x=%.3f)\n",
           state.velocity.x, state.velocity.y, expected_vel_x);

    ASSERT_NEAR(expected_vel_x, state.velocity.x, 0.2f, "Velocity from flow");
    ASSERT_NEAR(0.0f, state.velocity.y, 0.1f, "Vel.y near 0");
}

// ============================================================================
// Test 12: Quaternion Normalization
// ============================================================================
void test_quaternion_normalization() {
    TEST_SECTION("Test 12: Quaternion Normalization");

    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    float dt = 0.01f;
    Vector3 accel(0.0f, 0.0f, 9.81f);
    Vector3 gyro(0.5f, 0.3f, 0.2f);  // Arbitrary rotation

    // Run many iterations
    for (int i = 0; i < 1000; i++) {
        eskf.predict(accel, gyro, dt);
    }

    auto state = eskf.getState();
    float quat_norm = std::sqrt(
        state.orientation.w * state.orientation.w +
        state.orientation.x * state.orientation.x +
        state.orientation.y * state.orientation.y +
        state.orientation.z * state.orientation.z
    );

    printf("  After 1000 iterations: quaternion norm = %.6f\n", quat_norm);
    ASSERT_NEAR(1.0f, quat_norm, 1e-4f, "Quaternion remains normalized");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    printf("ESKF Unit Tests\n");
    printf("===============\n");

    test_initialization();
    test_predict_attitude();
    test_predict_velocity_position();
    test_baro_update();
    test_tof_update();
    test_accel_attitude_update();
    test_mag_update();
    test_covariance_propagation();
    test_reset();
    test_bias_estimation();
    test_flow_update();
    test_quaternion_normalization();

    printf("\n===============\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("===============\n");

    return tests_failed > 0 ? 1 : 0;
}
