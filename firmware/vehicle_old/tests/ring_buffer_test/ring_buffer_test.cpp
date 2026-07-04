/**
 * @file ring_buffer_test.cpp
 * @brief RingBuffer<T,N> Unit Tests (PC-side)
 *
 * Tests all public API methods of the RingBuffer template class:
 * - push / latest / get / count / capacity / full
 * - push with timestamp / raw_timestamp_at
 * - mean (scalar and Vector3)
 * - raw_index / raw_at (telemetry synchronization)
 * - reset
 * - wraparound behavior
 *
 * Compile: g++ -std=c++17 -I../../main -o ring_buffer_test ring_buffer_test.cpp && ./ring_buffer_test
 */

#include "ring_buffer.hpp"
#include <cstdio>
#include <cmath>
#include <cstdint>

// =========================================================================
// Minimal Vector3 for testing (matches stampfly::math::Vector3 interface)
// =========================================================================
struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    static Vec3 zero() { return {0, 0, 0}; }
};

// =========================================================================
// Test framework (same style as pid_test.cpp)
// =========================================================================
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_EQ(expected, actual, msg) \
    do { \
        auto _exp = (expected); \
        auto _act = (actual); \
        if (_exp == _act) { \
            tests_passed++; \
        } else { \
            tests_failed++; \
            printf("  FAIL: %s\n", msg); \
            printf("    Expected: %d, Actual: %d\n", (int)_exp, (int)_act); \
        } \
    } while(0)

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
            printf("    Expected: %.6f, Actual: %.6f, Diff: %.6f\n", \
                   _exp, _act, std::fabs(_exp - _act)); \
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

// =========================================================================
// Test 1: Basic push/latest/count for scalar
// =========================================================================
void test_basic_scalar() {
    TEST_SECTION("Test 1: Basic push/latest/count (scalar)");

    RingBuffer<float, 4> buf;

    ASSERT_EQ(0, buf.count(), "empty buffer count should be 0");
    ASSERT_EQ(4, buf.capacity(), "capacity should be 4");
    ASSERT_TRUE(!buf.full(), "empty buffer should not be full");

    buf.push(1.0f);
    ASSERT_EQ(1, buf.count(), "count after 1 push");
    ASSERT_NEAR(1.0f, buf.latest(), 0.001f, "latest after 1 push");

    buf.push(2.0f);
    buf.push(3.0f);
    ASSERT_EQ(3, buf.count(), "count after 3 pushes");
    ASSERT_NEAR(3.0f, buf.latest(), 0.001f, "latest after 3 pushes");
    ASSERT_TRUE(!buf.full(), "3/4 should not be full");

    buf.push(4.0f);
    ASSERT_EQ(4, buf.count(), "count after 4 pushes (full)");
    ASSERT_TRUE(buf.full(), "4/4 should be full");

    printf("  All basic scalar tests passed\n");
}

// =========================================================================
// Test 2: get(n_back) indexing
// =========================================================================
void test_get_indexing() {
    TEST_SECTION("Test 2: get(n_back) indexing");

    RingBuffer<float, 4> buf;
    buf.push(10.0f);
    buf.push(20.0f);
    buf.push(30.0f);

    ASSERT_NEAR(30.0f, buf.get(0), 0.001f, "get(0) = latest");
    ASSERT_NEAR(20.0f, buf.get(1), 0.001f, "get(1) = one before latest");
    ASSERT_NEAR(10.0f, buf.get(2), 0.001f, "get(2) = oldest");

    printf("  All get indexing tests passed\n");
}

// =========================================================================
// Test 3: Wraparound behavior
// =========================================================================
void test_wraparound() {
    TEST_SECTION("Test 3: Wraparound behavior");

    RingBuffer<int, 3> buf;

    // Fill: [1, 2, 3]
    buf.push(1);
    buf.push(2);
    buf.push(3);
    ASSERT_EQ(3, buf.count(), "count at capacity");
    ASSERT_TRUE(buf.full(), "should be full");

    // Overwrite oldest: [4, 2, 3] -> logical [2, 3, 4]
    buf.push(4);
    ASSERT_EQ(3, buf.count(), "count stays at capacity after overwrite");
    ASSERT_EQ(4, buf.get(0), "latest after wrap = 4");
    ASSERT_EQ(3, buf.get(1), "get(1) after wrap = 3");
    ASSERT_EQ(2, buf.get(2), "get(2) after wrap = 2");

    // Push more: logical [3, 4, 5]
    buf.push(5);
    ASSERT_EQ(5, buf.get(0), "latest = 5");
    ASSERT_EQ(4, buf.get(1), "get(1) = 4");
    ASSERT_EQ(3, buf.get(2), "get(2) = 3");

    // Push more: logical [4, 5, 6]
    buf.push(6);
    ASSERT_EQ(6, buf.get(0), "latest = 6");
    ASSERT_EQ(5, buf.get(1), "get(1) = 5");
    ASSERT_EQ(4, buf.get(2), "get(2) = 4");

    printf("  All wraparound tests passed\n");
}

// =========================================================================
// Test 4: Timestamp support
// =========================================================================
void test_timestamps() {
    TEST_SECTION("Test 4: Timestamp support");

    RingBuffer<float, 4> buf;

    buf.push(1.0f, 1000);
    buf.push(2.0f, 2000);
    buf.push(3.0f, 3000);

    ASSERT_EQ(3000, (int)buf.timestamp(0), "timestamp(0) = latest ts");
    ASSERT_EQ(2000, (int)buf.timestamp(1), "timestamp(1)");
    ASSERT_EQ(1000, (int)buf.timestamp(2), "timestamp(2) = oldest ts");

    // get_stamped
    auto s = buf.get_stamped(1);
    ASSERT_NEAR(2.0f, s.value, 0.001f, "get_stamped(1).value");
    ASSERT_EQ(2000, (int)s.ts_us, "get_stamped(1).ts_us");

    printf("  All timestamp tests passed\n");
}

// =========================================================================
// Test 5: raw_index / raw_at (telemetry pattern)
// =========================================================================
void test_raw_access() {
    TEST_SECTION("Test 5: raw_index / raw_at (telemetry pattern)");

    RingBuffer<float, 4> buf;

    ASSERT_EQ(0, buf.raw_index(), "initial raw_index = 0");

    buf.push(10.0f, 100);
    ASSERT_EQ(1, buf.raw_index(), "raw_index after 1 push");
    ASSERT_NEAR(10.0f, buf.raw_at(0), 0.001f, "raw_at(0)");
    ASSERT_EQ(100, (int)buf.raw_timestamp_at(0), "raw_timestamp_at(0)");

    buf.push(20.0f, 200);
    buf.push(30.0f, 300);
    ASSERT_EQ(3, buf.raw_index(), "raw_index after 3 pushes");
    ASSERT_NEAR(20.0f, buf.raw_at(1), 0.001f, "raw_at(1)");
    ASSERT_NEAR(30.0f, buf.raw_at(2), 0.001f, "raw_at(2)");

    // Simulate telemetry read pattern: read from read_idx to write head
    int read_idx = 0;
    int write_idx = buf.raw_index();
    int samples_read = 0;
    float sum = 0;
    while (read_idx != write_idx) {
        sum += buf.raw_at(read_idx);
        read_idx = (read_idx + 1) % buf.capacity();
        samples_read++;
    }
    ASSERT_EQ(3, samples_read, "telemetry read 3 samples");
    ASSERT_NEAR(60.0f, sum, 0.001f, "telemetry sum = 10+20+30");

    printf("  All raw access tests passed\n");
}

// =========================================================================
// Test 6: mean() for scalar
// =========================================================================
void test_mean_scalar() {
    TEST_SECTION("Test 6: mean() for scalar");

    RingBuffer<float, 8> buf;

    // Empty buffer
    ASSERT_NEAR(0.0f, buf.mean(), 0.001f, "mean of empty buffer = 0");

    buf.push(2.0f);
    buf.push(4.0f);
    buf.push(6.0f);
    ASSERT_NEAR(4.0f, buf.mean(), 0.001f, "mean of [2,4,6] = 4");
    ASSERT_NEAR(5.0f, buf.mean(2), 0.001f, "mean of latest 2 [4,6] = 5");
    ASSERT_NEAR(6.0f, buf.mean(1), 0.001f, "mean of latest 1 [6] = 6");

    printf("  All scalar mean tests passed\n");
}

// =========================================================================
// Test 7: mean() for Vec3
// =========================================================================
void test_mean_vec3() {
    TEST_SECTION("Test 7: mean() for Vec3");

    RingBuffer<Vec3, 8> buf;

    buf.push(Vec3(1, 2, 3));
    buf.push(Vec3(3, 4, 5));

    Vec3 avg = buf.mean();
    ASSERT_NEAR(2.0f, avg.x, 0.001f, "Vec3 mean.x = (1+3)/2");
    ASSERT_NEAR(3.0f, avg.y, 0.001f, "Vec3 mean.y = (2+4)/2");
    ASSERT_NEAR(4.0f, avg.z, 0.001f, "Vec3 mean.z = (3+5)/2");

    printf("  All Vec3 mean tests passed\n");
}

// =========================================================================
// Test 8: reset()
// =========================================================================
void test_reset() {
    TEST_SECTION("Test 8: reset()");

    RingBuffer<float, 4> buf;
    buf.push(1.0f);
    buf.push(2.0f);
    buf.push(3.0f);

    buf.reset();
    ASSERT_EQ(0, buf.count(), "count after reset = 0");
    ASSERT_TRUE(!buf.full(), "not full after reset");
    ASSERT_EQ(0, buf.raw_index(), "raw_index after reset = 0");

    // Can push again after reset
    buf.push(99.0f);
    ASSERT_EQ(1, buf.count(), "count after re-push = 1");
    ASSERT_NEAR(99.0f, buf.latest(), 0.001f, "latest after re-push");

    printf("  All reset tests passed\n");
}

// =========================================================================
// Test 9: get_latest_n (batch retrieval)
// =========================================================================
void test_get_latest_n() {
    TEST_SECTION("Test 9: get_latest_n (batch retrieval)");

    RingBuffer<int, 4> buf;
    buf.push(10);
    buf.push(20);
    buf.push(30);

    int out[4] = {};
    int n = buf.get_latest_n(out, 3);
    ASSERT_EQ(3, n, "get_latest_n returns 3");
    // Chronological order: oldest first
    ASSERT_EQ(10, out[0], "batch[0] = 10 (oldest)");
    ASSERT_EQ(20, out[1], "batch[1] = 20");
    ASSERT_EQ(30, out[2], "batch[2] = 30 (newest)");

    // Request more than available
    n = buf.get_latest_n(out, 10);
    ASSERT_EQ(3, n, "get_latest_n clamps to count");

    printf("  All batch retrieval tests passed\n");
}

// =========================================================================
// Test 10: Synchronized IMU buffer pattern
// =========================================================================
void test_imu_sync_pattern() {
    TEST_SECTION("Test 10: Synchronized IMU buffer pattern");

    // Simulate 4 IMU buffers pushed in same order
    RingBuffer<Vec3, 8> accel_buf, gyro_buf, accel_raw_buf, gyro_raw_buf;

    for (int i = 0; i < 5; i++) {
        float v = static_cast<float>(i);
        uint32_t ts = (i + 1) * 2500;  // 2.5ms intervals

        accel_raw_buf.push(Vec3(v, v, v), ts);
        gyro_raw_buf.push(Vec3(-v, -v, -v));
        accel_buf.push(Vec3(v * 0.9f, v * 0.9f, v * 0.9f));
        gyro_buf.push(Vec3(-v * 0.9f, -v * 0.9f, -v * 0.9f));
    }

    // All buffers should have same raw_index
    ASSERT_EQ(accel_buf.raw_index(), accel_raw_buf.raw_index(),
              "accel and accel_raw share raw_index");
    ASSERT_EQ(accel_buf.raw_index(), gyro_buf.raw_index(),
              "accel and gyro share raw_index");

    // Simulate telemetry reading: read all 4 buffers at same index
    int read_idx = 0;
    int write_idx = accel_buf.raw_index();
    int count = 0;
    while (read_idx != write_idx) {
        const auto& a = accel_buf.raw_at(read_idx);
        const auto& g = gyro_buf.raw_at(read_idx);
        const auto& ar = accel_raw_buf.raw_at(read_idx);
        const auto& gr = gyro_raw_buf.raw_at(read_idx);
        uint32_t ts = accel_raw_buf.raw_timestamp_at(read_idx);

        // Verify data consistency at each index
        float expected_raw = static_cast<float>(count);
        ASSERT_NEAR(expected_raw, ar.x, 0.001f, "raw accel matches at sync index");
        ASSERT_NEAR(-expected_raw, gr.x, 0.001f, "raw gyro matches at sync index");
        ASSERT_NEAR(expected_raw * 0.9f, a.x, 0.001f, "filtered accel matches");
        ASSERT_EQ((int)((count + 1) * 2500), (int)ts, "timestamp matches");

        read_idx = (read_idx + 1) % accel_buf.capacity();
        count++;
    }
    ASSERT_EQ(5, count, "telemetry read all 5 IMU samples");

    printf("  All IMU sync pattern tests passed\n");
}

// =========================================================================
// Test 11: Large wraparound (200 samples like IMU buffer)
// =========================================================================
void test_large_buffer_wraparound() {
    TEST_SECTION("Test 11: Large buffer wraparound (N=200)");

    RingBuffer<float, 200> buf;

    // Push 500 samples (2.5 full wraps)
    for (int i = 0; i < 500; i++) {
        buf.push(static_cast<float>(i));
    }

    ASSERT_EQ(200, buf.count(), "count capped at 200");
    ASSERT_TRUE(buf.full(), "full after 500 pushes");
    ASSERT_NEAR(499.0f, buf.latest(), 0.001f, "latest = 499");
    ASSERT_NEAR(498.0f, buf.get(1), 0.001f, "get(1) = 498");
    ASSERT_NEAR(300.0f, buf.get(199), 0.001f, "get(199) = oldest = 300");

    // Mean of latest 200 values: 300..499, avg = 399.5
    ASSERT_NEAR(399.5f, buf.mean(), 0.01f, "mean of 300..499 = 399.5");

    printf("  All large buffer tests passed\n");
}

// =========================================================================
// main
// =========================================================================
int main() {
    printf("RingBuffer<T,N> Unit Tests\n");
    printf("==========================\n");

    test_basic_scalar();
    test_get_indexing();
    test_wraparound();
    test_timestamps();
    test_raw_access();
    test_mean_scalar();
    test_mean_vec3();
    test_reset();
    test_get_latest_n();
    test_imu_sync_pattern();
    test_large_buffer_wraparound();

    printf("\n==========================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
