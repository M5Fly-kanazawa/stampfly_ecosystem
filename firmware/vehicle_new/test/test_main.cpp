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
#include "autotune.hpp"
#include "data_stream_wire.hpp"   // Data Stream wire layout (vs udp_capture.py)

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

    float out = pid.compute(1.0f, 0.0f, 0.01f);
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
        pid.compute(1.0f, 0.0f, 0.1f);
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

    pid.compute(1.0f, 0.0f, 0.1f);
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

    float out = pid.compute(1.0f, 0.0f, 0.01f);
    ASSERT_NEAR(out, 5.0f, 1e-6f);  // Clamped to limit
}

TEST(pid_derivative_on_measurement)
{
    // D-on-M: a setpoint step must NOT kick the derivative; a measurement step
    // must produce a negative (opposing) derivative response.
    // 測定値微分: 目標値ステップは微分を蹴らない。測定値ステップには負（抑制方向）の
    // 微分応答が出る。
    sf::PID pid;
    pid.kp = 1.0f;
    pid.ti = 1000.0f;   // effectively no integral / 実質積分なし
    pid.td = 0.1f;
    pid.output_limit = 100.0f;

    // Settle at sp=0, meas=0 (first call only primes the D input).
    // sp=0, meas=0 で慣らす（初回は微分入力の初期化のみ）。
    pid.compute(0.0f, 0.0f, 0.01f);
    pid.compute(0.0f, 0.0f, 0.01f);

    // Setpoint step: output = P only (no derivative kick).
    // 目標値ステップ: 出力は P のみ（微分キックなし）。
    float out_sp_step = pid.compute(1.0f, 0.0f, 0.01f);
    ASSERT_NEAR(out_sp_step, 1.0f, 1e-4f);

    // Measurement step: derivative opposes the rise → output < pure P (= 0.5).
    // 測定値ステップ: 微分が上昇に抗う → 出力は純粋な P（0.5）より小さい。
    float out_meas_step = pid.compute(1.0f, 0.5f, 0.01f);
    ASSERT_TRUE(out_meas_step < 0.5f);
}

// =============================================================================
// Onboard autotune (sf_autotune) — mirror of the Python rate_sysid self-test:
// synthesize frequency points from a KNOWN plant, recover it, tune it, verify
// the margins. The numbers must match tools/log_analyzer/rate_sysid.py.
// オンボード自動チューニング — Python 自己テストの鏡像: 既知プラントから周波数点を
// 合成→復元→チューニング→余裕検証。数値は rate_sysid.py と一致すること。
// =============================================================================

TEST(autotune_fit_and_tune)
{
    // True plant: roll-axis spec inertia, 25 ms lag, 6 ms delay.
    // 真のプラント: ロール軸仕様慣性、遅れ25ms、むだ時間6ms。
    const float b_true = 1.0f / 9.16e-6f;
    const float T_true = 0.025f, L_true = 0.006f;

    // Synthetic stepped-sine measurement points (G = Y/U with U = 1).
    // 合成ステップドサイン測定点（U=1 で G=Y/U）。
    const float freqs_hz[] = {2, 3, 4.5f, 7, 10, 14, 20, 27, 35};
    sf::autotune::FreqPoint pts[9];
    for (int i = 0; i < 9; i++) {
        const float w = 2.0f * 3.14159265f * freqs_hz[i];
        // G(jw) = b e^{-jwL} / (jw(jwT+1)) computed inline / 直接計算
        const float nr = b_true * cosf(-w * L_true);
        const float ni = b_true * sinf(-w * L_true);
        const float dr = -w * w * T_true;     // jw(jwT+1) = -w^2 T + jw
        const float di = w;
        const float dd = dr * dr + di * di;
        pts[i].w  = w;
        pts[i].ur = 1.0f;  pts[i].ui = 0.0f;
        pts[i].yr = (nr * dr + ni * di) / dd;
        pts[i].yi = (ni * dr - nr * di) / dd;
    }

    sf::autotune::Plant plant{};
    ASSERT_TRUE(sf::autotune::fitPlant(pts, 9, 1.0f / 9.16e-6f, plant));
    ASSERT_NEAR(plant.b / b_true, 1.0f, 0.05f);
    ASSERT_NEAR(plant.T, T_true, 0.005f);
    ASSERT_NEAR(plant.L, L_true, 0.002f);

    sf::autotune::TuneResult tune{};
    ASSERT_TRUE(sf::autotune::tunePid(plant, 20.0f, 60.0f, 10.0f, tune));
    ASSERT_NEAR(tune.wc, 20.0f, 1.0f);        // crossover met / 交差達成
    ASSERT_NEAR(tune.pm_deg, 60.0f, 2.0f);    // phase margin met / 余裕達成
    ASSERT_TRUE(tune.gm_db > 6.0f);           // healthy gain margin / 健全GM
    ASSERT_TRUE(tune.kp > 0 && tune.ti > 0);
}

// =============================================================================
// Gyro-bias deviation clamp (EskfConfig::bg_deviation_max)
// ジャイロバイアス偏差クランプ
// =============================================================================

TEST(eskf_gyro_bias_deviation_clamp)
{
    // A persistent yaw-rotated mag (a magnetic disturbance below the chi2 gate)
    // drags bg_z through the ATT-BG cross-covariance. The clamp must stop it at
    // nominal + bg_deviation_max — the bounded-damage contract for the rate loop.
    // 持続的にヨー回転した磁気（χ²ゲート以下の磁気外乱）はATT-BGクロス共分散経由で
    // bg_z を引きずる。クランプは nominal + bg_deviation_max で止めること —
    // レートループへの被害有界化の契約。
    sf::EskfConfig cfg;
    cfg.use_mag = true;
    cfg.use_tof = false;
    cfg.use_baro = false;
    cfg.use_flow = false;
    cfg.bg_deviation_max = 0.02f;
    sf::EskfCore eskf;
    eskf.init(cfg);

    // Boot calibration seeds the nominal.
    // 起動校正がノミナルを種付けする。
    const sf::math::Vec3 nominal(0.005f, -0.003f, 0.004f);
    eskf.setGyroBias(nominal);

    // Level rest with a SLOWLY ROTATING mag field (0.05 rad/s — e.g. a drifting
    // magnetic disturbance): the gyro says "not rotating", the mag says
    // "rotating" — the only consistent explanation the filter has is a gyro-bias
    // error, so bg_z gets dragged toward the rotation rate. (A CONSTANT mag
    // offset would NOT do this: with no other yaw reference the yaw state simply
    // absorbs it and the bias barely moves.) 0.05 rad/s exceeds the 0.02 clamp,
    // so the clamp must be what stops the drag.
    // 水平静止＋「ゆっくり回転し続ける」磁気（0.05 rad/s — 漂う磁気外乱を模擬）:
    // ジャイロは「回転していない」、磁気は「回転中」と言う — フィルタに残る整合的な
    // 説明はジャイロバイアス誤差のみで、bg_z が回転レートへ引きずられる。
    // （「一定の」磁気オフセットではこうならない: 他にヨー参照が無ければヨー状態が
    // 吸収しバイアスはほぼ動かない。）0.05 rad/s はクランプ 0.02 を超えるため、
    // 引きずりを止めるのはクランプでなければならない。
    const float disturb_rate = 0.05f;               // [rad/s] mag-field rotation
    const sf::math::Vec3 gyro(nominal);             // rest: raw gyro = true bias
    const sf::math::Vec3 accel(0, 0, -9.80665f);    // level rest specific force

    for (int i = 0; i < 40000; i++) {               // 100 s at 400 Hz
        eskf.predict(accel, gyro, 0.0025f);
        if (i % 16 == 0) {                          // mag at 25 Hz
            const float th = disturb_rate * static_cast<float>(i) * 0.0025f;
            const sf::math::Vec3 mag_rot(
                cosf(th) * cfg.mag_ref.x - sinf(th) * cfg.mag_ref.y,
                sinf(th) * cfg.mag_ref.x + cosf(th) * cfg.mag_ref.y,
                cfg.mag_ref.z);
            eskf.updateMag(mag_rot);
        }
    }

    const sf::math::Vec3 bg = eskf.getGyroBias();
    // The disturbance must have dragged the bias to the clamp boundary
    // (the coupling is real, and only the clamp stops it)...
    // 外乱はバイアスをクランプ境界まで引きずっているはず
    // （結合は実在し、止めるのはクランプのみ）…
    ASSERT_TRUE(fabsf(bg.z - nominal.z) > 0.015f);
    // ...but never past the deviation limit (+ float-rounding epsilon).
    // …しかし偏差上限（＋浮動小数の丸め分）を超えてはならない。
    ASSERT_TRUE(fabsf(bg.x - nominal.x) <= cfg.bg_deviation_max + 1e-5f);
    ASSERT_TRUE(fabsf(bg.y - nominal.y) <= cfg.bg_deviation_max + 1e-5f);
    ASSERT_TRUE(fabsf(bg.z - nominal.z) <= cfg.bg_deviation_max + 1e-5f);
}

// =============================================================================
// Main
// =============================================================================

// =============================================================================
// Data Stream wire-format tests — the byte layout is the contract with the PC
// parser (tools/log_analyzer/udp_capture.py); these assert the exact offsets
// and the XOR checksum the parser verifies.
// Data Stream 電文テスト — バイトレイアウトは PC 側パーサ（udp_capture.py）との
// 契約。パーサが検証するオフセットと XOR チェックサムを正確に assert する。
// =============================================================================

TEST(wire_unified_layout)
{
    using namespace sf::datastream;

    sf::LogStreamSample samples[kSamplesPerPacket] = {};
    for (int i = 0; i < kSamplesPerPacket; ++i) {
        samples[i].timestamp    = 1000u + static_cast<uint32_t>(i);
        samples[i].gyro[0]      = 0.5f;
        samples[i].accel[2]     = -9.8f;
        samples[i].quat[0]      = 1.0f;
        samples[i].gyro_bias[1] = 0.0123f;     // → int16 123
        samples[i].pos[2]       = -0.8f;
        samples[i].vel[0]       = 0.25f;
        samples[i].rate_ref[2]  = -1.234f;     // → int16 -1234
    }

    UnifiedPacketBuilder builder;
    builder.begin(7, samples);
    const size_t length = builder.finish();
    const uint8_t* buf = builder.buffer();

    // Fixed part: header 4 + 8*80 + 8*28 + 8*6 + entry_count 1 + checksum 1 = 918.
    // 固定部: ヘッダ4 + 8*80 + 8*28 + 8*6 + entry_count 1 + checksum 1 = 918。
    ASSERT_TRUE(length == 4 + 8 * 80 + 8 * 28 + 8 * 6 + 1 + 1);

    // Header: pkt_id 0x50, seq=7 (LE u16), count=8 — udp_capture FMT_HEADER '<B H B'.
    ASSERT_TRUE(buf[0] == 0x50);
    ASSERT_TRUE(buf[1] == 7 && buf[2] == 0);
    ASSERT_TRUE(buf[3] == 8);

    // ImuEskf block starts at 4; sample 0 timestamp little-endian = 1000.
    uint32_t ts;
    memcpy(&ts, &buf[4], 4);
    ASSERT_TRUE(ts == 1000u);

    // PosVel block starts at 4 + 640 = 644 (udp_capture offset arithmetic).
    memcpy(&ts, &buf[644], 4);
    ASSERT_TRUE(ts == 1000u);
    float pos_z;
    memcpy(&pos_z, &buf[644 + 4 + 8], 4);
    ASSERT_NEAR(pos_z, -0.8f, 1e-6f);

    // RateRef block starts at 644 + 224 = 868; yaw int16 = −1234 (×1000).
    int16_t rate_yaw;
    memcpy(&rate_yaw, &buf[868 + 4], 2);
    ASSERT_TRUE(rate_yaw == -1234);

    // entry_count at 868 + 48 = 916; zero entries in this build.
    ASSERT_TRUE(buf[916] == 0);

    // Gyro bias quantization: 0.0123 × 10000 = 123 (int16 at imu offset 68+2).
    int16_t bias;
    memcpy(&bias, &buf[4 + 68 + 2], 2);
    ASSERT_TRUE(bias == 123);

    // XOR checksum: parser computes xor(buf[0..len-2]) == buf[len-1].
    ASSERT_TRUE(xorChecksum(buf, length - 1) == buf[length - 1]);
}

TEST(wire_unified_entries)
{
    using namespace sf::datastream;

    sf::LogStreamSample samples[kSamplesPerPacket] = {};
    UnifiedPacketBuilder builder;
    builder.begin(0, samples);

    WireControl control = {};
    control.timestamp_us = 42;
    control.throttle     = 0.5f;
    ASSERT_TRUE(builder.addEntry(kPktControl, &control, sizeof(control)));

    WireCtrlRef ctrl_ref = {};
    ctrl_ref.flight_mode = 2;   // ALT_HOLD
    ASSERT_TRUE(builder.addEntry(kPktCtrlRef, &ctrl_ref, sizeof(ctrl_ref)));

    const size_t length = builder.finish();
    const uint8_t* buf = builder.buffer();

    // entry_count = 2; first entry [id][size][payload] right after it.
    ASSERT_TRUE(buf[916] == 2);
    ASSERT_TRUE(buf[917] == kPktControl && buf[918] == sizeof(WireControl));
    uint32_t ts;
    memcpy(&ts, &buf[919], 4);
    ASSERT_TRUE(ts == 42u);

    // Second entry follows the first; CtrlRef data_size 30 selects v3 on the PC.
    const size_t second = 919 + sizeof(WireControl);
    ASSERT_TRUE(buf[second] == kPktCtrlRef && buf[second + 1] == 30);

    ASSERT_TRUE(length == 917 + 2 + sizeof(WireControl) + 2 + sizeof(WireCtrlRef) + 1);
    ASSERT_TRUE(xorChecksum(buf, length - 1) == buf[length - 1]);
}

TEST(wire_status_packet)
{
    using namespace sf::datastream;

    WireStatusPayload payload = {};
    payload.uptime_ms    = 5000;
    payload.voltage      = 4.1f;
    payload.flight_state = 1;
    payload.pid_gains[0] = 1.83e-4f;

    uint8_t buf[64];
    const size_t length = buildStatusPacket(buf, 3, payload);

    // 53 bytes total — the size udp_capture.py expects for 0x4F with gains.
    ASSERT_TRUE(length == 53);
    ASSERT_TRUE(buf[0] == kPktStatus);
    ASSERT_TRUE(xorChecksum(buf, length - 1) == buf[length - 1]);

    float volt;
    memcpy(&volt, &buf[4 + 4], 4);
    ASSERT_NEAR(volt, 4.1f, 1e-6f);
}

TEST(wire_quantize_saturation)
{
    using namespace sf::datastream;
    ASSERT_TRUE(quantize(10.0f, 10000.0f) == 32767);     // saturate high
    ASSERT_TRUE(quantize(-10.0f, 10000.0f) == -32767);   // saturate low
    ASSERT_TRUE(quantize(0.5f, 1000.0f) == 500);
}

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
    run_eskf_gyro_bias_deviation_clamp();
    run_autotune_fit_and_tune();

    printf("\n[PID]\n");
    run_pid_proportional();
    run_pid_integral();
    run_pid_reset();
    run_pid_output_limit();
    run_pid_derivative_on_measurement();

    printf("\n[DataStream wire]\n");
    run_wire_unified_layout();
    run_wire_unified_entries();
    run_wire_status_packet();
    run_wire_quantize_saturation();

    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           tests_passed, tests_run, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
