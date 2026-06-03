/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file noise_test.cpp
 * @brief Unit tests for the N0 sensor-noise model (SensorNoise). MuJoCo-free.
 *        N0 センサノイズモデル（SensorNoise）の単体テスト。MuJoCo 不要。
 *
 * Validates the four properties RESET_PLAN §13 P5 requires:
 *   1. statistics  — measured white-noise σ ≈ density/√dt, mean ≈ 0
 *   2. off = clean — disabled model adds exactly nothing
 *   3. startup bias — constant per boot, differs across seeds
 *   4. bias RW     — ensemble std grows ∝ √t
 *   5. determinism — same seed → identical sequence; different seed → differs
 *
 * RESET_PLAN §13 P5 が要求する性質を検証する（統計・off=クリーン・起動バイアス・
 * バイアスRW の√t成長・決定論）。
 */

#include <cmath>
#include <cstdio>
#include <vector>

#include "sensor_noise.hpp"

using sil::SensorNoise;

namespace {

int g_failures = 0;

void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

// Mean and (population) standard deviation of a sample vector.
// サンプル列の平均と（母）標準偏差。
void meanStd(const std::vector<float>& v, double& mean, double& std) {
    double s = 0.0;
    for (float x : v) s += x;
    mean = s / static_cast<double>(v.size());
    double s2 = 0.0;
    for (float x : v) s2 += (x - mean) * (x - mean);
    std = std::sqrt(s2 / static_cast<double>(v.size()));
}

constexpr float DT = 0.0025f;  // 400 Hz

// --- 1. white-noise statistics: σ_d = density/√dt, mean ≈ 0 ---
void test_white_statistics() {
    SensorNoise::Config c;
    c.enable = true;
    c.seed = 1;
    c.gyro_density = 0.01f;
    c.accel_density = 0.05f;
    c.gyro_bias_sigma = 0.0f;     // isolate the white term
    c.accel_bias_sigma = 0.0f;
    c.gyro_bias_rw = 0.0f;
    c.accel_bias_rw = 0.0f;

    SensorNoise n;
    n.init(c);
    const int N = 200000;
    std::vector<float> ax, gx;
    ax.reserve(N);
    gx.reserve(N);
    for (int k = 0; k < N; ++k) {
        n.advance(DT);
        float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
        n.applyAccel(a);
        n.applyGyro(g);
        ax.push_back(a[0]);
        gx.push_back(g[0]);
    }
    double am, as, gm, gs;
    meanStd(ax, am, as);
    meanStd(gx, gm, gs);
    const double exp_a = c.accel_density / std::sqrt(DT);  // = 1.0
    const double exp_g = c.gyro_density / std::sqrt(DT);   // = 0.2
    std::printf("    accel: std=%.4f (exp %.4f) mean=%.4f\n", as, exp_a, am);
    std::printf("    gyro : std=%.4f (exp %.4f) mean=%.4f\n", gs, exp_g, gm);
    check(std::fabs(as - exp_a) / exp_a < 0.03, "accel white σ ≈ density/√dt (±3%)");
    check(std::fabs(gs - exp_g) / exp_g < 0.03, "gyro  white σ ≈ density/√dt (±3%)");
    check(std::fabs(am) < 0.02 * exp_a, "accel white mean ≈ 0");
    check(std::fabs(gm) < 0.02 * exp_g, "gyro  white mean ≈ 0");
}

// --- 1b. white σ tracks the firmware IMU read rate, NOT the physics substep ---
// After the timebase fix the Plant substeps at 4 kHz (advance called per 0.25 ms
// substep) while the firmware reads the IMU at 400 Hz. The σ the firmware SEES must
// stay the 400-Hz-sample σ (density/√0.0025), not inflate by √10 to the 4-kHz σ.
// This pins the Config::white_dt decoupling that keeps the effective σ correct.
// 時間基準修正後 Plant は 4kHz で substep（advance は 0.25ms 毎）するが、ファームは IMU を
// 400Hz で読む。ファームが見る σ は 400Hz サンプルσ（density/√0.0025）に留まり、√10 倍に
// 膨らんではならない。white_dt 分離が実効σを正しく保つことを固定する。
void test_white_substep_decoupled() {
    SensorNoise::Config c;
    c.enable = true;
    c.seed = 7;
    c.gyro_density = 0.01f;
    c.accel_density = 0.05f;
    c.gyro_bias_sigma = 0.0f;      // isolate the white term
    c.accel_bias_sigma = 0.0f;
    c.gyro_bias_rw = 0.0f;
    c.accel_bias_rw = 0.0f;
    // c.white_dt stays at its default 0.0025 (firmware 400 Hz read rate).

    SensorNoise n;
    n.init(c);
    const float H = 0.00025f;      // 4 kHz physics substep (model timestep)
    const int SUBSTEPS = 10;       // 10 substeps per 400 Hz firmware read
    const int N = 200000;
    std::vector<float> ax, gx;
    ax.reserve(N);
    gx.reserve(N);
    for (int k = 0; k < N; ++k) {
        for (int s = 0; s < SUBSTEPS; ++s) n.advance(H);  // Plant substeps at 4 kHz
        float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
        n.applyAccel(a);                                   // firmware reads at 400 Hz
        n.applyGyro(g);
        ax.push_back(a[0]);
        gx.push_back(g[0]);
    }
    double am, as, gm, gs;
    meanStd(ax, am, as);
    meanStd(gx, gm, gs);
    const double exp_a = c.accel_density / std::sqrt(0.0025);  // 400 Hz σ (NOT 4 kHz)
    const double exp_g = c.gyro_density / std::sqrt(0.0025);
    std::printf("    accel: std=%.4f (exp %.4f, 4kHz-σ would be %.4f)\n",
                as, exp_a, c.accel_density / std::sqrt(H));
    std::printf("    gyro : std=%.4f (exp %.4f, 4kHz-σ would be %.4f)\n",
                gs, exp_g, c.gyro_density / std::sqrt(H));
    check(std::fabs(as - exp_a) / exp_a < 0.03, "accel white σ tracks 400 Hz read, not 4 kHz substep");
    check(std::fabs(gs - exp_g) / exp_g < 0.03, "gyro  white σ tracks 400 Hz read, not 4 kHz substep");
}

// --- 2. disabled model adds exactly nothing ---
void test_off_is_clean() {
    SensorNoise::Config c;       // enable defaults to false
    SensorNoise n;
    n.init(c);
    n.advance(DT);
    float a[3] = {1.0f, -2.0f, 3.0f}, g[3] = {0.1f, 0.2f, 0.3f};
    n.applyAccel(a);
    n.applyGyro(g);
    bool clean = (a[0] == 1.0f && a[1] == -2.0f && a[2] == 3.0f &&
                  g[0] == 0.1f && g[1] == 0.2f && g[2] == 0.3f);
    check(clean, "disabled noise is a no-op (clean path byte-identical)");
}

// --- 3. startup bias: constant per boot, differs across seeds ---
void test_startup_bias() {
    SensorNoise::Config c;
    c.enable = true;
    c.gyro_density = 0.0f;        // isolate the bias term
    c.accel_density = 0.0f;
    c.gyro_bias_sigma = 0.01f;
    c.accel_bias_sigma = 0.10f;
    c.gyro_bias_rw = 0.0f;
    c.accel_bias_rw = 0.0f;

    c.seed = 1;
    SensorNoise n1;
    n1.init(c);
    float a0[3] = {0, 0, 0};
    n1.applyAccel(a0);                 // = startup bias
    bool constant = true;
    for (int k = 0; k < 1000; ++k) {
        n1.advance(DT);
        float a[3] = {0, 0, 0};
        n1.applyAccel(a);
        if (std::fabs(a[0] - a0[0]) > 1e-9f) constant = false;
    }
    check(constant, "startup bias constant when density+RW are 0");

    c.seed = 2;
    SensorNoise n2;
    n2.init(c);
    float b[3] = {0, 0, 0};
    n2.applyAccel(b);
    check(std::fabs(a0[0] - b[0]) > 1e-6f, "different seed → different startup bias");
}

// --- 4. bias random walk: ensemble std grows ∝ √t ---
void test_bias_random_walk() {
    SensorNoise::Config c;
    c.enable = true;
    c.gyro_density = 0.0f;
    c.accel_density = 0.0f;
    c.gyro_bias_sigma = 0.0f;
    c.accel_bias_sigma = 0.0f;
    c.gyro_bias_rw = 0.0f;
    c.accel_bias_rw = 0.01f;     // m/s²/√s

    const int S = 3000;          // ensemble size
    auto ensembleStd = [&](int K) {
        std::vector<float> finals;
        finals.reserve(S);
        for (int s = 0; s < S; ++s) {
            c.seed = 1000 + s;
            SensorNoise n;
            n.init(c);
            for (int k = 0; k < K; ++k) n.advance(DT);
            finals.push_back(n.accelBias()[0]);
        }
        double m, sd;
        meanStd(finals, m, sd);
        return sd;
    };
    const double s100 = ensembleStd(100);
    const double s400 = ensembleStd(400);
    const double exp100 = c.accel_bias_rw * std::sqrt(100 * DT);  // R·√t
    const double ratio = s400 / s100;                              // expect √4 = 2
    std::printf("    RW std@100=%.5f (exp %.5f)  std@400=%.5f  ratio=%.3f (exp 2.0)\n",
                s100, exp100, s400, ratio);
    check(std::fabs(s100 - exp100) / exp100 < 0.15, "RW std@100 ≈ R·√t (±15%)");
    check(std::fabs(ratio - 2.0) < 0.3, "RW std grows ∝ √t (ratio ≈ 2)");
}

// --- 5. determinism: same seed identical, different seed differs ---
void test_determinism() {
    SensorNoise::Config c;
    c.enable = true;
    c.seed = 42;

    auto sequence = [](SensorNoise::Config cfg) {
        SensorNoise n;
        n.init(cfg);
        std::vector<float> seq;
        for (int k = 0; k < 500; ++k) {
            n.advance(DT);
            float a[3] = {0, 0, 0};
            n.applyAccel(a);
            seq.push_back(a[0]);
        }
        return seq;
    };
    std::vector<float> s1 = sequence(c);
    std::vector<float> s2 = sequence(c);
    bool identical = (s1 == s2);
    check(identical, "same seed → identical sequence");

    SensorNoise::Config c2 = c;
    c2.seed = 43;
    std::vector<float> s3 = sequence(c2);
    bool differs = (s1 != s3);
    check(differs, "different seed → different sequence");
}

}  // namespace

int main() {
    std::printf("[noise_test] N0 sensor-noise model (SensorNoise)\n");
    test_white_statistics();
    test_white_substep_decoupled();
    test_off_is_clean();
    test_startup_bias();
    test_bias_random_walk();
    test_determinism();
    std::printf("[noise_test] %s (%d failure%s)\n",
                g_failures == 0 ? "OK" : "FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
