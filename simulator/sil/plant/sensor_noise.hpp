/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sensor_noise.hpp
 * @brief N0 sensor-noise model for the synthetic IMU (white Gaussian + startup
 *        bias + bias random walk). 合成IMU の N0 ノイズモデル。
 *
 * Layered on top of the clean synthetic IMU (Plant). Kept PHYSICS-FREE on purpose
 * so it unit-tests without MuJoCo, and SEEDED so the SIL stays deterministic
 * (same seed → same noise → same flight → same video; a comparison runs both
 * estimators on the same seed for a fair contrast). RESET_PLAN §13 (P5),
 * spec: firmware/vehicle_new/docs/noise_and_vibration_model.md §2-3.
 *
 * クリーンな合成IMU（Plant）に載せる N0 ノイズ。物理から切り離して MuJoCo 無しで
 * 単体テスト可能にし、シード付きで決定論を保つ（同じシード→同じノイズ→同じ飛行→同じ動画）。
 *
 * @design simulator/sil/RESET_PLAN.md §13 P5 — sensor noise N0   [--]
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace sil {

/// N0 IMU noise: white Gaussian + per-boot startup bias + slow bias random walk.
/// N0 IMU ノイズ: 白色ガウス＋起動時バイアス＋低速のバイアスランダムウォーク。
class SensorNoise {
public:
    struct Config {
        bool     enable = false;
        uint32_t seed   = 12345;        ///< RNG seed (determinism)
        // Static white-noise density [unit/√Hz] (datasheet class for N0).
        // 静的白色ノイズ密度 [単位/√Hz]（N0 はデータシート級）。
        float gyro_density   = 0.000122f;  ///< rad/s/√Hz
        float accel_density  = 0.00157f;   ///< m/s²/√Hz
        // Per-boot startup bias 1σ (drawn once at init). These are the RESIDUAL bias
        // AFTER the firmware's boot calibration — NOT the raw MEMS offset. A real
        // accel offset (~0.1–0.4 m/s²) is removed by the level-rest boot calibration
        // (ba_z≈2g, noise_and_vibration_model.md §3); what the estimator must handle
        // in flight is the small residual + the random walk below. Modeled small on
        // purpose: a SIL sweep showed a LARGE uncalibrated accel bias (≥0.1 m/s²)
        // destabilizes the ESKF attitude loop (tilt 0.02→5°, 0.05→13°, 0.10→47°),
        // while the complementary filter stays bounded — an ESKF accel-bias
        // robustness margin to watch, and the reason P6 should reproduce the boot
        // accel calibration so the full offset is captured, not surprise the filter.
        // 起動時バイアスの 1σ（init で1回抽選）＝ファーム起動校正「後」の残留バイアス
        // （生の MEMS オフセットではない）。生オフセット(~0.1–0.4)は水平静止の起動校正で
        // 除去される。SIL 掃引で大きな未校正 accel バイアス(≥0.1)が ESKF 姿勢ループを
        // 発散させると判明（相補は有界）→ 小さく模擬。P6 で起動校正を再現する動機。
        float gyro_bias_sigma  = 0.005f;   ///< rad/s  (residual after boot calibration)
        float accel_bias_sigma = 0.02f;    ///< m/s²   (residual after boot calibration)
        // Bias random-walk density [unit/√s].
        // バイアスランダムウォーク密度 [単位/√s]。
        float gyro_bias_rw  = 1.0e-4f;     ///< rad/s/√s
        float accel_bias_rw = 1.0e-3f;     ///< m/s²/√s
    };

    /// Seed the RNG and draw the per-boot startup bias + the first white sample.
    /// RNG をシードし、起動時バイアス＋最初の白色サンプルを抽選する。
    void init(const Config& c) {
        cfg_ = c;
        rng_.seed(c.seed);
        norm_.reset();
        for (int i = 0; i < 3; ++i) {
            gyro_bias_[i]  = c.enable ? c.gyro_bias_sigma  * norm_(rng_) : 0.0f;
            accel_bias_[i] = c.enable ? c.accel_bias_sigma * norm_(rng_) : 0.0f;
            gyro_white_[i]  = 0.0f;
            accel_white_[i] = 0.0f;
        }
        if (c.enable) drawWhite(0.0025f);   // prime the first sample (nominal dt)
    }

    /// Advance one step: random-walk the bias, then draw a fresh white-noise sample.
    /// 1ステップ進める: バイアスをランダムウォークさせ、新しい白色ノイズを抽選。
    void advance(float dt) {
        if (!cfg_.enable) return;
        const float sdt = std::sqrt(dt > 1.0e-6f ? dt : 1.0e-6f);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_[i]  += cfg_.gyro_bias_rw  * sdt * norm_(rng_);   // RW ∝ √dt
            accel_bias_[i] += cfg_.accel_bias_rw * sdt * norm_(rng_);
        }
        drawWhite(dt);
    }

    /// Add bias + white noise to a body-frame accel sample [m/s²] (no-op if disabled).
    /// 機体系の加速度サンプル [m/s²] にバイアス＋白色ノイズを加える（無効時は無操作）。
    void applyAccel(float a[3]) const {
        if (!cfg_.enable) return;
        for (int i = 0; i < 3; ++i) a[i] += accel_bias_[i] + accel_white_[i];
    }
    /// Add bias + white noise to a body-frame gyro sample [rad/s] (no-op if disabled).
    /// 機体系の角速度サンプル [rad/s] にバイアス＋白色ノイズを加える（無効時は無操作）。
    void applyGyro(float g[3]) const {
        if (!cfg_.enable) return;
        for (int i = 0; i < 3; ++i) g[i] += gyro_bias_[i] + gyro_white_[i];
    }

    bool enabled() const { return cfg_.enable; }
    const float* gyroBias()  const { return gyro_bias_; }   ///< test accessor
    const float* accelBias() const { return accel_bias_; }  ///< test accessor

private:
    /// White-noise sample with discrete σ = density / √dt (continuous density → per-sample).
    /// 離散 σ = 密度 / √dt の白色ノイズ（連続密度 → 1サンプルあたり）。
    void drawWhite(float dt) {
        const float inv_sdt = 1.0f / std::sqrt(dt > 1.0e-6f ? dt : 1.0e-6f);
        for (int i = 0; i < 3; ++i) {
            gyro_white_[i]  = cfg_.gyro_density  * inv_sdt * norm_(rng_);
            accel_white_[i] = cfg_.accel_density * inv_sdt * norm_(rng_);
        }
    }

    Config cfg_{};
    std::mt19937 rng_{12345};
    std::normal_distribution<float> norm_{0.0f, 1.0f};
    float gyro_bias_[3]   = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3]  = {0.0f, 0.0f, 0.0f};
    float gyro_white_[3]  = {0.0f, 0.0f, 0.0f};
    float accel_white_[3] = {0.0f, 0.0f, 0.0f};
};

}  // namespace sil
