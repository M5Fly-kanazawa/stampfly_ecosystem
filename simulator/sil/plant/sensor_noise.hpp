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
 * @design simulator/sil/RESET_PLAN.md §13 P5 — sensor noise N0   [OK]
 *   Verified on the emulator: N0 hover stays G2-bounded (alt std ~2.7 cm over a 90 s
 *   hold, attitude ~4° tilt, no divergence) across 7 seeds, runs are byte-identical
 *   per seed, and the white σ tracks the 400 Hz firmware read rate (white_dt), not
 *   the 4 kHz physics substep. エミュレータで検証済（90s保持で alt std~2.7cm 有界、
 *   7シードで決定論、白色σはファーム読み取りレートに追従）。
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
        // IMU read period the FIRMWARE samples at [s] — the white-noise discretization
        // rate (per-sample σ = density/√white_dt). DECOUPLED from the physics substep
        // ON PURPOSE: after the timebase fix the Plant substeps at the model timestep
        // (0.25 ms / 4 kHz) and calls advance() per substep, but the firmware reads the
        // BMI270 at 400 Hz (2.5 ms). The white σ the ESKF's R is tuned for is the
        // per-400-Hz-sample σ; discretizing white at the 4 kHz substep would inflate it
        // by √(2.5/0.25)=√10≈3.16×. Fixing white_dt to the firmware read rate keeps the
        // effective σ the firmware sees correct regardless of how finely the Plant steps.
        // ファームが IMU をサンプルする周期 [s]＝白色ノイズの離散化レート（1サンプルσ=density/√white_dt）。
        // 物理 substep（時間基準修正後 0.25ms/4kHz, advance は substep 毎）から意図的に分離する。
        // ファームは BMI270 を 400Hz(2.5ms) で読み、ESKF の R はこの 400Hz サンプルσに合わせて
        // ある。白色を 4kHz substep で離散化すると σ が √10≈3.16倍に膨らむ。white_dt をファーム
        // 読み取りレートに固定し、物理刻みの細かさに依らず実効σを正しく保つ。
        float white_dt = 0.0025f;          ///< 1/400 Hz IMU sample period [s]
        // N1 — throttle-dependent vibration (per-axis): σ_axis = K[axis]·duty². The
        // DOMINANT in-flight IMU noise is motor/propeller vibration, not the static
        // density (datasheet → in-flight is ×79 gyro, ×191 accel — noise_and_vibration
        // _model.md §1), so N0 alone is unrealistic in flight. Per-axis K because the
        // legacy hover02 backtest showed gyro Z is ~16× smaller than X/Y and accel Z
        // ~13% larger (an isotropic K mis-estimates each axis). duty is the mean of the
        // four motor commands (set per substep by the Plant). N1 treats the vibration
        // as BROADBAND white scaled by duty² (per-sample rms at the read rate); the
        // band-limited (500–667 Hz, aliasing at the 400 Hz read) refinement is the N2
        // tier and is NOT modeled here. Disabled → N0 (static only), byte-identical.
        // N1 — スロットル依存振動（軸別）: σ_axis = K[axis]·duty²。飛行中の支配的 IMU
        // ノイズはモータ/プロペラ振動で（データシート比 gyro×79/accel×191）、N0 だけでは
        // 飛行中は非現実的。軸別 K（hover02 実測で gyro Z は X/Y の約1/16、accel Z は約13%大）。
        // duty は4モータ指令の平均（Plant が substep 毎に設定）。N1 は振動を duty² でスケール
        // した広帯域白色として扱う（読み取りレートの1サンプル rms）。帯域制限（500–667Hz、
        // 400Hz 読みでエイリアシング）は N2 の精緻化で本段では非モデル化。無効時は N0＝byte-identical。
        bool  vib_enable = false;                       ///< N1: throttle vibration on
        float vib_accel_k[3] = {3.96f, 2.35f, 5.64f};   ///< σ_accel,axis = K·duty² [m/s²]
        float vib_gyro_k[3]  = {1.08f, 0.83f, 0.15f};   ///< σ_gyro,axis  = K·duty² [rad/s]
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
        if (c.enable) drawWhite();   // prime the first sample (at white_dt)
    }

    /// Advance one physics substep of length dt [s]: random-walk the bias by √dt
    /// (so the walk accumulates correctly however finely the Plant substeps), then
    /// draw a fresh white-noise sample at the FIXED firmware read rate (white_dt),
    /// NOT at dt — see Config::white_dt for why the white σ is substep-independent.
    /// 物理 substep（長さ dt [s]）を1回進める: バイアスを √dt でランダムウォーク
    /// （刻みの細かさに依らず正しく累積）させ、白色は固定のファーム読み取りレート
    /// （white_dt、dt ではない）で新規抽選する。理由は Config::white_dt 参照。
    void advance(float dt) {
        if (!cfg_.enable) return;
        const float sdt = std::sqrt(dt > 1.0e-6f ? dt : 1.0e-6f);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_[i]  += cfg_.gyro_bias_rw  * sdt * norm_(rng_);   // RW ∝ √dt
            accel_bias_[i] += cfg_.accel_bias_rw * sdt * norm_(rng_);
        }
        drawWhite();
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

    /// Set the current throttle (mean motor duty in [0,1]) used to scale the N1
    /// throttle-dependent vibration σ on the NEXT drawn sample. No-op for N0.
    /// 現在のスロットル（4モータ平均 duty [0,1]）を設定。次に引く N1 振動 σ のスケールに使う。
    void setThrottle(float duty01) { throttle_ = duty01; }

    bool enabled() const { return cfg_.enable; }
    const float* gyroBias()  const { return gyro_bias_; }   ///< test accessor
    const float* accelBias() const { return accel_bias_; }  ///< test accessor

private:
    /// White-noise sample with discrete σ = density / √white_dt (continuous density →
    /// per-sample at the firmware IMU read rate, independent of the physics substep).
    /// 離散 σ = 密度 / √white_dt の白色ノイズ（連続密度 → ファーム IMU 読み取りレートの
    /// 1サンプルあたり。物理 substep には依存しない）。
    void drawWhite() {
        const float wdt = cfg_.white_dt > 1.0e-6f ? cfg_.white_dt : 1.0e-6f;
        const float inv_sdt = 1.0f / std::sqrt(wdt);
        // N1 vibration σ scales with duty² (σ_axis = K[axis]·throttle²). This is a
        // per-sample rms (NOT a density), so it is added directly without 1/√dt.
        // N1 振動 σ は duty² でスケール（σ_axis=K·throttle²）。これは1サンプル rms
        // （密度ではない）ゆえ 1/√dt を掛けず直接加える。
        const float duty2 = cfg_.vib_enable ? throttle_ * throttle_ : 0.0f;
        for (int i = 0; i < 3; ++i) {
            gyro_white_[i]  = cfg_.gyro_density  * inv_sdt * norm_(rng_);
            accel_white_[i] = cfg_.accel_density * inv_sdt * norm_(rng_);
            if (cfg_.vib_enable) {
                gyro_white_[i]  += cfg_.vib_gyro_k[i]  * duty2 * norm_(rng_);
                accel_white_[i] += cfg_.vib_accel_k[i] * duty2 * norm_(rng_);
            }
        }
    }

    Config cfg_{};
    float throttle_ = 0.0f;   ///< current mean motor duty [0,1] for N1 vibration σ
    std::mt19937 rng_{12345};
    std::normal_distribution<float> norm_{0.0f, 1.0f};
    float gyro_bias_[3]   = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3]  = {0.0f, 0.0f, 0.0f};
    float gyro_white_[3]  = {0.0f, 0.0f, 0.0f};
    float accel_white_[3] = {0.0f, 0.0f, 0.0f};
};

}  // namespace sil
