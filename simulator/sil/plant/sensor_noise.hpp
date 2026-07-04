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
 * spec: firmware/vehicle/docs/noise_and_vibration_model.md §2-3.
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
        // N2 — band-limit the vibration to [f_low,f_high] (motor/prop band, ~500–667 Hz
        // at hover). Real vibration is NOT broadband: it sits near the rotor/blade-pass
        // frequencies, which the firmware's LPF/AAF attenuates — that is WHY filtering
        // works. N1 (broadband) instead leaks energy into the low band the LPF cannot
        // remove, so it corrupts the estimator more than reality. N2 generates the
        // vibration through a 2nd-order band-pass at the PHYSICS substep rate (4 kHz,
        // Nyquist 2 kHz — the band fits), then the firmware's 400 Hz IMU read ALIASES it
        // down (the band folds to ~100–177 Hz, still above a typical attitude LPF cut),
        // so the same K·duty² energy now lands where the firmware can filter it. Total
        // per-sample rms stays K·duty² (energy preserved, only the spectrum changes).
        // N2 — 振動を [f_low,f_high]（ホバーで ~500–667Hz のロータ/ブレード帯）に帯域制限する。
        // 実振動は広帯域でなくこの帯域に集中し、ファームの LPF/AAF が落とす＝フィルタが効く理由。
        // N1（広帯域）は LPF が除けない低域にも漏れ、実機以上に推定器を乱す。N2 は物理 substep
        // レート（4kHz、Nyquist 2kHz＝帯域が収まる）で2次帯域通過を通し、ファーム 400Hz 読みで
        // エイリアシング（帯域は ~100–177Hz に折り返る＝姿勢 LPF カットより上）。同じ K·duty²
        // エネルギーがファームの落とせる所に来る（1サンプル rms は K·duty² 不変、周波数だけ変わる）。
        bool  vib_bandlimit = false;       ///< N2: band-limit the vibration spectrum
        float vib_freq_low  = 500.0f;      ///< band-pass lower bound [Hz]
        float vib_freq_high = 667.0f;      ///< band-pass upper bound [Hz]
        // N2 — observation noise on the range/baro sensors. σ is the per-sample stddev
        // of the SENSOR OUTPUT (ToF distance [m], baro altitude [m]), added on top of the
        // realistic device models (VL53 quantization, BMP280 decode). NOTE: these are the
        // PLANT's TRUE sensor noise, which is SMALLER than the ESKF's R-table σ in doc §3
        // (ToF R-σ=0.03, baro=0.10). The R-table values are the filter's ASSUMED noise,
        // deliberately inflated for robustness; the plant injects the real datasheet-class
        // noise (ToF datasheet 0.005–0.03, low end at close range). Using the inflated
        // R-σ as the true ToF noise (0.03 > the 13 mm ground rest height) makes the
        // near-ground reading go negative/invalid and breaks the boot ToF gate.
        // N2 — 測距/気圧センサの観測ノイズ。σ はセンサ出力（ToF 距離[m]・気圧高度[m]）の1サンプル
        // 標準偏差。これは「プラントの真のノイズ」で、doc §3 の R 表 σ（ToF 0.03・baro 0.10＝
        // フィルタが仮定する値、ロバスト性で膨らませてある）より小さい。R 値を真のノイズに使うと
        // 0.03 > 地上静止高 13mm となり近地面で負/無効になり起動 ToF 判定を壊す。
        bool  obs_enable = false;          ///< N2: ToF/baro observation noise on
        float tof_sigma  = 0.01f;          ///< ToF range noise σ [m] (datasheet-class, < ESKF R-σ)
        float baro_sigma = 0.1f;           ///< baro altitude noise σ [m] (telemetry; not fused)
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
            gyro_vib_[i]  = 0.0f;
            accel_vib_[i] = 0.0f;
        }
        for (int i = 0; i < 6; ++i) { bq_z1_[i] = 0.0f; bq_z2_[i] = 0.0f; }
        tof_noise_ = 0.0f;
        baro_noise_ = 0.0f;
        last_vib_dt_ = -1.0f;   // force band-pass coefficient (re)compute on first advance
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
        // N2 band-limited vibration: step the per-axis band-pass at the substep rate
        // (so the band fits below the substep Nyquist; the firmware's 400 Hz read then
        // aliases it). Broadband N1 is added inside drawWhite instead. No-op for N0.
        // N2 帯域制限振動: 各軸の帯域通過を substep レートで進める（帯域が substep ナイキスト
        // 内に収まり、ファーム 400Hz 読みで折り返す）。広帯域 N1 は drawWhite 側。N0 では無操作。
        if (cfg_.vib_enable && cfg_.vib_bandlimit) stepVibration(dt);
        // N2 observation noise: pre-draw this step's ToF/baro samples (per-sample σ);
        // the const tof()/baro() add the latest. No-op when obs noise is disabled.
        // N2 観測ノイズ: 今ステップの ToF/baro サンプルを事前抽選（1サンプルσ）。const な
        // tof()/baro() が最新値を足す。無効時は無操作。
        if (cfg_.obs_enable) {
            tof_noise_  = cfg_.tof_sigma  * norm_(rng_);
            baro_noise_ = cfg_.baro_sigma * norm_(rng_);
        }
        drawWhite();
    }

    /// Add bias + white + band-limited vibration to a body-frame accel sample [m/s²]
    /// (no-op if disabled). accel_vib_ is zero except under N2, so N0/N1 are unchanged.
    /// 機体系の加速度サンプル [m/s²] にバイアス＋白色＋帯域制限振動を加える（無効時は無操作）。
    /// accel_vib_ は N2 以外ゼロゆえ N0/N1 は不変。
    void applyAccel(float a[3]) const {
        if (!cfg_.enable) return;
        for (int i = 0; i < 3; ++i) a[i] += accel_bias_[i] + accel_white_[i] + accel_vib_[i];
    }
    /// Add bias + white + band-limited vibration to a body-frame gyro sample [rad/s]
    /// (no-op if disabled). 機体系の角速度サンプル [rad/s] にバイアス＋白色＋帯域制限振動を加える。
    void applyGyro(float g[3]) const {
        if (!cfg_.enable) return;
        for (int i = 0; i < 3; ++i) g[i] += gyro_bias_[i] + gyro_white_[i] + gyro_vib_[i];
    }
    /// Add the pre-drawn observation noise to a ToF range [m] (no-op unless N2 obs on).
    /// ToF 距離 [m] に事前抽選の観測ノイズを加える（N2 観測ノイズ ON 時のみ）。
    void applyTof(float& distance) const {
        if (cfg_.enable && cfg_.obs_enable) distance += tof_noise_;
    }
    /// Add the pre-drawn observation noise to a baro altitude [m] (no-op unless N2 obs on).
    /// 気圧高度 [m] に事前抽選の観測ノイズを加える（N2 観測ノイズ ON 時のみ）。
    void applyBaro(float& altitude) const {
        if (cfg_.enable && cfg_.obs_enable) altitude += baro_noise_;
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
        // N1 BROADBAND vibration σ scales with duty² (σ_axis = K[axis]·throttle²). This
        // is a per-sample rms (NOT a density), added directly without 1/√dt. Only for
        // N1 (vib on, band-limit OFF); under N2 the vibration is the band-pass output
        // added in applyAccel/applyGyro instead (so the spectrum is shaped, not flat).
        // N1 広帯域振動 σ は duty² でスケール（σ_axis=K·throttle²）。1サンプル rms（密度でない）
        // ゆえ 1/√dt なしで直接加える。N1（振動ON・帯域制限OFF）のみ。N2 では帯域通過出力を
        // applyAccel/applyGyro 側で足す（スペクトルが平坦でなく整形される）。
        const bool broadband = cfg_.vib_enable && !cfg_.vib_bandlimit;
        const float duty2 = broadband ? throttle_ * throttle_ : 0.0f;
        for (int i = 0; i < 3; ++i) {
            gyro_white_[i]  = cfg_.gyro_density  * inv_sdt * norm_(rng_);
            accel_white_[i] = cfg_.accel_density * inv_sdt * norm_(rng_);
            if (broadband) {
                gyro_white_[i]  += cfg_.vib_gyro_k[i]  * duty2 * norm_(rng_);
                accel_white_[i] += cfg_.vib_accel_k[i] * duty2 * norm_(rng_);
            }
        }
    }

    /// Step the per-axis 2nd-order band-pass one substep (dt): recompute the biquad
    /// coefficients if the rate changed, drive each axis with fresh unit-variance white,
    /// and store the band-limited vibration scaled to σ_axis = K[axis]·duty². The output
    /// is normalized to unit rms by inv_gain so the K·duty² scaling sets the true rms.
    /// 各軸の2次帯域通過を1 substep（dt）進める: レート変化時に係数を再計算し、各軸を
    /// 単位分散白色で駆動、σ_axis=K·duty² にスケールした帯域制限振動を保存。出力は inv_gain で
    /// 単位 rms に正規化済みゆえ K·duty² がそのまま rms になる。
    void stepVibration(float dt) {
        const float fs = 1.0f / (dt > 1.0e-6f ? dt : 1.0e-6f);
        if (dt != last_vib_dt_) { computeBandpass(fs); last_vib_dt_ = dt; }
        const float duty2 = throttle_ * throttle_;
        for (int i = 0; i < 3; ++i) {
            gyro_vib_[i]  = cfg_.vib_gyro_k[i]  * duty2 * biquadStep(i,     norm_(rng_));
            accel_vib_[i] = cfg_.vib_accel_k[i] * duty2 * biquadStep(3 + i, norm_(rng_));
        }
    }

    /// One Direct-Form-II-Transposed biquad step for channel ch (0–2 gyro, 3–5 accel),
    /// returning the unit-rms-normalized band-pass output. b1 ≡ 0 for a band-pass.
    /// チャンネル ch（0–2 gyro, 3–5 accel）の DF2T バイカッド1ステップ。単位 rms 正規化した
    /// 帯域通過出力を返す。帯域通過は b1≡0。
    float biquadStep(int ch, float x) {
        const float y = bq_b0_ * x + bq_z1_[ch];
        bq_z1_[ch] = -bq_a1_ * y + bq_z2_[ch];      // b1 = 0
        bq_z2_[ch] =  bq_b2_ * x - bq_a2_ * y;
        return y * bq_inv_gain_;
    }

    /// Compute the RBJ band-pass (constant 0 dB peak gain) coefficients at sample rate
    /// fs for the band [f_low,f_high] (center f0=√(f_low·f_high), Q=f0/bandwidth), then
    /// the H2 norm (= output rms for unit-variance white in) from the impulse response,
    /// so inv_gain normalizes the band-pass output to unit rms.
    /// 標本化レート fs での RBJ 帯域通過（ピーク 0dB 一定）係数を帯域 [f_low,f_high]
    /// （中心 f0=√(f_low·f_high), Q=f0/帯域幅）に対し計算し、インパルス応答から H2 ノルム
    /// （=単位分散白色入力時の出力 rms）を求めて inv_gain で単位 rms に正規化する。
    void computeBandpass(float fs) {
        const float f0 = std::sqrt(cfg_.vib_freq_low * cfg_.vib_freq_high);
        const float bw = cfg_.vib_freq_high - cfg_.vib_freq_low;
        const float Q  = (bw > 1.0f) ? f0 / bw : 1.0f;
        const float w0 = 2.0f * 3.14159265358979f * f0 / fs;
        const float cw = std::cos(w0), sw = std::sin(w0);
        const float alpha = sw / (2.0f * Q);
        const float a0 = 1.0f + alpha;
        bq_b0_ =  alpha / a0;
        bq_b2_ = -alpha / a0;
        bq_a1_ = -2.0f * cw / a0;
        bq_a2_ = (1.0f - alpha) / a0;
        // H2 norm via the impulse response: variance of the output for unit-variance
        // white input = Σ h[n]². Run until it decays (band-pass with Q~3.5 settles in
        // ~100 samples; 4000 is ample), then inv_gain = 1/√(Σh²).
        // インパルス応答による H2 ノルム: 単位分散白色入力の出力分散 = Σh[n]²。
        float z1 = 0.0f, z2 = 0.0f, energy = 0.0f;
        for (int n = 0; n < 4000; ++n) {
            const float x = (n == 0) ? 1.0f : 0.0f;
            const float y = bq_b0_ * x + z1;
            z1 = -bq_a1_ * y + z2;
            z2 =  bq_b2_ * x - bq_a2_ * y;
            energy += y * y;
        }
        bq_inv_gain_ = (energy > 1.0e-12f) ? 1.0f / std::sqrt(energy) : 1.0f;
    }

    Config cfg_{};
    float throttle_ = 0.0f;   ///< current mean motor duty [0,1] for vibration σ
    std::mt19937 rng_{12345};
    std::normal_distribution<float> norm_{0.0f, 1.0f};
    float gyro_bias_[3]   = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3]  = {0.0f, 0.0f, 0.0f};
    float gyro_white_[3]  = {0.0f, 0.0f, 0.0f};
    float accel_white_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_vib_[3]    = {0.0f, 0.0f, 0.0f};   ///< N2 band-limited gyro vibration
    float accel_vib_[3]   = {0.0f, 0.0f, 0.0f};   ///< N2 band-limited accel vibration

    // N2 band-pass (shared shape, per-axis state). Channels 0–2 gyro, 3–5 accel.
    // N2 帯域通過（形状は共有・状態は軸別）。チャンネル 0–2 gyro, 3–5 accel。
    float bq_b0_ = 0.0f, bq_b2_ = 0.0f, bq_a1_ = 0.0f, bq_a2_ = 0.0f;  // b1 ≡ 0
    float bq_inv_gain_ = 1.0f;                    ///< normalizes the output to unit rms
    float bq_z1_[6] = {0,0,0,0,0,0};              ///< DF2T state z1 per channel
    float bq_z2_[6] = {0,0,0,0,0,0};              ///< DF2T state z2 per channel
    float last_vib_dt_ = -1.0f;                   ///< last substep dt (recompute coeffs on change)

    float tof_noise_  = 0.0f;   ///< N2 ToF observation-noise sample [m]
    float baro_noise_ = 0.0f;   ///< N2 baro observation-noise sample [m]
};

}  // namespace sil
