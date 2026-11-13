/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file baro_model.hpp
 * @brief What the real BMP280 adds to the true height: an absolute offset, a drift, a thrust-dependent
 *        term (propeller wash) and noise. 実機の BMP280 が真の高さに足すもの: 絶対オフセット・ドリフト・
 *        推力に依存する項（プロペラの吹き下ろし）・ノイズ。
 *
 * Physics-free and seeded (own RNG, so the other noise streams of the SILS keep their sequences): the Plant calls
 * apply() with the true height above the floor each time the BMP280 is read. OFF by default (apply() returns the true height
 * and draws nothing), so every scenario that does not say `baro ...` is bit-for-bit what it was. Typical magnitudes come from
 * the study of 17 real captures (analysis/scripts/baro_offset_study.py; CANDIDATE: few logs):
 *   offset  35 m      the BMP280 reads an ABSOLUTE altitude (real logs: -50 m, -14 m, +80 m, +117 m, +780 m); tens of metres is typical
 *   wash    1.4 m     the reading rises by wash x (mean duty)^2: 0.5 m at the hover duty 0.6, 0.23 m at 0.4 on the floor (real: 0.2-0.9 m, 0.1-0.4 m)
 *   drift   0.010 m/s linear, from the moment it is set (real: +0.014 m/s in the first minute after the power-on, about 0.001 m/s later, +-0.02 m/s on the floor)
 *   white   0.04 m   per sample (real: 0.03-0.05 m rms in 2-6.5 Hz)
 *   slow    0.12 m   first-order (time constant 1 s) noise (real: 0.08-0.14 m rms in 0.05-2 Hz)
 *
 * 物理を持たず、シード付き（専用の乱数列なので、SILS の他のノイズの列は変わらない）: BMP280 が読まれるたびに Plant が床からの
 * 真の高さを渡して apply() を呼ぶ。既定は OFF（apply() は真の高さを返し、乱数も引かない）なので、`baro ...` を書かない
 * シナリオは従来とビットまで同じ。典型値は実機ログ 17 本の調査（analysis/scripts/baro_offset_study.py。候補値: ログが少ない）による。
 *
 * @design simulator/sils/docs/scenario_tutorial.md — `baro` event     [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace sils {

/// Magnitudes of the modelled effects. モデルの各効果の大きさ。
struct BaroModelConfig {
    bool  enable            = false;
    float offset_m          = 0.0f;    ///< absolute offset [m] / 絶対オフセット
    float drift_mps         = 0.0f;    ///< linear drift [m/s], from `drift_start_s` / 一定のドリフト [m/s]
    float wash_m            = 0.0f;    ///< thrust term: wash_m x (mean duty)^2 [m] / 推力の項
    float white_sigma_m     = 0.0f;    ///< white noise per sample [m] / サンプルごとの白色ノイズ
    float slow_sigma_m      = 0.0f;    ///< slow noise rms [m] / 低速ノイズの rms
    float slow_tau_s        = 1.0f;    ///< slow noise time constant [s] / 低速ノイズの時定数
};

/// The "typical" magnitudes (scale 1). 「典型」の大きさ（倍率 1）。
inline constexpr float kBaroTypicalOffsetM  = 35.0f;
inline constexpr float kBaroTypicalWashM    = 1.4f;
inline constexpr float kBaroTypicalDriftMps = 0.010f;
inline constexpr float kBaroTypicalWhiteM   = 0.04f;
inline constexpr float kBaroTypicalSlowM    = 0.12f;

class BaroModel {
public:
    static constexpr uint32_t kSeed = 20261113;   ///< own RNG seed / 専用の乱数の種

    /// Apply a configuration at virtual time `now_s` (the drift starts counting here). 設定を適用する（ドリフトはここから数える）。
    void configure(const BaroModelConfig& config, double now_s)
    {
        config_        = config;
        drift_start_s_ = now_s;
        last_s_        = now_s;
        slow_m_        = 0.0f;
        rng_.seed(kSeed);
        normal_.reset();
    }

    /// `baro typical [scale]`: every effect at its typical magnitude times `scale` (the offset is not scaled). `baro typical [scale]`: 全効果を典型値 × scale にする（オフセットは倍率の対象外）。
    static BaroModelConfig typical(float scale)
    {
        BaroModelConfig c;
        c.enable = true;
        c.offset_m = kBaroTypicalOffsetM;
        c.wash_m = kBaroTypicalWashM * scale;
        c.drift_mps = kBaroTypicalDriftMps * scale;
        c.white_sigma_m = kBaroTypicalWhiteM * scale;
        c.slow_sigma_m = kBaroTypicalSlowM * scale;
        return c;
    }

    const BaroModelConfig& config() const { return config_; }

    /// The altitude the sensor would read: true height + offset + drift + wash + noise. センサが読む高度。
    float apply(float true_height_m, double now_s, float mean_duty)
    {
        if (!config_.enable) return true_height_m;
        const float elapsed_s = static_cast<float>(now_s - drift_start_s_);
        float altitude = true_height_m + config_.offset_m + config_.drift_mps * elapsed_s
                       + config_.wash_m * mean_duty * mean_duty;
        altitude += config_.white_sigma_m * normal_(rng_);
        altitude += advanceSlowNoise(now_s);
        return altitude;
    }

private:
    /// First-order (Ornstein-Uhlenbeck) noise stepped to `now_s`: exact discretisation, stationary rms slow_sigma_m.
    /// 1 次（Ornstein-Uhlenbeck）のノイズを now_s まで進める。厳密な離散化で、定常の rms は slow_sigma_m。
    float advanceSlowNoise(double now_s)
    {
        const float step_s = static_cast<float>(now_s - last_s_);
        last_s_ = now_s;
        if (config_.slow_sigma_m <= 0.0f || step_s <= 0.0f) return slow_m_;
        const float decay = std::exp(-step_s / config_.slow_tau_s);
        slow_m_ = slow_m_ * decay + config_.slow_sigma_m * std::sqrt(1.0f - decay * decay) * normal_(rng_);
        return slow_m_;
    }

    BaroModelConfig config_;
    double drift_start_s_ = 0.0;
    double last_s_        = 0.0;
    float  slow_m_        = 0.0f;
    std::mt19937 rng_{kSeed};
    std::normal_distribution<float> normal_{0.0f, 1.0f};
};

}  // namespace sils
