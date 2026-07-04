/**
 * @file stationary_detector.hpp
 * @brief Stationary state detector for landing calibration
 *
 * Detects when the vehicle is stationary (not moving) using EMA-filtered
 * gyroscope and accelerometer readings. Robust to transient noise spikes.
 *
 * 静止状態検出器。EMAフィルタ済みジャイロ・加速度から機体が静止しているかを判定。
 * 瞬間的なノイズスパイクに対してロバスト。
 */

#pragma once

#include "stampfly_math.hpp"
#include <cmath>

namespace stampfly {

class StationaryDetector {
public:
    struct Config {
        float gyro_threshold;
        float accel_norm_min;
        float accel_norm_max;
        float accel_variance_threshold;
        int required_samples;
        int averaging_samples;
        float ema_alpha;

        Config()
            : gyro_threshold(0.02f)              // rad/s - max filtered gyro magnitude
            , accel_norm_min(9.5f)               // m/s² - min filtered accel norm
            , accel_norm_max(10.1f)              // m/s² - max filtered accel norm
            , accel_variance_threshold(0.05f)    // (m/s²)² - max accel variance
            , required_samples(200)              // samples needed (0.5s @ 400Hz)
            , averaging_samples(200)             // samples for averaging (0.5s @ 400Hz)
            , ema_alpha(0.05f)                   // EMA smoothing factor (~3Hz cutoff @ 400Hz)
        {}
    };

    /**
     * @brief Initialize with configuration
     */
    void init(const Config& config = Config()) {
        config_ = config;
        reset();
    }

    /**
     * @brief Reset detector state
     */
    void reset() {
        stationary_count_ = 0;
        is_stationary_ = false;
        gyro_sum_ = math::Vector3::zero();
        accel_sum_ = math::Vector3::zero();
        accel_sq_sum_ = math::Vector3::zero();
        sample_count_ = 0;
        ema_initialized_ = false;
        gyro_mag_ema_ = 0.0f;
        accel_norm_ema_ = 0.0f;
    }

    /**
     * @brief Update with new sensor data
     * @param gyro Gyroscope reading [rad/s] (body frame)
     * @param accel Accelerometer reading [m/s²] (body frame)
     */
    void update(const math::Vector3& gyro, const math::Vector3& accel) {
        float gyro_mag = std::sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
        float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

        // Apply EMA low-pass filter to suppress transient noise spikes
        // EMAローパスフィルタで瞬間的なノイズスパイクを抑制
        const float a = config_.ema_alpha;
        if (!ema_initialized_) {
            gyro_mag_ema_ = gyro_mag;
            accel_norm_ema_ = accel_norm;
            ema_initialized_ = true;
        } else {
            gyro_mag_ema_ = a * gyro_mag + (1.0f - a) * gyro_mag_ema_;
            accel_norm_ema_ = a * accel_norm + (1.0f - a) * accel_norm_ema_;
        }

        // Stationary conditions on filtered values
        // フィルタ済み値で静止条件を判定
        bool gyro_ok = gyro_mag_ema_ < config_.gyro_threshold;
        bool accel_ok = (accel_norm_ema_ > config_.accel_norm_min) &&
                        (accel_norm_ema_ < config_.accel_norm_max);

        if (gyro_ok && accel_ok) {
            stationary_count_++;

            // Accumulate raw values for averaging (unfiltered for accuracy)
            // 平均計算には生の値を蓄積（精度のためフィルタなし）
            gyro_sum_ += gyro;
            accel_sum_ += accel;
            accel_sq_sum_.x += accel.x * accel.x;
            accel_sq_sum_.y += accel.y * accel.y;
            accel_sq_sum_.z += accel.z * accel.z;
            sample_count_++;

            // Check if stationary for required duration
            if (stationary_count_ >= config_.required_samples) {
                // Also check accel variance
                if (sample_count_ > 0) {
                    float n = static_cast<float>(sample_count_);
                    math::Vector3 mean = accel_sum_ * (1.0f / n);
                    math::Vector3 mean_sq = accel_sq_sum_ * (1.0f / n);

                    // Variance = E[x²] - E[x]²
                    float var_x = mean_sq.x - mean.x * mean.x;
                    float var_y = mean_sq.y - mean.y * mean.y;
                    float var_z = mean_sq.z - mean.z * mean.z;
                    float total_var = var_x + var_y + var_z;

                    if (total_var < config_.accel_variance_threshold) {
                        is_stationary_ = true;
                    }
                }
            }
        } else {
            // Not stationary - reset count and accumulators
            // 非静止 - カウンタと蓄積値をリセット
            stationary_count_ = 0;
            is_stationary_ = false;
            gyro_sum_ = math::Vector3::zero();
            accel_sum_ = math::Vector3::zero();
            accel_sq_sum_ = math::Vector3::zero();
            sample_count_ = 0;
        }
    }

    /**
     * @brief Check if currently stationary
     */
    bool isStationary() const { return is_stationary_; }

    /**
     * @brief Get averaged gyro value (valid when stationary)
     */
    math::Vector3 getGyroAverage() const {
        if (sample_count_ > 0) {
            return gyro_sum_ * (1.0f / static_cast<float>(sample_count_));
        }
        return math::Vector3::zero();
    }

    /**
     * @brief Get averaged accel value (valid when stationary)
     */
    math::Vector3 getAccelAverage() const {
        if (sample_count_ > 0) {
            return accel_sum_ * (1.0f / static_cast<float>(sample_count_));
        }
        return math::Vector3::zero();
    }

    /**
     * @brief Get number of samples collected
     */
    int getSampleCount() const { return sample_count_; }

    /**
     * @brief Get filtered gyro magnitude (for debugging)
     * フィルタ済みジャイロ振幅を取得（デバッグ用）
     */
    float getFilteredGyroMag() const { return gyro_mag_ema_; }

    /**
     * @brief Get filtered accel norm (for debugging)
     * フィルタ済み加速度ノルムを取得（デバッグ用）
     */
    float getFilteredAccelNorm() const { return accel_norm_ema_; }

private:
    Config config_;
    int stationary_count_ = 0;
    bool is_stationary_ = false;

    // EMA filter state
    // EMAフィルタ状態
    bool ema_initialized_ = false;
    float gyro_mag_ema_ = 0.0f;
    float accel_norm_ema_ = 0.0f;

    // Accumulators for averaging (raw values)
    // 平均計算用の蓄積値（生の値）
    math::Vector3 gyro_sum_;
    math::Vector3 accel_sum_;
    math::Vector3 accel_sq_sum_;  // For variance calculation
    int sample_count_ = 0;
};

}  // namespace stampfly
