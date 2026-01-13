/**
 * @file stationary_detector.hpp
 * @brief Stationary state detector for landing calibration
 *
 * Detects when the vehicle is stationary (not moving) based on
 * gyroscope and accelerometer readings. Used to trigger calibration
 * after landing.
 *
 * 静止状態検出器。ジャイロと加速度から機体が静止しているかを判定。
 * 着陸後のキャリブレーショントリガーに使用。
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

        Config()
            : gyro_threshold(0.02f)              // rad/s - max gyro magnitude for stationary
            , accel_norm_min(9.5f)               // m/s² - min accel norm (not free fall)
            , accel_norm_max(10.1f)              // m/s² - max accel norm (not accelerating)
            , accel_variance_threshold(0.05f)    // (m/s²)² - max accel variance
            , required_samples(200)              // samples needed (0.5s @ 400Hz)
            , averaging_samples(200)             // samples for averaging (0.5s @ 400Hz)
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
    }

    /**
     * @brief Update with new sensor data
     * @param gyro Gyroscope reading [rad/s] (body frame)
     * @param accel Accelerometer reading [m/s²] (body frame)
     */
    void update(const math::Vector3& gyro, const math::Vector3& accel) {
        // Check gyro magnitude
        float gyro_mag = std::sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);

        // Check accel norm (should be ~g when stationary)
        float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

        // Stationary conditions
        bool gyro_ok = gyro_mag < config_.gyro_threshold;
        bool accel_ok = (accel_norm > config_.accel_norm_min) &&
                        (accel_norm < config_.accel_norm_max);

        if (gyro_ok && accel_ok) {
            stationary_count_++;

            // Accumulate for averaging
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
            // Not stationary - reset
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

private:
    Config config_;
    int stationary_count_ = 0;
    bool is_stationary_ = false;

    // Accumulators for averaging
    math::Vector3 gyro_sum_;
    math::Vector3 accel_sum_;
    math::Vector3 accel_sq_sum_;  // For variance calculation
    int sample_count_ = 0;
};

}  // namespace stampfly
