/**
 * @file filter.hpp
 * @brief Filter Library
 *
 * LowPassFilter, MedianFilter, MovingAverage, OutlierDetector
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>

namespace stampfly {

/**
 * @brief 1st order IIR Low Pass Filter
 */
class LowPassFilter {
public:
    LowPassFilter() = default;

    /**
     * @brief Initialize filter
     * @param sample_freq Sampling frequency (Hz)
     * @param cutoff_freq Cutoff frequency (Hz)
     */
    void init(float sample_freq, float cutoff_freq);

    /**
     * @brief Apply filter to input
     * @param input Input value
     * @return Filtered value
     */
    float apply(float input);

    /**
     * @brief Reset filter state
     */
    void reset();

    /**
     * @brief Get current output
     */
    float getOutput() const { return output_; }

private:
    float alpha_ = 1.0f;
    float output_ = 0.0f;
    bool initialized_ = false;
};

/**
 * @brief Moving Average Filter
 */
template<typename T, size_t N>
class MovingAverage {
public:
    void reset() {
        for (size_t i = 0; i < N; i++) {
            buffer_[i] = T{};
        }
        index_ = 0;
        count_ = 0;
        sum_ = T{};
    }

    T apply(T input) {
        if (count_ >= N) {
            sum_ -= buffer_[index_];
        }
        buffer_[index_] = input;
        sum_ += input;
        index_ = (index_ + 1) % N;
        if (count_ < N) count_++;
        return sum_ / static_cast<T>(count_);
    }

    T getAverage() const {
        return count_ > 0 ? sum_ / static_cast<T>(count_) : T{};
    }

private:
    T buffer_[N] = {};
    size_t index_ = 0;
    size_t count_ = 0;
    T sum_ = {};
};

/**
 * @brief Median Filter
 */
template<typename T, size_t N>
class MedianFilter {
public:
    void reset() {
        for (size_t i = 0; i < N; i++) {
            buffer_[i] = T{};
        }
        index_ = 0;
        count_ = 0;
    }

    T apply(T input) {
        buffer_[index_] = input;
        index_ = (index_ + 1) % N;
        if (count_ < N) count_++;

        // Copy and sort
        T sorted[N];
        for (size_t i = 0; i < count_; i++) {
            sorted[i] = buffer_[i];
        }
        std::sort(sorted, sorted + count_);
        return sorted[count_ / 2];
    }

private:
    T buffer_[N] = {};
    size_t index_ = 0;
    size_t count_ = 0;
};

/**
 * @brief 3D Vector structure
 */
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }
};

/**
 * @brief 2nd order Notch (Band-Reject) Filter
 *
 * H(s) = (s² + ω₀²) / (s² + (ω₀/Q)·s + ω₀²)
 * Discretized using bilinear transform (Tustin).
 *
 * 2次ノッチフィルタ。双一次変換で離散化。
 */
class NotchFilter {
public:
    NotchFilter() = default;

    /**
     * @brief Initialize notch filter
     * @param sample_freq Sampling frequency [Hz]
     * @param notch_freq Center frequency to reject [Hz]
     * @param Q Quality factor (higher = narrower notch)
     */
    void init(float sample_freq, float notch_freq, float Q) {
        float w0 = 2.0f * 3.14159265f * notch_freq / sample_freq;
        float sin_w0 = std::sin(w0);
        float cos_w0 = std::cos(w0);
        float alpha = sin_w0 / (2.0f * Q);

        float a0_inv = 1.0f / (1.0f + alpha);
        b0_ = a0_inv;
        b1_ = -2.0f * cos_w0 * a0_inv;
        b2_ = a0_inv;
        a1_ = -2.0f * cos_w0 * a0_inv;
        a2_ = (1.0f - alpha) * a0_inv;

        reset();
    }

    /**
     * @brief Apply filter to input sample
     * @param input Input value
     * @return Filtered value
     */
    float apply(float input) {
        float output = b0_ * input + b1_ * x1_ + b2_ * x2_
                        - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_; x1_ = input;
        y2_ = y1_; y1_ = output;
        return output;
    }

    /**
     * @brief Reset filter state
     */
    void reset() {
        x1_ = x2_ = y1_ = y2_ = 0.0f;
    }

    /**
     * @brief Update notch frequency at runtime
     * @param sample_freq Sampling frequency [Hz]
     * @param notch_freq New center frequency [Hz]
     * @param Q Quality factor
     */
    void setFrequency(float sample_freq, float notch_freq, float Q) {
        init(sample_freq, notch_freq, Q);
    }

private:
    float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f;
    float a1_ = 0.0f, a2_ = 0.0f;
    float x1_ = 0.0f, x2_ = 0.0f;
    float y1_ = 0.0f, y2_ = 0.0f;
};

/**
 * @brief Outlier Detector
 */
class OutlierDetector {
public:
    /**
     * @brief Check if accelerometer data is valid
     * @param accel Accelerometer data (m/s^2)
     * @return true if valid (0.5g < norm < 3.0g)
     */
    static bool isAccelValid(const Vec3& accel) {
        float norm = accel.norm();
        constexpr float g = 9.81f;
        return (norm >= 0.5f * g) && (norm <= 3.0f * g);
    }

    /**
     * @brief Check if gyroscope data is valid
     * @param gyro Gyroscope data (dps)
     * @return true if valid (|value| <= 2000 dps)
     */
    static bool isGyroValid(const Vec3& gyro) {
        return (std::abs(gyro.x) <= 2000.0f) &&
               (std::abs(gyro.y) <= 2000.0f) &&
               (std::abs(gyro.z) <= 2000.0f);
    }

    /**
     * @brief Check if ToF data is valid
     * @param range_status Range status (0-4 valid)
     * @param signal_rate Signal rate
     * @param min_signal_rate Minimum signal rate threshold
     * @return true if valid
     */
    static bool isToFValid(uint8_t range_status, float signal_rate, float min_signal_rate = 1.0f) {
        return (range_status <= 4) && (signal_rate >= min_signal_rate);
    }

    /**
     * @brief Check if optical flow data is valid
     * @param squal Surface quality (0-255)
     * @return true if valid (squal >= 30)
     */
    static bool isFlowValid(uint8_t squal) {
        return squal >= 30;
    }

    /**
     * @brief Check if magnetometer data is valid
     * @param mag Magnetometer data (uT)
     * @param expected_norm Expected norm (local geomagnetic field)
     * @return true if valid (within +/-30% of expected)
     */
    static bool isMagValid(const Vec3& mag, float expected_norm) {
        float norm = mag.norm();
        return (norm >= expected_norm * 0.7f) && (norm <= expected_norm * 1.3f);
    }

    /**
     * @brief Check if barometer altitude rate is valid
     * @param current_alt Current altitude (m)
     * @param prev_alt Previous altitude (m)
     * @param dt Time delta (s)
     * @return true if valid (rate <= 10 m/s)
     */
    static bool isBaroValid(float current_alt, float prev_alt, float dt) {
        if (dt <= 0) return false;
        float rate = std::abs(current_alt - prev_alt) / dt;
        return rate <= 10.0f;
    }
};

}  // namespace stampfly
