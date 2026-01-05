/**
 * @file motor_driver.hpp
 * @brief Motor Driver (LEDC PWM)
 *
 * X-quad configuration, 150kHz PWM, 8-bit resolution
 * Mixer: M1=T-R+P+Y, M2=T-R-P-Y, M3=T+R-P+Y, M4=T+R+P-Y
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class MotorDriver {
public:
    // Motor indices (clockwise from front-right)
    static constexpr int MOTOR_FR = 0;  // M1: Front-Right (CCW)
    static constexpr int MOTOR_RR = 1;  // M2: Rear-Right (CW)
    static constexpr int MOTOR_RL = 2;  // M3: Rear-Left (CCW)
    static constexpr int MOTOR_FL = 3;  // M4: Front-Left (CW)
    static constexpr int NUM_MOTORS = 4;

    struct Config {
        int gpio[NUM_MOTORS];     // GPIO pins for each motor
        int pwm_freq_hz;          // PWM frequency (default: 150000)
        int pwm_resolution_bits;  // PWM resolution (default: 8)
    };

    MotorDriver() = default;
    ~MotorDriver() = default;

    /**
     * @brief Initialize motor driver
     * @param config PWM configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Arm motors (enable output)
     * @return ESP_OK on success
     */
    esp_err_t arm();

    /**
     * @brief Disarm motors (disable output, set to 0)
     * @return ESP_OK on success
     */
    esp_err_t disarm();

    /**
     * @brief Check if motors are armed
     * @return true if armed
     */
    bool isArmed() const { return armed_; }

    /**
     * @brief Set individual motor output
     * @param motor Motor index (0-3)
     * @param value Output value (0.0-1.0)
     */
    void setMotor(int motor, float value);

    /**
     * @brief Set mixer output
     * @param thrust Thrust (0.0-1.0)
     * @param roll Roll (-1.0 to 1.0)
     * @param pitch Pitch (-1.0 to 1.0)
     * @param yaw Yaw (-1.0 to 1.0)
     */
    void setMixerOutput(float thrust, float roll, float pitch, float yaw);

    /**
     * @brief Test single motor
     * @param motor Motor index (0-3)
     * @param throttle_percent Throttle (0-100)
     */
    void testMotor(int motor, int throttle_percent);

    bool isInitialized() const { return initialized_; }

    // Motor duty statistics
    struct MotorStats {
        float sum;
        float min;
        float max;
        uint32_t count;
    };

    /**
     * @brief Get motor duty statistics
     * @param motor Motor index (0-3)
     * @return Statistics for the motor
     */
    MotorStats getStats(int motor) const;

    /**
     * @brief Reset all motor statistics
     */
    void resetStats();

    /**
     * @brief Save statistics to NVS
     */
    void saveStatsToNVS();

    /**
     * @brief Load statistics from NVS
     * @return true if loaded successfully
     */
    bool loadStatsFromNVS();

    /**
     * @brief Get last flight statistics (from NVS)
     */
    MotorStats getLastFlightStats(int motor) const { return last_flight_stats_[motor]; }

private:
    bool initialized_ = false;
    bool armed_ = false;
    Config config_;
    float motor_output_[NUM_MOTORS] = {0};

    // Statistics (current flight)
    MotorStats stats_[NUM_MOTORS] = {};

    // Last flight statistics (loaded from NVS)
    MotorStats last_flight_stats_[NUM_MOTORS] = {};
};

}  // namespace stampfly
