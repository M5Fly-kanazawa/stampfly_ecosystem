/**
 * @file mag_calibration.hpp
 * @brief Magnetometer Calibration for BMM150
 *
 * Hard Iron / Soft Iron calibration using ellipsoid fitting.
 * - Hard Iron: Constant offset in each axis (additive)
 * - Soft Iron: Scale and cross-axis coupling (multiplicative)
 *
 * Calibration model:
 *   mag_calibrated = soft_iron * (mag_raw - hard_iron)
 *
 * Usage:
 *   1. Call startCalibration() to begin data collection
 *   2. Rotate device slowly in all orientations (figure-8 pattern)
 *   3. Call addSample() with each magnetometer reading
 *   4. Call computeCalibration() when enough samples collected
 *   5. Use applyCalibration() on subsequent readings
 *   6. Save/load calibration with NVS functions
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

/**
 * @brief Callback function type for calibration log messages
 * @param message Log message string
 */
using MagCalLogCallback = void (*)(const char* message);

/**
 * @brief Magnetometer calibration parameters
 */
struct MagCalibration {
    // Hard iron offset (μT)
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;

    // Soft iron scale factors (diagonal of scale matrix)
    // For simplicity, we use diagonal-only soft iron correction
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float scale_z = 1.0f;

    // Calibration quality metrics
    float fitness = 0.0f;        // How well data fits sphere (0-1, higher=better)
    float sphere_radius = 45.0f; // Expected magnetic field strength (μT)

    // Validity flag
    bool valid = false;

    /**
     * @brief Check if calibration is valid and reasonable
     */
    bool isValid() const {
        if (!valid) return false;
        // Sanity checks
        if (scale_x < 0.5f || scale_x > 2.0f) return false;
        if (scale_y < 0.5f || scale_y > 2.0f) return false;
        if (scale_z < 0.5f || scale_z > 2.0f) return false;
        return true;
    }
};

/**
 * @brief Magnetometer Calibrator
 *
 * Implements sphere fitting algorithm for hard iron calibration
 * and ellipsoid fitting for soft iron calibration.
 */
class MagCalibrator {
public:
    static constexpr int MAX_SAMPLES = 500;      // Maximum samples to collect
    static constexpr int MIN_SAMPLES = 100;      // Minimum samples for calibration
    static constexpr float MIN_COVERAGE = 0.5f;  // Minimum sphere coverage ratio

    enum class State {
        IDLE,           // Not calibrating
        COLLECTING,     // Collecting samples
        COMPUTING,      // Computing calibration
        DONE,           // Calibration complete
        ERROR           // Calibration failed
    };

    MagCalibrator() = default;
    ~MagCalibrator() = default;

    /**
     * @brief Start calibration data collection
     * @return ESP_OK on success
     */
    esp_err_t startCalibration();

    /**
     * @brief Stop calibration and discard data
     */
    void stopCalibration();

    /**
     * @brief Add a magnetometer sample
     * @param x X-axis reading (μT)
     * @param y Y-axis reading (μT)
     * @param z Z-axis reading (μT)
     * @return ESP_OK if sample added, ESP_ERR_INVALID_STATE if not collecting
     */
    esp_err_t addSample(float x, float y, float z);

    /**
     * @brief Compute calibration from collected samples
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not enough samples
     */
    esp_err_t computeCalibration();

    /**
     * @brief Apply calibration to raw magnetometer reading
     * @param raw_x Raw X reading (μT)
     * @param raw_y Raw Y reading (μT)
     * @param raw_z Raw Z reading (μT)
     * @param cal_x Calibrated X output (μT)
     * @param cal_y Calibrated Y output (μT)
     * @param cal_z Calibrated Z output (μT)
     */
    void applyCalibration(float raw_x, float raw_y, float raw_z,
                          float& cal_x, float& cal_y, float& cal_z) const;

    /**
     * @brief Save calibration to NVS
     * @return ESP_OK on success
     */
    esp_err_t saveToNVS();

    /**
     * @brief Load calibration from NVS
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not calibrated
     */
    esp_err_t loadFromNVS();

    /**
     * @brief Clear calibration from NVS
     * @return ESP_OK on success
     */
    esp_err_t clearNVS();

    // Getters
    State getState() const { return state_; }
    int getSampleCount() const { return sample_count_; }
    float getProgress() const { return static_cast<float>(sample_count_) / MIN_SAMPLES; }
    const MagCalibration& getCalibration() const { return calibration_; }
    bool isCalibrated() const { return calibration_.isValid(); }

    /**
     * @brief Set calibration parameters directly (for testing/debugging)
     */
    void setCalibration(const MagCalibration& cal) { calibration_ = cal; }

    /**
     * @brief Set log callback function for CLI output
     * @param callback Function to call with log messages (nullptr to disable)
     */
    void setLogCallback(MagCalLogCallback callback) { log_callback_ = callback; }

private:
    // Log callback for CLI output (bypasses ESP_LOG)
    MagCalLogCallback log_callback_ = nullptr;

    /**
     * @brief Log message via callback if set, otherwise use ESP_LOG
     */
    void log(const char* format, ...) const;
    State state_ = State::IDLE;

    // Sample storage
    float samples_x_[MAX_SAMPLES];
    float samples_y_[MAX_SAMPLES];
    float samples_z_[MAX_SAMPLES];
    int sample_count_ = 0;

    // For duplicate detection
    float last_x_ = 0, last_y_ = 0, last_z_ = 0;

    // Calibration result
    MagCalibration calibration_;

    /**
     * @brief Compute hard iron offset using simple min/max method
     * This is fast but less accurate than ellipsoid fitting
     */
    void computeHardIronSimple();

    /**
     * @brief Compute hard iron offset using sphere fitting (least squares)
     * More accurate than min/max method
     */
    void computeHardIronSphereFit();

    /**
     * @brief Compute soft iron scale factors
     */
    void computeSoftIron();

    /**
     * @brief Calculate calibration fitness (how well data fits sphere)
     */
    float calculateFitness() const;

    /**
     * @brief Check if samples cover enough of the sphere
     */
    float calculateCoverage() const;
};

}  // namespace stampfly
