/**
 * @file mag_calibration.cpp
 * @brief Magnetometer Calibration Implementation
 *
 * Implements hard iron and soft iron calibration for BMM150.
 *
 * Hard Iron: Constant offset caused by permanent magnets nearby
 *   - Shifts the center of the measurement sphere away from origin
 *   - Corrected by subtracting offset: mag_corrected = mag_raw - offset
 *
 * Soft Iron: Distortion caused by ferromagnetic materials nearby
 *   - Deforms the sphere into an ellipsoid
 *   - Corrected by scaling axes: mag_corrected = scale * (mag_raw - offset)
 *
 * Calibration procedure:
 *   1. Collect samples while rotating device in all directions (3D figure-8)
 *   2. Fit a sphere to find hard iron offset (center of sphere)
 *   3. Compute soft iron scale factors to normalize ellipsoid to sphere
 */

#include "mag_calibration.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdarg>
#include <cstdio>

static const char* TAG = "MagCal";

// NVS namespace and keys
static const char* NVS_NAMESPACE = "mag_cal";
static const char* NVS_KEY_OFFSET_X = "off_x";
static const char* NVS_KEY_OFFSET_Y = "off_y";
static const char* NVS_KEY_OFFSET_Z = "off_z";
static const char* NVS_KEY_SCALE_X = "scl_x";
static const char* NVS_KEY_SCALE_Y = "scl_y";
static const char* NVS_KEY_SCALE_Z = "scl_z";
static const char* NVS_KEY_RADIUS = "radius";
static const char* NVS_KEY_VALID = "valid";

namespace stampfly {

void MagCalibrator::log(const char* format, ...) const
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (log_callback_) {
        // Use callback (for CLI output)
        log_callback_(buffer);
    } else {
        // Fall back to ESP_LOG
        ESP_LOGI(TAG, "%s", buffer);
    }
}

esp_err_t MagCalibrator::startCalibration()
{
    ESP_LOGI(TAG, "Starting magnetometer calibration");
    ESP_LOGI(TAG, "Rotate device slowly in all 3D directions (figure-8 pattern)");

    // Reset state
    sample_count_ = 0;
    last_x_ = last_y_ = last_z_ = 0;
    calibration_ = MagCalibration();  // Reset calibration
    state_ = State::COLLECTING;

    return ESP_OK;
}

void MagCalibrator::stopCalibration()
{
    ESP_LOGI(TAG, "Stopping magnetometer calibration");
    state_ = State::IDLE;
    sample_count_ = 0;
}

esp_err_t MagCalibrator::addSample(float x, float y, float z)
{
    if (state_ != State::COLLECTING) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sample_count_ >= MAX_SAMPLES) {
        ESP_LOGW(TAG, "Sample buffer full");
        return ESP_ERR_NO_MEM;
    }

    // Skip NaN/Inf values
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Skip duplicate samples (sensor not moving)
    constexpr float MIN_CHANGE = 1.0f;  // μT
    float dx = x - last_x_;
    float dy = y - last_y_;
    float dz = z - last_z_;
    float change = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (sample_count_ > 0 && change < MIN_CHANGE) {
        return ESP_OK;  // Skip but don't error
    }

    // Store sample
    samples_x_[sample_count_] = x;
    samples_y_[sample_count_] = y;
    samples_z_[sample_count_] = z;
    sample_count_++;

    last_x_ = x;
    last_y_ = y;
    last_z_ = z;

    // Log progress periodically
    if (sample_count_ % 50 == 0) {
        ESP_LOGI(TAG, "Collected %d samples", sample_count_);
    }

    return ESP_OK;
}

esp_err_t MagCalibrator::computeCalibration()
{
    if (state_ != State::COLLECTING) {
        log("Error: Not in collecting state");
        return ESP_ERR_INVALID_STATE;
    }

    if (sample_count_ < MIN_SAMPLES) {
        log("Error: Not enough samples: %d < %d", sample_count_, MIN_SAMPLES);
        state_ = State::ERROR;
        return ESP_ERR_INVALID_SIZE;
    }

    log("Computing calibration from %d samples...", sample_count_);
    state_ = State::COMPUTING;

    // Log raw data statistics
    float min_x = samples_x_[0], max_x = samples_x_[0];
    float min_y = samples_y_[0], max_y = samples_y_[0];
    float min_z = samples_z_[0], max_z = samples_z_[0];
    for (int i = 1; i < sample_count_; i++) {
        min_x = std::min(min_x, samples_x_[i]);
        max_x = std::max(max_x, samples_x_[i]);
        min_y = std::min(min_y, samples_y_[i]);
        max_y = std::max(max_y, samples_y_[i]);
        min_z = std::min(min_z, samples_z_[i]);
        max_z = std::max(max_z, samples_z_[i]);
    }
    log("Raw data ranges:");
    log("  X: [%.1f, %.1f] range=%.1f", min_x, max_x, max_x - min_x);
    log("  Y: [%.1f, %.1f] range=%.1f", min_y, max_y, max_y - min_y);
    log("  Z: [%.1f, %.1f] range=%.1f", min_z, max_z, max_z - min_z);

    // Step 1: Compute hard iron offset using sphere fitting
    computeHardIronSphereFit();

    log("Hard iron offset: [%.2f, %.2f, %.2f]",
        calibration_.offset_x, calibration_.offset_y, calibration_.offset_z);
    log("Sphere radius: %.2f uT", calibration_.sphere_radius);

    // Step 2: Compute soft iron scale factors
    computeSoftIron();

    log("Soft iron scale: [%.3f, %.3f, %.3f]",
        calibration_.scale_x, calibration_.scale_y, calibration_.scale_z);

    // Step 3: Calculate fitness
    calibration_.fitness = calculateFitness();

    log("Calibration fitness: %.3f", calibration_.fitness);

    // Check coverage (should have samples in multiple octants)
    float coverage = calculateCoverage();
    log("Sphere coverage: %.1f%% (%d/8 octants)",
        coverage * 100.0f, static_cast<int>(coverage * 8));

    if (coverage < MIN_COVERAGE) {
        log("Warning: Poor coverage (%.1f%%). Rotate in more directions.", coverage * 100.0f);
    }

    // Check if calibration is acceptable
    if (calibration_.fitness < 0.5f) {
        log("Warning: Poor calibration fitness: %.2f (should be > 0.5)", calibration_.fitness);
        // Don't fail, but warn
    }

    calibration_.valid = true;
    state_ = State::DONE;

    return ESP_OK;
}

void MagCalibrator::computeHardIronSimple()
{
    // Simple min/max method
    // Hard iron offset = (max + min) / 2
    float min_x = samples_x_[0], max_x = samples_x_[0];
    float min_y = samples_y_[0], max_y = samples_y_[0];
    float min_z = samples_z_[0], max_z = samples_z_[0];

    for (int i = 1; i < sample_count_; i++) {
        min_x = std::min(min_x, samples_x_[i]);
        max_x = std::max(max_x, samples_x_[i]);
        min_y = std::min(min_y, samples_y_[i]);
        max_y = std::max(max_y, samples_y_[i]);
        min_z = std::min(min_z, samples_z_[i]);
        max_z = std::max(max_z, samples_z_[i]);
    }

    calibration_.offset_x = (max_x + min_x) / 2.0f;
    calibration_.offset_y = (max_y + min_y) / 2.0f;
    calibration_.offset_z = (max_z + min_z) / 2.0f;

    // Estimate sphere radius from range
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float range_z = max_z - min_z;
    calibration_.sphere_radius = (range_x + range_y + range_z) / 6.0f;  // Average diameter / 2
}

void MagCalibrator::computeHardIronSphereFit()
{
    // Sphere fitting using least squares
    // Model: (x - cx)^2 + (y - cy)^2 + (z - cz)^2 = r^2
    // Linearized: x^2 + y^2 + z^2 = 2*cx*x + 2*cy*y + 2*cz*z + (r^2 - cx^2 - cy^2 - cz^2)
    //
    // Let: A = 2*cx, B = 2*cy, C = 2*cz, D = r^2 - cx^2 - cy^2 - cz^2
    // Then: x^2 + y^2 + z^2 = A*x + B*y + C*z + D
    //
    // This is a linear system: [x y z 1] * [A B C D]^T = [x^2 + y^2 + z^2]

    // Build normal equations: (X^T * X) * params = X^T * b
    // where X is [n x 4] matrix of [x, y, z, 1] rows
    // and b is [n x 1] vector of [x^2 + y^2 + z^2] values

    // Compute X^T * X (4x4 symmetric matrix)
    double XtX[4][4] = {0};
    double Xtb[4] = {0};

    for (int i = 0; i < sample_count_; i++) {
        double x = samples_x_[i];
        double y = samples_y_[i];
        double z = samples_z_[i];
        double b = x*x + y*y + z*z;

        // X^T * X
        XtX[0][0] += x * x;
        XtX[0][1] += x * y;
        XtX[0][2] += x * z;
        XtX[0][3] += x;
        XtX[1][1] += y * y;
        XtX[1][2] += y * z;
        XtX[1][3] += y;
        XtX[2][2] += z * z;
        XtX[2][3] += z;
        XtX[3][3] += 1;

        // X^T * b
        Xtb[0] += x * b;
        Xtb[1] += y * b;
        Xtb[2] += z * b;
        Xtb[3] += b;
    }

    // Fill symmetric elements
    XtX[1][0] = XtX[0][1];
    XtX[2][0] = XtX[0][2];
    XtX[2][1] = XtX[1][2];
    XtX[3][0] = XtX[0][3];
    XtX[3][1] = XtX[1][3];
    XtX[3][2] = XtX[2][3];

    // Solve using Gaussian elimination with partial pivoting
    double A[4][5];  // Augmented matrix
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A[i][j] = XtX[i][j];
        }
        A[i][4] = Xtb[i];
    }

    // Forward elimination
    for (int k = 0; k < 4; k++) {
        // Find pivot
        int max_row = k;
        double max_val = std::abs(A[k][k]);
        for (int i = k + 1; i < 4; i++) {
            if (std::abs(A[i][k]) > max_val) {
                max_val = std::abs(A[i][k]);
                max_row = i;
            }
        }

        // Swap rows
        if (max_row != k) {
            for (int j = 0; j < 5; j++) {
                std::swap(A[k][j], A[max_row][j]);
            }
        }

        // Check for singular matrix
        if (std::abs(A[k][k]) < 1e-10) {
            ESP_LOGW(TAG, "Singular matrix in sphere fit, using simple method");
            computeHardIronSimple();
            return;
        }

        // Eliminate
        for (int i = k + 1; i < 4; i++) {
            double factor = A[i][k] / A[k][k];
            for (int j = k; j < 5; j++) {
                A[i][j] -= factor * A[k][j];
            }
        }
    }

    // Back substitution
    double params[4];
    for (int i = 3; i >= 0; i--) {
        params[i] = A[i][4];
        for (int j = i + 1; j < 4; j++) {
            params[i] -= A[i][j] * params[j];
        }
        params[i] /= A[i][i];
    }

    // Extract center and radius
    // A = 2*cx, B = 2*cy, C = 2*cz
    calibration_.offset_x = params[0] / 2.0f;
    calibration_.offset_y = params[1] / 2.0f;
    calibration_.offset_z = params[2] / 2.0f;

    // D = r^2 - cx^2 - cy^2 - cz^2
    // r^2 = D + cx^2 + cy^2 + cz^2
    float cx = calibration_.offset_x;
    float cy = calibration_.offset_y;
    float cz = calibration_.offset_z;
    float r_sq = params[3] + cx*cx + cy*cy + cz*cz;

    if (r_sq > 0) {
        calibration_.sphere_radius = std::sqrt(r_sq);
    } else {
        ESP_LOGW(TAG, "Invalid radius from sphere fit, using min/max method");
        computeHardIronSimple();
    }
}

void MagCalibrator::computeSoftIron()
{
    // Compute soft iron scale factors
    //
    // After removing hard iron offset, the data should form a sphere.
    // If there's soft iron distortion, it forms an ellipsoid instead.
    //
    // We compute the extent in each axis and scale to make it spherical.
    // Scale factors normalize each axis to the average radius.

    // Compute min/max in each axis after hard iron removal
    float min_x = 0, max_x = 0;
    float min_y = 0, max_y = 0;
    float min_z = 0, max_z = 0;
    bool first = true;

    for (int i = 0; i < sample_count_; i++) {
        float x = samples_x_[i] - calibration_.offset_x;
        float y = samples_y_[i] - calibration_.offset_y;
        float z = samples_z_[i] - calibration_.offset_z;

        if (first) {
            min_x = max_x = x;
            min_y = max_y = y;
            min_z = max_z = z;
            first = false;
        } else {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
            min_z = std::min(min_z, z);
            max_z = std::max(max_z, z);
        }
    }

    // Calculate semi-axes of ellipsoid (half the range in each direction)
    float semi_x = (max_x - min_x) / 2.0f;
    float semi_y = (max_y - min_y) / 2.0f;
    float semi_z = (max_z - min_z) / 2.0f;

    log("Ellipsoid semi-axes: [%.1f, %.1f, %.1f]", semi_x, semi_y, semi_z);

    // Target radius: use the sphere fit radius, or average of semi-axes
    float target_radius = calibration_.sphere_radius;
    if (target_radius < 10.0f || target_radius > 100.0f) {
        // Sphere fit radius seems wrong, use average of semi-axes
        target_radius = (semi_x + semi_y + semi_z) / 3.0f;
    }

    // Sanity check: semi-axes should be reasonably large
    const float MIN_SEMI_AXIS = 5.0f;  // Minimum 5 μT

    // Compute scale factors: target_radius / semi_axis
    // This scales each axis so that its extent equals the target radius
    if (semi_x > MIN_SEMI_AXIS) {
        calibration_.scale_x = target_radius / semi_x;
    } else {
        log("Warning: X-axis range too small (%.1f), setting scale to 1.0", semi_x);
        calibration_.scale_x = 1.0f;
    }

    if (semi_y > MIN_SEMI_AXIS) {
        calibration_.scale_y = target_radius / semi_y;
    } else {
        log("Warning: Y-axis range too small (%.1f), setting scale to 1.0", semi_y);
        calibration_.scale_y = 1.0f;
    }

    if (semi_z > MIN_SEMI_AXIS) {
        calibration_.scale_z = target_radius / semi_z;
    } else {
        log("Warning: Z-axis range too small (%.1f), setting scale to 1.0", semi_z);
        calibration_.scale_z = 1.0f;
    }

    // Clamp scale factors to reasonable range [0.5, 2.0]
    // Values outside this range indicate poor calibration data
    calibration_.scale_x = std::clamp(calibration_.scale_x, 0.5f, 2.0f);
    calibration_.scale_y = std::clamp(calibration_.scale_y, 0.5f, 2.0f);
    calibration_.scale_z = std::clamp(calibration_.scale_z, 0.5f, 2.0f);

    // Update sphere radius to target (for reference)
    calibration_.sphere_radius = target_radius;
}

float MagCalibrator::calculateFitness() const
{
    // Calculate how well the calibrated data fits a sphere
    // Fitness = 1 - coefficient_of_variation
    //
    // A perfect sphere would have all points at the same radius,
    // giving a coefficient of variation of 0 and fitness of 1.

    if (sample_count_ == 0) return 0.0f;

    double sum_r = 0;
    double sum_r_sq = 0;

    for (int i = 0; i < sample_count_; i++) {
        // Apply calibration
        float x = (samples_x_[i] - calibration_.offset_x) * calibration_.scale_x;
        float y = (samples_y_[i] - calibration_.offset_y) * calibration_.scale_y;
        float z = (samples_z_[i] - calibration_.offset_z) * calibration_.scale_z;

        double r = std::sqrt(x*x + y*y + z*z);
        sum_r += r;
        sum_r_sq += r * r;
    }

    double mean_r = sum_r / sample_count_;
    double var_r = (sum_r_sq / sample_count_) - (mean_r * mean_r);
    double std_r = std::sqrt(std::max(0.0, var_r));

    // Coefficient of variation (std / mean)
    double cv = (mean_r > 0) ? (std_r / mean_r) : 1.0;

    // Convert to fitness score (0-1, higher is better)
    // cv of 0 = perfect fit = fitness 1.0
    // cv of 0.1 = 10% deviation = fitness 0.9
    float fitness = std::max(0.0f, static_cast<float>(1.0 - cv));

    return fitness;
}

float MagCalibrator::calculateCoverage() const
{
    // Check how much of the sphere is covered by samples
    // Divide sphere into 8 octants and count which have samples
    //
    // Good 3D calibration requires samples in most octants

    if (sample_count_ == 0) return 0.0f;

    int octant_count[8] = {0};

    for (int i = 0; i < sample_count_; i++) {
        float x = samples_x_[i] - calibration_.offset_x;
        float y = samples_y_[i] - calibration_.offset_y;
        float z = samples_z_[i] - calibration_.offset_z;

        int octant = 0;
        if (x >= 0) octant |= 1;
        if (y >= 0) octant |= 2;
        if (z >= 0) octant |= 4;

        octant_count[octant]++;
    }

    int covered = 0;
    const int MIN_SAMPLES_PER_OCTANT = 3;

    for (int i = 0; i < 8; i++) {
        if (octant_count[i] >= MIN_SAMPLES_PER_OCTANT) {
            covered++;
        }
    }

    return static_cast<float>(covered) / 8.0f;
}

void MagCalibrator::applyCalibration(float raw_x, float raw_y, float raw_z,
                                      float& cal_x, float& cal_y, float& cal_z) const
{
    if (!calibration_.valid) {
        // No calibration, pass through
        cal_x = raw_x;
        cal_y = raw_y;
        cal_z = raw_z;
        return;
    }

    // Apply calibration:
    // 1. Remove hard iron offset
    // 2. Apply soft iron scale
    cal_x = (raw_x - calibration_.offset_x) * calibration_.scale_x;
    cal_y = (raw_y - calibration_.offset_y) * calibration_.scale_y;
    cal_z = (raw_z - calibration_.offset_z) * calibration_.scale_z;
}

esp_err_t MagCalibrator::saveToNVS()
{
    if (!calibration_.valid) {
        ESP_LOGE(TAG, "Cannot save invalid calibration");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving calibration to NVS");

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save as integers (scaled by 1000 for 3 decimal places)
    auto save_float = [&](const char* key, float value) -> esp_err_t {
        int32_t int_val = static_cast<int32_t>(value * 1000.0f);
        return nvs_set_i32(handle, key, int_val);
    };

    ret = save_float(NVS_KEY_OFFSET_X, calibration_.offset_x);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_OFFSET_Y, calibration_.offset_y);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_OFFSET_Z, calibration_.offset_z);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_SCALE_X, calibration_.scale_x);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_SCALE_Y, calibration_.scale_y);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_SCALE_Z, calibration_.scale_z);
    if (ret == ESP_OK) ret = save_float(NVS_KEY_RADIUS, calibration_.sphere_radius);
    if (ret == ESP_OK) ret = nvs_set_u8(handle, NVS_KEY_VALID, 1);

    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save calibration: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t MagCalibrator::loadFromNVS()
{
    ESP_LOGI(TAG, "Loading calibration from NVS");

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No calibration in NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check if valid flag is set
    uint8_t valid = 0;
    ret = nvs_get_u8(handle, NVS_KEY_VALID, &valid);
    if (ret != ESP_OK || valid != 1) {
        nvs_close(handle);
        ESP_LOGW(TAG, "No valid calibration in NVS");
        return ESP_ERR_NOT_FOUND;
    }

    // Load calibration values
    auto load_float = [&](const char* key, float& value) -> esp_err_t {
        int32_t int_val = 0;
        esp_err_t r = nvs_get_i32(handle, key, &int_val);
        if (r == ESP_OK) {
            value = static_cast<float>(int_val) / 1000.0f;
        }
        return r;
    };

    MagCalibration cal;
    ret = load_float(NVS_KEY_OFFSET_X, cal.offset_x);
    if (ret == ESP_OK) ret = load_float(NVS_KEY_OFFSET_Y, cal.offset_y);
    if (ret == ESP_OK) ret = load_float(NVS_KEY_OFFSET_Z, cal.offset_z);
    if (ret == ESP_OK) ret = load_float(NVS_KEY_SCALE_X, cal.scale_x);
    if (ret == ESP_OK) ret = load_float(NVS_KEY_SCALE_Y, cal.scale_y);
    if (ret == ESP_OK) ret = load_float(NVS_KEY_SCALE_Z, cal.scale_z);
    if (ret == ESP_OK) {
        // Radius is optional (older versions may not have it)
        load_float(NVS_KEY_RADIUS, cal.sphere_radius);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        cal.valid = true;
        calibration_ = cal;
        state_ = State::DONE;

        ESP_LOGI(TAG, "Calibration loaded from NVS:");
        ESP_LOGI(TAG, "  Hard Iron: [%.2f, %.2f, %.2f] uT",
                 calibration_.offset_x, calibration_.offset_y, calibration_.offset_z);
        ESP_LOGI(TAG, "  Soft Iron: [%.3f, %.3f, %.3f]",
                 calibration_.scale_x, calibration_.scale_y, calibration_.scale_z);
        ESP_LOGI(TAG, "  Sphere Radius: %.1f uT", calibration_.sphere_radius);
    } else {
        ESP_LOGE(TAG, "Failed to load calibration: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t MagCalibrator::clearNVS()
{
    ESP_LOGI(TAG, "Clearing calibration from NVS");

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    // Reset local state
    calibration_ = MagCalibration();
    state_ = State::IDLE;

    return ret;
}

}  // namespace stampfly
