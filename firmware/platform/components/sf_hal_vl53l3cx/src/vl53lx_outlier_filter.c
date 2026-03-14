/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 StampFly ToF Driver Contributors
 */

/**
 * @file vl53lx_outlier_filter.c
 * @brief VL53LX 1D Kalman Filter Implementation
 */

#include "vl53lx_outlier_filter.h"
#include <stdlib.h>

// Default configuration values
#define DEFAULT_MAX_CHANGE_RATE_MM  500     // 500mm max change between samples
#define DEFAULT_VALID_STATUS_MASK   0x01    // Only status 0 (valid) by default
#define DEFAULT_KALMAN_Q            1.0f    // Process noise (responsive)
#define DEFAULT_KALMAN_R            4.0f    // Measurement noise (~2mm std)

vl53lx_filter_config_t VL53LX_FilterGetDefaultConfig(void)
{
    vl53lx_filter_config_t config = {
        .enable_status_check = true,
        .enable_rate_limit = true,
        .max_change_rate_mm = DEFAULT_MAX_CHANGE_RATE_MM,
        .valid_status_mask = DEFAULT_VALID_STATUS_MASK,
        .kalman_process_noise = DEFAULT_KALMAN_Q,
        .kalman_measurement_noise = DEFAULT_KALMAN_R,
    };
    return config;
}

bool VL53LX_FilterInit(vl53lx_filter_t *filter)
{
    vl53lx_filter_config_t config = VL53LX_FilterGetDefaultConfig();
    return VL53LX_FilterInitWithConfig(filter, &config);
}

bool VL53LX_FilterInitWithConfig(vl53lx_filter_t *filter, const vl53lx_filter_config_t *config)
{
    if (filter == NULL || config == NULL) {
        return false;
    }

    // Copy configuration
    filter->config = *config;

    // Initialize state
    filter->last_output = 0;
    filter->rejected_count = 0;
    filter->samples_since_reset = 0;

    // Initialize Kalman filter state
    filter->kalman_x = 0.0f;
    filter->kalman_p = 1000.0f;  // High initial uncertainty
    filter->kalman_initialized = false;

    filter->initialized = true;

    return true;
}

void VL53LX_FilterDeinit(vl53lx_filter_t *filter)
{
    if (filter == NULL || !filter->initialized) {
        return;
    }

    filter->initialized = false;
}

void VL53LX_FilterReset(vl53lx_filter_t *filter)
{
    if (filter == NULL || !filter->initialized) {
        return;
    }

    filter->last_output = 0;
    filter->rejected_count = 0;
    filter->samples_since_reset = 0;

    // Reset Kalman filter
    filter->kalman_x = 0.0f;
    filter->kalman_p = 1000.0f;
    filter->kalman_initialized = false;
}

bool VL53LX_FilterIsValidRangeStatus(uint8_t range_status)
{
    // Status 0 = Valid measurement
    // Status 1,2 = Signal/Sigma failures (might be usable)
    // Status 4+ = Invalid (out of range, timeout, etc.)
    return (range_status == 0);
}

bool VL53LX_FilterUpdate(vl53lx_filter_t *filter, uint16_t distance_mm, uint8_t range_status, uint16_t *output_mm)
{
    if (filter == NULL || !filter->initialized || output_mm == NULL) {
        return false;
    }

    bool status_valid = true;
    bool rate_valid = true;

    // Check range status if enabled
    if (filter->config.enable_status_check) {
        if (!((1 << range_status) & filter->config.valid_status_mask)) {
            // Status is invalid
            status_valid = false;
            filter->rejected_count++;

            // If too many consecutive rejections, reset filter
            if (filter->rejected_count >= 5) {
                VL53LX_FilterReset(filter);
            }
        }
    }

    // Check rate of change if enabled and filter has previous output
    if (filter->config.enable_rate_limit && filter->kalman_initialized) {
        int32_t change = (int32_t)distance_mm - (int32_t)filter->last_output;

        // After reset, allow larger changes for first few samples
        uint16_t effective_rate_limit = filter->config.max_change_rate_mm;
        if (filter->samples_since_reset < 3) {
            effective_rate_limit = filter->config.max_change_rate_mm * 3;  // 3x more lenient
        }

        if (abs(change) > effective_rate_limit) {
            // Change too large
            rate_valid = false;
            filter->rejected_count++;

            // If too many consecutive rejections, reset filter to accept new baseline
            if (filter->rejected_count >= 5) {
                VL53LX_FilterReset(filter);
            }
        }
    }

    // Sample accepted (either valid or will do prediction-only), reset rejection counter if fully valid
    if (status_valid && rate_valid) {
        filter->rejected_count = 0;
    }

    // Kalman filter
    uint16_t filtered_value;

    if (!filter->kalman_initialized) {
        // Initialize with first measurement (only if valid)
        if (status_valid && rate_valid) {
            filter->kalman_x = (float)distance_mm;
            filter->kalman_p = filter->config.kalman_measurement_noise;
            filter->kalman_initialized = true;
            filtered_value = distance_mm;
        } else {
            // Cannot initialize with invalid sample
            return false;
        }
    } else {
        // Kalman filter update
        float Q = filter->config.kalman_process_noise;
        float R = filter->config.kalman_measurement_noise;

        // Prediction step (always execute)
        float x_pred = filter->kalman_x;  // No state transition for stationary model
        float p_pred = filter->kalman_p + Q;

        if (status_valid && rate_valid) {
            // Observation valid: perform full update (prediction + observation)
            float K = p_pred / (p_pred + R);  // Kalman gain
            float z = (float)distance_mm;      // Measurement
            filter->kalman_x = x_pred + K * (z - x_pred);
            filter->kalman_p = (1.0f - K) * p_pred;
        } else {
            // Observation invalid: prediction only (uncertainty increases)
            filter->kalman_x = x_pred;
            filter->kalman_p = p_pred;  // Uncertainty grows
        }

        filtered_value = (uint16_t)(filter->kalman_x + 0.5f);  // Round to nearest integer
    }

    *output_mm = filtered_value;
    filter->last_output = filtered_value;

    // Increment samples since reset only for fully valid samples (cap at 255 to prevent overflow)
    if (status_valid && rate_valid && filter->samples_since_reset < 255) {
        filter->samples_since_reset++;
    }

    return true;
}
