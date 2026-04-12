/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 StampFly ToF Driver Contributors
 */

/**
 * @file vl53lx_outlier_filter.h
 * @brief VL53LX 1D Kalman Filter for ToF Measurements
 *
 * Provides 1D Kalman filter with outlier rejection:
 * - Prediction-only mode for invalid observations
 * - Range status validation
 * - Rate-of-change limiter
 */

#ifndef VL53LX_OUTLIER_FILTER_H
#define VL53LX_OUTLIER_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Kalman filter configuration
 */
typedef struct {
    bool enable_status_check;            ///< Enable range status validation
    bool enable_rate_limit;              ///< Enable rate-of-change limiter
    uint16_t max_change_rate_mm;         ///< Maximum change rate (mm) between samples
    uint8_t valid_status_mask;           ///< Bitmask of valid range statuses (default: 0x01 for status 0 only)

    // Kalman filter parameters
    float kalman_process_noise;          ///< Process noise covariance Q (default: 1.0)
    float kalman_measurement_noise;      ///< Measurement noise covariance R (default: 4.0)
} vl53lx_filter_config_t;

/**
 * @brief Kalman filter state structure
 */
typedef struct {
    vl53lx_filter_config_t config;       ///< Filter configuration
    uint16_t last_output;                ///< Last filtered output value
    uint8_t rejected_count;              ///< Consecutive rejected samples count
    uint8_t samples_since_reset;         ///< Samples accepted since last reset

    // Kalman filter state (1D)
    float kalman_x;                      ///< Estimated state (distance in mm)
    float kalman_p;                      ///< Estimation error covariance
    bool kalman_initialized;             ///< Kalman filter initialized flag

    bool initialized;                    ///< Filter initialized flag
} vl53lx_filter_t;

/**
 * @brief Initialize Kalman filter with default configuration
 *
 * @param filter Pointer to filter structure
 * @return true if successful, false otherwise
 */
bool VL53LX_FilterInit(vl53lx_filter_t *filter);

/**
 * @brief Initialize Kalman filter with custom configuration
 *
 * @param filter Pointer to filter structure
 * @param config Pointer to configuration
 * @return true if successful, false otherwise
 */
bool VL53LX_FilterInitWithConfig(vl53lx_filter_t *filter, const vl53lx_filter_config_t *config);

/**
 * @brief Deinitialize filter
 *
 * @param filter Pointer to filter structure
 */
void VL53LX_FilterDeinit(vl53lx_filter_t *filter);

/**
 * @brief Reset filter state
 *
 * @param filter Pointer to filter structure
 */
void VL53LX_FilterReset(vl53lx_filter_t *filter);

/**
 * @brief Process new measurement through Kalman filter
 *
 * Uses prediction-only mode for invalid observations.
 *
 * @param filter Pointer to filter structure
 * @param distance_mm Raw distance measurement (mm)
 * @param range_status Range status from sensor
 * @param output_mm Pointer to store filtered output
 * @return true if output is valid, false if not initialized
 */
bool VL53LX_FilterUpdate(vl53lx_filter_t *filter, uint16_t distance_mm, uint8_t range_status, uint16_t *output_mm);

/**
 * @brief Get default Kalman filter configuration (Q=1.0, R=4.0)
 *
 * @return Default configuration structure
 */
vl53lx_filter_config_t VL53LX_FilterGetDefaultConfig(void);

/**
 * @brief Check if range status is valid
 *
 * @param range_status Range status from sensor
 * @return true if valid, false otherwise
 */
bool VL53LX_FilterIsValidRangeStatus(uint8_t range_status);

#ifdef __cplusplus
}
#endif

#endif // VL53LX_OUTLIER_FILTER_H
