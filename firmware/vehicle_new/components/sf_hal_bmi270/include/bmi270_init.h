/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Kouhei Ito
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmi270_init.h
 * @brief BMI270 Initialization API
 *
 * This file provides the initialization sequence for BMI270:
 * 1. Soft reset (optional)
 * 2. Disable advanced power save
 * 3. Upload 8KB configuration file
 * 4. Poll INTERNAL_STATUS for initialization complete
 * 5. Enable accelerometer and gyroscope
 *
 * The initialization must be completed before reading sensor data.
 */

#ifndef BMI270_INIT_H
#define BMI270_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi270_types.h"
#include "esp_err.h"

/**
 * @brief Initialize BMI270 sensor
 *
 * This function performs the complete BMI270 initialization sequence:
 * - Performs soft reset (clears previous state)
 * - Waits for power-on stabilization
 * - Disables advanced power save mode (PWR_CONF = 0x00)
 * - Uploads 8KB configuration file to INIT_DATA register
 * - Polls INTERNAL_STATUS until initialization complete (message = 0x01)
 * - Enables accelerometer and gyroscope (PWR_CTRL)
 * - Switches to normal mode timing (2µs delays)
 *
 * @param dev Pointer to BMI270 device structure (must be initialized with bmi270_spi_init)
 * @return esp_err_t
 *         - ESP_OK: Initialization successful
 *         - ESP_ERR_INVALID_ARG: NULL pointer
 *         - ESP_ERR_INVALID_STATE: SPI not initialized
 *         - ESP_ERR_TIMEOUT: Initialization timeout (>20ms)
 *         - ESP_FAIL: Initialization error reported by sensor
 */
esp_err_t bmi270_init(bmi270_dev_t *dev);

/**
 * @brief Perform soft reset of BMI270
 *
 * Resets the sensor to default state. All register settings are lost.
 * The sensor requires 2ms to complete reset.
 *
 * @param dev Pointer to BMI270 device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_soft_reset(bmi270_dev_t *dev);

/**
 * @brief Upload BMI270 configuration file
 *
 * Uploads the 8KB Bosch configuration file to the sensor.
 * This is done in 32-byte bursts via the INIT_DATA register (0x5E).
 *
 * @param dev Pointer to BMI270 device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_upload_config_file(bmi270_dev_t *dev);

/**
 * @brief Wait for BMI270 initialization to complete
 *
 * Polls INTERNAL_STATUS register until message field = 0x01 (init OK)
 * or 0x02 (init error). Maximum wait time is 20ms.
 *
 * @param dev Pointer to BMI270 device structure
 * @return esp_err_t
 *         - ESP_OK: Initialization complete (message = 0x01)
 *         - ESP_FAIL: Initialization error (message = 0x02)
 *         - ESP_ERR_TIMEOUT: Timeout waiting for completion
 */
esp_err_t bmi270_wait_init_complete(bmi270_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif // BMI270_INIT_H
