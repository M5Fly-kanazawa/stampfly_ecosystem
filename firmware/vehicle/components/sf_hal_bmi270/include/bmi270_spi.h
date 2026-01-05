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
 * @file bmi270_spi.h
 * @brief BMI270 SPI communication interface
 *
 * This file contains function prototypes for SPI communication
 * with the BMI270 sensor.
 */

#ifndef BMI270_SPI_H
#define BMI270_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi270_types.h"
#include "bmi270_defs.h"

/**
 * @brief Initialize SPI bus and add BMI270 device
 *
 * @param dev Pointer to BMI270 device structure
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config);

/**
 * @brief Read single register from BMI270
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Register address (0x00-0x7F)
 * @param data Pointer to store read data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Write single register to BMI270
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Register address (0x00-0x7F)
 * @param data Data to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read multiple registers from BMI270 (burst read)
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Starting register address
 * @param data Pointer to buffer for read data
 * @param length Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_read_burst(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length);

/**
 * @brief Write multiple registers to BMI270 (burst write)
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Starting register address
 * @param data Pointer to data to write
 * @param length Number of bytes to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_write_burst(bmi270_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length);

/**
 * @brief Mark BMI270 initialization as complete
 *
 * Call this function after BMI270 initialization sequence is complete
 * to switch to normal mode timing (2µs instead of 1000µs).
 *
 * @param dev Pointer to BMI270 device structure
 */
void bmi270_set_init_complete(bmi270_dev_t *dev);

/**
 * @brief Override low-power mode delay for testing (experimental)
 *
 * Temporarily override the low-power mode delay time for optimization experiments.
 * This function is for testing purposes only.
 *
 * @param delay_us New delay time in microseconds (0 = use default)
 */
void bmi270_set_lowpower_delay_override(uint32_t delay_us);

#ifdef __cplusplus
}
#endif

#endif // BMI270_SPI_H
