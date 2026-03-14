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
 * @file bmi270_interrupt.h
 * @brief BMI270 Interrupt Configuration API
 *
 * This file provides the API for configuring and using BMI270 interrupts,
 * particularly the Data Ready interrupt.
 */

#ifndef BMI270_INTERRUPT_H
#define BMI270_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi270_types.h"
#include "esp_err.h"

/* ====== Interrupt Pin Selection ====== */

/**
 * @brief BMI270 interrupt pin selection
 */
typedef enum {
    BMI270_INT_PIN_1 = 0,  ///< Use INT1 pin
    BMI270_INT_PIN_2 = 1   ///< Use INT2 pin
} bmi270_int_pin_t;

/* ====== Interrupt Pin Configuration ====== */

/**
 * @brief Interrupt pin output configuration
 */
typedef struct {
    bool output_enable;     ///< Enable interrupt output
    bool active_high;       ///< true = Active High, false = Active Low
    bool open_drain;        ///< true = Open-Drain, false = Push-Pull
} bmi270_int_pin_config_t;

/* ====== Interrupt Configuration Functions ====== */

/**
 * @brief Configure interrupt pin output characteristics
 *
 * @param dev Pointer to BMI270 device structure
 * @param int_pin Interrupt pin selection (INT1 or INT2)
 * @param config Pointer to pin configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bmi270_configure_int_pin(bmi270_dev_t *dev,
                                    bmi270_int_pin_t int_pin,
                                    const bmi270_int_pin_config_t *config);

/**
 * @brief Enable Data Ready interrupt
 *
 * Configures the BMI270 to generate an interrupt when new sensor data is ready.
 *
 * @param dev Pointer to BMI270 device structure
 * @param int_pin Interrupt pin to use (INT1 or INT2)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bmi270_enable_data_ready_interrupt(bmi270_dev_t *dev,
                                               bmi270_int_pin_t int_pin);

/**
 * @brief Disable Data Ready interrupt
 *
 * @param dev Pointer to BMI270 device structure
 * @param int_pin Interrupt pin to disable (INT1 or INT2)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bmi270_disable_data_ready_interrupt(bmi270_dev_t *dev,
                                                bmi270_int_pin_t int_pin);

/**
 * @brief Set interrupt latch mode
 *
 * @param dev Pointer to BMI270 device structure
 * @param latched true = Latched mode, false = Pulse mode (non-latched)
 * @return ESP_OK on success, error code otherwise
 *
 * @note In latched mode, interrupts must be manually cleared.
 *       In pulse mode, interrupts are automatically cleared.
 */
esp_err_t bmi270_set_int_latch_mode(bmi270_dev_t *dev, bool latched);

#ifdef __cplusplus
}
#endif

#endif // BMI270_INTERRUPT_H
