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
 * @file bmi270_interrupt.c
 * @brief BMI270 Interrupt Configuration Implementation
 *
 * This file implements interrupt configuration functions for the BMI270 sensor.
 */

#include "bmi270_interrupt.h"
#include "bmi270_defs.h"
#include "esp_log.h"

static const char *TAG = "BMI270_INT";

// Forward declarations from bmi270_spi.c
extern esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
extern esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);

/* ====== Interrupt Pin Configuration ====== */

/**
 * @brief Configure interrupt pin output characteristics
 */
esp_err_t bmi270_configure_int_pin(bmi270_dev_t *dev,
                                    bmi270_int_pin_t int_pin,
                                    const bmi270_int_pin_config_t *config) {
    if (dev == NULL || config == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_configure_int_pin");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Select register based on pin
    uint8_t reg_addr = (int_pin == BMI270_INT_PIN_1) ?
                       BMI270_REG_INT1_IO_CTRL : BMI270_REG_INT2_IO_CTRL;

    // Build configuration value
    uint8_t config_val = 0;
    if (config->output_enable) {
        config_val |= BMI270_INT_OUTPUT_EN;
    }
    if (config->active_high) {
        config_val |= BMI270_INT_ACTIVE_HIGH;
    }
    if (config->open_drain) {
        config_val |= BMI270_INT_OPEN_DRAIN;
    }

    // Write configuration
    esp_err_t ret = bmi270_write_register(dev, reg_addr, config_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT%d pin", int_pin + 1);
        return ret;
    }

    ESP_LOGI(TAG, "INT%d configured: output_en=%d, active_high=%d, open_drain=%d (reg=0x%02X)",
             int_pin + 1, config->output_enable, config->active_high,
             config->open_drain, config_val);

    return ESP_OK;
}

/* ====== Data Ready Interrupt Configuration ====== */

/**
 * @brief Enable Data Ready interrupt
 */
esp_err_t bmi270_enable_data_ready_interrupt(bmi270_dev_t *dev,
                                               bmi270_int_pin_t int_pin) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_enable_data_ready_interrupt");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read current INT_MAP_DATA register value
    uint8_t map_data;
    esp_err_t ret = bmi270_read_register(dev, BMI270_REG_INT_MAP_DATA, &map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INT_MAP_DATA register");
        return ret;
    }

    // Set the appropriate bit based on pin selection
    if (int_pin == BMI270_INT_PIN_1) {
        map_data |= BMI270_DRDY_INT1;
    } else {
        map_data |= BMI270_DRDY_INT2;
    }

    // Write updated value
    ret = bmi270_write_register(dev, BMI270_REG_INT_MAP_DATA, map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable Data Ready interrupt on INT%d", int_pin + 1);
        return ret;
    }

    ESP_LOGI(TAG, "Data Ready interrupt enabled on INT%d (INT_MAP_DATA=0x%02X)",
             int_pin + 1, map_data);

    return ESP_OK;
}

/**
 * @brief Disable Data Ready interrupt
 */
esp_err_t bmi270_disable_data_ready_interrupt(bmi270_dev_t *dev,
                                                bmi270_int_pin_t int_pin) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_disable_data_ready_interrupt");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read current INT_MAP_DATA register value
    uint8_t map_data;
    esp_err_t ret = bmi270_read_register(dev, BMI270_REG_INT_MAP_DATA, &map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INT_MAP_DATA register");
        return ret;
    }

    // Clear the appropriate bit based on pin selection
    if (int_pin == BMI270_INT_PIN_1) {
        map_data &= ~BMI270_DRDY_INT1;
    } else {
        map_data &= ~BMI270_DRDY_INT2;
    }

    // Write updated value
    ret = bmi270_write_register(dev, BMI270_REG_INT_MAP_DATA, map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable Data Ready interrupt on INT%d", int_pin + 1);
        return ret;
    }

    ESP_LOGI(TAG, "Data Ready interrupt disabled on INT%d (INT_MAP_DATA=0x%02X)",
             int_pin + 1, map_data);

    return ESP_OK;
}

/* ====== Interrupt Latch Mode Configuration ====== */

/**
 * @brief Set interrupt latch mode
 */
esp_err_t bmi270_set_int_latch_mode(bmi270_dev_t *dev, bool latched) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_set_int_latch_mode");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Set latch mode value
    uint8_t latch_val = latched ? BMI270_INT_LATCH_ENABLED : BMI270_INT_LATCH_DISABLED;

    // Write to INT_LATCH register
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_INT_LATCH, latch_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt latch mode");
        return ret;
    }

    ESP_LOGI(TAG, "Interrupt latch mode set to %s (INT_LATCH=0x%02X)",
             latched ? "LATCHED" : "PULSE", latch_val);

    return ESP_OK;
}
