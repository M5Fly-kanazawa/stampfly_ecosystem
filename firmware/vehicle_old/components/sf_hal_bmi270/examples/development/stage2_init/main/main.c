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
 * @file main.c
 * @brief Stage 2: BMI270 Initialization Sequence Test
 *
 * This example demonstrates:
 * - SPI initialization (from Stage 1)
 * - Complete BMI270 initialization sequence:
 *   - Soft reset
 *   - Config file upload (8KB)
 *   - INTERNAL_STATUS polling
 *   - Sensor enable (ACC + GYR)
 * - Verification that initialization completed successfully
 *
 * Expected output:
 * - Initialization should complete within 20ms
 * - INTERNAL_STATUS message field should = 0x01
 * - Sensors should be enabled (PWR_CTRL = 0x06)
 *
 * Hardware: M5StampS3 + BMI270
 * Connections:
 *   - MOSI: GPIO14
 *   - MISO: GPIO43
 *   - SCK:  GPIO44
 *   - CS:   GPIO46
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bmi270_defs.h"
#include "bmi270_types.h"
#include "bmi270_init.h"

static const char *TAG = "BMI270_TEST";

// Forward declarations
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config);
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Verify sensor power status
 *
 * @param dev Pointer to BMI270 device
 * @return esp_err_t ESP_OK if sensors are enabled
 */
static esp_err_t verify_sensor_power(bmi270_dev_t *dev) {
    uint8_t pwr_ctrl;
    esp_err_t ret;

    ESP_LOGI(TAG, "Verifying sensor power status...");

    ret = bmi270_read_register(dev, BMI270_REG_PWR_CTRL, &pwr_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PWR_CTRL");
        return ret;
    }

    ESP_LOGI(TAG, "PWR_CTRL = 0x%02X", pwr_ctrl);

    bool acc_enabled = (pwr_ctrl & BMI270_PWR_CTRL_ACC_EN) != 0;
    bool gyr_enabled = (pwr_ctrl & BMI270_PWR_CTRL_GYR_EN) != 0;

    ESP_LOGI(TAG, "  Accelerometer: %s", acc_enabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "  Gyroscope:     %s", gyr_enabled ? "ENABLED" : "DISABLED");

    if (acc_enabled && gyr_enabled) {
        ESP_LOGI(TAG, "✓ Both sensors enabled");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "✗ Sensors not properly enabled");
        return ESP_FAIL;
    }
}

/**
 * @brief Verify initialization status
 *
 * @param dev Pointer to BMI270 device
 * @return esp_err_t ESP_OK if init successful
 */
static esp_err_t verify_init_status(bmi270_dev_t *dev) {
    uint8_t internal_status;
    esp_err_t ret;

    ESP_LOGI(TAG, "Verifying initialization status...");

    ret = bmi270_read_register(dev, BMI270_REG_INTERNAL_STATUS, &internal_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INTERNAL_STATUS");
        return ret;
    }

    uint8_t message = internal_status & BMI270_INTERNAL_STATUS_MSG_MASK;

    ESP_LOGI(TAG, "INTERNAL_STATUS = 0x%02X, message = 0x%02X", internal_status, message);

    if (message == BMI270_INTERNAL_STATUS_MSG_INIT_OK) {
        ESP_LOGI(TAG, "✓ Initialization status: OK (message = 0x01)");
        return ESP_OK;
    } else if (message == BMI270_INTERNAL_STATUS_MSG_INIT_ERR) {
        ESP_LOGE(TAG, "✗ Initialization status: ERROR (message = 0x02)");
        return ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "⚠ Unexpected message: 0x%02X", message);
        return ESP_FAIL;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Stage 2: Initialization Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    esp_err_t ret;
    bmi270_dev_t dev = {0};

    // Configure BMI270 (StampFly pin assignment)
    bmi270_config_t config = {
        .gpio_mosi = 14,
        .gpio_miso = 43,
        .gpio_sclk = 44,
        .gpio_cs = 46,
        .spi_clock_hz = 10000000,  // 10 MHz
        .spi_host = SPI2_HOST,
        .gpio_other_cs = 12,       // PMW3901 CS (deactivate for shared SPI bus)
    };

    // Step 1: Initialize SPI
    ESP_LOGI(TAG, "Step 1: Initializing SPI...");
    ret = bmi270_spi_init(&dev, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "✓ SPI initialized successfully");
    ESP_LOGI(TAG, "");

    // Wait for BMI270 power-on (450µs min)
    ESP_LOGI(TAG, "Waiting for BMI270 power-on...");
    vTaskDelay(pdMS_TO_TICKS(5));

    // Step 2: Perform dummy read to activate SPI mode
    ESP_LOGI(TAG, "Step 2: Activating SPI mode...");
    uint8_t dummy;
    bmi270_read_register(&dev, BMI270_REG_CHIP_ID, &dummy);
    vTaskDelay(pdMS_TO_TICKS(5));
    bmi270_read_register(&dev, BMI270_REG_CHIP_ID, &dummy);
    ESP_LOGI(TAG, "SPI mode activated");
    ESP_LOGI(TAG, "");

    // Step 3: Initialize BMI270
    ESP_LOGI(TAG, "Step 3: Initializing BMI270...");
    ret = bmi270_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "");

    // Step 4: Verify initialization
    ESP_LOGI(TAG, "Step 4: Verifying initialization...");
    ret = verify_init_status(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization verification failed!");
        return;
    }
    ESP_LOGI(TAG, "");

    // Step 5: Verify sensor power
    ESP_LOGI(TAG, "Step 5: Verifying sensor power...");
    ret = verify_sensor_power(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor power verification failed!");
        return;
    }
    ESP_LOGI(TAG, "");

    // All tests passed
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ✓ ALL TESTS PASSED");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Stage 2 completed successfully!");
    ESP_LOGI(TAG, "BMI270 is initialized and ready for data reading.");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Next step: Stage 3 - Polling data read");
}
