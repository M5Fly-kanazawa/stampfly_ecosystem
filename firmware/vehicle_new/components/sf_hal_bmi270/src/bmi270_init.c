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
 * @file bmi270_init.c
 * @brief BMI270 Initialization Implementation
 *
 * This file implements the complete BMI270 initialization sequence
 * according to Bosch Sensortec specifications.
 */

#include "bmi270_init.h"
#include "bmi270_defs.h"
#include "bmi270_config_file.h"
#include "bmi270_data.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMI270_INIT";

// Forward declarations from bmi270_spi.c
extern esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
extern esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);
extern esp_err_t bmi270_write_burst(bmi270_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length);
extern void bmi270_set_init_complete(bmi270_dev_t *dev);

/**
 * @brief Perform soft reset of BMI270
 */
esp_err_t bmi270_soft_reset(bmi270_dev_t *dev) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_soft_reset");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 SPI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Performing soft reset...");

    // Send soft reset command
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_CMD, BMI270_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send soft reset command");
        return ret;
    }

    // Wait for reset to complete (2ms minimum)
    ESP_LOGI(TAG, "Waiting %d µs for reset to complete...", BMI270_DELAY_SOFT_RESET_US);
    esp_rom_delay_us(BMI270_DELAY_SOFT_RESET_US);

    // Re-activate SPI mode (soft reset returns sensor to power-on state)
    ESP_LOGI(TAG, "Re-activating SPI mode after reset...");
    uint8_t dummy;
    bmi270_read_register(dev, BMI270_REG_CHIP_ID, &dummy);  // First dummy read
    vTaskDelay(pdMS_TO_TICKS(5));  // 5ms stabilization
    bmi270_read_register(dev, BMI270_REG_CHIP_ID, &dummy);  // Second dummy read
    ESP_LOGI(TAG, "SPI mode re-activated");

    // Verify CHIP_ID after reset
    uint8_t chip_id;
    ret = bmi270_read_register(dev, BMI270_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID after reset");
        return ret;
    }

    if (chip_id != BMI270_CHIP_ID) {
        ESP_LOGE(TAG, "CHIP_ID mismatch after reset: 0x%02X (expected 0x%02X)", chip_id, BMI270_CHIP_ID);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Soft reset complete, CHIP_ID verified: 0x%02X", chip_id);
    return ESP_OK;
}

/**
 * @brief Upload BMI270 configuration file
 */
esp_err_t bmi270_upload_config_file(bmi270_dev_t *dev) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_upload_config_file");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 SPI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ESP_LOGI(TAG, "Starting config file upload (%d bytes)...", BMI270_CONFIG_FILE_SIZE);

    // Step 1: Disable advanced power save
    ESP_LOGI(TAG, "Disabling advanced power save...");
    ret = bmi270_write_register(dev, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_ADV_PWR_SAVE_EN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable advanced power save");
        return ret;
    }

    // Wait 450µs after power configuration change
    esp_rom_delay_us(BMI270_DELAY_POWER_ON_US);

    // Step 2: Prepare for config file upload
    ESP_LOGI(TAG, "Preparing for config file upload (INIT_CTRL = 0x00)...");
    ret = bmi270_write_register(dev, BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_PREPARE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare for config upload");
        return ret;
    }

    // Step 3: Upload config file in bursts with INIT_ADDR updates
    ESP_LOGI(TAG, "Uploading config file (%d bytes, %d-byte chunks)...",
             BMI270_CONFIG_FILE_SIZE, BMI270_CONFIG_BURST_SIZE);

    size_t bytes_written = 0;
    while (bytes_written < BMI270_CONFIG_FILE_SIZE) {
        size_t chunk_size = BMI270_CONFIG_BURST_SIZE;
        if (bytes_written + chunk_size > BMI270_CONFIG_FILE_SIZE) {
            chunk_size = BMI270_CONFIG_FILE_SIZE - bytes_written;
        }

        // Set INIT_ADDR for this chunk (word addressing: index / 2)
        uint16_t index = bytes_written;
        uint8_t addr_buf[2] = {
            (uint8_t)((index / 2) & 0x0F),    // INIT_ADDR_0: bits 0-3 only
            (uint8_t)((index / 2) >> 4)        // INIT_ADDR_1: bits 4-11
        };

        // Write INIT_ADDR (2 bytes starting from 0x5B)
        ret = bmi270_write_burst(dev, BMI270_REG_INIT_ADDR_0, addr_buf, 2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set INIT_ADDR at offset %d", bytes_written);
            return ret;
        }

        // Write config data chunk
        ret = bmi270_write_burst(dev, BMI270_REG_INIT_DATA,
                                 &bmi270_config_file[bytes_written],
                                 chunk_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write config chunk at offset %d", bytes_written);
            return ret;
        }

        bytes_written += chunk_size;

        // Progress update every 2KB
        if (bytes_written % 2048 == 0) {
            ESP_LOGI(TAG, "Progress: %d / %d bytes (%.1f%%)",
                     bytes_written, BMI270_CONFIG_FILE_SIZE,
                     (float)bytes_written / BMI270_CONFIG_FILE_SIZE * 100.0f);
        }
    }

    ESP_LOGI(TAG, "Config file upload complete (%d bytes)", bytes_written);

    // Step 4: Signal config upload complete
    ESP_LOGI(TAG, "Signaling config upload complete (INIT_CTRL = 0x01)...");
    ret = bmi270_write_register(dev, BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_COMPLETE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to signal config upload complete");
        return ret;
    }

    // Wait for sensor to start initialization process
    ESP_LOGI(TAG, "Waiting for initialization to start...");
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms wait for init process to start

    return ESP_OK;
}

/**
 * @brief Wait for BMI270 initialization to complete
 */
esp_err_t bmi270_wait_init_complete(bmi270_dev_t *dev) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_wait_init_complete");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 SPI not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Waiting for initialization to complete (max %d ms)...", BMI270_TIMEOUT_INIT_MS);

    uint32_t start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t internal_status;
    esp_err_t ret;
    int poll_count = 0;

    while (1) {
        // Read INTERNAL_STATUS register
        ret = bmi270_read_register(dev, BMI270_REG_INTERNAL_STATUS, &internal_status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read INTERNAL_STATUS");
            return ret;
        }

        // Extract message field (lower 4 bits)
        uint8_t message = internal_status & BMI270_INTERNAL_STATUS_MSG_MASK;

        // Log every 5th poll to reduce log spam
        if (poll_count % 5 == 0) {
            ESP_LOGI(TAG, "INTERNAL_STATUS = 0x%02X, message = 0x%02X", internal_status, message);
        }
        poll_count++;

        // Check initialization status
        if (message == BMI270_INTERNAL_STATUS_MSG_INIT_OK) {
            ESP_LOGI(TAG, "✓ Initialization complete (message = 0x01) after %d polls", poll_count);
            return ESP_OK;
        } else if (message == BMI270_INTERNAL_STATUS_MSG_INIT_ERR) {
            ESP_LOGE(TAG, "✗ Initialization error (message = 0x02)");
            return ESP_FAIL;
        }

        // Check timeout
        uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time_ms;
        if (elapsed_ms >= BMI270_TIMEOUT_INIT_MS) {
            ESP_LOGE(TAG, "✗ Initialization timeout after %lu ms (%d polls)", elapsed_ms, poll_count);
            return ESP_ERR_TIMEOUT;
        }

        // Wait 2ms before polling again
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief Initialize BMI270 sensor (complete sequence)
 */
esp_err_t bmi270_init(bmi270_dev_t *dev) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_init");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 SPI not initialized - call bmi270_spi_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Initialization Sequence");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret;

    // Step 1: Soft reset
    ESP_LOGI(TAG, "Step 1: Soft reset");
    ret = bmi270_soft_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        return ret;
    }

    // Step 2: Upload config file
    ESP_LOGI(TAG, "Step 2: Upload configuration file");
    ret = bmi270_upload_config_file(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config file upload failed");
        return ret;
    }

    // Step 3: Wait for initialization complete
    ESP_LOGI(TAG, "Step 3: Wait for initialization complete");
    ret = bmi270_wait_init_complete(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization wait failed");
        return ret;
    }

    // Step 4: Enable accelerometer, gyroscope, and temperature sensor
    ESP_LOGI(TAG, "Step 4: Enable sensors");
    uint8_t pwr_ctrl = BMI270_PWR_CTRL_ACC_EN | BMI270_PWR_CTRL_GYR_EN | BMI270_PWR_CTRL_TEMP_EN;
    ret = bmi270_write_register(dev, BMI270_REG_PWR_CTRL, pwr_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return ret;
    }

    ESP_LOGI(TAG, "Accelerometer, gyroscope, and temperature enabled (PWR_CTRL = 0x%02X)", pwr_ctrl);

    // Wait for sensors to stabilize (2ms)
    vTaskDelay(pdMS_TO_TICKS(2));

    // Step 5: Mark initialization complete (switch to normal mode timing)
    bmi270_set_init_complete(dev);

    // Step 6: Read and store default range settings
    ESP_LOGI(TAG, "Step 6: Read default range settings");

    bmi270_acc_range_t acc_range;
    bmi270_gyr_range_t gyr_range;

    ret = bmi270_get_accel_range(dev, &acc_range);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read accelerometer range, using default ±2g");
        dev->acc_range = BMI270_ACC_RANGE_2G;
    }

    ret = bmi270_get_gyro_range(dev, &gyr_range);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read gyroscope range, using default ±2000°/s");
        dev->gyr_range = BMI270_GYR_RANGE_2000DPS;
    }

    ESP_LOGI(TAG, "Default ranges: ACC=0x%02X, GYR=0x%02X", dev->acc_range, dev->gyr_range);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ✓ BMI270 Initialization Complete");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}
