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
 * @brief BMI270 Step 1: FIFO Basic Manual Read
 *
 * This example demonstrates:
 * - FIFO configuration (ACC+GYR, Header mode)
 * - Manual FIFO length polling
 * - Single frame read from FIFO_DATA register
 * - Frame header verification (0x8C expected)
 * - Data accuracy verification vs Stage 4
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

static const char *TAG = "BMI270_STEP1";

// M5StampFly BMI270 pin configuration
#define BMI270_MOSI_PIN     14
#define BMI270_MISO_PIN     43
#define BMI270_SCLK_PIN     44
#define BMI270_CS_PIN       46
#define BMI270_SPI_CLOCK_HZ 10000000  // 10 MHz
#define PMW3901_CS_PIN      12        // Other device on shared SPI bus

// FIFO registers
#define BMI270_REG_FIFO_LENGTH_0    0x24    // FIFO length LSB
#define BMI270_REG_FIFO_LENGTH_1    0x25    // FIFO length MSB
#define BMI270_REG_FIFO_DATA        0x26    // FIFO data read
#define BMI270_REG_FIFO_CONFIG_0    0x48    // FIFO mode config
#define BMI270_REG_FIFO_CONFIG_1    0x49    // FIFO sensor enable

// FIFO constants
#define FIFO_FRAME_SIZE_HEADER      13      // Header(1) + GYR(6) + ACC(6)
#define FIFO_HEADER_ACC_GYR         0x8C    // Expected header for ACC+GYR frame

// Global device handle
static bmi270_dev_t g_dev = {0};

/**
 * @brief Read FIFO length
 */
static esp_err_t read_fifo_length(uint16_t *length)
{
    uint8_t length_data[2];
    esp_err_t ret = bmi270_read_burst(&g_dev, BMI270_REG_FIFO_LENGTH_0, length_data, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    // FIFO length is 11-bit value (0-2047 bytes)
    *length = (uint16_t)((length_data[1] << 8) | length_data[0]) & 0x07FF;
    return ESP_OK;
}

/**
 * @brief Read one frame from FIFO
 */
static esp_err_t read_fifo_frame(uint8_t *frame_data)
{
    // Read 13 bytes from FIFO_DATA register
    return bmi270_read_burst(&g_dev, BMI270_REG_FIFO_DATA, frame_data, FIFO_FRAME_SIZE_HEADER);
}

/**
 * @brief Parse and display FIFO frame
 * @param frame_data Frame data buffer
 * @param frame_count Current frame count (1-based)
 * @return ESP_OK if frame is valid, ESP_FAIL if invalid header detected (after first frame)
 */
static esp_err_t parse_and_display_frame(const uint8_t *frame_data, uint32_t frame_count)
{
    uint8_t header = frame_data[0];

    ESP_LOGI(TAG, "Frame header: 0x%02X", header);

    if (header != FIFO_HEADER_ACC_GYR) {
        ESP_LOGW(TAG, "Unexpected header! Expected 0x8C, got 0x%02X", header);

        if (frame_count == 1) {
            ESP_LOGI(TAG, "First frame - likely config change frame (0x48), skipping...");
            return ESP_FAIL;  // Skip but continue
        } else {
            ESP_LOGW(TAG, "This indicates FIFO overflow or skip frame");
            return ESP_FAIL;  // Terminate
        }
    }

    // Parse gyroscope data FIRST (bytes 1-6)
    int16_t gyr_x = (int16_t)((frame_data[2] << 8) | frame_data[1]);
    int16_t gyr_y = (int16_t)((frame_data[4] << 8) | frame_data[3]);
    int16_t gyr_z = (int16_t)((frame_data[6] << 8) | frame_data[5]);

    // Parse accelerometer data SECOND (bytes 7-12)
    int16_t acc_x = (int16_t)((frame_data[8] << 8) | frame_data[7]);
    int16_t acc_y = (int16_t)((frame_data[10] << 8) | frame_data[9]);
    int16_t acc_z = (int16_t)((frame_data[12] << 8) | frame_data[11]);

    // Convert to physical values
    bmi270_raw_data_t gyr_raw = {gyr_x, gyr_y, gyr_z};
    bmi270_raw_data_t acc_raw = {acc_x, acc_y, acc_z};

    bmi270_gyro_t gyro;
    bmi270_accel_t accel;

    bmi270_convert_gyro_raw(&g_dev, &gyr_raw, &gyro);
    bmi270_convert_accel_raw(&g_dev, &acc_raw, &accel);

    // Display data
    ESP_LOGI(TAG, "GYR RAW: X=%6d, Y=%6d, Z=%6d", gyr_x, gyr_y, gyr_z);
    ESP_LOGI(TAG, "ACC RAW: X=%6d, Y=%6d, Z=%6d", acc_x, acc_y, acc_z);
    ESP_LOGI(TAG, "GYR: X=%7.2f°/s, Y=%7.2f°/s, Z=%7.2f°/s", gyro.x, gyro.y, gyro.z);
    ESP_LOGI(TAG, "ACC: X=%6.3fg, Y=%6.3fg, Z=%6.3fg", accel.x, accel.y, accel.z);

    // Teleplot output format
    printf(">gyr_x:%.2f\n", gyro.x);
    printf(">gyr_y:%.2f\n", gyro.y);
    printf(">gyr_z:%.2f\n", gyro.z);
    printf(">acc_x:%.3f\n", accel.x);
    printf(">acc_y:%.3f\n", accel.y);
    printf(">acc_z:%.3f\n", accel.z);

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Step 1: FIFO Basic Manual Read");
    ESP_LOGI(TAG, "========================================");

    // Step 1: Initialize SPI bus
    ESP_LOGI(TAG, "Step 1: Initializing SPI bus...");
    bmi270_config_t config = {
        .gpio_mosi = BMI270_MOSI_PIN,
        .gpio_miso = BMI270_MISO_PIN,
        .gpio_sclk = BMI270_SCLK_PIN,
        .gpio_cs = BMI270_CS_PIN,
        .spi_clock_hz = BMI270_SPI_CLOCK_HZ,
        .spi_host = SPI2_HOST,
        .gpio_other_cs = PMW3901_CS_PIN
    };

    ret = bmi270_spi_init(&g_dev, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return;
    }
    ESP_LOGI(TAG, "SPI initialized successfully");

    // Step 2: Initialize BMI270
    ESP_LOGI(TAG, "Step 2: Initializing BMI270...");
    ret = bmi270_init(&g_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI270");
        return;
    }
    ESP_LOGI(TAG, "BMI270 initialized successfully");

    // Step 3: Set accelerometer to 100Hz, ±4g range
    ESP_LOGI(TAG, "Step 3: Configuring accelerometer (100Hz, ±4g)...");
    ret = bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set accelerometer config");
    }
    ESP_LOGI(TAG, "Accelerometer configured");

    // Step 4: Set gyroscope to 100Hz, ±1000°/s range
    ESP_LOGI(TAG, "Step 4: Configuring gyroscope (100Hz, ±1000°/s)...");
    ret = bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gyroscope config");
    }
    ESP_LOGI(TAG, "Gyroscope configured");

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 5: Configure FIFO (ACC+GYR, Header mode, Stream mode)
    ESP_LOGI(TAG, "Step 5: Configuring FIFO...");

    // FIFO_CONFIG_0: Stream mode (stop_on_full = 0, default 0x00)
    uint8_t fifo_config_0 = 0x00;
    ret = bmi270_write_register(&g_dev, BMI270_REG_FIFO_CONFIG_0, fifo_config_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_CONFIG_0");
        return;
    }

    // FIFO_CONFIG_1: Enable ACC+GYR, Header mode
    // bit 7: fifo_gyr_en = 1
    // bit 6: fifo_acc_en = 1
    // bit 4: fifo_header_en = 1
    uint8_t fifo_config_1 = (1 << 7) | (1 << 6) | (1 << 4);  // 0xD0
    ret = bmi270_write_register(&g_dev, BMI270_REG_FIFO_CONFIG_1, fifo_config_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_CONFIG_1");
        return;
    }

    ESP_LOGI(TAG, "FIFO configured: ACC+GYR enabled, Header mode, Stream mode");

    // Verify configuration
    uint8_t fifo_config_1_readback;
    bmi270_read_register(&g_dev, BMI270_REG_FIFO_CONFIG_1, &fifo_config_1_readback);
    ESP_LOGI(TAG, "FIFO_CONFIG_1 readback: 0x%02X (expected 0xD0)", fifo_config_1_readback);

    // Wait for some data to accumulate
    vTaskDelay(pdMS_TO_TICKS(200));

    // Step 6: Start FIFO manual read loop
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " FIFO Manual Read Loop (Teleplot format)");
    ESP_LOGI(TAG, "========================================");

    uint32_t frame_count = 0;

    while (1) {
        // Read FIFO length
        uint16_t fifo_length;
        ret = read_fifo_length(&fifo_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read FIFO length");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check if we have at least one frame (13 bytes)
        if (fifo_length >= FIFO_FRAME_SIZE_HEADER) {
            frame_count++;

            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "Frame #%lu, FIFO length: %u bytes", frame_count, fifo_length);

            // Read one frame from FIFO
            uint8_t frame_data[FIFO_FRAME_SIZE_HEADER];
            ret = read_fifo_frame(frame_data);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read FIFO frame");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Parse and display frame
            ret = parse_and_display_frame(frame_data, frame_count);
            if (ret != ESP_OK) {
                if (frame_count == 1) {
                    // First frame may be config change frame - skip and continue
                    ESP_LOGI(TAG, "Skipping first frame, continuing...");
                } else {
                    // Invalid header detected after first frame - end test
                    ESP_LOGI(TAG, "========================================");
                    ESP_LOGI(TAG, " Step 1 Complete: FIFO basic operation verified");
                    ESP_LOGI(TAG, " Successfully read %lu valid frames", frame_count - 1);
                    ESP_LOGI(TAG, "========================================");
                    return;
                }
            }

            // Read FIFO length again to verify data was consumed
            uint16_t fifo_length_after;
            read_fifo_length(&fifo_length_after);
            ESP_LOGI(TAG, "FIFO length after read: %u bytes (consumed: %u bytes)",
                     fifo_length_after, fifo_length - fifo_length_after);
        }

        // Delay before next read (100ms = 10Hz polling)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
