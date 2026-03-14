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
 * @brief BMI270 Step 2: FIFO Multiple Frames Read (Averaged Output)
 *
 * This example demonstrates:
 * - Reading all available FIFO data in one burst
 * - Parsing multiple frames from FIFO buffer
 * - Handling special headers (0x40, 0x48)
 * - Preventing data loss by reading FIFO_LENGTH bytes
 * - Outputting averaged sensor data to reduce printf overhead
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

static const char *TAG = "BMI270_STEP2";

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
#define FIFO_HEADER_SKIP            0x40    // Skip frame (data loss)
#define FIFO_HEADER_CONFIG          0x48    // Config change frame
#define FIFO_MAX_SIZE               2048    // Maximum FIFO size

// Global device handle
static bmi270_dev_t g_dev = {0};

// Statistics
static uint32_t g_total_frames = 0;
static uint32_t g_valid_frames = 0;
static uint32_t g_skip_frames = 0;
static uint32_t g_config_frames = 0;

// Timestamp baseline (for relative timestamps)
static int64_t g_start_time_us = 0;

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
 * @brief Read multiple frames from FIFO
 */
static esp_err_t read_fifo_data(uint8_t *buffer, uint16_t length)
{
    return bmi270_read_burst(&g_dev, BMI270_REG_FIFO_DATA, buffer, length);
}

/**
 * @brief Parse one FIFO frame and accumulate data
 * @param frame_data Frame data buffer
 * @param gyro Output gyroscope data (physical values)
 * @param accel Output accelerometer data (physical values)
 * @return ESP_OK if frame is valid ACC+GYR frame, ESP_FAIL otherwise
 */
static esp_err_t parse_frame(const uint8_t *frame_data, bmi270_gyro_t *gyro, bmi270_accel_t *accel)
{
    uint8_t header = frame_data[0];

    // Handle special headers
    if (header == FIFO_HEADER_CONFIG) {
        ESP_LOGD(TAG, "Config change frame (0x48)");
        g_config_frames++;
        return ESP_FAIL;
    }

    if (header == FIFO_HEADER_SKIP) {
        ESP_LOGW(TAG, "Skip frame (0x40) - data loss detected!");
        g_skip_frames++;
        return ESP_FAIL;
    }

    if (header != FIFO_HEADER_ACC_GYR) {
        ESP_LOGW(TAG, "Unknown header: 0x%02X", header);
        return ESP_FAIL;
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

    bmi270_convert_gyro_raw(&g_dev, &gyr_raw, gyro);
    bmi270_convert_accel_raw(&g_dev, &acc_raw, accel);

    g_valid_frames++;
    return ESP_OK;
}

/**
 * @brief Parse all frames in FIFO buffer and output averaged data
 */
static void parse_fifo_buffer(const uint8_t *buffer, uint16_t length)
{
    int num_frames = length / FIFO_FRAME_SIZE_HEADER;
    int valid_count = 0;

    ESP_LOGI(TAG, "Parsing %d frames (%u bytes)", num_frames, length);

    // Get current timestamp (for this batch read)
    int64_t base_time_us = esp_timer_get_time();

    // Initialize baseline timestamp on first call
    if (g_start_time_us == 0) {
        g_start_time_us = base_time_us;
        ESP_LOGI(TAG, "Initialized timestamp baseline (t=0.000000s)");
    }

    // Accumulators for averaging
    double sum_gyr_x = 0.0, sum_gyr_y = 0.0, sum_gyr_z = 0.0;
    double sum_acc_x = 0.0, sum_acc_y = 0.0, sum_acc_z = 0.0;

    // Parse all frames and accumulate values
    for (int i = 0; i < num_frames; i++) {
        const uint8_t *frame = &buffer[i * FIFO_FRAME_SIZE_HEADER];
        g_total_frames++;

        bmi270_gyro_t gyro;
        bmi270_accel_t accel;

        if (parse_frame(frame, &gyro, &accel) == ESP_OK) {
            sum_gyr_x += gyro.x;
            sum_gyr_y += gyro.y;
            sum_gyr_z += gyro.z;
            sum_acc_x += accel.x;
            sum_acc_y += accel.y;
            sum_acc_z += accel.z;
            valid_count++;
        }
    }

    ESP_LOGI(TAG, "Valid frames: %d/%d", valid_count, num_frames);

    // Calculate average and output (only if we have valid data)
    if (valid_count > 0) {
        double avg_gyr_x = sum_gyr_x / valid_count;
        double avg_gyr_y = sum_gyr_y / valid_count;
        double avg_gyr_z = sum_gyr_z / valid_count;
        double avg_acc_x = sum_acc_x / valid_count;
        double avg_acc_y = sum_acc_y / valid_count;
        double avg_acc_z = sum_acc_z / valid_count;

        // Timestamp in relative seconds
        double timestamp_sec = (double)(base_time_us - g_start_time_us) / 1000000.0;

        // Teleplot output format (averaged data with timestamp)
        printf(">gyr_x:%.6f:%.2f\n", timestamp_sec, avg_gyr_x);
        printf(">gyr_y:%.6f:%.2f\n", timestamp_sec, avg_gyr_y);
        printf(">gyr_z:%.6f:%.2f\n", timestamp_sec, avg_gyr_z);
        printf(">acc_x:%.6f:%.3f\n", timestamp_sec, avg_acc_x);
        printf(">acc_y:%.6f:%.3f\n", timestamp_sec, avg_acc_y);
        printf(">acc_z:%.6f:%.3f\n", timestamp_sec, avg_acc_z);
    }
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Step 2: FIFO Multiple Frames Read");
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

    // Step 6: Start FIFO multi-frame read loop
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " FIFO Multi-Frame Read Loop (Teleplot format)");
    ESP_LOGI(TAG, "========================================");

    static uint8_t fifo_buffer[FIFO_MAX_SIZE];  // Static to avoid stack overflow
    uint32_t loop_count = 0;

    while (1) {
        loop_count++;

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
            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "Loop #%lu, FIFO length: %u bytes", loop_count, fifo_length);

            // Read all FIFO data
            ret = read_fifo_data(fifo_buffer, fifo_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read FIFO data");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Parse all frames in buffer
            parse_fifo_buffer(fifo_buffer, fifo_length);

            // Read FIFO length again to verify data was consumed
            uint16_t fifo_length_after;
            read_fifo_length(&fifo_length_after);
            ESP_LOGI(TAG, "FIFO length after read: %u bytes (consumed: %u bytes)",
                     fifo_length_after, fifo_length - fifo_length_after);

            // Statistics
            ESP_LOGI(TAG, "Statistics: Total=%lu Valid=%lu Skip=%lu Config=%lu",
                     g_total_frames, g_valid_frames, g_skip_frames, g_config_frames);
        }

        // Delay before next read (100ms = 10Hz polling)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
