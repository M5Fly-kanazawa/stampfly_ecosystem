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
 * @brief BMI270 Basic Polling Example
 *
 * Simple example demonstrating periodic polling of gyroscope and accelerometer data.
 * This is the easiest way to get started with the BMI270 sensor.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

static const char *TAG = "BMI270_BASIC";

// M5StampFly BMI270 pin configuration
#define BMI270_MOSI_PIN     14
#define BMI270_MISO_PIN     43
#define BMI270_SCLK_PIN     44
#define BMI270_CS_PIN       46
#define BMI270_SPI_CLOCK_HZ 10000000  // 10 MHz
#define PMW3901_CS_PIN      12        // Other device on shared SPI bus

// Sensor configuration
#define SENSOR_ODR_HZ       100       // Output data rate: 100 Hz
#define POLLING_INTERVAL_MS 10        // Poll every 10ms (100 Hz)

// Global device handle
static bmi270_dev_t g_dev = {0};

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Basic Polling Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
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
    ESP_LOGI(TAG, "SPI initialized");

    // Initialize BMI270
    ESP_LOGI(TAG, "Initializing BMI270 sensor...");
    ret = bmi270_init(&g_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI270");
        return;
    }
    ESP_LOGI(TAG, "BMI270 initialized (CHIP_ID: 0x%02X)", g_dev.chip_id);

    // Configure accelerometer: 100Hz, ±4g range
    ESP_LOGI(TAG, "Configuring accelerometer (100Hz, ±4g)...");
    ret = bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure accelerometer");
    }

    // Configure gyroscope: 100Hz, ±1000°/s range (±17.45 rad/s)
    ESP_LOGI(TAG, "Configuring gyroscope (100Hz, ±1000°/s)...");
    ret = bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure gyroscope");
    }

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Starting data acquisition (100 Hz)");
    ESP_LOGI(TAG, "========================================");

    uint32_t sample_count = 0;

    // Main polling loop
    while (1) {
        bmi270_gyro_t gyro;
        bmi270_accel_t accel;
        float temperature;

        // Read sensor data
        ret = bmi270_read_gyro_accel(&g_dev, &gyro, &accel);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read sensor data");
            vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
            continue;
        }

        // Read temperature (less frequently to reduce overhead)
        if (sample_count % 10 == 0) {
            esp_err_t temp_ret = bmi270_read_temperature(&g_dev, &temperature);
            if (temp_ret == ESP_OK) {
                sample_count++;

                // Print every 10 samples (1 second at 100Hz)
                ESP_LOGI(TAG, "Sample #%lu:", sample_count);
                ESP_LOGI(TAG, "  Gyro  [rad/s]: X=% 7.3f  Y=% 7.3f  Z=% 7.3f",
                         gyro.x, gyro.y, gyro.z);
                ESP_LOGI(TAG, "  Accel [g]:     X=% 7.3f  Y=% 7.3f  Z=% 7.3f",
                         accel.x, accel.y, accel.z);
                ESP_LOGI(TAG, "  Temp  [°C]:    % 7.2f", temperature);
            }
        } else {
            sample_count++;
        }

        // Teleplot output (for real-time visualization)
        printf(">gyr_x:%.3f\n", gyro.x);
        printf(">gyr_y:%.3f\n", gyro.y);
        printf(">gyr_z:%.3f\n", gyro.z);
        printf(">acc_x:%.3f\n", accel.x);
        printf(">acc_y:%.3f\n", accel.y);
        printf(">acc_z:%.3f\n", accel.z);

        // Wait for next sample (10ms = 100Hz polling rate)
        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
    }
}
