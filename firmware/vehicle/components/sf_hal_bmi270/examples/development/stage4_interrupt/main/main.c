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
 * @brief BMI270 Stage 4: Interrupt-based Data Read Example
 *
 * This example demonstrates:
 * - BMI270 initialization
 * - Data Ready interrupt configuration
 * - GPIO interrupt handling on ESP32
 * - Interrupt-driven data reading (no polling)
 * - Real-time data output for Teleplot
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"
#include "bmi270_interrupt.h"

static const char *TAG = "BMI270_STAGE4";

// M5StampFly BMI270 pin configuration
#define BMI270_MOSI_PIN     14
#define BMI270_MISO_PIN     43
#define BMI270_SCLK_PIN     44
#define BMI270_CS_PIN       46
#define BMI270_SPI_CLOCK_HZ 10000000  // 10 MHz
#define PMW3901_CS_PIN      12        // Other device on shared SPI bus

// Output options
#define OUTPUT_RAW_VALUES   0  // Set to 1 to output raw sensor values (LSB)

// BMI270 INT1 pin connected to ESP32 GPIO
#define BMI270_INT1_GPIO    GPIO_NUM_11  // INT1 pin from BMI270 (per M5StampFly hardware)

// Global device handle and interrupt queue
static bmi270_dev_t g_dev = {0};
static QueueHandle_t gpio_evt_queue = NULL;

/**
 * @brief GPIO interrupt handler (IRAM_ATTR for fast execution)
 *
 * This ISR is called when BMI270's INT1 pin triggers.
 * It simply sends a notification to the main task via queue.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief Configure ESP32 GPIO for BMI270 INT1 interrupt
 */
static esp_err_t setup_gpio_interrupt(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,      // Trigger on rising edge (INT1 is active high)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BMI270_INT1_GPIO),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull-down to avoid floating
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO");
        return ret;
    }

    // Create queue for GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_ERR_NO_MEM;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE = already installed
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }

    // Attach interrupt handler to GPIO
    ret = gpio_isr_handler_add(BMI270_INT1_GPIO, gpio_isr_handler, (void*) BMI270_INT1_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler");
        return ret;
    }

    ESP_LOGI(TAG, "GPIO interrupt configured on GPIO %d", BMI270_INT1_GPIO);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Stage 4: Interrupt Data Read");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    // Step 1: Initialize SPI
    ESP_LOGI(TAG, "Step 1: Initializing SPI...");
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
        ESP_LOGE(TAG, "✗ SPI initialization failed");
        return;
    }
    ESP_LOGI(TAG, "✓ SPI initialized successfully");
    ESP_LOGI(TAG, "");

    // Step 2: Activate SPI mode
    ESP_LOGI(TAG, "Step 2: Activating SPI mode...");
    uint8_t dummy;
    bmi270_read_register(&g_dev, BMI270_REG_CHIP_ID, &dummy);  // First dummy read
    vTaskDelay(pdMS_TO_TICKS(5));  // Wait 5ms
    bmi270_read_register(&g_dev, BMI270_REG_CHIP_ID, &dummy);  // Second dummy read
    ESP_LOGI(TAG, "SPI mode activated");
    ESP_LOGI(TAG, "");

    // Step 3: Initialize BMI270
    ESP_LOGI(TAG, "Step 3: Initializing BMI270...");
    ret = bmi270_init(&g_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ BMI270 initialization failed");
        return;
    }
    ESP_LOGI(TAG, "✓ BMI270 initialized successfully");
    ESP_LOGI(TAG, "");

    // Step 4: Configure sensor settings
    ESP_LOGI(TAG, "Step 4: Configuring sensors...");

    // Set accelerometer to ±4g range
    ret = bmi270_set_accel_range(&g_dev, BMI270_ACC_RANGE_4G);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set accelerometer range");
    } else {
        ESP_LOGI(TAG, "Accelerometer range set to ±4g");
    }

    // Set gyroscope to ±1000 °/s range
    ret = bmi270_set_gyro_range(&g_dev, BMI270_GYR_RANGE_1000DPS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gyroscope range");
    } else {
        ESP_LOGI(TAG, "Gyroscope range set to ±1000 °/s");
    }

    // Set accelerometer to 100Hz, Performance mode
    ret = bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set accelerometer config");
    } else {
        ESP_LOGI(TAG, "Accelerometer configured: 100Hz, Performance mode");
    }

    // Set gyroscope to 200Hz, Performance mode
    ret = bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_200HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gyroscope config");
    } else {
        ESP_LOGI(TAG, "Gyroscope configured: 200Hz, Performance mode");
    }

    ESP_LOGI(TAG, "✓ Sensor configuration complete");
    ESP_LOGI(TAG, "");

    // Step 5: Configure INT1 pin
    ESP_LOGI(TAG, "Step 5: Configuring BMI270 INT1 pin...");
    bmi270_int_pin_config_t int_config = {
        .output_enable = true,
        .active_high = true,     // Active high (rises on interrupt)
        .open_drain = false      // Push-pull output
    };

    ret = bmi270_configure_int_pin(&g_dev, BMI270_INT_PIN_1, &int_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to configure INT1 pin");
        return;
    }
    ESP_LOGI(TAG, "✓ INT1 pin configured (Active High, Push-Pull)");
    ESP_LOGI(TAG, "");

    // Step 6: Enable Data Ready interrupt
    ESP_LOGI(TAG, "Step 6: Enabling Data Ready interrupt...");
    ret = bmi270_set_int_latch_mode(&g_dev, false);  // Pulse mode (non-latched)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to set latch mode");
        return;
    }

    ret = bmi270_enable_data_ready_interrupt(&g_dev, BMI270_INT_PIN_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to enable Data Ready interrupt");
        return;
    }
    ESP_LOGI(TAG, "✓ Data Ready interrupt enabled on INT1 (Pulse mode)");
    ESP_LOGI(TAG, "");

    // Step 7: Setup ESP32 GPIO interrupt
    ESP_LOGI(TAG, "Step 7: Setting up ESP32 GPIO interrupt...");
    ret = setup_gpio_interrupt();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ Failed to setup GPIO interrupt");
        return;
    }
    ESP_LOGI(TAG, "✓ GPIO interrupt configured on GPIO %d", BMI270_INT1_GPIO);
    ESP_LOGI(TAG, "");

    // Step 8: Start interrupt-driven data reading
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Interrupt-Driven Data Stream");
    ESP_LOGI(TAG, " (Teleplot format, press Ctrl+] to stop)");
    ESP_LOGI(TAG, "========================================");

    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second before starting

    uint32_t gpio_num;
    uint32_t interrupt_count = 0;

    while (1) {
        // Wait for interrupt event from queue (blocking)
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            interrupt_count++;

            // Read raw data
            bmi270_raw_data_t acc_raw, gyr_raw;
            ret = bmi270_read_accel_raw(&g_dev, &acc_raw);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read accelerometer");
                continue;
            }

            ret = bmi270_read_gyro_raw(&g_dev, &gyr_raw);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read gyroscope");
                continue;
            }

            // Read physical values
            bmi270_accel_t accel;
            bmi270_gyro_t gyro;
            ret = bmi270_read_accel(&g_dev, &accel);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to convert accelerometer data");
                continue;
            }

            ret = bmi270_read_gyro(&g_dev, &gyro);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to convert gyroscope data");
                continue;
            }

            // Read temperature
            float temperature;
            ret = bmi270_read_temperature(&g_dev, &temperature);
            int16_t temp_raw = 0;
            if (ret == ESP_OK) {
                temp_raw = (int16_t)((temperature - 23.0f) * 512.0f);
            } else {
                temperature = 0.0f;
            }

            // Teleplot output format
#if OUTPUT_RAW_VALUES
            // Accelerometer raw (LSB)
            printf(">acc_raw_x:%d\n", acc_raw.x);
            printf(">acc_raw_y:%d\n", acc_raw.y);
            printf(">acc_raw_z:%d\n", acc_raw.z);
#endif

            // Accelerometer physical (g)
            printf(">acc_x:%.4f\n", accel.x);
            printf(">acc_y:%.4f\n", accel.y);
            printf(">acc_z:%.4f\n", accel.z);

#if OUTPUT_RAW_VALUES
            // Gyroscope raw (LSB)
            printf(">gyr_raw_x:%d\n", gyr_raw.x);
            printf(">gyr_raw_y:%d\n", gyr_raw.y);
            printf(">gyr_raw_z:%d\n", gyr_raw.z);
#endif

            // Gyroscope physical (°/s)
            printf(">gyr_x:%.3f\n", gyro.x);
            printf(">gyr_y:%.3f\n", gyro.y);
            printf(">gyr_z:%.3f\n", gyro.z);

#if OUTPUT_RAW_VALUES
            // Temperature raw (LSB)
            printf(">temp_raw:%d\n", temp_raw);
#endif
            // Temperature physical (°C)
            printf(">temp:%.2f\n", temperature);

            // Interrupt count
            printf(">int_count:%lu\n", interrupt_count);

            // Every 100 interrupts, print a status message
            if (interrupt_count % 100 == 0) {
                ESP_LOGI(TAG, "Interrupt count: %lu", interrupt_count);
            }
        }
    }
}
