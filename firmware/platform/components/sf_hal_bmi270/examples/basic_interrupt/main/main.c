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
 * @brief BMI270 Basic Interrupt Example
 *
 * Demonstrates efficient data acquisition using DATA_RDY interrupt.
 * Lower CPU usage compared to polling, as the MCU sleeps until new data is available.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

static const char *TAG = "BMI270_BASIC_INT";

// M5StampFly BMI270 pin configuration
#define BMI270_MOSI_PIN     14
#define BMI270_MISO_PIN     43
#define BMI270_SCLK_PIN     44
#define BMI270_CS_PIN       46
#define BMI270_INT1_PIN     11        // INT1 interrupt pin
#define BMI270_SPI_CLOCK_HZ 10000000  // 10 MHz
#define PMW3901_CS_PIN      12        // Other device on shared SPI bus

// Sensor configuration
#define SENSOR_ODR_HZ       100       // Output data rate: 100 Hz

// Register definitions
#define BMI270_REG_INT1_IO_CTRL     0x53    // INT1 pin config
#define BMI270_REG_INT_MAP_DATA     0x56    // Interrupt mapping

// Global device handle
static bmi270_dev_t g_dev = {0};

// Semaphore for interrupt notification
static SemaphoreHandle_t data_ready_sem = NULL;

// Statistics
static uint32_t g_sample_count = 0;

/**
 * @brief INT1 interrupt handler (called on DATA_RDY)
 */
static void IRAM_ATTR bmi270_int1_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(data_ready_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Configure GPIO for INT1 interrupt
 */
static esp_err_t configure_int1_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BMI270_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Rising edge
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    // Add ISR handler
    return gpio_isr_handler_add(BMI270_INT1_PIN, bmi270_int1_isr_handler, NULL);
}

/**
 * @brief Configure BMI270 DATA_RDY interrupt on INT1
 */
static esp_err_t configure_data_ready_interrupt(void)
{
    esp_err_t ret;

    // Configure INT1 pin: active high, push-pull, output enabled
    uint8_t int1_io_ctrl = (1 << 1) | (1 << 3);  // bit1: output_en, bit3: active high
    ret = bmi270_write_register(&g_dev, BMI270_REG_INT1_IO_CTRL, int1_io_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT1 pin");
        return ret;
    }

    // Map DATA_RDY interrupt to INT1
    uint8_t int_map_data = (1 << 2);  // bit2: drdy_int -> INT1
    ret = bmi270_write_register(&g_dev, BMI270_REG_INT_MAP_DATA, int_map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map DATA_RDY interrupt");
        return ret;
    }

    ESP_LOGI(TAG, "DATA_RDY interrupt configured on INT1 (GPIO%d)", BMI270_INT1_PIN);
    return ESP_OK;
}

/**
 * @brief Data acquisition task (triggered by interrupt)
 */
static void data_acquisition_task(void *arg)
{
    ESP_LOGI(TAG, "Data acquisition task started (waiting for interrupts)");
    float temperature = 0.0f;

    while (1) {
        // Wait for DATA_RDY interrupt
        if (xSemaphoreTake(data_ready_sem, portMAX_DELAY) == pdTRUE) {
            bmi270_gyro_t gyro;
            bmi270_accel_t accel;

            // Read sensor data
            esp_err_t ret = bmi270_read_gyro_accel(&g_dev, &gyro, &accel);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read sensor data");
                continue;
            }

            g_sample_count++;

            // Read temperature less frequently (every 10 samples)
            if (g_sample_count % 10 == 0) {
                esp_err_t temp_ret = bmi270_read_temperature(&g_dev, &temperature);
                if (temp_ret == ESP_OK) {
                    // Print every 10 samples (1 second at 100Hz)
                    ESP_LOGI(TAG, "Sample #%lu:", g_sample_count);
                    ESP_LOGI(TAG, "  Gyro  [rad/s]: X=% 7.3f  Y=% 7.3f  Z=% 7.3f",
                             gyro.x, gyro.y, gyro.z);
                    ESP_LOGI(TAG, "  Accel [g]:     X=% 7.3f  Y=% 7.3f  Z=% 7.3f",
                             accel.x, accel.y, accel.z);
                    ESP_LOGI(TAG, "  Temp  [°C]:    % 7.2f", temperature);
                }
            }

            // Teleplot output
            printf(">gyr_x:%.3f\n", gyro.x);
            printf(">gyr_y:%.3f\n", gyro.y);
            printf(">gyr_z:%.3f\n", gyro.z);
            printf(">acc_x:%.3f\n", accel.x);
            printf(">acc_y:%.3f\n", accel.y);
            printf(">acc_z:%.3f\n", accel.z);
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Basic Interrupt Example");
    ESP_LOGI(TAG, "========================================");

    // Initialize SPI bus
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
    ESP_LOGI(TAG, "SPI initialized");

    // Initialize BMI270
    ESP_LOGI(TAG, "Step 2: Initializing BMI270 sensor...");
    ret = bmi270_init(&g_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI270");
        return;
    }
    ESP_LOGI(TAG, "BMI270 initialized (CHIP_ID: 0x%02X)", g_dev.chip_id);

    // Configure accelerometer: 100Hz, ±4g
    ESP_LOGI(TAG, "Step 3: Configuring sensors (100Hz, ±4g, ±1000°/s)...");
    ret = bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure accelerometer");
    }

    // Configure gyroscope: 100Hz, ±1000°/s (±17.45 rad/s)
    ret = bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_100HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure gyroscope");
    }

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure GPIO INT1
    ESP_LOGI(TAG, "Step 4: Configuring GPIO INT1 (GPIO%d)...", BMI270_INT1_PIN);
    ret = configure_int1_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT1 GPIO");
        return;
    }

    // Create semaphore
    ESP_LOGI(TAG, "Step 5: Creating semaphore...");
    data_ready_sem = xSemaphoreCreateBinary();
    if (data_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // Configure DATA_RDY interrupt
    ESP_LOGI(TAG, "Step 6: Configuring DATA_RDY interrupt...");
    ret = configure_data_ready_interrupt();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupt");
        return;
    }

    // Create data acquisition task
    ESP_LOGI(TAG, "Step 7: Creating data acquisition task...");
    xTaskCreate(data_acquisition_task, "data_acq", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Interrupt-driven data acquisition active");
    ESP_LOGI(TAG, " Waiting for DATA_RDY interrupts @ 100Hz");
    ESP_LOGI(TAG, "========================================");

    // Main task can now do other work or enter low-power mode
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Periodic heartbeat every 10s
        ESP_LOGI(TAG, "Running... (Total samples: %lu)", g_sample_count);
    }
}
