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
 * @brief BMI270 Basic FIFO Example
 *
 * This example demonstrates:
 * - FIFO watermark configuration (416 bytes = 32 frames for 50Hz output)
 * - Interrupt-driven FIFO read using INT1 (GPIO11)
 * - Efficient data acquisition without polling (1600Hz ODR)
 * - Averaged sensor data output
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmi270_spi.h"
#include "bmi270_init.h"
#include "bmi270_data.h"

static const char *TAG = "BMI270_BASIC_FIFO";

// M5StampFly BMI270 pin configuration
#define BMI270_MOSI_PIN     14
#define BMI270_MISO_PIN     43
#define BMI270_SCLK_PIN     44
#define BMI270_CS_PIN       46
#define BMI270_INT1_PIN     11        // INT1 interrupt pin
#define BMI270_SPI_CLOCK_HZ 10000000  // 10 MHz
#define PMW3901_CS_PIN      12        // Other device on shared SPI bus

// FIFO registers
#define BMI270_REG_FIFO_LENGTH_0    0x24    // FIFO length LSB
#define BMI270_REG_FIFO_LENGTH_1    0x25    // FIFO length MSB
#define BMI270_REG_FIFO_DATA        0x26    // FIFO data read
#define BMI270_REG_FIFO_WTM_0       0x46    // FIFO watermark LSB
#define BMI270_REG_FIFO_WTM_1       0x47    // FIFO watermark MSB
#define BMI270_REG_FIFO_CONFIG_0    0x48    // FIFO mode config
#define BMI270_REG_FIFO_CONFIG_1    0x49    // FIFO sensor enable
#define BMI270_REG_INT1_IO_CTRL     0x53    // INT1 pin config
#define BMI270_REG_INT_MAP_DATA     0x58    // Interrupt mapping
#define BMI270_REG_CMD              0x7E    // Command register

// FIFO constants
#define FIFO_FRAME_SIZE_HEADER      13      // Header(1) + GYR(6) + ACC(6)
#define FIFO_HEADER_ACC_GYR         0x8C    // Expected header for ACC+GYR frame
#define FIFO_HEADER_SKIP            0x40    // Skip frame (data loss)
#define FIFO_HEADER_CONFIG          0x48    // Config change frame
#define FIFO_MAX_SIZE               2048    // Maximum FIFO size
#define FIFO_WATERMARK_BYTES        416     // Watermark: 32 frames = 416 bytes (50Hz output @ 1600Hz ODR)

// FIFO commands
#define BMI270_CMD_FIFO_FLUSH       0xB0    // FIFO flush command

// Global device handle
static bmi270_dev_t g_dev = {0};

// Statistics
static uint32_t g_total_frames = 0;
static uint32_t g_valid_frames = 0;
static uint32_t g_skip_frames = 0;
static uint32_t g_config_frames = 0;
static uint32_t g_interrupt_count = 0;

// Output decimation (reduce printf frequency)
#define OUTPUT_DECIMATION 1  // Output every Nth interrupt (1 = every interrupt = 50Hz)
static uint32_t g_output_counter = 0;

// Teleplot output enable/disable flag
static volatile bool g_teleplot_enabled = true;  // Default: enabled

// Semaphore for interrupt notification
static SemaphoreHandle_t fifo_semaphore = NULL;


/**
 * @brief INT1 interrupt handler
 */
static void IRAM_ATTR bmi270_int1_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(fifo_semaphore, &xHigherPriorityTaskWoken);
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
        .intr_type = GPIO_INTR_POSEDGE  // Rising edge (INT1 is active high)
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already installed
        return ret;
    }

    // Add ISR handler
    return gpio_isr_handler_add(BMI270_INT1_PIN, bmi270_int1_isr_handler, NULL);
}


/**
 * @brief Flush FIFO (clear all data)
 */
static esp_err_t fifo_flush(void)
{
    esp_err_t ret = bmi270_write_register(&g_dev, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "FIFO flushed");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ret;
}

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
        g_skip_frames++;
        return ESP_FAIL;
    }

    if (header != FIFO_HEADER_ACC_GYR) {
        // Unknown header - frame sync error, will be recovered by FIFO flush
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
 * @param buffer FIFO data buffer
 * @param length Length of buffer in bytes
 * @param output_enabled If true, output Teleplot data; if false, skip output
 * @return true if skip frame detected, false otherwise
 */
static bool parse_fifo_buffer(const uint8_t *buffer, uint16_t length, bool output_enabled)
{
    int num_frames = length / FIFO_FRAME_SIZE_HEADER;
    int valid_count = 0;
    bool skip_detected = false;

    ESP_LOGD(TAG, "Parsing %d frames (%u bytes)", num_frames, length);

    // Accumulators for averaging
    double sum_gyr_x = 0.0, sum_gyr_y = 0.0, sum_gyr_z = 0.0;
    double sum_acc_x = 0.0, sum_acc_y = 0.0, sum_acc_z = 0.0;

    // Parse all frames and accumulate values
    for (int i = 0; i < num_frames; i++) {
        const uint8_t *frame = &buffer[i * FIFO_FRAME_SIZE_HEADER];
        g_total_frames++;

        // Check for skip frame
        if (frame[0] == FIFO_HEADER_SKIP) {
            skip_detected = true;
            g_skip_frames++;
            continue;
        }

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

    ESP_LOGD(TAG, "Valid frames: %d/%d", valid_count, num_frames);

    // Calculate average and output (only if we have valid data AND output is enabled)
    if (valid_count > 0 && output_enabled) {
        double avg_gyr_x = sum_gyr_x / valid_count;
        double avg_gyr_y = sum_gyr_y / valid_count;
        double avg_gyr_z = sum_gyr_z / valid_count;
        double avg_acc_x = sum_acc_x / valid_count;
        double avg_acc_y = sum_acc_y / valid_count;
        double avg_acc_z = sum_acc_z / valid_count;

        // Teleplot output format (averaged data)
        printf(">gyr_x:%.3f\n", avg_gyr_x);
        printf(">gyr_y:%.3f\n", avg_gyr_y);
        printf(">gyr_z:%.3f\n", avg_gyr_z);
        printf(">acc_x:%.3f\n", avg_acc_x);
        printf(">acc_y:%.3f\n", avg_acc_y);
        printf(">acc_z:%.3f\n", avg_acc_z);
    }

    return skip_detected;
}

/**
 * @brief FIFO read task (triggered by interrupt)
 */
static void fifo_read_task(void *arg)
{
    static uint8_t fifo_buffer[FIFO_MAX_SIZE];  // Static to avoid stack overflow

    ESP_LOGD(TAG, "FIFO read task started (waiting for interrupts)");

    while (1) {
        // Wait for interrupt notification
        if (xSemaphoreTake(fifo_semaphore, portMAX_DELAY) == pdTRUE) {
            g_interrupt_count++;

            // Read FIFO length
            uint16_t fifo_length;
            esp_err_t ret = read_fifo_length(&fifo_length);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read FIFO length");
                continue;
            }

            // Check if we have at least one frame
            if (fifo_length >= FIFO_FRAME_SIZE_HEADER) {
                // Read all FIFO data
                ret = read_fifo_data(fifo_buffer, fifo_length);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to read FIFO data");
                    continue;
                }

                // Determine if we should output Teleplot data this time (decimation + enable flag)
                g_output_counter++;
                bool output_enabled = g_teleplot_enabled && (g_output_counter % OUTPUT_DECIMATION == 0);

                // Parse all frames in buffer
                bool skip_detected = parse_fifo_buffer(fifo_buffer, fifo_length, output_enabled);

                // If skip frame detected, flush FIFO to recover
                if (skip_detected) {
                    ret = fifo_flush();
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to flush FIFO");
                    }
                }
            }

            // Clear latched interrupt by reading INT_STATUS_1 register (0x1D)
            uint8_t int_status;
            bmi270_read_register(&g_dev, 0x1D, &int_status);
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Set log levels
    esp_log_level_set("*", ESP_LOG_INFO);        // Global: INFO and above
    esp_log_level_set(TAG, ESP_LOG_INFO);        // This module: INFO and above

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Step 4: FIFO Watermark Interrupt");
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

    // Step 3: Set accelerometer to 1600Hz, ±4g range
    ESP_LOGI(TAG, "Step 3: Configuring accelerometer (1600Hz, ±4g)...");
    ret = bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_1600HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set accelerometer config");
    }

    // Step 4: Set gyroscope to 1600Hz, ±1000°/s (±17.45 rad/s) range
    ESP_LOGI(TAG, "Step 4: Configuring gyroscope (1600Hz, ±1000°/s)...");
    ret = bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_1600HZ, BMI270_FILTER_PERFORMANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gyroscope config");
    }

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
    ESP_LOGD(TAG, "FIFO_CONFIG_1 readback: 0x%02X (expected 0xD0)", fifo_config_1_readback);

    // Step 6: Configure FIFO watermark (but NOT interrupt mapping yet)
    ESP_LOGI(TAG, "Step 6: Configuring FIFO watermark...");

    // Set FIFO watermark only
    uint16_t watermark = FIFO_WATERMARK_BYTES;
    uint8_t wtm_lsb = watermark & 0xFF;
    uint8_t wtm_msb = (watermark >> 8) & 0x07;

    ret = bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_0, wtm_lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_WTM_0");
        return;
    }

    ret = bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_1, wtm_msb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_WTM_1");
        return;
    }

    ESP_LOGD(TAG, "FIFO watermark set to %u bytes (%u frames)", watermark, watermark / FIFO_FRAME_SIZE_HEADER);

    // Configure INT1 pin (but interrupt not mapped yet, so no interrupt will fire)
    uint8_t int1_io_ctrl = (1 << 0) | (1 << 1) | (1 << 3);  // bit 0: latch, bit 1: output_en, bit 3: active high
    ret = bmi270_write_register(&g_dev, BMI270_REG_INT1_IO_CTRL, int1_io_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write INT1_IO_CTRL");
        return;
    }
    ESP_LOGD(TAG, "INT1 pin configured (not mapped yet)");

    // Step 7: Configure GPIO INT1
    ESP_LOGI(TAG, "Step 7: Configuring GPIO INT1 (GPIO%d)...", BMI270_INT1_PIN);
    ret = configure_int1_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT1 GPIO");
        return;
    }
    ESP_LOGI(TAG, "GPIO INT1 configured successfully");

    // Step 8: Flush FIFO before enabling interrupt
    ESP_LOGI(TAG, "Step 8: Flushing FIFO before enabling interrupt...");
    ret = fifo_flush();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to flush FIFO");
    }

    vTaskDelay(pdMS_TO_TICKS(10));


    // Step 9: Create semaphore for interrupt notification (BEFORE mapping interrupt)
    ESP_LOGI(TAG, "Step 9: Creating semaphore for interrupt notification...");
    fifo_semaphore = xSemaphoreCreateBinary();
    if (fifo_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // Step 10: NOW map FIFO watermark interrupt to INT1
    ESP_LOGI(TAG, "Step 10: Mapping FIFO watermark interrupt to INT1...");
    uint8_t int_map_data = (1 << 1);  // bit 1: fwm_int -> INT1
    ret = bmi270_write_register(&g_dev, BMI270_REG_INT_MAP_DATA, int_map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write INT_MAP_DATA");
        return;
    }
    ESP_LOGD(TAG, "FIFO watermark interrupt mapped to INT1");

    // Step 11: Create FIFO read task
    ESP_LOGI(TAG, "Step 11: Creating FIFO read task...");
    xTaskCreate(fifo_read_task, "fifo_read", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Interrupt-driven FIFO read active");
    ESP_LOGI(TAG, " Watermark: %u bytes (%u frames)", FIFO_WATERMARK_BYTES, FIFO_WATERMARK_BYTES / FIFO_FRAME_SIZE_HEADER);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Press 't' or 'o' to toggle Teleplot output ON/OFF");
    ESP_LOGI(TAG, "Teleplot output: %s", g_teleplot_enabled ? "ENABLED" : "DISABLED");

    // Main task: Monitor UART for commands and periodic status
    uint32_t last_status_time = 0;
    while (1) {
        // Check for UART input (non-blocking)
        int c = getchar();
        if (c != EOF) {
            if (c == 't' || c == 'T' || c == 'o' || c == 'O') {
                // Toggle Teleplot output
                g_teleplot_enabled = !g_teleplot_enabled;
                ESP_LOGI(TAG, "*** Teleplot output: %s ***", g_teleplot_enabled ? "ENABLED" : "DISABLED");
            }
        }

        // Periodic status update (every 10 seconds)
        uint32_t current_time = esp_timer_get_time() / 1000000;  // Convert to seconds
        if (current_time - last_status_time >= 10) {
            last_status_time = current_time;
            ESP_LOGD(TAG, "Status: INT=%lu, Total=%lu, Valid=%lu, Skip=%lu, Output=%s",
                     g_interrupt_count, g_total_frames, g_valid_frames, g_skip_frames,
                     g_teleplot_enabled ? "ON" : "OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}
