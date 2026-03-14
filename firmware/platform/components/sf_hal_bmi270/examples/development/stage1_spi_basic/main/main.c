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
 * @brief Stage 1: BMI270 SPI Basic Communication Test
 *
 * This example demonstrates:
 * - SPI initialization with ESP-IDF
 * - 3-byte read transaction implementation
 * - CHIP_ID verification
 * - Communication stability test
 *
 * Expected output:
 * - CHIP_ID should read 0x24
 * - 100% success rate over 10 consecutive reads
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

static const char *TAG = "BMI270_TEST";

// Forward declarations
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config);
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);
void bmi270_set_lowpower_delay_override(uint32_t delay_us);

/**
 * @brief Test CHIP_ID reading
 *
 * @param dev Pointer to BMI270 device
 * @return esp_err_t ESP_OK if CHIP_ID is correct
 */
static esp_err_t test_chip_id(bmi270_dev_t *dev) {
    uint8_t chip_id = 0;
    esp_err_t ret;

    ESP_LOGI(TAG, "Reading CHIP_ID...");

    ret = bmi270_read_register(dev, BMI270_REG_CHIP_ID, &chip_id);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CHIP_ID = 0x%02X (expected: 0x%02X)", chip_id, BMI270_CHIP_ID);

    if (chip_id == BMI270_CHIP_ID) {
        ESP_LOGI(TAG, "✓ CHIP_ID verification SUCCESS");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "✗ CHIP_ID verification FAILED");
        return ESP_FAIL;
    }
}

/**
 * @brief Test communication stability
 *
 * @param dev Pointer to BMI270 device
 * @param iterations Number of test iterations
 * @return esp_err_t ESP_OK if all reads successful
 */
static esp_err_t test_communication_stability(bmi270_dev_t *dev, int iterations) {
    int success_count = 0;
    int fail_count = 0;
    uint8_t chip_id;
    esp_err_t ret;

    ESP_LOGI(TAG, "Testing communication stability (%d iterations)...", iterations);

    for (int i = 0; i < iterations; i++) {
        ret = bmi270_read_register(dev, BMI270_REG_CHIP_ID, &chip_id);

        if (ret == ESP_OK && chip_id == BMI270_CHIP_ID) {
            success_count++;
        } else {
            fail_count++;
            ESP_LOGW(TAG, "Iteration %d: FAIL (ret=%s, chip_id=0x%02X)",
                     i + 1, esp_err_to_name(ret), chip_id);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay between reads
    }

    float success_rate = (float)success_count / iterations * 100.0f;

    ESP_LOGI(TAG, "Communication test results:");
    ESP_LOGI(TAG, "  Success: %d/%d (%.1f%%)", success_count, iterations, success_rate);
    ESP_LOGI(TAG, "  Failed:  %d/%d", fail_count, iterations);

    if (success_rate == 100.0f) {
        ESP_LOGI(TAG, "✓ Communication stability test PASSED");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "✗ Communication stability test FAILED");
        return ESP_FAIL;
    }
}

/**
 * @brief Test delay optimization
 *
 * Tests various delay times to find the minimum reliable delay.
 *
 * @param dev Pointer to BMI270 device
 * @param delay_us Delay time to test in microseconds
 * @param iterations Number of test iterations
 * @return Success rate (0-100)
 */
static float test_delay_optimization(bmi270_dev_t *dev, uint32_t delay_us, int iterations) {
    int success_count = 0;
    int fail_count = 0;
    uint8_t chip_id;
    esp_err_t ret;

    // Set the delay override
    bmi270_set_lowpower_delay_override(delay_us);

    for (int i = 0; i < iterations; i++) {
        ret = bmi270_read_register(dev, BMI270_REG_CHIP_ID, &chip_id);

        if (ret == ESP_OK && chip_id == BMI270_CHIP_ID) {
            success_count++;
        } else {
            fail_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay between reads
    }

    // Clear override
    bmi270_set_lowpower_delay_override(0);

    float success_rate = (float)success_count / iterations * 100.0f;
    return success_rate;
}

/**
 * @brief Perform dummy read to switch BMI270 to SPI mode
 *
 * The BMI270 powers up in I2C mode by default. A dummy SPI read
 * triggers the sensor to switch to SPI mode.
 *
 * IMPORTANT: Before initialization, BMI270 is in low-power mode,
 * requiring 450µs (5ms for safety) wait after each register access.
 *
 * @param dev Pointer to BMI270 device
 */
static void perform_dummy_read(bmi270_dev_t *dev) {
    uint8_t dummy;

    ESP_LOGI(TAG, "Performing dummy read to activate SPI mode...");

    // First read may fail as sensor switches modes
    bmi270_read_register(dev, BMI270_REG_CHIP_ID, &dummy);

    // Wait for sensor to stabilize in low-power mode (5ms)
    ESP_LOGI(TAG, "Waiting 5ms for sensor stabilization...");
    vTaskDelay(pdMS_TO_TICKS(5));

    // Second read should succeed
    bmi270_read_register(dev, BMI270_REG_CHIP_ID, &dummy);

    ESP_LOGI(TAG, "SPI mode activated");
}

void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " BMI270 Stage 1: SPI Basic Communication");
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

    // Wait for BMI270 power-on (450µs min, 5ms for safety)
    ESP_LOGI(TAG, "Waiting 5ms for BMI270 power-on...");
    vTaskDelay(pdMS_TO_TICKS(5));

    // Step 2: Perform dummy read to activate SPI mode
    ESP_LOGI(TAG, "Step 2: Activating SPI mode...");
    perform_dummy_read(&dev);
    ESP_LOGI(TAG, "");

    // Step 3: Test CHIP_ID
    ESP_LOGI(TAG, "Step 3: Testing CHIP_ID...");
    ret = test_chip_id(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CHIP_ID test failed!");
        return;
    }
    ESP_LOGI(TAG, "");

    // Step 4: Test communication stability
    ESP_LOGI(TAG, "Step 4: Testing communication stability...");
    ret = test_communication_stability(&dev, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stability test failed!");
        return;
    }
    ESP_LOGI(TAG, "");

    // All tests passed
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ✓ ALL TESTS PASSED");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Stage 1 completed successfully!");
    ESP_LOGI(TAG, "BMI270 SPI communication is working correctly.");
    ESP_LOGI(TAG, "");

    // Step 5: Delay optimization experiment
    ESP_LOGI(TAG, "Step 5: Delay optimization experiment...");
    ESP_LOGI(TAG, "Testing various delay times to find optimal value");
    ESP_LOGI(TAG, "");

    // Define test delays (in microseconds)
    uint32_t test_delays[] = {
        50,    // 50µs
        100,   // 100µs
        200,   // 200µs
        450,   // 450µs (datasheet minimum)
        500,   // 500µs
        1000,  // 1ms
        2000,  // 2ms
        5000,  // 5ms (current default)
    };
    int num_delays = sizeof(test_delays) / sizeof(test_delays[0]);
    int iterations_per_test = 100;

    ESP_LOGI(TAG, "┌──────────┬──────────┬──────────┐");
    ESP_LOGI(TAG, "│ Delay    │ Success  │ Status   │");
    ESP_LOGI(TAG, "├──────────┼──────────┼──────────┤");

    for (int i = 0; i < num_delays; i++) {
        uint32_t delay = test_delays[i];

        ESP_LOGI(TAG, "Testing %lu µs...", delay);
        float success_rate = test_delay_optimization(&dev, delay, iterations_per_test);

        const char *status = (success_rate == 100.0f) ? "PASS ✓" : "FAIL ✗";
        ESP_LOGI(TAG, "│ %5lu µs │ %6.1f%% │ %8s │", delay, success_rate, status);
    }

    ESP_LOGI(TAG, "└──────────┴──────────┴──────────┘");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Delay optimization experiment completed!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Next step: Stage 2 - Initialization sequence");
}
