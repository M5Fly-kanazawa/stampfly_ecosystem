/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 StampFly ToF Driver Contributors
 */

/**
 * @file main.c
 * @brief Basic VL53L3CX Interrupt Measurement Example
 *
 * Simple distance measurement using interrupt mode with bottom ToF sensor.
 * Measures distance 10 times using GPIO interrupts for efficient data acquisition.
 *
 * Hardware:
 * - Bottom ToF sensor (USB powered)
 * - I2C: SDA=GPIO3, SCL=GPIO4
 * - XSHUT: GPIO7 (bottom), GPIO9 (front - disabled)
 * - INT: GPIO6 (bottom sensor interrupt pin)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stampfly_tof_config.h"
#include "vl53lx_platform.h"
#include "vl53lx_api.h"

static const char *TAG = "BASIC_INTERRUPT";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static VL53LX_Dev_t tof_dev;
static SemaphoreHandle_t semaphore = NULL;

#define MEASUREMENT_COUNT   10      // Number of measurements

/**
 * @brief GPIO interrupt handler (called when sensor data is ready)
 */
static void IRAM_ATTR int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Initialize I2C bus
 */
static esp_err_t init_i2c(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = STAMPFLY_I2C_PORT,
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return err;
    }

    ESP_LOGI(TAG, "I2C initialized (SDA: GPIO%d, SCL: GPIO%d)",
             STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);
    return ESP_OK;
}

/**
 * @brief Initialize ToF sensor power (XSHUT pins)
 */
static void init_sensor_power(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_FRONT_XSHUT) |
                        (1ULL << STAMPFLY_TOF_BOTTOM_XSHUT),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // Enable bottom sensor only (USB powered)
    gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 0);   // Front: OFF
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);  // Bottom: ON

    ESP_LOGI(TAG, "Sensor power initialized (bottom sensor enabled)");
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for sensor boot
}

/**
 * @brief Initialize interrupt pin
 */
static esp_err_t init_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_BOTTOM_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // INT is active LOW
        .intr_type = GPIO_INTR_NEGEDGE,    // Trigger on falling edge
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Add ISR handler
    gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, int_isr_handler, NULL);

    ESP_LOGI(TAG, "Interrupt initialized (GPIO%d)", STAMPFLY_TOF_BOTTOM_INT);
    return ESP_OK;
}

/**
 * @brief Initialize VL53LX sensor
 */
static VL53LX_Error init_sensor(void)
{
    VL53LX_Error status;

    // Wait for device boot
    status = VL53LX_WaitDeviceBooted(&tof_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Device boot failed");
        return status;
    }

    // Initialize device
    status = VL53LX_DataInit(&tof_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Data init failed");
        return status;
    }

    // Verify device
    VL53LX_DeviceInfo_t info;
    status = VL53LX_GetDeviceInfo(&tof_dev, &info);
    if (status == VL53LX_ERROR_NONE) {
        ESP_LOGI(TAG, "VL53L3CX ready (Type: 0x%02X, Rev: %d.%d)",
                 info.ProductType, info.ProductRevisionMajor, info.ProductRevisionMinor);
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Main application
 */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Basic VL53L3CX Interrupt Example ===");

    // Create semaphore
    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // Initialize hardware
    if (init_i2c() != ESP_OK) {
        return;
    }
    init_sensor_power();
    init_interrupt();

    // Setup VL53LX device structure
    tof_dev.I2cDevAddr = 0x29;  // Default I2C address
    if (VL53LX_platform_init(&tof_dev, i2c_bus_handle) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Platform init failed");
        return;
    }

    // Initialize sensor
    if (init_sensor() != VL53LX_ERROR_NONE) {
        return;
    }

    // Configure measurement (Medium distance mode, 33ms timing)
    VL53LX_SetDistanceMode(&tof_dev, VL53LX_DISTANCEMODE_MEDIUM);
    VL53LX_SetMeasurementTimingBudgetMicroSeconds(&tof_dev, 33000);

    ESP_LOGI(TAG, "Starting interrupt-based measurements...\n");

    // Start measurement
    VL53LX_StartMeasurement(&tof_dev);

    // Perform measurements
    for (int i = 0; i < MEASUREMENT_COUNT; i++) {
        // Wait for interrupt (data ready)
        if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            VL53LX_MultiRangingData_t data;

            // Get measurement data
            if (VL53LX_GetMultiRangingData(&tof_dev, &data) == VL53LX_ERROR_NONE) {
                uint16_t distance = data.RangeData[0].RangeMilliMeter;
                uint8_t status = data.RangeData[0].RangeStatus;
                float signal = data.RangeData[0].SignalRateRtnMegaCps / 65536.0;

                printf("[%d] Distance: %4d mm, Status: %d, Signal: %.2f Mcps\n",
                       i + 1, distance, status, signal);
            }

            // Clear interrupt and restart measurement
            VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);
        } else {
            ESP_LOGW(TAG, "Timeout waiting for measurement");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second interval
    }

    // Stop measurement
    VL53LX_StopMeasurement(&tof_dev);
    ESP_LOGI(TAG, "\nMeasurements complete!");
}
