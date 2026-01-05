/**
 * @file main.c
 * @brief Stage 4: VL53L3CX Polling Distance Measurement
 *
 * This example demonstrates basic distance measurement using polling.
 * - Timing Budget: 33ms
 * - Distance Mode: MEDIUM
 * - Measurement Rate: 1 Hz (1 second interval)
 *
 * Hardware Setup:
 * - I2C SDA: GPIO3
 * - I2C SCL: GPIO4
 * - Bottom ToF XSHUT: GPIO7 (set HIGH to enable sensor) [DEFAULT]
 * - Front ToF XSHUT: GPIO9 (set LOW to disable sensor)
 *
 * Note: Bottom ToF works with USB power only. Front ToF requires battery.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stampfly_tof_config.h"
#include "vl53lx_platform.h"
#include "vl53lx_api.h"

static const char *TAG = "STAGE4_POLLING";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static VL53LX_Dev_t vl53lx_dev;

// Measurement configuration
#define TIMING_BUDGET_MS    33      // 33ms timing budget
#define MEASUREMENT_COUNT   20      // Number of measurements to perform

/**
 * @brief Initialize I2C master bus
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = STAMPFLY_I2C_PORT,
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);

    return ESP_OK;
}

/**
 * @brief Initialize XSHUT pins for ToF sensors
 */
static void tof_xshut_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_FRONT_XSHUT) |
                        (1ULL << STAMPFLY_TOF_BOTTOM_XSHUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Enable bottom sensor (USB powered), disable front sensor (requires battery)
    gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 0);
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);

    ESP_LOGI(TAG, "XSHUT pins initialized");
    ESP_LOGI(TAG, "Bottom ToF (GPIO%d): ENABLED [DEFAULT - USB powered]", STAMPFLY_TOF_BOTTOM_XSHUT);
    ESP_LOGI(TAG, "Front ToF (GPIO%d): DISABLED (requires battery)", STAMPFLY_TOF_FRONT_XSHUT);

    // Wait for sensor to boot
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief Initialize VL53LX device
 */
static VL53LX_Error initialize_sensor(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "Initializing VL53L3CX sensor...");

    // Wait for device boot
    status = VL53LX_WaitDeviceBooted(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Device boot failed (status: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "✓ Device booted");

    // Initialize device data
    status = VL53LX_DataInit(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Data init failed (status: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "✓ Data initialized");

    // Verify device info
    VL53LX_DeviceInfo_t device_info;
    status = VL53LX_GetDeviceInfo(&vl53lx_dev, &device_info);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Get device info failed (status: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "✓ Product Type: 0x%02X, Rev: %d.%d",
             device_info.ProductType,
             device_info.ProductRevisionMajor,
             device_info.ProductRevisionMinor);

    if (device_info.ProductType != 0xAA) {
        ESP_LOGW(TAG, "Warning: Not a VL53L3CX sensor (Type: 0x%02X)", device_info.ProductType);
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Configure measurement parameters
 */
static VL53LX_Error configure_measurement(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "Configuring measurement parameters...");

    // Set distance mode to MEDIUM (best for 1-3m range)
    status = VL53LX_SetDistanceMode(&vl53lx_dev, VL53LX_DISTANCEMODE_MEDIUM);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Set distance mode failed (status: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "✓ Distance mode: MEDIUM");

    // Set timing budget to 33ms (33000 microseconds)
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&vl53lx_dev, TIMING_BUDGET_MS * 1000);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Set timing budget failed (status: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "✓ Timing budget: %d ms", TIMING_BUDGET_MS);

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Perform distance measurements using polling
 */
static void perform_measurements(void)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t multi_ranging_data;
    uint8_t data_ready;
    uint32_t measurement_count = 0;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Starting distance measurements");
    ESP_LOGI(TAG, "Polling mode, %d measurements", MEASUREMENT_COUNT);
    ESP_LOGI(TAG, "==================================");

    // Start measurement
    status = VL53LX_StartMeasurement(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Start measurement failed (status: %d)", status);
        return;
    }

    // Polling loop
    while (measurement_count < MEASUREMENT_COUNT) {
        // Check if measurement data is ready
        status = VL53LX_GetMeasurementDataReady(&vl53lx_dev, &data_ready);
        if (status != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "GetMeasurementDataReady failed (status: %d)", status);
            return;
        }

        if (data_ready) {
            // Get multi-ranging measurement data
            status = VL53LX_GetMultiRangingData(&vl53lx_dev, &multi_ranging_data);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "Get multi-ranging data failed (status: %d)", status);
                return;
            }

            measurement_count++;

            // Display measurement results
            if (multi_ranging_data.NumberOfObjectsFound > 0) {
                VL53LX_TargetRangeData_t *target = &multi_ranging_data.RangeData[0];
                ESP_LOGI(TAG, "[%02d] Distance: %4d mm | Status: %d | Signal: %.2f Mcps",
                         (int)measurement_count,
                         (int)target->RangeMilliMeter,
                         (int)target->RangeStatus,
                         target->SignalRateRtnMegaCps / 65536.0);
            } else {
                ESP_LOGI(TAG, "[%02d] No objects detected",
                         (int)measurement_count);
            }

            // Clear interrupt and start next measurement
            status = VL53LX_ClearInterruptAndStartMeasurement(&vl53lx_dev);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "ClearInterruptAndStartMeasurement failed (status: %d)", status);
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms polling interval
    }

    // Stop measurement
    status = VL53LX_StopMeasurement(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Stop measurement failed (status: %d)", status);
    }

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Measurements complete!");
    ESP_LOGI(TAG, "==================================");
}

void app_main(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 4: Polling Distance Measurement");
    ESP_LOGI(TAG, "VL53L3CX ToF Sensor");
    ESP_LOGI(TAG, "==================================");

    // Initialize XSHUT pins
    tof_xshut_init();

    // Initialize I2C bus
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }

    // Initialize VL53LX platform layer
    status = VL53LX_PlatformInit(&vl53lx_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Platform init failed (status: %d)", status);
        return;
    }

    // Initialize sensor
    status = initialize_sensor();
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Sensor initialization failed!");
        VL53LX_PlatformDeinit(&vl53lx_dev);
        return;
    }

    // Configure measurement parameters
    // TEMPORARILY DISABLED: Testing with default settings like ST sample
    // status = configure_measurement();
    // if (status != VL53LX_ERROR_NONE) {
    //     ESP_LOGE(TAG, "Measurement configuration failed!");
    //     VL53LX_PlatformDeinit(&vl53lx_dev);
    //     return;
    // }
    ESP_LOGI(TAG, "Using default measurement parameters (no configuration)");

    // Perform measurements
    perform_measurements();

    // Cleanup
    VL53LX_PlatformDeinit(&vl53lx_dev);

    ESP_LOGI(TAG, "Test completed. Ready for Stage 5 (Interrupt-based measurement).");
}
