/**
 * @file main.c
 * @brief Stage 3: VL53L3CX Device Initialization Test
 *
 * This example tests VL53LX API device initialization sequence:
 * 1. VL53LX_WaitDeviceBooted() - Wait for sensor boot
 * 2. VL53LX_DataInit() - Initialize device data structures
 * 3. VL53LX_GetDeviceInfo() - Get device information
 *
 * Expected results:
 * - Device boots successfully
 * - DataInit completes without errors
 * - Device info shows VL53L3CX product type
 *
 * Hardware Setup:
 * - I2C SDA: GPIO3
 * - I2C SCL: GPIO4
 * - Front ToF XSHUT: GPIO9 (set HIGH to enable sensor)
 * - Bottom ToF XSHUT: GPIO7 (set LOW to disable sensor)
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

static const char *TAG = "STAGE3_DEV_INIT";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static VL53LX_Dev_t vl53lx_dev;

/**
 * @brief Initialize I2C master bus
 */
static esp_err_t i2c_master_init(void)
{
    // I2C master bus configuration
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
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d",
             STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);

    return ESP_OK;
}

/**
 * @brief Initialize XSHUT pins for ToF sensors
 */
static void tof_xshut_init(void)
{
    // Configure XSHUT pins as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_FRONT_XSHUT) |
                        (1ULL << STAMPFLY_TOF_BOTTOM_XSHUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Enable front sensor, disable bottom sensor
    gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 1);
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 0);

    ESP_LOGI(TAG, "XSHUT pins initialized");
    ESP_LOGI(TAG, "Front ToF (GPIO%d): ENABLED", STAMPFLY_TOF_FRONT_XSHUT);
    ESP_LOGI(TAG, "Bottom ToF (GPIO%d): DISABLED", STAMPFLY_TOF_BOTTOM_XSHUT);

    // Wait for sensor to boot (minimum 1.2ms, recommended 2-5ms)
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief Test VL53LX device initialization sequence
 */
static void test_device_initialization(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "VL53LX API Initialization Sequence");
    ESP_LOGI(TAG, "==================================");

    // Step 1: Wait for device to boot
    ESP_LOGI(TAG, "Step 1: Waiting for device boot...");
    status = VL53LX_WaitDeviceBooted(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Device boot failed (status: %d)", status);
        return;
    }
    ESP_LOGI(TAG, "✓ Device booted successfully");

    // Step 2: Initialize device data structures
    ESP_LOGI(TAG, "Step 2: Initializing device data...");
    status = VL53LX_DataInit(&vl53lx_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Data initialization failed (status: %d)", status);
        return;
    }
    ESP_LOGI(TAG, "✓ Device data initialized successfully");

    // Step 3: Get device information
    ESP_LOGI(TAG, "Step 3: Reading device information...");
    VL53LX_DeviceInfo_t device_info;
    status = VL53LX_GetDeviceInfo(&vl53lx_dev, &device_info);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to get device info (status: %d)", status);
        return;
    }

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Device Information:");
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Product Type    : 0x%02X", device_info.ProductType);
    ESP_LOGI(TAG, "Product Revision: %d.%d",
             device_info.ProductRevisionMajor,
             device_info.ProductRevisionMinor);
    ESP_LOGI(TAG, "==================================");

    // Verify Product Type (Module Type in datasheet)
    if (device_info.ProductType == 0xAA) {
        ESP_LOGI(TAG, "✓ VL53L3CX device confirmed (Product Type: 0xAA)");
    } else if (device_info.ProductType == 0xCC) {
        ESP_LOGW(TAG, "✗ VL53L1 device detected (Product Type: 0xCC)");
        ESP_LOGW(TAG, "  This is not a VL53L3CX sensor!");
    } else {
        ESP_LOGW(TAG, "? Unknown product type: 0x%02X", device_info.ProductType);
    }

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "✓ Device initialization complete!");
    ESP_LOGI(TAG, "==================================");
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 3: Device Initialization");
    ESP_LOGI(TAG, "VL53LX API Test");
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
    VL53LX_Error status = VL53LX_PlatformInit(&vl53lx_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX platform init failed (status: %d)", status);
        return;
    }

    // Test device initialization
    test_device_initialization();

    // Cleanup
    VL53LX_PlatformDeinit(&vl53lx_dev);

    ESP_LOGI(TAG, "Test completed. Ready for Stage 4 (Distance Measurement).");
}
