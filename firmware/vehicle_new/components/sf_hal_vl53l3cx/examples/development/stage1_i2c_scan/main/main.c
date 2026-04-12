/**
 * @file main.c
 * @brief Stage 1: I2C Bus Scan - VL53L3CX Device Detection
 *
 * This example scans the I2C bus to detect VL53L3CX ToF sensors.
 * Expected: Device found at address 0x29 (VL53L3CX default 7-bit address)
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

static const char *TAG = "I2C_SCAN";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;

/**
 * @brief Initialize I2C master
 */
static esp_err_t i2c_master_init(void)
{
    // I2C master bus configuration (all fields explicitly initialized)
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = STAMPFLY_I2C_PORT,
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,        // Auto-select interrupt priority
        .trans_queue_depth = 0,    // No queue needed for synchronous operations
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

    // Wait for sensor to boot
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief Scan I2C bus for devices
 */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    ESP_LOGI(TAG, "Scanning address range: 0x03 to 0x77");

    int devices_found = 0;

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        // Probe the device directly
        // Note: timeout parameter is in milliseconds (not ticks)
        // i2c_master_probe() internally calls pdMS_TO_TICKS()
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, 50);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
            devices_found++;

            // Check if it's the expected VL53L3CX address
            if (addr == VL53L3CX_DEFAULT_I2C_ADDR) {
                ESP_LOGI(TAG, "  -> VL53L3CX detected at default address!");
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout indicates bus busy or pull-up resistor issues
            ESP_LOGW(TAG, "Timeout at address 0x%02X - check pull-ups", addr);
            break;  // Stop scanning if timeout occurs
        }
        // ESP_ERR_NOT_FOUND (NACK) is normal and silent
    }

    ESP_LOGI(TAG, "I2C scan completed. Devices found: %d", devices_found);

    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Please check:");
        ESP_LOGW(TAG, "  - I2C wiring (SDA=GPIO%d, SCL=GPIO%d)",
                 STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);
        ESP_LOGW(TAG, "  - Pull-up resistors (2-5kΩ recommended)");
        ESP_LOGW(TAG, "  - Sensor power supply");
        ESP_LOGW(TAG, "  - XSHUT pin levels (Front=GPIO%d, Bottom=GPIO%d)",
                 STAMPFLY_TOF_FRONT_XSHUT, STAMPFLY_TOF_BOTTOM_XSHUT);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 1: I2C Bus Scan");
    ESP_LOGI(TAG, "VL53L3CX Device Detection Test");
    ESP_LOGI(TAG, "==================================");

    // Initialize XSHUT pins
    tof_xshut_init();

    // Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }

    // Scan I2C bus
    i2c_scan();

    ESP_LOGI(TAG, "Test completed. You can now flash Stage 2.");
}
