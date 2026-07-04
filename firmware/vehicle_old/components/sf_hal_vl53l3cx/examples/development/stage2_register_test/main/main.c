/**
 * @file main.c
 * @brief Stage 2: VL53L3CX Register Read/Write Test
 *
 * This example tests direct register access to VL53L3CX ToF sensor
 * using the VL53LX platform layer.
 *
 * Expected results:
 * - Model ID (0x010F): 0xEA
 * - Module Type (0x0110): 0xCC
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

static const char *TAG = "STAGE2_REG_TEST";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static VL53LX_Dev_t vl53lx_dev;

// VL53L3CX Register Addresses
#define VL53L3CX_REG_MODEL_ID           0x010F
#define VL53L3CX_REG_MODULE_TYPE        0x0110
#define VL53L3CX_REG_MASK_REVISION      0x0111

// Expected values
#define VL53L3CX_MODEL_ID_EXPECTED      0xEA
#define VL53L3CX_MODULE_TYPE_EXPECTED   0xAA  // VL53L3CX (Note: 0xCC is VL53L1)

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

    // Wait for sensor to boot
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief Test VL53L3CX register access
 */
static void test_register_access(void)
{
    uint8_t model_id = 0;
    uint8_t module_type = 0;
    uint8_t mask_revision = 0;
    int8_t status;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Reading VL53L3CX Identification");
    ESP_LOGI(TAG, "==================================");

    // Read Model ID
    status = VL53LX_ReadByte(&vl53lx_dev, VL53L3CX_REG_MODEL_ID, &model_id);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to read Model ID (status: %d)", status);
    } else {
        ESP_LOGI(TAG, "Model ID (0x%04X): 0x%02X %s",
                 VL53L3CX_REG_MODEL_ID, model_id,
                 (model_id == VL53L3CX_MODEL_ID_EXPECTED) ? "[OK]" : "[MISMATCH!]");
    }

    // Read Module Type
    status = VL53LX_ReadByte(&vl53lx_dev, VL53L3CX_REG_MODULE_TYPE, &module_type);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to read Module Type (status: %d)", status);
    } else {
        ESP_LOGI(TAG, "Module Type (0x%04X): 0x%02X %s",
                 VL53L3CX_REG_MODULE_TYPE, module_type,
                 (module_type == VL53L3CX_MODULE_TYPE_EXPECTED) ? "[OK]" :
                 (module_type == 0xCC) ? "[VL53L1 device!]" : "[UNKNOWN!]");
    }

    // Read Mask Revision (informational)
    status = VL53LX_ReadByte(&vl53lx_dev, VL53L3CX_REG_MASK_REVISION, &mask_revision);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to read Mask Revision (status: %d)", status);
    } else {
        ESP_LOGI(TAG, "Mask Revision (0x%04X): 0x%02X",
                 VL53L3CX_REG_MASK_REVISION, mask_revision);
    }

    ESP_LOGI(TAG, "==================================");

    // Verify results
    bool model_id_ok = (model_id == VL53L3CX_MODEL_ID_EXPECTED);
    bool module_type_ok = (module_type == VL53L3CX_MODULE_TYPE_EXPECTED);

    if (model_id_ok && module_type_ok) {
        ESP_LOGI(TAG, "✓ VL53L3CX identification successful!");
        ESP_LOGI(TAG, "  Platform layer is working correctly.");
    } else {
        ESP_LOGW(TAG, "✗ VL53L3CX identification failed!");
        if (!model_id_ok) {
            ESP_LOGW(TAG, "  Expected Model ID: 0x%02X, got: 0x%02X",
                     VL53L3CX_MODEL_ID_EXPECTED, model_id);
        }
        if (!module_type_ok) {
            ESP_LOGW(TAG, "  Expected Module Type: 0x%02X, got: 0x%02X",
                     VL53L3CX_MODULE_TYPE_EXPECTED, module_type);
            if (module_type == 0xCC) {
                ESP_LOGW(TAG, "  Note: 0xCC indicates VL53L1, not VL53L3CX");
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 2: Register Read/Write Test");
    ESP_LOGI(TAG, "VL53L3CX Platform Layer Test");
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
    int8_t status = VL53LX_PlatformInit(&vl53lx_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX platform init failed (status: %d)", status);
        return;
    }

    // Test register access
    test_register_access();

    // Cleanup
    VL53LX_PlatformDeinit(&vl53lx_dev);

    ESP_LOGI(TAG, "Test completed. You can now flash Stage 3.");
}
