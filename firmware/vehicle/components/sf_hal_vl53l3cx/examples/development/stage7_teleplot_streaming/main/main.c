/**
 * @file main.c
 * @brief Stage 7: VL53L3CX Teleplot Streaming
 *
 * Continuous distance measurement streaming for Teleplot visualization.
 * - Bottom ToF sensor enabled by default (USB powered)
 * - Front ToF sensor optional (requires battery)
 * - Interrupt-based measurement
 * - Teleplot format output (>variable:value)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "stampfly_tof_config.h"
#include "vl53lx_platform.h"
#include "vl53lx_api.h"

static const char *TAG = "STAGE7_TELEPLOT";

// Configuration: Enable/Disable front sensor
#define ENABLE_FRONT_SENSOR  1  // 0: Bottom only (USB), 1: Both sensors (Battery required)

// I2C addresses
#define BOTTOM_TOF_I2C_ADDR  0x30  // Bottom sensor (changed from default)
#define FRONT_TOF_I2C_ADDR   0x29  // Front sensor (default)

// Measurement configuration
#define TIMING_BUDGET_MS     33

// Device structures
static VL53LX_Dev_t bottom_dev;
static VL53LX_Dev_t front_dev;

// Semaphores for interrupt handling
static SemaphoreHandle_t bottom_semaphore = NULL;
static SemaphoreHandle_t front_semaphore = NULL;

// I2C bus handle (shared between sensors)
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

/**
 * @brief Initialize I2C master bus
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = STAMPFLY_I2C_SCL_GPIO,
        .sda_io_num = STAMPFLY_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed");
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", STAMPFLY_I2C_SDA_GPIO, STAMPFLY_I2C_SCL_GPIO);
    return ESP_OK;
}

/**
 * @brief ISR handler for bottom ToF interrupt
 */
static void IRAM_ATTR bottom_int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bottom_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#if ENABLE_FRONT_SENSOR
/**
 * @brief ISR handler for front ToF interrupt
 */
static void IRAM_ATTR front_int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(front_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
#endif

/**
 * @brief Initialize INT pins for interrupt handling
 */
static esp_err_t tof_int_init(void)
{
    // Configure bottom INT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_BOTTOM_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

#if ENABLE_FRONT_SENSOR
    // Configure front INT pin
    io_conf.pin_bit_mask = (1ULL << STAMPFLY_TOF_FRONT_INT);
    gpio_config(&io_conf);
#endif

    // Install ISR service
    gpio_install_isr_service(0);

    // Add ISR handlers
    gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, bottom_int_isr_handler, NULL);

#if ENABLE_FRONT_SENSOR
    gpio_isr_handler_add(STAMPFLY_TOF_FRONT_INT, front_int_isr_handler, NULL);
#endif

    ESP_LOGI(TAG, "INT pins initialized");
    ESP_LOGI(TAG, "Bottom INT: GPIO%d", STAMPFLY_TOF_BOTTOM_INT);
#if ENABLE_FRONT_SENSOR
    ESP_LOGI(TAG, "Front INT: GPIO%d", STAMPFLY_TOF_FRONT_INT);
#endif

    return ESP_OK;
}

/**
 * @brief Initialize XSHUT pins and perform I2C address change sequence
 */
static void tof_xshut_init_and_address_change(void)
{
    VL53LX_Error status;

    // Configure XSHUT pins as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_BOTTOM_XSHUT) | (1ULL << STAMPFLY_TOF_FRONT_XSHUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Starting I2C address change sequence...");

    // Step 1: Shutdown both sensors
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 0);
    gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "  1. Both sensors shutdown");

    // Step 2: Enable bottom sensor (will have default address 0x29)
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "  2. Bottom sensor enabled at default 0x29");

    // Step 3: Change bottom sensor address to 0x30
    status = VL53LX_PlatformInit(&bottom_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "  3. Bottom sensor platform init failed (status: %d)", status);
        return;
    }

    status = VL53LX_SetDeviceAddress(&bottom_dev, BOTTOM_TOF_I2C_ADDR << 1);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "  3. Bottom sensor address change failed (status: %d)", status);
        return;
    }

    // Deinit old I2C device handle (0x29) and reinit with new address (0x30)
    VL53LX_PlatformDeinit(&bottom_dev);
    status = VL53LX_PlatformInit(&bottom_dev, i2c_bus_handle, BOTTOM_TOF_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "  3. Bottom sensor re-init at new address failed (status: %d)", status);
        return;
    }
    ESP_LOGI(TAG, "  3. Bottom sensor address changed to 0x%02X", BOTTOM_TOF_I2C_ADDR);

#if ENABLE_FRONT_SENSOR
    // Step 4: Enable front sensor (will have default address 0x29)
    gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "  4. Front sensor enabled at default 0x29");

    status = VL53LX_PlatformInit(&front_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "  4. Front sensor platform init failed (status: %d)", status);
        return;
    }
#else
    ESP_LOGI(TAG, "  4. Front sensor DISABLED (set ENABLE_FRONT_SENSOR=1 to enable)");
#endif

    ESP_LOGI(TAG, "I2C address change sequence complete");
    ESP_LOGI(TAG, "Bottom ToF: GPIO%d (0x%02X) [ENABLED - USB powered]",
             STAMPFLY_TOF_BOTTOM_XSHUT, BOTTOM_TOF_I2C_ADDR);
#if ENABLE_FRONT_SENSOR
    ESP_LOGI(TAG, "Front ToF: GPIO%d (0x%02X) [ENABLED - Battery required]",
             STAMPFLY_TOF_FRONT_XSHUT, FRONT_TOF_I2C_ADDR);
#else
    ESP_LOGI(TAG, "Front ToF: GPIO%d [DISABLED]", STAMPFLY_TOF_FRONT_XSHUT);
#endif
}

/**
 * @brief Initialize a VL53L3CX sensor
 */
static VL53LX_Error initialize_sensor(VL53LX_Dev_t *pDev, const char *name)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "Initializing %s sensor...", name);

    status = VL53LX_WaitDeviceBooted(pDev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "%s: Device boot failed (status: %d)", name, status);
        return status;
    }
    ESP_LOGI(TAG, "%s: ✓ Device booted", name);

    status = VL53LX_DataInit(pDev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "%s: Data init failed (status: %d)", name, status);
        return status;
    }
    ESP_LOGI(TAG, "%s: ✓ Data initialized", name);

    VL53LX_DeviceInfo_t device_info;
    status = VL53LX_GetDeviceInfo(pDev, &device_info);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "%s: Get device info failed (status: %d)", name, status);
        return status;
    }
    ESP_LOGI(TAG, "%s: ✓ Product Type: 0x%02X, Rev: %d.%d", name,
             device_info.ProductType,
             device_info.ProductRevisionMajor,
             device_info.ProductRevisionMinor);

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Bottom sensor streaming task
 */
static void bottom_sensor_task(void *pvParameters)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t data;

    ESP_LOGI(TAG, "BOTTOM: Starting continuous measurements...");

    status = VL53LX_StartMeasurement(&bottom_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "BOTTOM: Start measurement failed (status: %d)", status);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xSemaphoreTake(bottom_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
            status = VL53LX_GetMultiRangingData(&bottom_dev, &data);
            if (status == VL53LX_ERROR_NONE) {
                if (data.NumberOfObjectsFound > 0) {
                    uint16_t distance = data.RangeData[0].RangeMilliMeter;
                    uint8_t range_status = data.RangeData[0].RangeStatus;
                    float signal = data.RangeData[0].SignalRateRtnMegaCps / 65536.0;

                    // Teleplot format output
                    printf(">bottom_distance:%u\n", distance);
                    printf(">bottom_signal:%.2f\n", signal);
                    printf(">bottom_status:%u\n", range_status);
                } else {
                    printf(">bottom_distance:0\n");
                    printf(">bottom_signal:0.00\n");
                    printf(">bottom_status:255\n");
                }

                status = VL53LX_ClearInterruptAndStartMeasurement(&bottom_dev);
                if (status != VL53LX_ERROR_NONE) {
                    ESP_LOGE(TAG, "BOTTOM: Clear interrupt failed (status: %d)", status);
                }
            } else {
                ESP_LOGE(TAG, "BOTTOM: Get data failed (status: %d)", status);
            }
        } else {
            ESP_LOGW(TAG, "BOTTOM: Timeout waiting for interrupt");
        }
    }
}

#if ENABLE_FRONT_SENSOR
/**
 * @brief Front sensor streaming task
 */
static void front_sensor_task(void *pvParameters)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t data;

    ESP_LOGI(TAG, "FRONT: Starting continuous measurements...");

    status = VL53LX_StartMeasurement(&front_dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "FRONT: Start measurement failed (status: %d)", status);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xSemaphoreTake(front_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
            status = VL53LX_GetMultiRangingData(&front_dev, &data);
            if (status == VL53LX_ERROR_NONE) {
                if (data.NumberOfObjectsFound > 0) {
                    uint16_t distance = data.RangeData[0].RangeMilliMeter;
                    uint8_t range_status = data.RangeData[0].RangeStatus;
                    float signal = data.RangeData[0].SignalRateRtnMegaCps / 65536.0;

                    // Teleplot format output
                    printf(">front_distance:%u\n", distance);
                    printf(">front_signal:%.2f\n", signal);
                    printf(">front_status:%u\n", range_status);
                } else {
                    printf(">front_distance:0\n");
                    printf(">front_signal:0.00\n");
                    printf(">front_status:255\n");
                }

                status = VL53LX_ClearInterruptAndStartMeasurement(&front_dev);
                if (status != VL53LX_ERROR_NONE) {
                    ESP_LOGE(TAG, "FRONT: Clear interrupt failed (status: %d)", status);
                }
            } else {
                ESP_LOGE(TAG, "FRONT: Get data failed (status: %d)", status);
            }
        } else {
            ESP_LOGW(TAG, "FRONT: Timeout waiting for interrupt");
        }
    }
}
#endif

void app_main(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 7: Teleplot Streaming");
    ESP_LOGI(TAG, "VL53L3CX ToF Sensors");
    ESP_LOGI(TAG, "==================================");

    // Create semaphores
    bottom_semaphore = xSemaphoreCreateBinary();
    if (bottom_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create bottom semaphore");
        return;
    }

#if ENABLE_FRONT_SENSOR
    front_semaphore = xSemaphoreCreateBinary();
    if (front_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create front semaphore");
        return;
    }
#endif

    // Initialize I2C bus
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }

    // Initialize XSHUT pins and change I2C addresses
    tof_xshut_init_and_address_change();

    // Initialize INT pins
    ret = tof_int_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT pin initialization failed!");
        return;
    }

    // Initialize sensors
    status = initialize_sensor(&bottom_dev, "BOTTOM");
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Bottom sensor initialization failed!");
        VL53LX_PlatformDeinit(&bottom_dev);
        return;
    }

#if ENABLE_FRONT_SENSOR
    status = initialize_sensor(&front_dev, "FRONT");
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Front sensor initialization failed!");
        VL53LX_PlatformDeinit(&front_dev);
        VL53LX_PlatformDeinit(&bottom_dev);
        return;
    }
#endif

    ESP_LOGI(TAG, "Using default measurement parameters");

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Starting continuous streaming");
    ESP_LOGI(TAG, "Interrupt mode, Teleplot format");
#if ENABLE_FRONT_SENSOR
    ESP_LOGI(TAG, "Both sensors active");
#else
    ESP_LOGI(TAG, "Bottom sensor only (USB powered)");
#endif
    ESP_LOGI(TAG, "==================================");

    // Create measurement tasks
    xTaskCreate(bottom_sensor_task, "bottom_tof", 4096, NULL, 5, NULL);

#if ENABLE_FRONT_SENSOR
    xTaskCreate(front_sensor_task, "front_tof", 4096, NULL, 5, NULL);
#endif

    ESP_LOGI(TAG, "Streaming tasks started. Use Teleplot to visualize data.");
}
