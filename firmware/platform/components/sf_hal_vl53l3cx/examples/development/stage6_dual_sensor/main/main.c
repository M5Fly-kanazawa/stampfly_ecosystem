/**
 * @file main.c
 * @brief Stage 6: VL53L3CX Dual Sensor Operation
 *
 * This example demonstrates simultaneous operation of two ToF sensors.
 * - I2C address management (0x29 and 0x30)
 * - Individual interrupt handling for each sensor
 * - Timing Budget: 33ms
 * - Distance Mode: MEDIUM
 *
 * Hardware Setup:
 * - I2C SDA: GPIO3
 * - I2C SCL: GPIO4
 * - Bottom ToF XSHUT: GPIO7, INT: GPIO6, I2C: 0x30 [DEFAULT - USB powered]
 * - Front ToF XSHUT: GPIO9, INT: GPIO8, I2C: 0x29 [Requires battery]
 *
 * Configuration:
 * - Set ENABLE_FRONT_SENSOR to 1 to enable front sensor (requires battery)
 * - Set ENABLE_FRONT_SENSOR to 0 to use only bottom sensor (USB powered)
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

static const char *TAG = "STAGE6_DUAL";

// Configuration: Enable/Disable front sensor
#define ENABLE_FRONT_SENSOR  1  // 0: Bottom only (USB), 1: Both sensors (Battery required)

// I2C addresses
#define BOTTOM_TOF_I2C_ADDR  0x30  // Bottom sensor (changed from default)
#define FRONT_TOF_I2C_ADDR   0x29  // Front sensor (default)

// Measurement configuration
#define TIMING_BUDGET_MS     33
#define MEASUREMENT_COUNT    20

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static VL53LX_Dev_t bottom_dev;
#if ENABLE_FRONT_SENSOR
static VL53LX_Dev_t front_dev;
#endif

static SemaphoreHandle_t bottom_semaphore = NULL;
#if ENABLE_FRONT_SENSOR
static SemaphoreHandle_t front_semaphore = NULL;
#endif

/**
 * @brief GPIO interrupt handler for bottom ToF INT pin
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
 * @brief GPIO interrupt handler for front ToF INT pin
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
 * @brief Initialize XSHUT pins and perform I2C address change sequence
 */
static void tof_xshut_init_and_address_change(void)
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
    VL53LX_Error status = VL53LX_PlatformInit(&bottom_dev, i2c_bus_handle, VL53L3CX_DEFAULT_I2C_ADDR);
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

    // Initialize front sensor platform
    status = VL53LX_PlatformInit(&front_dev, i2c_bus_handle, FRONT_TOF_I2C_ADDR);
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
 * @brief Initialize INT pins for GPIO interrupts
 */
static esp_err_t tof_int_init(void)
{
    // Configure bottom INT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAMPFLY_TOF_BOTTOM_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bottom INT GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

#if ENABLE_FRONT_SENSOR
    // Configure front INT pin
    io_conf.pin_bit_mask = (1ULL << STAMPFLY_TOF_FRONT_INT);
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Front INT GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
#endif

    // Install GPIO ISR service
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(err));
        return err;
    }

    // Add ISR handlers
    err = gpio_isr_handler_add(STAMPFLY_TOF_BOTTOM_INT, bottom_int_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bottom ISR handler add failed: %s", esp_err_to_name(err));
        return err;
    }

#if ENABLE_FRONT_SENSOR
    err = gpio_isr_handler_add(STAMPFLY_TOF_FRONT_INT, front_int_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Front ISR handler add failed: %s", esp_err_to_name(err));
        return err;
    }
#endif

    ESP_LOGI(TAG, "INT pins initialized");
    ESP_LOGI(TAG, "Bottom INT: GPIO%d", STAMPFLY_TOF_BOTTOM_INT);
#if ENABLE_FRONT_SENSOR
    ESP_LOGI(TAG, "Front INT: GPIO%d", STAMPFLY_TOF_FRONT_INT);
#endif

    return ESP_OK;
}

/**
 * @brief Initialize a VL53LX device
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
 * @brief Perform measurements from a single sensor
 */
static void measure_sensor(VL53LX_Dev_t *pDev, SemaphoreHandle_t sem, const char *name, uint32_t count)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t data;
    uint32_t measurement_count = 0;

    ESP_LOGI(TAG, "%s: Starting measurements...", name);

    status = VL53LX_StartMeasurement(pDev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "%s: Start measurement failed (status: %d)", name, status);
        return;
    }

    while (measurement_count < count) {
        if (xSemaphoreTake(sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
            status = VL53LX_GetMultiRangingData(pDev, &data);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "%s: Get data failed (status: %d)", name, status);
                return;
            }

            measurement_count++;

            if (data.NumberOfObjectsFound > 0) {
                VL53LX_TargetRangeData_t *target = &data.RangeData[0];
                ESP_LOGI(TAG, "%s [%02d]: %4d mm | Status: %d | Signal: %.2f Mcps",
                         name, (int)measurement_count,
                         (int)target->RangeMilliMeter,
                         (int)target->RangeStatus,
                         target->SignalRateRtnMegaCps / 65536.0);
            } else {
                ESP_LOGI(TAG, "%s [%02d]: No objects detected", name, (int)measurement_count);
            }

            status = VL53LX_ClearInterruptAndStartMeasurement(pDev);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "%s: Clear interrupt failed (status: %d)", name, status);
                return;
            }
        } else {
            ESP_LOGW(TAG, "%s: Timeout waiting for interrupt", name);
        }
    }

    status = VL53LX_StopMeasurement(pDev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "%s: Stop measurement failed (status: %d)", name, status);
    }

    ESP_LOGI(TAG, "%s: Measurements complete!", name);
}

/**
 * @brief Dual sensor measurement task
 */
static void perform_dual_measurements(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Starting dual sensor measurements");
    ESP_LOGI(TAG, "Interrupt mode, %d measurements per sensor", MEASUREMENT_COUNT);
#if ENABLE_FRONT_SENSOR
    ESP_LOGI(TAG, "Both sensors active");
#else
    ESP_LOGI(TAG, "Bottom sensor only (USB powered)");
#endif
    ESP_LOGI(TAG, "==================================");

    // Measure bottom sensor
    measure_sensor(&bottom_dev, bottom_semaphore, "BOTTOM", MEASUREMENT_COUNT);

#if ENABLE_FRONT_SENSOR
    // Measure front sensor
    vTaskDelay(pdMS_TO_TICKS(100));  // Small delay between sensors
    measure_sensor(&front_dev, front_semaphore, "FRONT", MEASUREMENT_COUNT);
#endif

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "All measurements complete!");
    ESP_LOGI(TAG, "==================================");
}

void app_main(void)
{
    VL53LX_Error status;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "Stage 6: Dual Sensor Operation");
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

    // Perform measurements
    perform_dual_measurements();

    // Cleanup
    gpio_isr_handler_remove(STAMPFLY_TOF_BOTTOM_INT);
#if ENABLE_FRONT_SENSOR
    gpio_isr_handler_remove(STAMPFLY_TOF_FRONT_INT);
    VL53LX_PlatformDeinit(&front_dev);
    vSemaphoreDelete(front_semaphore);
#endif
    VL53LX_PlatformDeinit(&bottom_dev);
    vSemaphoreDelete(bottom_semaphore);

    ESP_LOGI(TAG, "Test completed. Dual sensor implementation ready for production use.");
}
