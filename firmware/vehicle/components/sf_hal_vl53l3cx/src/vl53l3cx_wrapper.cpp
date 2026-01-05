/**
 * @file vl53l3cx_wrapper.cpp
 * @brief C++ wrapper implementation for VL53L3CX ToF Sensor
 */

#include "vl53l3cx_wrapper.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "VL53L3CX";

namespace stampfly {

VL53L3CXWrapper::~VL53L3CXWrapper()
{
    if (initialized_) {
        if (ranging_) {
            VL53LX_StopMeasurement(&device_);
        }
        if (i2c_dev_handle_) {
            i2c_master_bus_rm_device(i2c_dev_handle_);
        }
        initialized_ = false;
    }
}

VL53L3CXWrapper::VL53L3CXWrapper(VL53L3CXWrapper&& other) noexcept
    : device_(other.device_)
    , config_(other.config_)
    , initialized_(other.initialized_)
    , ranging_(other.ranging_)
    , i2c_dev_handle_(other.i2c_dev_handle_)
{
    other.initialized_ = false;
    other.ranging_ = false;
    other.i2c_dev_handle_ = nullptr;
}

VL53L3CXWrapper& VL53L3CXWrapper::operator=(VL53L3CXWrapper&& other) noexcept
{
    if (this != &other) {
        // Cleanup existing resources
        if (initialized_ && i2c_dev_handle_) {
            i2c_master_bus_rm_device(i2c_dev_handle_);
        }

        device_ = other.device_;
        config_ = other.config_;
        initialized_ = other.initialized_;
        ranging_ = other.ranging_;
        i2c_dev_handle_ = other.i2c_dev_handle_;

        other.initialized_ = false;
        other.ranging_ = false;
        other.i2c_dev_handle_ = nullptr;
    }
    return *this;
}

esp_err_t VL53L3CXWrapper::setXshut(bool enable)
{
    // Configure XSHUT pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config_.xshut_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    return gpio_set_level(config_.xshut_pin, enable ? 1 : 0);
}

esp_err_t VL53L3CXWrapper::waitDeviceBoot()
{
    VL53LX_Error status = VL53LX_WaitDeviceBooted(&device_);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "WaitDeviceBooted failed: %d", status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    // Initialize device structure
    memset(&device_, 0, sizeof(device_));

    // Put sensor in reset
    esp_err_t ret = setXshut(false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set XSHUT low");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Release reset
    ret = setXshut(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set XSHUT high");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Add I2C device at default address first
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEFAULT_I2C_ADDR,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    ret = i2c_master_bus_add_device(config_.i2c_bus, &dev_cfg, &i2c_dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device handle in VL53LX structure
    device_.I2cHandle = i2c_dev_handle_;
    device_.I2cDevAddr = DEFAULT_I2C_ADDR;

    // Wait for device boot
    ret = waitDeviceBoot();
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(i2c_dev_handle_);
        return ret;
    }

    // Change I2C address if different from default
    if (config_.i2c_addr != DEFAULT_I2C_ADDR) {
        VL53LX_Error status = VL53LX_SetDeviceAddress(&device_, config_.i2c_addr << 1);
        if (status != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "SetDeviceAddress failed: %d", status);
            i2c_master_bus_rm_device(i2c_dev_handle_);
            return ESP_FAIL;
        }

        // Remove old device and add with new address
        i2c_master_bus_rm_device(i2c_dev_handle_);

        dev_cfg.device_address = config_.i2c_addr;
        ret = i2c_master_bus_add_device(config_.i2c_bus, &dev_cfg, &i2c_dev_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device with new address");
            return ret;
        }

        device_.I2cHandle = i2c_dev_handle_;
        device_.I2cDevAddr = config_.i2c_addr;
    }

    // Initialize sensor
    VL53LX_Error status = VL53LX_DataInit(&device_);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "DataInit failed: %d", status);
        i2c_master_bus_rm_device(i2c_dev_handle_);
        return ESP_FAIL;
    }

    // Set distance mode (Long by default for altitude sensing)
    status = VL53LX_SetDistanceMode(&device_, VL53LX_DISTANCEMODE_LONG);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "SetDistanceMode failed: %d", status);
        i2c_master_bus_rm_device(i2c_dev_handle_);
        return ESP_FAIL;
    }

    // Set timing budget
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&device_,
                                                           config_.timing_budget_ms * 1000);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "SetTimingBudget failed: %d", status);
        i2c_master_bus_rm_device(i2c_dev_handle_);
        return ESP_FAIL;
    }

    initialized_ = true;

    const char* location_str = (config_.location == ToFLocation::BOTTOM) ? "BOTTOM" : "FRONT";
    ESP_LOGI(TAG, "VL53L3CX (%s) initialized at address 0x%02X", location_str, config_.i2c_addr);

    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::startRanging()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_Error status = VL53LX_StartMeasurement(&device_);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "StartMeasurement failed: %d", status);
        return ESP_FAIL;
    }

    ranging_ = true;
    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::stopRanging()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_Error status = VL53LX_StopMeasurement(&device_);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "StopMeasurement failed: %d", status);
        return ESP_FAIL;
    }

    ranging_ = false;
    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::isDataReady(bool& ready)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data_ready = 0;
    VL53LX_Error status = VL53LX_GetMeasurementDataReady(&device_, &data_ready);
    if (status != VL53LX_ERROR_NONE) {
        return ESP_FAIL;
    }

    ready = (data_ready != 0);
    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::getDistance(DistanceData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_MultiRangingData_t ranging_data;
    VL53LX_Error status = VL53LX_GetMultiRangingData(&device_, &ranging_data);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "GetMultiRangingData failed: %d", status);
        return ESP_FAIL;
    }

    data.num_objects = ranging_data.NumberOfObjectsFound;
    data.timestamp = ranging_data.StreamCount;

    if (ranging_data.NumberOfObjectsFound > 0) {
        // Use first (closest) target
        const VL53LX_TargetRangeData_t& target = ranging_data.RangeData[0];
        data.distance_mm = target.RangeMilliMeter;
        data.range_status = target.RangeStatus;
        data.signal_rate = target.SignalRateRtnMegaCps / 65536.0f;
        data.ambient_rate = target.AmbientRateRtnMegaCps / 65536.0f;
        data.sigma_mm = target.SigmaMilliMeter / 65536.0f;
    } else {
        data.distance_mm = 0;
        data.range_status = 255;  // No target
        data.signal_rate = 0;
        data.ambient_rate = 0;
        data.sigma_mm = 0;
    }

    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::getDistance(uint16_t& distance_mm, uint8_t& status)
{
    DistanceData data;
    esp_err_t ret = getDistance(data);
    if (ret != ESP_OK) {
        return ret;
    }

    distance_mm = (data.distance_mm > 0) ? static_cast<uint16_t>(data.distance_mm) : 0;
    status = data.range_status;
    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::clearInterruptAndStartMeasurement()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_Error status = VL53LX_ClearInterruptAndStartMeasurement(&device_);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "ClearInterruptAndStartMeasurement failed: %d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::setDistanceMode(DistanceMode mode)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_Error status = VL53LX_SetDistanceMode(&device_, static_cast<VL53LX_DistanceModes>(mode));
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "SetDistanceMode failed: %d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::setTimingBudget(uint32_t budget_ms)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53LX_Error status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&device_, budget_ms * 1000);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "SetTimingBudget failed: %d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t VL53L3CXWrapper::initDualSensors(
    VL53L3CXWrapper& bottom_sensor,
    VL53L3CXWrapper& front_sensor,
    i2c_master_bus_handle_t i2c_bus,
    gpio_num_t bottom_xshut,
    gpio_num_t front_xshut)
{
    ESP_LOGI(TAG, "Initializing dual ToF sensors...");

    // Configure both XSHUT pins as outputs and hold both sensors in reset
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << bottom_xshut) | (1ULL << front_xshut),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure XSHUT GPIO pins");
        return ret;
    }

    // Hold both sensors in reset
    gpio_set_level(bottom_xshut, 0);
    gpio_set_level(front_xshut, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Initialize bottom sensor first
    gpio_set_level(bottom_xshut, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    Config bottom_config = Config::defaultBottom(i2c_bus);
    bottom_config.xshut_pin = bottom_xshut;

    ret = bottom_sensor.init(bottom_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bottom sensor");
        return ret;
    }

    // Now try to initialize front sensor (optional - may not be connected)
    gpio_set_level(front_xshut, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    Config front_config = Config::defaultFront(i2c_bus);
    front_config.xshut_pin = front_xshut;

    ret = front_sensor.init(front_config);
    if (ret != ESP_OK) {
        // Front sensor not connected - this is OK, keep XSHUT low to avoid I2C errors
        gpio_set_level(front_xshut, 0);
        ESP_LOGW(TAG, "Front ToF not detected (optional sensor) - disabled");
        // Return OK since bottom sensor is working
        ESP_LOGI(TAG, "Bottom ToF sensor initialized successfully (front not available)");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Dual ToF sensors initialized successfully");
    return ESP_OK;
}

}  // namespace stampfly
