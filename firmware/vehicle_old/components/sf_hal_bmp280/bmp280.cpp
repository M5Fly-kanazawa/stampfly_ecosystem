/**
 * @file bmp280.cpp
 * @brief BMP280 Barometric Pressure Sensor Driver Implementation
 *
 * Reference: BMP280 Datasheet, Bosch Sensortec
 */

#include "bmp280.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "BMP280";

namespace stampfly {

// I2C timeout
constexpr int I2C_TIMEOUT_MS = 100;

// Soft reset command
constexpr uint8_t BMP280_RESET_CMD = 0xB6;

// Startup delay after power on
constexpr int STARTUP_DELAY_MS = 3;

BMP280::~BMP280()
{
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

esp_err_t BMP280::init(const Config& config)
{
    config_ = config;
    sea_level_pressure_pa_ = config.sea_level_pressure_pa;
    esp_err_t ret;

    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config_.i2c_addr,
        .scl_speed_hz = 400000,  // 400kHz
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    ret = i2c_master_bus_add_device(config_.i2c_bus, &dev_cfg, &dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read and verify chip ID
    uint8_t chip_id;
    ret = readRegister(bmp280_reg::CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X or 0x%02X)",
                 chip_id, BMP280_CHIP_ID, BME280_CHIP_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "BMP280 chip ID: 0x%02X", chip_id);

    // Perform soft reset
    ret = softReset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset");
        return ret;
    }

    // Read calibration data
    ret = readCalibrationData();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    // Configure sensor
    // Config register: standby time, filter, SPI mode (not used)
    uint8_t config_reg = (static_cast<uint8_t>(config_.standby) << 5) |
                         (static_cast<uint8_t>(config_.filter) << 2);
    ret = writeRegister(bmp280_reg::CONFIG, config_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config register");
        return ret;
    }

    // Ctrl_meas register: temp os, press os, mode
    uint8_t ctrl_meas = (static_cast<uint8_t>(config_.temp_os) << 5) |
                        (static_cast<uint8_t>(config_.press_os) << 2) |
                        static_cast<uint8_t>(config_.mode);
    ret = writeRegister(bmp280_reg::CTRL_MEAS, ctrl_meas);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ctrl_meas register");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "BMP280 initialized successfully");

    return ESP_OK;
}

esp_err_t BMP280::read(BaroData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read all data registers at once (0xF7-0xFC, 6 bytes)
    uint8_t buf[6];
    esp_err_t ret = readRegister(bmp280_reg::PRESS_MSB, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse raw data (20-bit values, MSB first)
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

    // Compensate temperature first (sets t_fine_ for pressure compensation)
    int32_t temp_raw = compensateTemperature(adc_T);
    data.temperature_c = temp_raw / 100.0f;

    // Compensate pressure
    uint32_t press_raw = compensatePressure(adc_P);
    data.pressure_pa = press_raw / 256.0f;

    // Calculate altitude
    data.altitude_m = calculateAltitude(data.pressure_pa);

    data.timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

esp_err_t BMP280::triggerMeasurement()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read current ctrl_meas
    uint8_t ctrl_meas;
    esp_err_t ret = readRegister(bmp280_reg::CTRL_MEAS, &ctrl_meas, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set forced mode (bits [1:0])
    ctrl_meas = (ctrl_meas & 0xFC) | static_cast<uint8_t>(BMP280Mode::FORCED);

    return writeRegister(bmp280_reg::CTRL_MEAS, ctrl_meas);
}

bool BMP280::isMeasuring()
{
    if (!initialized_) {
        return false;
    }

    uint8_t status;
    if (readRegister(bmp280_reg::STATUS, &status, 1) != ESP_OK) {
        return false;
    }

    // Bit 3: measuring, Bit 0: im_update
    return (status & 0x08) != 0;
}

float BMP280::calculateAltitude(float pressure_pa) const
{
    // International barometric formula
    // h = 44330 * (1 - (P/P0)^0.1903)
    return 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pressure_pa_, 0.1903f));
}

void BMP280::setSeaLevelPressure(float pressure_pa)
{
    sea_level_pressure_pa_ = pressure_pa;
}

esp_err_t BMP280::softReset()
{
    esp_err_t ret = writeRegister(bmp280_reg::RESET, BMP280_RESET_CMD);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t BMP280::readRegister(uint8_t reg, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len,
                                       I2C_TIMEOUT_MS);
}

esp_err_t BMP280::writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle_, buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t BMP280::readCalibrationData()
{
    // Read all calibration data at once (0x88-0x9F, 24 bytes)
    uint8_t buf[24];
    esp_err_t ret = readRegister(bmp280_reg::CALIB00, buf, 24);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse calibration data (little-endian)
    calib_.dig_T1 = (uint16_t)(buf[1] << 8) | buf[0];
    calib_.dig_T2 = (int16_t)(buf[3] << 8) | buf[2];
    calib_.dig_T3 = (int16_t)(buf[5] << 8) | buf[4];
    calib_.dig_P1 = (uint16_t)(buf[7] << 8) | buf[6];
    calib_.dig_P2 = (int16_t)(buf[9] << 8) | buf[8];
    calib_.dig_P3 = (int16_t)(buf[11] << 8) | buf[10];
    calib_.dig_P4 = (int16_t)(buf[13] << 8) | buf[12];
    calib_.dig_P5 = (int16_t)(buf[15] << 8) | buf[14];
    calib_.dig_P6 = (int16_t)(buf[17] << 8) | buf[16];
    calib_.dig_P7 = (int16_t)(buf[19] << 8) | buf[18];
    calib_.dig_P8 = (int16_t)(buf[21] << 8) | buf[20];
    calib_.dig_P9 = (int16_t)(buf[23] << 8) | buf[22];

    return ESP_OK;
}

int32_t BMP280::compensateTemperature(int32_t adc_T)
{
    // From BMP280 datasheet - 32-bit integer compensation
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib_.dig_T1 << 1))) *
            ((int32_t)calib_.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)calib_.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)calib_.dig_T1))) >> 12) *
            ((int32_t)calib_.dig_T3)) >> 14;

    t_fine_ = var1 + var2;
    T = (t_fine_ * 5 + 128) >> 8;

    return T;  // Temperature in 0.01 degrees Celsius
}

uint32_t BMP280::compensatePressure(int32_t adc_P)
{
    // From BMP280 datasheet - 64-bit integer compensation
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine_) - 128000;
    var2 = var1 * var1 * (int64_t)calib_.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_.dig_P3) >> 8) +
           ((var1 * (int64_t)calib_.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_.dig_P7) << 4);

    return (uint32_t)p;  // Pressure in Pa as Q24.8 format
}

}  // namespace stampfly
