/**
 * @file power_monitor.cpp
 * @brief INA3221 Power Monitor Driver Implementation
 *
 * Reference: INA3221 Datasheet, Texas Instruments
 */

#include "power_monitor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "PowerMonitor";

namespace stampfly {

// I2C timeout
constexpr int I2C_TIMEOUT_MS = 100;

// LSB values
constexpr float SHUNT_VOLTAGE_LSB_UV = 40.0f;    // 40 uV per bit
constexpr float BUS_VOLTAGE_LSB_MV = 8.0f;       // 8 mV per bit

// Reset delay
constexpr int RESET_DELAY_MS = 2;

PowerMonitor::~PowerMonitor()
{
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

esp_err_t PowerMonitor::init(const Config& config)
{
    config_ = config;
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

    // Read and verify manufacturer ID
    uint16_t manufacturer_id;
    ret = readRegister(ina3221_reg::MANUFACTURER_ID, &manufacturer_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read manufacturer ID");
        return ret;
    }

    if (manufacturer_id != INA3221_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Invalid manufacturer ID: 0x%04X (expected 0x%04X)",
                 manufacturer_id, INA3221_MANUFACTURER_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Read die ID
    uint16_t die_id;
    ret = readRegister(ina3221_reg::DIE_ID, &die_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read die ID");
        return ret;
    }

    ESP_LOGI(TAG, "INA3221 manufacturer ID: 0x%04X, die ID: 0x%04X",
             manufacturer_id, die_id);

    // Perform soft reset
    ret = softReset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset");
        return ret;
    }

    // Configure sensor
    // Config register format:
    // [15]    RST - Reset (0 = normal, 1 = reset)
    // [14]    CH1_EN - Channel 1 enable
    // [13]    CH2_EN - Channel 2 enable
    // [12]    CH3_EN - Channel 3 enable
    // [11:9]  AVG - Averaging mode
    // [8:6]   VBUS_CT - Bus voltage conversion time
    // [5:3]   VSH_CT - Shunt voltage conversion time
    // [2:0]   MODE - Operating mode

    uint16_t config_reg = 0;
    if (config_.enable_ch1) config_reg |= (1 << 14);
    if (config_.enable_ch2) config_reg |= (1 << 13);
    if (config_.enable_ch3) config_reg |= (1 << 12);
    config_reg |= (static_cast<uint16_t>(config_.averaging) << 9);
    config_reg |= (static_cast<uint16_t>(config_.conv_time) << 6);
    config_reg |= (static_cast<uint16_t>(config_.conv_time) << 3);
    config_reg |= static_cast<uint16_t>(config_.mode);

    ret = writeRegister(ina3221_reg::CONFIG, config_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config register");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "INA3221 initialized successfully");

    return ESP_OK;
}

esp_err_t PowerMonitor::read(PowerData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ChannelData ch_data;
    esp_err_t ret = readChannel(config_.battery_channel, ch_data);
    if (ret != ESP_OK) {
        return ret;
    }

    data.voltage_v = ch_data.bus_voltage_v;
    data.current_ma = ch_data.current_ma;
    data.power_mw = ch_data.power_mw;
    data.timestamp_us = esp_timer_get_time();

    last_voltage_v_ = data.voltage_v;

    return ESP_OK;
}

esp_err_t PowerMonitor::readChannel(uint8_t channel, ChannelData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= NUM_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate register addresses for channel
    uint8_t shunt_reg = ina3221_reg::CH1_SHUNT_VOLTAGE + (channel * 2);
    uint8_t bus_reg = ina3221_reg::CH1_BUS_VOLTAGE + (channel * 2);

    // Read shunt voltage
    uint16_t shunt_raw;
    esp_err_t ret = readRegister(shunt_reg, &shunt_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read bus voltage
    uint16_t bus_raw;
    ret = readRegister(bus_reg, &bus_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert raw values to physical units
    // Shunt voltage: 13-bit signed value, bits [15:3]
    int16_t shunt_signed = (int16_t)(shunt_raw & 0xFFF8) >> 3;
    // Sign extend if negative
    if (shunt_raw & 0x8000) {
        shunt_signed |= 0xE000;
    }
    data.shunt_voltage_mv = shunt_signed * SHUNT_VOLTAGE_LSB_UV / 1000.0f;

    // Bus voltage: 13-bit unsigned value, bits [15:3]
    uint16_t bus_unsigned = (bus_raw >> 3) & 0x1FFF;
    data.bus_voltage_v = bus_unsigned * BUS_VOLTAGE_LSB_MV / 1000.0f;

    // Calculate current (I = V_shunt / R_shunt)
    data.current_ma = data.shunt_voltage_mv / config_.shunt_resistor_ohm;

    // Calculate power (P = V_bus * I)
    data.power_mw = data.bus_voltage_v * data.current_ma;

    return ESP_OK;
}

float PowerMonitor::getBatteryPercent() const
{
    if (last_voltage_v_ >= FULL_BATTERY_V) {
        return 100.0f;
    }
    if (last_voltage_v_ <= EMPTY_BATTERY_V) {
        return 0.0f;
    }
    return (last_voltage_v_ - EMPTY_BATTERY_V) / (FULL_BATTERY_V - EMPTY_BATTERY_V) * 100.0f;
}

esp_err_t PowerMonitor::softReset()
{
    // Set reset bit (bit 15)
    esp_err_t ret = writeRegister(ina3221_reg::CONFIG, 0x8000);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t PowerMonitor::readRegister(uint8_t reg, uint16_t* data)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_, &reg, 1, buf, 2,
                                                 I2C_TIMEOUT_MS);
    if (ret == ESP_OK) {
        // INA3221 uses big-endian (MSB first)
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

esp_err_t PowerMonitor::writeRegister(uint8_t reg, uint16_t data)
{
    // INA3221 uses big-endian (MSB first)
    uint8_t buf[3] = {reg, (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return i2c_master_transmit(dev_handle_, buf, 3, I2C_TIMEOUT_MS);
}

}  // namespace stampfly
