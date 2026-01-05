/**
 * @file power_monitor.hpp
 * @brief INA3221 Power Monitor Driver
 *
 * 3-channel current/voltage monitor, low voltage warning (3.4V threshold)
 * Reference: INA3221 Datasheet, Texas Instruments
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

namespace stampfly {

// INA3221 I2C Address
constexpr uint8_t INA3221_I2C_ADDR_GND = 0x40;   // A0 = GND
constexpr uint8_t INA3221_I2C_ADDR_VS = 0x41;    // A0 = VS
constexpr uint8_t INA3221_I2C_ADDR_SDA = 0x42;   // A0 = SDA
constexpr uint8_t INA3221_I2C_ADDR_SCL = 0x43;   // A0 = SCL

// Register addresses
namespace ina3221_reg {
    constexpr uint8_t CONFIG = 0x00;
    // Channel 1
    constexpr uint8_t CH1_SHUNT_VOLTAGE = 0x01;
    constexpr uint8_t CH1_BUS_VOLTAGE = 0x02;
    // Channel 2
    constexpr uint8_t CH2_SHUNT_VOLTAGE = 0x03;
    constexpr uint8_t CH2_BUS_VOLTAGE = 0x04;
    // Channel 3
    constexpr uint8_t CH3_SHUNT_VOLTAGE = 0x05;
    constexpr uint8_t CH3_BUS_VOLTAGE = 0x06;
    // Alert limits
    constexpr uint8_t CH1_CRIT_LIMIT = 0x07;
    constexpr uint8_t CH1_WARN_LIMIT = 0x08;
    constexpr uint8_t CH2_CRIT_LIMIT = 0x09;
    constexpr uint8_t CH2_WARN_LIMIT = 0x0A;
    constexpr uint8_t CH3_CRIT_LIMIT = 0x0B;
    constexpr uint8_t CH3_WARN_LIMIT = 0x0C;
    // Sum registers
    constexpr uint8_t SHUNT_VOLTAGE_SUM = 0x0D;
    constexpr uint8_t SHUNT_VOLTAGE_SUM_LIMIT = 0x0E;
    // Mask/Enable
    constexpr uint8_t MASK_ENABLE = 0x0F;
    // Power valid limits
    constexpr uint8_t PV_UPPER_LIMIT = 0x10;
    constexpr uint8_t PV_LOWER_LIMIT = 0x11;
    // IDs
    constexpr uint8_t MANUFACTURER_ID = 0xFE;
    constexpr uint8_t DIE_ID = 0xFF;
}

// Expected IDs
constexpr uint16_t INA3221_MANUFACTURER_ID = 0x5449;  // "TI"
constexpr uint16_t INA3221_DIE_ID = 0x3220;

// Averaging mode
enum class INA3221Averaging : uint8_t {
    AVG_1 = 0x00,
    AVG_4 = 0x01,
    AVG_16 = 0x02,
    AVG_64 = 0x03,
    AVG_128 = 0x04,
    AVG_256 = 0x05,
    AVG_512 = 0x06,
    AVG_1024 = 0x07,
};

// Bus voltage conversion time
enum class INA3221ConvTime : uint8_t {
    US_140 = 0x00,
    US_204 = 0x01,
    US_332 = 0x02,
    US_588 = 0x03,
    US_1100 = 0x04,
    US_2116 = 0x05,
    US_4156 = 0x06,
    US_8244 = 0x07,
};

// Operating mode
enum class INA3221Mode : uint8_t {
    POWER_DOWN = 0x00,
    SHUNT_SINGLE = 0x01,
    BUS_SINGLE = 0x02,
    SHUNT_BUS_SINGLE = 0x03,
    POWER_DOWN_2 = 0x04,
    SHUNT_CONTINUOUS = 0x05,
    BUS_CONTINUOUS = 0x06,
    SHUNT_BUS_CONTINUOUS = 0x07,
};

struct PowerData {
    float voltage_v;       // Battery voltage (V)
    float current_ma;      // Battery current (mA)
    float power_mw;        // Power consumption (mW)
    uint32_t timestamp_us;
};

struct ChannelData {
    float shunt_voltage_mv;  // Shunt voltage (mV)
    float bus_voltage_v;     // Bus voltage (V)
    float current_ma;        // Current (mA)
    float power_mw;          // Power (mW)
};

class PowerMonitor {
public:
    static constexpr float LOW_BATTERY_THRESHOLD_V = 3.4f;
    static constexpr float FULL_BATTERY_V = 4.2f;
    static constexpr float EMPTY_BATTERY_V = 3.3f;
    static constexpr int NUM_CHANNELS = 3;

    struct Config {
        i2c_master_bus_handle_t i2c_bus;  // I2C bus handle (must be initialized)
        uint8_t i2c_addr = INA3221_I2C_ADDR_GND;
        uint8_t battery_channel = 0;       // Channel for battery monitoring (0-2)
        float shunt_resistor_ohm = 0.1f;   // Shunt resistor value
        INA3221Averaging averaging = INA3221Averaging::AVG_64;
        INA3221ConvTime conv_time = INA3221ConvTime::US_1100;
        INA3221Mode mode = INA3221Mode::SHUNT_BUS_CONTINUOUS;
        bool enable_ch1 = true;
        bool enable_ch2 = true;
        bool enable_ch3 = true;
    };

    PowerMonitor() = default;
    ~PowerMonitor();

    /**
     * @brief Initialize INA3221
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read power data from battery channel
     * @param data Power data output
     * @return ESP_OK on success
     */
    esp_err_t read(PowerData& data);

    /**
     * @brief Read data from specific channel
     * @param channel Channel number (0-2)
     * @param data Channel data output
     * @return ESP_OK on success
     */
    esp_err_t readChannel(uint8_t channel, ChannelData& data);

    /**
     * @brief Check if battery is low (<3.4V)
     * @return true if battery voltage is below threshold
     */
    bool isLowBattery() const { return last_voltage_v_ < LOW_BATTERY_THRESHOLD_V; }

    /**
     * @brief Get battery percentage
     * @return Battery percentage (0-100)
     */
    float getBatteryPercent() const;

    /**
     * @brief Get last read voltage
     * @return Voltage in volts
     */
    float getVoltage() const { return last_voltage_v_; }

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

    bool isInitialized() const { return initialized_; }

private:
    esp_err_t readRegister(uint8_t reg, uint16_t* data);
    esp_err_t writeRegister(uint8_t reg, uint16_t data);

    bool initialized_ = false;
    Config config_;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    float last_voltage_v_ = 4.2f;  // Initialize to full battery to avoid false low-battery warning
};

}  // namespace stampfly
