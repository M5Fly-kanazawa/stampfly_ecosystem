/**
 * @file bmp280.hpp
 * @brief BMP280 Barometric Pressure Sensor Driver
 *
 * I2C communication, temperature compensation, altitude calculation
 * Reference: BMP280 Datasheet, Bosch Sensortec
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

namespace stampfly {

// BMP280 I2C Address
constexpr uint8_t BMP280_I2C_ADDR_DEFAULT = 0x76;  // SDO = GND
constexpr uint8_t BMP280_I2C_ADDR_ALT = 0x77;      // SDO = VDD

// Chip ID
constexpr uint8_t BMP280_CHIP_ID = 0x58;
constexpr uint8_t BME280_CHIP_ID = 0x60;  // Compatible chip

// Register addresses
namespace bmp280_reg {
    constexpr uint8_t CHIP_ID = 0xD0;
    constexpr uint8_t RESET = 0xE0;
    constexpr uint8_t STATUS = 0xF3;
    constexpr uint8_t CTRL_MEAS = 0xF4;
    constexpr uint8_t CONFIG = 0xF5;
    constexpr uint8_t PRESS_MSB = 0xF7;
    constexpr uint8_t PRESS_LSB = 0xF8;
    constexpr uint8_t PRESS_XLSB = 0xF9;
    constexpr uint8_t TEMP_MSB = 0xFA;
    constexpr uint8_t TEMP_LSB = 0xFB;
    constexpr uint8_t TEMP_XLSB = 0xFC;
    // Calibration data
    constexpr uint8_t CALIB00 = 0x88;  // dig_T1 LSB
    constexpr uint8_t CALIB25 = 0xA1;  // dig_P9 MSB
}

// Power modes
enum class BMP280Mode : uint8_t {
    SLEEP = 0x00,
    FORCED = 0x01,
    NORMAL = 0x03,
};

// Oversampling settings
enum class BMP280Oversampling : uint8_t {
    SKIP = 0x00,     // Output set to 0x80000
    X1 = 0x01,
    X2 = 0x02,
    X4 = 0x03,
    X8 = 0x04,
    X16 = 0x05,
};

// Standby time (normal mode)
enum class BMP280Standby : uint8_t {
    MS_0_5 = 0x00,   // 0.5 ms
    MS_62_5 = 0x01,  // 62.5 ms
    MS_125 = 0x02,   // 125 ms
    MS_250 = 0x03,   // 250 ms
    MS_500 = 0x04,   // 500 ms
    MS_1000 = 0x05,  // 1000 ms
    MS_2000 = 0x06,  // 2000 ms
    MS_4000 = 0x07,  // 4000 ms
};

// IIR filter coefficient
enum class BMP280Filter : uint8_t {
    OFF = 0x00,
    COEF_2 = 0x01,
    COEF_4 = 0x02,
    COEF_8 = 0x03,
    COEF_16 = 0x04,
};

struct BaroData {
    float pressure_pa;     // Pascal
    float temperature_c;   // Celsius
    float altitude_m;      // Meters (calculated)
    uint32_t timestamp_us;
};

class BMP280 {
public:
    struct Config {
        i2c_master_bus_handle_t i2c_bus;  // I2C bus handle (must be initialized)
        uint8_t i2c_addr = BMP280_I2C_ADDR_DEFAULT;
        BMP280Mode mode = BMP280Mode::NORMAL;
        BMP280Oversampling press_os = BMP280Oversampling::X4;
        BMP280Oversampling temp_os = BMP280Oversampling::X2;
        BMP280Standby standby = BMP280Standby::MS_62_5;
        BMP280Filter filter = BMP280Filter::COEF_4;
        float sea_level_pressure_pa = 101325.0f;  // Reference pressure
    };

    BMP280() = default;
    ~BMP280();

    /**
     * @brief Initialize BMP280
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read barometer data
     * @param data Barometer data output
     * @return ESP_OK on success
     */
    esp_err_t read(BaroData& data);

    /**
     * @brief Trigger forced measurement
     * @return ESP_OK on success
     */
    esp_err_t triggerMeasurement();

    /**
     * @brief Check if measurement is in progress
     * @return true if measuring
     */
    bool isMeasuring();

    /**
     * @brief Calculate altitude from pressure
     * @param pressure_pa Pressure in Pascal
     * @return Altitude in meters
     */
    float calculateAltitude(float pressure_pa) const;

    /**
     * @brief Set sea level reference pressure
     * @param pressure_pa Sea level pressure in Pascal
     */
    void setSeaLevelPressure(float pressure_pa);

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

    bool isInitialized() const { return initialized_; }

private:
    // Calibration data structure
    struct CalibData {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    };

    esp_err_t readRegister(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t writeRegister(uint8_t reg, uint8_t data);
    esp_err_t readCalibrationData();

    // Compensation functions (from datasheet)
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);

    bool initialized_ = false;
    Config config_;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    CalibData calib_{};
    int32_t t_fine_ = 0;  // Fine temperature for pressure compensation
    float sea_level_pressure_pa_ = 101325.0f;
};

}  // namespace stampfly
