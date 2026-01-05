/**
 * @file bmm150.hpp
 * @brief BMM150 Magnetometer Driver
 *
 * I2C communication, XYZ magnetic field data
 * Reference: BMM150 Datasheet
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

namespace stampfly {

// BMM150 I2C Address
constexpr uint8_t BMM150_I2C_ADDR_DEFAULT = 0x10;  // SDO = GND
constexpr uint8_t BMM150_I2C_ADDR_ALT = 0x11;      // SDO = VDD
constexpr uint8_t BMM150_I2C_ADDR_ALT2 = 0x12;    // SDO = SDA
constexpr uint8_t BMM150_I2C_ADDR_ALT3 = 0x13;    // SDO = SCL

// Chip ID
constexpr uint8_t BMM150_CHIP_ID = 0x32;

// Register addresses
namespace bmm150_reg {
    constexpr uint8_t CHIP_ID = 0x40;
    constexpr uint8_t DATA_X_LSB = 0x42;
    constexpr uint8_t DATA_X_MSB = 0x43;
    constexpr uint8_t DATA_Y_LSB = 0x44;
    constexpr uint8_t DATA_Y_MSB = 0x45;
    constexpr uint8_t DATA_Z_LSB = 0x46;
    constexpr uint8_t DATA_Z_MSB = 0x47;
    constexpr uint8_t RHALL_LSB = 0x48;
    constexpr uint8_t RHALL_MSB = 0x49;
    constexpr uint8_t INT_STATUS = 0x4A;
    constexpr uint8_t POWER_CTRL = 0x4B;
    constexpr uint8_t OP_MODE = 0x4C;
    constexpr uint8_t INT_CTRL = 0x4D;
    constexpr uint8_t AXES_ENABLE = 0x4E;
    constexpr uint8_t REP_XY = 0x51;
    constexpr uint8_t REP_Z = 0x52;
    // Trim registers for compensation
    constexpr uint8_t DIG_X1 = 0x5D;
    constexpr uint8_t DIG_Y1 = 0x5E;
    constexpr uint8_t DIG_Z4_LSB = 0x62;
    constexpr uint8_t DIG_Z4_MSB = 0x63;
    constexpr uint8_t DIG_X2 = 0x64;
    constexpr uint8_t DIG_Y2 = 0x65;
    constexpr uint8_t DIG_Z2_LSB = 0x68;
    constexpr uint8_t DIG_Z2_MSB = 0x69;
    constexpr uint8_t DIG_Z1_LSB = 0x6A;
    constexpr uint8_t DIG_Z1_MSB = 0x6B;
    constexpr uint8_t DIG_XYZ1_LSB = 0x6C;
    constexpr uint8_t DIG_XYZ1_MSB = 0x6D;
    constexpr uint8_t DIG_Z3_LSB = 0x6E;
    constexpr uint8_t DIG_Z3_MSB = 0x6F;
    constexpr uint8_t DIG_XY2 = 0x70;
    constexpr uint8_t DIG_XY1 = 0x71;
}

// Power control modes
enum class BMM150PowerMode : uint8_t {
    SUSPEND = 0x00,
    NORMAL = 0x01,
};

// Operation modes
enum class BMM150OpMode : uint8_t {
    NORMAL = 0x00,
    FORCED = 0x01,
    SLEEP = 0x03,
};

// Data rate
enum class BMM150DataRate : uint8_t {
    ODR_10HZ = 0x00,
    ODR_2HZ = 0x01,
    ODR_6HZ = 0x02,
    ODR_8HZ = 0x03,
    ODR_15HZ = 0x04,
    ODR_20HZ = 0x05,
    ODR_25HZ = 0x06,
    ODR_30HZ = 0x07,
};

// Preset modes (repetitions for accuracy)
enum class BMM150Preset : uint8_t {
    LOW_POWER,      // XY: 3, Z: 3
    REGULAR,        // XY: 9, Z: 15
    ENHANCED,       // XY: 15, Z: 27
    HIGH_ACCURACY,  // XY: 47, Z: 83
};

struct MagData {
    float x;  // uT (micro Tesla)
    float y;
    float z;
    uint32_t timestamp_us;
    bool data_ready;
};

struct MagRawData {
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t rhall;
};

class BMM150 {
public:
    struct Config {
        i2c_master_bus_handle_t i2c_bus;  // I2C bus handle (must be initialized)
        uint8_t i2c_addr = BMM150_I2C_ADDR_DEFAULT;
        BMM150DataRate data_rate = BMM150DataRate::ODR_10HZ;
        BMM150Preset preset = BMM150Preset::REGULAR;
    };

    BMM150() = default;
    ~BMM150();

    /**
     * @brief Initialize BMM150
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read magnetometer data (compensated)
     * @param data Magnetometer data output in uT
     * @return ESP_OK on success
     */
    esp_err_t read(MagData& data);

    /**
     * @brief Read raw magnetometer data
     * @param raw Raw data output
     * @return ESP_OK on success
     */
    esp_err_t readRaw(MagRawData& raw);

    /**
     * @brief Set operation mode
     * @param mode Operation mode
     * @return ESP_OK on success
     */
    esp_err_t setOpMode(BMM150OpMode mode);

    /**
     * @brief Set data rate
     * @param rate Data rate
     * @return ESP_OK on success
     */
    esp_err_t setDataRate(BMM150DataRate rate);

    /**
     * @brief Set preset mode (affects accuracy/power)
     * @param preset Preset mode
     * @return ESP_OK on success
     */
    esp_err_t setPreset(BMM150Preset preset);

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

    /**
     * @brief Check if data is ready
     * @return true if new data available
     */
    bool isDataReady();

    bool isInitialized() const { return initialized_; }

private:
    // Trim data structure for compensation
    struct TrimData {
        int8_t dig_x1;
        int8_t dig_y1;
        int8_t dig_x2;
        int8_t dig_y2;
        uint16_t dig_z1;
        int16_t dig_z2;
        int16_t dig_z3;
        int16_t dig_z4;
        uint8_t dig_xy1;
        int8_t dig_xy2;
        uint16_t dig_xyz1;
    };

    esp_err_t readRegister(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t writeRegister(uint8_t reg, uint8_t data);
    esp_err_t readTrimData();

    // Compensation functions
    float compensateX(int16_t raw_x, uint16_t rhall);
    float compensateY(int16_t raw_y, uint16_t rhall);
    float compensateZ(int16_t raw_z, uint16_t rhall);

    bool initialized_ = false;
    Config config_;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    TrimData trim_data_{};
};

}  // namespace stampfly
