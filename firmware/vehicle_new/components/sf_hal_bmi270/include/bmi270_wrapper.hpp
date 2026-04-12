/**
 * @file bmi270_wrapper.hpp
 * @brief C++ wrapper for BMI270 IMU Sensor
 *
 * This header provides a modern C++ interface for the BMI270 sensor
 * with RAII support, type safety, and ESP-IDF error handling.
 */

#ifndef BMI270_WRAPPER_HPP
#define BMI270_WRAPPER_HPP

#include <cstdint>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// C driver headers
extern "C" {
#include "bmi270_types.h"
#include "bmi270_init.h"
#include "bmi270_spi.h"
#include "bmi270_data.h"
#include "bmi270_interrupt.h"
#include "bmi270_defs.h"
}

namespace stampfly {

/**
 * @brief Accelerometer data structure
 */
struct AccelData {
    float x;  ///< X-axis acceleration [g]
    float y;  ///< Y-axis acceleration [g]
    float z;  ///< Z-axis acceleration [g]
};

/**
 * @brief Gyroscope data structure
 */
struct GyroData {
    float x;  ///< X-axis angular velocity [rad/s]
    float y;  ///< Y-axis angular velocity [rad/s]
    float z;  ///< Z-axis angular velocity [rad/s]
};

/**
 * @brief FIFO frame data structure
 */
struct FIFOFrame {
    AccelData accel;  ///< Accelerometer data
    GyroData gyro;    ///< Gyroscope data
    bool has_accel;   ///< True if frame contains accel data
    bool has_gyro;    ///< True if frame contains gyro data
};

/**
 * @brief C++ wrapper class for BMI270 IMU sensor
 *
 * This class provides a modern C++ interface with:
 * - RAII (Resource Acquisition Is Initialization)
 * - ESP-IDF error handling (esp_err_t)
 * - Move semantics (no copy)
 * - FIFO support for high-speed data acquisition
 */
class BMI270Wrapper {
public:
    /**
     * @brief Configuration structure for BMI270
     */
    struct Config {
        gpio_num_t pin_mosi;          ///< MOSI pin
        gpio_num_t pin_miso;          ///< MISO pin
        gpio_num_t pin_sclk;          ///< SCLK pin
        gpio_num_t pin_cs;            ///< Chip select pin
        spi_host_device_t spi_host;   ///< SPI host (SPI2_HOST or SPI3_HOST)
        uint32_t spi_clock_hz;        ///< SPI clock frequency (max 10MHz)
        gpio_num_t other_cs;          ///< CS pin of other device on shared SPI bus (-1 if not used)

        // Sensor configuration
        bmi270_acc_range_t accel_range;   ///< Accelerometer range
        bmi270_gyr_range_t gyro_range;    ///< Gyroscope range
        bmi270_acc_odr_t accel_odr;       ///< Accelerometer ODR
        bmi270_gyr_odr_t gyro_odr;        ///< Gyroscope ODR
        bmi270_filter_perf_t filter_mode; ///< Filter performance mode

        /**
         * @brief Get default configuration for StampFly
         * @return Config Default configuration with StampFly pins
         */
        static Config defaultStampFly() {
            Config config;
            config.pin_mosi = GPIO_NUM_14;
            config.pin_miso = GPIO_NUM_43;
            config.pin_sclk = GPIO_NUM_44;
            config.pin_cs = GPIO_NUM_46;
            config.spi_host = SPI2_HOST;
            config.spi_clock_hz = 10000000;  // 10MHz
            config.other_cs = GPIO_NUM_12;   // PMW3901 CS

            // Default sensor settings for drone control
            config.accel_range = BMI270_ACC_RANGE_8G;
            config.gyro_range = BMI270_GYR_RANGE_2000DPS;
            config.accel_odr = BMI270_ACC_ODR_1600HZ;
            config.gyro_odr = BMI270_GYR_ODR_1600HZ;
            config.filter_mode = BMI270_FILTER_PERFORMANCE;
            return config;
        }
    };

    /**
     * @brief Default constructor (uninitialized)
     */
    BMI270Wrapper() = default;

    /**
     * @brief Destructor - automatically cleanup sensor resources
     */
    ~BMI270Wrapper();

    /**
     * @brief Copy constructor (deleted - SPI handle cannot be copied)
     */
    BMI270Wrapper(const BMI270Wrapper&) = delete;

    /**
     * @brief Copy assignment (deleted - SPI handle cannot be copied)
     */
    BMI270Wrapper& operator=(const BMI270Wrapper&) = delete;

    /**
     * @brief Move constructor
     */
    BMI270Wrapper(BMI270Wrapper&& other) noexcept;

    /**
     * @brief Move assignment operator
     */
    BMI270Wrapper& operator=(BMI270Wrapper&& other) noexcept;

    /**
     * @brief Initialize BMI270 sensor
     *
     * @param config Configuration parameters
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init(const Config& config = Config::defaultStampFly());

    /**
     * @brief Read accelerometer and gyroscope data simultaneously
     *
     * @param accel Output accelerometer data [g]
     * @param gyro Output gyroscope data [rad/s]
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t readSensorData(AccelData& accel, GyroData& gyro);

    /**
     * @brief Read accelerometer data only
     *
     * @param accel Output accelerometer data [g]
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t readAccel(AccelData& accel);

    /**
     * @brief Read gyroscope data only
     *
     * @param gyro Output gyroscope data [rad/s]
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t readGyro(GyroData& gyro);

    /**
     * @brief Read temperature
     *
     * @param temperature Output temperature [°C]
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t readTemperature(float& temperature);

    // ====== FIFO Functions ======

    /**
     * @brief Configure FIFO
     *
     * @param enable_accel Enable accelerometer data in FIFO
     * @param enable_gyro Enable gyroscope data in FIFO
     * @param watermark_frames Watermark level in frames (interrupt when reached)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t configureFIFO(bool enable_accel, bool enable_gyro, uint16_t watermark_frames = 4);

    /**
     * @brief Read FIFO data
     *
     * @param buffer Output buffer for FIFO frames
     * @param max_frames Maximum frames to read
     * @param frames_read Output: actual frames read
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t readFIFO(FIFOFrame* buffer, size_t max_frames, size_t& frames_read);

    /**
     * @brief Get current FIFO length in bytes
     *
     * @param length Output: FIFO length
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t getFIFOLength(uint16_t& length);

    /**
     * @brief Flush FIFO
     *
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t flushFIFO();

    // ====== Interrupt Functions ======

    /**
     * @brief Enable data ready interrupt
     *
     * @param int_pin Interrupt pin (INT1 or INT2)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t enableDataReadyInterrupt(bmi270_int_pin_t int_pin = BMI270_INT_PIN_1);

    /**
     * @brief Enable FIFO watermark interrupt
     *
     * @param int_pin Interrupt pin (INT1 or INT2)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t enableFIFOWatermarkInterrupt(bmi270_int_pin_t int_pin = BMI270_INT_PIN_1);

    /**
     * @brief Disable all interrupts on specified pin
     *
     * @param int_pin Interrupt pin (INT1 or INT2)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t disableInterrupt(bmi270_int_pin_t int_pin);

    // ====== Configuration Functions ======

    /**
     * @brief Set accelerometer range
     *
     * @param range Accelerometer range
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setAccelRange(bmi270_acc_range_t range);

    /**
     * @brief Set gyroscope range
     *
     * @param range Gyroscope range
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setGyroRange(bmi270_gyr_range_t range);

    /**
     * @brief Set accelerometer ODR and filter mode
     *
     * @param odr Output data rate
     * @param filter_perf Filter performance mode
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setAccelConfig(bmi270_acc_odr_t odr, bmi270_filter_perf_t filter_perf);

    /**
     * @brief Set gyroscope ODR and filter mode
     *
     * @param odr Output data rate
     * @param filter_perf Filter performance mode
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setGyroConfig(bmi270_gyr_odr_t odr, bmi270_filter_perf_t filter_perf);

    /**
     * @brief Check if sensor is initialized
     *
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get raw C device handle (for advanced use)
     *
     * @return Pointer to C device structure
     */
    bmi270_dev_t* getDeviceHandle() { return &device_; }

private:
    bmi270_dev_t device_;      ///< C device handle
    Config config_;            ///< Current configuration
    bool initialized_ = false; ///< Initialization status
    bool fifo_accel_enabled_ = false;
    bool fifo_gyro_enabled_ = false;

    /**
     * @brief Parse FIFO frame header
     */
    bool parseFIFOFrame(const uint8_t* data, size_t& offset, size_t max_len, FIFOFrame& frame);
};

}  // namespace stampfly

#endif // BMI270_WRAPPER_HPP
