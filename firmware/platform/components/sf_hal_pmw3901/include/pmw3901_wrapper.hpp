/**
 * @file pmw3901_wrapper.hpp
 * @brief C++ wrapper for PMW3901 Optical Flow Sensor
 *
 * This header provides a modern C++ interface for the PMW3901 sensor
 * with RAII support, exception-based error handling, and type safety.
 */

#ifndef PMW3901_WRAPPER_HPP
#define PMW3901_WRAPPER_HPP

#include <array>
#include <cstdint>
#include "pmw3901.h"
#include "pmw3901_exception.hpp"

namespace stampfly {

/**
 * @brief C++ wrapper class for PMW3901 optical flow sensor
 *
 * This class provides a modern C++ interface with:
 * - RAII (Resource Acquisition Is Initialization)
 * - Exception-based error handling
 * - Move semantics (no copy)
 * - Type-safe structs for return values
 *
 * @example
 * @code
 * stampfly::PMW3901 sensor;  // Auto-initialization
 * auto motion = sensor.readMotion();
 * // Auto cleanup on scope exit
 * @endcode
 */
class PMW3901 {
public:
    /**
     * @brief Configuration structure for PMW3901
     */
    struct Config {
        gpio_num_t pin_miso;          ///< MISO pin
        gpio_num_t pin_mosi;          ///< MOSI pin
        gpio_num_t pin_sclk;          ///< SCLK pin
        gpio_num_t pin_cs;            ///< Chip select pin
        spi_host_device_t spi_host;   ///< SPI host (SPI2_HOST or SPI3_HOST)
        bool skip_bus_init;           ///< Skip SPI bus init (if shared with another device)

        /**
         * @brief Get default configuration for StampFly
         *
         * @return Config Default configuration with StampFly pins
         */
        static Config defaultStampFly() {
            Config config;
            config.pin_miso = GPIO_NUM_43;
            config.pin_mosi = GPIO_NUM_14;
            config.pin_sclk = GPIO_NUM_44;
            config.pin_cs = GPIO_NUM_12;
            config.spi_host = SPI2_HOST;
            config.skip_bus_init = true;  // BMI270 already initialized SPI bus
            return config;
        }
    };

    /**
     * @brief Motion data structure (delta X and Y)
     */
    struct MotionData {
        int16_t delta_x;  ///< X displacement (pixels)
        int16_t delta_y;  ///< Y displacement (pixels)
    };

    /**
     * @brief Motion burst data structure (all sensor data)
     */
    struct MotionBurst {
        int16_t delta_x;        ///< X displacement (pixels)
        int16_t delta_y;        ///< Y displacement (pixels)
        uint8_t squal;          ///< Surface quality (0-255)
        uint16_t shutter;       ///< Shutter value
        uint8_t raw_data_sum;   ///< Raw data sum
        uint8_t motion;         ///< Motion register value
        uint8_t observation;    ///< Observation register value
        uint8_t max_raw_data;   ///< Maximum raw data
        uint8_t min_raw_data;   ///< Minimum raw data
    };


    /**
     * @brief Construct and initialize PMW3901 sensor
     *
     * @param config Configuration (defaults to StampFly pins)
     * @throws PMW3901Exception if initialization fails
     */
    explicit PMW3901(const Config& config = Config::defaultStampFly());

    /**
     * @brief Destructor - automatically cleanup sensor resources
     */
    ~PMW3901();

    /**
     * @brief Copy constructor (deleted - SPI handle cannot be copied)
     */
    PMW3901(const PMW3901&) = delete;

    /**
     * @brief Copy assignment (deleted - SPI handle cannot be copied)
     */
    PMW3901& operator=(const PMW3901&) = delete;

    /**
     * @brief Move constructor
     *
     * @param other Object to move from
     */
    PMW3901(PMW3901&& other) noexcept;

    /**
     * @brief Move assignment operator
     *
     * @param other Object to move from
     * @return Reference to this object
     */
    PMW3901& operator=(PMW3901&& other) noexcept;

    /**
     * @brief Get product ID
     *
     * @return uint8_t Product ID (should be 0x49)
     * @throws PMW3901Exception if read fails
     */
    uint8_t getProductId() const;

    /**
     * @brief Get revision ID
     *
     * @return uint8_t Revision ID
     * @throws PMW3901Exception if read fails
     */
    uint8_t getRevisionId() const;

    /**
     * @brief Read motion data (delta X and Y)
     *
     * @return MotionData X and Y displacement
     * @throws PMW3901Exception if read fails
     */
    MotionData readMotion();

    /**
     * @brief Read motion burst data (all sensor data in one SPI transaction)
     *
     * This is more efficient than reading individual registers.
     *
     * @return MotionBurst Complete motion data with quality metrics
     * @throws PMW3901Exception if read fails
     */
    MotionBurst readMotionBurst();

    /**
     * @brief Check if motion is detected
     *
     * @return true if motion detected, false otherwise
     * @throws PMW3901Exception if read fails
     */
    bool isMotionDetected();

    /* Note: Velocity calculation methods have been removed.
     * Use ESKF::updateFlowRaw() for proper velocity estimation.
     * See: components/stampfly_eskf/include/eskf.hpp
     */

    /**
     * @brief Enable frame capture mode
     *
     * @throws PMW3901Exception if operation fails
     */
    void enableFrameCapture();

    /**
     * @brief Read image frame (35x35 pixels, 8-bit grayscale)
     *
     * @return std::array<uint8_t, 1225> Frame data (35*35 = 1225 pixels)
     * @throws PMW3901Exception if read fails
     */
    std::array<uint8_t, PMW3901_FRAME_SIZE> readFrame();

    /**
     * @brief Read a register value (for debugging)
     *
     * @param reg Register address
     * @return uint8_t Register value
     * @throws PMW3901Exception if read fails
     */
    uint8_t readRegister(uint8_t reg) const;

    /**
     * @brief Write a register value (for debugging)
     *
     * @param reg Register address
     * @param value Value to write
     * @throws PMW3901Exception if write fails
     */
    void writeRegister(uint8_t reg, uint8_t value);

private:
    pmw3901_t device_;     ///< C device handle
    bool initialized_;     ///< Initialization status

    /**
     * @brief Check if device is initialized
     *
     * @throws PMW3901Exception if not initialized
     */
    void checkInitialized() const;
};

} // namespace stampfly

#endif // PMW3901_WRAPPER_HPP
