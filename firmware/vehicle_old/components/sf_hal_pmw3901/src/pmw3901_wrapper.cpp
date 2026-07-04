/**
 * @file pmw3901_wrapper.cpp
 * @brief Implementation of C++ wrapper for PMW3901 Optical Flow Sensor
 */

#include "pmw3901_wrapper.hpp"
#include <cstring>
#include <utility>

namespace stampfly {

// Constructor
PMW3901::PMW3901(const Config& config)
    : initialized_(false)
{
    // Convert C++ config to C config
    pmw3901_config_t c_config;
    c_config.spi_host = config.spi_host;
    c_config.pin_miso = config.pin_miso;
    c_config.pin_mosi = config.pin_mosi;
    c_config.pin_sclk = config.pin_sclk;
    c_config.pin_cs = config.pin_cs;
    c_config.skip_bus_init = config.skip_bus_init;

    // Initialize the sensor
    esp_err_t ret = pmw3901_init(&device_, &c_config);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::INIT_FAILED,
            ret,
            "Failed to initialize PMW3901 sensor"
        );
    }

    initialized_ = true;
}

// Destructor
PMW3901::~PMW3901()
{
    if (initialized_) {
        pmw3901_deinit(&device_);
        initialized_ = false;
    }
}

// Move constructor
PMW3901::PMW3901(PMW3901&& other) noexcept
    : device_(other.device_)
    , initialized_(other.initialized_)
{
    other.initialized_ = false;
}

// Move assignment operator
PMW3901& PMW3901::operator=(PMW3901&& other) noexcept
{
    if (this != &other) {
        // Clean up current resources
        if (initialized_) {
            pmw3901_deinit(&device_);
        }

        // Move from other
        device_ = other.device_;
        initialized_ = other.initialized_;

        // Invalidate other
        other.initialized_ = false;
    }
    return *this;
}

// Check if device is initialized
void PMW3901::checkInitialized() const
{
    if (!initialized_) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::DEVICE_NOT_INITIALIZED,
            ESP_ERR_INVALID_STATE,
            "PMW3901 device is not initialized"
        );
    }
}

// Get product ID
uint8_t PMW3901::getProductId() const
{
    checkInitialized();

    uint8_t product_id;
    esp_err_t ret = pmw3901_get_product_id(const_cast<pmw3901_t*>(&device_), &product_id);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::READ_ERROR,
            ret,
            "Failed to read product ID"
        );
    }

    return product_id;
}

// Get revision ID
uint8_t PMW3901::getRevisionId() const
{
    checkInitialized();

    uint8_t revision_id;
    esp_err_t ret = pmw3901_get_revision_id(const_cast<pmw3901_t*>(&device_), &revision_id);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::READ_ERROR,
            ret,
            "Failed to read revision ID"
        );
    }

    return revision_id;
}

// Read motion data
PMW3901::MotionData PMW3901::readMotion()
{
    checkInitialized();

    int16_t delta_x, delta_y;
    esp_err_t ret = pmw3901_read_motion(&device_, &delta_x, &delta_y);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::MOTION_READ_ERROR,
            ret,
            "Failed to read motion data"
        );
    }

    return MotionData{delta_x, delta_y};
}

// Read motion burst data
PMW3901::MotionBurst PMW3901::readMotionBurst()
{
    checkInitialized();

    pmw3901_motion_burst_t c_burst;
    esp_err_t ret = pmw3901_read_motion_burst(&device_, &c_burst);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::MOTION_READ_ERROR,
            ret,
            "Failed to read motion burst data"
        );
    }

    MotionBurst result;
    result.delta_x = c_burst.delta_x;
    result.delta_y = c_burst.delta_y;
    result.squal = c_burst.squal;
    result.shutter = c_burst.shutter;
    result.raw_data_sum = c_burst.raw_data_sum;
    result.motion = c_burst.motion;
    result.observation = c_burst.observation;
    result.max_raw_data = c_burst.max_raw_data;
    result.min_raw_data = c_burst.min_raw_data;
    return result;
}

// Check if motion is detected
bool PMW3901::isMotionDetected()
{
    checkInitialized();

    bool motion_detected;
    esp_err_t ret = pmw3901_is_motion_detected(&device_, &motion_detected);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::READ_ERROR,
            ret,
            "Failed to check motion detection"
        );
    }

    return motion_detected;
}

/* Velocity calculation methods removed.
 * Use ESKF::updateFlowRaw() for proper velocity estimation.
 */

// Enable frame capture
void PMW3901::enableFrameCapture()
{
    checkInitialized();

    esp_err_t ret = pmw3901_enable_frame_capture(&device_);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::FRAME_CAPTURE_ERROR,
            ret,
            "Failed to enable frame capture"
        );
    }
}

// Read frame
std::array<uint8_t, PMW3901_FRAME_SIZE> PMW3901::readFrame()
{
    checkInitialized();

    std::array<uint8_t, PMW3901_FRAME_SIZE> frame;
    esp_err_t ret = pmw3901_read_frame(&device_, frame.data());
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::FRAME_CAPTURE_ERROR,
            ret,
            "Failed to read frame"
        );
    }

    return frame;
}

// Read register
uint8_t PMW3901::readRegister(uint8_t reg) const
{
    checkInitialized();

    uint8_t value;
    esp_err_t ret = pmw3901_read_register(const_cast<pmw3901_t*>(&device_), reg, &value);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::READ_ERROR,
            ret,
            "Failed to read register"
        );
    }

    return value;
}

// Write register
void PMW3901::writeRegister(uint8_t reg, uint8_t value)
{
    checkInitialized();

    esp_err_t ret = pmw3901_write_register(&device_, reg, value);
    if (ret != ESP_OK) {
        throw PMW3901Exception(
            PMW3901Exception::ErrorCode::WRITE_ERROR,
            ret,
            "Failed to write register"
        );
    }
}

} // namespace stampfly
