/**
 * @file pmw3901_exception.hpp
 * @brief Exception classes for PMW3901 C++ wrapper
 */

#ifndef PMW3901_EXCEPTION_HPP
#define PMW3901_EXCEPTION_HPP

#include <stdexcept>
#include <string>
#include "esp_err.h"

namespace stampfly {

/**
 * @brief Exception class for PMW3901 errors
 *
 * This exception is thrown when PMW3901 operations fail.
 * It contains both a high-level error code and the underlying ESP-IDF error code.
 */
class PMW3901Exception : public std::runtime_error {
public:
    /**
     * @brief Error codes for PMW3901 operations
     */
    enum class ErrorCode {
        INIT_FAILED,              ///< Sensor initialization failed
        SPI_ERROR,                ///< SPI communication error
        READ_ERROR,               ///< Register read error
        WRITE_ERROR,              ///< Register write error
        INVALID_PRODUCT_ID,       ///< Product ID verification failed
        DEVICE_NOT_INITIALIZED,   ///< Device not initialized before use
        INVALID_PARAMETER,        ///< Invalid parameter passed
        FRAME_CAPTURE_ERROR,      ///< Frame capture failed
        MOTION_READ_ERROR         ///< Motion data read failed
    };

    /**
     * @brief Construct a new PMW3901Exception
     *
     * @param code High-level error code
     * @param esp_error ESP-IDF error code
     * @param message Error message
     */
    PMW3901Exception(ErrorCode code, esp_err_t esp_error, const std::string& message)
        : std::runtime_error(message)
        , code_(code)
        , esp_error_(esp_error)
    {}

    /**
     * @brief Get the error code
     *
     * @return ErrorCode High-level error code
     */
    ErrorCode code() const noexcept {
        return code_;
    }

    /**
     * @brief Get the ESP-IDF error code
     *
     * @return esp_err_t ESP-IDF error code
     */
    esp_err_t espError() const noexcept {
        return esp_error_;
    }

    /**
     * @brief Convert error code to string
     *
     * @param code Error code to convert
     * @return const char* String representation
     */
    static const char* errorCodeToString(ErrorCode code) {
        switch (code) {
            case ErrorCode::INIT_FAILED:
                return "Initialization failed";
            case ErrorCode::SPI_ERROR:
                return "SPI communication error";
            case ErrorCode::READ_ERROR:
                return "Register read error";
            case ErrorCode::WRITE_ERROR:
                return "Register write error";
            case ErrorCode::INVALID_PRODUCT_ID:
                return "Invalid product ID";
            case ErrorCode::DEVICE_NOT_INITIALIZED:
                return "Device not initialized";
            case ErrorCode::INVALID_PARAMETER:
                return "Invalid parameter";
            case ErrorCode::FRAME_CAPTURE_ERROR:
                return "Frame capture error";
            case ErrorCode::MOTION_READ_ERROR:
                return "Motion read error";
            default:
                return "Unknown error";
        }
    }

private:
    ErrorCode code_;
    esp_err_t esp_error_;
};

} // namespace stampfly

#endif // PMW3901_EXCEPTION_HPP
