/**
 * @file system_manager.hpp
 * @brief System Manager
 *
 * Event group synchronization, initialization coordination
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace stampfly {

// Event bits
constexpr uint32_t EVENT_IMU_READY     = (1 << 0);
constexpr uint32_t EVENT_MAG_READY     = (1 << 1);
constexpr uint32_t EVENT_BARO_READY    = (1 << 2);
constexpr uint32_t EVENT_TOF_READY     = (1 << 3);
constexpr uint32_t EVENT_FLOW_READY    = (1 << 4);
constexpr uint32_t EVENT_POWER_READY   = (1 << 5);
constexpr uint32_t EVENT_COMM_READY    = (1 << 6);
constexpr uint32_t EVENT_CALIB_DONE    = (1 << 7);

constexpr uint32_t EVENT_ALL_SENSORS = EVENT_IMU_READY | EVENT_MAG_READY |
                                       EVENT_BARO_READY | EVENT_TOF_READY |
                                       EVENT_FLOW_READY | EVENT_POWER_READY;

class SystemManager {
public:
    struct Config {
        uint32_t init_timeout_ms;
        uint32_t calib_timeout_ms;
    };

    static SystemManager& getInstance();

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    esp_err_t init(const Config& config);
    esp_err_t start();

    /**
     * @brief Wait for events
     * @param events Event bits to wait for
     * @param timeout_ms Timeout in milliseconds
     * @return true if all events occurred
     */
    bool waitForEvents(uint32_t events, uint32_t timeout_ms);

    /**
     * @brief Set event
     * @param event Event bit to set
     */
    void setEvent(uint32_t event);

    /**
     * @brief Clear event
     * @param event Event bit to clear
     */
    void clearEvent(uint32_t event);

    /**
     * @brief Run calibration sequence
     * @return ESP_OK on success
     */
    esp_err_t runCalibration();

    /**
     * @brief Run gyro calibration
     * @return ESP_OK on success
     */
    esp_err_t runGyroCalibration();

    /**
     * @brief Run magnetometer calibration
     * @return ESP_OK on success
     */
    esp_err_t runMagCalibration();

private:
    SystemManager() = default;

    EventGroupHandle_t event_group_ = nullptr;
    Config config_;
    bool initialized_ = false;
};

}  // namespace stampfly
