/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file vl53l3cx_wrapper.hpp
 * @brief C++ wrapper for VL53L3CX ToF Sensor
 *
 * This header provides a modern C++ interface for the VL53L3CX ToF sensor
 * with RAII support, type safety, and dual sensor management for StampFly.
 */

#ifndef VL53L3CX_WRAPPER_HPP
#define VL53L3CX_WRAPPER_HPP

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "tof_histogram_merge.hpp"

// C driver headers
extern "C" {
#include "vl53lx_api.h"
#include "vl53lx_platform.h"
}

namespace stampfly {

/**
 * @brief ToF sensor location
 */
enum class ToFLocation {
    BOTTOM,  ///< Bottom sensor (altitude)
    FRONT    ///< Front sensor (obstacle detection)
};

/**
 * @brief Distance measurement data
 */
struct DistanceData {
    int16_t distance_mm;      ///< Distance in millimeters
    uint8_t range_status;     ///< Range status (0 = valid)
    uint8_t num_objects;      ///< Number of detected objects
    float signal_rate;        ///< Signal rate (MCPS)
    float ambient_rate;       ///< Ambient rate (MCPS)
    float sigma_mm;           ///< Sigma in mm (measurement uncertainty)
    uint32_t timestamp;       ///< Timestamp (stream count)
};

/**
 * @brief C++ wrapper class for VL53L3CX ToF sensor
 *
 * This class provides:
 * - RAII (Resource Acquisition Is Initialization)
 * - ESP-IDF error handling (esp_err_t)
 * - Dual sensor management (front and bottom)
 * - XSHUT control for address assignment
 */
class VL53L3CXWrapper {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x29;
    static constexpr uint8_t BOTTOM_I2C_ADDR = 0x30;
    static constexpr uint8_t FRONT_I2C_ADDR = 0x31;

    /**
     * @brief Configuration structure for VL53L3CX
     */
    struct Config {
        i2c_master_bus_handle_t i2c_bus;  ///< I2C bus handle (must be initialized)
        gpio_num_t xshut_pin;             ///< XSHUT pin for this sensor
        uint8_t i2c_addr;                 ///< Target I2C address
        ToFLocation location;             ///< Sensor location
        uint32_t timing_budget_ms;        ///< Timing budget in ms
        /// Histogram merge of the ST driver (true = driver default, on; false = off). Held per sensor, like the other
        /// per-sensor settings here (timing_budget_ms), so one sensor can be changed without the other. Applied once in
        /// init(), before ranging starts, and again at every re-initialisation. See tof_histogram_merge.hpp.
        /// ST ドライバのヒストグラム併合（true = ドライバ既定の有効、false = 無効）。init() で測距の開始前に 1 回掛け、
        /// 再初期化のたびに掛け直す。tof_histogram_merge.hpp を参照。
        bool histogram_merge_enabled;

        /**
         * @brief Get default configuration for bottom sensor
         */
        static Config defaultBottom(i2c_master_bus_handle_t bus) {
            Config config;
            config.i2c_bus = bus;
            config.xshut_pin = GPIO_NUM_7;
            config.i2c_addr = BOTTOM_I2C_ADDR;
            config.location = ToFLocation::BOTTOM;
            config.timing_budget_ms = 33;  // 30Hz
            // Merging off: no ~220 ms staleness and no multi-centimetre jump for the height estimator.
            // PAIRED with the altitude gains (params.cpp: altitude.alt.kp 0.85, altitude.vel.kp 0.12,
            // altitude.vel.ti_hover 2.0), which were chosen for the ~50-75 ms reading age of merge off; with merging
            // ON they leave a phase margin of only ~28 degrees (model). Do not turn merging on without redoing them.
            // 併合なし: 高さの推定にとって、約 220 ms の古さと数 cm の跳びが無くなる。
            // 高さのゲイン（params.cpp: altitude.alt.kp 0.85、altitude.vel.kp 0.12、altitude.vel.ti_hover 2.0）と対:
            // ゲインは併合なしの読みの古さ約 50〜75 ms に合わせて選んだ値で、併合ありでは位相余裕が約 28 度しか
            // 残らない（モデル）。併合を有効にするときは、ゲインを選び直すこと。
            config.histogram_merge_enabled = false;
            return config;
        }

        /**
         * @brief Get default configuration for front sensor
         */
        static Config defaultFront(i2c_master_bus_handle_t bus) {
            Config config;
            config.i2c_bus = bus;
            config.xshut_pin = GPIO_NUM_9;
            config.i2c_addr = FRONT_I2C_ADDR;
            config.location = ToFLocation::FRONT;
            config.timing_budget_ms = 33;  // 30Hz
            // Merging off, the user's decision (2026-11-23), same as the bottom sensor. On hardware merging off was
            // checked only for the downward sensor up to 1.5 m: the share of valid readings and the noise of the front
            // sensor at 1-3 m are NOT yet confirmed (merging used to support the long range).
            // 併合なし。利用者の決定（2026-11-23）で下向きと同じ。実機で併合なしを確かめたのは下向きの 1.5 m までで、
            // 前方の 1〜3 m での有効な読みの割合と雑音は未確認（併合は遠距離を支えていた）。
            config.histogram_merge_enabled = false;
            return config;
        }
    };

    /**
     * @brief Distance mode
     */
    enum class DistanceMode {
        SHORT = VL53LX_DISTANCEMODE_SHORT,    ///< Short range (up to 1.3m)
        MEDIUM = VL53LX_DISTANCEMODE_MEDIUM,  ///< Medium range (up to 3m)
        LONG = VL53LX_DISTANCEMODE_LONG       ///< Long range (up to 4m)
    };

    /**
     * @brief Default constructor (uninitialized)
     */
    VL53L3CXWrapper() = default;

    /**
     * @brief Destructor
     */
    ~VL53L3CXWrapper();

    /**
     * @brief Copy constructor (deleted)
     */
    VL53L3CXWrapper(const VL53L3CXWrapper&) = delete;

    /**
     * @brief Copy assignment (deleted)
     */
    VL53L3CXWrapper& operator=(const VL53L3CXWrapper&) = delete;

    /**
     * @brief Move constructor
     */
    VL53L3CXWrapper(VL53L3CXWrapper&& other) noexcept;

    /**
     * @brief Move assignment operator
     */
    VL53L3CXWrapper& operator=(VL53L3CXWrapper&& other) noexcept;

    /**
     * @brief Initialize single VL53L3CX sensor
     *
     * This function:
     * 1. Controls XSHUT to reset the sensor
     * 2. Waits for boot
     * 3. Changes I2C address if needed
     * 4. Initializes the sensor
     *
     * @param config Configuration parameters
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Give back the I2C device handle and clear the state, so init() can run again
     *
     * Makes no I2C transaction (the XSHUT pulse of the next init() stops the sensor).
     * I2C 通信はしない（次の init() の XSHUT パルスがセンサを止める）。
     */
    void release();

    /**
     * @brief Start continuous ranging
     *
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t startRanging();

    /**
     * @brief Stop ranging
     *
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t stopRanging();

    /**
     * @brief Check if measurement data is ready
     *
     * @param ready Output: true if data is ready
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t isDataReady(bool& ready);

    /**
     * @brief Get distance measurement
     *
     * @param data Output: distance data
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t getDistance(DistanceData& data);

    /**
     * @brief Get distance in millimeters (simple interface)
     *
     * @param distance_mm Output: distance in mm
     * @param status Output: range status (0 = valid)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t getDistance(uint16_t& distance_mm, uint8_t& status);

    /**
     * @brief Clear interrupt and start next measurement
     *
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t clearInterruptAndStartMeasurement();

    /**
     * @brief Set distance mode
     *
     * @param mode Distance mode
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setDistanceMode(DistanceMode mode);

    /**
     * @brief Set timing budget
     *
     * @param budget_ms Timing budget in milliseconds
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t setTimingBudget(uint32_t budget_ms);

    /**
     * @brief Histogram merge setting in force (true = driver default, on)
     * 有効なヒストグラム併合の設定（true = ドライバ既定の有効）
     */
    bool histogramMergeEnabled() const { return config_.histogram_merge_enabled; }

    /**
     * @brief Get sensor location
     *
     * @return Sensor location (BOTTOM or FRONT)
     */
    ToFLocation getLocation() const { return config_.location; }

    /**
     * @brief Check if sensor is initialized
     *
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get raw C device handle (for advanced use)
     *
     * @return Pointer to VL53LX device structure
     */
    VL53LX_Dev_t* getDeviceHandle() { return &device_; }

    // ====== Static Helper Functions for Dual Sensor Setup ======

    /**
     * @brief Initialize dual ToF sensors (bottom and front)
     *
     * This is a convenience function that properly sequences the
     * initialization of both sensors with XSHUT control.
     *
     * @param bottom_sensor Output: bottom sensor wrapper
     * @param front_sensor Output: front sensor wrapper
     * @param i2c_bus I2C bus handle
     * @param bottom_xshut Bottom sensor XSHUT pin (default: GPIO7)
     * @param front_xshut Front sensor XSHUT pin (default: GPIO9)
     * @return ESP_OK on success, error code otherwise
     */
    static esp_err_t initDualSensors(
        VL53L3CXWrapper& bottom_sensor,
        VL53L3CXWrapper& front_sensor,
        i2c_master_bus_handle_t i2c_bus,
        gpio_num_t bottom_xshut = GPIO_NUM_7,
        gpio_num_t front_xshut = GPIO_NUM_9
    );

private:
    VL53LX_Dev_t device_;        ///< VL53LX device structure
    Config config_;              ///< Current configuration
    bool initialized_ = false;   ///< Initialization status
    bool ranging_ = false;       ///< Ranging active status
    i2c_master_dev_handle_t i2c_dev_handle_ = nullptr;

    /**
     * @brief Apply config_.histogram_merge_enabled to the driver and read it back (no I2C traffic)
     * config_.histogram_merge_enabled をドライバに掛け、読み戻して確かめる（I2C 通信なし）
     *
     * Leaves the driver default untouched when merging is enabled. A set or read-back failure is a bring-up failure.
     * 併合が有効のときはドライバの既定に触れない。設定・読み戻しの失敗は起動（立ち上げ）の失敗。
     */
    esp_err_t applyHistogramMerge();

    /**
     * @brief Remove the I2C device handle (if any) and forget it
     */
    void releaseI2cDevice();

    /**
     * @brief Control XSHUT pin
     */
    esp_err_t setXshut(bool enable);

    /**
     * @brief Wait for device boot
     */
    esp_err_t waitDeviceBoot();
};

}  // namespace stampfly

#endif // VL53L3CX_WRAPPER_HPP
