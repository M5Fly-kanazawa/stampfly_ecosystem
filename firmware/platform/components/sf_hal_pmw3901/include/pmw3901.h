/**
 * @file pmw3901.h
 * @brief PMW3901 Optical Flow Sensor Driver for StampFly (ESP32-S3)
 *
 * This driver provides an interface for the PixArt PMW3901MB-TXQT optical flow sensor
 * using SPI communication on ESP32-S3 based StampFly drone.
 *
 * Pin Configuration (StampFly):
 *   MISO: GPIO 43
 *   MOSI: GPIO 14
 *   SCLK: GPIO 44
 *   CS:   GPIO 12 (Note: GPIO 46 is BMI270 IMU CS)
 */

#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* StampFly Default Pin Configuration */
#define PMW3901_DEFAULT_PIN_MISO    43
#define PMW3901_DEFAULT_PIN_MOSI    14
#define PMW3901_DEFAULT_PIN_SCLK    44
#define PMW3901_DEFAULT_PIN_CS      12  // PMW3901 CS (Note: GPIO 46 is BMI270 IMU CS)

/* PMW3901 Register Addresses */
#define PMW3901_PRODUCT_ID              0x00
#define PMW3901_REVISION_ID             0x01
#define PMW3901_MOTION                  0x02
#define PMW3901_DELTA_X_L               0x03
#define PMW3901_DELTA_X_H               0x04
#define PMW3901_DELTA_Y_L               0x05
#define PMW3901_DELTA_Y_H               0x06
#define PMW3901_SQUAL                   0x07
#define PMW3901_RAW_DATA_SUM            0x08
#define PMW3901_MAXIMUM_RAW_DATA        0x09
#define PMW3901_MINIMUM_RAW_DATA        0x0A
#define PMW3901_SHUTTER_LOWER           0x0B
#define PMW3901_SHUTTER_UPPER           0x0C
#define PMW3901_OBSERVATION             0x15
#define PMW3901_MOTION_BURST            0x16
#define PMW3901_POWER_UP_RESET          0x3A
#define PMW3901_SHUTDOWN                0x3B
#define PMW3901_RAW_DATA_GRAB           0x58
#define PMW3901_RAW_DATA_GRAB_STATUS    0x59
#define PMW3901_INVERSE_PRODUCT_ID      0x5F
#define PMW3901_BANK_SELECT             0x7F

/* Expected values for chip verification */
#define PMW3901_PRODUCT_ID_VALUE        0x49
#define PMW3901_INVERSE_PRODUCT_ID_VALUE 0xB6

/* SPI Settings */
#define PMW3901_SPI_CLOCK_SPEED_HZ      2000000  // 2MHz (max 2MHz for PMW3901)
#define PMW3901_SPI_READ_DELAY_US       50       // Delay after each SPI read (μs)
#define PMW3901_SPI_WRITE_DELAY_US      50       // Delay after each SPI write (μs)

/* Timing constants */
#define PMW3901_POWER_UP_DELAY_MS       45       // Initial power-up delay
#define PMW3901_RESET_DELAY_MS          5        // Reset delay
#define PMW3901_CS_TOGGLE_DELAY_MS      1        // CS toggle delay during init

/* Frame capture constants */
#define PMW3901_FRAME_WIDTH             35       // Frame width in pixels
#define PMW3901_FRAME_HEIGHT            35       // Frame height in pixels
#define PMW3901_FRAME_SIZE              (PMW3901_FRAME_WIDTH * PMW3901_FRAME_HEIGHT)

/**
 * @brief Motion burst data structure
 */
typedef struct {
    uint8_t motion;          // Motion detected register
    uint8_t observation;     // Observation register
    int16_t delta_x;         // X displacement
    int16_t delta_y;         // Y displacement
    uint8_t squal;           // Surface quality
    uint8_t raw_data_sum;    // Raw data sum
    uint8_t max_raw_data;    // Maximum raw data
    uint8_t min_raw_data;    // Minimum raw data
    uint16_t shutter;        // Shutter value
} pmw3901_motion_burst_t;

/**
 * @brief PMW3901 configuration structure
 */
typedef struct {
    spi_host_device_t spi_host;  // SPI host (SPI2_HOST or SPI3_HOST)
    gpio_num_t pin_miso;         // MISO pin
    gpio_num_t pin_mosi;         // MOSI pin
    gpio_num_t pin_sclk;         // SCLK pin
    gpio_num_t pin_cs;           // Chip select pin
    bool skip_bus_init;          // Skip SPI bus init (if shared with another device)
} pmw3901_config_t;

/**
 * @brief PMW3901 device handle
 */
typedef struct {
    spi_device_handle_t spi_handle;
    gpio_num_t cs_pin;
    bool initialized;
    uint8_t product_id;
    uint8_t revision_id;
} pmw3901_t;

/**
 * @brief Get default configuration for StampFly
 *
 * @param config Pointer to configuration structure to fill
 */
void pmw3901_get_default_config(pmw3901_config_t *config);

/**
 * @brief Initialize the PMW3901 sensor
 *
 * @param dev Pointer to PMW3901 device handle
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_init(pmw3901_t *dev, const pmw3901_config_t *config);

/**
 * @brief Deinitialize the PMW3901 sensor
 *
 * @param dev Pointer to PMW3901 device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_deinit(pmw3901_t *dev);

/**
 * @brief Read a register from PMW3901
 *
 * @param dev Pointer to PMW3901 device handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_read_register(pmw3901_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief Write a register to PMW3901
 *
 * @param dev Pointer to PMW3901 device handle
 * @param reg Register address
 * @param value Value to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_write_register(pmw3901_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Select register bank
 *
 * @param dev Pointer to PMW3901 device handle
 * @param bank Bank number to select
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_select_bank(pmw3901_t *dev, uint8_t bank);

/**
 * @brief Perform soft reset
 *
 * @param dev Pointer to PMW3901 device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_reset(pmw3901_t *dev);

/**
 * @brief Read motion data (delta X and Y)
 *
 * @param dev Pointer to PMW3901 device handle
 * @param delta_x Pointer to store X displacement
 * @param delta_y Pointer to store Y displacement
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_read_motion(pmw3901_t *dev, int16_t *delta_x, int16_t *delta_y);

/**
 * @brief Read motion burst data (all motion data in one transaction)
 *
 * @param dev Pointer to PMW3901 device handle
 * @param burst Pointer to store burst data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_read_motion_burst(pmw3901_t *dev, pmw3901_motion_burst_t *burst);

/**
 * @brief Get product ID
 *
 * @param dev Pointer to PMW3901 device handle
 * @param product_id Pointer to store product ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_get_product_id(pmw3901_t *dev, uint8_t *product_id);

/**
 * @brief Get revision ID
 *
 * @param dev Pointer to PMW3901 device handle
 * @param revision_id Pointer to store revision ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_get_revision_id(pmw3901_t *dev, uint8_t *revision_id);

/**
 * @brief Check if motion is detected
 *
 * @param dev Pointer to PMW3901 device handle
 * @param motion_detected Pointer to store motion detection status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_is_motion_detected(pmw3901_t *dev, bool *motion_detected);

/**
 * @brief Enable frame capture mode
 *
 * @param dev Pointer to PMW3901 device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_enable_frame_capture(pmw3901_t *dev);

/**
 * @brief Read image frame (35x35 pixels, 8-bit grayscale)
 *
 * @param dev Pointer to PMW3901 device handle
 * @param image Pointer to buffer to store image (must be at least PMW3901_FRAME_SIZE bytes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pmw3901_read_frame(pmw3901_t *dev, uint8_t *image);

/* Note: Velocity calculation functions have been removed.
 * Use ESKF::updateFlowRaw() for proper velocity estimation with:
 * - Gyro compensation (rotation removal)
 * - Camera-to-body coordinate transformation
 * - Physically correct angular velocity calculation
 * See: components/stampfly_eskf/include/eskf.hpp
 */

#ifdef __cplusplus
}
#endif

#endif /* PMW3901_H */
