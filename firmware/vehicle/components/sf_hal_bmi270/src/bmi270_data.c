/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Kouhei Ito
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmi270_data.c
 * @brief BMI270 Data Reading Implementation
 *
 * This file implements data reading and configuration functions
 * for the BMI270 sensor.
 */

#include "bmi270_data.h"
#include "bmi270_defs.h"
#include "esp_log.h"

static const char *TAG = "BMI270_DATA";

// Forward declarations from bmi270_spi.c
extern esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data);
extern esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data);
extern esp_err_t bmi270_read_burst(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length);

/* ====== Helper Functions ====== */

/**
 * @brief Get scale factor for accelerometer based on range setting
 */
static float bmi270_get_accel_scale(uint8_t range) {
    switch (range) {
        case BMI270_ACC_RANGE_2G:  return BMI270_ACC_SCALE_2G;
        case BMI270_ACC_RANGE_4G:  return BMI270_ACC_SCALE_4G;
        case BMI270_ACC_RANGE_8G:  return BMI270_ACC_SCALE_8G;
        case BMI270_ACC_RANGE_16G: return BMI270_ACC_SCALE_16G;
        default: return BMI270_ACC_SCALE_2G;  // Default to ±2g
    }
}

/**
 * @brief Get scale factor for gyroscope based on range setting
 */
static float bmi270_get_gyro_scale(uint8_t range) {
    switch (range) {
        case BMI270_GYR_RANGE_125DPS:  return BMI270_GYR_SCALE_125DPS;
        case BMI270_GYR_RANGE_250DPS:  return BMI270_GYR_SCALE_250DPS;
        case BMI270_GYR_RANGE_500DPS:  return BMI270_GYR_SCALE_500DPS;
        case BMI270_GYR_RANGE_1000DPS: return BMI270_GYR_SCALE_1000DPS;
        case BMI270_GYR_RANGE_2000DPS: return BMI270_GYR_SCALE_2000DPS;
        default: return BMI270_GYR_SCALE_2000DPS;  // Default to ±2000°/s
    }
}

/* ====== Data Reading Functions ====== */

/**
 * @brief Read raw accelerometer data (all 3 axes)
 *
 * Uses burst read starting from ACC_X_LSB to ensure data consistency
 * through BMI270's shadowing mechanism.
 */
esp_err_t bmi270_read_accel_raw(bmi270_dev_t *dev, bmi270_raw_data_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_accel_raw");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read 6 bytes starting from ACC_X_LSB (0x0C)
    // This ensures shadowing: reading LSB locks corresponding MSB
    uint8_t buf[6];
    esp_err_t ret = bmi270_read_burst(dev, BMI270_REG_ACC_X_LSB, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    // Combine LSB and MSB (little-endian, 2's complement)
    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

/**
 * @brief Read raw gyroscope data (all 3 axes)
 *
 * Uses burst read starting from GYR_X_LSB to ensure data consistency
 * through BMI270's shadowing mechanism.
 */
esp_err_t bmi270_read_gyro_raw(bmi270_dev_t *dev, bmi270_raw_data_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_gyro_raw");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read 6 bytes starting from GYR_X_LSB (0x12)
    // This ensures shadowing: reading LSB locks corresponding MSB
    uint8_t buf[6];
    esp_err_t ret = bmi270_read_burst(dev, BMI270_REG_GYR_X_LSB, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }

    // Combine LSB and MSB (little-endian, 2's complement)
    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

/**
 * @brief Read accelerometer data in physical units (g)
 */
esp_err_t bmi270_read_accel(bmi270_dev_t *dev, bmi270_accel_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_accel");
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw data
    bmi270_raw_data_t raw;
    esp_err_t ret = bmi270_read_accel_raw(dev, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_accel_scale(dev->acc_range);

    // Convert to physical units (g)
    data->x = (float)raw.x / scale;
    data->y = (float)raw.y / scale;
    data->z = (float)raw.z / scale;

    return ESP_OK;
}

/**
 * @brief Read gyroscope data in physical units (rad/s)
 */
esp_err_t bmi270_read_gyro(bmi270_dev_t *dev, bmi270_gyro_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_gyro");
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw data
    bmi270_raw_data_t raw;
    esp_err_t ret = bmi270_read_gyro_raw(dev, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units (°/s first, then to rad/s)
    data->x = ((float)raw.x / scale) * BMI270_DEG_TO_RAD;
    data->y = ((float)raw.y / scale) * BMI270_DEG_TO_RAD;
    data->z = ((float)raw.z / scale) * BMI270_DEG_TO_RAD;

    return ESP_OK;
}

/**
 * @brief Read gyroscope data in degrees per second (dps)
 */
esp_err_t bmi270_read_gyro_dps(bmi270_dev_t *dev, bmi270_gyro_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_gyro_dps");
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw data
    bmi270_raw_data_t raw;
    esp_err_t ret = bmi270_read_gyro_raw(dev, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units (°/s)
    data->x = (float)raw.x / scale;
    data->y = (float)raw.y / scale;
    data->z = (float)raw.z / scale;

    return ESP_OK;
}

/**
 * @brief Read both gyroscope and accelerometer data simultaneously (rad/s and g)
 */
esp_err_t bmi270_read_gyro_accel(bmi270_dev_t *dev, bmi270_gyro_t *gyro, bmi270_accel_t *accel) {
    if (dev == NULL || gyro == NULL || accel == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_gyro_accel");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read 12 bytes starting from ACC_X_LSB (0x0C)
    // This reads both accelerometer and gyroscope in one burst
    uint8_t buf[12];
    esp_err_t ret = bmi270_read_burst(dev, BMI270_REG_ACC_X_LSB, buf, 12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyro+accel data");
        return ret;
    }

    // Parse accelerometer data (bytes 0-5)
    int16_t acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t acc_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t acc_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Parse gyroscope data (bytes 6-11)
    int16_t gyr_x = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gyr_y = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t gyr_z = (int16_t)((buf[11] << 8) | buf[10]);

    // Get scale factors
    float acc_scale = bmi270_get_accel_scale(dev->acc_range);
    float gyr_scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units
    accel->x = (float)acc_x / acc_scale;
    accel->y = (float)acc_y / acc_scale;
    accel->z = (float)acc_z / acc_scale;

    gyro->x = ((float)gyr_x / gyr_scale) * BMI270_DEG_TO_RAD;  // Convert to rad/s
    gyro->y = ((float)gyr_y / gyr_scale) * BMI270_DEG_TO_RAD;
    gyro->z = ((float)gyr_z / gyr_scale) * BMI270_DEG_TO_RAD;

    return ESP_OK;
}

/**
 * @brief Read both gyroscope (dps) and accelerometer data simultaneously
 */
esp_err_t bmi270_read_gyro_accel_dps(bmi270_dev_t *dev, bmi270_gyro_t *gyro, bmi270_accel_t *accel) {
    if (dev == NULL || gyro == NULL || accel == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_gyro_accel_dps");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read 12 bytes starting from ACC_X_LSB (0x0C)
    // This reads both accelerometer and gyroscope in one burst
    uint8_t buf[12];
    esp_err_t ret = bmi270_read_burst(dev, BMI270_REG_ACC_X_LSB, buf, 12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyro+accel data");
        return ret;
    }

    // Parse accelerometer data (bytes 0-5)
    int16_t acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t acc_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t acc_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Parse gyroscope data (bytes 6-11)
    int16_t gyr_x = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gyr_y = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t gyr_z = (int16_t)((buf[11] << 8) | buf[10]);

    // Get scale factors
    float acc_scale = bmi270_get_accel_scale(dev->acc_range);
    float gyr_scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units
    accel->x = (float)acc_x / acc_scale;
    accel->y = (float)acc_y / acc_scale;
    accel->z = (float)acc_z / acc_scale;

    gyro->x = (float)gyr_x / gyr_scale;  // Keep in °/s
    gyro->y = (float)gyr_y / gyr_scale;
    gyro->z = (float)gyr_z / gyr_scale;

    return ESP_OK;
}

/**
 * @brief Convert raw accelerometer data to physical units (g)
 */
esp_err_t bmi270_convert_accel_raw(bmi270_dev_t *dev, const bmi270_raw_data_t *raw, bmi270_accel_t *accel) {
    if (dev == NULL || raw == NULL || accel == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_convert_accel_raw");
        return ESP_ERR_INVALID_ARG;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_accel_scale(dev->acc_range);

    // Convert to physical units (g)
    accel->x = (float)raw->x / scale;
    accel->y = (float)raw->y / scale;
    accel->z = (float)raw->z / scale;

    return ESP_OK;
}

/**
 * @brief Convert raw gyroscope data to physical units (rad/s)
 */
esp_err_t bmi270_convert_gyro_raw(bmi270_dev_t *dev, const bmi270_raw_data_t *raw, bmi270_gyro_t *gyro) {
    if (dev == NULL || raw == NULL || gyro == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_convert_gyro_raw");
        return ESP_ERR_INVALID_ARG;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units (°/s first, then to rad/s)
    gyro->x = ((float)raw->x / scale) * BMI270_DEG_TO_RAD;
    gyro->y = ((float)raw->y / scale) * BMI270_DEG_TO_RAD;
    gyro->z = ((float)raw->z / scale) * BMI270_DEG_TO_RAD;

    return ESP_OK;
}

/**
 * @brief Convert raw gyroscope data to degrees per second (dps)
 */
esp_err_t bmi270_convert_gyro_raw_dps(bmi270_dev_t *dev, const bmi270_raw_data_t *raw, bmi270_gyro_t *gyro) {
    if (dev == NULL || raw == NULL || gyro == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_convert_gyro_raw_dps");
        return ESP_ERR_INVALID_ARG;
    }

    // Get scale factor based on current range
    float scale = bmi270_get_gyro_scale(dev->gyr_range);

    // Convert to physical units (°/s)
    gyro->x = (float)raw->x / scale;
    gyro->y = (float)raw->y / scale;
    gyro->z = (float)raw->z / scale;

    return ESP_OK;
}

/**
 * @brief Read temperature sensor data in °C
 */
esp_err_t bmi270_read_temperature(bmi270_dev_t *dev, float *temperature) {
    if (dev == NULL || temperature == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_temperature");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read 2 bytes: TEMP_MSB (0x22), TEMP_LSB (0x23)
    uint8_t temp_data[2];
    esp_err_t ret = bmi270_read_burst(dev, BMI270_REG_TEMP_MSB, temp_data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }

    // Combine MSB and LSB (little-endian, same as accelerometer/gyroscope)
    // Note: Despite datasheet saying big-endian, actual hardware appears to use little-endian
    // temp_data[0] = LSB, temp_data[1] = MSB
    int16_t temp_raw = (int16_t)((temp_data[1] << 8) | temp_data[0]);

    // Convert to physical units using BMI270 formula:
    // Temperature (°C) = (temp_raw / 512.0) + 23.0
    *temperature = ((float)temp_raw / BMI270_TEMP_SCALE) + BMI270_TEMP_OFFSET;

    return ESP_OK;
}

/* ====== Unit Conversion Utilities ====== */

/**
 * @brief Convert angular velocity from rad/s to degrees/s
 */
float bmi270_rad_to_dps(float rad_per_sec) {
    return rad_per_sec * BMI270_RAD_TO_DEG;
}

/**
 * @brief Convert angular velocity from degrees/s to rad/s
 */
float bmi270_dps_to_rad(float deg_per_sec) {
    return deg_per_sec * BMI270_DEG_TO_RAD;
}

/* ====== Configuration Functions ====== */

/**
 * @brief Configure accelerometer range
 */
esp_err_t bmi270_set_accel_range(bmi270_dev_t *dev, bmi270_acc_range_t range) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_set_accel_range");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Write range setting to ACC_RANGE register
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_ACC_RANGE, range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer range");
        return ret;
    }

    // Update device structure
    dev->acc_range = range;

    ESP_LOGI(TAG, "Accelerometer range set to 0x%02X", range);
    return ESP_OK;
}

/**
 * @brief Configure gyroscope range
 */
esp_err_t bmi270_set_gyro_range(bmi270_dev_t *dev, bmi270_gyr_range_t range) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_set_gyro_range");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Write range setting to GYR_RANGE register
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_GYR_RANGE, range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope range");
        return ret;
    }

    // Update device structure
    dev->gyr_range = range;

    ESP_LOGI(TAG, "Gyroscope range set to 0x%02X", range);
    return ESP_OK;
}

/**
 * @brief Configure accelerometer ODR and filter performance
 */
esp_err_t bmi270_set_accel_config(bmi270_dev_t *dev,
                                   bmi270_acc_odr_t odr,
                                   bmi270_filter_perf_t filter_perf) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_set_accel_config");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare ACC_CONF value
    uint8_t conf_val = odr & 0x0F;  // ODR in lower 4 bits
    if (filter_perf == BMI270_FILTER_PERFORMANCE) {
        conf_val |= BMI270_ACC_CONF_FILTER_PERF;  // Set bit 7
    }

    // Write to ACC_CONF register
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_ACC_CONF, conf_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer config");
        return ret;
    }

    ESP_LOGI(TAG, "Accelerometer config set: ODR=0x%02X, filter_perf=%d (ACC_CONF=0x%02X)",
             odr, filter_perf, conf_val);
    return ESP_OK;
}

/**
 * @brief Configure gyroscope ODR and filter performance
 */
esp_err_t bmi270_set_gyro_config(bmi270_dev_t *dev,
                                  bmi270_gyr_odr_t odr,
                                  bmi270_filter_perf_t filter_perf) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_set_gyro_config");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare GYR_CONF value
    uint8_t conf_val = odr & 0x0F;  // ODR in lower 4 bits
    if (filter_perf == BMI270_FILTER_PERFORMANCE) {
        conf_val |= BMI270_GYR_CONF_FILTER_PERF;  // Set bit 7
    }

    // Write to GYR_CONF register
    esp_err_t ret = bmi270_write_register(dev, BMI270_REG_GYR_CONF, conf_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope config");
        return ret;
    }

    ESP_LOGI(TAG, "Gyroscope config set: ODR=0x%02X, filter_perf=%d (GYR_CONF=0x%02X)",
             odr, filter_perf, conf_val);
    return ESP_OK;
}

/**
 * @brief Get current accelerometer range setting
 */
esp_err_t bmi270_get_accel_range(bmi270_dev_t *dev, bmi270_acc_range_t *range) {
    if (dev == NULL || range == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_get_accel_range");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read ACC_RANGE register
    uint8_t range_val;
    esp_err_t ret = bmi270_read_register(dev, BMI270_REG_ACC_RANGE, &range_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer range");
        return ret;
    }

    // Update device structure and output
    dev->acc_range = range_val & 0x03;  // Range is in lower 2 bits
    *range = (bmi270_acc_range_t)dev->acc_range;

    return ESP_OK;
}

/**
 * @brief Get current gyroscope range setting
 */
esp_err_t bmi270_get_gyro_range(bmi270_dev_t *dev, bmi270_gyr_range_t *range) {
    if (dev == NULL || range == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_get_gyro_range");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->init_complete) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read GYR_RANGE register
    uint8_t range_val;
    esp_err_t ret = bmi270_read_register(dev, BMI270_REG_GYR_RANGE, &range_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope range");
        return ret;
    }

    // Update device structure and output
    dev->gyr_range = range_val & 0x07;  // Range is in lower 3 bits
    *range = (bmi270_gyr_range_t)dev->gyr_range;

    return ESP_OK;
}
