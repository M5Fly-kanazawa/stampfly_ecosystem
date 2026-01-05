/**
 * @file bmi270_wrapper.cpp
 * @brief C++ wrapper implementation for BMI270 IMU Sensor
 */

#include "bmi270_wrapper.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "BMI270Wrapper";

namespace stampfly {

BMI270Wrapper::~BMI270Wrapper()
{
    if (initialized_) {
        // Cleanup would be handled by C driver if needed
        initialized_ = false;
    }
}

BMI270Wrapper::BMI270Wrapper(BMI270Wrapper&& other) noexcept
    : device_(other.device_)
    , config_(other.config_)
    , initialized_(other.initialized_)
    , fifo_accel_enabled_(other.fifo_accel_enabled_)
    , fifo_gyro_enabled_(other.fifo_gyro_enabled_)
{
    other.initialized_ = false;
}

BMI270Wrapper& BMI270Wrapper::operator=(BMI270Wrapper&& other) noexcept
{
    if (this != &other) {
        device_ = other.device_;
        config_ = other.config_;
        initialized_ = other.initialized_;
        fifo_accel_enabled_ = other.fifo_accel_enabled_;
        fifo_gyro_enabled_ = other.fifo_gyro_enabled_;
        other.initialized_ = false;
    }
    return *this;
}

esp_err_t BMI270Wrapper::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    // Configure C driver config
    bmi270_config_t c_config = {
        .gpio_mosi = static_cast<uint8_t>(config.pin_mosi),
        .gpio_miso = static_cast<uint8_t>(config.pin_miso),
        .gpio_sclk = static_cast<uint8_t>(config.pin_sclk),
        .gpio_cs = static_cast<uint8_t>(config.pin_cs),
        .spi_clock_hz = config.spi_clock_hz,
        .spi_host = config.spi_host,
        .gpio_other_cs = static_cast<int8_t>(config.other_cs)
    };

    // Initialize SPI
    esp_err_t ret = bmi270_spi_init(&device_, &c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize BMI270
    ret = bmi270_init(&device_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMI270: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure sensor ranges and ODR
    ret = bmi270_set_accel_range(&device_, config.accel_range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bmi270_set_gyro_range(&device_, config.gyro_range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bmi270_set_accel_config(&device_, config.accel_odr, config.filter_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bmi270_set_gyro_config(&device_, config.gyro_odr, config.filter_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro config: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "BMI270 initialized successfully");

    return ESP_OK;
}

esp_err_t BMI270Wrapper::readSensorData(AccelData& accel, GyroData& gyro)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    bmi270_gyro_t c_gyro;
    bmi270_accel_t c_accel;

    esp_err_t ret = bmi270_read_gyro_accel(&device_, &c_gyro, &c_accel);
    if (ret != ESP_OK) {
        return ret;
    }

    accel.x = c_accel.x;
    accel.y = c_accel.y;
    accel.z = c_accel.z;

    gyro.x = c_gyro.x;
    gyro.y = c_gyro.y;
    gyro.z = c_gyro.z;

    return ESP_OK;
}

esp_err_t BMI270Wrapper::readAccel(AccelData& accel)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    bmi270_accel_t c_accel;
    esp_err_t ret = bmi270_read_accel(&device_, &c_accel);
    if (ret != ESP_OK) {
        return ret;
    }

    accel.x = c_accel.x;
    accel.y = c_accel.y;
    accel.z = c_accel.z;

    return ESP_OK;
}

esp_err_t BMI270Wrapper::readGyro(GyroData& gyro)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    bmi270_gyro_t c_gyro;
    esp_err_t ret = bmi270_read_gyro(&device_, &c_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro.x = c_gyro.x;
    gyro.y = c_gyro.y;
    gyro.z = c_gyro.z;

    return ESP_OK;
}

esp_err_t BMI270Wrapper::readTemperature(float& temperature)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_read_temperature(&device_, &temperature);
}

esp_err_t BMI270Wrapper::configureFIFO(bool enable_accel, bool enable_gyro, uint16_t watermark_frames)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate watermark in bytes based on enabled sensors
    uint16_t frame_size = 0;
    if (enable_accel && enable_gyro) {
        frame_size = BMI270_FIFO_FRAME_ACC_GYR_SIZE;  // 13 bytes
    } else if (enable_accel) {
        frame_size = BMI270_FIFO_FRAME_ACC_SIZE;      // 7 bytes
    } else if (enable_gyro) {
        frame_size = BMI270_FIFO_FRAME_GYR_SIZE;      // 7 bytes
    }

    uint16_t watermark_bytes = watermark_frames * frame_size;
    if (watermark_bytes > BMI270_FIFO_SIZE) {
        watermark_bytes = BMI270_FIFO_SIZE;
    }

    // FIFO_CONFIG_0: Stream mode (overwrite on full)
    uint8_t fifo_config_0 = 0x00;  // Stream mode
    esp_err_t ret = bmi270_write_register(&device_, BMI270_REG_FIFO_CONFIG_0, fifo_config_0);
    if (ret != ESP_OK) {
        return ret;
    }

    // FIFO_CONFIG_1: Enable sensors and header mode
    uint8_t fifo_config_1 = BMI270_FIFO_HEADER_EN;
    if (enable_accel) {
        fifo_config_1 |= BMI270_FIFO_ACC_EN;
    }
    if (enable_gyro) {
        fifo_config_1 |= BMI270_FIFO_GYR_EN;
    }

    ret = bmi270_write_register(&device_, BMI270_REG_FIFO_CONFIG_1, fifo_config_1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set watermark
    ret = bmi270_write_register(&device_, BMI270_REG_FIFO_WTM_0, watermark_bytes & 0xFF);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = bmi270_write_register(&device_, BMI270_REG_FIFO_WTM_1, (watermark_bytes >> 8) & 0xFF);
    if (ret != ESP_OK) {
        return ret;
    }

    fifo_accel_enabled_ = enable_accel;
    fifo_gyro_enabled_ = enable_gyro;

    ESP_LOGI(TAG, "FIFO configured: accel=%d, gyro=%d, watermark=%d bytes",
             enable_accel, enable_gyro, watermark_bytes);

    return ESP_OK;
}

esp_err_t BMI270Wrapper::getFIFOLength(uint16_t& length)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t length_data[2];
    esp_err_t ret = bmi270_read_burst(&device_, BMI270_REG_FIFO_LENGTH_0, length_data, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    // Lower 11 bits contain length
    length = ((uint16_t)length_data[1] << 8) | length_data[0];
    length &= 0x07FF;

    return ESP_OK;
}

esp_err_t BMI270Wrapper::flushFIFO()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_write_register(&device_, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}

bool BMI270Wrapper::parseFIFOFrame(const uint8_t* data, size_t& offset, size_t max_len, FIFOFrame& frame)
{
    if (offset >= max_len) {
        return false;
    }

    uint8_t header = data[offset++];

    frame.has_accel = false;
    frame.has_gyro = false;

    // Check header type
    if (header == BMI270_FIFO_HEAD_ACC_GYR) {
        // Accel + Gyro frame (12 bytes data)
        if (offset + 12 > max_len) {
            return false;
        }

        // Parse accelerometer (first 6 bytes)
        int16_t acc_x = (int16_t)((data[offset + 1] << 8) | data[offset + 0]);
        int16_t acc_y = (int16_t)((data[offset + 3] << 8) | data[offset + 2]);
        int16_t acc_z = (int16_t)((data[offset + 5] << 8) | data[offset + 4]);

        // Parse gyroscope (next 6 bytes)
        int16_t gyr_x = (int16_t)((data[offset + 7] << 8) | data[offset + 6]);
        int16_t gyr_y = (int16_t)((data[offset + 9] << 8) | data[offset + 8]);
        int16_t gyr_z = (int16_t)((data[offset + 11] << 8) | data[offset + 10]);

        offset += 12;

        // Convert raw values to physical units
        bmi270_raw_data_t raw_accel = {acc_x, acc_y, acc_z};
        bmi270_raw_data_t raw_gyro = {gyr_x, gyr_y, gyr_z};

        bmi270_accel_t accel;
        bmi270_gyro_t gyro;

        bmi270_convert_accel_raw(&device_, &raw_accel, &accel);
        bmi270_convert_gyro_raw(&device_, &raw_gyro, &gyro);

        frame.accel.x = accel.x;
        frame.accel.y = accel.y;
        frame.accel.z = accel.z;
        frame.gyro.x = gyro.x;
        frame.gyro.y = gyro.y;
        frame.gyro.z = gyro.z;

        frame.has_accel = true;
        frame.has_gyro = true;

        return true;

    } else if (header == BMI270_FIFO_HEAD_ACC) {
        // Accelerometer only frame (6 bytes data)
        if (offset + 6 > max_len) {
            return false;
        }

        int16_t acc_x = (int16_t)((data[offset + 1] << 8) | data[offset + 0]);
        int16_t acc_y = (int16_t)((data[offset + 3] << 8) | data[offset + 2]);
        int16_t acc_z = (int16_t)((data[offset + 5] << 8) | data[offset + 4]);
        offset += 6;

        bmi270_raw_data_t raw_accel = {acc_x, acc_y, acc_z};
        bmi270_accel_t accel;
        bmi270_convert_accel_raw(&device_, &raw_accel, &accel);

        frame.accel.x = accel.x;
        frame.accel.y = accel.y;
        frame.accel.z = accel.z;
        frame.has_accel = true;

        return true;

    } else if (header == BMI270_FIFO_HEAD_GYR) {
        // Gyroscope only frame (6 bytes data)
        if (offset + 6 > max_len) {
            return false;
        }

        int16_t gyr_x = (int16_t)((data[offset + 1] << 8) | data[offset + 0]);
        int16_t gyr_y = (int16_t)((data[offset + 3] << 8) | data[offset + 2]);
        int16_t gyr_z = (int16_t)((data[offset + 5] << 8) | data[offset + 4]);
        offset += 6;

        bmi270_raw_data_t raw_gyro = {gyr_x, gyr_y, gyr_z};
        bmi270_gyro_t gyro;
        bmi270_convert_gyro_raw(&device_, &raw_gyro, &gyro);

        frame.gyro.x = gyro.x;
        frame.gyro.y = gyro.y;
        frame.gyro.z = gyro.z;
        frame.has_gyro = true;

        return true;

    } else if (header == BMI270_FIFO_HEAD_SKIP) {
        // Skip frame (1 byte)
        if (offset + 1 > max_len) {
            return false;
        }
        offset += 1;
        return false;  // No valid data in skip frame

    } else if (header == BMI270_FIFO_HEAD_SENSOR_TIME) {
        // Sensor time frame (3 bytes)
        if (offset + 3 > max_len) {
            return false;
        }
        offset += 3;
        return false;  // No accel/gyro data

    } else if (header == BMI270_FIFO_HEAD_CONFIG_CHANGE) {
        // Config change frame (1 byte)
        if (offset + 1 > max_len) {
            return false;
        }
        offset += 1;
        return false;
    }

    // Unknown or empty header
    return false;
}

esp_err_t BMI270Wrapper::readFIFO(FIFOFrame* buffer, size_t max_frames, size_t& frames_read)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    frames_read = 0;

    // Get FIFO length
    uint16_t fifo_length;
    esp_err_t ret = getFIFOLength(fifo_length);
    if (ret != ESP_OK) {
        return ret;
    }

    if (fifo_length == 0) {
        return ESP_OK;
    }

    // Cap FIFO length to prevent buffer overflow
    if (fifo_length > BMI270_FIFO_SIZE) {
        fifo_length = BMI270_FIFO_SIZE;
    }

    // Allocate read buffer
    uint8_t* fifo_data = new uint8_t[fifo_length];
    if (fifo_data == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Read FIFO data
    ret = bmi270_read_burst(&device_, BMI270_REG_FIFO_DATA, fifo_data, fifo_length);
    if (ret != ESP_OK) {
        delete[] fifo_data;
        return ret;
    }

    // Parse frames
    size_t offset = 0;
    while (offset < fifo_length && frames_read < max_frames) {
        FIFOFrame frame;
        if (parseFIFOFrame(fifo_data, offset, fifo_length, frame)) {
            if (frame.has_accel || frame.has_gyro) {
                buffer[frames_read++] = frame;
            }
        }
    }

    delete[] fifo_data;
    return ESP_OK;
}

esp_err_t BMI270Wrapper::enableDataReadyInterrupt(bmi270_int_pin_t int_pin)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Configure interrupt pin (active high, push-pull)
    bmi270_int_pin_config_t pin_config = {
        .output_enable = true,
        .active_high = true,
        .open_drain = false
    };

    esp_err_t ret = bmi270_configure_int_pin(&device_, int_pin, &pin_config);
    if (ret != ESP_OK) {
        return ret;
    }

    return bmi270_enable_data_ready_interrupt(&device_, int_pin);
}

esp_err_t BMI270Wrapper::enableFIFOWatermarkInterrupt(bmi270_int_pin_t int_pin)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Configure interrupt pin (active high, push-pull)
    bmi270_int_pin_config_t pin_config = {
        .output_enable = true,
        .active_high = true,
        .open_drain = false
    };

    esp_err_t ret = bmi270_configure_int_pin(&device_, int_pin, &pin_config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Map FIFO watermark interrupt to selected pin
    uint8_t int_map = 0;
    ret = bmi270_read_register(&device_, BMI270_REG_INT_MAP_DATA, &int_map);
    if (ret != ESP_OK) {
        return ret;
    }

    if (int_pin == BMI270_INT_PIN_1) {
        int_map |= BMI270_FIFO_WM_INT1;
    } else {
        int_map |= BMI270_FIFO_WM_INT2;
    }

    return bmi270_write_register(&device_, BMI270_REG_INT_MAP_DATA, int_map);
}

esp_err_t BMI270Wrapper::disableInterrupt(bmi270_int_pin_t int_pin)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_disable_data_ready_interrupt(&device_, int_pin);
}

esp_err_t BMI270Wrapper::setAccelRange(bmi270_acc_range_t range)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_set_accel_range(&device_, range);
}

esp_err_t BMI270Wrapper::setGyroRange(bmi270_gyr_range_t range)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_set_gyro_range(&device_, range);
}

esp_err_t BMI270Wrapper::setAccelConfig(bmi270_acc_odr_t odr, bmi270_filter_perf_t filter_perf)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_set_accel_config(&device_, odr, filter_perf);
}

esp_err_t BMI270Wrapper::setGyroConfig(bmi270_gyr_odr_t odr, bmi270_filter_perf_t filter_perf)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    return bmi270_set_gyro_config(&device_, odr, filter_perf);
}

}  // namespace stampfly
