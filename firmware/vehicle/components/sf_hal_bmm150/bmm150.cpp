/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file bmm150.cpp
 * @brief BMM150 Magnetometer Driver Implementation
 *
 * Reference: BMM150 Datasheet, Bosch Sensortec
 */

#include "bmm150.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BMM150";

namespace stampfly {

// I2C timeout
constexpr int I2C_TIMEOUT_MS = 100;

// Soft reset delay
constexpr int SOFT_RESET_DELAY_MS = 3;

// Startup delay after power on
constexpr int STARTUP_DELAY_MS = 3;

BMM150::~BMM150()
{
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

esp_err_t BMM150::init(const Config& config)
{
    // Re-init guard: a second init would add a duplicate device handle to the
    // I2C bus and leak the old one (same convention as the VL53/LED HALs).
    // 再init ガード: 2回目の init は I2C バスへ重複ハンドルを追加し旧ハンドルが
    // リークする（VL53/LED HAL と同じ流儀）。
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    config_ = config;
    esp_err_t ret;

    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config_.i2c_addr,
        .scl_speed_hz = 400000,  // 400kHz
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    ret = i2c_master_bus_add_device(config_.i2c_bus, &dev_cfg, &dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Power on - set power control bit
    ret = writeRegister(bmm150_reg::POWER_CTRL, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power control");
        return ret;
    }

    // Wait for startup
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    // Read and verify chip ID
    uint8_t chip_id;
    ret = readRegister(bmm150_reg::CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != BMM150_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BMM150_CHIP_ID);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "BMM150 chip ID: 0x%02X", chip_id);

    // Read trim data for compensation
    ret = readTrimData();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read trim data");
        return ret;
    }

    // Set preset (repetitions)
    ret = setPreset(config_.preset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set preset");
        return ret;
    }

    // Set data rate and normal operation mode
    ret = setDataRate(config_.data_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set data rate");
        return ret;
    }

    ret = setOpMode(BMM150OpMode::NORMAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "BMM150 initialized successfully");

    return ESP_OK;
}

esp_err_t BMM150::read(MagData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    MagRawData raw;
    esp_err_t ret = readRaw(raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Gate on the burst-embedded DRDY (0x48 bit0, read in the same transaction
    // as the data): only a finished conversion sets it, so accepted samples are
    // always consistent. A SEPARATE status pre-read proved unreliable on
    // hardware (mag died entirely); the in-burst bit cannot disagree with the
    // data it arrived with.
    // バースト内 DRDY（0x48 bit0、データと同一トランザクションで読む）でゲート:
    // 変換完了時のみ立つので、受理サンプルは常に一貫している。「別読み」の
    // ステータス事前確認は実機で不安定（mag が全滅）だった。バースト内ビットは
    // 一緒に届いたデータと矛盾し得ない。
    if (!raw.drdy) {
        data.data_ready = false;
        return ESP_OK;   // no new sample yet — silent skip / 新サンプルなし
    }

    // Apply compensation, then remap chip axes → body frame (FRD) here, in the driver,
    // so callers receive body-axis field and never deal with the sensor mounting
    // (project policy: drivers return body-axis quantities). Mapping verified on the
    // proven firmware/vehicle hardware (mag_task): body.x = -chip.y, body.y = chip.x,
    // body.z = chip.z (note: differs from the IMU/flow mounting).
    // 補償後、チップ軸→機体(FRD)へここ（ドライバ）で remap し、呼び出し側は機体軸の磁場を
    // 受け取り搭載向きを意識しない（方針: ドライバは機体軸の量を返す）。対応は実証済みハード
    // (firmware/vehicle mag_task)で確認: body.x=-chip.y, body.y=chip.x, body.z=chip.z
    // （IMU/flow とは異なる搭載向き）。
    const float chip_x = compensateX(raw.x, raw.rhall);
    const float chip_y = compensateY(raw.y, raw.rhall);
    const float chip_z = compensateZ(raw.z, raw.rhall);

    // Saturation guard: the Bosch compensation returns NaN on sensor overflow
    // (magnetic saturation — realistic near high motor currents). A NaN must
    // never leave the driver: one NaN entering a Kalman update poisons the
    // whole state/covariance irrecoverably. Report "no data" instead.
    // 飽和ガード: Bosch の補償はセンサオーバーフロー時に NaN を返す（磁気飽和 —
    // モータ大電流の近傍で現実に起こる）。NaN をドライバの外に出してはならない:
    // Kalman 更新に NaN が1つ入ると状態・共分散全体が回復不能に汚染される。
    // 代わりに「データなし」として報告する。
    if (!std::isfinite(chip_x) || !std::isfinite(chip_y) || !std::isfinite(chip_z)) {
        data.data_ready = false;
        return ESP_ERR_INVALID_RESPONSE;
    }

    data.x = -chip_y;   // body forward (FRD X)
    data.y =  chip_x;   // body right   (FRD Y)
    data.z =  chip_z;   // body down    (FRD Z)
    data.timestamp_us = esp_timer_get_time();
    data.data_ready = true;

    return ESP_OK;
}

esp_err_t BMM150::readRaw(MagRawData& raw)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read all data registers at once (0x42-0x49, 8 bytes)
    uint8_t buf[8];
    esp_err_t ret = readRegister(bmm150_reg::DATA_X_LSB, buf, 8);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse raw data
    // X: 13-bit signed, bits [4:0] of LSB and [7:0] of MSB
    raw.x = (int16_t)((((int16_t)buf[1]) << 8) | (buf[0] & 0xF8)) >> 3;

    // Y: 13-bit signed
    raw.y = (int16_t)((((int16_t)buf[3]) << 8) | (buf[2] & 0xF8)) >> 3;

    // Z: 15-bit signed
    raw.z = (int16_t)((((int16_t)buf[5]) << 8) | (buf[4] & 0xFE)) >> 1;

    // RHALL: 14-bit unsigned
    raw.rhall = (uint16_t)((((uint16_t)buf[7]) << 8) | (buf[6] & 0xFC)) >> 2;

    // DRDY status (0x48 bit0), captured in the SAME transaction as the data —
    // a separate status read raced the conversion and proved unreliable on
    // hardware; this bit is by definition consistent with the bytes above.
    // DRDY ステータス（0x48 bit0）。データと「同一トランザクション」で取得 —
    // 別読みのステータスは変換と競合して実機で不安定だった。このビットは定義上
    // 上のバイト列と一貫している。
    raw.drdy = (buf[6] & 0x01) != 0;

    return ESP_OK;
}

esp_err_t BMM150::setOpMode(BMM150OpMode mode)
{
    // Read current register value
    uint8_t reg_data;
    esp_err_t ret = readRegister(bmm150_reg::OP_MODE, &reg_data, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Modify opmode bits [2:1]
    reg_data = (reg_data & 0xF9) | (static_cast<uint8_t>(mode) << 1);

    return writeRegister(bmm150_reg::OP_MODE, reg_data);
}

esp_err_t BMM150::setDataRate(BMM150DataRate rate)
{
    // Read current register value
    uint8_t reg_data;
    esp_err_t ret = readRegister(bmm150_reg::OP_MODE, &reg_data, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Modify data rate bits [5:3]
    reg_data = (reg_data & 0xC7) | (static_cast<uint8_t>(rate) << 3);

    return writeRegister(bmm150_reg::OP_MODE, reg_data);
}

esp_err_t BMM150::setPreset(BMM150Preset preset)
{
    uint8_t rep_xy, rep_z;

    switch (preset) {
        case BMM150Preset::LOW_POWER:
            rep_xy = 1;   // nXY = 1+1 = 2 (effective 3)
            rep_z = 2;    // nZ = 1+1 = 2 (effective 3)
            break;
        case BMM150Preset::REGULAR:
            rep_xy = 4;   // nXY = 4+1 = 5 (effective 9)
            rep_z = 14;   // nZ = 14+1 = 15
            break;
        case BMM150Preset::ENHANCED:
            rep_xy = 7;   // nXY = 7+1 = 8 (effective 15)
            rep_z = 26;   // nZ = 26+1 = 27
            break;
        case BMM150Preset::HIGH_ACCURACY:
            rep_xy = 23;  // nXY = 23+1 = 24 (effective 47)
            rep_z = 82;   // nZ = 82+1 = 83
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = writeRegister(bmm150_reg::REP_XY, rep_xy);
    if (ret != ESP_OK) {
        return ret;
    }

    return writeRegister(bmm150_reg::REP_Z, rep_z);
}

esp_err_t BMM150::softReset()
{
    // Set soft reset bit (bit 7 and bit 1)
    esp_err_t ret = writeRegister(bmm150_reg::POWER_CTRL, 0x82);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(SOFT_RESET_DELAY_MS));

    // Re-enable power
    ret = writeRegister(bmm150_reg::POWER_CTRL, 0x01);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    return ESP_OK;
}

bool BMM150::isDataReady()
{
    if (!initialized_) {
        return false;
    }

    uint8_t status;
    if (readRegister(bmm150_reg::RHALL_LSB, &status, 1) != ESP_OK) {
        return false;
    }

    // Data ready bit is bit 0 of RHALL_LSB register
    return (status & 0x01) != 0;
}

esp_err_t BMM150::readRegister(uint8_t reg, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len,
                                       I2C_TIMEOUT_MS);
}

esp_err_t BMM150::writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle_, buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t BMM150::readTrimData()
{
    uint8_t trim_x1y1[2];
    uint8_t trim_xyz[4];
    uint8_t trim_xy1xy2[10];

    // Read trim registers
    esp_err_t ret = readRegister(bmm150_reg::DIG_X1, trim_x1y1, 2);
    if (ret != ESP_OK) return ret;

    ret = readRegister(bmm150_reg::DIG_Z4_LSB, trim_xyz, 4);
    if (ret != ESP_OK) return ret;

    ret = readRegister(bmm150_reg::DIG_Z2_LSB, trim_xy1xy2, 10);
    if (ret != ESP_OK) return ret;

    // Parse trim data
    trim_data_.dig_x1 = (int8_t)trim_x1y1[0];
    trim_data_.dig_y1 = (int8_t)trim_x1y1[1];
    trim_data_.dig_x2 = (int8_t)trim_xyz[2];
    trim_data_.dig_y2 = (int8_t)trim_xyz[3];
    trim_data_.dig_z4 = (int16_t)(((uint16_t)trim_xyz[1] << 8) | trim_xyz[0]);
    trim_data_.dig_z2 = (int16_t)(((uint16_t)trim_xy1xy2[1] << 8) | trim_xy1xy2[0]);
    trim_data_.dig_z1 = (uint16_t)(((uint16_t)trim_xy1xy2[3] << 8) | trim_xy1xy2[2]);
    trim_data_.dig_xyz1 = (uint16_t)(((uint16_t)trim_xy1xy2[5] << 8) | trim_xy1xy2[4]);
    trim_data_.dig_z3 = (int16_t)(((uint16_t)trim_xy1xy2[7] << 8) | trim_xy1xy2[6]);
    trim_data_.dig_xy2 = (int8_t)trim_xy1xy2[8];
    trim_data_.dig_xy1 = trim_xy1xy2[9];

    return ESP_OK;
}

float BMM150::compensateX(int16_t raw_x, uint16_t rhall)
{
    // Overflow check
    if (raw_x == -4096) {
        return NAN;
    }

    if (rhall == 0) {
        rhall = trim_data_.dig_xyz1;
    }

    float retval;
    float process_comp_x0 = (((float)trim_data_.dig_xyz1) * 16384.0f / rhall);
    retval = (process_comp_x0 - 16384.0f);

    float process_comp_x1 = ((float)trim_data_.dig_xy2) * (retval * retval / 268435456.0f);
    float process_comp_x2 = process_comp_x1 + retval * ((float)trim_data_.dig_xy1) / 16384.0f;
    float process_comp_x3 = ((float)trim_data_.dig_x2) + 160.0f;
    float process_comp_x4 = raw_x * ((process_comp_x2 + 256.0f) * process_comp_x3);

    retval = ((process_comp_x4 / 8192.0f) + (((float)trim_data_.dig_x1) * 8.0f)) / 16.0f;

    return retval;
}

float BMM150::compensateY(int16_t raw_y, uint16_t rhall)
{
    // Overflow check
    if (raw_y == -4096) {
        return NAN;
    }

    if (rhall == 0) {
        rhall = trim_data_.dig_xyz1;
    }

    float retval;
    float process_comp_y0 = ((float)trim_data_.dig_xyz1) * 16384.0f / rhall;
    retval = process_comp_y0 - 16384.0f;

    float process_comp_y1 = ((float)trim_data_.dig_xy2) * (retval * retval / 268435456.0f);
    float process_comp_y2 = process_comp_y1 + retval * ((float)trim_data_.dig_xy1) / 16384.0f;
    float process_comp_y3 = ((float)trim_data_.dig_y2) + 160.0f;
    float process_comp_y4 = raw_y * ((process_comp_y2 + 256.0f) * process_comp_y3);

    retval = ((process_comp_y4 / 8192.0f) + (((float)trim_data_.dig_y1) * 8.0f)) / 16.0f;

    return retval;
}

float BMM150::compensateZ(int16_t raw_z, uint16_t rhall)
{
    // Overflow check
    if (raw_z == -16384) {
        return NAN;
    }

    if (rhall == 0 || trim_data_.dig_z2 == 0 || trim_data_.dig_z1 == 0) {
        return NAN;
    }

    float retval;
    float process_comp_z0 = ((float)raw_z) - ((float)trim_data_.dig_z4);
    float process_comp_z1 = ((float)rhall) - ((float)trim_data_.dig_xyz1);
    float process_comp_z2 = (((float)trim_data_.dig_z3) * process_comp_z1);
    float process_comp_z3 = ((float)trim_data_.dig_z1) * ((float)rhall) / 32768.0f;
    float process_comp_z4 = ((float)trim_data_.dig_z2) + process_comp_z3;
    float process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;

    retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;

    return retval;
}

}  // namespace stampfly
