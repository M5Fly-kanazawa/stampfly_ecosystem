/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file motor_driver.cpp
 * @brief Motor Driver Implementation (LEDC PWM)
 */

#include "motor_driver.hpp"
#include "esp_log.h"
#include "driver/ledc.h"
#include <algorithm>

static const char* TAG = "MotorDriver";

namespace stampfly {

// LEDC configuration constants
static constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;

// Channel mapping for each motor
static constexpr ledc_channel_t MOTOR_CHANNELS[MotorDriver::NUM_MOTORS] = {
    LEDC_CHANNEL_0,  // M1 (FR)
    LEDC_CHANNEL_1,  // M2 (RR)
    LEDC_CHANNEL_2,  // M3 (RL)
    LEDC_CHANNEL_3,  // M4 (FL)
};

esp_err_t MotorDriver::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    ESP_LOGI(TAG, "Initializing Motor Driver");
    ESP_LOGI(TAG, "  M1(FR): GPIO%d, M2(RR): GPIO%d, M3(RL): GPIO%d, M4(FL): GPIO%d",
             config.gpio[0], config.gpio[1], config.gpio[2], config.gpio[3]);
    ESP_LOGI(TAG, "  PWM Freq: %d Hz, Resolution: %d bits",
             config.pwm_freq_hz, config.pwm_resolution_bits);

    // Configure LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(config.pwm_resolution_bits),
        .timer_num = LEDC_TIMER,
        .freq_hz = static_cast<uint32_t>(config.pwm_freq_hz),
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channels for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = config.gpio[i],
            .speed_mode = LEDC_MODE,
            .channel = MOTOR_CHANNELS[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = {
                .output_invert = 0,
            },
        };

        ret = ledc_channel_config(&channel_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Motor Driver initialized successfully");

    return ESP_OK;
}

esp_err_t MotorDriver::arm()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Motors armed");
    armed_ = true;
    return ESP_OK;
}

esp_err_t MotorDriver::disarm()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Motors disarmed");
    armed_ = false;

    // Set all motors to 0
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_output_[i] = 0.0f;
        ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[i], 0);
        ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[i]);
    }

    return ESP_OK;
}

void MotorDriver::setMotor(int motor, float value)
{
    if (!initialized_ || !armed_ || motor < 0 || motor >= NUM_MOTORS) {
        return;
    }

    motor_output_[motor] = std::clamp(value, 0.0f, 1.0f);

    // Calculate duty cycle based on resolution
    // 分解能に基づき duty を計算
    uint32_t max_duty = (1U << config_.pwm_resolution_bits) - 1;
    uint32_t duty = static_cast<uint32_t>(motor_output_[motor] * max_duty);

    ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[motor], duty);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[motor]);
}

void MotorDriver::setMotorDuties(const float duties[4])
{
    if (!initialized_ || !armed_) {
        return;
    }

    // Direct duty assignment (physical units mode). The mixer (control
    // allocation) already ran in sf_actuator; this driver only writes.
    // 直接 Duty 設定（物理単位モード）。ミキサー（制御配分）は sf_actuator で実行済み。
    // 本ドライバは書き込むだけ。
    // Order: M1(FR), M2(RR), M3(RL), M4(FL)
    setMotor(MOTOR_FR, duties[0]);
    setMotor(MOTOR_RR, duties[1]);
    setMotor(MOTOR_RL, duties[2]);
    setMotor(MOTOR_FL, duties[3]);
}

}  // namespace stampfly
