/**
 * @file motor_driver.cpp
 * @brief Motor Driver Implementation (LEDC PWM)
 */

#include "motor_driver.hpp"
#include "esp_log.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"
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

    // Update statistics
    float v = motor_output_[motor];
    stats_[motor].sum += v;
    stats_[motor].count++;
    if (stats_[motor].count == 1) {
        stats_[motor].min = v;
        stats_[motor].max = v;
    } else {
        if (v < stats_[motor].min) stats_[motor].min = v;
        if (v > stats_[motor].max) stats_[motor].max = v;
    }

    // Calculate duty cycle based on resolution
    uint32_t max_duty = (1U << config_.pwm_resolution_bits) - 1;
    uint32_t duty = static_cast<uint32_t>(motor_output_[motor] * max_duty);

    ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[motor], duty);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[motor]);
}

void MotorDriver::setMixerOutput(float thrust, float roll, float pitch, float yaw)
{
    if (!initialized_ || !armed_) {
        return;
    }

    // X-quad mixer
    // M1 (FR, CCW): T - R + P + Y
    // M2 (RR, CW):  T - R - P - Y
    // M3 (RL, CCW): T + R - P + Y
    // M4 (FL, CW):  T + R + P - Y
    float m1 = thrust + 0.25*(- roll + pitch + yaw)/3.7;
    float m2 = thrust + 0.25*(- roll - pitch - yaw)/3.7;
    float m3 = thrust + 0.25*(  roll - pitch + yaw)/3.7;
    float m4 = thrust + 0.25*(  roll + pitch - yaw)/3.7;

    setMotor(MOTOR_FR, m1);
    setMotor(MOTOR_RR, m2);
    setMotor(MOTOR_RL, m3);
    setMotor(MOTOR_FL, m4);
}

void MotorDriver::testMotor(int motor, int throttle_percent)
{
    if (!initialized_ || motor < 0 || motor >= NUM_MOTORS) {
        return;
    }

    float value = std::clamp(throttle_percent, 0, 100) / 100.0f;
    ESP_LOGI(TAG, "Testing motor %d at %d%%", motor + 1, throttle_percent);

    // Calculate duty cycle based on resolution
    uint32_t max_duty = (1U << config_.pwm_resolution_bits) - 1;
    uint32_t duty = static_cast<uint32_t>(value * max_duty);

    // Directly set PWM for testing (bypasses arm check)
    ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[motor], duty);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[motor]);
}

MotorDriver::MotorStats MotorDriver::getStats(int motor) const
{
    if (motor < 0 || motor >= NUM_MOTORS) {
        return {};
    }
    return stats_[motor];
}

void MotorDriver::resetStats()
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        stats_[i] = {};
    }
}

void MotorDriver::saveStatsToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("motor", NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for motor stats: %s", esp_err_to_name(ret));
        return;
    }

    // Save as blob
    ret = nvs_set_blob(handle, "stats", stats_, sizeof(stats_));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save motor stats: %s", esp_err_to_name(ret));
    } else {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Motor stats saved to NVS");
    }

    nvs_close(handle);
}

bool MotorDriver::loadStatsFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("motor", NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No motor stats in NVS");
        return false;
    }

    size_t size = sizeof(last_flight_stats_);
    ret = nvs_get_blob(handle, "stats", last_flight_stats_, &size);
    nvs_close(handle);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load motor stats: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Motor stats loaded from NVS");
    return true;
}

}  // namespace stampfly
