/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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

// LEDC configuration constants. These mirror sf_board's kMotorTimer /
// kMotorSpeedMode (the timer's owner under R1). When skip_timer_init is set,
// sf_board has configured this timer and we bind channels to it without
// re-config; the two definitions must stay in agreement.
// これらは sf_board の kMotorTimer / kMotorSpeedMode (R1 のタイマ所有者) と一致
// させる。skip_timer_init のとき sf_board が構成済みなので、再構成せずチャンネル
// を紐付ける。両定義は常に一致させること。
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

    esp_err_t ret = ESP_OK;

    // Configure the LEDC timer unless sf_board already owns it (R1). When the
    // BSP configured LEDC_TIMER_0 in Phase 1, we skip re-config and only bind
    // our channels to it below — sf_board is the single timer owner.
    // sf_board が LEDC タイマを所有(R1)していなければ構成する。BSP が Phase 1 で
    // LEDC_TIMER_0 を構成済みのときは再構成せず、下のチャンネルを紐付けるだけ。
    if (config.skip_timer_init) {
        ESP_LOGI(TAG, "LEDC timer config skipped (owned by sf_board)");
    } else {
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(config.pwm_resolution_bits),
            .timer_num = LEDC_TIMER,
            .freq_hz = static_cast<uint32_t>(config.pwm_freq_hz),
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };
        ret = ledc_timer_config(&timer_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
            return ret;
        }
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

    // Set all motors to 0. This is the SAFETY path — check every LEDC write and
    // retry once on failure, then report the aggregate result instead of
    // assuming the zero landed (a silently failed write here means a motor
    // keeps spinning while the system believes it is stopped).
    // 全モータを 0 に。ここは「安全」経路 — LEDC 書き込みを毎回検査し、失敗時は
    // 1回リトライした上で集約結果を返す（ここで黙って失敗すると、システムは停止した
    // つもりなのにモータが回り続ける）。
    esp_err_t result = ESP_OK;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_output_[i] = 0.0f;
        esp_err_t e1 = ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[i], 0);
        esp_err_t e2 = ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[i]);
        if (e1 != ESP_OK || e2 != ESP_OK) {
            e1 = ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[i], 0);
            e2 = ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[i]);
        }
        if (e1 != ESP_OK || e2 != ESP_OK) {
            ESP_LOGE(TAG, "disarm: motor %d duty-zero write FAILED (%s/%s)",
                     i, esp_err_to_name(e1), esp_err_to_name(e2));
            result = (e1 != ESP_OK) ? e1 : e2;
        }
    }

    return result;
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

    // Log (rate-limited by rarity — these calls only fail on bad arguments) so a
    // wiring/config regression is not silently swallowed at 400Hz.
    // 失敗時はログする（これらは引数不正時のみ失敗するため実質まれ）。配線/設定の
    // 退行が 400Hz で黙って握り潰されないようにする。
    esp_err_t e1 = ledc_set_duty(LEDC_MODE, MOTOR_CHANNELS[motor], duty);
    esp_err_t e2 = ledc_update_duty(LEDC_MODE, MOTOR_CHANNELS[motor]);
    if (e1 != ESP_OK || e2 != ESP_OK) {
        ESP_LOGW(TAG, "setMotor(%d): LEDC write failed (%s/%s)",
                 motor, esp_err_to_name(e1), esp_err_to_name(e2));
    }
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
