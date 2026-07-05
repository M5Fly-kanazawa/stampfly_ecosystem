/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file motor_driver.hpp
 * @brief Motor Driver (LEDC PWM) — per-motor duty output for the X-quad.
 *        モータードライバ（LEDC PWM）— X-quad の各モータ duty 出力。
 *
 * X-quad configuration, 150kHz PWM, 8-bit resolution. This is a pure HAL: it
 * takes ready-made per-motor duties [0,1] and writes them to LEDC. The mixer
 * (control allocation) lives in sf_actuator — this driver does NOT mix.
 * X-quad 配置、150kHz PWM、8bit 分解能。純粋な HAL: 出来上がった各モータ duty[0,1] を
 * 受け取り LEDC に書く。ミキサー（制御配分）は sf_actuator にあり、本ドライバは混合しない。
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class MotorDriver {
public:
    // Motor indices (clockwise from front-right)
    static constexpr int MOTOR_FR = 0;  // M1: Front-Right (CCW)
    static constexpr int MOTOR_RR = 1;  // M2: Rear-Right (CW)
    static constexpr int MOTOR_RL = 2;  // M3: Rear-Left (CCW)
    static constexpr int MOTOR_FL = 3;  // M4: Front-Left (CW)
    static constexpr int NUM_MOTORS = 4;

    struct Config {
        int gpio[NUM_MOTORS];     // GPIO pins for each motor
        int pwm_freq_hz;          // PWM frequency (default: 150000)
        int pwm_resolution_bits;  // PWM resolution (default: 8)
        // Skip ledc_timer_config() when sf_board already configured the shared
        // LEDC timer (R1). The driver then only does ledc_channel_config().
        // A plain bool (no driver/ledc.h here) keeps this header host-buildable
        // for the SIL motor stub. / sf_board が共有 LEDC タイマを構成済み(R1)の
        // とき ledc_timer_config() を省く。本ドライバは channel 設定のみ行う。
        // ここを bool にして driver/ledc.h を持ち込まず SIL スタブのビルドを保つ。
        bool skip_timer_init = false;
    };

    MotorDriver() = default;
    ~MotorDriver() = default;

    /**
     * @brief Initialize motor driver
     * @param config PWM configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Arm motors (enable output)
     * @return ESP_OK on success
     */
    esp_err_t arm();

    /**
     * @brief Disarm motors (disable output, set to 0)
     * @return ESP_OK on success
     */
    esp_err_t disarm();

    /**
     * @brief Set motor duties directly (physical units mode)
     * モーターDutyを直接設定（物理単位モード）
     *
     * @param duties Array of 4 duty cycles [0.0-1.0] for M1(FR), M2(RR), M3(RL), M4(FL)
     */
    void setMotorDuties(const float duties[4]);

    bool isInitialized() const { return initialized_; }

private:
    /**
     * @brief Write one motor's duty to LEDC (internal; clamped to [0,1]).
     *        1モータの duty を LEDC に書く（内部用・[0,1] にクランプ）。
     * @param motor Motor index (0-3)
     * @param value Output value (0.0-1.0)
     */
    void setMotor(int motor, float value);

    bool initialized_ = false;
    bool armed_ = false;
    Config config_;
    float motor_output_[NUM_MOTORS] = {0};
};

}  // namespace stampfly
