/**
 * @file main.cpp
 * @brief Example 07 — Spin a single motor slowly (with safety)
 *        サンプル07 — モーター1個をゆっくり回す（安全機能付き）
 *
 * Drives Motor 1 (Front-Right, GPIO 42) at low duty cycle (20%)
 * using LEDC PWM. The button (GPIO 0) acts as an emergency stop.
 *
 * M1（右前、GPIO42）をLEDC PWMで低デューティ（20%）で駆動します。
 * ボタン（GPIO0）は緊急停止ボタンとして機能します。
 *
 * !! WARNING / 警告 !!
 * - REMOVE PROPELLERS before running this example!
 *   プロペラを外してからこのサンプルを実行してください！
 * - Keep fingers away from the motor.
 *   モーターに指を近づけないでください。
 *
 * Hardware: StampFly — M1=GPIO42, Button=GPIO0
 * ハードウェア: StampFly — M1=GPIO42, ボタン=GPIO0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Motor M1 (Front-Right) GPIO
// モーターM1（右前）のGPIO
static constexpr int GPIO_MOTOR_M1 = 42;

// Button GPIO (emergency stop)
// ボタンGPIO（緊急停止）
static constexpr gpio_num_t GPIO_BUTTON = GPIO_NUM_0;

// PWM configuration for motor
// モーター用PWM設定
static constexpr int PWM_FREQ_HZ     = 150000;  // 150 kHz
static constexpr int PWM_RESOLUTION  = LEDC_TIMER_8_BIT;  // 8-bit (0-255)
static constexpr int MAX_DUTY_PERCENT = 20;  // Safety: max 20% duty
static constexpr int MAX_DUTY = (255 * MAX_DUTY_PERCENT) / 100;  // ~51

static const char* TAG = "motor";

/// Stop the motor immediately
/// モーターを即座に停止
static void motor_stop(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

extern "C" void app_main(void)
{
    ESP_LOGW(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    ESP_LOGW(TAG, "!! REMOVE PROPELLERS BEFORE RUNNING !!");
    ESP_LOGW(TAG, "!! プロペラを外してから実行すること  !!");
    ESP_LOGW(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    ESP_LOGI(TAG, "Motor will start in 3 seconds...");
    ESP_LOGI(TAG, "Press button (GPIO 0) to emergency stop.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Configure button as input (emergency stop)
    // ボタンを入力に設定（緊急停止）
    gpio_config_t btn_config = {};
    btn_config.pin_bit_mask = (1ULL << GPIO_BUTTON);
    btn_config.mode         = GPIO_MODE_INPUT;
    btn_config.pull_up_en   = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&btn_config));

    // Initialize LEDC timer for motor PWM (150 kHz, 8-bit)
    // モーターPWM用LEDCタイマーを初期化（150kHz、8ビット）
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION);
    timer_config.timer_num       = LEDC_TIMER_0;
    timer_config.freq_hz         = PWM_FREQ_HZ;
    timer_config.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Initialize LEDC channel for M1
    // M1用LEDCチャネルを初期化
    ledc_channel_config_t channel_config = {};
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel    = LEDC_CHANNEL_0;
    channel_config.timer_sel  = LEDC_TIMER_0;
    channel_config.gpio_num   = GPIO_MOTOR_M1;
    channel_config.duty       = 0;
    channel_config.hpoint     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    // Ramp up to MAX_DUTY slowly (safety: gradual start)
    // MAX_DUTYまでゆっくり上昇（安全: 段階的な開始）
    ESP_LOGI(TAG, "Ramping motor to %d%% duty...", MAX_DUTY_PERCENT);
    for (int duty = 0; duty <= MAX_DUTY; duty += 5) {
        // Check emergency stop button
        // 緊急停止ボタンをチェック
        if (gpio_get_level(GPIO_BUTTON) == 0) {
            motor_stop();
            ESP_LOGW(TAG, "EMERGENCY STOP! Button pressed.");
            return;
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ESP_LOGI(TAG, "Duty: %d / 255 (%d%%)", duty, (duty * 100) / 255);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "Running at %d%%. Press button to stop.", MAX_DUTY_PERCENT);

    // Hold at MAX_DUTY, check for emergency stop
    // MAX_DUTYを維持、緊急停止をチェック
    while (true) {
        if (gpio_get_level(GPIO_BUTTON) == 0) {
            motor_stop();
            ESP_LOGW(TAG, "STOPPED by button press.");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change MAX_DUTY_PERCENT (keep it under 30% for safety!)
//    MAX_DUTY_PERCENTを変えてみよう（安全のため30%以下で！）
//
// 2. Try a different motor: M2=GPIO41, M3=GPIO10, M4=GPIO5
//    別のモーターを試してみよう: M2=GPIO41, M3=GPIO10, M4=GPIO5
//
// 3. Make the motor oscillate: ramp up then ramp down
//    モーターを往復させてみよう: 上昇→下降の繰り返し
//
// 4. Add a timeout to auto-stop after 10 seconds
//    10秒後に自動停止するタイムアウトを追加してみよう
// ============================================================
