/*
 * ブザー制御 - ESP-IDF LEDC API版
 *
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 * SPDX-License-Identifier: MIT
 */
#include "buzzer.h"
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char* TAG = "BUZZER";

// ブザー設定
#define BUZZER_GPIO         GPIO_NUM_5
#define BUZZER_LEDC_TIMER   LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define BUZZER_DUTY_RES     LEDC_TIMER_8_BIT
#define BUZZER_DUTY_50PCT   128  // 50% duty cycle (8bit: 256/2)

static bool buzzer_initialized = false;

void buzzer_init(void)
{
    if (buzzer_initialized) {
        return;
    }

    // LEDCタイマー設定
    ledc_timer_config_t timer_conf = {
        .speed_mode      = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_DUTY_RES,
        .timer_num       = BUZZER_LEDC_TIMER,
        .freq_hz         = 4000,  // 初期周波数
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // LEDCチャンネル設定
    ledc_channel_config_t channel_conf = {
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,  // 初期はオフ
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    buzzer_initialized = true;
    ESP_LOGI(TAG, "ブザー初期化完了 (GPIO%d)", BUZZER_GPIO);
}

void buzzer_sound(uint32_t frequency, uint32_t duration_ms)
{
    if (!buzzer_initialized) {
        buzzer_init();
    }

    if (frequency == 0) {
        buzzer_stop();
        return;
    }

    // 周波数設定
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, frequency);

    // デューティサイクル設定 (50%)
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_50PCT);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);

    // 指定時間待機
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // 停止
    buzzer_stop();
}

void buzzer_stop(void)
{
    if (!buzzer_initialized) {
        return;
    }

    // デューティサイクルを0に設定して停止
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void beep(void)
{
    buzzer_sound(4000, 100);
}

void beep_start_tone(void)
{
    buzzer_sound(2000, 100);
    buzzer_sound(1000, 100);
}

// ビーコンロス: 単発高音ビープ (緊急警告)
void beep_beacon_loss(void)
{
    buzzer_sound(4000, 100);
}

// スロットタイミングエラー: 中高音2回
void beep_slot_error(void)
{
    buzzer_sound(3000, 80);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_sound(3000, 80);
}

// ドローンオフライン: 低音長ビープ
void beep_drone_offline(void)
{
    buzzer_sound(1000, 300);
}

// Mutexタイムアウト: 中音3回 (速いパターン)
void beep_mutex_timeout(void)
{
    buzzer_sound(2000, 60);
    vTaskDelay(pdMS_TO_TICKS(40));
    buzzer_sound(2000, 60);
    vTaskDelay(pdMS_TO_TICKS(40));
    buzzer_sound(2000, 60);
}
