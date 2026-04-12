/**
 * @file main.cpp
 * @brief Example 02 — Play a melody with the buzzer
 *        サンプル02 — ブザーでメロディを鳴らす
 *
 * Uses the LEDC PWM peripheral to generate musical tones
 * on the buzzer (GPIO 40). Plays a simple ascending scale
 * (C4 → C5), then repeats.
 *
 * LEDC PWMペリフェラルを使ってブザー（GPIO40）で音階を生成します。
 * C4からC5までの上昇音階を再生し、繰り返します。
 *
 * Hardware: StampFly — Buzzer on GPIO 40
 * ハードウェア: StampFly — GPIO40にブザー
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

// Buzzer GPIO pin
// ブザーのGPIOピン
static constexpr int GPIO_BUZZER = 40;

// LEDC configuration
// LEDC設定
static constexpr ledc_timer_t   LEDC_TIMER   = LEDC_TIMER_0;
static constexpr ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_mode_t    LEDC_MODE    = LEDC_LOW_SPEED_MODE;

// Musical note frequencies (Hz)
// 音階の周波数 (Hz)
static constexpr uint16_t NOTE_C4 = 262;
static constexpr uint16_t NOTE_D4 = 294;
static constexpr uint16_t NOTE_E4 = 330;
static constexpr uint16_t NOTE_F4 = 349;
static constexpr uint16_t NOTE_G4 = 392;
static constexpr uint16_t NOTE_A4 = 440;
static constexpr uint16_t NOTE_B4 = 494;
static constexpr uint16_t NOTE_C5 = 523;

static const char* TAG = "buzzer";

/// Play a single tone at the given frequency for duration_ms
/// 指定周波数のトーンをduration_msミリ秒間鳴らす
static void play_tone(uint16_t freq_hz, uint32_t duration_ms)
{
    // Set the PWM frequency and 50% duty cycle to produce a tone
    // PWM周波数と50%デューティサイクルを設定してトーンを生成
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 128);  // 50% of 8-bit
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop the tone (duty = 0)
    // トーンを停止（デューティ = 0）
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

extern "C" void app_main(void)
{
    // Initialize LEDC timer
    // LEDCタイマーを初期化
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode      = LEDC_MODE;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;
    timer_config.timer_num       = LEDC_TIMER;
    timer_config.freq_hz         = 1000;  // Initial frequency (will be changed)
    timer_config.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Initialize LEDC channel
    // LEDCチャネルを初期化
    ledc_channel_config_t channel_config = {};
    channel_config.speed_mode = LEDC_MODE;
    channel_config.channel    = LEDC_CHANNEL;
    channel_config.timer_sel  = LEDC_TIMER;
    channel_config.gpio_num   = GPIO_BUZZER;
    channel_config.duty       = 0;
    channel_config.hpoint     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    // Melody: C major scale (C4 to C5)
    // メロディ: Cメジャースケール（C4からC5）
    constexpr uint16_t melody[] = {
        NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4,
        NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5,
    };
    constexpr int MELODY_LEN = sizeof(melody) / sizeof(melody[0]);
    constexpr uint32_t NOTE_DURATION_MS = 300;
    constexpr uint32_t GAP_MS = 100;

    ESP_LOGI(TAG, "Playing melody...");

    while (true) {
        for (int i = 0; i < MELODY_LEN; i++) {
            ESP_LOGI(TAG, "Note %d: %d Hz", i + 1, melody[i]);
            play_tone(melody[i], NOTE_DURATION_MS);

            // Short gap between notes
            // 音符間の短い休止
            vTaskDelay(pdMS_TO_TICKS(GAP_MS));
        }

        // Pause before repeating
        // 繰り返す前に一休み
        ESP_LOGI(TAG, "--- Repeat ---");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change NOTE_DURATION_MS to play faster or slower
//    NOTE_DURATION_MSを変えてテンポを変えてみよう
//
// 2. Create your own melody by rearranging the notes
//    音符を並べ替えてオリジナルのメロディを作ってみよう
//
// 3. Add rests by using play_tone(0, ...) or just vTaskDelay
//    休符を追加してみよう（vTaskDelayのみ使用）
//
// 4. Change the duty cycle (128 → 64) to hear volume difference
//    デューティサイクルを変えて音量の違いを聴いてみよう
// ============================================================
