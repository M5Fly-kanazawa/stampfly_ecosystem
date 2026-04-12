/**
 * @file main.cpp
 * @brief Example 03 — Detect button press and show on LED
 *        サンプル03 — ボタン押下を検出してLEDで表示
 *
 * Polls the on-board button (GPIO 0, active-low) with software
 * debouncing. When pressed, the MCU LED turns green; when
 * released, it turns off.
 *
 * ボタン（GPIO0、アクティブLOW）をポーリングし、ソフトウェア
 * デバウンスを行います。押すとMCU LEDが緑に、離すと消灯します。
 *
 * Hardware: M5Stamp S3 — Button on GPIO 0, LED on GPIO 21
 * ハードウェア: M5Stamp S3 — GPIO0にボタン、GPIO21にLED
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_log.h"

// Pin definitions
// ピン定義
static constexpr gpio_num_t GPIO_BUTTON  = GPIO_NUM_0;
static constexpr int        GPIO_LED_MCU = 21;

// Debounce time in milliseconds
// デバウンス時間（ミリ秒）
static constexpr uint32_t DEBOUNCE_MS = 50;

static const char* TAG = "button";

extern "C" void app_main(void)
{
    // Configure button as input with pull-up (active-low)
    // ボタンをプルアップ付き入力に設定（アクティブLOW）
    gpio_config_t btn_config = {};
    btn_config.pin_bit_mask = (1ULL << GPIO_BUTTON);
    btn_config.mode         = GPIO_MODE_INPUT;
    btn_config.pull_up_en   = GPIO_PULLUP_ENABLE;
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_config.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&btn_config));

    // Configure LED strip (1 WS2812 on GPIO 21)
    // LEDストリップを設定（GPIO21に1個のWS2812）
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = GPIO_LED_MCU;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;

    led_strip_handle_t led_strip = nullptr;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "Press the button (GPIO 0)...");

    bool last_stable_state = true;   // true = released (pull-up)
    bool last_raw_state    = true;
    uint32_t last_change_tick = 0;
    uint32_t press_count = 0;

    while (true) {
        // Read the raw button state (0 = pressed, 1 = released)
        // ボタンの生値を読む（0 = 押下、1 = 開放）
        bool raw = gpio_get_level(GPIO_BUTTON);
        uint32_t now = xTaskGetTickCount();

        // Reset debounce timer if the raw state changed
        // 生値が変化したらデバウンスタイマーをリセット
        if (raw != last_raw_state) {
            last_change_tick = now;
            last_raw_state = raw;
        }

        // Accept the state after it has been stable for DEBOUNCE_MS
        // DEBOUNCE_MSの間安定していたら状態を確定
        if ((now - last_change_tick) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
            if (raw != last_stable_state) {
                last_stable_state = raw;

                if (!raw) {
                    // Button pressed — LED green
                    // ボタン押下 — LED緑
                    press_count++;
                    ESP_LOGI(TAG, "PRESSED  (count: %lu)", (unsigned long)press_count);
                    led_strip_set_pixel(led_strip, 0, 0, 32, 0);
                } else {
                    // Button released — LED off
                    // ボタン開放 — LED消灯
                    ESP_LOGI(TAG, "RELEASED");
                    led_strip_set_pixel(led_strip, 0, 0, 0, 0);
                }
                led_strip_refresh(led_strip);
            }
        }

        // Poll at 100 Hz (10 ms interval)
        // 100Hzでポーリング（10ms間隔）
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change the LED color when pressed (green → red, blue, etc.)
//    押下時のLED色を変えてみよう（緑→赤、青など）
//
// 2. Change DEBOUNCE_MS to see what happens with shorter debounce
//    DEBOUNCE_MSを短くするとどうなるか試してみよう
//
// 3. Toggle the LED on each press instead of holding
//    押すたびにLEDをトグルする動作に変えてみよう
//
// 4. Add a long-press detection (e.g., 2 seconds = different color)
//    長押し検出を追加してみよう（例: 2秒で別の色）
// ============================================================
