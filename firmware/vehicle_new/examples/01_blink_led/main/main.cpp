/**
 * @file main.cpp
 * @brief Example 01 — Blink the on-board WS2812 RGB LED
 *        サンプル01 — 内蔵WS2812 RGB LEDを点滅させる
 *
 * This is the simplest StampFly example. It cycles the MCU LED
 * (GPIO 21) through Red → Green → Blue at 1-second intervals.
 * これはStampFlyの最もシンプルなサンプルです。MCU LED（GPIO21）を
 * 赤→緑→青の順に1秒間隔で切り替えます。
 *
 * Hardware: M5Stamp S3 — WS2812 on GPIO 21
 * ハードウェア: M5Stamp S3 — GPIO21にWS2812
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"

// GPIO pin for the MCU built-in LED (WS2812)
// MCU内蔵LED（WS2812）のGPIOピン
static constexpr int GPIO_LED_MCU = 21;

static const char* TAG = "blink_led";

extern "C" void app_main(void)
{
    // Configure the WS2812 LED strip (1 LED on GPIO 21)
    // WS2812 LEDストリップを設定（GPIO21に1個）
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = GPIO_LED_MCU;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10 MHz

    led_strip_handle_t led_strip = nullptr;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Color table: Red, Green, Blue
    // カラーテーブル: 赤、緑、青
    constexpr uint8_t colors[][3] = {
        {32, 0,  0 },  // Red   / 赤
        {0,  32, 0 },  // Green / 緑
        {0,  0,  32},  // Blue  / 青
    };
    constexpr int NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

    ESP_LOGI(TAG, "Starting LED color cycle...");

    int color_index = 0;
    while (true) {
        // Set the LED color and refresh
        // LEDの色を設定してリフレッシュ
        led_strip_set_pixel(led_strip, 0,
                            colors[color_index][0],
                            colors[color_index][1],
                            colors[color_index][2]);
        led_strip_refresh(led_strip);

        ESP_LOGI(TAG, "Color: %s",
                 color_index == 0 ? "RED" :
                 color_index == 1 ? "GREEN" : "BLUE");

        // Wait 1 second before switching to the next color
        // 次の色に切り替えるまで1秒待つ
        vTaskDelay(pdMS_TO_TICKS(1000));

        color_index = (color_index + 1) % NUM_COLORS;
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change the brightness values (32 → 128) to make it brighter
//    明るさの値を変えてみよう（32 → 128）
//
// 2. Add more colors (e.g., yellow = {32, 32, 0})
//    色を追加してみよう（例: 黄色 = {32, 32, 0}）
//
// 3. Change the delay to make it faster or slower
//    delayを変えて速くしたり遅くしたりしてみよう
//
// 4. Try GPIO_LED_BODY (GPIO 39) which has 2 LEDs in daisy-chain
//    GPIO_LED_BODY（GPIO39）に変えてみよう（LEDが2個直列）
// ============================================================
