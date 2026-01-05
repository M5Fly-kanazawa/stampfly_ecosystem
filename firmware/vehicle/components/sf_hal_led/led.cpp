/**
 * @file led.cpp
 * @brief WS2812 RGB LED Driver Implementation (RMT)
 */

#include "led.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include <cmath>

// NVS namespace and key for LED settings
static constexpr const char* NVS_NAMESPACE = "led";
static constexpr const char* NVS_KEY_BRIGHTNESS = "brightness";

static const char* TAG = "LED";

namespace stampfly {

// Pattern timing constants
static constexpr uint32_t BLINK_SLOW_PERIOD_MS = 1000;
static constexpr uint32_t BLINK_FAST_PERIOD_MS = 200;
static constexpr uint32_t BREATHE_PERIOD_MS = 2000;
static constexpr uint32_t RAINBOW_PERIOD_MS = 5000;

esp_err_t LED::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    ESP_LOGI(TAG, "Initializing LED on GPIO%d, %d LEDs", config.gpio, config.num_leds);

    // Configure LED strip using RMT
    led_strip_config_t strip_config = {
        .strip_gpio_num = config.gpio,
        .max_leds = static_cast<uint32_t>(config.num_leds),
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10MHz
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear all LEDs
    led_strip_clear(led_strip_);

    initialized_ = true;
    last_update_ms_ = esp_timer_get_time() / 1000;

    // Load saved brightness from NVS
    if (loadFromNVS() == ESP_OK) {
        ESP_LOGI(TAG, "LED brightness loaded from NVS: %d", brightness_);
    }

    ESP_LOGI(TAG, "LED initialized successfully");
    return ESP_OK;
}

void LED::setColor(uint8_t index, uint32_t color)
{
    if (!initialized_ || index >= config_.num_leds || led_strip_ == nullptr) {
        return;
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    led_strip_set_pixel(led_strip_, index, r, g, b);
    led_strip_refresh(led_strip_);
}

void LED::setPattern(Pattern pattern, uint32_t color)
{
    if (!initialized_) return;
    current_pattern_ = pattern;
    current_color_ = color;
}

void LED::setBrightness(uint8_t brightness, bool save_to_nvs)
{
    brightness_ = brightness;
    if (save_to_nvs) {
        saveToNVS();
    }
}

esp_err_t LED::saveToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_BRIGHTNESS, brightness_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write brightness: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Brightness saved to NVS: %d", brightness_);
    }
    return ret;
}

esp_err_t LED::loadFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        // NVS not found is expected on first boot
        return ret;
    }

    uint8_t brightness;
    ret = nvs_get_u8(handle, NVS_KEY_BRIGHTNESS, &brightness);
    nvs_close(handle);

    if (ret == ESP_OK) {
        brightness_ = brightness;
    }
    return ret;
}

void LED::update()
{
    if (!initialized_ || led_strip_ == nullptr) return;

    uint32_t now_ms = esp_timer_get_time() / 1000;
    uint32_t elapsed = now_ms - last_update_ms_;
    (void)elapsed;  // Suppress unused warning

    // Extract color and apply brightness
    uint8_t r = ((current_color_ >> 16) & 0xFF) * brightness_ / 255;
    uint8_t g = ((current_color_ >> 8) & 0xFF) * brightness_ / 255;
    uint8_t b = (current_color_ & 0xFF) * brightness_ / 255;

    switch (current_pattern_) {
        case Pattern::OFF:
            for (int i = 0; i < config_.num_leds; i++) {
                led_strip_set_pixel(led_strip_, i, 0, 0, 0);
            }
            break;

        case Pattern::SOLID:
            for (int i = 0; i < config_.num_leds; i++) {
                led_strip_set_pixel(led_strip_, i, r, g, b);
            }
            break;

        case Pattern::BLINK_SLOW: {
            bool on = ((now_ms / (BLINK_SLOW_PERIOD_MS / 2)) % 2) == 0;
            for (int i = 0; i < config_.num_leds; i++) {
                if (on) {
                    led_strip_set_pixel(led_strip_, i, r, g, b);
                } else {
                    led_strip_set_pixel(led_strip_, i, 0, 0, 0);
                }
            }
            break;
        }

        case Pattern::BLINK_FAST: {
            bool on = ((now_ms / (BLINK_FAST_PERIOD_MS / 2)) % 2) == 0;
            for (int i = 0; i < config_.num_leds; i++) {
                if (on) {
                    led_strip_set_pixel(led_strip_, i, r, g, b);
                } else {
                    led_strip_set_pixel(led_strip_, i, 0, 0, 0);
                }
            }
            break;
        }

        case Pattern::BREATHE: {
            // Sine wave breathing effect (brightness already applied to r,g,b)
            float phase = (float)(now_ms % BREATHE_PERIOD_MS) / BREATHE_PERIOD_MS;
            float breath = (1.0f - std::cos(phase * 2.0f * M_PI)) / 2.0f;
            uint8_t br = static_cast<uint8_t>(r * breath);
            uint8_t bg = static_cast<uint8_t>(g * breath);
            uint8_t bb = static_cast<uint8_t>(b * breath);
            for (int i = 0; i < config_.num_leds; i++) {
                led_strip_set_pixel(led_strip_, i, br, bg, bb);
            }
            break;
        }

        case Pattern::RAINBOW: {
            // HSV to RGB rainbow cycle with brightness
            float hue = (float)(now_ms % RAINBOW_PERIOD_MS) / RAINBOW_PERIOD_MS * 360.0f;
            float bright_scale = brightness_ / 255.0f;
            for (int i = 0; i < config_.num_leds; i++) {
                float h = fmodf(hue + (float)i * 360.0f / config_.num_leds, 360.0f);
                // Simple HSV to RGB conversion (S=1, V=brightness)
                float c = 1.0f;
                float x = c * (1.0f - std::fabs(fmodf(h / 60.0f, 2.0f) - 1.0f));
                float m = 0.0f;
                float rf, gf, bf;

                if (h < 60) { rf = c; gf = x; bf = 0; }
                else if (h < 120) { rf = x; gf = c; bf = 0; }
                else if (h < 180) { rf = 0; gf = c; bf = x; }
                else if (h < 240) { rf = 0; gf = x; bf = c; }
                else if (h < 300) { rf = x; gf = 0; bf = c; }
                else { rf = c; gf = 0; bf = x; }

                uint8_t pr = static_cast<uint8_t>((rf + m) * 255 * bright_scale);
                uint8_t pg = static_cast<uint8_t>((gf + m) * 255 * bright_scale);
                uint8_t pb = static_cast<uint8_t>((bf + m) * 255 * bright_scale);

                led_strip_set_pixel(led_strip_, i, pr, pg, pb);
            }
            break;
        }
    }

    led_strip_refresh(led_strip_);
    last_update_ms_ = now_ms;
}

void LED::showInit()      { setPattern(Pattern::BREATHE, 0x0000FF); }  // Blue breathe
void LED::showCalibrating() { setPattern(Pattern::BLINK_FAST, 0xFFFF00); }  // Yellow fast blink
void LED::showIdle()      { setPattern(Pattern::SOLID, 0x00FF00); }  // Green solid
void LED::showArmed()     { setPattern(Pattern::BLINK_SLOW, 0x00FF00); }  // Green slow blink
void LED::showFlying()    { setPattern(Pattern::SOLID, 0xFFFF00); }  // Yellow solid
void LED::showLanding()   { setPattern(Pattern::BLINK_FAST, 0x00FF00); }  // Green fast blink
void LED::showError()     { setPattern(Pattern::BLINK_FAST, 0xFF0000); }  // Red fast blink
void LED::showLowBattery() { setPattern(Pattern::BLINK_SLOW, 0xFF0000); }  // Red slow blink
void LED::showLowBatteryCyan() { setPattern(Pattern::BLINK_SLOW, 0x00FFFF); }  // Cyan slow blink (battery replace)
void LED::showPairing()   { setPattern(Pattern::BLINK_FAST, 0x0000FF); }  // Blue fast blink

}  // namespace stampfly
