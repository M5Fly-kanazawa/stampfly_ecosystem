/**
 * @file button.cpp
 * @brief Button Driver Implementation (GPIO0)
 */

#include "button.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Button";

namespace stampfly {

esp_err_t Button::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    ESP_LOGI(TAG, "Initializing Button on GPIO%d", config.gpio);

    // Configure GPIO as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config.gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,  // We'll poll instead of using interrupts
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Button initialized successfully");

    return ESP_OK;
}

void Button::setCallback(EventCallback callback)
{
    callback_ = callback;
}

void Button::tick()
{
    if (!initialized_) return;

    // Read GPIO state (active low with pull-up)
    bool raw_pressed = (gpio_get_level(static_cast<gpio_num_t>(config_.gpio)) == 0);

    // Simple debounce - only consider stable readings
    static bool last_raw_state = false;
    static uint32_t debounce_start = 0;
    static bool debounced_state = false;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // If raw state changed, start debounce timer
    if (raw_pressed != last_raw_state) {
        debounce_start = now;
        last_raw_state = raw_pressed;
    }

    // Only update debounced state after debounce period
    if ((now - debounce_start) >= config_.debounce_ms) {
        debounced_state = raw_pressed;
    }

    bool pressed = debounced_state;

    if (pressed && !was_pressed_) {
        // Button just pressed
        press_start_time_ = now;
        was_pressed_ = true;
        long_press_3s_triggered_ = false;
        long_press_5s_triggered_ = false;

        last_event_ = Event::LONG_PRESS_START;
        if (callback_) callback_(Event::LONG_PRESS_START);
    }
    else if (!pressed && was_pressed_) {
        // Button released
        uint32_t duration = now - press_start_time_;
        was_pressed_ = false;

        if (duration < 1000 && !long_press_3s_triggered_) {
            last_event_ = Event::CLICK;
            if (callback_) callback_(Event::CLICK);
            ESP_LOGI(TAG, "Button click detected");
        }
    }
    else if (pressed && was_pressed_) {
        // Button held
        uint32_t duration = now - press_start_time_;

        if (duration >= 5000 && !long_press_5s_triggered_) {
            long_press_5s_triggered_ = true;
            last_event_ = Event::LONG_PRESS_5S;
            if (callback_) callback_(Event::LONG_PRESS_5S);
            ESP_LOGI(TAG, "Long press 5s detected - System reset");
        }
        else if (duration >= 3000 && !long_press_3s_triggered_) {
            long_press_3s_triggered_ = true;
            last_event_ = Event::LONG_PRESS_3S;
            if (callback_) callback_(Event::LONG_PRESS_3S);
            ESP_LOGI(TAG, "Long press 3s detected - Clear pairing");
        }
    }

    is_pressed_ = pressed;
}

}  // namespace stampfly
