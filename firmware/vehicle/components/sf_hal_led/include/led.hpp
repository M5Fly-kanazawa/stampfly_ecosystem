/**
 * @file led.hpp
 * @brief WS2812 RGB LED Driver (RMT)
 *
 * Pattern support, state display
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "led_strip.h"

namespace stampfly {

class LED {
public:
    enum class Pattern {
        OFF,
        SOLID,
        BLINK_SLOW,
        BLINK_FAST,
        BREATHE,
        RAINBOW
    };

    struct Config {
        int gpio;
        int num_leds;
    };

    LED() = default;
    ~LED() = default;

    /**
     * @brief Initialize LED driver
     * @param config GPIO configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Set LED color
     * @param index LED index
     * @param color RGB color (0xRRGGBB)
     */
    void setColor(uint8_t index, uint32_t color);

    /**
     * @brief Set LED pattern
     * @param pattern Pattern type
     * @param color RGB color for pattern
     */
    void setPattern(Pattern pattern, uint32_t color);

    /**
     * @brief Set LED brightness
     * @param brightness 0-255 (0=off, 255=full)
     * @param save_to_nvs If true, save to NVS
     */
    void setBrightness(uint8_t brightness, bool save_to_nvs = false);

    /**
     * @brief Get current brightness
     * @return brightness 0-255
     */
    uint8_t getBrightness() const { return brightness_; }

    /**
     * @brief Save brightness to NVS
     * @return ESP_OK on success
     */
    esp_err_t saveToNVS();

    /**
     * @brief Load brightness from NVS
     * @return ESP_OK on success
     */
    esp_err_t loadFromNVS();

    /**
     * @brief Update LED (call periodically for patterns)
     */
    void update();

    // State display helpers
    void showInit();
    void showCalibrating();
    void showIdle();
    void showArmed();
    void showFlying();
    void showLanding();
    void showError();
    void showLowBattery();
    void showLowBatteryCyan();  // Battery replace warning (cyan blink)
    void showPairing();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    Pattern current_pattern_ = Pattern::OFF;
    uint32_t current_color_ = 0;
    uint32_t last_update_ms_ = 0;
    uint8_t brightness_ = 32;  // Default brightness (0-255), 32 = ~12%
    led_strip_handle_t led_strip_ = nullptr;  // インスタンス固有のハンドル
};

}  // namespace stampfly
