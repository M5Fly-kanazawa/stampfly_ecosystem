/**
 * @file button.hpp
 * @brief Button Driver (GPIO0)
 *
 * Debounce, long press detection
 * - Short press (<1s): Mode switch
 * - Long press (3s): Clear pairing, enter pairing mode
 * - Long press (5s): System reset
 */

#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"

namespace stampfly {

class Button {
public:
    enum class Event {
        NONE,
        CLICK,
        DOUBLE_CLICK,
        LONG_PRESS_3S,
        LONG_PRESS_5S,
        LONG_PRESS_START
    };

    using EventCallback = std::function<void(Event)>;

    struct Config {
        int gpio;
        uint32_t debounce_ms;
    };

    Button() = default;
    ~Button() = default;

    /**
     * @brief Initialize button
     * @param config GPIO configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Set event callback
     * @param callback Function to call on button events
     */
    void setCallback(EventCallback callback);

    /**
     * @brief Update button state (call periodically)
     */
    void tick();

    /**
     * @brief Get last event
     * @return Last button event
     */
    Event getLastEvent() const { return last_event_; }

    /**
     * @brief Check if button is pressed
     * @return true if pressed
     */
    bool isPressed() const { return is_pressed_; }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    EventCallback callback_;
    Event last_event_ = Event::NONE;
    bool is_pressed_ = false;
    bool was_pressed_ = false;
    uint32_t press_start_time_ = 0;
    bool long_press_3s_triggered_ = false;
    bool long_press_5s_triggered_ = false;
};

}  // namespace stampfly
