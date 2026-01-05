/**
 * @file buzzer.hpp
 * @brief Buzzer Driver (LEDC PWM)
 *
 * Tone generation, preset sounds
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

// Musical notes (Hz)
constexpr uint16_t NOTE_C4 = 262;
constexpr uint16_t NOTE_D4 = 294;
constexpr uint16_t NOTE_E4 = 330;
constexpr uint16_t NOTE_F4 = 349;
constexpr uint16_t NOTE_G4 = 392;
constexpr uint16_t NOTE_A4 = 440;
constexpr uint16_t NOTE_B4 = 494;
constexpr uint16_t NOTE_C5 = 523;
constexpr uint16_t NOTE_D5 = 587;
constexpr uint16_t NOTE_E5 = 659;
constexpr uint16_t NOTE_F5 = 698;
constexpr uint16_t NOTE_G5 = 784;
constexpr uint16_t NOTE_A5 = 880;
constexpr uint16_t NOTE_B5 = 988;
constexpr uint16_t NOTE_C6 = 1047;

class Buzzer {
public:
    struct Config {
        int gpio;
        int ledc_channel;
        int ledc_timer;
    };

    Buzzer() = default;
    ~Buzzer() = default;

    /**
     * @brief Initialize buzzer
     * @param config GPIO configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Play tone (blocking)
     * @param frequency Frequency in Hz
     * @param duration_ms Duration in milliseconds
     */
    void playTone(uint16_t frequency, uint32_t duration_ms);

    /**
     * @brief Play tone (non-blocking)
     * @param frequency Frequency in Hz
     * @param duration_ms Duration in milliseconds
     */
    void playToneAsync(uint16_t frequency, uint32_t duration_ms);

    /**
     * @brief Stop current tone
     */
    void stop();

    // Preset sounds
    void beep();
    void startTone();
    void armTone();
    void disarmTone();
    void lowBatteryWarning();
    void errorTone();
    void pairingTone();

    bool isInitialized() const { return initialized_; }

    // Mute control
    bool isMuted() const { return muted_; }
    void setMuted(bool muted, bool save_to_nvs = false);

    // NVS persistence
    esp_err_t loadFromNVS();
    esp_err_t saveToNVS();

private:
    bool initialized_ = false;
    bool muted_ = false;
    Config config_;
};

}  // namespace stampfly
