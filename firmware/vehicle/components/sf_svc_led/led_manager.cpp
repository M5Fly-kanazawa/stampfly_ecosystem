/**
 * @file led_manager.cpp
 * @brief LED管理システム実装
 */

#include "led_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "LEDManager";

namespace stampfly {

LEDManager& LEDManager::getInstance()
{
    static LEDManager instance;
    return instance;
}

esp_err_t LEDManager::init()
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing LED Manager");

    // MCU LED初期化 (GPIO21, 1個)
    LED::Config mcu_cfg;
    mcu_cfg.gpio = 21;
    mcu_cfg.num_leds = 1;
    esp_err_t ret = mcu_led_.init(mcu_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init MCU LED: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "MCU LED initialized (GPIO21)");

    // Body LED初期化 (GPIO39, 2個)
    LED::Config body_cfg;
    body_cfg.gpio = 39;
    body_cfg.num_leds = 2;
    ret = body_led_.init(body_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Body LED: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Body LED initialized (GPIO39, 2 LEDs)");

    // チャンネルとLEDのマッピング設定
    channels_[static_cast<int>(LEDChannel::SYSTEM)].target_led = LEDIndex::MCU;
    channels_[static_cast<int>(LEDChannel::FLIGHT)].target_led = LEDIndex::BODY_TOP;
    channels_[static_cast<int>(LEDChannel::STATUS)].target_led = LEDIndex::BODY_BOTTOM;

    // 初期状態：全LED緑点灯（DEFAULT優先度）
    for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].active = true;
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].pattern = LEDPattern::SOLID;
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].color = 0x00FF00;  // Green
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].expire_time_ms = 0;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "LED Manager initialized successfully");

    return ESP_OK;
}

void LEDManager::requestChannel(LEDChannel channel, LEDPriority priority,
                                 LEDPattern pattern, uint32_t color,
                                 uint32_t timeout_ms)
{
    if (!initialized_) return;

    int ch_idx = static_cast<int>(channel);
    int pri_idx = static_cast<int>(priority);

    if (ch_idx >= static_cast<int>(LEDChannel::NUM_CHANNELS)) return;
    if (pri_idx >= static_cast<int>(LEDPriority::NUM_PRIORITIES)) return;

    DisplayRequest& req = channels_[ch_idx].requests[pri_idx];
    req.active = true;
    req.pattern = pattern;
    req.color = color;

    if (timeout_ms > 0) {
        req.expire_time_ms = (esp_timer_get_time() / 1000) + timeout_ms;
    } else {
        req.expire_time_ms = 0;  // 永続
    }
}

void LEDManager::releaseChannel(LEDChannel channel, LEDPriority priority)
{
    if (!initialized_) return;

    int ch_idx = static_cast<int>(channel);
    int pri_idx = static_cast<int>(priority);

    if (ch_idx >= static_cast<int>(LEDChannel::NUM_CHANNELS)) return;
    if (pri_idx >= static_cast<int>(LEDPriority::NUM_PRIORITIES)) return;

    channels_[ch_idx].requests[pri_idx].active = false;
}

void LEDManager::setDirect(LEDIndex led, LEDPattern pattern, uint32_t color)
{
    if (!initialized_) return;
    applyToLED(led, pattern, color);
}

void LEDManager::onFlightStateChanged(FlightState state)
{
    if (!initialized_) return;

    LEDPattern pattern = LEDPattern::SOLID;
    uint32_t color = 0x00FF00;  // Default green

    switch (state) {
        case FlightState::INIT:
            pattern = LEDPattern::BREATHE;
            color = 0x0000FF;  // Blue
            break;
        case FlightState::CALIBRATING:
            pattern = LEDPattern::BLINK_FAST;
            color = 0xFFFF00;  // Yellow
            break;
        case FlightState::IDLE:
            pattern = LEDPattern::SOLID;
            color = 0x00FF00;  // Green
            break;
        case FlightState::ARMED:
            pattern = LEDPattern::BLINK_SLOW;
            color = 0x00FF00;  // Green blink
            break;
        case FlightState::FLYING:
            pattern = LEDPattern::SOLID;
            color = 0xFFFF00;  // Yellow
            break;
        case FlightState::LANDING:
            pattern = LEDPattern::BLINK_FAST;
            color = 0x00FF00;  // Green fast blink
            break;
        case FlightState::ERROR:
            pattern = LEDPattern::BLINK_FAST;
            color = 0xFF0000;  // Red
            break;
    }

    requestChannel(LEDChannel::FLIGHT, LEDPriority::FLIGHT_STATE, pattern, color);
}

void LEDManager::onBatteryStateChanged(float voltage, bool low_battery)
{
    if (!initialized_) return;

    if (low_battery) {
        // 低電圧警告（シアン点滅）
        requestChannel(LEDChannel::STATUS, LEDPriority::LOW_BATTERY,
                       LEDPattern::BLINK_SLOW, 0x00FFFF);
    } else {
        // 正常（緑点灯）
        releaseChannel(LEDChannel::STATUS, LEDPriority::LOW_BATTERY);
    }
}

void LEDManager::onSensorHealthChanged(bool all_healthy)
{
    if (!initialized_) return;

    if (!all_healthy) {
        // センサー異常（赤点滅）
        requestChannel(LEDChannel::STATUS, LEDPriority::CRITICAL_ERROR,
                       LEDPattern::BLINK_FAST, 0xFF0000);
    } else {
        releaseChannel(LEDChannel::STATUS, LEDPriority::CRITICAL_ERROR);
    }
}

void LEDManager::update()
{
    if (!initialized_) return;

    uint32_t now_ms = esp_timer_get_time() / 1000;

    // タイムアウト処理
    for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
        for (int pri = 0; pri < static_cast<int>(LEDPriority::NUM_PRIORITIES); pri++) {
            DisplayRequest& req = channels_[ch].requests[pri];
            if (req.active && req.expire_time_ms > 0 && now_ms >= req.expire_time_ms) {
                req.active = false;
            }
        }
    }

    // 各チャンネルの最優先要求を適用
    for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
        const DisplayRequest* req = getActiveRequest(static_cast<LEDChannel>(ch));
        LEDIndex target = channels_[ch].target_led;

        if (req != nullptr) {
            // 変更があった場合のみ適用
            int led_idx = (target == LEDIndex::MCU) ? 0 :
                          (target == LEDIndex::BODY_TOP) ? 1 : 2;
            if (current_state_[led_idx].pattern != req->pattern ||
                current_state_[led_idx].color != req->color) {
                applyToLED(target, req->pattern, req->color);
                current_state_[led_idx].pattern = req->pattern;
                current_state_[led_idx].color = req->color;
            }
        }
    }

    // LEDアニメーション更新
    mcu_led_.update();
    body_led_.update();
}

void LEDManager::setBrightness(uint8_t brightness, bool save_to_nvs)
{
    if (!initialized_) return;
    mcu_led_.setBrightness(brightness, save_to_nvs);
    body_led_.setBrightness(brightness, false);  // NVSには1回だけ保存
}

uint8_t LEDManager::getBrightness() const
{
    if (!initialized_) return 32;
    return mcu_led_.getBrightness();
}

const LEDManager::DisplayRequest* LEDManager::getActiveRequest(LEDChannel channel) const
{
    int ch_idx = static_cast<int>(channel);
    if (ch_idx >= static_cast<int>(LEDChannel::NUM_CHANNELS)) return nullptr;

    // 優先度の高い順（低い数値から）にチェック
    for (int pri = 0; pri < static_cast<int>(LEDPriority::NUM_PRIORITIES); pri++) {
        const DisplayRequest& req = channels_[ch_idx].requests[pri];
        if (req.active) {
            return &req;
        }
    }
    return nullptr;
}

void LEDManager::applyToLED(LEDIndex led, LEDPattern pattern, uint32_t color)
{
    LED::Pattern hal_pattern = toHALPattern(pattern);

    switch (led) {
        case LEDIndex::MCU:
            mcu_led_.setPattern(hal_pattern, color);
            break;
        case LEDIndex::BODY_TOP:
            // GPIO39の最初のLED
            body_led_.setPattern(hal_pattern, color);
            // TODO: 個別LED制御が必要な場合はLED HALを拡張
            break;
        case LEDIndex::BODY_BOTTOM:
            // GPIO39の2番目のLED
            // 現在のHALは全LEDを同じパターンにするため、
            // 個別制御が必要な場合はHALを拡張する必要がある
            body_led_.setPattern(hal_pattern, color);
            break;
        case LEDIndex::ALL:
            mcu_led_.setPattern(hal_pattern, color);
            body_led_.setPattern(hal_pattern, color);
            break;
        default:
            break;
    }
}

LED::Pattern LEDManager::toHALPattern(LEDPattern pattern)
{
    switch (pattern) {
        case LEDPattern::OFF:        return LED::Pattern::OFF;
        case LEDPattern::SOLID:      return LED::Pattern::SOLID;
        case LEDPattern::BLINK_SLOW: return LED::Pattern::BLINK_SLOW;
        case LEDPattern::BLINK_FAST: return LED::Pattern::BLINK_FAST;
        case LEDPattern::BREATHE:    return LED::Pattern::BREATHE;
        case LEDPattern::RAINBOW:    return LED::Pattern::RAINBOW;
        default:                     return LED::Pattern::OFF;
    }
}

}  // namespace stampfly
