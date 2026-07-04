/**
 * @file led_manager.cpp
 * @brief LED管理システム実装
 *
 * LED役割分担:
 *   SYSTEM (StampS3内蔵 GPIO21): モード + キャリブレーション（地上確認用）
 *   BODY (上面+下面 GPIO39):     フライト状態 + 警告（飛行中に見える情報）
 *
 * BODY チャンネルは上面・下面の両LEDに同一パターン・色を出力する。
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

    // MCU LED init (GPIO21, 1 LED) — SYSTEM channel
    // MCU LED初期化 (GPIO21, 1個) — SYSTEMチャンネル
    LED::Config mcu_cfg;
    mcu_cfg.gpio = 21;
    mcu_cfg.num_leds = 1;
    esp_err_t ret = mcu_led_.init(mcu_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init MCU LED: %s", esp_err_to_name(ret));
        return ret;
    }

    // Body LED init (GPIO39, 2 LEDs) — BODY channel (top + bottom, same display)
    // Body LED初期化 (GPIO39, 2個) — BODYチャンネル（上面+下面、同一表示）
    LED::Config body_cfg;
    body_cfg.gpio = 39;
    body_cfg.num_leds = 2;
    ret = body_led_.init(body_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Body LED: %s", esp_err_to_name(ret));
        return ret;
    }

    // Default state: green solid on all channels
    // デフォルト: 全チャンネル緑点灯
    for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].active = true;
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].pattern = LEDPattern::SOLID;
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].color = 0x00FF00;
        channels_[ch].requests[static_cast<int>(LEDPriority::DEFAULT)].expire_time_ms = 0;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "LED Manager initialized (SYSTEM=MCU, BODY=TOP+BOTTOM)");

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
        req.expire_time_ms = 0;
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

void LEDManager::setSystemEventsEnabled(bool enabled)
{
    system_events_enabled_ = enabled;
    if (!enabled) {
        for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
            auto channel = static_cast<LEDChannel>(ch);
            releaseChannel(channel, LEDPriority::FLIGHT_STATE);
            releaseChannel(channel, LEDPriority::CALIBRATION);
            releaseChannel(channel, LEDPriority::LOW_BATTERY);
            releaseChannel(channel, LEDPriority::PAIRING);
            releaseChannel(channel, LEDPriority::DEBUG_ALERT);
        }
    }
}

// =========================================================================
// Event handlers — map system events to LED channels
// イベントハンドラ — システムイベントをLEDチャンネルにマッピング
// =========================================================================

void LEDManager::onFlightStateChanged(FlightState state)
{
    if (!initialized_ || !system_events_enabled_) return;

    // BODY channel: flight state (visible during flight)
    // BODYチャンネル: フライト状態（飛行中に見える）
    LEDPattern body_pattern = LEDPattern::SOLID;
    uint32_t body_color = 0x00FF00;

    switch (state) {
        case FlightState::IDLE:
            body_pattern = LEDPattern::SOLID;
            body_color = 0x00FF00;  // Green: ready to ARM
            break;
        case FlightState::ARMED:
            body_pattern = LEDPattern::BLINK_SLOW;
            body_color = 0x00FF00;  // Green blink: armed
            break;
        case FlightState::FLYING:
            body_pattern = LEDPattern::SOLID;
            body_color = 0xFFFF00;  // Yellow: flying
            break;
        case FlightState::ERROR:
            body_pattern = LEDPattern::BLINK_FAST;
            body_color = 0xFF0000;  // Red: error
            break;
        default:
            // INIT, CALIBRATING, LANDING — no change to BODY
            return;
    }

    requestChannel(LEDChannel::BODY, LEDPriority::FLIGHT_STATE, body_pattern, body_color);
}

void LEDManager::onFlightModeChanged(FlightMode mode)
{
    if (!initialized_ || !system_events_enabled_) return;

    // SYSTEM channel (StampS3 LED): flight mode
    // SYSTEMチャンネル（StampS3 LED）: フライトモード
    uint32_t color;
    switch (mode) {
        case FlightMode::STABILIZE:     color = 0x80FF00; break;  // Yellow-green
        case FlightMode::ALTITUDE_HOLD: color = 0xFF8000; break;  // Orange
        case FlightMode::POSITION_HOLD: color = 0xFF00FF; break;  // Magenta
        case FlightMode::ACRO:
        default:                        color = 0x0000FF; break;  // Blue
    }
    requestChannel(LEDChannel::SYSTEM, LEDPriority::FLIGHT_STATE,
                   LEDPattern::SOLID, color);
}

void LEDManager::onBatteryStateChanged(float voltage, bool low_battery)
{
    if (!initialized_ || !system_events_enabled_) return;

    // BODY channel: low battery warning (cyan blink overlays flight state)
    // BODYチャンネル: 低電圧警告（シアン点滅がフライト状態に重畳）
    if (low_battery) {
        requestChannel(LEDChannel::BODY, LEDPriority::LOW_BATTERY,
                       LEDPattern::BLINK_SLOW, 0x00FFFF);
    } else {
        releaseChannel(LEDChannel::BODY, LEDPriority::LOW_BATTERY);
    }
}

void LEDManager::onSensorHealthChanged(bool all_healthy)
{
    if (!initialized_) return;

    // BODY channel: sensor error (red blink, highest priority)
    // BODYチャンネル: センサ異常（赤点滅、最高優先度）
    if (!all_healthy) {
        requestChannel(LEDChannel::BODY, LEDPriority::CRITICAL_ERROR,
                       LEDPattern::BLINK_FAST, 0xFF0000);
    } else {
        releaseChannel(LEDChannel::BODY, LEDPriority::CRITICAL_ERROR);
    }
}

// =========================================================================
// Update — process timeouts and apply to physical LEDs
// 更新 — タイムアウト処理と物理LEDへの適用
// =========================================================================

void LEDManager::update()
{
    if (!initialized_) return;

    uint32_t now_ms = esp_timer_get_time() / 1000;

    // Process timeouts
    // タイムアウト処理
    for (int ch = 0; ch < static_cast<int>(LEDChannel::NUM_CHANNELS); ch++) {
        for (int pri = 0; pri < static_cast<int>(LEDPriority::NUM_PRIORITIES); pri++) {
            DisplayRequest& req = channels_[ch].requests[pri];
            if (req.active && req.expire_time_ms > 0 && now_ms >= req.expire_time_ms) {
                req.active = false;
            }
        }
    }

    // SYSTEM channel → MCU LED
    // SYSTEMチャンネル → MCU LED
    {
        const DisplayRequest* req = getActiveRequest(LEDChannel::SYSTEM);
        if (req && (current_state_[0].pattern != req->pattern ||
                    current_state_[0].color != req->color)) {
            applyToLED(LEDIndex::MCU, req->pattern, req->color);
            current_state_[0].pattern = req->pattern;
            current_state_[0].color = req->color;
        }
    }

    // BODY channel → BODY_TOP + BODY_BOTTOM (same display)
    // BODYチャンネル → 上面 + 下面（同一表示）
    {
        const DisplayRequest* req = getActiveRequest(LEDChannel::BODY);
        if (req && (current_state_[1].pattern != req->pattern ||
                    current_state_[1].color != req->color)) {
            // Apply to both body LEDs
            // 両方のボディLEDに適用
            applyToLED(LEDIndex::BODY_TOP, req->pattern, req->color);
            applyToLED(LEDIndex::BODY_BOTTOM, req->pattern, req->color);
            current_state_[1].pattern = req->pattern;
            current_state_[1].color = req->color;
        }
    }

    // Update LED animations
    // LEDアニメーション更新
    mcu_led_.update();
    body_led_.update();
}

// =========================================================================
// Utility
// =========================================================================

void LEDManager::setBrightness(uint8_t brightness, bool save_to_nvs)
{
    if (!initialized_) return;
    mcu_led_.setBrightness(brightness, save_to_nvs);
    body_led_.setBrightness(brightness, false);
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
        case LEDIndex::BODY_BOTTOM:
            // Both body LEDs share the same driver (GPIO39)
            // 両ボディLEDは同一ドライバ（GPIO39）を共有
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
