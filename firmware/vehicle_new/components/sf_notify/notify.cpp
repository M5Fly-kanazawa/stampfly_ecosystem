/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file notify.cpp
 * @brief Notification manager implementation
 *        通知マネージャー実装
 *
 * @design architecture.md §8 — Notification subsystem                  [--]
 * @design detailed_design.md §10 — LED pattern table                   [--]
 */

#include "notify.hpp"
#include "topics.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "notify";

namespace sf {

// =============================================================================
// LED pattern table — one entry per FlightState
// LEDパターンテーブル — FlightState毎に1エントリ
//
// @design detailed_design.md §10 — LED pattern definitions              [--]
// =============================================================================

static const LedPattern kPatternTable[FLIGHT_STATE_COUNT] = {
    // INIT:         blue slow blink   / 青 ゆっくり点滅
    { {0, 0, 255},    500, 500 },
    // IDLE_GROUND:  green solid       / 緑 常灯
    { {0, 255, 0},   1000,   0 },
    // IDLE_HELD:    cyan fast blink   / シアン 高速点滅
    { {0, 255, 255},  200, 200 },
    // ARMED_GROUND: yellow solid      / 黄色 常灯
    { {255, 255, 0}, 1000,   0 },
    // TAKEOFF:      white fast blink  / 白 高速点滅
    { {255, 255, 255}, 100, 100 },
    // FLYING:       green solid       / 緑 常灯
    { {0, 255, 0},   1000,   0 },
    // LANDING:      orange slow blink / オレンジ ゆっくり点滅
    { {255, 128, 0},  300, 300 },
};

// -----------------------------------------------------------------------------
// init — initialize LED and buzzer HAL
// 初期化 — LEDとブザーHALを初期化
// -----------------------------------------------------------------------------
void Notify::init(const NotifyConfig& config)
{
    led_count_ = config.led_count;

    // WS2812 body LED. HAL guards its own calls, so a failed init simply makes
    // setColor() a no-op.
    // WS2812 ボディ LED。HAL が自前でガードするので init 失敗時は setColor() が
    // no-op になるだけ。
    stampfly::LED::Config led_cfg{};
    led_cfg.gpio     = config.led_gpio;
    led_cfg.num_leds = config.led_count;
    led_.init(led_cfg);

    // LEDC buzzer (timer separate from the motor's timer 0).
    // LEDC ブザー (モータのタイマ0とは別タイマ)。
    stampfly::Buzzer::Config buz_cfg{};
    buz_cfg.gpio         = config.buzzer_gpio;
    buz_cfg.ledc_channel = config.buzzer_ledc_channel;
    buz_cfg.ledc_timer   = config.buzzer_ledc_timer;
    buzzer_.init(buz_cfg);

    ESP_LOGI(TAG, "Notify initialized (LED gpio=%d x%d, buzzer gpio=%d)",
             config.led_gpio, config.led_count, config.buzzer_gpio);
}

// -----------------------------------------------------------------------------
// update — read system_mode topic, apply LED pattern
// 更新 — system_modeトピックを読み、LEDパターンを適用
// -----------------------------------------------------------------------------
void Notify::update()
{
    // Read current system mode from topic
    // トピックから現在のシステムモードを読み取る
    SystemMode mode = system_mode.latest();
    FlightState state = static_cast<FlightState>(mode.state);

    // Get and apply LED pattern for current state
    // 現在の状態のLEDパターンを取得して適用
    const LedPattern& pattern = getPattern(state);
    applyLedPattern(pattern);

    // Discrete notify events (ARM/DISARM tones, low-battery warning) arrive on the
    // notify_command queue. Notify is its ONLY consumer, so draining it here is
    // safe. We deliberately do NOT read system_alert — state_task owns that queue
    // (failsafe), and a shared FreeRTOS queue would have Notify steal its events.
    // 離散通知イベント（ARM/DISARM 音、低電圧警告）は notify_command キューで届く。
    // 消費者は Notify のみゆえここで drain して安全。system_alert は読まない —
    // それは state_task(failsafe) が所有する共有キューで、読むとイベントを奪う。
    NotifyCommand cmd;
    while (notify_command.read(cmd)) {
        playEvent(static_cast<NotifyEvent>(cmd.event));
    }
}

// -----------------------------------------------------------------------------
// playEvent — buzzer sequence for a discrete notify event (notify_command)
// playEvent — 離散通知イベント用のブザー（notify_command）
// -----------------------------------------------------------------------------
void Notify::playEvent(NotifyEvent event)
{
    switch (event) {
        case NotifyEvent::ArmTone:     buzzer_.armTone();           break;
        case NotifyEvent::DisarmTone:  buzzer_.disarmTone();        break;
        case NotifyEvent::LowBattery:  buzzer_.lowBatteryWarning(); break;
        case NotifyEvent::Calibrating: buzzer_.beep();              break;
        case NotifyEvent::Ready:       buzzer_.beep();              break;
        case NotifyEvent::None:
        default:                                                    break;
    }
}

// -----------------------------------------------------------------------------
// playTone — play buzzer tone for alert event
// トーン再生 — アラートイベント用ブザートーンを再生
// -----------------------------------------------------------------------------
void Notify::playTone(AlertType type)
{
    // Maps a failsafe AlertType to a buzzer sound. Public helper for callers that
    // hold an alert directly; the periodic update() path uses notify_command/
    // playEvent so it does not consume the state_task-owned system_alert queue.
    // failsafe の AlertType をブザー音に対応づける公開ヘルパ。アラートを直接持つ
    // 呼び出し側用。周期 update() は notify_command/playEvent を使い、state_task が
    // 所有する system_alert キューを消費しない。
    switch (type) {
        case AlertType::LOW_BATTERY:
        case AlertType::USB_POWER:     buzzer_.lowBatteryWarning(); break;
        case AlertType::COMM_LOST:
        case AlertType::IMPACT:
        case AlertType::ESKF_DIVERGED:
        case AlertType::GYRO_ANOMALY:  buzzer_.errorTone();         break;
        case AlertType::NONE:
        default:                                                    break;
    }
}

// -----------------------------------------------------------------------------
// getPattern — look up LED pattern by flight state
// パターン取得 — フライト状態でLEDパターンを検索
// -----------------------------------------------------------------------------
const LedPattern& Notify::getPattern(FlightState state) const
{
    uint8_t idx = static_cast<uint8_t>(state);
    if (idx >= FLIGHT_STATE_COUNT) {
        idx = 0;  // Fallback to INIT pattern / INITパターンにフォールバック
    }
    return kPatternTable[idx];
}

// -----------------------------------------------------------------------------
// applyLedPattern — toggle LED based on on/off timing
// LEDパターン適用 — ON/OFFタイミングに基づきLEDを切り替え
// -----------------------------------------------------------------------------
void Notify::applyLedPattern(const LedPattern& pattern)
{
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    // Solid mode (off_ms == 0): always on
    // 常灯モード (off_ms == 0): 常にON
    if (pattern.off_ms == 0) {
        setBodyLeds(pattern.color);
        return;
    }

    // Blink mode: toggle based on timing
    // 点滅モード: タイミングに基づき切り替え
    uint32_t period = led_on_ ? pattern.on_ms : pattern.off_ms;
    if (now_ms - last_toggle_ms_ >= period) {
        led_on_ = !led_on_;
        last_toggle_ms_ = now_ms;
        setBodyLeds(led_on_ ? pattern.color : LedColor{0, 0, 0});
    }
}

// -----------------------------------------------------------------------------
// setBodyLeds — set every body LED to one color (0xRRGGBB packed for the HAL)
// setBodyLeds — 全ボディ LED を 1 色に（HAL 用に 0xRRGGBB へパック）
// -----------------------------------------------------------------------------
void Notify::setBodyLeds(const LedColor& color)
{
    const uint32_t packed = (static_cast<uint32_t>(color.r) << 16) |
                            (static_cast<uint32_t>(color.g) << 8)  |
                             static_cast<uint32_t>(color.b);
    for (int i = 0; i < led_count_; ++i) {
        led_.setColor(static_cast<uint8_t>(i), packed);
    }
}

}  // namespace sf
