/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file button_task.cpp
 * @brief Button input task (50Hz)
 *        ボタン入力タスク（50Hz）
 *
 * Polls the on-board button (GPIO0), detects click/long-press gestures, and
 * publishes each gesture as a FACT on the button_event topic. It does NOT decide
 * the action: the StateManager (sole transition authority) maps a click to an
 * ARM/DISARM toggle (architecture §2 — detection reports, state management decides;
 * R5 — cross-component flow by topic, not a direct call).
 *
 * 機体ボタン（GPIO0）をポーリングし、クリック/長押しジェスチャを検出して、
 * button_event トピックに「事実」として発行する。動作の判断はしない: 判断は唯一の
 * 遷移実行者 StateManager が行い、クリックを ARM/DISARM トグルに対応づける
 * （architecture §2 — 検出は報告・判断は状態管理、R5 — コンポーネント間はトピック経由）。
 *
 * @publisher button_event
 * @design architecture.md §6 — ButtonTask                            [OK]
 * @design architecture.md §2 — detection reports, state decides       [OK]
 * @design detailed_design.md §8 — ButtonTask: 50Hz, priority 10      [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "button.hpp"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "ButtonTask";

namespace {

/// Map a HAL Button::Event to the published ButtonGesture. Events we don't surface
/// (press-start, none) map to None and are not published.
/// HAL の Button::Event を発行用 ButtonGesture に対応づける。発行しないイベント
/// （押下開始・なし）は None にマップし、publish しない。
sf::ButtonGesture toGesture(stampfly::Button::Event event)
{
    using HalEvent = stampfly::Button::Event;
    switch (event) {
    case HalEvent::CLICK:         return sf::ButtonGesture::Click;
    case HalEvent::DOUBLE_CLICK:  return sf::ButtonGesture::DoubleClick;
    case HalEvent::LONG_PRESS_3S: return sf::ButtonGesture::LongPress3s;
    case HalEvent::LONG_PRESS_5S: return sf::ButtonGesture::LongPress5s;
    default:                      return sf::ButtonGesture::None;  // NONE / LONG_PRESS_START
    }
}

}  // namespace

void ButtonTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ButtonTask started");

    // The button is Optional hardware: if init fails the task still runs (no gestures),
    // so a missing/faulty button never blocks the rest of the system.
    // ボタンは Optional ハードウェア: init 失敗でもタスクは動き続ける（ジェスチャなし）。
    // ボタン不在/不良がシステム全体を止めないようにする。
    stampfly::Button button;
    stampfly::Button::Config cfg{};
    cfg.gpio        = config::GPIO_BUTTON;
    cfg.debounce_ms = static_cast<uint32_t>(config::BUTTON_DEBOUNCE_MS);
    if (button.init(cfg) != ESP_OK) {
        ESP_LOGW(TAG, "Button init failed — gestures disabled");
    }

    // On each detected gesture, publish it as a fact. The callback runs inside tick()
    // on this task's stack, so publishing here is single-threaded and safe.
    // 検出したジェスチャを「事実」として publish する。コールバックは tick() 内・本タスクの
    // スタックで走るため、ここでの publish は単一スレッドで安全。
    button.setCallback([](stampfly::Button::Event event) {
        const sf::ButtonGesture gesture = toGesture(event);
        if (gesture == sf::ButtonGesture::None) {
            return;
        }
        sf::button_event.publish({static_cast<uint8_t>(gesture),
                                  static_cast<uint32_t>(esp_timer_get_time())});
    });

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        button.tick();  // poll GPIO, debounce, fire the gesture callback
        vTaskDelayUntil(&last_wake, period);
    }
}
