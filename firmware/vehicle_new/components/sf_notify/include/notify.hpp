/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file notify.hpp
 * @brief Notification manager — LED patterns and buzzer tones
 *        通知マネージャー — LEDパターンとブザートーン
 *
 * Maps flight state and alerts to visual/audio feedback:
 * - LED: color + blink pattern per state
 * - Buzzer: tone sequence for events (arm, disarm, alert)
 *
 * フライト状態とアラートを視覚/聴覚フィードバックに変換する:
 * - LED: 状態ごとの色＋点滅パターン
 * - ブザー: イベント（ARM、DISARM、アラート）のトーンシーケンス
 *
 * @design architecture.md §8 — Notification subsystem                  [--]
 * @design detailed_design.md §10 — LED pattern table                   [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
 */

#pragma once

#include "flight_state.hpp"
#include "data_types.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include <cstdint>

namespace sf {

/// LED color (RGB values packed)
/// LED色（RGB値パック）
struct LedColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/// LED pattern entry: color + timing
/// LEDパターンエントリ: 色＋タイミング
struct LedPattern {
    LedColor color;           // LED color        / LED色
    uint16_t on_ms;           // ON duration [ms]  / ON期間
    uint16_t off_ms;          // OFF duration [ms] / OFF期間
};

/// HW configuration for the notification subsystem. Passed in by the caller
/// (notify_task) so this component does not depend on main/config.hpp (which
/// lives under main/ and would create a circular dependency — same convention
/// as sf_actuator).
/// 通知サブシステムの HW 構成。呼び出し側 (notify_task) が渡すことで、本コンポーネントが
/// main/config.hpp に依存しないようにする (main/ にあり循環依存になる — sf_actuator と同方針)。
struct NotifyConfig {
    int led_gpio;             // WS2812 body LED GPIO        / ボディ LED GPIO
    int led_count;            // Number of body LEDs         / ボディ LED 数
    int buzzer_gpio;          // Buzzer GPIO                 / ブザー GPIO
    int buzzer_ledc_channel;  // Buzzer LEDC channel         / ブザー LEDC チャンネル
    int buzzer_ledc_timer;    // Buzzer LEDC timer           / ブザー LEDC タイマー
};

/// Notification manager: LED + buzzer
/// 通知マネージャー: LED＋ブザー
class Notify {
public:
    /// Initialize notification subsystem with the caller-supplied HW config
    /// 呼び出し側提供の HW 構成で通知サブシステムを初期化する
    void init(const NotifyConfig& config);

    /// Update LED/buzzer based on current state (call at ~10Hz)
    /// 現在の状態に基づきLED/ブザーを更新する（約10Hzで呼ぶ）
    void update();

    /// Play one-shot buzzer tone for an event
    /// イベント用のワンショットブザートーンを鳴らす
    void playTone(AlertType type);

    /// Play a one-shot buzzer sequence for a discrete notify event
    /// 離散通知イベント用のワンショットブザー音を鳴らす
    void playEvent(NotifyEvent event);

private:
    /// Get LED pattern for a given flight state
    /// 指定フライト状態のLEDパターンを取得する
    const LedPattern& getPattern(FlightState state) const;

    /// Apply LED pattern (toggle on/off based on timing)
    /// LEDパターンを適用する（タイミングに基づきON/OFFを切り替え）
    void applyLedPattern(const LedPattern& pattern);

    /// Set every body LED to one color (off = {0,0,0})
    /// 全ボディ LED を 1 色に設定する（消灯 = {0,0,0}）
    void setBodyLeds(const LedColor& color);

    stampfly::LED    led_;            // WS2812 body LED HAL  / WS2812 ボディ LED HAL
    stampfly::Buzzer buzzer_;         // LEDC buzzer HAL      / LEDC ブザー HAL
    int      led_count_      = 0;     // Body LED count       / ボディ LED 数
    uint32_t last_toggle_ms_ = 0;    // Last LED toggle time / 最終LED切替時刻
    bool     led_on_         = false; // Current LED state    / 現在のLED状態
};

}  // namespace sf
