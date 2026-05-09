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

/// Notification manager: LED + buzzer
/// 通知マネージャー: LED＋ブザー
class Notify {
public:
    /// Initialize notification subsystem
    /// 通知サブシステムを初期化する
    void init();

    /// Update LED/buzzer based on current state (call at ~10Hz)
    /// 現在の状態に基づきLED/ブザーを更新する（約10Hzで呼ぶ）
    void update();

    /// Play one-shot buzzer tone for an event
    /// イベント用のワンショットブザートーンを鳴らす
    void playTone(AlertType type);

private:
    /// Get LED pattern for a given flight state
    /// 指定フライト状態のLEDパターンを取得する
    const LedPattern& getPattern(FlightState state) const;

    /// Apply LED pattern (toggle on/off based on timing)
    /// LEDパターンを適用する（タイミングに基づきON/OFFを切り替え）
    void applyLedPattern(const LedPattern& pattern);

    uint32_t last_toggle_ms_ = 0;    // Last LED toggle time / 最終LED切替時刻
    bool     led_on_         = false; // Current LED state    / 現在のLED状態
};

}  // namespace sf
