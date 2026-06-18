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
 * @design architecture.md §2 — Notification (responsibility #14)       [OK]
 * @design architecture.md §5 — Notify data flow (LED/buzzer)           [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
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
    int mcu_led_gpio;         // M5Stamp S3 built-in LED GPIO / StampS3 内蔵 LED GPIO
    int mcu_led_count;        // Number of MCU LEDs (1)       / MCU LED 数（1）
    int buzzer_gpio;          // Buzzer GPIO                 / ブザー GPIO
    int buzzer_ledc_channel;  // Buzzer LEDC channel         / ブザー LEDC チャンネル
    int buzzer_ledc_timer;    // Buzzer LEDC timer           / ブザー LEDC タイマー
};

/// Per-LED-channel blink bookkeeping (body and MCU LEDs blink independently)
/// LED チャネルごとの点滅状態（本体と MCU の LED は独立に点滅する）
struct LedChannelState {
    uint32_t last_toggle_ms = 0;  // Last toggle time [ms] / 最終切替時刻
    bool     on             = false;  // Currently lit?    / 現在点灯中か
};

/// Notification manager: LED + buzzer.
///
/// Two independent LED channels (UI design):
/// - BODY LEDs (board top+bottom, always identical — one of them is hidden
///   depending on the craft's pose): FLIGHT-MODE colour channel. On the ground
///   it previews the pilot's mode-switch position (slow blink = disarmed,
///   solid = armed); in the air it shows the ACTIVE mode (solid). Low battery
///   overrides with cyan fast blink (must be visible in flight).
/// - MCU LED (M5Stamp S3 built-in, visible on the bench): SYSTEM-STATE channel.
///   INIT/calibrating/pairing/idle/armed/takeoff/flying/landing + overlays.
///
/// 通知マネージャー: LED＋ブザー。
/// 独立した 2 つの LED チャネル（UI 設計）:
/// - 本体 LED（基板の表裏、常に同一 — 機体姿勢によりどちらかは見えないため）:
///   「フライトモード色」チャネル。地上ではモードスイッチ位置のプレビュー
///   （低速点滅=DISARM、常灯=ARM）、空中では実際のモード（常灯）。低電圧は
///   シアン高速点滅で上書き（飛行中に見える必要があるため）。
/// - MCU LED（M5Stamp S3 内蔵、ベンチで見える）: 「システム状態」チャネル。
///   INIT/校正中/ペアリング/IDLE/ARMED/離陸/飛行/着陸＋オーバーレイ。
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
    /// System-state channel pattern (MCU LED), by priority overlay (mirrors the
    /// legacy vehicle's LEDManager priority): low-battery > pairing > calibrating
    /// > flight state.
    /// システム状態チャネルのパターン（MCU LED）。優先度オーバーレイ（旧 vehicle の
    /// LEDManager 優先度を踏襲）: 低電圧 > ペアリング > 校正中 > 飛行状態。
    LedPattern computeStatePattern() const;

    /// Flight-mode channel pattern (body LEDs): mode colour preview/active.
    /// フライトモードチャネルのパターン（本体 LED）: モード色のプレビュー/実モード。
    LedPattern computeModePattern() const;

    /// Get LED pattern for a given flight state (table lookup)
    /// 指定フライト状態のLEDパターンを取得する（テーブル参照）
    const LedPattern& getPattern(FlightState state) const;

    /// Flight-mode colour (ACRO blue / STABILIZE yellow-green / ALT_HOLD orange /
    /// POS_HOLD magenta), matching the legacy vehicle's mode colours.
    /// モード色（ACRO青/STABILIZE黄緑/ALT_HOLD橙/POS_HOLDマゼンタ）、旧 vehicle 準拠。
    LedColor modeColor(FlightMode mode) const;

    /// Apply an LED pattern to one channel (toggle on/off based on timing)
    /// LEDパターンを1チャネルに適用する（タイミングに基づきON/OFFを切り替え）
    void applyPattern(const LedPattern& pattern, LedChannelState& channel,
                      stampfly::LED& led, int count);

    /// Set every LED of one strip to one color (off = {0,0,0})
    /// 1 ストリップの全 LED を 1 色に設定する（消灯 = {0,0,0}）
    void setLeds(stampfly::LED& led, int count, const LedColor& color);

    stampfly::LED    body_led_;       // WS2812 body LEDs (board top+bottom) / 本体 LED（表裏）
    stampfly::LED    mcu_led_;        // M5Stamp S3 built-in LED / StampS3 内蔵 LED
    stampfly::Buzzer buzzer_;         // LEDC buzzer HAL      / LEDC ブザー HAL
    int      body_led_count_ = 0;     // Body LED count       / 本体 LED 数
    int      mcu_led_count_  = 0;     // MCU LED count        / MCU LED 数
    LedChannelState body_channel_;    // Body blink state     / 本体の点滅状態
    LedChannelState mcu_channel_;     // MCU blink state      / MCU の点滅状態
    float    low_v_threshold_ = 3.4f; // Low-battery LED threshold [V] (from params)
                                      // 低電圧 LED 閾値 [V]（params から）
    // Boot mute window: while > 0, EVERY buzzer tone is suppressed and the power-on
    // chime is held. esptool enters download mode ~1-2 s after the reset `sf flash`
    // triggers; ANY LEDC tone active at that instant survives the entry and drones
    // for the whole write. Faster boot (-O2) moved the calibration-start beep into
    // that window, so deferring only the power-on chime was no longer enough — this
    // covers all of them (beep, ready, etc.). 90 cycles ≈ 3 s at 30 Hz. The power-on
    // chime plays as this hits 0; the ready chime (calibration ≈ 4 s) lands after it.
    // 起動ミュート窓: > 0 の間、全ブザー音を抑止し起動音を保留する。esptool は `sf flash`
    // のリセットから約1〜2秒でダウンロードモードに入り、その瞬間に鳴っている LEDC 音は
    // 突入後も残り書き込み中ずっと鳴り続ける。-O2 で起動が速くなり校正開始 beep がこの窓に
    // 入ったため、起動音だけの遅延では不十分になった — 本窓で全音（beep/ready 等）を覆う。
    // 30Hz×90周期≈3秒。起動音は 0 で再生、ready 音（校正≈4秒）は窓の後に自然に鳴る。
    uint8_t  boot_mute_countdown_ = 90;
};

}  // namespace sf
