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
 * @design architecture.md §2 — Notification (responsibility #14)       [OK]
 * @design architecture.md §5 — Notify data flow (LED/buzzer)           [OK]
 */

#include "notify.hpp"
#include "topics.hpp"
#include "params.hpp"   // low-battery LED threshold (safety.battery.low_v)
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "notify";

namespace sf {

// =============================================================================
// LED colours and patterns — matched to the legacy vehicle's LEDManager scheme.
// LED 色とパターン — 旧 vehicle の LEDManager 配色に合わせる。
//
// @design architecture.md §5 — Notify data flow (LED pattern per state) [OK]
// =============================================================================

// Named colours (RGB), so the table below has no magic numbers.
// 名前付き色（RGB）。下のテーブルにマジックナンバーを置かないため。
static constexpr LedColor kWhite       = {255, 255, 255};
static constexpr LedColor kGreen       = {0, 255, 0};
static constexpr LedColor kBlue        = {0, 0, 255};
static constexpr LedColor kCyan        = {0, 255, 255};
static constexpr LedColor kMagenta     = {255, 0, 255};
static constexpr LedColor kOrange      = {255, 128, 0};   // ALT_HOLD mode / 高度保持
static constexpr LedColor kYellowGreen = {128, 255, 0};   // STABILIZE mode / 姿勢安定

// Blink timings [ms]. SOLID = always on (off_ms 0). / 点滅タイミング。SOLID は常灯。
static constexpr uint16_t kSolidOn = 1000, kSolidOff = 0;
static constexpr uint16_t kSlowOn  = 500,  kSlowOff  = 500;   // 1 Hz blink / 低速点滅
static constexpr uint16_t kFastOn  = 100,  kFastOff  = 100;   // 5 Hz blink / 高速点滅

// Flight-state LED table (legacy colours): white(INIT)→green(IDLE)→green-blink(ARMED)→
// mode colour(FLYING)→orange-blink(LANDING). FLYING is replaced by flyingPattern().
// 飛行状態 LED テーブル（旧の色）。FLYING は flyingPattern() が置き換える。
static const LedPattern kPatternTable[FLIGHT_STATE_COUNT] = {
    { kWhite,  kSolidOn, kSolidOff },  // INIT:         white solid     / 白 常灯（静置）
    { kGreen,  kSolidOn, kSolidOff },  // IDLE_GROUND:  green solid      / 緑 常灯（ARM可）
    { kCyan,   kFastOn,  kFastOff  },  // IDLE_HELD:    cyan fast blink  / シアン 高速点滅
    { kGreen,  kSlowOn,  kSlowOff  },  // ARMED_GROUND: green slow blink / 緑 低速点滅
    { kWhite,  kFastOn,  kFastOff  },  // TAKEOFF:      white fast blink / 白 高速点滅
    { kGreen,  kSolidOn, kSolidOff },  // FLYING:       (overridden by flight-mode colour)
    { kOrange, kSlowOn,  kSlowOff  },  // LANDING:      orange slow blink/ オレンジ 低速点滅
};

// Overlay patterns (priority over the flight-state table, see computeActivePattern):
// オーバーレイ（飛行状態テーブルより優先、computeActivePattern 参照）:
static constexpr LedPattern kLowBatteryPattern = { kCyan,    kSlowOn, kSlowOff };  // 低電圧
static constexpr LedPattern kPairingPattern    = { kBlue,    kFastOn, kFastOff };  // 探索中
static constexpr LedPattern kCalibratingPattern= { kMagenta, kSlowOn, kSlowOff };  // 校正中

// Below this voltage the sensor_power reading is "unknown" (power monitor absent or a
// failed read publishes 0) — do NOT show the low-battery LED for it.
// この電圧未満は sensor_power の読みが「不明」（電源モニタ不在 or 0 発行）— 低電圧表示しない。
static constexpr float kVoltageValidMin = 0.1f;

// -----------------------------------------------------------------------------
// init — initialize LED and buzzer HAL
// 初期化 — LEDとブザーHALを初期化
// -----------------------------------------------------------------------------
void Notify::init(const NotifyConfig& config)
{
    body_led_count_ = config.led_count;
    mcu_led_count_  = config.mcu_led_count;

    // Two independent WS2812 strips (see the class comment for the UI design):
    // body LEDs = flight-mode channel, MCU LED = system-state channel. The HAL
    // guards its own calls, so a failed init simply makes setColor() a no-op.
    // The MCU LED GPIO is also used by sf_board's FATAL halt path (fast red
    // blink) — that path never returns, so there is no runtime conflict.
    // 独立した 2 つの WS2812（UI 設計はクラスコメント参照）: 本体 LED=モード色
    // チャネル、MCU LED=システム状態チャネル。HAL が自前でガードするので init
    // 失敗時は setColor() が no-op になるだけ。MCU LED の GPIO は sf_board の
    // 致命停止経路（赤高速点滅）も使うが、あちらは戻らないため実行時競合はない。
    stampfly::LED::Config led_cfg{};
    led_cfg.gpio     = config.led_gpio;
    led_cfg.num_leds = config.led_count;
    body_led_.init(led_cfg);

    stampfly::LED::Config mcu_cfg{};
    mcu_cfg.gpio     = config.mcu_led_gpio;
    mcu_cfg.num_leds = config.mcu_led_count;
    mcu_led_.init(mcu_cfg);

    // LEDC buzzer (timer separate from the motor's timer 0).
    // LEDC ブザー (モータのタイマ0とは別タイマ)。
    stampfly::Buzzer::Config buz_cfg{};
    buz_cfg.gpio         = config.buzzer_gpio;
    buz_cfg.ledc_channel = config.buzzer_ledc_channel;
    buz_cfg.ledc_timer   = config.buzzer_ledc_timer;
    buzzer_.init(buz_cfg);

    // Cache the low-battery LED threshold (params are loaded in Phase 3, before this
    // task runs). The value rarely changes at runtime, so a one-time read suffices.
    // 低電圧 LED 閾値をキャッシュ（params は Phase 3 で本タスク起動前に読込済）。実行時に
    // ほぼ変わらないので一度読めば十分。
    params::get_float("safety.battery.low_v", low_v_threshold_);

    // Restore the buzzer mute setting. The power-on chime (startTone, the
    // legacy vehicle's C5→E5→G5 melody) is NOT played here but DEFERRED ~2s
    // into update(): `sf flash` resets the chip and esptool enters download
    // mode within ~1s of the reboot — if that lands mid-melody, the LEDC PWM
    // state survives the download-mode entry and the buzzer drones for the
    // whole write. Playing at ~3s keeps the chime clear of that window (a
    // flash-bound reboot never reaches it). The "ready to arm" chime
    // (readyTone) still follows the boot calibration (NotifyEvent::Ready).
    // ブザーの mute 設定を復元する。起動音（startTone, 旧 vehicle の C5→E5→G5）は
    // ここでは鳴らさず update() 側で約2秒「遅延」させる: `sf flash` はチップを
    // リセットし、再起動から約1秒以内に esptool がダウンロードモードへ突入する —
    // それがメロディ再生中に重なると LEDC の PWM 状態がダウンロードモード突入後も
    // 残り、書き込み中ずっとブザーが鳴り続ける。約3秒時点での再生ならその窓に
    // 入らない（フラッシュ目的の再起動はそこまで到達しない）。「ARM 可能」音
    // （readyTone）は従来どおり起動校正完了に続く（NotifyEvent::Ready）。
    buzzer_.loadFromNVS();

    ESP_LOGI(TAG, "Notify initialized (body LED gpio=%d x%d, MCU LED gpio=%d, "
                  "buzzer gpio=%d)",
             config.led_gpio, config.led_count, config.mcu_led_gpio,
             config.buzzer_gpio);
}

// -----------------------------------------------------------------------------
// computeStatePattern — system-state channel (MCU LED) by priority overlay.
// computeStatePattern — システム状態チャネル（MCU LED）。優先度オーバーレイ。
//
// Mirrors the legacy vehicle's LEDManager priority order:
//   low-battery > pairing > calibrating > flight state.
// 旧 vehicle の LEDManager 優先度を踏襲: 低電圧 > ペアリング > 校正中 > 飛行状態。
// -----------------------------------------------------------------------------
LedPattern Notify::computeStatePattern() const
{
    // 1. Low battery (cyan slow blink) — a VALID reading at/below the threshold.
    // 1. 低電圧（シアン低速点滅）— 有効な読みが閾値以下。
    const float voltage = sensor_power.latest().voltage;
    if (voltage > kVoltageValidMin && voltage <= low_v_threshold_) {
        return kLowBatteryPattern;
    }

    // 2. Pairing (blue fast blink) — searching for a transmitter.
    // 2. ペアリング（青高速点滅）— 送信機を探索中。
    if (pairing_state.latest().state == static_cast<uint8_t>(PairingState::Pairing)) {
        return kPairingPattern;
    }

    // 3. Calibrating (magenta slow blink) — on the ground, boot bias not yet done.
    // 3. 校正中（マゼンタ低速点滅）— 地上、起動バイアス校正が未完了。
    const SystemMode mode = system_mode.latest();
    const FlightState state = static_cast<FlightState>(mode.state);
    if (state == FlightState::IDLE_GROUND && !system_status.latest().calibrated) {
        return kCalibratingPattern;
    }

    // 4. Flight state (table). The flight-mode colour lives on the BODY LEDs
    // (computeModePattern) — here FLYING is plain green solid.
    // 4. 飛行状態（テーブル）。モード色は本体 LED（computeModePattern）が担当 —
    // ここでは FLYING は緑の常灯。
    return getPattern(state);
}

// -----------------------------------------------------------------------------
// computeModePattern — flight-mode channel (body LEDs).
// Shows the ACTIVE flight mode (sub_mode) everywhere: the state machine accepts
// mode changes on the ground (IDLE_GROUND/ARMED_GROUND), so flipping the
// transmitter switch while parked changes the actual mode — and this LED —
// immediately. Solid = stationary on the ground, slow blink = airborne. Low
// battery overrides with cyan fast blink — the body LEDs are what the pilot
// sees in flight, so the urgent overlay must live here too.
// computeModePattern — フライトモードチャネル（本体 LED）。
// 常に「実モード」（sub_mode）を表示する: 状態機械が地上（IDLE_GROUND/ARMED_GROUND）
// でのモード変更を受理するため、設置中にスイッチを切り替えると実モードも —
// この LED も — 即座に変わる。常灯=地上静止、低速点滅=空中。低電圧はシアン
// 高速点滅で上書き — 飛行中にパイロットが見るのは本体 LED のため、緊急表示は
// こちらにも必要。
// -----------------------------------------------------------------------------
LedPattern Notify::computeModePattern() const
{
    // Urgent overlay: low battery (valid reading at/below threshold).
    // 緊急オーバーレイ: 低電圧（有効な読みが閾値以下）。
    const float voltage = sensor_power.latest().voltage;
    if (voltage > kVoltageValidMin && voltage <= low_v_threshold_) {
        return { kCyan, kFastOn, kFastOff };
    }

    const SystemMode mode = system_mode.latest();
    const FlightState state = static_cast<FlightState>(mode.state);

    // Booting: mirror INIT white until the system is up.
    // 起動中: システムが立ち上がるまで INIT の白をそのまま。
    if (state == FlightState::INIT) {
        return getPattern(state);
    }

    // Airborne: ACTIVE mode, slow blink — motion is signalled by blinking
    // (pilot request 2026-06-12; ARM state on the ground stays visible on the
    // MCU LED's state channel, which blinks green for ARMED_GROUND).
    // 空中: 実モードの低速点滅 — 「動いている」ことを点滅で示す（2026-06-12 の
    // パイロット要望。地上での ARM 状態は MCU LED の状態チャネルが緑点滅で表示）。
    if (isAirborne(state)) {
        return { modeColor(static_cast<FlightMode>(mode.sub_mode)),
                 kSlowOn, kSlowOff };
    }

    // On the ground (idle, held, or armed): ACTIVE mode (follows the switch),
    // solid — the craft is stationary, so the light is steady.
    // 地上（IDLE/手持ち/ARM 中）: 実モード（スイッチに追従）の常灯 — 機体が
    // 静止しているので灯りも静止。
    return { modeColor(static_cast<FlightMode>(mode.sub_mode)),
             kSolidOn, kSolidOff };
}

// -----------------------------------------------------------------------------
// modeColor — flight-mode colour (legacy vehicle mode colours).
// modeColor — モード色（旧 vehicle のモード色）。
// -----------------------------------------------------------------------------
LedColor Notify::modeColor(FlightMode mode) const
{
    switch (mode) {
        case FlightMode::ACRO:      return kBlue;         // 青
        case FlightMode::STABILIZE: return kYellowGreen;  // 黄緑
        case FlightMode::ALT_HOLD:  return kOrange;       // オレンジ
        case FlightMode::POS_HOLD:  return kMagenta;      // マゼンタ
        default:                    return kGreen;
    }
}

// -----------------------------------------------------------------------------
// update — read system_mode topic, apply LED pattern
// 更新 — system_modeトピックを読み、LEDパターンを適用
// -----------------------------------------------------------------------------
void Notify::update()
{
    // Boot mute window (flash-safety, see notify.hpp): while it counts down, ALL
    // buzzer tones are suppressed (playEvent/playTone). When it reaches 0, play the
    // deferred power-on chime once. A flash-bound reboot enters download mode within
    // the window, so no LEDC tone is ever active when esptool resets the chip.
    // 起動ミュート窓（フラッシュ安全、notify.hpp 参照）: カウントダウン中は全ブザー音を
    // 抑止（playEvent/playTone）。0 で遅延起動音を1回再生する。フラッシュ目的の再起動は
    // 窓内にダウンロードモードへ入るため、esptool リセット時に鳴っている LEDC 音は無い。
    if (boot_mute_countdown_ > 0 && --boot_mute_countdown_ == 0) {
        buzzer_.startTone();
    }

    // Two independent LED channels (class comment): MCU LED = system state,
    // body LEDs = flight-mode colour (preview on the ground, active in the air).
    // 独立した 2 チャネル（クラスコメント参照）: MCU LED=システム状態、
    // 本体 LED=モード色（地上はプレビュー、空中は実モード）。
    applyPattern(computeStatePattern(), mcu_channel_, mcu_led_, mcu_led_count_);
    applyPattern(computeModePattern(), body_channel_, body_led_, body_led_count_);

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

    // UI commands (CLI → here): apply buzzer mute / LED brightness and persist to NVS.
    // Topic-based so a future WiFi/UDP path can inject the same commands.
    // UI コマンド（CLI → ここ）: ブザー mute / LED 輝度を適用し NVS に保存。トピック経由ゆえ
    // 将来 WiFi/UDP からも同じコマンドを注入できる。
    UiCommand uic;
    while (ui_command.read(uic)) {
        switch (static_cast<UiCmd>(uic.command)) {
            case UiCmd::SoundMute:
                buzzer_.setMuted(uic.value != 0, /*save_to_nvs=*/true);
                break;
            case UiCmd::LedBrightness:
                // Same brightness for both channels; persist once (shared NVS key).
                // 両チャネル同輝度。保存は1回（NVS キーは共有）。
                body_led_.setBrightness(uic.value, /*save_to_nvs=*/true);
                mcu_led_.setBrightness(uic.value, /*save_to_nvs=*/false);
                break;
            case UiCmd::None:
            default:
                break;
        }
    }
}

// -----------------------------------------------------------------------------
// playEvent — buzzer sequence for a discrete notify event (notify_command)
// playEvent — 離散通知イベント用のブザー（notify_command）
// -----------------------------------------------------------------------------
void Notify::playEvent(NotifyEvent event)
{
    // Boot mute window (flash-safety): drop every tone so none is mid-play when
    // esptool enters download mode. The calibration-start beep falls here; the
    // ready chime fires after calibration (~4s), past the window.
    // 起動ミュート窓（フラッシュ安全）: esptool のダウンロードモード突入時に音が鳴って
    // いないよう全音を捨てる。校正開始 beep はここに入る。ready 音は校正後（≈4秒）で窓外。
    if (boot_mute_countdown_ > 0) return;

    switch (event) {
        case NotifyEvent::ArmTone:     buzzer_.armTone();           break;
        case NotifyEvent::DisarmTone:  buzzer_.disarmTone();        break;
        case NotifyEvent::LowBattery:  buzzer_.lowBatteryWarning(); break;
        case NotifyEvent::Calibrating: buzzer_.beep();              break;
        case NotifyEvent::Ready:       buzzer_.readyTone();         break;
        case NotifyEvent::PairingMode: buzzer_.pairingTone();       break;
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
    // Boot mute window (flash-safety): no tone during esptool's download-mode entry.
    // 起動ミュート窓（フラッシュ安全）: esptool のダウンロードモード突入中は鳴らさない。
    if (boot_mute_countdown_ > 0) return;
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
// applyPattern — toggle one LED channel based on on/off timing
// applyPattern — ON/OFFタイミングに基づき1チャネルのLEDを切り替え
// -----------------------------------------------------------------------------
void Notify::applyPattern(const LedPattern& pattern, LedChannelState& channel,
                          stampfly::LED& led, int count)
{
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    // Solid mode (off_ms == 0): always on
    // 常灯モード (off_ms == 0): 常にON
    if (pattern.off_ms == 0) {
        setLeds(led, count, pattern.color);
        return;
    }

    // Blink mode: toggle based on timing
    // 点滅モード: タイミングに基づき切り替え
    uint32_t period = channel.on ? pattern.on_ms : pattern.off_ms;
    if (now_ms - channel.last_toggle_ms >= period) {
        channel.on = !channel.on;
        channel.last_toggle_ms = now_ms;
        setLeds(led, count, channel.on ? pattern.color : LedColor{0, 0, 0});
    }
}

// -----------------------------------------------------------------------------
// setLeds — set every LED of one strip to one color (0xRRGGBB packed for the HAL)
// setLeds — 1 ストリップの全 LED を 1 色に（HAL 用に 0xRRGGBB へパック）
// -----------------------------------------------------------------------------
void Notify::setLeds(stampfly::LED& led, int count, const LedColor& color)
{
    // Apply the strip's brightness HERE: the HAL's brightness_ scaling lives in
    // LED::update(), but Notify drives the strips via setColor() (it never calls
    // update()), so without this the CLI `led <n>` brightness command — and the
    // ~12% default — would be a silent no-op and the LEDs would blaze at full
    // brightness (code_review M-4).
    // 輝度をここで適用する: HAL の brightness_ スケーリングは LED::update() にあるが、
    // Notify は setColor() 経路で駆動し update() を呼ばないため、これが無いと CLI
    // `led <n>` 輝度コマンド（と既定 ~12%）が無効化され LED が常時フル輝度になる (M-4)。
    const uint8_t bri = led.getBrightness();
    const uint8_t r = static_cast<uint8_t>(color.r * bri / 255);
    const uint8_t g = static_cast<uint8_t>(color.g * bri / 255);
    const uint8_t b = static_cast<uint8_t>(color.b * bri / 255);
    const uint32_t packed = (static_cast<uint32_t>(r) << 16) |
                            (static_cast<uint32_t>(g) << 8)  |
                             static_cast<uint32_t>(b);
    for (int i = 0; i < count; ++i) {
        led.setColor(static_cast<uint8_t>(i), packed);
    }
}

}  // namespace sf
