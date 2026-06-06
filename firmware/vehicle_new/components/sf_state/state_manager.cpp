/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_manager.cpp
 * @brief StateManager implementation
 *        StateManager実装
 *
 * @design requirements.md §4 — Component #3: State Management         [--]
 * @design architecture.md §2 — Sole transition owner                  [--]
 * @design detailed_design.md §3 — State transition table              [--]
 */

#include "state_manager.hpp"
#include "params.hpp"
#include "esp_log.h"

static const char* TAG = "StateManager";

namespace sf {

/// Below this voltage the sensor_power reading is "unknown" (power monitor absent or a
/// failed read publishes 0). A 0/unknown reading must NOT block ARM — power monitoring is
/// Optional (hardware_init.md §5) — so the pre-arm voltage gate ignores readings below it.
/// この電圧未満は sensor_power の読みが「不明」（電源モニタ不在 or 読み失敗で 0 が発行）。
/// 電源監視は Optional（hardware_init.md §5）ゆえ 0/不明は ARM を阻まない — ARM 前電圧ゲートは
/// これ未満の読みを無視する。
static constexpr float kVoltageValidMin = 0.1f;

// =============================================================================
// Initialization
// 初期化
// =============================================================================

void StateManager::init()
{
    state_ = FlightState::INIT;
    mode_ = FlightMode::STABILIZE;
    exit_callback_count_ = 0;
    enter_callback_count_ = 0;
    mode_callback_count_ = 0;

    ESP_LOGI(TAG, "StateManager initialized (state=%s)",
             flightStateName(state_));
}

// =============================================================================
// Transition Requests
// 遷移リクエスト
// =============================================================================

/// @design requirements.md §2 — INIT → IDLE_GROUND on init complete   [--]
void StateManager::notifyInitComplete()
{
    if (state_ != FlightState::INIT) {
        return;   // already past INIT — nothing to do / INIT を過ぎていれば無操作
    }
    ESP_LOGI(TAG, "Init complete → IDLE_GROUND");
    transition(FlightState::IDLE_GROUND);
}

/// @design requirements.md §2 — ARM from GROUND only                  [OK]
/// @design requirements.md §9 — USB power / low-V: ARM prohibited     [OK]
/// @design detailed_design.md §3 — ARM gated on calibration complete  [OK]
bool StateManager::requestArm()
{
    if (state_ != FlightState::IDLE_GROUND) {
        ESP_LOGD(TAG, "ARM rejected: not in IDLE_GROUND (state=%s)",
                 flightStateName(state_));
        return false;
    }

    // --- Pre-arm gate 1: battery / USB power (requirements §9) ------------------------
    // Reject ARM if a VALID voltage reading is at or below the unsafe threshold — either
    // running on the USB rail (no pack) or a critically low battery. A 0/unknown reading
    // (power monitor absent → Optional, hardware_init.md §5) does NOT block. Threshold
    // from the safety.battery.usb_v param.
    // ARM 前ゲート1: 電池/USB 電源（要件§9）。有効な電圧読みが危険閾値以下なら ARM を拒否
    // （USB レール=電池なし or 危険な低電圧）。0/不明（電源モニタ不在→Optional）は阻まない。
    // 閾値は safety.battery.usb_v param から。
    float usb_v = 3.3f;
    params::get_float("safety.battery.usb_v", usb_v);
    const float voltage = sensor_power.latest().voltage;
    if (voltage > kVoltageValidMin && voltage <= usb_v) {
        ESP_LOGW(TAG, "ARM rejected: battery %.2fV <= %.2fV (USB/low — unsafe to fly)",
                 voltage, usb_v);
        return false;
    }

    // --- Pre-arm gate 2: boot calibration complete (requirements §9 / design §3) ------
    // The boot gyro/accel bias calibration must no longer be pending (ImuTask publishes
    // system_status.calibrated). Don't fly on a half-measured bias. Status via topic
    // (R16-style), not a cross-task object.
    // ARM 前ゲート2: 起動校正完了（要件§9 / 設計§3）。起動バイアス校正が保留中でないこと
    // （ImuTask が system_status.calibrated を発行）。半端なバイアスで飛ばさない。状態は
    // トピック経由（R16 流）。
    if (!system_status.latest().calibrated) {
        ESP_LOGW(TAG, "ARM rejected: boot calibration not complete");
        return false;
    }

    // --- Pre-arm gate 3: sensor health — DEFERRED ------------------------------------
    // A meaningful health gate needs sf_board::sensor_present() (the M2b per-sensor
    // presence infrastructure, which still returns false today), so it is wired with
    // that work, not here.
    // ARM 前ゲート3: センサ健全性 — 繰延。意味あるゲートには sf_board::sensor_present()
    // （M2b の per-sensor presence、現状 false）が要るため、その作業で配線する。

    ESP_LOGI(TAG, "ARM accepted");
    transition(FlightState::ARMED_GROUND);
    return true;
}

/// @design requirements.md §2 — ARMED_GROUND→IDLE_GROUND (DISARM),       [--]
/// @design requirements.md §2 — FLYING→IDLE_GROUND (pilot DISARM)        [--]
bool StateManager::requestDisarm()
{
    // DISARM is a pilot kill action. From ARMED_GROUND it stops the idle motors;
    // from any airborne state (TAKEOFF/FLYING/LANDING) it is an emergency cut
    // straight to IDLE_GROUND. Requirements §2 lists both "ARMED_GROUND →
    // IDLE_GROUND (DISARM)" and "FLYING → IDLE_GROUND (pilot DISARM)", so the
    // gate is "armed", not "ARMED_GROUND only". Motors are zeroed by ControlTask
    // once isArmed() goes false. Use the free function (the member isArmed()
    // hides it inside this scope).
    // DISARM はパイロットのキル操作。ARMED_GROUND ではアイドルのモータを止め、空中状態
    // (TAKEOFF/FLYING/LANDING)からは IDLE_GROUND への緊急カット。要件§2 は
    // 「ARMED_GROUND→IDLE_GROUND」と「FLYING→IDLE_GROUND(パイロット DISARM)」の両方を
    // 挙げるため、ゲートは「ARMED_GROUND 限定」でなく「armed」。モータは isArmed() が
    // false になり次第 ControlTask が 0 にする。
    if (!sf::isArmed(state_)) {
        ESP_LOGD(TAG, "DISARM rejected: not armed (state=%s)",
                 flightStateName(state_));
        return false;
    }

    ESP_LOGI(TAG, "DISARM accepted (%s → IDLE_GROUND)", flightStateName(state_));
    transition(FlightState::IDLE_GROUND);
    return true;
}

void StateManager::notifyTakeoff()
{
    if (state_ != FlightState::ARMED_GROUND) {
        return;
    }
    ESP_LOGI(TAG, "Takeoff detected");
    transition(FlightState::TAKEOFF);
}

void StateManager::notifyTakeoffComplete()
{
    if (state_ != FlightState::TAKEOFF) {
        return;
    }
    ESP_LOGI(TAG, "Takeoff complete → FLYING/%s", flightModeName(mode_));
    transition(FlightState::FLYING);
}

void StateManager::notifyLandingRequest()
{
    if (state_ != FlightState::FLYING) {
        return;
    }
    ESP_LOGI(TAG, "Landing requested");
    transition(FlightState::LANDING);
}

void StateManager::notifyLandingComplete()
{
    if (state_ != FlightState::LANDING) {
        return;
    }
    ESP_LOGI(TAG, "Landing complete");
    transition(FlightState::IDLE_GROUND);
}

/// @design requirements.md §2 — Soft landing / touch-and-go           [--]
void StateManager::notifySoftLanding()
{
    if (state_ != FlightState::FLYING) {
        return;
    }
    ESP_LOGI(TAG, "Soft landing → ARMED_GROUND");
    transition(FlightState::ARMED_GROUND);
}

/// @design requirements.md §2 — IDLE_GROUND ↔ IDLE_HELD by ToF        [--]
void StateManager::notifyIdleGroundHeld(bool is_held)
{
    if (is_held && state_ == FlightState::IDLE_GROUND) {
        ESP_LOGI(TAG, "Drone held in hand");
        transition(FlightState::IDLE_HELD);
    } else if (!is_held && state_ == FlightState::IDLE_HELD) {
        ESP_LOGI(TAG, "Drone placed on ground");
        transition(FlightState::IDLE_GROUND);
    }
}

bool StateManager::requestModeChange(FlightMode new_mode)
{
    if (state_ != FlightState::FLYING) {
        ESP_LOGD(TAG, "Mode change rejected: not FLYING");
        return false;
    }

    if (new_mode == mode_) {
        return true;  // Already in requested mode / 既にリクエストされたモード
    }

    FlightMode old_mode = mode_;
    ESP_LOGI(TAG, "Mode change: %s → %s",
             flightModeName(old_mode), flightModeName(new_mode));

    mode_ = new_mode;

    // Fire mode change callbacks
    // モード変更コールバックを発火
    for (int i = 0; i < mode_callback_count_; i++) {
        if (mode_callbacks_[i]) {
            mode_callbacks_[i](old_mode, new_mode);
        }
    }

    publishMode();
    return true;
}

// =============================================================================
// Alert Handling
// アラート処理
//
// @design architecture.md §4 — FAILSAFE as event                      [--]
// @design requirements.md §9 — Safety requirements                    [--]
// =============================================================================

void StateManager::handleAlert(const SystemAlert& alert)
{
    auto type = static_cast<AlertType>(alert.type);
    auto severity = static_cast<AlertSeverity>(alert.severity);

    ESP_LOGW(TAG, "Alert received: type=%d severity=%d", alert.type, alert.severity);

    switch (type) {
        case AlertType::IMPACT:
        case AlertType::GYRO_ANOMALY:
            // Crash → immediate DISARM
            // 衝突 → 即時DISARM
            if (isAirborne(state_)) {
                ESP_LOGE(TAG, "Impact/anomaly → emergency IDLE");
                transition(FlightState::IDLE_GROUND);
            }
            break;

        case AlertType::COMM_LOST:
            // Communication lost → hover 3s then auto-land
            // 通信途絶 → ホバー3秒後に自動着陸
            if (state_ == FlightState::FLYING) {
                ESP_LOGW(TAG, "Comm lost → LANDING");
                // TODO: Implement 3s hover delay before landing
                transition(FlightState::LANDING);
            }
            break;

        case AlertType::LOW_BATTERY:
            // Low battery → warning only (buzzer handled by notification)
            // 低電圧 → 警告のみ（ブザーは通知コンポーネントが処理）
            ESP_LOGW(TAG, "Low battery warning");
            break;

        case AlertType::USB_POWER:
            // USB power → ARM prohibited (checked in requestArm)
            // USB給電 → ARM禁止（requestArmで確認）
            break;

        case AlertType::ESKF_DIVERGED:
            // ESKF diverged → reset estimator (not a state change)
            // ESKF発散 → 推定器リセット（状態変更なし）
            ESP_LOGW(TAG, "ESKF diverged → reset requested");
            break;

        default:
            break;
    }
}

void StateManager::forceIdle()
{
    ESP_LOGW(TAG, "Force IDLE_GROUND from %s", flightStateName(state_));
    transition(FlightState::IDLE_GROUND);
}

// =============================================================================
// Callback Registration
// コールバック登録
// =============================================================================

void StateManager::onExit(OnExitCallback callback)
{
    if (exit_callback_count_ < MAX_CALLBACKS) {
        exit_callbacks_[exit_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onExit callbacks (max=%d)", MAX_CALLBACKS);
    }
}

void StateManager::onEnter(OnEnterCallback callback)
{
    if (enter_callback_count_ < MAX_CALLBACKS) {
        enter_callbacks_[enter_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onEnter callbacks (max=%d)", MAX_CALLBACKS);
    }
}

void StateManager::onModeChange(OnModeChangeCallback callback)
{
    if (mode_callback_count_ < MAX_CALLBACKS) {
        mode_callbacks_[mode_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onModeChange callbacks (max=%d)", MAX_CALLBACKS);
    }
}

// =============================================================================
// Internal: Transition Execution
// 内部: 遷移実行
// =============================================================================

void StateManager::transition(FlightState new_state)
{
    FlightState old_state = state_;

    if (old_state == new_state) {
        return;
    }

    ESP_LOGI(TAG, "Transition: %s → %s",
             flightStateName(old_state), flightStateName(new_state));

    // Fire onExit callbacks for old state
    // 旧状態のonExitコールバックを発火
    for (int i = 0; i < exit_callback_count_; i++) {
        if (exit_callbacks_[i]) {
            exit_callbacks_[i](old_state);
        }
    }

    // Update state
    // 状態を更新
    state_ = new_state;

    // Fire onEnter callbacks for new state
    // 新状態のonEnterコールバックを発火
    for (int i = 0; i < enter_callback_count_; i++) {
        if (enter_callbacks_[i]) {
            enter_callbacks_[i](new_state);
        }
    }

    // Publish to system.mode topic
    // system.modeトピックに発行
    publishMode();
}

void StateManager::publishMode()
{
    SystemMode mode_msg = {};
    mode_msg.state = static_cast<uint8_t>(state_);
    mode_msg.sub_mode = static_cast<uint8_t>(mode_);
    mode_msg.armed = sf::isArmed(state_);
    mode_msg.timestamp = 0;  // TODO: use esp_timer_get_time()

    system_mode.publish(mode_msg);
}

}  // namespace sf
