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
#include "esp_log.h"

static const char* TAG = "StateManager";

namespace sf {

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

/// @design requirements.md §2 — ARM from GROUND only                  [--]
/// @design requirements.md §9 — USB power: ARM prohibited             [--]
bool StateManager::requestArm()
{
    if (state_ != FlightState::IDLE_GROUND) {
        ESP_LOGD(TAG, "ARM rejected: not in IDLE_GROUND (state=%s)",
                 flightStateName(state_));
        return false;
    }

    // TODO: Check USB power (voltage <= 3.3V → reject)
    // TODO: Check calibration complete
    // TODO: Check sensor health

    ESP_LOGI(TAG, "ARM accepted");
    transition(FlightState::ARMED_GROUND);
    return true;
}

bool StateManager::requestDisarm()
{
    if (state_ != FlightState::ARMED_GROUND) {
        ESP_LOGD(TAG, "DISARM rejected: not in ARMED_GROUND (state=%s)",
                 flightStateName(state_));
        return false;
    }

    ESP_LOGI(TAG, "DISARM accepted");
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
