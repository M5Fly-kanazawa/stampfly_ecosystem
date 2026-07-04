/**
 * @file system_state.cpp
 * @brief Unified System State Manager Implementation
 * @brief 統合システム状態管理 実装
 */

#include "system_state.hpp"
#include "control_arbiter.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "SystemState";

namespace stampfly {

// ============================================================================
// Singleton
// ============================================================================

SystemStateManager& SystemStateManager::getInstance()
{
    static SystemStateManager instance;
    return instance;
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t SystemStateManager::init()
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize state
    // 状態を初期化
    state_ = SystemState();
    callback_count_ = 0;

    ESP_LOGI(TAG, "SystemStateManager initialized");
    return ESP_OK;
}

// ============================================================================
// State Queries
// ============================================================================

const SystemState SystemStateManager::getState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    SystemState state_copy = state_;
    xSemaphoreGive(mutex_);
    return state_copy;
}

FlightState SystemStateManager::getFlightState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    FlightState state = state_.flight_state;
    xSemaphoreGive(mutex_);
    return state;
}

CalibrationState SystemStateManager::getCalibrationState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    CalibrationState state = state_.calibration_state;
    xSemaphoreGive(mutex_);
    return state;
}

CommandState SystemStateManager::getCommandState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    CommandState state = state_.command_state;
    xSemaphoreGive(mutex_);
    return state;
}

ErrorCode SystemStateManager::getErrorCode() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ErrorCode code = state_.error_code;
    xSemaphoreGive(mutex_);
    return code;
}

bool SystemStateManager::isReady(uint32_t flags) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool ready = (state_.readiness_flags & flags) == flags;
    xSemaphoreGive(mutex_);
    return ready;
}

uint32_t SystemStateManager::getReadinessFlags() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    uint32_t flags = state_.readiness_flags;
    xSemaphoreGive(mutex_);
    return flags;
}

// Forward declaration of transition validation
// 遷移検証の前方宣言
extern bool validateTransition(const SystemState& state, FlightState target, bool debug_mode);

bool SystemStateManager::canTransitionTo(FlightState target) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool can_transition = validateTransition(state_, target, debug_mode_);
    xSemaphoreGive(mutex_);
    return can_transition;
}

// ============================================================================
// State Transitions
// ============================================================================

bool SystemStateManager::requestTransition(FlightState target, const char* reason)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Check if transition is allowed
    // 遷移が許可されているか確認
    if (!validateTransition(state_, target, debug_mode_)) {
        xSemaphoreGive(mutex_);
        ESP_LOGW(TAG, "Transition %d -> %d rejected (preconditions not met)",
                 static_cast<int>(state_.flight_state), static_cast<int>(target));
        return false;
    }

    // Execute transition
    // 遷移を実行
    FlightState old_state = state_.flight_state;
    state_.flight_state = target;
    state_.last_transition_ms = getCurrentTimeMs();

    xSemaphoreGive(mutex_);

    // Log transition
    // 遷移をログ
    if (reason != nullptr) {
        ESP_LOGI(TAG, "State transition: %d -> %d (%s)",
                 static_cast<int>(old_state), static_cast<int>(target), reason);
    } else {
        ESP_LOGI(TAG, "State transition: %d -> %d",
                 static_cast<int>(old_state), static_cast<int>(target));
    }

    // Notify callbacks
    // コールバックを通知
    StateTransitionEvent event;
    event.from_state = old_state;
    event.to_state = target;
    event.timestamp_ms = state_.last_transition_ms;
    event.reason = reason;
    notifyCallbacks(event);

    return true;
}

bool SystemStateManager::requestArm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    bool can_arm = false;

    // In debug mode, allow ARM from IDLE or ERROR state with relaxed checks
    // デバッグモードでは、IDLE または ERROR 状態からの ARM を許可（チェック緩和）
    if (debug_mode_) {
        can_arm = (state_.flight_state == FlightState::IDLE ||
                   state_.flight_state == FlightState::ERROR);
        if (can_arm) {
            state_.flight_state = FlightState::ARMED;
            state_.last_transition_ms = getCurrentTimeMs();
            xSemaphoreGive(mutex_);
            ESP_LOGW(TAG, "Armed (DEBUG MODE - preconditions bypassed)");
            return true;
        }
    }

    // Normal mode: check all preconditions
    // 通常モード: すべての前提条件をチェック
    if (state_.flight_state != FlightState::IDLE) {
        xSemaphoreGive(mutex_);
        ESP_LOGW(TAG, "ARM rejected: not in IDLE state");
        return false;
    }

    if (state_.error_code != ErrorCode::NONE) {
        xSemaphoreGive(mutex_);
        ESP_LOGW(TAG, "ARM rejected: error present (%d)", static_cast<int>(state_.error_code));
        return false;
    }

    // Check readiness flags
    // 準備フラグをチェック
    uint32_t required = ReadinessFlags::CALIBRATED | ReadinessFlags::LANDED;
    if ((state_.readiness_flags & required) != required) {
        xSemaphoreGive(mutex_);
        ESP_LOGW(TAG, "ARM rejected: not ready (flags: 0x%lx, required: 0x%lx)",
                 state_.readiness_flags, required);
        return false;
    }

    // ARM allowed
    // ARM許可
    state_.flight_state = FlightState::ARMED;
    state_.last_transition_ms = getCurrentTimeMs();
    xSemaphoreGive(mutex_);
    ESP_LOGI(TAG, "Armed");

    // Notify callbacks
    // コールバックを通知
    StateTransitionEvent event;
    event.from_state = FlightState::IDLE;
    event.to_state = FlightState::ARMED;
    event.timestamp_ms = state_.last_transition_ms;
    event.reason = "ARM requested";
    notifyCallbacks(event);

    return true;
}

bool SystemStateManager::requestDisarm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (state_.flight_state != FlightState::ARMED &&
        state_.flight_state != FlightState::FLYING) {
        xSemaphoreGive(mutex_);
        return false;
    }

    FlightState old_state = state_.flight_state;
    state_.flight_state = FlightState::IDLE;
    state_.last_transition_ms = getCurrentTimeMs();
    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Disarmed");

    // Notify callbacks
    // コールバックを通知
    StateTransitionEvent event;
    event.from_state = old_state;
    event.to_state = FlightState::IDLE;
    event.timestamp_ms = state_.last_transition_ms;
    event.reason = "DISARM requested";
    notifyCallbacks(event);

    return true;
}

void SystemStateManager::setFlightState(FlightState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    FlightState old_state = state_.flight_state;
    state_.flight_state = state;
    state_.last_transition_ms = getCurrentTimeMs();
    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Flight state set: %d -> %d (forced)",
             static_cast<int>(old_state), static_cast<int>(state));

    // Notify callbacks
    // コールバックを通知
    StateTransitionEvent event;
    event.from_state = old_state;
    event.to_state = state;
    event.timestamp_ms = state_.last_transition_ms;
    event.reason = "Forced state change";
    notifyCallbacks(event);
}

// ============================================================================
// Readiness Management
// ============================================================================

void SystemStateManager::setReady(uint32_t flag, bool ready)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (ready) {
        state_.readiness_flags |= flag;
    } else {
        state_.readiness_flags &= ~flag;
    }
    xSemaphoreGive(mutex_);
}

void SystemStateManager::updateCalibrationState(CalibrationState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.calibration_state = state;

    // Update CALIBRATED readiness flag
    // CALIBRATED 準備フラグを更新
    if (state == CalibrationState::COMPLETED) {
        state_.readiness_flags |= ReadinessFlags::CALIBRATED;
    } else {
        state_.readiness_flags &= ~ReadinessFlags::CALIBRATED;
    }
    xSemaphoreGive(mutex_);
}

void SystemStateManager::updateCommandState(CommandState state,
                                            FlightCommandType command,
                                            float progress)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.command_state = state;
    state_.current_command = command;
    state_.command_progress = progress;
    xSemaphoreGive(mutex_);
}

void SystemStateManager::updateControlSource(ControlSource source)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.active_control_source = source;
    xSemaphoreGive(mutex_);
}

void SystemStateManager::setError(ErrorCode code)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.error_code = code;
    if (code != ErrorCode::NONE) {
        FlightState old_state = state_.flight_state;
        state_.flight_state = FlightState::ERROR;
        state_.last_transition_ms = getCurrentTimeMs();
        xSemaphoreGive(mutex_);

        ESP_LOGE(TAG, "Error set: %d (state: %d -> ERROR)",
                 static_cast<int>(code), static_cast<int>(old_state));

        // Notify callbacks
        // コールバックを通知
        StateTransitionEvent event;
        event.from_state = old_state;
        event.to_state = FlightState::ERROR;
        event.timestamp_ms = state_.last_transition_ms;
        event.reason = "Error occurred";
        notifyCallbacks(event);
    } else {
        xSemaphoreGive(mutex_);
    }
}

void SystemStateManager::clearError()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_.error_code = ErrorCode::NONE;
    xSemaphoreGive(mutex_);
    ESP_LOGI(TAG, "Error cleared");
}

// ============================================================================
// Event Callbacks
// ============================================================================

int SystemStateManager::subscribeStateChange(StateCallback callback)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (callback_count_ >= MAX_CALLBACKS) {
        xSemaphoreGive(mutex_);
        ESP_LOGE(TAG, "Cannot subscribe: callback limit reached");
        return -1;
    }

    int id = callback_count_;
    callbacks_[callback_count_++] = callback;
    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "State change callback registered (ID: %d)", id);
    return id;
}

void SystemStateManager::notifyCallbacks(const StateTransitionEvent& event)
{
    // Note: Don't take mutex here - callbacks may query state
    // 注意: ここでミューテックスを取得しない - コールバックが状態をクエリする可能性がある

    for (int i = 0; i < callback_count_; i++) {
        if (callbacks_[i]) {
            callbacks_[i](event);
        }
    }
}

// ============================================================================
// Utility
// ============================================================================

uint32_t SystemStateManager::getCurrentTimeMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace stampfly
