/**
 * @file state_transitions.cpp
 * @brief State transition rules and validation
 * @brief 状態遷移ルールと検証
 *
 * Defines explicit transition rules between flight states.
 * Validates preconditions (readiness flags, error state, etc.)
 *
 * フライト状態間の明示的な遷移ルールを定義。
 * 前提条件（準備フラグ、エラー状態等）を検証。
 */

#include "system_state.hpp"
#include "esp_log.h"

static const char* TAG = "StateTransitions";

namespace stampfly {

// ============================================================================
// Transition Rule Definition
// ============================================================================

/**
 * @brief State transition rule
 * @brief 状態遷移ルール
 */
struct TransitionRule {
    FlightState from;                       ///< Source state / 遷移元状態
    FlightState to;                         ///< Target state / 遷移先状態
    uint32_t required_flags;                ///< Required readiness flags / 必要な準備フラグ
    bool (*validator)(const SystemState&);  ///< Custom validator / カスタム検証関数
};

// ============================================================================
// Custom Validators
// ============================================================================

/**
 * @brief Validate ARM transition (IDLE → ARMED)
 * @brief ARM遷移検証 (IDLE → ARMED)
 */
static bool validateArm(const SystemState& state)
{
    // Error must be cleared
    // エラーがクリアされている必要がある
    if (state.error_code != ErrorCode::NONE) {
        return false;
    }

    // Must be calibrated and landed
    // キャリブレーション完了かつ着陸済みである必要がある
    if (state.calibration_state != CalibrationState::COMPLETED) {
        return false;
    }

    // Battery should be OK (if power monitoring available)
    // バッテリーOKである必要がある（電源モニタリングが利用可能な場合）
    // Note: This is checked via readiness flags
    // 注意: これは準備フラグでチェックされる

    return true;
}

/**
 * @brief Validate FLYING transition (ARMED → FLYING)
 * @brief FLYING遷移検証 (ARMED → FLYING)
 */
static bool validateFlying(const SystemState& state)
{
    // No specific checks needed - throttle up is implicit
    // 特別なチェック不要 - スロットルアップは暗黙的
    return true;
}

/**
 * @brief Validate LANDING transition (FLYING → LANDING)
 * @brief LANDING遷移検証 (FLYING → LANDING)
 */
static bool validateLanding(const SystemState& state)
{
    // No specific checks needed - can always initiate landing
    // 特別なチェック不要 - 常に着陸を開始可能
    return true;
}

/**
 * @brief Validate IDLE transition from LANDING
 * @brief LANDING からの IDLE 遷移検証
 */
static bool validateIdleFromLanding(const SystemState& state)
{
    // Must be landed
    // 着陸済みである必要がある
    return (state.readiness_flags & ReadinessFlags::LANDED) != 0;
}

/**
 * @brief Validate ERROR recovery (ERROR → IDLE)
 * @brief エラー回復検証 (ERROR → IDLE)
 */
static bool validateErrorRecovery(const SystemState& state)
{
    // Error must be cleared
    // エラーがクリアされている必要がある
    return state.error_code == ErrorCode::NONE;
}

// ============================================================================
// Transition Rule Table
// ============================================================================

/**
 * @brief Complete state transition rule table
 * @brief 完全な状態遷移ルールテーブル
 */
static const TransitionRule TRANSITION_TABLE[] = {
    // ========================================================================
    // Initialization sequence
    // 初期化シーケンス
    // ========================================================================

    // INIT → CALIBRATING (always allowed)
    // INIT → CALIBRATING (常に許可)
    {
        FlightState::INIT,
        FlightState::CALIBRATING,
        ReadinessFlags::NONE,
        nullptr
    },

    // CALIBRATING → IDLE (all sensors ready)
    // CALIBRATING → IDLE (全センサー準備完了)
    {
        FlightState::CALIBRATING,
        FlightState::IDLE,
        ReadinessFlags::ALL_SENSORS_READY,
        nullptr
    },

    // ========================================================================
    // Normal flight sequence
    // 通常の飛行シーケンス
    // ========================================================================

    // IDLE → ARMED (calibration complete, sensors ready, landed, battery OK)
    // IDLE → ARMED (キャリブレーション完了、センサー準備完了、着陸済み、バッテリーOK)
    {
        FlightState::IDLE,
        FlightState::ARMED,
        ReadinessFlags::ARM_READY,
        validateArm
    },

    // ARMED → FLYING (throttle up detected)
    // ARMED → FLYING (スロットルアップ検出)
    {
        FlightState::ARMED,
        FlightState::FLYING,
        ReadinessFlags::CONTROL_INPUT_OK,
        validateFlying
    },

    // FLYING → LANDING (landing initiated)
    // FLYING → LANDING (着陸開始)
    {
        FlightState::FLYING,
        FlightState::LANDING,
        ReadinessFlags::NONE,
        validateLanding
    },

    // LANDING → IDLE (landed)
    // LANDING → IDLE (着陸完了)
    {
        FlightState::LANDING,
        FlightState::IDLE,
        ReadinessFlags::LANDED,
        validateIdleFromLanding
    },

    // ========================================================================
    // Emergency and abort sequences
    // 緊急・中断シーケンス
    // ========================================================================

    // ARMED → IDLE (disarm before takeoff)
    // ARMED → IDLE (離陸前のdisarm)
    {
        FlightState::ARMED,
        FlightState::IDLE,
        ReadinessFlags::NONE,
        nullptr
    },

    // FLYING → IDLE (emergency disarm in flight - dangerous!)
    // FLYING → IDLE (飛行中の緊急disarm - 危険!)
    {
        FlightState::FLYING,
        FlightState::IDLE,
        ReadinessFlags::NONE,
        nullptr
    },

    // ========================================================================
    // Error handling
    // エラー処理
    // ========================================================================

    // Any state → ERROR (error occurred)
    // 任意の状態 → ERROR (エラー発生)
    // Note: This is handled specially in setError(), not via transition validation
    // 注意: これは setError() で特別に処理され、遷移検証経由ではない

    // ERROR → IDLE (error cleared, system recovered)
    // ERROR → IDLE (エラークリア、システム復旧)
    {
        FlightState::ERROR,
        FlightState::IDLE,
        ReadinessFlags::ALL_SENSORS_READY,
        validateErrorRecovery
    },

    // ERROR → CALIBRATING (re-initialization)
    // ERROR → CALIBRATING (再初期化)
    {
        FlightState::ERROR,
        FlightState::CALIBRATING,
        ReadinessFlags::NONE,
        validateErrorRecovery
    },
};

static constexpr size_t TRANSITION_TABLE_SIZE = sizeof(TRANSITION_TABLE) / sizeof(TransitionRule);

// ============================================================================
// Transition Validation
// ============================================================================

/**
 * @brief Validate if transition is allowed
 * @brief 遷移が許可されているか検証
 *
 * @param state Current system state / 現在のシステム状態
 * @param target Target flight state / 目標フライト状態
 * @param debug_mode Debug mode flag / デバッグモードフラグ
 * @return true if transition is allowed / 遷移が許可されている場合true
 */
bool validateTransition(const SystemState& state, FlightState target, bool debug_mode)
{
    FlightState current = state.flight_state;

    // No transition needed
    // 遷移不要
    if (current == target) {
        return true;
    }

    // In debug mode, allow most transitions
    // デバッグモードでは、ほとんどの遷移を許可
    if (debug_mode) {
        ESP_LOGW(TAG, "Transition %d -> %d allowed (DEBUG MODE)",
                 static_cast<int>(current), static_cast<int>(target));
        return true;
    }

    // Find matching rule in transition table
    // 遷移テーブルで一致するルールを検索
    for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
        const TransitionRule& rule = TRANSITION_TABLE[i];

        if (rule.from == current && rule.to == target) {
            // Check required readiness flags
            // 必要な準備フラグをチェック
            if (rule.required_flags != ReadinessFlags::NONE) {
                if ((state.readiness_flags & rule.required_flags) != rule.required_flags) {
                    ESP_LOGW(TAG, "Transition %d -> %d rejected: readiness flags not met "
                             "(have: 0x%lx, need: 0x%lx)",
                             static_cast<int>(current), static_cast<int>(target),
                             state.readiness_flags, rule.required_flags);
                    return false;
                }
            }

            // Run custom validator if present
            // カスタム検証関数が存在する場合実行
            if (rule.validator != nullptr) {
                if (!rule.validator(state)) {
                    ESP_LOGW(TAG, "Transition %d -> %d rejected: validator failed",
                             static_cast<int>(current), static_cast<int>(target));
                    return false;
                }
            }

            // Transition allowed
            // 遷移許可
            return true;
        }
    }

    // No matching rule found - transition not allowed
    // 一致するルールが見つからない - 遷移不許可
    ESP_LOGW(TAG, "Transition %d -> %d rejected: no rule defined",
             static_cast<int>(current), static_cast<int>(target));
    return false;
}

/**
 * @brief Get human-readable state name
 * @brief 人間が読める状態名を取得
 */
const char* getFlightStateName(FlightState state)
{
    switch (state) {
        case FlightState::INIT:        return "INIT";
        case FlightState::CALIBRATING: return "CALIBRATING";
        case FlightState::IDLE:        return "IDLE";
        case FlightState::ARMED:       return "ARMED";
        case FlightState::FLYING:      return "FLYING";
        case FlightState::LANDING:     return "LANDING";
        case FlightState::ERROR:       return "ERROR";
        default:                       return "UNKNOWN";
    }
}

}  // namespace stampfly
