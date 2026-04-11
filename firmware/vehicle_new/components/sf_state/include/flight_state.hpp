/**
 * @file flight_state.hpp
 * @brief Flight state and mode enum definitions
 *        フライト状態およびモードのenum定義
 *
 * Defines all possible states and sub-modes of the vehicle.
 * ARM/DISARM are actions, not states.
 *
 * 機体の全状態とサブモードを定義する。
 * ARM/DISARMはアクション（モードではない）。
 *
 * @design requirements.md §2 — State model                            [--]
 * @design architecture.md §4 — State machine design                   [--]
 */

#pragma once

#include <cstdint>

namespace sf {

// =============================================================================
// Flight State — top-level state of the vehicle
// フライト状態 — 機体のトップレベル状態
//
// @design requirements.md §2 — State transition diagram               [--]
// =============================================================================

enum class FlightState : uint8_t {
    INIT          = 0,   // Sequential initialization / 逐次初期化
    IDLE_GROUND   = 1,   // On ground, ToF invalid range / 地上、ToF無効距離
    IDLE_HELD     = 2,   // Held in hand, ToF valid / 手持ち、ToF有効
    ARMED_GROUND  = 3,   // Armed, on ground / ARM済み、地上
    TAKEOFF       = 4,   // Takeoff sequence / 離陸シーケンス
    FLYING        = 5,   // In flight (see FlightMode) / 飛行中（FlightMode参照）
    LANDING       = 6,   // Landing sequence / 着陸シーケンス
};

/// Number of flight states (for array sizing)
/// フライト状態数（配列サイズ用）
constexpr int FLIGHT_STATE_COUNT = 7;

/// Get human-readable state name
/// 状態の名前を取得する
inline const char* flightStateName(FlightState state)
{
    switch (state) {
        case FlightState::INIT:         return "INIT";
        case FlightState::IDLE_GROUND:  return "IDLE_GROUND";
        case FlightState::IDLE_HELD:    return "IDLE_HELD";
        case FlightState::ARMED_GROUND: return "ARMED_GROUND";
        case FlightState::TAKEOFF:      return "TAKEOFF";
        case FlightState::FLYING:       return "FLYING";
        case FlightState::LANDING:      return "LANDING";
        default:                        return "UNKNOWN";
    }
}

// =============================================================================
// Flight Mode — sub-mode within FLYING state
// フライトモード — FLYING状態内のサブモード
//
// @design requirements.md §2 — ACRO, STABILIZE, ALT_HOLD, POS_HOLD   [--]
// =============================================================================

enum class FlightMode : uint8_t {
    ACRO      = 0,   // Rate control / 角速度制御
    STABILIZE = 1,   // Attitude stabilization / 姿勢安定
    ALT_HOLD  = 2,   // Altitude hold / 高度維持
    POS_HOLD  = 3,   // Position hold / 位置維持
};

/// Number of flight modes (for array sizing)
/// フライトモード数（配列サイズ用）
constexpr int FLIGHT_MODE_COUNT = 4;

/// Get human-readable mode name
/// モードの名前を取得する
inline const char* flightModeName(FlightMode mode)
{
    switch (mode) {
        case FlightMode::ACRO:      return "ACRO";
        case FlightMode::STABILIZE: return "STABILIZE";
        case FlightMode::ALT_HOLD:  return "ALT_HOLD";
        case FlightMode::POS_HOLD:  return "POS_HOLD";
        default:                    return "UNKNOWN";
    }
}

// =============================================================================
// Alert Type — failsafe event types
// アラート種別 — フェイルセーフイベント種別
//
// @design architecture.md §4 — FAILSAFE as event                      [--]
// @design requirements.md §9 — Safety requirements                    [--]
// =============================================================================

enum class AlertType : uint8_t {
    NONE           = 0,
    COMM_LOST      = 1,   // Communication timeout / 通信途絶
    IMPACT         = 2,   // Crash detected / 衝突検知
    LOW_BATTERY    = 3,   // LiPo low voltage / LiPo低電圧
    USB_POWER      = 4,   // USB power only, no LiPo / USB給電のみ
    ESKF_DIVERGED  = 5,   // State estimator diverged / 推定器発散
    GYRO_ANOMALY   = 6,   // Abnormal angular rate / 異常角速度
};

/// Alert severity levels
/// アラート重要度レベル
enum class AlertSeverity : uint8_t {
    INFO     = 0,   // Informational / 情報
    WARNING  = 1,   // Warning, no action needed / 警告
    CRITICAL = 2,   // Critical, action required / 重大、対応必要
    EMERGENCY = 3,  // Emergency, immediate action / 緊急、即時対応
};

// =============================================================================
// Utility: is the vehicle armed in a given state?
// ユーティリティ: 指定状態でARMされているか？
//
// @design requirements.md §2 — ARM/DISARM as actions                  [--]
// =============================================================================

/// Check if a state implies armed (motors can spin)
/// 状態がARMを意味するか確認する（モーターが回転可能か）
inline bool isArmed(FlightState state)
{
    return state == FlightState::ARMED_GROUND ||
           state == FlightState::TAKEOFF ||
           state == FlightState::FLYING ||
           state == FlightState::LANDING;
}

/// Check if a state implies airborne (in the air)
/// 状態が空中を意味するか確認する
inline bool isAirborne(FlightState state)
{
    return state == FlightState::TAKEOFF ||
           state == FlightState::FLYING ||
           state == FlightState::LANDING;
}

}  // namespace sf
