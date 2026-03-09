/**
 * @file control_types.hpp
 * @brief Shared control type definitions
 *        制御関連の共有型定義
 *
 * Extracted from control_arbiter.hpp to break circular dependency
 * between sf_svc_control_arbiter and sf_svc_state.
 * sf_svc_control_arbiter と sf_svc_state の循環依存を解消するため
 * control_arbiter.hpp から抽出。
 */

#pragma once

#include <cstdint>

namespace stampfly {

/**
 * @brief Control input source enumeration
 *        制御入力ソース列挙
 */
enum class ControlSource : uint8_t {
    NONE = 0,       ///< No source active / ソースなし
    ESPNOW,         ///< ESP-NOW from Controller / ESP-NOW
    UDP,            ///< UDP from Controller / UDP
    WEBSOCKET,      ///< WebSocket from GCS / WebSocket
};

/**
 * @brief Communication mode enumeration
 *        通信モード列挙
 */
enum class CommMode : uint8_t {
    ESPNOW = 0,     ///< ESP-NOW mode (default) / ESP-NOWモード
    UDP = 1,        ///< UDP mode / UDPモード
};

/**
 * @brief Normalized control input structure
 *        正規化済み制御入力構造体
 */
struct ControlInput {
    float throttle;         ///< [0, 1] normalized throttle / 正規化スロットル
    float roll;             ///< [-1, 1] normalized roll / 正規化ロール
    float pitch;            ///< [-1, 1] normalized pitch / 正規化ピッチ
    float yaw;              ///< [-1, 1] normalized yaw / 正規化ヨー
    uint8_t flags;          ///< Control flags / 制御フラグ
    ControlSource source;   ///< Input source / 入力ソース
    uint32_t timestamp_ms;  ///< Timestamp in ms / タイムスタンプ(ms)
};

}  // namespace stampfly
