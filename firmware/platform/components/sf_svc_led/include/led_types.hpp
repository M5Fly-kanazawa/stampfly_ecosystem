/**
 * @file led_types.hpp
 * @brief LED管理システムの型定義
 */

#pragma once

#include <cstdint>

namespace stampfly {

/**
 * @brief LEDインデックス
 */
enum class LEDIndex : uint8_t {
    MCU = 0,       // M5Stamp S3内蔵 (GPIO21)
    BODY_TOP,      // ドローン上面/表 (GPIO39, index 0)
    BODY_BOTTOM,   // ドローン下面/裏 (GPIO39, index 1)
    ALL,           // 全LED一括
    NUM_LEDS = 3
};

/**
 * @brief LEDチャンネル（各LEDの担当カテゴリ）
 */
enum class LEDChannel : uint8_t {
    SYSTEM = 0,    // MCU LED: システム状態（起動/エラー/ペアリング）
    FLIGHT,        // BODY_TOP: 飛行状態
    STATUS,        // BODY_BOTTOM: センサー/バッテリー状態
    NUM_CHANNELS
};

/**
 * @brief LED表示の優先度（低い値 = 高優先度）
 */
enum class LEDPriority : uint8_t {
    BOOT = 0,           // 最高: 起動シーケンス
    CRITICAL_ERROR = 1, // 致命的エラー（センサー故障等）
    LOW_BATTERY = 2,    // 低電圧警告
    DEBUG_ALERT = 3,    // デバッグ表示（yaw_alert等）
    PAIRING = 4,        // ペアリングモード
    FLIGHT_STATE = 5,   // 通常の飛行状態表示
    DEFAULT = 6,        // 最低: デフォルト表示
    NUM_PRIORITIES
};

/**
 * @brief LEDパターン
 */
enum class LEDPattern : uint8_t {
    OFF = 0,
    SOLID,
    BLINK_SLOW,
    BLINK_FAST,
    BREATHE,
    RAINBOW
};

}  // namespace stampfly
