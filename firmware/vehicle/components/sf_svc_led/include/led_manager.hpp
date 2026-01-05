/**
 * @file led_manager.hpp
 * @brief LED管理システム
 *
 * 3つのLEDを優先度ベースで管理するサービス層
 * - MCU LED (GPIO21): システム状態
 * - BODY_TOP (GPIO39-0): 飛行状態
 * - BODY_BOTTOM (GPIO39-1): センサー/バッテリー
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "led_types.hpp"
#include "led.hpp"
#include "stampfly_state.hpp"

namespace stampfly {

class LEDManager {
public:
    /**
     * @brief シングルトンインスタンス取得
     */
    static LEDManager& getInstance();

    /**
     * @brief 初期化（2つのLEDストリップを初期化）
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief 初期化済みかチェック
     */
    bool isInitialized() const { return initialized_; }

    // =========================================================================
    // チャンネルベースAPI（推奨）
    // =========================================================================

    /**
     * @brief チャンネルに表示を要求
     * @param channel 対象チャンネル
     * @param priority 優先度
     * @param pattern パターン
     * @param color RGB色 (0xRRGGBB)
     * @param timeout_ms タイムアウト（0=永続、>0=指定時間後に自動解除）
     */
    void requestChannel(LEDChannel channel, LEDPriority priority,
                        LEDPattern pattern, uint32_t color,
                        uint32_t timeout_ms = 0);

    /**
     * @brief チャンネルの表示要求を解除
     * @param channel 対象チャンネル
     * @param priority 解除する優先度
     */
    void releaseChannel(LEDChannel channel, LEDPriority priority);

    // =========================================================================
    // 直接制御API（デバッグ/緊急用）
    // =========================================================================

    /**
     * @brief 特定のLEDを直接制御（優先度システムをバイパス）
     * @param led 対象LED
     * @param pattern パターン
     * @param color RGB色
     */
    void setDirect(LEDIndex led, LEDPattern pattern, uint32_t color);

    // =========================================================================
    // イベント通知API
    // =========================================================================

    /**
     * @brief フライト状態変更通知
     * @param state 新しいフライト状態
     */
    void onFlightStateChanged(FlightState state);

    /**
     * @brief バッテリー状態変更通知
     * @param voltage 電圧 [V]
     * @param low_battery 低電圧フラグ
     */
    void onBatteryStateChanged(float voltage, bool low_battery);

    /**
     * @brief センサー健全性変更通知
     * @param all_healthy 全センサー正常フラグ
     */
    void onSensorHealthChanged(bool all_healthy);

    // =========================================================================
    // 更新（タスクから呼び出し）
    // =========================================================================

    /**
     * @brief LED状態を更新（タイムアウト処理とアニメーション）
     * LEDタスクから30Hz程度で呼び出すこと
     */
    void update();

    /**
     * @brief 輝度設定
     * @param brightness 0-255
     * @param save_to_nvs NVSに保存するか
     */
    void setBrightness(uint8_t brightness, bool save_to_nvs = false);

    /**
     * @brief 輝度取得
     */
    uint8_t getBrightness() const;

private:
    LEDManager() = default;
    ~LEDManager() = default;
    LEDManager(const LEDManager&) = delete;
    LEDManager& operator=(const LEDManager&) = delete;

    /**
     * @brief 表示要求
     */
    struct DisplayRequest {
        bool active = false;
        LEDPattern pattern = LEDPattern::OFF;
        uint32_t color = 0;
        uint32_t expire_time_ms = 0;  // 0=永続
    };

    /**
     * @brief チャンネル状態
     */
    struct ChannelState {
        DisplayRequest requests[static_cast<int>(LEDPriority::NUM_PRIORITIES)];
        LEDIndex target_led;
    };

    /**
     * @brief チャンネルの最も優先度の高い要求を取得
     */
    const DisplayRequest* getActiveRequest(LEDChannel channel) const;

    /**
     * @brief LEDにパターンを適用
     */
    void applyToLED(LEDIndex led, LEDPattern pattern, uint32_t color);

    /**
     * @brief LEDPatternをLED::Patternに変換
     */
    static LED::Pattern toHALPattern(LEDPattern pattern);

    bool initialized_ = false;

    // チャンネル状態
    ChannelState channels_[static_cast<int>(LEDChannel::NUM_CHANNELS)];

    // HAL（2つのLEDストリップ）
    LED mcu_led_;    // GPIO21, 1個
    LED body_led_;   // GPIO39, 2個

    // 現在の表示状態（変更検出用）
    struct CurrentState {
        LEDPattern pattern = LEDPattern::OFF;
        uint32_t color = 0;
    };
    CurrentState current_state_[3];  // MCU, BODY_TOP, BODY_BOTTOM
};

}  // namespace stampfly
