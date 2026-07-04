/**
 * @file control_arbiter.hpp
 * @brief Control input arbiter for multiple sources
 *        複数ソースからの制御入力アービター
 *
 * Manages control inputs from ESP-NOW, UDP, and WebSocket sources.
 * Handles priority, timeout, and failsafe logic.
 * ESP-NOW、UDP、WebSocketからの制御入力を管理。
 * 優先度、タイムアウト、フェイルセーフロジックを処理。
 */

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

/**
 * @brief Control Arbiter - manages control inputs from multiple sources
 *        複数ソースからの制御入力を管理するアービター
 *
 * Singleton class that:
 * - Receives control inputs from ESP-NOW, UDP, and WebSocket
 * - Tracks the active source based on communication mode
 * - Handles timeout and failsafe
 *
 * シングルトンクラス:
 * - ESP-NOW、UDP、WebSocketからの制御入力を受信
 * - 通信モードに基づいてアクティブソースを追跡
 * - タイムアウトとフェイルセーフを処理
 */
class ControlArbiter {
public:
    /// Get singleton instance / シングルトンインスタンスを取得
    static ControlArbiter& getInstance();

    // Delete copy/move
    ControlArbiter(const ControlArbiter&) = delete;
    ControlArbiter& operator=(const ControlArbiter&) = delete;

    /**
     * @brief Initialize the arbiter
     *        アービターを初期化
     *
     * @return ESP_OK on success / 成功時ESP_OK
     */
    esp_err_t init();

    // ========================================================================
    // Communication Mode
    // 通信モード
    // ========================================================================

    /**
     * @brief Set communication mode
     *        通信モードを設定
     *
     * @param mode Communication mode / 通信モード
     */
    void setCommMode(CommMode mode);

    /**
     * @brief Get current communication mode
     *        現在の通信モードを取得
     */
    CommMode getCommMode() const { return comm_mode_; }

    /**
     * @brief Get communication mode name
     *        通信モード名を取得
     */
    static const char* getCommModeName(CommMode mode);

    // ========================================================================
    // Control Input Updates
    // 制御入力更新
    // ========================================================================

    /**
     * @brief Update control input from ESP-NOW
     *        ESP-NOWからの制御入力を更新
     *
     * @param throttle Raw throttle [0-4095] / 生スロットル値
     * @param roll Raw roll [0-4095], 2048=center / 生ロール値
     * @param pitch Raw pitch [0-4095], 2048=center / 生ピッチ値
     * @param yaw Raw yaw [0-4095], 2048=center / 生ヨー値
     * @param flags Control flags / 制御フラグ
     */
    void updateFromESPNOW(uint16_t throttle, uint16_t roll,
                          uint16_t pitch, uint16_t yaw, uint8_t flags);

    /**
     * @brief Update control input from UDP
     *        UDPからの制御入力を更新
     *
     * @param throttle Raw throttle [0-4095] / 生スロットル値
     * @param roll Raw roll [0-4095], 2048=center / 生ロール値
     * @param pitch Raw pitch [0-4095], 2048=center / 生ピッチ値
     * @param yaw Raw yaw [0-4095], 2048=center / 生ヨー値
     * @param flags Control flags / 制御フラグ
     */
    void updateFromUDP(uint16_t throttle, uint16_t roll,
                       uint16_t pitch, uint16_t yaw, uint8_t flags);

    /**
     * @brief Update control input from WebSocket
     *        WebSocketからの制御入力を更新
     *
     * @param throttle Normalized throttle [0-1] / 正規化スロットル
     * @param roll Normalized roll [-1, 1] / 正規化ロール
     * @param pitch Normalized pitch [-1, 1] / 正規化ピッチ
     * @param yaw Normalized yaw [-1, 1] / 正規化ヨー
     * @param flags Control flags / 制御フラグ
     */
    void updateFromWebSocket(float throttle, float roll,
                             float pitch, float yaw, uint8_t flags);

    // ========================================================================
    // Active Control Access
    // アクティブ制御アクセス
    // ========================================================================

    /**
     * @brief Get the active control input
     *        アクティブな制御入力を取得
     *
     * Returns the control input from the active source if within timeout.
     * Returns neutral values if no active source or timeout.
     *
     * タイムアウト内であればアクティブソースからの制御入力を返す。
     * アクティブソースがないかタイムアウトの場合はニュートラル値を返す。
     *
     * @param input Output control input / 出力制御入力
     * @return true if valid active input, false otherwise / 有効な入力があればtrue
     */
    bool getActiveControl(ControlInput& input) const;

    /**
     * @brief Get the currently active source
     *        現在のアクティブソースを取得
     */
    ControlSource getActiveSource() const;

    /**
     * @brief Check if there is an active control source
     *        アクティブな制御ソースがあるか確認
     */
    bool hasActiveControl() const;

    /**
     * @brief Get time since last control input
     *        最後の制御入力からの経過時間を取得
     *
     * @return Time in ms since last input / 最後の入力からの時間(ms)
     */
    uint32_t getTimeSinceLastInput() const;

    // ========================================================================
    // Timeout Configuration
    // タイムアウト設定
    // ========================================================================

    /**
     * @brief Set control timeout
     *        制御タイムアウトを設定
     *
     * @param timeout_ms Timeout in milliseconds / タイムアウト(ms)
     */
    void setTimeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    /**
     * @brief Get control timeout
     *        制御タイムアウトを取得
     */
    uint32_t getTimeout() const { return timeout_ms_; }

    // ========================================================================
    // Statistics
    // 統計情報
    // ========================================================================

    /**
     * @brief Get ESP-NOW packet count
     *        ESP-NOWパケット数を取得
     */
    uint32_t getESPNOWCount() const { return espnow_count_; }

    /**
     * @brief Get UDP packet count
     *        UDPパケット数を取得
     */
    uint32_t getUDPCount() const { return udp_count_; }

    /**
     * @brief Get WebSocket packet count
     *        WebSocketパケット数を取得
     */
    uint32_t getWebSocketCount() const { return ws_count_; }

    /**
     * @brief Reset statistics
     *        統計情報をリセット
     */
    void resetStats();

private:
    ControlArbiter() = default;
    ~ControlArbiter() = default;

    // Normalize raw ADC values
    // 生ADC値を正規化
    static float normalizeThrottle(uint16_t raw);
    static float normalizeAxis(uint16_t raw);

    // Get current time in ms
    // 現在時刻をmsで取得
    static uint32_t getCurrentTimeMs();

    // Check if source matches current comm mode
    // ソースが現在の通信モードに一致するか確認
    bool isSourceAllowed(ControlSource source) const;

    // State
    bool initialized_ = false;
    CommMode comm_mode_ = CommMode::ESPNOW;  // Default to ESP-NOW
    mutable SemaphoreHandle_t mutex_ = nullptr;

    // Control inputs from each source
    // 各ソースからの制御入力
    ControlInput espnow_input_ = {};
    ControlInput udp_input_ = {};
    ControlInput ws_input_ = {};

    // Timeout
    uint32_t timeout_ms_ = 500;  // Default 500ms

    // Statistics
    uint32_t espnow_count_ = 0;
    uint32_t udp_count_ = 0;
    uint32_t ws_count_ = 0;

    // Constants
    static constexpr uint16_t ADC_CENTER = 2048;
    static constexpr uint16_t ADC_MAX = 4095;
};

}  // namespace stampfly
