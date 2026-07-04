/**
 * @file system_state.hpp
 * @brief Unified System State Manager
 * @brief 統合システム状態管理
 *
 * Single Source of Truth for all system states.
 * Integrates flight state, calibration, commands, and readiness flags.
 *
 * システム全体の状態を一元管理する Single Source of Truth。
 * フライト状態、キャリブレーション、コマンド、準備状態フラグを統合。
 */

#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "stampfly_state.hpp"

namespace stampfly {

// Forward declarations
// 前方宣言
enum class ControlSource : uint8_t;

/**
 * @brief Flight command type (defined here to avoid circular dependency)
 * @brief フライトコマンドタイプ（循環依存回避のためここで定義）
 *
 * NOTE: This is duplicated from flight_command.hpp
 * 注意: flight_command.hppと重複定義（値は同一）
 */
enum class FlightCommandType {
    NONE = 0,
    TAKEOFF,         ///< Takeoff / 離陸
    LAND,            ///< Land / 着陸
    HOVER,           ///< Hover at specified altitude / ホバリング
    JUMP,            ///< Jump (takeoff → hover → land) / ジャンプ
    MOVE_VERTICAL,   ///< Vertical move (up/down) / 垂直移動
    ROTATE_YAW,      ///< Yaw rotation (cw/ccw) / ヨー回転
    MOVE_HORIZONTAL, ///< Horizontal move (forward/back/left/right) / 水平移動
};

/**
 * @brief Calibration state
 * @brief キャリブレーション状態
 */
enum class CalibrationState : uint8_t {
    NOT_STARTED,      ///< Initial state or after arm / 初期状態またはアーム後
    WAITING_LANDING,  ///< Waiting for landing (flying) / 着陸待機中（飛行中）
    CALIBRATING,      ///< Calibration in progress (white LED, arm blocked) / キャリブレーション実行中（白LED、アーム不可）
    COMPLETED         ///< Calibration done (green LED, arm allowed) / キャリブレーション完了（緑LED、アーム可）
};

/**
 * @brief Command execution state
 * @brief コマンド実行状態
 */
enum class CommandState : uint8_t {
    IDLE,       ///< Not executing any command / コマンド実行中でない
    RUNNING,    ///< Executing command / コマンド実行中
    COMPLETED,  ///< Completed successfully / 正常完了
    FAILED,     ///< Failed / 失敗
};

/**
 * @brief System readiness flags (bitfield)
 * @brief システム準備状態フラグ（ビットフィールド）
 */
enum ReadinessFlags : uint32_t {
    NONE              = 0,
    IMU_READY         = 1 << 0,   ///< IMU initialized / IMU初期化完了
    MAG_READY         = 1 << 1,   ///< Magnetometer initialized / 磁気センサー初期化完了
    BARO_READY        = 1 << 2,   ///< Barometer initialized / 気圧センサー初期化完了
    TOF_READY         = 1 << 3,   ///< ToF initialized / ToF初期化完了
    FLOW_READY        = 1 << 4,   ///< Optical flow initialized / オプティカルフロー初期化完了
    POWER_READY       = 1 << 5,   ///< Power monitor initialized / 電源モニタ初期化完了
    CALIBRATED        = 1 << 6,   ///< Landing calibration complete / 着陸キャリブレーション完了
    LANDED            = 1 << 7,   ///< Vehicle is landed / 着陸済み
    BATTERY_OK        = 1 << 8,   ///< Battery voltage OK / バッテリー電圧OK
    ATTITUDE_OK       = 1 << 9,   ///< Attitude estimation OK / 姿勢推定OK
    CONTROL_INPUT_OK  = 1 << 10,  ///< Control input valid / 制御入力有効

    // Composite flags
    // 複合フラグ
    ALL_SENSORS_READY = IMU_READY | MAG_READY | BARO_READY | TOF_READY | FLOW_READY | POWER_READY,
    ARM_READY         = ALL_SENSORS_READY | CALIBRATED | LANDED | BATTERY_OK,
};

/**
 * @brief Unified system state structure
 * @brief 統合システム状態構造体
 *
 * Single Source of Truth for the entire system.
 * システム全体の単一の真実の源。
 */
struct SystemState {
    FlightState flight_state;              ///< Current flight state / 現在のフライト状態
    CommandState command_state;            ///< Command execution state / コマンド実行状態
    CalibrationState calibration_state;    ///< Calibration state / キャリブレーション状態
    ControlSource active_control_source;   ///< Active control input source / アクティブな制御入力ソース
    ErrorCode error_code;                  ///< Error code / エラーコード
    uint32_t readiness_flags;              ///< Readiness bitfield / 準備状態ビットフィールド

    FlightCommandType current_command;     ///< Current executing command / 実行中のコマンド
    float command_progress;                ///< Command progress [0.0-1.0] / コマンド進捗率
    uint32_t last_transition_ms;           ///< Last state transition time / 最後の状態遷移時刻

    SystemState()
        : flight_state(FlightState::INIT)
        , command_state(CommandState::IDLE)
        , calibration_state(CalibrationState::NOT_STARTED)
        , active_control_source(static_cast<ControlSource>(0))  // ControlSource::NONE
        , error_code(ErrorCode::NONE)
        , readiness_flags(ReadinessFlags::NONE)
        , current_command(FlightCommandType::NONE)
        , command_progress(0.0f)
        , last_transition_ms(0)
    {}
};

/**
 * @brief State transition event
 * @brief 状態遷移イベント
 */
struct StateTransitionEvent {
    FlightState from_state;
    FlightState to_state;
    uint32_t timestamp_ms;
    const char* reason;
};

/**
 * @brief State change callback type
 * @brief 状態変更コールバック型
 */
using StateCallback = std::function<void(const StateTransitionEvent&)>;

/**
 * @brief Unified System State Manager (Singleton)
 * @brief 統合システム状態管理 (シングルトン)
 *
 * Manages all system states in one place:
 * - Flight state transitions with precondition validation
 * - Calibration state tracking
 * - Command execution state
 * - Readiness flags (sensor initialization, calibration, etc.)
 * - Event callbacks for state changes
 *
 * システム全体の状態を一元管理：
 * - 前提条件検証付きフライト状態遷移
 * - キャリブレーション状態追跡
 * - コマンド実行状態
 * - 準備状態フラグ（センサー初期化、キャリブレーション等）
 * - 状態変更イベントコールバック
 */
class SystemStateManager {
public:
    static SystemStateManager& getInstance();

    // Delete copy/move
    SystemStateManager(const SystemStateManager&) = delete;
    SystemStateManager& operator=(const SystemStateManager&) = delete;

    /**
     * @brief Initialize the state manager
     * @brief 状態管理を初期化
     */
    esp_err_t init();

    // ========================================================================
    // State Queries
    // 状態クエリ
    // ========================================================================

    /**
     * @brief Get complete system state
     * @brief システム状態全体を取得
     */
    const SystemState getState() const;

    /**
     * @brief Get current flight state
     * @brief 現在のフライト状態を取得
     */
    FlightState getFlightState() const;

    /**
     * @brief Get calibration state
     * @brief キャリブレーション状態を取得
     */
    CalibrationState getCalibrationState() const;

    /**
     * @brief Get command state
     * @brief コマンド状態を取得
     */
    CommandState getCommandState() const;

    /**
     * @brief Get error code
     * @brief エラーコードを取得
     */
    ErrorCode getErrorCode() const;

    /**
     * @brief Check if specific readiness flags are set
     * @brief 指定された準備フラグがセットされているか確認
     */
    bool isReady(uint32_t flags) const;

    /**
     * @brief Get all readiness flags
     * @brief 全準備フラグを取得
     */
    uint32_t getReadinessFlags() const;

    /**
     * @brief Check if transition to target state is allowed
     * @brief 目標状態への遷移が許可されているか確認
     */
    bool canTransitionTo(FlightState target) const;

    // ========================================================================
    // State Transitions
    // 状態遷移
    // ========================================================================

    /**
     * @brief Request state transition with precondition validation
     * @brief 前提条件検証付き状態遷移をリクエスト
     *
     * @param target Target flight state / 目標フライト状態
     * @param reason Reason for transition (for logging) / 遷移理由（ログ用）
     * @return true if transition allowed and executed / 遷移が許可され実行された場合true
     */
    bool requestTransition(FlightState target, const char* reason = nullptr);

    /**
     * @brief Request ARM (IDLE → ARMED)
     * @brief ARMをリクエスト (IDLE → ARMED)
     *
     * Checks preconditions: calibration complete, sensors ready, landed, battery OK
     * 前提条件チェック: キャリブレーション完了、センサー準備完了、着陸済み、バッテリーOK
     */
    bool requestArm();

    /**
     * @brief Request DISARM (ARMED/FLYING → IDLE)
     * @brief DISARMをリクエスト (ARMED/FLYING → IDLE)
     */
    bool requestDisarm();

    /**
     * @brief Force state change (bypass validation, use with caution)
     * @brief 状態を強制変更（検証バイパス、注意して使用）
     */
    void setFlightState(FlightState state);

    // ========================================================================
    // Readiness Management
    // 準備状態管理
    // ========================================================================

    /**
     * @brief Set or clear readiness flag
     * @brief 準備フラグをセット/クリア
     */
    void setReady(uint32_t flag, bool ready);

    /**
     * @brief Update calibration state
     * @brief キャリブレーション状態を更新
     */
    void updateCalibrationState(CalibrationState state);

    /**
     * @brief Update command state
     * @brief コマンド状態を更新
     */
    void updateCommandState(CommandState state, FlightCommandType command = FlightCommandType::NONE, float progress = 0.0f);

    /**
     * @brief Update active control source
     * @brief アクティブ制御ソースを更新
     */
    void updateControlSource(ControlSource source);

    /**
     * @brief Set error code
     * @brief エラーコードを設定
     */
    void setError(ErrorCode code);

    /**
     * @brief Clear error
     * @brief エラーをクリア
     */
    void clearError();

    // ========================================================================
    // Event Callbacks
    // イベントコールバック
    // ========================================================================

    /**
     * @brief Subscribe to state change events
     * @brief 状態変更イベントを購読
     *
     * @param callback Callback function / コールバック関数
     * @return Subscription ID (for future unsubscribe support) / 購読ID
     */
    int subscribeStateChange(StateCallback callback);

    // ========================================================================
    // Debug Mode
    // デバッグモード
    // ========================================================================

    /**
     * @brief Enable/disable debug mode (bypasses some safety checks)
     * @brief デバッグモード有効/無効（一部の安全チェックをバイパス）
     */
    void setDebugMode(bool enable) { debug_mode_ = enable; }

    /**
     * @brief Check if debug mode is enabled
     * @brief デバッグモードが有効か確認
     */
    bool isDebugMode() const { return debug_mode_; }

private:
    SystemStateManager() = default;

    // Notify callbacks about state transition
    // 状態遷移をコールバックに通知
    void notifyCallbacks(const StateTransitionEvent& event);

    // Get current time in milliseconds
    // 現在時刻をミリ秒で取得
    static uint32_t getCurrentTimeMs();

    // State
    mutable SemaphoreHandle_t mutex_ = nullptr;
    SystemState state_;
    bool debug_mode_ = false;

    // Event callbacks
    // イベントコールバック
    static constexpr int MAX_CALLBACKS = 4;
    StateCallback callbacks_[MAX_CALLBACKS];
    int callback_count_ = 0;
};

}  // namespace stampfly
