/**
 * @file command_queue.hpp
 * @brief Flight Command Queue with Precondition Handling
 * @brief フライトコマンドキュー（前提条件処理付き）
 *
 * Manages a queue of flight commands with:
 * - Precondition checking (readiness flags)
 * - Automatic retry when ready
 * - Timeout handling
 * - Priority ordering
 *
 * フライトコマンドをキュー管理：
 * - 前提条件チェック（準備フラグ）
 * - 準備完了時の自動リトライ
 * - タイムアウト処理
 * - 優先度順序付け
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "system_state.hpp"

namespace stampfly {

/**
 * @brief Command entry in the queue
 * @brief キュー内のコマンドエントリ
 */
struct CommandEntry {
    FlightCommandType type;              ///< Command type / コマンドタイプ
    float target_altitude;               ///< Target altitude [m] / 目標高度
    float duration_s;                    ///< Duration [s] / 持続時間
    float climb_rate;                    ///< Climb rate [m/s] / 上昇速度
    float descent_rate;                  ///< Descent rate [m/s] / 降下速度
    float target_yaw_deg;               ///< Target yaw angle [deg] / 目標ヨー角
    float target_pos_x;                 ///< Target position NED X [m] / 目標位置 NED X
    float target_pos_y;                 ///< Target position NED Y [m] / 目標位置 NED Y

    uint32_t enqueue_time_ms;            ///< Enqueue timestamp / キュー登録時刻
    uint32_t timeout_ms;                 ///< Timeout [ms] / タイムアウト
    uint8_t priority;                    ///< Priority (0-255, higher = more urgent) / 優先度（高いほど優先）
    uint32_t precondition_flags;         ///< Required ReadinessFlags / 必要な準備フラグ
    uint8_t max_retries;                 ///< Maximum retry count / 最大リトライ回数
    uint8_t retry_count;                 ///< Current retry count / 現在のリトライ回数
    int command_id;                      ///< Unique command ID / 一意のコマンドID
    bool active;                         ///< Entry is active / エントリが有効

    CommandEntry()
        : type(FlightCommandType::NONE)
        , target_altitude(0.0f)
        , duration_s(0.0f)
        , climb_rate(0.5f)
        , descent_rate(0.3f)
        , target_yaw_deg(0.0f)
        , target_pos_x(0.0f)
        , target_pos_y(0.0f)
        , enqueue_time_ms(0)
        , timeout_ms(30000)
        , priority(100)
        , precondition_flags(ReadinessFlags::CALIBRATED | ReadinessFlags::LANDED)
        , max_retries(3)
        , retry_count(0)
        , command_id(-1)
        , active(false)
    {}
};

/**
 * @brief Flight Command Queue (Singleton)
 * @brief フライトコマンドキュー（シングルトン）
 *
 * Thread-safe command queue with precondition handling.
 * Commands wait in queue until system is ready to execute them.
 *
 * スレッドセーフなコマンドキュー（前提条件処理付き）。
 * システムが実行可能になるまでキューで待機。
 */
class CommandQueue {
public:
    static CommandQueue& getInstance();

    // Delete copy/move
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;

    /**
     * @brief Initialize the command queue
     * @brief コマンドキューを初期化
     */
    esp_err_t init();

    // ========================================================================
    // Command Enqueueing
    // コマンド登録
    // ========================================================================

    /**
     * @brief Enqueue a command
     * @brief コマンドをキューに登録
     *
     * @param type Command type / コマンドタイプ
     * @param target_altitude Target altitude [m] / 目標高度
     * @param duration_s Duration [s] / 持続時間
     * @param timeout_ms Timeout [ms] (default: 30s) / タイムアウト
     * @param priority Priority (0-255, default: 100) / 優先度
     * @return Command ID, or -1 on failure / コマンドID、失敗時は-1
     */
    int enqueue(FlightCommandType type,
                float target_altitude,
                float duration_s = 0.0f,
                uint32_t timeout_ms = 30000,
                uint8_t priority = 100);

    /**
     * @brief Enqueue a command with full parameters
     * @brief フルパラメータでコマンドを登録
     */
    int enqueueDetailed(const CommandEntry& entry);

    // ========================================================================
    // Command Cancellation
    // コマンドキャンセル
    // ========================================================================

    /**
     * @brief Cancel a specific command by ID
     * @brief IDを指定してコマンドをキャンセル
     */
    bool cancel(int command_id);

    /**
     * @brief Cancel the currently executing command
     * @brief 現在実行中のコマンドをキャンセル
     */
    bool cancelCurrent();

    /**
     * @brief Cancel all pending and current commands
     * @brief すべての待機中・実行中コマンドをキャンセル
     */
    void cancelAll();

    // ========================================================================
    // Queue Processing
    // キュー処理
    // ========================================================================

    /**
     * @brief Process the queue (called periodically from control task)
     * @brief キューを処理（制御タスクから定期的に呼ばれる）
     *
     * Checks if the current command can start, handles timeout, retries.
     * Returns true if a command was started.
     *
     * 現在のコマンドが開始可能かチェック、タイムアウト・リトライ処理。
     * コマンドが開始された場合trueを返す。
     *
     * @return true if a command was started / コマンドが開始された場合true
     */
    bool processQueue();

    /**
     * @brief Check if a command is ready to execute
     * @brief コマンドが実行可能か確認
     */
    bool isReadyToExecute(const CommandEntry& cmd) const;

    /**
     * @brief Mark current command as completed
     * @brief 現在のコマンドを完了としてマーク
     */
    void markCompleted();

    /**
     * @brief Mark current command as failed
     * @brief 現在のコマンドを失敗としてマーク
     */
    void markFailed();

    // ========================================================================
    // Queue State Queries
    // キュー状態クエリ
    // ========================================================================

    /**
     * @brief Check if queue is empty
     * @brief キューが空か確認
     */
    bool isEmpty() const;

    /**
     * @brief Get number of pending commands
     * @brief 待機中のコマンド数を取得
     */
    int getQueueSize() const;

    /**
     * @brief Get currently executing command
     * @brief 現在実行中のコマンドを取得
     *
     * @return Pointer to current command, or nullptr / 現在のコマンドへのポインタ、なければnullptr
     */
    const CommandEntry* getCurrentCommand() const;

    /**
     * @brief Get command by ID
     * @brief IDでコマンドを取得
     */
    const CommandEntry* getCommandById(int command_id) const;

private:
    CommandQueue() = default;

    // Find next executable command in queue
    // キュー内の次の実行可能コマンドを検索
    int findNextExecutable();

    // Remove command from queue
    // キューからコマンドを削除
    void removeCommand(int index);

    // Get current time in milliseconds
    // 現在時刻をミリ秒で取得
    static uint32_t getCurrentTimeMs();

    // State
    mutable SemaphoreHandle_t mutex_ = nullptr;
    bool initialized_ = false;

    // Command queue (static allocation)
    // コマンドキュー（静的割り当て）
    static constexpr int MAX_QUEUE_SIZE = 8;
    CommandEntry queue_[MAX_QUEUE_SIZE];
    int queue_count_ = 0;
    int current_executing_index_ = -1;  ///< Index of currently executing command / 実行中コマンドのインデックス
    int next_command_id_ = 1;           ///< Auto-incrementing command ID / 自動インクリメントコマンドID
};

}  // namespace stampfly
