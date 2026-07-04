/**
 * @file command_queue.cpp
 * @brief Flight Command Queue Implementation
 * @brief フライトコマンドキュー実装
 */

#include "command_queue.hpp"
#include "flight_command.hpp"
#include "system_state.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "CommandQueue";

namespace stampfly {

// ============================================================================
// Singleton
// ============================================================================

CommandQueue& CommandQueue::getInstance()
{
    static CommandQueue instance;
    return instance;
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t CommandQueue::init()
{
    if (initialized_) {
        return ESP_OK;
    }

    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize queue
    // キューを初期化
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        queue_[i] = CommandEntry();
    }
    queue_count_ = 0;
    current_executing_index_ = -1;
    next_command_id_ = 1;

    initialized_ = true;
    ESP_LOGI(TAG, "CommandQueue initialized (max queue size: %d)", MAX_QUEUE_SIZE);
    return ESP_OK;
}

// ============================================================================
// Command Enqueueing
// ============================================================================

int CommandQueue::enqueue(FlightCommandType type,
                          float target_altitude,
                          float duration_s,
                          uint32_t timeout_ms,
                          uint8_t priority)
{
    CommandEntry entry;
    entry.type = type;
    entry.target_altitude = target_altitude;
    entry.duration_s = duration_s;
    entry.timeout_ms = timeout_ms;
    entry.priority = priority;

    // Set default preconditions based on command type
    // コマンドタイプに基づいてデフォルト前提条件を設定
    switch (type) {
        case FlightCommandType::TAKEOFF:
        case FlightCommandType::JUMP:
            entry.precondition_flags = ReadinessFlags::CALIBRATED | ReadinessFlags::LANDED;
            break;
        case FlightCommandType::HOVER:
            entry.precondition_flags = ReadinessFlags::NONE;  // Can hover anytime when armed
            break;
        case FlightCommandType::LAND:
            entry.precondition_flags = ReadinessFlags::NONE;
            break;
        case FlightCommandType::MOVE_VERTICAL:
        case FlightCommandType::ROTATE_YAW:
        case FlightCommandType::MOVE_HORIZONTAL:
            entry.precondition_flags = ReadinessFlags::NONE;  // Allowed while flying
            break;
        default:
            entry.precondition_flags = ReadinessFlags::NONE;
            break;
    }

    return enqueueDetailed(entry);
}

int CommandQueue::enqueueDetailed(const CommandEntry& entry)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Check if queue is full
    // キューが満杯かチェック
    if (queue_count_ >= MAX_QUEUE_SIZE) {
        xSemaphoreGive(mutex_);
        ESP_LOGE(TAG, "Queue full, cannot enqueue command");
        return -1;
    }

    // Find empty slot
    // 空きスロットを検索
    int index = -1;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (!queue_[i].active) {
            index = i;
            break;
        }
    }

    if (index == -1) {
        xSemaphoreGive(mutex_);
        ESP_LOGE(TAG, "No available slot in queue");
        return -1;
    }

    // Add command to queue
    // コマンドをキューに追加
    queue_[index] = entry;
    queue_[index].active = true;
    queue_[index].enqueue_time_ms = getCurrentTimeMs();
    queue_[index].command_id = next_command_id_++;
    queue_[index].retry_count = 0;

    queue_count_++;
    int cmd_id = queue_[index].command_id;

    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Command enqueued: ID=%d, type=%d, alt=%.2f, queue_size=%d",
             cmd_id, static_cast<int>(entry.type), entry.target_altitude, queue_count_);

    return cmd_id;
}

// ============================================================================
// Command Cancellation
// ============================================================================

bool CommandQueue::cancel(int command_id)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (queue_[i].active && queue_[i].command_id == command_id) {
            queue_[i].active = false;
            queue_count_--;

            if (i == current_executing_index_) {
                current_executing_index_ = -1;
                // Update SystemStateManager command state
                // SystemStateManager のコマンド状態を更新
                SystemStateManager::getInstance().updateCommandState(
                    CommandState::IDLE, FlightCommandType::NONE, 0.0f);
            }

            xSemaphoreGive(mutex_);
            ESP_LOGI(TAG, "Command cancelled: ID=%d", command_id);
            return true;
        }
    }

    xSemaphoreGive(mutex_);
    ESP_LOGW(TAG, "Command ID not found: %d", command_id);
    return false;
}

bool CommandQueue::cancelCurrent()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (current_executing_index_ >= 0 &&
        current_executing_index_ < MAX_QUEUE_SIZE &&
        queue_[current_executing_index_].active) {
        int cmd_id = queue_[current_executing_index_].command_id;
        queue_[current_executing_index_].active = false;
        queue_count_--;
        current_executing_index_ = -1;

        // Update SystemStateManager command state
        // SystemStateManager のコマンド状態を更新
        SystemStateManager::getInstance().updateCommandState(
            CommandState::IDLE, FlightCommandType::NONE, 0.0f);

        xSemaphoreGive(mutex_);
        ESP_LOGI(TAG, "Current command cancelled: ID=%d", cmd_id);
        return true;
    }

    xSemaphoreGive(mutex_);
    return false;
}

void CommandQueue::cancelAll()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    int cancelled_count = 0;
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (queue_[i].active) {
            queue_[i].active = false;
            cancelled_count++;
        }
    }

    queue_count_ = 0;
    current_executing_index_ = -1;

    // Update SystemStateManager command state
    // SystemStateManager のコマンド状態を更新
    SystemStateManager::getInstance().updateCommandState(
        CommandState::IDLE, FlightCommandType::NONE, 0.0f);

    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "All commands cancelled (%d total)", cancelled_count);
}

// ============================================================================
// Queue Processing
// ============================================================================

bool CommandQueue::processQueue()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    // If currently executing a command, don't start a new one
    // 実行中のコマンドがある場合、新しいコマンドを開始しない
    if (current_executing_index_ >= 0 &&
        current_executing_index_ < MAX_QUEUE_SIZE &&
        queue_[current_executing_index_].active) {
        xSemaphoreGive(mutex_);
        return false;
    }

    // Find next executable command
    // 次の実行可能コマンドを検索
    int next_index = findNextExecutable();

    if (next_index < 0) {
        xSemaphoreGive(mutex_);
        return false;  // No executable command found
    }

    CommandEntry& cmd = queue_[next_index];

    // Check timeout
    // タイムアウトチェック
    uint32_t now = getCurrentTimeMs();
    if (now - cmd.enqueue_time_ms > cmd.timeout_ms) {
        ESP_LOGW(TAG, "Command timeout: ID=%d (enqueued %lu ms ago)",
                 cmd.command_id, now - cmd.enqueue_time_ms);
        cmd.active = false;
        queue_count_--;

        // Update SystemStateManager
        SystemStateManager::getInstance().updateCommandState(
            CommandState::FAILED, cmd.type, 0.0f);

        xSemaphoreGive(mutex_);
        return false;
    }

    // Check if ready to execute
    // 実行可能かチェック
    if (!isReadyToExecute(cmd)) {
        // Not ready yet - keep waiting
        // まだ準備できていない - 待機継続
        xSemaphoreGive(mutex_);
        return false;
    }

    // Start command
    // コマンド開始
    current_executing_index_ = next_index;
    cmd.retry_count++;

    int cmd_id = cmd.command_id;
    FlightCommandType type = cmd.type;

    // Update SystemStateManager
    SystemStateManager::getInstance().updateCommandState(
        CommandState::RUNNING, type, 0.0f);

    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Starting command: ID=%d, type=%d, retry=%d/%d",
             cmd_id, static_cast<int>(type), cmd.retry_count, cmd.max_retries);

    return true;
}

bool CommandQueue::isReadyToExecute(const CommandEntry& cmd) const
{
    // Check if system is in a state that allows command execution
    // システムがコマンド実行可能な状態かチェック
    auto& state_mgr = SystemStateManager::getInstance();

    FlightState flight_state = state_mgr.getFlightState();

    // Must be IDLE, ARMED, or FLYING (for in-flight commands like MOVE_VERTICAL, ROTATE_YAW)
    // IDLE、ARMED、または FLYING 状態である必要がある（飛行中コマンド対応）
    if (flight_state != FlightState::IDLE && flight_state != FlightState::ARMED &&
        flight_state != FlightState::FLYING) {
        return false;
    }

    // Check readiness flags
    // 準備フラグをチェック
    if (cmd.precondition_flags != ReadinessFlags::NONE) {
        if (!state_mgr.isReady(cmd.precondition_flags)) {
            return false;
        }
    }

    // Check calibration state
    // キャリブレーション状態をチェック
    if ((cmd.precondition_flags & ReadinessFlags::CALIBRATED) != 0) {
        if (state_mgr.getCalibrationState() != CalibrationState::COMPLETED) {
            return false;
        }
    }

    return true;
}

void CommandQueue::markCompleted()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (current_executing_index_ >= 0 &&
        current_executing_index_ < MAX_QUEUE_SIZE &&
        queue_[current_executing_index_].active) {
        int cmd_id = queue_[current_executing_index_].command_id;
        queue_[current_executing_index_].active = false;
        queue_count_--;
        current_executing_index_ = -1;

        // Update SystemStateManager
        SystemStateManager::getInstance().updateCommandState(
            CommandState::COMPLETED, FlightCommandType::NONE, 1.0f);

        xSemaphoreGive(mutex_);
        ESP_LOGI(TAG, "Command completed: ID=%d", cmd_id);
        return;
    }

    xSemaphoreGive(mutex_);
}

void CommandQueue::markFailed()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (current_executing_index_ >= 0 &&
        current_executing_index_ < MAX_QUEUE_SIZE &&
        queue_[current_executing_index_].active) {
        int cmd_id = queue_[current_executing_index_].command_id;
        FlightCommandType type = queue_[current_executing_index_].type;

        queue_[current_executing_index_].active = false;
        queue_count_--;
        current_executing_index_ = -1;

        // Update SystemStateManager
        SystemStateManager::getInstance().updateCommandState(
            CommandState::FAILED, type, 0.0f);

        xSemaphoreGive(mutex_);
        ESP_LOGW(TAG, "Command failed: ID=%d", cmd_id);
        return;
    }

    xSemaphoreGive(mutex_);
}

// ============================================================================
// Queue State Queries
// ============================================================================

bool CommandQueue::isEmpty() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool empty = (queue_count_ == 0);
    xSemaphoreGive(mutex_);
    return empty;
}

int CommandQueue::getQueueSize() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    int size = queue_count_;
    xSemaphoreGive(mutex_);
    return size;
}

const CommandEntry* CommandQueue::getCurrentCommand() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    if (current_executing_index_ >= 0 &&
        current_executing_index_ < MAX_QUEUE_SIZE &&
        queue_[current_executing_index_].active) {
        const CommandEntry* cmd = &queue_[current_executing_index_];
        xSemaphoreGive(mutex_);
        return cmd;
    }

    xSemaphoreGive(mutex_);
    return nullptr;
}

const CommandEntry* CommandQueue::getCommandById(int command_id) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);

    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (queue_[i].active && queue_[i].command_id == command_id) {
            const CommandEntry* cmd = &queue_[i];
            xSemaphoreGive(mutex_);
            return cmd;
        }
    }

    xSemaphoreGive(mutex_);
    return nullptr;
}

// ============================================================================
// Private Helper Functions
// ============================================================================

int CommandQueue::findNextExecutable()
{
    // Find highest priority command that is ready
    // 準備完了している最高優先度コマンドを検索
    int best_index = -1;
    uint8_t best_priority = 0;

    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        if (!queue_[i].active) {
            continue;
        }

        // Skip if already executing
        // 既に実行中の場合はスキップ
        if (i == current_executing_index_) {
            continue;
        }

        // Check if higher priority
        // より高い優先度かチェック
        if (queue_[i].priority > best_priority) {
            best_index = i;
            best_priority = queue_[i].priority;
        } else if (queue_[i].priority == best_priority && best_index >= 0) {
            // Same priority - pick older one (FIFO)
            // 同じ優先度 - より古いものを選択（FIFO）
            if (queue_[i].enqueue_time_ms < queue_[best_index].enqueue_time_ms) {
                best_index = i;
            }
        }
    }

    return best_index;
}

uint32_t CommandQueue::getCurrentTimeMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace stampfly
