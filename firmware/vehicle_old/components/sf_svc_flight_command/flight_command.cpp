// High-Level Flight Command Service Implementation
// 高レベル飛行コマンドサービス実装
#include "flight_command.hpp"
#include "command_queue.hpp"
#include "system_state.hpp"
#include "control_arbiter.hpp"
#include "sensor_fusion.hpp"
#include "landing_handler.hpp"
#include "stampfly_state.hpp"
#include "controller_comm.hpp"
#include "position_controller.hpp"
#include "esp_log.h"
#include <cmath>

// Forward declarations for globals
// グローバル変数の前方宣言
#include "ring_buffer.hpp"
#include "stampfly_math.hpp"
namespace globals {
    extern stampfly::LandingHandler g_landing_handler;
    extern sf::SensorFusion g_fusion;

    // ToF sensor ring buffer (for open-loop climb altitude check)
    // ToFセンサーリングバッファ（開ループ上昇の高度チェック用）
    inline constexpr int TOF_BUFFER_SIZE = 16;
    extern RingBuffer<float, TOF_BUFFER_SIZE> g_tof_bottom_buf;
}

static const char* TAG = "FlightCmd";

namespace stampfly {

// Get singleton instance
// シングルトンインスタンスを取得
FlightCommandService& FlightCommandService::getInstance() {
    static FlightCommandService instance;
    return instance;
}

// Initialize the service
// サービスの初期化
esp_err_t FlightCommandService::init() {
    ESP_LOGI(TAG, "Initializing Flight Command Service");

    state_ = FlightCommandState::IDLE;
    current_command_ = FlightCommandType::NONE;
    phase_ = ExecutionPhase::INIT;
    elapsed_time_ = 0.0f;
    hover_timer_ = 0.0f;

    return ESP_OK;
}

// Execute command (non-blocking)
// コマンド実行（非ブロッキング）
bool FlightCommandService::executeCommand(FlightCommandType type, const FlightCommandParams& params) {
    // Enqueue command to CommandQueue
    // CommandQueueにコマンドを登録
    auto& queue = CommandQueue::getInstance();

    // Check if another command is running or queued
    // 他のコマンドが実行中またはキュー待ちかチェック
    if (isRunning() || !queue.isEmpty()) {
        ESP_LOGW(TAG, "Command queue not empty (running or queued commands exist)");
        // Still allow enqueueing - queue will handle priority
        // キューイング自体は許可 - キューが優先度を処理
    }

    // Auto-ARM for WiFi commands (enable debug mode)
    // WiFiコマンド用の自動ARM機能（デバッグモード有効化）
    auto& state_mgr = StampFlyState::getInstance();
    auto& sys_state_mgr = SystemStateManager::getInstance();

    // Enable debug mode to bypass some safety checks for WiFi commands
    // WiFiコマンド用に一部の安全チェックをバイパス
    state_mgr.setDebugMode(true);
    sys_state_mgr.setDebugMode(true);

    FlightState current_state = state_mgr.getFlightState();

    if (current_state != FlightState::ARMED && current_state != FlightState::FLYING) {
        // Need to ARM first
        // まずARMする必要がある

        // If not IDLE or ERROR, transition to IDLE first
        // IDLEまたはERROR状態でない場合、まずIDLEに遷移
        if (current_state != FlightState::IDLE && current_state != FlightState::ERROR) {
            ESP_LOGW(TAG, "Cannot ARM from state %d, transitioning to IDLE", static_cast<int>(current_state));
            state_mgr.setFlightState(FlightState::IDLE);
            sys_state_mgr.setFlightState(FlightState::IDLE);
        }

        // Request ARM
        // ARM要求
        if (!state_mgr.requestArm()) {
            ESP_LOGE(TAG, "Failed to ARM vehicle for WiFi command");
            state_mgr.setDebugMode(false);
            sys_state_mgr.setDebugMode(false);
            return false;
        }

        globals::g_landing_handler.reset();  // キャリブレーション状態をリセット

        // Also ARM in SystemStateManager
        // SystemStateManagerでもARM
        sys_state_mgr.requestArm();

        ESP_LOGI(TAG, "Auto-ARMed for WiFi command");
    }

    // Enqueue to CommandQueue with full parameters
    // フルパラメータでCommandQueueに登録
    CommandEntry entry;
    entry.type = type;
    entry.target_altitude = params.target_altitude;
    entry.duration_s = params.duration_s;
    entry.climb_rate = params.climb_rate;
    entry.descent_rate = params.descent_rate;
    entry.target_yaw_deg = params.target_yaw_deg;
    entry.target_pos_x = params.target_pos_x;
    entry.target_pos_y = params.target_pos_y;

    // Set preconditions based on command type
    // コマンドタイプに基づいて前提条件を設定
    switch (type) {
        case FlightCommandType::TAKEOFF:
        case FlightCommandType::JUMP:
            entry.precondition_flags = ReadinessFlags::CALIBRATED | ReadinessFlags::LANDED;
            break;
        default:
            entry.precondition_flags = ReadinessFlags::NONE;
            break;
    }

    int cmd_id = queue.enqueueDetailed(entry);

    if (cmd_id < 0) {
        ESP_LOGE(TAG, "Failed to enqueue command (queue full)");
        state_mgr.setDebugMode(false);
        sys_state_mgr.setDebugMode(false);
        return false;
    }

    // Note: Do NOT set current_command_id_ here - it will be set in update()
    // when CommandQueue actually starts the command
    // 注意: ここではcurrent_command_id_を設定しない - CommandQueueが実際に
    // コマンドを開始したときにupdate()で設定される
    ESP_LOGI(TAG, "Command enqueued: ID=%d, type=%d", cmd_id, static_cast<int>(type));

    return true;
}

// Cancel current command
// 現在のコマンドをキャンセル
void FlightCommandService::cancel() {
    // Cancel current command in queue
    // キュー内の現在のコマンドをキャンセル
    auto& queue = CommandQueue::getInstance();

    if (current_command_id_ >= 0) {
        queue.cancel(current_command_id_);
        current_command_id_ = -1;
    }

    if (isRunning()) {
        ESP_LOGI(TAG, "Cancelling command: %d", static_cast<int>(current_command_));
        state_ = FlightCommandState::IDLE;
        current_command_ = FlightCommandType::NONE;
        phase_ = ExecutionPhase::INIT;

        // Send zero throttle to stop
        // ゼロスロットルで停止
        sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

// Update command execution (called from control task at 400Hz)
// 定期更新（制御タスクから400Hzで呼ばれる）
void FlightCommandService::update(float dt) {
    // Get current command from CommandQueue
    // CommandQueueから現在のコマンドを取得
    auto& queue = CommandQueue::getInstance();
    const CommandEntry* cmd = queue.getCurrentCommand();

    if (cmd == nullptr) {
        // No command running - reset state
        // コマンド実行中でない - 状態リセット
        if (isRunning()) {
            state_ = FlightCommandState::IDLE;
            current_command_ = FlightCommandType::NONE;
            phase_ = ExecutionPhase::INIT;
            elapsed_time_ = 0.0f;
            hover_timer_ = 0.0f;
            current_command_id_ = -1;
        }
        return;
    }

    // New command started - initialize
    // 新しいコマンド開始 - 初期化
    if (current_command_id_ != cmd->command_id) {
        ESP_LOGI(TAG, "Starting command from queue: ID=%d, type=%d",
                 cmd->command_id, static_cast<int>(cmd->type));
        current_command_id_ = cmd->command_id;
        current_command_ = cmd->type;
        params_.target_altitude = cmd->target_altitude;
        params_.duration_s = cmd->duration_s;
        params_.climb_rate = cmd->climb_rate;
        params_.descent_rate = cmd->descent_rate;
        params_.target_yaw_deg = cmd->target_yaw_deg;
        params_.target_pos_x = cmd->target_pos_x;
        params_.target_pos_y = cmd->target_pos_y;
        state_ = FlightCommandState::RUNNING;
        phase_ = ExecutionPhase::INIT;
        elapsed_time_ = 0.0f;
        hover_timer_ = 0.0f;
    }

    elapsed_time_ += dt;

    // Get current altitude
    // 現在の高度を取得
    float current_altitude = getCurrentAltitude();

    // Debug log every 100 cycles (~250ms @ 400Hz)
    static int update_log_counter = 0;
    if (++update_log_counter >= 100) {
        ESP_LOGI(TAG, "FlightCommand::update() - cmd=%d, phase=%d, alt=%.3f m, elapsed=%.1f s",
                 static_cast<int>(current_command_), static_cast<int>(phase_),
                 current_altitude, elapsed_time_);
        update_log_counter = 0;
    }

    // Command-specific state machine
    // コマンド固有の状態マシン
    switch (current_command_) {
        case FlightCommandType::JUMP:
            updateJumpCommand(dt, current_altitude);
            break;

        case FlightCommandType::TAKEOFF:
            updateTakeoffCommand(dt, current_altitude);
            break;

        case FlightCommandType::LAND:
            updateLandCommand(dt, current_altitude);
            break;

        case FlightCommandType::HOVER:
            updateHoverCommand(dt, current_altitude);
            break;

        case FlightCommandType::MOVE_VERTICAL:
            updateMoveVerticalCommand(dt, current_altitude);
            break;

        case FlightCommandType::ROTATE_YAW:
            updateRotateYawCommand(dt, current_altitude);
            break;

        case FlightCommandType::MOVE_HORIZONTAL:
            updateMoveHorizontalCommand(dt, current_altitude);
            break;

        default:
            // Unknown command, fail
            // 未知のコマンド、失敗
            ESP_LOGW(TAG, "Unknown command type: %d", static_cast<int>(current_command_));
            state_ = FlightCommandState::FAILED;
            queue.markFailed();
            current_command_id_ = -1;
            break;
    }

    // Check if command completed or failed
    // コマンドが完了または失敗したかチェック
    if (state_ == FlightCommandState::COMPLETED || phase_ == ExecutionPhase::DONE) {
        ESP_LOGI(TAG, "Command completed: ID=%d", current_command_id_);
        queue.markCompleted();
        state_ = FlightCommandState::IDLE;
        current_command_ = FlightCommandType::NONE;
        phase_ = ExecutionPhase::INIT;
        current_command_id_ = -1;
    } else if (state_ == FlightCommandState::FAILED) {
        ESP_LOGW(TAG, "Command failed: ID=%d", current_command_id_);
        queue.markFailed();
        state_ = FlightCommandState::IDLE;
        current_command_ = FlightCommandType::NONE;
        phase_ = ExecutionPhase::INIT;
        current_command_id_ = -1;
        sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);  // Stop motors
    }

    // Safety timeout - fail if command takes too long
    // 安全タイムアウト - コマンドが長時間かかる場合は失敗
    const float MAX_COMMAND_TIME = 30.0f;  // 30 seconds
    if (elapsed_time_ > MAX_COMMAND_TIME) {
        ESP_LOGW(TAG, "Command timeout after %.1f seconds", elapsed_time_);
        state_ = FlightCommandState::FAILED;
        queue.markFailed();
        current_command_id_ = -1;
        sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);  // Stop motors
    }
}

// Check if command can be executed
// 実行可能かチェック
bool FlightCommandService::canExecute() const {
    // Check landing status
    // 着陸状態をチェック
    if (!globals::g_landing_handler.isLanded()) {
        ESP_LOGW(TAG, "Warning: vehicle not landed (allowing for WiFi commands)");
        // Don't block - allow WiFi commands even when not landed
        // WiFiコマンドのために許可（着陸検出なしでも実行可能）
    }

    // Check calibration status
    // キャリブレーション状態をチェック
    if (!globals::g_landing_handler.canArm()) {
        ESP_LOGW(TAG, "Calibration not completed - command rejected");
        // Block execution - calibration is required for safe flight
        // キャリブレーション必須（安全な飛行のため）
        return false;
    }

    // TODO: Add battery voltage check
    // バッテリー電圧チェックを追加

    // For WiFi commands, always allow execution (user responsibility)
    // WiFiコマンドは常に実行許可（ユーザーの責任）
    return true;
}

// Send control input to ControlArbiter
// 制御出力を ControlArbiter に送る
void FlightCommandService::sendControlInput(float throttle, float roll, float pitch, float yaw, uint8_t flags) {
    // Send to ControlArbiter via WebSocket source
    // WebSocketソース経由でControlArbiterに送信
    auto& arbiter = ControlArbiter::getInstance();
    arbiter.updateFromWebSocket(throttle, roll, pitch, yaw, flags);

    // Debug log every 100 cycles (~250ms @ 400Hz)
    static int send_log_counter = 0;
    if (++send_log_counter >= 100) {
        ESP_LOGI(TAG, "→ ControlArbiter: T=%.2f R=%.2f P=%.2f Y=%.2f", throttle, roll, pitch, yaw);
        send_log_counter = 0;
    }
}

// Get current altitude estimate
// 現在の高度推定値を取得
float FlightCommandService::getCurrentAltitude() const {
    // Get altitude from sensor fusion (NED frame, so negative = up)
    // センサーフュージョンから高度を取得（NEDフレームなので負の値が上方向）
    auto state = globals::g_fusion.getState();

    // Return positive altitude (negate Z in NED frame)
    // 正の高度を返す（NEDフレームのZを反転）
    return -state.position.z;
}

// ============================================================================
// Command-specific update functions
// コマンド固有の更新関数
// ============================================================================

void FlightCommandService::updateJumpCommand(float dt, float current_altitude) {
    // JUMP command state machine: INIT → CLIMBING → DESCENDING → DONE
    // JUMPコマンド状態マシン: INIT → 上昇 → 降下 → 完了
    //
    // Simple proportional control using raw ToF altitude:
    // シンプルな比例制御（ToF生値を使用）:
    //   duty = hover_throttle + Kp * (target - tof_altitude)

    // Get raw ToF altitude (used throughout jump command)
    // ToF生値を取得（Jumpコマンド全体で使用）
    float tof_altitude = 0.0f;
    if (globals::g_tof_bottom_buf.count() > 0) {
        tof_altitude = globals::g_tof_bottom_buf.latest();
    }

    switch (phase_) {
        case ExecutionPhase::INIT:
            ESP_LOGI(TAG, "JUMP: Starting climb to %.2f m (current ToF: %.2f m)",
                     params_.target_altitude, tof_altitude);
            phase_ = ExecutionPhase::CLIMBING;
            break;

        case ExecutionPhase::CLIMBING:
            // Climb with proportional control using raw ToF altitude
            // ToF生値を使った比例制御で上昇
            {
                constexpr float HOVER_THROTTLE = 0.60f;  // Hover thrust / ホバリング推力
                constexpr float KP = 0.8f;               // Proportional gain / 比例ゲイン

                float altitude_error = params_.target_altitude - tof_altitude;

                if (tof_altitude >= params_.target_altitude - 0.02f) {  // Within 2cm of target
                    // Reached target, start descending
                    // 目標到達、降下開始
                    ESP_LOGI(TAG, "JUMP: Reached %.2f m (ToF), starting descent", tof_altitude);
                    phase_ = ExecutionPhase::DESCENDING;
                } else {
                    // Proportional control: duty = hover + Kp * error
                    // 比例制御: duty = ホバリング推力 + Kp * 誤差
                    float throttle = HOVER_THROTTLE + (KP * altitude_error);
                    throttle = constrain(throttle, 0.50f, 0.90f);

                    sendControlInput(throttle, 0.0f, 0.0f, 0.0f);

                    // Debug log every 100 cycles (~250ms @ 400Hz)
                    static int log_counter = 0;
                    if (++log_counter >= 100) {
                        ESP_LOGI(TAG, "JUMP CLIMBING: tof=%.3f m, target=%.2f m, error=%.3f m, throttle=%.2f",
                                 tof_altitude, params_.target_altitude, altitude_error, throttle);
                        log_counter = 0;
                    }
                }
            }
            break;

        case ExecutionPhase::HOVERING:
            // Not used in JUMP command (直接DESCENDINGへ遷移)
            // Jumpコマンドでは使用しない
            phase_ = ExecutionPhase::DESCENDING;
            break;

        case ExecutionPhase::DESCENDING:
            // Free fall until landed
            // 自由落下で着陸まで
            if (tof_altitude < 0.05f) {  // Below 5cm = landed
                ESP_LOGI(TAG, "JUMP: Landed (ToF: %.2f m), command complete", tof_altitude);
                phase_ = ExecutionPhase::DONE;
                sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);  // Stop motors
            } else {
                // Free fall with zero throttle
                // ゼロスロットルで自由落下
                sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);
            }
            break;

        case ExecutionPhase::DONE:
            // Command complete, reset to IDLE
            // コマンド完了、IDLE状態にリセット
            ESP_LOGI(TAG, "JUMP: Command complete, returning to IDLE");
            state_ = FlightCommandState::IDLE;
            current_command_ = FlightCommandType::NONE;
            phase_ = ExecutionPhase::INIT;
            break;

        default:
            break;
    }
}

void FlightCommandService::updateTakeoffCommand(float dt, float current_altitude) {
    // TAKEOFF: INIT → CLIMBING → DONE
    switch (phase_) {
        case ExecutionPhase::INIT:
            ESP_LOGI(TAG, "TAKEOFF: Starting climb to %.2f m", params_.target_altitude);
            phase_ = ExecutionPhase::CLIMBING;
            break;

        case ExecutionPhase::CLIMBING:
            {
                float altitude_error = params_.target_altitude - current_altitude;
                if (altitude_error < 0.05f) {
                    ESP_LOGI(TAG, "TAKEOFF: Reached %.2f m", current_altitude);
                    phase_ = ExecutionPhase::DONE;
                } else {
                    float throttle = 0.75f + (altitude_error * 0.6f);
                    throttle = constrain(throttle, 0.70f, 0.95f);
                    sendControlInput(throttle, 0.0f, 0.0f, 0.0f);
                }
            }
            break;

        case ExecutionPhase::DONE:
            state_ = FlightCommandState::COMPLETED;
            current_command_ = FlightCommandType::NONE;
            // Keep motors running at hover throttle
            // ホバースロットルでモーター継続
            sendControlInput(0.55f, 0.0f, 0.0f, 0.0f);
            break;

        default:
            break;
    }
}

void FlightCommandService::updateLandCommand(float dt, float current_altitude) {
    // LAND: INIT → DESCENDING → DONE
    switch (phase_) {
        case ExecutionPhase::INIT:
            ESP_LOGI(TAG, "LAND: Starting descent");
            phase_ = ExecutionPhase::DESCENDING;
            break;

        case ExecutionPhase::DESCENDING:
            if (current_altitude < 0.05f) {
                ESP_LOGI(TAG, "LAND: Landed");
                phase_ = ExecutionPhase::DONE;
                sendControlInput(0.0f, 0.0f, 0.0f, 0.0f);
            } else {
                // Gentle descent with proportional throttle
                // 穏やかなスロットルで降下継続
                float throttle = 0.35f + (current_altitude * 0.05f);
                throttle = constrain(throttle, 0.25f, 0.45f);
                sendControlInput(throttle, 0.0f, 0.0f, 0.0f);
            }
            break;

        case ExecutionPhase::DONE:
            // Command complete, reset to IDLE
            // コマンド完了、IDLE状態にリセット
            ESP_LOGI(TAG, "LAND: Command complete, returning to IDLE");
            state_ = FlightCommandState::IDLE;
            current_command_ = FlightCommandType::NONE;
            phase_ = ExecutionPhase::INIT;
            break;

        default:
            break;
    }
}

void FlightCommandService::updateHoverCommand(float dt, float current_altitude) {
    // HOVER: INIT → HOVERING → DONE
    switch (phase_) {
        case ExecutionPhase::INIT:
            ESP_LOGI(TAG, "HOVER: Hovering at %.2f m for %.1f s",
                     params_.target_altitude, params_.duration_s);
            phase_ = ExecutionPhase::HOVERING;
            hover_timer_ = 0.0f;
            break;

        case ExecutionPhase::HOVERING:
            hover_timer_ += dt;
            if (hover_timer_ >= params_.duration_s) {
                ESP_LOGI(TAG, "HOVER: Duration complete");
                phase_ = ExecutionPhase::DONE;
            } else {
                float altitude_error = params_.target_altitude - current_altitude;
                float throttle = 0.55f + (altitude_error * 0.6f);
                throttle = constrain(throttle, 0.50f, 0.75f);
                sendControlInput(throttle, 0.0f, 0.0f, 0.0f);
            }
            break;

        case ExecutionPhase::DONE:
            state_ = FlightCommandState::COMPLETED;
            current_command_ = FlightCommandType::NONE;
            sendControlInput(0.55f, 0.0f, 0.0f, 0.0f);  // Keep hovering
            break;

        default:
            break;
    }
}

void FlightCommandService::updateMoveVerticalCommand(float dt, float current_altitude) {
    // MOVE_VERTICAL: INIT → CLIMBING or DESCENDING → DONE
    // 垂直移動: INIT → 上昇 or 降下 → 完了
    //
    // target_altitude is absolute altitude set by CLI handler (current ± distance)
    // target_altitude は CLI ハンドラで設定された絶対高度（現在値 ± 移動距離）
    switch (phase_) {
        case ExecutionPhase::INIT:
            {
                float diff = params_.target_altitude - current_altitude;
                if (diff > 0.0f) {
                    ESP_LOGI(TAG, "MOVE_VERTICAL: Climbing to %.2f m (current: %.2f m)",
                             params_.target_altitude, current_altitude);
                    phase_ = ExecutionPhase::CLIMBING;
                } else {
                    ESP_LOGI(TAG, "MOVE_VERTICAL: Descending to %.2f m (current: %.2f m)",
                             params_.target_altitude, current_altitude);
                    phase_ = ExecutionPhase::DESCENDING;
                }
            }
            break;

        case ExecutionPhase::CLIMBING:
            {
                float altitude_error = params_.target_altitude - current_altitude;
                if (altitude_error < 0.05f) {
                    ESP_LOGI(TAG, "MOVE_VERTICAL: Reached %.2f m", current_altitude);
                    phase_ = ExecutionPhase::DONE;
                } else {
                    // Proportional control: hover_throttle + Kp * error
                    // 比例制御: ホバースロットル + Kp * 誤差
                    constexpr float HOVER_THROTTLE = 0.55f;
                    constexpr float KP = 0.6f;
                    float throttle = HOVER_THROTTLE + (KP * altitude_error);
                    throttle = constrain(throttle, 0.50f, 0.90f);
                    sendControlInput(throttle, 0.0f, 0.0f, 0.0f);
                }
            }
            break;

        case ExecutionPhase::DESCENDING:
            {
                float altitude_error = current_altitude - params_.target_altitude;
                if (altitude_error < 0.05f) {
                    ESP_LOGI(TAG, "MOVE_VERTICAL: Reached %.2f m", current_altitude);
                    phase_ = ExecutionPhase::DONE;
                } else {
                    // Reduced throttle for descent, proportional to error
                    // 降下用の減少スロットル、誤差に比例
                    constexpr float HOVER_THROTTLE = 0.55f;
                    constexpr float KP = 0.4f;
                    float throttle = HOVER_THROTTLE - (KP * altitude_error);
                    throttle = constrain(throttle, 0.25f, 0.55f);
                    sendControlInput(throttle, 0.0f, 0.0f, 0.0f);
                }
            }
            break;

        case ExecutionPhase::DONE:
            // Keep hovering at target altitude
            // 目標高度でホバリング維持
            state_ = FlightCommandState::COMPLETED;
            current_command_ = FlightCommandType::NONE;
            sendControlInput(0.55f, 0.0f, 0.0f, 0.0f);
            break;

        default:
            break;
    }
}

void FlightCommandService::updateRotateYawCommand(float dt, float current_altitude) {
    // ROTATE_YAW: INIT → ROTATING → DONE
    // ヨー回転: INIT → 回転中 → 完了
    //
    // target_yaw_deg is absolute yaw set by CLI handler (current ± angle)
    // target_yaw_deg は CLI ハンドラで設定された絶対ヨー角（現在値 ± 回転角）
    switch (phase_) {
        case ExecutionPhase::INIT:
            ESP_LOGI(TAG, "ROTATE_YAW: Rotating to %.1f deg (current: %.1f deg)",
                     params_.target_yaw_deg, getCurrentYaw() * 180.0f / 3.14159265f);
            phase_ = ExecutionPhase::ROTATING;
            break;

        case ExecutionPhase::ROTATING:
            {
                float current_yaw_deg = getCurrentYaw() * 180.0f / 3.14159265f;
                float angle_error = normalizeAngleDeg(params_.target_yaw_deg - current_yaw_deg);

                if (fabsf(angle_error) < 5.0f) {
                    // Within 5 degrees of target
                    // 目標の5度以内
                    ESP_LOGI(TAG, "ROTATE_YAW: Reached %.1f deg (error: %.1f deg)",
                             current_yaw_deg, angle_error);
                    phase_ = ExecutionPhase::DONE;
                } else {
                    // Proportional yaw control while maintaining hover
                    // ホバー維持しつつ比例ヨー制御
                    constexpr float HOVER_THROTTLE = 0.55f;
                    constexpr float YAW_KP = 0.005f;  // deg → normalized yaw input
                    float yaw_input = YAW_KP * angle_error;
                    yaw_input = constrain(yaw_input, -0.5f, 0.5f);

                    // Maintain altitude with proportional control
                    // 高度を比例制御で維持
                    float alt_error = params_.target_altitude - current_altitude;
                    float throttle = HOVER_THROTTLE + (alt_error * 0.6f);
                    throttle = constrain(throttle, 0.50f, 0.75f);

                    sendControlInput(throttle, 0.0f, 0.0f, yaw_input);

                    // Debug log every 100 cycles (~250ms @ 400Hz)
                    static int yaw_log_counter = 0;
                    if (++yaw_log_counter >= 100) {
                        ESP_LOGI(TAG, "ROTATE_YAW: current=%.1f deg, target=%.1f deg, error=%.1f deg, yaw_in=%.3f",
                                 current_yaw_deg, params_.target_yaw_deg, angle_error, yaw_input);
                        yaw_log_counter = 0;
                    }
                }
            }
            break;

        case ExecutionPhase::DONE:
            // Keep hovering at current position
            // 現在位置でホバリング維持
            state_ = FlightCommandState::COMPLETED;
            current_command_ = FlightCommandType::NONE;
            sendControlInput(0.55f, 0.0f, 0.0f, 0.0f);
            break;

        default:
            break;
    }
}

void FlightCommandService::updateMoveHorizontalCommand(float dt, float current_altitude) {
    // MOVE_HORIZONTAL: INIT → MOVING → DONE
    // 水平移動: INIT → 移動中 → 完了
    //
    // Uses PositionController setpoint to move to target position.
    // PositionController のセットポイントを直接設定して目標位置へ移動。
    switch (phase_) {
        case ExecutionPhase::INIT:
            {
                // Enable POSITION_HOLD mode and set target position
                // POSITION_HOLD モードを有効化し、目標位置を設定
                auto state = globals::g_fusion.getState();
                float current_x = state.position.x;
                float current_y = state.position.y;

                // Capture current position, then override setpoint with target
                // 現在位置をキャプチャ後、セットポイントを目標位置に上書き
                g_position_controller.capturePosition(current_x, current_y);
                g_position_controller.pos_setpoint_x = params_.target_pos_x;
                g_position_controller.pos_setpoint_y = params_.target_pos_y;

                ESP_LOGI(TAG, "MOVE_HORIZONTAL: Moving from (%.2f, %.2f) to (%.2f, %.2f) m",
                         current_x, current_y, params_.target_pos_x, params_.target_pos_y);

                phase_ = ExecutionPhase::MOVING;
            }
            break;

        case ExecutionPhase::MOVING:
            {
                // Send control input with POSITION_HOLD flag, zero sticks
                // POSITION_HOLD フラグ付きで制御入力送信、スティックはゼロ
                constexpr float HOVER_THROTTLE = 0.55f;

                // Maintain altitude with proportional control
                // 高度を比例制御で維持
                float alt_error = params_.target_altitude - current_altitude;
                float throttle = HOVER_THROTTLE + (alt_error * 0.6f);
                throttle = constrain(throttle, 0.50f, 0.75f);

                sendControlInput(throttle, 0.0f, 0.0f, 0.0f, CTRL_FLAG_POS_MODE);

                // Check if position reached
                // 位置到達チェック
                auto state = globals::g_fusion.getState();
                float error_x = params_.target_pos_x - state.position.x;
                float error_y = params_.target_pos_y - state.position.y;
                float error_dist = sqrtf(error_x * error_x + error_y * error_y);

                if (error_dist < 0.05f) {
                    // Reached target position (within 5cm)
                    // 目標位置に到達（5cm以内）
                    ESP_LOGI(TAG, "MOVE_HORIZONTAL: Reached target (error: %.3f m)", error_dist);
                    phase_ = ExecutionPhase::DONE;
                }

                // Debug log every 100 cycles (~250ms @ 400Hz)
                static int move_log_counter = 0;
                if (++move_log_counter >= 100) {
                    ESP_LOGI(TAG, "MOVE_HORIZONTAL: pos=(%.2f, %.2f), target=(%.2f, %.2f), error=%.3f m",
                             state.position.x, state.position.y,
                             params_.target_pos_x, params_.target_pos_y, error_dist);
                    move_log_counter = 0;
                }
            }
            break;

        case ExecutionPhase::DONE:
            // Keep hovering at target position with POSITION_HOLD
            // POSITION_HOLD で目標位置にホバリング維持
            state_ = FlightCommandState::COMPLETED;
            current_command_ = FlightCommandType::NONE;
            sendControlInput(0.55f, 0.0f, 0.0f, 0.0f, CTRL_FLAG_POS_MODE);
            break;

        default:
            break;
    }
}

// Get current yaw angle estimate [rad]
// 現在のヨー角推定値を取得 [rad]
float FlightCommandService::getCurrentYaw() const {
    auto state = globals::g_fusion.getState();
    return state.yaw;
}

// Constrain helper function
// 制約ヘルパー関数
float FlightCommandService::constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Normalize angle to [-180, 180] degrees
// 角度を [-180, 180] 度に正規化
float FlightCommandService::normalizeAngleDeg(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

} // namespace stampfly
