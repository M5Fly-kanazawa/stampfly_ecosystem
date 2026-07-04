// High-Level Flight Command Service
// 高レベル飛行コマンドサービス
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "system_state.hpp"  // For FlightCommandType

namespace stampfly {

// FlightCommandType is now defined in system_state.hpp to avoid circular dependency
// FlightCommandTypeは循環依存回避のためsystem_state.hppで定義

// Command state
// コマンド状態
enum class FlightCommandState {
    IDLE,         // Not executing any command / コマンド実行中でない
    RUNNING,      // Executing command / コマンド実行中
    COMPLETED,    // Completed successfully / 完了
    FAILED,       // Failed / 失敗
};

// Command parameters
// コマンドパラメータ
struct FlightCommandParams {
    float target_altitude;    // Target altitude [m] / 目標高度 [m]
    float duration_s;         // Duration [s] (used for HOVER/JUMP) / 持続時間 [s]（HOVERやJUMPで使用）
    float climb_rate;         // Climb rate [m/s] / 上昇速度 [m/s]
    float descent_rate;       // Descent rate [m/s] / 降下速度 [m/s]
    float target_yaw_deg;     // Target yaw angle [deg] (for ROTATE_YAW) / 目標ヨー角 [deg]
    float target_pos_x;       // Target position NED X [m] (for MOVE_HORIZONTAL) / 目標位置 NED X [m]
    float target_pos_y;       // Target position NED Y [m] (for MOVE_HORIZONTAL) / 目標位置 NED Y [m]
};

// Flight Command Service
// フライトコマンドサービス
class FlightCommandService {
public:
    static FlightCommandService& getInstance();

    // Initialize the service
    // サービスの初期化
    esp_err_t init();

    // Execute command (non-blocking)
    // コマンド実行（非ブロッキング）
    bool executeCommand(FlightCommandType type, const FlightCommandParams& params);

    // Get current state
    // 状態確認
    FlightCommandState getState() const { return state_; }
    bool isRunning() const { return state_ == FlightCommandState::RUNNING; }
    FlightCommandType getCurrentCommand() const { return current_command_; }

    // Cancel current command
    // 現在のコマンドをキャンセル
    void cancel();

    // Update command execution (called from control task at 400Hz)
    // 定期更新（制御タスクから400Hzで呼ばれる）
    void update(float dt);

    // Check if command can be executed
    // 実行可能かチェック（着陸済み、キャリブレーション完了等）
    bool canExecute() const;

    // Get elapsed time for current command
    // 現在のコマンドの経過時間を取得
    float getElapsedTime() const { return elapsed_time_; }

private:
    FlightCommandService() = default;

    FlightCommandState state_ = FlightCommandState::IDLE;
    FlightCommandType current_command_ = FlightCommandType::NONE;
    FlightCommandParams params_{};

    // Internal execution phase
    // 内部状態（コマンド実行中の段階）
    enum class ExecutionPhase {
        INIT,
        CLIMBING,
        HOVERING,
        DESCENDING,
        ROTATING,     // Yaw rotation in progress / ヨー回転中
        MOVING,       // Horizontal movement in progress / 水平移動中
        DONE,
    };
    ExecutionPhase phase_ = ExecutionPhase::INIT;

    float elapsed_time_ = 0.0f;
    float hover_timer_ = 0.0f;

    // CommandQueue integration
    // CommandQueue統合
    int current_command_id_ = -1;  ///< Current command ID from queue / キューからの現在のコマンドID

    // Send control input to ControlArbiter
    // 制御出力を ControlArbiter に送る
    void sendControlInput(float throttle, float roll, float pitch, float yaw, uint8_t flags = 0);

    // Get current altitude estimate
    // 現在の高度推定値を取得
    float getCurrentAltitude() const;

    // Command-specific update functions
    // コマンド固有の更新関数
    void updateJumpCommand(float dt, float current_altitude);
    void updateTakeoffCommand(float dt, float current_altitude);
    void updateLandCommand(float dt, float current_altitude);
    void updateHoverCommand(float dt, float current_altitude);
    void updateMoveVerticalCommand(float dt, float current_altitude);
    void updateRotateYawCommand(float dt, float current_altitude);
    void updateMoveHorizontalCommand(float dt, float current_altitude);

    // Get current yaw angle estimate [rad]
    // 現在のヨー角推定値を取得 [rad]
    float getCurrentYaw() const;

    // Helper function
    // ヘルパー関数
    static float constrain(float value, float min, float max);

    // Normalize angle to [-180, 180] degrees
    // 角度を [-180, 180] 度に正規化
    static float normalizeAngleDeg(float angle);
};

} // namespace stampfly
