/**
 * @file pid_controller.cpp
 * @brief PID controller stub implementation
 *        PIDコントローラスタブ実装
 *
 * Minimal stub for pipeline integration testing.
 * Full cascade PID will be ported from vehicle firmware.
 *
 * パイプライン結合テスト用の最小スタブ。
 * 完全なカスケードPIDはvehicleファームから移植予定。
 *
 * @design detailed_design.md §4 — IController                        [--]
 */

#include "pid_controller.hpp"
#include "esp_log.h"

static const char* TAG = "PID";

namespace sf {

void PidController::init()
{
    reset();
    ESP_LOGI(TAG, "PID controller initialized (stub)");
}

ControlOutput PidController::compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt)
{
    ControlOutput output = {};
    output.timestamp = state.timestamp;

    // TODO: Full cascade PID computation
    // TODO: 完全なカスケードPID演算
    // Rate control → Attitude control → Altitude control → Position control

    return output;
}

void PidController::reset()
{
    ESP_LOGI(TAG, "PID controller reset");
    // TODO: Reset all PID integrators and filters
    // TODO: 全PID積分器とフィルタをリセット
}

void PidController::onModeChange(FlightMode new_mode)
{
    ESP_LOGI(TAG, "Mode change: %s → %s",
             flightModeName(current_mode_), flightModeName(new_mode));
    current_mode_ = new_mode;
    // TODO: Reconfigure cascade loops for new mode
    // TODO: 新モード用にカスケードループを再構成
}

}  // namespace sf
