/**
 * @file actuator.cpp
 * @brief Mixer and motor output implementation
 *        ミキサー＋モーター出力の実装
 *
 * @design architecture.md §5 — Actuator subsystem                       [--]
 * @design detailed_design.md §5 — X-quad mixer                          [--]
 */

#include "actuator.hpp"
#include "topics.hpp"
#include "esp_log.h"

// TODO: Include HAL motor driver header
// TODO: HALモータードライバヘッダをインクルード
// #include "motor.hpp"

static const char* TAG = "actuator";

namespace sf {

// -----------------------------------------------------------------------------
// init — initialize actuator subsystem
// 初期化 — アクチュエータサブシステムを初期化
// -----------------------------------------------------------------------------
void Actuator::init()
{
    ESP_LOGI(TAG, "Actuator initialized");
    // TODO: Initialize motor HAL
    // TODO: モーターHALを初期化
}

// -----------------------------------------------------------------------------
// update — read control output, mix, publish and apply motor duties
// 更新 — 制御出力を読み取り、ミキシングし、モーターdutyを発行・適用
// -----------------------------------------------------------------------------
void Actuator::update()
{
    // Subscribe to control output topic
    // 制御出力トピックを購読
    ControlOutput cmd = control_output.latest();

    // Apply X-quad mixer
    // X-quadミキサーを適用
    MotorOutput out = mix(cmd);

    // Clamp to valid range
    // 有効範囲にクランプ
    clampDuties(out);

    // Publish motor output to topic
    // モーター出力をトピックに発行
    actuator_motor.publish(out);

    // TODO: Apply to physical motors via HAL
    // TODO: HAL経由で物理モーターに適用
    // motor_hal::set_duty(out.duty[0], out.duty[1], out.duty[2], out.duty[3]);
}

// -----------------------------------------------------------------------------
// mix — X-quad mixer: thrust + torque → 4 motor duties
// ミキシング — X-quadミキサー: 推力＋トルク → 4モーターduty
// -----------------------------------------------------------------------------
MotorOutput Actuator::mix(const ControlOutput& cmd) const
{
    // Extract thrust and torque components
    // 推力とトルク成分を取り出す
    const float T = cmd.thrust;
    const float R = cmd.torque[0];   // Roll torque  / ロールトルク
    const float P = cmd.torque[1];   // Pitch torque / ピッチトルク
    const float Y = cmd.torque[2];   // Yaw torque   / ヨートルク

    MotorOutput out = {};
    out.duty[0] = T - R + P + Y;   // M1: front-right (CCW)
    out.duty[1] = T - R - P - Y;   // M2: rear-right  (CW)
    out.duty[2] = T + R - P + Y;   // M3: rear-left   (CCW)
    out.duty[3] = T + R + P - Y;   // M4: front-left  (CW)
    out.timestamp = cmd.timestamp;
    return out;
}

// -----------------------------------------------------------------------------
// clampDuties — clamp motor duties to [0..1]
// dutyクランプ — モーターdutyを[0..1]にクランプ
// -----------------------------------------------------------------------------
void Actuator::clampDuties(MotorOutput& out) const
{
    for (int i = 0; i < 4; ++i) {
        if (out.duty[i] < 0.0f) out.duty[i] = 0.0f;
        if (out.duty[i] > 1.0f) out.duty[i] = 1.0f;
    }
}

}  // namespace sf
