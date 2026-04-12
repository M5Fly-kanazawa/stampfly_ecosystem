/**
 * @file command.cpp
 * @brief Command processor implementation
 *        コマンド処理の実装
 *
 * @design architecture.md §6 — Command processing pipeline             [--]
 * @design detailed_design.md §4 — Input normalization                   [--]
 */

#include "command.hpp"
#include "topics.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>

static const char* TAG = "command";

namespace sf {

// ADC range constant
// ADCレンジ定数
static constexpr uint16_t ADC_MAX = 4095;
static constexpr float ADC_MID = 2047.5f;

// -----------------------------------------------------------------------------
// init — initialize command processor
// 初期化 — コマンドプロセッサを初期化
// -----------------------------------------------------------------------------
void CommandProcessor::init()
{
    ESP_LOGI(TAG, "CommandProcessor initialized (deadband=%.3f)", deadband_);
}

// -----------------------------------------------------------------------------
// processRawInput — normalize, deadband, publish
// 生入力処理 — 正規化、デッドバンド適用、発行
// -----------------------------------------------------------------------------
void CommandProcessor::processRawInput(uint16_t raw_throttle, uint16_t raw_roll,
                                       uint16_t raw_pitch, uint16_t raw_yaw,
                                       CommandSource source)
{
    // Normalize inputs
    // 入力を正規化
    float throttle = normalizeThrottle(raw_throttle);
    float roll     = applyDeadband(normalize(raw_roll));
    float pitch    = applyDeadband(normalize(raw_pitch));
    float yaw      = applyDeadband(normalize(raw_yaw));

    // Publish to topic
    // トピックに発行
    publishSetpoint(throttle, roll, pitch, yaw, source);
}

// -----------------------------------------------------------------------------
// normalize — [0..4095] → [-1..1]
// 正規化 — [0..4095] → [-1..1]
// -----------------------------------------------------------------------------
float CommandProcessor::normalize(uint16_t raw) const
{
    return (static_cast<float>(raw) - ADC_MID) / ADC_MID;
}

// -----------------------------------------------------------------------------
// normalizeThrottle — [0..4095] → [0..1]
// スロットル正規化 — [0..4095] → [0..1]
// -----------------------------------------------------------------------------
float CommandProcessor::normalizeThrottle(uint16_t raw) const
{
    return static_cast<float>(raw) / static_cast<float>(ADC_MAX);
}

// -----------------------------------------------------------------------------
// applyDeadband — zero out values within +-deadband
// デッドバンド適用 — +-deadband内の値をゼロにする
// -----------------------------------------------------------------------------
float CommandProcessor::applyDeadband(float value) const
{
    if (std::fabs(value) < deadband_) {
        return 0.0f;
    }
    // Rescale remaining range to [0..1] or [-1..0]
    // 残りの範囲を[0..1]または[-1..0]にリスケール
    float sign = (value > 0.0f) ? 1.0f : -1.0f;
    return sign * (std::fabs(value) - deadband_) / (1.0f - deadband_);
}

// -----------------------------------------------------------------------------
// publishSetpoint — publish to command_setpoint topic
// セットポイント発行 — command_setpointトピックに発行
// -----------------------------------------------------------------------------
void CommandProcessor::publishSetpoint(float throttle, float roll,
                                        float pitch, float yaw,
                                        CommandSource source)
{
    CommandSetpoint sp = {};
    sp.throttle  = throttle;
    sp.roll      = roll;
    sp.pitch     = pitch;
    sp.yaw       = yaw;
    sp.source    = static_cast<uint8_t>(source);
    sp.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    command_setpoint.publish(sp);
}

}  // namespace sf
