/**
 * @file topics.hpp
 * @brief All topic definitions (single location)
 *        全トピック定義（1箇所）
 *
 * Adding a new topic requires only one line here.
 * トピック追加はここに1行追加するだけ。
 *
 * @design architecture.md §3 — Topic list (12 topics)                 [--]
 * @design detailed_design.md §2 — Topic definitions                   [--]
 */

#pragma once

#include "topic.hpp"
#include "data_types.hpp"

namespace sf {

/// Initialize all topics (call once at startup)
/// 全トピックを初期化する（起動時に1回呼ぶ）
void topics_init();

// =============================================================================
// Sensor Topics
// センサトピック
// =============================================================================

extern Topic<ImuData,         RingBuffer, 8>  sensor_imu;
extern Topic<TofData,         Queue, 2>       sensor_tof;
extern Topic<FlowData,        Queue, 2>       sensor_flow;
extern Topic<MagData,         Queue, 2>       sensor_mag;
extern Topic<BaroData,        Queue, 2>       sensor_baro;
extern Topic<PowerData,       Latest, 1>      sensor_power;

// =============================================================================
// Estimation Topics
// 推定トピック
// =============================================================================

extern Topic<StateEstimate,   Latest, 1>      estimate_state;

// =============================================================================
// Command Topics
// コマンドトピック
// =============================================================================

extern Topic<CommandSetpoint, Latest, 1>      command_setpoint;

// =============================================================================
// Control Topics
// 制御トピック
// =============================================================================

extern Topic<ControlOutput,   Latest, 1>      control_output;

// =============================================================================
// Actuation Topics
// アクチュエーショントピック
// =============================================================================

extern Topic<MotorOutput,     Latest, 1>      actuator_motor;

// =============================================================================
// System Topics
// システムトピック
// =============================================================================

extern Topic<SystemMode,      Latest, 1>      system_mode;
extern Topic<SystemAlert,     Queue, 4>       system_alert;

}  // namespace sf
