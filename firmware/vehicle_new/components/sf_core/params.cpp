/**
 * @file params.cpp
 * @brief Parameter system implementation
 *        パラメータシステム実装
 *
 * @design detailed_design.md §6 — Parameter system                    [--]
 * @design requirements.md §3 — Parameter management                   [--]
 */

#include "topics.hpp"

namespace sf {

// =============================================================================
// Topic instances (defined here, declared extern in topics.hpp)
// トピックインスタンス（ここで定義、topics.hppでextern宣言）
// =============================================================================

Topic<ImuData,         RingBuffer, 8>  sensor_imu;
Topic<TofData,         Queue, 2>       sensor_tof;
Topic<FlowData,        Queue, 2>       sensor_flow;
Topic<MagData,         Queue, 2>       sensor_mag;
Topic<BaroData,        Queue, 2>       sensor_baro;
Topic<PowerData,       Latest, 1>      sensor_power;
Topic<StateEstimate,   Latest, 1>      estimate_state;
Topic<CommandSetpoint, Latest, 1>      command_setpoint;
Topic<ControlOutput,   Latest, 1>      control_output;
Topic<MotorOutput,     Latest, 1>      actuator_motor;
Topic<SystemMode,      Latest, 1>      system_mode;
Topic<SystemAlert,     Queue, 4>       system_alert;

/// Initialize all topics
/// 全トピックを初期化する
void topics_init()
{
    sensor_imu.init();
    sensor_tof.init();
    sensor_flow.init();
    sensor_mag.init();
    sensor_baro.init();
    sensor_power.init();
    estimate_state.init();
    command_setpoint.init();
    control_output.init();
    actuator_motor.init();
    system_mode.init();
    system_alert.init();
}

// =============================================================================
// Parameter system implementation
// パラメータシステム実装
//
// TODO: Implement parameter registry, NVS persistence, WiFi/CLI access
// TODO: パラメータレジストリ、NVS永続化、WiFi/CLIアクセスを実装
// =============================================================================

}  // namespace sf
