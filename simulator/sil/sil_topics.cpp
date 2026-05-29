/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test support).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sil_topics.cpp
 * @brief Topic extern instances for host SIL/test builds
 *        ホストSIL/テストビルド用のトピックextern実体
 *
 * The firmware defines these 12 topic instances in
 * firmware/vehicle_new/components/sf_core/params.cpp, but that file pulls in
 * nvs_flash.h / nvs.h (ESP-IDF NVS), which does not exist on the host. We
 * replicate ONLY the extern instance definitions and topics_init() here —
 * declarations, not logic — so the unmodified state_manager.cpp / failsafe.cpp
 * link on a PC while Code Identity is preserved.
 *
 * ファームはこの12個のトピック実体を params.cpp で定義するが、同ファイルは
 * NVS依存(nvs_flash.h/nvs.h)でホストに存在しない。ここでは extern 実体定義と
 * topics_init() のみ（ロジックではなく宣言）を複製し、無改変の
 * state_manager.cpp / failsafe.cpp を PC でリンク可能にする。
 *
 * ⚠️ SYNC OBLIGATION / 同期義務:
 *   This block must stay byte-identical to params.cpp:35-62 (the topic
 *   definition block + topics_init). params.cpp is the source of truth — if
 *   they ever diverge, params.cpp wins and this file must be updated.
 *   本ブロックは params.cpp:35-62（トピック定義 + topics_init）と完全一致を
 *   保つこと。乖離時は params.cpp が正。
 */

#include "topics.hpp"

namespace sf {

// =============================================================================
// Topic instances (mirror of params.cpp)
// トピックインスタンス（params.cpp のミラー）
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

}  // namespace sf
