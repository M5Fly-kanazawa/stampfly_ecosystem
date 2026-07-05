/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file command.cpp
 * @brief Command processor implementation
 *        コマンド処理の実装
 *
 * @design architecture.md §6 — Command processing pipeline             [OK]
 * @design detailed_design.md §4 — Input normalization                   [OK]
 */

#include "command.hpp"
#include "topics.hpp"
#include "esp_log.h"

#include <cmath>

static const char* TAG = "command";

namespace sf {

// -----------------------------------------------------------------------------
// ADC / stick constants — the single source of the stick→command mapping.
// Spring-centred 12-bit ADC: 2048 = centre/zero, half-span 2048. This matches
// the FIXED transmitter (firmware/controller) and the flight-proven legacy
// vehicle firmware. The earlier dead CommandProcessor used 2047.5 and a
// throttle/4095 mapping, both wrong (off-centre, and throttle would over-drive
// 1.5x); unified here to the on-air contract.
// ADC/スティック定数 — スティック→指令 変換の唯一の出どころ。バネ中央 12bit ADC:
// 2048=中央/ゼロ、半幅 2048。固定送信機(firmware/controller)・飛行実績のある旧 vehicle
// と一致。以前の死蔵 CommandProcessor は 2047.5 と throttle/4095 を使っており双方とも
// 誤り（中央ずれ、スロットルが 1.5 倍過大）だった。on-air 規約に統一する。
// -----------------------------------------------------------------------------
static constexpr float kAdcCenter   = 2048.0f;
static constexpr float kAdcHalfSpan = 2048.0f;

// ControlPacket.flags bit masks (protocol SSOT / controller_comm). sf_comm
// delivers the raw flags byte; this Service component interprets the bits.
// ControlPacket.flags のビットマスク（プロトコル SSOT / controller_comm）。sf_comm は
// 生の flags バイトを渡し、本 Service コンポーネントがビットを解釈する。
static constexpr uint8_t kFlagArm     = 0x01;  // bit0: ARM
static constexpr uint8_t kFlagMode    = 0x04;  // bit2: ACRO (rate) mode
static constexpr uint8_t kFlagAltMode = 0x08;  // bit3: ALTITUDE_HOLD
static constexpr uint8_t kFlagPosMode = 0x10;  // bit4: POSITION_HOLD

// -----------------------------------------------------------------------------
// init — initialize command processor
// 初期化 — コマンドプロセッサを初期化
// -----------------------------------------------------------------------------
void CommandProcessor::init()
{
    ESP_LOGI(TAG, "CommandProcessor initialized (deadband=%.3f)", deadband_);
}

// -----------------------------------------------------------------------------
// process — normalize sticks, deadband, decode switches, publish both topics
// 処理 — スティック正規化、デッドバンド、スイッチデコード、両トピック発行
// -----------------------------------------------------------------------------
void CommandProcessor::process(const RawControlInput& raw)
{
    publishSetpoint(raw);
    publishPilotRequest(raw);
}

// -----------------------------------------------------------------------------
// normalizeAxis — spring-centred [0..4095] → [-1..1]
// normalizeAxis — バネ中央 [0..4095] → [-1..1]
// -----------------------------------------------------------------------------
float CommandProcessor::normalizeAxis(uint16_t raw) const
{
    float v = (static_cast<float>(raw) - kAdcCenter) / kAdcHalfSpan;
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

// -----------------------------------------------------------------------------
// normalizeThrottle — [0..4095] → [0..1] (lower half clipped to 0; 2048 = zero)
// normalizeThrottle — [0..4095] → [0..1]（下半分は 0 にクリップ、2048=ゼロ）
// -----------------------------------------------------------------------------
float CommandProcessor::normalizeThrottle(uint16_t raw) const
{
    float v = (static_cast<float>(raw) - kAdcCenter) / kAdcHalfSpan;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

// -----------------------------------------------------------------------------
// applyDeadband — zero out values within +-deadband, rescale the remainder
// デッドバンド適用 — +-deadband 内の値をゼロにし、残りを再スケール
// -----------------------------------------------------------------------------
float CommandProcessor::applyDeadband(float value) const
{
    if (std::fabs(value) < deadband_) {
        return 0.0f;
    }
    // Rescale remaining range to [0..1] or [-1..0] so full deflection reaches +-1.
    // 残りの範囲を [0..1] / [-1..0] に再スケールし、全振りで +-1 に届くようにする。
    float sign = (value > 0.0f) ? 1.0f : -1.0f;
    return sign * (std::fabs(value) - deadband_) / (1.0f - deadband_);
}

// -----------------------------------------------------------------------------
// publishSetpoint — normalize sticks (deadband on roll/pitch/yaw) → publish
// publishSetpoint — スティック正規化（ロール/ピッチ/ヨーにデッドバンド）→ 発行
// -----------------------------------------------------------------------------
void CommandProcessor::publishSetpoint(const RawControlInput& raw)
{
    CommandSetpoint sp = {};
    sp.throttle      = normalizeThrottle(raw.throttle);              // [0..1] STABILIZE thrust
    // Symmetric throttle axis for ALT_HOLD/POS_HOLD vertical: centre(2048)=0=hold,
    // up=climb, down=descend. No deadband here — the controller owns the centre
    // deadzone and the re-center gate (so it can see the raw centre crossing).
    // ALT_HOLD/POS_HOLD 鉛直用の対称スロットル軸: 中央(2048)=0=ホールド、上=上昇、
    // 下=降下。ここではデッドバンドを掛けない — 中央デッドゾーンと再センターゲートは
    // 制御器が所有する（中央通過を生で見られるように）。
    sp.throttle_axis = normalizeAxis(raw.throttle);                  // [-1..1] ALT_HOLD vertical
    sp.roll      = applyDeadband(normalizeAxis(raw.roll));
    sp.pitch     = applyDeadband(normalizeAxis(raw.pitch));
    sp.yaw       = applyDeadband(normalizeAxis(raw.yaw));
    sp.source    = raw.source;
    sp.timestamp = raw.timestamp;

    command_setpoint.publish(sp);
}

// -----------------------------------------------------------------------------
// publishPilotRequest — decode the discrete switch bits as FACTS → publish
// publishPilotRequest — 離散スイッチビットを「事実」としてデコード → 発行
//
// This component only reports which switches the pilot sent; the StateManager
// (sole authority) edge-detects ARM and derives the FlightMode (R5: reaches
// StateTask by topic, never a direct call).
// 本コンポーネントはどのスイッチが来たかを報告するだけ。ARM のエッジ検出と FlightMode
// の決定は StateManager（唯一の権限）が行う（R5: トピック経由・直接呼出禁止）。
// -----------------------------------------------------------------------------
void CommandProcessor::publishPilotRequest(const RawControlInput& raw)
{
    PilotRequest req = {};
    req.arm       = (raw.flags & kFlagArm)     != 0;
    req.acro      = (raw.flags & kFlagMode)    != 0;
    req.alt_hold  = (raw.flags & kFlagAltMode) != 0;
    req.pos_hold  = (raw.flags & kFlagPosMode) != 0;
    req.source    = raw.source;
    req.timestamp = raw.timestamp;

    pilot_request.publish(req);
}

}  // namespace sf
