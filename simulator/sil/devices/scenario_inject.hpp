/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario_inject.hpp
 * @brief Single shared builder for the on-air ControlPacket + the RC injector.
 *        電波上 ControlPacket の唯一の共有ビルダ＋RC インジェクタ。
 *
 * ONE place builds the 14-byte ControlPacket and delivers it through the ESP-NOW
 * hub, so the scripted scenario driver and the legacy virtual pilot share exactly
 * the same on-air bytes (no divergence). The firmware decodes UNMODIFIED.
 *
 * IMPORTANT: the firmware treats the stick fields as raw 12-bit ADC counts,
 * 0..4095 with 2048 = centre (ControlArbiter::normalizeThrottle/normalizeAxis use
 * ADC_CENTER=2048). The struct comment in controller_comm.hpp says 0..1000 but
 * that is STALE — send ADC-scale or throttle clamps to 0 and sticks deflect.
 *
 * 1箇所で 14 バイト ControlPacket を組み ESP-NOW ハブへ配信。台本ドライバと従来の仮想
 * パイロットが同一の電波バイト列を共有する。スティックは raw 12bit ADC（0..4095, 中央
 * 2048）。controller_comm.hpp の 0..1000 コメントは古い。
 */

#pragma once

#include <cstdint>

namespace sil {

// Stick neutral (ControlArbiter ADC_CENTER) / スティック中央。
constexpr uint16_t kAdcCentre = 2048;

// CTRL_FLAG_ARM bit / アームフラグ。
constexpr uint8_t kFlagArm = 0x01;

// CTRL_FLAG_MODE bit (bit2) — on vehicle_new this selects ACRO (rate) mode
// (sf_comm decodes it to PilotRequest.acro). Must match the protocol SSOT /
// firmware/vehicle_new sf_comm kFlagMode = 0x04.
// CTRL_FLAG_MODE ビット(bit2) — vehicle_new では ACRO（角速度）モードを選択
// （sf_comm が PilotRequest.acro にデコード）。SSOT / sf_comm kFlagMode=0x04 と一致。
constexpr uint8_t kFlagMode = 0x04;

// CTRL_FLAG_ALT_MODE bit — selects ALTITUDE_HOLD on the firmware side.
// Must match controller_comm.hpp:38 (CTRL_FLAG_ALT_MODE = 0x08).
// CTRL_FLAG_ALT_MODE ビット — 本体側で ALTITUDE_HOLD を選択。controller_comm.hpp:38 と一致。
constexpr uint8_t kFlagAltMode = 0x08;

// CTRL_FLAG_POS_MODE bit (bit4) — selects POSITION_HOLD on the firmware side.
// Must match sf_comm kFlagPosMode (0x10). POS_HOLD implies ALT_HOLD (mode hierarchy).
// CTRL_FLAG_POS_MODE ビット(bit4) — 本体側で POSITION_HOLD を選択。sf_comm kFlagPosMode=0x10
// と一致。POS_HOLD は ALT_HOLD を含む（モード階層）。
constexpr uint8_t kFlagPosMode = 0x10;

// Build the 14-byte ControlPacket into `out` (must hold >= 14 bytes).
// Layout: [0..2]=drone_mac, [3..4]=throttle, [5..6]=roll, [7..8]=pitch,
// [9..10]=yaw, [11]=flags, [12]=reserved, [13]=checksum (sum of bytes 0..12).
// 14 バイト ControlPacket を `out` に構築。
void build_control_packet(uint8_t* out, uint16_t throttle, uint16_t roll,
                          uint16_t pitch, uint16_t yaw, uint8_t flags);

// Build one ControlPacket and deliver it into the firmware via the ESP-NOW hub
// (which records it). Stick values are ADC-scale (0..4095, centre 2048).
// ControlPacket を1つ組んで ESP-NOW ハブ経由で本体へ配信（ハブが記録）。
void inject_rc(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
               uint8_t flags);

// Seed the firmware's pairing NVS so the emulated vehicle boots PAIRED to the
// injector's transmitter MAC (kPilotMac). Real hardware boots unpaired and auto-
// enters Pairing, which would block ARM; the flight scenarios inject RC without a
// pairing handshake, so we pre-bind them here (a vehicle that "was paired before").
// Call BEFORE app_main() (so comm::init() loads it). No-op when the SIL_EMU_UNPAIRED
// environment variable is set — used by the pairing scenario to test the handshake.
// 本体のペアリング NVS を seed し、エミュ機体がインジェクタの送信機 MAC（kPilotMac）に
// ペア済みで起動するようにする。実機は未ペア起動→自動 Pairing で ARM が阻まれるが、飛行
// シナリオはペアリングなしで RC を注入するため、ここで事前バインドする（「以前ペアした」機体）。
// app_main() の前に呼ぶこと（comm::init() が読むため）。環境変数 SIL_EMU_UNPAIRED が設定
// されていれば no-op — ペアリングシナリオがハンドシェイクを試験するために使う。
void seed_pairing_nvs();

}  // namespace sil
