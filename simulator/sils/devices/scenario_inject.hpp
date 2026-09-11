/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
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

namespace sils {

// Stick neutral (ControlArbiter ADC_CENTER) / スティック中央。
constexpr uint16_t kAdcCentre = 2048;

// CTRL_FLAG_ARM bit / アームフラグ。
constexpr uint8_t kFlagArm = 0x01;

// CTRL_FLAG_MODE bit (bit2) — on vehicle this selects ACRO (rate) mode
// (sf_comm decodes it to PilotRequest.acro). Must match the protocol SSOT /
// firmware/vehicle sf_comm kFlagMode = 0x04.
// CTRL_FLAG_MODE ビット(bit2) — vehicle では ACRO（角速度）モードを選択
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

// This vehicle's own MAC, lower 3 bytes — the drone_mac value a legitimately
// addressed ControlPacket must carry to be accepted as a pending-bind candidate
// while Pairing (pairing-methods-plan.md §4.1). Reads the SAME host shim
// (esp_wifi_get_mac(WIFI_IF_STA)) that Comm::init() uses for own_mac_, so it can
// never drift from the value the firmware's own-address filter compares against.
// この機体自身の MAC 下位3バイト — Pairing 中に保留バインド候補として受理される
// ために正しく宛先指定された ControlPacket が持つべき drone_mac の値
// （pairing-methods-plan.md §4.1）。Comm::init() が own_mac_ に使うのと同じホスト
// シム（esp_wifi_get_mac(WIFI_IF_STA)）を読むため、本体の自分宛フィルタが照合する
// 値とずれない。
void own_drone_mac(uint8_t out[3]);

// Build the 14-byte ControlPacket into `out` (must hold >= 14 bytes). drone_mac
// is the 3-byte destination address (bytes 0..2) — callers normally pass
// own_drone_mac() so the packet is accepted while Pairing; a scenario testing the
// own-address filter itself passes a DIFFERENT 3 bytes (a "wrong vehicle").
// Layout: [0..2]=drone_mac, [3..4]=throttle, [5..6]=roll, [7..8]=pitch,
// [9..10]=yaw, [11]=flags, [12]=reserved, [13]=checksum (sum of bytes 0..12).
// 14 バイト ControlPacket を `out` に構築。drone_mac は宛先3バイト（bytes 0..2）—
// 通常は own_drone_mac() を渡し Pairing 中に受理されるようにする。自分宛フィルタ
// 自体を試すシナリオは別の3バイト（「誤った機体」）を渡す。
void build_control_packet(uint8_t* out, const uint8_t drone_mac[3], uint16_t throttle,
                          uint16_t roll, uint16_t pitch, uint16_t yaw, uint8_t flags);

// Build one ControlPacket (addressed to THIS vehicle, own_drone_mac()) and
// deliver it into the firmware via the ESP-NOW hub (which records it). Stick
// values are ADC-scale (0..4095, centre 2048).
// ControlPacket を1つ（この機体宛、own_drone_mac()）組んで ESP-NOW ハブ経由で
// 本体へ配信（ハブが記録）。
void inject_rc(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
               uint8_t flags);

// Same as inject_rc (still addressed to THIS vehicle) but delivered from a
// DIFFERENT (non-paired) transmitter MAC. Used by the pairing scenario to verify
// the POST-bind crosstalk filter drops ControlPackets from a transmitter the
// vehicle is not paired with (it stays disarmed / motionless) — that filter
// checks the ESP-NOW sender MAC, not drone_mac, so addressing is irrelevant here.
// inject_rc と同じ（宛先はこの機体のまま）だが別の（未ペアの）送信機 MAC から配信
// する。ペアリングシナリオが、機体がペアしていない送信機の ControlPacket をバインド後の
// 混信フィルタが破棄すること（disarmed・不動のまま）を検証するために使う — そのフィルタは
// ESP-NOW 送信元 MAC を見る（drone_mac ではない）ため宛先指定はここでは無関係。
void inject_rc_foreign(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
                       uint8_t flags);

// Two-controller pairing test (pairing-methods-plan.md §4.4, own-address filter):
// "Controller A" addresses a DIFFERENT vehicle (drone_mac != own_drone_mac()) from
// a source MAC of its own. The vehicle's own-address filter must REJECT this as a
// pending-bind candidate while Pairing — it must never bind to Controller A.
// 2台コントローラのペアリング試験（pairing-methods-plan.md §4.4、自分宛フィルタ）:
// 「コントローラA」は別の機体宛（drone_mac != own_drone_mac()）に、専用の送信元 MAC
// から送る。機体の自分宛フィルタは Pairing 中の保留バインド候補として棄却しなければ
// ならない — コントローラAにバインドしてはならない。
void inject_rc_controller_a(uint16_t throttle, uint16_t roll, uint16_t pitch,
                            uint16_t yaw, uint8_t flags);

// Same test, "Controller B": correctly addresses THIS vehicle (own_drone_mac()),
// from yet another source MAC. The vehicle must bind to Controller B.
// 同じ試験の「コントローラB」: この機体宛（own_drone_mac()）を正しく指定し、さらに
// 別の送信元 MAC から送る。機体はコントローラBにバインドしなければならない。
void inject_rc_controller_b(uint16_t throttle, uint16_t roll, uint16_t pitch,
                            uint16_t yaw, uint8_t flags);

// Seed the firmware's pairing NVS so the emulated vehicle boots PAIRED to the
// injector's transmitter MAC (kPilotMac). Real hardware boots unpaired and auto-
// enters Pairing, which would block ARM; the flight scenarios inject RC without a
// pairing handshake, so we pre-bind them here (a vehicle that "was paired before").
// Call BEFORE app_main() (so comm::init() loads it). No-op when the SILS_EMU_UNPAIRED
// environment variable is set — used by the pairing scenario to test the handshake.
// 本体のペアリング NVS を seed し、エミュ機体がインジェクタの送信機 MAC（kPilotMac）に
// ペア済みで起動するようにする。実機は未ペア起動→自動 Pairing で ARM が阻まれるが、飛行
// シナリオはペアリングなしで RC を注入するため、ここで事前バインドする（「以前ペアした」機体）。
// app_main() の前に呼ぶこと（comm::init() が読むため）。環境変数 SILS_EMU_UNPAIRED が設定
// されていれば no-op — ペアリングシナリオがハンドシェイクを試験するために使う。
void seed_pairing_nvs();

}  // namespace sils
