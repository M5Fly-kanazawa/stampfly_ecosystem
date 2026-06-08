/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario_inject.cpp
 * @brief Shared ControlPacket builder + RC injector implementation.
 *        共有 ControlPacket ビルダ＋RC インジェクタの実装。
 */

#include "scenario_inject.hpp"

#include <cstdlib>  // getenv (SIL_EMU_UNPAIRED)
#include "nvs.h"    // host NVS shim — seed the firmware's pairing store

// ESP-NOW delivery seam (espnow_hub.cpp); records the frame as it delivers.
// ESP-NOW 配信口（espnow_hub.cpp）。配信時にフレームを記録する。
extern "C" void sil_espnow_deliver(const uint8_t* src_mac, const uint8_t* data, int len);

namespace {

// Locally-administered "controller" MAC (0x02 = locally administered bit).
// ローカル管理アドレスの「送信機」MAC。
constexpr uint8_t kPilotMac[6] = {0x02, 0x53, 0x49, 0x4C, 0x00, 0x01};  // "SIL"

// A DIFFERENT transmitter MAC (a bystander's controller) for crosstalk tests. Must
// differ from kPilotMac so the vehicle's pairing filter rejects it once paired.
// 混信試験用の別の送信機 MAC（他人のコントローラ）。ペア後にフィルタが弾くよう
// kPilotMac と異なる値にする。
constexpr uint8_t kForeignMac[6] = {0x02, 0x53, 0x49, 0x4C, 0xFF, 0xFE};  // "SIL"+FFFE

}  // namespace

namespace sil {

void build_control_packet(uint8_t* p, uint16_t throttle, uint16_t roll,
                          uint16_t pitch, uint16_t yaw, uint8_t flags)
{
    p[0] = p[1] = p[2] = 0;                       // drone_mac lower 3 (match-any)
    p[3] = (uint8_t)(throttle & 0xFF); p[4] = (uint8_t)(throttle >> 8);
    p[5] = (uint8_t)(roll     & 0xFF); p[6] = (uint8_t)(roll     >> 8);
    p[7] = (uint8_t)(pitch    & 0xFF); p[8] = (uint8_t)(pitch    >> 8);
    p[9] = (uint8_t)(yaw      & 0xFF); p[10] = (uint8_t)(yaw     >> 8);
    p[11] = flags;
    p[12] = 0;
    uint32_t sum = 0;
    for (int i = 0; i < 13; ++i) sum += p[i];
    p[13] = (uint8_t)(sum & 0xFF);
}

void inject_rc(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
               uint8_t flags)
{
    uint8_t pkt[14];
    build_control_packet(pkt, throttle, roll, pitch, yaw, flags);
    sil_espnow_deliver(kPilotMac, pkt, (int)sizeof(pkt));
}

void inject_rc_foreign(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
                       uint8_t flags)
{
    uint8_t pkt[14];
    build_control_packet(pkt, throttle, roll, pitch, yaw, flags);
    sil_espnow_deliver(kForeignMac, pkt, (int)sizeof(pkt));
}

void seed_pairing_nvs()
{
    // Pairing scenario opts out (tests the real unpaired→Pairing→bind handshake).
    // ペアリングシナリオはオプトアウト（実際の未ペア→Pairing→bind を試験する）。
    if (std::getenv("SIL_EMU_UNPAIRED") != nullptr) {
        return;
    }
    // Write kPilotMac under the SAME namespace/key comm uses (sf_pair / ctrl_mac), so
    // comm::loadPairingFromNvs() restores it → the vehicle boots Paired to the injector.
    // comm が使う namespace/key（sf_pair / ctrl_mac）と同じ場所に kPilotMac を書く。
    // comm::loadPairingFromNvs() が復元し、機体はインジェクタにペア済みで起動する。
    nvs_handle_t handle;
    if (nvs_open("sf_pair", NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_blob(handle, "ctrl_mac", kPilotMac, sizeof(kPilotMac));
    nvs_commit(handle);
    nvs_close(handle);
}

}  // namespace sil
