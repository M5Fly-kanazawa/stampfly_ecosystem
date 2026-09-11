/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario_inject.cpp
 * @brief Shared ControlPacket builder + RC injector implementation.
 *        共有 ControlPacket ビルダ＋RC インジェクタの実装。
 */

#include "scenario_inject.hpp"

#include <cstdlib>  // getenv (SILS_EMU_UNPAIRED)
#include "esp_wifi.h"  // esp_wifi_get_mac (host shim) — same call Comm::init() uses
#include "nvs.h"    // host NVS shim — seed the firmware's pairing store

// ESP-NOW delivery seam (espnow_hub.cpp); records the frame as it delivers.
// ESP-NOW 配信口（espnow_hub.cpp）。配信時にフレームを記録する。
extern "C" void sils_espnow_deliver(const uint8_t* src_mac, const uint8_t* data, int len);

namespace {

// Locally-administered "controller" MAC (0x02 = locally administered bit).
// ローカル管理アドレスの「送信機」MAC。
constexpr uint8_t kPilotMac[6] = {0x02, 0x53, 0x49, 0x4C, 0x00, 0x01};  // "SILS"

// A DIFFERENT transmitter MAC (a bystander's controller) for crosstalk tests. Must
// differ from kPilotMac so the vehicle's pairing filter rejects it once paired.
// 混信試験用の別の送信機 MAC（他人のコントローラ）。ペア後にフィルタが弾くよう
// kPilotMac と異なる値にする。
constexpr uint8_t kForeignMac[6] = {0x02, 0x53, 0x49, 0x4C, 0xFF, 0xFE};  // "SILS"+FFFE

// Two-controller own-address-filter test (pairing-methods-plan.md §4.4):
// "Controller A" picks the WRONG vehicle, "Controller B" picks the right one.
// Source MACs distinct from kPilotMac/kForeignMac above so all four roles never
// collide on-air.
// 2台コントローラの自分宛フィルタ試験（pairing-methods-plan.md §4.4）:
// 「コントローラA」は誤った機体を選び、「コントローラB」は正しい機体を選ぶ。送信元
// MAC は上の2つと重複せず、4役が電波上で衝突しない。
constexpr uint8_t kControllerAMac[6] = {0x02, 0x53, 0x49, 0x4C, 0xAA, 0x01};  // "SILS"+AA01
constexpr uint8_t kControllerBMac[6] = {0x02, 0x53, 0x49, 0x4C, 0xBB, 0x01};  // "SILS"+BB01

// A drone_mac that is NOT this vehicle's own address — the "different vehicle"
// Controller A has (deliberately, for this test) selected.
// この機体自身のアドレスではない drone_mac — コントローラAが（本試験のため意図的に）
// 選んだ「別の機体」。
constexpr uint8_t kOtherVehicleMac[3] = {0x00, 0x00, 0x99};

}  // namespace

namespace sils {

void own_drone_mac(uint8_t out[3])
{
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);  // same host shim Comm::init() reads into own_mac_
    out[0] = mac[3];
    out[1] = mac[4];
    out[2] = mac[5];
}

void build_control_packet(uint8_t* p, const uint8_t drone_mac[3], uint16_t throttle,
                          uint16_t roll, uint16_t pitch, uint16_t yaw, uint8_t flags)
{
    p[0] = drone_mac[0]; p[1] = drone_mac[1]; p[2] = drone_mac[2];
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
    uint8_t own3[3];
    own_drone_mac(own3);
    uint8_t pkt[14];
    build_control_packet(pkt, own3, throttle, roll, pitch, yaw, flags);
    sils_espnow_deliver(kPilotMac, pkt, (int)sizeof(pkt));
}

void inject_rc_foreign(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw,
                       uint8_t flags)
{
    uint8_t own3[3];
    own_drone_mac(own3);
    uint8_t pkt[14];
    build_control_packet(pkt, own3, throttle, roll, pitch, yaw, flags);
    sils_espnow_deliver(kForeignMac, pkt, (int)sizeof(pkt));
}

void inject_rc_controller_a(uint16_t throttle, uint16_t roll, uint16_t pitch,
                            uint16_t yaw, uint8_t flags)
{
    uint8_t pkt[14];
    build_control_packet(pkt, kOtherVehicleMac, throttle, roll, pitch, yaw, flags);
    sils_espnow_deliver(kControllerAMac, pkt, (int)sizeof(pkt));
}

void inject_rc_controller_b(uint16_t throttle, uint16_t roll, uint16_t pitch,
                            uint16_t yaw, uint8_t flags)
{
    uint8_t own3[3];
    own_drone_mac(own3);
    uint8_t pkt[14];
    build_control_packet(pkt, own3, throttle, roll, pitch, yaw, flags);
    sils_espnow_deliver(kControllerBMac, pkt, (int)sizeof(pkt));
}

void seed_pairing_nvs()
{
    // Pairing scenario opts out (tests the real unpaired→Pairing→bind handshake).
    // ペアリングシナリオはオプトアウト（実際の未ペア→Pairing→bind を試験する）。
    if (std::getenv("SILS_EMU_UNPAIRED") != nullptr) {
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

}  // namespace sils
