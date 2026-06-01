/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file virtual_pilot_vehicle.cpp
 * @brief Virtual ESP-NOW transmitter for firmware/vehicle (the old firmware).
 *        旧ファーム firmware/vehicle 用の仮想 ESP-NOW 送信機。
 *
 * Acts as the StampFly hand controller: it builds REAL 14-byte ControlPackets
 * (the on-air layout from controller_comm.hpp) and delivers them into the
 * firmware's ESP-NOW receive callback via sil_espnow_deliver(). The unmodified
 * comm decoder, arm/disarm edge logic, ControlArbiter and rate/attitude control
 * loop all run from genuine controller input — no test back-door. This is the
 * radio seam: bytes-on-air reproduced, RF layer not (RESET_PLAN §11).
 *
 * Sequence: stream disarmed-centred packets while the firmware calibrates, then
 * a rising edge of the ARM flag, then ramp throttle to a hover setting and hold.
 *
 * StampFly の送信機を模す: 実 14 バイト ControlPacket を組み、recv コールバックへ配信。
 * 本体の comm デコード・アーム判定・ControlArbiter・制御ループが実コントローラ入力で走る。
 * 手順: 校正中は disarmed 中央値を流す → ARM フラグ立ち上げ → スロットルをホバーへ。
 *
 * @design simulator/.../robust-seeking-quail.md E3 — esp_now virtual pilot
 */

#include <cstdint>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void sil_espnow_deliver(const uint8_t* src_mac, const uint8_t* data, int len);

namespace {

// On-air ControlPacket layout (controller_comm.hpp): 14 bytes, packed.
// byte0-2 drone_mac, 3-4 throttle, 5-6 roll, 7-8 pitch, 9-10 yaw, 11 flags
// (bit0 Arm), 12 reserved, 13 checksum = sum(bytes0..12).
//
// IMPORTANT: although the struct comment says 0..1000, the firmware actually
// treats the stick fields as raw 12-bit ADC counts, 0..4095 with 2048 = centre
// (ControlArbiter::normalizeThrottle/normalizeAxis use ADC_CENTER=2048). The
// real hand controller sends ADC counts; the "0..1000" comment is stale. So the
// pilot MUST send ADC-scale values, or throttle clamps to 0 and sticks deflect.
// 重要: 構造体コメントは 0..1000 だが、本体はスティックを 12bit ADC（0..4095, 中央
// 2048）として扱う。実送信機は ADC 値を送る。コメントが古い。ADC スケールで送る。
constexpr uint16_t kAdcCentre   = 2048;  // ADC_CENTER — stick neutral / 中央
// Hover throttle. The firmware maps throttle -> total_thrust = T·MAX_TOTAL_THRUST
// directly (no altitude loop in STABILIZE), so hover (thrust = mg) is at T≈0.5,
// i.e. ADC ≈ 2048 + 0.5·2048 = 3072.
//
// KNOWN GAP (a real finding the SIL surfaced): the thrust->duty conversion is
// battery-voltage-compensated (control_task.cpp ~L829-855). The PowerMonitor
// (INA3221) is not yet emulated, so state.getVoltage() returns 0 and the vbat
// LPF decays 3.7 V → 0, inflating every duty to 1.0 → the craft lifts off but
// then climbs away. A stable hover is gated on the E3 INA3221 device model
// (valid vbat) and, for altitude-hold, the E2 VL53L3CX ToF.
// ホバースロットル。本体は throttle→total_thrust 直結（STABILIZE に高度ループ無し）で
// ホバーは T≈0.5（ADC≈3072）。既知の欠落（SIL が炙り出した本物の発見）: thrust→duty 変換は
// 電池電圧補償付きだが INA3221 未実装で getVoltage()=0 → vbat が 3.7→0 に減衰し duty が 1.0 へ
// 膨張 → 離陸後に上昇し続ける。安定ホバーは E3(INA3221) と高度保持に E2(VL53 ToF) が前提。
constexpr uint16_t kHoverThr    = 3072;  // ~0.50 normalized -> total_thrust ≈ mg
constexpr uint8_t  kFlagArm     = 0x01;  // CTRL_FLAG_ARM

// A locally-administered "controller" MAC (0x02 = locally administered bit).
// ローカル管理アドレスの「送信機」MAC。
constexpr uint8_t kPilotMac[6] = {0x02, 0x53, 0x49, 0x4C, 0x00, 0x01};  // "SIL"

void build_packet(uint8_t* p, uint16_t throttle, uint16_t roll, uint16_t pitch,
                  uint16_t yaw, uint8_t flags)
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

void send(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw, uint8_t flags)
{
    uint8_t pkt[14];
    build_packet(pkt, throttle, roll, pitch, yaw, flags);
    sil_espnow_deliver(kPilotMac, pkt, sizeof(pkt));
}

// Send at ~20 Hz for `ms` milliseconds of virtual time.
// 仮想時間 `ms` ミリ秒のあいだ ~20 Hz で送信する。
void stream(int ms, uint16_t throttle, uint8_t flags)
{
    const int frames = ms / 50;
    for (int i = 0; i < frames; ++i) {
        send(throttle, kAdcCentre, kAdcCentre, kAdcCentre, flags);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

}  // namespace

// Pilot task. Linked only into emu_vehicle; emu_main_generic spawns it via the
// weak sil_virtual_pilot_task symbol (absent for other firmwares → no pilot).
// パイロットタスク。emu_vehicle のみにリンクされ、emu_main_generic が weak シンボル
// 経由で起動する（他ファームでは未定義 → パイロット無し）。
extern "C" void sil_virtual_pilot_task(void* /*arg*/)
{
    std::printf("[pilot] virtual controller online (ESP-NOW, 20 Hz)\n");

    // Phase A — disarmed & centred while the firmware boots and calibrates the
    // gyro (~13 s). Keeps the arm flag LOW so the next phase is a clean rising
    // edge, and keeps the link "connected".
    // フェーズA — 起動＋ジャイロ校正中は disarmed 中央値。arm フラグは LOW のまま。
    stream(20000, 0, 0x00);

    // Phase B — raise the ARM flag (rising edge) with throttle at zero. The
    // firmware arms only from IDLE with calibration complete.
    // フェーズB — スロットル 0 で ARM フラグ立ち上げ（IDLE＋校正完了でアーム）。
    std::printf("[pilot] ARM\n");
    stream(1000, 0, kFlagArm);

    // Phase C — ramp throttle from idle (ADC centre = 0 thrust) up to the hover
    // setting over ~1.5 s, then hold. Sticks centred (2048) so the attitude/rate
    // loop keeps the craft level on the IMU. The throttle-up also drives the
    // firmware's ARMED -> FLYING transition.
    // フェーズC — スロットルを中央(=推力0)からホバー値へ約1.5秒で上げて維持。
    // スティックは中央(2048)。スロットルアップで ARMED→FLYING も駆動。
    std::printf("[pilot] throttle ramp -> hover\n");
    for (int t = kAdcCentre; t <= kHoverThr; t += 45) {   // centre..hover at 20 Hz
        send((uint16_t)t, kAdcCentre, kAdcCentre, kAdcCentre, kFlagArm);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    std::printf("[pilot] hold hover throttle (ADC %u)\n", (unsigned)kHoverThr);
    stream(60000, kHoverThr, kFlagArm);          // hold until the run ends

    vTaskDelete(nullptr);
}
