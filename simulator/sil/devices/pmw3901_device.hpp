/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file pmw3901_device.hpp
 * @brief PMW3901 optical-flow chip model (SPI), shared by the emulator and probe.
 *        PMW3901 オプティカルフローチップモデル（SPI）、エミュレータと probe で共有。
 *
 * Models the PixArt PMW3901 at the SPI-register level so the UNMODIFIED firmware
 * driver (sf_hal_pmw3901/pmw3901.c) boots, verifies the product ID, and reads the
 * 12-byte motion burst. The flow counts (delta_x / delta_y) are synthesized from the
 * Plant's true horizontal velocity so the firmware's ESKF recovers that velocity.
 *
 * PixArt PMW3901 を SPI レジスタ層でモデル化し、無改変ドライバが起動・product ID 検証・
 * 12バイト motion burst 読みを通す。フローカウント(delta_x/delta_y)は Plant の真の水平
 * 速度から合成し、ファームの ESKF がその速度を復元できるようにする。
 *
 * @design simulator/sil/RESET_PLAN.md §6 — synthetic sensors (optical flow)  [--]
 */

#pragma once

#include <cstdint>

namespace sil_pmw3901 {

/// PMW3901 chip-select GPIO on the StampFly (sf_board). BMI270 shares the bus on 46.
/// StampFly の PMW3901 チップセレクト GPIO（sf_board）。BMI270 は同じバスの 46。
constexpr int CS_PIN = 12;

/// Synthesize the next motion burst from the body-frame horizontal velocity AND the
/// body angular rate. Total optical flow = translational (v_body / height) +
/// rotational (flow_gyro_scale * body rate); the firmware ESKF removes the rotational
/// part, so including it makes the round-trip recover the true velocity even while
/// the craft pitches/rolls. Uses the SAME constants as the firmware (eskf_core.hpp).
/// Below the minimum height the flow is invalid (zero motion, zero surface quality).
/// gyro_x/gyro_y are the body roll/pitch rates [rad/s] (FRD).
/// body 水平速度＋body 角速度から次の motion burst を合成する。総フロー = 並進(v_body/
/// height) + 回転(flow_gyro_scale·body角速度)。ファーム ESKF が回転分を除去するので、
/// ここで含めると pitch/roll 中でも round-trip が真速度を復元する。gyro_x/gyro_y は
/// body の roll/pitch 角速度 [rad/s]（FRD）。最小高度未満はフロー無効。
void set_motion_from_velocity(float vx_body, float vy_body, float height_m,
                              float gyro_x, float gyro_y);

/// Directly set the raw motion the next burst reports (pixel counts + quality).
/// Used by the offline probe to inject known counts.
/// 次の burst が報告する生 motion を直接設定（probe が既知カウントを注入）。
void set_motion(int16_t dx, int16_t dy, uint8_t squal);

/// One SPI transaction. The driver frames a 2-byte register access (read =
/// {reg&0x7F, 0x00} with data in rx[1]; write = {reg|0x80, value}) or a 13-byte
/// motion-burst read (command 0x16 followed by 12 data bytes). Returns 0 (= ESP_OK).
/// 1 SPI トランザクション。ドライバは2バイトのレジスタアクセス、または13バイトの
/// motion burst 読み（コマンド0x16＋12データバイト）を組む。戻り値 0（= ESP_OK）。
int xfer(const uint8_t* tx, uint8_t* rx, int n);

}  // namespace sil_pmw3901
