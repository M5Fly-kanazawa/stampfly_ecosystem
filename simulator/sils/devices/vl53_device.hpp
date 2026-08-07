/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file vl53_device.hpp
 * @brief VL53L3CX ToF chip model (I2C-byte level) — the unmodified ST BareDriver
 *        runs on the host against this model. Extracted from virtual_board.cpp so
 *        both the emulator (virtual_board) and the offline gen4 probe
 *        (smoke/vl53_probe.cpp) link the SAME model.
 *        VL53L3CX ToF チップモデル（I2Cバイトレベル）。無改変 ST ベアドライバが
 *        ホストでこのモデルに対し動く。エミュレータとオフライン gen4 プローブが
 *        同一モデルをリンクできるよう virtual_board.cpp から分離。
 *
 * @design simulator/sils/RESET_PLAN.md §E2 — I2C device model (VL53L3CX)  [--]
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace sils_vl53 {

// Power-on / re-addressed I2C addresses (the model is stateless w.r.t. address;
// the board routes both to xfer()). front 0x31 is left to the board catch-all.
// 起動時 / 再アドレス後の I2C アドレス（モデルはアドレス非依存; 両方を xfer へ）。
constexpr uint16_t ADDR_DEFAULT = 0x29;   // power-on I2C address
constexpr uint16_t ADDR_BOTTOM  = 0x30;   // re-addressed bottom altimeter

// Push the current down-range target distance [mm] that the synthesized histogram
// should encode (the board supplies the Plant ToF each transaction). The env
// override SILS_VL53_TEST_MM, when set, always wins over this pushed value.
// 合成 histogram が符号化すべき下向き目標距離[mm]を渡す（ボードが毎取引で Plant
// ToF を供給）。環境変数 SILS_VL53_TEST_MM があれば常にそちらが優先。
void set_distance_mm(float mm);

// One VL53 I2C transaction. 16-bit BE register pointer in wbuf[0..1] (write-then-
// read = repeated-START). Returns 0 (ACK). Mirrors the real chip's register map
// only where the ST driver reads it; everything else is zero/ACK self-heal.
// VL53 の1 I2Cトランザクション。16bit BE レジスタポインタは wbuf[0..1]。
int xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen);

}  // namespace sils_vl53
