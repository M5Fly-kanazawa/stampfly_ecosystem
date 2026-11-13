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

// Fault injection (scenario `tof_silent`): while silent, the model never reports
// "data ready", so the firmware's TofTask sees no new sample — the sensor has stopped
// delivering, as in the 2026-11 accident (last sample, then nothing). The model keeps
// answering every I2C transaction with ACK, so this is "no new data", not an I2C error.
// 故障注入（シナリオ `tof_silent`）: 出力停止中はモデルが「データ準備完了」を返さないので、
// ファームの TofTask は新サンプルを得ない — 2026-11 の事故（最後のサンプルの後は何も来ない）
// と同じく、センサが出力を止めた状態。I2C の応答は常に ACK のままなので、これは
// 「新データ無し」であって I2C エラーではない。
void set_silent(bool silent);

// Fault injection (scenario `tof_invalid`): while invalid, every sample is "no target" (the
// histogram has no return pulse), whatever the Plant distance — the sensor keeps delivering
// samples on time, but all of them are invalid (e.g. a floor material it cannot range). The
// firmware's silence watch cannot see this: the stamps keep advancing.
// 故障注入（シナリオ `tof_invalid`）: 無効中は、Plant の距離に関わらず全サンプルが「目標なし」
// （histogram に反射パルスが無い）— センサはサンプルを時間どおり届け続けるが、全て無効（例: 測距
// できない床材）。ファームの出力停止の監視には見えない: 時刻印は進み続けるため。
void set_invalid(bool invalid);

// Close-range dead zone (scenario `tof_deadzone`): on (the default) = a true distance below MIN_MM has no
// return pulse (reads "no target"), as the real sensor on the floor; off = the true distance is ranged down
// to the floor (the model before 2026-11-09).
// 近距離の不感帯（シナリオ `tof_deadzone`）: 有効（既定）= 真の距離が MIN_MM 未満では反射パルスが無い
// （「目標なし」。床の上の実機のセンサと同じ）。無効 = 床まで真の距離を測る（2026-11-09 より前のモデル）。
void set_deadzone(bool enabled);

// Fault injection (scenario `tof_offline`): while offline, the chip does not answer on the bus at all (every transaction is
// NACKed, xfer() returns nonzero), unlike `tof_silent` where it answers but delivers no sample. When it answers
// again it has lost its configuration: it delivers no sample until the firmware has re-initialised it and started ranging
// (the driver's full init-and-start write), at which point the frame counters also start over.
// 故障注入（シナリオ `tof_offline`）: 停止中はチップがバスに全く応答しない（全ての通信が NACK され、xfer() が非ゼロを
// 返す）。サンプルを出さないだけで応答はする `tof_silent` とは違う。再び応答するときは設定を失っていて、ファームが
// 再初期化して測距を開始する（ドライバの完全な初期化＋開始の書き込み）までサンプルを出さない。そのときフレーム
// カウンタも数え直す。
void set_offline(bool offline);

// One VL53 I2C transaction. 16-bit BE register pointer in wbuf[0..1] (write-then-
// read = repeated-START). Returns 0 (ACK). Mirrors the real chip's register map
// only where the ST driver reads it; everything else is zero/ACK self-heal.
// VL53 の1 I2Cトランザクション。16bit BE レジスタポインタは wbuf[0..1]。
int xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen);

}  // namespace sils_vl53
