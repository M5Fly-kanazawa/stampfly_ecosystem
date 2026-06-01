/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file virtual_board.hpp
 * @brief StampFly virtual board — the bus-level device models the ESP-IDF host
 *        shims dispatch to (SPI chips, LEDC motors), connected to the Plant.
 *        StampFly 仮想ボード — ESP-IDF host シムが分岐する先のバスレベル
 *        デバイスモデル（SPIチップ・LEDCモータ）。Plant に接続。
 *
 * The driver shims (driver/spi_master.h, driver/ledc.h) call these C-linkage
 * hooks instead of touching real hardware. The board routes an SPI transaction
 * by chip-select to the right device model (E1: BMI270 register-level from the
 * Plant IMU), and LEDC duty to the 4 motors → Plant. Firmware-agnostic: any
 * firmware driving these chips over ESP-IDF reaches the same models.
 *
 * ドライバシムがこれら C リンケージのフックを呼ぶ。CS でSPIトランザクションを
 * デバイスへ振り分け（E1: BMI270 をレジスタレベルで Plant IMU から）、LEDC duty を
 * 4モータ→Plant へ。ファーム非依存。
 *
 * @design simulator/sil/RESET_PLAN.md §5-6 — register-level chip emulation  [--]
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Attach the MuJoCo Plant (opaque sil::Plant*). Called once from emu_main.
// MuJoCo Plant（不透明 sil::Plant*）を接続。emu_main から1回。
void sil_board_attach_plant(void* plant);

// Advance physics one step: push the latched motor duties into the Plant, step.
// 物理を1ステップ進める: ラッチされたモータ duty を Plant に渡して step。
void sil_board_step_plant(float dt_s);

// SPI transaction hook (driver/spi_master.h → here). cs = chip-select GPIO.
// tx/rx are the full-duplex buffers, nbytes the transfer length in bytes.
// SPI トランザクションフック。cs = CS GPIO、tx/rx は全二重バッファ。
int sil_board_spi_transfer(int cs, const uint8_t* tx, uint8_t* rx, size_t nbytes);

// I2C transaction hook (driver/i2c_master.h → here). addr = 7/10-bit device
// address (decoded from the opaque handle by the shim). A transfer has an
// optional write phase (write_buf/write_size) followed by an optional read phase
// (read_buf/read_size); write-then-read in one call models a repeated-START
// register read. Returns 0 on ACK. Unmodeled addresses zero-fill the read buffer
// and still return 0, matching the old inert shim (Optional sensors fail
// gracefully on their chip-ID check until their device models land).
// I2C トランザクションフック。addr = デバイスアドレス（シムが不透明ハンドルから復号）。
// 書き込み相＋読み出し相（同一呼び出しで repeated-START のレジスタ読みを表現）。
// 0 = ACK。未模型アドレスは read をゼロ埋めし 0 を返す（旧 inert シムと同一）。
int sil_board_i2c_xfer(uint16_t addr,
                       const uint8_t* write_buf, size_t write_size,
                       uint8_t* read_buf, size_t read_size);

// LEDC duty hook (driver/ledc.h → here). channel 0..3 = motors M1..M4, duty is
// the raw N-bit LEDC value; max_duty is the resolution full-scale (e.g. 255).
// LEDC duty フック。channel 0..3 = モータ M1..M4、duty は生 LEDC 値。
void sil_board_ledc_set_duty(int channel, uint32_t duty, uint32_t max_duty);

// Read the latched per-motor duties [0,1] (M1..M4) the board last received over
// LEDC. Used by the review-video trajectory recorder; does not touch the Plant.
// ボードが LEDC で最後に受け取った各モータ duty[0,1]（M1..M4）を読む。動画軌跡記録用。
void sil_board_get_motor_duty(float out[4]);

#ifdef __cplusplus
}  // extern "C"
#endif
