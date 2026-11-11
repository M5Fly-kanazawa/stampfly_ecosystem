/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
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
 * @design simulator/sils/RESET_PLAN.md §5-6 — register-level chip emulation  [--]
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Attach the MuJoCo Plant (opaque sils::Plant*). Called once from emu_main.
// MuJoCo Plant（不透明 sils::Plant*）を接続。emu_main から1回。
void sils_board_attach_plant(void* plant);

// Advance physics one step: push the latched motor duties into the Plant, step.
// 物理を1ステップ進める: ラッチされたモータ duty を Plant に渡して step。
void sils_board_step_plant(float dt_s);

// SPI transaction hook (driver/spi_master.h → here). cs = chip-select GPIO.
// tx/rx are the full-duplex buffers, nbytes the transfer length in bytes.
// SPI トランザクションフック。cs = CS GPIO、tx/rx は全二重バッファ。
int sils_board_spi_transfer(int cs, const uint8_t* tx, uint8_t* rx, size_t nbytes);

// I2C transaction hook (driver/i2c_master.h → here). addr = 7/10-bit device
// address (decoded from the opaque handle by the shim). A transfer has an
// optional write phase (write_buf/write_size) followed by an optional read phase
// (read_buf/read_size); write-then-read in one call models a repeated-START
// register read. Returns 0 on ACK. Unmodeled addresses zero-fill the read buffer
// and still return 0, matching the old inert shim (Optional sensors fail
// gracefully on their chip-ID check until their device models land).
// I2C トランザクションフック。addr = デバイスアドレス（シムが不透明ハンドルから復号）。
// 書き込み相＋読み出し相（同一呼び出しで repeated-START のレジスタ読みを表現）。
// 0 = ACK。未モデルアドレスは read をゼロ埋めし 0 を返す（旧 inert シムと同一）。
int sils_board_i2c_xfer(uint16_t addr,
                       const uint8_t* write_buf, size_t write_size,
                       uint8_t* read_buf, size_t read_size);

// LEDC duty hook (driver/ledc.h → here). channel 0..3 = motors M1..M4, duty is
// the raw N-bit LEDC value; max_duty is the resolution full-scale (e.g. 255).
// LEDC duty フック。channel 0..3 = モータ M1..M4、duty は生 LEDC 値。
void sils_board_ledc_set_duty(int channel, uint32_t duty, uint32_t max_duty);

// Read the latched per-motor duties [0,1] (M1..M4) the board last received over
// LEDC. Used by the review-video trajectory recorder; does not touch the Plant.
// ボードが LEDC で最後に受け取った各モータ duty[0,1]（M1..M4）を読む。動画軌跡記録用。
void sils_board_get_motor_duty(float out[4]);

// P7 disturbance hooks (scenario wind/fault events → Plant). Set the external wind
// force in NED [N], or degrade one motor's thrust health gain (motor 0..3, 1=healthy,
// 0=dead). No-op until a Plant is attached. RESET_PLAN §13 P7.
// P7 外乱フック（シナリオの wind/fault → Plant）。NED 風力 [N] の設定、または1モータの
// 推力健全度ゲインの劣化（motor 0..3, 1=正常/0=停止）。Plant 未接続なら no-op。
void sils_board_set_wind(float fx, float fy, float fz);

void sils_board_set_motor_health(int motor, float gain);

// Scenario `battery <V>`: hold the Plant's battery terminal voltage at `volts` [V] (the motors' supply and
// the INA3221 reading); `volts` <= 0 releases the hold (the battery model takes over again).
// シナリオ `battery <V>`: Plant の電池端子電圧（モータの電源と INA3221 の読み）を `volts` [V] に保つ。
// `volts` が 0 以下なら保持を解く（電池モデルが再び決める）。
void sils_board_hold_battery_voltage(float volts);

// Set a deterministic raw IMU bias (body FRD): accel [m/s²], gyro [rad/s]. Models a
// pre-calibration MEMS offset so the firmware boot calibration has something to remove
// (P2-3 contrast test). No-op until a Plant is attached.
// 決定論的な生 IMU バイアス（機体 FRD）を設定: accel[m/s²], gyro[rad/s]。校正前 MEMS
// オフセットを模擬し、ファーム起動校正に除去対象を与える（P2-3 対照試験）。Plant 未接続なら no-op。
void sils_board_set_imu_bias(float ax, float ay, float az, float gx, float gy, float gz);

// Fault injection (scenario `tof_silent`): stop (1) or resume (0) the downward ToF's
// sample output. While stopped the chip model never reports "data ready" (no new sample, no
// I2C error), so the firmware sees the sensor go silent. Independent of the Plant.
// 故障注入（シナリオ `tof_silent`）: 下向き ToF のサンプル出力を停止（1）/再開（0）する。
// 停止中はチップモデルが「データ準備完了」を返さない（新サンプル無し・I2C エラー無し）ので、
// ファームからはセンサが出力停止したように見える。Plant には依存しない。
void sils_board_set_tof_silent(int silent);

// Fault injection (scenario `tof_invalid`): make every sample of the downward ToF invalid ("no
// target", 1) or restore it (0). Samples keep arriving on time, so this is NOT a silence: it is the
// sensor that cannot range the surface below. Independent of the Plant.
// 故障注入（シナリオ `tof_invalid`）: 下向き ToF の全サンプルを無効（「目標なし」、1）にする、または
// 元に戻す（0）。サンプルは時間どおり届き続けるので出力停止ではない: 下の面を測れないセンサ。
// Plant には依存しない。
void sils_board_set_tof_invalid(int invalid);

// Scenario `tof_deadzone`: model (1, the default) or ignore (0) the close-range dead zone of the downward
// ToF (vl53_device.cpp MIN_MM): below it the sensor has no return pulse and reads "no target", as the real
// sensor does on the floor. Independent of the Plant.
// シナリオ `tof_deadzone`: 下向き ToF の近距離の不感帯（vl53_device.cpp の MIN_MM）を模擬する（1、既定）/
// しない（0）。この距離より近いとセンサは反射パルスが無く「目標なし」を返す（実機が床の上でそうなるのと同じ）。
// Plant には依存しない。
void sils_board_set_tof_deadzone(int enabled);

// P8 handling hook (scenario `handle` event → Plant). Begin a physical handling maneuver:
// the hand lifts the (crashed) craft to carry_alt [m up], rights it to level, carries it to
// place_x/place_y [NED m] and sets it back on the ground, over lift/carry/place seconds.
// The Plant prescribes a continuous kinematic trajectory and synthesizes the IMU/ToF/gyro
// (no teleport). No-op until a Plant is attached. RESET_PLAN §13 P8.
// P8 ハンドリングフック（シナリオ `handle` → Plant）。物理ハンドリング動作を開始: 手が（墜落
// した）機体を carry_alt[上方m]へ持上げ、水平へ起こし、place_x/place_y[NED m]へ運び地面に戻す
// （lift/carry/place 秒）。Plant が連続キネマティック軌道を規定し IMU/ToF/gyro を合成（teleport
// なし）。Plant 未接続なら no-op。
void sils_board_handle_place(float carry_alt_m, float place_x_ned, float place_y_ned,
                            float lift_s, float carry_s, float place_s);

#ifdef __cplusplus
}  // extern "C"
#endif
