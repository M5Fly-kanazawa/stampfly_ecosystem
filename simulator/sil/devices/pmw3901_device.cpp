/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file pmw3901_device.cpp
 * @brief PMW3901 optical-flow chip model — implementation.
 *        PMW3901 オプティカルフローチップモデル — 実装。
 *
 * Register protocol (from sf_hal_pmw3901/src/pmw3901.c):
 *   - 2-byte access: read = {reg & 0x7F, 0x00} → data in rx[1]; write = {reg|0x80, value}.
 *   - Motion burst: 13-byte transaction, tx[0] = 0x16, rx[1..12] = burst payload.
 * Init gates on PRODUCT_ID(0x00)=0x49 and INVERSE_PRODUCT_ID(0x5F)=0xB6; the rest of
 * the init writes are performance tuning the model simply stores and reads back.
 *
 * レジスタ規約（sf_hal_pmw3901 ドライバ準拠）。init は PRODUCT_ID=0x49 と
 * INVERSE_PRODUCT_ID=0xB6 を判定。他の init 書き込みは性能調整で、モデルは保存し読み戻す。
 *
 * @design simulator/sil/RESET_PLAN.md §6 — synthetic sensors (optical flow)  [--]
 */

#include "pmw3901_device.hpp"

#include <cmath>
#include <cstring>

namespace sil_pmw3901 {

namespace {

// --- Register addresses / values (match sf_hal_pmw3901/include/pmw3901.h) ----
constexpr uint8_t REG_PRODUCT_ID         = 0x00;
constexpr uint8_t REG_REVISION_ID        = 0x01;
constexpr uint8_t REG_INVERSE_PRODUCT_ID = 0x5F;
constexpr uint8_t REG_MOTION_BURST       = 0x16;
// Init verification (pmw3901_init_registers Step 6.2): after writing 0x43=0x10 in
// bank 0x0E the driver reads 0x47 and requires 0x08, else init fails. Model it as a
// fixed chip constant (0x47 is read only in this verification).
// init 検証(Step 6.2): bank 0x0E で 0x43=0x10 書込後 0x47 が 0x08 を返すことを要求。
// チップ固定値としてモデル化（0x47 はこの検証でのみ読まれる）。
constexpr uint8_t REG_INIT_VERIFY        = 0x47;
constexpr uint8_t INIT_VERIFY_VALUE      = 0x08;
constexpr uint8_t PRODUCT_ID_VALUE       = 0x49;
constexpr uint8_t INVERSE_PRODUCT_ID_VALUE = 0xB6;
constexpr uint8_t REVISION_ID_VALUE      = 0x00;

constexpr uint8_t WRITE_BIT      = 0x80;  // MSB set = write, clear = read
constexpr int     BURST_BYTES    = 13;    // 1 command + 12 data
constexpr int     REG_XFER_BYTES = 2;     // address + data

// --- Optical-flow physics (must match the firmware ESKF for a clean round-trip) --
// flow_rad_per_pixel / frame period mirror eskf_core.hpp + flow_task (100Hz).
// flow_rad_per_pixel・フレーム周期は eskf_core.hpp と flow_task(100Hz)に一致させる。
constexpr float FLOW_RAD_PER_PIXEL = 0.00222f;  // [rad/pixel] eskf_core.hpp:77
constexpr float FLOW_FRAME_DT      = 0.01f;     // [s] flow_task 100Hz period
constexpr float FLOW_MIN_HEIGHT    = 0.02f;     // [m] eskf_core.hpp:79 flow_min_height
constexpr uint8_t SQUAL_GOOD       = 0x50;      // surface quality when locked (~80)

// --- Motion-burst payload constants (plausible, not gated by the driver) ----
constexpr uint8_t BURST_MOTION_DETECTED = 0x80; // motion register bit7
constexpr uint8_t BURST_RAW_DATA_SUM    = 0x20;
constexpr uint8_t BURST_MAX_RAW_DATA    = 0x7F;
constexpr uint8_t BURST_MIN_RAW_DATA    = 0x00;
constexpr uint8_t BURST_SHUTTER_UPPER   = 0x10;
constexpr uint8_t BURST_SHUTTER_LOWER   = 0x00;

uint8_t  g_regs[256] = {};   // backing store for written config registers
int16_t  g_dx    = 0;        // delta_x the next burst reports [pixel counts]
int16_t  g_dy    = 0;        // delta_y the next burst reports [pixel counts]
uint8_t  g_squal = 0;        // surface quality the next burst reports

uint8_t readRegister(uint8_t reg)
{
    switch (reg) {
        case REG_PRODUCT_ID:         return PRODUCT_ID_VALUE;
        case REG_INVERSE_PRODUCT_ID: return INVERSE_PRODUCT_ID_VALUE;
        case REG_REVISION_ID:        return REVISION_ID_VALUE;
        case REG_INIT_VERIFY:        return INIT_VERIFY_VALUE;
        default:                     return g_regs[reg];
    }
}

}  // namespace

void set_motion_from_velocity(float vx_body, float vy_body, float height_m)
{
    // Below the usable height the flow has no ground lock → report no motion.
    // 使用可能高度未満はフローが床にロックしない → motion なしを報告。
    if (height_m < FLOW_MIN_HEIGHT) {
        set_motion(0, 0, 0);
        return;
    }

    // Translational optical-flow rate [rad/s] = horizontal velocity / height.
    // Pixel count over one frame = rate * frame_dt / rad_per_pixel.
    // 並進フロー角速度 [rad/s] = 水平速度 / 高度。1フレームのピクセル数 = 角速度·dt/rad_per_pixel。
    const float kPixPerRate = FLOW_FRAME_DT / FLOW_RAD_PER_PIXEL;
    const long dx = std::lround((vx_body / height_m) * kPixPerRate);
    const long dy = std::lround((vy_body / height_m) * kPixPerRate);

    set_motion(static_cast<int16_t>(dx), static_cast<int16_t>(dy), SQUAL_GOOD);
}

void set_motion(int16_t dx, int16_t dy, uint8_t squal)
{
    g_dx = dx;
    g_dy = dy;
    g_squal = squal;
}

int xfer(const uint8_t* tx, uint8_t* rx, int n)
{
    if (tx == nullptr) {
        if (rx != nullptr) std::memset(rx, 0, static_cast<size_t>(n));
        return 0;
    }

    // Motion burst: command 0x16 (MSB clear) in a 13-byte transaction.
    // motion burst: コマンド 0x16（MSB クリア）の13バイト転送。
    if (n >= BURST_BYTES && (tx[0] & 0x7F) == REG_MOTION_BURST) {
        if (rx != nullptr) {
            std::memset(rx, 0, static_cast<size_t>(n));
            rx[1] = (g_dx != 0 || g_dy != 0) ? BURST_MOTION_DETECTED : 0x00;  // motion
            rx[2] = 0x00;                                       // observation
            rx[3] = static_cast<uint8_t>(g_dx & 0xFF);          // delta_x L
            rx[4] = static_cast<uint8_t>((g_dx >> 8) & 0xFF);   // delta_x H
            rx[5] = static_cast<uint8_t>(g_dy & 0xFF);          // delta_y L
            rx[6] = static_cast<uint8_t>((g_dy >> 8) & 0xFF);   // delta_y H
            rx[7] = g_squal;                                    // SQUAL
            rx[8] = BURST_RAW_DATA_SUM;
            rx[9] = BURST_MAX_RAW_DATA;
            rx[10] = BURST_MIN_RAW_DATA;
            rx[11] = BURST_SHUTTER_UPPER;
            rx[12] = BURST_SHUTTER_LOWER;
        }
        return 0;
    }

    // 2-byte register access. MSB set = write (store), clear = read (data in rx[1]).
    // 2バイトのレジスタアクセス。MSB セット=書込（保存）、クリア=読出（rx[1] にデータ）。
    if (n >= REG_XFER_BYTES) {
        const uint8_t reg = tx[0];
        if (reg & WRITE_BIT) {
            g_regs[reg & 0x7F] = tx[1];
            if (rx != nullptr) std::memset(rx, 0, static_cast<size_t>(n));
        } else if (rx != nullptr) {
            rx[0] = 0x00;
            rx[1] = readRegister(reg & 0x7F);
        }
        return 0;
    }

    if (rx != nullptr) std::memset(rx, 0, static_cast<size_t>(n));
    return 0;
}

}  // namespace sil_pmw3901
