/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file pmw3901_probe.cpp
 * @brief Offline unit probe for the PMW3901 optical-flow chip model.
 *        PMW3901 オプティカルフローチップモデルのオフライン単体 probe。
 *
 * Drives sil_pmw3901 directly (no FreeRTOS / MuJoCo) to verify, deterministically:
 *   1. the SPI register protocol the firmware driver depends on (product ID, inverse
 *      product ID, init-verify register, write/read-back),
 *   2. the motion-burst framing (command 0x16 → 12-byte payload), and
 *   3. the translational flow round-trip: set a body velocity, read the burst, and
 *      confirm delta_x/delta_y == round(v_body / height * frame_dt / rad_per_pixel).
 *
 * sil_pmw3901 を直接駆動し、SPI レジスタ規約・motion burst・並進フロー round-trip を
 * 決定論的に検証する。
 */

#include "pmw3901_device.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

int g_failures = 0;

void check(bool ok, const char* what, long got, long want)
{
    std::printf("  [%s] %s (got %ld, want %ld)\n", ok ? "PASS" : "FAIL", what, got, want);
    if (!ok) ++g_failures;
}

// 2-byte register read via the model (driver framing: {reg & 0x7F, 0x00} → rx[1]).
// モデル経由の2バイトレジスタ読み（ドライバ規約: {reg&0x7F,0x00} → rx[1]）。
uint8_t readReg(uint8_t reg)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    sil_pmw3901::xfer(tx, rx, 2);
    return rx[1];
}

void writeReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), val};
    uint8_t rx[2] = {0, 0};
    sil_pmw3901::xfer(tx, rx, 2);
}

// Read the 12-byte motion burst (command 0x16 + 12 data bytes) and decode dx/dy.
// 12バイト motion burst（コマンド0x16＋12データ）を読み dx/dy をデコード。
void readBurst(int16_t& dx, int16_t& dy, uint8_t& squal)
{
    uint8_t tx[13] = {0x16};
    uint8_t rx[13] = {0};
    sil_pmw3901::xfer(tx, rx, 13);
    dx = static_cast<int16_t>(rx[3] | (rx[4] << 8));
    dy = static_cast<int16_t>(rx[5] | (rx[6] << 8));
    squal = rx[7];
}

// Mirror the model's flow physics (must match pmw3901_device.cpp).
// モデルのフロー物理を再掲（pmw3901_device.cpp に一致させる）。
constexpr float kRadPerPixel = 0.00222f;
constexpr float kFrameDt     = 0.01f;
constexpr float kMinHeight   = 0.02f;
constexpr float kGyroScale   = 1.0f;

// Expected delta_x = (vx/h + scale*pitch_rate) * dt / rad_per_pixel.
// Expected delta_y = (vy/h - scale*roll_rate)  * dt / rad_per_pixel.
long expectedDx(float vx, float height, float gyro_y_pitch)
{
    return std::lround((vx / height + kGyroScale * gyro_y_pitch) * (kFrameDt / kRadPerPixel));
}
long expectedDy(float vy, float height, float gyro_x_roll)
{
    return std::lround((vy / height - kGyroScale * gyro_x_roll) * (kFrameDt / kRadPerPixel));
}

}  // namespace

int main()
{
    std::printf("[pmw3901_probe] --- (1) SPI register protocol ---\n");
    check(readReg(0x00) == 0x49, "product ID == 0x49",          readReg(0x00), 0x49);
    check(readReg(0x5F) == 0xB6, "inverse product ID == 0xB6",  readReg(0x5F), 0xB6);
    check(readReg(0x47) == 0x08, "init-verify 0x47 == 0x08",    readReg(0x47), 0x08);
    writeReg(0x55, 0x5A);
    check(readReg(0x55) == 0x5A, "config write/read-back",      readReg(0x55), 0x5A);

    std::printf("[pmw3901_probe] --- (2) flow round-trip (translational + rotational) ---\n");
    struct Case { float vx, vy, h, gx, gy; };   // gx=roll rate, gy=pitch rate [rad/s]
    const Case cases[] = {
        {0.0f,  0.0f,  0.60f,  0.0f,  0.0f},   // hover: no motion
        {0.5f,  0.0f,  0.60f,  0.0f,  0.0f},   // forward 0.5 m/s, no rotation
        {0.0f, -0.3f,  0.60f,  0.0f,  0.0f},   // left 0.3 m/s, no rotation
        {0.0f,  0.0f,  0.60f,  0.0f,  0.5f},   // pure pitch 0.5 rad/s → dx only
        {0.0f,  0.0f,  0.60f,  0.4f,  0.0f},   // pure roll 0.4 rad/s → dy only
        {0.4f,  0.2f,  1.00f,  0.1f, -0.2f},   // translation + rotation combined
    };
    for (const Case& c : cases) {
        sil_pmw3901::set_motion_from_velocity(c.vx, c.vy, c.h, c.gx, c.gy);
        int16_t dx = 0, dy = 0; uint8_t squal = 0;
        readBurst(dx, dy, squal);
        const long want_dx = expectedDx(c.vx, c.h, c.gy);
        const long want_dy = expectedDy(c.vy, c.h, c.gx);
        std::printf("  vx=%.2f vy=%.2f h=%.2f gx=%.2f gy=%.2f → dx=%d dy=%d\n",
                    c.vx, c.vy, c.h, c.gx, c.gy, dx, dy);
        check(dx == want_dx, "delta_x round-trip", dx, want_dx);
        check(dy == want_dy, "delta_y round-trip", dy, want_dy);
    }

    std::printf("[pmw3901_probe] --- (3) below min height → no flow ---\n");
    sil_pmw3901::set_motion_from_velocity(0.5f, 0.5f, kMinHeight - 0.001f, 0.0f, 0.0f);
    int16_t dx = 0, dy = 0; uint8_t squal = 0;
    readBurst(dx, dy, squal);
    check(dx == 0 && dy == 0, "no motion below min height", (dx == 0 && dy == 0), 1);
    check(squal == 0, "zero surface quality when no lock", squal, 0);

    if (g_failures == 0) {
        std::printf("[pmw3901_probe] OK (0 failures)\n");
        return 0;
    }
    std::printf("[pmw3901_probe] FAILED (%d failures)\n", g_failures);
    return 1;
}
