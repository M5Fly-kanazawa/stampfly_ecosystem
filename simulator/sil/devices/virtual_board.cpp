/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file virtual_board.cpp
 * @brief StampFly virtual board implementation — BMI270 SPI device model +
 *        LEDC motor mapping, connected to the MuJoCo Plant (E1).
 *        StampFly 仮想ボード実装 — BMI270 SPI デバイスモデル＋LEDCモータ写像、
 *        MuJoCo Plant に接続（E1）。
 *
 * BMI270 register protocol (from sf_hal_bmi270/src/bmi270_spi.c):
 *   read : tx=[reg|0x80, dummy, ...], rx[2..] = register data (1 dummy byte).
 *   write: tx=[reg, data, ...].
 * Data path: the real driver burst-reads 12 bytes at ACC_X_LSB(0x0C) and
 * converts raw → g / rad/s by the configured range; imu_task then remaps the
 * chip axes to body-FRD. We produce CHIP-frame raw from the Plant body-FRD IMU
 * so the full real chain reconstructs the Plant's physical values exactly.
 *
 * BMI270 レジスタ規約。データ経路: 実ドライバが 0x0C で12バイトburst読みし範囲で
 * raw→g/rad/s 変換、imu_task が chip軸→body-FRD remap。Plant の body-FRD IMU から
 * chip系 raw を生成し、実チェーンが Plant 物理値を厳密に復元する。
 */

#include "virtual_board.hpp"

#include <cmath>
#include <cstring>

#include "plant.hpp"        // sil::Plant
#include "data_types.hpp"   // sf::ImuData, sf::MotorOutput

namespace {

sil::Plant* g_plant = nullptr;
float g_motor_duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// --- BMI270 register addresses (match sf_hal_bmi270 defines) ------------------
constexpr uint8_t REG_CHIP_ID         = 0x00;
constexpr uint8_t REG_STATUS          = 0x03;
constexpr uint8_t REG_ACC_X_LSB       = 0x0C;
constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
constexpr uint8_t REG_ACC_RANGE       = 0x41;
constexpr uint8_t REG_GYR_RANGE       = 0x43;
constexpr uint8_t CHIP_ID_VALUE       = 0x24;

constexpr float kG        = 9.80665f;
constexpr float kRad2Deg  = 57.2957795131f;

// BMI270 virtual chip state. regs[] backs read-back of written config/range.
// BMI270 仮想チップ状態。regs[] は書き込んだ設定/範囲の読み戻しを支える。
struct Bmi270State {
    uint8_t regs[128] = {0};
};
Bmi270State g_bmi;

// Accel LSB/g and gyro LSB/(deg/s) for each range code (match bmi270_data.c).
// 各範囲コードの accel LSB/g と gyro LSB/(°/s)（bmi270_data.c と一致）。
float acc_scale(uint8_t range)
{
    switch (range) {
        case 0x00: return 16384.0f;  // ±2g
        case 0x01: return 8192.0f;   // ±4g
        case 0x02: return 4096.0f;   // ±8g
        case 0x03: return 2048.0f;   // ±16g
        default:   return 16384.0f;
    }
}
float gyr_scale(uint8_t range)
{
    switch (range) {
        case 0x00: return 16.4f;     // ±2000 dps
        case 0x01: return 32.8f;     // ±1000
        case 0x02: return 65.6f;     // ±500
        case 0x03: return 131.2f;    // ±250
        case 0x04: return 262.4f;    // ±125
        default:   return 16.4f;
    }
}

void pack16le(uint8_t* p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

int16_t sat16(float v)
{
    if (v > 32767.0f)  return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

// Fill 12 bytes [acc x/y/z, gyr x/y/z] (int16 LE) from the Plant body-FRD IMU,
// inverse-remapped to chip axes and scaled to raw by the configured range.
// imu_task remap is: body.x=chip.y, body.y=chip.x, body.z=-chip.z (accel & gyro).
// Plant body-FRD IMU から12バイトを生成。chip軸へ逆remap・範囲でraw化。
void bmi270_fill_data(uint8_t* dst)
{
    sf::ImuData imu = {};
    if (g_plant != nullptr) imu = g_plant->imu();

    const float as = acc_scale(g_bmi.regs[REG_ACC_RANGE]);
    const float gs = gyr_scale(g_bmi.regs[REG_GYR_RANGE]);

    // body → chip (accel in g): chip.x=body.y/G, chip.y=body.x/G, chip.z=-body.z/G
    pack16le(dst + 0, sat16((imu.accel[1] / kG) * as));   // acc chip X
    pack16le(dst + 2, sat16((imu.accel[0] / kG) * as));   // acc chip Y
    pack16le(dst + 4, sat16((-imu.accel[2] / kG) * as));  // acc chip Z
    // body → chip (gyro in deg/s): chip.x=body.y, chip.y=body.x, chip.z=-body.z
    pack16le(dst + 6,  sat16((imu.gyro[1] * kRad2Deg) * gs));   // gyr chip X
    pack16le(dst + 8,  sat16((imu.gyro[0] * kRad2Deg) * gs));   // gyr chip Y
    pack16le(dst + 10, sat16((-imu.gyro[2] * kRad2Deg) * gs));  // gyr chip Z
}

// One BMI270 SPI transaction. Returns 0 (ESP_OK-equivalent at the caller).
// BMI270 の1 SPIトランザクション。
int bmi270_xfer(const uint8_t* tx, uint8_t* rx, int n)
{
    if (n < 1) return 0;
    const uint8_t b0 = tx[0];
    const bool is_read = (b0 & 0x80) != 0;
    const uint8_t reg = b0 & 0x7F;

    if (is_read) {
        if (rx == nullptr) return 0;
        std::memset(rx, 0, (size_t)n);
        const int dn = n - 2;          // data bytes follow [addr, dummy]
        if (dn <= 0) return 0;
        uint8_t* d = rx + 2;
        if (reg == REG_ACC_X_LSB) {
            uint8_t buf[12];
            bmi270_fill_data(buf);
            for (int i = 0; i < dn && i < 12; ++i) d[i] = buf[i];
        } else {
            for (int i = 0; i < dn; ++i) {
                const uint8_t rr = (uint8_t)((reg + i) & 0x7F);
                uint8_t v;
                if (rr == REG_CHIP_ID)              v = CHIP_ID_VALUE;
                else if (rr == REG_INTERNAL_STATUS) v = 0x01;        // init OK
                else if (rr == REG_STATUS)          v = 0xC0;        // acc+gyr drdy
                else                                v = g_bmi.regs[rr];
                d[i] = v;
            }
        }
    } else {
        // write: reg = tx[0], data = tx[1..n-1]
        for (int i = 1; i < n; ++i) {
            const uint8_t rr = (uint8_t)((reg + (i - 1)) & 0x7F);
            g_bmi.regs[rr] = tx[i];
        }
    }
    return 0;
}

}  // namespace

// --- C-linkage hooks called by the ESP-IDF driver shims ----------------------
extern "C" {

void sil_board_attach_plant(void* plant)
{
    g_plant = static_cast<sil::Plant*>(plant);
}

void sil_board_step_plant(float dt_s)
{
    if (g_plant == nullptr) return;
    sf::MotorOutput cmd = {};
    for (int i = 0; i < 4; ++i) cmd.duty[i] = g_motor_duty[i];
    g_plant->setDuty(cmd);
    g_plant->step(dt_s);
}

int sil_board_spi_transfer(int cs, const uint8_t* tx, uint8_t* rx, size_t nbytes)
{
    // BMI270 is on CS GPIO46 (sf_board). Others (PMW3901 on 12) get zeros →
    // their driver init fails gracefully (Optional sensors) until E3.
    // BMI270 は CS GPIO46。他（PMW3901=12）はゼロ→ドライバが優雅に失敗（E3まで）。
    if (cs == 46 && tx != nullptr) {
        return bmi270_xfer(tx, rx, (int)nbytes);
    }
    if (rx != nullptr) std::memset(rx, 0, nbytes);
    return 0;
}

void sil_board_ledc_set_duty(int channel, uint32_t duty, uint32_t max_duty)
{
    // LEDC channels 0..3 map 1:1 to motors M1..M4 (sf_hal_motor MOTOR_CHANNELS).
    // LEDC channel 0..3 はモータ M1..M4 に 1:1（sf_hal_motor）。
    if (channel >= 0 && channel < 4) {
        const float scale = (max_duty > 0) ? (float)max_duty : 255.0f;
        float d = (float)duty / scale;
        if (d < 0.0f) d = 0.0f;
        if (d > 1.0f) d = 1.0f;
        g_motor_duty[channel] = d;
    }
}

}  // extern "C"
