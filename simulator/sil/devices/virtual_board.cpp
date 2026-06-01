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

// --- INA3221 power monitor (I2C addr 0x40) -----------------------------------
// 3-channel current/voltage monitor (TI). The firmware's power HAL reads a
// register by writing the 1-byte register pointer then reading 2 big-endian
// bytes (i2c_master_transmit_receive); it writes a register as [reg, hi, lo]
// (i2c_master_transmit). Battery voltage lives on the bus-voltage register of
// the configured channel (POWER_BATTERY_CHANNEL); we report the PLANT terminal
// voltage so the firmware's thrust→duty voltage compensation (duty = V/Vbat)
// closes the loop consistently. Without this the HAL read Vbat=0, the firmware's
// Vbat LPF decayed 3.7→0, duty saturated to 1.0 and the craft climbed away.
//
// INA3221 電源モニタ（I2C 0x40）。ファームの電源 HAL は「レジスタポインタ1バイト書込→
// 2バイト big-endian 読出」でレジスタを読む。電池電圧は設定チャンネルのバス電圧
// レジスタにある。Plant の端子電圧を返し、ファームの thrust→duty 電圧補償
// （duty = V/Vbat）の閉ループを整合させる。これが無いと Vbat=0→duty 飽和→暴走上昇。
constexpr uint16_t INA3221_ADDR       = 0x40;
constexpr uint16_t INA3221_MANUF_ID   = 0x5449;   // "TI"
constexpr uint16_t INA3221_DIE_ID     = 0x3220;
constexpr uint8_t  INA3221_REG_CONFIG = 0x00;
constexpr uint8_t  INA3221_REG_MANUF  = 0xFE;
constexpr uint8_t  INA3221_REG_DIE    = 0xFF;
constexpr float    INA3221_BUS_LSB_V  = 0.008f;   // 8 mV per bit, value in bits [15:3]

// INA3221 virtual chip state: the config register (read back after writes) and a
// register pointer for a bare receive that follows a 1-byte write.
// INA3221 仮想チップ状態: config レジスタ（書込後の読み戻し用）と、1バイト書込に
// 続く単独読出のためのレジスタポインタ。
struct Ina3221State {
    uint16_t config = 0x7127;   // datasheet power-on default (cosmetic; FW overwrites)
    uint8_t  ptr    = 0x00;     // last-addressed register
};
Ina3221State g_ina;

// Encode a battery voltage [V] into the INA3221 bus-voltage register layout:
// a 13-bit count of 8 mV steps left-shifted into bits [15:3] (bits [2:0] = 0).
// 電池電圧[V]を INA3221 バス電圧レジスタ形式へ（8mV刻み13bitを[15:3]に配置）。
uint16_t ina3221_bus_reg(float volts)
{
    long counts = lrintf(volts / INA3221_BUS_LSB_V);
    if (counts < 0)      counts = 0;
    if (counts > 0x1FFF) counts = 0x1FFF;
    return (uint16_t)((counts << 3) & 0xFFF8);
}

// 16-bit value of an INA3221 register. Bus-voltage regs (CH1/2/3 = 0x02/0x04/
// 0x06) report the Plant battery (any channel works → robust to the configured
// POWER_BATTERY_CHANNEL); shunt-voltage regs (0x01/0x03/0x05) read 0 (no current
// model yet); ID regs are constant; CONFIG reads back what was written.
// INA3221 レジスタの16bit値。バス電圧レジスタは Plant 電池を返す（全チャンネル対応）。
uint16_t ina3221_read_reg(uint8_t reg)
{
    switch (reg) {
        case INA3221_REG_MANUF:  return INA3221_MANUF_ID;
        case INA3221_REG_DIE:    return INA3221_DIE_ID;
        case INA3221_REG_CONFIG: return g_ina.config;
        case 0x02: case 0x04: case 0x06: {   // CH1/CH2/CH3 bus voltage
            const float v = (g_plant != nullptr) ? g_plant->batteryVoltage() : 3.7f;
            return ina3221_bus_reg(v);
        }
        case 0x01: case 0x05:                // CH1/CH3 shunt voltage (no current)
        case 0x03:                           // CH2 shunt voltage
            return 0x0000;
        default:
            return 0x0000;
    }
}

// One INA3221 I2C transaction. Wire format is big-endian (MSB first).
// INA3221 の1 I2Cトランザクション。バス上は big-endian（MSB 先頭）。
int ina3221_xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen)
{
    // Write phase: [reg] sets the pointer; [reg, hi, lo] also stores a 16-bit reg.
    // 書き込み相: [reg] でポインタ設定、[reg,hi,lo] なら16bit値も格納。
    if (wbuf != nullptr && wlen >= 1) {
        g_ina.ptr = wbuf[0];
        if (wlen >= 3 && g_ina.ptr == INA3221_REG_CONFIG) {
            // RST (bit 15) self-clears on the real chip → keep it clear here.
            // 実チップは RST(bit15) が自動クリア → ここでも落としておく。
            g_ina.config = (uint16_t)(((wbuf[1] << 8) | wbuf[2]) & 0x7FFF);
        }
    }
    // Read phase: 2 big-endian bytes of the addressed register (rest zero-filled).
    // 読み出し相: 対象レジスタの2バイトを big-endian で（残りはゼロ埋め）。
    if (rbuf != nullptr && rlen >= 1) {
        const uint16_t v = ina3221_read_reg(g_ina.ptr);
        rbuf[0] = (uint8_t)(v >> 8);
        if (rlen >= 2) rbuf[1] = (uint8_t)(v & 0xFF);
        for (size_t i = 2; i < rlen; ++i) rbuf[i] = 0;
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

int sil_board_i2c_xfer(uint16_t addr, const uint8_t* write_buf, size_t write_size,
                       uint8_t* read_buf, size_t read_size)
{
    // INA3221 power monitor on 0x40 → real register model (battery voltage).
    // Others (BMM150/BMP280/VL53L3CX) are unmodeled until E2: zero-fill the read
    // and ACK, exactly as the old inert shim did, so those Optional drivers fail
    // gracefully on their chip-ID check (no behaviour change for them).
    // INA3221 は 0x40 でレジスタモデルへ。他は未模型（E2まで）→read をゼロ埋めし ACK
    // （旧 inert シムと同一）。Optional ドライバは chip-ID チェックで優雅に失敗。
    if (addr == INA3221_ADDR) {
        return ina3221_xfer(write_buf, write_size, read_buf, read_size);
    }
    if (read_buf != nullptr && read_size > 0) std::memset(read_buf, 0, read_size);
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

void sil_board_get_motor_duty(float out[4])
{
    // Hand back the latched motor duties (M1..M4) for the trajectory recorder.
    // ラッチ済みのモータ duty（M1..M4）を軌跡レコーダへ返す。
    if (out == nullptr) return;
    for (int i = 0; i < 4; ++i) out[i] = g_motor_duty[i];
}

}  // extern "C"
