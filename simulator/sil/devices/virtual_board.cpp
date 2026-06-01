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
#include <cstdlib>
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

// --- BMP280 barometer (I2C addr 0x76) ----------------------------------------
// Bosch BMP280. The firmware driver reads chip-id 0xD0 (=0x58), a 24-byte
// little-endian calibration block at 0x88, and a 6-byte burst at 0xF7 (20-bit
// adc_P, adc_T, MSB-first), then applies the datasheet 32/64-bit compensation and
// the international barometric formula. We synthesize the raw registers so the
// UNMODIFIED driver reconstructs the Plant's pressure-altitude: load the canonical
// Bosch datasheet reference calibration, then per-read INVERT the (monotonic)
// 64-bit pressure compensation to the 20-bit adc_P that decodes to Plant pressure.
//
// BMP280 気圧計（I2C 0x76）。ドライバは chip-id(0xD0=0x58)・24バイトLE校正(0x88)・
// 6バイトburst(0xF7, 20bit adc_P/adc_T MSB先頭)を読み、データシートの32/64bit補償と
// 国際気圧式で高度化する。無改変ドライバが Plant の気圧高度を復元できるよう raw を合成:
// 正規の Bosch データシート参照校正を載せ、単調な64bit気圧補償を毎回逆算して
// Plant 気圧に復号される 20bit adc_P を求める。
constexpr uint16_t BMP280_ADDR          = 0x76;
constexpr uint8_t  BMP280_CHIP_ID_VALUE = 0x58;

// Canonical Bosch datasheet reference calibration (the worked-example constants,
// so the forward compensation is verifiable against the published example).
// 正規の Bosch データシート参照校正（worked-example 定数）。
constexpr uint16_t kDigT1 = 27504;
constexpr int16_t  kDigT2 = 26435, kDigT3 = -1000;
constexpr uint16_t kDigP1 = 36477;
constexpr int16_t  kDigP2 = -10685, kDigP3 = 3024, kDigP4 = 2855, kDigP5 = 140,
                   kDigP6 = -7, kDigP7 = 15500, kDigP8 = -14600, kDigP9 = 6000;
// Fixed raw temperature → ~25.09 °C (Plant baro temperature is 25 °C). Its t_fine
// also feeds pressure compensation, so the inversion must use the same t_fine.
// 固定の生温度 → 約25.09°C。その t_fine は気圧補償にも入るので逆算と一致させる。
constexpr int32_t kAdcT = 519888;

uint8_t g_bmp_ptr = 0;   // last addressed register (write-then-read pointer)

// Driver-identical temperature compensation → t_fine (datasheet 32-bit routine).
// ドライバと同一の温度補償 → t_fine。
int32_t bmp280_t_fine(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)kDigT1 << 1))) * (int32_t)kDigT2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)kDigT1) * ((adc_T >> 4) - (int32_t)kDigT1)) >> 12)
                    * (int32_t)kDigT3) >> 14;
    return var1 + var2;
}

// Driver-identical 64-bit pressure compensation → Q24.8 Pa (a copy of the firmware
// routine with the calibration above, so our inversion decodes EXACTLY as the
// firmware will). ドライバと同一の64bit気圧補償 → Q24.8 Pa（逆算が厳密に一致）。
uint32_t bmp280_press_q24_8(int32_t adc_P, int32_t t_fine)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)kDigP6;
    var2 = var2 + ((var1 * (int64_t)kDigP5) << 17);
    var2 = var2 + (((int64_t)kDigP4) << 35);
    var1 = ((var1 * var1 * (int64_t)kDigP3) >> 8) + ((var1 * (int64_t)kDigP2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)kDigP1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)kDigP9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)kDigP8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)kDigP7) << 4);
    return (uint32_t)p;
}

// Invert the (monotonically decreasing) pressure compensation: find the 20-bit
// adc_P whose decoded pressure ≈ target_pa. FIXED-iteration bisection over the full
// 20-bit range → deterministic (no tolerance early-exit), preserving SIL determinism.
// 単調減少の気圧補償を逆算: target_pa に復号される 20bit adc_P を固定回数二分探索で。
int32_t bmp280_invert_adc_p(float target_pa, int32_t t_fine)
{
    int32_t lo = 0, hi = 0x100000;            // 20-bit adc_P range
    for (int i = 0; i < 32; ++i) {
        const int32_t mid = lo + (hi - lo) / 2;
        const float pa = (float)bmp280_press_q24_8(mid, t_fine) / 256.0f;
        if (pa > target_pa) lo = mid;          // pressure falls as adc_P rises
        else                hi = mid;
    }
    return lo + (hi - lo) / 2;
}

// Fill the 24-byte calibration block at 0x88 (little-endian per 16-bit word).
// 0x88 の24バイト校正ブロックを LE で埋める。
void bmp280_fill_calib(uint8_t* d, size_t n)
{
    const uint16_t words[12] = {
        kDigT1, (uint16_t)kDigT2, (uint16_t)kDigT3,
        kDigP1, (uint16_t)kDigP2, (uint16_t)kDigP3, (uint16_t)kDigP4,
        (uint16_t)kDigP5, (uint16_t)kDigP6, (uint16_t)kDigP7,
        (uint16_t)kDigP8, (uint16_t)kDigP9 };
    for (size_t i = 0; i < n; ++i) {
        const size_t w = i / 2;
        if (w >= 12) { d[i] = 0; continue; }
        d[i] = (i & 1) ? (uint8_t)(words[w] >> 8) : (uint8_t)(words[w] & 0xFF);
    }
}

// Fill the 6-byte data burst at 0xF7 from the Plant pressure-altitude: invert the
// target pressure to adc_P (20-bit) and emit it + the fixed adc_T, MSB-first.
// 0xF7 の6バイトを Plant 気圧高度から: 目標気圧を adc_P に逆算し固定 adc_T と共に MSB先頭で。
void bmp280_fill_data(uint8_t* d, size_t n)
{
    sf::BaroData baro = {};
    if (g_plant != nullptr) baro = g_plant->baro();
    const float target_pa = (baro.pressure > 0.0f) ? baro.pressure : 101325.0f;
    const int32_t t_fine = bmp280_t_fine(kAdcT);
    const int32_t adc_P = bmp280_invert_adc_p(target_pa, t_fine);

    uint8_t buf[6];
    buf[0] = (uint8_t)((adc_P >> 12) & 0xFF);   // P[19:12]
    buf[1] = (uint8_t)((adc_P >> 4) & 0xFF);    // P[11:4]
    buf[2] = (uint8_t)((adc_P << 4) & 0xF0);    // P[3:0] in high nibble
    buf[3] = (uint8_t)((kAdcT >> 12) & 0xFF);   // T[19:12]
    buf[4] = (uint8_t)((kAdcT >> 4) & 0xFF);    // T[11:4]
    buf[5] = (uint8_t)((kAdcT << 4) & 0xF0);    // T[3:0] in high nibble
    for (size_t i = 0; i < n; ++i) d[i] = (i < 6) ? buf[i] : 0;
}

// One BMP280 I2C transaction (driver: read = write[reg] then read N bytes;
// write = write[reg, val]). BMP280 の1 I2Cトランザクション。
int bmp280_xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen)
{
    // Write-only (transmit): [reg, data] — reset/ctrl_meas/config. Latch ptr; ack.
    // 書き込みのみ: [reg,data]（reset/ctrl_meas/config）。ポインタ更新して ack。
    if (rbuf == nullptr) {
        if (wbuf != nullptr && wlen >= 1) g_bmp_ptr = wbuf[0];
        return 0;
    }
    // Read (transmit_receive): wbuf=[reg], rbuf=data[rlen].
    // 読み出し: wbuf=[reg], rbuf=data[rlen]。
    const uint8_t reg = (wbuf != nullptr && wlen >= 1) ? wbuf[0] : g_bmp_ptr;
    std::memset(rbuf, 0, rlen);
    if (reg == 0xD0)         rbuf[0] = BMP280_CHIP_ID_VALUE;  // CHIP_ID
    else if (reg == 0x88)    bmp280_fill_calib(rbuf, rlen);   // calibration block
    else if (reg == 0xF7)    bmp280_fill_data(rbuf, rlen);    // pressure+temp burst
    else if (reg == 0xF3)    rbuf[0] = 0x00;                  // STATUS: never busy
    // other regs (ctrl_meas/config read-back) → 0, harmless.
    return 0;
}

// --- VL53L3CX ToF (I2C addr 0x29 → re-addressed to 0x30 bottom) ---------------
// ST BareDriver 1.2.14 in HISTOGRAM mode runs UNMODIFIED on the host. Unlike
// INA3221/BMP280 (8-bit register pointer), VL53 uses a 16-bit BIG-ENDIAN register
// index. The real driver boots, inits (reading NVM blocks we zero-fill → it
// self-heals fast_osc to 0xBCCC), sets HISTOGRAM_LONG_RANGE, then in the ranging
// loop polls data-ready (0x0031, ACTIVE_LOW → ready when bit0==0) and reads an
// 83-byte histogram block at 0x0088 that the on-host gen4 pipeline decodes to a
// range. We synthesize that histogram from the Plant down-range distance: a single
// peak whose center-of-mass phase encodes the range (range = 0.093994·phase, 1 bin
// = 192.5 mm; vl53lx_core_support.c VL53LX_range_maths). Verified register facts:
// 0x0088 size 83 (=0xDA-0x88+1), 0x00E5 boot bit0, 0x0031 ACTIVE_LOW data-ready.
//
// VL53L3CX ToF（I2C 0x29→bottom 0x30）。ST ベアドライバをホストで無改変稼働。16bit
// BE レジスタポインタ。実ドライバが boot/init（NVMはゼロ返しで fast_osc 0xBCCC へ自己
// 修復）→HISTOGRAM_LONG→測定ループで data-ready(0x0031, ACTIVE_LOW)をポーリングし
// 0x0088 の83バイト histogram を読む。それを Plant の下向き距離から合成（距離=0.094·位相,
// 1bin=192.5mm）。
constexpr uint16_t VL53_ADDR_DEFAULT  = 0x29;   // power-on I2C address
constexpr uint16_t VL53_ADDR_BOTTOM   = 0x30;   // re-addressed bottom altimeter
constexpr uint16_t VL53_REG_SYS_STATUS = 0x00E5;  // FIRMWARE__SYSTEM_STATUS (boot gate)
constexpr uint16_t VL53_REG_GPIO_STAT  = 0x0031;  // GPIO__TIO_HV_STATUS (data-ready)
constexpr uint16_t VL53_REG_INT_CLEAR  = 0x0086;  // SYSTEM__INTERRUPT_CLEAR (frame advance)
constexpr uint16_t VL53_REG_HIST       = 0x0088;  // RESULT__INTERRUPT_STATUS (histogram base)
constexpr uint16_t VL53_REG_OSC_CAL    = 0x00DE;  // RESULT__OSC_CALIBRATE_VAL (must be non-zero)
constexpr int      VL53_HIST_BYTES     = 83;      // 0x00DA - 0x0088 + 1

struct Vl53State {
    uint16_t ptr    = 0;   // 16-bit register pointer (write-then-read)
    uint8_t  stream = 0;   // stream_count, advanced on interrupt-clear
};
Vl53State g_vl;

// Histogram synthesis constants. The runtime range slope (vl53lx_core_support.c
// VL53LX_range_maths with fast_osc=0xBCCC, gain=1987, offset=0, zdp=0) is
// 0.093994 mm per phase unit → its inverse 10.639 phase units per mm. One bin = 2048
// phase units = 192.5 mm. A clean single peak passes the gen4 sigma/signal gates.
// 距離傾き 0.093994 mm/位相 の逆数 10.639 位相/mm（1bin=192.5mm）。単峰でゲート通過。
constexpr float    VL53_PHASE_PER_MM = 10.639f;
constexpr uint32_t VL53_AMBIENT      = 40;     // ambient floor events / bin
constexpr uint32_t VL53_PEAK         = 2000;   // peak amplitude over floor
constexpr uint32_t VL53_SHOULDER     = 200;    // neighbor amplitude over floor (1-2 bin width)
constexpr float    VL53_MAX_MM       = 3200.0f;  // single-peak phase window cap (~3.27 m)

// Target down-range distance [mm]: the Plant ToF, or a test override (SIL_VL53_TEST_MM)
// for sweeping the histogram→range mapping without an airborne craft. Cached once.
// 目標下向き距離[mm]: Plant ToF、または検証用の SIL_VL53_TEST_MM 上書き（1回キャッシュ）。
float vl53_target_mm()
{
    static float override_mm = -1.0f;
    static bool  checked = false;
    if (!checked) {
        checked = true;
        const char* e = std::getenv("SIL_VL53_TEST_MM");
        if (e != nullptr && e[0] != '\0') override_mm = (float)std::atof(e);
    }
    if (override_mm >= 0.0f) return override_mm;
    sf::TofData t = {};
    if (g_plant != nullptr) t = g_plant->tof();
    return t.distance * 1000.0f;
}

// Pack a 24-bit big-endian event count into 3 bytes at d (clamped to 0..0xFFFFFF).
// 24bit BE のイベント数を3バイトに詰める。
void vl53_pack_bin(uint8_t* d, uint32_t v)
{
    if (v > 0x00FFFFFFu) v = 0x00FFFFFFu;
    d[0] = (uint8_t)((v >> 16) & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
    d[2] = (uint8_t)(v & 0xFF);
}

// Build the 83-byte histogram block at 0x0088 (offset = absreg - 0x0088) from the
// target distance: an ambient floor + a single peak whose center-of-mass phase
// encodes the range. Milestone 2a = SYMMETRIC peak at the nearest bin (±96 mm
// quantization); shoulder-skew for finer accuracy is added next. range_status
// (off1)=0x09 (RANGECOMPLETE) avoids the RANGE_ERROR abort; RangeStatus upgrades to
// 0 from frame 2 (phase-consistency) when the distance is stable.
// 83バイト histogram を目標距離から構築。ambient floor＋単峰（重心位相が距離を符号化）。
// 2a は最近傍 bin の対称ピーク（±96mm量子化）。skew は次段で。
void vl53_fill_histogram(uint8_t* d, size_t n)
{
    std::memset(d, 0, n);
    if (n < (size_t)VL53_HIST_BYTES) return;

    float dist_mm = vl53_target_mm();
    if (dist_mm < 0.0f)        dist_mm = 0.0f;
    if (dist_mm > VL53_MAX_MM) dist_mm = VL53_MAX_MM;
    const int32_t phase = (int32_t)lrintf(dist_mm * VL53_PHASE_PER_MM);   // target phase (zdp=0)
    // Nearest bin whose center phase (b0*2048+1024) matches the target; clamp to a
    // safe range (bin window cap, leave room for shoulders, avoid the bin-23 fixup).
    // 目標に最も近い中心位相を持つ bin（端は回避）。
    int b0 = (int)lrintf(((float)phase - 1024.0f) / 2048.0f);
    if (b0 < 1)  b0 = 1;
    if (b0 > 16) b0 = 16;

    uint32_t bins[24];
    for (int k = 0; k < 24; ++k) bins[k] = VL53_AMBIENT;
    bins[b0]     = VL53_AMBIENT + VL53_PEAK;
    bins[b0 - 1] = VL53_AMBIENT + VL53_SHOULDER;     // symmetric shoulders → COM at bin center
    bins[b0 + 1] = VL53_AMBIENT + VL53_SHOULDER;

    d[0] = 0x03;                    // interrupt_status (cosmetic)
    d[1] = 0x09;                    // range_status = RANGECOMPLETE (no abort)
    d[2] = 0x00;                    // report_status
    d[3] = g_vl.stream;             // stream_count (advances per frame)
    d[4] = 0x00; d[5] = 0xC0;       // dss_actual_effective_spads = 192 (BE)
    for (int k = 0; k < 24; ++k) vl53_pack_bin(d + 6 + 3 * k, bins[k]);
    d[78] = 0x00; d[79] = 0x00;     // phasecal_result__reference_phase = 0 → zdp 0
    d[80] = 0x00;                   // phasecal_result__vcsel_start = 0
    const uint8_t bin23_lsb = (uint8_t)(bins[23] & 0xFF);
    d[81] = (uint8_t)(bin23_lsb >> 2);    // BIN_23_0_MSB: reproduce bin23 low byte
    d[82] = (uint8_t)(bin23_lsb & 0x03);  // BIN_23_0_LSB
}

// One VL53 I2C transaction. 16-bit BE register pointer in write_buf[0..1].
// VL53 の1 I2Cトランザクション。16bit BE レジスタポインタは write_buf[0..1]。
int vl53_xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen)
{
    if (wbuf != nullptr && wlen >= 2) {
        g_vl.ptr = (uint16_t)((wbuf[0] << 8) | wbuf[1]);
    }
    // Write-only (transmit): config writes — ACK. Advance the frame on interrupt-clear.
    // 書き込みのみ（config）: ACK。interrupt-clear でフレームを進める。
    if (rbuf == nullptr) {
        if (g_vl.ptr == VL53_REG_INT_CLEAR) g_vl.stream++;
        return 0;
    }
    // Read phase.
    std::memset(rbuf, 0, rlen);
    if (g_vl.ptr == VL53_REG_SYS_STATUS) {
        rbuf[0] = 0x01;                                       // boot complete (bit0=1)
    } else if (g_vl.ptr == VL53_REG_GPIO_STAT) {
        rbuf[0] = 0x00;                                       // data ready (ACTIVE_LOW: bit0=0)
    } else if (g_vl.ptr == VL53_REG_OSC_CAL && rlen >= 2) {
        // RESULT__OSC_CALIBRATE_VAL (BE word) — MUST be non-zero or the driver's
        // set_inter_measurement_period_ms returns DIVISION_BY_ZERO during init. The
        // value only scales the (unused, back-to-back) inter-measurement timer.
        // 非ゼロ必須（0だと init で DIVISION_BY_ZERO）。back-to-back では未使用の計時係数。
        rbuf[0] = 0x06; rbuf[1] = 0x00;                       // 0x0600
    } else if (g_vl.ptr == VL53_REG_HIST) {
        vl53_fill_histogram(rbuf, rlen);
    }
    // else: NVM/identity/config read-back → zeros (driver self-heals).
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
    if (addr == BMP280_ADDR) {
        return bmp280_xfer(write_buf, write_size, read_buf, read_size);
    }
    // VL53L3CX bottom altimeter: power-on 0x29, then re-addressed to 0x30. Route both
    // (the model is stateless w.r.t. address; front 0x31 is left to the catch-all).
    // VL53L3CX 下向き測距: 起動時 0x29 → 0x30 へ再アドレス。両方を振り分け（front 0x31 は catch-all）。
    if (addr == VL53_ADDR_DEFAULT || addr == VL53_ADDR_BOTTOM) {
        return vl53_xfer(write_buf, write_size, read_buf, read_size);
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
