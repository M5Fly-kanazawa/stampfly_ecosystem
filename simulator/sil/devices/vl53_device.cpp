/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file vl53_device.cpp
 * @brief VL53L3CX ToF (I2C addr 0x29 -> re-addressed to 0x30 bottom) chip model.
 *        ST BareDriver 1.2.14 in HISTOGRAM mode runs UNMODIFIED on the host.
 *        Unlike INA3221/BMP280 (8-bit register pointer), VL53 uses a 16-bit
 *        BIG-ENDIAN register index. The real driver boots, inits (reading NVM
 *        blocks we zero-fill -> it self-heals fast_osc to 0xBCCC), sets
 *        HISTOGRAM_LONG_RANGE, then in the ranging loop polls data-ready (0x0031,
 *        ACTIVE_LOW -> ready when bit0==0) and reads an 83-byte histogram block at
 *        0x0088 that the on-host gen4 pipeline decodes to a range. We synthesize
 *        that histogram from the down-range distance: a single peak whose
 *        center-of-mass phase encodes the range (range = 0.093994*phase, 1 bin =
 *        192.5 mm; vl53lx_core_support.c VL53LX_range_maths).
 *
 *        VL53L3CX ToF（I2C 0x29->bottom 0x30）。ST ベアドライバをホストで無改変稼働。
 *        16bit BE レジスタポインタ。実ドライバが boot/init（NVMはゼロ返しで fast_osc
 *        0xBCCC へ自己修復）->HISTOGRAM_LONG->測定ループで data-ready(0x0031,
 *        ACTIVE_LOW)をポーリングし 0x0088 の83バイト histogram を読む。それを下向き
 *        距離から合成（距離=0.094*位相, 1bin=192.5mm）。
 *
 * @design simulator/sil/RESET_PLAN.md §E2 — I2C device model (VL53L3CX)  [--]
 */

#include "vl53_device.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace sil_vl53 {
namespace {

// Verified register facts: 0x0088 size 83 (=0xDA-0x88+1), 0x00E5 boot bit0,
// 0x0031 ACTIVE_LOW data-ready, 0x00DE osc-cal must be non-zero.
// 検証済みレジスタ事実。
constexpr uint16_t REG_SYS_STATUS  = 0x00E5;  // FIRMWARE__SYSTEM_STATUS (boot gate)
constexpr uint16_t REG_GPIO_STAT   = 0x0031;  // GPIO__TIO_HV_STATUS (data-ready)
constexpr uint16_t REG_MODE_START  = 0x0087;  // SYSTEM__MODE_START (frame/measurement (re)start)
constexpr uint16_t REG_HIST        = 0x0088;  // RESULT__INTERRUPT_STATUS (histogram base)
constexpr uint16_t REG_OSC_CAL     = 0x00DE;  // RESULT__OSC_CALIBRATE_VAL (must be non-zero)
constexpr int      HIST_BYTES      = 83;      // 0x00DA - 0x0088 + 1

// Histogram synthesis constants. The runtime range slope (vl53lx_core_support.c
// VL53LX_range_maths with fast_osc=0xBCCC, gain=1987, offset=0) is 0.093994 mm per
// phase unit -> its inverse 10.639 phase units per mm. One bin = 2048 phase units =
// 192.5 mm. A clean single peak passes the gen4 sigma/signal gates.
// 距離傾き 0.093994 mm/位相 の逆数 10.639 位相/mm（1bin=192.5mm）。
constexpr float    PHASE_PER_MM = 10.639f;
constexpr uint32_t AMBIENT      = 40;     // ambient floor events / bin
constexpr uint32_t PEAK         = 2000;   // total peak amplitude over floor (split across 2 bins)

// Zero-distance phase. A real sensor's peak sits at phase = zdp + range*slope (the
// driver SUBTRACTS zdp to recover the distance). The driver computes zdp from the
// HISTOGRAM_LONG preset (vcsel period 9, cal_config__vcsel_start) in
// VL53LX_hist_calc_zero_distance_phase (vl53lx_core_support.c:165); with our
// phasecal_result__reference_phase=0 and vcsel_start=0 it lands at 22528 = 11 bins,
// deterministic and verified by smoke/vl53_probe.cpp. Synthesizing the peak at
// phase=0 (zdp=0 assumed) produced NEGATIVE ranges (status 4, OUT_OF_BOUNDS); we
// must offset by zdp so gen4's (peak_phase - zdp) is the true distance.
// 零距離位相。実センサのピークは phase = zdp + 距離*傾き に立つ（driver が zdp を引いて
// 距離を復元）。HISTOGRAM_LONG プリセットで zdp=22528=11bin（決定論的, プローブで検証）。
// zdp=0 と仮定して合成すると負レンジ（status 4）になるため zdp 分オフセットする。
constexpr float    ZERO_DIST_PHASE = 22528.0f;   // driver zdp for HISTOGRAM_LONG (= 11 bins)

// Ambient-bin strip. The HISTOGRAM_LONG preset's bin_seq is [7,0,1,2,3,4]; the 0x07
// code marks 4 ambient bins (number_of_ambient_bins=4), and the gen4 pipeline strips
// them from the FRONT of the histogram and left-shifts the bins BEFORE peak detection
// (VL53LX_hist_remove_ambient_bins, vl53lx_core_support.c:244). So a peak the driver
// should decode at OUTPUT bin Bo must be packed into the RAW buffer at Bo+4. Verified
// by smoke/vl53_probe.cpp (a raw-bin-5 peak decoded as output bin 1 → range -1828).
// 先頭 ambient bin の strip。HISTOGRAM_LONG の bin_seq=[7,0,1,2,3,4] で先頭4bin が
// ambient 扱い→ピーク検出前に左へ4シフトされる。よって driver が output bin Bo で復号
// すべきピークは RAW buffer の Bo+4 に置く（プローブで検証）。
constexpr int      AMBIENT_BIN_STRIP = 4;
constexpr float    MAX_MM            = 1400.0f;  // peak stays in usable output bins 11..18 (valid window)

struct State {
    uint16_t ptr        = 0;      // 16-bit register pointer (write-then-read)
    uint8_t  stream     = 0xFF;   // stream_count; first MODE_START write wraps it to 0
    float    pushed_mm  = -1.0f;  // board-pushed Plant distance [mm] (-1 = none)
};
State g_st;

// Resolve the target down-range distance [mm]: the env override SIL_VL53_TEST_MM
// (cached once) wins; otherwise the last board-pushed Plant distance; otherwise 0.
// 目標下向き距離[mm]: 環境変数 SIL_VL53_TEST_MM（一回キャッシュ）が優先、無ければ
// 直近にボードが渡した Plant 距離、無ければ 0。
float target_mm()
{
    static float override_mm = -1.0f;
    static bool  checked = false;
    if (!checked) {
        checked = true;
        const char* e = std::getenv("SIL_VL53_TEST_MM");
        if (e != nullptr && e[0] != '\0') override_mm = (float)std::atof(e);
    }
    if (override_mm >= 0.0f)    return override_mm;
    if (g_st.pushed_mm >= 0.0f) return g_st.pushed_mm;
    return 0.0f;
}

// Pack a 24-bit big-endian event count into 3 bytes at d (clamped to 0..0xFFFFFF).
// 24bit BE のイベント数を3バイトに詰める。
void pack_bin(uint8_t* d, uint32_t v)
{
    if (v > 0x00FFFFFFu) v = 0x00FFFFFFu;
    d[0] = (uint8_t)((v >> 16) & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
    d[2] = (uint8_t)(v & 0xFF);
}

// Build the 83-byte histogram block at 0x0088 (offset = absreg - 0x0088) from the
// target distance: an ambient floor + a sub-bin-resolved pulse whose gen4-decoded
// centroid phase encodes the range. A TWO-BIN split (b0 / b0+1, weighted by the
// fractional position) moves the centroid continuously, so the decoded range tracks
// the target to ~±17 mm (vs ±96 mm for a single symmetric peak at the nearest bin).
// range_status (off1)=0x09 (RANGECOMPLETE) avoids the RANGE_ERROR abort; the driver's
// RangeStatus upgrades to 0 (VALID) from ~frame 3 once the rolling/phase-consistency
// state settles (frames 0,1=6 NO_WRAP_CHECK, frame 2=4 transient, frame 3+=0).
// 83バイト histogram を構築。ambient floor＋サブbin分解パルス（gen4 復号重心が距離を符号化）。
// b0/b0+1 の2分割で重心を連続移動→復号レンジが ~±17mm で追従（単峰±96mm 比）。
void fill_histogram(uint8_t* d, size_t n)
{
    std::memset(d, 0, n);
    if (n < (size_t)HIST_BYTES) return;

    float dist_mm = target_mm();
    if (dist_mm < 0.0f) dist_mm = 0.0f;

    // Beyond the sensor's usable histogram range there is NO return pulse: a real
    // VL53L3CX reports "no target" (range_status != 0 / NumberOfObjectsFound 0), it
    // does NOT saturate to a constant distance reported as VALID. So out of range we
    // emit an ambient-ONLY histogram (no peak) — gen4 finds no object and the driver
    // flags the reading invalid, instead of feeding a bogus constant the ESKF would
    // trust as a valid altitude (which corrupts the vertical estimate — see
    // simulator/sil/docs/hover_resume.md / project_eskf_vertical_divergence).
    // 真の histogram レンジ超ではピーク無し: 実センサは no-target(status!=0)を返し、飽和定数を
    // valid と偽らない。ゆえにレンジ超では ambient のみ(ピーク無し)を出し gen4 に no-object 判定
    // させる(ESKF が偽の一定高度を信頼して鉛直推定が腐敗するのを防ぐ)。
    const bool out_of_range = (dist_mm > MAX_MM);

    uint32_t bins[24];
    for (int k = 0; k < 24; ++k) bins[k] = AMBIENT;

    if (!out_of_range) {
        // Phase a real sensor's return peak would sit at: zero-distance phase + range.
        // gen4 strips the 4 leading ambient bins and left-shifts, so it decodes the peak
        // at OUTPUT bin out_bin; pack the pulse into the RAW buffer at out_bin +
        // AMBIENT_BIN_STRIP so it survives the shift. fbin is the CONTINUOUS raw-bin
        // position of the true target phase (sub-bin resolved below).
        // 実センサの戻りピーク位相 = 零距離位相 + 距離。gen4 は先頭4bin を strip+左シフトして
        // output bin で復号するため RAW buffer には out_bin+4 に置く。fbin は連続的な raw bin 位置。
        const float peak_phase = ZERO_DIST_PHASE + dist_mm * PHASE_PER_MM;
        const float fbin = (peak_phase - 1024.0f) / 2048.0f + (float)AMBIENT_BIN_STRIP;

        // Sub-bin skew. A real return spreads across adjacent bins by the SPAD response;
        // gen4 recovers the phase as a centroid of the filtered pulse. We model the target
        // as a TWO-BIN split between b0 and b0+1 weighted by the fractional position frac,
        // so the decoded centroid tracks the distance CONTINUOUSLY (not just bin centers).
        // The surrounding ambient floor (below the gen4 detection threshold) is itself the
        // clean pulse edge — NO extra raised shoulders (they would distort the centroid and,
        // at frac≈0/1 where one split bin collapses to the floor, leave a 1-bin gap to the
        // shoulder that gen4 misreads as a wrap target).
        // サブbin skew。目標を b0/b0+1 に frac 比で2分割し復号重心を距離に連続追従させる。
        // 周囲の ambient floor（検出閾値以下）自体が pulse edge ＝外側 shoulder は付けない
        // （重心を歪め、frac≈0/1 の退化時に gap を生み wrap-target 誤判定を招くため）。
        int   b0   = (int)floorf(fbin);
        float frac = fbin - (float)b0;              // [0,1): centroid sits at b0 + frac
        if (b0 < AMBIENT_BIN_STRIP + 1) { b0 = AMBIENT_BIN_STRIP + 1; frac = 0.0f; }
        if (b0 > 22) { b0 = 22; frac = 0.0f; }      // need b0, b0+1 within bins 0..23

        const uint32_t main_lo = (uint32_t)lrintf((float)PEAK * (1.0f - frac));
        const uint32_t main_hi = (uint32_t)lrintf((float)PEAK * frac);

        bins[b0]     = AMBIENT + main_lo;       // two-bin split → centroid at b0 + frac
        bins[b0 + 1] = AMBIENT + main_hi;
    }

    d[0] = 0x03;                    // interrupt_status (cosmetic)
    d[1] = 0x09;                    // range_status = RANGECOMPLETE (no abort)
    d[2] = 0x00;                    // report_status
    d[3] = g_st.stream;             // stream_count (advances per frame)
    d[4] = 0x00; d[5] = 0xC0;       // dss_actual_effective_spads = 192 (BE)
    for (int k = 0; k < 24; ++k) pack_bin(d + 6 + 3 * k, bins[k]);
    d[78] = 0x00; d[79] = 0x00;     // phasecal_result__reference_phase = 0 (zdp → 22528)
    d[80] = 0x00;                   // phasecal_result__vcsel_start = 0
    const uint8_t bin23_lsb = (uint8_t)(bins[23] & 0xFF);
    d[81] = (uint8_t)(bin23_lsb >> 2);    // BIN_23_0_MSB: reproduce bin23 low byte
    d[82] = (uint8_t)(bin23_lsb & 0x03);  // BIN_23_0_LSB
}

}  // namespace

void set_distance_mm(float mm)
{
    g_st.pushed_mm = mm;
}

// One VL53 I2C transaction. 16-bit BE register pointer in wbuf[0..1].
// VL53 の1 I2Cトランザクション。16bit BE レジスタポインタは wbuf[0..1]。
int xfer(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen)
{
    if (wbuf != nullptr && wlen >= 2) {
        g_st.ptr = (uint16_t)((wbuf[0] << 8) | wbuf[1]);
    }
    // Write-only (transmit): config writes — ACK. A real sensor latches a NEW result
    // (incrementing result__stream_count) when a measurement (re)starts. The driver
    // (re)starts ranging via init_and_start_range, which writes ONE block ending at
    // SYSTEM__MODE_START (0x0087): StartMeasurement writes 0x0001..0x0088, each
    // ClearInterruptAndStartMeasurement writes 0x0044..0x0088 — both COVER 0x0087.
    // So advance stream_count on any write that covers MODE_START (a plain register-
    // pointer match fails because the pointer is the block's first, lower, address).
    // 書き込み(config): ACK。実センサは測定(再)開始時に新結果を latch し stream_count を進める。
    // driver は init_and_start_range で MODE_START(0x0087) を末尾に含むブロックを1回書く
    // ため、0x0087 を含む書き込みで stream_count を進める（先頭ポインタは下位アドレスなので
    // 完全一致では捕捉できない）。
    if (rbuf == nullptr) {
        const uint16_t write_end = (uint16_t)(g_st.ptr + (wlen >= 2 ? wlen - 2 : 0));
        if (g_st.ptr <= REG_MODE_START && REG_MODE_START < write_end) {
            g_st.stream = (uint8_t)((g_st.stream + 1) & 0xFF);
        }
        return 0;
    }
    // Read phase.
    std::memset(rbuf, 0, rlen);
    if (g_st.ptr == REG_SYS_STATUS) {
        rbuf[0] = 0x01;                                       // boot complete (bit0=1)
    } else if (g_st.ptr == REG_GPIO_STAT) {
        rbuf[0] = 0x00;                                       // data ready (ACTIVE_LOW: bit0=0)
    } else if (g_st.ptr == REG_OSC_CAL && rlen >= 2) {
        // RESULT__OSC_CALIBRATE_VAL (BE word) — MUST be non-zero or the driver's
        // set_inter_measurement_period_ms returns DIVISION_BY_ZERO during init. The
        // value only scales the (unused, back-to-back) inter-measurement timer.
        // 非ゼロ必須（0だと init で DIVISION_BY_ZERO）。back-to-back では未使用の計時係数。
        rbuf[0] = 0x06; rbuf[1] = 0x00;                       // 0x0600
    } else if (g_st.ptr == REG_HIST) {
        fill_histogram(rbuf, rlen);
    }
    // else: NVM/identity/config read-back → zeros (driver self-heals).
    return 0;
}

}  // namespace sil_vl53
