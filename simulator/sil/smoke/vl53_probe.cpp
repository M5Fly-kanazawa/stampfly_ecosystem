/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file smoke/vl53_probe.cpp
 * @brief Offline gen4 probe — drives the UNMODIFIED ST VL53L3CX BareDriver against
 *        our chip model (devices/vl53_device.cpp) directly on the host, with NO
 *        FreeRTOS / esp_timer / MuJoCo / scheduler. It runs the real init + ranging
 *        loop and prints, per frame, the full VL53LX_MultiRangingData_t AND the
 *        decoded VL53LX_histogram_bin_data_t (the exact input gen4 sees) plus the
 *        histogram post-process config. This is the fast iteration tool for
 *        Milestone 2: making gen4 decode our synthesized histogram into a range.
 *
 *        オフライン gen4 プローブ — 無改変 ST VL53L3CX ベアドライバを、ホスト上で
 *        チップモデル（devices/vl53_device.cpp）に直接ぶつけて駆動する（FreeRTOS /
 *        esp_timer / MuJoCo / スケジューラ無し）。実 init＋測距ループを回し、毎フレーム
 *        VL53LX_MultiRangingData_t 全体と、decoded VL53LX_histogram_bin_data_t（gen4 が
 *        実際に見る入力）＋ histogram 後処理 config を表示する。M2 の高速反復ツール。
 *
 *        The driver core (the vl53lx core .c files) is linked unmodified — Code
 *        Identity. Only the
 *        platform port (integrator-specific I2C/timer glue) is harness-local, routing
 *        reads/writes straight to the chip model. The firmware's vl53lx_platform.c
 *        (FreeRTOS/esp) is deliberately NOT linked.
 *        ドライバコアは無改変リンク（Code Identity）。プラットフォーム port（I2C/timer の
 *        継ぎ目）だけハーネスローカルでモデルへ直結。ファームの vl53lx_platform.c は不使用。
 *
 * Usage:  vl53_probe [target_mm] [frames] [step_mm]   (default 1000 mm, 6 frames, 0)
 *         step_mm adds a per-frame distance increment so the probe can reproduce a
 *         DYNAMIC (changing-distance) flight, not just a static stand-off. This is the
 *         切り分け tool for "does status=0 survive a moving target?"
 *         step_mm は毎フレーム距離を加算し、静止でなく「動的（距離変化）飛行」を再現する。
 *
 * @design simulator/sil/docs/vl53_m2_resume.md §3 Option B — offline gen4 harness
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "vl53lx_api.h"
#include "vl53lx_api_core.h"            // VL53LX_get_histogram_bin_data
#include "vl53lx_platform.h"            // platform fn prototypes (we implement them)
#include "vl53lx_platform_user_data.h"  // VL53LX_Dev_t, VL53LXDevStructGetLLDriverHandle
#include "vl53lx_ll_def.h"             // VL53LX_LLDriverData_t, histpostprocess
}

#include "vl53_device.hpp"  // sil_vl53 chip model

// =====================================================================================
// Harness platform port — the ONE integrator seam. Routes the ST driver's byte-level
// I2C straight to the chip model; timers are a deterministic virtual ms counter.
// ハーネスのプラットフォーム port。ST ドライバの I2C をモデルへ直結。タイマは決定論的な
// 仮想 ms カウンタ。
// =====================================================================================
extern "C" {

static uint32_t g_tick_ms = 0;  // monotonic virtual clock [ms] / 単調仮想時計

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t* dev, uint16_t index, uint8_t* pdata, uint32_t count)
{
    (void)dev;
    uint8_t buf[512];
    if (count + 2 > sizeof(buf)) return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
    buf[0] = (uint8_t)((index >> 8) & 0xFF);
    buf[1] = (uint8_t)(index & 0xFF);
    std::memcpy(buf + 2, pdata, count);
    sil_vl53::xfer(buf, count + 2, nullptr, 0);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t* dev, uint16_t index, uint8_t* pdata, uint32_t count)
{
    (void)dev;
    uint8_t reg[2] = { (uint8_t)((index >> 8) & 0xFF), (uint8_t)(index & 0xFF) };
    sil_vl53::xfer(reg, 2, pdata, count);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t* d, uint16_t i, uint8_t v) { return VL53LX_WriteMulti(d, i, &v, 1); }
VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t* d, uint16_t i, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    return VL53LX_WriteMulti(d, i, b, 2);
}
VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t* d, uint16_t i, uint32_t v)
{
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    return VL53LX_WriteMulti(d, i, b, 4);
}
VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t* d, uint16_t i, uint8_t* p) { return VL53LX_ReadMulti(d, i, p, 1); }
VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t* d, uint16_t i, uint16_t* p)
{
    uint8_t b[2];
    VL53LX_Error s = VL53LX_ReadMulti(d, i, b, 2);
    if (s == VL53LX_ERROR_NONE) *p = ((uint16_t)b[0] << 8) | b[1];
    return s;
}
VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t* d, uint16_t i, uint32_t* p)
{
    uint8_t b[4];
    VL53LX_Error s = VL53LX_ReadMulti(d, i, b, 4);
    if (s == VL53LX_ERROR_NONE)
        *p = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    return s;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t* d, int32_t us)
{
    (void)d;
    if (us < 0) return VL53LX_ERROR_INVALID_PARAMS;
    g_tick_ms += (uint32_t)((us + 999) / 1000);
    return VL53LX_ERROR_NONE;
}
VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t* d, int32_t ms)
{
    (void)d;
    if (ms < 0) return VL53LX_ERROR_INVALID_PARAMS;
    g_tick_ms += (uint32_t)ms;
    return VL53LX_ERROR_NONE;
}
VL53LX_Error VL53LX_GetTimerFrequency(int32_t* f) { *f = 1000000; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GetTimerValue(int32_t* c) { *c = (int32_t)(g_tick_ms * 1000); return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV d, uint32_t* ms) { (void)d; if (ms) *ms = g_tick_ms; return VL53LX_ERROR_NONE; }

VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t* d, uint8_t t, uint16_t s) { (void)d; (void)t; (void)s; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t* d) { (void)d; return VL53LX_ERROR_NONE; }

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t* d, uint32_t timeout_ms, uint16_t index,
                                    uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
    // The model returns a fixed ready state, so the masked value matches immediately.
    // モデルは固定の ready を返すので即マッチする。
    const uint32_t step = poll_delay_ms ? poll_delay_ms : 1;
    for (uint32_t t = 0; t <= timeout_ms; t += step) {
        uint8_t v = 0;
        VL53LX_RdByte(d, index, &v);
        if ((v & mask) == value) return VL53LX_ERROR_NONE;
        g_tick_ms += step;
    }
    return VL53LX_ERROR_TIME_OUT;
}

// GPIO seams are unused by the host histogram path — inert stubs.
// GPIO 継ぎ目は histogram 経路では未使用 — 無動作スタブ。
VL53LX_Error VL53LX_GpioSetMode(uint8_t a, uint8_t b) { (void)a; (void)b; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioSetValue(uint8_t a, uint8_t b) { (void)a; (void)b; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioGetValue(uint8_t a, uint8_t* b) { (void)a; if (b) *b = 0; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioXshutdown(uint8_t a) { (void)a; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioCommsSelect(uint8_t a) { (void)a; return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioPowerEnable(uint8_t a) { (void)a; return VL53LX_ERROR_NONE; }

}  // extern "C"

// =====================================================================================
// Diagnostics
// =====================================================================================

static void print_post_process_config(VL53LX_DEV dev)
{
    VL53LX_LLDriverData_t* pdev = VL53LXDevStructGetLLDriverHandle(dev);
    const VL53LX_hist_post_process_config_t* c = &pdev->histpostprocess;
    std::printf("--- histpostprocess config ---\n");
    std::printf("  signal_total_events_limit = %d\n", (int)c->signal_total_events_limit);
    std::printf("  min_ambient_thresh_events = %d\n", (int)c->min_ambient_thresh_events);
    std::printf("  ambient_thresh_sigma0/1   = %u / %u\n",
                (unsigned)c->ambient_thresh_sigma0, (unsigned)c->ambient_thresh_sigma1);
    std::printf("  noise_threshold           = %u\n", (unsigned)c->noise_threshold);
    std::printf("  sigma_thresh              = %u\n", (unsigned)c->sigma_thresh);
    std::printf("  valid_phase_low / high    = %u / %u\n",
                (unsigned)c->valid_phase_low, (unsigned)c->valid_phase_high);
    std::printf("  gain_factor               = %u\n", (unsigned)c->gain_factor);
    std::printf("  hist_target_order         = %u\n", (unsigned)c->hist_target_order);
    std::printf("  xtalk_compensation_enable = %u\n",
                (unsigned)c->algo__crosstalk_compensation_enable);
}

static void print_bin_data(VL53LX_DEV dev)
{
    // Decode the SAME 83-byte block the ranging loop processes, into the exact struct
    // gen4 consumes. This shows config-derived fields (number_of_ambient_bins,
    // total_periods_elapsed, vcsel_width, bin_seq) that come from RAM, not our bytes.
    // 測距ループが処理するのと同じ83バイトを、gen4 が食う構造体へ decode。RAM 由来の
    // config フィールド（number_of_ambient_bins 等）が見える。
    VL53LX_histogram_bin_data_t bd;
    std::memset(&bd, 0, sizeof(bd));
    VL53LX_Error s = VL53LX_get_histogram_bin_data(dev, &bd);
    std::printf("--- decoded histogram_bin_data (status=%d) ---\n", (int)s);
    std::printf("  number_of_bins(p_020/p_021) = %u / %u\n",
                (unsigned)bd.VL53LX_p_020, (unsigned)bd.VL53LX_p_021);
    std::printf("  number_of_ambient_bins      = %u   <== if >0, ambient-bin removal shifts bins\n",
                (unsigned)bd.number_of_ambient_bins);
    std::printf("  bin_seq[0..5]               = %u %u %u %u %u %u\n",
                bd.bin_seq[0], bd.bin_seq[1], bd.bin_seq[2], bd.bin_seq[3], bd.bin_seq[4], bd.bin_seq[5]);
    std::printf("  total_periods_elapsed       = %u\n", (unsigned)bd.total_periods_elapsed);
    std::printf("  vcsel_width                 = %u\n", (unsigned)bd.vcsel_width);
    std::printf("  VL53LX_p_005 (vcsel period) = %u\n", (unsigned)bd.VL53LX_p_005);
    std::printf("  VL53LX_p_015 (fast_osc)     = 0x%04X\n", (unsigned)bd.VL53LX_p_015);
    std::printf("  zero_distance_phase         = %u\n", (unsigned)bd.zero_distance_phase);
    std::printf("  dss_effective_spads         = %u\n",
                (unsigned)bd.result__dss_actual_effective_spads);
    std::printf("  result__range_status        = %u   result__stream_count = %u\n",
                (unsigned)bd.result__range_status, (unsigned)bd.result__stream_count);
    std::printf("  ambient_events_sum=%d  number_of_ambient_samples=%u  p_028=%d\n",
                (int)bd.ambient_events_sum, (unsigned)bd.number_of_ambient_samples, (int)bd.VL53LX_p_028);
    std::printf("  bin_data[0..23] =");
    for (int k = 0; k < VL53LX_HISTOGRAM_BUFFER_SIZE; ++k) std::printf(" %d", (int)bd.bin_data[k]);
    std::printf("\n");
}

// Dump the INTERNAL gen4 state AFTER the ranging pipeline: the post-merge histogram
// (pdev->hist_data — shows motion smearing) and each range object's decoded phase
// (VL53LX_p_011) + internal range_status (the device-error code BEFORE the public
// RangeStatus conversion). This exposes which gen4 check fails on a moving target.
// パイプライン後の gen4 内部状態を出す: merge 後 histogram（動き smear が見える）と
// 各オブジェクトの復号位相 p_011・内部 range_status（public 変換前のデバイスエラー）。
static void print_internal(VL53LX_DEV dev)
{
    VL53LX_LLDriverData_t*    pdev = VL53LXDevStructGetLLDriverHandle(dev);
    VL53LX_LLDriverResults_t* pres = VL53LXDevStructGetLLResultsHandle(dev);
    const VL53LX_histogram_bin_data_t& hd = pdev->hist_data;

    std::printf("   [internal] merged_zdp=%u stream=%u amb_bins=%u  merged_bins=",
                (unsigned)hd.zero_distance_phase, (unsigned)hd.result__stream_count,
                (unsigned)hd.number_of_ambient_bins);
    for (int k = 0; k < VL53LX_HISTOGRAM_BUFFER_SIZE; ++k) std::printf(" %d", (int)hd.bin_data[k]);
    std::printf("\n");

    const VL53LX_range_results_t& rr = pres->range_results;
    std::printf("   [internal] active_results=%u :", (unsigned)rr.active_results);
    for (int i = 0; i < rr.active_results && i < VL53LX_MAX_RANGE_RESULTS; ++i) {
        std::printf(" obj%d{p_011=%u dev_status=%u}", i,
                    (unsigned)rr.VL53LX_p_003[i].VL53LX_p_011,
                    (unsigned)rr.VL53LX_p_003[i].range_status);
    }
    std::printf("\n");
}

static void print_ranging(int frame, const VL53LX_MultiRangingData_t& r)
{
    std::printf("[frame %d] NumberOfObjectsFound=%u StreamCount=%u\n",
                frame, (unsigned)r.NumberOfObjectsFound, (unsigned)r.StreamCount);
    int n = r.NumberOfObjectsFound;
    if (n > VL53LX_MAX_RANGE_RESULTS) n = VL53LX_MAX_RANGE_RESULTS;
    for (int i = 0; i < n; ++i) {
        const VL53LX_TargetRangeData_t& t = r.RangeData[i];
        std::printf("   target[%d] status=%u range=%dmm signal=%.3fMcps ambient=%.3fMcps sigma=%.2fmm\n",
                    i, (unsigned)t.RangeStatus, (int)t.RangeMilliMeter,
                    t.SignalRateRtnMegaCps / 65536.0, t.AmbientRateRtnMegaCps / 65536.0,
                    t.SigmaMilliMeter / 65536.0);
    }
}

// =====================================================================================
// main — real init + ranging loop against the chip model.
// =====================================================================================
int main(int argc, char** argv)
{
    const float  target_mm = (argc > 1) ? (float)std::atof(argv[1]) : 1000.0f;
    const int    frames    = (argc > 2) ? std::atoi(argv[2]) : 6;
    const float  step_mm   = (argc > 3) ? (float)std::atof(argv[3]) : 0.0f;
    // hold: number of consecutive frames that share one distance sample (test the
    // "dual-config wrap pair sees one instant" hypothesis). hold=2 -> A/B pair holds.
    // hold: 同一距離を共有する連続フレーム数（デュアル設定 wrap 組が一瞬を見る仮説の検証）。
    const int    hold      = (argc > 4) ? std::atoi(argv[4]) : 1;

    std::printf("=== VL53L3CX offline gen4 probe ===\n");
    std::printf("target = %.1f mm, frames = %d, step = %.1f mm/frame "
                "(expect bin ~ %.1f, phase ~ %.0f)\n",
                target_mm, frames, step_mm, (target_mm / 192.5f), target_mm * 10.639f);

    VL53LX_Dev_t device;
    std::memset(&device, 0, sizeof(device));
    device.I2cDevAddr = (uint8_t)sil_vl53::ADDR_DEFAULT;

    sil_vl53::set_distance_mm(target_mm);

    VL53LX_Error st = VL53LX_WaitDeviceBooted(&device);
    std::printf("WaitDeviceBooted: %d\n", (int)st);

    st = VL53LX_DataInit(&device);
    std::printf("DataInit: %d\n", (int)st);

    st = VL53LX_SetDistanceMode(&device, VL53LX_DISTANCEMODE_LONG);
    std::printf("SetDistanceMode(LONG): %d\n", (int)st);

    st = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&device, 50000);
    std::printf("SetTimingBudget(50ms): %d\n", (int)st);

    print_post_process_config(&device);

    st = VL53LX_StartMeasurement(&device);
    std::printf("StartMeasurement: %d\n", (int)st);

    for (int f = 0; f < frames; ++f) {
        const int   hf     = (hold > 1) ? (f / hold) : f;     // hold distance across `hold` frames
        const float dist_f = target_mm + step_mm * (float)hf;  // dynamic ramp if step != 0
        sil_vl53::set_distance_mm(dist_f);

        uint8_t ready = 0;
        VL53LX_Error rs = VL53LX_GetMeasurementDataReady(&device, &ready);
        if (rs != VL53LX_ERROR_NONE) { std::printf("[frame %d] DataReady err %d\n", f, (int)rs); break; }

        if (f < 2) print_bin_data(&device);   // decode the gen4 input on the first frames

        VL53LX_MultiRangingData_t rd;
        std::memset(&rd, 0, sizeof(rd));
        VL53LX_Error gs = VL53LX_GetMultiRangingData(&device, &rd);
        if (gs != VL53LX_ERROR_NONE) { std::printf("[frame %d] GetMultiRangingData err %d\n", f, (int)gs); break; }

        print_ranging(f, rd);
        print_internal(&device);

        VL53LX_ClearInterruptAndStartMeasurement(&device);
    }

    std::printf("=== probe done ===\n");
    return 0;
}
