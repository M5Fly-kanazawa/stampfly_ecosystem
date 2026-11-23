/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tof_histogram_merge.hpp
 * @brief Histogram-merge setting of the VL53L3CX ToF sensor (pure constants and checks, no ESP-IDF dependency)
 *        VL53L3CX ToF センサのヒストグラム併合の設定（定数と検査だけ。ESP-IDF に依存しない）
 *
 * The ST driver adds up the photon counts of the last (up to 6) frames of the same emission setting before it
 * computes a distance ("histogram merge", tuning parameter VL53LX_TUNINGPARM_HIST_MERGE, default 1 = on). Merging
 * lowers the noise but makes a slowly moving target read about 220 ms late and makes the reading jump by several
 * centimetres in one sample when the accumulation is discarded. Both ToF sensors (downward = height estimator, front =
 * log and obstacle distance) run with merging OFF by the user's decision (2026-11-23); the setting is held per sensor
 * (VL53L3CXWrapper::Config::histogram_merge_enabled).
 * ST のドライバは、距離を出す前に、同じ発光の設定の過去最大 6 フレームの光子の計数を足し合わせる（ヒストグラム併合。
 * 調整値 VL53LX_TUNINGPARM_HIST_MERGE、既定 1 = 有効）。併合は雑音を減らすが、ゆっくり動く対象では読みが約 220 ms
 * 古くなり、蓄積が捨てられる瞬間に読みが 1 標本で数 cm 跳ぶ。下向き（高さの推定）・前方（記録・障害物距離）の両方を、
 * 利用者の決定（2026-11-23）で併合なしで動かす。設定はセンサごとに持つ（VL53L3CXWrapper::Config::histogram_merge_enabled）。
 *
 * @design detailed_design.md §8 — Downward ToF histogram merge off                [OK]
 * @design hardware_init.md §5 — setting failure = bring-up failure (Recoverable)   [OK]
 */

#ifndef TOF_HISTOGRAM_MERGE_HPP
#define TOF_HISTOGRAM_MERGE_HPP

#include <cstdint>

namespace stampfly {
namespace tof_histogram_merge {

/// Value of VL53LX_TUNINGPARM_HIST_MERGE that switches merging off.
/// ヒストグラム併合を切る VL53LX_TUNINGPARM_HIST_MERGE の値。
constexpr int32_t kTuningValueOff = 0;

/// Value of VL53LX_TUNINGPARM_HIST_MERGE that switches merging on (the driver default).
/// ヒストグラム併合を有効にする VL53LX_TUNINGPARM_HIST_MERGE の値（ドライバの既定）。
constexpr int32_t kTuningValueOn = 1;

/// Tuning value for a merge setting.
/// 併合の設定に対応する調整値。
constexpr int32_t tuningValue(bool merge_enabled)
{
    return merge_enabled ? kTuningValueOn : kTuningValueOff;
}

/// True when the value read back from the driver is the one that was requested (the setting really took effect).
/// ドライバから読み戻した値が、要求した値と等しい（設定が実際に掛かった）とき true。
constexpr bool readbackMatches(int32_t requested_value, int32_t readback_value)
{
    return requested_value == readback_value;
}

}  // namespace tof_histogram_merge
}  // namespace stampfly

#endif  // TOF_HISTOGRAM_MERGE_HPP
