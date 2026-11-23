/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 */

/**
 * @file test_tof_histogram_merge.cpp
 * @brief Histogram-merge setting of the VL53L3CX (tof_histogram_merge.hpp) against the ST driver's own constants
 *        VL53L3CX のヒストグラム併合の設定（tof_histogram_merge.hpp）を、ST ドライバ自身の定数と照合する試験
 *
 * What is checked / 確認すること:
 *  1. The "on" value is the driver default (so a sensor left on its default keeps merging) and the "off" value is 0.
 *     「有効」の値がドライバの既定（既定のままのセンサは併合する）、「無効」の値が 0 であること。
 *  2. The driver's default merge depth fits the driver's record array (merging off leaves the depth untouched).
 *     ドライバの既定の併合の深さが記録配列に収まること（併合なしでは深さに触れない）。
 *  3. tuningValue() and readbackMatches() for every input (a failed read-back must be detected).
 *     tuningValue() と readbackMatches() の全入力での値（読み戻しの食い違いを検出できること）。
 *
 * @design detailed_design.md §8 — Downward ToF histogram merge off   [OK]
 */

#include <cstdint>
#include <cstdio>

// The ST headers test for a standard-integer header by name and warn when it cannot tell; <cstdint> is included.
// ST のヘッダは標準整数ヘッダの有無を名前で調べ、判別できないと警告する。ここでは <cstdint> を取り込み済み。
#ifndef STDINT_H
#define STDINT_H
#endif
#include "vl53lx_ll_def.h"
#include "vl53lx_tuning_parm_defaults.h"
#include "tof_histogram_merge.hpp"

namespace merge = stampfly::tof_histogram_merge;

// 1. On = driver default, off = 0 / 有効 = ドライバ既定、無効 = 0
static_assert(merge::kTuningValueOn == VL53LX_TUNINGPARM_HIST_MERGE_DEFAULT, "on must be the driver default");
static_assert(merge::kTuningValueOff == 0, "off must be 0");
static_assert(merge::kTuningValueOn != merge::kTuningValueOff, "on and off must differ");

// 2. Default depth fits the record array / 既定の深さが記録配列に収まる
static_assert(VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE_DEFAULT <= VL53LX_BIN_REC_SIZE, "default depth must fit the record array");
static_assert(VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE_DEFAULT >= 1, "default depth must be at least 1");

static int g_failures = 0;
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            ++g_failures; \
            printf("      FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

/// 3a. tuningValue maps the flag to the two driver values. / tuningValue がフラグを 2 つのドライバ値に写す。
static void testTuningValue()
{
    CHECK(merge::tuningValue(true) == merge::kTuningValueOn);
    CHECK(merge::tuningValue(false) == merge::kTuningValueOff);
}

/// 3b. readbackMatches accepts only the requested value. / readbackMatches は要求した値だけを受け入れる。
static void testReadbackMatches()
{
    CHECK(merge::readbackMatches(0, 0));
    CHECK(merge::readbackMatches(1, 1));
    CHECK(!merge::readbackMatches(0, 1));
    CHECK(!merge::readbackMatches(1, 0));
    CHECK(!merge::readbackMatches(0, -1));
    CHECK(!merge::readbackMatches(0, 15000));
}

int main()
{
    printf("=== ToF histogram merge setting unit tests ===\n");
    testTuningValue();
    testReadbackMatches();
    if (g_failures == 0) {
        printf("ALL PASS (plus the static_asserts against the ST driver headers)\n");
        return 0;
    }
    printf("%d FAILED\n", g_failures);
    return 1;
}
