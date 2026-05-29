/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL — M11 regression challenge).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file old_eskf_check.cpp
 * @brief M11 link check — compile & run the LEGACY firmware/vehicle ESKF on host
 *        M11リンク確認 — 旧 firmware/vehicle のESKFをホストでコンパイル・実行
 *
 * Proves the legacy ESKF (active_mask type, stampfly:: namespace) links and
 * initializes on a PC via the compat shim, with the legacy source UNMODIFIED.
 * This is the foundation of the regression challenge: reproducing known legacy
 * bugs (P-matrix collapse, BA divergence) in the SIL to validate that the SIL
 * can actually catch them.
 *
 * 旧ESKF(active_mask型, stampfly:: 名前空間)が compat シム経由でPCにリンク・
 * 初期化できることを、旧コード無改変で実証する。回帰チャレンジ(既知の旧バグを
 * SILで再現し、SILが検出できることを実証する)の基盤。
 */

#include <cstdio>
#include "eskf.hpp"  // legacy ESKF (namespace stampfly)

using namespace stampfly;

int main()
{
    ESKF eskf;
    ESKF::Config cfg = {};  // zero-init; set the fields init() actually uses
    cfg.gyro_noise          = 0.01f;
    cfg.accel_noise         = 0.3f;
    cfg.gyro_bias_noise     = 1e-5f;
    cfg.accel_bias_noise    = 1e-4f;
    cfg.baro_noise          = 0.05f;
    cfg.tof_noise           = 0.01f;
    cfg.accel_att_noise     = 0.5f;
    cfg.gravity             = 9.81f;
    cfg.init_pos_std        = 0.1f;
    cfg.init_vel_std        = 0.1f;
    cfg.init_att_std        = 0.1f;
    cfg.init_gyro_bias_std  = 0.01f;
    cfg.init_accel_bias_std = 0.1f;

    esp_err_t rc = eskf.init(cfg);
    printf("legacy ESKF init: rc=%d initialized=%d active_mask=0x%04x\n",
           rc, (int)eskf.isInitialized(), eskf.getActiveMask());
    printf("pos_var=%.4f vel_var=%.4f\n",
           eskf.getPositionVariance(), eskf.getVelocityVariance());

    bool ok = (rc == ESP_OK) && eskf.isInitialized();
    printf("%s\n", ok ? "M11 link OK" : "M11 link FAIL");
    return ok ? 0 : 1;
}
