/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file plant_bridge.hpp
 * @brief One-way bridge from the SILS Plant to the sim sensor HALs.
 *        SILS Plant から sim センサ HAL への一方向ブリッジ。
 *
 * The closed-loop harness updates `current_imu` from Plant::imu() once per IMU
 * cycle and sets `has_plant = true`; the sim BMI270 wrapper then returns that
 * (instead of a static at-rest constant). This decouples the sim HAL from MuJoCo:
 * the HAL sees only a plain sf::ImuData, so it still links into rtos_smoke (which
 * has no MuJoCo). When has_plant is false the HAL keeps its P1.1 constant.
 *
 * 閉ループ harness が IMU サイクルごとに Plant::imu() で current_imu を更新し
 * has_plant=true にする。sim BMI270 ラッパはそれを返す（静止定数の代わりに）。これで
 * sim HAL を MuJoCo から切り離せる: HAL は素の sf::ImuData だけ見るので、MuJoCo を
 * 持たない rtos_smoke にも依然リンクできる。has_plant=false なら HAL は P1.1 定数を保つ。
 *
 * current_imu is in body-FRD, driver-normalized units (accel m/s², −9.8 at rest;
 * gyro rad/s) — the convention Plant::imu() produces. The sim BMI270 wrapper
 * inverse-remaps it to the chip frame so the (unchanged) imu_task remap restores
 * body-FRD (round-trip; removed once the 论点2 driver normalization lands).
 *
 * current_imu は機体 FRD・ドライバ正規化単位（加速度 m/s²・静止 −9.8、角速度 rad/s）で
 * Plant::imu() の規約。sim BMI270 ラッパがこれをチップ系へ逆 remap し、(無改変の)
 * imu_task remap が機体 FRD を復元する（往復。論点2 ドライバ正規化が入れば不要）。
 */

#pragma once

#include "data_types.hpp"  // sf::ImuData (header-only, no MuJoCo)

namespace sils {
namespace bridge {

/// true once the harness has wired a Plant; false → sim HAL uses its constant.
/// harness が Plant を配線したら true。false → sim HAL は定数を使う。
inline bool has_plant = false;

/// Latest synthetic IMU from the Plant (body-FRD, driver-normalized).
/// Plant からの最新合成 IMU（機体 FRD・ドライバ正規化）。
inline sf::ImuData current_imu = {};

}  // namespace bridge
}  // namespace sils
