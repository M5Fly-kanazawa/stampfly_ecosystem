/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_vehicle_glue.cpp
 * @brief Host glue for the vehicle emulator target — registers the ApiTask
 *        scripted-input injection entry point with the scenario engine.
 *        vehicle エミュレータ用 host glue — ApiTask のシナリオ注入入口を
 *        シナリオエンジンへ登録する。
 *
 * The estimate-vs-truth overlay this file used to provide (a STRONG
 * `sils_emu_estimate` override reading sf::estimate_state for
 * devices/emu_trajectory.cpp's review-video trajectory, plus a
 * SILS_EMU_ESKF_DIAG diagnostic dump) is superseded by the flight-log bundle
 * recorder (devices/emu_flightlog_vehicle.cpp): attitude.csv/posvel.csv now
 * carry every ESKF-published quantity (attitude, position, velocity, gyro/
 * accel bias) at the full 400Hz control rate, which is a strict superset of
 * what the old alt/roll/pitch-only overlay and diagnostic dump reported.
 * 本ファイルがかつて提供していた推定-対-真値オーバーレイ（sf::estimate_state を
 * 読む強い `sils_emu_estimate` 上書き、および SILS_EMU_ESKF_DIAG 診断ダンプ）は
 * フライトログ一式レコーダ（devices/emu_flightlog_vehicle.cpp）に統合された:
 * attitude.csv/posvel.csv が ESKF の発行する全量（姿勢・位置・速度・ジャイロ/
 * 加速度バイアス）を 400Hz フルレートで持ち、旧オーバーレイ（高度/ロール/
 * ピッチのみ）と診断ダンプの上位互換になっている。
 *
 * @design docs/plans/flight-log-format-plan.md §3.3 — SILS/vehicle unification [--]
 */

// --- Scenario "api" channel registration -----------------------------------
// vehicle is the only target with an ApiTask; register its injection entry
// with the scenario engine at process start (a direct symbol reference inside
// scenario.cpp would break the vehicle_old emu link).
// vehicle だけが ApiTask を持つ。プロセス開始時に注入入口をシナリオエンジンへ
// 登録する（scenario.cpp からの直接参照は vehicle_old emu のリンクを壊す）。

extern "C" void sils_scenario_register_api_inject(void (*fn)(const char*));
extern "C" void sf_api_inject_line(const char* line);

namespace {
struct ApiInjectRegistrar {
    ApiInjectRegistrar()
    {
        sils_scenario_register_api_inject(&sf_api_inject_line);
    }
};
ApiInjectRegistrar g_api_inject_registrar;
}  // namespace
