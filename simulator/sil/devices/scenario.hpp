/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario.hpp
 * @brief Declarative input-scenario loader + deterministic driver task.
 *        宣言的な入力シナリオのローダ＋決定論的ドライバタスク。
 *
 * A scenario is a line-based *.scn timeline of prepared inputs fed to the
 * UNMODIFIED firmware at chosen VIRTUAL times. It generalizes the scripted
 * virtual pilot: ESP-NOW ControlPackets (rc / rc_ramp), console bytes (key) and
 * — later — GPIO button / wind / fault. The whole file is parsed, validated and
 * frozen into a fixed sorted vector BEFORE the scheduler starts (no parsing or
 * host I/O during the run), so a run is fully deterministic and reproducible.
 *
 * The driver runs as a TASK holding the run-token (so injecting into the firmware
 * is a normal cooperative step), waiting on the virtual clock between events.
 *
 * シナリオは行ベースの *.scn タイムライン。無改変ファームへ仮想時刻で用意した入力を流す。
 * 仮想 pilot の一般化。起動前に全体をパース・検証して固定 vector に凍結（実行中は IO 無し）
 * ＝完全に決定論的・再現可能。ドライバは run-token 保持タスクとして仮想時計を待つ。
 *
 * Grammar (one event per line; '#' comment; blank ignored):
 *   <t> rc      <thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz]
 *   <t> rc_ramp <throttle|roll|pitch|yaw> <from> <to> <step> <rate_hz> <arm>
 *   <t> key     "<text with \n \t \r \\ \" escapes>"
 *   <t> btn     <down|up|click|hold_ms>            (deferred -> warn+skip)
 *   <t> wind    <fx> <fy> <fz> [dur_ms]            external force NED [N]
 *   <t> fault   <motor 0-3> <gain 0-1>             degrade one motor's thrust health
 *   <t> bias    <ax> <ay> <az> <gx> <gy> <gz>      deterministic raw IMU bias (FRD)
 *   <t> api     "<command line>"                  Tello-style API command into the
 *                                                  firmware ApiTask parser
 *   <t> handle  <carry_alt> <px> <py> <lift_ms> <carry_ms> <place_ms>
 *                                                  physical handling: lift→right→carry→place
 *                                                  (instantaneous trigger; Plant kinematic)
 * where <t> is either an absolute millisecond time, or '+<ms>' / '+' meaning
 * "<ms> after the previous event ends" (bare '+' = +0, i.e. immediately after).
 *
 * @design simulator/sil/RESET_PLAN.md §13 P8 — scripted input / scenario seam
 */

#pragma once

extern "C" {

// Parse + validate the scenario file. Returns:
//   1  = loaded a non-empty scenario (active),
//   0  = no scenario (path null/empty),
//  -1  = parse/validation error (message already printed to stderr with line no.)
// パース＋検証。1=有効なシナリオ, 0=シナリオ無し, -1=エラー（行番号付きで stderr 出力済）。
int sil_scenario_load(const char* path);

// True when a non-empty scenario is loaded.
// 有効なシナリオが読み込まれていれば true。
bool sil_scenario_active(void);

// Driver task: dispatches every event at its virtual time. Spawn only when
// sil_scenario_load() returned 1. Deletes itself when the timeline is exhausted.
// ドライバタスク: 各事象を仮想時刻で発火。load が 1 のときだけ起動。完了で自タスク削除。
void sil_scenario_driver_task(void* arg);

}  // extern "C"
