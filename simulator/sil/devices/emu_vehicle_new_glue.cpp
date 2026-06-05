/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file emu_vehicle_new_glue.cpp
 * @brief Host glue for the vehicle_new emulator target — overlays the firmware's
 *        OWN fused state estimate (ESKF) on the truth in the review-video trajectory.
 *        vehicle_new エミュレータ用 host glue — ファーム自身の状態推定（ESKF）を
 *        レビュー動画の真値軌跡に重ねる。
 *
 * The trajectory recorder (devices/emu_trajectory.cpp) exposes a WEAK no-op
 * sil_emu_estimate hook; defining it STRONG here makes the recorder log the ESKF
 * estimate (alt/roll/pitch) alongside truth, so est-vs-truth divergence is visible
 * in trajectory.csv and the review video. This is legitimate SIL instrumentation:
 * it READS the firmware's published estimate topic (sf::estimate_state) without
 * modifying any firmware source (Code Identity preserved).
 * 軌跡レコーダは弱い no-op の sil_emu_estimate フックを持つ。ここで強い定義を与えると
 * レコーダが ESKF 推定（高度/ロール/ピッチ）を真値と並べて記録し、est-vs-truth の
 * 乖離が trajectory.csv と動画で見える。ファーム発行トピック sf::estimate_state を
 * 読むだけでファームソースは無改変（Code Identity 保持）。
 *
 * Boot-safety: sf::estimate_state is a global Topic whose mutex is created at static
 * init (constructor), BEFORE the scheduler starts, so reading .latest() from the
 * recorder's on_advance context never dereferences a null mutex. While on_advance
 * runs the scheduler holds the single run-token, so no firmware task is mid-critical-
 * section holding the topic mutex — the take is uncontended.
 * 起動安全性: sf::estimate_state はグローバル Topic で mutex は静的初期化で生成される
 * （スケジューラ起動前）ため、on_advance から .latest() を読んでも null mutex を踏まない。
 * on_advance 実行中はスケジューラが単一実行トークンを保持し、どのタスクも mutex 保持中の
 * クリティカルセクションにいないので、take は競合しない。
 *
 * @design simulator/sil/RESET_PLAN.md §9 — reproducible review video (estimate overlay)  [--]
 */

#include "topics.hpp"     // sf::estimate_state
#include "data_types.hpp" // sf::StateEstimate
#include "sf_math.hpp"    // sf::math::Quat

extern "C" void sil_emu_estimate(float* alt_est, float* roll_est,
                                 float* pitch_est, float* yawcmd)
{
    // Snapshot the firmware's latest fused state (ESKF output).
    // ファーム最新の状態推定（ESKF 出力）をスナップショット。
    const sf::StateEstimate s = sf::estimate_state.latest();

    // Altitude (up positive) from NED position z. NED 位置 z から高度（上が正）。
    if (alt_est != nullptr) *alt_est = -s.position[2];

    // Roll / pitch from the attitude quaternion [w,x,y,z] → Euler.
    // 姿勢クォータニオン [w,x,y,z] → オイラー角でロール/ピッチ。
    const sf::math::Quat q(s.attitude[0], s.attitude[1],
                           s.attitude[2], s.attitude[3]);
    const float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (n2 > 1e-6f) {
        const sf::math::Vec3 e = q.to_euler();   // [rad] (x=roll, y=pitch, z=yaw)
        constexpr float kRad2Deg = 57.2957795131f;
        if (roll_est  != nullptr) *roll_est  = e.x * kRad2Deg;
        if (pitch_est != nullptr) *pitch_est = e.y * kRad2Deg;
    }

    // yawcmd: leave the recorder's seed (truth) untouched — not part of the estimate.
    // yawcmd: レコーダの初期値（真値）を保持＝推定の一部ではない。
    (void)yawcmd;
}
