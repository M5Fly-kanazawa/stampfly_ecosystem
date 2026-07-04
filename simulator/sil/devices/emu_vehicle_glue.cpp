/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file emu_vehicle_glue.cpp
 * @brief Host glue for the vehicle emulator target — overlays the firmware's
 *        OWN fused state estimate (ESKF) on the truth in the review-video trajectory.
 *        vehicle エミュレータ用 host glue — ファーム自身の状態推定（ESKF）を
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

#include <cstdio>          // std::fopen / fprintf / fflush
#include <cstdlib>         // std::getenv

#include "topics.hpp"     // sf::estimate_state
#include "data_types.hpp" // sf::StateEstimate
#include "sf_math.hpp"    // sf::math::Quat

namespace {

// --- POS_HOLD est_roll-bias diagnostic (env-gated, read-only) ---------------
// SIL_EMU_ESKF_DIAG=<path> dumps the firmware's published ESKF internals (accel
// bias, gyro bias, NED vel/pos, Euler estimate, sensor mask) at the trajectory
// cadence, one row per recorded trajectory sample (this hook is called exactly
// once per trajectory.csv row), so diag row N aligns with trajectory row N (the
// truth roll lives there). Lets us decide WHERE the steady -6° est_roll bias is
// born: H1 = accel_bias_y absorbs the steady tilt (ba_y ≈ g·sin(roll_err)), or
// H2 = thrust-contamination residual (est_roll biased while ba_y ≈ 0). Purely
// additive instrumentation: it READS the published estimate, no firmware change.
// SIL_EMU_ESKF_DIAG=<path> でファーム発行の ESKF 内部（加速度バイアス・ジャイロ
// バイアス・NED速度/位置・オイラー推定・センサマスク）を軌跡と同じ間引きで出力。
// このフックは軌跡1行につき必ず1回呼ばれるので diag 行 N = trajectory 行 N（真値
// roll はそちらにある）。定常 -6° est_roll バイアスの発生源（H1: accel_bias_y が
// 定常傾斜を吸収 / H2: 推力汚染残差）を切り分ける。発行トピックを読むだけの計装。
std::FILE* g_diag = nullptr;
bool       g_diag_tried = false;

std::FILE* diag_file()
{
    if (!g_diag_tried) {
        g_diag_tried = true;
        const char* path = std::getenv("SIL_EMU_ESKF_DIAG");
        if (path != nullptr && path[0] != '\0') {
            g_diag = std::fopen(path, "w");
            if (g_diag != nullptr) {
                std::fprintf(g_diag,
                    "t_us,est_roll,est_pitch,est_yaw,"
                    "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,"
                    "vn,ve,vd,pn,pe,pd,mask\n");
            }
        }
    }
    return g_diag;
}

// --- Scenario "api" channel registration -----------------------------------
// vehicle is the only target with an ApiTask; register its injection entry
// with the scenario engine at process start (a direct symbol reference inside
// scenario.cpp would break the vehicle_old emu link).
// vehicle だけが ApiTask を持つ。プロセス開始時に注入入口をシナリオエンジンへ
// 登録する（scenario.cpp からの直接参照は vehicle_old emu のリンクを壊す）。
}  // namespace

extern "C" void sil_scenario_register_api_inject(void (*fn)(const char*));
extern "C" void sf_api_inject_line(const char* line);

namespace {
struct ApiInjectRegistrar {
    ApiInjectRegistrar()
    {
        sil_scenario_register_api_inject(&sf_api_inject_line);
    }
};
ApiInjectRegistrar g_api_inject_registrar;
}  // namespace

extern "C" void sil_emu_estimate(float* alt_est, float* roll_est,
                                 float* pitch_est, float* yawcmd)
{
    // Snapshot the firmware's latest fused state (ESKF output).
    // ファーム最新の状態推定（ESKF 出力）をスナップショット。
    const sf::StateEstimate s = sf::estimate_state.latest();

    // Altitude (up positive) from NED position z. NED 位置 z から高度（上が正）。
    if (alt_est != nullptr) *alt_est = -s.position[2];

    // Roll / pitch / yaw from the attitude quaternion [w,x,y,z] → Euler.
    // 姿勢クォータニオン [w,x,y,z] → オイラー角でロール/ピッチ/ヨー。
    constexpr float kRad2Deg = 57.2957795131f;
    const sf::math::Quat q(s.attitude[0], s.attitude[1],
                           s.attitude[2], s.attitude[3]);
    const float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    sf::math::Vec3 e{0.0f, 0.0f, 0.0f};
    if (n2 > 1e-6f) {
        e = q.to_euler();                        // [rad] (x=roll, y=pitch, z=yaw)
        if (roll_est  != nullptr) *roll_est  = e.x * kRad2Deg;
        if (pitch_est != nullptr) *pitch_est = e.y * kRad2Deg;
    }

    // Diagnostic dump (env-gated). flush each row so _Exit(0) loses nothing.
    // 診断ダンプ（env ゲート）。_Exit(0) で取りこぼさぬよう各行 flush。
    std::FILE* d = diag_file();
    if (d != nullptr) {
        std::fprintf(d,
            "%u,%.4f,%.4f,%.4f,"
            "%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,"
            "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u\n",
            s.timestamp,
            e.x * kRad2Deg, e.y * kRad2Deg, e.z * kRad2Deg,
            s.accel_bias[0], s.accel_bias[1], s.accel_bias[2],
            s.gyro_bias[0], s.gyro_bias[1], s.gyro_bias[2],
            s.velocity[0], s.velocity[1], s.velocity[2],
            s.position[0], s.position[1], s.position[2],
            static_cast<unsigned>(s.sensor_mask));
        std::fflush(d);
    }

    // yawcmd: leave the recorder's seed (truth) untouched — not part of the estimate.
    // yawcmd: レコーダの初期値（真値）を保持＝推定の一部ではない。
    (void)yawcmd;
}
