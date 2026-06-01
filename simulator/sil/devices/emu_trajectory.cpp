/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file emu_trajectory.cpp
 * @brief Review-video trajectory recorder — implementation.
 *        レビュー動画用の軌跡レコーダ — 実装。
 *
 * Truth pose (MuJoCo qpos + Plant truth) and motor duty are firmware-agnostic; the
 * estimate columns come from the weak sil_emu_estimate hook (a per-firmware glue
 * may override it). Column layout matches viz/render_video.py exactly.
 *
 * 真値姿勢（MuJoCo qpos＋Plant 真値）とモータ duty はファーム非依存、estimate 列は弱
 * フック sil_emu_estimate から（ファーム固有 glue が上書き可）。列は render_video.py と一致。
 *
 * @design simulator/sil/RESET_PLAN.md §9 — reproducible review video  [--]
 */

#include "emu_trajectory.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "plant.hpp"          // sil::Plant
#include "virtual_board.hpp"  // sil_board_get_motor_duty
#include "sf_math.hpp"        // sf::math::Quat / Vec3

// Per-firmware estimate hook with a WEAK no-op default. A firmware-specific glue may
// provide a STRONG definition to publish the firmware's own attitude/altitude
// estimate for the estimate-vs-truth overlay; with no override this default leaves
// the out-params untouched, so the recorder's truth seed stands (estimate ≡ truth).
// A weak DEFINITION (not just a weak declaration) is used so the reference always
// resolves on macOS/clang as well as ELF linkers.
// ファーム固有の推定フック＋弱い no-op 既定。ファーム固有 glue が強い定義で推定値を出せる。
// 上書きが無ければ何もせずレコーダの真値初期化が残る（estimate ≡ truth）。弱「定義」を使い
// macOS/clang でも ELF でも参照が必ず解決するようにする。
extern "C" __attribute__((weak))
void sil_emu_estimate(float* alt_est, float* roll_est, float* pitch_est, float* yawcmd)
{
    (void)alt_est; (void)roll_est; (void)pitch_est; (void)yawcmd;
}

namespace {

std::FILE* g_traj = nullptr;
int64_t    g_next_us = 0;                 // next virtual time to record / 次に記録する仮想時刻
constexpr int64_t kFramePeriodUs = 20000;  // 50 fps cadence / 50fps の周期
constexpr float   kRad2Deg = 57.2957795131f;

// Signed roll/pitch [deg] from a body→NED quaternion (same convention as the
// hover_smoke recorder, so emulator and hover_smoke videos read alike).
// body→NED クォータニオンから符号付きロール/ピッチ[deg]（hover_smoke と同じ規約）。
void roll_pitch_deg(const sf::math::Quat& q, float& roll_deg, float& pitch_deg)
{
    const float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (n2 < 1e-6f) { roll_deg = 0.0f; pitch_deg = 0.0f; return; }
    const sf::math::Vec3 e = q.to_euler();   // [rad] (x=roll, y=pitch, z=yaw)
    roll_deg  = e.x * kRad2Deg;
    pitch_deg = e.y * kRad2Deg;
}

}  // namespace

extern "C" void sil_emu_traj_open(const char* path)
{
    if (path == nullptr || path[0] == '\0') return;
    g_traj = std::fopen(path, "w");
    if (g_traj == nullptr) return;
    // Exact 20-column header render_video.py expects (truth + estimate + duty).
    // render_video.py が期待する厳密な20列ヘッダ（真値＋推定＋duty）。
    std::fprintf(g_traj,
        "t,px,py,pz,qw,qx,qy,qz,alt,roll,pitch,yawrate,yawcmd,"
        "alt_est,roll_est,pitch_est,m0,m1,m2,m3\n");
    g_next_us = 0;
}

extern "C" void sil_emu_traj_sample(double t_s, void* plant_ptr)
{
    if (g_traj == nullptr || plant_ptr == nullptr) return;

    // Decimate to ~50 fps on the virtual clock (deterministic — same t each run).
    // 仮想時計で約50fpsに間引き（決定論的＝毎回同じ t）。
    const int64_t now_us = (int64_t)std::llround(t_s * 1e6);
    if (now_us < g_next_us) return;
    g_next_us = now_us + kFramePeriodUs;

    sil::Plant* plant = static_cast<sil::Plant*>(plant_ptr);

    // Truth: MuJoCo qpos[0..6] = position + quaternion; Plant truth = NED/FRD.
    // 真値: MuJoCo qpos[0..6] = 位置＋姿勢、Plant 真値 = NED/FRD。
    const double* q = plant->data()->qpos;
    const sil::Plant::Truth tr = plant->truth();
    const float alt = -tr.pos_ned.z;
    float roll, pitch;
    roll_pitch_deg(tr.q_nb, roll, pitch);
    const float yawrate = tr.omega_frd.z;

    // Estimate columns: seed with truth, then let the per-firmware hook override the
    // channels it can observe (default/no hook → estimate mirrors truth).
    // 推定列: 真値で初期化し、ファーム固有フックが観測できる軸を上書き（無ければ真値を写す）。
    float alt_est = alt, roll_est = roll, pitch_est = pitch, yawcmd = 0.0f;
    sil_emu_estimate(&alt_est, &roll_est, &pitch_est, &yawcmd);   // weak: no-op default

    float duty[4];
    sil_board_get_motor_duty(duty);

    std::fprintf(g_traj,
        "%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        t_s, q[0], q[1], q[2], q[3], q[4], q[5], q[6],
        alt, roll, pitch, yawrate, yawcmd,
        alt_est, roll_est, pitch_est, duty[0], duty[1], duty[2], duty[3]);
}

extern "C" void sil_emu_traj_close(void)
{
    if (g_traj != nullptr) {
        std::fclose(g_traj);
        g_traj = nullptr;
    }
}
