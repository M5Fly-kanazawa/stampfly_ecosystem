/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_flightlog.cpp
 * @brief Firmware-agnostic flight-log bundle writer — implementation.
 *        ファーム非依存のフライトログ一式ライタ — 実装。
 *
 * @design docs/plans/flight-log-format-plan.md §3.3 — SILS/vehicle unification [--]
 */

#include "emu_flightlog.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>   // mkdir — create the bundle directory if missing

#include "plant.hpp"    // sils::Plant (opaque pointer in the public API)

namespace {

// Fixed-size stream table (name -> lazily-opened FILE*). 20 covers every
// stream protocol/spec/flight_log.yaml defines today (imu/attitude/posvel/
// rate_ref/motor/ctrl_output/pilot/ctrl_ref/baro/tof_bottom/tof_front/flow/
// mag/status/eskf_cov/truth/events) with headroom.
// 固定サイズのストリーム表（名前→遅延オープンした FILE*）。20 は
// flight_log.yaml が現在定義する全ストリームに余裕を持って収まる数。
constexpr int kMaxStreams = 20;

// truth.csv virtual-clock cadence: 2500us = 400Hz, matching the vehicle's own
// control-cycle rate so a SILS run and a real flight carry comparable row
// density.
// truth.csv の仮想クロック周期: 2500us=400Hz。vehicle 自身の制御周期と揃え、
// SILS 実行と実飛行のログを近い密度にする。
constexpr int64_t kTruthPeriodUs = 2500;

struct StreamSlot {
    char name[32] = {0};
    std::FILE* file = nullptr;
};

char       g_dir[512] = {0};     // empty = closed (no-op discipline) / 空=未オープン
StreamSlot g_streams[kMaxStreams];
int        g_stream_count = 0;
int64_t    g_truth_next_us = 0;  // next virtual time due for a truth.csv row

}  // namespace

extern "C" void sils_emu_flightlog_open(const char* dir)
{
    if (dir == nullptr || dir[0] == '\0') { g_dir[0] = '\0'; return; }
    std::snprintf(g_dir, sizeof(g_dir), "%s", dir);
    // Best-effort create; EEXIST (already there) is not an error. Nested
    // parents are not our job -- the sf CLI creates the parent directory.
    // ベストエフォートで作成。既存(EEXIST)はエラーにしない。親ディレクトリの
    // 再帰作成はここの責務ではない -- sf CLI が親を作る。
    mkdir(g_dir, 0755);
    g_stream_count = 0;
    g_truth_next_us = 0;
}

extern "C" std::FILE* sils_emu_flightlog_stream(const char* name, const char* header)
{
    if (g_dir[0] == '\0') return nullptr;   // recorder closed -- no-op

    for (int i = 0; i < g_stream_count; ++i) {
        if (std::strcmp(g_streams[i].name, name) == 0) return g_streams[i].file;
    }
    if (g_stream_count >= kMaxStreams) return nullptr;   // table exhausted (defensive)

    char path[600];
    std::snprintf(path, sizeof(path), "%s/%s.csv", g_dir, name);
    std::FILE* f = std::fopen(path, "w");
    if (f == nullptr) return nullptr;
    std::fprintf(f, "%s\n", header);

    StreamSlot& slot = g_streams[g_stream_count++];
    std::snprintf(slot.name, sizeof(slot.name), "%s", name);
    slot.file = f;
    return f;
}

// Weak no-op defaults -- a per-firmware glue (emu_flightlog_vehicle.cpp)
// overrides one or both with a STRONG definition. Weak DEFINITIONS (not just
// declarations) so the reference always resolves on macOS/clang as well as
// ELF linkers (same trick emu_trajectory.cpp used for sils_emu_estimate).
// 弱い no-op 既定 -- ファーム固有 glue が強い定義で上書きする。弱「定義」
// （宣言だけでない）にすることで macOS/clang でも ELF でも参照が必ず解決する
// （emu_trajectory.cpp が sils_emu_estimate に使ったのと同じ手法）。
extern "C" __attribute__((weak))
void sils_emu_flightlog_firmware_sample(int64_t now_us) { (void)now_us; }

extern "C" __attribute__((weak))
void sils_emu_flightlog_write_gains(void) {}

namespace {

// Write one truth.csv row from the Plant's ground-truth pose/rate. Firmware-
// agnostic: reads only the opaque sils::Plant, never a firmware topic.
// Plant の真値姿勢/速度/角速度から truth.csv を1行書く。ファーム非依存 --
// 不透明な sils::Plant のみを読み、ファームのトピックには触れない。
void write_truth_row(int64_t now_us, const sils::Plant& plant)
{
    std::FILE* f = sils_emu_flightlog_stream("truth",
        "timestamp_us,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
        "vel_x,vel_y,vel_z,rate_x,rate_y,rate_z");
    if (f == nullptr) return;

    const sils::Plant::Truth tr = plant.truth();
    std::fprintf(f,
        "%lld,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
        (long long)now_us,
        (double)tr.pos_ned.x, (double)tr.pos_ned.y, (double)tr.pos_ned.z,
        (double)tr.q_nb.w, (double)tr.q_nb.x, (double)tr.q_nb.y, (double)tr.q_nb.z,
        (double)tr.vel_ned.x, (double)tr.vel_ned.y, (double)tr.vel_ned.z,
        (double)tr.omega_frd.x, (double)tr.omega_frd.y, (double)tr.omega_frd.z);
}

}  // namespace

extern "C" void sils_emu_flightlog_sample(int64_t now_us, const void* plant_ptr)
{
    if (g_dir[0] == '\0') return;   // recorder closed -- no-op

    if (plant_ptr != nullptr && now_us >= g_truth_next_us) {
        g_truth_next_us = now_us + kTruthPeriodUs;
        write_truth_row(now_us, *static_cast<const sils::Plant*>(plant_ptr));
    }

    // Firmware-specific streams (imu/attitude/posvel/... ) -- no-op unless a
    // glue overrides this weak hook.
    // ファーム固有ストリーム -- glue が弱フックを上書きしない限り no-op。
    sils_emu_flightlog_firmware_sample(now_us);
}

extern "C" void sils_emu_flightlog_close(void)
{
    for (int i = 0; i < g_stream_count; ++i) {
        if (g_streams[i].file != nullptr) {
            std::fflush(g_streams[i].file);
            std::fclose(g_streams[i].file);
            g_streams[i].file = nullptr;
        }
    }
    g_stream_count = 0;
    g_dir[0] = '\0';
}
