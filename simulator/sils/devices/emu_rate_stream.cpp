/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_rate_stream.cpp
 * @brief 400Hz rate-loop recorder — implementation.
 *        400Hzレートループレコーダ — 実装。
 *
 * @design docs/architecture/simulation-policy.md §4 — model-match gate  [--]
 */

#include "emu_rate_stream.hpp"

#include <cstdio>
#include <cstdint>
#include <string>

#include "topics.hpp"      // sf::control_output (Latest, peek-only), sf::sensor_imu (Ring, peek-only)
#include "data_types.hpp"  // sf::ControlOutput, sf::ImuData
#include "params.hpp"      // sf::params::get_float — SSOT rate-loop gains

namespace {

std::FILE*  g_csv = nullptr;
std::string g_gains_path;   // "<path>.gains.json" sidecar, filled by sils_emu_rate_write_gains()
uint32_t    g_last_ts = 0;  // last recorded control_output.timestamp (edge detect); 0 = "not started"

// Rate-loop output torque limit [Nm] for roll/pitch. This is NOT a params-table
// entry: PidController::max_roll_pitch_torque_ (pid_controller.hpp) is a fixed
// compile-time default — unlike rate.yaw.max_torque, which the NT-Kanazawa
// saturation fix DID expose as a tunable param. Firmware is out of scope for this
// change (see task constraints), so this constant mirrors the compiled default
// rather than being read live; if the firmware ever exposes it as a param, switch
// this to sf::params::get_float() too. Flagged explicitly in the gate report.
// ロール/ピッチのレートループ出力トルク上限[Nm]。paramsテーブルの項目ではない —
// PidController::max_roll_pitch_torque_ は固定コンパイル時既定値（rate.yaw.max_torque
// とは異なりNVS調整不可）。本変更ではファームに触れない制約のため、コンパイル時既定値を
// そのまま写す（実測ではなく「ミラー」）。ファームが将来param化したらここも読み出しに
// 切り替える。ゲート報告で明示的に注記する設計判断。
constexpr float kRollPitchTorqueLimit = 5.2e-3f;  // [Nm] mirrors pid_controller.hpp:590

}  // namespace

extern "C" void sils_emu_rate_open(const char* path)
{
    if (path == nullptr || path[0] == '\0') return;
    g_csv = std::fopen(path, "w");
    if (g_csv == nullptr) return;
    // Exact column names tools/log_analyzer/rate_sysid.py's load_csv() expects
    // (rate_ref_<axis> + gyro_x/y/z; timestamp is read but optional there).
    // rate_sysid.py の load_csv() が期待する列名そのもの（timestamp は任意）。
    std::fprintf(g_csv, "timestamp,rate_ref_roll,rate_ref_pitch,rate_ref_yaw,gyro_x,gyro_y,gyro_z\n");
    g_gains_path = std::string(path) + ".gains.json";
    g_last_ts = 0;
}

extern "C" void sils_emu_rate_write_gains(void)
{
    if (g_csv == nullptr) return;   // recorder not open (SILS_EMU_RATE_STREAM unset) → no-op

    // Read the SSOT params exactly as PidController::loadParams() does (same
    // param names, params.cpp table). This is the gain set the run actually flew
    // (post any SILS_EMU_PARAMS_FILE override — call this AFTER that block).
    // PidController::loadParams() と同じパラメータ名で SSOT を読む（実際に飛んだ
    // ゲイン。SILS_EMU_PARAMS_FILE 上書き適用より後に呼ぶこと）。
    float roll_kp = 0.0f, roll_ti = 0.0f, roll_td = 0.0f;
    float pitch_kp = 0.0f, pitch_ti = 0.0f, pitch_td = 0.0f;
    float yaw_kp = 0.0f, yaw_ti = 0.0f, yaw_td = 0.0f, yaw_limit = 0.0f;
    sf::params::get_float("rate.roll.kp",  roll_kp);
    sf::params::get_float("rate.roll.ti",  roll_ti);
    sf::params::get_float("rate.roll.td",  roll_td);
    sf::params::get_float("rate.pitch.kp", pitch_kp);
    sf::params::get_float("rate.pitch.ti", pitch_ti);
    sf::params::get_float("rate.pitch.td", pitch_td);
    sf::params::get_float("rate.yaw.kp",   yaw_kp);
    sf::params::get_float("rate.yaw.ti",   yaw_ti);
    sf::params::get_float("rate.yaw.td",   yaw_td);
    sf::params::get_float("rate.yaw.max_torque", yaw_limit);

    std::FILE* gf = std::fopen(g_gains_path.c_str(), "w");
    if (gf == nullptr) return;
    // Hand-rolled JSON (no JSON library in the SILS host build — same convention
    // as emu_record.cpp's events.jsonl writer). Shape: {"<axis>": {kp,ti,td,limit}}.
    // JSON ライブラリ非依存の手書き出力（emu_record.cpp の events.jsonl と同じ流儀）。
    std::fprintf(gf,
        "{\n"
        "  \"roll\":  {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g},\n"
        "  \"pitch\": {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g},\n"
        "  \"yaw\":   {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g}\n"
        "}\n",
        (double)roll_kp,  (double)roll_ti,  (double)roll_td,  (double)kRollPitchTorqueLimit,
        (double)pitch_kp, (double)pitch_ti, (double)pitch_td, (double)kRollPitchTorqueLimit,
        (double)yaw_kp,   (double)yaw_ti,   (double)yaw_td,   (double)yaw_limit);
    std::fclose(gf);
}

extern "C" void sils_emu_rate_sample(void)
{
    if (g_csv == nullptr) return;

    // Peek (never pop) — sf::control_output is a Latest(1) topic, so .latest()
    // is always non-destructive; this is the same access pattern emu_trajectory
    // uses for sf::command_setpoint.
    // Peek のみ（消費しない）— sf::control_output は Latest(1) なので .latest() は
    // 常に非破壊。emu_trajectory の sf::command_setpoint と同じアクセス方法。
    const sf::ControlOutput ctrl = sf::control_output.latest();

    // Edge-detect a NEW control cycle by timestamp. 0 doubles as "no cycle has
    // completed yet" (the topic is zero-initialized before ControlTask's first
    // publish; the virtual clock only increases afterward, so 0 never recurs).
    // タイムスタンプでエッジ検出。0 は「まだ制御周期が1度も走っていない」を兼ねる
    // （ControlTask 初回発行前はゼロ初期化。仮想時計は単調増加なので以後 0 は再来しない）。
    if (ctrl.timestamp == 0 || ctrl.timestamp == g_last_ts) return;
    g_last_ts = ctrl.timestamp;

    // sf::sensor_imu is a RingBuffer(8) topic with a REAL single consumer (the
    // estimator). .latest() peeks the most recent sample without popping it —
    // draining it here with .read() would steal samples from the estimator and
    // corrupt the very run under test.
    // sf::sensor_imu は実消費者(推定器)を持つ RingBuffer(8)。.latest() は最新値を
    // 消費せず peek する — ここで .read() すると推定器のサンプルを横取りし被試験系
    // そのものを壊す。
    const sf::ImuData imu = sf::sensor_imu.latest();

    std::fprintf(g_csv, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                 (double)ctrl.timestamp * 1e-6,
                 (double)ctrl.rate_ref[0], (double)ctrl.rate_ref[1], (double)ctrl.rate_ref[2],
                 (double)imu.gyro[0], (double)imu.gyro[1], (double)imu.gyro[2]);
}

extern "C" void sils_emu_rate_close(void)
{
    if (g_csv != nullptr) {
        std::fclose(g_csv);
        g_csv = nullptr;
    }
}
