/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tello_state.hpp
 * @brief Tello SDK state-string builder — the ASCII telemetry that the DJI Tello
 *        pushes on UDP 8890 at ~10 Hz. djitellopy's connect() REQUIRES at least one
 *        such packet (else it raises), and every get_*() getter reads its fields, so
 *        emitting this string is what makes existing Tello Python programs run on
 *        StampFly. This header is the FORMAT (a pure function); the socket send and
 *        the topic reads live in the TelloStateTask (tasks/api_task.cpp).
 *        Tello SDK 状態文字列ビルダ — DJI Tello が UDP 8890 へ ~10Hz で送る ASCII
 *        テレメトリ。djitellopy の connect() はこのパケットが最低1個届くことを必須とし
 *        （無いと例外）、全 get_*() getter がそのフィールドを読む。これを送ることが
 *        既存 Tello Python プログラムを StampFly で動かす鍵。本ヘッダは「書式」（純粋
 *        関数）のみ。ソケット送信とトピック読みは TelloStateTask に置く。
 *
 * Wire form (SDK 2.0 superset, mission-pad fields prepended so every djitellopy
 * getter has a value and its int()/float() cast never throws on a missing key):
 *   mid:-2;x:0;y:0;z:0;mpry:0,0,0;pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy:%d;vgz:%d;
 *   templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f;time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n
 * mid:-2 means "mission-pad detection disabled" (an EDU-only feature we do not
 * implement) — a valid value, not an error.
 *
 * Dependency-free (cstddef/cstdio) so the FORMAT is verifiable by the host unit
 * test (test/test_main.cpp): the keys/order are asserted against what djitellopy
 * parses, without a socket or any firmware topic.
 * 依存フリー（cstddef/cstdio）で、書式をホスト単体テスト（test_main.cpp）で検証できる
 * — ソケットもファームのトピックも介さず、djitellopy がパースするキー/並びを assert する。
 *
 * @design requirements.md §7 — テレメトリ（外部向け）                  [--]
 * @design architecture.md §5 — API/テレメトリフロー                    [--]
 */

#pragma once

#include <cstddef>
#include <cstdio>

namespace sf {
namespace tello {

inline constexpr int kStatePort = 8890;   // client binds 0.0.0.0:8890 (Tello state)

/// One snapshot of the fields djitellopy reads. All physical-unit conversions
/// (rad→deg, m/s→cm/s, NED→body, m→cm, accel→milli-g) are done by the CALLER
/// (TelloStateTask), so this struct holds the exact integer/float values that go
/// on the wire and the builder is pure formatting.
/// djitellopy が読むフィールドの1スナップショット。物理単位変換（rad→deg, m/s→cm/s,
/// NED→機体系, m→cm, 加速度→ミリ g）は呼び出し側（TelloStateTask）が行うので、本構造体は
/// 電文に載る整数/浮動小数そのものを持ち、ビルダは純粋な書式整形に徹する。
struct TelloStateInputs {
    int   pitch = 0;    // attitude pitch [deg]     / ピッチ
    int   roll  = 0;    // attitude roll  [deg]     / ロール
    int   yaw   = 0;    // attitude yaw   [deg]     / ヨー
    int   vgx   = 0;    // ground speed x [cm/s] (body forward) / 機体前後速度
    int   vgy   = 0;    // ground speed y [cm/s] (body)         / 機体左右速度
    int   vgz   = 0;    // ground speed z [cm/s]                / 上下速度
    int   templ = 0;    // lowest  board temperature [°C] / 最低温度
    int   temph = 0;    // highest board temperature [°C] / 最高温度
    int   tof   = 0;    // time-of-flight distance [cm]   / ToF 距離
    int   h     = 0;    // height [cm]                    / 高度
    int   bat   = 0;    // battery [%]                    / バッテリー
    float baro  = 0.0f; // barometer altitude [cm] (djitellopy get_barometer ×100) / 気圧高度
    int   time_s = 0;   // motor-on / flight time [s]     / 飛行秒数
    float agx   = 0.0f; // acceleration x [0.001 g]       / 加速度
    float agy   = 0.0f; // acceleration y [0.001 g]
    float agz   = 0.0f; // acceleration z [0.001 g]
};

/// Format `in` into `buf` as the Tello state string. Returns the snprintf result
/// (bytes that WOULD be written, excluding the NUL) so the caller can detect
/// truncation and pass the length to sendto(). buf should be >= 256 bytes (the
/// full superset with worst-case widths is ~170 B; 256 leaves headroom).
/// `in` を Tello 状態文字列として `buf` に整形。snprintf の戻り値（NUL を除く本来の
/// 書き込みバイト数）を返すので、呼び出し側は切り詰めを検出し sendto() に長さを渡せる。
/// buf は 256 バイト以上を推奨（全フィールド最悪幅で ~170B、256 で余裕）。
inline int buildTelloState(char* buf, size_t n, const TelloStateInputs& in)
{
    return std::snprintf(
        buf, n,
        "mid:-2;x:0;y:0;z:0;mpry:0,0,0;"
        "pitch:%d;roll:%d;yaw:%d;"
        "vgx:%d;vgy:%d;vgz:%d;"
        "templ:%d;temph:%d;"
        "tof:%d;h:%d;bat:%d;baro:%.2f;time:%d;"
        "agx:%.2f;agy:%.2f;agz:%.2f;\r\n",
        in.pitch, in.roll, in.yaw,
        in.vgx, in.vgy, in.vgz,
        in.templ, in.temph,
        in.tof, in.h, in.bat, static_cast<double>(in.baro), in.time_s,
        static_cast<double>(in.agx), static_cast<double>(in.agy),
        static_cast<double>(in.agz));
}

}  // namespace tello
}  // namespace sf
