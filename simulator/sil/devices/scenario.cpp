/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario.cpp
 * @brief Input-scenario parser + deterministic driver task implementation.
 *        入力シナリオのパーサ＋決定論的ドライバタスクの実装。
 */

#include "scenario.hpp"
#include "scenario_inject.hpp"
#include "console_feeder.hpp"
#include "emu_record.hpp"
#include "virtual_board.hpp"     // P7: sil_board_set_wind / sil_board_set_motor_health

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

enum class Channel { Rc, RcRamp, Key, Btn, Wind, Fault, Bias, Handle };

struct Event {
    int64_t at_us = 0;        // absolute virtual time (frozen) / 絶対仮想時刻
    Channel ch = Channel::Rc;
    int     line = 0;         // source line (diagnostics) / ソース行

    // rc
    uint16_t thr = 0, roll = 0, pitch = 0, yaw = 0;
    uint8_t  arm = 0;
    uint8_t  alt = 0;         // 1 => CTRL_FLAG_ALT_MODE (ALTITUDE_HOLD) / 1 で高度保持
    uint8_t  acro = 0;        // 1 => CTRL_FLAG_MODE (ACRO/rate) / 1 で ACRO（角速度）
    uint8_t  pos = 0;         // 1 => CTRL_FLAG_POS_MODE (POSITION_HOLD) / 1 で位置保持
    int      hold_ms = 0;     // 0 = single frame / 0=単発
    int      rate_hz = 20;

    // rc_ramp
    int      ramp_field = 0;  // 0=throttle 1=roll 2=pitch 3=yaw
    uint16_t ramp_from = 0, ramp_to = 0;
    int      ramp_step = 1;

    // key
    std::string text;

    // wind (P7): external force NED [N]; dur_ms>0 = gust pulse (revert to 0 after).
    // wind (P7): NED 外乱力 [N]。dur_ms>0 で突風パルス（後で 0 に戻す）。
    float wind_fx = 0.0f, wind_fy = 0.0f, wind_fz = 0.0f;
    int   wind_dur_ms = 0;

    // fault (P7): degrade one motor's thrust health gain (motor 0..3, gain 0..1).
    // fault (P7): 1モータの推力健全度ゲインを劣化（motor 0..3, gain 0..1）。
    int   fault_motor = 0;
    float fault_gain = 1.0f;

    // bias (P2-3): deterministic raw IMU bias (body FRD), accel [m/s²] + gyro [rad/s].
    // bias (P2-3): 決定論的な生 IMU バイアス（機体 FRD）、accel[m/s²] + gyro[rad/s]。
    float bias_ax = 0.0f, bias_ay = 0.0f, bias_az = 0.0f;
    float bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;

    // handle (P8): physical handling maneuver — carry altitude [m up], placement
    // x/y [NED m], and lift/carry/place phase durations [ms]. Instantaneous on the
    // timeline (triggers the Plant trajectory; surrounding disarmed rc holds keep the
    // link alive and occupy the maneuver's wall-clock).
    // handle (P8): 物理ハンドリング動作 — carry 高度[上方m]、設置 x/y[NED m]、lift/carry/
    // place 各相の所要時間[ms]。タイムライン上は瞬時（Plant 軌道を起動。周囲の disarmed rc
    // ホールドがリンクを維持し動作の実時間を占有する）。
    float handle_carry_alt = 0.40f;
    float handle_place_x = 0.0f, handle_place_y = 0.0f;
    int   handle_lift_ms = 1500, handle_carry_ms = 1500, handle_place_ms = 1500;

    // btn (still deferred): keep raw args for the warn note.
    // btn（未実装）: warn 用に生引数を保持。
    std::string raw;
};

std::vector<Event> g_events;
bool g_active = false;

// period (ms) for a given send rate, integer-floored (1 tick = 1 ms). The parser
// validates rate_hz in [1,1000] so period is always >= 1 (no vTaskDelay(0)).
// 送信周期(ms)。パーサが rate_hz を [1,1000] に制限するので period>=1（vTaskDelay(0)なし）。
int period_ms(int rate_hz) { return rate_hz > 0 ? (1000 / rate_hz) : 50; }

// Frames the driver emits for an rc hold: at least one for any positive hold,
// rounded to the nearest send period (hold_ms is quantized to the period). The
// 64-bit product guards against overflow. ONE helper shared by the parser
// (timeline duration) and the driver (emit loop) so '+'-relative timing matches
// the bytes actually sent — and a sub-period hold can never emit zero frames.
// rc ホールドの発射フレーム数: 正のホールドは最低1、周期最近接に丸め（64bitで桁あふれ回避）。
// パーサ（時間幅）とドライバ（発射ループ）が同一ヘルパを共有 → '+' 相対時刻が実送出と一致。
long long rc_hold_frames(int hold_ms, int rate_hz)
{
    if (hold_ms <= 0) return 0;
    long long f = ((long long)hold_ms * rate_hz + 500) / 1000;  // round to nearest
    return f < 1 ? 1 : f;
}

int ramp_frames(uint16_t from, uint16_t to, int step)
{
    int s = step < 0 ? -step : step;
    if (s == 0) return 1;
    int span = (to >= from) ? (to - from) : (from - to);
    return span / s + 1;
}

// Duration the event occupies on the timeline (for '+'-relative resolution).
// For rc this is EXACTLY what the driver spends (frames * period), so the parser
// and driver agree even when rate_hz does not divide 1000.
// 事象がタイムライン上で占める時間。rc はドライバの実消費（frames*period）と厳密一致。
int64_t event_duration_us(const Event& e)
{
    switch (e.ch) {
        case Channel::Rc:
            return rc_hold_frames(e.hold_ms, e.rate_hz)
                 * (int64_t)period_ms(e.rate_hz) * 1000;
        case Channel::RcRamp:
            return (int64_t)ramp_frames(e.ramp_from, e.ramp_to, e.ramp_step)
                 * period_ms(e.rate_hz) * 1000;
        case Channel::Wind:
            // A gust pulse (dur_ms>0) holds the wind then reverts, so it occupies
            // dur_ms on the timeline; a sustained step (dur_ms=0) is instantaneous.
            // 突風パルス(dur_ms>0)は風を保持後に戻すので dur_ms 占有。定常(0)は瞬時。
            return (int64_t)e.wind_dur_ms * 1000;
        default:
            return 0;  // key/btn/fault are instantaneous on the timeline
    }
}

void err(const char* path, int line, const std::string& msg)
{
    std::fprintf(stderr, "[scenario] %s:%d: %s\n", path, line, msg.c_str());
}

// Parse a quoted "..." string (with \n \t \r \\ \" escapes) from `rest`.
bool parse_quoted(const std::string& rest, std::string& out)
{
    size_t i = rest.find('"');
    if (i == std::string::npos) return false;
    ++i;
    out.clear();
    for (; i < rest.size(); ++i) {
        char c = rest[i];
        if (c == '"') return true;               // closing quote
        if (c == '\\' && i + 1 < rest.size()) {
            char n = rest[++i];
            switch (n) {
                case 'n': out.push_back('\n'); break;
                case 't': out.push_back('\t'); break;
                case 'r': out.push_back('\r'); break;
                case '\\': out.push_back('\\'); break;
                case '"': out.push_back('"'); break;
                default: out.push_back(n); break;
            }
        } else {
            out.push_back(c);
        }
    }
    return false;  // unterminated
}

bool parse_rc_field(const std::string& s, int& field)
{
    if (s == "throttle") { field = 0; return true; }
    if (s == "roll")     { field = 1; return true; }
    if (s == "pitch")    { field = 2; return true; }
    if (s == "yaw")      { field = 3; return true; }
    return false;
}

bool in_adc(long v) { return v >= 0 && v <= 4095; }

}  // namespace

extern "C" {

int sil_scenario_load(const char* path)
{
    g_events.clear();
    g_active = false;
    if (path == nullptr || path[0] == '\0') return 0;

    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "[scenario] cannot open '%s'\n", path);
        return -1;
    }

    int64_t prev_end_us = 0;
    std::string line;
    int lineno = 0;

    while (std::getline(in, line)) {
        ++lineno;
        // strip comment + skip blank
        size_t hash = line.find('#');
        std::string body = (hash == std::string::npos) ? line : line.substr(0, hash);
        std::istringstream iss(body);
        std::string ttok;
        if (!(iss >> ttok)) continue;  // blank/comment-only

        // --- time token: absolute ms, or '+<ms>'/'+' relative to prev end ----
        int64_t at_us;
        if (ttok[0] == '+') {
            long ms = (ttok.size() > 1) ? std::atol(ttok.c_str() + 1) : 0;
            if (ms < 0) { err(path, lineno, "negative relative time"); return -1; }
            at_us = prev_end_us + (int64_t)ms * 1000;
        } else {
            char* endp = nullptr;
            long ms = std::strtol(ttok.c_str(), &endp, 10);
            if (endp == ttok.c_str() || *endp != '\0' || ms < 0) {
                err(path, lineno, "bad time token '" + ttok + "'"); return -1;
            }
            at_us = (int64_t)ms * 1000;
        }
        // Reject an event that starts BEFORE the previous event finishes: the
        // single-threaded driver runs events sequentially, so an absolute time
        // landing inside a prior rc hold/ramp would fire late and silently. '+'
        // resolves to exactly prev_end_us, so relative events always pass.
        // 直前事象の終了より前に始まる事象は拒否（逐次ドライバで遅延発火＝見えない不具合に）。
        if (!g_events.empty() && at_us < prev_end_us) {
            char m[96];
            std::snprintf(m, sizeof(m),
                "event starts before the previous one ends (prev ends at %lld ms)",
                (long long)(prev_end_us / 1000));
            err(path, lineno, m);
            return -1;
        }

        std::string ch;
        if (!(iss >> ch)) { err(path, lineno, "missing channel"); return -1; }

        Event e;
        e.at_us = at_us;
        e.line = lineno;

        if (ch == "rc") {
            long v[5];
            for (int k = 0; k < 5; ++k) {
                if (!(iss >> v[k])) { err(path, lineno, "rc needs <thr> <roll> <pitch> <yaw> <arm>"); return -1; }
            }
            for (int k = 0; k < 4; ++k) {
                if (!in_adc(v[k])) { err(path, lineno, "rc stick out of ADC range 0..4095"); return -1; }
            }
            if (v[4] != 0 && v[4] != 1) { err(path, lineno, "rc <arm> must be 0 or 1"); return -1; }
            e.ch = Channel::Rc;
            e.thr = (uint16_t)v[0]; e.roll = (uint16_t)v[1];
            e.pitch = (uint16_t)v[2]; e.yaw = (uint16_t)v[3];
            e.arm = (uint8_t)v[4];
            long hold = 0, rate = 20;
            if (iss >> hold) {
                if (hold < 0) { err(path, lineno, "rc hold_ms must be >= 0"); return -1; }
                if (iss >> rate) {
                    if (rate < 1 || rate > 1000) { err(path, lineno, "rc rate_hz must be 1..1000 (1 tick = 1 ms)"); return -1; }
                }
            }
            e.hold_ms = (int)hold; e.rate_hz = (int)rate;
            // OPTIONAL 4th token: alt (1 => ALTITUDE_HOLD). Only reachable after
            // hold_ms and rate_hz are present, so always write the full form when
            // requesting alt: `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt>`.
            // 任意の4番目トークン alt（1 で高度保持）。hold_ms・rate_hz の後にのみ到達するため、
            // alt を出す行は完全形で書くこと。
            long alt = 0;
            if (iss >> alt) {
                if (alt != 0 && alt != 1) { err(path, lineno, "rc <alt> must be 0 or 1"); return -1; }
            }
            e.alt = (uint8_t)alt;
            // OPTIONAL 5th token: acro (1 => CTRL_FLAG_MODE → ACRO/rate mode).
            // After alt, so the full form is:
            // `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt> <acro>`.
            // 任意の5番目トークン acro（1 で CTRL_FLAG_MODE → ACRO/角速度モード）。alt の後。
            long acro = 0;
            if (iss >> acro) {
                if (acro != 0 && acro != 1) { err(path, lineno, "rc <acro> must be 0 or 1"); return -1; }
            }
            e.acro = (uint8_t)acro;
            // OPTIONAL 6th token: pos (1 => CTRL_FLAG_POS_MODE → POSITION_HOLD). After
            // acro, so the full form is:
            // `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt> <acro> <pos>`.
            // 任意の6番目トークン pos（1 で CTRL_FLAG_POS_MODE → POSITION_HOLD）。acro の後。
            long pos = 0;
            if (iss >> pos) {
                if (pos != 0 && pos != 1) { err(path, lineno, "rc <pos> must be 0 or 1"); return -1; }
            }
            e.pos = (uint8_t)pos;

        } else if (ch == "rc_ramp") {
            std::string field; long from, to, step, rate, arm;
            if (!(iss >> field >> from >> to >> step >> rate >> arm)) {
                err(path, lineno, "rc_ramp needs <field> <from> <to> <step> <rate_hz> <arm>"); return -1;
            }
            if (!parse_rc_field(field, e.ramp_field)) { err(path, lineno, "rc_ramp field must be throttle|roll|pitch|yaw"); return -1; }
            if (!in_adc(from) || !in_adc(to)) { err(path, lineno, "rc_ramp from/to out of ADC range 0..4095"); return -1; }
            if (step == 0) { err(path, lineno, "rc_ramp step must be != 0"); return -1; }
            if (rate < 1 || rate > 1000) { err(path, lineno, "rc_ramp rate_hz must be 1..1000 (1 tick = 1 ms)"); return -1; }
            if (arm != 0 && arm != 1) { err(path, lineno, "rc_ramp <arm> must be 0 or 1"); return -1; }
            e.ch = Channel::RcRamp;
            e.ramp_from = (uint16_t)from; e.ramp_to = (uint16_t)to;
            e.ramp_step = (int)(step < 0 ? -step : step);  // magnitude; direction from from/to
            e.rate_hz = (int)rate; e.arm = (uint8_t)arm;
            // OPTIONAL 7th token: alt (1 => ALTITUDE_HOLD), after the 6 required.
            // 任意の7番目トークン alt（1 で高度保持）。必須6個の後。
            long ralt = 0;
            if (iss >> ralt) {
                if (ralt != 0 && ralt != 1) { err(path, lineno, "rc_ramp <alt> must be 0 or 1"); return -1; }
            }
            e.alt = (uint8_t)ralt;
            // OPTIONAL 8th token: acro (1 => CTRL_FLAG_MODE → ACRO/rate mode), after alt.
            // 任意の8番目トークン acro（1 で ACRO/角速度モード）。alt の後。
            long racro = 0;
            if (iss >> racro) {
                if (racro != 0 && racro != 1) { err(path, lineno, "rc_ramp <acro> must be 0 or 1"); return -1; }
            }
            e.acro = (uint8_t)racro;

        } else if (ch == "key") {
            std::string rest;
            std::getline(iss, rest);
            if (!parse_quoted(rest, e.text)) { err(path, lineno, "key needs a quoted \"...\" string"); return -1; }
            e.ch = Channel::Key;

        } else if (ch == "wind") {
            // wind <fx> <fy> <fz> [dur_ms] — external force NED [N]; dur_ms>0 = gust.
            // wind <fx> <fy> <fz> [dur_ms] — NED 外乱力 [N]、dur_ms>0 で突風。
            e.ch = Channel::Wind;
            if (!(iss >> e.wind_fx >> e.wind_fy >> e.wind_fz)) {
                err(path, lineno, "wind needs <fx> <fy> <fz> [dur_ms]"); return -1;
            }
            if (!(iss >> e.wind_dur_ms)) e.wind_dur_ms = 0;     // optional → sustained step
            if (e.wind_dur_ms < 0) { err(path, lineno, "wind dur_ms must be >= 0"); return -1; }

        } else if (ch == "fault") {
            // fault <motor 0-3> <gain 0-1> — degrade one motor's thrust health.
            // fault <motor 0-3> <gain 0-1> — 1モータの推力健全度を劣化。
            e.ch = Channel::Fault;
            if (!(iss >> e.fault_motor >> e.fault_gain)) {
                err(path, lineno, "fault needs <motor 0-3> <gain 0-1>"); return -1;
            }
            if (e.fault_motor < 0 || e.fault_motor > 3) {
                err(path, lineno, "fault <motor> must be 0..3"); return -1;
            }
            if (e.fault_gain < 0.0f || e.fault_gain > 1.0f) {
                err(path, lineno, "fault <gain> must be 0.0..1.0"); return -1;
            }

        } else if (ch == "bias") {
            // bias <ax> <ay> <az> <gx> <gy> <gz> — deterministic raw IMU bias (body
            // FRD): accel [m/s²] + gyro [rad/s]. Models a pre-calibration MEMS offset.
            // bias <ax> <ay> <az> <gx> <gy> <gz> — 決定論的な生 IMU バイアス（機体 FRD）:
            // accel[m/s²] + gyro[rad/s]。校正前 MEMS オフセットを模擬。
            e.ch = Channel::Bias;
            if (!(iss >> e.bias_ax >> e.bias_ay >> e.bias_az
                      >> e.bias_gx >> e.bias_gy >> e.bias_gz)) {
                err(path, lineno, "bias needs <ax> <ay> <az> <gx> <gy> <gz>"); return -1;
            }

        } else if (ch == "handle") {
            // handle <carry_alt_m> <place_x_ned> <place_y_ned> <lift_ms> <carry_ms> <place_ms>
            // Trigger a physical handling maneuver (lift→right→carry→place). Instantaneous
            // on the timeline; the maneuver runs in Plant time while the next disarmed rc
            // hold provides the link + wall-clock.
            // handle … — 物理ハンドリング動作を起動。タイムライン上は瞬時、動作は Plant 時間で
            // 進み、次の disarmed rc ホールドがリンクと実時間を与える。
            e.ch = Channel::Handle;
            if (!(iss >> e.handle_carry_alt >> e.handle_place_x >> e.handle_place_y
                      >> e.handle_lift_ms >> e.handle_carry_ms >> e.handle_place_ms)) {
                err(path, lineno,
                    "handle needs <carry_alt_m> <place_x> <place_y> <lift_ms> <carry_ms> <place_ms>");
                return -1;
            }
            if (e.handle_carry_alt <= 0.0f) {
                err(path, lineno, "handle <carry_alt_m> must be > 0"); return -1;
            }
            if (e.handle_lift_ms <= 0 || e.handle_carry_ms <= 0 || e.handle_place_ms <= 0) {
                err(path, lineno, "handle lift/carry/place durations must be > 0 ms"); return -1;
            }

        } else if (ch == "btn") {
            std::string rest; std::getline(iss, rest);
            e.ch = Channel::Btn;
            e.raw = ch + rest;

        } else {
            err(path, lineno, "unknown channel '" + ch + "'");
            return -1;
        }

        prev_end_us = at_us + event_duration_us(e);
        g_events.push_back(std::move(e));
    }

    g_active = !g_events.empty();
    if (g_active) {
        std::printf("[scenario] loaded %zu events from %s\n", g_events.size(), path);
    } else {
        std::fprintf(stderr, "[scenario] %s has no events\n", path);
    }
    return g_active ? 1 : -1;
}

bool sil_scenario_active(void) { return g_active; }

void sil_scenario_driver_task(void* /*arg*/)
{
    std::printf("[scenario] driver online (%zu events)\n", g_events.size());

    bool warned_deferred = false;

    for (const Event& e : g_events) {
        // Wait until the virtual clock reaches this event's time (absolute tick).
        // 仮想時計がこの事象の時刻（絶対 tick）に達するまで待つ。
        TickType_t target = (TickType_t)(e.at_us / 1000);
        while (xTaskGetTickCount() < target) {
            vTaskDelay(1);
        }

        switch (e.ch) {
            case Channel::Rc: {
                const int period = period_ms(e.rate_hz);
                const uint8_t flags = (e.arm  ? sil::kFlagArm     : 0) |
                                      (e.alt  ? sil::kFlagAltMode : 0) |
                                      (e.acro ? sil::kFlagMode    : 0) |
                                      (e.pos  ? sil::kFlagPosMode : 0);
                if (e.hold_ms <= 0) {
                    sil::inject_rc(e.thr, e.roll, e.pitch, e.yaw, flags);
                } else {
                    // Same frame count the parser used for the timeline duration
                    // (>=1 for any positive hold) so emit and timing agree.
                    // パーサの時間幅と同じフレーム数（正のホールドは最低1）で発射・時刻を一致。
                    const long long frames = rc_hold_frames(e.hold_ms, e.rate_hz);
                    for (long long i = 0; i < frames; ++i) {
                        sil::inject_rc(e.thr, e.roll, e.pitch, e.yaw, flags);
                        vTaskDelay((TickType_t)period);
                    }
                }
                break;
            }
            case Channel::RcRamp: {
                const int period = period_ms(e.rate_hz);
                const uint8_t flags = (e.arm  ? sil::kFlagArm     : 0) |
                                      (e.alt  ? sil::kFlagAltMode : 0) |
                                      (e.acro ? sil::kFlagMode    : 0) |
                                      (e.pos  ? sil::kFlagPosMode : 0);
                const int astep = (e.ramp_to >= e.ramp_from) ? e.ramp_step : -e.ramp_step;
                for (long v = e.ramp_from;
                     (astep > 0) ? (v <= e.ramp_to) : (v >= e.ramp_to);
                     v += astep) {
                    uint16_t s[4] = {sil::kAdcCentre, sil::kAdcCentre, sil::kAdcCentre, sil::kAdcCentre};
                    s[e.ramp_field] = (uint16_t)v;
                    sil::inject_rc(s[0], s[1], s[2], s[3], flags);
                    vTaskDelay((TickType_t)period);
                }
                break;
            }
            case Channel::Key:
                sil_console_write(e.text.data(), (int)e.text.size());
                break;
            case Channel::Wind: {
                // P7: apply the external wind force to the Plant. A gust pulse
                // (dur_ms>0) holds the force, then reverts to zero so the controller
                // must recover; a sustained step (dur_ms=0) leaves it applied.
                // P7: 外乱風力を Plant に適用。突風パルス(dur_ms>0)は保持後ゼロに戻し制御の
                // 回復を要求。定常(dur_ms=0)は適用したまま。
                sil_board_set_wind(e.wind_fx, e.wind_fy, e.wind_fz);
                char note[96];
                std::snprintf(note, sizeof(note), "wind %.3f %.3f %.3f dur=%dms",
                              e.wind_fx, e.wind_fy, e.wind_fz, e.wind_dur_ms);
                sil_emu_record_note("wind", note);
                if (e.wind_dur_ms > 0) {
                    vTaskDelay((TickType_t)e.wind_dur_ms);
                    sil_board_set_wind(0.0f, 0.0f, 0.0f);
                    sil_emu_record_note("wind", "0 0 0 (gust end)");
                }
                break;
            }
            case Channel::Fault: {
                // P7: degrade one motor's thrust health (sustained). The mixer must
                // compensate to keep attitude/altitude bounded (G3).
                // P7: 1モータの推力健全度を劣化（持続）。ミキサーが補償し姿勢/高度を有界に保つ（G3）。
                sil_board_set_motor_health(e.fault_motor, e.fault_gain);
                char note[64];
                std::snprintf(note, sizeof(note), "fault motor=%d gain=%.2f",
                              e.fault_motor, e.fault_gain);
                sil_emu_record_note("fault", note);
                break;
            }
            case Channel::Bias: {
                // P2-3: inject a deterministic raw IMU bias (body FRD). The firmware
                // boot calibration should measure and remove it; without calibration
                // the ESKF attitude loop degrades (the contrast test).
                // P2-3: 決定論的な生 IMU バイアス（機体 FRD）を注入。ファーム起動校正が
                // 測定・除去すべき。校正なしでは ESKF 姿勢ループが劣化（対照試験）。
                sil_board_set_imu_bias(e.bias_ax, e.bias_ay, e.bias_az,
                                       e.bias_gx, e.bias_gy, e.bias_gz);
                char note[96];
                std::snprintf(note, sizeof(note),
                              "bias accel=[%.3f %.3f %.3f] gyro=[%.4f %.4f %.4f]",
                              e.bias_ax, e.bias_ay, e.bias_az,
                              e.bias_gx, e.bias_gy, e.bias_gz);
                sil_emu_record_note("bias", note);
                break;
            }
            case Channel::Handle: {
                // P8: trigger the physical handling maneuver on the Plant (lift→right→
                // carry→place). Instantaneous here; the maneuver advances in Plant time
                // while the following disarmed rc holds keep the link alive and provide the
                // wall-clock. Durations ms → s for the Plant API.
                // P8: 物理ハンドリング動作を Plant に起動。ここでは瞬時、動作は Plant 時間で
                // 進み、続く disarmed rc ホールドがリンクを維持し実時間を与える。
                sil_board_handle_place(e.handle_carry_alt, e.handle_place_x, e.handle_place_y,
                                       e.handle_lift_ms  * 1e-3f,
                                       e.handle_carry_ms * 1e-3f,
                                       e.handle_place_ms * 1e-3f);
                char note[112];
                std::snprintf(note, sizeof(note),
                              "handle carry=%.2fm place=(%.2f,%.2f) lift=%dms carry=%dms place=%dms",
                              e.handle_carry_alt, e.handle_place_x, e.handle_place_y,
                              e.handle_lift_ms, e.handle_carry_ms, e.handle_place_ms);
                sil_emu_record_note("handle", note);
                break;
            }
            case Channel::Btn:
                if (!warned_deferred) {
                    std::printf("[scenario] note: btn is deferred (GPIO future) — skipping\n");
                    warned_deferred = true;
                }
                sil_emu_record_note("skip", e.raw.c_str());
                break;
        }
    }

    std::printf("[scenario] timeline exhausted — driver done\n");
    vTaskDelete(nullptr);
}

}  // extern "C"
