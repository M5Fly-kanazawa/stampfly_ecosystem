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

enum class Channel { Rc, RcRamp, Key, Btn, Wind, Fault };

struct Event {
    int64_t at_us = 0;        // absolute virtual time (frozen) / 絶対仮想時刻
    Channel ch = Channel::Rc;
    int     line = 0;         // source line (diagnostics) / ソース行

    // rc
    uint16_t thr = 0, roll = 0, pitch = 0, yaw = 0;
    uint8_t  arm = 0;
    int      hold_ms = 0;     // 0 = single frame / 0=単発
    int      rate_hz = 20;

    // rc_ramp
    int      ramp_field = 0;  // 0=throttle 1=roll 2=pitch 3=yaw
    uint16_t ramp_from = 0, ramp_to = 0;
    int      ramp_step = 1;

    // key
    std::string text;

    // btn/wind/fault (deferred): keep raw args for the warn note
    std::string raw;
};

std::vector<Event> g_events;
bool g_active = false;

// period (ms) for a given send rate, integer-floored (1 tick = 1 ms).
int period_ms(int rate_hz) { return rate_hz > 0 ? (1000 / rate_hz) : 50; }

int ramp_frames(uint16_t from, uint16_t to, int step)
{
    int s = step < 0 ? -step : step;
    if (s == 0) return 1;
    int span = (to >= from) ? (to - from) : (from - to);
    return span / s + 1;
}

// duration the event occupies on the timeline (for '+'-relative resolution).
int64_t event_duration_us(const Event& e)
{
    switch (e.ch) {
        case Channel::Rc:
            return (e.hold_ms > 0) ? (int64_t)e.hold_ms * 1000 : 0;
        case Channel::RcRamp:
            return (int64_t)ramp_frames(e.ramp_from, e.ramp_to, e.ramp_step)
                 * period_ms(e.rate_hz) * 1000;
        default:
            return 0;  // key/btn/wind/fault are instantaneous on the timeline
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
        if (!g_events.empty() && at_us < g_events.back().at_us) {
            err(path, lineno, "time goes backwards (must be non-decreasing)");
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
                if (iss >> rate) { if (rate <= 0) { err(path, lineno, "rc rate_hz must be > 0"); return -1; } }
            }
            e.hold_ms = (int)hold; e.rate_hz = (int)rate;

        } else if (ch == "rc_ramp") {
            std::string field; long from, to, step, rate, arm;
            if (!(iss >> field >> from >> to >> step >> rate >> arm)) {
                err(path, lineno, "rc_ramp needs <field> <from> <to> <step> <rate_hz> <arm>"); return -1;
            }
            if (!parse_rc_field(field, e.ramp_field)) { err(path, lineno, "rc_ramp field must be throttle|roll|pitch|yaw"); return -1; }
            if (!in_adc(from) || !in_adc(to)) { err(path, lineno, "rc_ramp from/to out of ADC range 0..4095"); return -1; }
            if (step == 0) { err(path, lineno, "rc_ramp step must be != 0"); return -1; }
            if (rate <= 0) { err(path, lineno, "rc_ramp rate_hz must be > 0"); return -1; }
            if (arm != 0 && arm != 1) { err(path, lineno, "rc_ramp <arm> must be 0 or 1"); return -1; }
            e.ch = Channel::RcRamp;
            e.ramp_from = (uint16_t)from; e.ramp_to = (uint16_t)to;
            e.ramp_step = (int)(step < 0 ? -step : step);  // magnitude; direction from from/to
            e.rate_hz = (int)rate; e.arm = (uint8_t)arm;

        } else if (ch == "key") {
            std::string rest;
            std::getline(iss, rest);
            if (!parse_quoted(rest, e.text)) { err(path, lineno, "key needs a quoted \"...\" string"); return -1; }
            e.ch = Channel::Key;

        } else if (ch == "btn" || ch == "wind" || ch == "fault") {
            std::string rest; std::getline(iss, rest);
            e.ch = (ch == "btn") ? Channel::Btn : (ch == "wind") ? Channel::Wind : Channel::Fault;
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
                const uint8_t flags = e.arm ? sil::kFlagArm : 0;
                if (e.hold_ms <= 0) {
                    sil::inject_rc(e.thr, e.roll, e.pitch, e.yaw, flags);
                } else {
                    const int frames = e.hold_ms * e.rate_hz / 1000;
                    for (int i = 0; i < frames; ++i) {
                        sil::inject_rc(e.thr, e.roll, e.pitch, e.yaw, flags);
                        vTaskDelay((TickType_t)period);
                    }
                }
                break;
            }
            case Channel::RcRamp: {
                const int period = period_ms(e.rate_hz);
                const uint8_t flags = e.arm ? sil::kFlagArm : 0;
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
            case Channel::Btn:
            case Channel::Wind:
            case Channel::Fault:
                if (!warned_deferred) {
                    std::printf("[scenario] note: btn/wind/fault are deferred (E6 future) — skipping\n");
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
