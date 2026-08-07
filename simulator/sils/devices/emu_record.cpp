/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_record.cpp
 * @brief Implementation of the deterministic input/event recorder.
 *        決定論的な入力/事象レコーダの実装。
 */

#include "emu_record.hpp"

#include <cstdio>
#include <cstring>

// Virtual microsecond clock (esp_timer_shim.cpp). Declared here to avoid a
// header dependency; the signature is the stable ESP-IDF one.
// 仮想マイクロ秒時計。ヘッダ依存を避けるため宣言のみ（ESP-IDF の安定シグネチャ）。
extern "C" int64_t esp_timer_get_time(void);

namespace {

FILE* g_out = nullptr;

void write_hex(FILE* f, const uint8_t* data, int len)
{
    static const char kHex[] = "0123456789abcdef";
    for (int i = 0; i < len; ++i) {
        std::fputc(kHex[(data[i] >> 4) & 0xF], f);
        std::fputc(kHex[data[i] & 0xF], f);
    }
}

// Emit a JSON string value with the few escapes a control channel can produce.
// 制御チャネルが出しうる最小限のエスケープで JSON 文字列値を出力。
void write_json_string(FILE* f, const char* s)
{
    std::fputc('"', f);
    for (const char* p = s; *p; ++p) {
        switch (*p) {
            case '"':  std::fputs("\\\"", f); break;
            case '\\': std::fputs("\\\\", f); break;
            case '\n': std::fputs("\\n", f);  break;
            case '\r': std::fputs("\\r", f);  break;
            case '\t': std::fputs("\\t", f);  break;
            default:
                // RFC 8259 §7: every control character U+0000..U+001F must be
                // escaped, else events.jsonl is not valid JSON.
                if (static_cast<unsigned char>(*p) < 0x20) {
                    std::fprintf(f, "\\u%04x", static_cast<unsigned char>(*p));
                } else {
                    std::fputc(*p, f);
                }
                break;
        }
    }
    std::fputc('"', f);
}

}  // namespace

extern "C" {

void sils_emu_record_open(const char* path)
{
    if (g_out != nullptr) { std::fclose(g_out); g_out = nullptr; }
    if (path == nullptr || path[0] == '\0') return;
    g_out = std::fopen(path, "w");
}

void sils_emu_record_close(void)
{
    if (g_out != nullptr) { std::fflush(g_out); std::fclose(g_out); g_out = nullptr; }
}

bool sils_emu_record_active(void) { return g_out != nullptr; }

void sils_emu_record_bytes(const char* channel, const uint8_t* data, int len)
{
    if (g_out == nullptr) return;  // no-op when closed (default run unchanged)
    std::fprintf(g_out, "{\"t_us\":%lld,\"ch\":", (long long)esp_timer_get_time());
    write_json_string(g_out, channel);
    std::fprintf(g_out, ",\"n\":%d,\"bytes\":\"", len);
    if (data != nullptr && len > 0) write_hex(g_out, data, len);
    std::fputs("\"}\n", g_out);
    // Flush per line so a crash mid-run still leaves the events observed so far.
    // 行ごとに flush（途中クラッシュでも既観測事象は残る）。
    std::fflush(g_out);
}

void sils_emu_record_note(const char* channel, const char* text)
{
    if (g_out == nullptr) return;
    std::fprintf(g_out, "{\"t_us\":%lld,\"ch\":", (long long)esp_timer_get_time());
    write_json_string(g_out, channel);
    std::fputs(",\"note\":", g_out);
    write_json_string(g_out, text != nullptr ? text : "");
    std::fputs("}\n", g_out);
    std::fflush(g_out);
}

}  // extern "C"
