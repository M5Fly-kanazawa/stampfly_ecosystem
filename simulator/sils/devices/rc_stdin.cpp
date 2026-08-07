/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file rc_stdin.cpp
 * @brief Live RC-over-stdin injector implementation — see rc_stdin.hpp.
 *        RC-over-stdinライブ注入器の実装 — rc_stdin.hpp 参照。
 */

#include "rc_stdin.hpp"
#include "scenario_inject.hpp"   // sils::inject_rc / kAdcCentre / kFlagArm (SSOT builder)

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <unistd.h>

namespace {

// -1 = disabled (SILS_EMU_RC_STDIN unset, or init() never called/failed). A
// valid fd here means the feature is live.
// -1=無効（SILS_EMU_RC_STDIN 未設定、または init() 未呼出/失敗）。有効な fd が
// 入っていれば本機能が動作中。
int g_fd = -1;

std::string g_linebuf;   // partial-line carry-over across ticks / tick跨ぎの断片行

// Persisted stick state (ADC raw, centre 2048) — held across ticks and
// re-injected at a fixed cadence regardless of how often new `rc` lines arrive.
// 保持中のスティック状態（ADC生値、中央2048）— tick を跨いで保持し、新しい `rc`
// 行の到着頻度に関わらず一定周期で再注入する。
uint16_t g_roll  = sils::kAdcCentre;
uint16_t g_pitch = sils::kAdcCentre;
uint16_t g_yaw   = sils::kAdcCentre;
uint16_t g_thr   = sils::kAdcCentre;

int64_t g_next_inject_us   = 0;    // next virtual time due for a re-injection
int64_t g_arm_pulse_until  = -1;   // ARM wire bit is high while now_us < this

bool g_quit_requested = false;

// Re-injection cadence: 50 Hz, matching the *.scn convention's common rc rate
// and the firmware's own command-processing rate — fast enough that a wire
// disconnect (dropped ControlPacket) never reads as "stick released" for long.
// 再注入周期: 50Hz（*.scn の一般的な rc レート、ファーム側コマンド処理レートと
// 揃える）。パケット欠落が「スティック中立」と誤認される時間を短く保つ。
constexpr int64_t kInjectPeriodUs = 20'000;

// ARM pulse hold: long enough to be seen as a clean press-then-release edge
// pair regardless of the consumer's own poll rate (stab_flight.scn's own ARM
// presses hold ~200-1000 ms for the same reason).
// ARMパルス保持時間: 受信側のポーリング周期に関わらず、明確な押下→解放の
// エッジ対として見える長さ（stab_flight.scn自身のARM押下も同じ理由で
// 約200〜1000ms保持している）。
constexpr int64_t kArmPulseUs = 300'000;

uint16_t clamp_adc(long v)
{
    if (v < 0) return 0;
    if (v > 4095) return 4095;
    return static_cast<uint16_t>(v);
}

// Parse and act on one complete line (newline already stripped).
// 完成した1行を解釈・実行する（改行は除去済み）。
void process_line(const std::string& raw_line, int64_t now_us)
{
    std::string line = raw_line;
    if (!line.empty() && line.back() == '\r') line.pop_back();   // tolerate CRLF

    std::istringstream iss(line);
    std::string cmd;
    if (!(iss >> cmd)) return;   // blank line

    if (cmd == "rc") {
        long roll, pitch, yaw, thr;
        if (!(iss >> roll >> pitch >> yaw >> thr)) {
            std::fprintf(stderr,
                "[rc_stdin] bad 'rc' line (need <roll> <pitch> <yaw> <throttle>): %s\n",
                line.c_str());
            return;
        }
        g_roll  = clamp_adc(roll);
        g_pitch = clamp_adc(pitch);
        g_yaw   = clamp_adc(yaw);
        g_thr   = clamp_adc(thr);
    } else if (cmd == "arm" || cmd == "land" || cmd == "disarm") {
        // Same wire action regardless of the word used: a rising-then-falling
        // edge on the ARM bit. The firmware's own edge-toggle (state_task.cpp)
        // decides ARM vs DISARM from its current state — exactly like a real
        // transmitter's single momentary button.
        // 使う単語に関わらず同じ電文動作: ARMビットの立ち上がり→立ち下がり。
        // ARM/DISARM のどちらになるかはファーム自身のエッジトグル
        // （state_task.cpp）が現在状態から決める — 実機送信機の単一モーメンタリ
        // ボタンと同じ。
        g_arm_pulse_until = now_us + kArmPulseUs;
    } else if (cmd == "quit") {
        g_quit_requested = true;
    } else {
        std::fprintf(stderr, "[rc_stdin] unknown command '%s' (want: rc/arm/land/quit)\n",
                     cmd.c_str());
    }
}

}  // namespace

extern "C" {

void sils_rc_stdin_init(void)
{
    if (std::getenv("SILS_EMU_RC_STDIN") == nullptr) return;   // disabled: untouched

    // Save the process's REAL stdin (terminal, or the launcher's pipe) before
    // main() repurposes STDIN_FILENO for the firmware's own CLI. Must run
    // first — see rc_stdin.hpp.
    // プロセスの実stdin（ターミナル、または起動元のpipe）を、main()がSTDIN_FILENOを
    // ファームCLI用に差し替える前に保存する。最初に呼ぶこと — rc_stdin.hpp 参照。
    int real_stdin = dup(STDIN_FILENO);
    if (real_stdin < 0) {
        std::fprintf(stderr, "[rc_stdin] dup(stdin) failed (errno=%d) — RC-over-stdin disabled\n",
                     errno);
        return;
    }
    int flags = fcntl(real_stdin, F_GETFL, 0);
    if (flags < 0 || fcntl(real_stdin, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::fprintf(stderr, "[rc_stdin] fcntl(O_NONBLOCK) failed — RC-over-stdin disabled\n");
        close(real_stdin);
        return;
    }
    g_fd = real_stdin;
    std::printf("[rc_stdin] SILS_EMU_RC_STDIN enabled — reading "
                "'rc <roll> <pitch> <yaw> <throttle>' / 'arm' / 'land' / 'quit' from stdin\n");
}

void sils_rc_stdin_tick(int64_t now_us)
{
    if (g_fd < 0) return;   // disabled

    // Drain everything currently available (non-blocking) into the line buffer.
    // 現在読める分を全て（非ブロッキングで）行バッファへ取り込む。
    char chunk[256];
    for (;;) {
        ssize_t n = read(g_fd, chunk, sizeof(chunk));
        if (n > 0) {
            g_linebuf.append(chunk, static_cast<size_t>(n));
            continue;
        }
        break;   // n==0 (peer closed) or n<0 (EAGAIN/EWOULDBLOCK/other) — nothing more now
    }

    // Process every complete line; leave a trailing partial line for next tick.
    // 完成した行を全て処理。末尾の未完成行は次回に持ち越す。
    size_t pos;
    while ((pos = g_linebuf.find('\n')) != std::string::npos) {
        process_line(g_linebuf.substr(0, pos), now_us);
        g_linebuf.erase(0, pos + 1);
    }

    // Re-inject the persisted state at a fixed cadence, decoupled from however
    // often stdin lines actually arrive (the whole point of "held until the
    // next line" — see rc_stdin.hpp).
    // 保持中の状態を一定周期で再注入（stdin行の到着頻度から切り離す — 「次の行
    // まで保持」の要点、rc_stdin.hpp 参照）。
    if (now_us >= g_next_inject_us) {
        g_next_inject_us = now_us + kInjectPeriodUs;
        const uint8_t flags = (now_us < g_arm_pulse_until) ? sils::kFlagArm : 0;
        sils::inject_rc(g_thr, g_roll, g_pitch, g_yaw, flags);
    }
}

bool sils_rc_stdin_quit_requested(void) { return g_quit_requested; }

}  // extern "C"
