/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file rc_stdin.hpp
 * @brief Live RC-over-stdin injector for keyboard-piloted SILS (P6 stage 1).
 *        キーボード操縦SILS向けのRC-over-stdinライブ注入器（P6 stage 1）。
 *
 * SILS_EMU_RC_STDIN=1 reads line-oriented commands from the process's REAL
 * stdin (the terminal, or `sf sils fly`'s pipe) and injects them as ESP-NOW
 * ControlPackets through the SAME builder the *.scn scenario driver uses
 * (scenario_inject.hpp) — same on-air bytes, same ADC-raw scale (0..4095,
 * centre 2048), no separate wire format to keep in sync.
 *
 *   rc <roll> <pitch> <yaw> <throttle>   set sticks (raw ADC, held until the
 *                                        next `rc` line — re-injected at a
 *                                        fixed cadence regardless of how often
 *                                        new lines arrive)
 *   arm | land | disarm                  pulse the ARM wire bit high for a
 *                                        short hold then release — mirrors a
 *                                        real transmitter's momentary ARM
 *                                        button (state_task.cpp toggles
 *                                        ARM/DISARM on the press, whichever
 *                                        applies). `land` is the same pulse
 *                                        under the name a mid-flight pilot
 *                                        would reach for.
 *   quit                                  request a clean emulator shutdown
 *
 * SILS_EMU_RC_STDIN=1 でプロセスの実 stdin（ターミナル、または `sf sils fly` の
 * パイプ）から行指向コマンドを読み、*.scn シナリオドライバと同じビルダ
 * （scenario_inject.hpp）で ESP-NOW ControlPacket として注入する — 電波バイト・
 * ADC生スケール（0..4095, 中央2048）とも共通、別規格を同期させる必要がない。
 *
 * Read-side note: the emulator's main() re-points STDIN_FILENO at a private
 * non-blocking pipe for the FIRMWARE's own CLI (see emu_main.cpp). This module
 * must therefore dup() the process's real stdin BEFORE that happens —
 * sils_rc_stdin_init() does this and must be called first in main().
 * 読み取り注記: emu の main() は STDIN_FILENO をファーム自身の CLI 用の
 * 非ブロッキング pipe に差し替える（emu_main.cpp 参照）。よって本モジュールは
 * それより前にプロセスの実 stdin を dup() しておく必要がある —
 * sils_rc_stdin_init() がこれを行うので main() の最初に呼ぶこと。
 *
 * @design docs/architecture/simulation-policy.md — SILS fidelity / tooling  [--]
 */

#pragma once

#include <cstdint>

extern "C" {

// Call ONCE, as the very first statement in main() — before STDIN_FILENO is
// repurposed for the firmware's own CLI pipe. No-op unless SILS_EMU_RC_STDIN is
// set (the real stdin is left completely alone on the default path).
// main() の最初の文として1回だけ呼ぶ — STDIN_FILENO がファームCLI用pipeに
// 差し替えられる前に。SILS_EMU_RC_STDIN 未設定なら no-op（既定経路は実stdinに
// 一切触れない）。
void sils_rc_stdin_init(void);

// Drain any newly available line(s) and (re-)inject the persisted stick/arm
// state at a fixed cadence. Call from on_advance(now_us) every tick. No-op
// unless sils_rc_stdin_init() actually activated.
// 到着済みの行を排出し、保持中のスティック/ARM状態を一定周期で再注入する。
// on_advance(now_us) から毎回呼ぶ。sils_rc_stdin_init() が有効化していなければ no-op。
void sils_rc_stdin_tick(int64_t now_us);

// True once a "quit" line has been received (the caller should shut the
// emulator down cleanly, e.g. via _Exit(0) — see emu_main.cpp's own end-of-run
// teardown rationale). 「quit」行受信後 true（呼び出し側はエミュレータを綺麗に
// 終了させる — 手法は emu_main.cpp 末尾の _Exit(0) と同じ理屈）。
bool sils_rc_stdin_quit_requested(void);

}  // extern "C"
