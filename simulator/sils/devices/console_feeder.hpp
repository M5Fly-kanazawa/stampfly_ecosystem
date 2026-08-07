/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file console_feeder.hpp
 * @brief Deterministic console byte feeder (scripted stdin for the firmware CLI).
 *        決定論的なコンソール・バイトフィーダ（ファーム CLI への台本 stdin）。
 *
 * The firmware's serial-CLI task does read(STDIN_FILENO,...). The emulator points
 * STDIN at the read end of a non-blocking pipe; this feeder writes scripted bytes
 * into the WRITE end at scheduled virtual times. The firmware's real read() then
 * returns those bytes (firmware UNMODIFIED); when the pipe is empty read() gets
 * EAGAIN and the firmware's own poll loop yields (no scheduler stall).
 *
 * The feeder must be called from a TASK holding the run-token (the scenario
 * driver), never from on_advance. It writes in a NON-BLOCKING, bounded way: if
 * the pipe is full it yields one tick and retries, so a large payload can never
 * block the cooperative scheduler in the kernel (which would trip the hang
 * detector). Each chunk is recorded on the "key" channel for the events log.
 *
 * firmware CLI は read(STDIN_FILENO) する。STDIN は非ブロッキング pipe の読み端。本
 * フィーダは仮想時刻で書き端へ台本バイトを書く。満杯なら 1tick yield して再試行（協調
 * スケジューラをカーネルでブロックさせない）。run-token 保持タスクからのみ呼ぶこと。
 */

#pragma once

extern "C" {

// Set the pipe write-end fd the feeder writes into (must be O_NONBLOCK).
// フィーダが書き込む pipe 書き端 fd を設定（O_NONBLOCK 必須）。
void sils_console_set_fd(int fd);

// Write `n` scripted bytes to the firmware console, non-blocking and bounded,
// yielding a tick on backpressure. Records the bytes on the "key" channel.
// Must be called from a run-token-holding task (the scenario driver).
// 台本バイトを非ブロッキング・有界に書く（満杯時 1tick yield）。run-token 保持タスクから。
void sils_console_write(const char* bytes, int n);

}  // extern "C"
