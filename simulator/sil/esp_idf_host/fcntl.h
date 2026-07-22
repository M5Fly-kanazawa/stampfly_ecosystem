/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file fcntl.h
 * @brief Host shim for <fcntl.h> — fills MinGW-w64 gaps used by the
 *        firmware's non-blocking-stdin CLI pattern (F_GETFL/F_SETFL/
 *        O_NONBLOCK/fcntl()), which real POSIX hosts (Linux/macOS) already
 *        provide, so this file is a no-op there (see the #if guard below).
 *        <fcntl.h> のホスト用シム — MinGW-w64 に無い、ファームの非ブロッキング
 *        stdin CLI パターン向けシンボルを補う（実 POSIX ホストは元々ある。
 *        下の #if によりそちらでは何もしない）。
 *
 * mingw-w64's own <fcntl.h> lacks F_GETFL/F_SETFL/O_NONBLOCK and fcntl()
 * entirely (Windows file descriptors have no POSIX fcntl model). We
 * #include_next the real mingw <fcntl.h> for everything else (O_RDONLY etc.)
 * and only ADD what is missing, MinGW-only.
 *
 * The two call sites in this codebase (simulator/sil/emu/emu_main*.cpp,
 * firmware/vehicle_old/components/sf_svc_serial_cli/serial_cli.cpp) only ever
 * use F_GETFL to read back flags for a log line, and F_SETFL(O_NONBLOCK) on
 * pipe ends that this bench ALREADY creates non-blocking on Windows (see
 * esp_idf_host/unistd.h's pipe() — a PIPE_NOWAIT named pipe). So this fcntl()
 * only needs to be a well-behaved bookkeeping shim, not a real Win32
 * blocking-mode switch.
 *
 * mingw-w64 の <fcntl.h> には F_GETFL/F_SETFL/O_NONBLOCK/fcntl() が丸ごと無い
 * （Windows のファイル記述子に POSIX 流の fcntl モデルが無いため）。他は全て
 * 実 mingw <fcntl.h> を #include_next で引き込み、無い物だけ MinGW 限定で足す。
 *
 * 本リポジトリの2つの呼び出し元（simulator/sil/emu/emu_main*.cpp,
 * firmware/vehicle_old の serial_cli.cpp）は F_GETFL でログ用にフラグを
 * 読み戻すだけ、F_SETFL(O_NONBLOCK) は本ベンチが Windows で最初から non-blocking
 * に作る pipe 端（esp_idf_host/unistd.h の pipe() 参照 — PIPE_NOWAIT ネームド
 * パイプ）に対してのみ呼ばれる。よってこの fcntl() は実 Win32 のブロッキング
 * モード切替ではなく、単純な記帳シムで足りる。
 */

#pragma once

#include_next <fcntl.h>

#if defined(_WIN32) && !defined(F_GETFL)

#define F_GETFL 1
#define F_SETFL 2
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x0800
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Minimal fcntl(): F_GETFL always reports O_NONBLOCK set (every fd this bench
// hands to fcntl() is a pipe end created non-blocking up front — see
// unistd.h's pipe()); F_SETFL is accepted and ignored (no-op success); any
// other command also succeeds as a no-op. Never fails, never blocks.
// 最小限の fcntl(): F_GETFL は常に O_NONBLOCK 設定済みとして報告する（本ベンチが
// fcntl() に渡す fd は全て最初から non-blocking で作った pipe 端 — unistd.h の
// pipe() 参照）。F_SETFL は受理して無視（no-op 成功）。他コマンドも no-op で
// 成功する。失敗もブロックもしない。
static inline int sil_win_fcntl(int fd, int cmd, int arg)
{
    (void)fd; (void)arg;
    return (cmd == F_GETFL) ? O_NONBLOCK : 0;
}
#define fcntl(fd, cmd, arg) sil_win_fcntl((fd), (cmd), (int)(arg))

#ifdef __cplusplus
}
#endif

#endif  // _WIN32 && !F_GETFL
