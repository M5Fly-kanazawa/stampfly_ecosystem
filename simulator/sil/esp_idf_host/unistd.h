/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file unistd.h
 * @brief Host shim for <unistd.h> — adds a working non-blocking pipe() on
 *        MinGW, where it does not exist at all (only the non-POSIX _pipe()
 *        does, and it cannot be switched to non-blocking mode). Real POSIX
 *        hosts (Linux/macOS) already have a real pipe(), so this file is a
 *        no-op there.
 *        <unistd.h> のホスト用シム — MinGW に丸ごと無い、動作する非ブロッキング
 *        pipe() を足す（POSIX 互換でない _pipe() のみあり、非ブロッキングに
 *        切替不可）。実 POSIX ホストは元々 pipe() があるので何もしない。
 *
 * simulator/sil/emu/emu_main*.cpp create a pipe, dup2 its read end onto
 * stdin, and want reads on an empty pipe to return immediately (never block
 * the cooperative scheduler's single run-token) rather than blocking or
 * returning EOF. A Windows ANONYMOUS pipe (what _pipe()/CreatePipe make)
 * cannot be put in non-blocking mode via any documented Win32 API
 * (SetNamedPipeHandleState only works on NAMED pipes). So pipe() here
 * actually creates a NAMED pipe (both ends local to this process, its name
 * private/PID-qualified) and immediately switches the read end to
 * PIPE_NOWAIT — verified empirically: read() on the empty read end returns
 * -1 without blocking, and returns real bytes once written, without ever
 * signalling EOF. (mingw-w64's CRT maps the underlying ERROR_NO_DATA to
 * errno=EINVAL rather than EAGAIN — a cosmetic difference from POSIX; nothing
 * in this bench currently branches on that errno value, since the emulator's
 * REPL is a no-op stub on host — see esp_idf_host/esp_console.h.)
 *
 * emu_main*.cpp は pipe を作り、読み出し端を stdin へ dup2 して、空パイプへの
 * read が協調スケジューラの単一トークンを止めない（ブロックしない）ことを
 * 期待する（EOF にもならない）。Windows の匿名パイプ（_pipe()/CreatePipe が
 * 作るもの）はどの Win32 API でも非ブロッキングに切替不可
 * （SetNamedPipeHandleState はネームドパイプにしか効かない）。そこで pipe() は
 * 実際にはネームドパイプ（両端ともこのプロセス内だけ、名前は PID で一意化）を
 * 作り、読み出し端を即座に PIPE_NOWAIT へ切り替える — 実証済み: 空の読み出し端
 * への read はブロックせず -1 を返し、書き込まれれば実バイトを返し、EOF には
 * ならない（mingw-w64 の CRT は ERROR_NO_DATA を errno=EAGAIN でなく EINVAL に
 * 変換する — POSIX との見た目の違いだが、本ベンチの現状のコードはこの errno
 * 値で分岐していない。エミュレータの REPL は host では no-op スタブのため —
 * esp_idf_host/esp_console.h 参照）。
 */

#pragma once

#include_next <unistd.h>

#if defined(_WIN32) && !defined(SIL_WIN_PIPE_DEFINED)
#define SIL_WIN_PIPE_DEFINED 1

#include <windows.h>
#include <io.h>
#include <fcntl.h>   // _O_RDONLY/_O_WRONLY/_O_BINARY

#ifdef __cplusplus
extern "C" {
#endif

// Non-blocking pipe: fds[0]=read end (PIPE_NOWAIT), fds[1]=write end.
// Returns 0 on success, -1 on failure (matches POSIX pipe()'s contract).
// 非ブロッキング pipe: fds[0]=読み出し端(PIPE_NOWAIT)、fds[1]=書き込み端。
// 成功で0、失敗で-1（POSIX pipe() と同じ契約）。
static inline int sil_win_pipe(int fds[2])
{
    char name[128];
    static volatile long s_counter = 0;
    long id = InterlockedIncrement(&s_counter);
    wsprintfA(name, "\\\\.\\pipe\\sil_cli_%lu_%ld", GetCurrentProcessId(), id);

    HANDLE server = CreateNamedPipeA(
        name, PIPE_ACCESS_DUPLEX,
        PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
        1, 4096, 4096, 0, NULL);
    if (server == INVALID_HANDLE_VALUE) return -1;

    HANDLE client = CreateFileA(name, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                OPEN_EXISTING, 0, NULL);
    if (client == INVALID_HANDLE_VALUE) { CloseHandle(server); return -1; }
    ConnectNamedPipe(server, NULL);  // client already opened -- completes immediately

    DWORD mode = PIPE_READMODE_BYTE | PIPE_NOWAIT;
    if (!SetNamedPipeHandleState(server, &mode, NULL, NULL)) {
        CloseHandle(server); CloseHandle(client); return -1;
    }

    int rfd = _open_osfhandle((intptr_t)server, _O_RDONLY | _O_BINARY);
    int wfd = _open_osfhandle((intptr_t)client, _O_WRONLY | _O_BINARY);
    if (rfd < 0 || wfd < 0) {
        CloseHandle(server); CloseHandle(client); return -1;
    }
    fds[0] = rfd;
    fds[1] = wfd;
    return 0;
}
#define pipe(fds) sil_win_pipe(fds)

// fsync(): flush a file descriptor's buffers to disk. Used by
// firmware/vehicle_old's serial_cli.cpp after writing to STDOUT_FILENO.
// _commit() is mingw-w64's equivalent (via the CRT, not a raw Win32 call).
// fsync(): fd のバッファをディスクへ反映。firmware/vehicle_old の
// serial_cli.cpp が STDOUT_FILENO 書き込み後に使用。_commit() が mingw-w64 の
// 相当関数（生 Win32 でなく CRT 経由）。
static inline int fsync(int fd) { return _commit(fd); }

#ifdef __cplusplus
}
#endif

#endif  // _WIN32 && !SIL_WIN_PIPE_DEFINED
