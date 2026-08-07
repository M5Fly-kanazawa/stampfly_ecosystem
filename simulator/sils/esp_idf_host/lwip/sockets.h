/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file lwip/sockets.h
 * @brief Self-contained INERT BSD-socket shim for the SILS (no real network).
 *        SILS 用の自己完結 inert BSD ソケットシム（実ネットワーク無し）。
 *
 * The SILS has no network (RESET_PLAN §11 — the radio/comm physical layer cannot
 * be reproduced). The firmware's UDP/TCP/telemetry/CLI tasks still RUN here, but
 * against inert sockets: send/sendto are dropped, and the BLOCKING receivers
 * (recvfrom/recv/accept/select) YIELD ~1 tick and report "no data" so they do not
 * stall the single-token cooperative scheduler (a real blocking recvfrom would
 * hold the run-token in the kernel and the scheduler's hang detector would fire).
 * Self-contained on purpose: we do NOT pull the host <sys/socket.h>, so there is
 * no clash between our inert functions and the real libc ones.
 *
 * SILS に網は無い（§11）。UDP/TCP/telemetry/CLI タスクは inert ソケットで走る: 送信は破棄、
 * ブロッキング受信は ~1tick yield して「データなし」を返す（協調スケジューラを止めない）。
 * ホストの <sys/socket.h> を取り込まない自己完結ゆえ libc と衝突しない。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

// struct timeval: on Windows (MinGW) we self-define it here rather than
// pulling <sys/time.h>, so this shim stays self-contained and never clashes
// with the host socket stack. On Linux/macOS we instead use the SYSTEM
// <sys/time.h>: glibc's <bits/types/struct_timeval.h> (transitively pulled in
// by many standard headers, e.g. <cstdlib>, via <sys/select.h>) defines the
// real struct timeval unconditionally under its own guard (__timeval_defined),
// which our local definition does not set — so a local re-definition causes
// "redefinition of struct timeval" at compile time on Linux. Including
// <sys/time.h> ourselves gives every TU the same real type with no clash, and
// is a no-op if some other header already pulled it in first. macOS's SDK
// guards its definition with the _STRUCT_TIMEVAL sentinel too, so this stays
// compatible with the mutual-guard scheme that was already in place there.
// fd_set is intentionally NOT provided on any platform (see below).
//
// struct timeval: Windows(MinGW)ではここで自前定義する（<sys/time.h>を引かず
// 自己完結を保つ）。Linux/macOSではシステムの<sys/time.h>を使う: glibcの
// <bits/types/struct_timeval.h>（<cstdlib>等の標準ヘッダから<sys/select.h>
// 経由で間接的に取り込まれる）が本物のstruct timevalを、こちらの自前定義とは
// 別のガード（__timeval_defined）で無条件に定義するため、ローカルに再定義すると
// Linuxでは「redefinition of struct timeval」になる。自前で<sys/time.h>を
// includeしておけば、どのTUでも同じ本物の型が使え、他ヘッダが先に引き込んで
// いても no-op になる。macOS SDK側も_STRUCT_TIMEVALで相互ガードしているので、
// 従来の仕組みとの互換性も保たれる。fd_setはどのプラットフォームでも意図的に
// 提供しない（下記参照）。
#if defined(_WIN32)
#ifndef _STRUCT_TIMEVAL
#define _STRUCT_TIMEVAL
struct timeval { long tv_sec; long tv_usec; };
#endif
#else
#include <sys/time.h>
#endif

// Cooperative yield so a task polling an inert socket does not busy-loop.
#ifdef __cplusplus
extern "C" void vTaskDelay(uint32_t ticks);
#else
void vTaskDelay(uint32_t ticks);
#endif

#ifdef __cplusplus
extern "C" {
#endif

// --- address family / socket type / protocol / options ----------------------
#define AF_INET        2
#define AF_UNSPEC      0
#define PF_INET        AF_INET
#define SOCK_STREAM    1
#define SOCK_DGRAM     2
#define IPPROTO_IP     0
#define IPPROTO_TCP    6
#define IPPROTO_UDP    17
#define SOL_SOCKET     0xfff
#define SO_REUSEADDR   0x0004
#define SO_BROADCAST   0x0020
#define SO_KEEPALIVE   0x0008
#define SO_RCVTIMEO    0x1006
#define SO_SNDTIMEO    0x1005
#define TCP_NODELAY    0x01
#define INADDR_ANY        ((uint32_t)0x00000000UL)
#define INADDR_BROADCAST  ((uint32_t)0xffffffffUL)
#define INADDR_NONE       ((uint32_t)0xffffffffUL)
#define MSG_DONTWAIT   0x08
#define SHUT_RDWR      2
// F_GETFL/F_SETFL/O_NONBLOCK + the fcntl() stub below exist so firmware's
// non-blocking-socket pattern (fcntl(sock, F_GETFL, 0); fcntl(sock, F_SETFL,
// flags|O_NONBLOCK);) compiles and no-ops against our fake socket() ints,
// without ever handing those fake fds to the real libc fcntl() (see the
// fcntl() comment below for why). CAUTION: a translation unit that includes
// BOTH this header AND the real <fcntl.h> gets a hard compile error on
// Linux/macOS — see the fcntl() stub's comment below. Prefer relying on this
// header alone (as telemetry.cpp / api_task.cpp already do) over also
// including <fcntl.h>.
// F_GETFL/F_SETFL/O_NONBLOCK と下の fcntl() スタブは、firmware の非ブロッキング
// ソケットパターン（fcntl(sock, F_GETFL, 0); fcntl(sock, F_SETFL,
// flags|O_NONBLOCK);）をコンパイル・no-op させるため（下の fcntl() のコメント
// 参照 — 偽 fd を実 libc の fcntl() へ渡さない）。注意: 本ヘッダと本物の
// <fcntl.h> の両方を同一翻訳単位で include すると Linux/macOS でコンパイル
// エラーになる（下記参照）。<fcntl.h> を追加せず本ヘッダ単体に頼ること
// （telemetry.cpp / api_task.cpp は既にそうしている）。
#define F_GETFL        3
#define F_SETFL        4
#define O_NONBLOCK     0x0004
#define MSG_DONTWAIT   0x08      /* lwip value; per-call non-blocking receive */

typedef uint32_t socklen_t;
typedef uint16_t sa_family_t;
typedef uint16_t in_port_t;

struct in_addr { uint32_t s_addr; };
struct sockaddr     { uint8_t sa_len; sa_family_t sa_family; char sa_data[14]; };
struct sockaddr_in  { uint8_t sin_len; sa_family_t sin_family; in_port_t sin_port;
                      struct in_addr sin_addr; char sin_zero[8]; };
struct sockaddr_storage { uint8_t s2_len; sa_family_t ss_family; char s2_data[26]; };

// fd_set / select are intentionally NOT provided: the firmware never calls
// select() (only mentioned in comments), and defining our own fd_set clashes
// with the host's (pulled transitively by C++ headers such as <functional>).
// fd_set / select は提供しない（firmware は select() を呼ばず、自前 fd_set は
// <functional> 等が引く host の fd_set と衝突するため）。

// --- byte order / address conversion ----------------------------------------
static inline uint16_t lwip_htons(uint16_t x) { return (uint16_t)((x >> 8) | (x << 8)); }
static inline uint32_t lwip_htonl(uint32_t x)
{ return ((x & 0xff) << 24) | ((x & 0xff00) << 8) | ((x >> 8) & 0xff00) | ((x >> 24) & 0xff); }
#define htons(x) lwip_htons((uint16_t)(x))
#define htonl(x) lwip_htonl((uint32_t)(x))
#define ntohs(x) lwip_htons((uint16_t)(x))
#define ntohl(x) lwip_htonl((uint32_t)(x))

static inline uint32_t inet_addr(const char* cp)
{
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (!cp) return INADDR_NONE;
    if (sscanf(cp, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return INADDR_NONE;
    return lwip_htonl((a << 24) | (b << 16) | (c << 8) | d);
}
static inline int inet_aton(const char* cp, struct in_addr* addr)
{ uint32_t v = inet_addr(cp); if (addr) addr->s_addr = v; return v != INADDR_NONE; }
static inline char* inet_ntoa(struct in_addr in)
{
    static char buf[16]; uint32_t h = lwip_htonl(in.s_addr);
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             (h >> 24) & 0xff, (h >> 16) & 0xff, (h >> 8) & 0xff, h & 0xff);
    return buf;
}
static inline char* inet_ntoa_r(struct in_addr in, char* buf, int buflen)
{ const char* s = inet_ntoa(in); if (buf && buflen > 0) { strncpy(buf, s, buflen - 1); buf[buflen-1] = 0; } return buf; }
static inline const char* inet_ntop(int af, const void* src, char* dst, socklen_t size)
{ (void)af; struct in_addr a; a.s_addr = *(const uint32_t*)src; return inet_ntoa_r(a, dst, (int)size); }
static inline int inet_pton(int af, const char* src, void* dst)
{ (void)af; struct in_addr a; int ok = inet_aton(src, &a); *(uint32_t*)dst = a.s_addr; return ok; }

// --- inert socket calls -----------------------------------------------------
static inline int socket(int domain, int type, int protocol)
{ (void)domain; (void)type; (void)protocol; static int next = 3; return next++; }
static inline int bind(int s, const struct sockaddr* a, socklen_t l) { (void)s; (void)a; (void)l; return 0; }
static inline int connect(int s, const struct sockaddr* a, socklen_t l) { (void)s; (void)a; (void)l; return 0; }
static inline int listen(int s, int backlog) { (void)s; (void)backlog; return 0; }
static inline int setsockopt(int s, int lvl, int opt, const void* v, socklen_t l) { (void)s; (void)lvl; (void)opt; (void)v; (void)l; return 0; }
static inline int getsockopt(int s, int lvl, int opt, void* v, socklen_t* l) { (void)s; (void)lvl; (void)opt; (void)v; (void)l; return 0; }
static inline int shutdown(int s, int how) { (void)s; (void)how; return 0; }
// FIXED 3-arg signature by design (matches every call site in this codebase:
// fcntl(fd, F_GETFL, 0) / fcntl(fd, F_SETFL, flags)). The real libc fcntl()
// is variadic — int fcntl(int, int, ...) — so if a TU ALSO includes the real
// <fcntl.h>, that declaration conflicts with this one ("conflicting
// declaration of C function") on Linux/macOS. This is intentional and
// unavoidable without either (a) calling the real fcntl() on our fake
// socket() ints — which could alias a real, unrelated file descriptor at the
// same small integer value and mutate ITS flags — or (b) a fragile
// macro-rename trick (tried and confirmed broken: renaming via #define still
// re-declares the target under the real declaration's linkage/exception-spec,
// which mismatches ours). See the F_GETFL comment above for the "don't also
// include <fcntl.h>" rule this implies.
static inline int fcntl(int s, int cmd, int arg) { (void)s; (void)cmd; (void)arg; return 0; }
static inline int lwip_close(int s) { (void)s; return 0; }
#define closesocket(s) lwip_close(s)
// ESP-IDF's lwip publishes POSIX names (LWIP_POSIX_SOCKETS_IO_NAMES); firmware
// calls bare close(sock). Map it to the inert lwip_close so a fake socket fd is
// NEVER passed to the host close() (which would shut a real file descriptor).
// Only TUs that include lwip/sockets.h see this macro; serial/UART code that
// uses the real close() does not include this header, so there is no clash.
// firmware は素の close(sock) を呼ぶ。inert な lwip_close へ写像（偽 fd を実 close に渡さない）。
#ifndef close
#define close(s) lwip_close(s)
#endif

// Blocking receivers — the SILS has no network, so no datagram or connection
// ever arrives. We model this faithfully: the call BLOCKS (cooperatively yields
// for a very long virtual interval), exactly like a real blocking socket waiting
// on a peer that never speaks. The task runs its setup + first receive, then
// parks for the rest of the run — no busy-loop, and no spurious EAGAIN error
// spam from server loops that treat a non-blocking -1 as a failure to log.
// 接続もデータも来ないので「ブロック」を忠実に再現（長時間 yield）。各タスクは
// セットアップ＋初回受信まで走って以後パークする（ビジーループも EAGAIN 連発もなし）。
#define SILS_SOCK_BLOCK_TICKS ((uint32_t)0x40000000)   // ~12 virtual days (1 tick = 1 ms)
// MSG_DONTWAIT callers poll (their loop has other work — e.g. the ApiTask
// drains scenario injections); report "no data" after a 1-tick yield instead
// of parking. Blocking callers park as before.
// MSG_DONTWAIT の呼び出し側はポーリング（ループに他の仕事がある — 例: ApiTask の
// シナリオ注入 drain）。パークせず 1 tick yield して「データなし」を返す。
// ブロッキング呼び出しは従来どおりパーク。
static inline long recvfrom(int s, void* buf, size_t len, int flags, struct sockaddr* from, socklen_t* fl)
{ (void)s; (void)buf; (void)len; (void)from; (void)fl;
  vTaskDelay((flags & MSG_DONTWAIT) ? 1 : SILS_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }
static inline long recv(int s, void* buf, size_t len, int flags)
{ (void)s; (void)buf; (void)len;
  vTaskDelay((flags & MSG_DONTWAIT) ? 1 : SILS_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }
static inline int accept(int s, struct sockaddr* a, socklen_t* l)
{ (void)s; (void)a; (void)l; vTaskDelay(SILS_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }

// Senders — pretend success (dropped on the wire).
static inline long sendto(int s, const void* buf, size_t len, int flags, const struct sockaddr* to, socklen_t tl)
{ (void)s; (void)buf; (void)flags; (void)to; (void)tl; return (long)len; }
static inline long send(int s, const void* buf, size_t len, int flags)
{ (void)s; (void)buf; (void)flags; return (long)len; }

#ifdef __cplusplus
}
#endif
