/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file lwip/sockets.h
 * @brief Self-contained INERT BSD-socket shim for the SIL (no real network).
 *        SIL 用の自己完結 inert BSD ソケットシム（実ネットワーク無し）。
 *
 * The SIL has no network (RESET_PLAN §11 — the radio/comm physical layer cannot
 * be reproduced). The firmware's UDP/TCP/telemetry/CLI tasks still RUN here, but
 * against inert sockets: send/sendto are dropped, and the BLOCKING receivers
 * (recvfrom/recv/accept/select) YIELD ~1 tick and report "no data" so they do not
 * stall the single-token cooperative scheduler (a real blocking recvfrom would
 * hold the run-token in the kernel and the scheduler's hang detector would fire).
 * Self-contained on purpose: we do NOT pull the host <sys/socket.h>, so there is
 * no clash between our inert functions and the real libc ones.
 *
 * SIL に網は無い（§11）。UDP/TCP/telemetry/CLI タスクは inert ソケットで走る: 送信は破棄、
 * ブロッキング受信は ~1tick yield して「データなし」を返す（協調スケジューラを止めない）。
 * ホストの <sys/socket.h> を取り込まない自己完結ゆえ libc と衝突しない。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

// struct timeval / fd_set are defined HERE rather than pulled from host
// <sys/time.h> / <sys/select.h>, so this shim stays self-contained and never
// clashes with the host socket stack. Guard timeval against the macOS SDK's
// own definition (it uses the _STRUCT_TIMEVAL sentinel).
// timeval / fd_set はここで自前定義（ホスト <sys/time.h>/<sys/select.h> を引かない）。
#ifndef _STRUCT_TIMEVAL
#define _STRUCT_TIMEVAL
struct timeval { long tv_sec; long tv_usec; };
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
#define F_GETFL        3
#define F_SETFL        4
#define O_NONBLOCK     0x0004

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

// Blocking receivers — the SIL has no network, so no datagram or connection
// ever arrives. We model this faithfully: the call BLOCKS (cooperatively yields
// for a very long virtual interval), exactly like a real blocking socket waiting
// on a peer that never speaks. The task runs its setup + first receive, then
// parks for the rest of the run — no busy-loop, and no spurious EAGAIN error
// spam from server loops that treat a non-blocking -1 as a failure to log.
// 接続もデータも来ないので「ブロック」を忠実に再現（長時間 yield）。各タスクは
// セットアップ＋初回受信まで走って以後パークする（ビジーループも EAGAIN 連発もなし）。
#define SIL_SOCK_BLOCK_TICKS ((uint32_t)0x40000000)   // ~12 virtual days (1 tick = 1 ms)
static inline long recvfrom(int s, void* buf, size_t len, int flags, struct sockaddr* from, socklen_t* fl)
{ (void)s; (void)buf; (void)len; (void)flags; (void)from; (void)fl; vTaskDelay(SIL_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }
static inline long recv(int s, void* buf, size_t len, int flags)
{ (void)s; (void)buf; (void)len; (void)flags; vTaskDelay(SIL_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }
static inline int accept(int s, struct sockaddr* a, socklen_t* l)
{ (void)s; (void)a; (void)l; vTaskDelay(SIL_SOCK_BLOCK_TICKS); errno = EAGAIN; return -1; }

// Senders — pretend success (dropped on the wire).
static inline long sendto(int s, const void* buf, size_t len, int flags, const struct sockaddr* to, socklen_t tl)
{ (void)s; (void)buf; (void)flags; (void)to; (void)tl; return (long)len; }
static inline long send(int s, const void* buf, size_t len, int flags)
{ (void)s; (void)buf; (void)flags; return (long)len; }

#ifdef __cplusplus
}
#endif
