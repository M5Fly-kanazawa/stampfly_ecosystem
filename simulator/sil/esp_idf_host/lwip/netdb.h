/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file lwip/netdb.h
 * @brief Self-contained INERT name-resolution shim for the SIL (no real DNS).
 *        SIL 用の自己完結 inert 名前解決シム（実 DNS 無し）。
 *
 * Self-contained on purpose: it does NOT include the host <netdb.h>, because
 * that pulls the real <sys/socket.h> (with a struct sockaddr / sa_family_t that
 * differ from our inert lwip/sockets.h types) and would clash. The firmware's
 * resolver calls run but resolve nothing — consistent with "the SIL has no
 * network" (RESET_PLAN §11). getaddrinfo reports failure; gethostbyname returns
 * NULL. Any code that proceeds to send() simply drops on the inert socket.
 *
 * 自己完結ゆえホストの <netdb.h>（実 <sys/socket.h> を引き sockaddr/sa_family_t が
 * シムと衝突する）を取り込まない。リゾルバ呼び出しは走るが何も解決しない（§11: 網無し）。
 */

#pragma once

#include "lwip/sockets.h"   // our inert sockaddr / sockaddr_in types

#ifdef __cplusplus
extern "C" {
#endif

#define EAI_NONAME      200
#define EAI_FAIL        202
#define AI_PASSIVE      0x01
#define AI_CANONNAME    0x02
#define AI_NUMERICHOST  0x04

struct addrinfo {
    int               ai_flags;
    int               ai_family;
    int               ai_socktype;
    int               ai_protocol;
    socklen_t         ai_addrlen;
    struct sockaddr*  ai_addr;
    char*             ai_canonname;
    struct addrinfo*  ai_next;
};

struct hostent {
    char*  h_name;
    char** h_aliases;
    int    h_addrtype;
    int    h_length;
    char** h_addr_list;
};

// Inert: resolve nothing (no DNS on the SIL).
// inert: 何も解決しない（SIL に DNS 無し）。
static inline int getaddrinfo(const char* node, const char* service,
                              const struct addrinfo* hints, struct addrinfo** res)
{ (void)node; (void)service; (void)hints; if (res) *res = NULL; return EAI_FAIL; }
static inline void freeaddrinfo(struct addrinfo* res) { (void)res; }
static inline struct hostent* gethostbyname(const char* name) { (void)name; return NULL; }
static inline const char* gai_strerror(int e) { (void)e; return "no DNS on SIL"; }

#ifndef lwip_getaddrinfo
#define lwip_getaddrinfo   getaddrinfo
#endif
#ifndef lwip_freeaddrinfo
#define lwip_freeaddrinfo  freeaddrinfo
#endif
#ifndef lwip_gethostbyname
#define lwip_gethostbyname gethostbyname
#endif

#ifdef __cplusplus
}
#endif
