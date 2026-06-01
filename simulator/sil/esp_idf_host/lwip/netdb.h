/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file lwip/netdb.h
 * @brief Host shim for ESP-IDF's lwIP name-resolution API
 *        ESP-IDF の lwIP 名前解決 API のホスト用シム
 *
 * Forwards to the host <netdb.h> so getaddrinfo()/gethostbyname() resolve
 * against the real resolver. The firmware includes this header alongside
 * lwip/sockets.h; on the host both map onto the native stack.
 *
 * ホストの <netdb.h> へ転送し、getaddrinfo()/gethostbyname() を本物の
 * リゾルバで解決する。本体は lwip/sockets.h と併せて本ヘッダを include する。
 * ホストでは両者ともネイティブスタックへ写像される。
 */

#pragma once

/* Pull in the lwIP socket shim first (matches ESP-IDF, which makes the
 * socket types available before the resolver API).
 * 先に lwIP ソケットシムを取り込む（ESP-IDF と同様、リゾルバ API の前に
 * ソケット型を利用可能にする）。 */
#include "lwip/sockets.h"

/* Forward to the host name-resolution stack.
 * ホストの名前解決スタックへ転送する。 */
#include <netdb.h>

/* lwIP also publishes resolver calls under lwip_-prefixed names; provide
 * the aliases in case any path references them.
 * lwIP はリゾルバ呼び出しも lwip_ 接頭辞付きで公開する。念のため別名を用意。 */
#ifndef lwip_getaddrinfo
#define lwip_getaddrinfo   getaddrinfo
#endif
#ifndef lwip_freeaddrinfo
#define lwip_freeaddrinfo  freeaddrinfo
#endif
#ifndef lwip_gethostbyname
#define lwip_gethostbyname gethostbyname
#endif
