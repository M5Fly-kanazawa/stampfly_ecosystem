/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file arpa/inet.h
 * @brief Host shim for <arpa/inet.h> — redirects to the inert lwIP socket shim.
 *        <arpa/inet.h> のホスト用シム — inert lwIP ソケットシムへ転送。
 *
 * On ESP-IDF, <arpa/inet.h> is supplied by lwIP and exposes inet_addr/htons/etc.
 * The firmware includes it directly (e.g. cmd_wifi_bench.cpp). On the host we
 * must NOT fall through to the macOS SDK's <arpa/inet.h> (it pulls the real
 * <sys/socket.h>, whose struct sockaddr / in_addr clash with our inert types).
 * Forwarding to lwip/sockets.h gives the firmware the same symbols, inert.
 *
 * ESP-IDF では <arpa/inet.h> は lwIP 提供で inet_addr/htons 等を公開する。ホストでは
 * macOS SDK 版（実 <sys/socket.h> を引き型が衝突）に落とさず lwip/sockets.h へ転送する。
 */

#pragma once

#include "lwip/sockets.h"
