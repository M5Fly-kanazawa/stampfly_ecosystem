/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file lwip/sockets.h
 * @brief Host shim for ESP-IDF's lwIP BSD socket API
 *        ESP-IDF の lwIP BSD ソケット API のホスト用シム
 *
 * On the host we forward to the real POSIX socket stack so that
 * socket()/bind()/sendto()/recvfrom()/setsockopt() actually work
 * (UDP telemetry/blackbox can loopback for real). Only a handful of
 * lwIP-specific names that the firmware references are re-exported here.
 *
 * ホストでは本物の POSIX ソケットへ転送する。socket()/bind()/sendto()/
 * recvfrom()/setsockopt() は実際に動作し、UDP テレメトリ/ブラックボックスは
 * ループバックで実通信できる。本体が参照する lwIP 固有の名前だけを補う。
 */

#pragma once

/* Forward straight to the host BSD socket stack.
 * ホストの BSD ソケットスタックへそのまま転送する。 */
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* lwIP compile-time feature flags the firmware may #if on.
 * On the host these features are always present.
 * 本体が #if で参照しうる lwIP 機能フラグ。ホストでは常に有効。 */
#ifndef LWIP_SOCKET
#define LWIP_SOCKET 1
#endif
#ifndef LWIP_IPV4
#define LWIP_IPV4 1
#endif
#ifndef LWIP_IPV6
#define LWIP_IPV6 1
#endif
#ifndef LWIP_DNS
#define LWIP_DNS 1
#endif

/*
 * lwIP exposes the BSD calls under both bare names and lwip_-prefixed
 * names. The firmware uses the bare names (socket/bind/...), which the host
 * already provides. Provide the lwip_* aliases too, in case any code path
 * references them.
 *
 * lwIP は BSD 呼び出しを素の名前と lwip_ 接頭辞付きの両方で公開する。本体は
 * 素の名前（socket/bind/...）を使用しており、それらはホストが既に提供する。
 * 念のため lwip_* エイリアスも用意しておく。
 */
#ifndef lwip_socket
#define lwip_socket      socket
#endif
#ifndef lwip_bind
#define lwip_bind        bind
#endif
#ifndef lwip_connect
#define lwip_connect     connect
#endif
#ifndef lwip_listen
#define lwip_listen      listen
#endif
#ifndef lwip_accept
#define lwip_accept      accept
#endif
#ifndef lwip_send
#define lwip_send        send
#endif
#ifndef lwip_sendto
#define lwip_sendto      sendto
#endif
#ifndef lwip_recv
#define lwip_recv        recv
#endif
#ifndef lwip_recvfrom
#define lwip_recvfrom    recvfrom
#endif
#ifndef lwip_setsockopt
#define lwip_setsockopt  setsockopt
#endif
#ifndef lwip_getsockopt
#define lwip_getsockopt  getsockopt
#endif
#ifndef lwip_shutdown
#define lwip_shutdown    shutdown
#endif
#ifndef lwip_select
#define lwip_select      select
#endif
#ifndef lwip_fcntl
#define lwip_fcntl       fcntl
#endif
#ifndef lwip_close
#define lwip_close       close
#endif

/* lwIP defines closesocket() as an alias of close().
 * lwIP は closesocket() を close() のエイリアスとして定義する。 */
#ifndef closesocket
#define closesocket(s)   close(s)
#endif
