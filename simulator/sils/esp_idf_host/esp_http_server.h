/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_http_server.h
 * @brief Host shim for the ESP-IDF HTTP server (old firmware web/telemetry).
 *        ESP-IDF HTTP サーバのホスト用シム（旧ファームの web/telemetry）。
 *
 * Inert on the host: httpd_start hands back a non-null handle, register/send are
 * no-ops. The web/WS server has no real socket on the SILS (no browser client).
 * ホストでは inert。実ソケットなし（SILS にブラウザクライアントは無い）。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* httpd_handle_t;

typedef enum { HTTP_GET = 1, HTTP_POST = 3, HTTP_PUT = 4, HTTP_DELETE = 0 } httpd_method_t;

// Session open/close callbacks (the firmware assigns real functions to these).
// セッション open/close コールバック（本体が実関数を代入する）。
typedef esp_err_t (*httpd_open_func_t)(httpd_handle_t hd, int sockfd);
typedef void      (*httpd_close_func_t)(httpd_handle_t hd, int sockfd);

typedef struct httpd_req {
    httpd_handle_t handle;
    int            method;
    char           uri[128];
    size_t         content_len;
    void*          user_ctx;
    void*          sess_ctx;
} httpd_req_t;

typedef esp_err_t (*httpd_uri_handler_t)(httpd_req_t* r);

typedef struct {
    const char*          uri;
    httpd_method_t       method;
    httpd_uri_handler_t  handler;
    void*                user_ctx;
    int                  is_websocket;
    int                  handle_ws_control_frames;
    const char*          supported_subprotocol;
} httpd_uri_t;

typedef struct {
    uint16_t task_priority;
    size_t   stack_size;
    int      core_id;
    uint16_t server_port;
    uint16_t ctrl_port;
    uint16_t max_open_sockets;
    uint16_t max_uri_handlers;
    uint16_t max_resp_headers;
    uint16_t backlog_conn;
    int      lru_purge_enable;
    uint16_t recv_wait_timeout;
    uint16_t send_wait_timeout;
    void*    global_user_ctx;
    void*    global_transport_ctx;
    int      enable_so_linger;
    int      linger_timeout;
    int      keep_alive_enable;
    httpd_open_func_t  open_fn;
    httpd_close_func_t close_fn;
    void*    uri_match_fn;
} httpd_config_t;

#define HTTPD_DEFAULT_CONFIG() { 5, 4096, 0, 80, 32768, 7, 8, 8, 5, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0 }

typedef enum {
    HTTPD_WS_TYPE_CONTINUE = 0x0, HTTPD_WS_TYPE_TEXT = 0x1, HTTPD_WS_TYPE_BINARY = 0x2,
    HTTPD_WS_TYPE_CLOSE = 0x8, HTTPD_WS_TYPE_PING = 0x9, HTTPD_WS_TYPE_PONG = 0xA,
} httpd_ws_type_t;

typedef struct {
    int             final;
    int             fragmented;
    httpd_ws_type_t type;
    uint8_t*        payload;
    size_t          len;
} httpd_ws_frame_t;

static inline esp_err_t httpd_start(httpd_handle_t* h, const httpd_config_t* c)
{ (void)c; if (h) *h = (httpd_handle_t)0x1; return ESP_OK; }
static inline esp_err_t httpd_stop(httpd_handle_t h) { (void)h; return ESP_OK; }
static inline esp_err_t httpd_register_uri_handler(httpd_handle_t h, const httpd_uri_t* u) { (void)h; (void)u; return ESP_OK; }
static inline esp_err_t httpd_resp_send(httpd_req_t* r, const char* buf, ptrdiff_t len) { (void)r; (void)buf; (void)len; return ESP_OK; }
static inline esp_err_t httpd_resp_set_status(httpd_req_t* r, const char* s) { (void)r; (void)s; return ESP_OK; }
static inline esp_err_t httpd_resp_set_type(httpd_req_t* r, const char* t) { (void)r; (void)t; return ESP_OK; }
static inline esp_err_t httpd_req_get_url_query_str(httpd_req_t* r, char* buf, size_t n) { (void)r; if (buf && n) buf[0] = 0; return ESP_FAIL; }
static inline esp_err_t httpd_query_key_value(const char* qs, const char* key, char* val, size_t n) { (void)qs; (void)key; if (val && n) val[0] = 0; return ESP_FAIL; }
static inline int  httpd_req_to_sockfd(httpd_req_t* r) { (void)r; return -1; }
static inline esp_err_t httpd_sess_trigger_close(httpd_handle_t h, int sockfd) { (void)h; (void)sockfd; return ESP_OK; }
static inline esp_err_t httpd_ws_recv_frame(httpd_req_t* r, httpd_ws_frame_t* f, size_t n) { (void)r; (void)f; (void)n; return ESP_OK; }
static inline esp_err_t httpd_ws_send_frame_async(httpd_handle_t h, int fd, httpd_ws_frame_t* f) { (void)h; (void)fd; (void)f; return ESP_OK; }

#ifdef __cplusplus
}
#endif
