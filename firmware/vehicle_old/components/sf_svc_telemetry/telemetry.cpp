/**
 * @file telemetry.cpp
 * @brief WiFi WebSocket Telemetry Implementation
 */

#include "telemetry.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <cstring>
#include <unistd.h>  // for close()

static const char* TAG = "Telemetry";

// Embedded files
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t three_min_js_start[] asm("_binary_three_min_js_start");
extern const uint8_t three_min_js_end[] asm("_binary_three_min_js_end");

namespace stampfly {

// Singleton instance pointer for static callbacks
static Telemetry* s_instance = nullptr;

Telemetry& Telemetry::getInstance()
{
    static Telemetry instance;
    return instance;
}

esp_err_t Telemetry::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing WebSocket telemetry server...");

    config_ = config;
    s_instance = this;

    // Create client mutex
    client_mutex_ = xSemaphoreCreateMutex();
    if (client_mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize client FDs
    for (int i = 0; i < MAX_CLIENTS; i++) {
        client_fds_[i] = -1;
    }

    // Start HTTP server
    httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
    httpd_config.server_port = config_.port;
    httpd_config.max_open_sockets = MAX_CLIENTS + 2;  // +2 for HTTP requests headroom
    httpd_config.lru_purge_enable = true;
    // Close callback to auto-cleanup disconnected WebSocket clients
    // クローズコールバックで切断されたWebSocketクライアントを自動クリーンアップ
    httpd_config.close_fn = close_handler;

    esp_err_t ret = httpd_start(&server_, &httpd_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        vSemaphoreDelete(client_mutex_);
        client_mutex_ = nullptr;
        return ret;
    }

    // Register URI handlers

    // Root page (HTML)
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_get_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &uri_root);

    // Three.js library
    httpd_uri_t uri_threejs = {
        .uri = "/three.min.js",
        .method = HTTP_GET,
        .handler = http_get_threejs_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &uri_threejs);

    // Captive portal detection handler (macOS/iOS)
    // macOS sends GET /hotspot-detect.html after WiFi AP connection.
    // Returning 200 with expected body tells macOS the network is usable,
    // preventing it from blocking TCP traffic to this AP.
    // macOS は WiFi AP 接続後に /hotspot-detect.html にアクセスする。
    // 期待されるレスポンスを返すことで、macOS がこの AP への TCP をブロックするのを防ぐ。
    httpd_uri_t uri_captive = {
        .uri = "/hotspot-detect.html",
        .method = HTTP_GET,
        .handler = captive_portal_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &uri_captive);

    // Android/Windows captive portal detection
    httpd_uri_t uri_generate_204 = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = captive_portal_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server_, &uri_generate_204);

    // WebSocket endpoint
    httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = nullptr,
        .is_websocket = true,
        .handle_ws_control_frames = true
    };
    httpd_register_uri_handler(server_, &uri_ws);

    initialized_ = true;
    ESP_LOGI(TAG, "WebSocket telemetry server started on port %d", config_.port);
    ESP_LOGI(TAG, "  Open http://192.168.4.1/ in browser");

    return ESP_OK;
}

esp_err_t Telemetry::stop()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }

    if (client_mutex_ != nullptr) {
        vSemaphoreDelete(client_mutex_);
        client_mutex_ = nullptr;
    }

    initialized_ = false;
    client_count_ = 0;
    s_instance = nullptr;

    ESP_LOGI(TAG, "Telemetry server stopped");
    return ESP_OK;
}

void Telemetry::close_handler(httpd_handle_t hd, int sockfd)
{
    // Called by httpd when any socket is closed (including LRU purge)
    // httpd がソケットをクローズする時に呼ばれる（LRU purge 含む）
    if (s_instance != nullptr) {
        s_instance->removeClient(sockfd);
    }
    // Default close behavior
    close(sockfd);
}

esp_err_t Telemetry::captive_portal_handler(httpd_req_t* req)
{
    // macOS expects: HTTP 200 with body containing "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"
    // iOS expects the same. Android expects 204 No Content.
    // Returning the Apple-expected response satisfies all platforms.
    // macOS/iOS: 200 + "Success" ボディで captive portal 検出を通過
    ESP_LOGI(TAG, "Captive portal check: %s", req->uri);
    const char* response = "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

esp_err_t Telemetry::http_get_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "HTTP GET %s", req->uri);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

esp_err_t Telemetry::http_get_threejs_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "HTTP GET %s", req->uri);

    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)three_min_js_start,
                    three_min_js_end - three_min_js_start);
    return ESP_OK;
}

esp_err_t Telemetry::ws_handler(httpd_req_t* req)
{
    if (s_instance == nullptr) {
        return ESP_FAIL;
    }

    // Get socket fd
    int fd = httpd_req_to_sockfd(req);

    // Handle WebSocket handshake (HTTP GET request)
    if (req->method == HTTP_GET) {
        // Detect log client via query parameter: /ws?mode=log
        // クエリパラメータでログクライアントを識別: /ws?mode=log
        // (Subprotocol headers are not reliably available in ESP-IDF httpd
        //  during WebSocket upgrade, so we use query params instead)
        bool is_log_client = false;
        char query_buf[32] = {};
        if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) == ESP_OK) {
            char mode_val[8] = {};
            if (httpd_query_key_value(query_buf, "mode", mode_val, sizeof(mode_val)) == ESP_OK) {
                is_log_client = (strcmp(mode_val, "log") == 0);
            }
        }

        if (is_log_client) {
            ESP_LOGI(TAG, "Log client handshake (fd=%d) — entering exclusive mode", fd);
            // Disconnect existing non-log clients for exclusive access
            // 専用アクセスのため既存の非ログクライアントを切断
            s_instance->disconnectNonLogClients();
        } else if (s_instance->log_client_fd_ >= 0) {
            // Reject non-log connections while exclusive log mode is active.
            // Browser auto-reconnects after onclose, so we must refuse here.
            // 専用ログモード中は非ログ接続を拒否。
            // ブラウザは onclose 後に自動再接続するため、ここで拒否が必要。
            ESP_LOGI(TAG, "Rejecting non-log client (fd=%d) — log mode active", fd);
            httpd_resp_set_status(req, "503 Service Unavailable");
            httpd_resp_send(req, "Log capture in progress", -1);
            return ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "WebSocket handshake from client (fd=%d)", fd);
        }

        // Register client on successful handshake
        s_instance->addClient(fd, is_log_client);
        return ESP_OK;
    }

    // Receive frame
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // First call to get frame info
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(ret));
        s_instance->removeClient(fd);
        return ret;
    }

    // Handle different frame types
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Client disconnected (fd=%d)", fd);
        s_instance->removeClient(fd);
        return ESP_OK;
    }

    // Allocate buffer and receive payload
    if (ws_pkt.len > 0) {
        uint8_t* buf = (uint8_t*)malloc(ws_pkt.len + 1);
        if (buf == nullptr) {
            return ESP_ERR_NO_MEM;
        }

        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }

        // Handle text message (future: command parsing)
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            buf[ws_pkt.len] = '\0';
            ESP_LOGD(TAG, "Received: %s", buf);
        }

        free(buf);
    }

    return ESP_OK;
}

void Telemetry::addClient(int fd, bool is_log)
{
    xSemaphoreTake(client_mutex_, portMAX_DELAY);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_fds_[i] == -1) {
            client_fds_[i] = fd;
            client_is_log_[i] = is_log;
            client_count_++;
            if (is_log) {
                log_client_fd_ = fd;
            }
            // Signal telemetry task to resync read_index to latest
            // テレメトリタスクに read_index を最新位置にリセットするよう通知
            needs_resync_.store(true);
            ESP_LOGI(TAG, "Client connected (fd=%d, log=%d, total=%d)",
                     fd, is_log, client_count_);
            break;
        }
    }

    xSemaphoreGive(client_mutex_);
}

void Telemetry::removeClient(int fd)
{
    xSemaphoreTake(client_mutex_, portMAX_DELAY);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_fds_[i] == fd) {
            client_fds_[i] = -1;
            client_is_log_[i] = false;
            client_count_--;
            if (log_client_fd_ == fd) {
                log_client_fd_ = -1;
                ESP_LOGI(TAG, "Log client disconnected — exclusive mode ended");
            }
            ESP_LOGI(TAG, "Client disconnected (fd=%d, total=%d)", fd, client_count_);
            break;
        }
    }

    xSemaphoreGive(client_mutex_);
}

void Telemetry::disconnectNonLogClients()
{
    // Called BEFORE addClient for the new log client, so mutex is not held yet.
    // 新ログクライアントの addClient 前に呼ばれるため、mutex はまだ保持していない。
    xSemaphoreTake(client_mutex_, portMAX_DELAY);

    // Collect fds to close (close outside mutex to avoid deadlock with close_handler)
    // クローズ対象の fd を収集（close_handler とのデッドロック防止のため mutex 外でクローズ）
    int fds_to_close[MAX_CLIENTS];
    int close_count = 0;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        int fd = client_fds_[i];
        if (fd == -1 || client_is_log_[i]) continue;

        ESP_LOGI(TAG, "Disconnecting non-log client fd=%d for exclusive log mode", fd);
        fds_to_close[close_count++] = fd;

        client_fds_[i] = -1;
        client_is_log_[i] = false;
        client_count_--;
    }

    xSemaphoreGive(client_mutex_);

    // Close sockets outside mutex via httpd API.
    // Direct close(fd) causes EBADF in httpd's select() loop, so we use
    // httpd_sess_trigger_close which schedules a clean close on next tick.
    // The browser will auto-reconnect, but ws_handler now rejects non-log
    // clients while log mode is active.
    // mutex 外で httpd API 経由でソケットをクローズ。
    // 直接 close(fd) すると httpd の select() で EBADF が発生するため、
    // httpd_sess_trigger_close で次のティックでクリーンにクローズする。
    // ブラウザは自動再接続するが、ws_handler がログモード中は拒否する。
    for (int i = 0; i < close_count; i++) {
        httpd_sess_trigger_close(server_, fds_to_close[i]);
    }
}

int Telemetry::broadcast(const void* data, size_t len)
{
    if (!initialized_ || server_ == nullptr) {
        return 0;
    }

    int sent_count = 0;

    // Use finite timeout to prevent telemetry task from blocking
    // テレメトリタスクのブロックを防ぐため有限タイムアウトを使用
    if (xSemaphoreTake(client_mutex_, pdMS_TO_TICKS(5)) != pdTRUE) {
        // Mutex busy (client connect/disconnect in progress) — drop this frame
        // Mutex取得失敗（クライアント接続/切断中）— フレームをドロップ
        drop_count_++;
        return 0;
    }

    for (int i = 0; i < MAX_CLIENTS; i++) {
        int fd = client_fds_[i];
        if (fd == -1) continue;

        // Send binary frame
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)data;
        ws_pkt.len = len;
        ws_pkt.type = HTTPD_WS_TYPE_BINARY;
        ws_pkt.final = true;

        esp_err_t ret = httpd_ws_send_frame_async(server_, fd, &ws_pkt);
        if (ret == ESP_OK) {
            sent_count++;
        } else {
            // Any send failure removes the client to prevent socket leak
            // 送信失敗時はソケットリークを防ぐためクライアントを即座に除去
            ESP_LOGI(TAG, "Client fd=%d send failed (%s), removing",
                     fd, esp_err_to_name(ret));
            client_fds_[i] = -1;
            client_count_--;
        }
    }

    xSemaphoreGive(client_mutex_);

    return sent_count;
}

}  // namespace stampfly
