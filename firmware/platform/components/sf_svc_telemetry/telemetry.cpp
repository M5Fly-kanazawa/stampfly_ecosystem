/**
 * @file telemetry.cpp
 * @brief WiFi WebSocket Telemetry Implementation
 */

#include "telemetry.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <cstring>

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
    httpd_config.max_open_sockets = MAX_CLIENTS + 1;  // +1 for HTTP requests
    httpd_config.lru_purge_enable = true;

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
        ESP_LOGI(TAG, "WebSocket handshake from client (fd=%d)", fd);
        // Register client on successful handshake
        s_instance->addClient(fd);
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

void Telemetry::addClient(int fd)
{
    xSemaphoreTake(client_mutex_, portMAX_DELAY);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_fds_[i] == -1) {
            client_fds_[i] = fd;
            client_count_++;
            ESP_LOGI(TAG, "Client connected (fd=%d, total=%d)", fd, client_count_);
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
            client_count_--;
            ESP_LOGI(TAG, "Client disconnected (fd=%d, total=%d)", fd, client_count_);
            break;
        }
    }

    xSemaphoreGive(client_mutex_);
}

int Telemetry::broadcast(const void* data, size_t len)
{
    if (!initialized_ || server_ == nullptr) {
        return 0;
    }

    int sent_count = 0;

    xSemaphoreTake(client_mutex_, portMAX_DELAY);

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
        } else if (ret == ESP_ERR_INVALID_ARG) {
            // Client no longer valid - remove
            ESP_LOGI(TAG, "Client fd=%d no longer valid, removing", fd);
            client_fds_[i] = -1;
            client_count_--;
        }
        // Other errors (e.g., EAGAIN) - keep client, retry next time
    }

    xSemaphoreGive(client_mutex_);

    return sent_count;
}

}  // namespace stampfly
