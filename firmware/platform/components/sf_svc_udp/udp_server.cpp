/**
 * @file udp_server.cpp
 * @brief UDP server implementation
 *        UDPサーバー実装
 */

#include "udp_server.hpp"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <cstring>

static const char* TAG = "UDPServer";

namespace stampfly {

UDPServer& UDPServer::getInstance() {
    static UDPServer instance;
    return instance;
}

esp_err_t UDPServer::init(const Config& config) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    config_ = config;

    // Create mutex
    // ミューテックスを作成
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized (control port: %d, telemetry port: %d)",
             config_.control_port, config_.telemetry_port);

    return ESP_OK;
}

esp_err_t UDPServer::start() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (running_) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    // Create UDP socket
    // UDPソケットを作成
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set socket options
    // ソケットオプションを設定
    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set receive timeout
    // 受信タイムアウトを設定
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Bind to control port
    // 制御ポートにバインド
    SockAddrIn server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(config_.control_port);

    if (bind(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: errno %d", errno);
        close(sock_fd_);
        sock_fd_ = -1;
        return ESP_FAIL;
    }

    running_ = true;

    // Create RX task
    // 受信タスクを作成
    BaseType_t ret = xTaskCreatePinnedToCore(
        rxTask,
        "udp_rx",
        RX_TASK_STACK_SIZE,
        this,
        RX_TASK_PRIORITY,
        &rx_task_handle_,
        1  // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        running_ = false;
        close(sock_fd_);
        sock_fd_ = -1;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Started on port %d", config_.control_port);
    return ESP_OK;
}

esp_err_t UDPServer::stop() {
    if (!running_) {
        return ESP_OK;
    }

    running_ = false;

    // Wait for task to finish
    // タスクの終了を待つ
    if (rx_task_handle_ != nullptr) {
        // Give task time to exit
        vTaskDelay(pdMS_TO_TICKS(200));
        rx_task_handle_ = nullptr;
    }

    // Close socket
    // ソケットを閉じる
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }

    // Clear clients
    // クライアントをクリア
    client_count_ = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients_[i].active = false;
    }

    ESP_LOGI(TAG, "Stopped");
    return ESP_OK;
}

bool UDPServer::hasActiveControl() const {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now - last_control_time_ms_) < config_.control_timeout_ms;
}

bool UDPServer::getLastControl(udp::ControlPacket& pkt) const {
    if (!hasActiveControl()) {
        return false;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        pkt = last_control_;
        xSemaphoreGive(mutex_);
        return true;
    }

    return false;
}

void UDPServer::sendTelemetry(const udp::TelemetryPacket& pkt) {
    if (!running_ || sock_fd_ < 0) {
        return;
    }

    // Remove stale clients periodically
    // 定期的に古いクライアントを削除
    removeStaleClients();

    // Send to all active clients
    // 全アクティブクライアントに送信
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients_[i].active) {
            // Set telemetry port for sending
            // 送信用にテレメトリポートを設定
            SockAddrIn dest = clients_[i].addr;
            dest.sin_port = htons(config_.telemetry_port);

            ssize_t sent = sendto(
                sock_fd_,
                &pkt,
                sizeof(pkt),
                0,
                (struct sockaddr*)&dest,
                sizeof(dest)
            );

            if (sent < 0) {
                ESP_LOGW(TAG, "Failed to send telemetry: errno %d", errno);
                error_count_++;
            } else {
                tx_count_++;
            }
        }
    }
}

void UDPServer::resetStats() {
    rx_count_ = 0;
    tx_count_ = 0;
    error_count_ = 0;
}

void UDPServer::rxTask(void* arg) {
    UDPServer* server = static_cast<UDPServer*>(arg);
    uint8_t buffer[RX_BUFFER_SIZE];
    SockAddrIn src_addr;
    socklen_t addr_len = sizeof(src_addr);

    ESP_LOGI(TAG, "RX task started");

    while (server->running_) {
        // Receive packet
        // パケットを受信
        ssize_t len = recvfrom(
            server->sock_fd_,
            buffer,
            sizeof(buffer),
            0,
            (struct sockaddr*)&src_addr,
            &addr_len
        );

        if (len > 0) {
            server->processPacket(buffer, len, &src_addr);
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "recvfrom error: errno %d", errno);
            server->error_count_++;
        }
    }

    ESP_LOGI(TAG, "RX task stopped");
    vTaskDelete(nullptr);
}

void UDPServer::processPacket(const uint8_t* data, size_t len, const SockAddrIn* src_addr) {
    // Check minimum size
    // 最小サイズをチェック
    if (len < 2) {
        error_count_++;
        return;
    }

    // Check header
    // ヘッダをチェック
    if (data[0] != udp::PACKET_HEADER) {
        error_count_++;
        return;
    }

    uint8_t pkt_type = data[1];

    switch (pkt_type) {
        case udp::PKT_TYPE_CONTROL: {
            if (len != sizeof(udp::ControlPacket)) {
                ESP_LOGW(TAG, "Invalid control packet size: %d", len);
                error_count_++;
                return;
            }

            const udp::ControlPacket* pkt = reinterpret_cast<const udp::ControlPacket*>(data);

            // Validate checksum
            // チェックサムを検証
            if (!udp::validateControlPacket(*pkt)) {
                ESP_LOGW(TAG, "Control packet checksum error");
                error_count_++;
                return;
            }

            // Update client tracking
            // クライアントトラッキングを更新
            updateClient(src_addr);

            // Store last control
            // 最後の制御を保存
            if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                last_control_ = *pkt;
                last_control_time_ms_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
                xSemaphoreGive(mutex_);
            }

            rx_count_++;

            // Invoke callback
            // コールバックを呼び出し
            if (control_callback_) {
                control_callback_(*pkt, src_addr);
            }

            break;
        }

        case udp::PKT_TYPE_HEARTBEAT: {
            // Update client tracking on heartbeat
            // ハートビートでクライアントトラッキングを更新
            updateClient(src_addr);
            rx_count_++;
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown packet type: 0x%02X", pkt_type);
            error_count_++;
            break;
    }
}

void UDPServer::updateClient(const SockAddrIn* addr) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Check if client already exists
    // クライアントが既に存在するか確認
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients_[i].active &&
            clients_[i].addr.sin_addr.s_addr == addr->sin_addr.s_addr) {
            clients_[i].last_seen_ms = now;
            return;
        }
    }

    // Add new client
    // 新規クライアントを追加
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients_[i].active) {
            clients_[i].addr = *addr;
            clients_[i].last_seen_ms = now;
            clients_[i].active = true;
            client_count_++;

            char ip_str[16];
            inet_ntoa_r(addr->sin_addr, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "Client connected: %s (total: %d)", ip_str, client_count_);
            return;
        }
    }

    ESP_LOGW(TAG, "Max clients reached, ignoring new connection");
}

void UDPServer::removeStaleClients() {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients_[i].active &&
            (now - clients_[i].last_seen_ms) > CLIENT_TIMEOUT_MS) {

            char ip_str[16];
            inet_ntoa_r(clients_[i].addr.sin_addr, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "Client disconnected (timeout): %s", ip_str);

            clients_[i].active = false;
            client_count_--;
        }
    }
}

}  // namespace stampfly
