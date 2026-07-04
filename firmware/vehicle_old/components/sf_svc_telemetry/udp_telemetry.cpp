/**
 * @file udp_telemetry.cpp
 * @brief UDP Telemetry Log Server implementation
 *
 * UDP テレメトリログサーバー実装
 *
 * Listens on UDP port 8890 for start/stop/heartbeat commands from PC.
 * When active, telemetry_task sends sensor data via send().
 * PC 側からの開始/停止/ハートビートコマンドを UDP ポート 8890 で待ち受け。
 * アクティブ時、telemetry_task が send() でセンサデータを送信する。
 */

#include "udp_telemetry.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <cstring>

static const char* TAG = "UDPTelem";

namespace stampfly {
namespace udp_telem {

// =============================================================================
// Initialization
// 初期化
// =============================================================================

esp_err_t UDPLogServer::init()
{
    if (sock_fd_ >= 0) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create UDP socket
    // UDPソケット作成
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set socket options
    // ソケットオプション設定
    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set receive timeout (100ms) for RX task polling
    // 受信タイムアウト（100ms）設定（RXタスクのポーリング用）
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Bind to log port
    // ログポートにバインド
    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(UDP_LOG_PORT);

    if (bind(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind port %d: errno %d", UDP_LOG_PORT, errno);
        close(sock_fd_);
        sock_fd_ = -1;
        return ESP_FAIL;
    }

    // Create RX task (low priority, listens for commands)
    // 受信タスク作成（低優先度、コマンド待ち受け）
    BaseType_t ret = xTaskCreatePinnedToCore(
        rxTask,
        "udp_log_rx",
        3072,     // Stack size
        this,
        5,        // Low priority
        &rx_task_handle_,
        0         // Core 0 (protocol core)
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        close(sock_fd_);
        sock_fd_ = -1;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "UDP Log Server initialized on port %d", UDP_LOG_PORT);
    return ESP_OK;
}

// =============================================================================
// Send (fire-and-forget)
// 送信（送りっぱなし）
// =============================================================================

int UDPLogServer::send(const void* data, size_t len)
{
    if (!active_.load() || !client_registered_ || sock_fd_ < 0) {
        return -1;
    }

    ssize_t sent = sendto(sock_fd_, data, len, 0,
                          (struct sockaddr*)&client_addr_, sizeof(client_addr_));
    if (sent < 0) {
        // Send failure — don't deactivate, just drop this packet
        // 送信失敗 — 非アクティブ化はしない、パケットをドロップ
        return -1;
    }

    return (int)sent;
}

// =============================================================================
// Heartbeat check
// ハートビートチェック
// =============================================================================

void UDPLogServer::checkHeartbeat()
{
    if (!active_.load()) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t elapsed = now_ms - last_heartbeat_ms_;

    if (elapsed > HEARTBEAT_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Client heartbeat timeout (%lu ms), deactivating", elapsed);
        active_.store(false);
        client_registered_ = false;
    }
}

// =============================================================================
// RX Task: listens for start/stop/heartbeat commands
// 受信タスク: 開始/停止/ハートビートコマンドを待ち受け
// =============================================================================

void UDPLogServer::rxTask(void* arg)
{
    auto* server = static_cast<UDPLogServer*>(arg);

    uint8_t rx_buf[16];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    ESP_LOGI(TAG, "RX task started, listening on port %d", UDP_LOG_PORT);

    while (true) {
        ssize_t len = recvfrom(server->sock_fd_, rx_buf, sizeof(rx_buf), 0,
                               (struct sockaddr*)&src_addr, &addr_len);

        if (len <= 0) {
            // Timeout or error — check heartbeat and continue
            // タイムアウトまたはエラー — ハートビートチェックして継続
            server->checkHeartbeat();
            continue;
        }

        uint8_t cmd = rx_buf[0];

        switch (cmd) {
            case CMD_START_LOG: {
                // Register client and activate
                // クライアント登録してアクティブ化
                server->client_addr_ = src_addr;
                server->client_registered_ = true;
                server->last_heartbeat_ms_ = (uint32_t)(esp_timer_get_time() / 1000);
                server->active_.store(true);

                char ip_str[16];
                inet_ntoa_r(src_addr.sin_addr, ip_str, sizeof(ip_str));
                ESP_LOGI(TAG, "Log started: client %s:%d",
                         ip_str, ntohs(src_addr.sin_port));
                break;
            }

            case CMD_STOP_LOG: {
                // Deactivate
                // 非アクティブ化
                server->active_.store(false);
                server->client_registered_ = false;
                ESP_LOGI(TAG, "Log stopped by client");
                break;
            }

            case CMD_HEARTBEAT: {
                // Update heartbeat timestamp
                // ハートビートタイムスタンプ更新
                if (server->active_.load()) {
                    server->last_heartbeat_ms_ = (uint32_t)(esp_timer_get_time() / 1000);
                }
                break;
            }

            default:
                ESP_LOGD(TAG, "Unknown command: 0x%02X", cmd);
                break;
        }
    }
}

}  // namespace udp_telem
}  // namespace stampfly
