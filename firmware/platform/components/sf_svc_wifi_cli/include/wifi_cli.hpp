/**
 * @file wifi_cli.hpp
 * @brief WiFi CLI Server (Telnet-like TCP server)
 *
 * Provides CLI access over WiFi via TCP socket (port 23 by default).
 * Shares command handlers with USB CLI.
 *
 * WiFi CLI サーバー（Telnet風TCPサーバー）
 * TCPソケット経由でCLIアクセスを提供（デフォルトポート23）
 * USB CLIとコマンドハンドラを共有
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace stampfly {

/**
 * @brief WiFi CLI Server
 *
 * Singleton class that provides Telnet-like CLI access over WiFi.
 * Uses the same command handlers as the USB CLI.
 */
class WiFiCLI {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to the WiFiCLI instance
     */
    static WiFiCLI& getInstance();

    // Delete copy/move
    // コピー/ムーブ禁止
    WiFiCLI(const WiFiCLI&) = delete;
    WiFiCLI& operator=(const WiFiCLI&) = delete;

    /**
     * @brief Configuration options
     */
    struct Config {
        uint16_t port;              // Telnet standard port
        uint8_t max_clients;        // Max simultaneous connections
        size_t rx_buffer_size;      // Receive buffer size
        uint32_t idle_timeout_ms;   // Idle timeout in ms

        // Constructor with default values
        // デフォルト値付きコンストラクタ
        Config()
            : port(23)
            , max_clients(2)
            , rx_buffer_size(256)
            , idle_timeout_ms(300000)  // 5 min
        {}
    };

    /**
     * @brief Initialize WiFi CLI server
     * @param config Server configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Start the server
     * @return ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop the server
     * @return ESP_OK on success
     */
    esp_err_t stop();

    /**
     * @brief Check if server is running
     * @return true if running
     */
    bool isRunning() const { return running_; }

    /**
     * @brief Broadcast message to all connected clients
     * @param message Message to send
     *
     * Useful for async output (e.g., state changes)
     */
    void broadcast(const char* message);

    /**
     * @brief Get number of connected clients
     * @return Client count
     */
    int getClientCount() const { return client_count_; }

    /**
     * @brief Check if any clients are connected
     * @return true if at least one client is connected
     */
    bool hasClients() const { return client_count_ > 0; }

    /**
     * @brief Disconnect all clients
     */
    void disconnectAll();

private:
    WiFiCLI() = default;

    // Accept task (listens for new connections)
    // 接続受付タスク（新規接続をリッスン）
    static void acceptTask(void* arg);

    // Client handler task (per-client)
    // クライアントハンドラタスク（クライアント毎）
    static void clientTask(void* arg);

    /**
     * @brief Client context (per-connection state)
     */
    struct ClientContext {
        WiFiCLI* server;           // Parent server reference
        int client_fd;             // Client socket FD
        uint32_t client_ip;        // Client IP address
        char rx_buffer[256];       // Receive buffer
        size_t rx_pos;             // Current position in buffer
        uint32_t last_activity_ms; // Last activity timestamp
        bool active;               // Connection active flag
    };

    // Handle client connection
    // クライアント接続を処理
    void handleClient(ClientContext* ctx);

    // Send data to a client
    // クライアントにデータを送信
    void sendToClient(int fd, const char* data, size_t len);

    // Send welcome message
    // ウェルカムメッセージを送信
    void sendWelcome(int fd);

    Config config_;
    int server_fd_ = -1;
    bool initialized_ = false;
    bool running_ = false;

    // Client tracking
    // クライアント管理
    static constexpr int MAX_CLIENTS = 2;
    ClientContext* clients_[MAX_CLIENTS] = {};
    int client_count_ = 0;
    SemaphoreHandle_t client_mutex_ = nullptr;

    // Task handles
    TaskHandle_t accept_task_handle_ = nullptr;
};

}  // namespace stampfly
