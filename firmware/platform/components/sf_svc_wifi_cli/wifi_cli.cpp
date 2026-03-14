/**
 * @file wifi_cli.cpp
 * @brief WiFi CLI Server Implementation
 *
 * Telnet-like TCP server for CLI access over WiFi.
 *
 * WiFi CLI サーバー実装
 * WiFi経由でのCLIアクセスを提供するTelnet風TCPサーバー
 */

#include "wifi_cli.hpp"
#include "console.hpp"
#include "line_editor.hpp"

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "esp_log.h"
#include "esp_system.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char* TAG = "WiFiCLI";

namespace stampfly {

// =============================================================================
// Singleton Instance
// =============================================================================

WiFiCLI& WiFiCLI::getInstance()
{
    static WiFiCLI instance;
    return instance;
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t WiFiCLI::init(const Config& config)
{
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    config_ = config;

    // Create client mutex
    // クライアントミューテックスを作成
    client_mutex_ = xSemaphoreCreateMutex();
    if (client_mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized (port %d, max_clients %d)",
             config_.port, config_.max_clients);

    return ESP_OK;
}

// =============================================================================
// Start / Stop
// =============================================================================

esp_err_t WiFiCLI::start()
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (running_) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    // Create server socket
    // サーバーソケットを作成
    server_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set socket options
    // ソケットオプションを設定
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind to port
    // ポートにバインド
    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(config_.port);

    if (bind(server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind: errno %d", errno);
        close(server_fd_);
        server_fd_ = -1;
        return ESP_FAIL;
    }

    // Listen for connections
    // 接続をリッスン
    if (listen(server_fd_, config_.max_clients) < 0) {
        ESP_LOGE(TAG, "Failed to listen: errno %d", errno);
        close(server_fd_);
        server_fd_ = -1;
        return ESP_FAIL;
    }

    running_ = true;

    // Create accept task
    // 接続受付タスクを作成
    BaseType_t ret = xTaskCreate(
        acceptTask,
        "wifi_cli_accept",
        4096,
        this,
        5,  // Priority
        &accept_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create accept task");
        close(server_fd_);
        server_fd_ = -1;
        running_ = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Started on port %d", config_.port);
    return ESP_OK;
}

esp_err_t WiFiCLI::stop()
{
    if (!running_) {
        return ESP_OK;
    }

    running_ = false;

    // Disconnect all clients
    // 全クライアントを切断
    disconnectAll();

    // Close server socket (this will cause accept() to fail)
    // サーバーソケットを閉じる（accept()が失敗する）
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }

    // Wait for accept task to finish
    // 接続受付タスクの終了を待つ
    if (accept_task_handle_ != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(100));
        accept_task_handle_ = nullptr;
    }

    ESP_LOGI(TAG, "Stopped");
    return ESP_OK;
}

// =============================================================================
// Accept Task
// =============================================================================

void WiFiCLI::acceptTask(void* arg)
{
    WiFiCLI* self = static_cast<WiFiCLI*>(arg);

    ESP_LOGI(TAG, "Accept task started");

    while (self->running_) {
        struct sockaddr_in client_addr = {};
        socklen_t addr_len = sizeof(client_addr);

        // Accept new connection (blocks until connection or error)
        // 新規接続を受け入れる（接続またはエラーまでブロック）
        int client_fd = accept(self->server_fd_,
                               (struct sockaddr*)&client_addr, &addr_len);

        if (client_fd < 0) {
            if (self->running_) {
                ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            }
            continue;
        }

        // Check client limit
        // クライアント数制限をチェック
        if (self->client_count_ >= MAX_CLIENTS) {
            ESP_LOGW(TAG, "Max clients reached, rejecting connection");
            const char* msg = "Server busy, try again later.\r\n";
            send(client_fd, msg, strlen(msg), 0);
            close(client_fd);
            continue;
        }

        ESP_LOGI(TAG, "New connection from %s",
                 inet_ntoa(client_addr.sin_addr));

        // Create client context
        // クライアントコンテキストを作成
        ClientContext* ctx = new ClientContext();
        ctx->server = self;
        ctx->client_fd = client_fd;
        ctx->client_ip = client_addr.sin_addr.s_addr;
        ctx->rx_pos = 0;
        ctx->last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ctx->active = true;

        // Add to client list
        // クライアントリストに追加
        xSemaphoreTake(self->client_mutex_, portMAX_DELAY);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (self->clients_[i] == nullptr) {
                self->clients_[i] = ctx;
                self->client_count_++;
                break;
            }
        }
        xSemaphoreGive(self->client_mutex_);

        // Create client handler task
        // クライアントハンドラタスクを作成
        char task_name[24];
        snprintf(task_name, sizeof(task_name), "wcli_%d", client_fd);

        BaseType_t ret = xTaskCreate(
            clientTask,
            task_name,
            4096,
            ctx,
            5,  // Priority
            nullptr
        );

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create client task");
            close(client_fd);

            xSemaphoreTake(self->client_mutex_, portMAX_DELAY);
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (self->clients_[i] == ctx) {
                    self->clients_[i] = nullptr;
                    self->client_count_--;
                    break;
                }
            }
            xSemaphoreGive(self->client_mutex_);

            delete ctx;
        }
    }

    ESP_LOGI(TAG, "Accept task ended");
    vTaskDelete(nullptr);
}

// =============================================================================
// Client Task
// =============================================================================

void WiFiCLI::clientTask(void* arg)
{
    ClientContext* ctx = static_cast<ClientContext*>(arg);
    WiFiCLI* self = ctx->server;

    self->handleClient(ctx);

    // Remove from client list
    // クライアントリストから削除
    xSemaphoreTake(self->client_mutex_, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (self->clients_[i] == ctx) {
            self->clients_[i] = nullptr;
            self->client_count_--;
            break;
        }
    }
    xSemaphoreGive(self->client_mutex_);

    // Close socket and free context
    // ソケットを閉じてコンテキストを解放
    close(ctx->client_fd);
    delete ctx;

    vTaskDelete(nullptr);
}

// =============================================================================
// Output Callback for Console
// =============================================================================

// Output function for redirecting Console output to WiFi client
// Console出力をWiFiクライアントにリダイレクトする関数
static void wifiOutputCallback(const char* str, void* ctx)
{
    if (ctx == nullptr || str == nullptr) return;
    int fd = *static_cast<int*>(ctx);
    size_t len = strlen(str);
    if (len > 0 && fd >= 0) {
        send(fd, str, len, 0);
    }
}

// =============================================================================
// Socket I/O Callbacks for LineEditor
// =============================================================================

/**
 * @brief Read a byte from socket
 * @param ctx Socket file descriptor (passed as void*)
 * @return Byte read, or -1 on error
 *
 * ソケットから1バイト読み取る
 */
static int socket_read_byte(void* ctx)
{
    int fd = *static_cast<int*>(ctx);
    char c;
    int ret = recv(fd, &c, 1, 0);
    if (ret <= 0) {
        return -1;  // Error or disconnect
    }
    return static_cast<unsigned char>(c);
}

/**
 * @brief Write data to socket
 * @param ctx Socket file descriptor (passed as void*)
 * @param data Data to write
 * @param len Length of data
 * @return Number of bytes written
 *
 * ソケットにデータを書き込む
 */
static int socket_write_data(void* ctx, const void* data, size_t len)
{
    int fd = *static_cast<int*>(ctx);
    if (data == nullptr || len == 0 || fd < 0) {
        return 0;
    }
    return send(fd, data, len, 0);
}

// =============================================================================
// Completion Callback for WiFi CLI
// =============================================================================

// Tab completion callback for WiFi clients
// WiFiクライアント用のTab補完コールバック
static void wifiCompletionCallback(const char* buf, LineCompletions* lc)
{
    if (buf == nullptr || lc == nullptr) {
        return;
    }

    size_t buf_len = strlen(buf);

    // List of known commands for completion
    // 補完用の既知コマンドリスト
    static const char* commands[] = {
        "help",
        "status",
        "reboot",
        "version",
        "sensor",
        "loglevel",
        "binlog",
        "motor",
        "trim",
        "gain",
        "comm",
        "wifi",
        "pair",
        "unpair",
        "calib",
        "magcal",
        "led",
        "sound",
        "pos",
        "debug",
        "ctrl",
        "attitude",
        "exit",
        "quit",
        nullptr
    };

    for (int i = 0; commands[i] != nullptr; i++) {
        if (strncmp(buf, commands[i], buf_len) == 0) {
            lc->add(commands[i]);
        }
    }
}

// =============================================================================
// Client Handler
// =============================================================================

void WiFiCLI::handleClient(ClientContext* ctx)
{
    ESP_LOGI(TAG, "Handling client on fd %d", ctx->client_fd);

    // Send welcome message
    // ウェルカムメッセージを送信
    sendWelcome(ctx->client_fd);

    // Create line editor for this client with socket I/O callbacks
    // このクライアント用の行エディタをソケットI/Oコールバックで作成
    int client_fd = ctx->client_fd;
    LineEditorIO io = {
        .read_byte = socket_read_byte,
        .write_data = socket_write_data,
        .ctx = &client_fd,
    };

    LineEditorConfig config = {
        .enable_telnet_negotiation = true,
        .history_max_len = 10,
    };

    LineEditor editor(io, config);
    editor.setCompletionCallback(wifiCompletionCallback);

    // Get console reference
    // コンソール参照を取得
    auto& console = Console::getInstance();

    while (ctx->active && running_) {
        // Get line using LineEditor (handles history, editing, completion)
        // LineEditorを使って行を取得（履歴、編集、補完を処理）
        char* line = editor.getLine("stampfly> ");

        if (line == nullptr) {
            // Error or disconnect
            // エラーまたは切断
            ESP_LOGI(TAG, "Client disconnected or error");
            break;
        }

        // Update activity timestamp
        // アクティビティタイムスタンプを更新
        ctx->last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Skip empty lines
        // 空行をスキップ
        if (line[0] == '\0') {
            editor.freeLine(line);
            continue;
        }

        // Add to history
        // 履歴に追加
        editor.addHistory(line);

        // Handle built-in exit commands
        // 組み込みexitコマンドを処理
        if (strcmp(line, "exit") == 0 || strcmp(line, "quit") == 0) {
            const char* msg = "Goodbye.\r\n";
            send(ctx->client_fd, msg, strlen(msg), 0);
            editor.freeLine(line);
            break;
        }

        // Execute command via Console
        // Console経由でコマンドを実行
        if (!console.isInitialized()) {
            const char* msg = "Console not available\r\n";
            send(ctx->client_fd, msg, strlen(msg), 0);
        } else {
            // Set output redirection to this client's socket
            // このクライアントのソケットに出力をリダイレクト
            int client_fd = ctx->client_fd;
            console.setOutput(wifiOutputCallback, &client_fd);

            // Execute the command
            // コマンドを実行
            int ret = console.run(line);
            if (ret != 0) {
                char buf[32];
                snprintf(buf, sizeof(buf), "Error: %d\r\n", ret);
                send(ctx->client_fd, buf, strlen(buf), 0);
            }

            // Clear output redirection
            // 出力リダイレクトをクリア
            console.clearOutput();
        }

        editor.freeLine(line);
    }

    ESP_LOGI(TAG, "Client handler ended for fd %d", ctx->client_fd);
}

// =============================================================================
// Helper Functions
// =============================================================================

void WiFiCLI::sendToClient(int fd, const char* data, size_t len)
{
    if (fd >= 0 && data != nullptr && len > 0) {
        send(fd, data, len, 0);
    }
}

void WiFiCLI::sendWelcome(int fd)
{
    const char* welcome =
        "\r\n"
        "========================================\r\n"
        "  StampFly WiFi CLI\r\n"
        "========================================\r\n"
        "Type 'help' for available commands.\r\n"
        "Type 'exit' to disconnect.\r\n"
        "\r\n";
    send(fd, welcome, strlen(welcome), 0);
}

void WiFiCLI::broadcast(const char* message)
{
    if (message == nullptr) return;

    size_t len = strlen(message);
    if (len == 0) return;

    xSemaphoreTake(client_mutex_, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients_[i] != nullptr && clients_[i]->active) {
            send(clients_[i]->client_fd, message, len, 0);
        }
    }
    xSemaphoreGive(client_mutex_);
}

void WiFiCLI::disconnectAll()
{
    xSemaphoreTake(client_mutex_, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients_[i] != nullptr) {
            clients_[i]->active = false;
            // Close socket to force recv() to fail
            // recv()を失敗させるためにソケットを閉じる
            shutdown(clients_[i]->client_fd, SHUT_RDWR);
        }
    }
    xSemaphoreGive(client_mutex_);
}

}  // namespace stampfly
