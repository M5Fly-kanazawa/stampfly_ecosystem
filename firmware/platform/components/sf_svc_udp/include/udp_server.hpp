/**
 * @file udp_server.hpp
 * @brief UDP server for receiving control packets and sending telemetry
 *        制御パケット受信とテレメトリ送信用UDPサーバー
 *
 * This component provides UDP-based communication as an alternative to ESP-NOW.
 * ESP-NOWの代替としてUDPベースの通信を提供するコンポーネント。
 */

#pragma once

// Include system headers before namespace to avoid forward declaration issues
// 名前空間問題を避けるためシステムヘッダを先にインクルード
#include "lwip/sockets.h"
#include <functional>

// Type alias for sockaddr_in in global namespace
// グローバル名前空間でのsockaddr_in用型エイリアス
using SockAddrIn = struct sockaddr_in;

#include "udp_protocol.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace stampfly {

/**
 * @brief UDP Server for Vehicle
 *        Vehicle用UDPサーバー
 *
 * Singleton class that manages UDP communication with Controller/GCS.
 * Controller/GCSとのUDP通信を管理するシングルトンクラス。
 */
class UDPServer {
public:
    /// Get singleton instance / シングルトンインスタンスを取得
    static UDPServer& getInstance();

    // Delete copy/move
    UDPServer(const UDPServer&) = delete;
    UDPServer& operator=(const UDPServer&) = delete;

    /**
     * @brief Server configuration
     *        サーバー設定
     */
    struct Config {
        uint16_t control_port;       ///< Control receive port / 制御受信ポート
        uint16_t telemetry_port;     ///< Telemetry send port / テレメトリ送信ポート
        uint32_t control_timeout_ms; ///< Control timeout / 制御タイムアウト
        uint32_t telemetry_rate_hz;  ///< Telemetry rate / テレメトリレート

        /// Default constructor with default values / デフォルト値でのコンストラクタ
        Config() :
            control_port(udp::CONTROL_PORT),
            telemetry_port(udp::TELEMETRY_PORT),
            control_timeout_ms(udp::CONTROL_TIMEOUT_MS),
            telemetry_rate_hz(udp::TELEMETRY_RATE_HZ) {}
    };

    /**
     * @brief Callback type for control packet reception
     *        制御パケット受信時のコールバック型
     *
     * @note Uses global namespace SockAddrIn to avoid namespace issues
     *       名前空間問題を避けるためグローバル名前空間のSockAddrInを使用
     */
    using ControlCallback = std::function<void(const udp::ControlPacket&, const SockAddrIn*)>;

    /**
     * @brief Initialize the UDP server
     *        UDPサーバーを初期化
     *
     * @param config Server configuration / サーバー設定
     * @return ESP_OK on success / 成功時ESP_OK
     */
    esp_err_t init(const Config& config = Config());

    /**
     * @brief Start the UDP server
     *        UDPサーバーを開始
     *
     * @return ESP_OK on success / 成功時ESP_OK
     */
    esp_err_t start();

    /**
     * @brief Stop the UDP server
     *        UDPサーバーを停止
     *
     * @return ESP_OK on success / 成功時ESP_OK
     */
    esp_err_t stop();

    /**
     * @brief Check if server is running
     *        サーバーが動作中か確認
     */
    bool isRunning() const { return running_; }

    /**
     * @brief Set callback for control packet reception
     *        制御パケット受信時のコールバックを設定
     *
     * @param callback Callback function / コールバック関数
     */
    void setControlCallback(ControlCallback callback) { control_callback_ = callback; }

    // ========================================================================
    // Control Input Access
    // 制御入力アクセス
    // ========================================================================

    /**
     * @brief Check if control is active (recent packet received)
     *        制御がアクティブか確認（最近パケットを受信したか）
     */
    bool hasActiveControl() const;

    /**
     * @brief Get the last received control packet
     *        最後に受信した制御パケットを取得
     *
     * @param pkt Output packet / 出力パケット
     * @return true if valid packet available / 有効なパケットがあればtrue
     */
    bool getLastControl(udp::ControlPacket& pkt) const;

    /**
     * @brief Get timestamp of last control packet
     *        最後の制御パケットのタイムスタンプを取得
     */
    uint32_t getLastControlTime() const { return last_control_time_ms_; }

    // ========================================================================
    // Telemetry Sending
    // テレメトリ送信
    // ========================================================================

    /**
     * @brief Send telemetry packet to all connected clients
     *        接続中の全クライアントにテレメトリパケットを送信
     *
     * @param pkt Telemetry packet to send / 送信するテレメトリパケット
     */
    void sendTelemetry(const udp::TelemetryPacket& pkt);

    // ========================================================================
    // Client Management
    // クライアント管理
    // ========================================================================

    /**
     * @brief Get number of connected clients
     *        接続中のクライアント数を取得
     */
    int getClientCount() const { return client_count_; }

    /**
     * @brief Check if any client is connected
     *        クライアントが接続されているか確認
     */
    bool hasClients() const { return client_count_ > 0; }

    // ========================================================================
    // Statistics
    // 統計情報
    // ========================================================================

    /**
     * @brief Get received packet count
     *        受信パケット数を取得
     */
    uint32_t getRxCount() const { return rx_count_; }

    /**
     * @brief Get transmitted packet count
     *        送信パケット数を取得
     */
    uint32_t getTxCount() const { return tx_count_; }

    /**
     * @brief Get error count
     *        エラー数を取得
     */
    uint32_t getErrorCount() const { return error_count_; }

    /**
     * @brief Reset statistics
     *        統計情報をリセット
     */
    void resetStats();

private:
    UDPServer() = default;
    ~UDPServer() = default;

    /// RX task function / 受信タスク関数
    static void rxTask(void* arg);

    /// Process received packet / 受信パケットを処理
    void processPacket(const uint8_t* data, size_t len, const SockAddrIn* src_addr);

    /// Update client tracking / クライアントトラッキングを更新
    void updateClient(const SockAddrIn* addr);

    /// Remove stale clients / 古いクライアントを削除
    void removeStaleClients();

    // Configuration
    Config config_;

    // Socket
    int sock_fd_ = -1;
    bool running_ = false;
    bool initialized_ = false;

    // Client tracking
    static constexpr int MAX_CLIENTS = 4;
    struct ClientInfo {
        SockAddrIn addr;
        uint32_t last_seen_ms;
        bool active;
    };
    ClientInfo clients_[MAX_CLIENTS] = {};
    int client_count_ = 0;

    // Last control packet
    udp::ControlPacket last_control_ = {};
    uint32_t last_control_time_ms_ = 0;
    mutable SemaphoreHandle_t mutex_ = nullptr;

    // Callback
    ControlCallback control_callback_ = nullptr;

    // Statistics
    uint32_t rx_count_ = 0;
    uint32_t tx_count_ = 0;
    uint32_t error_count_ = 0;

    // Task handle
    TaskHandle_t rx_task_handle_ = nullptr;

    // Constants
    static constexpr uint32_t CLIENT_TIMEOUT_MS = 5000;  // Client timeout / クライアントタイムアウト
    static constexpr size_t RX_BUFFER_SIZE = 64;         // RX buffer size / 受信バッファサイズ
    static constexpr uint32_t RX_TASK_STACK_SIZE = 4096; // Task stack size / タスクスタックサイズ
    static constexpr UBaseType_t RX_TASK_PRIORITY = 5;   // Task priority / タスク優先度
};

}  // namespace stampfly
