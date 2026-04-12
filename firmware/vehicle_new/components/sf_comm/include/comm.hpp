/**
 * @file comm.hpp
 * @brief Communication manager — ESP-NOW and UDP
 *        通信マネージャー — ESP-NOWおよびUDP
 *
 * Manages all wireless communication:
 * - ESP-NOW: receive control commands from transmitter
 * - UDP: receive API commands, send telemetry
 *
 * 全無線通信を管理する:
 * - ESP-NOW: 送信機からの制御コマンド受信
 * - UDP: APIコマンド受信、テレメトリ送信
 *
 * @design architecture.md §6 — Communication subsystem                  [--]
 * @design detailed_design.md §7 — ESP-NOW + WiFi AP coexistence        [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
 */

#pragma once

#include <cstdint>

namespace sf {

/// Communication manager
/// 通信マネージャー
class Comm {
public:
    /// Initialize communication subsystem (WiFi AP + ESP-NOW + UDP)
    /// 通信サブシステムを初期化する（WiFi AP + ESP-NOW + UDP）
    void init();

    /// Process incoming data (call periodically)
    /// 受信データを処理する（定期的に呼ぶ）
    void update();

    /// Check if ESP-NOW link is active
    /// ESP-NOWリンクがアクティブか確認する
    bool isEspNowConnected() const { return espnow_connected_; }

    /// Get time since last ESP-NOW packet [ms]
    /// 最後のESP-NOWパケットからの経過時間を取得する [ms]
    uint32_t timeSinceLastPacket() const;

private:
    /// ESP-NOW receive callback (static, called from ISR context)
    /// ESP-NOW受信コールバック（静的、ISRコンテキストから呼ばれる）
    static void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);

    /// Parse received ESP-NOW data and publish to command topic
    /// 受信したESP-NOWデータを解析しコマンドトピックに発行する
    void parseEspNowData(const uint8_t* data, int len);

    /// Initialize WiFi in AP+STA mode for ESP-NOW + UDP coexistence
    /// ESP-NOW + UDP共存のためWiFiをAP+STAモードで初期化する
    void initWifi();

    /// Initialize ESP-NOW protocol
    /// ESP-NOWプロトコルを初期化する
    void initEspNow();

    /// Initialize UDP socket for API commands
    /// APIコマンド用UDPソケットを初期化する
    void initUdpSocket();

    bool espnow_connected_ = false;     // ESP-NOW link status
    uint32_t last_packet_ms_ = 0;       // Last packet timestamp [ms]
    int udp_socket_ = -1;              // UDP socket fd
};

}  // namespace sf
