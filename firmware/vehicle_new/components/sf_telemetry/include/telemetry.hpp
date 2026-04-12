/**
 * @file telemetry.hpp
 * @brief Telemetry — UDP packet construction and transmission
 *        テレメトリ — UDPパケット構築と送信
 *
 * Collects data from various topics (state estimate, control output,
 * sensor data, system mode) and sends unified telemetry packets
 * over UDP to connected clients.
 *
 * 各種トピック（状態推定、制御出力、センサデータ、システムモード）から
 * データを収集し、統合テレメトリパケットをUDP経由で接続クライアントに送信する。
 *
 * @design architecture.md §6 — Telemetry subsystem                     [--]
 * @design detailed_design.md §8 — UDP telemetry packet format          [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
 */

#pragma once

#include <cstdint>

namespace sf {

/// Telemetry packet header
/// テレメトリパケットヘッダ
struct TelemetryHeader {
    uint8_t  version;       // Protocol version / プロトコルバージョン
    uint8_t  type;          // Packet type      / パケット種別
    uint16_t sequence;      // Sequence number  / シーケンス番号
    uint32_t timestamp;     // [us]             / タイムスタンプ
};

/// Telemetry manager: collect, pack, and send via UDP
/// テレメトリマネージャー: 収集、パック、UDP送信
class Telemetry {
public:
    /// Initialize telemetry subsystem
    /// テレメトリサブシステムを初期化する
    void init();

    /// Collect latest data from topics and send UDP packet
    /// トピックから最新データを収集しUDPパケットを送信する
    void update();

    /// Set destination address and port
    /// 送信先アドレスとポートを設定する
    void setDestination(uint32_t ip, uint16_t port);

private:
    /// Build telemetry packet from topic data
    /// トピックデータからテレメトリパケットを構築する
    int buildPacket(uint8_t* buf, int max_len);

    /// Send UDP packet to destination
    /// 送信先にUDPパケットを送信する
    void sendPacket(const uint8_t* buf, int len);

    int      socket_fd_ = -1;         // UDP socket fd
    uint32_t dest_ip_   = 0;          // Destination IP (network order)
    uint16_t dest_port_ = 0;          // Destination port
    uint16_t sequence_  = 0;          // Packet sequence counter
};

}  // namespace sf
