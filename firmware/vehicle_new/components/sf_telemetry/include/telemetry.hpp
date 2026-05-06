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
 * @design architecture.md §6 — Telemetry subsystem                     [OK]
 * @design detailed_design.md §8 — UDP telemetry packet format          [OK]
 * @design coding_and_education.md §2 — Bilingual comments              [OK]
 */

#pragma once

#include <cstdint>

namespace sf {

// -----------------------------------------------------------------------------
// Phase 2a basic telemetry packet (binary, fixed layout)
// Phase 2a 基本テレメトリパケット（バイナリ・固定レイアウト）
//
// All multi-byte fields are little-endian (ESP32 native order).
// Wire layout is locked by the packed attribute below.
//
// 全多バイトフィールドはリトルエンディアン（ESP32ネイティブ順）。
// パケットレイアウトは以下の packed 属性で固定する。
// -----------------------------------------------------------------------------

/// Magic word — sentinel for valid Phase 2a packets / 有効パケット識別子
inline constexpr uint16_t TELEM_MAGIC = 0xCAFE;

/// Telemetry protocol version / テレメトリプロトコルバージョン
inline constexpr uint8_t  TELEM_VERSION = 1;

/// Packet type — Phase 2a basic / パケット種別: Phase 2a 基本
inline constexpr uint8_t  TELEM_TYPE_PHASE2A_BASIC = 0x01;

/// Default broadcast destination port / 既定のブロードキャスト送信先ポート
inline constexpr uint16_t UDP_TELEMETRY_PORT = 5005;

#pragma pack(push, 1)
/// Unified Phase 2a telemetry packet
/// Phase 2a 統合テレメトリパケット
struct TelemetryPacket {
    // Header / ヘッダ
    uint16_t magic;          // 0xCAFE
    uint8_t  version;        // = TELEM_VERSION
    uint8_t  packet_type;    // = TELEM_TYPE_PHASE2A_BASIC
    uint32_t timestamp_us;   // Microsecond timestamp / マイクロ秒タイムスタンプ

    // Attitude (Euler) / 姿勢（オイラー角）
    float roll, pitch, yaw;          // [rad]

    // Body-rate gyro / 機体角速度
    float gyro_x, gyro_y, gyro_z;    // [rad/s]

    // Body-frame acceleration / 機体加速度
    float accel_x, accel_y, accel_z; // [m/s^2]

    // Position (NED) / 位置（NED）
    float pos_x, pos_y, pos_z;       // [m]

    // Velocity (NED) / 速度（NED）
    float vel_x, vel_y, vel_z;       // [m/s]

    // Control output / 制御出力
    float thrust, roll_torque, pitch_torque, yaw_torque;

    // Motor duty M1..M4 / モーターduty
    float motor_duty[4];

    // System mode / システムモード
    uint8_t system_mode;             // FlightState enum value / FlightState値
    uint8_t reserved[3];             // Pad to 4-byte boundary / 4Bアライン
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 104,
              "TelemetryPacket layout drift — wire format must stay 104 bytes");

/// Telemetry manager: collect, pack, and send via UDP
/// テレメトリマネージャー: 収集、パック、UDP送信
class Telemetry {
public:
    /// Initialize telemetry subsystem (open UDP socket)
    /// テレメトリサブシステムを初期化（UDPソケットを開く）
    void init();

    /// Collect latest data from topics and send one UDP packet
    /// トピックから最新データを収集し、UDPパケットを1回送信する
    void update();

    /// Override destination address/port (host byte order for port)
    /// 送信先アドレスとポートを上書き（portはホストバイトオーダ）
    void setDestination(uint32_t ip_be, uint16_t port);

private:
    /// Wait until WiFi STA has an IP address (poll-with-backoff)
    /// WiFi STA がIPを取得するまで待機（ポーリング＋バックオフ）
    void waitForWifi();

    /// Build the binary packet from current topic latest()
    /// 現在のトピック latest() からバイナリパケットを組み立てる
    void buildPacket(TelemetryPacket& pkt);

    /// Send packet via UDP, with rate-limited error logging
    /// UDP でパケット送信。エラーはレート制限付きでログ出力
    void sendPacket(const TelemetryPacket& pkt);

    int      socket_fd_   = -1;     // UDP socket fd / UDPソケットfd
    uint32_t dest_ip_be_  = 0;      // Destination IP (network/big-endian)
    uint16_t dest_port_   = 0;      // Destination port (host order)
    uint32_t err_count_   = 0;      // sendto failure counter / 送信失敗カウンタ
    bool     ready_       = false;  // True after socket bound / ソケット準備完了
};

}  // namespace sf
