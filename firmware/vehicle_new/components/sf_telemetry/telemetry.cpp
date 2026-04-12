/**
 * @file telemetry.cpp
 * @brief Telemetry implementation (stub)
 *        テレメトリ実装（スタブ）
 *
 * @design architecture.md §6 — Telemetry subsystem                     [--]
 * @design detailed_design.md §8 — UDP telemetry packet format          [--]
 */

#include "telemetry.hpp"
#include "topics.hpp"
#include "esp_log.h"

static const char* TAG = "telemetry";

namespace sf {

// -----------------------------------------------------------------------------
// init — create UDP socket for telemetry output
// 初期化 — テレメトリ出力用UDPソケットを作成
// -----------------------------------------------------------------------------
void Telemetry::init()
{
    // TODO: Create UDP socket
    // TODO: UDPソケットを作成
    // socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    ESP_LOGI(TAG, "Telemetry initialized (stub)");
}

// -----------------------------------------------------------------------------
// update — collect topic data and send telemetry packet
// 更新 — トピックデータを収集しテレメトリパケットを送信
// -----------------------------------------------------------------------------
void Telemetry::update()
{
    // TODO: Build unified telemetry packet from topics
    // TODO: トピックから統合テレメトリパケットを構築
    //   - estimate_state.latest()
    //   - control_output.latest()
    //   - sensor_power.latest()
    //   - system_mode.latest()

    // TODO: Send via UDP
    // TODO: UDP経由で送信
}

// -----------------------------------------------------------------------------
// setDestination — configure UDP send target
// 送信先設定 — UDP送信ターゲットを設定
// -----------------------------------------------------------------------------
void Telemetry::setDestination(uint32_t ip, uint16_t port)
{
    dest_ip_   = ip;
    dest_port_ = port;
}

// -----------------------------------------------------------------------------
// buildPacket — serialize topic data into packet buffer
// パケット構築 — トピックデータをパケットバッファにシリアライズ
// -----------------------------------------------------------------------------
int Telemetry::buildPacket(uint8_t* buf, int max_len)
{
    // TODO: Pack header + state estimate + control output + system mode
    // TODO: ヘッダ＋状態推定＋制御出力＋システムモードをパック
    (void)buf;
    (void)max_len;
    return 0;
}

// -----------------------------------------------------------------------------
// sendPacket — transmit UDP datagram
// パケット送信 — UDPデータグラムを送信
// -----------------------------------------------------------------------------
void Telemetry::sendPacket(const uint8_t* buf, int len)
{
    // TODO: sendto(socket_fd_, buf, len, 0, &dest_addr, sizeof(dest_addr))
    // TODO: sendto(socket_fd_, buf, len, 0, &dest_addr, sizeof(dest_addr))
    (void)buf;
    (void)len;
}

}  // namespace sf
