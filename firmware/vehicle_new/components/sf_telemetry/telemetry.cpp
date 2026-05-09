/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file telemetry.cpp
 * @brief Telemetry implementation — Phase 2a UDP broadcast
 *        テレメトリ実装 — Phase 2a UDPブロードキャスト
 *
 * Sends a single 132-byte binary packet over UDP to the broadcast address
 * 255.255.255.255:UDP_TELEMETRY_PORT at 50 Hz. WiFi STA mode is owned by
 * sf_comm; this module merely waits until WiFi reports an IP address.
 *
 * 132バイトのバイナリパケットを 255.255.255.255:UDP_TELEMETRY_PORT に
 * 50 Hz でUDPブロードキャストする。WiFi STAモードは sf_comm が所有しており、
 * 本モジュールは IP 取得を待つだけ。
 *
 * @design architecture.md §6 — Telemetry subsystem                     [OK]
 * @design detailed_design.md §8 — UDP telemetry packet format          [OK]
 */

#include "telemetry.hpp"
#include "comm.hpp"
#include "topics.hpp"
#include "sf_math.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <cstring>

static const char* TAG = "telemetry";

namespace sf {

// -----------------------------------------------------------------------------
// Internal constants — kept local to avoid leaking into headers
// 内部定数 — ヘッダに漏らさないためローカルに置く
// -----------------------------------------------------------------------------
namespace {

// Polling parameters while waiting for WiFi STA to acquire an IP.
// WiFi STA が IP を取得するまでの待機ポーリング設定。
constexpr TickType_t kWifiPollIntervalTicks = pdMS_TO_TICKS(200);
constexpr int        kWifiPollMaxAttempts   = 150;   // 200ms x 150 = 30s budget

// Rate-limit sendto errors so the log isn't flooded.
// sendto エラーログのレート制限（フラッディング防止）。
constexpr uint32_t kErrorLogEveryN = 50;             // ~1 per second at 50Hz

}  // namespace

// -----------------------------------------------------------------------------
// init — wait for WiFi, open the UDP socket, set defaults
// 初期化 — WiFi待機、UDPソケットを開き、既定値を設定
// -----------------------------------------------------------------------------
void Telemetry::init()
{
    // Block until sf_comm has WiFi up — we don't own WiFi.
    // sf_comm が WiFi を起動するまでブロックする（WiFi所有はsf_comm）。
    waitForWifi();

    // Create a UDP datagram socket on IPv4.
    // IPv4 の UDP データグラムソケットを生成。
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd_ < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return;
    }

    // Enable broadcast so 255.255.255.255 is permitted.
    // ブロードキャストを許可（255.255.255.255 を有効化）。
    int broadcast_enable = 1;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST,
                     &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ESP_LOGW(TAG, "SO_BROADCAST failed: errno=%d", errno);
    }

    // Make sendto() non-blocking so a missing receiver cannot stall the loop.
    // sendto をノンブロッキング化（受信者欠如でループ停滞を防ぐ）。
    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
        ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    // Default destination: limited broadcast on UDP_TELEMETRY_PORT.
    // 既定送信先: UDP_TELEMETRY_PORT へのリミテッドブロードキャスト。
    dest_ip_be_ = htonl(INADDR_BROADCAST);   // 255.255.255.255
    dest_port_  = UDP_TELEMETRY_PORT;

    ready_ = true;
    ESP_LOGI(TAG, "Telemetry ready: dest=255.255.255.255:%u, packet=%u bytes",
             static_cast<unsigned>(dest_port_),
             static_cast<unsigned>(sizeof(TelemetryPacket)));
}

// -----------------------------------------------------------------------------
// waitForWifi — block on sf_comm's WiFi-ready EventGroup
// WiFi待機 — sf_comm の WiFi 準備完了 EventGroup を待つ
// -----------------------------------------------------------------------------
//
// Previously this polled esp_netif_get_ip_info() in a loop. Now it
// delegates to sf::waitForWifiReady() which sleeps efficiently on an
// EventGroup bit set by sf_comm's IP_EVENT_STA_GOT_IP handler (R16).
//
// 旧実装は esp_netif_get_ip_info() を polling ループで呼んでいた。
// 現在は sf::waitForWifiReady() に委譲し、sf_comm の
// IP_EVENT_STA_GOT_IP ハンドラがセットする EventGroup ビットで効率的に
// スリープする (R16)。
//
// On timeout we log a warning and return — callers must tolerate the
// case where IP is never obtained (e.g. ESP-NOW-only operation without
// AP association). The UDP socket is still opened; broadcast may or
// may not work depending on the actual network state.
//
// タイムアウト時は警告ログを出して return する。呼び出し側は IP を
// 取得しないケース (ESP-NOW のみで AP 非接続) を許容する必要がある。
// UDP ソケット自体は開かれるが、ブロードキャスト送信が機能するかは
// 実ネットワーク状態に依存する。
// -----------------------------------------------------------------------------
void Telemetry::waitForWifi()
{
    constexpr uint32_t kReadyTimeoutMs = 30000;  // 30s: previous polling cap
    if (waitForWifiReady(kReadyTimeoutMs)) {
        esp_netif_t* sta_netif =
            esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        esp_netif_ip_info_t ip{};
        if (sta_netif != nullptr &&
            esp_netif_get_ip_info(sta_netif, &ip) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi ready: IP=" IPSTR, IP2STR(&ip.ip));
        } else {
            ESP_LOGI(TAG, "WiFi ready (IP info unavailable)");
        }
        return;
    }
    ESP_LOGW(TAG, "WiFi readiness timeout (%lu ms) — proceeding anyway",
             static_cast<unsigned long>(kReadyTimeoutMs));
}

// -----------------------------------------------------------------------------
// update — build one packet from latest topic data and send it
// 更新 — 最新トピックデータから1パケットを構築して送信
// -----------------------------------------------------------------------------
void Telemetry::update()
{
    if (!ready_) return;

    TelemetryPacket pkt{};
    buildPacket(pkt);
    sendPacket(pkt);
}

// -----------------------------------------------------------------------------
// buildPacket — fill the packet from sf::* topic latest() snapshots
// パケット構築 — sf::* トピックの latest() スナップショットを詰める
// -----------------------------------------------------------------------------
void Telemetry::buildPacket(TelemetryPacket& pkt)
{
    // Snapshot all topics first to minimize skew between fields.
    // フィールド間のずれを最小化するため、まず全トピックをスナップショット。
    const StateEstimate state   = estimate_state.latest();
    const ControlOutput control = control_output.latest();
    const MotorOutput   motor   = actuator_motor.latest();
    const ImuData       imu     = sensor_imu.latest();
    const SystemMode    mode    = system_mode.latest();

    // Header / ヘッダ
    pkt.magic        = TELEM_MAGIC;
    pkt.version      = TELEM_VERSION;
    pkt.packet_type  = TELEM_TYPE_PHASE2A_BASIC;
    pkt.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());

    // Convert quaternion (w,x,y,z) to Euler (roll,pitch,yaw).
    // クォータニオン (w,x,y,z) をオイラー角 (roll,pitch,yaw) に変換。
    sf::math::Quat q(state.attitude[0], state.attitude[1],
                     state.attitude[2], state.attitude[3]);
    sf::math::Vec3 euler = q.to_euler();
    pkt.roll  = euler.x;
    pkt.pitch = euler.y;
    pkt.yaw   = euler.z;

    // IMU body-frame rates and acceleration (raw latest sample).
    // IMU 機体系角速度・加速度（最新サンプルの生値）。
    pkt.gyro_x  = imu.gyro[0];
    pkt.gyro_y  = imu.gyro[1];
    pkt.gyro_z  = imu.gyro[2];
    pkt.accel_x = imu.accel[0];
    pkt.accel_y = imu.accel[1];
    pkt.accel_z = imu.accel[2];

    // Position / Velocity in NED frame from estimator.
    // 推定器から得た NED 系の位置・速度。
    pkt.pos_x = state.position[0];
    pkt.pos_y = state.position[1];
    pkt.pos_z = state.position[2];
    pkt.vel_x = state.velocity[0];
    pkt.vel_y = state.velocity[1];
    pkt.vel_z = state.velocity[2];

    // Control output: thrust [N] + body torque vector [Nm].
    // 制御出力: 推力 [N] + 機体トルク [Nm]。
    pkt.thrust       = control.thrust;
    pkt.roll_torque  = control.torque[0];
    pkt.pitch_torque = control.torque[1];
    pkt.yaw_torque   = control.torque[2];

    // Motor duty M1..M4 (X-quad order).
    // モーターduty M1..M4（X-quad順）。
    for (int i = 0; i < 4; ++i) {
        pkt.motor_duty[i] = motor.duty[i];
    }

    // System mode (flight state byte; sub_mode/armed are not in Phase 2a).
    // システムモード（FlightState のみ。sub_mode/armed は Phase 2a 対象外）。
    pkt.system_mode = mode.state;
}

// -----------------------------------------------------------------------------
// sendPacket — UDP sendto with rate-limited error logging
// パケット送信 — UDP sendto。エラーはレート制限付きでログ
// -----------------------------------------------------------------------------
void Telemetry::sendPacket(const TelemetryPacket& pkt)
{
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(dest_port_);
    addr.sin_addr.s_addr = dest_ip_be_;

    int sent = ::sendto(socket_fd_, &pkt, sizeof(pkt), 0,
                        reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (sent == static_cast<int>(sizeof(pkt))) return;

    // Rate-limit failure logging (~1/sec at 50Hz).
    // 送信失敗ログをレート制限（50Hz で約1回/秒）。
    if ((err_count_++ % kErrorLogEveryN) == 0) {
        ESP_LOGW(TAG, "sendto failed: ret=%d errno=%d (count=%lu)",
                 sent, errno, static_cast<unsigned long>(err_count_));
    }
}

// -----------------------------------------------------------------------------
// setDestination — override defaults (used by CLI or tests)
// 送信先設定 — 既定値を上書き（CLIやテスト用）
// -----------------------------------------------------------------------------
void Telemetry::setDestination(uint32_t ip_be, uint16_t port)
{
    dest_ip_be_ = ip_be;
    dest_port_  = port;
}

}  // namespace sf
