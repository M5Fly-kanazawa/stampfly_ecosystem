/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file data_stream.hpp
 * @brief Data Stream server — control-rate (400Hz) analysis log over UDP,
 *        wire-compatible with the proven firmware/vehicle protocol.
 *        Data Stream サーバ — 制御周期（400Hz）の解析用ログを UDP 配信。
 *        実証済み firmware/vehicle のプロトコルと電文互換。
 *
 * Client workflow (identical to the old vehicle):
 *   `sf log wifi --ip <vehicle>` sends CMD_START_LOG to UDP 8890; the vehicle
 *   streams unified packets (8 × 400Hz samples per packet, 50 packets/s,
 *   ≈48 KB/s) back to the sender until CMD_STOP_LOG or 5 s without a heartbeat.
 * クライアント手順（旧 vehicle と同一）:
 *   `sf log wifi --ip <機体>` が UDP 8890 へ CMD_START_LOG を送ると、機体は
 *   統合パケット（1 パケット 8 サンプル × 400Hz、50 パケット/s ≈ 48KB/s）を
 *   送信元へ配信する。CMD_STOP_LOG かハートビート 5 秒断で停止。
 *
 * Data path: ControlTask publishes one LogStreamSample per control cycle on the
 * log_stream ring (32 deep = 80 ms of jitter margin); TelemetryTask calls
 * update() at 50Hz which drains the ring, batches 8 samples per packet, and
 * appends 50Hz/low-rate sensor entries.
 * データ経路: ControlTask が制御周期毎に LogStreamSample を log_stream リング
 * （深さ 32 = 80ms のジッタ余裕）へ発行し、TelemetryTask が 50Hz で update() を
 * 呼ぶ。update() はリングを排出し 8 サンプル/パケットでバッチし、50Hz・低レートの
 * センサエントリを付加する。
 *
 * @design requirements.md §7 — Data Stream（解析用・全レート・UDP）       [OK]
 * @design architecture.md §5 — ログフロー: Data Stream                    [OK]
 */

#pragma once

#include <cstdint>

#include "lwip/sockets.h"
#include "data_types.hpp"
#include "data_stream_wire.hpp"

namespace sf {

class DataStream {
public:
    /// Open and bind the UDP command/stream socket (call after WiFi is ready).
    /// UDP コマンド/配信ソケットを開いて bind する（WiFi 準備完了後に呼ぶ）。
    void init();

    /// One 50Hz cycle: process client commands, drain the log_stream ring,
    /// emit unified packets, and the 1Hz status packet.
    /// 50Hz の 1 周期: クライアントコマンド処理、log_stream リング排出、
    /// 統合パケット送信、1Hz ステータス送信。
    void update();

    /// True while a client is subscribed. TelemetryTask suppresses the 50Hz
    /// monitoring packet during a capture (exclusive-log policy: the analysis
    /// stream owns the WiFi bandwidth — proven loss-prevention design).
    /// クライアント購読中は true。キャプチャ中 TelemetryTask は 50Hz モニタ用
    /// パケットを抑止する（排他ログ方針: 解析ストリームが WiFi 帯域を専有 —
    /// 実証済みの欠損防止設計）。
    bool isActive() const { return active_; }

private:
    void handleCommands();
    void startStream(const sockaddr_in& client);
    void stopStream(const char* reason);
    void emitUnified();
    void appendEntries(datastream::UnifiedPacketBuilder& builder);
    void sendStatus();
    void sendDatagram(const uint8_t* data, size_t length);
    void countDrop(bool added);   // entry-overflow accounting / エントリ溢れの計数

    int  socket_fd_ = -1;
    bool active_    = false;

    sockaddr_in client_ = {};          // registered stream client / 登録クライアント
    int64_t last_heartbeat_us_ = 0;

    // Batch accumulator: samples held until kSamplesPerPacket are available.
    // バッチ蓄積: kSamplesPerPacket 件たまるまで保持。
    LogStreamSample batch_[datastream::kSamplesPerPacket] = {};
    int batch_count_ = 0;

    uint16_t unified_seq_ = 0;         // loss-detection counter / 欠落検出カウンタ
    uint16_t status_seq_  = 0;
    uint32_t update_count_ = 0;        // 1Hz status divider / 1Hz ステータス分周

    // Last-seen per-sensor stamps (new-sample detection for low-rate entries).
    // 低レートエントリの新サンプル検出用の最終時刻。
    uint32_t last_tof_ts_  = 0;
    uint32_t last_baro_ts_ = 0;
    uint32_t last_mag_ts_  = 0;

    uint32_t send_drops_ = 0;          // sendto failures (counted, not fatal) / 送信失敗数
    uint32_t entry_drops_ = 0;         // packet-full entry rejections / パケット満杯による棄却数
};

}  // namespace sf
