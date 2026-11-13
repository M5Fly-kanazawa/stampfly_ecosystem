/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file data_stream.hpp
 * @brief Data Stream server — control-rate (400Hz) analysis log over UDP,
 *        wire-compatible with the proven firmware/vehicle protocol.
 *        Data Stream サーバ — 制御周期（400Hz）の解析用ログを UDP 配信。
 *        実証済み firmware/vehicle のプロトコルと電文互換。
 *
 * Client workflow (identical to another flight-proven firmware):
 *   `sf log wifi --ip <vehicle>` sends CMD_START_LOG to UDP 8890; the vehicle
 *   streams unified packets (8 × 400Hz samples per packet, 50 packets/s,
 *   ≈48 KB/s) back to the sender until CMD_STOP_LOG or 5 s without a heartbeat.
 * クライアント手順（飛行実績のある別ファームと同一）:
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
 * Besides the unified packets the same cycle sends (1) the 1 Hz status packet v4 (v3 + ToF
 * diagnostics, loop timing, non-finite counters, log-path loss counters) and (2) the text-log
 * packet 0x51 (the ESP_LOG lines of the log sink; only while a capture is active and a line waits;
 * at most every kTextLogSendPeriodCycles-th cycle). Neither changes the unified packet.
 * 統合パケットのほかに、同じ周期が (1) 1 Hz のステータスパケット v4（v3＋ToF の診断値・ループ周期・有限でない数・
 * ログ経路の欠落の計数）と (2) テキストログのパケット 0x51（ログシンクの ESP_LOG の行。取得中で行が待っている
 * ときだけ、最大 kTextLogSendPeriodCycles 周期に 1 回）を送る。どちらも統合パケットは変えない。
 *
 * @design requirements.md §7 — Data Stream（解析用・全レート・UDP）       [OK]
 * @design architecture.md §5 — ログフロー: Data Stream                    [OK]
 * @design detailed_design.md §8 — Log sink: WiFi exit (text log), status packet v4  [OK]
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
    void append400HzEntries(datastream::UnifiedPacketBuilder& builder);   // 400Hz duty, pre-mixer output / 400Hz の duty、ミキサー手前の出力
    void appendFlightPhase400Entry(datastream::UnifiedPacketBuilder& builder);   // 400Hz flight state + flip phase / 400Hz の飛行状態＋フリップのフェーズ
    void appendFlightFlagsEntry(datastream::UnifiedPacketBuilder& builder);   // 50Hz flags / 50Hz のフラグ
    void appendPilotAndReferenceEntries(datastream::UnifiedPacketBuilder& builder);   // pilot input, control references, flow / パイロット入力、制御目標、フロー
    void appendAbsoluteSensorEntries(datastream::UnifiedPacketBuilder& builder);   // low-rate absolute sensors, on change / 低レートの絶対量センサ（更新時のみ）
    void appendBaroHeightEntry(datastream::UnifiedPacketBuilder& builder, const SensorSnapshot& snap);   // barometric height above the floor (0x52), with each new baro sample / 床からの気圧高度（0x52）。気圧計の新サンプルごと
    void appendToFSilenceEntry(datastream::UnifiedPacketBuilder& builder);   // 50Hz ToF-silence facts (0x4E) / 50Hz ToF 出力停止の事実
    void emitTextLog();   // ESP_LOG text lines from the log sink, packet 0x51 (every kTextLogSendPeriodCycles-th cycle, <= kTextLogBytesPerPacket) / ログシンクの ESP_LOG の行を送る（パケット 0x51。kTextLogSendPeriodCycles 周期に 1 回、kTextLogBytesPerPacket 以下）
    void sendStatus();
    datastream::WireStatusV4Extension buildStatusExtension() const;   // v4 part of the status packet / ステータスパケットの v4 部分
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
    uint16_t text_seq_    = 0;
    int      text_cycle_  = 0;         // 50Hz cycles since the last text-log send opportunity / 前回のテキストログ送信の機会からの 50Hz 周期数         // text-log packet counter (own, loss detection) / テキストログのパケット通し番号（専用。欠落検出）
    uint32_t log_stream_overflow_base_ = 0;   // log_stream overflow count at the capture start / 取得開始時の log_stream 溢れ数
    uint32_t update_count_ = 0;        // 1Hz status divider / 1Hz ステータス分周

    // Last-seen per-sensor stamps (new-sample detection for low-rate entries).
    // 低レートエントリの新サンプル検出用の最終時刻。
    uint32_t last_tof_ts_  = 0;
    uint32_t last_tof_front_ts_ = 0;
    uint32_t last_baro_ts_ = 0;
    uint32_t last_mag_ts_  = 0;

    uint32_t send_drops_ = 0;          // sendto failures (counted, not fatal) / 送信失敗数
    uint32_t entry_drops_ = 0;         // packet-full entry rejections / パケット満杯による棄却数
};

}  // namespace sf
