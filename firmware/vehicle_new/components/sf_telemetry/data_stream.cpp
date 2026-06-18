/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file data_stream.cpp
 * @brief Data Stream server implementation (see data_stream.hpp).
 *        Data Stream サーバ実装（data_stream.hpp 参照）。
 *
 * Loss-prevention design carried over from the proven firmware/vehicle:
 *   1. exclusive log mode — the 50Hz monitoring telemetry is suppressed while
 *      a capture is active (TelemetryTask checks isActive())
 *   2. reset-on-start — the ring backlog is discarded when a client connects,
 *      so a capture never begins with stale half-batches
 *   3. non-blocking fire-and-forget sendto — a slow/absent receiver cannot
 *      stall the 50Hz loop; failures are counted, not retried
 *   4. ring catch-up — the drain loop empties however many samples are
 *      pending, so a late cycle sends 2 packets instead of losing data
 *   5. heartbeat watchdog — 5 s of client silence stops the stream and frees
 *      the WiFi bandwidth
 * 実証済み firmware/vehicle から引き継いだ欠損防止設計:
 *   1. 排他ログモード — キャプチャ中は 50Hz モニタ用テレメトリを抑止
 *      （TelemetryTask が isActive() を見る）
 *   2. 接続時リセット — クライアント接続時にリングの滞留を破棄し、
 *      古い半端バッチからキャプチャが始まらないようにする
 *   3. ノンブロッキング fire-and-forget sendto — 受信者が遅い/居なくても
 *      50Hz ループは止まらない。失敗は計数のみで再送しない
 *   4. リング追いつき — 排出ループは滞留分を全て掃くので、遅れた周期は
 *      データを失わず 2 パケット送る
 *   5. ハートビート監視 — クライアント 5 秒無音で配信を止め WiFi 帯域を解放
 *
 * @design requirements.md §7 — Data Stream（解析用・全レート・UDP）       [OK]
 * @design architecture.md §5 — ログフロー: Data Stream                    [OK]
 */

#include "data_stream.hpp"

#include <cstring>
#include <fcntl.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"   // esp_reset_reason() — boot crash cause for telemetry

#include "topics.hpp"
#include "params.hpp"

static const char* TAG = "DataStream";

namespace sf {

// -----------------------------------------------------------------------------
// init — bind UDP 8890 (non-blocking) and wait for client commands.
// init — UDP 8890 を bind（ノンブロッキング）しクライアントコマンドを待つ。
// -----------------------------------------------------------------------------
void DataStream::init()
{
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd_ < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return;
    }

    sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(datastream::kUdpLogPort);
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(%u) failed: errno=%d",
                 static_cast<unsigned>(datastream::kUdpLogPort), errno);
        ::close(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    // Non-blocking: both the command recvfrom (polled at 50Hz) and the stream
    // sendto (fire-and-forget) must never stall the telemetry loop.
    // ノンブロッキング: コマンド recvfrom（50Hz ポーリング）も配信 sendto
    // （fire-and-forget）もテレメトリループを止めてはならない。
    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
        ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    ESP_LOGI(TAG, "Data Stream ready on UDP %u (start=0x%02X stop=0x%02X)",
             static_cast<unsigned>(datastream::kUdpLogPort),
             datastream::kCmdStartLog, datastream::kCmdStopLog);
}

// -----------------------------------------------------------------------------
// update — one 50Hz cycle.
// update — 50Hz の 1 周期。
// -----------------------------------------------------------------------------
void DataStream::update()
{
    if (socket_fd_ < 0) {
        return;
    }

    handleCommands();

    if (!active_) {
        return;   // ring keeps drop-oldest on its own / リングは自前で最古上書き
    }

    // Heartbeat watchdog (5 s of client silence → stop).
    // ハートビート監視（クライアント 5 秒無音 → 停止）。
    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_heartbeat_us_ >
        static_cast<int64_t>(datastream::kHeartbeatTimeoutMs) * 1000) {
        stopStream("heartbeat timeout");
        return;
    }

    // Drain the ring: nominal 8 samples per 50Hz cycle; a delayed cycle finds
    // more and emits several packets (catch-up, loss-prevention #4).
    // リング排出: 公称は 50Hz 周期あたり 8 件。遅れた周期は多めに見つけて複数
    // パケットを送る（追いつき、欠損防止④）。
    LogStreamSample sample;
    while (log_stream.read(sample)) {
        batch_[batch_count_++] = sample;
        if (batch_count_ == datastream::kSamplesPerPacket) {
            emitUnified();
            batch_count_ = 0;
        }
    }

    // 1Hz status (uptime, battery, rate-PID gains for PC-side reconstruction).
    // 1Hz ステータス（稼働時間・電池・PC 側再構成用レート PID ゲイン）。
    if ((++update_count_ % 50) == 0) {
        sendStatus();
    }
}

// -----------------------------------------------------------------------------
// handleCommands — drain all pending 1-byte client commands.
// handleCommands — 滞留しているクライアントコマンド（1 バイト）を全て処理。
// -----------------------------------------------------------------------------
void DataStream::handleCommands()
{
    uint8_t command = 0;
    sockaddr_in src = {};
    socklen_t src_len = sizeof(src);

    while (true) {
        const ssize_t received =
            ::recvfrom(socket_fd_, &command, 1, 0,
                       reinterpret_cast<sockaddr*>(&src), &src_len);
        if (received < 1) {
            return;   // no more commands (non-blocking) / コマンドなし
        }

        switch (command) {
        case datastream::kCmdStartLog:
            startStream(src);
            break;
        case datastream::kCmdStopLog:
            stopStream("client stop");
            break;
        case datastream::kCmdHeartbeat:
            last_heartbeat_us_ = esp_timer_get_time();
            break;
        default:
            break;   // unknown command — ignore / 不明コマンドは無視
        }
        src_len = sizeof(src);
    }
}

// -----------------------------------------------------------------------------
// startStream — register the sender as the client; reset the stream state.
// startStream — 送信元をクライアントとして登録し、配信状態をリセットする。
// -----------------------------------------------------------------------------
void DataStream::startStream(const sockaddr_in& client)
{
    client_            = client;
    active_            = true;
    last_heartbeat_us_ = esp_timer_get_time();
    batch_count_       = 0;
    send_drops_        = 0;
    entry_drops_       = 0;

    // Reset-on-start (loss-prevention #2): discard the ring backlog so the
    // capture starts at "now", not with however many stale samples piled up.
    // 接続時リセット（欠損防止②）: リングの滞留を破棄し、キャプチャが古い
    // 滞留サンプルからでなく「今」から始まるようにする。
    LogStreamSample discard;
    while (log_stream.read(discard)) {
    }
    FlowData flow_discard;
    while (log_flow.read(flow_discard)) {
    }

    ESP_LOGI(TAG, "Capture START → client registered (50 pkt/s, ~48 KB/s)");
}

void DataStream::stopStream(const char* reason)
{
    if (!active_) {
        return;
    }
    active_ = false;
    ESP_LOGI(TAG, "Capture STOP (%s) — %lu sendto drops, %lu entry drops",
             reason, static_cast<unsigned long>(send_drops_),
             static_cast<unsigned long>(entry_drops_));
}

// -----------------------------------------------------------------------------
// emitUnified — build one 0x50 packet from the 8-sample batch + entries, send.
// emitUnified — 8 サンプルバッチ＋エントリから 0x50 を 1 つ組んで送信する。
// -----------------------------------------------------------------------------
void DataStream::emitUnified()
{
    datastream::UnifiedPacketBuilder builder;
    builder.begin(unified_seq_++, batch_);
    appendEntries(builder);
    const size_t length = builder.finish();
    sendDatagram(builder.buffer(), length);
}

// -----------------------------------------------------------------------------
// countDrop — make entry overflow LOUD. addEntry() rejects an entry when the
// packet would exceed kUnifiedMaxSize; silently losing it cost us the mag
// channel once already (found only by a hardware capture). Every rejection is
// counted and warned about, so adding future entries (front ToF, ESKF P-diag)
// can never silently shrink the log again ("no silent caps").
// countDrop — エントリ溢れを「大声」にする。addEntry() はパケットが
// kUnifiedMaxSize を超えるとエントリを拒否する。黙って失うと mag チャネルを
// 丸ごと失った実績がある（実機キャプチャで偶然発見）。拒否は全て計数・警告し、
// 将来のエントリ追加（前方 ToF・ESKF P対角）で二度と黙ってログが痩せないようにする
// （silent caps 禁止）。
// -----------------------------------------------------------------------------
void DataStream::countDrop(bool added)
{
    if (added) {
        return;
    }
    if ((++entry_drops_ % 50) == 1) {   // ~once per second at 50Hz when dropping
        ESP_LOGW(TAG, "unified packet FULL — entry dropped (x%lu): raise "
                      "kUnifiedMaxSize (%u B)",
                 static_cast<unsigned long>(entry_drops_),
                 static_cast<unsigned>(datastream::kUnifiedMaxSize));
    }
}

// -----------------------------------------------------------------------------
// appendEntries — 50Hz control entries + low-rate sensor entries (on change).
// appendEntries — 50Hz の制御エントリ＋低レートセンサエントリ（更新時のみ）。
// -----------------------------------------------------------------------------
void DataStream::appendEntries(datastream::UnifiedPacketBuilder& builder)
{
    // Pilot input (50Hz cadence — one per packet, like the old vehicle).
    // パイロット入力（50Hz — 旧 vehicle と同じくパケットあたり 1 件）。
    const CommandSetpoint setpoint = command_setpoint.latest();
    datastream::WireControl control = {};
    control.timestamp_us = setpoint.timestamp;
    control.throttle     = setpoint.throttle;
    control.roll         = setpoint.roll;
    control.pitch        = setpoint.pitch;
    control.yaw          = setpoint.yaw;
    countDrop(builder.addEntry(datastream::kPktControl, &control, sizeof(control)));

    // Control references + motor duty, taken from the batch's LAST sample
    // (v3 wire layout: data_size 30 selects the version on the PC side).
    // 制御目標＋モータ duty。バッチ最終サンプルから採取（v3 電文: PC 側は
    // data_size=30 でバージョンを判別する）。
    const LogStreamSample& last = batch_[datastream::kSamplesPerPacket - 1];
    datastream::WireCtrlRef ctrl_ref = {};
    ctrl_ref.timestamp_us = last.timestamp;
    ctrl_ref.flight_mode  = last.flight_mode;
    ctrl_ref.angle_ref[0] = datastream::quantize(last.angle_ref[0],
                                                 datastream::kAngleRefScale);
    ctrl_ref.angle_ref[1] = datastream::quantize(last.angle_ref[1],
                                                 datastream::kAngleRefScale);
    ctrl_ref.total_thrust = last.thrust;
    for (int i = 0; i < 4; ++i) {
        ctrl_ref.motor_duty[i] = last.duty[i];   // FR, RR, RL, FL
    }
    countDrop(builder.addEntry(datastream::kPktCtrlRef, &ctrl_ref, sizeof(ctrl_ref)));

    // Flow: drain EVERY sample from the log_flow ring (typically 2 per packet
    // at 100Hz). Flow dx/dy are INCREMENTAL — displacement since the previous
    // read — so a latest()-snapshot peek would silently drop half the
    // displacement and corrupt any offline integration/replay. Absolute
    // sensors (ToF/baro/mag) below can use the snapshot safely.
    // フロー: log_flow リングから「全」サンプルを排出（100Hz なら 1 パケットに
    // 通常 2 件）。フローの dx/dy は「差分量」— 前回読み出しからの変位 — なので、
    // latest() スナップショット方式では変位の半分が黙って失われ、オフラインの
    // 積分・再生が壊れる。下の絶対量センサ（ToF/気圧/磁気）はスナップショットで安全。
    FlowData flow_sample;
    while (log_flow.read(flow_sample)) {
        datastream::WireFlow flow = {};
        flow.timestamp_us = flow_sample.timestamp;
        flow.dx           = flow_sample.dx;
        flow.dy           = flow_sample.dy;
        flow.quality      = flow_sample.squal;
        countDrop(builder.addEntry(datastream::kPktFlow, &flow, sizeof(flow)));
    }

    // Low-rate ABSOLUTE sensors: append only when the per-sensor stamp advanced.
    // 低レートの「絶対量」センサ: センサ別時刻が進んだときだけ追加する。
    const SensorSnapshot snap = sensor_snapshot.latest();
    if (snap.tof_timestamp != last_tof_ts_) {
        last_tof_ts_ = snap.tof_timestamp;
        datastream::WireTof tof = {};
        tof.timestamp_us = snap.tof_timestamp;
        tof.distance     = snap.tof_distance;
        tof.status       = snap.tof_status;
        countDrop(builder.addEntry(datastream::kPktTofBottom, &tof, sizeof(tof)));
    }
    if (snap.baro_timestamp != last_baro_ts_) {
        last_baro_ts_ = snap.baro_timestamp;
        datastream::WireBaro baro = {};
        baro.timestamp_us = snap.baro_timestamp;
        baro.altitude     = snap.baro_altitude;
        baro.pressure     = snap.baro_pressure * 0.01f;   // Pa → hPa (wire unit)
        countDrop(builder.addEntry(datastream::kPktBaro, &baro, sizeof(baro)));
    }
    if (snap.mag_timestamp != last_mag_ts_) {
        last_mag_ts_ = snap.mag_timestamp;
        datastream::WireMag mag = {};
        mag.timestamp_us = snap.mag_timestamp;
        for (int i = 0; i < 3; ++i) {
            mag.mag[i] = snap.mag[i];
        }
        countDrop(builder.addEntry(datastream::kPktMag, &mag, sizeof(mag)));
    }
}

// -----------------------------------------------------------------------------
// sendStatus — standalone 1Hz status packet (0x4F).
// sendStatus — 単独 1Hz ステータスパケット（0x4F）。
// -----------------------------------------------------------------------------
void DataStream::sendStatus()
{
    const PowerData    power  = sensor_power.latest();
    const SystemMode   mode   = system_mode.latest();
    const SensorHealth health = sensor_health.latest();

    datastream::WireStatusPayload payload = {};
    payload.uptime_ms     = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    payload.voltage       = power.voltage;
    payload.flight_state  = mode.state;
    payload.sensor_health = health.healthy_mask;
    payload.eskf_status   = 0x01;   // estimator running / 推定器稼働中
    // Why the chip last reset — readable over WiFi so a crash cause (e.g. BROWNOUT
    // from motor-current sag during an oscillation) can be diagnosed without a serial
    // monitor. Constant per boot; sent every 1Hz status so any post-crash reboot shows it.
    // 直近のリセット理由。シリアル無しでも無線で墜落原因（発振時のモータ電流降下による
    // BROWNOUT 等）を診断できる。起動毎に一定で、再起動後の status に必ず乗る。
    payload.reset_reason  = static_cast<uint8_t>(esp_reset_reason());

    // Rate-PID gains for PC-side motor-command reconstruction tools.
    // PC 側のモータ指令再構成ツール用レート PID ゲイン。
    const char* names[9] = {
        "rate.roll.kp",  "rate.roll.ti",  "rate.roll.td",
        "rate.pitch.kp", "rate.pitch.ti", "rate.pitch.td",
        "rate.yaw.kp",   "rate.yaw.ti",   "rate.yaw.td",
    };
    for (int i = 0; i < 9; ++i) {
        params::get_float(names[i], payload.pid_gains[i]);
    }

    uint8_t buffer[64];
    const size_t length =
        datastream::buildStatusPacket(buffer, status_seq_++, payload);
    sendDatagram(buffer, length);
}

// -----------------------------------------------------------------------------
// sendDatagram — fire-and-forget sendto with drop counting.
// sendDatagram — 送信失敗を計数する fire-and-forget sendto。
// -----------------------------------------------------------------------------
void DataStream::sendDatagram(const uint8_t* data, size_t length)
{
    const ssize_t sent =
        ::sendto(socket_fd_, data, length, 0,
                 reinterpret_cast<const sockaddr*>(&client_), sizeof(client_));
    if (sent != static_cast<ssize_t>(length)) {
        // Count, don't retry (loss-prevention #3): the PC detects gaps via the
        // sequence counter; blocking here would lose MORE data upstream.
        // 計数のみで再送しない（欠損防止③）: 欠落は PC がシーケンスで検出する。
        // ここでブロックすると上流でより多くを失う。
        if ((++send_drops_ % 250) == 1) {
            ESP_LOGW(TAG, "sendto failed (x%lu): errno=%d",
                     static_cast<unsigned long>(send_drops_), errno);
        }
    }
}

}  // namespace sf
