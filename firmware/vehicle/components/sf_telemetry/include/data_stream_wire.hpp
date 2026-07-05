/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file data_stream_wire.hpp
 * @brief Data Stream wire format — WIRE-COMPATIBLE with the proven
 *        firmware/vehicle 400Hz UDP log protocol, so the existing PC tooling
 *        (`sf log wifi` → tools/log_analyzer/udp_capture.py → `sf log viz`)
 *        works unchanged on vehicle.
 *        Data Stream の電文定義 — 実証済み firmware/vehicle の 400Hz UDP ログ
 *        プロトコルと「電文互換」。既存の PC ツール（`sf log wifi` →
 *        udp_capture.py → `sf log viz`）が vehicle でも無改造で動く。
 *
 * Protocol summary (SSOT shared with tools/log_analyzer/udp_capture.py):
 *   - vehicle binds UDP port 8890; the PC sends 1-byte commands:
 *     0xF0 START (the sender address becomes the stream client),
 *     0xF1 STOP, 0xF2 HEARTBEAT (vehicle auto-stops after 5 s silence).
 *   - Unified packet 0x50, sent at 50 Hz, batches 8 control cycles (400 Hz):
 *       [Header 4B][ImuEskf 80B x8][PosVel 28B x8][RateRef 6B x8]
 *       [entry_count 1B][SensorEntry...][XOR checksum 1B]
 *     SensorEntry = [sensor_id 1B][data_size 1B][payload data_size B].
 *   - Status packet 0x4F (53B) at 1 Hz: battery, state, rate-PID gains.
 *   - XOR checksum over every preceding byte of the datagram.
 *
 * This header is dependency-free (cstdint/cstring + LogStreamSample) so the
 * packing layer is verifiable by the HOST unit tests (test_main.cpp) — the
 * byte layout is asserted against udp_capture.py's struct formats.
 * 本ヘッダは依存フリー（cstdint/cstring + LogStreamSample）で、パッキング層を
 * ホスト単体テスト（test_main.cpp）で検証できる — バイトレイアウトは
 * udp_capture.py の struct フォーマットと突き合わせて assert される。
 *
 * @design requirements.md §7 — Data Stream（解析用・全レート・UDP）       [OK]
 * @design architecture.md §5 — ログフロー: Data Stream                    [OK]
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "data_types.hpp"   // LogStreamSample

namespace sf {
namespace datastream {

// =============================================================================
// Protocol constants (must match udp_capture.py)
// プロトコル定数（udp_capture.py と一致必須）
// =============================================================================

inline constexpr uint16_t kUdpLogPort   = 8890;
inline constexpr uint8_t  kCmdStartLog  = 0xF0;
inline constexpr uint8_t  kCmdStopLog   = 0xF1;
inline constexpr uint8_t  kCmdHeartbeat = 0xF2;
inline constexpr uint32_t kHeartbeatTimeoutMs = 5000;

inline constexpr uint8_t kPktControl   = 0x42;  // pilot stick input (50Hz entry)
inline constexpr uint8_t kPktFlow      = 0x43;
inline constexpr uint8_t kPktTofBottom = 0x44;
inline constexpr uint8_t kPktBaro      = 0x45;
inline constexpr uint8_t kPktMag       = 0x46;
inline constexpr uint8_t kPktCtrlRef   = 0x48;  // outer-loop refs + duty (50Hz entry)
inline constexpr uint8_t kPktStatus    = 0x4F;  // standalone 1Hz packet
inline constexpr uint8_t kPktUnified   = 0x50;  // 50Hz batched packet

/// 8 control cycles (400 Hz) per unified packet → 50 sendto()/s, ≈48 KB/s.
/// The packet rate, not the sample rate, is what the WiFi stack pays for —
/// this is the bandwidth design proven on firmware/vehicle.
/// 統合パケット1個 = 制御8周期（400Hz）→ 50 sendto()/s ≈ 48KB/s。WiFi スタックの
/// コストはサンプルレートでなく「パケットレート」— firmware/vehicle で実証済みの
/// 帯域設計。
inline constexpr int    kSamplesPerPacket = 8;

/// Datagram size cap. The legacy firmware used 1024, but delivering EVERY flow
/// sample (2 entries/packet at 100Hz) pushed the typical payload to ~1040B and
/// the LAST entry (mag, 18B) was silently rejected on every packet — measured
/// on hardware as "Mag: 0 samples". 1200 leaves ~160B of entry headroom and
/// stays well inside the WiFi MTU (1472) and the PC capture buffer (2048).
/// データグラム上限。旧ファームは 1024 だったが、フロー全量配送（100Hz で 1 パケット
/// 2 エントリ）で典型ペイロードが ~1040B となり、最後に積む mag（18B）が毎パケット
/// 黙って弾かれた — 実機計測で「Mag: 0 サンプル」。1200 ならエントリ余白 ~160B を
/// 確保しつつ WiFi MTU（1472）と PC 受信バッファ（2048）に余裕で収まる。
inline constexpr size_t kUnifiedMaxSize   = 1200;

// Quantization scales (PC side divides by these to restore physical units)
// 量子化スケール（PC 側はこれで割って物理量に復元する）
inline constexpr float kBiasScale    = 10000.0f;  // int16 = value × 10000
inline constexpr float kRateRefScale = 1000.0f;   // int16 = value × 1000
inline constexpr float kAngleRefScale = 10000.0f; // int16 = value × 10000

// =============================================================================
// Wire structs (packed; little-endian on both ESP32 and the host PC)
// 電文構造体（packed。ESP32 もホスト PC もリトルエンディアン）
// =============================================================================

#pragma pack(push, 1)

/// Common packet header — udp_capture.py FMT_HEADER '<B H B'
struct WireHeader {
    uint8_t  packet_id;
    uint16_t sequence;      // per-type rolling counter (loss detection) / 種別毎の欠落検出カウンタ
    uint8_t  sample_count;
};
static_assert(sizeof(WireHeader) == 4, "wire drift");

/// 400Hz IMU + estimator sample — FMT_IMU_ESKF '<I 3f 3f 3f 3f 4f 3h 3h' (80B)
struct WireImuEskf {
    uint32_t timestamp_us;
    float    gyro[3];        // [rad/s] estimator-input gyro    / 推定器入力ジャイロ
    float    accel[3];       // [m/s²]
    float    gyro_raw[3];    // [rad/s] pre-filter raw (vehicle has no IMU LPF,
                             // so raw == filtered — sent for wire compatibility)
                             // フィルタ前生値（vehicle は IMU LPF なしのため
                             // raw == filtered。電文互換のため両方送る）
    float    accel_raw[3];   // [m/s²]
    float    quat[4];        // [w,x,y,z]
    int16_t  gyro_bias[3];   // value × 10000 [rad/s]
    int16_t  accel_bias[3];  // value × 10000 [m/s²]
};
static_assert(sizeof(WireImuEskf) == 80, "wire drift");

/// 400Hz position/velocity sample — FMT_POS_VEL '<I 3f 3f' (28B)
struct WirePosVel {
    uint32_t timestamp_us;
    float    pos[3];   // NED [m]
    float    vel[3];   // NED [m/s]
};
static_assert(sizeof(WirePosVel) == 28, "wire drift");

/// 400Hz inner-loop rate reference — FMT_RATE_REF '<3h' (6B, no timestamp:
/// the PC pairs it with the same-index ImuEskf sample)
/// 内側ループ角速度目標（6B、時刻なし: PC 側が同 index の ImuEskf と対にする）
struct WireRateRef {
    int16_t rate_ref[3];   // value × 1000 [rad/s] R,P,Y
};
static_assert(sizeof(WireRateRef) == 6, "wire drift");

/// 50Hz pilot input entry — FMT_CONTROL '<I 4f' (20B)
struct WireControl {
    uint32_t timestamp_us;
    float    throttle;   // 0..1
    float    roll, pitch, yaw;   // −1..1
};
static_assert(sizeof(WireControl) == 20, "wire drift");

/// 50Hz control-reference entry — FMT_CTRL_REF_V3 '<I 2B 2h 5f' (30B)
struct WireCtrlRef {
    uint32_t timestamp_us;
    uint8_t  flight_mode;      // 0=ACRO 1=STAB 2=ALT 3=POS
    uint8_t  reserved;
    int16_t  angle_ref[2];     // value × 10000 [rad] R,P
    float    total_thrust;     // [N]
    float    motor_duty[4];    // FR, RR, RL, FL
};
static_assert(sizeof(WireCtrlRef) == 30, "wire drift");

/// Low-rate sensor entries — FMT_FLOW '<I 2h B' / FMT_TOF '<I f B' /
/// FMT_BARO '<I 2f' / FMT_MAG '<I 3f'
struct WireFlow {
    uint32_t timestamp_us;
    int16_t  dx, dy;       // [counts]
    uint8_t  quality;
};
static_assert(sizeof(WireFlow) == 9, "wire drift");

struct WireTof {
    uint32_t timestamp_us;
    float    distance;     // [m]
    uint8_t  status;       // 0 = valid
};
static_assert(sizeof(WireTof) == 9, "wire drift");

struct WireBaro {
    uint32_t timestamp_us;
    float    altitude;     // [m] relative
    float    pressure;     // [hPa] (NOTE: hPa on the wire, Pa inside the firmware)
};
static_assert(sizeof(WireBaro) == 12, "wire drift");

struct WireMag {
    uint32_t timestamp_us;
    float    mag[3];       // [µT] body
};
static_assert(sizeof(WireMag) == 16, "wire drift");

/// Standalone 1Hz status packet — 53B total (header + payload + checksum).
/// The 9 rate-PID gains let the PC-side tools reconstruct motor commands.
/// 単独 1Hz ステータスパケット（計 53B）。レート PID ゲイン 9 個は PC 側ツールの
/// モータ指令再構成用。
struct WireStatusPayload {
    uint32_t uptime_ms;
    float    voltage;          // [V]
    uint8_t  flight_state;     // FlightState
    uint8_t  sensor_health;    // healthy_mask bits
    uint8_t  eskf_status;      // bit0 = estimator initialized
    uint8_t  reset_reason;     // esp_reset_reason() at boot: 1=POWERON, 3=SW, 4=PANIC,
                               // 5=INT_WDT, 6=TASK_WDT, 9=BROWNOUT (was `padding`, so
                               // the 48B wire size is unchanged). Lets a crash cause be
                               // read over WiFi (no serial); it is constant per boot.
                               // 起動時リセット理由。墜落→再起動後に無線で原因を読める。
    float    pid_gains[9];     // roll kp/ti/td, pitch kp/ti/td, yaw kp/ti/td
};
static_assert(sizeof(WireStatusPayload) == 48, "wire drift");

#pragma pack(pop)

// =============================================================================
// Packing helpers (pure functions — host-unit-testable)
// パッキングヘルパ（純粋関数 — ホスト単体テスト可能）
// =============================================================================

/// Quantize a float to int16: round to nearest, saturate. (Truncation would
/// turn 0.0123 × 10000 = 122.99… into 122 — a systematic −0.5 LSB bias.)
/// float → int16 量子化: 四捨五入＋飽和。（切り捨てだと 0.0123×10000 = 122.99…
/// が 122 になり、系統的な −0.5 LSB バイアスが乗る。）
inline int16_t quantize(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >=  32767.0f) return  32767;
    if (scaled <= -32767.0f) return -32767;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

/// XOR checksum over a byte range — matches udp_capture.py's verifier.
/// バイト列の XOR チェックサム — udp_capture.py の検証と一致。
inline uint8_t xorChecksum(const uint8_t* data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum ^= data[i];
    }
    return sum;
}

/// Incremental builder for the unified packet 0x50. Usage:
///   begin(seq, samples) → addEntry()... → finish() → send buffer()/length().
/// 統合パケット 0x50 のビルダ。begin → addEntry... → finish の順に使う。
class UnifiedPacketBuilder {
public:
    /// Write the header and the three fixed 8-sample blocks.
    /// ヘッダと固定 3 ブロック（8 サンプル分）を書き込む。
    void begin(uint16_t sequence, const LogStreamSample samples[kSamplesPerPacket])
    {
        WireHeader header = {};
        header.packet_id    = kPktUnified;
        header.sequence     = sequence;
        header.sample_count = kSamplesPerPacket;
        length_ = 0;
        append(&header, sizeof(header));

        for (int i = 0; i < kSamplesPerPacket; ++i) {
            WireImuEskf imu = {};
            imu.timestamp_us = samples[i].timestamp;
            for (int axis = 0; axis < 3; ++axis) {
                imu.gyro[axis]       = samples[i].gyro[axis];
                imu.accel[axis]      = samples[i].accel[axis];
                imu.gyro_raw[axis]   = samples[i].gyro[axis];
                imu.accel_raw[axis]  = samples[i].accel[axis];
                imu.gyro_bias[axis]  = quantize(samples[i].gyro_bias[axis], kBiasScale);
                imu.accel_bias[axis] = quantize(samples[i].accel_bias[axis], kBiasScale);
            }
            for (int k = 0; k < 4; ++k) {
                imu.quat[k] = samples[i].quat[k];
            }
            append(&imu, sizeof(imu));
        }
        for (int i = 0; i < kSamplesPerPacket; ++i) {
            WirePosVel pv = {};
            pv.timestamp_us = samples[i].timestamp;
            for (int axis = 0; axis < 3; ++axis) {
                pv.pos[axis] = samples[i].pos[axis];
                pv.vel[axis] = samples[i].vel[axis];
            }
            append(&pv, sizeof(pv));
        }
        for (int i = 0; i < kSamplesPerPacket; ++i) {
            WireRateRef rr = {};
            for (int axis = 0; axis < 3; ++axis) {
                rr.rate_ref[axis] = quantize(samples[i].rate_ref[axis], kRateRefScale);
            }
            append(&rr, sizeof(rr));
        }

        // entry_count placeholder — patched by addEntry()
        // entry_count の場所取り — addEntry() が更新する
        entry_count_offset_ = length_;
        const uint8_t zero = 0;
        append(&zero, 1);
    }

    /// Append one [id][size][payload] sensor entry. False if it would not fit.
    /// [id][size][payload] エントリを 1 つ追加。収まらなければ false。
    bool addEntry(uint8_t sensor_id, const void* payload, uint8_t size)
    {
        if (length_ + 2 + size + 1 > kUnifiedMaxSize) {   // +1 = checksum reserve
            return false;
        }
        buffer_[length_++] = sensor_id;
        buffer_[length_++] = size;
        std::memcpy(&buffer_[length_], payload, size);
        length_ += size;
        ++buffer_[entry_count_offset_];
        return true;
    }

    /// Append the XOR checksum; returns the final datagram length.
    /// XOR チェックサムを付け、最終データグラム長を返す。
    size_t finish()
    {
        buffer_[length_] = xorChecksum(buffer_, length_);
        ++length_;
        return length_;
    }

    const uint8_t* buffer() const { return buffer_; }
    size_t length() const { return length_; }

private:
    void append(const void* data, size_t size)
    {
        std::memcpy(&buffer_[length_], data, size);
        length_ += size;
    }

    uint8_t buffer_[kUnifiedMaxSize] = {};
    size_t  length_ = 0;
    size_t  entry_count_offset_ = 0;
};

/// Build the standalone 1Hz status packet into `buffer` (≥ 53B); returns length.
/// 単独 1Hz ステータスパケットを buffer（53B 以上）へ構築し、長さを返す。
inline size_t buildStatusPacket(uint8_t* buffer, uint16_t sequence,
                                const WireStatusPayload& payload)
{
    WireHeader header = {};
    header.packet_id    = kPktStatus;
    header.sequence     = sequence;
    header.sample_count = 1;
    std::memcpy(buffer, &header, sizeof(header));
    std::memcpy(buffer + sizeof(header), &payload, sizeof(payload));
    const size_t body = sizeof(header) + sizeof(payload);
    buffer[body] = xorChecksum(buffer, body);
    return body + 1;
}

}  // namespace datastream
}  // namespace sf
