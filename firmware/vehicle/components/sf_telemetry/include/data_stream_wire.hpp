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
 *   - Status packet 0x4F (57B) at 1 Hz: battery (V+mA), state, rate-PID gains.
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
// NOTE: 0x49 is NOT free -- tools/log_analyzer/udp_capture.py already reserves
// it as PKT_ESKF_PDIAG (64B ESKF P-diagonal, unimplemented by any firmware
// today but present in its SAMPLE_INFO decode table). Using 0x49 here would
// make an old/new udp_capture.py silently mis-decode this 64B duty payload as
// P-diagonal covariance floats (both entries happen to be 64B). 0x4A is the
// next free id after the existing 0x40-0x49/0x4F block.
// 注意: 0x49 は空きではない -- udp_capture.py が PKT_ESKF_PDIAG（64B ESKF
// P対角、現行ファームは未実装だが SAMPLE_INFO デコード表には存在）として
// 既に予約済み。ここで 0x49 を使うと udp_capture.py がこの 64B duty
// ペイロードを P対角共分散として誤デコードしてしまう（両方たまたま64B）。
// 0x4A は既存の 0x40-0x49/0x4F ブロックの次に空いている id。
inline constexpr uint8_t kPktDuty400   = 0x4A;  // 400Hz motor duty (8 samples/entry)
inline constexpr uint8_t kPktCtrlOutput400 = 0x4B;  // 400Hz commanded thrust+torque,
                                                     // pre-mixer (8 samples/entry)
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
/// on hardware as "Mag: 0 samples". 1200 left ~160B of entry headroom.
/// Raised to 1300 when the 400Hz duty entry (kPktDuty400) was added: fixed
/// part 916B (header+ImuEskf+PosVel+RateRef blocks) + entry_count 1B + duty
/// entry 66B (2B [id][size] + 64B payload) = 983B, leaving ~130B for the
/// other sensor entries (control 22B, ctrl_ref 32B, flow/tof/baro/mag ~70B
/// combined) that used to fit in the old 1200B cap — and still ~170B of
/// headroom under the WiFi MTU (1472) and the PC capture buffer (2048).
/// データグラム上限。旧ファームは 1024 だったが、フロー全量配送（100Hz で 1 パケット
/// 2 エントリ）で典型ペイロードが ~1040B となり、最後に積む mag（18B）が毎パケット
/// 黙って弾かれた — 実機計測で「Mag: 0 サンプル」。1200 でエントリ余白 ~160B。
/// 400Hz duty エントリ（kPktDuty400）追加に伴い 1300 へ引き上げ: 固定部 916B
/// （ヘッダ+ImuEskf+PosVel+RateRefブロック）+ entry_count 1B + duty エントリ
/// 66B（[id][size] 2B + payload 64B）= 983B、旧 1200B 上限に収まっていた
/// 他のセンサエントリ（control 22B、ctrl_ref 32B、flow/tof/baro/mag 計 ~70B）用に
/// ~130B、さらに WiFi MTU（1472）・PC 受信バッファ（2048）に対して ~170B の
/// 余白を残す。
/// 400Hz control_output エントリ（kPktCtrlOutput400）追加に伴い 1400 へ再度
/// 引き上げ: 前回の典型合計 1107B + 新エントリ 130B（[id][size] 2B + payload
/// 128B = float4 × 8サンプル）= 1237B、旧 1300B 上限に対して ~63B しか余白が
/// 無かったため、WiFi MTU（1472）・PC 受信バッファ（2048）に対する余白を
/// ~163B に戻す。
inline constexpr size_t kUnifiedMaxSize   = 1400;

// Quantization scales (PC side divides by these to restore physical units)
// 量子化スケール（PC 側はこれで割って物理量に復元する）
inline constexpr float kBiasScale    = 10000.0f;  // int16 = value × 10000
inline constexpr float kRateRefScale = 1000.0f;   // int16 = value × 1000
inline constexpr float kAngleRefScale = 10000.0f; // int16 = value × 10000
inline constexpr float kDutyScale     = 65535.0f; // uint16 = duty(0..1) × 65535

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

/// 400Hz motor duty sample — the actual PLANT INPUT for rate-loop system
/// identification (`sf sysid fit`). One kPktDuty400 ENTRY carries
/// kSamplesPerPacket (8) of these (64B total payload), paired by INDEX with
/// the same-cycle ImuEskf/RateRef samples above — exactly like WireRateRef,
/// but appended via UnifiedPacketBuilder::addEntry() (variable, [id][size]
/// framed) rather than written unconditionally by begin(), because unlike
/// rate_ref this entry did not exist on older firmware. Recording the real
/// duty means `sf sysid fit` no longer has to reconstruct the plant input as
/// Kp*(rate_ref-gyro) from an assumed/typed-in Kp — it reads what the mixer
/// actually sent, correct even if Kp was mistyped, changed mid-flight
/// (autotune/gain-schedule), or the duty saturated. A parser that does not
/// know entry id 0x4A simply skips it via the [id][size] framing (see
/// udp_capture.py parse_packet()) and falls back to the Kp reconstruction.
/// 400Hz モータduty サンプル — レートループ同定（`sf sysid fit`）の「実際の
/// プラント入力」。kPktDuty400 の1エントリに kSamplesPerPacket（8）個分
/// （payload計64B）を積み、上の ImuEskf/RateRef と同じ index で対応させる
/// （WireRateRef と同じ考え方）。ただし rate_ref と違い旧ファームには存在し
/// なかったため、begin() が無条件で書く固定ブロックではなく
/// UnifiedPacketBuilder::addEntry()（[id][size] 可変枠）で追加する。実際の
/// duty を記録すれば、`sf sysid fit` は Kp*(rate_ref−gyro) という「仮定した
/// Kp からの再構成」に頼らずに済む — Kp の入力ミス・飛行中のゲイン変更
/// （自動チューニング・ゲインスケジューリング）・duty の飽和があっても
/// 正しい。0x4A を判別できないパーサは [id][size] 枠組みで単純にスキップし
/// （udp_capture.py の parse_packet() 参照）、Kp再構成にフォールバックする。
struct WireDuty400 {
    uint16_t duty[4];   // FR, RR, RL, FL — value × kDutyScale (duty 0..1)
};
static_assert(sizeof(WireDuty400) == 8, "wire drift");

/// 400Hz commanded thrust+torque sample — the PRE-MIXER control command
/// (`control_output` topic: `ControlOutput.thrust`/`.torque`), i.e. what the
/// controller asked for before the mixer (legacy linear on firmware/workshop,
/// physical B^-1 + nonlinear motor curve on firmware/vehicle) turned it into
/// per-motor duty. One kPktCtrlOutput400 ENTRY carries kSamplesPerPacket (8)
/// of these (128B total payload), paired by INDEX with the same-cycle
/// ImuEskf/RateRef/Duty400 samples — same convention as WireDuty400.
/// Reading this directly lets `sf sysid fit`/`rate-fit` identify G_p(s)
/// without knowing which mixer flew (no --mixer selection, no nonlinear
/// duty->thrust inversion) -- see docs/events/sci_tutorial_2026 rate-sysid
/// design memo, 2026-09-09. Comparing this against the duty-reconstructed
/// actual torque (WireDuty400 + the real motor curve) also gives the mixer's
/// static gain error `c` as a diagnostic, entirely from logged data. Floats,
/// not quantized (unlike duty's fixed [0,1] range, thrust/torque have no
/// natural fixed scale to quantize against without risking silent clipping).
/// 400Hz 指令推力＋トルクサンプル — ミキサー手前の制御指令（`control_output`
/// トピック: `ControlOutput.thrust`/`.torque`）。コントローラがミキサー
/// （firmware/workshop の単純線形、firmware/vehicle の物理B^-1＋非線形モータ
/// 曲線）に渡す前に「これだけ出してほしい」と要求した値そのもの。
/// kPktCtrlOutput400 の1エントリに kSamplesPerPacket（8）個分（payload計
/// 128B）を積み、上の ImuEskf/RateRef/Duty400 と同じ index で対応させる
/// （WireDuty400 と同じ考え方）。これを直接読めば `sf sysid fit`/`rate-fit`
/// はどのミキサーで飛んだかを判別せずに（--mixer選択も非線形duty->thrust逆算も
/// 不要に）G_p(s) を同定できる — 2026-09-09 のレート同定設計メモ参照。
/// duty から逆算した実トルク（WireDuty400 ＋ 実モータ曲線）と突き合わせれば、
/// ログだけからミキサーの静的ゲイン誤差 `c` を診断値として求められる。duty の
/// ような固定 [0,1] レンジが無く量子化すると黙ってクリップする恐れがあるため
/// float のまま積む（量子化しない）。
struct WireControlOutput400 {
    float thrust;      // [N] commanded total thrust
    float torque[3];   // [Nm] commanded body torque R, P, Y
};
static_assert(sizeof(WireControlOutput400) == 16, "wire drift");

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

/// Standalone 1Hz status packet — 57B total (header 4B + payload 52B + checksum 1B).
/// The 9 rate-PID gains let the PC-side tools reconstruct motor commands.
/// current_ma was appended AFTER pid_gains (not inserted) so udp_capture.py can
/// keep decoding older firmware images by TOTAL packet length alone: v1=17B
/// (12B payload, no gains, no current), v2=53B (48B payload, +9 gains), v3=57B
/// (52B payload, +current_ma) — see udp_capture.py's PKT_STATUS length dispatch.
/// 単独 1Hz ステータスパケット（計 57B = header 4B + payload 52B + checksum 1B）。
/// レート PID ゲイン 9 個は PC 側ツールのモータ指令再構成用。current_ma は pid_gains
/// の後ろに追記した（挿入ではない）ので、udp_capture.py はパケット全長だけで旧ファーム
/// も読み続けられる: v1=17B（payload12B, ゲイン無し・電流無し）、v2=53B（payload48B,
/// +ゲイン9個）、v3=57B（payload52B, +current_ma）— udp_capture.py の PKT_STATUS
/// 長分岐を参照。
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
    float    current_ma;       // [mA] battery current (PowerData.current) — CW/CCW motor
                               // asymmetry diagnostics + in-flight current monitoring.
                               // Appended last for wire compatibility (see struct comment).
                               // バッテリ電流。CW/CCW モータ非対称診断＋飛行中電流監視用。
                               // 電文互換のため末尾に追記（struct コメント参照）。
};
static_assert(sizeof(WireStatusPayload) == 52, "wire drift");

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

/// Quantize a 0..1 duty to uint16: round to nearest, clamp to [0, kDutyScale].
/// A separate function from quantize() (signed, symmetric saturation for
/// biases/rates) because duty is never negative — a shared signed helper
/// would carry a low-clamp branch this caller can never hit.
/// duty(0..1) を uint16 へ量子化: 四捨五入、[0, kDutyScale] にクランプ。
/// duty は負にならないため quantize()（バイアス/レート用の符号付き対称
/// 飽和）とは別関数にした — 共用すると絶対に作動しない下側クランプ分岐を
/// 抱えることになる。
inline uint16_t quantizeDuty(float duty)
{
    const float scaled = duty * kDutyScale;
    if (scaled <= 0.0f) return 0;
    if (scaled >= kDutyScale) return static_cast<uint16_t>(kDutyScale);
    return static_cast<uint16_t>(scaled + 0.5f);
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

/// Build the standalone 1Hz status packet into `buffer` (≥ 57B); returns length.
/// 単独 1Hz ステータスパケットを buffer（57B 以上）へ構築し、長さを返す。
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
