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
 *   - Status packet 0x4F at 1 Hz: battery (V+mA), state, rate-PID gains (57B, v3), and since v4
 *     (224B) the ToF diagnostics, the loop-timing counters of both 400 Hz tasks, the non-finite
 *     counters and the loss counters of the log paths.
 *   - Text-log packet 0x51 (at most 1029B, at most every 100 ms and only while text lines are waiting):
 *     the ESP_LOG lines of the vehicle with their vehicle time, in order.
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
#include "log_sink.hpp"     // log_sink::kMaxLineBytes (line limit; the header itself has no FreeRTOS) / 1 行の上限（このヘッダ自体は FreeRTOS を使わない）

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
inline constexpr uint8_t kPktTofFront  = 0x47;  // forward ToF, same layout as WireTof (optional sensor) / 前方 ToF。WireTof と同じ並び（任意のセンサ）
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
inline constexpr uint8_t kPktFlightPhase400 = 0x4C;  // 400Hz flight state + flip phase/result/angle
                                                      // (8 samples/entry)
inline constexpr uint8_t kPktFlightFlags    = 0x4D;  // 50Hz pilot buttons + pre-arm/flip readiness
                                                      // flags and block reasons
inline constexpr uint8_t kPktToFSilence      = 0x4E;  // 50Hz downward-ToF silence facts: vertical velocity
                                                      // estimate, rise, ToF-less descent thrust ratio,
                                                      // failsafe verdict, ToF health, mode block
                                                      // 50Hz 下向き ToF 出力停止の事実: 鉛直速度の推定、
                                                      // 上昇量、ToF なし降下の推力比、フェイルセーフの判定、
                                                      // ToF の健全状態、モード拒否
inline constexpr uint8_t kPktStatus    = 0x4F;  // standalone 1Hz packet
inline constexpr uint8_t kPktUnified   = 0x50;  // 50Hz batched packet
inline constexpr uint8_t kPktTextLog   = 0x51;  // ESP_LOG text lines (separate packet, sent only while lines wait)
                                                // ESP_LOG の文字列の行（別パケット。行が待っている間だけ送る）
inline constexpr uint8_t kPktBaroHeight = 0x52; // 50Hz barometric height above the floor (unified-packet entry)
                                                // 50Hz 床からの気圧高度（統合パケットのエントリ）

/// 8 control cycles (400 Hz) per unified packet → 50 sendto()/s, ≈48 KB/s.
/// The packet rate, not the sample rate, is what the WiFi stack pays for —
/// this is the bandwidth design proven on firmware/vehicle.
/// 統合パケット1個 = 制御8周期（400Hz）→ 50 sendto()/s ≈ 48KB/s。WiFi スタックの
/// コストはサンプルレートでなく「パケットレート」— firmware/vehicle で実証済みの
/// 帯域設計。
inline constexpr int    kSamplesPerPacket = 8;

/// Datagram size cap: 1400B. Delivering EVERY flow sample (2 entries/packet at
/// 100Hz) alone pushes the typical payload to ~1040B, so a 1024B cap silently
/// rejects the LAST entry (mag, 18B) on every packet ("Mag: 0 samples" on
/// hardware). The cap must cover the fixed part 916B (header+ImuEskf+PosVel+
/// RateRef blocks) + entry_count 1B + duty entry 66B (2B [id][size] + 64B
/// payload) + control_output entry 130B (2B [id][size] + 128B payload = float4
/// x 8 samples) + the other sensor entries (control 22B, ctrl_ref 32B,
/// flow/tof/baro/mag ~70B combined) = 1237B typical, and 1400B leaves ~163B of
/// headroom under the WiFi MTU (1472) and the PC capture buffer (2048).
/// The 50Hz ToF-silence entry (kPktToFSilence, 2B + 19B = 21B) brings the worst
/// case to 1317B, which still leaves 83B under this cap (test_main.cpp,
/// wire_unified_worst_case_fits builds that worst case through the real builder
/// and asserts that every entry is accepted).
/// データグラム上限は 1400B。フロー全量配送（100Hz で 1 パケット 2 エントリ）だけで
/// 典型ペイロードが ~1040B となるため、上限 1024B では最後に積む mag（18B）が毎パケット
/// 黙って弾かれる（実機で「Mag: 0 サンプル」）。上限は、固定部 916B（ヘッダ+ImuEskf+
/// PosVel+RateRef ブロック）+ entry_count 1B + duty エントリ 66B（[id][size] 2B +
/// payload 64B）+ control_output エントリ 130B（[id][size] 2B + payload 128B =
/// float4 × 8サンプル）+ 他のセンサエントリ（control 22B、ctrl_ref 32B、
/// flow/tof/baro/mag 計 ~70B）= 典型 1237B を収める必要があり、1400B なら WiFi MTU
/// （1472）・PC 受信バッファ（2048）に対して ~163B の余白が残る。
/// 50Hz の ToF 出力停止エントリ（kPktToFSilence、2B + 19B = 21B）を含めた最悪ケースは
/// 1317B、50Hz の気圧高度エントリ（kPktBaroHeight、2B + 15B = 17B）を含めると 1334B で、この上限まで 66B の余裕が残る（test_main.cpp の
/// wire_unified_worst_case_fits が、実際のビルダでこの最悪ケースを組み、全エントリが
/// 受理されることを検査する）。
inline constexpr size_t kUnifiedMaxSize   = 1400;

/// Budget of the text-log exit. Two constants bound its cost:
///   kTextLogSendPeriodCycles = 5: at most one text packet every 5th 50 Hz cycle = 10 packets/s. Basis: the
///     cost of the WiFi radio is per sendto() call, about 2.5 ms each (udp-telemetry-design.md), and the
///     unified packet already takes 50 calls/s = 12.5 % of the radio time that ESP-NOW (the pilot's control
///     link) shares. 10 more calls/s add at most 2.5 %, and none while no line is waiting (a typical flight
///     logs about 2 lines/s). A line then waits at most 100 ms, which a text log can afford.
///   kTextLogBytesPerPacket = 1024: at most this many bytes of RECORDS (an 8-byte record header plus the text)
///     per packet. Basis: it must hold the longest record (8 + the 384-byte line limit of the log sink = 392 B,
///     asserted below) twice over, and the packet (4 + 1024 + 1 = 1029 B) stays under the 1400 B datagram cap.
///     The wire cost of the text exit is therefore bounded by 1029 B x 10 packets/s = 10.3 KB/s (about a fifth
///     of the numeric stream) even in a log storm, and the 4 KB WiFi ring holds four packets' worth (400 ms) of it.
///     An ARM-transition burst of about 600 bytes goes out in one packet.
/// Lines that do not fit wait in the WiFi ring for the next packet; what the ring cannot hold is counted as
/// dropped (never silent).
/// テキストログの出口の予算。2 つの定数が費用を抑える:
///   kTextLogSendPeriodCycles = 5: テキストパケットは 50 Hz の周期 5 回につき最大 1 個 = 毎秒 10 個まで。根拠: WiFi の
///     電波の費用は sendto() の呼び出し 1 回ごとにかかり、約 2.5 ms（udp-telemetry-design.md）。統合パケットは既に毎秒 50 回で、
///     ESP-NOW（操縦者の制御リンク）と共有する電波時間の 12.5 % を使う。追加の毎秒 10 回は最大 2.5 %、待つ行が無ければ 0
///     （普通の飛行のログは毎秒 2 行ほど）。行は最大 100 ms 待つが、文字列のログには許せる遅れ。
///   kTextLogBytesPerPacket = 1024: 1 パケットのレコード（8 バイトのレコードヘッダ＋本文）はこのバイト数まで。根拠: 最長の
///     レコード（8＋シンクの 1 行の上限 384 バイト＝392 B。下で static_assert）を 2 つ収められ、パケット（4＋1024＋1＝1029 B）が
///     データグラムの上限 1400 B の内に収まる。テキスト側の WiFi の負担はログの嵐でも 1029 B × 毎秒 10 パケット = 10.3 KB/s
///     （数値ストリームの約 5 分の 1）で抑えられ、4 KB の WiFi 用リングはその 4 パケット分（400 ms）を抱える。ARM の遷移の
///     バースト（約 600 バイト）は 1 パケットで出る。
/// 収まらない行は次のパケットまで WiFi 用リングで待つ。リングが抱えきれない分は欠落として数える（黙って落とさない）。
inline constexpr size_t kTextLogBytesPerPacket   = 1024;
inline constexpr int    kTextLogSendPeriodCycles = 5;

/// Size of the record header in front of each text line (WireTextRecord). / 各テキストの行の前のレコードヘッダの大きさ。
inline constexpr size_t kTextRecordHeaderBytes = 8;

/// Largest text-log datagram: header 4 B + records + checksum 1 B. / テキストログのデータグラムの最大: ヘッダ 4 B＋レコード＋チェックサム 1 B。
inline constexpr size_t kTextLogPacketMaxSize = 4 + kTextLogBytesPerPacket + 1;

/// WireTextRecord::flags bit: the text was cut to the sink's line limit (it ends with a newline all the same).
/// WireTextRecord::flags のビット: 本文がシンクの 1 行の上限で切り詰められた（それでも改行で終わる）。
inline constexpr uint8_t kTextFlagTruncated = 0x01;

static_assert(2 * (kTextRecordHeaderBytes + log_sink::kMaxLineBytes) <= kTextLogBytesPerPacket,
              "two longest text records must fit in one packet's budget / 最長のレコードが 2 つ 1 パケットの予算に収まること");
static_assert(kTextLogPacketMaxSize <= kUnifiedMaxSize,
              "a text packet never exceeds the datagram cap of the unified packet / テキストパケットは統合パケットの上限を超えない");

// Quantization scales (PC side divides by these to restore physical units)
// 量子化スケール（PC 側はこれで割って物理量に復元する）
inline constexpr float kBiasScale    = 10000.0f;  // int16 = value × 10000
inline constexpr float kRateRefScale = 1000.0f;   // int16 = value × 1000
inline constexpr float kAngleRefScale = 10000.0f; // int16 = value × 10000
inline constexpr float kDutyScale     = 65535.0f; // uint16 = duty(0..1) × 65535
inline constexpr float kFlipPhiScale  = 1000.0f;  // int16 = flip rotation angle [rad] × 1000

// WireFlightFlags::pilot_buttons bits (transmitter flags byte, mirrors PilotRequest)
// WireFlightFlags::pilot_buttons のビット（送信機 flags バイト、PilotRequest を写す）
inline constexpr uint8_t kPilotButtonArm  = 0x01;   // ARM switch   / ARM スイッチ
inline constexpr uint8_t kPilotButtonFlip = 0x02;   // FLIP button  / FLIP ボタン

// WireFlightFlags::status_flags bits (controller + system status facts)
// WireFlightFlags::status_flags のビット（制御器・システム状態の事実）
inline constexpr uint8_t kFlagFlipReady          = 0x01;   // ControllerStatus.flip_ready
inline constexpr uint8_t kFlagAttitudeMismatch   = 0x02;   // SystemStatus.attitude_mismatch
inline constexpr uint8_t kFlagAttitudeVerified   = 0x04;   // SystemStatus.attitude_verified

// WireToFSilence::tof_flags bits (failsafe verdict + ToF health facts)
// WireToFSilence::tof_flags のビット（フェイルセーフの判定と ToF 健全状態の事実）
inline constexpr uint8_t kToFFlagSilent           = 0x01;   // FailsafeStatus.tof_silent (a silence episode is in progress) / 出力停止のエピソード中
inline constexpr uint8_t kToFFlagAltitudeReliable = 0x02;   // ToFHealth.altitude_reliable / 推定器が ToF を取り込み直し済み
inline constexpr uint8_t kToFFlagInvalidStuck     = 0x04;   // ToFHealth.invalid_stuck / 無効サンプルだけが続いている
inline constexpr uint8_t kToFFlagStatusStale      = 0x08;   // failsafe_status itself is no longer refreshed (display test) / failsafe_status 自体が更新されていない（表示用の判定）

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
/// rate_ref this entry is not present in older firmware images. Recording the real
/// duty means `sf sysid fit` no longer has to reconstruct the plant input as
/// Kp*(rate_ref-gyro) from an assumed/typed-in Kp — it reads what the mixer
/// actually sent, correct even if Kp was mistyped, changed mid-flight
/// (autotune/gain-schedule), or the duty saturated. A parser that does not
/// know entry id 0x4A simply skips it via the [id][size] framing (see
/// udp_capture.py parse_packet()) and falls back to the Kp reconstruction.
/// 400Hz モータduty サンプル — レートループ同定（`sf sysid fit`）の「実際の
/// プラント入力」。kPktDuty400 の1エントリに kSamplesPerPacket（8）個分
/// （payload計64B）を積み、上の ImuEskf/RateRef と同じ index で対応させる
/// （WireRateRef と同じ考え方）。ただし rate_ref と違い旧版ファームのイメージには存在し
/// ないため、begin() が無条件で書く固定ブロックではなく
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
/// controller asked for before the mixer (simple linear on firmware/workshop,
/// physical B^-1 + nonlinear motor curve on firmware/vehicle) turned it into
/// per-motor duty. One kPktCtrlOutput400 ENTRY carries kSamplesPerPacket (8)
/// of these (128B total payload), paired by INDEX with the same-cycle
/// ImuEskf/RateRef/Duty400 samples — same convention as WireDuty400.
/// Reading this directly lets `sf sysid fit`/`rate-fit` identify G_p(s)
/// without knowing which mixer flew (no --mixer selection, no nonlinear
/// duty->thrust inversion) -- see docs/events/sci_tutorial_2026 rate-sysid
/// design memo. Comparing this against the duty-reconstructed
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
/// 不要に）G_p(s) を同定できる — レート同定設計メモ参照。
/// duty から逆算した実トルク（WireDuty400 ＋ 実モータ曲線）と突き合わせれば、
/// ログだけからミキサーの静的ゲイン誤差 `c` を診断値として求められる。duty の
/// ような固定 [0,1] レンジが無く量子化すると黙ってクリップする恐れがあるため
/// float のまま積む（量子化しない）。
struct WireControlOutput400 {
    float thrust;      // [N] commanded total thrust
    float torque[3];   // [Nm] commanded body torque R, P, Y
};
static_assert(sizeof(WireControlOutput400) == 16, "wire drift");

/// 400Hz flight-state / flip-sequencer sample — lets a flip be reconstructed
/// phase by phase at full rate (status.csv carries flight_state at only 1 Hz,
/// so state transitions were known only to +-1 s). One kPktFlightPhase400 ENTRY
/// carries kSamplesPerPacket (8) of these (40B total payload), paired by INDEX
/// with the same-cycle ImuEskf samples — same convention as WireDuty400. A
/// parser that does not know entry id 0x4C skips it via the [id][size] framing.
/// 400Hz 飛行状態/フリップ列生成器サンプル — フリップをフェーズ単位で全レートに
/// 再構成できるようにする（status.csv の flight_state は 1Hz のみで、状態遷移は
/// ±1 s の精度しか分からなかった）。kPktFlightPhase400 の1エントリに
/// kSamplesPerPacket（8）個分（payload計40B）を積み、同周期の ImuEskf と index で
/// 対応させる（WireDuty400 と同じ考え方）。0x4C を判別できないパーサは
/// [id][size] 枠組みで単純にスキップする。
struct WireFlightPhase400 {
    uint8_t flight_state;   // FlightState                  / 飛行状態
    uint8_t flip_phase;     // FlipPhase (0 = Idle)         / フリップのフェーズ
    uint8_t flip_result;    // FlipResult                   / フリップの結果
    int16_t flip_phi;       // flip angle [rad] × kFlipPhiScale / フリップ積算回転角
};
static_assert(sizeof(WireFlightPhase400) == 5, "wire drift");

/// 50Hz flags entry — pilot buttons, controller flip readiness, pre-arm block
/// reason and attitude-check verdicts. These are slow facts (a button press
/// lasts >= 20 ms), so one record per unified packet (50 Hz, the pilot-record
/// rate) is enough. Entry id 0x4D; an unknown-id parser skips it by size.
/// 50Hz フラグエントリ — 操縦ボタン、制御器のフリップ準備状態、ARM 前の阻害理由、
/// 姿勢チェックの判定。遅い事実（ボタン押下は 20 ms 以上続く）なので統合パケット
/// あたり 1 件（50Hz、pilot レコードと同じレート）で足りる。エントリ id 0x4D。
/// id を判別できないパーサは size でスキップする。
struct WireFlightFlags {
    uint32_t timestamp_us;
    uint8_t  pilot_buttons;       // kPilotButton* bits
    uint8_t  status_flags;        // kFlag* bits
    uint8_t  flip_block_reason;   // FlipBlockReason (meaningful when !flip_ready)
    uint8_t  arm_block;           // ArmBlock (0 = pre-arm gates pass)
};
static_assert(sizeof(WireFlightFlags) == 8, "wire drift");

/// 50Hz downward-ToF silence entry — what the ToF-silence protection saw and did, so that a flight with a
/// failing ToF can be reconstructed: the accelerometer-based vertical velocity estimate, the estimated rise, the
/// commanded / support thrust of the descent without a ToF, the failsafe verdict and its reason, the ToF
/// health facts, and why the latest flight-mode request was refused. Floats are not quantized (a ratio, a
/// velocity and a height have no natural fixed scale). Slow facts (the verdicts change at 10 Hz), so one record
/// per unified packet is enough. Entry id 0x4E; a parser that does not know the id skips it by size.
/// controller_status is not republished while disarmed: read the three controller fields together with
/// flight_phase.flight_state.
/// 50Hz 下向き ToF 出力停止エントリ — ToF の出力停止の保護が何を見て何をしたか。ToF が壊れた飛行を再構成できる
/// ようにする: 加速度計に基づく鉛直速度の推定、上昇量の推定、ToF なし降下の指令推力 ÷ 支持推力、フェイルセーフの
/// 判定とその理由、ToF の健全状態、直近のモード要求が拒否された理由。float は量子化しない（比・速度・高さに固定の
/// 自然なスケールが無い）。遅い事実（判定は 10 Hz で変わる）なので統合パケットあたり 1 件で足りる。エントリ id 0x4E。
/// id を判別できないパーサは size でスキップする。controller_status は disarm 中は再発行されない: 制御器由来の
/// 3 つの値は flight_phase.flight_state と合わせて読むこと。
/// @design requirements.md §7 — Data Stream; protocol/spec/flight_log.yaml stream tof_silence [OK]
struct WireToFSilence {
    uint32_t timestamp_us;
    float    vertical_velocity_up;   // [m/s] accelerometer-based vertical velocity estimate, up positive / 加速度計に基づく鉛直速度の推定（上が正）
    float    vertical_rise_m;        // [m]   estimated rise since the craft last rested (0 at rest)       / 静止からの上昇量の推定
    float    descent_thrust_ratio;   // [-]   commanded / support thrust of the descent without a ToF (0 = none) / ToF なし降下の指令÷支持推力（0=なし）
    uint8_t  tof_flags;              // kToFFlag* bits / kToFFlag* のビット
    uint8_t  tof_reason;             // ToFSilenceReason (0 = no episode) / ToFSilenceReason（0 = エピソードなし）
    uint8_t  mode_block;             // ModeBlock (0 = no refused mode request pending) / ModeBlock（0 = 拒否されたモード要求なし）
};
static_assert(sizeof(WireToFSilence) == 19, "wire drift");

/// 50Hz barometric-height entry — the floor-relative height of sf::BarometricHeightEstimator, next to the raw baro entry
/// (0x45, unchanged). Observation only. Written with every sensor_baro sample (the sensor converts at about 13 Hz, so values repeat).
/// flags: kBaroHeightFlagValid / kBaroHeightFlagLearned (barometric_height_estimator.hpp). offset_age_decis: time since the
/// offset was last learned in 0.1 s units, saturating at 65534; 65535 = never learned. Entry id 0x52; a parser that does not
/// know the id skips it by size.
/// 50Hz 気圧高度エントリ — sf::BarometricHeightEstimator の床からの高さ。生の baro エントリ（0x45、変更なし）の隣に置く。
/// 観測のみ。sensor_baro の各サンプルごとに書く（センサの変換は約 13 Hz なので値は繰り返す）。
/// flags: kBaroHeightFlagValid / kBaroHeightFlagLearned（barometric_height_estimator.hpp）。offset_age_decis: オフセットを
/// 最後に学習してからの時間（0.1 s 単位、65534 で飽和）。65535 = 未学習。エントリ id 0x52。id を判別できないパーサは
/// size でスキップする。
/// @design requirements.md §7 — Data Stream; protocol/spec/flight_log.yaml stream baro_height [OK]
struct WireBaroHeight {
    uint32_t timestamp_us;        // baro sample time / 気圧計サンプルの時刻
    float    height_m;            // [m] height above the floor, offset-corrected (valid only with the valid flag) / 床からの高さ（オフセット補正済み。valid フラグがあるときのみ有効）
    float    offset_m;            // [m] learned offset / 学習したオフセット
    uint16_t offset_age_decis;    // [0.1 s] age of the offset, 65535 = never learned / オフセットの経過時間、65535 = 未学習
    uint8_t  flags;               // kBaroHeightFlag* bits / kBaroHeightFlag* のビット
};
static_assert(sizeof(WireBaroHeight) == 15, "wire drift");

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
/// current_ma follows pid_gains (it is not inserted before them) so udp_capture.py can
/// keep decoding older firmware images by TOTAL packet length alone: v1=17B
/// (12B payload, no gains, no current), v2=53B (48B payload, +9 gains), v3=57B
/// (52B payload, +current_ma) — see udp_capture.py's PKT_STATUS length dispatch.
/// 単独 1Hz ステータスパケット（計 57B = header 4B + payload 52B + checksum 1B）。
/// レート PID ゲイン 9 個は PC 側ツールのモータ指令再構成用。current_ma は pid_gains
/// の後ろに置く（前に挿入しない）ので、udp_capture.py はパケット全長だけで旧ファーム
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
                               // 5=INT_WDT, 6=TASK_WDT, 9=BROWNOUT (placed in a padding slot, so
                               // the 48B wire size is not increased). Lets a crash cause be
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


/// Text-log record header (packet 0x51; the text follows, `length` bytes, no terminator, with the
/// trailing newline of the ESP_LOG line). A packet is [WireHeader][record header + text]...[checksum];
/// WireHeader::sample_count is the number of records and WireHeader::sequence counts text packets
/// (loss detection, own counter). The PC restores the order of records from the packet sequence and the
/// order inside the packet.
/// テキストログのレコードヘッダ（パケット 0x51。この後に本文が `length` バイト続く。終端文字なし、ESP_LOG の行の
/// 末尾の改行を含む）。パケットは [WireHeader][レコードヘッダ＋本文]...[チェックサム]。WireHeader::sample_count は
/// レコード数、WireHeader::sequence はテキストパケットの通し番号（欠落検出。専用のカウンタ）。PC はパケットの
/// 通し番号とパケット内の順序でレコードの順序を復元する。
/// @design requirements.md §7 — Data Stream; protocol/spec/flight_log.yaml stream text_log [OK]
struct WireTextRecord {
    uint32_t timestamp_us;      // [us] vehicle esp_timer time (low 32 bits) when the line entered the sink / 行がシンクに入った時刻（esp_timer の下位 32 ビット）
    uint8_t  flags;             // kTextFlag* bits / kTextFlag* のビット
    uint8_t  dropped_before;    // lines the sink refused just before this one (saturates at 255; the exact total is in the status packet) / 直前にシンクが断った行数（255 で飽和。正確な合計はステータスパケット）
    uint16_t length;            // bytes of text that follow / 続く本文のバイト数
};
static_assert(sizeof(WireTextRecord) == kTextRecordHeaderBytes, "wire drift");

/// ToF measurement-quality diagnostics of one sensor, in the status packet v4 (the latest sample of the
/// sensor as the VL53L3CX driver reports it). All zero when the sensor never produced a sample.
/// ステータスパケット v4 の、センサ 1 個の ToF の測定品質の診断値（VL53L3CX ドライバが報告する、そのセンサの
/// 最新サンプル）。センサが一度もサンプルを出していなければすべて 0。
struct WireToFDiagnostic {
    uint8_t range_status;   // VL53LX range status of the latest sample: 0 = valid, 254 = every target invalid, 255 = no target; others are the driver's codes / 最新サンプルの VL53LX range status: 0 = 有効、254 = 全ターゲット無効、255 = ターゲットなし、その他はドライバのコード
    float   signal_rate;    // [Mcps] return signal rate (megacounts per second) / 反射信号レート
    float   ambient_rate;   // [Mcps] ambient light rate / 環境光レート
    float   sigma_mm;       // [mm] range standard deviation estimate / 距離の標準偏差の推定
};
static_assert(sizeof(WireToFDiagnostic) == 13, "wire drift");

/// Loop-timing counters of one 400 Hz task in the status packet v4 — the fields of sf::LoopTiming (see
/// data_types.hpp for their meaning) without the publish time stamp. Counts are since boot.
/// ステータスパケット v4 の、400 Hz タスク 1 本の周期の数え上げ — sf::LoopTiming の項目（意味は data_types.hpp）から
/// 発行時刻を除いたもの。数え方は起動以降の累計。
struct WireLoopTiming {
    uint32_t cycles;                    // wakes counted / 数えた起床の回数
    uint32_t period_max_us;             // [us] longest interval between two wakes / 連続する 2 回の起床の間隔の最大
    uint32_t late_count;                // intervals longer than 1.5 periods / 間隔が 1.5 周期を超えた回数
    uint32_t busy_max_us;               // [us] longest wake-to-wait time / 起床から次の待ちまでの最大
    uint32_t over_budget_count;         // cycles whose busy time exceeded one period / 処理が 1 周期を超えた周期の数
    uint32_t last_late_us;              // [us] esp_timer time of the latest late wake (0 = none) / 最後に遅れた起床の時刻（0 = なし）
    uint32_t armed_cycles;              // the same counts, armed cycles only: / 同じ数え上げ（ARM 中の周期だけ）:
    uint32_t armed_period_max_us;       // [us]
    uint32_t armed_late_count;
    uint32_t armed_busy_max_us;         // [us]
    uint32_t armed_over_budget_count;
};
static_assert(sizeof(WireLoopTiming) == 44, "wire drift");

/// Non-finite counters of the control path in the status packet v4 — the fields of sf::NonFiniteStatus (see
/// data_types.hpp) without the publish time stamp. stage_count is indexed by NonFiniteStage.
/// ステータスパケット v4 の、制御経路の有限でない数の数え上げ — sf::NonFiniteStatus の項目（data_types.hpp）から
/// 発行時刻を除いたもの。stage_count は NonFiniteStage で引く。
struct WireNonFinite {
    uint32_t checked_cycles;                         // armed control cycles checked / 検査した ARM 中の制御周期の数
    uint32_t detected_cycles;                        // cycles in which any boundary was non-finite / どれかの境界が有限でなかった周期の数
    uint32_t stage_count[kNonFiniteStageCount];      // per boundary / 境界ごとの周期数
    uint32_t first_us;                               // [us] esp_timer time of the first detection / 最初の検出の時刻
    uint8_t  first_stage;                            // NonFiniteStage of the first detection, 0xFF = none / 最初の検出の境界。0xFF = なし
};
static_assert(sizeof(WireNonFinite) == 37, "wire drift");

/// Loss counters of the log paths in the status packet v4. The first three count since the capture
/// started; log_stream_overflow_count too (it is the ring's own counter minus its value at the start).
/// ステータスパケット v4 の、ログ経路の欠落の計数。取得の開始以降の数（log_stream_overflow_count もリング自身の
/// 計数から開始時の値を引いたもの）。
struct WireLinkHealth {
    uint32_t log_stream_overflow_count;   // 400 Hz samples the log_stream ring dropped (the Data Stream did not drain it in time) / log_stream リングが落とした 400 Hz サンプル数（Data Stream が間に合って取り出せなかった）
    uint32_t send_drops;                  // sendto failures (the PC sees the gap as a sequence jump) / sendto の失敗数（PC は通し番号の飛びとして見る）
    uint32_t entry_drops;                 // entries the unified packet refused because it was full / 統合パケットが満杯で棄却したエントリ数
    uint32_t text_dropped_lines;          // text lines the WiFi ring refused (exact; the per-line count saturates at 255) / WiFi 用リングが断ったテキストの行数（正確。行ごとの数は 255 で飽和）
};
static_assert(sizeof(WireLinkHealth) == 16, "wire drift");

/// Status packet v4 extension — appended after the v3 payload (WireStatusPayload), so a reader that
/// knows only the v1 to v3 lengths still decodes the prefix. 167 B; the whole packet is 4 + 52 + 167 + 1 =
/// 224 B (kStatusV4PacketSize). The PC picks the version by the TOTAL length: 17, 53, 57, 224.
/// ステータスパケット v4 の拡張部 — v3 のペイロード（WireStatusPayload）の後ろに追記する。v1〜v3 の長さしか知らない
/// 読み手も先頭部をデコードできる。167 B。パケット全体は 4 + 52 + 167 + 1 = 224 B（kStatusV4PacketSize）。
/// PC は全長でバージョンを選ぶ: 17、53、57、224。
/// @design requirements.md §7 — Data Stream; protocol/spec/flight_log.yaml streams tof_diagnostics, loop_timing, nonfinite, link_health [OK]
struct WireStatusV4Extension {
    WireToFDiagnostic tof_bottom;       // downward ToF / 下向き ToF
    WireToFDiagnostic tof_front;        // front ToF (all zero when the sensor is absent) / 前方 ToF（センサなしならすべて 0）
    WireLoopTiming    imu_loop;         // ImuTask (imu_loop_timing)
    WireLoopTiming    control_loop;     // ControlTask (control_loop_timing; all zero in the workshop firmware) / ControlTask（control_loop_timing。講習用ファームではすべて 0）
    WireNonFinite     nonfinite;        // ControlTask's non-finite counters / ControlTask の有限でない数の数え上げ
    WireLinkHealth    link;             // loss counters of the log paths / ログ経路の欠落の計数
};
static_assert(sizeof(WireStatusV4Extension) == 167, "wire drift");

#pragma pack(pop)

/// Total sizes of the status packet by version (header 4 B + payload + checksum 1 B).
/// ステータスパケットのバージョン別の全長（ヘッダ 4 B＋ペイロード＋チェックサム 1 B）。
inline constexpr size_t kStatusV3PacketSize = 4 + sizeof(WireStatusPayload) + 1;
inline constexpr size_t kStatusV4PacketSize = kStatusV3PacketSize + sizeof(WireStatusV4Extension);
static_assert(kStatusV3PacketSize == 57, "wire drift");
static_assert(kStatusV4PacketSize == 224, "wire drift");

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
/// 飽和）とは別関数にした — 別関数なので絶対に作動しない下側クランプ分岐を
/// 持たずに済む。
inline uint16_t quantizeDuty(float duty)
{
    const float scaled = duty * kDutyScale;
    if (scaled <= 0.0f) return 0;
    if (scaled >= kDutyScale) return static_cast<uint16_t>(kDutyScale);
    return static_cast<uint16_t>(scaled + 0.5f);
}

/// Quantize a flip rotation angle [rad] to int16 (× kFlipPhiScale, saturating).
/// フリップ積算回転角 [rad] を int16 へ量子化（× kFlipPhiScale、飽和）。
inline int16_t quantizeFlipPhi(float phi_rad)
{
    return quantize(phi_rad, kFlipPhiScale);
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


/// sf::LoopTiming -> status packet v4 block (the publish time stamp is dropped).
/// sf::LoopTiming -> ステータスパケット v4 のブロック（発行時刻は落とす）。
inline WireLoopTiming toWire(const LoopTiming& timing)
{
    WireLoopTiming wire = {};
    wire.cycles                   = timing.cycles;
    wire.period_max_us            = timing.period_max_us;
    wire.late_count               = timing.late_count;
    wire.busy_max_us              = timing.busy_max_us;
    wire.over_budget_count        = timing.over_budget_count;
    wire.last_late_us             = timing.last_late_us;
    wire.armed_cycles             = timing.armed_cycles;
    wire.armed_period_max_us      = timing.armed_period_max_us;
    wire.armed_late_count         = timing.armed_late_count;
    wire.armed_busy_max_us        = timing.armed_busy_max_us;
    wire.armed_over_budget_count  = timing.armed_over_budget_count;
    return wire;
}

/// sf::NonFiniteStatus -> status packet v4 block (the publish time stamp is dropped).
/// sf::NonFiniteStatus -> ステータスパケット v4 のブロック（発行時刻は落とす）。
inline WireNonFinite toWire(const NonFiniteStatus& status)
{
    WireNonFinite wire = {};
    wire.checked_cycles  = status.checked_cycles;
    wire.detected_cycles = status.detected_cycles;
    for (size_t stage = 0; stage < kNonFiniteStageCount; ++stage) {
        wire.stage_count[stage] = status.stage_count[stage];
    }
    wire.first_us    = status.first_us;
    wire.first_stage = status.first_stage;
    return wire;
}

/// The four ToF diagnostic values of one sensor -> status packet v4 block.
/// センサ 1 個の ToF の 4 つの診断値 -> ステータスパケット v4 のブロック。
inline WireToFDiagnostic toWireToFDiagnostic(uint8_t range_status, float signal_rate, float ambient_rate,
                                             float sigma_mm)
{
    WireToFDiagnostic wire = {};
    wire.range_status = range_status;
    wire.signal_rate  = signal_rate;
    wire.ambient_rate = ambient_rate;
    wire.sigma_mm     = sigma_mm;
    return wire;
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


/// Build the status packet v4 (v3 + WireStatusV4Extension) into `buffer` (>= kStatusV4PacketSize B);
/// returns the length. The v3 prefix is byte-identical to buildStatusPacket().
/// ステータスパケット v4（v3＋WireStatusV4Extension）を buffer（kStatusV4PacketSize B 以上）へ構築し、長さを返す。
/// 先頭の v3 部分は buildStatusPacket() とバイト単位で同一。
inline size_t buildStatusPacketV4(uint8_t* buffer, uint16_t sequence, const WireStatusPayload& payload,
                                  const WireStatusV4Extension& extension)
{
    WireHeader header = {};
    header.packet_id    = kPktStatus;
    header.sequence     = sequence;
    header.sample_count = 1;
    std::memcpy(buffer, &header, sizeof(header));
    std::memcpy(buffer + sizeof(header), &payload, sizeof(payload));
    std::memcpy(buffer + sizeof(header) + sizeof(payload), &extension, sizeof(extension));
    const size_t body = sizeof(header) + sizeof(payload) + sizeof(extension);
    buffer[body] = xorChecksum(buffer, body);
    return body + 1;
}

/// Incremental builder of the text-log packet 0x51. Usage: begin(seq) → addLine()... → finish() → send
/// buffer()/length(). The records of one packet never exceed kTextLogBytesPerPacket bytes.
/// テキストログのパケット 0x51 のビルダ。begin(seq) → addLine()... → finish() の順に使う。1 パケットのレコードは
/// kTextLogBytesPerPacket バイトを超えない。
class TextLogPacketBuilder {
public:
    /// Start a packet: write the header (no record yet). / パケットを始める: ヘッダを書く（レコードはまだ無い）。
    void begin(uint16_t sequence)
    {
        WireHeader header = {};
        header.packet_id    = kPktTextLog;
        header.sequence     = sequence;
        header.sample_count = 0;
        std::memcpy(buffer_, &header, sizeof(header));
        length_      = sizeof(header);
        record_bytes_ = 0;
        record_count_ = 0;
    }

    /// Longest text the next record can still carry within the budget (0 = the packet is full).
    /// 予算の範囲で次のレコードが運べる本文の最大長（0 = パケットは満杯）。
    size_t remainingTextBytes() const
    {
        const size_t left = kTextLogBytesPerPacket - record_bytes_;
        return (left > kTextRecordHeaderBytes) ? left - kTextRecordHeaderBytes : 0;
    }

    /// Append one record. False when it does not fit in the budget (nothing is written).
    /// レコードを 1 つ追加する。予算に収まらなければ false（何も書かない）。
    bool addLine(uint32_t timestamp_us, uint8_t flags, uint8_t dropped_before, const char* text, size_t text_length)
    {
        if (text_length > remainingTextBytes()) {
            return false;
        }
        WireTextRecord record = {};
        record.timestamp_us  = timestamp_us;
        record.flags         = flags;
        record.dropped_before = dropped_before;
        record.length        = static_cast<uint16_t>(text_length);
        std::memcpy(&buffer_[length_], &record, sizeof(record));
        length_ += sizeof(record);
        std::memcpy(&buffer_[length_], text, text_length);
        length_ += text_length;
        record_bytes_ += sizeof(record) + text_length;
        ++record_count_;
        ++buffer_[3];   // WireHeader::sample_count = number of records / レコード数
        return true;
    }

    /// Number of records in the packet. / パケット内のレコード数。
    size_t recordCount() const { return record_count_; }

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
    uint8_t buffer_[kTextLogPacketMaxSize] = {};
    size_t  length_       = 0;
    size_t  record_bytes_ = 0;   // bytes of records so far (the budget counts these) / ここまでのレコードのバイト数（予算はこれを数える）
    size_t  record_count_ = 0;
};

}  // namespace datastream
}  // namespace sf
