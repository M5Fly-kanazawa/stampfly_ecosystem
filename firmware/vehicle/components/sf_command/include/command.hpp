/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file command.hpp
 * @brief Command processor — input normalization, deadband, arbitration
 *        コマンド処理 — 入力正規化、デッドバンド、調停
 *
 * Responsibility #8 (Service): absorb every input source, normalize the raw
 * sticks, apply a centre deadband, decode the discrete switch bits, and publish
 * the clean CommandSetpoint + PilotRequest topics. sf_comm (HAL, #9) only
 * delivers the raw wire packet as a RawControlInput fact — this component is the
 * single place where raw ADC counts become physical commands, so the
 * normalization rule lives in exactly one location.
 *
 * 責務#8（Service）: 全入力ソースを吸収し、生スティックを正規化、中央デッドバンドを
 * 適用、離散スイッチビットをデコードして、クリーンな CommandSetpoint と PilotRequest
 * トピックを発行する。sf_comm（HAL・#9）は生の電波パケットを RawControlInput という
 * 「事実」として渡すだけ。本コンポーネントが「生 ADC 値→物理指令」変換の唯一の場所
 * なので、正規化規則は一箇所だけに存在する。
 *
 * @design architecture.md §6 — Command processing pipeline             [OK]
 * @design detailed_design.md §4 — Input normalization                   [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "data_types.hpp"

namespace sf {

/// Command input source identifiers
/// コマンド入力ソース識別子
enum class CommandSource : uint8_t {
    ESPNOW     = 0,   // Transmitter via ESP-NOW / 送信機（ESP-NOW）
    UDP_API    = 1,   // External API via UDP    / 外部API（UDP）
    ONBOARD    = 2,   // Onboard autonomous      / オンボード自律
};

/// Command processor: normalize, deadband, decode switches, publish
/// コマンドプロセッサ: 正規化、デッドバンド、スイッチデコード、発行
class CommandProcessor {
public:
    /// Initialize command processor with default deadband
    /// デフォルトデッドバンドでコマンドプロセッサを初期化する
    void init();

    /// Process one raw input fact: normalize sticks, apply deadband, decode the
    /// switch flags, and publish CommandSetpoint + PilotRequest.
    /// 1件の生入力（事実）を処理する: スティック正規化、デッドバンド適用、スイッチ
    /// flags をデコードし、CommandSetpoint と PilotRequest を発行する。
    void process(const RawControlInput& raw);

    /// Set deadband width (applied to roll, pitch, yaw — never to throttle)
    /// デッドバンド幅を設定する（ロール、ピッチ、ヨーに適用、スロットルには非適用）
    void setDeadband(float deadband) { deadband_ = deadband; }

private:
    /// Normalize a spring-centred axis [0..4095] (2048 = centre) to [-1..1]
    /// バネ中央軸 [0..4095]（2048=中央）を [-1..1] に正規化する
    float normalizeAxis(uint16_t raw) const;

    /// Normalize throttle [0..4095] (2048 = zero) to [0..1] (lower half clipped)
    /// スロットル [0..4095]（2048=ゼロ）を [0..1] に正規化する（下半分は 0 にクリップ）
    float normalizeThrottle(uint16_t raw) const;

    /// Apply deadband: values within +-deadband become zero, the remaining range
    /// is rescaled so full deflection still reaches +-1.
    /// デッドバンド適用: +-deadband 内の値を 0 にし、残りの範囲は全振りで +-1 に届くよう
    /// 再スケールする。
    float applyDeadband(float value) const;

    /// Publish the normalized stick setpoint to command_setpoint.
    /// 正規化済みスティックセットポイントを command_setpoint に発行する。
    void publishSetpoint(const RawControlInput& raw);

    /// Decode the discrete switch bits and publish them to pilot_request.
    /// 離散スイッチビットをデコードし pilot_request に発行する。
    void publishPilotRequest(const RawControlInput& raw);

    // Deadband width for roll/pitch/yaw (never applied to throttle). 0.05 = a 5% centre
    // deadzone so small stick noise near centre does not drift the craft. This was held at
    // 0 while the accel-attitude χ² latch-up made the per-axis POSITION_HOLD/STABILIZE
    // scenarios marginally stable (a non-zero deadband shrinks the command and tipped them
    // over the χ² cliff). With the latch-up root-caused and fixed (accel_att_noise 0.06→0.8,
    // chi2_latchup_finding §8) those scenarios hold with large margin, so the deadband is
    // now enabled. Settable via setDeadband().
    // ロール/ピッチ/ヨーのデッドバンド幅（スロットルには適用しない）。0.05 = 中央5%の不感帯で、
    // 中央付近の微小なスティックノイズで機体がドリフトしないようにする。accel-attitude χ²
    // ラッチアップで毎軸 POSITION_HOLD/STABILIZE が限界安定だった間は0に固定していた（非ゼロは
    // 指令を縮め χ² の崖を越えさせた）。ラッチアップを根治した今（accel_att_noise 0.06→0.8、
    // chi2_latchup_finding §8）これらは大きな余裕で保持するので、デッドバンドを有効化する。
    float deadband_ = 0.05f;
};

}  // namespace sf
