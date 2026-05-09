/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file command.hpp
 * @brief Command processor — input normalization, deadband, arbitration
 *        コマンド処理 — 入力正規化、デッドバンド、調停
 *
 * Processes raw pilot inputs from ESP-NOW or API, applies deadband
 * and normalization, then publishes a clean CommandSetpoint to the
 * command_setpoint topic.
 *
 * ESP-NOWまたはAPIからの生パイロット入力を処理し、デッドバンドと
 * 正規化を適用して、クリーンなCommandSetpointをcommand_setpointトピックに発行する。
 *
 * @design architecture.md §6 — Command processing pipeline             [--]
 * @design detailed_design.md §4 — Input normalization                   [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
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

/// Command processor: normalize, deadband, arbitrate, publish
/// コマンドプロセッサ: 正規化、デッドバンド、調停、発行
class CommandProcessor {
public:
    /// Initialize command processor with default deadband
    /// デフォルトデッドバンドでコマンドプロセッサを初期化する
    void init();

    /// Process raw input and publish setpoint to topic
    /// 生入力を処理しセットポイントをトピックに発行する
    ///
    /// @param raw_throttle  Raw throttle [0..4095]    / 生スロットル
    /// @param raw_roll      Raw roll     [0..4095]    / 生ロール
    /// @param raw_pitch     Raw pitch    [0..4095]    / 生ピッチ
    /// @param raw_yaw       Raw yaw      [0..4095]    / 生ヨー
    /// @param source        Input source               / 入力ソース
    void processRawInput(uint16_t raw_throttle, uint16_t raw_roll,
                         uint16_t raw_pitch, uint16_t raw_yaw,
                         CommandSource source);

    /// Set deadband width (applied to roll, pitch, yaw)
    /// デッドバンド幅を設定する（ロール、ピッチ、ヨーに適用）
    void setDeadband(float deadband) { deadband_ = deadband; }

private:
    /// Normalize raw ADC value [0..4095] to [-1..1]
    /// 生ADC値 [0..4095] を [-1..1] に正規化する
    float normalize(uint16_t raw) const;

    /// Normalize throttle [0..4095] to [0..1]
    /// スロットル [0..4095] を [0..1] に正規化する
    float normalizeThrottle(uint16_t raw) const;

    /// Apply deadband: values within +-deadband become zero
    /// デッドバンド適用: +-deadband内の値をゼロにする
    float applyDeadband(float value) const;

    /// Publish processed setpoint to command_setpoint topic
    /// 処理済みセットポイントをcommand_setpointトピックに発行する
    void publishSetpoint(float throttle, float roll, float pitch,
                         float yaw, CommandSource source);

    float deadband_ = 0.05f;   // Default deadband width / デフォルトデッドバンド幅
};

}  // namespace sf
