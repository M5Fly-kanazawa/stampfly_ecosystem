/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file battery_model.hpp
 * @brief Synthetic battery voltage model for SIL failsafe testing
 *        SIL フェイルセーフ検証用の合成バッテリー電圧モデル
 *
 * The real vehicle reads voltage from INA3221; on the host there is no sensor,
 * so the integrated SIL feeds this synthetic voltage into the sensor_power
 * topic. A simple linear sag under load is enough to exercise the failsafe
 * battery thresholds (3.3V warning / 3.0V critical).
 *
 * 実機は INA3221 から電圧を読むが、ホストにはセンサがないため統合SILは
 * この合成電圧を sensor_power トピックに流す。負荷時の線形低下だけで
 * フェイルセーフの電圧閾値（3.3V警告 / 3.0V危険）を検証できる。
 */

#pragma once

namespace sf {
namespace sil {

/// Synthetic battery: linear voltage sag while armed
/// 合成バッテリー: ARM中は電圧が線形低下
class BatteryModel {
public:
    /// Set initial voltage and drain rate [V/s] (applied only while armed)
    /// 初期電圧と低下率[V/s]を設定する（ARM中のみ適用）
    void reset(float initial_v, float drain_v_per_s)
    {
        voltage_ = initial_v;
        drain_   = drain_v_per_s;
    }

    /// Advance the model one step
    /// モデルを1ステップ進める
    void step(float dt, bool armed)
    {
        if (armed) {
            voltage_ -= drain_ * dt;
            if (voltage_ < 0.0f) voltage_ = 0.0f;
        }
    }

    /// Current battery voltage [V]
    /// 現在のバッテリー電圧 [V]
    float voltage() const { return voltage_; }

private:
    float voltage_ = 4.0f;  // Nominal LiPo cell voltage / 公称LiPoセル電圧
    float drain_   = 0.0f;  // Drain rate [V/s]           / 低下率
};

}  // namespace sil
}  // namespace sf
