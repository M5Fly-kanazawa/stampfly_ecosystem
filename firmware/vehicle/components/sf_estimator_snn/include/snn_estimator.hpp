/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file snn_estimator.hpp
 * @brief Spiking-neural-network attitude estimator — Stage 1 (wiring-only,
 *        UNTRAINED) IEstimator implementation, port of Stroobants et al. 2025
 *        "Neuromorphic Attitude Estimation and Control" (arXiv:2411.13945).
 *        スパイキングニューラルネット姿勢推定器 — Stroobants et al. 2025
 *        「Neuromorphic Attitude Estimation and Control」(arXiv:2411.13945)
 *        の移植、Stage1（配線のみ、未学習）IEstimator 実装。
 *
 * Scope (task design point 4 — deliberately ACRO-only): estimates ONLY
 * roll/pitch/yaw attitude from the 6-axis IMU (gyro+accel). Position,
 * velocity, and sensor biases are NOT estimated and stay zero. ToF/Flow/
 * Mag/Baro observations are accepted (interface compliance) but are pure
 * no-ops — this estimator does not fuse them at all.
 * スコープ（タスク設計方針4 — 意図的にACRO限定）: 6軸IMU(gyro+accel)から
 * roll/pitch/yaw姿勢のみを推定する。位置・速度・センサバイアスは推定せず
 * ゼロのまま。ToF/Flow/Mag/Baro 観測は（インターフェース準拠のため）受け付ける
 * が完全な no-op — 本推定器はこれらを一切融合しない。
 *
 * Network: one CubaLifLayer(6 -> 32, recurrent) + one LinearReadout(32 -> 3)
 * producing [roll, pitch, yaw] each cycle, converted to a quaternion. ALL
 * WEIGHTS ARE AN UNTRAINED, FIXED-SEED PLACEHOLDER (sf::snn::fillUniformRandom) —
 * do not expect meaningful attitude tracking; Stage 1's only goal is that the
 * SILS runs this network at 400Hz without crashing (see the task report for
 * what Stage 2/3 need to do next).
 * ネットワーク: CubaLifLayer(6→32, 再帰あり) 1つ + LinearReadout(32→3) 1つで
 * 毎サイクル [roll, pitch, yaw] を出力しクォータニオンへ変換する。全重みは
 * 未学習の固定シードプレースホルダー（sf::snn::fillUniformRandom）—意味のある
 * 姿勢追従は期待できない。Stage1のゴールは SILS が本ネットワークを 400Hz で
 * クラッシュせず動かすことのみ（Stage2/3 の申し送りはタスク報告参照）。
 *
 * @design estimator.hpp — IEstimator contract implemented                [OK]
 * @design arXiv:2411.13945 — attitude estimation network (UNTRAINED)     [--]
 * @design coding_and_education.md §2 — bilingual comments                [OK]
 */

#pragma once

#include "cuba_lif_layer.hpp"
#include "estimator.hpp"
#include "linear_readout.hpp"

namespace sf {

/// SNN attitude estimator (a swappable IEstimator, ACRO-scope only).
/// SNN姿勢推定器（差し替え可能な IEstimator、ACROスコープのみ）。
class SnnEstimator : public IEstimator {
public:
    /// Generate the placeholder weights and reset state. Call ONCE before
    /// first use (mirrors ComplementaryEstimator::init()).
    /// プレースホルダー重みを生成し状態をリセットする。初回使用前に1回呼ぶ
    /// （ComplementaryEstimator::init() と同じ流儀）。
    void init();

    void predict(const ImuData& imu, float dt) override;
    void updateTof(const TofData& /*tof*/) override {}    // not fused / 未融合
    void updateFlow(const FlowData& /*flow*/) override {} // not fused / 未融合
    void updateMag(const MagData& /*mag*/) override {}    // not fused / 未融合
    void updateBaro(const BaroData& /*baro*/) override {} // not fused / 未融合
    StateEstimate getState() const override { return state_; }
    void reset() override;
    void resetPositionVelocity() override {}  // no position/velocity state (always 0)
                                              // 位置・速度状態を持たない（常に0）

private:
    static constexpr int kInputDim  = 6;   // gyro[3] + accel[3] / ジャイロ+加速度
    static constexpr int kHiddenDim = 32;  // task spec: "estimator hidden ~32"
    static constexpr int kOutputDim = 3;   // roll, pitch, yaw [rad]

    snn::CubaLifLayer  hidden_{kInputDim, kHiddenDim, /*recurrent=*/true};
    snn::LinearReadout readout_{kHiddenDim, kOutputDim};

    StateEstimate state_{};
};

}  // namespace sf
