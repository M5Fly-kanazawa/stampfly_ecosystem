/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file snn_controller.hpp
 * @brief Spiking-neural-network attitude controller — Stage 1 (wiring-only,
 *        UNTRAINED) IController implementation, port of Stroobants et al.
 *        2025 "Neuromorphic Attitude Estimation and Control" (arXiv:2411.13945).
 *        スパイキングニューラルネット姿勢制御器 — Stroobants et al. 2025
 *        「Neuromorphic Attitude Estimation and Control」(arXiv:2411.13945)
 *        の移植、Stage1（配線のみ、未学習）IController 実装。
 *
 * Network: one CubaLifLayer(10 -> 16, non-recurrent) + one
 * LinearReadout(16 -> 4) producing [thrust, torque_roll, torque_pitch,
 * torque_yaw] each cycle. Input vector (10): body rate p,q,r
 * (state.angular_rate), estimated roll/pitch/yaw (from state.attitude), and
 * the pilot setpoint roll/pitch/yaw/throttle — i.e. everything a cascade PID
 * would use, handed to the network raw instead of through hand-designed
 * error terms (the network is meant to learn the mapping itself in Stage 3).
 * ネットワーク: CubaLifLayer(10→16, 再帰なし) 1つ + LinearReadout(16→4) 1つで
 * 毎サイクル [推力, ロールトルク, ピッチトルク, ヨートルク] を出力する。
 * 入力ベクトル(10次元): 機体角速度 p,q,r（state.angular_rate）、推定
 * roll/pitch/yaw（state.attitude から）、パイロット目標 roll/pitch/yaw/
 * throttle — カスケードPIDが使う情報を、手作りの誤差項を介さず生のまま
 * ネットワークへ渡す（写像自体をStage3でネットワークに学習させる想定）。
 *
 * ALL WEIGHTS ARE AN UNTRAINED, FIXED-SEED PLACEHOLDER — do not expect
 * meaningful attitude control; a crash/fall in SILS is the EXPECTED outcome
 * (task spec). The readout->physical-units mapping (thrust/torque scale +
 * a hover-thrust baseline) is a hand-picked placeholder ONLY so an untrained
 * network still drives the mixer with a plausible signal instead of ~0 N;
 * see the .cpp for the exact constants and the task report for why.
 * 全重みは未学習の固定シードプレースホルダー — 意味のある姿勢制御は期待
 * できない。SILSでの落下/クラッシュは想定内（タスク仕様）。読み出し→物理単位
 * の写像（推力/トルクスケール＋ホバー推力ベースライン）は、未学習ネットワーク
 * でもミキサーに約0Nでない妥当な信号を渡すためだけの手作りプレースホルダー
 * — 定数の詳細は .cpp、選定理由はタスク報告を参照。
 *
 * @design controller.hpp — IController contract implemented              [OK]
 * @design arXiv:2411.13945 — attitude control network (UNTRAINED)        [--]
 * @design coding_and_education.md §2 — bilingual comments                [OK]
 */

#pragma once

#include "cuba_lif_layer.hpp"
#include "controller.hpp"
#include "linear_readout.hpp"

namespace sf {

/// SNN attitude controller (a swappable IController).
/// SNN姿勢制御器（差し替え可能な IController）。
class SnnController : public IController {
public:
    /// Generate the placeholder weights and reset state. Call ONCE before
    /// first use (mirrors PidController::init()).
    /// プレースホルダー重みを生成し状態をリセットする。初回使用前に1回呼ぶ
    /// （PidController::init() と同じ流儀）。
    void init();

    ControlOutput compute(
        const StateEstimate& state,
        const CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;

    /// Stage 1 placeholder: the network does not reconfigure per flight mode
    /// yet (the paired SnnEstimator is ACRO-scope only; see its header) —
    /// kept as a hook for Stage 2/3.
    /// Stage1プレースホルダー: ネットワークはまだフライトモード別の再構成を
    /// 行わない（対になる SnnEstimator が ACRO スコープのみのため、ヘッダ
    /// 参照）— Stage2/3向けのフックとして残す。
    void onModeChange(FlightMode /*new_mode*/) override {}

private:
    static constexpr int kInputDim  = 10;  // p,q,r + roll,pitch,yaw + sp.roll/pitch/yaw/throttle
    static constexpr int kHiddenDim = 16;  // task spec: "controller hidden ~16"
    static constexpr int kOutputDim = 4;   // thrust, torque_roll, torque_pitch, torque_yaw

    snn::CubaLifLayer  hidden_{kInputDim, kHiddenDim, /*recurrent=*/false};
    snn::LinearReadout readout_{kHiddenDim, kOutputDim};
};

}  // namespace sf
