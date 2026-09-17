/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file linear_readout.hpp
 * @brief Linear (no activation) readout layer: y(t) = W_o * s(t) — reads out
 *        a CubaLifLayer's spikes into continuous outputs.
 *        線形（活性化関数なし）読み出し層: y(t) = W_o * s(t) — CubaLifLayer の
 *        スパイクを連続値出力へ変換する。
 *
 * Design point 2 (task spec): the network's output stage is a plain
 * weighted sum of the last hidden layer's spikes, no nonlinearity. Shared by
 * sf_estimator_snn (attitude readout) and sf_controller_snn (thrust/torque
 * readout) so the "spikes -> continuous value" step is implemented once.
 * 設計方針2（タスク仕様）: ネットワークの出力段はスパイクの重み付き和のみで
 * 活性化関数を持たない。sf_estimator_snn（姿勢読み出し）と sf_controller_snn
 * （推力/トルク読み出し）が共有し、「スパイク→連続値」変換を一箇所に実装する。
 *
 * @design arXiv:2411.13945 — linear readout layer (UNTRAINED weights)   [--]
 * @design coding_and_education.md §2 — bilingual comments, no dynamic alloc [OK]
 */

#pragma once

namespace sf {
namespace snn {

/// Linear readout: y = W_o * x, no bias, no activation.
/// 線形読み出し: y = W_o * x、バイアス無し、活性化関数無し。
class LinearReadout {
public:
    static constexpr int kMaxInputs  = 48;  ///< matches CubaLifLayer::kMaxNeurons
    static constexpr int kMaxOutputs = 8;

    /// @param n_in   Input dimension (<= kMaxInputs), typically the source
    ///                layer's neuron count / 入力次元（通常は元の層のニューロン数）
    /// @param n_out  Output dimension (<= kMaxOutputs) / 出力次元
    LinearReadout(int n_in, int n_out);

    /// Set weights, n_out*n_in entries, row-major (output, input).
    /// 重みを設定する。n_out*n_in 要素（行優先: 出力×入力）。
    void setWeights(const float* w);

    /// Compute y = W_o * input. Returns a pointer to the internal output
    /// buffer (n_out() entries), valid until the next compute() call.
    /// y = W_o * input を計算する。内部出力バッファ（n_out() 個）への
    /// ポインタを返す。次の compute() 呼び出しまで有効。
    const float* compute(const float* input);

    int n_in() const { return n_in_; }
    int n_out() const { return n_out_; }

private:
    float dot(int out_idx, const float* input) const;

    int n_in_;
    int n_out_;
    float w_[kMaxOutputs * kMaxInputs] = {};
    float out_[kMaxOutputs] = {};
};

}  // namespace snn
}  // namespace sf
