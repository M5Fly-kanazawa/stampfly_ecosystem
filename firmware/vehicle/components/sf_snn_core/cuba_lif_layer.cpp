/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file cuba_lif_layer.cpp
 * @brief CUBA-LIF layer implementation. See the header for the equations
 *        and design notes.
 *        CUBA-LIF層の実装。数式・設計メモはヘッダ参照。
 */

#include "cuba_lif_layer.hpp"

#include <algorithm>  // std::fill
#include <cassert>
#include <cstring>    // memcpy

namespace sf {
namespace snn {

CubaLifLayer::CubaLifLayer(int n_in, int n_neurons, bool recurrent)
    : n_in_(n_in), n_neurons_(n_neurons), recurrent_(recurrent)
{
    assert(n_in > 0 && n_in <= kMaxInputs);
    assert(n_neurons > 0 && n_neurons <= kMaxNeurons);

    // Stage 1 placeholder defaults (task spec): "roughly plausible" fixed
    // values, to be replaced by learned/derived parameters in Stage 3.
    // Stage1プレースホルダー既定値（仕様）:「おおよそ妥当」な固定値。
    // Stage3で学習/導出値に置き換える。
    std::fill(tau_mem_, tau_mem_ + n_neurons_, 0.9f);
    std::fill(tau_syn_, tau_syn_ + n_neurons_, 0.8f);
    std::fill(theta_,   theta_   + n_neurons_, 1.0f);
}

void CubaLifLayer::setWeights(const float* w_in, const float* w_rec)
{
    std::memcpy(w_in_, w_in, sizeof(float) * static_cast<size_t>(n_neurons_) * n_in_);
    if (recurrent_ && w_rec != nullptr) {
        std::memcpy(w_rec_, w_rec, sizeof(float) * static_cast<size_t>(n_neurons_) * n_neurons_);
    }
}

void CubaLifLayer::setUniformParams(float tau_mem, float tau_syn, float theta)
{
    std::fill(tau_mem_, tau_mem_ + n_neurons_, tau_mem);
    std::fill(tau_syn_, tau_syn_ + n_neurons_, tau_syn);
    std::fill(theta_,   theta_   + n_neurons_, theta);
}

void CubaLifLayer::setFixedIntegrator(int index, bool fixed)
{
    is_fixed_[index] = fixed;
    if (fixed) {
        // Per design: a fixed integrator neuron has no leak (tau_mem=1.0)
        // and a unit threshold (theta=1.0); this metadata flag is what a
        // future trainer will check to skip these neurons.
        // 設計通り: 固定積分ニューロンは無漏洩(tau_mem=1.0)・単位閾値
        // (theta=1.0)。将来の学習器はこのメタデータで対象外にする。
        tau_mem_[index] = 1.0f;
        theta_[index]   = 1.0f;
    }
}

void CubaLifLayer::fireAndReset()
{
    for (int n = 0; n < n_neurons_; ++n) {
        const bool fired = v_[n] >= theta_[n];
        spikes_[n] = fired ? 1.0f : 0.0f;
        if (fired) {
            v_[n] = 0.0f;
        }
    }
}

float CubaLifLayer::feedforwardCurrent(int neuron, const float* input) const
{
    float sum = 0.0f;
    const float* row = &w_in_[neuron * n_in_];
    for (int j = 0; j < n_in_; ++j) {
        sum += row[j] * input[j];
    }
    return sum;
}

float CubaLifLayer::recurrentCurrent(int neuron) const
{
    float sum = 0.0f;
    const float* row = &w_rec_[neuron * n_neurons_];
    for (int k = 0; k < n_neurons_; ++k) {
        sum += row[k] * spikes_[k];
    }
    return sum;
}

const float* CubaLifLayer::step(const float* input)
{
    // Eq.2's spike condition reads v_i(t) BEFORE the current update below,
    // so the threshold check/reset comes first (see header comment).
    // Eq.2 の発火条件は「この更新の前」の v_i(t) を見るため、判定+リセットを
    // 先に行う（ヘッダのコメント参照）。
    fireAndReset();

    for (int n = 0; n < n_neurons_; ++n) {
        // Eq.1: v_i(t+1) = tau_mem_i * v_i(t) + i_i(t)  (v_ already carries
        // this cycle's reset; i_ is still i(t), not yet advanced below).
        v_next_[n] = tau_mem_[n] * v_[n] + i_[n];
    }

    for (int n = 0; n < n_neurons_; ++n) {
        // Eq.2: i_i(t+1) = tau_syn_i * i_i(t) + Σ_j(w_ij*s_j(t)) [+ Σ_k(...)]
        float syn = feedforwardCurrent(n, input);
        if (recurrent_) {
            syn += recurrentCurrent(n);
        }
        i_next_[n] = tau_syn_[n] * i_[n] + syn;
    }

    for (int n = 0; n < n_neurons_; ++n) {
        v_[n] = v_next_[n];
        i_[n] = i_next_[n];
    }
    return spikes_;
}

void CubaLifLayer::reset()
{
    std::fill(v_,      v_      + n_neurons_, 0.0f);
    std::fill(i_,      i_      + n_neurons_, 0.0f);
    std::fill(spikes_, spikes_ + n_neurons_, 0.0f);
}

}  // namespace snn
}  // namespace sf
