/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file linear_readout.cpp
 * @brief LinearReadout implementation. See the header for the design note.
 *        LinearReadout の実装。設計メモはヘッダ参照。
 */

#include "linear_readout.hpp"

#include <cassert>
#include <cstring>  // memcpy

namespace sf {
namespace snn {

LinearReadout::LinearReadout(int n_in, int n_out)
    : n_in_(n_in), n_out_(n_out)
{
    assert(n_in > 0 && n_in <= kMaxInputs);
    assert(n_out > 0 && n_out <= kMaxOutputs);
}

void LinearReadout::setWeights(const float* w)
{
    std::memcpy(w_, w, sizeof(float) * static_cast<size_t>(n_out_) * n_in_);
}

float LinearReadout::dot(int out_idx, const float* input) const
{
    float sum = 0.0f;
    const float* row = &w_[out_idx * n_in_];
    for (int j = 0; j < n_in_; ++j) {
        sum += row[j] * input[j];
    }
    return sum;
}

const float* LinearReadout::compute(const float* input)
{
    for (int o = 0; o < n_out_; ++o) {
        out_[o] = dot(o, input);
    }
    return out_;
}

}  // namespace snn
}  // namespace sf
