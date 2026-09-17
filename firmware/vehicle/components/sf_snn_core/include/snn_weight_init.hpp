/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file snn_weight_init.hpp
 * @brief Deterministic placeholder weight generator for the UNTRAINED SNN
 *        Stage 1 skeleton (task spec, design point 5).
 *        未学習 SNN Stage1 骨格向けの決定的なプレースホルダー重み生成
 *        （タスク仕様 設計方針5）。
 *
 * There is no trained model yet — Stage 1's only goal is to prove the
 * wiring, so the weights are small fixed-seed uniform random numbers
 * (std::mt19937, seed given by the caller). Reproducible across builds and
 * hosts: same seed -> byte-identical weights. Stage 3 replaces every call
 * site of this function with a loader for real learned weights.
 * 学習済みモデルはまだ無い — Stage1のゴールは配線の実証のみなので、重みは
 * 固定シードの一様乱数（std::mt19937、シードは呼び出し側指定）。ビルド・
 * ホストをまたいで再現可能（同シード=同一バイト列）。Stage3では本関数の
 * 呼び出し箇所を学習済み重みのロードに置き換える。
 *
 * @design task spec design point 5 — fixed-seed placeholder weights [--]
 */

#pragma once

#include <random>

namespace sf {
namespace snn {

/// Fill `out[0..count)` with i.i.d. uniform random values in [lo, hi] using
/// a fixed-seed Mersenne Twister — UNTRAINED PLACEHOLDER, not a learned
/// weight matrix.
/// `out[0..count)` を [lo, hi] の一様乱数（固定シード Mersenne Twister）で
/// 埋める — 学習済み重み行列ではない、未学習プレースホルダー。
inline void fillUniformRandom(float* out, int count, uint32_t seed,
                               float lo = -0.1f, float hi = 0.1f)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(lo, hi);
    for (int i = 0; i < count; ++i) {
        out[i] = dist(rng);
    }
}

}  // namespace snn
}  // namespace sf
