/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file cuba_lif_layer.hpp
 * @brief CUBA-LIF (current-based leaky integrate-and-fire) spiking neuron
 *        layer — the shared math used by sf_estimator_snn and sf_controller_snn.
 *        CUBA-LIF（電流ベース漏洩積分発火）スパイキングニューロン層 —
 *        sf_estimator_snn と sf_controller_snn が共有する数学の実体。
 *
 * Ports the neuron model from Stroobants et al. 2025, "Neuromorphic Attitude
 * Estimation and Control" (arXiv:2411.13945), Eq.1-2, to a StampFly-sized
 * placeholder network (Stage 1 — wiring only, UNTRAINED weights). One
 * `CubaLifLayer` instance is a single hidden layer with optional
 * layer-internal recurrence; both the attitude estimator and the attitude
 * controller instantiate their OWN layer(s) from this shared class instead
 * of the paper's single combined estimation+control network — StampFly keeps
 * IEstimator/IController as separate interfaces, and the extra compute is
 * negligible (design decision, see the task's background).
 * Stroobants et al. 2025「Neuromorphic Attitude Estimation and Control」
 * (arXiv:2411.13945) Eq.1-2 のニューロンモデルを、StampFly サイズの
 * プレースホルダーネットワーク（Stage1 — 配線のみ、重みは未学習）へ移植する。
 * `CubaLifLayer` 1個が層内再帰結合を持てる隠れ層1つに相当し、姿勢推定器と
 * 姿勢制御器はそれぞれ「自分の」層をこのクラスから生成する — 論文の推定+制御を
 * 1つに合成したネットワークではない（StampFly は IEstimator/IController が
 * 別インターフェースのため合成しない。演算負荷増は無視できる、背景参照）。
 *
 * Neuron dynamics (Eq.1-2, per-neuron i):
 *   v_i(t)   >= theta_i  -> spike s_i(t)=1, then v_i(t) := 0 (reset)
 *   v_i(t+1)  = tau_mem_i * v_i(t) + i_i(t)
 *   i_i(t+1)  = tau_syn_i * i_i(t) + sum_j(w_ij * s_j(t))
 *                                  [+ sum_k(w_ik * s_k(t)) if recurrent]
 * ニューロン力学（Eq.1-2、ニューロン i ごと）:
 *   v_i(t) >= theta_i で発火 s_i(t)=1、その後 v_i(t) を0にリセット
 *   v_i(t+1) = tau_mem_i * v_i(t) + i_i(t)
 *   i_i(t+1) = tau_syn_i * i_i(t) + Σ_j(w_ij * s_j(t))
 *                                  [+ Σ_k(w_ik * s_k(t))（層内再帰がある場合）]
 *
 * Fixed-integrator neurons (future use): the paper freezes some neurons at
 * tau_mem=1.0 / theta=1.0 (pure integrators) and excludes them from training.
 * setFixedIntegrator() records that intent as metadata for a FUTURE trainer —
 * today's placeholder network trains nothing, so it only forces the two
 * numeric values; no neuron is marked fixed by default.
 * 固定積分ニューロン（将来用）: 論文は一部ニューロンを tau_mem=1.0/theta=1.0
 * （純積分器）に固定し学習対象から外す。setFixedIntegrator() は「将来の学習器」
 * 向けのメタデータとしてその意図を記録する — 今回のプレースホルダーは何も学習
 * しないため、2つの数値を強制するだけで、既定では固定ニューロンは無い。
 *
 * Sizing: runtime-configurable (constructor args), backed by fixed-capacity
 * plain arrays sized to kMaxInputs/kMaxNeurons — no heap, matching this
 * codebase's embedded convention (@design coding_and_education.md — no
 * dynamic allocation in the 400Hz control path). Bump the caps if a later
 * trained network needs a bigger layer.
 * サイズ: コンストラクタ引数で実行時に指定可能。実体は kMaxInputs/kMaxNeurons
 * を上限とする固定容量の配列（ヒープなし、本プロジェクトの組込み流儀 — 400Hz
 * 制御パスで動的確保しない）。将来の学習済みネットワークがより大きい層を必要
 * とする場合は上限定数を引き上げる。
 *
 * @design arXiv:2411.13945 Eq.1-2 — CUBA-LIF neuron model (UNTRAINED weights) [--]
 * @design coding_and_education.md §2 — bilingual comments, no dynamic alloc  [OK]
 */

#pragma once

namespace sf {
namespace snn {

/// A single CUBA-LIF hidden layer (optionally recurrent).
/// 単一の CUBA-LIF 隠れ層（層内再帰結合は任意）。
class CubaLifLayer {
public:
    static constexpr int kMaxInputs  = 16;  ///< upper bound on n_in  / n_in の上限
    static constexpr int kMaxNeurons = 48;  ///< upper bound on n_neurons / n_neurons の上限

    /// @param n_in       Input dimension (<= kMaxInputs) / 入力次元
    /// @param n_neurons  Neuron count (<= kMaxNeurons)    / ニューロン数
    /// @param recurrent  Enable layer-internal recurrent connections (w_ik term)
    ///                   層内再帰結合（w_ik 項）を有効にするか
    CubaLifLayer(int n_in, int n_neurons, bool recurrent = false);

    /// Set feedforward (and, if recurrent, recurrent) weights. `w_in` has
    /// n_neurons*n_in entries, row-major (neuron, input). `w_rec` has
    /// n_neurons*n_neurons entries and is ignored unless this layer is
    /// recurrent AND non-null.
    /// フィードフォワード（再帰があれば再帰も）重みを設定する。`w_in` は
    /// n_neurons*n_in 要素（行優先: ニューロン×入力）。`w_rec` は
    /// n_neurons*n_neurons 要素で、再帰層かつ非nullのときのみ使用する。
    void setWeights(const float* w_in, const float* w_rec = nullptr);

    /// Set the same (tau_mem, tau_syn, theta) for every neuron — the Stage 1
    /// placeholder uses this once at init() with the paper-adjacent defaults
    /// baked into the constructor (0.9, 0.8, 1.0); kept for future per-run
    /// overrides.
    /// 全ニューロンに同一の (tau_mem, tau_syn, theta) を設定する — Stage1の
    /// プレースホルダーはコンストラクタ既定値(0.9, 0.8, 1.0)をそのまま使うため
    /// init() からは呼ばない。将来の一括上書き用に用意。
    void setUniformParams(float tau_mem, float tau_syn, float theta);

    /// Mark neuron `index` as a fixed integrator (tau_mem=tau_syn=theta=1.0,
    /// excluded from future training) or release it back to trainable.
    /// ニューロン `index` を固定積分器（tau_mem=tau_syn=theta=1.0、将来の学習
    /// 対象外）に指定する、または解除する。
    void setFixedIntegrator(int index, bool fixed);
    bool isFixedIntegrator(int index) const { return is_fixed_[index]; }

    /// Advance the layer by one timestep. `input` must have n_in() valid
    /// entries (raw analog values for a first hidden layer, or the previous
    /// layer's spikes). Returns a pointer to this layer's spike output
    /// (n_neurons() entries, 0.0f/1.0f), valid until the next step()/reset().
    /// 層を1ステップ進める。`input` は n_in() 個の有効値（最初の隠れ層なら
    /// 生のアナログ値、それ以外は前層のスパイク）。このステップのスパイク
    /// 出力（n_neurons() 個、0.0f/1.0f）へのポインタを返す。次の
    /// step()/reset() まで有効。
    const float* step(const float* input);

    /// Zero the membrane/synaptic-current/spike state (NOT the weights).
    /// 膜電位・シナプス電流・スパイク状態をゼロにする（重みは変えない）。
    void reset();

    int n_in() const { return n_in_; }
    int n_neurons() const { return n_neurons_; }
    bool recurrent() const { return recurrent_; }

private:
    /// Threshold-check + reset for every neuron, using v_i(t) (BEFORE this
    /// cycle's synaptic-current update) — fills spikes_ and zeros v_ on fire.
    /// 全ニューロンの発火判定+リセット（このサイクルのシナプス電流更新の「前」の
    /// v_i(t) を使用）— spikes_ を埋め、発火したニューロンの v_ をゼロにする。
    void fireAndReset();

    /// Σ_j w_ij * input_j for one neuron (feedforward term of Eq.2).
    /// あるニューロンの Σ_j w_ij * input_j（Eq.2 のフィードフォワード項）。
    float feedforwardCurrent(int neuron, const float* input) const;

    /// Σ_k w_ik * spikes_k for one neuron (recurrent term of Eq.2, this
    /// timestep's own spikes_ — see fireAndReset()).
    /// あるニューロンの Σ_k w_ik * spikes_k（Eq.2 の再帰項。fireAndReset() が
    /// 埋めた「このタイムステップの」spikes_ を使う）。
    float recurrentCurrent(int neuron) const;

    int n_in_;
    int n_neurons_;
    bool recurrent_;

    // Weights, row-major (neuron, source). Sized to the compile-time caps so
    // no heap is used; only the [0, n_neurons_*n_in_) / [0, n_neurons_^2)
    // prefix is ever read or written (see feedforwardCurrent/recurrentCurrent).
    // 重み（行優先: ニューロン×入力元）。ヒープを使わないようコンパイル時上限で
    // 確保し、実際に読み書きするのは [0, n_neurons_*n_in_) / [0, n_neurons_^2)
    // の先頭部分のみ（feedforwardCurrent/recurrentCurrent 参照）。
    float w_in_[kMaxNeurons * kMaxInputs]  = {};
    float w_rec_[kMaxNeurons * kMaxNeurons] = {};

    // Per-neuron parameters, defaulted in the constructor to the Stage 1
    // placeholder values (tau_mem=0.9, tau_syn=0.8, theta=1.0 — "roughly
    // plausible, to be replaced by learned/derived values in Stage 3").
    // ニューロン別パラメータ。コンストラクタで Stage1 プレースホルダー値
    // （tau_mem=0.9, tau_syn=0.8, theta=1.0 —「おおよそ妥当、Stage3で学習/
    // 導出値に置き換え予定」）を既定設定する。
    float tau_mem_[kMaxNeurons];
    float tau_syn_[kMaxNeurons];
    float theta_[kMaxNeurons];
    bool  is_fixed_[kMaxNeurons] = {};  ///< metadata only, see class comment

    // State, carried across step() calls.
    // step() 呼び出しをまたいで保持する状態。
    float v_[kMaxNeurons] = {};       ///< membrane potential v_i(t)
    float i_[kMaxNeurons] = {};       ///< synaptic current i_i(t)
    float v_next_[kMaxNeurons] = {};  ///< scratch for v_i(t+1) (avoids read/write aliasing)
    float i_next_[kMaxNeurons] = {};  ///< scratch for i_i(t+1)
    float spikes_[kMaxNeurons] = {};  ///< this timestep's spike output s_i(t)
};

}  // namespace snn
}  // namespace sf
