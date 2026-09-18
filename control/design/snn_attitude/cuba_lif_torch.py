#!/usr/bin/env python3
"""
cuba_lif_torch.py -- PyTorch CUBA-LIF (current-based leaky integrate-and-fire)
spiking neuron layer, trainable via BPTT with a surrogate gradient.

This is the PC-side training-time twin of
firmware/vehicle/components/sf_snn_core/include/cuba_lif_layer.hpp -- same
equations (Stroobants et al. 2025, "Neuromorphic Attitude Estimation and
Control", arXiv:2411.13945, Eq.1-2), same fixed-integrator-neuron mechanism
(paper Section II-C.4), same weight layout (row-major [n_neurons, n_in] /
[n_neurons, n_neurons]) so a trained state_dict can be exported as a
drop-in replacement for snn_weight_init.hpp's placeholder weights.

CUBA-LIF層（電流ベース漏洩積分発火）のPyTorch実装。BPTT + サロゲート勾配で
学習可能。firmware/vehicle/components/sf_snn_core/include/cuba_lif_layer.hpp
の学習時側の対（同じ数式: Stroobants et al. 2025 Eq.1-2、同じ固定積分ニューロン
機構: 論文II-C.4節、同じ重みレイアウト: 行優先 [n_neurons, n_in] /
[n_neurons, n_neurons]）。学習済み state_dict を snn_weight_init.hpp の
プレースホルダー重みの置き換えとしてそのままエクスポートできる。

Neuron dynamics (per-neuron i, matches cuba_lif_layer.cpp::step() exactly --
fire-and-reset uses v_i(t) from BEFORE this cycle's current update):
  v_i(t)   >= theta_i  -> spike s_i(t)=1, then v_i(t) := 0 (reset)
  v_i(t+1)  = tau_mem_i * v_i(t) + i_i(t)
  i_i(t+1)  = tau_syn_i * i_i(t) + sum_j(w_ij * s_j(t)) [+ sum_k(w_ik * s_k(t))]

Surrogate gradient (paper Eq.6, slope s=7 -- the derivative of a scaled
arctan used in the BACKWARD pass only; the forward pass is the exact
Heaviside step, so numerical outputs match the C++ inference engine
bit-for-bit given the same weights/inputs):
  d/dx (1/s * arctan(s*x)) = 1 / (1 + (s*x)^2),  x = v - theta
"""

from __future__ import annotations

import torch
import torch.nn as nn


class SpikeFn(torch.autograd.Function):
    """Heaviside step forward (v - theta >= 0 -> 1.0 else 0.0); scaled-arctan
    surrogate derivative backward (paper Eq.6). `slope` is NOT a learnable
    tensor -- it is a fixed hyperparameter (paper uses 7) passed as a plain
    float via `ctx`, so no gradient is returned for it (see the `None` in
    backward's return tuple, matching forward's `(v_minus_theta, slope)`
    signature).
    Heaviside関数を forward、scaled-arctan のサロゲート微分を backward で使う
    （論文Eq.6）。`slope` は学習対象ではない固定ハイパーパラメータ（論文は7）
    なので、backward はこれに対する勾配を返さない（forward の引数
    `(v_minus_theta, slope)` に対応する `None`）。
    """

    @staticmethod
    def forward(ctx, v_minus_theta: torch.Tensor, slope: float) -> torch.Tensor:
        ctx.save_for_backward(v_minus_theta)
        ctx.slope = slope
        return (v_minus_theta >= 0).to(v_minus_theta.dtype)

    @staticmethod
    def backward(ctx, grad_output: torch.Tensor):
        (x,) = ctx.saved_tensors
        s = ctx.slope
        surrogate = 1.0 / (1.0 + (s * x) ** 2)
        return grad_output * surrogate, None


spike_fn = SpikeFn.apply


class CubaLifLayer(nn.Module):
    """One CUBA-LIF hidden layer, optionally recurrent.

    n_in, n_neurons: layer size. MUST match the C++ side's constructor args
    (SnnEstimator: 6/32, SnnController: 10/16 as of Stage 1) so the exported
    weights are a drop-in replacement -- see export_weights.py.
    recurrent: enables the w_rec term (Eq.2's sum_k term); False just skips
    it, matching cuba_lif_layer.cpp's `if (recurrent_)` branch.
    n_fixed: the LAST n_fixed neurons (index n_neurons-n_fixed .. n_neurons-1)
    get tau_mem=tau_syn=theta=1.0 permanently pinned. Per the paper (Section
    II-C.4), this is ONLY used for the CONTROLLER's hidden layer (10 of its
    150 neurons in the paper) -- the estimator's layer uses n_fixed=0.
    Pinning is enforced every forward() call via effective_tau_theta()
    (torch.where against a boolean mask), so gradients w.r.t. the pinned
    entries are naturally zero -- no separate optimizer param-group
    exclusion is needed; the optimizer sees zero gradient there and leaves
    the value where it started (1.0).
    slope: the surrogate gradient's scaled-arctan slope (paper: 7).

    単一の CUBA-LIF 隠れ層（層内再帰結合は任意）。

    n_in, n_neurons: 層のサイズ。エクスポートした重みがそのまま差し替え可能なよう
    C++側のコンストラクタ引数と一致させること（Stage1時点: SnnEstimator=6/32,
    SnnController=10/16）。
    recurrent: w_rec 項（Eq.2のsum_k項）を有効にするか。Falseならcuba_lif_layer.cpp
    の `if (recurrent_)` 分岐同様にスキップする。
    n_fixed: 末尾 n_fixed 個のニューロン（index n_neurons-n_fixed .. n_neurons-1）
    を tau_mem=tau_syn=theta=1.0 に恒久固定する。論文(II-C.4節)通り、これは
    「制御器の隠れ層のみ」（論文では150ニューロン中10個）が対象 — 推定器の層は
    n_fixed=0 にする。固定は forward() 毎に effective_tau_theta()（ブール
    マスクに対する torch.where）で強制するため、固定対象への勾配は自然に0になり、
    別途オプティマイザのparam_group除外は不要（勾配0なので初期値1.0のまま動かない）。
    slope: サロゲート勾配の scaled-arctan の傾き（論文: 7）。
    """

    def __init__(
        self,
        n_in: int,
        n_neurons: int,
        recurrent: bool = False,
        n_fixed: int = 0,
        slope: float = 7.0,
        init_scale: float = 0.1,
    ):
        super().__init__()
        assert 0 <= n_fixed <= n_neurons
        self.n_in = n_in
        self.n_neurons = n_neurons
        self.recurrent = recurrent
        self.n_fixed = n_fixed
        self.slope = slope

        self.w_in = nn.Parameter(torch.empty(n_neurons, n_in).uniform_(-init_scale, init_scale))
        if recurrent:
            self.w_rec = nn.Parameter(torch.empty(n_neurons, n_neurons).uniform_(-init_scale, init_scale))
        else:
            self.register_parameter("w_rec", None)

        # Stage 1's C++ placeholder defaults (cuba_lif_layer.cpp ctor):
        # tau_mem=0.9, tau_syn=0.8, theta=1.0 for every neuron, THEN pin the
        # fixed subset to 1.0/1.0/1.0 -- same starting point as the untrained
        # firmware skeleton, now made trainable (except the fixed subset).
        # Stage1のC++プレースホルダー既定値（cuba_lif_layer.cppコンストラクタ）:
        # 全ニューロン tau_mem=0.9, tau_syn=0.8, theta=1.0。その後、固定対象
        # だけ1.0/1.0/1.0に上書き — 未学習ファームウェア骨格と同じ出発点を
        # （固定対象を除き）学習可能にする。
        init_tau_mem = torch.full((n_neurons,), 0.9)
        init_tau_syn = torch.full((n_neurons,), 0.8)
        init_theta = torch.full((n_neurons,), 1.0)
        if n_fixed > 0:
            init_tau_mem[-n_fixed:] = 1.0
            init_tau_syn[-n_fixed:] = 1.0
        self.tau_mem = nn.Parameter(init_tau_mem)
        self.tau_syn = nn.Parameter(init_tau_syn)
        self.theta = nn.Parameter(init_theta)

        fixed_mask = torch.zeros(n_neurons, dtype=torch.bool)
        if n_fixed > 0:
            fixed_mask[-n_fixed:] = True
        self.register_buffer("fixed_mask", fixed_mask)

    def effective_tau_theta(self):
        """(tau_mem, tau_syn, theta) with the fixed-integrator subset pinned
        to 1.0 regardless of the Parameter's current value -- used by both
        forward() and anything inspecting "what will actually run".
        固定積分ニューロンの部分を、Parameterの現在値に関わらず1.0に固定した
        (tau_mem, tau_syn, theta) を返す -- forward() と「実際に使われる値」を
        確認したい側の両方から使う。
        """
        one = torch.ones_like(self.tau_mem)
        tau_mem = torch.where(self.fixed_mask, one, self.tau_mem)
        tau_syn = torch.where(self.fixed_mask, one, self.tau_syn)
        theta = torch.where(self.fixed_mask, one, self.theta)
        return tau_mem, tau_syn, theta

    def init_state(self, batch_size: int, device=None, dtype=None):
        v = torch.zeros(batch_size, self.n_neurons, device=device, dtype=dtype)
        i = torch.zeros(batch_size, self.n_neurons, device=device, dtype=dtype)
        return v, i

    def forward(self, input_t: torch.Tensor, state):
        """input_t: (batch, n_in) -- raw analog values for the first hidden
        layer, or the previous layer's spikes (0/1) otherwise. state: (v, i)
        from init_state() or the previous step's return. Returns
        (spikes, new_state); spikes: (batch, n_neurons) in {0,1} (float,
        differentiable through the surrogate gradient).

        Order of operations mirrors cuba_lif_layer.cpp::step() exactly:
        threshold-check + reset uses v(t) BEFORE this cycle's synaptic
        current update, then v(t+1)=tau_mem*v(t)+i(t) [v already reset],
        i(t+1)=tau_syn*i(t)+feedforward(+recurrent, using THIS step's
        spikes).
        cuba_lif_layer.cpp::step() と全く同じ演算順序: 発火判定+リセットは
        「このサイクルのシナプス電流更新の前」の v(t) を使い、その後
        v(t+1)=tau_mem*v(t)+i(t)（v はリセット済み）、
        i(t+1)=tau_syn*i(t)+フィードフォワード（+再帰、この時刻のスパイクを使用）。
        """
        v, i = state
        tau_mem, tau_syn, theta = self.effective_tau_theta()

        spikes = spike_fn(v - theta, self.slope)
        # Reset-by-multiply: v*(1-spike) is numerically IDENTICAL to "if
        # fired, hard-set v=0, else unchanged" (spikes is exactly 0.0/1.0 in
        # the forward pass), while staying differentiable so gradient can
        # flow back through which neurons fired.
        # 乗算によるリセット: v*(1-spike) は「発火したら v=0、しなければ
        # 不変」と数値的に完全に一致する（forwardではspikesは厳密に0.0/1.0）。
        # かつ微分可能なので、どのニューロンが発火したかを通じて勾配が流れる。
        v = v * (1.0 - spikes)

        v_next = tau_mem * v + i
        syn = input_t @ self.w_in.t()
        if self.recurrent:
            syn = syn + spikes @ self.w_rec.t()
        i_next = tau_syn * i + syn

        return spikes, (v_next, i_next)


class LinearReadout(nn.Module):
    """Linear readout layer: y(t) = W_o * s(t) (paper's Eq. right after
    Eq.2, no activation function). Mirrors sf_snn_core/include/linear_readout.hpp.
    線形読み出し層: y(t) = W_o * s(t)（論文Eq.2直後の式、活性化関数なし）。
    sf_snn_core/include/linear_readout.hpp に対応。
    """

    def __init__(self, n_in: int, n_out: int, init_scale: float = 0.1):
        super().__init__()
        self.weight = nn.Parameter(torch.empty(n_out, n_in).uniform_(-init_scale, init_scale))

    def forward(self, spikes: torch.Tensor) -> torch.Tensor:
        return spikes @ self.weight.t()
