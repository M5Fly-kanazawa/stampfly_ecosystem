#!/usr/bin/env python3
"""
losses.py -- the imitation-learning loss from Stroobants et al. 2025
"Neuromorphic Attitude Estimation and Control" (arXiv:2411.13945), Eq.5:

    J(p) = MSE(x, x_hat) + 0.5 * (1 - rho(x, x_hat))

where rho is the Pearson correlation coefficient between the target x and
the network's response x_hat. The paper computes this as "a weighted sum"
without spelling out per-channel vs. pooled weighting; this implementation
computes MSE and Pearson correlation PER OUTPUT CHANNEL along the time axis
(rho needs a time series to correlate against, so it cannot be computed
per-timestep), then averages across channels and batch. See
`imitation_loss()`'s docstring for the exact shapes.

模倣学習の損失（Stroobants et al. 2025 Eq.5）: 目標 x とネットワーク応答 x_hat
の間の MSE と、ピアソン相関係数 rho による (1 - rho)/2 の加重和。論文は
「加重和」とだけ書いてチャネル別か全体プールかは明記していないため、本実装は
時間軸方向にチャネルごとの MSE・ピアソン相関を計算してからチャネル・バッチで
平均する（rho は時系列でないと計算できないため）。
"""

from __future__ import annotations

import torch


def pearson_corr(x: torch.Tensor, y: torch.Tensor, dim: int = 1, eps: float = 1e-8) -> torch.Tensor:
    """Pearson correlation coefficient along `dim`, computed independently
    for every other dimension (e.g. batch, channel). x, y: same shape,
    typically (batch, time, channel) with dim=1 (time). Returns a tensor
    with `dim` removed.
    `dim` に沿ったピアソン相関係数を、他の次元（バッチ・チャネル等）ごとに
    独立に計算する。x, y は同形状、通常 (batch, time, channel) で dim=1（時間軸）。
    `dim` を潰した形状のテンソルを返す。
    """
    x_c = x - x.mean(dim=dim, keepdim=True)
    y_c = y - y.mean(dim=dim, keepdim=True)
    num = (x_c * y_c).sum(dim=dim)
    # eps goes INSIDE sqrt, not added after it: d/dx sqrt(x) = 1/(2*sqrt(x))
    # is infinite at x=0, so a zero-variance window (a real occurrence in
    # this dataset -- e.g. an unexcited axis in a per-axis scenario, or an
    # untrained network's constant output) produces an inf/nan gradient
    # through `sqrt(var_x * var_y) + eps`. sqrt(var_x * var_y + eps) keeps
    # the gradient finite everywhere while being numerically identical to
    # the old formula away from zero (eps=1e-8 is negligible next to any
    # non-trivial variance product).
    # eps は sqrt の外ではなく中に入れる: d/dx sqrt(x) = 1/(2*sqrt(x)) は
    # x=0 で発散するため、分散ゼロのウィンドウ（本データセットで実際に起こる
    # -- 軸別シナリオで励起されていない軸、または未学習ネットワークの一定
    # 出力など）では `sqrt(var_x * var_y) + eps` の勾配が inf/nan になる。
    # `sqrt(var_x * var_y + eps)` なら勾配はどこでも有限になり、分散が
    # ゼロから離れていれば旧式と数値的に同一（eps=1e-8は非自明な分散の積に
    # 対して無視できるほど小さい）。
    den = torch.sqrt((x_c**2).sum(dim=dim) * (y_c**2).sum(dim=dim) + eps)
    return num / den


def imitation_loss(pred: torch.Tensor, target: torch.Tensor, valid_mask: torch.Tensor | None = None):
    """Paper Eq.5: J = MSE(x, x_hat) + 0.5*(1 - rho(x, x_hat)), per channel
    along the time axis, then averaged over channels and batch.

    pred, target: (batch, time, channel) -- e.g. channel=3 for the
    estimator's [roll, pitch, yaw] or channel=4 for the controller's
    [thrust, torque_roll, torque_pitch, torque_yaw].
    valid_mask: optional (batch, time) bool/float mask (1=use, 0=ignore),
    e.g. for rows where a target is NaN in the source log (angle_ref during
    pure ACRO -- see build_dataset.py's column_doc) or for the tail rows
    consumed by the time-shift (see train_common.py's `shift_target()`).
    Masked-out rows are excluded from BOTH the MSE and the Pearson mean/std,
    not just zeroed, so they do not bias either statistic.

    Returns (loss, mse, pearson) -- all scalars (mse/pearson are the
    channel-and-batch-averaged values, for logging).

    論文Eq.5。pred, target: (batch, time, channel)。valid_mask: (batch, time)
    のオプションのマスク（NaNターゲット行や時間シフトの末尾行を除外）。
    マスクされた行はMSE・ピアソン相関のどちらからも除外する（0埋めではなく
    統計量自体から取り除く）。戻り値 (loss, mse, pearson) はすべてスカラー
    （mse/pearsonはロギング用にチャネル・バッチ平均したもの）。
    """
    if valid_mask is not None:
        m = valid_mask.to(pred.dtype).unsqueeze(-1)  # (batch, time, 1), broadcasts over channel
        n = m.sum(dim=1).clamp_min(1.0)  # (batch, 1) -- valid timesteps per (batch,)
        # Per-channel mean over VALID timesteps only, for both MSE and the
        # Pearson centering step below.
        # 有効なタイムステップのみでのチャネル別平均（MSE、以下のピアソン
        # 中心化の両方に使う）。
        sq_err = ((pred - target) ** 2) * m
        mse = (sq_err.sum(dim=1) / n).mean()

        pred_mean = (pred * m).sum(dim=1, keepdim=True) / n.unsqueeze(1)
        target_mean = (target * m).sum(dim=1, keepdim=True) / n.unsqueeze(1)
        pred_c = (pred - pred_mean) * m
        target_c = (target - target_mean) * m
        num = (pred_c * target_c).sum(dim=1)
        # Same eps-inside-sqrt fix as pearson_corr() above -- see that
        # function's comment. This branch hit it in practice: axis-only
        # scenarios (pos_roll/pos_pitch/alt_flight) have an exactly-zero
        # torque/euler variance on their unexcited axes, which produced
        # inf/nan gradients on the very first training batch before this
        # fix (worked around upstream in train.py with tiny added noise
        # until this was fixed -- that workaround is no longer necessary
        # but harmless to keep).
        # 上の pearson_corr() と同じ eps-inside-sqrt 修正（コメント参照）。
        # この分岐は実際に踏んでいた: 軸別シナリオ(pos_roll/pos_pitch/
        # alt_flight)は励起していない軸のトルク/オイラー角分散が厳密に
        # ゼロで、修正前は最初の学習バッチから inf/nan 勾配が出ていた
        # （この修正までは train.py 側で微小ノイズを加える回避策を併用。
        # 修正後は不要だが残しても無害）。
        den = torch.sqrt((pred_c**2).sum(dim=1) * (target_c**2).sum(dim=1) + 1e-8)
        rho = (num / den).mean()
    else:
        mse = torch.nn.functional.mse_loss(pred, target)
        rho = pearson_corr(pred, target, dim=1).mean()

    loss = mse + 0.5 * (1.0 - rho)
    return loss, mse.detach(), rho.detach()
