#!/usr/bin/env python3
"""
train.py -- BPTT imitation-learning training loop for the SNN attitude
estimator/controller (Stage 2b of the Stroobants et al. 2025 port, see
README.md). Trains ONE of the two networks per run (`--target
estimator|controller`) against the paper's Eq.5 loss (losses.py) with
gradient clipping (see the CLI's `--grad-clip` docstring below for why it is
mandatory here).

SNN姿勢推定器/制御器の模倣学習BPTT学習ループ（Stage2b）。`--target` で
推定器/制御器のどちらか一方を1回の実行で学習する。損失は論文Eq.5
（losses.py）、勾配クリッピング必須（理由は下記 `--grad-clip` 参照）。

Network sizing is hard-coded to match firmware/vehicle's Stage 1 C++
skeleton EXACTLY (snn_estimator.hpp/snn_controller.hpp kInputDim/kHiddenDim/
kOutputDim) so `export_weights.py` can produce a drop-in replacement for
snn_weight_init.hpp's placeholder weights -- see _NETWORK_SPECS below.

Usage / 使い方:
    ./venv/bin/python train.py --target estimator
    ./venv/bin/python train.py --target controller
    ./venv/bin/python train.py --target controller --epochs 50 --lr 5e-4

=============================================================================
UPDATE (main session, after this Stage 2b report): the root cause below is
now FIXED in losses.py (eps moved inside both sqrt() calls in pearson_corr()
and imitation_loss()'s masked branch). The workaround in this file
(_SINGULARITY_GUARD_EPS, below) is no longer necessary but is left in place
-- it is harmless (noise scale 1e-6 is far below any real signal here) and
this file's trained checkpoints/reported numbers were produced WITH it
active, so removing it would invalidate reproducibility of this run without
adding correctness. A future retrain may drop it.
更新（メインセッション、本Stage2b報告の後）: 下記の根本原因は losses.py で
修正済み（pearson_corr() と imitation_loss() のマスク分岐、両方の sqrt()
の中に eps を移動）。本ファイルの回避策（_SINGULARITY_GUARD_EPS、下記）は
もう不要だが、害もない（ノイズ規模1e-6はどの実信号よりずっと小さい）ため
残す -- 本ファイルの学習済みチェックポイント・報告数値はこの回避策が有効な
状態で作られたため、外すと正しさは変わらないまま再現性だけ失う。将来の
再学習時に外してよい。
=============================================================================

=============================================================================
losses.py numerical singularity found here, WORKED AROUND (not fixed in
losses.py -- per task constraints: reported, not silently patched; the
math in losses.py/cuba_lif_torch.py is off-limits, the file owner decides).
losses.py の数値特異点をここで発見。losses.py 自体は変更せず（タスク制約:
数式は変更禁止、判断はファイル作成者に委ねる）回避する。
=============================================================================

`pearson_corr`/`imitation_loss` (losses.py) compute, per channel along the
time axis:
    den = sqrt((pred_c**2).sum(dim=time) * (target_c**2).sum(dim=time)) + eps
with `eps` added AFTER the sqrt, not inside it (i.e. `sqrt(u) + eps`, not
`sqrt(u + eps)`). Verified directly that `torch.sqrt` has an
INFINITE/NaN gradient at exactly 0:
    torch.sqrt(torch.tensor(0.0, requires_grad=True)).backward()  # grad = inf
    (and the vector form sqrt(sum(x**2)) at x=all-zero gives grad=nan per
    element -- the classic 0/0 gradient of a norm at the origin)
So whenever a channel's pred_c OR target_c (the window's time-centered
series) is EXACTLY constant, `loss.backward()` yields NaN gradients for
every trainable parameter -- confirmed empirically (see this task's report)
that this reproduces starting from `--epochs 1` on the real data, not a
contrived case.

This is NOT rare here, on BOTH sides:
  - target side: several Stage 2a scenarios are deliberately single-axis
    excitations run through a deterministic, noise-free SILS (`--noise
    off`) -- alt_flight's attitude+torque and pos_roll/pos_pitch's
    off-axis torque are bit-for-bit 0.0 for the ENTIRE scenario (verified:
    `d["torque"].std(axis=0)` is exactly 0.0 on those channels, vs. the
    smallest genuinely-excited channel at std=4.97e-5). A real sensor/
    controller would essentially never hold exactly-zero variance for
    seconds; this is an artifact of clean simulated teacher data.
  - prediction side: an UNTRAINED (or under-trained / dead-neuron) SNN
    layer can easily produce an all-zero (non-firing) or otherwise
    constant spike train for an entire 1s window, especially early in
    training -- making pred_c exactly constant too, independent of the
    target.
Either side alone triggers the singularity (den's product-under-sqrt is 0
if EITHER factor is 0), so a target-only workaround (e.g. dithering just
the degenerate scenarios' labels in dataset.py) was tried first and found
INSUFFICIENT: it fixed some batches but real training still produced NaN
gradients from prediction-side constant outputs (see this task's report for
the batch-by-batch reproduction). Dropping the affected scenarios entirely
was rejected too: pos_roll/pos_pitch/alt_flight are ~80% of the
controller's training windows, and Stage 2a deliberately chose this
per-axis layered-excitation design (README.md).

Fix used here (`_SINGULARITY_GUARD_EPS`, applied in `_run_epoch` only when
`train=True`, i.e. only on the path that calls `.backward()`): add
independent N(0, eps) noise, freshly drawn every forward call, to BOTH
`pred` and `target` before calling `imitation_loss()`. Independent
per-channel-per-timestep noise has continuous (non-atomic) support, so
`P(pred_c exactly constant) = 0` and likewise for target_c -- the
singularity is avoided with probability 1, on both sides, unconditionally
(no per-scenario/per-channel bookkeeping needed). eps=1e-6 is 4-5 orders of
magnitude below the smallest genuine signal observed (~5e-5) and below any
reporting precision used in this project, so it does not measurably change
MSE/Pearson on real (non-degenerate) channels. VALIDATION forward passes
(`train=False`) never call `.backward()`, so they are NEVER guarded --
`val_loss`/`val_mse`/`val_rho` and every reported RMSE are always computed
against the untouched, exact pred/target (see `_channel_sq_err`, which also
always uses the unguarded tensors, even in the `train=True` branch).

=============================================================================
Output-channel scale imbalance (a training-quality issue found empirically,
not a losses.py bug -- worked around on the DATA side, see
`_target_channel_scale`/`output_scale` below).
出力チャネルのスケール不均衡（学習品質の問題、losses.py のバグではない
-- データ側で回避、下記 `_target_channel_scale`/`output_scale` 参照）。
=============================================================================

losses.py's MSE/Pearson terms average uniformly across output channels
(paper Eq.5 as given -- no per-channel weighting). The controller's 4
output channels differ in physical scale by ~2-3 orders of magnitude
(thrust ~0.1-0.4 N vs torque ~1e-4-1e-3 N*m); the estimator's 3 channels
also differ substantially across the training scenarios (e.g. acro_flight's
yaw swings to 87 deg while its roll/pitch stay within ~1-7 deg). An early
100-epoch run confirmed this in practice: the controller's thrust RMSE
dropped steadily (0.33 N -> 0.22 N) while torque RMSE barely moved and
stayed ~20x larger than the torque target's own dynamic range (~1.3e-3
N*m) -- the large-scale channel dominates the raw MSE gradient, starving
the small-scale channels of effective learning signal.

Fix: `_target_channel_scale()` computes each channel's std over the
TRAINING split only (never val, to avoid leaking val statistics), floored
at 1e-4 native units. The network is trained to predict `target /
output_scale` (every channel ~O(1)) instead of raw physical units; this is
purely a DATA-side rescaling of what the (unchanged) loss function is asked
to fit, not a change to losses.py/cuba_lif_torch.py's math. Because the
readout is linear with no bias (`y = W_o @ spikes`, see
cuba_lif_torch.py's LinearReadout), this rescaling is EXACT and reversible:
`pred_physical = pred_normalized * output_scale` for every physical-unit
use (RMSE here, and the exported readout weights in export_weights.py,
which multiplies each output row of `W_o` by the corresponding
`output_scale` entry -- see its docstring). `output_scale` is saved in the
checkpoint (`runs/<target>_best.pt`) precisely so export_weights.py can
apply this without re-deriving it from the training data.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless -- no display available in this environment
import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from cuba_lif_torch import CubaLifLayer, LinearReadout
from dataset import TRAIN_SCENARIOS, VAL_SCENARIOS, WindowedImitationDataset
from losses import imitation_loss

_THIS_DIR = Path(__file__).resolve().parent
_DEFAULT_OUT_DIR = _THIS_DIR / "runs"

# See the module docstring's "losses.py numerical singularity" section:
# independent noise at this scale, added to (pred, target) ONLY on the
# training (backward) path, keeps losses.imitation_loss()'s Pearson term
# from differentiating sqrt(0) when a channel is exactly constant.
# モジュール docstring の「losses.py 数値特異点」節参照: この規模の独立
# ノイズを学習（backward）経路にのみ加え、チャンネルが完全に一定な場合の
# sqrt(0)微分を回避する。
_SINGULARITY_GUARD_EPS = 1e-6

# Network sizing per target -- MUST match the C++ Stage1 skeleton exactly
# (see the task's cross-reference: snn_estimator.cpp/snn_controller.cpp).
# n_fixed for the controller (fixed-integrator neurons, paper Section
# II-C.4): the paper uses 10 of the controller's 150 hidden neurons (~6.7%).
# This network has 16 neurons; scaling that fraction gives ~1, but the
# controller here has 4 DISTINCT output channels (thrust + 3 torques) versus
# the paper's single-purpose control network, so 4 fixed integrators (one
# per output channel's worth of slow/DC-tracking capacity) is used instead
# -- see cuba_lif_torch.py's CubaLifLayer docstring for what "fixed" means
# mechanically (tau_mem=tau_syn=theta=1.0, excluded from gradient updates).
# This is a judgment call, not derived from the paper; adjust if training
# results suggest otherwise.
# ネットワークサイズはC++Stage1と厳密一致。制御器のn_fixed=4は論文の
# 150ニューロン中10個(~6.7%)を単純比例した場合の~1ではなく、本ネットワークが
# 推力+3軸トルクの4出力チャンネルを持つため「出力チャンネルごとに最低限の
# 積分容量」という意図で4を採用（要調整可、根拠はここに記載）。
_NETWORK_SPECS = {
    "estimator": dict(n_in=6, n_neurons=32, recurrent=True, n_fixed=0, n_out=3),
    "controller": dict(n_in=10, n_neurons=16, recurrent=False, n_fixed=4, n_out=4),
}


class SnnModel(nn.Module):
    """hidden CubaLifLayer + LinearReadout run over a (batch, time, n_in)
    window, hidden state freshly reset (zeroed) at t=0 of every window --
    matches dataset.py's windowing contract (each window is an independent
    BPTT truncation, no state carried across windows).
    隠れ層CubaLifLayer + LinearReadoutを(batch,time,n_in)ウィンドウ全体で
    実行し、各ウィンドウの先頭で隠れ状態をゼロリセットする
    （dataset.pyのウィンドウ契約に対応 -- ウィンドウ間で状態を持ち越さない）。
    """

    def __init__(self, n_in: int, n_neurons: int, n_out: int, recurrent: bool, n_fixed: int):
        super().__init__()
        self.hidden = CubaLifLayer(n_in, n_neurons, recurrent=recurrent, n_fixed=n_fixed)
        self.readout = LinearReadout(n_neurons, n_out)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        batch, steps, _ = x.shape
        state = self.hidden.init_state(batch, device=x.device, dtype=x.dtype)
        spikes_seq = []
        for t in range(steps):
            spikes, state = self.hidden(x[:, t, :], state)
            spikes_seq.append(spikes)
        spikes_seq = torch.stack(spikes_seq, dim=1)  # (batch, steps, n_neurons)
        return self.readout(spikes_seq)  # (batch, steps, n_out)


def _channel_sq_err(pred: torch.Tensor, target: torch.Tensor, valid: torch.Tensor):
    """Sum of squared error per output channel and valid-timestep count,
    over VALID (batch,time) entries only -- accumulated across batches by
    the caller to get an exact (not batch-averaged) RMSE at the end of an
    epoch. Returns (sq_err_sum: (C,), n_valid: scalar).
    有効な(batch,time)要素のみのチャネル別二乗誤差和と有効タイムステップ数
    -- 呼び出し側がバッチをまたいで累積し、エポック終端で
    （バッチ平均でなく）厳密なRMSEを得る。
    """
    mask = valid.bool()  # (B,T)
    err2 = (pred - target) ** 2  # (B,T,C)
    sq_sum = err2[mask].sum(dim=0)  # (C,)
    n_valid = mask.sum()
    return sq_sum, n_valid


def _run_epoch(model, loader, optimizer, grad_clip, device, output_scale: torch.Tensor, train: bool):
    """One pass over `loader`. If `train`, updates weights (with grad-norm
    clipping); otherwise runs under torch.no_grad(). Returns a dict of
    epoch-pooled metrics: loss/mse/rho (batch-count-weighted mean, for
    logging) and sq_err_sum/n_valid (exact, for RMSE -- see caller).

    `output_scale` (n_out,): per-channel target std from the TRAINING split
    (see `_target_channel_scale`). The network is trained against
    `y / output_scale` (comparable ~O(1) magnitude on every channel) rather
    than raw physical units -- see the module docstring's "output channel
    scale imbalance" section for why. `pred` (the network's raw output) is
    therefore in NORMALIZED units throughout; it is rescaled by
    `output_scale` before every physical-unit use (RMSE here,
    export_weights.py's readout-weight export).
    `loader` を1周する。train=Trueなら（勾配クリッピング付きで）重みを更新、
    そうでなければ torch.no_grad() 下で実行する。`output_scale` の意味は
    モジュール docstring の「出力チャネルのスケール不均衡」節参照。
    """
    model.train(train)
    total_loss = total_mse = total_rho = 0.0
    n_batches = 0
    sq_sum_total = None
    n_valid_total = 0.0

    context = torch.enable_grad() if train else torch.no_grad()
    with context:
        for x, y, v in loader:
            x, y, v = x.to(device), y.to(device), v.to(device)
            pred = model(x)  # normalized units (see output_scale doc above)
            target_norm = y / output_scale

            if train:
                # Singularity guard -- training/backward path only, see the
                # module docstring. Logging/RMSE below always uses the
                # UNGUARDED `pred`/`y` (rescaled to physical units).
                pred_for_loss = pred + torch.randn_like(pred) * _SINGULARITY_GUARD_EPS
                target_for_loss = target_norm + torch.randn_like(target_norm) * _SINGULARITY_GUARD_EPS
            else:
                pred_for_loss = pred
                target_for_loss = target_norm

            loss, mse, rho = imitation_loss(pred_for_loss, target_for_loss, valid_mask=v)

            if train:
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=grad_clip)
                optimizer.step()

            total_loss += loss.item()
            total_mse += mse.item()
            total_rho += rho.item()
            n_batches += 1

            with torch.no_grad():
                pred_physical = pred * output_scale
                sq_sum, n_valid = _channel_sq_err(pred_physical, y, v)
                sq_sum_total = sq_sum if sq_sum_total is None else sq_sum_total + sq_sum
                n_valid_total += n_valid.item()

    return {
        "loss": total_loss / n_batches,
        "mse": total_mse / n_batches,
        "rho": total_rho / n_batches,
        "sq_err_sum": sq_sum_total.detach().cpu().numpy(),
        "n_valid": n_valid_total,
    }


def _report_metrics(target: str, epoch_stats: dict) -> str:
    """Human-readable physical-unit summary line appended to the per-epoch
    log (RMSE in degrees for the estimator; thrust[N]/torque[N*m] RMSE for
    the controller -- task spec report items).
    エポックログに付記する物理単位の人間可読サマリ（推定器: 姿勢RMSE[deg]、
    制御器: thrust[N]/torque[N*m] RMSE）。
    """
    rmse_per_channel = np.sqrt(epoch_stats["sq_err_sum"] / epoch_stats["n_valid"])
    if target == "estimator":
        rmse_deg = np.degrees(rmse_per_channel)
        pooled_deg = float(np.sqrt(np.mean(epoch_stats["sq_err_sum"]) / epoch_stats["n_valid"]))
        pooled_deg = np.degrees(pooled_deg)
        return (
            f"attitude_rmse_deg(pooled)={pooled_deg:.3f} "
            f"[roll={rmse_deg[0]:.3f} pitch={rmse_deg[1]:.3f} yaw={rmse_deg[2]:.3f}]"
        )
    thrust_rmse_n = float(rmse_per_channel[0])
    torque_sq_sum = float(epoch_stats["sq_err_sum"][1:].sum())
    torque_n = epoch_stats["n_valid"] * 3  # 3 torque channels pooled together
    torque_rmse_nm = float(np.sqrt(torque_sq_sum / torque_n))
    return f"thrust_rmse_N={thrust_rmse_n:.4f} torque_rmse_Nm={torque_rmse_nm:.6f}"


def _target_channel_scale(train_ds: WindowedImitationDataset, floor: float = 1e-4) -> torch.Tensor:
    """Per-channel target std over `train_ds` (TRAINING split only -- never
    val, to avoid leaking validation statistics into training), floored at
    `floor` native units. See the module docstring's "output channel scale
    imbalance" section for why this exists.
    `train_ds`（学習分割のみ、検証統計量の漏洩を避けるためvalは使わない）での
    チャネル別目標値stdを、`floor`（ネイティブ単位）で下限クリップして返す。
    理由はモジュール docstring の「出力チャネルのスケール不均衡」節参照。
    """
    std = train_ds.all_targets().std(dim=0)
    return torch.clamp(std, min=floor)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", choices=["estimator", "controller"], required=True)
    parser.add_argument("--data-dir", type=Path, default=None, help="default: ./data")
    parser.add_argument("--out-dir", type=Path, default=_DEFAULT_OUT_DIR)
    parser.add_argument("--window", type=int, default=400, help="samples per BPTT window (400 = 1s @ 400Hz)")
    parser.add_argument("--stride", type=int, default=100, help="window start stride (100 = 0.25s @ 400Hz)")
    parser.add_argument("--shift", type=int, default=6, help="target time-shift, steps into the future")
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument(
        "--grad-clip",
        type=float,
        default=1.0,
        help=(
            "max grad-norm for clip_grad_norm_ -- MANDATORY, not optional: a sanity "
            "check over a 500-step rollout showed gradient norms reaching the "
            "thousands without clipping, i.e. this can diverge if disabled."
        ),
    )
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--n-fixed", type=int, default=None, help="override the controller's n_fixed (default: see _NETWORK_SPECS)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    data_kwargs = dict(window=args.window, stride=args.stride, shift=args.shift)
    if args.data_dir is not None:
        data_kwargs["data_dir"] = args.data_dir
    train_ds = WindowedImitationDataset(TRAIN_SCENARIOS, args.target, **data_kwargs)
    val_ds = WindowedImitationDataset(VAL_SCENARIOS, args.target, **data_kwargs)

    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, generator=generator)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False)

    device = torch.device(args.device)
    output_scale = _target_channel_scale(train_ds).to(device)

    spec = dict(_NETWORK_SPECS[args.target])
    if args.target == "controller" and args.n_fixed is not None:
        spec["n_fixed"] = args.n_fixed
    model = SnnModel(**spec).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    history = []
    best_val_loss = float("inf")
    best_path = args.out_dir / f"{args.target}_best.pt"

    print(
        f"[{args.target}] train_windows={len(train_ds)} val_windows={len(val_ds)} "
        f"spec={spec} window={args.window} stride={args.stride} shift={args.shift} "
        f"lr={args.lr} epochs={args.epochs} grad_clip={args.grad_clip} batch_size={args.batch_size} "
        f"output_scale={output_scale.tolist()}"
    )

    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        train_stats = _run_epoch(model, train_loader, optimizer, args.grad_clip, device, output_scale, train=True)
        val_stats = _run_epoch(model, val_loader, optimizer, args.grad_clip, device, output_scale, train=False)

        history.append(
            {
                "epoch": epoch,
                "train_loss": train_stats["loss"],
                "train_mse": train_stats["mse"],
                "train_rho": train_stats["rho"],
                "val_loss": val_stats["loss"],
                "val_mse": val_stats["mse"],
                "val_rho": val_stats["rho"],
            }
        )

        improved = val_stats["loss"] < best_val_loss
        if improved:
            best_val_loss = val_stats["loss"]
            torch.save(
                {
                    "state_dict": model.state_dict(),
                    "spec": spec,
                    "epoch": epoch,
                    "val_loss": best_val_loss,
                    # Readout-rescaling factor -- see the module docstring's
                    # "output channel scale imbalance" section. Required by
                    # export_weights.py to produce a PHYSICAL-unit readout
                    # weight matrix from this normalized-unit checkpoint.
                    # 読み出しの再スケール係数（モジュール docstring
                    # 「出力チャネルのスケール不均衡」節参照）。
                    # export_weights.py がこの正規化単位のチェックポイントから
                    # 物理単位の読み出し重み行列を作るのに必要。
                    "output_scale": output_scale.detach().cpu(),
                },
                best_path,
            )

        if epoch == 1 or epoch % 10 == 0 or epoch == args.epochs or improved:
            elapsed = time.time() - t0
            marker = " *" if improved else ""
            print(
                f"  epoch {epoch:4d}/{args.epochs} "
                f"train_loss={train_stats['loss']:.4f} (mse={train_stats['mse']:.5f} rho={train_stats['rho']:.4f}) "
                f"val_loss={val_stats['loss']:.4f} (mse={val_stats['mse']:.5f} rho={val_stats['rho']:.4f}) "
                f"{_report_metrics(args.target, val_stats)} "
                f"[{elapsed:6.1f}s]{marker}"
            )

    # Final report against the best checkpoint (not necessarily the last
    # epoch) so the numbers we print match what export_weights.py will read.
    # 最終報告はベストチェックポイント基準（必ずしも最終エポックではない）
    # -- export_weights.py が読む重みと表示する数値を一致させるため。
    ckpt = torch.load(best_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["state_dict"])
    final_train = _run_epoch(model, train_loader, optimizer, args.grad_clip, device, output_scale, train=False)
    final_val = _run_epoch(model, val_loader, optimizer, args.grad_clip, device, output_scale, train=False)
    print(
        f"[{args.target}] BEST checkpoint (epoch {ckpt['epoch']}): "
        f"train_loss={final_train['loss']:.4f} val_loss={final_val['loss']:.4f} "
        f"train:[{_report_metrics(args.target, final_train)}] "
        f"val:[{_report_metrics(args.target, final_val)}]"
    )

    _plot_history(args.target, history, args.out_dir)

    history_path = args.out_dir / f"{args.target}_history.json"
    history_path.write_text(json.dumps(history, indent=2))
    print(f"[{args.target}] saved best={best_path} history={history_path}")
    return 0


def _plot_history(target: str, history: list[dict], out_dir: Path) -> None:
    epochs = [h["epoch"] for h in history]
    fig, axes = plt.subplots(3, 1, figsize=(7, 9), sharex=True)

    axes[0].plot(epochs, [h["train_loss"] for h in history], label="train")
    axes[0].plot(epochs, [h["val_loss"] for h in history], label="val")
    axes[0].set_ylabel("loss (Eq.5)")
    axes[0].legend()
    axes[0].set_title(f"{target}: imitation loss")

    axes[1].plot(epochs, [h["train_mse"] for h in history], label="train")
    axes[1].plot(epochs, [h["val_mse"] for h in history], label="val")
    axes[1].set_ylabel("MSE")
    axes[1].legend()

    axes[2].plot(epochs, [h["train_rho"] for h in history], label="train")
    axes[2].plot(epochs, [h["val_rho"] for h in history], label="val")
    axes[2].set_ylabel("Pearson rho")
    axes[2].set_xlabel("epoch")
    axes[2].legend()

    fig.tight_layout()
    out_path = out_dir / f"{target}_loss.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[{target}] saved loss curve -> {out_path}")


if __name__ == "__main__":
    raise SystemExit(main())
