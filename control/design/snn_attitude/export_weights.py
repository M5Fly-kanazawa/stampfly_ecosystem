#!/usr/bin/env python3
"""
export_weights.py -- export a trained checkpoint (`runs/<target>_best.pt`,
written by train.py) into:
  1. `runs/<target>_weights.npz` -- an archival numpy dump of every
     parameter array (w_in, w_rec if recurrent, w_out, tau_mem, tau_syn,
     theta), for re-export or PC-side inference experiments.
  2. `runs/<target>_weights_draft.hpp` -- the same arrays as C++
     `constexpr float` arrays, row-major in the layout
     `CubaLifLayer::setWeights()`/`LinearReadout::setWeights()` expect
     ([n_neurons,n_in] / [n_neurons,n_neurons] / [n_out,n_neurons] -- see
     their headers), for Stage 3 to wire up as snn_weight_init.hpp's
     replacement.

学習済みチェックポイント（train.py が書く `runs/<target>_best.pt`）を
(1) 全パラメータ配列のアーカイブ用 .npz と (2) C++ constexpr float 配列の
ドラフトヘッダへ書き出す。

Usage / 使い方:
    ./venv/bin/python export_weights.py --target estimator
    ./venv/bin/python export_weights.py --target controller
    ./venv/bin/python export_weights.py --target estimator --target controller

=============================================================================
TWO KNOWN GAPS for whoever wires these weights into the C++ Stage1 skeleton
(both documented here, in the generated .hpp's header comment, and in this
task's report -- NEITHER is fixed here; both are explicitly out of this
task's scope per the task spec / firmware-is-off-limits constraint).
このスクリプトの出力をC++ Stage1骨格に配線する人向けの、既知の2つの
ギャップ（ここでは修正しない -- タスク仕様上のスコープ外 / firmware
変更禁止のため）。
=============================================================================

GAP 1 -- per-neuron tau/theta setter does not exist yet (task-spec-mandated
TODO, see the task instructions verbatim). `CubaLifLayer::setWeights()`
exists, but the only tau/theta setter is `setUniformParams(tau_mem, tau_syn,
theta)` (ONE value applied to every neuron -- cuba_lif_layer.hpp). This
export produces PER-NEURON tau_mem/tau_syn/theta arrays (trained values
differ neuron-to-neuron), which `setUniformParams()` cannot apply. Stage 3
must add a per-neuron setter (e.g. `setNeuronParams(const float* tau_mem,
const float* tau_syn, const float* theta)`) to CubaLifLayer before this
header's kTauMem/kTauSyn/kTheta arrays can be wired up.

GAP 2 -- the C++ Stage1 readout has its OWN placeholder post-scale
constants, which will DOUBLE-SCALE these exported weights if left as-is:
  - snn_estimator.cpp: `roll = y[0] * kAttitudeReadoutScale` (0.2f)
  - snn_controller.cpp: `thrust = kHoverThrustN + y[0] * kThrustReadoutGain`
    (hover-thrust prior 0.363N ADDED, then *0.05), `torque = y[i] *
    kTorqueReadoutGain` (*1e-3)
This export's `w_out` is ALREADY in physical units end-to-end: training
regressed roll/pitch/yaw [rad] / thrust [N] / torque [N*m] DIRECTLY (see
dataset.py/train.py), and train.py's `output_scale` un-normalization (see
its docstring) is folded into `w_out` below (`w_out_physical = w_out_raw *
output_scale[:, None]`) so `y = w_out @ spikes` already equals the physical
target. If Stage 3 calls `readout_.setWeights(w_out)` and leaves
snn_estimator.cpp/snn_controller.cpp's post-scale constants in place
UNCHANGED, every output would be additionally scaled (attitude x0.2,
torque x1e-3, thrust x0.05 PLUS offset by +0.363N) -- wrong. Stage 3 must
either (a) remove/neutralize those C++-side constants (scale=1, gain=1,
hover-prior=0) when wiring trained weights, or (b) pre-divide the relevant
rows of this export's `w_out` by the matching C++ constant before calling
setWeights() -- NOT done here because (b) cannot exactly cancel the
controller's ADDITIVE `kHoverThrustN` term via a weight rescaling alone (a
linear no-bias readout cannot represent a constant offset by scaling), so
picking (a) vs (b) is a Stage 3 design decision, not this script's to make.
firmware/vehicle/ is explicitly off-limits for this task, so this script
only documents the gap -- it does not touch snn_estimator.cpp/
snn_controller.cpp.
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path

import numpy as np
import torch

from cuba_lif_torch import CubaLifLayer, LinearReadout

_THIS_DIR = Path(__file__).resolve().parent
_DEFAULT_RUNS_DIR = _THIS_DIR / "runs"


def _load_checkpoint(target: str, runs_dir: Path) -> dict:
    path = runs_dir / f"{target}_best.pt"
    if not path.exists():
        raise FileNotFoundError(f"{path} not found -- run train.py --target {target} first")
    return torch.load(path, map_location="cpu", weights_only=False)


def _extract_arrays(ckpt: dict) -> dict:
    """Rebuild the trained CubaLifLayer/LinearReadout from `ckpt["spec"]` +
    `ckpt["state_dict"]` and pull out plain numpy arrays in the C++
    row-major layout ([n_neurons,n_in] / [n_neurons,n_neurons] /
    [n_out,n_neurons] -- PyTorch's default nn.Parameter storage order
    already matches this exactly, see cuba_lif_torch.py's module
    docstring, so no transpose is needed).

    tau_mem/tau_syn/theta come from `hidden.effective_tau_theta()` (NOT the
    raw `.tau_mem` etc. Parameters directly) so the fixed-integrator subset
    (n_fixed>0, controller only) is guaranteed pinned at exactly 1.0 in the
    export regardless of any float drift in the stored Parameter (in
    practice these never move: cuba_lif_torch.py's `effective_tau_theta()`
    masks them out of the forward pass every call, so their gradient -- and
    hence Adam's update -- is exactly zero every step).

    `w_out` is rescaled by `ckpt["output_scale"]` (train.py's per-channel
    normalization factor, see its docstring's "output channel scale
    imbalance" section) so the exported readout maps spikes directly to
    PHYSICAL units (rad / N / N*m) -- see this module's GAP 2 for how that
    interacts with the C++ Stage1 readout's OWN placeholder scale
    constants.
    """
    spec = ckpt["spec"]
    hidden = CubaLifLayer(
        n_in=spec["n_in"], n_neurons=spec["n_neurons"], recurrent=spec["recurrent"], n_fixed=spec["n_fixed"]
    )
    readout = LinearReadout(spec["n_neurons"], spec["n_out"])
    hidden_sd = {k[len("hidden.") :]: v for k, v in ckpt["state_dict"].items() if k.startswith("hidden.")}
    readout_sd = {k[len("readout.") :]: v for k, v in ckpt["state_dict"].items() if k.startswith("readout.")}
    hidden.load_state_dict(hidden_sd)
    readout.load_state_dict(readout_sd)

    tau_mem, tau_syn, theta = hidden.effective_tau_theta()
    output_scale = ckpt["output_scale"].to(torch.float32)  # (n_out,)
    w_out_physical = readout.weight.detach() * output_scale.unsqueeze(1)  # (n_out, n_neurons)

    arrays = {
        "w_in": hidden.w_in.detach().numpy().astype(np.float32),  # (n_neurons, n_in)
        "w_out": w_out_physical.numpy().astype(np.float32),  # (n_out, n_neurons), PHYSICAL units
        "tau_mem": tau_mem.detach().numpy().astype(np.float32),  # (n_neurons,)
        "tau_syn": tau_syn.detach().numpy().astype(np.float32),  # (n_neurons,)
        "theta": theta.detach().numpy().astype(np.float32),  # (n_neurons,)
        "output_scale": output_scale.numpy().astype(np.float32),  # (n_out,), for reference/re-export only
    }
    if spec["recurrent"]:
        arrays["w_rec"] = hidden.w_rec.detach().numpy().astype(np.float32)  # (n_neurons, n_neurons)
    return arrays


def _write_npz(target: str, arrays: dict, spec: dict, ckpt: dict, out_path: Path) -> None:
    payload = dict(arrays)
    payload["spec_n_in"] = np.array(spec["n_in"])
    payload["spec_n_neurons"] = np.array(spec["n_neurons"])
    payload["spec_n_out"] = np.array(spec["n_out"])
    payload["spec_recurrent"] = np.array(spec["recurrent"])
    payload["spec_n_fixed"] = np.array(spec["n_fixed"])
    payload["train_epoch"] = np.array(ckpt["epoch"])
    payload["train_val_loss"] = np.array(ckpt["val_loss"])
    np.savez(out_path, **payload)


def _format_c_array(name: str, values: np.ndarray, per_line: int = 8) -> str:
    flat = values.ravel(order="C")  # row-major -- matches setWeights()'s expected flat layout
    lines = []
    for i in range(0, len(flat), per_line):
        chunk = flat[i : i + per_line]
        lines.append("    " + ", ".join(f"{v:.8f}f" for v in chunk) + ",")
    body = "\n".join(lines)
    return f"constexpr float {name}[{len(flat)}] = {{\n{body}\n}};"


def _write_draft_hpp(target: str, arrays: dict, spec: dict, ckpt: dict, out_path: Path) -> None:
    n_in, n_neurons, n_out = spec["n_in"], spec["n_neurons"], spec["n_out"]
    now = datetime.datetime.now().isoformat(timespec="seconds")
    parts = [
        "/*",
        " * AUTO-GENERATED DRAFT -- control/design/snn_attitude/export_weights.py",
        f" * target={target}  generated={now}",
        f" * source checkpoint: runs/{target}_best.pt "
        f"(epoch={ckpt['epoch']}, val_loss={ckpt['val_loss']:.4f})",
        " *",
        " * NOT WIRED UP -- see this file's header comment (and",
        " * export_weights.py's module docstring, and the Stage 2b task",
        " * report) for TWO KNOWN GAPS before this can replace",
        " * snn_weight_init.hpp's placeholder weights:",
        " *   GAP 1: CubaLifLayer has no per-neuron tau/theta setter yet",
        " *          (only setUniformParams(), one value for ALL neurons).",
        " *          Add e.g. setNeuronParams(tau_mem[], tau_syn[], theta[])",
        " *          before wiring kTauMem/kTauSyn/kTheta below.",
        " *   GAP 2: snn_estimator.cpp/snn_controller.cpp apply their OWN",
        " *          placeholder post-readout scale (kAttitudeReadoutScale/",
        " *          kThrustReadoutGain/kTorqueReadoutGain/kHoverThrustN).",
        " *          kWOut below is ALREADY in physical units (rad/N/N*m) --",
        " *          those C++ constants must be neutralized (or kWOut",
        " *          further pre-divided) or every output will be scaled",
        " *          twice. Firmware is off-limits for Stage 2b, so this is",
        " *          intentionally left as a Stage 3 decision.",
        " */",
        "#pragma once",
        "",
        "namespace sf {",
        "namespace snn {",
        f"namespace weights_{target}_draft {{",
        "",
        f"constexpr int kNIn = {n_in};",
        f"constexpr int kNNeurons = {n_neurons};",
        f"constexpr int kNOut = {n_out};",
        "",
        _format_c_array("kWIn", arrays["w_in"]),
        "",
    ]
    if "w_rec" in arrays:
        parts.append(_format_c_array("kWRec", arrays["w_rec"]))
        parts.append("")
    parts.append("// Physical units (rad / N / N*m) -- see GAP 2 above before wiring.")
    parts.append(_format_c_array("kWOut", arrays["w_out"]))
    parts.append("")
    parts.append("// Per-neuron -- see GAP 1 above before wiring (setUniformParams() cannot apply these).")
    parts.append(_format_c_array("kTauMem", arrays["tau_mem"]))
    parts.append(_format_c_array("kTauSyn", arrays["tau_syn"]))
    parts.append(_format_c_array("kTheta", arrays["theta"]))
    parts.append("")
    parts.append(f"}}  // namespace weights_{target}_draft")
    parts.append("}  // namespace snn")
    parts.append("}  // namespace sf")
    parts.append("")
    out_path.write_text("\n".join(parts))


def export_one(target: str, runs_dir: Path) -> None:
    ckpt = _load_checkpoint(target, runs_dir)
    arrays = _extract_arrays(ckpt)
    spec = ckpt["spec"]

    npz_path = runs_dir / f"{target}_weights.npz"
    _write_npz(target, arrays, spec, ckpt, npz_path)

    hpp_path = runs_dir / f"{target}_weights_draft.hpp"
    _write_draft_hpp(target, arrays, spec, ckpt, hpp_path)

    print(
        f"[{target}] epoch={ckpt['epoch']} val_loss={ckpt['val_loss']:.4f} "
        f"output_scale={arrays['output_scale'].tolist()} -> {npz_path.name}, {hpp_path.name}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--target", choices=["estimator", "controller"], action="append", dest="targets",
        help="repeatable; default: both",
    )
    parser.add_argument("--runs-dir", type=Path, default=_DEFAULT_RUNS_DIR)
    args = parser.parse_args()
    targets = args.targets or ["estimator", "controller"]

    for target in targets:
        export_one(target, args.runs_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
