#!/usr/bin/env python3
"""
dataset.py -- windowed PyTorch Dataset for the SNN attitude estimator/
controller imitation-learning training (Stage 2b). Reads
`data/*_aligned.npz` (Stage 2a's `build_dataset.py` output) and produces
sliding-window (input, target, valid_mask) samples for BPTT training against
cuba_lif_torch.py's CubaLifLayer/LinearReadout + losses.py's imitation_loss().

SNN 姿勢推定器/制御器の模倣学習（Stage 2b）用の PyTorch Dataset。
`data/*_aligned.npz`（Stage 2a の `build_dataset.py` 出力）を読み、
cuba_lif_torch.py の CubaLifLayer/LinearReadout + losses.py の
imitation_loss() で BPTT 学習するためのスライディングウィンドウ
(入力, 目標, valid_mask) サンプルを作る。

=============================================================================
Input feature verification (task requirement -- do not skip)
入力特徴量の検証（タスク要件 -- 省略しないこと）
=============================================================================

1) Estimator's 6-axis IMU input: `gyro` vs `gyro_raw`, `accel` vs `accel_raw`?
   推定器の6軸IMU入力: `gyro` と `gyro_raw`、`accel` と `accel_raw` のどちらか?

   snn_estimator.cpp::predict() builds its `input[kInputDim]` directly from
   the `ImuData& imu` argument it is called with:
       input = { imu.gyro[0..2], imu.accel[0..2] * kAccelInputScale }
   (snn_estimator.cpp lines 105-110; kAccelInputScale = 1/kGravity, line 41).
   `ImuData` comes from the `sensor_imu` topic, published by imu_task.cpp's
   400Hz loop. imu_task.cpp::applyImuTransform() (lines 162-177) is the ONLY
   thing that runs on the sensor reading before `sensor_imu.publish(imu)`
   (line 827, comment: "Publish raw IMU data to topic" / "生IMUデータを
   トピックに発行") -- and applyImuTransform() does UNIT CONVERSION ONLY
   (accel g -> m/s^2; gyro already rad/s), no bias/scale correction (the
   boot-calibration bias is applied inside the ESKF's internal state via
   `g_estimator->applyCalibration()`, never rewritten back into the ImuData
   struct or the sensor_imu topic -- see imu_task.cpp's calibration-apply
   block, `g_estimator->applyCalibration(d.gyro_bias, d.accel_bias)`).

   So SnnEstimator::predict()'s ImuData is the UNCALIBRATED sensor reading,
   unit-converted only. protocol/spec/flight_log.yaml confirms this from the
   other end: gyro_x/accel_x are documented as "Estimator-input angular rate/
   acceleration X (body frame, post-filter)" (lines 254-280), while
   gyro_raw_x/accel_raw_x are documented as "Pre-filter raw ... vehicle では
   gyro_x と同値" / "firmware/vehicle has no IMU-side LPF, so raw == filtered
   here" (lines 284-315) -- i.e. on THIS firmware target the two are defined
   to be identical, and `gyro`/`accel` (not the `_raw` variants) are the ones
   explicitly named as "the estimator's input". Verified numerically too:
   `np.allclose(d["gyro"], d["gyro_raw"])` and `accel`/`accel_raw` are exact
   matches (max abs diff 0.0) in every one of the 5 aligned npz files.

   Conclusion: use `gyro`/`accel` (either column would give byte-identical
   results here, but `gyro`/`accel` is the semantically-correct name to read
   for future scenarios where a real IMU-side filter might exist).

2) Controller's setpoint inputs: does `CommandSetpoint.roll/pitch/yaw/throttle`
   match `pilot_rpy`/`pilot_throttle`'s scale/meaning?
   制御器のセットポイント入力: `CommandSetpoint.roll/pitch/yaw/throttle` は
   `pilot_rpy`/`pilot_throttle` と同じスケール・意味か?

   snn_controller.cpp::compute() reads `setpoint.roll/pitch/yaw/throttle`
   straight off its `const CommandSetpoint& setpoint` argument (lines
   111-115), no extra scaling. control_task.cpp (line 272) passes
   `sf::command_setpoint.latest()` verbatim into `controller->compute(state,
   setpoint, dt)`. Meanwhile sf_telemetry/data_stream.cpp (lines 318-325),
   the code that produces the "pilot" (0x42) log stream build_dataset.py's
   `pilot_rpy`/`pilot_throttle` are read from, ALSO reads
   `command_setpoint.latest()` and copies `.throttle/.roll/.pitch/.yaw`
   VERBATIM into the wire struct -- the exact same Pub-Sub value, not a
   re-derived or re-scaled one. So the two consumers (the real-time
   controller and the telemetry logger) see byte-identical CommandSetpoint
   values every cycle; there is no separate "stick scaling" path to
   reconcile. This also matches by construction: command.cpp's
   normalizeThrottle()/normalizeAxis() (lines 76-94) produce throttle in
   [0..1] and roll/pitch/yaw in [-1..1], exactly the units
   `flight_log.yaml`'s pilot.csv documents (lines 594-613) and exactly
   data_types.hpp's CommandSetpoint doc comments (lines 180-190).

   Conclusion: `pilot_rpy` (order [roll,pitch,yaw], -1..1) and
   `pilot_throttle` (0..1) can be used AS-IS as the controller's
   setpoint.roll/pitch/yaw/throttle inputs -- no conversion needed.

3) Controller's `state.angular_rate` input -- NOT explicitly asked for by
   the task, but required to build a correct 10-value input vector, so
   documented here too. snn_controller.cpp reads `state.angular_rate[0..2]`
   from `StateEstimate`, which for the ESKF teacher (estimator.type=0, the
   classical estimator whose PID output build_dataset.py's `thrust`/`torque`
   columns are) is `core_.getAngularRate()` = `gyro_raw - bg_`, i.e.
   BIAS-CORRECTED body rate (eskf_core.cpp predict(), line 243/247;
   eskf_estimator.cpp convertState(), line 310-311) -- NOT the raw `gyro`
   used for the estimator's input above. Stage 2a's original column set did
   not export the ESKF's `gyro_bias` (only quat/euler were pulled from the
   "attitude" stream); this was added to build_dataset.py in this Stage 2b
   pass (see its diff / column_doc) so `angular_rate = gyro - gyro_bias` can
   be reconstructed exactly here instead of approximated with raw gyro. The
   bias is small (<1 deg/s peak here) but non-zero, so this is a real
   (if minor) correctness fix, not a no-op.

NOTE on a losses.py numerical edge case found while wiring this dataset up
to train.py (not fixed here -- see train.py's module docstring for the
full writeup and why the fix lives there instead of here): several
scenarios have channels that are bit-for-bit CONSTANT for their entire
duration (e.g. alt_flight's attitude/torque, pos_roll/pos_pitch's
off-axis torque), which is fine for THIS module (plain windowing, no
gradients computed here) but interacts badly with
losses.imitation_loss()'s Pearson term once training starts.
losses.py との数値上のエッジケースについては train.py の docstring 参照
（本モジュールは単なるウィンドウ化で勾配計算をしないため無関係だが、
学習開始後に losses.imitation_loss() のピアソン項と相互作用する）。

=============================================================================
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset

# Mirrors sf_math.hpp's `constexpr float kGravity = 9.80665f;` -- duplicated
# here (PC-side Python, not sharable with the C++ build) so the estimator's
# accel input uses the SAME g-normalization snn_estimator.cpp applies
# (kAccelInputScale = 1/kGravity, snn_estimator.cpp line 41).
# sf_math.hpp の `constexpr float kGravity = 9.80665f;` に対応（C++ビルドと
# 共有できないPC側Pythonなのでここで複製）。推定器の加速度入力を
# snn_estimator.cpp と同じg正規化（kAccelInputScale = 1/kGravity）にする。
GRAVITY_MPS2 = 9.80665

_DEFAULT_DATA_DIR = Path(__file__).resolve().parent / "data"

# Scenario-level train/val split (task spec): stab_flight is held out
# ENTIRELY for validation; the other 4 scenarios train. Splitting by scenario
# (not by row) keeps every window's time-series continuity intact and gives
# a val set the network never saw any part of during training.
# シナリオ単位のtrain/val分割: stab_flight を検証用に完全分離、残り4シナリオで
# 学習。行単位でなくシナリオ単位に分けることで、各ウィンドウの時系列連続性を
# 保ち、検証セットは学習中に一切見ていないものになる。
TRAIN_SCENARIOS = ("acro_flight", "alt_flight", "pos_roll", "pos_pitch")
VAL_SCENARIOS = ("stab_flight",)
ALL_SCENARIOS = TRAIN_SCENARIOS + VAL_SCENARIOS


def _load_npz(name: str, data_dir: Path) -> dict:
    """Load one `<name>_aligned.npz`, returning every array except the
    string-metadata ones (`column_doc`, `scenario`).
    `<name>_aligned.npz` を1つ読み込み、文字列メタデータ（`column_doc`,
    `scenario`）以外の全配列を返す。
    """
    path = data_dir / f"{name}_aligned.npz"
    with np.load(path) as npz:
        return {k: npz[k] for k in npz.files if k not in ("column_doc", "scenario")}


def estimator_features(d: dict) -> tuple[np.ndarray, np.ndarray]:
    """(input, target) full-length arrays for the ESTIMATOR network.

    input (N,6) = [gyro_x,y,z (rad/s), accel_x,y,z/g] -- matches
    snn_estimator.cpp::predict()'s `input[kInputDim]` exactly (see module
    docstring item 1). No further normalization (gyro already rad/s, matches
    the C++ side's direct pass-through).

    target (N,3) = euler [roll,pitch,yaw] rad, the ESKF estimate that IS
    Stage 2a's teacher (estimator.type=0 unchanged during data collection).
    Trained directly in radians -- see export_weights.py's docstring for how
    this interacts with the C++ readout's kAttitudeReadoutScale placeholder.
    推定器ネットワーク用の(入力, 目標)フル長配列。仕様はモジュール docstring
    項目1参照。目標はラジアン直接回帰 -- C++読み出し側の
    kAttitudeReadoutScale placeholder との関係は export_weights.py 参照。
    """
    gyro = d["gyro"].astype(np.float32)
    accel_g = d["accel"].astype(np.float32) / np.float32(GRAVITY_MPS2)
    x = np.concatenate([gyro, accel_g], axis=1)
    y = d["euler"].astype(np.float32)
    return x, y


def controller_features(d: dict) -> tuple[np.ndarray, np.ndarray]:
    """(input, target) full-length arrays for the CONTROLLER network.

    input (N,10) = [angular_rate_x,y,z (rad/s, bias-corrected),
                     euler_roll,pitch,yaw (rad),
                     setpoint_roll,pitch,yaw (-1..1), setpoint_throttle (0..1)]
    -- matches snn_controller.cpp::compute()'s `input[kInputDim]` order
    exactly (state.angular_rate, euler, setpoint.roll/pitch/yaw/throttle).
    See module docstring items 2-3 for the setpoint/angular_rate
    verification.

    target (N,4) = [thrust N, torque_roll, torque_pitch, torque_yaw N*m] --
    matches ControlOutput{thrust, torque[0..2]} order in
    snn_controller.cpp::compute(). Direct physical units; see
    export_weights.py's docstring for how this interacts with the C++
    readout's hover-thrust-prior + gain placeholders.
    制御器ネットワーク用の(入力, 目標)フル長配列。仕様はモジュール docstring
    項目2-3参照。目標は物理単位直接回帰 -- C++読み出し側の
    ホバー推力事前値+ゲイン placeholder との関係は export_weights.py 参照。
    """
    angular_rate = (d["gyro"] - d["gyro_bias"]).astype(np.float32)
    euler = d["euler"].astype(np.float32)
    setpoint = np.concatenate(
        [d["pilot_rpy"].astype(np.float32), d["pilot_throttle"].astype(np.float32)[:, None]],
        axis=1,
    )
    x = np.concatenate([angular_rate, euler, setpoint], axis=1)
    y = np.concatenate(
        [d["thrust"].astype(np.float32)[:, None], d["torque"].astype(np.float32)],
        axis=1,
    )
    return x, y


_FEATURE_FNS = {"estimator": estimator_features, "controller": controller_features}


class WindowedImitationDataset(Dataset):
    """Sliding-window (input, target, valid_mask) samples drawn from one or
    more scenarios, never crossing a scenario boundary (build_dataset.py
    writes one npz per scenario; each is treated as one continuous series --
    task spec).

    Windowing: window-length `window` slices of the input, stride `stride`
    apart, starting fresh (hidden state reset) at each window (the training
    loop is responsible for calling `layer.init_state()` per window -- see
    train.py).

    Time shift: paper-style delay compensation. For a window starting at
    absolute row `start`, sample t in [0, window) pairs input row
    `start+t` with target row `start+t+shift` (shift steps INTO THE FUTURE
    relative to the input). Rows that don't exist yet (past the scenario's
    end) are zero-padded and marked invalid via `valid_mask`, which
    `losses.imitation_loss()`'s `valid_mask` argument excludes from both the
    MSE and the Pearson-correlation statistics (not just zeroed).

    複数シナリオから、シナリオ境界をまたがないスライディングウィンドウの
    (入力, 目標, valid_mask) サンプルを作る。ウィンドウ長・ストライド・時間
    シフトの意味は上記英語部参照。存在しない範囲（シナリオ終端超過）はゼロ
    パディングし valid_mask で除外する。
    """

    def __init__(
        self,
        scenarios,
        target: str,
        window: int = 400,
        stride: int = 100,
        shift: int = 6,
        data_dir: Path | str = _DEFAULT_DATA_DIR,
    ):
        if target not in _FEATURE_FNS:
            raise ValueError(f"target must be one of {list(_FEATURE_FNS)}, got {target!r}")
        if not (window > shift >= 0):
            raise ValueError(f"require window > shift >= 0, got window={window} shift={shift}")

        feature_fn = _FEATURE_FNS[target]
        data_dir = Path(data_dir)

        self.target = target
        self.window = window
        self.shift = shift
        self.scenarios = tuple(scenarios)

        self._inputs: list[torch.Tensor] = []
        self._targets: list[torch.Tensor] = []  # shift-padded with zeros
        self._valid: list[torch.Tensor] = []  # shift-padded with 0 (invalid)
        self._index: list[tuple[int, int]] = []  # (scenario slot, window start)

        for scen in scenarios:
            d = _load_npz(scen, data_dir)
            x, y = feature_fn(d)
            # Defensive: build_dataset.py restricts to the armed flight
            # window (dropna(subset=["thrust"])) and we've verified
            # numerically that none of the columns used above contain NaN
            # in the current 5 scenarios -- fail loudly rather than
            # silently corrupting training if a future scenario violates
            # that assumption (NaN inside a window would poison the
            # recurrent hidden state for every later timestep in that
            # window, which a per-timestep valid_mask cannot fix).
            # 防御的チェック: build_dataset.py は ARM 中の飛行区間に絞って
            # おり、現行5シナリオでは上記の列にNaNが無いことを数値確認済み。
            # 将来のシナリオがこの前提を破ったら黙らず失敗させる（ウィンドウ
            # 内のNaNは再帰隠れ状態を通じて以降の全タイムステップを汚染し、
            # タイムステップ単位のvalid_maskでは救えないため）。
            if np.isnan(x).any() or np.isnan(y).any():
                raise ValueError(
                    f"scenario '{scen}' ({target} features) contains NaN -- "
                    "expected none; check build_dataset.py's flight-window restriction"
                )

            n = y.shape[0]
            if n < window:
                raise ValueError(f"scenario '{scen}' has {n} rows, shorter than window={window}")

            d_out = y.shape[1]
            y_padded = np.concatenate([y, np.zeros((shift, d_out), dtype=np.float32)], axis=0)
            valid_padded = np.concatenate(
                [np.ones(n, dtype=np.float32), np.zeros(shift, dtype=np.float32)]
            )

            slot = len(self._inputs)
            self._inputs.append(torch.from_numpy(np.ascontiguousarray(x)))
            self._targets.append(torch.from_numpy(y_padded))
            self._valid.append(torch.from_numpy(valid_padded))

            last_start = n - window
            for start in range(0, last_start + 1, stride):
                self._index.append((slot, start))

        if not self._index:
            raise ValueError(f"no windows produced for scenarios={scenarios} (window={window})")

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, idx: int):
        slot, start = self._index[idx]
        w, s = self.window, self.shift
        x_win = self._inputs[slot][start : start + w]
        y_win = self._targets[slot][start + s : start + s + w]
        v_win = self._valid[slot][start + s : start + s + w]
        return x_win, y_win, v_win

    def all_targets(self) -> torch.Tensor:
        """Every VALID (non-padding) target row across every scenario in
        this dataset, concatenated -- e.g. for train.py to compute
        per-channel normalization statistics from the TRAINING split only.
        Shape (total_valid_rows, target_dim).
        このデータセットの全シナリオから有効な（パディングでない）目標行を
        連結して返す -- train.py が学習分割のみからチャネル別正規化統計量を
        計算する用途等。
        """
        parts = [t[m.bool()] for t, m in zip(self._targets, self._valid)]
        return torch.cat(parts, dim=0)

    @property
    def input_dim(self) -> int:
        return self._inputs[0].shape[1]

    @property
    def target_dim(self) -> int:
        return self._targets[0].shape[1]


if __name__ == "__main__":
    # Quick sanity self-test (no training): print dataset sizes for both
    # targets, both splits. `./venv/bin/python dataset.py`
    # 簡易セルフテスト（学習なし）: 両ターゲット・両分割のデータセットサイズを表示。
    for target in ("estimator", "controller"):
        train_ds = WindowedImitationDataset(TRAIN_SCENARIOS, target)
        val_ds = WindowedImitationDataset(VAL_SCENARIOS, target)
        x0, y0, v0 = train_ds[0]
        print(
            f"[{target}] train_windows={len(train_ds)} val_windows={len(val_ds)} "
            f"input_dim={train_ds.input_dim} target_dim={train_ds.target_dim} "
            f"sample_shapes=x{tuple(x0.shape)} y{tuple(y0.shape)} v{tuple(v0.shape)} "
            f"valid_frac={v0.mean().item():.3f}"
        )
