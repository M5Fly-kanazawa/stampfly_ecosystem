#!/usr/bin/env python3
"""
build_dataset.py -- SILS 飛行ログ(.sflog.zip)を SNN 姿勢推定・制御の模倣学習
(Stage 2b) 向けの numpy 配列（.npz）へ変換する。

build_dataset.py -- Convert SILS flight logs (.sflog.zip) into numpy arrays
(.npz) for the SNN attitude estimation/control imitation-learning work
(Stage 2b).

Stage 2a のスコープ: データ収集とデータセット化のみ。学習ループ（BPTT・損失関数）は
実装しない（Stage 2b で別途実装）。
Stage 2a scope: data collection and dataset packaging only. The training loop
(BPTT / loss functions) is NOT implemented here (that is Stage 2b).

Usage / 使い方:
    control/design/snn_attitude/venv/bin/python build_dataset.py
    control/design/snn_attitude/venv/bin/python build_dataset.py data/acro_flight.sflog.zip
    control/design/snn_attitude/venv/bin/python build_dataset.py --data-dir data --out-dir data

With no positional args, every "*.sflog.zip" directly under --data-dir
(default: control/design/snn_attitude/data/) is converted. Each input
"<name>.sflog.zip" produces "<name>_aligned.npz" in --out-dir.
位置引数なしなら --data-dir 直下の全 "*.sflog.zip" を変換する。各入力
"<name>.sflog.zip" は --out-dir に "<name>_aligned.npz" を1つ書く。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# lib/sflog is a plain module directory (not pip-installed) -- add lib/ to
# sys.path so `import sflog` resolves regardless of the caller's cwd. See
# README.md "venv セットアップ" for the equivalent PYTHONPATH instructions.
# lib/sflog は pip パッケージ化されていない素のモジュールディレクトリ -- 呼び出し元の
# cwd に関わらず `import sflog` が通るよう lib/ を sys.path に追加する。PYTHONPATH で
# 同じことをする手順は README.md「venv セットアップ」参照。
_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "lib"))

import sflog  # noqa: E402  (import after sys.path setup, see above)

_DEFAULT_DATA_DIR = Path(__file__).resolve().parent / "data"

# Streams merged onto the 400 Hz IMU base by sflog.aligned(). See
# lib/sflog/align.py's docstring for the exact merge rule (lockstep streams
# join on `seq`; others merge_asof on timestamp_us).
# 400Hz IMU を基準に sflog.aligned() が結合するストリーム。正確な結合規則は
# lib/sflog/align.py の docstring 参照（ロックステップ系は `seq` 結合、それ以外は
# timestamp_us での merge_asof）。
_STREAMS = ["attitude", "posvel", "rate_ref", "motor", "ctrl_output", "ctrl_ref", "pilot"]


def _euler_from_quat_ned(qw, qx, qy, qz):
    """roll/pitch/yaw [rad] from a body(FRD)->NED quaternion (w,x,y,z),
    vectorized over numpy arrays. Copied verbatim from
    simulator/sils/viz/render_video.py so this dataset uses the SAME
    convention as the project's other quaternion->Euler consumers (standard
    aerospace 3-2-1 extraction; pitch clipped to arcsin's domain for
    +-90 deg round-off).
    機体(FRD)→NED のクォータニオンから roll/pitch/yaw [rad]（numpy 配列対応）。
    simulator/sils/viz/render_video.py からそのまま複製 -- 本データセットも
    プロジェクト内の他のクォータニオン->オイラー角消費者と同じ規約を使うため
    （標準的な航空3-2-1抽出、pitch は arcsin 定義域にクランプ）。
    """
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def build_one(zip_path: Path, out_path: Path) -> dict:
    """Load one *.sflog.zip, align it to 400 Hz IMU time, restrict to the
    control-active (armed/flying) window, and write a compact .npz.
    Returns a small summary dict for the caller to print/report.
    1つの *.sflog.zip を読み、400Hz IMU 時刻に整列し、制御ループが動いている
    （ARM〜DISARM の飛行）区間に絞って .npz を書く。呼び出し元が表示・報告に
    使う小さな summary dict を返す。
    """
    # Path.stem only strips the LAST suffix, so "acro_flight.sflog.zip".stem
    # is "acro_flight.sflog" -- strip the full ".sflog.zip" here once so the
    # embedded scenario name and the printed summary agree.
    # Path.stem は最後の拡張子しか除かない（"acro_flight.sflog.zip".stem は
    # "acro_flight.sflog"）ため、ここで ".sflog.zip" 全体を一度に取り除き、
    # 埋め込むシナリオ名と表示するサマリを一致させる。
    scenario_name = zip_path.name.removesuffix(".sflog.zip")

    log = sflog.load(zip_path)
    df = sflog.aligned(log, base="imu", streams=_STREAMS)

    # The control loop (rate_ref/motor/ctrl_output -- all LOCKSTEP_STREAMS
    # joined on `seq`) only exists while armed; rows outside that window
    # merge in as NaN. `thrust` (ctrl_output) is a reliable "control loop
    # was running this cycle" flag -- restrict the dataset to that window
    # so a static, motors-off ground/boot period doesn't dilute the
    # imitation-learning targets.
    # 制御ループ（rate_ref/motor/ctrl_output -- 全て `seq` 結合のロックステップ系）は
    # ARM 中のみ存在し、その外側の行は NaN で結合される。`thrust`（ctrl_output）は
    # 「この周期は制御ループが動いていた」の信頼できる指標 -- 静止した地上/起動区間が
    # 模倣学習の教師データを薄めないよう、この区間に絞る。
    flight = df.dropna(subset=["thrust"]).reset_index(drop=True)

    qw, qx, qy, qz = (flight[c].to_numpy(dtype=np.float64) for c in ("quat_w", "quat_x", "quat_y", "quat_z"))
    roll, pitch, yaw = _euler_from_quat_ned(qw, qx, qy, qz)

    t_s = (flight["timestamp_us"].to_numpy(dtype=np.float64) - flight["timestamp_us"].iloc[0]) * 1e-6

    arrays = dict(
        # -- time / bookkeeping --
        t_s=t_s,
        seq=flight["seq"].to_numpy(dtype=np.int64),
        # -- IMU raw 6-axis (gyro rad/s, accel m/s^2) --
        gyro_raw=flight[["gyro_raw_x", "gyro_raw_y", "gyro_raw_z"]].to_numpy(dtype=np.float64),
        accel_raw=flight[["accel_raw_x", "accel_raw_y", "accel_raw_z"]].to_numpy(dtype=np.float64),
        # -- IMU filtered (bias/scale corrected, still pre-ESKF-fusion) --
        gyro=flight[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(dtype=np.float64),
        accel=flight[["accel_x", "accel_y", "accel_z"]].to_numpy(dtype=np.float64),
        # -- ESKF-estimated attitude (body FRD -> NED), quaternion + Euler --
        quat=flight[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy(dtype=np.float64),
        euler=np.stack([roll, pitch, yaw], axis=1),
        # -- ESKF-estimated gyro bias [rad/s] -- added for Stage 2b: the PID
        # teacher's rate-loop input is core_.getAngularRate() = gyro_raw - bg_
        # (bias-corrected, see eskf_core.cpp predict()), NOT the raw/filtered
        # gyro above. Stage 2a's original column set omitted this (only
        # quat/euler were pulled from the "attitude" stream); added here so
        # dataset.py can reconstruct the same bias-corrected rate the real
        # teacher controller saw, instead of approximating it with raw gyro.
        # -- ESKF推定ジャイロバイアス[rad/s] -- Stage2bで追加: PID教師のレート
        # ループ入力は core_.getAngularRate() = gyro_raw - bg_（バイアス補正済み、
        # eskf_core.cpp predict()参照）であり、上のgyro（生値）ではない。
        # Stage2aの元の列集合はこれを含んでいなかった（"attitude"ストリームから
        # quat/eulerのみ取得）。ここで追加し、dataset.py が生gyroでの近似ではなく
        # 実際の教師コントローラが見ていたバイアス補正済みレートを再構成できる
        # ようにする。
        gyro_bias=flight[["gyro_bias_x", "gyro_bias_y", "gyro_bias_z"]].to_numpy(dtype=np.float64),
        # -- PID controller output (post rate-PID, pre-mixer): thrust [N], torque [N*m] --
        thrust=flight["thrust"].to_numpy(dtype=np.float64),
        torque=flight[["torque_roll", "torque_pitch", "torque_yaw"]].to_numpy(dtype=np.float64),
        # -- mixer output: per-motor duty [0..1], order FR/RR/RL/FL --
        duty=flight[["duty_FR", "duty_RR", "duty_RL", "duty_FL"]].to_numpy(dtype=np.float64),
        # -- targets: inner-loop rate setpoint [rad/s] (ACRO's controlled state) --
        rate_ref=flight[["rate_ref_roll", "rate_ref_pitch", "rate_ref_yaw"]].to_numpy(dtype=np.float64),
        # -- targets: outer-loop attitude setpoint [rad] (STABILIZE/POS_HOLD; NaN in pure ACRO) --
        angle_ref=flight[["angle_ref_roll", "angle_ref_pitch"]].to_numpy(dtype=np.float64),
        # -- targets: outer-loop thrust command [N] (pre inner-loop, 50 Hz hold) --
        total_thrust_ref=flight["total_thrust"].to_numpy(dtype=np.float64),
        # -- targets: raw pilot stick, normalized (throttle 0..1, roll/pitch/yaw -1..1) --
        pilot_throttle=flight["throttle"].to_numpy(dtype=np.float64),
        pilot_rpy=flight[["roll", "pitch", "yaw"]].to_numpy(dtype=np.float64),
        # -- flight mode enum (see protocol/spec/flight_log.yaml ctrl_ref.flight_mode) --
        flight_mode=flight["flight_mode"].to_numpy(dtype=np.float64),
    )

    # Column-name documentation travels WITH the array (np.savez can't hold
    # a docstring), as a small object array of "key: [col0, col1, ...]"
    # strings -- avoids a second file that could drift out of sync.
    # 列名の対応表は配列と一緒に持ち歩く（np.savez は docstring を保持できない）。
    # "key: [col0, col1, ...]" の小さな文字列配列として埋め込み、別ファイルにして
    # 対応がずれるのを避ける。
    column_doc = [
        "gyro_raw/gyro/accel_raw/accel: [x, y, z]",
        "quat: [w, x, y, z] (body FRD -> NED)",
        "euler: [roll, pitch, yaw] rad (3-2-1, see _euler_from_quat_ned)",
        "torque: [roll, pitch, yaw] N*m",
        "duty: [FR, RR, RL, FL] 0..1",
        "rate_ref: [roll, pitch, yaw] rad/s",
        "angle_ref: [roll, pitch] rad",
        "pilot_rpy: [roll, pitch, yaw] -1..1",
        "gyro_bias: [x, y, z] rad/s (ESKF-estimated, see eskf_core.cpp bg_)",
    ]
    arrays["column_doc"] = np.array(column_doc)
    arrays["scenario"] = np.array([scenario_name])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out_path, **arrays)

    return {
        "scenario": scenario_name,
        "total_rows": len(df),
        "flight_rows": len(flight),
        "duration_s": float(t_s[-1]) if len(t_s) else 0.0,
        "roll_deg_range": (float(np.degrees(roll.min())), float(np.degrees(roll.max()))) if len(roll) else (0.0, 0.0),
        "pitch_deg_range": (float(np.degrees(pitch.min())), float(np.degrees(pitch.max()))) if len(pitch) else (0.0, 0.0),
        "yaw_deg_range": (float(np.degrees(yaw.min())), float(np.degrees(yaw.max()))) if len(yaw) else (0.0, 0.0),
        "gyro_abs_max_dps": float(np.degrees(np.abs(arrays["gyro"]).max())) if len(flight) else 0.0,
        "out_path": str(out_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("zips", nargs="*", type=Path,
                         help="specific *.sflog.zip file(s) to convert (default: all under --data-dir)")
    parser.add_argument("--data-dir", type=Path, default=_DEFAULT_DATA_DIR,
                         help=f"directory to scan when no zips are given (default: {_DEFAULT_DATA_DIR})")
    parser.add_argument("--out-dir", type=Path, default=None,
                         help="output directory for *_aligned.npz (default: same as --data-dir)")
    args = parser.parse_args()

    zips = args.zips if args.zips else sorted(args.data_dir.glob("*.sflog.zip"))
    if not zips:
        print(f"no *.sflog.zip found under {args.data_dir}", file=sys.stderr)
        return 1

    out_dir = args.out_dir if args.out_dir is not None else args.data_dir

    summaries = []
    for zip_path in zips:
        out_path = out_dir / f"{zip_path.name.removesuffix('.sflog.zip')}_aligned.npz"
        summary = build_one(zip_path, out_path)
        summaries.append(summary)
        print(f"[{summary['scenario']}] flight_rows={summary['flight_rows']}/{summary['total_rows']} "
              f"duration={summary['duration_s']:.2f}s "
              f"roll={summary['roll_deg_range'][0]:.2f}..{summary['roll_deg_range'][1]:.2f}deg "
              f"pitch={summary['pitch_deg_range'][0]:.2f}..{summary['pitch_deg_range'][1]:.2f}deg "
              f"yaw={summary['yaw_deg_range'][0]:.2f}..{summary['yaw_deg_range'][1]:.2f}deg "
              f"gyro_max={summary['gyro_abs_max_dps']:.1f}dps -> {out_path.name}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
