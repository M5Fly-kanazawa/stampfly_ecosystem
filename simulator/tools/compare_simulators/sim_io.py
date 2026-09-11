#!/usr/bin/env python3
"""
sim_io.py - Simulator Input/Output Format
シミュレータ入出力フォーマット

Common input-sequence format for simulator comparison, and a StampFly
flight-log v1 bundle (lib/sflog; docs/plans/flight-log-format-plan.md)
writer/reader for the simulators' output (`truth.csv` + `pilot.csv`).
シミュレータ比較用の共通入力シーケンス形式と、出力（`truth.csv` +
`pilot.csv`）用の StampFly フライトログ v1 一式（lib/sflog; 計画書参照）
の読み書き。

Input CSV format (unchanged -- this is a hand-authored test-input file,
not a flight-log stream, so it stays a plain CSV):
  time,throttle,roll,pitch,yaw
  0.000,0.0,0.0,0.0,0.0
  0.0025,0.0,0.3,0.0,0.0
  ...

Output format (v2, 2026-09-11): a `.sflog.zip` bundle written by
save_output_bundle() / read by load_output_bundle(). The old plain
`time,x,y,z,roll,pitch,yaw,p,q,r` CSV with `#`-comment metadata is gone --
see docs/plans/flight-log-format-plan.md section 3.1 ("sf sim headless").
出力形式（v2、2026-09-11）: save_output_bundle() が書き / load_output_bundle()
が読む `.sflog.zip` 一式。旧来の `#` コメントでメタデータを埋め込む素の
CSV は廃止した（計画書 3.1節「sf sim headless」参照）。
"""

import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import pandas as pd

# `lib/sflog` (the flight-log v1 read/write package) lives at the repo
# root's lib/ directory. When this module runs under the sf CLI's own
# interpreter, `sflog` is already importable (editable-installed via
# pyproject.toml's package auto-discovery of lib/). When a headless script
# runs under a DIFFERENT interpreter (e.g. Genesis's own dedicated venv,
# simulator/genesis/venv), that venv has no such install, so this path is
# inserted defensively. That venv still needs pandas/PyYAML for `sflog` to
# import at all -- unverified here (Genesis is not installed in this
# environment; see this module's test file / the change's report).
# `lib/sflog`（フライトログ v1 読み書きパッケージ）はリポジトリルートの
# lib/ にある。本モジュールが sf CLI 自身のインタプリタで動く場合は
# pyproject.toml のパッケージ自動探索により `sflog` は既に import 可能。
# ヘッドレススクリプトが別のインタプリタ（Genesis 専用 venv 等）で動く
# 場合はその venv にこの導入が無いため、ここで防御的にパスを追加する。
# その venv 自体に pandas/PyYAML が入っていなければ `sflog` の import
# 自体が失敗する -- 本環境では Genesis 未導入のため未検証（本変更の
# テストファイル/報告を参照）。
_REPO_ROOT = Path(__file__).resolve().parents[3]
_LIB_DIR = _REPO_ROOT / "lib"
if str(_LIB_DIR) not in sys.path:
    sys.path.insert(0, str(_LIB_DIR))

import sflog  # noqa: E402  (path insertion above must run first)


# =============================================================================
# Constants
# =============================================================================
CONTROL_DT = 0.0025  # Control loop timestep [s] (400Hz)

_TOOL_NAME = "sf sim headless"
_TOOL_VERSION = "0.1.0"


# =============================================================================
# Input Format
# =============================================================================

@dataclass
class ControlInput:
    """Control input at a given time / 指定時刻の制御入力"""
    time: float           # Time [s]
    throttle: float       # Throttle [-1, 1] (0 = hover)
    roll: float          # Roll stick [-1, 1]
    pitch: float         # Pitch stick [-1, 1]
    yaw: float           # Yaw stick [-1, 1]


def load_input_csv(filepath: str) -> List[ControlInput]:
    """Load input sequence from CSV / CSVから入力シーケンスを読み込み"""
    inputs = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            inputs.append(ControlInput(
                time=float(row['time']),
                throttle=float(row['throttle']),
                roll=float(row['roll']),
                pitch=float(row['pitch']),
                yaw=float(row['yaw']),
            ))
    return inputs


def save_input_csv(filepath: str, inputs: List[ControlInput]):
    """Save input sequence to CSV / 入力シーケンスをCSVに保存"""
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'throttle', 'roll', 'pitch', 'yaw'])
        for inp in inputs:
            writer.writerow([f"{inp.time:.6f}", f"{inp.throttle:.6f}",
                           f"{inp.roll:.6f}", f"{inp.pitch:.6f}", f"{inp.yaw:.6f}"])


def get_input_at_time(inputs: List[ControlInput], t: float, dt: float = 0.0025) -> ControlInput:
    """
    Get control input at given time (nearest neighbor).
    指定時刻の制御入力を取得（最近傍）
    """
    if not inputs:
        return ControlInput(time=t, throttle=0, roll=0, pitch=0, yaw=0)

    # Find nearest index
    idx = int(t / dt)
    idx = max(0, min(idx, len(inputs) - 1))
    return inputs[idx]


# =============================================================================
# Output Format
# =============================================================================

@dataclass
class StateLog:
    """State log entry / 状態ログエントリ

    `vx/vy/vz` (added 2026-09-11): both current headless backends can
    supply a true velocity STATE from the physics model (vpython:
    `rigidbody.velocity`, updated every step from the inertial-frame
    integration; Genesis: `get_dofs_velocity()`'s translational part) --
    see run_vpython_headless.py / run_genesis_headless.py. Neither backend
    needs to derive velocity by differencing position, which the flight-log
    plan (docs/plans/flight-log-format-plan.md) forbids for a primary
    record. Default NaN documents "no velocity available" for any future
    backend that truly lacks one; today both are populated.
    `vx/vy/vz`（2026-09-11 追加）: 現行の2バックエンドはどちらも物理モデル
    から真の速度状態を取得できる（vpython: 積分の度に更新される
    `rigidbody.velocity`。Genesis: `get_dofs_velocity()` の並進成分）--
    run_vpython_headless.py / run_genesis_headless.py 参照。どちらも
    位置の差分で速度を算出する必要はない（一次記録でそれを禁じる計画書の
    方針どおり）。既定 NaN は「速度状態を持たないバックエンド」向けの
    表現で、現状は両方とも実値が入る。
    """
    time: float    # Time [s]
    x: float       # Position X [m]
    y: float       # Position Y [m]
    z: float       # Position Z [m]
    roll: float    # Roll angle [rad]
    pitch: float   # Pitch angle [rad]
    yaw: float     # Yaw angle [rad]
    p: float       # Roll rate [rad/s]
    q: float       # Pitch rate [rad/s]
    r: float       # Yaw rate [rad/s]
    vx: float = math.nan   # Velocity X [m/s]
    vy: float = math.nan   # Velocity Y [m/s]
    vz: float = math.nan   # Velocity Z [m/s]


# -----------------------------------------------------------------------
# Euler <-> quaternion (standard aerospace 3-2-1 / ZYX convention)
# オイラー角 <-> クォータニオン（標準的な航空機規約 3-2-1 / ZYX 変換）
# -----------------------------------------------------------------------
#
# roll (phi, about body X), pitch (theta, about body Y), yaw (psi, about
# body Z), applied intrinsically in the order yaw -> pitch -> roll (so the
# rotation matrix is R = Rz(yaw) @ Ry(pitch) @ Rx(roll), the convention
# used throughout this repo's simulators -- see
# simulator/vpython/core/physics.py's `euler_dcm()`, whose matrix is the
# same 3-2-1 form). Quaternion is Hamilton (w, x, y, z), unit norm.
#
# roll(phi, ボディX軸まわり), pitch(theta, ボディY軸まわり),
# yaw(psi, ボディZ軸まわり)を yaw -> pitch -> roll の順で内的合成する
# （回転行列は R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。本リポジトリの
# シミュレータ全体で使う規約 -- simulator/vpython/core/physics.py の
# `euler_dcm()` と同じ 3-2-1 形）。クォータニオンは Hamilton 表現
# (w, x, y, z)、単位ノルム。

def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """roll/pitch/yaw [rad] -> (quat_w, quat_x, quat_y, quat_z).

    quat_w = cr*cp*cy + sr*sp*sy
    quat_x = sr*cp*cy - cr*sp*sy
    quat_y = cr*sp*cy + sr*cp*sy
    quat_z = cr*cp*sy - sr*sp*cy
    (c/s + r/p/y = cos/sin of half the roll/pitch/yaw angle)
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def quat_to_euler(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """(quat_w, quat_x, quat_y, quat_z) -> roll/pitch/yaw [rad]. Exact
    inverse of euler_to_quat() away from the pitch = +-90 deg gimbal-lock
    singularity (clamped there via asin's domain, not hit by ordinary
    quadrotor attitudes).
    euler_to_quat() の逆変換（pitch = ±90度のジンバルロック特異点近傍を
    除き厳密に一致。通常のマルチコプタ姿勢では到達しない）。
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for asin's domain (gimbal lock)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# -----------------------------------------------------------------------
# Backend-native-frame -> NED conversion (flight_log.yaml's "position: m
# (NED frame)" / "velocity: m/s (NED frame)" convention)
# バックエンド固有座標系 -> NED 変換（flight_log.yaml の NED 規約）
# -----------------------------------------------------------------------
#
# vpython (simulator/vpython/core/{dynamics,physics}.py): the rigid-body
# model already integrates in a North-East-Down-like inertial frame --
# gravity is applied as +Z (`multicopter.__init__`'s `_gravity =
# [[0],[0],[mass*9.81]]`, and `force_moment_fast()`'s total thrust pushes
# -Z, i.e. "up" is -Z), `rigidbody.euler_dcm()` is the standard aerospace
# 3-2-1 (yaw-pitch-roll) body<->inertial DCM, and `rigidbody.velocity` is
# that same uvw (body-frame velocity) rotated into the inertial frame by
# the same DCM. So vpython's own x/y/z, roll/pitch/yaw, p/q/r and vx/vy/vz
# are ALREADY NED -- no conversion applied (identity, see
# _state_log_to_ned() below).
#
# vpython: 剛体モデルは既に NED 相当の慣性座標系で積分している --
# 重力を +Z 方向に加えており（`multicopter.__init__` の `_gravity`、
# `force_moment_fast()` の全推力は -Z方向、つまり「上」が -Z）、
# `rigidbody.euler_dcm()` は標準的な航空機規約の 3-2-1（yaw-pitch-roll）
# ボディ<->慣性 DCM、`rigidbody.velocity` は同じ uvw（ボディ座標系速度）を
# 同じ DCM で慣性座標系へ回転したもの。よって vpython 自身の x/y/z、
# roll/pitch/yaw、p/q/r、vx/vy/vz は既に NED -- 変換しない（恒等変換、
# 下記 _state_log_to_ned() 参照）。
#
# Genesis (simulator/genesis/scripts/run_genesis_headless.py): the URDF/
# physics engine uses a Z-up, X-right, Y-forward world frame (see that
# script's `quat_to_euler()`/`ned_to_genesis_force()` and
# simulator/tools/compare_simulators/visualize_comparison.py's
# `genesis_to_ned()`, whose mapping is reproduced here). ASSUMPTION
# (unverified -- Genesis is not installed in this environment, see this
# change's test report): `get_dofs_velocity()`'s translational part [0:3]
# is linear velocity of the base in that SAME world frame as `get_pos()`
# (the standard free-joint convention also used by Genesis's rotational
# part [3:6], which IS documented as body-frame, see run_genesis_sim.py's
# comment on `get_dofs_velocity()[3:6]`), so position and velocity take the
# identical point transform below.
#
# Genesis: URDF/物理エンジンは Z-up, X-right, Y-forward のワールド座標系
# （run_genesis_headless.py の `quat_to_euler()`/`ned_to_genesis_force()`、
# および visualize_comparison.py の `genesis_to_ned()`。その変換をここでも
# 再利用）。前提（未検証 -- 本環境に Genesis 未導入。本変更の報告を参照）:
# `get_dofs_velocity()` の並進成分 [0:3] は `get_pos()` と同じワールド座標系
# の基部線速度（回転成分 [3:6] がボディ座標系と明記されている free-joint の
# 標準的な規約。run_genesis_sim.py の該当コメント参照）。よって位置と速度は
# 下記の同一の点変換を使う。

def _genesis_vec_to_ned(vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Point/vector transform: Genesis (X-right, Y-forward, Z-up) -> NED
    (X-north/forward, Y-east/right, Z-down). Used for position, velocity,
    and body angular rate -- all ordinary 3-vectors under this same
    axis swap-and-negate.
    点・ベクトルの変換: Genesis(X-right, Y-forward, Z-up) -> NED
    (X-north/forward, Y-east/right, Z-down)。位置・速度・機体角速度
    （いずれもこの同じ軸入れ替え＋符号反転で変換できる通常の3次元
    ベクトル）に使う。
    """
    x, y, z = vec
    return y, x, -z


def _genesis_euler_to_ned(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
    """Euler-angle transform: Genesis body-321 angles -> NED body-321
    angles. NOT the same formula as `_genesis_vec_to_ned` -- an Euler
    angle is defined about one specific body axis, so swapping the X/Y
    axis ROLES (not just relabeling vector components) swaps which angle
    is called roll vs pitch. Matches
    simulator/tools/compare_simulators/visualize_comparison.py's
    `genesis_to_ned()` (kept in sync here; that function's own
    conversion becomes unnecessary once bundles already store NED, see
    that file's changelog comment).
    オイラー角の変換: Genesis の body-321 角 -> NED の body-321 角。
    `_genesis_vec_to_ned` と同じ式ではない -- オイラー角は特定のボディ軸
    まわりの回転として定義されるため、X/Y軸の「役割」を入れ替えると
    roll/pitch の呼び名も入れ替わる。visualize_comparison.py の
    `genesis_to_ned()` と同じ変換（同期を保つ）。
    """
    return pitch, roll, -yaw


def _state_log_to_ned(log: StateLog, backend: str) -> StateLog:
    """Apply the backend-appropriate native-frame -> NED conversion to one
    StateLog. Unknown/unrecognized `backend` values pass through
    unchanged (treated as already-NED) rather than raising, so a bundle is
    still produced even if `metadata` is missing the 'simulator' key.
    バックエンドに応じたネイティブ座標系 -> NED 変換を1件の StateLog へ
    適用する。未知の `backend` はそのまま通す（既に NED とみなす）--
    `metadata` に 'simulator' キーが無くても一式は作れる。
    """
    if backend == "genesis":
        x, y, z = _genesis_vec_to_ned((log.x, log.y, log.z))
        vx, vy, vz = _genesis_vec_to_ned((log.vx, log.vy, log.vz))
        p, q, r = _genesis_vec_to_ned((log.p, log.q, log.r))
        roll, pitch, yaw = _genesis_euler_to_ned(log.roll, log.pitch, log.yaw)
        return StateLog(
            time=log.time, x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
            p=p, q=q, r=r, vx=vx, vy=vy, vz=vz,
        )
    return log  # vpython (and anything else): already NED, see module comment above


# -----------------------------------------------------------------------
# Bundle read/write (flight-log v1: truth.csv + pilot.csv)
# 一式の読み書き（フライトログ v1: truth.csv + pilot.csv）
# -----------------------------------------------------------------------

def save_output_bundle(filepath, logs: List[StateLog], inputs: List[ControlInput],
                        metadata: dict) -> None:
    """
    Save a headless simulation run as a StampFly flight-log v1 bundle
    (lib/sflog; docs/plans/flight-log-format-plan.md section 3.3):
    `truth.csv` (physics-model ground truth, converted to NED here) +
    `pilot.csv` (the input sequence at ITS OWN sample times -- never
    resampled to the state log's rate, per the plan's "一次記録には観測値
    以外を書かない") + meta.json/schema.json.
    ヘッドレスシミュレーション実行結果を StampFly フライトログ v1 一式
    として保存する: `truth.csv`（物理モデルの真値。ここで NED へ変換）+
    `pilot.csv`（入力シーケンスをその時刻のまま記録。状態ログのレートへの
    整列はしない）+ meta.json/schema.json。

    Args:
        filepath: bundle path (a `.sflog.zip` file, or a directory -- see
            `sflog.FlightLog.save()`).
        logs: the run's state trajectory, in the ORIGINATING backend's
            native frame (see `_state_log_to_ned()` for what "native"
            means per backend) -- this function does the NED conversion.
        inputs: the ControlInput sequence that drove the run, written to
            pilot.csv at its own sample times.
        metadata: {'simulator': 'vpython'|'genesis', 'physics_hz',
            'control_hz', 'input_file', 'duration_s'} as built by each
            headless script. `metadata['simulator']` selects the frame
            conversion and is echoed into meta.json's `notes`.
    """
    backend = metadata.get("simulator", "unknown")
    ned_logs = [_state_log_to_ned(log, backend) for log in logs]

    truth_rows = []
    for log in ned_logs:
        qw, qx, qy, qz = euler_to_quat(log.roll, log.pitch, log.yaw)
        truth_rows.append({
            "timestamp_us": int(round(log.time * 1e6)),
            "pos_x": log.x, "pos_y": log.y, "pos_z": log.z,
            "quat_w": qw, "quat_x": qx, "quat_y": qy, "quat_z": qz,
            "vel_x": log.vx, "vel_y": log.vy, "vel_z": log.vz,
            "rate_x": log.p, "rate_y": log.q, "rate_z": log.r,
        })
    truth_df = pd.DataFrame(truth_rows, columns=sflog.schema.COLUMN_NAMES["truth"])

    pilot_rows = [{
        "timestamp_us": int(round(inp.time * 1e6)),
        "throttle": inp.throttle, "roll": inp.roll, "pitch": inp.pitch, "yaw": inp.yaw,
    } for inp in inputs]
    pilot_df = pd.DataFrame(pilot_rows, columns=sflog.schema.COLUMN_NAMES["pilot"])

    streams = {"truth": truth_df, "pilot": pilot_df}

    notes = (
        f"sf sim headless backend={backend} "
        f"duration_s={metadata.get('duration_s')} "
        f"input_file={metadata.get('input_file')} "
        f"physics_hz={metadata.get('physics_hz')} "
        f"control_hz={metadata.get('control_hz')}"
    )
    meta = sflog.make_meta(
        source="sim", tool_name=_TOOL_NAME, tool_version=_TOOL_VERSION, notes=notes,
        streams=streams,
    )
    flight_log = sflog.FlightLog(
        meta=meta, schema=sflog.schema.schema_for(streams.keys()), streams=streams,
    )
    flight_log.save(filepath)


def load_output_bundle(filepath) -> Tuple[List[StateLog], dict]:
    """Load a bundle written by save_output_bundle().

    Returns the `truth` stream as a list of StateLog (roll/pitch/yaw
    recovered from the stored quaternion via quat_to_euler(); positions/
    velocities/rates are read as-is since the NED conversion already
    happened once, at write time -- this function does not need to know
    which backend produced the bundle), plus the bundle's meta.json dict.

    save_output_bundle() が書いた一式を読み込む。`truth` ストリームを
    StateLog のリストとして返す（roll/pitch/yaw は保存済みクォータニオンから
    quat_to_euler() で復元。位置・速度・角速度は NED 変換が書き込み時に
    既に済んでいるためそのまま読む -- どのバックエンド由来かを知る必要は
    無い）。加えて一式の meta.json を返す。
    """
    log = sflog.FlightLog.load(filepath)
    truth = log.streams.get("truth")
    if truth is None or len(truth) == 0:
        return [], log.meta

    state_logs = []
    for row in truth.itertuples(index=False):
        roll, pitch, yaw = quat_to_euler(row.quat_w, row.quat_x, row.quat_y, row.quat_z)
        state_logs.append(StateLog(
            time=row.timestamp_us / 1e6,
            x=row.pos_x, y=row.pos_y, z=row.pos_z,
            roll=roll, pitch=pitch, yaw=yaw,
            p=row.rate_x, q=row.rate_y, r=row.rate_z,
            vx=row.vel_x, vy=row.vel_y, vz=row.vel_z,
        ))
    return state_logs, log.meta


# =============================================================================
# Input Sequence Generators
# =============================================================================

def generate_step_sequence(duration: float = 10.0, dt: float = 0.0025) -> List[ControlInput]:
    """
    Generate step input sequence for testing.
    ステップ入力シーケンスを生成

    Sequence:
    - 0-1s: Hover (no input)
    - 1-2s: Roll step (+0.3)
    - 2-3s: Hover
    - 3-4s: Pitch step (+0.3)
    - 4-5s: Hover
    - 5-6s: Yaw step (+0.3)
    - 6-7s: Hover
    - 7-8s: Throttle step (+0.2)
    - 8-10s: Hover
    """
    inputs = []
    t = 0.0

    while t < duration:
        throttle = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if 1.0 <= t < 2.0:
            roll = 0.3
        elif 3.0 <= t < 4.0:
            pitch = 0.3
        elif 5.0 <= t < 6.0:
            yaw = 0.3
        elif 7.0 <= t < 8.0:
            throttle = 0.2

        inputs.append(ControlInput(time=t, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw))
        t += dt

    return inputs


def generate_doublet_sequence(duration: float = 10.0, dt: float = 0.0025,
                               pulse_duration: float = 0.3, amplitude: float = 0.5) -> List[ControlInput]:
    """
    Generate doublet input sequence (system identification pattern).
    ダブレット入力シーケンスを生成（システム同定用パターン）
    """
    inputs = []
    t = 0.0

    while t < duration:
        throttle = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Roll doublet at t=1s
        if 1.0 <= t < 1.0 + pulse_duration:
            roll = amplitude
        elif 1.0 + pulse_duration <= t < 1.0 + 2 * pulse_duration:
            roll = -amplitude

        # Pitch doublet at t=3s
        if 3.0 <= t < 3.0 + pulse_duration:
            pitch = amplitude
        elif 3.0 + pulse_duration <= t < 3.0 + 2 * pulse_duration:
            pitch = -amplitude

        # Yaw doublet at t=5s
        if 5.0 <= t < 5.0 + pulse_duration:
            yaw = amplitude
        elif 5.0 + pulse_duration <= t < 5.0 + 2 * pulse_duration:
            yaw = -amplitude

        inputs.append(ControlInput(time=t, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw))
        t += dt

    return inputs


# Sequence registry
SEQUENCES = {
    'step': generate_step_sequence,
    'doublet': generate_doublet_sequence,
}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Generate test input sequence')
    parser.add_argument('--type', '-t', choices=list(SEQUENCES.keys()), default='step',
                       help='Sequence type')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                       help='Duration [s]')
    parser.add_argument('--output', '-o', type=str, default='input_sequence.csv',
                       help='Output file')
    args = parser.parse_args()

    inputs = SEQUENCES[args.type](duration=args.duration)
    save_input_csv(args.output, inputs)
    print(f"Generated {args.type} sequence ({len(inputs)} samples) -> {args.output}")
