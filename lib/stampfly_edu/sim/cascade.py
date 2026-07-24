"""
Cascade Control Simulator for Education
教育用カスケード制御シミュレータ

Demonstrates single-loop vs cascade control with
drone-specific parameters and visualization.
"""

import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Reuse PID
# PID を再利用
_vpython_dir = Path(__file__).parent.parent.parent.parent / "simulator" / "vpython" / "control"
if str(_vpython_dir) not in sys.path:
    sys.path.insert(0, str(_vpython_dir))

from pid import PID

# Import parameters
# パラメータをインポート
_tools_dir = Path(__file__).parent.parent.parent.parent / "tools" / "sysid"
if str(_tools_dir) not in sys.path:
    sys.path.insert(0, str(_tools_dir))

try:
    from defaults import get_flat_defaults
    _DEFAULTS = get_flat_defaults()
except ImportError:
    # Fallback values must be kept in sync with tools/sysid/defaults.py
    # get_flat_defaults() (checked by `sf params check`) — this dict is only
    # used when tools/sysid is not importable (e.g. standalone script use),
    # so it cannot pull values from defaults.py automatically and has to be
    # updated by hand whenever the measured parameters change.
    # このフォールバック値は tools/sysid/defaults.py の get_flat_defaults() と
    # 同期必須（`sf params check` が検査）— tools/sysid をインポートできない
    # 場合（スタンドアロン実行時等）のみ使われるため defaults.py から自動取得
    # できず、実測パラメータが更新されるたびに手動更新が必要。
    _DEFAULTS = {
        "mass": 0.035, "Ixx": 9.16e-6, "arm_length": 0.023,
        "max_thrust": 0.15, "tau_m": 0.02, "g": 9.80665,
    }


def simulate_cascade_attitude(
    Kp_angle: float = 5.0,
    Kp_rate: float = 0.5,
    Ti_rate: float = 0.0,
    Td_rate: float = 0.01,
    setpoint_deg: float = 10.0,
    duration: float = 2.0,
    dt: float = 0.0025,
    axis: str = "roll",
    params: Optional[dict] = None,
) -> pd.DataFrame:
    """Simulate cascade attitude control (angle -> rate -> motor).

    カスケード姿勢制御をシミュレート（角度 → 角速度 → モーター）。

    Args:
        Kp_angle: Outer loop (angle) P gain / 外側ループ（角度）P ゲイン
        Kp_rate: Inner loop (rate) P gain / 内側ループ（角速度）P ゲイン
        Ti_rate: Inner loop integral time / 内側ループ積分時間
        Td_rate: Inner loop derivative time / 内側ループ微分時間
        setpoint_deg: Angle setpoint (degrees) / 角度目標値（度）
        duration: Simulation time (s) / シミュレーション時間
        dt: Time step (s) / タイムステップ
        axis: "roll" or "pitch" / 軸
        params: Physical parameters / 物理パラメータ

    Returns:
        DataFrame with time, angle_sp, angle, rate_sp, rate, motor columns
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    J = p["Ixx"] if axis == "roll" else p.get("Iyy", p["Ixx"])
    L = p["arm_length"]
    max_thrust = p["max_thrust"]
    tau_m = p["tau_m"]

    # Controllers
    # コントローラ
    angle_pid = PID(Kp=Kp_angle, output_min=-500, output_max=500)
    rate_pid = PID(Kp=Kp_rate, Ti=Ti_rate, Td=Td_rate, output_min=-1.0, output_max=1.0)

    # State
    angle = 0.0  # rad
    rate = 0.0   # rad/s
    motor = 0.0  # normalized

    n = int(duration / dt)
    data = {
        "time": np.zeros(n),
        "angle_sp": np.zeros(n),
        "angle": np.zeros(n),
        "rate_sp": np.zeros(n),
        "rate": np.zeros(n),
        "motor": np.zeros(n),
    }

    setpoint_rad = np.radians(setpoint_deg)

    for i in range(n):
        t = i * dt
        sp = setpoint_rad if t >= 0.1 else 0.0

        # Outer loop: angle -> rate setpoint
        # 外側ループ: 角度 → 角速度目標
        rate_sp = angle_pid.update(sp, angle, dt)

        # Inner loop: rate -> motor command
        # 内側ループ: 角速度 → モーターコマンド
        u = rate_pid.update(rate_sp, rate, dt)

        # Motor dynamics (first-order lag)
        # モータダイナミクス（1次遅れ）
        motor += (u - motor) / tau_m * dt

        # Plant dynamics
        # プラントダイナミクス
        torque = motor * max_thrust * L * 2
        rate += (torque / J) * dt
        angle += rate * dt

        data["time"][i] = t
        data["angle_sp"][i] = np.degrees(sp)
        data["angle"][i] = np.degrees(angle)
        data["rate_sp"][i] = np.degrees(rate_sp)
        data["rate"][i] = np.degrees(rate)
        data["motor"][i] = motor

    return pd.DataFrame(data)


def simulate_single_loop_attitude(
    Kp: float = 2.0,
    Ti: float = 0.0,
    Td: float = 0.1,
    setpoint_deg: float = 10.0,
    duration: float = 2.0,
    dt: float = 0.0025,
    axis: str = "roll",
    params: Optional[dict] = None,
) -> pd.DataFrame:
    """Simulate single-loop attitude control (angle directly to motor).

    単ループ姿勢制御をシミュレート（角度から直接モーターへ）。

    This shows why cascade is preferred.
    カスケードが好まれる理由を示します。
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    J = p["Ixx"] if axis == "roll" else p.get("Iyy", p["Ixx"])
    L = p["arm_length"]
    max_thrust = p["max_thrust"]
    tau_m = p["tau_m"]

    pid = PID(Kp=Kp, Ti=Ti, Td=Td, output_min=-1.0, output_max=1.0)

    angle = 0.0
    rate = 0.0
    motor = 0.0

    n = int(duration / dt)
    data = {
        "time": np.zeros(n),
        "angle_sp": np.zeros(n),
        "angle": np.zeros(n),
        "rate": np.zeros(n),
        "motor": np.zeros(n),
    }

    setpoint_rad = np.radians(setpoint_deg)

    for i in range(n):
        t = i * dt
        sp = setpoint_rad if t >= 0.1 else 0.0

        # Single loop: angle -> motor
        u = pid.update(sp, angle, dt)

        motor += (u - motor) / tau_m * dt
        torque = motor * max_thrust * L * 2
        rate += (torque / J) * dt
        angle += rate * dt

        data["time"][i] = t
        data["angle_sp"][i] = np.degrees(sp)
        data["angle"][i] = np.degrees(angle)
        data["rate"][i] = np.degrees(rate)
        data["motor"][i] = motor

    return pd.DataFrame(data)


def compare_cascade_vs_single(
    setpoint_deg: float = 10.0,
    duration: float = 2.0,
) -> plt.Figure:
    """Compare cascade vs single-loop attitude control.

    カスケード vs 単ループ姿勢制御を比較する。

    Returns:
        Matplotlib Figure

    Usage:
        >>> fig = compare_cascade_vs_single(setpoint_deg=10.0)
        >>> len(fig.axes)
        3
        >>> plt.close()
    """
    cascade = simulate_cascade_attitude(setpoint_deg=setpoint_deg, duration=duration)
    single = simulate_single_loop_attitude(setpoint_deg=setpoint_deg, duration=duration)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Angle response
    axes[0].plot(cascade["time"], cascade["angle"], "b-", linewidth=1.5, label="Cascade / カスケード")
    axes[0].plot(single["time"], single["angle"], "r--", linewidth=1.5, label="Single loop / 単ループ")
    axes[0].plot(cascade["time"], cascade["angle_sp"], "k:", linewidth=0.8, label="Setpoint / 目標値")
    axes[0].set_ylabel("Angle (deg) / 角度")
    axes[0].set_title("Cascade vs Single-Loop Attitude Control\nカスケード vs 単ループ姿勢制御")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Rate
    axes[1].plot(cascade["time"], cascade["rate"], "b-", linewidth=1.0, label="Cascade")
    axes[1].plot(single["time"], single["rate"], "r--", linewidth=1.0, label="Single loop")
    if "rate_sp" in cascade.columns:
        axes[1].plot(cascade["time"], cascade["rate_sp"], "b:", linewidth=0.5, alpha=0.5, label="Rate SP")
    axes[1].set_ylabel("Rate (deg/s) / 角速度")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # Motor
    axes[2].plot(cascade["time"], cascade["motor"], "b-", linewidth=1.0, label="Cascade")
    axes[2].plot(single["time"], single["motor"], "r--", linewidth=1.0, label="Single loop")
    axes[2].set_ylabel("Motor command / モーター指令")
    axes[2].set_xlabel("Time (s) / 時間")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def simulate_altitude_control(
    Kp: float = 2.0,
    Ti: float = 1.0,
    Td: float = 0.1,
    ff_enabled: bool = True,
    antiwindup: bool = True,
    setpoint_m: float = 0.5,
    step_time: float = 2.0,
    step_size: float = 0.3,
    duration: float = 10.0,
    dt: float = 0.0025,
    disturbance_time: Optional[float] = None,
    disturbance_force: float = 0.0,
    params: Optional[dict] = None,
) -> pd.DataFrame:
    """Simulate altitude control with feedforward and anti-windup.

    フィードフォワードとアンチワインドアップ付き高度制御をシミュレート。

    Args:
        Kp: Altitude PID Kp
        Ti: Altitude PID Ti
        Td: Altitude PID Td
        ff_enabled: Enable hover thrust feedforward / ホバー推力 FF を有効化
        antiwindup: Enable anti-windup / アンチワインドアップを有効化
        setpoint_m: Initial altitude setpoint (m) / 初期高度目標値
        step_time: Time of altitude step (s) / 高度ステップ時刻
        step_size: Step size (m) / ステップ量
        duration: Simulation duration (s) / シミュレーション時間
        dt: Time step (s) / タイムステップ
        disturbance_time: Time for disturbance / 外乱時刻
        disturbance_force: Disturbance force (N) / 外乱力
        params: Physical parameters / 物理パラメータ

    Returns:
        DataFrame with time, setpoint, altitude, velocity, thrust columns
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    mass = p["mass"]
    g = p["g"]

    # Hover thrust (feedforward)
    # ホバー推力（フィードフォワード）
    hover_thrust = mass * g
    max_total_thrust = p["max_thrust"] * 4

    pid = PID(
        Kp=Kp, Ti=Ti, Td=Td,
        output_min=-hover_thrust * 0.5 if antiwindup else -float("inf"),
        output_max=hover_thrust * 0.5 if antiwindup else float("inf"),
    )

    alt = setpoint_m
    vel = 0.0

    n = int(duration / dt)
    data = {
        "time": np.zeros(n),
        "setpoint": np.zeros(n),
        "altitude": np.zeros(n),
        "velocity": np.zeros(n),
        "thrust": np.zeros(n),
        "ff_component": np.zeros(n),
        "pid_component": np.zeros(n),
    }

    for i in range(n):
        t = i * dt
        sp = setpoint_m + step_size if t >= step_time else setpoint_m

        # PID output
        pid_out = pid.update(sp, alt, dt)

        # Total thrust = FF + PID
        ff = hover_thrust if ff_enabled else 0.0
        thrust = ff + pid_out
        thrust = np.clip(thrust, 0, max_total_thrust)

        # External disturbance
        ext_force = 0.0
        if disturbance_time is not None and t >= disturbance_time:
            ext_force = disturbance_force

        # Dynamics: m * a = thrust - m*g + ext_force
        accel = (thrust - mass * g + ext_force) / mass
        vel += accel * dt
        alt += vel * dt
        alt = max(0, alt)  # Ground constraint

        data["time"][i] = t
        data["setpoint"][i] = sp
        data["altitude"][i] = alt
        data["velocity"][i] = vel
        data["thrust"][i] = thrust
        data["ff_component"][i] = ff
        data["pid_component"][i] = pid_out

    return pd.DataFrame(data)
