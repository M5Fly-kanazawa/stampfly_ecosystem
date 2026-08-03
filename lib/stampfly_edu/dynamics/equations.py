"""
Drone Equations of Motion
ドローン運動方程式

Derives 6DoF equations symbolically using SymPy and provides
numerical utilities for hover analysis and linearization.
"""

import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Import physical parameters
# 物理パラメータのインポート
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
        "mass": 0.037,  # 実測36.8g、2026-07-24統一（旧0.035は更新漏れ）
        "Ixx": 9.16e-6, "Iyy": 13.3e-6, "Izz": 20.4e-6,
        "arm_length": 0.023, "Ct": 1.0e-8,  # 2026-08-03暫定採用（2026-07-15のthrust stand値は撤回）
        "Cq": 4.10e-11,  # 2026-07-15実測（コーストダウン法）
        "max_thrust": 0.15, "g": 9.80665, "Vbat": 3.7,
        "Rm": 0.593,  # 論文LCR実測採用値, 2026-07-15決定
        "Km": 5.682e-4,  # 2026-07-16確定（コーストダウン開回路EMF実測・SCPI公式換算）
        "tau_m": 0.02,
    }


def derive_equations_of_motion():
    """Derive drone 6DoF equations of motion symbolically.

    ドローン 6DoF 運動方程式を記号的に導出する。

    Returns:
        dict with SymPy expressions for translational and rotational dynamics

    Usage:
        >>> eqs = derive_equations_of_motion()
        >>> 'translational' in eqs and 'rotational' in eqs
        True
    """
    import sympy as sp

    # Define symbols
    # 記号を定義
    m, g = sp.symbols("m g", positive=True)
    Ixx, Iyy, Izz = sp.symbols("I_xx I_yy I_zz", positive=True)
    phi, theta, psi = sp.symbols("phi theta psi")  # Roll, pitch, yaw
    p_sym, q_sym, r_sym = sp.symbols("p q r")  # Angular rates (body frame)
    x, y, z = sp.symbols("x y z")  # Position (NED)
    u, v, w = sp.symbols("u v w")  # Velocity (body frame)
    F1, F2, F3, F4 = sp.symbols("F_1 F_2 F_3 F_4", positive=True)  # Motor thrusts
    L, kappa = sp.symbols("L kappa", positive=True)  # Arm length, torque/thrust ratio

    # Total thrust (body Z-axis, pointing up)
    # 合計推力（機体Z軸、上向き）
    F_total = F1 + F2 + F3 + F4

    # Torques from motor configuration (X-configuration)
    # モーター配置からのトルク（X型）
    # M1=FR, M2=RR, M3=RL, M4=FL
    tau_x = L * (-F1 + F2 + F3 - F4)    # Roll torque
    tau_y = L * (-F1 - F2 + F3 + F4)    # Pitch torque
    tau_z = kappa * (F1 - F2 + F3 - F4)  # Yaw torque

    # Rotation matrix (ZYX convention, NED to body)
    # 回転行列（ZYX 規約、NED → 機体）
    R = sp.Matrix([
        [sp.cos(theta)*sp.cos(psi),
         sp.cos(theta)*sp.sin(psi),
         -sp.sin(theta)],
        [sp.sin(phi)*sp.sin(theta)*sp.cos(psi) - sp.cos(phi)*sp.sin(psi),
         sp.sin(phi)*sp.sin(theta)*sp.sin(psi) + sp.cos(phi)*sp.cos(psi),
         sp.sin(phi)*sp.cos(theta)],
        [sp.cos(phi)*sp.sin(theta)*sp.cos(psi) + sp.sin(phi)*sp.sin(psi),
         sp.cos(phi)*sp.sin(theta)*sp.sin(psi) - sp.sin(phi)*sp.cos(psi),
         sp.cos(phi)*sp.cos(theta)],
    ])

    # Translational dynamics (NED frame)
    # 並進ダイナミクス（NED座標系）
    gravity = sp.Matrix([0, 0, m*g])
    thrust_body = sp.Matrix([0, 0, -F_total])
    thrust_ned = R.T * thrust_body

    accel_ned = (thrust_ned + gravity) / m

    # Rotational dynamics (Euler's equation in body frame)
    # 回転ダイナミクス（機体座標系のオイラー方程式）
    dp = (tau_x + (Iyy - Izz) * q_sym * r_sym) / Ixx
    dq = (tau_y + (Izz - Ixx) * p_sym * r_sym) / Iyy
    dr = (tau_z + (Ixx - Iyy) * p_sym * q_sym) / Izz

    return {
        "translational": {
            "ddx": accel_ned[0],
            "ddy": accel_ned[1],
            "ddz": accel_ned[2],
        },
        "rotational": {
            "dp": dp,
            "dq": dq,
            "dr": dr,
        },
        "forces": {
            "F_total": F_total,
            "tau_x": tau_x,
            "tau_y": tau_y,
            "tau_z": tau_z,
        },
        "symbols": {
            "m": m, "g": g,
            "Ixx": Ixx, "Iyy": Iyy, "Izz": Izz,
            "phi": phi, "theta": theta, "psi": psi,
            "p": p_sym, "q": q_sym, "r": r_sym,
            "F1": F1, "F2": F2, "F3": F3, "F4": F4,
            "L": L, "kappa": kappa,
        },
        "rotation_matrix": R,
    }


def hover_condition(params: Optional[dict] = None) -> dict:
    """Calculate hover condition for StampFly.

    StampFly のホバー条件を計算する。

    Args:
        params: Physical parameters override / 物理パラメータの上書き

    Returns:
        dict with hover thrust per motor, total thrust, duty cycle estimate

    Usage:
        >>> result = hover_condition()
        >>> result["thrust_per_motor_N"] > 0
        True
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    mass = p["mass"]
    g = p["g"]
    max_thrust = p["max_thrust"]

    total_thrust = mass * g
    thrust_per_motor = total_thrust / 4.0
    duty_estimate = thrust_per_motor / max_thrust

    return {
        "total_thrust_N": total_thrust,
        "thrust_per_motor_N": thrust_per_motor,
        "duty_estimate": duty_estimate,
        "mass_kg": mass,
        "gravity_m_s2": g,
        "thrust_to_weight_ratio": (4 * max_thrust) / total_thrust,
    }


def linearize_at_hover(params: Optional[dict] = None) -> dict:
    """Linearize drone dynamics at hover condition.

    ホバー条件でドローンのダイナミクスを線形化する。

    Returns state-space matrices (A, B) for small perturbations
    around hover.

    Returns:
        dict with A_roll, B_roll (roll axis), A_pitch, B_pitch (pitch axis)

    Usage:
        >>> result = linearize_at_hover()
        >>> result["A_roll"].shape
        (2, 2)
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    # Roll axis: state = [phi, p], input = delta_torque
    # d(phi)/dt = p
    # d(p)/dt = tau / Ixx
    Ixx = p["Ixx"]
    Iyy = p["Iyy"]
    L = p["arm_length"]
    max_thrust = p["max_thrust"]

    # A and B matrices for roll axis
    # ロール軸の A, B 行列
    A_roll = np.array([
        [0, 1],
        [0, 0],
    ])
    B_roll = np.array([
        [0],
        [2 * L * max_thrust / Ixx],  # Torque per unit differential command
    ])

    A_pitch = np.array([
        [0, 1],
        [0, 0],
    ])
    B_pitch = np.array([
        [0],
        [2 * L * max_thrust / Iyy],
    ])

    return {
        "A_roll": A_roll,
        "B_roll": B_roll,
        "A_pitch": A_pitch,
        "B_pitch": B_pitch,
    }


def motor_curve(
    voltage_range: Optional[np.ndarray] = None,
    params: Optional[dict] = None,
) -> dict:
    """Calculate motor thrust vs input voltage/duty curve.

    モーター推力 vs 入力電圧/デューティ曲線を計算する。

    Args:
        voltage_range: Voltage range (default: 0 to Vbat)
        params: Physical parameters

    Returns:
        dict with voltage, rpm, thrust, torque, current arrays

    Usage:
        >>> result = motor_curve()
        >>> len(result["voltage"]) > 0
        True
    """
    p = dict(_DEFAULTS)
    if params:
        p.update(params)

    Vbat = p["Vbat"]
    Rm = p["Rm"]
    Km = p["Km"]
    Ct = p["Ct"]
    Cq = p.get("Cq", 9.71e-11)

    if voltage_range is None:
        voltage_range = np.linspace(0, Vbat, 100)

    rpm_list = []
    thrust_list = []
    torque_list = []
    current_list = []

    for V in voltage_range:
        # Steady-state: V = Km*omega + Rm*I, I = (Cq*omega^2 + Qf) / Km
        # Simplified: V = Km*omega + Rm*Cq*omega^2/Km (ignoring static friction)
        # Quadratic in omega
        # 2次方程式 for omega
        a = Rm * Cq / Km
        b = Km
        c = -V

        disc = b**2 - 4*a*c
        if disc < 0 or V <= 0:
            rpm_list.append(0)
            thrust_list.append(0)
            torque_list.append(0)
            current_list.append(0)
            continue

        omega = (-b + np.sqrt(disc)) / (2 * a)
        omega = max(0, omega)

        thrust = Ct * omega**2
        torque = Cq * omega**2
        current = (V - Km * omega) / Rm

        rpm_list.append(omega * 60 / (2 * np.pi))
        thrust_list.append(thrust)
        torque_list.append(torque)
        current_list.append(max(0, current))

    return {
        "voltage": voltage_range,
        "rpm": np.array(rpm_list),
        "thrust_N": np.array(thrust_list),
        "torque_Nm": np.array(torque_list),
        "current_A": np.array(current_list),
    }
