"""
Default StampFly Physical Parameters
StampFly デフォルト物理パラメータ

Reference: docs/architecture/stampfly-parameters.md
"""

from typing import Dict, Any


# Default StampFly parameters from measured/identified values
# 測定・同定値に基づくデフォルトパラメータ
DEFAULT_PARAMS: Dict[str, Any] = {
    # ==========================================================================
    # Mass Properties / 質量特性
    # ==========================================================================
    "mass": {
        "value": 0.035,  # kg
        "unit": "kg",
        "description": "Vehicle mass including battery / バッテリー込み機体質量",
    },

    # Moments of inertia / 慣性モーメント
    "inertia": {
        "Ixx": {
            "value": 9.16e-6,  # kg·m²
            "unit": "kg·m²",
            "description": "Roll moment of inertia / ロール慣性モーメント",
        },
        "Iyy": {
            "value": 13.3e-6,  # kg·m²
            "unit": "kg·m²",
            "description": "Pitch moment of inertia / ピッチ慣性モーメント",
        },
        "Izz": {
            "value": 20.4e-6,  # kg·m²
            "unit": "kg·m²",
            "description": "Yaw moment of inertia / ヨー慣性モーメント",
        },
    },

    # ==========================================================================
    # Geometry / 機体形状
    # ==========================================================================
    "geometry": {
        "arm_length": {
            "value": 0.023,  # m
            "unit": "m",
            "description": "Moment arm (X/Y offset) / モーメントアーム",
        },
        "motor_height": {
            "value": 0.005,  # m
            "unit": "m",
            "description": "Motor height from CG / 重心からのモーター高さ",
        },
    },

    # ==========================================================================
    # Motor & Propeller / モーター・プロペラ
    # ==========================================================================
    "motor": {
        # Thrust and torque coefficients
        "Ct": {
            "value": 1.00e-8,  # N/(rad/s)²
            "unit": "N/(rad/s)²",
            "description": "Thrust coefficient / 推力係数",
        },
        "Cq": {
            "value": 9.71e-11,  # N·m/(rad/s)²
            "unit": "N·m/(rad/s)²",
            "description": "Torque coefficient / トルク係数",
        },
        "kappa": {
            "value": 9.71e-3,  # m
            "unit": "m",
            "description": "Torque/thrust ratio (Cq/Ct) / トルク推力比",
        },

        # Electrical characteristics
        "Rm": {
            "value": 0.34,  # Ω
            "unit": "Ω",
            "description": "Winding resistance / 巻線抵抗",
        },
        "Km": {
            "value": 6.125e-4,  # V/(rad/s)
            "unit": "V/(rad/s)",
            "description": "Back-EMF constant / 逆起電力定数",
        },
        "Dm": {
            "value": 3.69e-8,  # N·m·s/rad
            "unit": "N·m·s/rad",
            "description": "Viscous friction coefficient / 粘性摩擦係数",
        },
        "Qf": {
            "value": 2.76e-5,  # N·m
            "unit": "N·m",
            "description": "Static friction torque / 静止摩擦トルク",
        },
        "Jmp": {
            "value": 1.375e-8,  # kg·m² (2026-07-15実測: 写真法+諸元法)
            "unit": "kg·m²",
            "description": "Rotor inertia / 回転子慣性モーメント",
        },

        # Time constant (to be identified)
        "tau_m": {
            "value": 0.02,  # s (estimated)
            "unit": "s",
            "description": "Motor time constant / モータ時定数",
        },

        # Limits
        "max_thrust": {
            "value": 0.15,  # N per motor
            "unit": "N",
            "description": "Max thrust per motor / 1モーターあたり最大推力",
        },
    },

    # ==========================================================================
    # Aerodynamics / 空気力学
    # ==========================================================================
    "aero": {
        "Cd_trans": {
            "value": 0.1,  # dimensionless
            "unit": "-",
            "description": "Translational drag coefficient / 並進抗力係数",
        },
        "Cd_rot": {
            "value": 1.0e-5,  # dimensionless
            "unit": "-",
            "description": "Rotational drag coefficient / 回転抗力係数",
        },
        "rho": {
            "value": 1.225,  # kg/m³
            "unit": "kg/m³",
            "description": "Air density / 空気密度",
        },
    },

    # ==========================================================================
    # Sensor Noise / センサノイズ
    # ==========================================================================
    "sensor_noise": {
        "gyro": {
            "noise_density": {
                "value": 1.22e-4,  # rad/s/√Hz (0.007 deg/s/√Hz)
                "unit": "rad/s/√Hz",
                "description": "Gyro noise density (ARW) / ジャイロノイズ密度",
            },
            "bias_instability": {
                "value": 1.75e-3,  # rad/s (0.1 deg/s)
                "unit": "rad/s",
                "description": "Gyro bias instability / ジャイロバイアス不安定性",
            },
        },
        "accel": {
            "noise_density": {
                "value": 1.18e-3,  # m/s²/√Hz (120 µg/√Hz)
                "unit": "m/s²/√Hz",
                "description": "Accel noise density (VRW) / 加速度ノイズ密度",
            },
            "bias_instability": {
                "value": 0.02,  # m/s² (0.002 g)
                "unit": "m/s²",
                "description": "Accel bias instability / 加速度バイアス不安定性",
            },
        },
        "baro": {
            "noise": {
                "value": 0.11,  # m (from 1.3 Pa RMS)
                "unit": "m",
                "description": "Barometer altitude noise / 気圧高度ノイズ",
            },
        },
        "tof": {
            "noise": {
                "value": 0.01,  # m
                "unit": "m",
                "description": "ToF range noise / ToF測距ノイズ",
            },
        },
        "flow": {
            "noise": {
                "value": 1.0,  # counts
                "unit": "counts",
                "description": "Optical flow noise / オプティカルフローノイズ",
            },
        },
    },

    # ==========================================================================
    # Physical Constants / 物理定数
    # ==========================================================================
    "constants": {
        "g": 9.80665,  # m/s²
        "Vbat": 3.7,   # V (nominal LiPo)
    },
}


def get_default_params() -> Dict[str, Any]:
    """Get a copy of default parameters
    デフォルトパラメータのコピーを取得

    Returns:
        Deep copy of DEFAULT_PARAMS
    """
    import copy
    return copy.deepcopy(DEFAULT_PARAMS)


def get_flat_defaults() -> Dict[str, float]:
    """Get flattened default values (for quick access)
    フラット化したデフォルト値を取得（クイックアクセス用）

    Returns:
        Dictionary with dot-notation keys and float values
    """
    return {
        # Mass
        "mass": 0.035,

        # Inertia
        "Ixx": 9.16e-6,
        "Iyy": 13.3e-6,
        "Izz": 20.4e-6,

        # Geometry
        "arm_length": 0.023,

        # Motor
        "Ct": 6.7e-9,      # 2026-07-15実測（現行プロペラ）
        "Cq": 4.10e-11,    # 2026-07-15実測（コーストダウン）
        "kappa": 6.12e-3,  # = Cq/Ct（2026-07-15実測）
        "Rm": 0.593,     # 論文LCR実測を採用（2026-07-15決定）
        "Km": 5.682e-4,  # コーストダウン開回路EMF実測（SCPI公式換算・2026-07-16確定）
        "tau_m": 0.02,
        "max_thrust": 0.15,

        # Aero
        "Cd_trans": 0.1,
        "Cd_rot": 1.0e-5,

        # Sensor noise
        "gyro_noise": 1.22e-4,
        "gyro_bias_instability": 1.75e-3,
        "accel_noise": 1.18e-3,
        "accel_bias_instability": 0.02,
        "baro_noise": 0.11,
        "tof_noise": 0.01,
        "flow_noise": 1.0,

        # Constants
        "g": 9.80665,
        "Vbat": 3.7,
    }
