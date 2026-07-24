"""
Default StampFly Physical Parameters
StampFly デフォルト物理パラメータ

Reference: docs/architecture/stampfly-parameters.md
Reference: docs/architecture/simulation-policy.md §6 (backlog #2 motor ODE / #3 model-match gate)
"""

from typing import Any, Dict, Tuple

# ==============================================================================
# Motor & Propeller measured coefficients / モーター・プロペラの実測係数
#
# These are the single-source values that both DEFAULT_PARAMS (below) and
# get_flat_defaults() derive kappa/Ct/Cq from. Defining them once as module
# constants — instead of writing the same literal twice — is what keeps the
# two public APIs from re-diverging the way they did before this file was
# unified (Ct=1.00e-8/Cq=9.71e-11/kappa=9.71e-3 in DEFAULT_PARAMS vs.
# Ct=6.7e-9/Cq=4.10e-11/kappa=6.12e-3 in get_flat_defaults()).
#
# ここで一度だけ定義した値から、DEFAULT_PARAMS (後述) と get_flat_defaults() の
# 両方が kappa/Ct/Cq を導出する。値を2箇所に書き写すのではなくモジュール定数を
# 1箇所に置くことで、このファイルが統一される前に発生していた再分岐
# (DEFAULT_PARAMS側はCt=1.00e-8/Cq=9.71e-11/kappa=9.71e-3の旧値、
#  get_flat_defaults側はCt=6.7e-9/Cq=4.10e-11/kappa=6.12e-3の実測値、
#  という同一ファイル内矛盾)を構造的に防ぐ。
# ==============================================================================
_CT_VALUE = 6.7e-9   # N/(rad/s)^2 — thrust coeff, 2026-07-15 実測（現行プロペラ・推力測定）
_CQ_VALUE = 4.10e-11  # N・m/(rad/s)^2 — torque coeff, 2026-07-15 実測（コーストダウン法）
_KAPPA_VALUE = _CQ_VALUE / _CT_VALUE  # m — kappa = Cq/Ct, mechanically derived, not a
                                       # separately hand-entered literal
                                       # kappa = Cq/Ct として機械的に導出（別途手入力しない）
_JMP_VALUE = 1.375e-8  # kg·m^2 — rotor inertia, 2026-07-15 実測（写真法+諸元法）

# Winding resistance / 巻線抵抗:
# RESOLVED 2026-07-24 (teacher decision, commit 9a656a9f 2026-07-15): 0.593 Ω
# (paper's LCR measurement) is the single adopted value. The two other
# candidates that used to coexist — 0.34 Ω (older/possibly-different-unit LCR
# measurement) and 0.63 Ω (vpython's own stale initial value) — were update
# gaps, not competing measurements, and have since been cross-file unified to
# 0.593 Ω in simulator/genesis/motor_model.py, simulator/vpython/core/motors.py,
# and simulator/sil/plant/plant.hpp (see tools/params_audit/params_manifest.py
# "Rm" group, checked by `sf params check`).
# 巻線抵抗: 2026-07-24決着（先生決定、コミット9a656a9f 2026-07-15）: 0.593Ω
# （論文LCR実測）を唯一の採用値とする。かつて並立していた他の2値 — 0.34Ω
# （旧/別個体疑いのLCR実測）と0.63Ω（vpython側の更新漏れの旧初期値）— は
# 競合する実測ではなく単なる更新漏れであり、simulator/genesis/motor_model.py・
# simulator/vpython/core/motors.py・simulator/sil/plant/plant.hpp を含め
# クロスファイルで0.593Ωへ統一済み（tools/params_audit/params_manifest.py の
# "Rm" グループ、`sf params check` で検査）。
_RM_VALUE = 0.593  # Ω

# Back-EMF constant / 逆起電力定数:
# Cq×Rm/Am の理論式ではなく、コーストダウン開回路EMF実測（SCPI公式換算）による
# 直接測定値。Am(電圧-角速度2次係数)を本ファイルでは保持していないため式では
# 表せず、実測値としてそのまま保持する。
# Not derived from the Cq×Rm/Am formula here — this is a direct measurement from
# coastdown open-circuit EMF (converted via the SCPI formula). Am (the V-ω²
# coefficient) is not tracked in this file, so there is no in-file formula to
# express Km through; it is kept as a directly measured literal instead.
_KM_VALUE = 5.682e-4  # V/(rad/s), 2026-07-16 確定（コーストダウン開回路EMF実測・SCPI公式換算）


# Default StampFly parameters from measured/identified values
# 測定・同定値に基づくデフォルトパラメータ
#
# This nested, metadata-carrying dict (value/unit/description) is the
# single source of truth. get_flat_defaults() below derives its flat view
# from this dict via _FLAT_KEY_PATHS instead of re-declaring the numbers.
# この入れ子構造(value/unit/description付き)が唯一の正。下の
# get_flat_defaults() は _FLAT_KEY_PATHS を介してこの辞書からフラット表現を
# 導出する（数値を再宣言しない）。
DEFAULT_PARAMS: Dict[str, Any] = {
    # ==========================================================================
    # Mass Properties / 質量特性
    # ==========================================================================
    "mass": {
        "value": 0.037,  # kg
        "unit": "kg",
        "description": (
            "Vehicle mass including battery / バッテリー込み機体質量 "
            "(実測36.8g、firmware は commit 43841314 で2026-03-31反映済み。"
            "0.035 はシミュレータ側の反映前の設計値だった更新漏れ — 2026-07-24統一)"
        ),
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
            "value": _CT_VALUE,
            "unit": "N/(rad/s)²",
            "description": "Thrust coefficient / 推力係数 (2026-07-15実測)",
        },
        "Cq": {
            "value": _CQ_VALUE,
            "unit": "N·m/(rad/s)²",
            "description": "Torque coefficient / トルク係数 (2026-07-15実測)",
        },
        "kappa": {
            "value": _KAPPA_VALUE,
            "unit": "m",
            "description": (
                "Torque/thrust ratio (Cq/Ct) / トルク推力比 "
                "(2026-07-15実測、firmware actuator.cpp のB⁻¹ミキサーで採用済み)"
            ),
        },

        # Electrical characteristics
        "Rm": {
            "value": _RM_VALUE,
            "unit": "Ω",
            "description": (
                "Winding resistance / 巻線抵抗 "
                "(0.34=旧LCR実測 / 0.63=vpython実測 と並立中・実測確定待ち。"
                "本ファイルでは論文LCR実測の0.593を採用)"
            ),
        },
        "Km": {
            "value": _KM_VALUE,
            "unit": "V/(rad/s)",
            "description": (
                "Back-EMF constant / 逆起電力定数 "
                "(2026-07-16確定、コーストダウン開回路EMF実測・SCPI公式換算)"
            ),
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
            "value": _JMP_VALUE,
            "unit": "kg·m²",
            "description": "Rotor inertia / 回転子慣性モーメント (2026-07-15実測: 写真法+諸元法)",
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


# Mapping from historical flat key (as returned by get_flat_defaults(), and
# depended on by tools/sysid/{motor,plant_fit,inertia,drag,validation}.py,
# lib/sfcli/commands/sysid.py, and lib/stampfly_edu/{dynamics,sim}/*.py) to
# the path inside DEFAULT_PARAMS that holds the authoritative value.
#
# This table intentionally does NOT cover every DEFAULT_PARAMS entry (e.g.
# Dm, Qf, motor_height, rho are omitted) — it only reproduces the flat-key
# surface get_flat_defaults() already exposed, so existing callers keep
# working unchanged.
#
# get_flat_defaults() が返す従来のフラットキーから、DEFAULT_PARAMS 内で値を
# 保持しているパスへの対応表。tools/sysid/* や lib/sfcli、lib/stampfly_edu の
# 既存呼び出し元はこのキー集合に依存している。
#
# 注: DEFAULT_PARAMS の全項目を網羅するわけではない(Dm, Qf, motor_height, rho
# 等は対象外) — get_flat_defaults() が従来公開していたフラットキー集合のみを
# 再現し、既存呼び出し元をそのまま動かし続けることが目的。
_FLAT_KEY_PATHS: Dict[str, Tuple[str, ...]] = {
    # Mass
    "mass": ("mass",),

    # Inertia
    "Ixx": ("inertia", "Ixx"),
    "Iyy": ("inertia", "Iyy"),
    "Izz": ("inertia", "Izz"),

    # Geometry
    "arm_length": ("geometry", "arm_length"),

    # Motor
    "Ct": ("motor", "Ct"),
    "Cq": ("motor", "Cq"),
    "kappa": ("motor", "kappa"),
    "Rm": ("motor", "Rm"),
    "Km": ("motor", "Km"),
    "tau_m": ("motor", "tau_m"),
    "max_thrust": ("motor", "max_thrust"),

    # Aero
    "Cd_trans": ("aero", "Cd_trans"),
    "Cd_rot": ("aero", "Cd_rot"),

    # Sensor noise
    "gyro_noise": ("sensor_noise", "gyro", "noise_density"),
    "gyro_bias_instability": ("sensor_noise", "gyro", "bias_instability"),
    "accel_noise": ("sensor_noise", "accel", "noise_density"),
    "accel_bias_instability": ("sensor_noise", "accel", "bias_instability"),
    "baro_noise": ("sensor_noise", "baro", "noise"),
    "tof_noise": ("sensor_noise", "tof", "noise"),
    "flow_noise": ("sensor_noise", "flow", "noise"),

    # Constants
    "g": ("constants", "g"),
    "Vbat": ("constants", "Vbat"),
}


def get_flat_defaults() -> Dict[str, float]:
    """Get flattened default values (for quick access)
    フラット化したデフォルト値を取得（クイックアクセス用）

    Values are pulled live from DEFAULT_PARAMS via _FLAT_KEY_PATHS, so this
    can never re-diverge from DEFAULT_PARAMS the way it did previously.
    値は _FLAT_KEY_PATHS 経由で DEFAULT_PARAMS から取得するため、以前のように
    DEFAULT_PARAMS と再び食い違うことは構造的に起こらない。

    Returns:
        Dictionary with dot-notation-free keys and float values
    """
    flat: Dict[str, float] = {}
    for flat_key, path in _FLAT_KEY_PATHS.items():
        node: Any = DEFAULT_PARAMS
        for part in path:
            node = node[part]
        # "constants" leaves are plain floats; everything else is a
        # {"value": ..., "unit": ..., "description": ...} node.
        # "constants" 配下は素の float、それ以外は
        # {"value": ..., "unit": ..., "description": ...} 構造。
        flat[flat_key] = node["value"] if isinstance(node, dict) else node
    return flat
