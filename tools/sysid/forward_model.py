"""
Forward-model check, building block 1: logged duty -> body angular acceleration
順モデル検査・第1部品: 記録した duty -> 機体角加速度

Given a flight log (`.sflog.zip`), this module takes the LOGGED per-motor duty
(and battery voltage) and predicts the body angular acceleration (roll, pitch,
yaw) exactly the way the SILS plant (simulator/sils/plant/plant.cpp) computes
it, so that the prediction can later be compared with what a real vehicle did
(docs/architecture/simulation-policy.md, backlog #21). This file does NOT do
that comparison; its job is to reproduce the SILS plant, and the self-check is
test_forward_model.py (prediction vs. the SILS plant's own ground truth).
フライトログ(`.sflog.zip`)から、記録された各モータ duty (と電池電圧) を取り出し、
SILS プラント (simulator/sils/plant/plant.cpp) と全く同じ計算で機体角加速度
(ロール・ピッチ・ヨー) を予測する。実機との比較は後続の作業 (simulation-policy.md
バックログ #21) で、本ファイルは SILS プラントの再現までを担う。自己無矛盾性の
確認は test_forward_model.py (予測 vs SILS プラント自身の真値)。

Every term below is copied from the plant (file:line of simulator/sils/plant/):
以下の各項はプラントからの写し (simulator/sils/plant/ の file:line):

  0. PWM quantisation (the LOGGED duty is the float command BEFORE it):
     the firmware writes duty_counts = (uint32)(duty * 255) (truncation, 8-bit
     LEDC; firmware/vehicle/components/sf_hal_motor/motor_driver.cpp:163-164)
     and the SILS board turns that back into counts/255
     (simulator/sils/devices/virtual_board.cpp:501-510, esp_idf_host/driver/
     ledc.h:190). The plant therefore sees floor(duty*255)/255 (a mean loss of
     half a count, 0.2 % duty, and up to 0.4 % duty at a time).
     PWM 量子化 (ログの duty はこの前の float 指令): 8bit LEDC, 切り捨て。
  1. Duty path (plant.cpp:576-594): duty -> [optional transport delay, 4 kHz
     substeps] -> [optional slew limit, NOT modelled here: default 0 = off]
     -> clamp to [0, 1] (plant.cpp:606-608). Held constant within one 400 Hz
     control period (the plant steps 10 substeps of h = 0.25 ms per period,
     models/stampfly.xml:44 timestep, plant.cpp:516-531).
     duty 経路: 輸送遅れ (任意) -> スルー制限 (本ファイルは未実装, 既定 0=OFF) ->
     [0,1] クランプ。400Hz 制御周期ごとに保持, 周期内に h=0.25ms の substep 10 回。
  2. Motor ODE (plant.cpp:384-399): I_m = (V - Ke*w)/R_out(D) (0 when D < 0.01),
     J dw/dt = Kt*I_m - Cq*w^2 - Kt*(I0a + I0b*w^2), R_out(D) linearly
     interpolated from the 101-entry table (plant.cpp:365-382). Integrated
     with classical RK4 per substep, w clamped >= 0 (plant.cpp:610-619).
     モータ ODE: 上式, R_out(D) は表の線形補間, 1 substep ごとに古典 RK4, w>=0 に制限。
  3. Supply voltage (plant.cpp:564, 454-469): V is the plant's battery model
     V = OCV(SoC) - I_total*R_int, held within a substep, updated after each
     substep from the Coulomb count. The log only carries a 1 Hz "voltage"
     (INA3221 reading), so the default here re-runs the same battery model
     from the predicted motor currents (`voltage_mode="battery"`); the logged
     voltage can be used instead (`voltage_mode="log"`).
     電源電圧: プラントの電池モデル V=OCV(SoC)-I_total*R_int (substep 内保持, 毎 substep
     後に更新)。ログの voltage は 1Hz のみなので、既定は同じ電池モデルを予測電流から
     再計算する ("battery")。ログ値を使うことも可 ("log")。
  4. Thrust (plant.cpp:622): T_i = eta*Ct*w_i^2 * health_i * ge_mult. health = 1 and
     ground effect = 0 (plant.hpp:301 ge_gain = 0) are assumed.
     推力。health=1, 地面効果 OFF (ge_gain=0) を仮定。
  5. Roll/pitch torque: MuJoCo applies each thrust along its rotor site +Z at
     (+-0.023, +-0.023, +0.005) m (models/stampfly.xml:121-124, gear
     "0 0 1 0 0 0" at :129-132), so tau = r x F gives the arm moments for
     free; torque_authority = 1 (plant.hpp:211, bypass at plant.cpp:701).
     ロール/ピッチ: MuJoCo がロータ site の +Z に推力を加えるので r x F がそのまま
     腕のモーメント。torque_authority=1 (素通し)。
  6. Yaw torque (plant.cpp:645-647): per motor (Cq*w^2 + J*dw/dt) with
     dw/dt = (w_after - w_before)/h of that substep, sign -Z_FLU for CCW
     motors (FR, RL) and +Z_FLU for CW motors (RR, FL). This J*dw/dt reaction
     term IS included (it is the plant's rotor-inertia reaction torque).
     ヨー: モータ毎 (Cq*w^2 + J*dw/dt)。CCW(FR,RL)は -Z_FLU, CW(RR,FL)は +Z_FLU。
     J*dw/dt (ロータ慣性反作用) を含む。
  7. Rigid body (models/stampfly.xml:101 inertial, diag 9.16e-6 / 13.3e-6 /
     20.4e-6 kg m^2): MuJoCo integrates I*alpha + w x (I*w) = tau. The
     gyroscopic term w x (I*w) is included when the logged body rates are
     given (`gyro` argument) and omitted otherwise. Aerodynamic drag on the
     body and turbulence/wind torque are NOT modelled by the plant's torque
     path in the default scenarios (turbulence_n = 0, plant.cpp:726).
     剛体: I*alpha + w x (I*w) = tau。ジャイロ項は記録角速度を与えたときのみ含める。
     機体の空力抵抗モーメントはプラントに無い。乱流 (turbulence_n) は既定 0 で未実装。

Not modelled (state: 未実装): motor slew limit, ground effect, per-motor health,
torque_authority != 1, turbulence torque, contact torque on the ground, and the
prescribed "handling" (crash-recovery) trajectory.

Frames / 座標系: output is in the LOG gyro frame, body FRD (x forward, y right,
z down), so it can be compared with `gyro_*` as logged. The plant works in FLU
internally (plant.cpp:299); FLU->FRD flips y and z.
出力はログの gyro と同じ機体 FRD 系 (x前, y右, z下)。プラント内部は FLU。

Motor order / モータ順: the log columns are duty_FR, duty_RR, duty_RL, duty_FL,
which is the plant's index 0..3 (M1 FR, M2 RR, M3 RL, M4 FL; plant.cpp:144-146,
xml rotor1..4) -- no reshuffle needed.
ログ列 duty_FR/RR/RL/FL = プラントの添字 0..3 (並べ替え不要)。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from . import _generated_params as gp

# --------------------------------------------------------------------------
# Constants not in the SSOT-generated file (copied from plant.hpp Config defaults)
# SSOT 生成物に無い定数 (plant.hpp の Config 既定値から)
# --------------------------------------------------------------------------
PWM_RESOLUTION_BITS = 8            # actuator.cpp:175 MOTOR_PWM_RESOLUTION_BIT / ledc.h:190
PLANT_SUBSTEP_S = 0.00025          # models/stampfly.xml:44 timestep [s]
DUTY_DRIVE_MIN = 0.01              # plant.cpp:388 / ROUT_D[1]: below this = no drive
BATT_CAPACITY_MAH = 300.0          # plant.hpp:320
BATT_R_INT_OHM = 0.1               # plant.hpp:321
BATT_FULL_V = 4.2                  # plant.hpp:322
BATT_EMPTY_V = 3.3                 # plant.hpp:323
AVIONICS_CURRENT_A = 0.1           # plant.hpp:325
AS_PER_MAH = 3.6                   # A*s per mAh (plant.cpp:463)
OCV_KNEE_HIGH = 0.95               # plant.cpp:442 SoC above which OCV is steep
OCV_KNEE_LOW = 0.05                # plant.cpp:443 SoC below which OCV is steep
OCV_STEEP_OFFSET = 0.9             # plant.cpp:443
OCV_STEEP_GAIN = 2.0               # plant.cpp:443-444

# SILS-only log convention: against the plant's own truth the prediction lines up best
# when delayed by exactly one plant substep (measured on sysid_gate: roll/pitch/yaw
# residual 1.3/1.5/2.4 % -> 0.01/0.28/0.07 %; cause in the SILS scheduler/log ordering
# NOT traced -> 未確認). A real vehicle has its own delay: fit it, do not use this.
# SILS 専用のログ規約: プラント自身の真値と比べると予測を 1 substep (0.25 ms) 遅らせた
# ときに最もよく合う (原因はスケジューラとログ順序、未追跡 = 未確認)。実機では使わず
# 実機側の遅れを当てはめる。
SILS_LOG_ALIGNMENT_DELAY_S = PLANT_SUBSTEP_S
AIRBORNE_MIN_ALT_M = 0.05          # rows below this are excluded (ground contact is not modelled)

# Rotor sites in body FLU [m] (models/stampfly.xml:121-124) and spin sign.
# Order = log columns FR, RR, RL, FL. Reaction torque sign about +Z_FLU:
# CCW props (FR, RL) -> -1, CW props (RR, FL) -> +1 (plant.cpp:645-647).
# ロータ位置 (FLU) と反トルク符号。順序はログ列 FR, RR, RL, FL。
MOTOR_NAMES = ("FR", "RR", "RL", "FL")
_ARM = gp.EXPECTED_ARM
MOTOR_X_FLU = np.array([+_ARM, -_ARM, -_ARM, +_ARM])
MOTOR_Y_FLU = np.array([-_ARM, -_ARM, +_ARM, +_ARM])
MOTOR_REACTION_SIGN_Z_FLU = np.array([-1.0, +1.0, -1.0, +1.0])


@dataclass(frozen=True)
class PlantConstants:
    """Propulsion + rigid-body constants of the SILS plant (defaults = SSOT).
    SILS プラントの推進系・剛体定数 (既定は SSOT 生成値)。"""

    ke: float = gp.EXPECTED_PLANT_KE
    kt: float = gp.EXPECTED_PLANT_KT
    cq: float = gp.EXPECTED_PLANT_CQ
    ct: float = gp.EXPECTED_PLANT_CT
    rotor_inertia: float = gp.EXPECTED_PLANT_J
    i0a: float = gp.EXPECTED_PLANT_I0A
    i0b: float = gp.EXPECTED_PLANT_I0B
    thrust_efficiency: float = gp.EXPECTED_PLANT_THRUST_EFFICIENCY
    rout_duty: tuple = gp.PLANT_ROUT_D
    rout_ohm: tuple = gp.PLANT_ROUT_OHM
    inertia_diag: tuple = (gp.EXPECTED_IXX, gp.EXPECTED_IYY, gp.EXPECTED_IZZ)


DEFAULT_CONSTANTS = PlantConstants()


def quantise_duty(duty, bits: Optional[int] = PWM_RESOLUTION_BITS):
    """Duty as the plant receives it: floor(duty*(2^bits-1))/(2^bits-1) in float32
    (motor_driver.cpp:163-164, virtual_board.cpp:506-510). bits=None: pass through.
    プラントが受け取る duty。切り捨て量子化 (float32)。bits=None なら素通し。"""
    duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
    if bits is None:
        return duty
    counts = np.float32((1 << bits) - 1)
    return (np.floor(duty.astype(np.float32) * counts) / counts).astype(float)


# --------------------------------------------------------------------------
# Motor model / モータモデル
# --------------------------------------------------------------------------
def rout_ohm(duty, const: PlantConstants = DEFAULT_CONSTANTS):
    """R_out(D) [ohm], linear interpolation, clamped like plant.cpp:365-382.
    R_out(D) [Ω]。plant.cpp と同じ線形補間とクランプ。"""
    d = np.clip(duty, DUTY_DRIVE_MIN, 1.0)
    return np.interp(d, const.rout_duty, const.rout_ohm)


def motor_current(omega, duty, voltage, rout, const: PlantConstants = DEFAULT_CONSTANTS):
    """Mean motor current I_m [A] (plant.cpp:384-391); 0 when D < 0.01.
    平均モータ電流 [A]。D<0.01 で 0。"""
    current = (voltage - const.ke * omega) / rout
    return np.where(duty < DUTY_DRIVE_MIN, 0.0, current)


def omega_dot(omega, duty, voltage, rout, const: PlantConstants = DEFAULT_CONSTANTS):
    """Right-hand side of the motor ODE [rad/s^2] (plant.cpp:393-399).
    モータ ODE の右辺 [rad/s^2]。"""
    i_m = motor_current(omega, duty, voltage, rout, const)
    drag = const.cq * omega**2 + const.kt * (const.i0a + const.i0b * omega**2)
    return (const.kt * i_m - drag) / const.rotor_inertia


def ocv_from_charge(charge_mah: float) -> float:
    """Open-circuit voltage [V] from remaining charge (plant.cpp:436-451).
    残容量から開回路電圧 [V]。"""
    soc = min(max(charge_mah / BATT_CAPACITY_MAH, 0.0), 1.0)
    if soc > OCV_KNEE_HIGH:
        adj = OCV_STEEP_OFFSET + (soc - OCV_STEEP_OFFSET) * OCV_STEEP_GAIN
    elif soc < OCV_KNEE_LOW:
        adj = soc * OCV_STEEP_GAIN
    else:
        adj = soc
    volts = BATT_EMPTY_V + adj * (BATT_FULL_V - BATT_EMPTY_V)
    return min(max(volts, BATT_EMPTY_V), BATT_FULL_V)


# --------------------------------------------------------------------------
# Motor simulation / モータのシミュレーション
# --------------------------------------------------------------------------
@dataclass
class MotorTrace:
    """Motor state at plant-substep resolution (M = n_rows * substeps_per_row).
    プラント substep 分解能のモータ状態 (M = 行数 × 1行あたり substep 数)。

    omega:      rotor speed AFTER each substep [rad/s], shape (M, 4)
    omega_dot:  realised (w_after - w_before)/h of each substep [rad/s^2], (M, 4)
    voltage:    supply voltage used by each substep [V], (M,)
    substep_s:  h [s];  substeps_per_row: control period / h
    """

    omega: np.ndarray
    omega_dot: np.ndarray
    voltage: np.ndarray
    substep_s: float
    substeps_per_row: int


def simulate_motors(
    duty: np.ndarray,
    dt: float,
    voltage: Optional[np.ndarray] = None,
    const: PlantConstants = DEFAULT_CONSTANTS,
    substep_s: float = PLANT_SUBSTEP_S,
    omega_init: Optional[np.ndarray] = None,
    battery_initial_frac: float = 1.0,
    pre_elapsed_s: float = 0.0,
    pwm_bits: Optional[int] = PWM_RESOLUTION_BITS,
    charge_float32: bool = True,
) -> MotorTrace:
    """Integrate the four motor ODEs exactly like plant.cpp substep().
    plant.cpp substep() と同じ手順で 4 モータの ODE を積分する。

    Args:
        duty: (N, 4) per-row duty in log order FR, RR, RL, FL; row k is held over
            [t_k, t_k + dt). 行 k は [t_k, t_k+dt) の間保持される。
        dt: control period [s] (0.0025 for 400 Hz).
        voltage: (N,) supply voltage per row, held; None -> run the plant's battery
            model (V = OCV - I*R_int) from the predicted currents.
            行ごとに保持する電源電圧。None なら電池モデルで再計算。
        omega_init: (4,) initial rotor speed; default = steady state of duty[0]
            at the starting voltage (plant.cpp:161 primeMotors) -- for a log that
            starts with motors off this is 0.
        pwm_bits: LEDC resolution the logged float duty is quantised with (8);
            None = feed the logged duty unquantised.
            ログの float duty を量子化する LEDC 分解能 (8)。None で無量子化。
        charge_float32: keep the Coulomb-count state in float32 like the plant
            (plant.hpp `float batt_charge_mah_`). IMPORTANT: at ~300 mAh a float32
            step is 3e-5 mAh, bigger than the 7e-6 mAh the avionics-only draw removes
            per 0.25 ms substep, so the plant's idle drain rounds to ZERO until the
            motors run; this moves the OCV step at SoC = 0.95 (plant.cpp:442) by
            seconds and changes thrust by ~1.5 % around it. False = ideal double
            arithmetic (what a real battery would do).
            クーロンカウント状態をプラントと同じ float32 で持つ。~300mAh では float32 の
            刻みが avionics のみの消費量より大きく、待機中の消費が 0 に丸められる。
            SoC=0.95 の OCV 段差の時刻が秒単位でずれ、その前後で推力が約1.5%変わる。
            False なら倍精度 (実電池の挙動)。
        battery_initial_frac / pre_elapsed_s: battery model start; the avionics
            draw during `pre_elapsed_s` before the first row is Coulomb-counted.
            電池モデルの初期条件。最初の行より前の avionics 電流による消費も数える。

    Returns:
        MotorTrace at substep resolution (no delay applied; use `row_mean` with
        `delay_s` to shift, which is exact for this time-invariant system).
        substep 分解能の MotorTrace (遅れは未適用。`row_mean` の delay_s で
        ずらす。時不変系なので等価)。
    """
    duty = quantise_duty(duty, pwm_bits)  # firmware PWM, then plant.cpp:606-608 clamp
    n_rows = duty.shape[0]
    per_row = int(round(dt / substep_s))
    if abs(per_row * substep_s - dt) > 1e-9:
        raise ValueError(f"dt={dt} is not an integer multiple of substep {substep_s}")
    duty_sub = np.repeat(duty, per_row, axis=0)
    rout_sub = rout_ohm(duty_sub, const)
    n_sub = duty_sub.shape[0]

    use_battery = voltage is None
    volt_sub = None if use_battery else np.repeat(np.asarray(voltage, float), per_row)
    f32 = np.float32 if charge_float32 else float
    charge = f32(battery_initial_frac * BATT_CAPACITY_MAH)
    # Avionics-only drain before the first row, stepped like the plant (substep by substep)
    # so that float32 rounding behaves identically.
    # 最初の行より前の avionics のみの消費。プラントと同じ substep 刻みで引く。
    pre_decrement = f32(f32(AVIONICS_CURRENT_A) * f32(substep_s) / f32(AS_PER_MAH))
    for _ in range(int(round(pre_elapsed_s / substep_s))):
        charge = max(charge - pre_decrement, f32(0.0))
    v_now = ocv_from_charge(float(charge)) - AVIONICS_CURRENT_A * BATT_R_INT_OHM

    omega = (np.zeros(4) if omega_init is None else np.asarray(omega_init, float).copy())
    out_omega = np.empty((n_sub, 4))
    out_dot = np.empty((n_sub, 4))
    out_volt = np.empty(n_sub)
    h = substep_s
    for j in range(n_sub):
        d, r = duty_sub[j], rout_sub[j]
        v = v_now if use_battery else volt_sub[j]
        k1 = omega_dot(omega, d, v, r, const)
        k2 = omega_dot(omega + 0.5 * h * k1, d, v, r, const)
        k3 = omega_dot(omega + 0.5 * h * k2, d, v, r, const)
        k4 = omega_dot(omega + h * k3, d, v, r, const)
        w_next = np.maximum(omega + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4), 0.0)
        out_dot[j] = (w_next - omega) / h
        omega = w_next
        out_omega[j] = omega
        out_volt[j] = v
        if use_battery:
            i_total = AVIONICS_CURRENT_A + float(
                np.sum(np.maximum(motor_current(omega, d, v, r, const), 0.0))
            )
            charge = max(charge - f32(f32(i_total) * f32(h) / f32(AS_PER_MAH)), f32(0.0))
            v_now = ocv_from_charge(float(charge)) - i_total * BATT_R_INT_OHM
    return MotorTrace(out_omega, out_dot, out_volt, h, per_row)


def body_torque(trace: MotorTrace, const: PlantConstants = DEFAULT_CONSTANTS) -> np.ndarray:
    """Body torque [N m] about FRD x/y/z at substep resolution, shape (M, 3).
    FRD の x/y/z まわり機体トルク [N m] (substep 分解能)。

    Roll/pitch: r x F of the four thrusts T_i = eta*Ct*w_i^2 (plant.cpp:622) at the
    rotor sites; yaw: sum of +-(Cq*w^2 + J*dw/dt) (plant.cpp:645-647).
    Torque uses the post-substep omega, as the plant does.
    ロール/ピッチ: 4 推力の r x F。ヨー: ±(Cq w^2 + J dw/dt)。プラント同様
    substep 後の omega を使う。
    """
    w2 = trace.omega**2
    thrust = const.thrust_efficiency * const.ct * w2
    tau_x_flu = thrust @ MOTOR_Y_FLU          # r x F: tau_x = y*Fz
    tau_y_flu = -(thrust @ MOTOR_X_FLU)       # tau_y = -x*Fz
    reaction = const.cq * w2 + const.rotor_inertia * trace.omega_dot
    tau_z_flu = reaction @ MOTOR_REACTION_SIGN_Z_FLU
    # FLU -> FRD: y and z flip / FLU -> FRD は y,z 反転
    return np.stack([tau_x_flu, -tau_y_flu, -tau_z_flu], axis=1)


def row_mean(series_sub: np.ndarray, trace: MotorTrace, delay_s: float = 0.0) -> np.ndarray:
    """Average a substep-resolution series over each control period, optionally
    delayed by `delay_s` (a duty transport delay of d equals shifting the whole
    response by d). Delay is rounded to whole substeps; before the series starts
    the first value is used.
    substep 系列を制御周期ごとに平均する。duty の輸送遅れ d は応答全体を d ずらすのと
    等価。遅れは substep 単位に丸める。系列の手前は先頭値で埋める。"""
    per_row = trace.substeps_per_row
    shift = int(round(delay_s / trace.substep_s))
    x = np.asarray(series_sub)
    if shift > 0:
        pad = np.repeat(x[:1], shift, axis=0)
        x = np.concatenate([pad, x[:-shift]], axis=0)
    elif shift < 0:
        pad = np.repeat(x[-1:], -shift, axis=0)
        x = np.concatenate([x[-shift:], pad], axis=0)
    n_rows = x.shape[0] // per_row
    return x.reshape(n_rows, per_row, *x.shape[1:]).mean(axis=1)


def angular_accel(
    torque: np.ndarray,
    gyro: Optional[np.ndarray] = None,
    const: PlantConstants = DEFAULT_CONSTANTS,
) -> np.ndarray:
    """alpha = I^-1 (tau - w x I w) in FRD; w x I w only if `gyro` (rad/s) is given.
    FRD での alpha = I^-1 (tau - w x I w)。ジャイロ項は gyro を与えたときのみ。"""
    inertia = np.asarray(const.inertia_diag)
    net = np.asarray(torque, float).copy()
    if gyro is not None:
        net -= np.cross(gyro, gyro * inertia)
    return net / inertia


# --------------------------------------------------------------------------
# Flight-log front end / フライトログ入口
# --------------------------------------------------------------------------
@dataclass
class LoggedMotion:
    """What the forward model needs from a flight log.
    順モデルがログから必要とするもの。"""

    t_s: np.ndarray                    # (N,) row time [s]
    dt: float                          # control period [s]
    duty: np.ndarray                   # (N, 4) FR, RR, RL, FL
    gyro: np.ndarray                   # (N, 3) FRD [rad/s]
    voltage: Optional[np.ndarray]      # (N,) logged supply voltage [V] (1 Hz held) or None


_DUTY_COLUMNS = [f"duty_{name}" for name in MOTOR_NAMES]
_GYRO_COLUMNS = ["gyro_x", "gyro_y", "gyro_z"]
UNIFORM_DT_TOLERANCE = 0.02  # relative spread of timestamp spacing allowed


def load_logged_motion(path) -> LoggedMotion:
    """Load a .sflog.zip (or extracted bundle) via tools/sysid/loader.load_aligned
    and return duty / voltage / gyro on the imu time base, rows where duty exists.
    load_aligned 経由でログを読み、duty が存在する行の duty・電圧・gyro を返す。"""
    from .loader import load_aligned

    df = load_aligned(path)
    missing = [c for c in _DUTY_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: no motor duty columns {missing} (motor stream absent)")
    df = df[df[_DUTY_COLUMNS].notna().all(axis=1)].reset_index(drop=True)
    t_s = df["timestamp_us"].to_numpy(dtype=float) * 1e-6
    spacing = np.diff(t_s)
    dt = float(np.median(spacing))
    if spacing.size and (spacing.max() - spacing.min()) > UNIFORM_DT_TOLERANCE * dt:
        raise ValueError(f"{path}: non-uniform row spacing (rows were dropped or reused)")
    voltage = df["voltage"].to_numpy(dtype=float) if "voltage" in df.columns else None
    return LoggedMotion(
        t_s=t_s,
        dt=dt,
        duty=df[_DUTY_COLUMNS].to_numpy(dtype=float),
        gyro=df[_GYRO_COLUMNS].to_numpy(dtype=float),
        voltage=voltage,
    )


def predict_torque_trace(
    motion: LoggedMotion,
    voltage_mode="battery",
    const: PlantConstants = DEFAULT_CONSTANTS,
    pwm_bits: Optional[int] = PWM_RESOLUTION_BITS,
):
    """Run the motor simulation once; returns (trace, torque_substeps (M,3)).
    Reuse it for many delays through `accel_from_trace`.
    モータシミュレーションを 1 回だけ走らせる。遅れを変えた複数予測は accel_from_trace で。

    voltage_mode: "battery" (plant battery model re-run; default), "log" (logged
    1 Hz voltage held), or a float (constant volts, e.g. plant.hpp v_batt 3.7).
    "battery"=電池モデル再計算 / "log"=ログ電圧保持 / 数値=定電圧。
    """
    if voltage_mode == "battery":
        volt = None
    elif voltage_mode == "log":
        if motion.voltage is None:
            raise ValueError("log has no voltage column")
        volt = motion.voltage
    else:
        volt = np.full(len(motion.t_s), float(voltage_mode))
    trace = simulate_motors(
        motion.duty, motion.dt, volt, const,
        pre_elapsed_s=float(motion.t_s[0]), pwm_bits=pwm_bits,
    )
    return trace, body_torque(trace, const)


def accel_from_trace(
    motion: LoggedMotion,
    trace: MotorTrace,
    torque_sub: np.ndarray,
    delay_s: float = 0.0,
    include_gyroscopic: bool = True,
    const: PlantConstants = DEFAULT_CONSTANTS,
) -> np.ndarray:
    """Mean body angular acceleration per control period, FRD, (N, 3), for one delay.
    1 つの遅れに対する制御周期ごとの機体角加速度 (周期内平均), FRD, (N, 3)。"""
    tau = row_mean(torque_sub, trace, delay_s)
    gyro = None
    if include_gyroscopic:
        # Mean of the rates at both ends of the period / 周期両端の角速度の平均
        gyro = 0.5 * (motion.gyro + np.vstack([motion.gyro[1:], motion.gyro[-1:]]))
    return angular_accel(tau, gyro, const)


def predict_angular_accel(
    motion: LoggedMotion,
    voltage_mode="battery",
    delay_s: float = 0.0,
    include_gyroscopic: bool = True,
    const: PlantConstants = DEFAULT_CONSTANTS,
) -> np.ndarray:
    """One-shot prediction (see predict_torque_trace / accel_from_trace).
    1 回限りの予測 (詳細は predict_torque_trace / accel_from_trace)。"""
    trace, torque_sub = predict_torque_trace(motion, voltage_mode, const)
    return accel_from_trace(motion, trace, torque_sub, delay_s, include_gyroscopic, const)


def load_truth(path):
    """Ground truth of a SILS bundle: (t_s, body rate FRD (K,3) [rad/s], altitude (K,) [m]).
    truth.csv has NED position (z down) and body rates in FRD.
    SILS バンドルの真値: (時刻, 機体角速度 FRD, 高度)。pos_z は NED (下向き正)。"""
    import zipfile
    from pathlib import Path
    import pandas as pd

    p = Path(path)
    if p.is_dir():
        df = pd.read_csv(p / "truth.csv")
    else:
        with zipfile.ZipFile(p) as zf:
            with zf.open("truth.csv") as fh:
                df = pd.read_csv(fh)
    t = df["timestamp_us"].to_numpy(dtype=float) * 1e-6
    rate = df[["rate_x", "rate_y", "rate_z"]].to_numpy(dtype=float)
    return t, rate, -df["pos_z"].to_numpy(dtype=float)


# --------------------------------------------------------------------------
# Comparison helpers / 比較ヘルパー
# --------------------------------------------------------------------------
def accel_from_rate(
    t_s: np.ndarray, rate: np.ndarray, dt: float, shift_s: float = 0.0,
    t_src: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Mean angular acceleration over [t_k, t_k+dt) from a rate series:
    (w(t_k+dt+shift) - w(t_k+shift))/dt, linear interpolation. Last row is NaN.
    角速度系列から [t_k, t_k+dt) の平均角加速度を求める (線形補間)。最終行は NaN。
    `t_src`: time stamps of `rate` when they differ from the row times `t_s` (e.g. truth.csv).
    `t_src`: rate の時刻が行時刻 t_s と異なるとき (truth.csv など) に指定。"""
    rate = np.asarray(rate, float)
    src = t_s if t_src is None else t_src
    out = np.empty((len(t_s), rate.shape[1]))
    for axis in range(rate.shape[1]):
        w0 = np.interp(t_s + shift_s, src, rate[:, axis])
        w1 = np.interp(t_s + dt + shift_s, src, rate[:, axis])
        out[:, axis] = (w1 - w0) / dt
    out[-1] = np.nan
    return out


def relative_residual(ref: np.ndarray, pred: np.ndarray, mask: Optional[np.ndarray] = None) -> np.ndarray:
    """Per-axis sqrt(mean((ref-pred)^2)) / sqrt(mean(ref^2)) over `mask` rows.
    軸ごとの相対残差 (mask 行のみ)。"""
    ok = np.all(np.isfinite(ref) & np.isfinite(pred), axis=1)
    if mask is not None:
        ok &= mask
    err = np.sqrt(np.mean((ref[ok] - pred[ok]) ** 2, axis=0))
    scale = np.sqrt(np.mean(ref[ok] ** 2, axis=0))
    return err / scale


def best_delay_and_scale(
    ref: np.ndarray,
    trace_series: Sequence[np.ndarray],
    delays_s: Sequence[float],
    mask: Optional[np.ndarray] = None,
):
    """For one axis find the delay (index into `delays_s`) minimising the relative
    residual, plus the least-squares scale ref ~ scale*pred.
    1 軸について相対残差が最小になる遅れと最小二乗スケールを返す。
    `trace_series[i]` is the prediction for delays_s[i], shape (N,).
    Returns (best_delay_s, scale, residual_at_best, residual_at_best_after_scale)."""
    best = None
    for delay, pred in zip(delays_s, trace_series):
        ok = np.isfinite(ref) & np.isfinite(pred)
        if mask is not None:
            ok &= mask
        denom = np.sqrt(np.mean(ref[ok] ** 2))
        res = np.sqrt(np.mean((ref[ok] - pred[ok]) ** 2)) / denom
        if best is None or res < best[2]:
            scale = float(np.dot(ref[ok], pred[ok]) / np.dot(pred[ok], pred[ok]))
            res_s = np.sqrt(np.mean((ref[ok] - scale * pred[ok]) ** 2)) / denom
            best = (float(delay), scale, float(res), float(res_s))
    return best
