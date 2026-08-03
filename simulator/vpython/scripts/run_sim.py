# MIT License
#
# Copyright (c) 2025 Kouhei Ito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
High-Frequency Physical Units Mode Flight Simulation (Experimental)
高周波物理単位モードフライトシミュレーション（実験版）

Timing configuration:
タイミング設定:
- Physics: 1000Hz (h = 1.0ms)
- Control: 200Hz (5 physics steps per control)
- Visualization: 30Hz

Loop structure: outer render loop + inner control loop
ループ構造: 外側レンダリングループ + 内側制御ループ

Real-time synchronization via VPython rate().
VPython rate()によるリアルタイム同期。
"""

import sys
import os
import gc
import math
import time
import numpy as np
import matplotlib.pyplot as plt

# Add vpython package to path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_VPYTHON_DIR = os.path.dirname(_SCRIPT_DIR)
if _VPYTHON_DIR not in sys.path:
    sys.path.insert(0, _VPYTHON_DIR)

from core import dynamics as mc
from core import motors as motor_model
from visualization.vpython_backend import render
from vpython import *
from control.pid import PID
from interfaces.joystick import Joystick


# =============================================================================
# Motor Model Parameters (matching firmware motor_model.hpp)
# =============================================================================
# Single source of truth: the PLANT's own motor model (core/motors.py). The
# inverse model below (thrust -> voltage) must use the exact same parameters
# as the plant it is inverting, or the hover equilibrium drifts — a hand-
# copied parameter set here silently diverged after the 2026-07-15 sysid
# update (plant kept old Ct/electrical package, this copy got new Ct) and
# the sim climbed continuously at neutral stick. Deriving from the plant
# instance makes controller∘plant self-consistent BY CONSTRUCTION: future
# sysid updates to core/motors.py can never reintroduce a trim offset.
# 単一の真実はプラント自身のモータモデル（core/motors.py）。下の逆モデル
# （推力→電圧）は反転対象のプラントと厳密に同じパラメータを使わなければ
# ホバー平衡がずれる — ここに手書き複製されたパラメータは 2026-07-15 の
# 同定更新後に黙って乖離し（プラントは旧 Ct/電気系パッケージのまま、複製側
# だけ新 Ct）、スティック中立でも上昇し続けた。プラントのインスタンスから
# 導出することで controller∘plant は「構造的に」自己整合になり、今後
# core/motors.py の同定更新でトリムずれが再発することはない。
_PLANT_MOTOR = motor_model.motor_prop(1)


class MotorParams:
    """Motor parameters for thrust-to-voltage conversion.
    Derived from the plant model — do NOT hand-copy numbers here.
    プラントモデルから導出 — 数値を手書き複製しないこと。"""
    Ct = _PLANT_MOTOR.Ct      # Thrust coefficient [N/(rad/s)²]
    Cq = _PLANT_MOTOR.Cq      # Torque coefficient [Nm/(rad/s)²]
    Rm = _PLANT_MOTOR.Rm      # Motor resistance [Ω]
    Km = _PLANT_MOTOR.Km      # Motor constant [V·s/rad]
    Dm = _PLANT_MOTOR.Dm      # Viscous damping [Nm·s/rad]
    Qf = _PLANT_MOTOR.Qf      # Friction torque [Nm]
    Vbat = 3.7                # Battery voltage [V] (supply, not a motor param)


def thrust_to_omega(thrust: float) -> float:
    """Convert thrust to angular velocity (steady-state)"""
    if thrust <= 0:
        return 0.0
    return math.sqrt(thrust / MotorParams.Ct)


def omega_to_voltage(omega: float) -> float:
    """Convert angular velocity to required voltage (steady-state)"""
    if omega <= 0:
        return 0.0
    p = MotorParams
    viscous_term = (p.Dm + p.Km * p.Km / p.Rm) * omega
    aero_term = p.Cq * omega * omega
    friction_term = p.Qf
    voltage = p.Rm * (viscous_term + aero_term + friction_term) / p.Km
    return voltage


def thrust_to_voltage(thrust: float) -> float:
    """Convert thrust to voltage (steady-state approximation)"""
    if thrust <= 0:
        return 0.0
    omega = thrust_to_omega(thrust)
    voltage = omega_to_voltage(omega)
    return min(max(voltage, 0.0), MotorParams.Vbat)


# =============================================================================
# Control Allocation (matching firmware control_allocation.cpp)
# =============================================================================
class ControlAllocator:
    """
    Control allocation for X-Quad using B⁻¹ matrix
    X-Quad用のB⁻¹行列による制御配分
    """
    def __init__(self):
        # Quad parameters
        self.d = 0.023          # Moment arm [m]
        # kappa derived from the plant (Cq/Ct = 4.10e-3 as of 2026-08-03,
        # core/motors.py) — matches the firmware mixer KAPPA. History:
        # 9.71e-3 (stale hand-copy) -> 6.12e-3 (2026-07-15..2026-08-02,
        # thrust-stand Ct) -> 4.10e-3 (2026-08-03, thrust-stand Ct retracted,
        # see core/motors.py's Ct comment for the full provenance).
        # kappa はプラントから導出（Cq/Ct = 4.10e-3、2026-08-03時点、
        # core/motors.py）— ファームのミキサー KAPPA と一致。履歴:
        # 9.71e-3（陳腐化した手書き複製）→ 6.12e-3（2026-07-15〜2026-08-02、
        # thrust stand Ct）→ 4.10e-3（2026-08-03、thrust stand Ct 撤回。
        # 詳細は core/motors.py の Ct コメント参照）。
        self.kappa = MotorParams.Cq / MotorParams.Ct
        self.max_thrust = 0.15  # Max thrust per motor [N]

        # Build B⁻¹ matrix
        inv_d = 1.0 / self.d
        inv_kappa = 1.0 / self.kappa

        # B⁻¹ matrix: T = B⁻¹ × [total_thrust, roll_torque, pitch_torque, yaw_torque]
        # Motor order: M1(FR), M2(RR), M3(RL), M4(FL)
        self.B_inv = np.array([
            [0.25, -0.25 * inv_d,  0.25 * inv_d,  0.25 * inv_kappa],  # M1 (FR)
            [0.25, -0.25 * inv_d, -0.25 * inv_d, -0.25 * inv_kappa],  # M2 (RR)
            [0.25,  0.25 * inv_d, -0.25 * inv_d,  0.25 * inv_kappa],  # M3 (RL)
            [0.25,  0.25 * inv_d,  0.25 * inv_d, -0.25 * inv_kappa],  # M4 (FL)
        ])

        # Pre-allocate buffers (avoids 200 alloc/s)
        # バッファ事前割り当て（200回/秒のalloc回避）
        self._control_buf = np.zeros(4)
        self._thrust_buf = np.zeros(4)

    def mix(self, total_thrust: float, roll_torque: float,
            pitch_torque: float, yaw_torque: float) -> np.ndarray:
        """
        Convert control inputs to motor thrusts
        制御入力をモータ推力に変換
        """
        self._control_buf[0] = total_thrust
        self._control_buf[1] = roll_torque
        self._control_buf[2] = pitch_torque
        self._control_buf[3] = yaw_torque
        np.dot(self.B_inv, self._control_buf, out=self._thrust_buf)
        np.clip(self._thrust_buf, 0.0, self.max_thrust, out=self._thrust_buf)
        return self._thrust_buf

    def thrusts_to_voltages(self, thrusts: np.ndarray) -> list:
        """Convert motor thrusts to voltages (returns list for direct use)"""
        return [thrust_to_voltage(thrusts[0]), thrust_to_voltage(thrusts[1]),
                thrust_to_voltage(thrusts[2]), thrust_to_voltage(thrusts[3])]


# =============================================================================
# Physical Units PID Configuration (matching corrected firmware config.hpp)
# =============================================================================
class PhysicalUnitsPIDConfig:
    """
    PID configuration with corrected k_τ conversion
    修正されたk_τ変換係数を使用したPID設定
    """
    # Rate sensitivity (matching firmware)
    roll_rate_max = 1.0    # rad/s (~57 deg/s)
    pitch_rate_max = 1.0   # rad/s (~57 deg/s)
    yaw_rate_max = 5.0     # rad/s (~286 deg/s)

    # Roll Rate PID [Nm/(rad/s)]
    roll_kp = 9.1e-4
    roll_ti = 0.7
    roll_td = 0.01
    roll_limit = 5.2e-3  # Nm

    # Pitch Rate PID [Nm/(rad/s)]
    pitch_kp = 1.33e-3
    pitch_ti = 0.7
    pitch_td = 0.025
    pitch_limit = 5.2e-3  # Nm

    # Yaw Rate PID [Nm/(rad/s)]
    yaw_kp = 1.77e-3
    yaw_ti = 0.8
    yaw_td = 0.01
    yaw_limit = 2.2e-3  # Nm

    # Common
    eta = 0.125


# =============================================================================
# Main Simulation
# =============================================================================
def flight_sim_2000hz(world_type='voxel', seed=None, control_mode='rate',
                      no_joystick=False):
    """
    Flight simulation with high-frequency physics and control
    高周波物理・制御によるフライトシミュレーション

    Timing:
    - Physics: 2000Hz
    - Control: 400Hz
    - Visualization: 60Hz
    """
    # ===========================================
    # Timing Configuration
    # ===========================================
    PHYSICS_HZ = 1000
    PHYSICS_DT = 1.0 / PHYSICS_HZ  # 0.001s = 1.0ms

    CONTROL_HZ = 200
    CONTROL_DT = 1.0 / CONTROL_HZ  # 0.005s = 5.0ms
    PHYSICS_PER_CONTROL = PHYSICS_HZ // CONTROL_HZ  # 5 physics steps per control

    RENDER_FPS = 30
    RENDER_DT = 1.0 / RENDER_FPS  # ~33.33ms

    # Number of control cycles per render frame to maintain real-time
    # リアルタイム維持のための1描画フレームあたりの制御サイクル数
    # 400Hz control / 30 FPS ≈ 13.3
    CONTROLS_PER_FRAME = CONTROL_HZ // RENDER_FPS  # 13

    print("=" * 60)
    print("High-Frequency Simulation (Experimental)")
    print("高周波シミュレーション（実験版）")
    print("=" * 60)
    print(f"Physics:       {PHYSICS_HZ} Hz (dt = {PHYSICS_DT*1000:.2f} ms)")
    print(f"Control:       {CONTROL_HZ} Hz (dt = {CONTROL_DT*1000:.2f} ms)")
    print(f"Visualization: {RENDER_FPS} FPS")
    print(f"Physics steps per control: {PHYSICS_PER_CONTROL}")
    print(f"Controls per frame:  {CONTROLS_PER_FRAME}")
    print(f"Effective physics:   {RENDER_FPS * CONTROLS_PER_FRAME * PHYSICS_PER_CONTROL} Hz")
    print("=" * 60)

    # ===========================================
    # Drone Setup
    # ===========================================
    # 実測36.8g、2026-07-24統一（旧0.035は更新漏れ）
    mass = 0.037
    weight = mass * 9.81
    hover_thrust = weight

    stampfly = mc.multicopter(
        mass=mass,
        inersia=[[9.16e-6, 0.0, 0.0], [0.0, 13.3e-6, 0.0], [0.0, 0.0, 20.4e-6]]
    )

    # Initialize renderer
    Render = render(RENDER_FPS, world_type=world_type, seed=seed)
    Render.physics_hz = PHYSICS_HZ  # For timing report / タイミングレポート用

    # Get safe spawn position
    spawn_x, spawn_y, spawn_z = Render.get_safe_spawn_position(x=0.0, y=0.0, clearance=1.0)
    stampfly.body.set_position([[spawn_x], [spawn_y], [spawn_z]])
    print(f"Spawn position: ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f})")

    # Initialize joystick (skipped with --no-joystick; open() failure also
    # leaves device=None and the sim keeps running without stick input)
    # ジョイスティック初期化（--no-joystick 時はスキップ。open() 失敗時も
    # device=None のままスティック入力なしでシミュレーションを継続する）
    joystick = Joystick()
    if not no_joystick:
        joystick.open()

    # Initialize drone state
    stampfly.set_pqr([[0.0], [0.0], [0.0]])
    stampfly.set_uvw([[0.0], [0.0], [0.0]])
    stampfly.set_euler([[0], [0], [0]])

    # Initialize motors at hover
    nominal_omega = stampfly.motor_prop[0].equilibrium_anguler_velocity(weight / 4)
    for mp in stampfly.motor_prop:
        mp.omega = nominal_omega

    stampfly.set_disturbance(moment=[0, 0, 0], force=[0, 0, 0])

    # Initialize control allocator
    allocator = ControlAllocator()

    # ===========================================
    # PID Controllers
    # ===========================================
    cfg = PhysicalUnitsPIDConfig

    # Rate PID (inner loop) - output is torque [Nm]
    roll_rate_pid = PID(cfg.roll_kp, cfg.roll_ti, cfg.roll_td,
                        output_min=-cfg.roll_limit, output_max=cfg.roll_limit)
    pitch_rate_pid = PID(cfg.pitch_kp, cfg.pitch_ti, cfg.pitch_td,
                         output_min=-cfg.pitch_limit, output_max=cfg.pitch_limit)
    yaw_rate_pid = PID(cfg.yaw_kp, cfg.yaw_ti, cfg.yaw_td,
                       output_min=-cfg.yaw_limit, output_max=cfg.yaw_limit)

    # Attitude PID (outer loop) - output is angular rate [rad/s]
    roll_pid = PID(5.0, 1.0, 0.0)
    pitch_pid = PID(5.0, 1.0, 0.0)

    # Control mode
    use_rate_mode = (control_mode == 'rate')
    prev_mode_button = False
    print(f"Control mode: {'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'}")

    # ===========================================
    # Joystick Calibration
    # ===========================================
    print("Calibrating joystick...")
    thrust_offset = 0.0
    roll_offset = 0.0
    pitch_offset = 0.0
    yaw_offset = 0.0

    if joystick.device is None:
        # No controller: keep zero offsets; the drone hovers with neutral input
        # コントローラ無し: オフセット0のまま。中立入力で機体はホバーする
        print("コントローラ未接続のためキャリブレーションをスキップ / "
              "Skipping calibration (no controller)")
    else:
        i = 0
        num = 100
        while i < num:
            joydata = joystick.read()
            if joydata is not None:
                thrust_offset += (joydata[0] - 127) / 127.0
                roll_offset += (joydata[1] - 127) / 127.0
                pitch_offset += (joydata[2] - 127) / 127.0
                yaw_offset += (joydata[3] - 127) / 127.0
                i += 1
                print(i)

        thrust_offset /= num
        roll_offset /= num
        pitch_offset /= num
        yaw_offset /= num
        print(f"Calibration done: thrust={thrust_offset:.4f}, roll={roll_offset:.4f}, "
              f"pitch={pitch_offset:.4f}, yaw={yaw_offset:.4f}")

    # ===========================================
    # Simulation State
    # ===========================================
    sim_time = 0.0  # Simulation time [s]
    physics_steps = 0
    control_steps = 0
    render_steps = 0

    # Control state
    roll_ref = 0.0
    pitch_ref = 0.0
    yaw_ref = 0.0
    delta_thrust = 0.0

    # Control outputs (torque)
    roll_torque = 0.0
    pitch_torque = 0.0
    yaw_torque = 0.0

    # Current motor voltages
    voltage = [0.0, 0.0, 0.0, 0.0]

    # Data logging (reduced frequency to save memory)
    LOG_INTERVAL = 50  # Log every 50 physics steps (20Hz logging)
    T_log = []
    PQR_log = []
    EULER_log = []
    POS_log = []

    # Performance monitoring (legacy - kept for compatibility)
    # パフォーマンス監視（レガシー - 互換性のため保持）

    # Frame timing diagnostics (sampled to reduce overhead)
    # フレームタイミング診断（オーバーヘッド削減のためサンプリング）
    physics_step_times = []  # Sampled physics step times / サンプリングされた物理演算時間
    control_step_times = []  # Sampled control step times / サンプリングされた制御演算時間
    last_frame_num = 0  # Track renderer frame number / レンダラフレーム番号追跡
    steps_at_last_frame = 0  # Physics steps at last frame / 前回フレーム時の物理ステップ数

    # Instantaneous RT tracking (per-second)
    # 瞬時RT追跡（1秒ごと）
    rt_prev_sim_time = 0.0
    rt_prev_wall_time = 0.0

    print("\n" + "=" * 60)
    print(f"Roll Rate PID: Kp={cfg.roll_kp:.2e} Nm/(rad/s)")
    print(f"Pitch Rate PID: Kp={cfg.pitch_kp:.2e} Nm/(rad/s)")
    print(f"Yaw Rate PID: Kp={cfg.yaw_kp:.2e} Nm/(rad/s)")
    print(f"Hover thrust: {hover_thrust*1000:.1f} mN")
    print("=" * 60)
    print("\nStarting simulation...")

    # ===========================================
    # Main Simulation Loop (Real-time synchronized)
    # メインシミュレーションループ（リアルタイム同期）
    #
    # Structure: Outer render loop + inner control loop
    # 構造: 外側レンダリングループ + 内側制御ループ
    #
    # The inner loop runs control cycles until the next render time,
    # then the outer loop handles rendering via rate().
    # This reduces render-check overhead from 200/sec to 30/sec.
    # ===========================================
    # Disable GC during simulation to prevent GC pauses from inflating physics time
    # GCポーズによる物理演算時間の膨張を防ぐためGCを無効化
    gc.collect()  # Run final GC before disabling / 無効化前に最終GC実行
    gc.disable()

    wall_start = time.perf_counter()
    last_print_time = -1
    keyname = ''

    # Cache frequently used references (avoid repeated attribute lookups)
    # 頻繁にアクセスする参照をキャッシュ（属性ルックアップの繰り返しを回避）
    body = stampfly.body
    _step = stampfly.step_fast  # Use scalar fast path / スカラー高速パスを使用
    _joy_read = joystick.read
    _check_collision = Render.check_collision
    _rendering = Render.rendering
    _anim_time_ref = Render  # Access anim_time via Render.anim_time

    # Full loop timing diagnostics (measure total iteration overhead)
    # フルループタイミング診断（イテレーション全体のオーバーヘッドを計測）
    full_loop_times = []

    try:
        while sim_time < 6000.0:
            # =============================================
            # Inner control loop: run until next render time
            # 内側制御ループ: 次のレンダリング時刻まで実行
            # =============================================
            next_render_sim_time = _anim_time_ref.anim_time

            while sim_time < next_render_sim_time:
                # Measure full control cycle (sampled)
                # 制御サイクル全体を計測（サンプリング）
                _sample_full = (control_steps % 40 == 0)
                if _sample_full:
                    t_full_start = time.perf_counter()

                # --- Joystick read (200Hz) ---
                joydata = _joy_read()
                if joydata is not None:
                    thrust_raw = (joydata[0] - 127) / 127.0 - thrust_offset
                    roll_raw = (joydata[1] - 127) / 127.0 - roll_offset
                    pitch_raw = (joydata[2] - 127) / 127.0 - pitch_offset
                    yaw_raw = (joydata[3] - 127) / 127.0 - yaw_offset

                    # Mode button
                    buttons = joydata[4] if len(joydata) > 4 else 0
                    mode_button = bool(buttons & 0x04)

                    if mode_button and not prev_mode_button:
                        use_rate_mode = not use_rate_mode
                        mode_name = 'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'
                        print(f"Mode changed: {mode_name}")
                    prev_mode_button = mode_button

                    delta_thrust = 0.5 * thrust_raw * hover_thrust

                    if use_rate_mode:
                        roll_ref = cfg.roll_rate_max * roll_raw
                        pitch_ref = cfg.pitch_rate_max * pitch_raw
                        yaw_ref = cfg.yaw_rate_max * yaw_raw
                    else:
                        roll_ref = 0.25 * roll_raw * np.pi
                        pitch_ref = 0.25 * pitch_raw * np.pi
                        yaw_ref = 0.3 * yaw_raw * np.pi

                # --- Control update (200Hz) ---
                _sample_ctrl = (control_steps % 20 == 0)
                if _sample_ctrl:
                    t_ctrl_start = time.perf_counter()

                # Cache body state (reduce attribute lookups)
                # ボディ状態をキャッシュ（属性ルックアップ削減）
                _pqr = body.pqr
                rate_p = _pqr[0][0]
                rate_q = _pqr[1][0]
                rate_r = _pqr[2][0]

                if use_rate_mode:
                    roll_rate_ref = roll_ref
                    pitch_rate_ref = pitch_ref
                    yaw_rate_ref = yaw_ref
                else:
                    _euler = body.euler
                    phi = _euler[0][0]
                    theta = _euler[1][0]
                    roll_rate_ref = roll_pid.update(roll_ref, phi, CONTROL_DT)
                    pitch_rate_ref = pitch_pid.update(pitch_ref, theta, CONTROL_DT)
                    yaw_rate_ref = yaw_ref

                roll_torque = roll_rate_pid.update(roll_rate_ref, rate_p, CONTROL_DT)
                pitch_torque = pitch_rate_pid.update(pitch_rate_ref, rate_q, CONTROL_DT)
                yaw_torque = yaw_rate_pid.update(yaw_rate_ref, rate_r, CONTROL_DT)

                # Control allocation (uses pre-allocated buffers)
                # 制御配分（事前割り当てバッファ使用）
                total_thrust = hover_thrust + delta_thrust
                if total_thrust < 0.0:
                    total_thrust = 0.0
                elif total_thrust > 0.6:
                    total_thrust = 0.6
                motor_thrusts = allocator.mix(total_thrust, roll_torque, pitch_torque, yaw_torque)
                voltage = allocator.thrusts_to_voltages(motor_thrusts)

                control_steps += 1
                if _sample_ctrl:
                    control_step_times.append(time.perf_counter() - t_ctrl_start)

                # --- Physics inner loop (5 steps per control) ---
                _sample_phys = (control_steps % 20 == 0)
                if _sample_phys:
                    t_phys_start = time.perf_counter()

                for _ in range(PHYSICS_PER_CONTROL):
                    _step(voltage, PHYSICS_DT)
                physics_steps += PHYSICS_PER_CONTROL

                if _sample_phys:
                    physics_step_times.append((time.perf_counter() - t_phys_start) / PHYSICS_PER_CONTROL)

                sim_time = physics_steps * PHYSICS_DT

                # --- Data logging (reduced frequency: 20Hz) ---
                if physics_steps % LOG_INTERVAL == 0:
                    T_log.append(sim_time)
                    PQR_log.append(body.pqr.copy())
                    EULER_log.append(body.euler.copy())
                    POS_log.append(body.position.copy())

                # --- Collision detection ---
                _pos = body.position
                if _check_collision(_pos[0][0], _pos[1][0], _pos[2][0]):
                    print(f"COLLISION at t={sim_time:.2f}s")
                    Render.show_collision(_pos[0][0], _pos[1][0], _pos[2][0])
                    raise KeyboardInterrupt

                # Full control cycle measurement
                if _sample_full:
                    full_loop_times.append(time.perf_counter() - t_full_start)

            # =============================================
            # Rendering — rate(fps) is the master clock
            # 描画 — rate(fps)がマスタークロック
            # =============================================
            keyname = _rendering(sim_time, stampfly)

            # Track physics steps between frames
            # フレーム間の物理ステップ数を追跡
            if Render.frame_num > last_frame_num:
                Render.record_physics_steps(physics_steps - steps_at_last_frame)
                steps_at_last_frame = physics_steps
                last_frame_num = Render.frame_num

            if keyname == 'q':
                raise KeyboardInterrupt

            # ===========================================
            # Performance Report (every second)
            # ===========================================
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second

                wall_now = time.perf_counter() - wall_start
                rt_cumulative = sim_time / wall_now if wall_now > 0 else 1.0

                # Instantaneous RT (per-second measurement)
                # 瞬時RT（1秒ごとの計測）
                dt_sim = sim_time - rt_prev_sim_time
                dt_wall = wall_now - rt_prev_wall_time
                rt_instant = dt_sim / dt_wall if dt_wall > 0 else 1.0
                rt_prev_sim_time = sim_time
                rt_prev_wall_time = wall_now

                _euler = body.euler
                _pos = body.position

                mode_str = "ACRO" if use_rate_mode else "STAB"
                print(f"[{mode_str}] t={sim_time:.1f}s RT={rt_instant:.2f}x(avg={rt_cumulative:.2f}x) | "
                      f"pos=({_pos[0][0]:+.1f},{_pos[1][0]:+.1f},{_pos[2][0]:.1f}) | "
                      f"RPY=({np.degrees(_euler[0][0]):+.0f},{np.degrees(_euler[1][0]):+.0f},{np.degrees(_euler[2][0]):+.0f})")

    except KeyboardInterrupt:
        pass

    # Re-enable GC after simulation
    # シミュレーション後にGCを再有効化
    gc.enable()
    gc.collect()

    wall_total = time.perf_counter() - wall_start
    print("\nSimulation ended.")
    print(f"Total physics steps: {physics_steps}")
    print(f"Total control steps: {control_steps}")
    print(f"Sim time: {sim_time:.2f}s  Wall time: {wall_total:.2f}s  RT: {sim_time/wall_total:.2f}x")
    print(f"Effective physics: {physics_steps/wall_total:.0f} Hz")

    # ===========================================
    # Detailed Frame Timing Report
    # 詳細フレームタイミングレポート
    # ===========================================
    import statistics

    print()
    print(Render.get_timing_report())

    # Physics step timing report
    # 物理演算ステップタイミングレポート
    if len(physics_step_times) > 100:
        pst = physics_step_times[100:]  # Skip warm-up / ウォームアップをスキップ
        print("--- Physics step time / 物理演算ステップ時間 ---")
        print(f"  Average:  {statistics.mean(pst)*1e6:.1f} us ({statistics.mean(pst)*1000:.4f} ms)")
        print(f"  Median:   {statistics.median(pst)*1e6:.1f} us")
        print(f"  Min:      {min(pst)*1e6:.1f} us")
        print(f"  Max:      {max(pst)*1e6:.1f} us")
        if len(pst) > 1:
            print(f"  Stdev:    {statistics.stdev(pst)*1e6:.1f} us")
        max_physics_hz = 1.0 / statistics.mean(pst)
        print(f"  Max achievable physics rate: {max_physics_hz:.0f} Hz")
        print()

    # Control step timing report
    # 制御ステップタイミングレポート
    if len(control_step_times) > 20:
        cst = control_step_times[20:]  # Skip warm-up / ウォームアップをスキップ
        print("--- Control step time / 制御ステップ時間 ---")
        print(f"  Average:  {statistics.mean(cst)*1e6:.1f} us ({statistics.mean(cst)*1000:.4f} ms)")
        print(f"  Median:   {statistics.median(cst)*1e6:.1f} us")
        print(f"  Min:      {min(cst)*1e6:.1f} us")
        print(f"  Max:      {max(cst)*1e6:.1f} us")
        print()

    # Full control cycle timing report (including all overhead)
    # 制御サイクル全体タイミングレポート（全オーバーヘッド含む）
    if len(full_loop_times) > 20:
        flt = full_loop_times[20:]  # Skip warm-up / ウォームアップをスキップ
        print("--- Full control cycle time / 制御サイクル全体時間 ---")
        print(f"  Average:  {statistics.mean(flt)*1e6:.1f} us ({statistics.mean(flt)*1000:.3f} ms)")
        print(f"  Median:   {statistics.median(flt)*1e6:.1f} us")
        print(f"  Min:      {min(flt)*1e6:.1f} us")
        print(f"  Max:      {max(flt)*1e6:.1f} us")
        print(f"  Per second ({CONTROL_HZ} Hz): {statistics.mean(flt)*CONTROL_HZ*1000:.1f} ms")
        # Overhead = full_cycle - physics - control
        if len(physics_step_times) > 10 and len(control_step_times) > 10:
            avg_phys = statistics.mean(physics_step_times[10:])
            avg_ctrl = statistics.mean(control_step_times[10:])
            avg_full = statistics.mean(flt)
            overhead = avg_full - (avg_phys * PHYSICS_PER_CONTROL) - avg_ctrl
            print(f"  Overhead per cycle: {overhead*1e6:.1f} us (joystick+collision+logging+loop)")
            print(f"  Overhead per sec:   {overhead*CONTROL_HZ*1000:.1f} ms")
        print()

    # Feasibility analysis (FPS-independent)
    # 実現可能性分析（FPS非依存）
    print("=" * 70)
    print("REAL-TIME FEASIBILITY ANALYSIS / リアルタイム実現可能性分析")
    print("=" * 70)
    if len(physics_step_times) > 10:
        pst = physics_step_times[10:]  # Skip warm-up / ウォームアップをスキップ
        avg_physics_us = statistics.mean(pst) * 1e6
        avg_physics_ms = statistics.mean(pst) * 1000

        # Per-second computation budget (1000ms available)
        # 1秒あたりの演算バジェット（1000ms利用可能）
        physics_per_sec_ms = PHYSICS_HZ * avg_physics_ms
        if len(control_step_times) > 10:
            avg_ctrl_ms = statistics.mean(control_step_times[10:]) * 1000
        else:
            avg_ctrl_ms = 0
        control_per_sec_ms = CONTROL_HZ * avg_ctrl_ms

        td = Render._timing_data if hasattr(Render, '_timing_data') else {}
        if td.get('render_overheads') and len(td['render_overheads']) > 5:
            avg_render_ms = statistics.mean(td['render_overheads'][5:]) * 1000
        else:
            avg_render_ms = 0
        render_per_sec_ms = RENDER_FPS * avg_render_ms

        # Loop overhead from full-cycle measurement
        # フルサイクル計測からのループオーバーヘッド
        if len(full_loop_times) > 20:
            avg_full_ms = statistics.mean(full_loop_times[20:]) * 1000
            full_per_sec_ms = CONTROL_HZ * avg_full_ms
            overhead_per_sec_ms = full_per_sec_ms - physics_per_sec_ms - control_per_sec_ms
        else:
            overhead_per_sec_ms = 0
            full_per_sec_ms = 0

        total_per_sec_ms = physics_per_sec_ms + control_per_sec_ms + render_per_sec_ms + overhead_per_sec_ms
        theoretical_rt = 1000.0 / total_per_sec_ms if total_per_sec_ms > 0 else float('inf')

        print(f"\n  Per-second computation breakdown (for 1s of simulation):")
        print(f"  1秒のシミュレーションに必要な実時間:")
        print(f"    Physics  ({PHYSICS_HZ} Hz x {avg_physics_us:.1f} us): {physics_per_sec_ms:.1f} ms ({physics_per_sec_ms/10:.1f}%)")
        print(f"    Control  ({CONTROL_HZ} Hz x {avg_ctrl_ms*1000:.1f} us):  {control_per_sec_ms:.1f} ms ({control_per_sec_ms/10:.1f}%)")
        print(f"    Render   ({RENDER_FPS} FPS x {avg_render_ms:.3f} ms):   {render_per_sec_ms:.1f} ms ({render_per_sec_ms/10:.1f}%)")
        print(f"    Overhead (joystick+collision+loop):  {overhead_per_sec_ms:.1f} ms ({overhead_per_sec_ms/10:.1f}%)")
        print(f"    ─────────────────────────────────────")
        print(f"    Total:                              {total_per_sec_ms:.1f} ms")
        print(f"    Available (real-time):               1000.0 ms")
        print()
        print(f"  Theoretical RT: {theoretical_rt:.2f}x")
        print(f"  Measured RT:    {sim_time/wall_total:.2f}x")
        print()

        if total_per_sec_ms <= 1000.0:
            print(f"  RESULT: REAL-TIME ACHIEVABLE / リアルタイム達成可能")
            print(f"  CPU utilization: {total_per_sec_ms/10:.1f}%")
        else:
            max_physics_hz = 1.0 / (avg_physics_ms / 1000.0)
            print(f"  RESULT: REAL-TIME NOT ACHIEVABLE / リアルタイム達成不可能")
            print(f"  Bottleneck: Physics engine ({physics_per_sec_ms/total_per_sec_ms*100:.0f}% of total)")
            print(f"  Max physics throughput: {max_physics_hz:.0f} Hz (target: {PHYSICS_HZ} Hz)")
            print()

            # Recommend physics rate for RT=1.0
            # RT=1.0達成のための物理レート推奨
            avail_for_physics_ms = 1000.0 - control_per_sec_ms - render_per_sec_ms
            if avail_for_physics_ms > 0 and avg_physics_ms > 0:
                recommended_physics_hz = int(avail_for_physics_ms / avg_physics_ms)
                recommended_dt = 1.0 / recommended_physics_hz
                print(f"  Recommendation for RT=1.0:")
                print(f"    Physics: {recommended_physics_hz} Hz (dt = {recommended_dt*1000:.2f} ms)")
                print(f"    Control: {CONTROL_HZ} Hz")
                print(f"    Render:  {RENDER_FPS} FPS")

    print()
    print("=" * 70)

    # ===========================================
    # Plot Results
    # ===========================================
    if len(T_log) > 0:
        T_log = np.array(T_log)
        PQR_log = np.array(PQR_log)
        EULER_log = np.array(EULER_log)
        POS_log = np.array(POS_log)

        plt.figure(figsize=(12, 10))

        plt.subplot(3, 1, 1)
        plt.plot(T_log, PQR_log[:, 0, 0], label='P (roll rate)')
        plt.plot(T_log, PQR_log[:, 1, 0], label='Q (pitch rate)')
        plt.plot(T_log, PQR_log[:, 2, 0], label='R (yaw rate)')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Rate (rad/s)')
        plt.title('Angular Rates (2000Hz Physics, 400Hz Control)')

        plt.subplot(3, 1, 2)
        plt.plot(T_log, np.rad2deg(EULER_log[:, 0, 0]), label='φ (roll)')
        plt.plot(T_log, np.rad2deg(EULER_log[:, 1, 0]), label='θ (pitch)')
        plt.plot(T_log, np.rad2deg(EULER_log[:, 2, 0]), label='ψ (yaw)')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Euler Angle (deg)')
        plt.title('Euler Angles')

        plt.subplot(3, 1, 3)
        plt.plot(T_log, POS_log[:, 0, 0], label='X')
        plt.plot(T_log, POS_log[:, 1, 0], label='Y')
        plt.plot(T_log, POS_log[:, 2, 0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Position')

        plt.tight_layout()
        plt.show()

    # Force exit to terminate VPython's WebSocket server thread
    # VPythonのWebSocketサーバースレッドを終了するため強制終了
    # (VPython's server thread is non-daemon and keeps the process alive on macOS)
    os._exit(0)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='High-Frequency Flight Simulator (2000Hz Physics)')
    parser.add_argument('--world', '-w', type=str, default='voxel',
                        choices=['ringworld', 'voxel', 'minimal'],
                        help='World type (default: voxel, minimal for debugging)')
    parser.add_argument('--seed', '-s', type=int, default=None,
                        help='Terrain seed (random if not specified)')
    parser.add_argument('--mode', '-m', type=str, default='rate',
                        choices=['rate', 'angle'],
                        help='Control mode: rate=ACRO, angle=STABILIZE (default: rate)')
    parser.add_argument('--no-joystick', action='store_true',
                        help='Run without a controller (sf CLI passes this '
                             'when hidapi is unavailable)')
    args = parser.parse_args()

    print(f"Starting simulation: world={args.world}, mode={args.mode}")
    flight_sim_2000hz(world_type=args.world, seed=args.seed, control_mode=args.mode,
                      no_joystick=args.no_joystick)
