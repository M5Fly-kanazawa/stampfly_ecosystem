#!/usr/bin/env python3
"""
run_genesis_sim.py - Genesis StampFly Simulator with Physical Units Control
Genesis StampFly シミュレータ（物理単位ベース制御）

Features / 機能:
  - Physical units control (PID output: torque [Nm])
  - 2000Hz physics, 400Hz control, 30Hz rendering
  - RK4 motor/propeller dynamics
  - Real-time performance monitoring
  - Joystick control with ACRO/STABILIZE modes

Performance Note / 性能に関する注意:
  - 60 FPS tested but Genesis viewer is bottleneck (~25ms/frame)
  - 30 FPS is stable upper limit for real-time rendering
  - 60FPSテスト済みだがGenesisビューアがボトルネック（~25ms/フレーム）
  - 30FPSがリアルタイム描画の安定上限

Physical units mode matching firmware USE_PHYSICAL_UNITS=1:
ファームウェア USE_PHYSICAL_UNITS=1 と一致する物理単位モード:
  - PID output in torque [Nm]
  - Control allocation: [thrust(N), roll_torque(Nm), pitch_torque(Nm), yaw_torque(Nm)]

Control modes / 制御モード:
  - ACRO (Rate): Stick -> Rate setpoint [rad/s] -> PID -> Torque [Nm]
  - STABILIZE (Angle): Stick -> Angle setpoint [rad] -> Attitude PID -> Rate -> PID -> Torque

Controls (StampFly Controller USB HID) / 操作:
  - Throttle axis (Axis 0): Total thrust (N)
  - Roll axis (Axis 1): Roll rate/angle command
  - Pitch axis (Axis 2): Pitch rate/angle command
  - Yaw axis (Axis 3): Yaw rate command
  - Arm button (Button 0): Reset simulation
  - Mode button (Button 2): Toggle ACRO/STABILIZE mode
  - Option button (Button 3) / Q key: Exit
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
genesis_dir = script_dir.parent
simulator_dir = genesis_dir.parent
vpython_dir = simulator_dir / "vpython"

sys.path.insert(0, str(genesis_dir))  # for motor_model, control_allocation
sys.path.insert(0, str(vpython_dir))  # for control.pid

import genesis as gs
import genesis.utils.geom as gu
import numpy as np
import time
import pygame

from motor_model import QuadMotorSystem, compute_hover_conditions
from control_allocation import ControlAllocator, thrusts_to_duties

# Import PID from simulator/vpython/control
from control.pid import PID


# =============================================================================
# Physical Constants
# =============================================================================
GRAVITY = 9.81  # m/s^2
MASS = 0.035    # kg (35g)
WEIGHT = MASS * GRAVITY  # N
MAX_THRUST = 4 * 0.15  # 4 motors × max thrust per motor (N)


# =============================================================================
# Physical Units PID Configuration (matching firmware config.hpp)
# 物理単位PID設定（ファームウェア config.hpp と一致）
# =============================================================================

class PhysicalUnitsRateController:
    """
    Physical units rate controller.
    物理単位ベースのレートコントローラ

    Output: torque [Nm] (not voltage scale)
    出力: トルク [Nm]（電圧スケールではない）
    """

    def __init__(self):
        # Physical units PID gains (from firmware config.hpp)
        # Kp units: [Nm / (rad/s)]
        # Output limits: [Nm]

        # Roll PID - match firmware config.hpp (corrected k_τ)
        # k_τ = 4 × (0.25/3.7) × 0.23 × 0.023 ≈ 1.4e-3 Nm/V
        self.roll_pid = PID(
            Kp=9.1e-4,   # Nm/(rad/s) = 0.65 × 1.4e-3
            Ti=0.7,      # s
            Td=0.01,     # s
            eta=0.125,
            output_min=-5.2e-3,  # Nm = 3.7 × 1.4e-3
            output_max=5.2e-3,   # Nm
        )

        # Pitch PID - match firmware config.hpp (corrected k_τ)
        self.pitch_pid = PID(
            Kp=1.33e-3,  # Nm/(rad/s) = 0.95 × 1.4e-3
            Ti=0.7,      # s
            Td=0.025,    # s
            eta=0.125,
            output_min=-5.2e-3,  # Nm = 3.7 × 1.4e-3
            output_max=5.2e-3,   # Nm
        )

        # Yaw PID - match firmware config.hpp (corrected k_τ)
        # k_τ_yaw = 4 × (0.25/3.7) × 0.23 × 0.00971 ≈ 5.9e-4 Nm/V
        self.yaw_pid = PID(
            Kp=1.77e-3,  # Nm/(rad/s) = 3.0 × 5.9e-4
            Ti=0.8,      # s
            Td=0.01,     # s
            eta=0.125,
            output_min=-2.2e-3,  # Nm = 3.7 × 5.9e-4
            output_max=2.2e-3,   # Nm
        )

        # Rate limits (rad/s) - match firmware config.hpp
        self.roll_rate_max = 1.0    # rad/s (~57 deg/s)
        self.pitch_rate_max = 1.0   # rad/s (~57 deg/s)
        self.yaw_rate_max = 5.0     # rad/s (~286 deg/s)

    def update(self, rate_setpoint: np.ndarray, gyro: np.ndarray, dt: float):
        """
        Update rate controller.

        Args:
            rate_setpoint: Target angular rates [roll, pitch, yaw] (rad/s)
            gyro: Current angular rates [p, q, r] (rad/s)
            dt: Time step (s)

        Returns:
            (roll_torque, pitch_torque, yaw_torque) in Nm
        """
        roll_torque = self.roll_pid.update(rate_setpoint[0], gyro[0], dt)
        pitch_torque = self.pitch_pid.update(rate_setpoint[1], gyro[1], dt)
        yaw_torque = self.yaw_pid.update(rate_setpoint[2], gyro[2], dt)

        return roll_torque, pitch_torque, yaw_torque

    def update_from_stick(self, stick_roll: float, stick_pitch: float, stick_yaw: float,
                          gyro: np.ndarray, dt: float):
        """
        Update from normalized stick inputs (-1 to 1).
        ACRO mode: stick -> rate setpoint.
        """
        rate_setpoint = np.array([
            stick_roll * self.roll_rate_max,
            stick_pitch * self.pitch_rate_max,
            stick_yaw * self.yaw_rate_max,
        ])
        return self.update(rate_setpoint, gyro, dt)

    def reset(self):
        """Reset all PID controllers."""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()


class AttitudeController:
    """
    Attitude (angle) controller for STABILIZE mode.
    STABILIZE モード用の姿勢（角度）コントローラ

    Stick -> Angle setpoint -> Attitude PID -> Rate setpoint
    """

    def __init__(self):
        # Attitude PID gains
        self.roll_pid = PID(Kp=5.0, Ti=1.0, Td=0.0, eta=0.125,
                            output_min=-4.0, output_max=4.0)
        self.pitch_pid = PID(Kp=5.0, Ti=1.0, Td=0.0, eta=0.125,
                             output_min=-4.0, output_max=4.0)

        # Angle limits (rad)
        self.roll_angle_max = np.radians(30)   # ±30 deg
        self.pitch_angle_max = np.radians(30)  # ±30 deg

    def update(self, stick_roll: float, stick_pitch: float, attitude: np.ndarray, dt: float):
        """
        Update attitude controller.

        Args:
            stick_roll: Roll stick (-1 to 1)
            stick_pitch: Pitch stick (-1 to 1)
            attitude: Current attitude [roll, pitch, yaw] (rad)
            dt: Time step (s)

        Returns:
            (roll_rate_setpoint, pitch_rate_setpoint) in rad/s
        """
        roll_ref = stick_roll * self.roll_angle_max
        pitch_ref = stick_pitch * self.pitch_angle_max

        roll_rate_ref = self.roll_pid.update(roll_ref, attitude[0], dt)
        pitch_rate_ref = self.pitch_pid.update(pitch_ref, attitude[1], dt)

        return roll_rate_ref, pitch_rate_ref

    def reset(self):
        """Reset all PID controllers."""
        self.roll_pid.reset()
        self.pitch_pid.reset()


# =============================================================================
# Follow Camera
# =============================================================================

# =============================================================================
# Performance Monitor
# =============================================================================

class PerformanceMonitor:
    """
    Monitor real-time performance (FPS, timing).
    リアルタイム性能モニタ（FPS、タイミング）
    """

    def __init__(self, window_size=60):
        """
        Args:
            window_size: Number of frames for rolling average
        """
        self.window_size = window_size
        self.frame_times = []
        self.physics_times = []
        self.last_frame_time = None
        self.total_frames = 0
        self.total_physics_steps = 0
        self.behind_count = 0  # Number of times sim fell behind real-time
        self.start_time = None

    def start(self):
        """Start monitoring."""
        self.start_time = time.perf_counter()
        self.last_frame_time = self.start_time

    def record_frame(self, physics_steps_this_frame, sim_behind_realtime=False):
        """
        Record a rendered frame.

        Args:
            physics_steps_this_frame: Number of physics steps executed this frame
            sim_behind_realtime: True if simulation is behind real-time
        """
        now = time.perf_counter()
        if self.last_frame_time is not None:
            frame_time = now - self.last_frame_time
            self.frame_times.append(frame_time)
            if len(self.frame_times) > self.window_size:
                self.frame_times.pop(0)

        self.last_frame_time = now
        self.total_frames += 1
        self.total_physics_steps += physics_steps_this_frame

        if sim_behind_realtime:
            self.behind_count += 1

    def get_fps(self):
        """Get current rolling average FPS."""
        if len(self.frame_times) < 2:
            return 0.0
        avg_frame_time = sum(self.frame_times) / len(self.frame_times)
        return 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

    def get_frame_time_stats(self):
        """Get frame time statistics (min, avg, max) in ms."""
        if not self.frame_times:
            return 0.0, 0.0, 0.0
        min_t = min(self.frame_times) * 1000
        avg_t = (sum(self.frame_times) / len(self.frame_times)) * 1000
        max_t = max(self.frame_times) * 1000
        return min_t, avg_t, max_t

    def get_summary(self, target_fps=60):
        """Get performance summary string."""
        fps = self.get_fps()
        min_t, avg_t, max_t = self.get_frame_time_stats()
        elapsed = time.perf_counter() - self.start_time if self.start_time else 0

        realtime_ok = "OK" if fps >= target_fps * 0.95 else "SLOW"
        behind_pct = (self.behind_count / self.total_frames * 100) if self.total_frames > 0 else 0

        return (f"FPS: {fps:.1f}/{target_fps} ({realtime_ok}) | "
                f"frame: {avg_t:.1f}ms (min:{min_t:.1f}, max:{max_t:.1f}) | "
                f"behind: {behind_pct:.1f}%")

    def reset(self):
        """Reset all statistics."""
        self.frame_times = []
        self.physics_times = []
        self.last_frame_time = None
        self.total_frames = 0
        self.total_physics_steps = 0
        self.behind_count = 0
        self.start_time = None


class FollowCamera:
    """
    Follow camera that tracks drone from behind.
    後ろから追いかけるカメラ

    Genesis coordinate system:
    - X: forward, Y: left, Z: up
    """

    def __init__(self, distance=1.0, height=0.3, alpha_pos=0.08, alpha_look=0.15,
                 alpha_height=1.0, alpha_yaw=0.05):
        """
        Args:
            distance: Distance behind drone [m]
            height: Height above drone [m]
            alpha_pos: Smoothing factor for camera XY position (smaller = smoother)
            alpha_look: Smoothing factor for lookat point XY
            alpha_height: Smoothing factor for height tracking (larger = faster response)
            alpha_yaw: Smoothing factor for yaw angle tracking (smaller = smoother rotation)
        """
        self.distance = distance
        self.height = height
        self.alpha_pos = alpha_pos
        self.alpha_look = alpha_look
        self.alpha_height = alpha_height
        self.alpha_yaw = alpha_yaw

        # Smoothed camera state
        self.cam_pos = None
        self.lookat_pos = None
        self.smoothed_yaw = None
        self.initialized = False

    def _wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _smooth_angle(self, current, target, alpha):
        """
        Smooth angle with proper wrapping.
        角度を正しくラップしながらスムージング
        """
        # Calculate shortest angular difference
        diff = self._wrap_angle(target - current)
        # Apply smoothing
        return self._wrap_angle(current + alpha * diff)

    def update(self, scene, drone_pos, drone_yaw):
        """
        Update camera position to follow drone.

        Args:
            scene: Genesis scene object
            drone_pos: Drone position (x, y, z) in Genesis frame
            drone_yaw: Drone yaw angle [rad] in Genesis frame
        """
        # Target lookat = drone position
        lookat_target = np.array([
            float(drone_pos[0]),
            float(drone_pos[1]),
            float(drone_pos[2]),
        ])

        # Initialize or smooth yaw
        if not self.initialized:
            self.smoothed_yaw = drone_yaw
            self.lookat_pos = lookat_target.copy()
        else:
            # Smooth yaw angle (handles wrapping at ±π)
            self.smoothed_yaw = self._smooth_angle(self.smoothed_yaw, drone_yaw, self.alpha_yaw)
            # Smooth lookat position
            self.lookat_pos[0] += self.alpha_look * (lookat_target[0] - self.lookat_pos[0])
            self.lookat_pos[1] += self.alpha_look * (lookat_target[1] - self.lookat_pos[1])
            self.lookat_pos[2] += self.alpha_height * (lookat_target[2] - self.lookat_pos[2])

        # Target camera position = behind and above drone (using smoothed yaw)
        # Genesis coordinate: X=Right, Y=Forward, Z=Up
        # Drone's "front" in Genesis is +Y direction when yaw=0
        # Behind vector: (sin(yaw), -cos(yaw), 0)
        cam_target = np.array([
            self.lookat_pos[0] + self.distance * np.sin(self.smoothed_yaw),
            self.lookat_pos[1] - self.distance * np.cos(self.smoothed_yaw),
            self.lookat_pos[2] + self.height,
        ])

        # Initialize or smooth camera position
        if not self.initialized:
            self.cam_pos = cam_target.copy()
            self.initialized = True
        else:
            # Low-pass filter smoothing for camera position
            self.cam_pos[0] += self.alpha_pos * (cam_target[0] - self.cam_pos[0])
            self.cam_pos[1] += self.alpha_pos * (cam_target[1] - self.cam_pos[1])
            self.cam_pos[2] += self.alpha_height * (cam_target[2] - self.cam_pos[2])

        # Update viewer camera using 4x4 pose matrix
        # This allows explicit control of up vector (always Z-up for level horizon)
        up = np.array([0.0, 0.0, 1.0])
        pose = gu.pos_lookat_up_to_T(self.cam_pos, self.lookat_pos, up)
        scene.viewer.set_camera_pose(pose=pose)

    def reset(self):
        """Reset camera state."""
        self.cam_pos = None
        self.lookat_pos = None
        self.smoothed_yaw = None
        self.initialized = False


# =============================================================================
# Utility Functions
# =============================================================================

def quat_to_rotation_matrix(quat):
    """Quaternion (w, x, y, z) to rotation matrix"""
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def quat_to_euler(quat):
    """Quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in rad"""
    w, x, y, z = [float(v) for v in quat]
    # Genesis conventions (adjust for NED if needed)
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def ned_to_genesis_force(force_ned):
    """NED body force to Genesis body force"""
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    """NED body moment to Genesis body moment"""
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def world_ang_vel_to_body(ang_vel_world, R):
    """Transform angular velocity from world frame to body frame"""
    return R.T @ ang_vel_world


def genesis_gyro_to_ned(gyro_genesis_body):
    """
    Convert Genesis body frame angular velocity to NED body frame.
    Genesis body: (wx, wy, wz)
    NED body: (p, q, r) - roll rate, pitch rate, yaw rate
    """
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def genesis_euler_to_ned(euler_genesis):
    """
    Convert Genesis Euler angles to NED Euler angles.
    """
    # Genesis: (roll, pitch, yaw) with Z-up
    # NED: (roll, pitch, yaw) with Z-down
    return np.array([euler_genesis[1], euler_genesis[0], -euler_genesis[2]])


# =============================================================================
# World Building Functions
# =============================================================================

def add_block(scene, pos, size, color):
    """Add single block"""
    scene.add_entity(
        gs.morphs.Box(size=(size, size, size), pos=pos, fixed=True, collision=False),
        surface=gs.surfaces.Default(color=color),
    )


def add_grid_floor(scene, size, tile_size=1.0, line_width=0.02):
    """Create grid floor"""
    half_size = size / 2
    num_lines = int(size / tile_size) + 1
    z = 0.001
    for i in range(num_lines):
        y = -half_size + i * tile_size
        scene.add_entity(
            gs.morphs.Box(size=(size, line_width, line_width), pos=(0, y, z),
                          fixed=True, collision=False),
            surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
        )
    for i in range(num_lines):
        x = -half_size + i * tile_size
        scene.add_entity(
            gs.morphs.Box(size=(line_width, size, line_width), pos=(x, 0, z),
                          fixed=True, collision=False),
            surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
        )


def add_simple_obstacles(scene, block_size=0.5):
    """Add simple obstacle structures"""
    colors = {
        'red': (0.8, 0.2, 0.2),
        'green': (0.2, 0.7, 0.3),
        'blue': (0.2, 0.3, 0.8),
        'yellow': (0.9, 0.8, 0.1),
    }

    # Simple pillars
    pillars = [
        ((5, 0, 0), 6, colors['red']),
        ((-5, 0, 0), 8, colors['blue']),
        ((0, 5, 0), 4, colors['green']),
        ((0, -5, 0), 10, colors['yellow']),
    ]

    for (bx, by, bz), height, color in pillars:
        for i in range(height):
            z = bz + i * block_size + block_size / 2
            add_block(scene, (bx, by, z), block_size, color)


# =============================================================================
# Main Function
# =============================================================================

def main():
    print("=" * 60)
    print("Physical Units Rate Control")
    print("物理単位ベース角速度制御")
    print("=" * 60)
    print("Arm button (Button 0): Reset simulation")
    print("Mode button (Button 2): Toggle ACRO/STABILIZE")
    print("Option button (Button 3) / Q: Exit")
    print("=" * 60)

    # Path setup
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # Timing settings
    PHYSICS_HZ = 2000  # 2000Hz physics
    PHYSICS_DT = 1 / PHYSICS_HZ  # 0.0005s = 0.5ms
    CONTROL_HZ = 400  # 5 physics steps per control (integer ratio)
    CONTROL_DT = 1 / CONTROL_HZ
    # Note: 60 FPS tested but Genesis viewer bottleneck limits to ~40 FPS
    # Genesis viewer takes ~25ms per frame, so 30 FPS is stable upper limit
    # 注: 60FPSをテストしたがGenesisビューアがボトルネックで~40FPSが限界
    # ビューア更新に~25ms/フレームかかるため30FPSが安定上限
    RENDER_FPS = 30
    RENDER_DT = 1 / RENDER_FPS

    # World settings
    WORLD_SIZE = 50.0
    TILE_SIZE = 1.0
    BLOCK_SIZE = 0.5

    # Control settings
    DEADZONE = 0.05
    THROTTLE_SCALE = 0.5  # Scale throttle stick to thrust change

    print(f"\nControl rate: {CONTROL_HZ}Hz, Physics: {PHYSICS_HZ}Hz")

    # pygame initialization
    print("\n[1] Initializing controller...")
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No controller found!")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"  Controller: {joystick.get_name()}")

    # Control system initialization
    print("\n[2] Initializing control system...")
    allocator = ControlAllocator()
    motor_system = QuadMotorSystem()

    # Initialize motors at hover speed (critical for stability!)
    # モータをホバー回転数で初期化（安定性のために必須！）
    hover = compute_hover_conditions()
    for motor in motor_system.motors:
        motor.omega = hover['omega_hover']
    print(f"  Motors initialized at hover: ω={hover['omega_hover']:.0f} rad/s")

    # Controllers
    rate_controller = PhysicalUnitsRateController()
    attitude_controller = AttitudeController()

    # Follow camera (behind drone)
    # Increased alpha values for tighter following
    # Follow camera with yaw smoothing for smooth rotation tracking
    # alpha_yaw=0.1: smooth camera rotation when drone yaws
    # height=0.05: slightly above drone for natural perspective
    follow_camera = FollowCamera(distance=0.3, height=0.05, alpha_pos=0.9, alpha_look=0.9,
                                 alpha_height=1.0, alpha_yaw=0.1)

    # Control mode: True = ACRO (rate), False = STABILIZE (angle)
    use_acro_mode = True
    prev_mode_button = False

    # Performance monitor for 60 FPS real-time check
    # 60FPSリアルタイム性能チェック用モニタ
    perf_monitor = PerformanceMonitor(window_size=60)

    print(f"  Initial mode: ACRO (Rate Control)")
    print(f"  Camera: Follow mode (behind drone)")
    print(f"  Physical units PID gains (corrected k_τ):")
    print(f"    Roll:  Kp={9.1e-4:.2e} Nm/(rad/s)")
    print(f"    Pitch: Kp={1.33e-3:.2e} Nm/(rad/s)")
    print(f"    Yaw:   Kp={1.77e-3:.2e} Nm/(rad/s)")

    # Genesis initialization
    print("\n[3] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # Scene creation
    # Initial camera: behind drone
    # Genesis coordinate: X=Right, Y=Forward, Z=Up
    # Drone front = +Y when yaw=0, so behind = -Y
    DRONE_SPAWN_POS = (0, 0, 2.0)
    print("\n[4] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(DRONE_SPAWN_POS[0],
                        DRONE_SPAWN_POS[1] - follow_camera.distance,
                        DRONE_SPAWN_POS[2] + follow_camera.height),
            camera_lookat=DRONE_SPAWN_POS,
            camera_fov=100,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    # Ground + grid
    print("\n[5] Adding ground with grid...")
    scene.add_entity(
        gs.morphs.Plane(collision=True),
        surface=gs.surfaces.Default(color=(0.12, 0.12, 0.12)),
    )
    add_grid_floor(scene, WORLD_SIZE, TILE_SIZE)

    # Simple obstacles
    print("\n[6] Adding obstacles...")
    add_simple_obstacles(scene, BLOCK_SIZE)

    # Load StampFly
    print("\n[7] Loading StampFly...")
    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=DRONE_SPAWN_POS,
            euler=(0, 0, 0),  # yaw=0: facing +X direction
            fixed=False,
            prioritize_urdf_material=True,
        ),
    )

    # Build scene
    print("\n[8] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # Initialize follow camera with drone's initial position and yaw
    initial_pos = drone.get_pos()
    initial_quat = drone.get_quat()
    initial_euler = quat_to_euler(initial_quat)
    initial_yaw = initial_euler[2]
    follow_camera.update(scene, initial_pos, initial_yaw)

    # Info display
    print("\n" + "=" * 60)
    print("Physical Units Rate Control - Ready")
    print("=" * 60)
    print(f"  Mode: ACRO (Rate Control)")
    print(f"  Hover thrust: {WEIGHT*1000:.1f}mN")
    print(f"  Rate limits: Roll/Pitch=±{np.degrees(rate_controller.roll_rate_max):.0f} deg/s")
    print(f"  Output limits: Roll/Pitch=±{5.2e-3*1e3:.1f}mNm, Yaw=±{2.2e-3*1e3:.1f}mNm")
    print("=" * 60)

    # Simulation loop
    print("\n[9] Running simulation...")
    print(f"    Target: {RENDER_FPS} FPS real-time")

    physics_steps = 0
    control_steps = 0
    next_render_time = 0
    next_control_time = 0
    start_time = time.perf_counter()
    last_print_time = -1
    physics_steps_at_last_render = 0

    # Start performance monitoring
    perf_monitor.start()

    # Current control outputs
    current_torque = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] Nm

    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    def reset_simulation():
        nonlocal physics_steps, control_steps, next_render_time, next_control_time
        nonlocal start_time, current_torque
        scene.reset()
        motor_system.reset()
        rate_controller.reset()
        attitude_controller.reset()
        follow_camera.reset()
        perf_monitor.reset()
        physics_steps = 0
        control_steps = 0
        next_render_time = 0
        next_control_time = 0
        start_time = time.perf_counter()
        perf_monitor.start()
        current_torque = np.array([0.0, 0.0, 0.0])
        print("\n>>> Reset")

    try:
        while scene.viewer.is_alive():
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        print("\n>>> Exit (Q key)")
                        raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # Arm button
                        reset_simulation()

                    if event.button == 2:  # Mode button
                        # Toggle mode
                        use_acro_mode = not use_acro_mode
                        mode_name = "ACRO (Rate)" if use_acro_mode else "STABILIZE (Angle)"
                        print(f"\n>>> Mode changed: {mode_name}")
                        # Reset controllers on mode change
                        rate_controller.reset()
                        attitude_controller.reset()

                    if event.button == 3:  # Option button
                        print("\n>>> Exit")
                        raise KeyboardInterrupt

            # Controller input
            throttle_raw = apply_deadzone(joystick.get_axis(0), DEADZONE)
            roll_raw = apply_deadzone(joystick.get_axis(1), DEADZONE)
            pitch_raw = apply_deadzone(joystick.get_axis(2), DEADZONE)
            yaw_raw = apply_deadzone(joystick.get_axis(3), DEADZONE)

            # Thrust command: hover + delta
            u_thrust = WEIGHT + throttle_raw * THROTTLE_SCALE * WEIGHT

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # Physics loop with integrated control
            # 物理ループと制御ループを統合
            while sim_time <= real_time:
                # Control loop (at CONTROL_HZ) - inside physics loop
                # 制御ループは物理ループの中で実行
                if sim_time >= next_control_time:
                    # Get current state from Genesis
                    quat = drone.get_quat()

                    # Get angular velocity from DOFs (body frame directly)
                    # Genesis DOF convention:
                    #   get_dofs_velocity()[3:6] = body frame angular velocity (wx, wy, wz)
                    #   get_ang() = world frame angular velocity
                    dofs_vel = drone.get_dofs_velocity()
                    gyro_genesis_body = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
                    # Convert Genesis body frame (wx, wy, wz) to NED body frame (p, q, r)
                    # See docs/architecture/genesis-integration.md:
                    #   (wx, wy, wz)_genesis → (wy, wx, -wz) = (p, q, r)_ned
                    gyro_ned = genesis_gyro_to_ned(gyro_genesis_body)

                    if use_acro_mode:
                        # ACRO mode: Stick -> Rate setpoint -> PID -> Torque
                        roll_torque, pitch_torque, yaw_torque = rate_controller.update_from_stick(
                            stick_roll=roll_raw,
                            stick_pitch=pitch_raw,
                            stick_yaw=yaw_raw,
                            gyro=gyro_ned,
                            dt=CONTROL_DT,
                        )
                    else:
                        # STABILIZE mode: Stick -> Angle -> Attitude PID -> Rate -> Rate PID -> Torque
                        euler_genesis = quat_to_euler(quat)
                        euler_ned = genesis_euler_to_ned(euler_genesis)

                        # Attitude controller gives rate setpoints
                        roll_rate_ref, pitch_rate_ref = attitude_controller.update(
                            stick_roll=roll_raw,
                            stick_pitch=pitch_raw,
                            attitude=euler_ned,
                            dt=CONTROL_DT,
                        )
                        yaw_rate_ref = yaw_raw * rate_controller.yaw_rate_max

                        # Rate controller
                        rate_setpoint = np.array([roll_rate_ref, pitch_rate_ref, yaw_rate_ref])
                        roll_torque, pitch_torque, yaw_torque = rate_controller.update(
                            rate_setpoint, gyro_ned, CONTROL_DT
                        )

                    current_torque = np.array([roll_torque, pitch_torque, yaw_torque])

                    # Control allocation: [thrust, roll_torque, pitch_torque, yaw_torque] -> motor thrusts
                    control = np.array([u_thrust, current_torque[0], current_torque[1], current_torque[2]])
                    target_thrusts = allocator.mix(control)
                    target_duties = thrusts_to_duties(target_thrusts)

                    control_steps += 1
                    next_control_time = control_steps * CONTROL_DT

                # Motor dynamics (at PHYSICS_HZ)
                force_ned, moment_ned = motor_system.step_with_duty(
                    target_duties.tolist(), PHYSICS_DT
                )

                # Convert NED body frame to Genesis body frame
                # See docs/architecture/genesis-integration.md:
                #   (fx, fy, fz)_ned → (fy, fx, -fz)_genesis
                #   (τx, τy, τz)_ned → (τy, τx, -τz)_genesis
                force_genesis_body = ned_to_genesis_force(force_ned)
                moment_genesis_body = ned_to_genesis_moment(moment_ned)

                # Genesis DOF force convention:
                #   dof_force[0:3] = WORLD frame force
                #   dof_force[3:6] = BODY frame torque
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)

                force_world = R @ force_genesis_body  # Body → World (for force)
                # Torque stays in body frame (no transformation needed)

                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world           # World frame force
                dof_force[3:6] = moment_genesis_body   # Body frame torque
                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            # Rendering with follow camera
            if real_time >= next_render_time:
                # Track physics steps this frame
                physics_this_frame = physics_steps - physics_steps_at_last_render
                physics_steps_at_last_render = physics_steps

                # Check if we're behind real-time (sim_time < real_time means we're catching up)
                sim_behind = (sim_time < real_time - RENDER_DT)
                perf_monitor.record_frame(physics_this_frame, sim_behind)

                # Update follow camera position
                pos = drone.get_pos()
                quat = drone.get_quat()
                euler_genesis = quat_to_euler(quat)
                drone_yaw = euler_genesis[2]  # Genesis yaw
                follow_camera.update(scene, pos, drone_yaw)

                scene.visualizer.update()
                next_render_time += RENDER_DT

            # Wait
            sleep_time = sim_time - real_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Status display (every second)
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second

                pos = drone.get_pos()
                quat = drone.get_quat()
                euler_genesis = quat_to_euler(quat)
                euler_ned = genesis_euler_to_ned(euler_genesis)

                # Get gyro for display using get_dofs_velocity() (body frame directly)
                dofs_vel_disp = drone.get_dofs_velocity()
                gyro_genesis_body = np.array([float(dofs_vel_disp[3]), float(dofs_vel_disp[4]), float(dofs_vel_disp[5])])
                gyro_ned = genesis_gyro_to_ned(gyro_genesis_body)

                mode_str = "ACRO" if use_acro_mode else "STAB"

                # Performance stats line
                perf_summary = perf_monitor.get_summary(target_fps=RENDER_FPS)
                print(f"  [{mode_str}] t={sim_time:.0f}s | {perf_summary}")
                print(f"         pos=({float(pos[0]):+.1f},{float(pos[1]):+.1f},{float(pos[2]):.1f}) | "
                      f"RPY=({np.degrees(euler_ned[0]):+.0f},{np.degrees(euler_ned[1]):+.0f},{np.degrees(euler_ned[2]):+.0f})°")

    except KeyboardInterrupt:
        pass

    # Final performance summary
    print("\n" + "=" * 60)
    print("Performance Summary")
    print("=" * 60)
    fps = perf_monitor.get_fps()
    min_t, avg_t, max_t = perf_monitor.get_frame_time_stats()
    behind_pct = (perf_monitor.behind_count / perf_monitor.total_frames * 100) if perf_monitor.total_frames > 0 else 0

    print(f"  Target FPS:     {RENDER_FPS}")
    print(f"  Achieved FPS:   {fps:.1f}")
    print(f"  Frame time:     avg={avg_t:.2f}ms, min={min_t:.2f}ms, max={max_t:.2f}ms")
    print(f"  Total frames:   {perf_monitor.total_frames}")
    print(f"  Behind count:   {perf_monitor.behind_count} ({behind_pct:.1f}%)")

    if fps >= RENDER_FPS * 0.95:
        print(f"\n  Result: PASS - Real-time 60 FPS achieved")
    else:
        print(f"\n  Result: FAIL - Cannot sustain 60 FPS real-time")
    print("=" * 60)

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
