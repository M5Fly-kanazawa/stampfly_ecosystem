"""
Connection helper with simulator fallback
シミュレータフォールバック付き接続ヘルパー

Attempts to connect to a real StampFly drone via WiFi.
If the connection fails, falls back to a lightweight simulator stub.
"""

import time
from typing import Optional


class SimulatedStampFly:
    """Lightweight simulator stub for offline use.

    オフライン使用のための軽量シミュレータスタブ。
    実機がない環境でもノートブックを実行できるようにする。
    """

    def __init__(self):
        self._connected = True
        self._flying = False
        self._x = 0.0  # cm
        self._y = 0.0  # cm
        self._z = 0.0  # cm
        self._yaw = 0.0  # degrees
        self._battery = 100
        self._speed = 30  # cm/s
        self._trajectory = []  # List of (x, y, z, t)
        self._t = 0.0

    def __repr__(self) -> str:
        return "StampFly(mode=simulator)"

    # -- Connection --

    def connect(self, timeout: float = 10.0) -> None:
        print("[SIM] Connected to simulator")
        # シミュレータに接続

    def end(self) -> None:
        print("[SIM] Disconnected from simulator")
        # シミュレータから切断
        self._connected = False

    # -- Flight commands --

    def takeoff(self) -> None:
        print("[SIM] Takeoff")
        self._flying = True
        self._z = 50.0  # Default hover height 50cm
        self._record_position()

    def land(self) -> None:
        print("[SIM] Land")
        self._z = 0.0
        self._flying = False
        self._record_position()

    def emergency(self) -> None:
        print("[SIM] Emergency stop")
        self._flying = False

    def stop(self) -> None:
        print("[SIM] Stop - hovering in place")
        # その場でホバリング

    # -- Move commands --

    def move_up(self, x: int) -> None:
        self._z += x
        self._simulate_move(0, 0, x)
        print(f"[SIM] Move up {x}cm -> z={self._z:.0f}cm")

    def move_down(self, x: int) -> None:
        self._z = max(0, self._z - x)
        self._simulate_move(0, 0, -x)
        print(f"[SIM] Move down {x}cm -> z={self._z:.0f}cm")

    def move_forward(self, x: int) -> None:
        import math
        dx = x * math.cos(math.radians(self._yaw))
        dy = x * math.sin(math.radians(self._yaw))
        self._x += dx
        self._y += dy
        self._simulate_move(dx, dy, 0)
        print(f"[SIM] Move forward {x}cm -> ({self._x:.0f}, {self._y:.0f})")

    def move_back(self, x: int) -> None:
        import math
        dx = -x * math.cos(math.radians(self._yaw))
        dy = -x * math.sin(math.radians(self._yaw))
        self._x += dx
        self._y += dy
        self._simulate_move(dx, dy, 0)
        print(f"[SIM] Move back {x}cm -> ({self._x:.0f}, {self._y:.0f})")

    def move_left(self, x: int) -> None:
        import math
        dx = -x * math.sin(math.radians(self._yaw))
        dy = x * math.cos(math.radians(self._yaw))
        self._x += dx
        self._y += dy
        self._simulate_move(dx, dy, 0)
        print(f"[SIM] Move left {x}cm -> ({self._x:.0f}, {self._y:.0f})")

    def move_right(self, x: int) -> None:
        import math
        dx = x * math.sin(math.radians(self._yaw))
        dy = -x * math.cos(math.radians(self._yaw))
        self._x += dx
        self._y += dy
        self._simulate_move(dx, dy, 0)
        print(f"[SIM] Move right {x}cm -> ({self._x:.0f}, {self._y:.0f})")

    # -- Rotation commands --

    def rotate_clockwise(self, x: int) -> None:
        self._yaw = (self._yaw + x) % 360
        self._simulate_move(0, 0, 0)
        print(f"[SIM] Rotate CW {x}deg -> yaw={self._yaw:.0f}deg")

    def rotate_counter_clockwise(self, x: int) -> None:
        self._yaw = (self._yaw - x) % 360
        self._simulate_move(0, 0, 0)
        print(f"[SIM] Rotate CCW {x}deg -> yaw={self._yaw:.0f}deg")

    # -- RC control --

    def send_rc_control(
        self,
        left_right_velocity: int,
        forward_backward_velocity: int,
        up_down_velocity: int,
        yaw_velocity: int,
    ) -> None:
        pass  # Non-blocking, no output in sim
        # ノンブロッキング、シミュレータでは出力なし

    # -- Query commands --

    def get_battery(self) -> int:
        return self._battery

    def get_height(self) -> int:
        return int(self._z)

    def get_distance_tof(self) -> int:
        return int(self._z)

    def get_barometer(self) -> float:
        return float(self._z)

    def get_attitude(self) -> dict:
        return {"pitch": 0, "roll": 0, "yaw": int(self._yaw)}

    def get_acceleration(self) -> dict:
        return {"x": 0.0, "y": 0.0, "z": -981.0}

    def get_speed(self) -> int:
        return self._speed

    # -- StampFly extensions --

    def get_telemetry(self) -> dict:
        return {
            "x": self._x / 100.0,
            "y": self._y / 100.0,
            "z": self._z / 100.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": self._yaw,
            "battery": self._battery,
        }

    def hover(self, altitude: float = 0.5, duration: float = 5.0) -> None:
        self._z = altitude * 100
        self._simulate_move(0, 0, 0)
        print(f"[SIM] Hover at {altitude}m for {duration}s")

    def jump(self, altitude: float = 0.15) -> None:
        self._z = altitude * 100
        self._simulate_move(0, 0, 0)
        print(f"[SIM] Jump to {altitude}m")

    # -- Trajectory access --

    def get_trajectory(self) -> list:
        """Get recorded trajectory as list of (x, y, z, t) tuples (cm, s).

        記録された軌跡を (x, y, z, t) タプルのリストで取得 (cm, s)
        """
        return list(self._trajectory)

    # -- Internal helpers --

    def _simulate_move(self, dx: float, dy: float, dz: float):
        """Record position and simulate time passing.

        位置を記録し時間経過をシミュレートする
        """
        import math
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        self._t += max(dist / self._speed, 0.5)
        self._record_position()

    def _record_position(self):
        self._trajectory.append((self._x, self._y, self._z, self._t))


def connect_or_simulate(
    host: str = "192.168.10.1",
    timeout: float = 3.0,
    force_sim: bool = False,
) -> "StampFly | SimulatedStampFly":
    """Connect to StampFly drone or fall back to simulator.

    StampFly ドローンに接続、失敗時はシミュレータにフォールバック。

    Args:
        host: StampFly WiFi AP IP address / WiFi AP の IP アドレス
        timeout: Connection timeout in seconds / 接続タイムアウト（秒）
        force_sim: Force simulator mode / シミュレータモードを強制

    Returns:
        StampFly or SimulatedStampFly instance

    Usage:
        >>> drone = connect_or_simulate(force_sim=True)
        [SIM] Connected to simulator
        >>> type(drone).__name__
        'SimulatedStampFly'
    """
    if force_sim:
        sim = SimulatedStampFly()
        sim.connect()
        return sim

    try:
        from stampfly import StampFly
        drone = StampFly(host=host)
        drone.connect(timeout=timeout)
        print(f"Connected to StampFly at {host}")
        # StampFly に接続成功
        return drone
    except Exception as e:
        print(f"Could not connect to StampFly ({e})")
        print("Falling back to simulator mode")
        # シミュレータモードにフォールバック
        sim = SimulatedStampFly()
        sim.connect()
        return sim
