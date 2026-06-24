"""
StampFly — djitellopy-compatible Python SDK for StampFly drone

djitellopy 互換の StampFly Python SDK。
既存の Tello 教材コードを最小限の変更で StampFly に移行可能にする。

Usage:
    from stampfly import StampFly
    drone = StampFly()
    drone.connect()
    drone.takeoff()
    drone.move_forward(100)
    drone.land()
    drone.end()
"""

import asyncio
import threading
from typing import Optional

from sfcli.utils.vehicle_connection import VehicleConnection
from .exceptions import StampFlyConnectionError, StampFlyCommandError


class StampFly:
    """djitellopy-compatible interface for StampFly drone.

    StampFly ドローン用の djitellopy 互換インターフェース。
    バックグラウンドスレッドに永続イベントループを持ち、
    connect() で接続を確立し end() まで維持する。
    """

    # Distance range for move commands (cm)
    # 移動コマンドの距離範囲 (cm)
    MOVE_DISTANCE_MIN = 20
    MOVE_DISTANCE_MAX = 200

    # Rotation range for cw/ccw commands (degrees)
    # 回転コマンドの角度範囲 (度)
    ROTATION_MIN = 1
    ROTATION_MAX = 360

    # RC control range
    # RC 制御の範囲
    RC_MIN = -100
    RC_MAX = 100

    def __init__(self, host: str = "192.168.10.1"):
        self._host = host
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._conn: Optional[VehicleConnection] = None
        self._telemetry: dict = {}
        self._telem_task: Optional[asyncio.Task] = None

    def __repr__(self) -> str:
        connected = self._conn is not None and self._loop is not None
        return f"StampFly(host={self._host!r}, connected={connected})"

    # ------------------------------------------------------------------
    # Connection / 接続
    # ------------------------------------------------------------------

    def connect(self, timeout: float = 10.0) -> None:
        """Connect to StampFly via WiFi (TCP CLI + WebSocket telemetry).

        StampFly に WiFi 経由で接続する（TCP CLI + WebSocket テレメトリ）
        """
        if self._loop is not None:
            raise StampFlyConnectionError("Already connected")

        # Start background event loop in a daemon thread
        # デーモンスレッドでバックグラウンドイベントループを起動
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever,
            daemon=True,
            name="stampfly-event-loop",
        )
        self._thread.start()

        # Connect via VehicleConnection
        # VehicleConnection で接続
        self._conn = VehicleConnection()
        try:
            self._run_async(self._conn.connect(self._host, timeout))
        except Exception as e:
            self._cleanup()
            raise StampFlyConnectionError(f"Connection failed: {e}") from e

        # Start background telemetry receiver
        # バックグラウンドテレメトリ受信を開始
        self._telem_task = asyncio.run_coroutine_threadsafe(
            self._telemetry_loop(), self._loop
        )

    def end(self) -> None:
        """Disconnect from StampFly and clean up resources.

        StampFly から切断しリソースを解放する
        """
        if self._loop is None:
            return

        # Cancel telemetry task
        # テレメトリタスクをキャンセル
        if self._telem_task is not None:
            self._telem_task.cancel()
            try:
                self._telem_task.result(timeout=2.0)
            except (asyncio.CancelledError, Exception):
                pass
            self._telem_task = None

        # Disconnect
        # 切断
        if self._conn is not None:
            try:
                self._run_async(self._conn.disconnect())
            except Exception:
                pass

        self._cleanup()

    def _cleanup(self) -> None:
        """Stop event loop and thread.

        イベントループとスレッドを停止する
        """
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
            if self._thread is not None:
                self._thread.join(timeout=5.0)
            self._loop.close()

        self._loop = None
        self._thread = None
        self._conn = None
        self._telemetry = {}

    # ------------------------------------------------------------------
    # Flight commands / フライトコマンド
    # ------------------------------------------------------------------

    def takeoff(self) -> None:
        """Take off and hover at default altitude.

        離陸してデフォルト高度でホバリングする
        """
        self._flight_command("takeoff")

    def land(self) -> None:
        """Land the drone.

        着陸する
        """
        self._flight_command("land")

    def emergency(self) -> None:
        """Emergency motor stop (immediate, non-blocking).

        緊急モーター停止（即時、ノンブロッキング）
        """
        self._send_command("emergency")

    def stop(self) -> None:
        """Stop movement and hover in place (non-blocking).

        移動を停止してその場でホバリング（ノンブロッキング）
        """
        self._send_command("stop")

    # ------------------------------------------------------------------
    # Move commands / 移動コマンド
    # ------------------------------------------------------------------

    def move_up(self, x: int) -> None:
        """Move up by x cm (20-200).

        x cm 上昇する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"up {x}")

    def move_down(self, x: int) -> None:
        """Move down by x cm (20-200).

        x cm 下降する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"down {x}")

    def move_forward(self, x: int) -> None:
        """Move forward by x cm (20-200).

        x cm 前進する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"forward {x}")

    def move_back(self, x: int) -> None:
        """Move backward by x cm (20-200).

        x cm 後退する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"back {x}")

    def move_left(self, x: int) -> None:
        """Move left by x cm (20-200).

        x cm 左に移動する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"left {x}")

    def move_right(self, x: int) -> None:
        """Move right by x cm (20-200).

        x cm 右に移動する (20-200)
        """
        self._validate_distance(x)
        self._flight_command(f"right {x}")

    # ------------------------------------------------------------------
    # Rotation commands / 回転コマンド
    # ------------------------------------------------------------------

    def rotate_clockwise(self, x: int) -> None:
        """Rotate clockwise by x degrees (1-360).

        時計回りに x 度回転する (1-360)
        """
        self._validate_rotation(x)
        self._flight_command(f"cw {x}")

    def rotate_counter_clockwise(self, x: int) -> None:
        """Rotate counter-clockwise by x degrees (1-360).

        反時計回りに x 度回転する (1-360)
        """
        self._validate_rotation(x)
        self._flight_command(f"ccw {x}")

    # ------------------------------------------------------------------
    # RC control / RC 制御
    # ------------------------------------------------------------------

    def send_rc_control(
        self,
        left_right_velocity: int,
        forward_backward_velocity: int,
        up_down_velocity: int,
        yaw_velocity: int,
    ) -> None:
        """Send RC control values (each -100 to 100, non-blocking).

        RC 制御値を送信する（各 -100〜100、ノンブロッキング）

        Args:
            left_right_velocity: Left/right (-100=left, 100=right)
            forward_backward_velocity: Forward/backward (-100=back, 100=forward)
            up_down_velocity: Up/down (-100=down, 100=up)
            yaw_velocity: Yaw (-100=ccw, 100=cw)
        """
        a = self._clamp(left_right_velocity, self.RC_MIN, self.RC_MAX)
        b = self._clamp(forward_backward_velocity, self.RC_MIN, self.RC_MAX)
        c = self._clamp(up_down_velocity, self.RC_MIN, self.RC_MAX)
        d = self._clamp(yaw_velocity, self.RC_MIN, self.RC_MAX)
        self._send_command(f"rc {a} {b} {c} {d}")

    # ------------------------------------------------------------------
    # Query commands / クエリコマンド
    # ------------------------------------------------------------------

    def get_battery(self) -> int:
        """Get battery level in percent.

        バッテリー残量をパーセントで取得する
        """
        resp = self._send_command("battery?")
        return int(resp.strip())

    def get_height(self) -> int:
        """Get height in cm.

        高度を cm で取得する
        """
        resp = self._send_command("height?")
        return int(resp.strip())

    def get_distance_tof(self) -> int:
        """Get ToF distance in cm.

        ToF 距離を cm で取得する
        """
        resp = self._send_command("tof?")
        return int(resp.strip())

    def get_barometer(self) -> float:
        """Get barometric altitude in cm.

        気圧高度を cm で取得する
        """
        resp = self._send_command("baro?")
        return float(resp.strip())

    def get_attitude(self) -> dict:
        """Get attitude as dict with keys: pitch, roll, yaw (degrees).

        姿勢を dict で取得する（キー: pitch, roll, yaw、単位: 度）
        """
        resp = self._send_command("attitude?")
        parts = resp.strip().split()
        return {
            "pitch": int(parts[0]),
            "roll": int(parts[1]),
            "yaw": int(parts[2]),
        }

    def get_acceleration(self) -> dict:
        """Get acceleration as dict with keys: x, y, z (cm/s^2).

        加速度を dict で取得する（キー: x, y, z、単位: cm/s²）
        """
        resp = self._send_command("acceleration?")
        parts = resp.strip().split()
        return {
            "x": float(parts[0]),
            "y": float(parts[1]),
            "z": float(parts[2]),
        }

    def get_speed(self) -> int:
        """Get speed setting in cm/s.

        移動速度設定を cm/s で取得する
        """
        resp = self._send_command("speed?")
        return int(resp.strip())

    # ------------------------------------------------------------------
    # StampFly extensions / StampFly 拡張
    # ------------------------------------------------------------------

    def get_telemetry(self) -> dict:
        """Get latest WebSocket telemetry data (400Hz, StampFly extension).

        WebSocket テレメトリの最新値を取得する（400Hz、StampFly 拡張）
        """
        return dict(self._telemetry)

    def hover(self, altitude: float = 0.5, duration: float = 5.0) -> None:
        """Hover at specified altitude for duration (StampFly extension).

        指定高度で指定時間ホバリングする（StampFly 拡張）

        Args:
            altitude: Target altitude in meters (default 0.5)
            duration: Duration in seconds (default 5.0)
        """
        alt_cm = int(altitude * 100)
        self._flight_command(f"hover {alt_cm} {duration}")

    def jump(self, altitude: float = 0.15) -> None:
        """Jump to specified altitude (StampFly extension).

        指定高度にジャンプする（StampFly 拡張）

        Args:
            altitude: Target altitude in meters (default 0.15)
        """
        alt_cm = int(altitude * 100)
        self._flight_command(f"jump {alt_cm}")

    # ------------------------------------------------------------------
    # Internal helpers / 内部ヘルパー
    # ------------------------------------------------------------------

    def _run_async(self, coro, timeout: float = 30.0):
        """Run a coroutine on the background event loop and return result.

        バックグラウンドイベントループでコルーチンを実行し結果を返す
        """
        if self._loop is None:
            raise StampFlyConnectionError("Not connected")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    def _send_command(self, cmd: str, timeout: float = 5.0) -> str:
        """Send a CLI command and return response.

        CLI コマンドを送信して応答を返す
        """
        if self._conn is None:
            raise StampFlyConnectionError("Not connected")
        return self._run_async(self._conn.cli.send_command(cmd, timeout))

    def _flight_command(self, cmd: str, timeout: float = 120.0) -> None:
        """Send a flight command and wait for completion.

        フライトコマンドを送信して完了を待機する
        """
        if self._conn is None:
            raise StampFlyConnectionError("Not connected")

        async def _exec():
            resp = await self._conn.send_flight_command(cmd)
            if "Error" in resp or "Failed" in resp:
                raise StampFlyCommandError(resp)
            # Wait for flight to complete by polling status
            # ステータスポーリングでフライト完了を待機
            await self._conn.monitor_flight(
                callback=lambda t, s: None,
                poll_interval=0.5,
                timeout=timeout,
            )

        self._run_async(_exec(), timeout=timeout + 5.0)

    async def _telemetry_loop(self) -> None:
        """Background task to continuously receive telemetry.

        テレメトリを継続受信するバックグラウンドタスク
        """
        while True:
            try:
                data = await self._conn.telemetry.receive(timeout=1.0)
                if data:
                    self._telemetry = data
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(0.1)

    @staticmethod
    def _validate_distance(x: int) -> None:
        if x < StampFly.MOVE_DISTANCE_MIN or x > StampFly.MOVE_DISTANCE_MAX:
            raise ValueError(
                f"Distance must be {StampFly.MOVE_DISTANCE_MIN}-"
                f"{StampFly.MOVE_DISTANCE_MAX} cm, got {x}"
            )

    @staticmethod
    def _validate_rotation(x: int) -> None:
        if x < StampFly.ROTATION_MIN or x > StampFly.ROTATION_MAX:
            raise ValueError(
                f"Rotation must be {StampFly.ROTATION_MIN}-"
                f"{StampFly.ROTATION_MAX} degrees, got {x}"
            )

    @staticmethod
    def _clamp(value: int, min_val: int, max_val: int) -> int:
        return max(min_val, min(max_val, value))
