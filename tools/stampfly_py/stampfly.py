"""
StampFly Python SDK — Tello-style programmatic flight
StampFly Python SDK — Tello 風プログラム飛行

Talks the Tello SDK text protocol to the vehicle_new firmware's ApiTask
(UDP :8889). Every command blocks until the vehicle replies ("ok" after the
move is REACHED, Tello-style), so a script reads like a flight plan:

vehicle_new ファームの ApiTask（UDP :8889）と Tello SDK テキストプロトコルで
通信する。各コマンドは機体の応答（移動は「到達後」に ok — Tello 流）まで
ブロックするので、スクリプトがそのまま飛行計画として読める:

    from stampfly import StampFly

    fly = StampFly()              # AP mode default 192.168.4.1 / AP モード既定
    fly.connect()                 # SDK mode ("command")
    fly.takeoff()                 # POS_HOLD auto-takeoff -> hover at 0.8 m
    fly.forward(50)               # move 50 cm forward (blocks until reached)
    fly.cw(90)                    # turn 90 deg clockwise
    fly.up(30)
    print(fly.battery(), "%")
    fly.land()

Safety / 安全:
  - Keep the paired RC transmitter ON with neutral sticks: moving any stick
    cancels API guidance instantly (the pilot always wins), and the comm-loss
    failsafe (auto-landing) stays armed underneath the API.
  - emergency() cuts the motors immediately in ANY state.
  - ペアリング済み送信機を中立で持っておくこと: スティックを動かすと API 誘導は
    即解除（パイロット優先）。通信断フェイルセーフ（自動着陸）も API の下で有効。
  - emergency() はどの状態でも即時モータ停止。
"""

import socket


class StampFlyError(RuntimeError):
    """Raised when the vehicle replies "error ..." or a reply times out.
    機体が "error ..." を返すか応答がタイムアウトしたときに送出。"""


class StampFly:
    """Tello-style client for the StampFly vehicle_new API (UDP :8889).
    StampFly vehicle_new API（UDP :8889）の Tello 風クライアント。"""

    def __init__(self, ip: str = "192.168.4.1", port: int = 8889):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))   # ephemeral local port; replies come back here

    # -- core ----------------------------------------------------------------

    def send(self, command: str, timeout: float = 10.0) -> str:
        """Send one command and wait for the reply (blocking, Tello-style).
        コマンドを1つ送り応答を待つ（ブロッキング、Tello 流）。"""
        self.sock.settimeout(timeout)
        self.sock.sendto(command.encode(), self.addr)
        try:
            data, _ = self.sock.recvfrom(1024)
        except socket.timeout:
            raise StampFlyError(f"no reply to '{command}' within {timeout:.0f}s "
                                "(same WiFi? vehicle powered?)") from None
        reply = data.decode(errors="replace").strip()
        if reply.startswith("error"):
            raise StampFlyError(f"'{command}' -> {reply}")
        return reply

    def connect(self) -> None:
        """Enter SDK mode ("command"). / SDK モードに入る（"command"）。"""
        self.send("command", timeout=3.0)

    # -- flight verbs / 飛行 verb ---------------------------------------------

    def takeoff(self) -> None:
        """POS_HOLD auto-takeoff -> hover at 0.8 m (blocks until reached).
        POS_HOLD 自動離陸 → 0.8 m ホバー（到達までブロック）。"""
        self.send("takeoff", timeout=20.0)

    def land(self) -> None:
        """Autonomous landing (0.3 m/s, blocks until touchdown).
        自動着陸（0.3 m/s、接地までブロック）。"""
        self.send("land", timeout=30.0)

    def emergency(self) -> None:
        """Immediate motor cut — any state. / 即時モータ停止 — どの状態でも。"""
        self.send("emergency", timeout=3.0)

    def stop(self) -> None:
        """Halt and hold the current position. / 現在位置で停止保持。"""
        self.send("stop", timeout=5.0)

    # -- moves (cm, 10..300; block until reached) / 移動（cm、到達までブロック） --

    def up(self, cm: int) -> None:      self._move("up", cm)
    def down(self, cm: int) -> None:    self._move("down", cm)
    def left(self, cm: int) -> None:    self._move("left", cm)
    def right(self, cm: int) -> None:   self._move("right", cm)
    def forward(self, cm: int) -> None: self._move("forward", cm)
    def back(self, cm: int) -> None:    self._move("back", cm)

    def _move(self, verb: str, cm: int) -> None:
        # generous timeout: distance / 0.3 m/s + margin / 余裕あるタイムアウト
        self.send(f"{verb} {int(cm)}", timeout=cm / 30.0 + 10.0)

    def cw(self, deg: int) -> None:
        """Turn clockwise [deg]. / 時計回りに回頭 [deg]。"""
        self.send(f"cw {int(deg)}", timeout=15.0)

    def ccw(self, deg: int) -> None:
        """Turn counter-clockwise [deg]. / 反時計回りに回頭 [deg]。"""
        self.send(f"ccw {int(deg)}", timeout=15.0)

    def go(self, x_cm: int, y_cm: int, z_cm: int, speed_cms: int = 30) -> None:
        """Relative move in the body frame (x fwd, y left, z up) at speed [cm/s].
        機体座標の相対移動（x前/y左/z上）、速度 [cm/s]。"""
        dist = (x_cm**2 + y_cm**2 + z_cm**2) ** 0.5
        self.send(f"go {int(x_cm)} {int(y_cm)} {int(z_cm)} {int(speed_cms)}",
                  timeout=dist / max(10, speed_cms) + 10.0)

    # -- queries / クエリ ------------------------------------------------------

    def battery(self) -> int:
        """Battery [%] (1S LiPo voltage mapped 3.3 V=0 .. 4.2 V=100).
        電池残量 [%]（1S LiPo 電圧の写像）。"""
        return int(self.send("battery?", timeout=3.0))

    def height(self) -> int:
        """Estimated altitude [cm]. / 推定高度 [cm]。"""
        return int(self.send("height?", timeout=3.0))

    def attitude(self) -> dict:
        """Attitude {pitch, roll, yaw} [deg]. / 姿勢 [deg]。"""
        out = {}
        for part in self.send("attitude?", timeout=3.0).split(";"):
            if ":" in part:
                key, val = part.split(":")
                out[key] = int(val)
        return out

    def speed(self) -> int:
        """Estimated speed magnitude [cm/s]. / 推定速度の大きさ [cm/s]。"""
        return int(self.send("speed?", timeout=3.0))

    def close(self) -> None:
        self.sock.close()

    # Context manager: `with StampFly() as fly:` lands on exceptions.
    # コンテキストマネージャ: 例外時は着陸を試みる。
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is not None:
            try:
                self.land()
            except Exception:
                try:
                    self.emergency()
                except Exception:
                    pass
        self.close()
        return False
