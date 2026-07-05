"""
example_djitellopy2.py — variant of example_djitellopy.py: hover pause + zigzag pattern
example_djitellopy2.py — example_djitellopy.py のバリエーション（ホバー待機＋ジグザグ飛行）

This is an ORDINARY Tello program. The only reason it runs on StampFly is that the
firmware speaks the Tello SDK protocol (UDP:8889 text commands + UDP:8890 state
stream) and the SoftAP is addressed at 192.168.10.1 — djitellopy's default host —
so `Tello()` connects with no code change. Camera (streamon / get_frame_read) is
the one thing StampFly cannot do; everything else below works.
これは「ふつうの Tello プログラム」。StampFly で動くのは、ファームが Tello SDK
プロトコル（UDP:8889 テキストコマンド + UDP:8890 状態ストリーム）を話し、SoftAP が
192.168.10.1（djitellopy の既定ホスト）で動くから。`Tello()` を無改変で接続できる。
カメラ（streamon / get_frame_read）だけは StampFly では不可、それ以外は下記の通り動く。

Setup / 準備:
  pip install djitellopy
  1. Power the vehicle, place it still, wait for the ready chime (still calibration).
     Keep the paired RC ON with neutral sticks — it is the SAFETY PILOT: moving any
     stick cancels the API instantly and the pilot takes over.
     機体を起動・静置し Ready 音まで待つ（静止校正）。ペアリング済み送信機は中立で
     ON のまま — 安全パイロット: スティックを動かすと API は即解除されパイロットが勝つ。
  2. Join the vehicle's WiFi AP (SSID: StampFly-XXXX). Your PC gets 192.168.10.x.
     機体の WiFi AP（SSID: StampFly-XXXX）に接続。PC は 192.168.10.x を取得。
  3. python3 example_djitellopy2.py

Note: do NOT run `sf log wifi` at the same time — it also uses UDP:8890 on the PC.
注意: `sf log wifi` と同時に実行しない — PC 側で UDP:8890 を取り合う。
"""

from djitellopy import Tello
import time

# Tello() defaults to host 192.168.10.1 — exactly StampFly's SoftAP. No edit needed.
# For STA mode, pass Tello(host="<vehicle-LAN-ip>").
# Tello() の既定ホストは 192.168.10.1 = StampFly の SoftAP。無改変。STA は host= で渡す。
tello = Tello()

tello.connect()                       # sends "command"; needs the 8890 state stream
print(f"battery: {tello.get_battery()} %")
print(f"height : {tello.get_height()} cm   tof: {tello.get_distance_tof()} cm")

tello.takeoff()                       # POS_HOLD auto-takeoff -> hover
time.sleep(20)
# A 50 cm square, turning 90 deg at each corner.
# 各角で 90 度回りながら 50 cm 四方を飛ぶ。
for corner in range(3):
    tello.move_forward(30)            # cm
    tello.rotate_clockwise(180)        # deg
    print(f"corner {corner + 1}: height {tello.get_height()} cm")

tello.land()
tello.end()
print("done.")
