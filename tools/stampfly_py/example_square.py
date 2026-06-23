"""
example_square.py — fly a 50 cm square and come home
example_square.py — 50 cm の正方形を飛んで戻る

Run / 実行:
  1. Power the vehicle on its battery, place it still, wait for the ready
     chime (still calibration). Keep the paired RC ON with neutral sticks
     (safety pilot — moving a stick cancels the API instantly).
     機体をバッテリーで起動し静置、Ready 音まで待つ（静止校正）。ペアリング済み
     送信機は中立のまま保持（安全要員 — スティックを動かすと API は即解除）。
  2. Join the vehicle's WiFi (AP mode: StampFly-XXXX) or the shared router.
     機体の WiFi（AP: StampFly-XXXX）または共有ルータに接続。
  3. python3 example_square.py [vehicle_ip]

The SoftAP is addressed at 192.168.10.1 (the DJI Tello subnet), so the default
below matches a real Tello. In STA mode pass the vehicle's LAN IP as an argument.
SoftAP は 192.168.10.1（DJI Tello のサブネット）で動くため、下の既定は実機 Tello と
同じ。STA モードでは機体の LAN IP を引数で渡す。
"""

import sys
from stampfly import StampFly

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.1"

# The context manager enters SDK mode and auto-lands on any exception.
# コンテキストマネージャが SDK モードに入り、例外時は自動着陸する。
with StampFly(ip) as fly:
    print("battery:", fly.battery(), "%")
    fly.takeoff()
    print("hovering at", fly.height(), "cm")

    for side in range(4):
        fly.forward(50)
        fly.cw(90)
        print(f"corner {side + 1}: attitude {fly.attitude()}")

    fly.land()
    print("landed.")
