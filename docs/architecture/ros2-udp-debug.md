# ROS2 UDP Control デバッグメモ

> 作成日: 2026-01-21
> ステータス: デバッグ中断 - ROS2ブリッジからのUDP送信が動作しない

> **注記（2026年promotion後）:** 本メモが対象としていた機体側コンポーネント（`sf_svc_udp`・`sf_svc_control_arbiter`、および `main/init.cpp` でのControlArbiter初期化）は、レイヤード命名の旧ファーム（現在は `firmware/vehicle_old` として凍結、実機POS_HOLD検証を経て現行機は `firmware/vehicle`＝旧 `vehicle_new` にpromotion済み）にのみ存在し、現行の `firmware/vehicle` には存在しない。現行ファームには本メモのようなUDPベースのControlArbiter機構自体がなく、外部プログラムからの制御は Tello SDK 互換の `sf_api` コンポーネント（UDPポート8889でコマンド、8890でテレメトリ／状態ストリーム、`docs/architecture/tello-api-reference.md` 参照）が担う。ただし `sf_api` のパケット形式はTello準拠のテキストコマンドであり、本メモが示す独自バイナリ制御パケット（CRC16付き）とは別物 — ROS2ブリッジ作業を再開する場合は `firmware/vehicle_old` を対象に続けるか、`sf_api` 向けに設計し直す必要がある。以下は当時（`firmware/vehicle_old`）のデバッグ記録としてそのまま残す。
>
> なお、Controller-Vehicle間のESP-NOW制御パケット（本メモの独自UDPパケットとは別チャネル）は、旧vehicle・新vehicle・送信機で個別に重複定義されていたが、`firmware/common/protocol/include/espnow_protocol.hpp`（`stampfly::protocol::ControlPacket`）に一本化済み。本メモのUDPパケットはこの統合の対象外（別プロトコル）。

## 1. 現状サマリー

### 動作確認済み
- ✅ StampFly側のUDPサーバー (port 8888) は正常に動作
- ✅ ControlArbiterの初期化問題を修正済み (commit: cd675c4)
- ✅ Pythonスクリプトからの直接UDP送信は成功 (UDP packets カウント増加確認)
- ✅ VMからStampFlyへのネットワーク接続は正常 (192.168.4.x)

### 未解決
- ❌ ROS2 stampfly_bridge からのUDP制御パケット送信が動作しない

## 2. 環境

- **ホスト**: Mac (UTM)
- **VM**: Ubuntu + ROS2 Humble
- **ネットワーク**: UTM Bridged mode (en0 WiFi) → StampFly AP (192.168.4.1)
- **VM IP**: 192.168.4.x (StampFly WiFiに接続時)

## 3. StampFly側の確認コマンド

```bash
# シリアルモニタ接続
sf monitor

# 通信状態確認
comm

# 期待される出力（正常時）
# UDP:
#   Running: yes
#   Clients: 1
#   RX count: XXXX
#   Errors: 0
#
# Control Arbiter:
#   UDP packets: XXXX  ← これが増えれば受信OK
#   Active control: yes
```

## 4. 動作するテスト（Pythonスクリプト直接）

VM上で実行:

```bash
python3 << 'EOF'
import socket
import struct

def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

packet_without_crc = struct.pack(
    '<BBBB HHHH BB',
    0xAA, 0x01, 0x00, 0x01,
    0, 2048, 2048, 2048,
    0, 0
)
crc = calculate_crc16(packet_without_crc)
packet = packet_without_crc + struct.pack('<H', crc)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(10):
    sock.sendto(packet, ("192.168.4.1", 8888))
print("Sent 10 packets")
sock.close()
EOF
```

## 5. 動作しないテスト（ROS2ブリッジ経由）

### ブリッジ起動
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "from stampfly_bridge.bridge_node import main; main()" --ros-args -p enable_control:=true
```

期待されるログ:
```
Control enabled at 50.0 Hz
```

### 制御コマンド送信
```bash
# ARM
ros2 service call /stampfly/arm std_srvs/srv/SetBool "{data: true}"

# cmd_vel送信
ros2 topic pub /stampfly/cmd_vel geometry_msgs/Twist "{linear: {z: 0.1}}" --once
```

## 6. デバッグ手順

### Step 1: udp_client単体テスト
```bash
python3 << 'EOF'
import sys
sys.path.insert(0, '/home/ubuntu/ros2_ws/src/stampfly_bridge/stampfly_bridge')
from udp_client import UDPControlClient, ControlInput

client = UDPControlClient(host="192.168.4.1")
client.connect()

for i in range(100):
    client.send_control(ControlInput(throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0, armed=False))

print(f"Sent {client.packets_sent} packets, errors: {client.errors}")
client.disconnect()
EOF
```

→ StampFlyで `comm` 確認。UDP packets増加すればudp_client.pyは正常。

### Step 2: ブリッジのenable_control確認
ブリッジ起動時のログで `Control enabled at 50.0 Hz` が表示されているか確認。

### Step 3: control_timer_callback確認
bridge_node.py の `control_timer_callback()` が呼ばれているか確認（ログ追加）。

### Step 4: UDPClient初期化確認
`self.udp_client` が None でないか確認。

## 7. 関連ファイル

| ファイル | 説明 |
|---------|------|
| `ros/src/stampfly_bridge/stampfly_bridge/bridge_node.py` | ROS2ブリッジノード |
| `ros/src/stampfly_bridge/stampfly_bridge/udp_client.py` | UDP送信クライアント |
| `ros/src/stampfly_bridge/launch/bridge.launch.py` | ローンチファイル |
| `firmware/vehicle_old/main/init.cpp`（当時の記録では `firmware/vehicle/main/init.cpp`） | ControlArbiter初期化（修正済み）。現行 `firmware/vehicle` には存在しない |
| `firmware/vehicle_old/components/sf_svc_udp/udp_server.cpp` | UDPサーバー実装。現行 `firmware/vehicle` には存在しない |
| `firmware/vehicle_old/components/sf_svc_control_arbiter/control_arbiter.cpp` | 制御アービター。現行 `firmware/vehicle` には存在しない |

## 8. 疑わしい箇所

1. **bridge_node.py での UDPControlClient 初期化**
   - `enable_control` パラメータが正しく取得されているか
   - `self.udp_client` が作成されているか

2. **control_timer_callback() の実行**
   - タイマーが正しく動作しているか
   - `send_control()` が呼ばれているか

3. **パラメータの型変換**
   - `enable_control` が bool として正しく解釈されているか

## 9. 次のアクション

1. udp_client単体テスト実行 → 動作確認
2. bridge_node.py にデバッグログ追加
3. enable_control パラメータの取得確認
4. control_timer_callback の実行確認
