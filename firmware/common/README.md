# firmware/common

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

vehicle / controller で共有される組込み向け共通実装。

## ディレクトリ構成

```
common/
└── protocol/
    └── include/
        └── udp_protocol.hpp    # UDP通信プロトコル定義
```

- `protocol/` - 通信プロトコルの組込み側実装（エンコード・デコード、CRC、パケット構造体）

## UDP プロトコル

`protocol/include/udp_protocol.hpp` は Vehicle と Controller で共有される UDP 通信プロトコルを定義：

### 主要な構造体

| 構造体 | サイズ | 用途 |
|--------|--------|------|
| `ControlPacket` | 16 bytes | Controller → Vehicle 制御コマンド |
| `TelemetryPacket` | 20 bytes | Vehicle → Controller テレメトリ |
| `HeartbeatPacket` | 8 bytes | 接続維持用ハートビート |

### 定数

| 定数 | 値 | 説明 |
|------|-----|------|
| `CONTROL_PORT` | 8888 | 制御パケット受信ポート |
| `TELEMETRY_PORT` | 8889 | テレメトリ送信ポート |
| `DEFAULT_VEHICLE_IP` | 192.168.4.1 | Vehicle の IP アドレス |

### 使用例

```cpp
#include "udp_protocol.hpp"

// 制御パケット構築
auto pkt = stampfly::udp::buildControlPacket(
    seq++,                           // シーケンス番号
    stampfly::udp::DEVICE_ID_CONTROLLER,  // デバイスID
    throttle, roll, pitch, yaw,      // 制御値
    flags                            // 制御フラグ
);

// パケット検証
if (stampfly::udp::validateControlPacket(pkt)) {
    // 有効なパケット
}
```

※ 仕様の単一の真実（SSOT）は `protocol/` ディレクトリに置く。

---

<a id="english"></a>

# firmware/common

Shared embedded code between vehicle and controller.

## Directory Structure

```
common/
└── protocol/
    └── include/
        └── udp_protocol.hpp    # UDP communication protocol
```

- `protocol/` - Communication protocol implementation (encode/decode, CRC, packet structures)

## UDP Protocol

`protocol/include/udp_protocol.hpp` defines the UDP communication protocol shared between Vehicle and Controller:

### Key Structures

| Structure | Size | Purpose |
|-----------|------|---------|
| `ControlPacket` | 16 bytes | Controller → Vehicle control command |
| `TelemetryPacket` | 20 bytes | Vehicle → Controller telemetry |
| `HeartbeatPacket` | 8 bytes | Connection keep-alive |

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `CONTROL_PORT` | 8888 | Control packet receive port |
| `TELEMETRY_PORT` | 8889 | Telemetry send port |
| `DEFAULT_VEHICLE_IP` | 192.168.4.1 | Vehicle's IP address |

### Usage Example

```cpp
#include "udp_protocol.hpp"

// Build control packet
auto pkt = stampfly::udp::buildControlPacket(
    seq++,                           // Sequence number
    stampfly::udp::DEVICE_ID_CONTROLLER,  // Device ID
    throttle, roll, pitch, yaw,      // Control values
    flags                            // Control flags
);

// Validate packet
if (stampfly::udp::validateControlPacket(pkt)) {
    // Valid packet
}
```

Note: The Single Source of Truth (SSOT) for the specification is in the `protocol/` directory.
