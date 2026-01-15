# StampFly Protocol Specification

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

通信プロトコルの仕様定義（Single Source of Truth）。

## 1. 概要

StampFlyエコシステムは以下の通信プロトコルを使用：

| プロトコル | 用途 | 方向 | レート |
|-----------|------|------|--------|
| ESP-NOW + TDMA | 制御・基本テレメトリ | Controller ↔ Vehicle | 50Hz |
| WebSocket | 拡張テレメトリ（ESKF+センサ） | Vehicle → GCS | 400Hz |

## 2. ディレクトリ構成

```
protocol/
├── README.md              # 本ドキュメント
├── spec/                  # 機械可読なプロトコル仕様
│   ├── messages.yaml      # メッセージ定義
│   ├── espnow_tdma.yaml   # ESP-NOW & TDMAプロトコル
│   └── websocket.yaml     # WebSocketテレメトリ
├── generated/             # 仕様から生成されたコード
└── tools/                 # 仕様検証、コード生成ツール
```

## 3. メッセージ一覧

### ControlPacket (14 bytes)
コントローラから機体への制御コマンド。

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 3 | drone_mac | 宛先MACアドレス（下位3バイト）|
| 3 | 2 | throttle | スロットル (0-1000) |
| 5 | 2 | roll | ロール (500=中央) |
| 7 | 2 | pitch | ピッチ (500=中央) |
| 9 | 2 | yaw | ヨー (500=中央) |
| 11 | 1 | flags | 制御フラグ |
| 12 | 1 | reserved | 予約 |
| 13 | 1 | checksum | チェックサム（sum） |

**制御フラグ:**
- bit0: ARM - モーターアーム
- bit1: FLIP - フリップトリガー
- bit2: MODE - フライトモード切替
- bit3: ALT_MODE - 高度維持モード

### TelemetryPacket (22 bytes)
機体からコントローラへの基本テレメトリ（ESP-NOW）。

| Offset | Size | Field | Unit | Description |
|--------|------|-------|------|-------------|
| 0 | 1 | header | - | 0xAA |
| 1 | 1 | packet_type | - | 0x01 |
| 2 | 1 | sequence | - | シーケンス番号 |
| 3 | 2 | battery_mv | mV | バッテリー電圧 |
| 5 | 2 | altitude_cm | cm | 高度 |
| 7 | 2 | velocity_x | mm/s | X速度 |
| 9 | 2 | velocity_y | mm/s | Y速度 |
| 11 | 2 | velocity_z | mm/s | Z速度 |
| 13 | 2 | roll_deg10 | 0.1deg | ロール角 |
| 15 | 2 | pitch_deg10 | 0.1deg | ピッチ角 |
| 17 | 2 | yaw_deg10 | 0.1deg | ヨー角 |
| 19 | 1 | state | - | FlightState |
| 20 | 1 | flags | - | 警告フラグ |
| 21 | 1 | checksum | - | チェックサム（sum） |

### TelemetryWSPacket (116 bytes) - Legacy
機体からGCSへの拡張テレメトリ（WebSocket、レガシー50Hz用）。

| Offset | Size | Field | Unit | Description |
|--------|------|-------|------|-------------|
| 0 | 1 | header | - | 0xAA |
| 1 | 1 | packet_type | - | 0x20 |
| 2 | 4 | timestamp_ms | ms | 起動からの経過時間 |
| 6 | 4 | roll | rad | ロール角（ESKF推定） |
| 10 | 4 | pitch | rad | ピッチ角（ESKF推定） |
| 14 | 4 | yaw | rad | ヨー角（ESKF推定） |
| 18 | 12 | position | m | 位置 (x,y,z) NED座標 |
| 30 | 12 | velocity | m/s | 速度 (x,y,z) |
| 42 | 12 | gyro | rad/s | ジャイロ（バイアス補正済） |
| 54 | 12 | accel | m/s² | 加速度（バイアス補正済） |
| 66 | 16 | control | - | 制御入力 (throttle,roll,pitch,yaw) |
| 82 | 12 | magnetometer | uT | 磁力計 (x,y,z) |
| 94 | 4 | voltage | V | バッテリー電圧 |
| 98 | 4 | tof_bottom | m | 底面ToF距離 |
| 102 | 4 | tof_front | m | 前方ToF距離 |
| 106 | 1 | flight_state | - | FlightState |
| 107 | 1 | sensor_status | - | センサ状態フラグ |
| 108 | 4 | heartbeat | - | パケットカウンタ |
| 112 | 1 | checksum | - | チェックサム（XOR） |
| 113 | 3 | padding | - | アラインメント |

### TelemetryExtendedBatchPacket (552 bytes) - 現行
400Hz統一テレメトリ。4サンプル×136バイト/サンプルをバッチ送信。

**パケットヘッダ (4 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | header | 0xBD |
| 1 | 1 | packet_type | 0x32 |
| 2 | 1 | sample_count | 4 |
| 3 | 1 | reserved | 0 |

**ExtendedSample (136 bytes × 4 = 544 bytes):**

各サンプルは以下の3セクションで構成：

**センサデータ (56 bytes):**

| Offset | Size | Field | Unit | Description |
|--------|------|-------|------|-------------|
| 0 | 4 | timestamp_us | μs | 起動からの経過時間 |
| 4 | 12 | gyro_xyz | rad/s | 生ジャイロ（LPFのみ）|
| 16 | 12 | accel_xyz | m/s² | 生加速度（LPFのみ）|
| 28 | 12 | gyro_corrected_xyz | rad/s | バイアス補正済みジャイロ |
| 40 | 16 | ctrl_throttle/roll/pitch/yaw | - | 正規化制御入力 [-1,1] |

**ESKF推定値 (60 bytes):**

| Offset | Size | Field | Unit | Description |
|--------|------|-------|------|-------------|
| 56 | 16 | quat_wxyz | - | 姿勢クォータニオン |
| 72 | 12 | pos_xyz | m | 位置 NED座標 |
| 84 | 12 | vel_xyz | m/s | 速度 NED座標 |
| 96 | 6 | gyro_bias_xyz | rad/s×10000 | ジャイロバイアス（int16） |
| 102 | 6 | accel_bias_xyz | m/s²×10000 | 加速度バイアス（int16） |
| 108 | 1 | eskf_status | - | bit0: initialized |
| 109 | 7 | padding | - | アラインメント |

**追加センサ (20 bytes):**

| Offset | Size | Field | Unit | Description |
|--------|------|-------|------|-------------|
| 116 | 4 | baro_altitude | m | 気圧高度 |
| 120 | 4 | tof_bottom | m | 底面ToF距離 |
| 124 | 4 | tof_front | m | 前方ToF距離 |
| 128 | 4 | flow_x/y | counts | 光学フロー生データ（int16×2） |
| 132 | 1 | flow_quality | - | フロー品質（SQUAL） |
| 133 | 3 | padding | - | アラインメント |

**パケットフッタ (4 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 548 | 1 | checksum | XOR（bytes 0-547） |
| 549 | 3 | padding | アラインメント |

**帯域計算:**
- パケットサイズ: 552 bytes
- フレームレート: 100 fps
- 実効サンプルレート: 400 Hz (4 samples × 100 fps)
- 帯域幅: 552 × 100 × 8 = 442 kbps

## 4. TDMAフレーム構造

```
|<------------------- 20ms Frame ------------------->|
|Beacon|Slot0|Slot1|Slot2|Slot3|Slot4|...|Slot8|Slot9|
|500us |2ms  |2ms  |2ms  |2ms  |2ms  |...|2ms  |2ms  |
```

- **フレーム周期**: 20ms (50Hz)
- **スロット数**: 10
- **スロット幅**: 2ms
- **ビーコン**: フレーム開始500μs前に送信

## 5. チェックサム

### Sum (ControlPacket, TelemetryPacket)
```c
uint8_t sum = 0;
for (size_t i = 0; i < packet_size - 1; i++) {
    sum += data[i];
}
```

### XOR (TelemetryWSPacket)
```c
uint8_t checksum = 0;
for (size_t i = 0; i < checksum_offset; i++) {
    checksum ^= data[i];
}
```

## 6. タイムアウト

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| BEACON_LOSS | 50ms | ビーコンロス検出 |
| COMM_TIMEOUT | 500ms | 通信途絶検出 |
| DRONE_RETRY | 5000ms | 再接続間隔 |

## 7. ペアリング

1. コントローラがペアリングモードに入る（ボタン押下）
2. コントローラがブロードキャストでペアリング要求を送信
3. 機体がMACアドレスを含むレスポンスを返す
4. コントローラがMACアドレスをNVSに保存
5. TDMA通信開始

**ペアリングパケット（11 bytes）:**
- Byte 0: WiFiチャンネル
- Byte 1-6: ドローンMACアドレス
- Byte 7-10: シグネチャ (0xAA 0x55 0x16 0x88)

## 8. 実装ファイル

### Vehicle
- `firmware/vehicle/components/sf_svc_comm/` - ESP-NOW通信
- `firmware/vehicle/components/sf_svc_telemetry/` - WebSocketテレメトリ

### Controller
- `firmware/controller/components/espnow_tdma/` - TDMA実装

## 9. 設計原則

エコシステム内の全ての通信実装はこの仕様から派生する。

1. **Single Source of Truth**: `protocol/spec/` が正式な仕様
2. **バイナリ互換**: パケット構造はバイト単位で定義
3. **拡張性**: packet_type による将来のメッセージ追加対応
4. **シンプルなチェックサム**: 低レイテンシ優先

---

<a id="english"></a>

# Protocol Specification (English)

Communication protocol specification (Single Source of Truth).

## 1. Overview

StampFly ecosystem uses the following protocols:

| Protocol | Purpose | Direction | Rate |
|----------|---------|-----------|------|
| ESP-NOW + TDMA | Control & basic telemetry | Controller ↔ Vehicle | 50Hz |
| WebSocket | Extended telemetry (ESKF+sensors) | Vehicle → GCS | 400Hz |

See `spec/*.yaml` for detailed machine-readable specifications.

**Key Packet Types:**
- `0xAA/0x20`: Legacy WebSocket packet (116 bytes, deprecated)
- `0xBD/0x32`: Extended batch packet (552 bytes, current)
  - 4 samples × 136 bytes with ESKF estimates and sensor data
  - Effective rate: 400 Hz (4 samples × 100 fps)
