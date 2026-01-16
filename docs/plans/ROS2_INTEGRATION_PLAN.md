# ROS2 Integration Plan

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 目的

StampFlyエコシステムとROS2を連携させ、以下を実現する：

- ROS2エコシステムとの相互運用（rviz2、nav2、Gazebo等）
- 研究・教育目的での標準的なロボティクスインターフェース提供
- 将来的な自律飛行システムへの拡張基盤

### ステータス

| 項目 | 状態 |
|------|------|
| 計画策定 | ✅ 完了 (2026-01) |
| プロトタイプ | ⏳ 未着手 |
| 実装 | ⏳ 未着手 |

## 2. アーキテクチャ選択肢

### 比較表

| 項目 | A: ブリッジ | B: micro-ROS | C: コンパニオン |
|------|------------|--------------|----------------|
| 実装難易度 | ★☆☆ 低 | ★★★ 高 | ★★☆ 中 |
| ファーム改修 | 不要 | 大規模 | 小規模 |
| レイテンシ | ~10-20ms | ~5-10ms | ~5ms |
| 機体重量 | 変化なし | 変化なし | +15-30g |
| ROS2機能 | 制限あり | フル | フル |
| 推奨用途 | 研究・教育 | 製品開発 | 自律飛行 |

### A. ブリッジノード方式（推奨）

現在のStampFlyアーキテクチャを変更せず、PC上のROS2ノードがWebSocket経由でブリッジする方式。

```
┌──────────┐     WiFi      ┌──────────┐    ROS2     ┌──────────┐
│ StampFly │◄────────────►│  Bridge  │◄──────────►│ ROS2     │
│ (ESP32)  │  WebSocket    │  Node    │   Topics    │ Ecosystem│
└──────────┘               │  (PC)    │             └──────────┘
     │                     └──────────┘
     │ ESP-NOW
     ▼
┌──────────┐
│Controller│
└──────────┘
```

**メリット:**
- ファームウェア変更不要
- 既存のWebSocketテレメトリをそのまま活用
- 段階的な導入が可能

**デメリット:**
- PC依存（ブリッジノードが必要）
- レイテンシがやや大きい

### B. micro-ROS方式

ESP32-S3上でmicro-ROSを直接実行し、DDS-XRCEプロトコルでROS2と通信する方式。

```
┌──────────────────────┐          ┌──────────────────────┐
│   StampFly (ESP32)   │          │      Linux PC        │
│  ┌─────────────────┐ │  WiFi    │  ┌────────────────┐  │
│  │  micro-ROS App  │ │   UDP    │  │  micro-ROS     │  │
│  │  (ROS2 Node)    │ │ ◄──────► │  │  Agent         │  │
│  └────────┬────────┘ │ DDS-XRCE │  └───────┬────────┘  │
│  ┌────────┴────────┐ │          │  ┌───────┴────────┐  │
│  │ Micro XRCE-DDS  │ │          │  │   Full DDS     │  │
│  │ Client (~100KB) │ │          │  │   (FastDDS)    │  │
│  └─────────────────┘ │          │  └───────┬────────┘  │
└──────────────────────┘          │  ┌───────┴────────┐  │
                                  │  │  ROS2 Nodes    │  │
                                  │  └────────────────┘  │
                                  └──────────────────────┘
```

**メリット:**
- ネイティブROS2ノードとして動作
- 低レイテンシ
- Agent以外のPC依存なし

**デメリット:**
- ファームウェア大規模改修
- リソース制約（RAM ~100KB必要）
- デバッグが複雑

### C. コンパニオンコンピュータ方式

機体にRaspberry Pi等を搭載し、ROS2を直接実行する方式。

```
┌─────────────────────────┐          ┌──────────┐
│      StampFly           │  WiFi    │ ROS2     │
│  ┌───────┐  ┌────────┐  │◄────────►│ Ground   │
│  │ESP32  │◄─│Rasp Pi │  │ ROS2 DDS │ Station  │
│  │Flight │  │ +ROS2  │  │          └──────────┘
│  │Control│  │  Node  │  │
│  └───────┘  └────────┘  │
└─────────────────────────┘
```

**メリット:**
- フルROS2機能
- 高度な自律飛行が可能
- カメラ等の処理能力

**デメリット:**
- 重量増加（+15-30g）
- 消費電力増加
- 機体サイズ・バランスへの影響

## 3. micro-ROS詳細設計

### DDS-XRCEプロトコル

DDS for eXtremely Resource Constrained Environments の略。通常のDDSを軽量化したプロトコル。

| 項目 | Full DDS | DDS-XRCE |
|------|----------|----------|
| RAM | 数百MB | ~100KB |
| Flash | 数GB | ~100KB |
| Discovery | 自動 | Agent経由 |
| Transport | UDP Multicast | Serial/UDP P2P |

### ESP32-S3リソース評価

```
現在のファームウェア:
  Binary size: 1.8MB / 3MB partition (43% free)

RAM使用量（概算）:
  Static:           ~100KB
  FreeRTOS Tasks:   ~50KB
  Sensor Buffers:   ~20KB
  ESKF:             ~10KB
  WiFi/Telemetry:   ~40KB
  ─────────────────────────
  合計:             ~220KB
  ESP32-S3 Total:   512KB
  Available:        ~290KB ← micro-ROSに使用可能
```

**結論:** ESP32-S3でmicro-ROSは動作可能だが、余裕は少ない

### タスク構成案

```cpp
// 優先度設計
// Priority design
#define PRIORITY_IMU        (configMAX_PRIORITIES - 1)  // 最高
#define PRIORITY_CONTROL    (configMAX_PRIORITIES - 2)
#define PRIORITY_MICRO_ROS  (configMAX_PRIORITIES - 3)  // 中
#define PRIORITY_TELEMETRY  (configMAX_PRIORITIES - 4)
#define PRIORITY_SENSORS    (configMAX_PRIORITIES - 5)

// タスク間通信
// Inter-task communication
QueueHandle_t g_imu_to_ros_queue;      // IMU → micro-ROS (non-blocking)
QueueHandle_t g_ros_to_control_queue;  // micro-ROS → Control
```

### ROS2トピック設計

```yaml
# Publishers (StampFly → ROS2)
/stampfly/imu/raw:           sensor_msgs/Imu          # 400Hz
/stampfly/imu/filtered:      sensor_msgs/Imu          # 400Hz
/stampfly/pose:              geometry_msgs/PoseStamped # 100Hz
/stampfly/velocity:          geometry_msgs/TwistStamped # 100Hz
/stampfly/battery:           sensor_msgs/BatteryState  # 1Hz
/stampfly/range/bottom:      sensor_msgs/Range         # 30Hz
/stampfly/optical_flow:      stampfly_msgs/OpticalFlow # 100Hz

# Subscribers (ROS2 → StampFly)
/stampfly/cmd_vel:           geometry_msgs/Twist       # 速度指令
/stampfly/cmd_attitude:      stampfly_msgs/AttitudeCmd # 姿勢指令
/stampfly/arm:               std_msgs/Bool             # Arm/Disarm

# Services
/stampfly/set_mode:          stampfly_msgs/SetMode
/stampfly/calibrate:         std_srvs/Trigger
```

### QoS設定

```cpp
// センサデータ: Best Effort（ドロップ許容、低レイテンシ優先）
rmw_qos_profile_t sensor_qos = {
    .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 1,
};

// コマンド: Reliable（確実な配信）
rmw_qos_profile_t command_qos = {
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 10,
};
```

### Agent接続断への対応

```cpp
// Fallback機構
// Agent切断時はESP-NOWコントローラにフォールバック
void check_agent_connection() {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        g_use_ros2_control = false;  // ESP-NOW fallback
        ESP_LOGW(TAG, "Agent disconnected, using ESP-NOW");
    } else {
        g_use_ros2_control = true;
    }
}
```

## 4. 実装ロードマップ

### Phase 1: ブリッジノード（推奨開始点）

```
目標: 最小限の変更でROS2連携を実現
期間: 2-3週間

1. ROS2パッケージ作成
   - stampfly_msgs（カスタムメッセージ）
   - stampfly_bridge（ブリッジノード）

2. WebSocket → ROS2 Publisher実装
   - /stampfly/imu/raw
   - /stampfly/pose

3. rviz2での可視化確認
```

### Phase 2: 双方向通信

```
目標: ROS2からの制御を実現
期間: 2-3週間

1. ファームウェア: WebSocketコマンド受信追加
2. ブリッジ: /cmd_vel Subscriber追加
3. 制御ループの検証
```

### Phase 3: micro-ROS移行（オプション）

```
目標: ネイティブROS2ノード化
期間: 1-2ヶ月

1. micro-ROS ESP-IDFコンポーネント統合
2. 既存タスクとの共存設計
3. パフォーマンス最適化
4. ESP-NOWフォールバック実装
```

## 5. 参考リソース

| リソース | URL | 説明 |
|---------|-----|------|
| micro-ROS公式 | https://micro.ros.org/ | ドキュメント・チュートリアル |
| ESP-IDF Component | https://github.com/micro-ROS/micro_ros_espidf_component | ESP-IDF向け |
| PlatformIO版 | https://github.com/micro-ROS/micro_ros_platformio | Arduino/PlatformIO向け |
| ROS2 Humble | https://docs.ros.org/en/humble/ | ROS2 LTS版ドキュメント |

## 6. 備考

- 本計画は2026年1月時点の検討結果
- 実装優先度は他の開発タスクとの兼ね合いで決定
- micro-ROS方式はESP-IDFのバージョン互換性に注意

---

<a id="english"></a>

## 1. Overview

### Purpose

Integrate StampFly ecosystem with ROS2 to achieve:

- Interoperability with ROS2 ecosystem (rviz2, nav2, Gazebo, etc.)
- Standard robotics interface for research and education
- Foundation for future autonomous flight systems

### Status

| Item | Status |
|------|--------|
| Planning | ✅ Complete (2026-01) |
| Prototype | ⏳ Not started |
| Implementation | ⏳ Not started |

## 2. Architecture Options

### Comparison

| Item | A: Bridge | B: micro-ROS | C: Companion |
|------|-----------|--------------|--------------|
| Difficulty | ★☆☆ Low | ★★★ High | ★★☆ Medium |
| Firmware changes | None | Major | Minor |
| Latency | ~10-20ms | ~5-10ms | ~5ms |
| Weight impact | None | None | +15-30g |
| ROS2 features | Limited | Full | Full |
| Recommended for | Research/Education | Product dev | Autonomous flight |

### A. Bridge Node Approach (Recommended)

PC-based ROS2 node bridges via WebSocket without firmware changes.

**Pros:**
- No firmware changes required
- Leverages existing WebSocket telemetry
- Gradual adoption possible

**Cons:**
- PC dependency (bridge node required)
- Higher latency

### B. micro-ROS Approach

Run micro-ROS directly on ESP32-S3, communicate via DDS-XRCE protocol.

**Pros:**
- Native ROS2 node operation
- Low latency
- No PC dependency except Agent

**Cons:**
- Major firmware changes
- Resource constraints (~100KB RAM needed)
- Complex debugging

### C. Companion Computer Approach

Mount Raspberry Pi or similar on the vehicle running ROS2 directly.

**Pros:**
- Full ROS2 functionality
- Advanced autonomous flight capable
- Processing power for cameras

**Cons:**
- Weight increase (+15-30g)
- Power consumption increase
- Vehicle size/balance impact

## 3. micro-ROS Detailed Design

### ESP32-S3 Resource Evaluation

```
Current firmware:
  Binary size: 1.8MB / 3MB partition (43% free)

RAM usage (estimated):
  Static:           ~100KB
  FreeRTOS Tasks:   ~50KB
  Sensor Buffers:   ~20KB
  ESKF:             ~10KB
  WiFi/Telemetry:   ~40KB
  ─────────────────────────
  Total:            ~220KB
  ESP32-S3 Total:   512KB
  Available:        ~290KB ← usable for micro-ROS
```

**Conclusion:** micro-ROS is feasible on ESP32-S3 but with limited headroom.

### ROS2 Topic Design

See Japanese section for detailed topic list.

## 4. Implementation Roadmap

### Phase 1: Bridge Node (Recommended Starting Point)
- Goal: Minimal changes for ROS2 integration
- Duration: 2-3 weeks

### Phase 2: Bidirectional Communication
- Goal: Enable control from ROS2
- Duration: 2-3 weeks

### Phase 3: micro-ROS Migration (Optional)
- Goal: Native ROS2 node
- Duration: 1-2 months

## 5. References

| Resource | URL | Description |
|----------|-----|-------------|
| micro-ROS Official | https://micro.ros.org/ | Documentation & tutorials |
| ESP-IDF Component | https://github.com/micro-ROS/micro_ros_espidf_component | For ESP-IDF |
| PlatformIO | https://github.com/micro-ROS/micro_ros_platformio | For Arduino/PlatformIO |
| ROS2 Humble | https://docs.ros.org/en/humble/ | ROS2 LTS documentation |

## 6. Notes

- This plan reflects considerations as of January 2026
- Implementation priority to be determined alongside other development tasks
- micro-ROS approach requires attention to ESP-IDF version compatibility
