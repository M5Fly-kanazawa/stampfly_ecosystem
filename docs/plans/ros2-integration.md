# StampFly ROS2 連携計画（再設計）

状態: **計画中（再設計・再実装）**。作成 2026-01-16、最終更新 2026-09-13。

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> **2026-09-13 更新:** 旧ブリッジ実装 `ros/stampfly_bridge`（`stampfly_msgs` を含む。WebSocket:80 でのテレメトリ受信＋UDP:8888 での独自バイナリ制御パケット送信、対象は `firmware/vehicle_old` 専用）と、その配線調査メモ `docs/architecture/ros2-udp-debug.md` は、`vehicle_old` の削除に伴い 2026-09-13 に削除した（タグ `archive/2026-09-13`。原文は `git show archive/2026-09-13:ros/README.md` 等で参照できる）。旧ブリッジは削除前の時点で、対象としていた `vehicle_old` の最終版とも、現行 `firmware/vehicle` とも、いずれとも通信できない状態だった（詳細は §1「現状」）。本書はこれを踏まえ、現行 vehicle の外部 API を土台にした**再設計の計画**として全面的に書き直したものである。

## 1. 概要

### 目的

ROS2 連携は方針として維持する（プロジェクトオーナー決定）。rviz2・nav2・Gazebo 等の ROS2 エコシステムとの相互運用、研究・教育目的での標準的なロボティクスインターフェース提供、将来の自律飛行システムへの拡張基盤という狙いは旧計画から変わらない。実装は現行 `firmware/vehicle` の外部 API に合わせて一から作り直す。

### 現状

- 旧ブリッジ `ros/stampfly_bridge`（ROS2 Humble 想定、`ament_python`）は WebSocket:80 でテレメトリを受信し、UDP:8888 で独自バイナリ `ControlPacket` を送信する設計だった。これらの通信方式はいずれも `firmware/vehicle_old`（2026-09-13 削除済みの凍結レガシー）専用のもので、現行 `firmware/vehicle` には存在しない。
- `ros/` への実質的な最終機能コミットは 2026-01-20 で、以後およそ8か月放置されていた。その間 `firmware/vehicle` は `vehicle_new → vehicle` 昇格を含む大規模刷新を経ており、`ros/` はこれに一切追随していなかった。
- 削除前時点で ROS2 側 `packet_parser.py` が前提としていたテレメトリ構造体（136バイトの `ExtendedSample`）は、対象としていた `vehicle_old` 自身の最終フォーマット（208バイト、3回の拡張を経たもの）とも既に不一致だった。
- 以上により、旧ブリッジは削除前の時点で、現行 vehicle は元より対象としていた `vehicle_old` の最終版とも通信できない状態であり、削除による新規の機能損失は無い。

### 現行 vehicle の外部 API 一覧

再設計の土台となる、現行 `firmware/vehicle` が実際に話す通信（2026-09-13 時点。詳細は `docs/architecture/tello-api-reference.md`）:

| ポート/経路 | 方向 | 内容 | 定義ファイル |
|---|---|---|---|
| UDP 8889 | PC→機体（コマンド）、機体→PC（応答） | Tello 互換テキストコマンド（`command`/`takeoff`/`land`/`rc`/移動系/クエリ系 等） | `firmware/vehicle/tasks/api_task.cpp` |
| UDP 8890（送信専用） | 機体→PC | Tello 互換の状態文字列、10Hz | `firmware/vehicle/components/sf_telemetry/include/tello_state.hpp` |
| UDP 8890（機体側 bind の別ソケット、`sf log wifi` 用） | PC↔機体 | 400Hzサンプルのバイナリ統合パケット、50Hz送出 | `components/sf_telemetry/{data_stream.hpp,data_stream.cpp}` |
| UDP 5005（ブロードキャスト） | 機体→ブロードキャスト | `sf::TelemetryPacket`（104バイト：姿勢・角速度・加速度・位置・速度・推力/トルク・モータduty）、50Hz | `components/sf_telemetry/{telemetry.hpp,telemetry.cpp}` |
| TCP 23 | PC↔機体（対話式） | telnet的CLI。**飛行コマンドは無い** | `firmware/vehicle/tasks/cli_task.cpp` |
| ESP-NOW | コントローラ↔機体 | RF操縦リンク | `firmware/common/protocol/include/espnow_protocol.hpp` |

WebSocket、および UDP:8888 の独自バイナリ制御パケットは現行 vehicle には存在しない（`firmware/vehicle/docs/requirements.md` に撤去済みと明記）。

## 2. 再設計の選択肢

**決定はしない。** 着手時に選ぶための判断材料として並べる。

| 観点 | (a) PC側ブリッジノード方式 | (b) 機体側ROS2ネイティブ通信方式 |
|---|---|---|
| 方式概要 | PC上のROS2ノードがUDP 8889のTelloテキストコマンドで機体を制御し、UDP 8890/5005のテレメトリを購読してROS2トピック・サービスへ変換する | 機体ファームに micro-ROS 等で ROS2 互換通信（DDS-XRCE 等）を直接実装する |
| ファーム改修 | 不要（現行APIをそのまま使う） | 大規模（新規タスク追加・リソース確保・優先度設計が要る） |
| 実装コスト | 比較的低い（旧ブリッジのノード構成・メッセージ変換ロジックの一部を§3の記録から再利用できる） | 高い（ESP32-S3のRAM余裕・タスク優先度から要検討） |
| レイテンシ・頻度 | UDP 8889 のコマンドの大半はブロッキング応答方式で高頻度制御に不向き（`rc` のみ撃ちっぱなし＝連続送信可）。テレメトリは 8890/5005 で最大50Hz | ネイティブ通信のため低レイテンシが期待できる（要実測） |
| ワイヤフォーマットの安定性 | DataStream（UDP 8890バイナリ）は過去に複数回変更された実績があり、「安定した契約」になっていない点がリスク | 機体側で直接定義するため、ROS2 メッセージ定義自体がSSOTになりうる |
| 実績 | Tello互換API自体は `tools/stampfly_py/` で実利用実績がある | 実績なし（ゼロからの実装） |
| 向く場面 | 研究・教育、まず動かしたい場合 | 自律飛行など低レイテンシ・高信頼性が要る場合 |

## 3. 旧ブリッジから引き継げるもの（削除前の記録）

`ros/`（`ros/src/stampfly_bridge`・`ros/src/stampfly_msgs`）は前述の通り 2026-09-13 に削除したが、再実装時の参考として、削除前に確認したノード構成・トピック名・メッセージ定義・依存パッケージを以下に記録する。**本節が唯一の引き継ぎ先である**（原文は `git show archive/2026-09-13:ros/<path>` でも参照できる）。

### パッケージ構成

| パッケージ | ビルドツール | 依存 |
|---|---|---|
| `stampfly_msgs` | `ament_cmake` + `rosidl_default_generators` | `std_msgs`, `geometry_msgs`, `rosidl_default_runtime` |
| `stampfly_bridge` | `ament_python` | `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `stampfly_msgs`, `python3-websockets` |

対象 ROS2 ディストリビューション: Humble（`ros/README.md` のインストール手順が `ros-humble-tf2-ros` を指定）。

### ノード構成（`bridge_node.py` → 実行名 `bridge_node`）

- 単一ノード `stampfly_bridge`。`websocket_client.py`（テレメトリ受信）、`udp_client.py`（制御送信）、`transforms.py`（サンプル→ROSメッセージ変換）、`packet_parser.py`（バイナリテレメトリ解析）で構成。
- パラメータ: `host`（既定 `192.168.4.1`）、`port`（80）、`path`（`/ws`）、`auto_reconnect`、`publish_tf`、`odom_frame`（`odom`）、`base_frame`（`base_link`）、`imu_frame`（`imu_link`）、`enable_control`、`control_rate`（50Hz）、`max_throttle`/`max_roll_rate`/`max_pitch_rate`/`max_yaw_rate`。
- テレメトリは 500Hz ポーリングタイマー（2ms周期）でWebSocketクライアントの受信キューを drain し、サンプル毎に全トピックへ publish。接続状況は1Hzでログ出力。
- 制御は `enable_control=true` の場合のみ有効化。`control_rate` Hzのタイマーで `UDPControlClient` へ送信し、ノード終了時は自動 DISARM。

### トピック・サービス一覧

| 種別 | 名前 | 型 | 備考 |
|---|---|---|---|
| Pub | `/stampfly/imu/raw` | `stampfly_msgs/ImuRaw` | サンプル受信ごと |
| Pub | `/stampfly/imu/corrected` | `stampfly_msgs/ImuCorrected` | 同上 |
| Pub | `/stampfly/eskf/state` | `stampfly_msgs/ESKFState` | 同上 |
| Pub | `/stampfly/control/input` | `stampfly_msgs/ControlInput` | 同上 |
| Pub | `/stampfly/range/sensors` | `stampfly_msgs/RangeSensors` | 同上 |
| Pub | `/stampfly/flow` | `stampfly_msgs/OpticalFlow` | 同上 |
| Pub | `/stampfly/pose` | `geometry_msgs/PoseStamped` | 同上 |
| Pub | `/stampfly/velocity` | `geometry_msgs/TwistStamped` | 同上 |
| Pub | `/stampfly/imu` | `sensor_msgs/Imu` | 同上 |
| Pub | `/stampfly/range/bottom` | `sensor_msgs/Range` | 同上 |
| Pub | `/stampfly/range/front` | `sensor_msgs/Range` | 同上 |
| TF | `odom` → `base_link` | `tf2_ros.TransformBroadcaster` | `publish_tf=true` 時 |
| Sub | `/stampfly/cmd_vel` | `geometry_msgs/Twist` | `enable_control=true` 時のみ。`linear.z`→スロットル、`angular.x/y/z`→roll/pitch/yawレート |
| Srv | `/stampfly/arm` | `std_srvs/SetBool` | ARM/DISARM |

QoS: テレメトリ系トピックは `BEST_EFFORT` / `KEEP_LAST` / depth 10（高頻度データの取りこぼしを許容）。

### メッセージ定義（`stampfly_msgs`）

| メッセージ | フィールド |
|---|---|
| `ImuRaw` | `header`, `angular_velocity`(Vector3, rad/s), `linear_acceleration`(Vector3, m/s²) |
| `ImuCorrected` | `header`, `angular_velocity`（バイアス補正済み）, `linear_acceleration`（バイアス補正済み） |
| `ESKFState` | `header`, `orientation`(Quaternion), `position`(Point, ENU), `velocity`(Vector3, ENU), `gyro_bias`(Vector3), `accel_bias`(Vector3), `status`(uint8) |
| `ControlInput` | `header`, `throttle`(float32, 0〜1), `roll`/`pitch`/`yaw`(float32, -1〜1) |
| `RangeSensors` | `header`, `tof_bottom`/`tof_front`/`baro_altitude`(float32, m) |
| `OpticalFlow` | `header`, `flow_x`/`flow_y`(int16), `quality`(uint8, 0〜255) |

### 制御パケット（UDP:8888、`udp_client.py`）

`firmware/common/protocol/include/udp_protocol.hpp` の `ControlPacket`（16バイト）を送る設計だった: `header`（0xAA固定）／`packet_type`（0x01）／`sequence`（uint8）／`device_id`（uint8）／`throttle`・`roll`・`pitch`・`yaw`（各uint16、0〜4095・中央2048）／`flags`（bit0=ARM, bit1=FLIP, bit2=MODE, bit3=ALT_MODE）／`reserved`／`checksum`（CRC16-CCITT）。**この受信側実装は `firmware/vehicle_old` の `sf_svc_udp` にのみ存在し、`vehicle_old` と共に削除済み。現行 vehicle に対応する受信機構は無い。**

## 4. 未決事項

- 対応する ROS2 ディストリビューション（旧計画は Humble を前提にしていたが、再設計時に現行の推奨LTSを再検討する）
- §2 の (a)/(b) どちらの方式を採るか（本書では決定しない）
- テレメトリの購読のみを対象にするか、制御も含めるか（要件を再確認してから着手する）
- DataStream（UDP 8890バイナリ）のワイヤフォーマットの安定性をどう扱うか（過去に複数回変更された実績があるため、ROS2側の契約として使う場合は安定化が前提になる）

## 5. 参考リンク（旧計画から引き継ぎ）

| リソース | URL | 説明 |
|---|---|---|
| micro-ROS公式 | https://micro.ros.org/ | ドキュメント・チュートリアル |
| ESP-IDF Component | https://github.com/micro-ROS/micro_ros_espidf_component | ESP-IDF向け |
| ROS2 Humble | https://docs.ros.org/en/humble/ | ROS2 LTS版ドキュメント |

---

<a id="english"></a>

# StampFly ROS2 Integration Plan (Redesign)

Status: **Planning (redesign / reimplementation)**. Created 2026-01-16, last updated 2026-09-13.

> **2026-09-13 update:** The old bridge implementation `ros/stampfly_bridge` (with `stampfly_msgs`; it received telemetry over WebSocket:80 and sent a custom binary control packet over UDP:8888, targeting `firmware/vehicle_old` only) and its wiring investigation note `docs/architecture/ros2-udp-debug.md` were deleted on 2026-09-13 along with `vehicle_old` (tag `archive/2026-09-13`; the original text can be retrieved via `git show archive/2026-09-13:ros/README.md` etc.). Even before deletion, the old bridge could not communicate with either the final version of its target (`vehicle_old`) or the current `firmware/vehicle` (see "Current State" in §1). This document has been fully rewritten as a **redesign plan** built on the current vehicle's external API.

## 1. Overview

### Purpose

ROS2 integration remains a firm goal (project-owner decision). Interoperability with the ROS2 ecosystem (rviz2, nav2, Gazebo, etc.), providing a standard robotics interface for research and education, and a foundation for future autonomous-flight systems are unchanged from the old plan. The implementation will be built from scratch against the current `firmware/vehicle`'s external API.

### Current State

- The old bridge `ros/stampfly_bridge` (assumed ROS2 Humble, `ament_python`) was designed to receive telemetry over WebSocket:80 and send a custom binary `ControlPacket` over UDP:8888. Both of these communication methods are exclusive to `firmware/vehicle_old` (the frozen legacy firmware deleted 2026-09-13) and do not exist in the current `firmware/vehicle`.
- The last substantive feature commit to `ros/` was on 2026-01-20, roughly 8 months before this rewrite. In that time, `firmware/vehicle` underwent a major overhaul including the `vehicle_new → vehicle` promotion, which `ros/` never followed.
- Before deletion, the telemetry struct assumed by the ROS2 side's `packet_parser.py` (a 136-byte `ExtendedSample`) already did not match even the final format of its own target, `vehicle_old` (208 bytes, after three rounds of extension).
- As a result, the old bridge could not talk to the current vehicle, nor even to the final version of `vehicle_old` it targeted, before it was deleted — so no new functionality was lost by removing it.

### Current Vehicle's External API

The foundation for the redesign — what the current `firmware/vehicle` actually speaks as of 2026-09-13 (see `docs/architecture/tello-api-reference.md` for detail):

| Port/Path | Direction | Content | Definition |
|---|---|---|---|
| UDP 8889 | PC→vehicle (commands), vehicle→PC (replies) | Tello-compatible text commands (`command`/`takeoff`/`land`/`rc`/movement/queries, etc.) | `firmware/vehicle/tasks/api_task.cpp` |
| UDP 8890 (send-only) | vehicle→PC | Tello-compatible state string, 10Hz | `firmware/vehicle/components/sf_telemetry/include/tello_state.hpp` |
| UDP 8890 (a separate socket on the vehicle, for `sf log wifi`) | PC↔vehicle | Binary bundle packets of 400Hz samples, sent at 50Hz | `components/sf_telemetry/{data_stream.hpp,data_stream.cpp}` |
| UDP 5005 (broadcast) | vehicle→broadcast | `sf::TelemetryPacket` (104 bytes: attitude, angular rate, acceleration, position, velocity, thrust/torque, motor duty), 50Hz | `components/sf_telemetry/{telemetry.hpp,telemetry.cpp}` |
| TCP 23 | PC↔vehicle (interactive) | telnet-style CLI. **No flight commands** | `firmware/vehicle/tasks/cli_task.cpp` |
| ESP-NOW | controller↔vehicle | RF control link | `firmware/common/protocol/include/espnow_protocol.hpp` |

WebSocket and the custom binary control packet on UDP:8888 do not exist on the current vehicle (noted as removed in `firmware/vehicle/docs/requirements.md`).

## 2. Redesign Options

**No decision is made here.** These are laid out as material for a future decision.

| Aspect | (a) PC-side bridge node | (b) Vehicle-side native ROS2 communication |
|---|---|---|
| Approach | A ROS2 node on the PC controls the vehicle with Tello text commands over UDP 8889, and subscribes to UDP 8890/5005 telemetry, translating both to/from ROS2 topics and services | Implement ROS2-compatible communication (e.g. DDS-XRCE) directly on the firmware, e.g. via micro-ROS |
| Firmware changes | None (uses the current API as-is) | Major (new task(s), resource budgeting, priority design) |
| Implementation cost | Relatively low (some of the old bridge's node structure and message-conversion logic in §3 can be reused) | High (needs review of ESP32-S3 RAM headroom and task priorities) |
| Latency / rate | Most UDP 8889 commands are blocking-reply and unsuited to high-rate control (only `rc` is fire-and-forget / continuous). Telemetry tops out at 50Hz via 8890/5005 | Low latency expected from native communication (needs measurement) |
| Wire-format stability | The DataStream (UDP 8890 binary) has changed multiple times historically — a risk if used as a stable ROS2 contract | Defined directly on the vehicle side, so the ROS2 message definitions themselves could become the SSOT |
| Track record | The Tello-compatible API itself has real usage via `tools/stampfly_py/` | None (implementation from scratch) |
| Fits | Research/education, wanting something working quickly | Autonomous flight and other cases needing low latency and high reliability |

## 3. What Can Be Inherited From the Old Bridge (Recorded Before Deletion)

`ros/` (`ros/src/stampfly_bridge` and `ros/src/stampfly_msgs`) was deleted on 2026-09-13 as noted above. For reference during reimplementation, the node structure, topic names, message definitions, and dependencies confirmed before deletion are recorded here. **This section is the sole surviving record** (the original text can also be retrieved via `git show archive/2026-09-13:ros/<path>`).

### Package Structure

| Package | Build tool | Dependencies |
|---|---|---|
| `stampfly_msgs` | `ament_cmake` + `rosidl_default_generators` | `std_msgs`, `geometry_msgs`, `rosidl_default_runtime` |
| `stampfly_bridge` | `ament_python` | `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `stampfly_msgs`, `python3-websockets` |

Target ROS2 distribution: Humble (`ros/README.md`'s install instructions specified `ros-humble-tf2-ros`).

### Node Structure (`bridge_node.py` → executable `bridge_node`)

- A single node, `stampfly_bridge`, composed of `websocket_client.py` (telemetry receive), `udp_client.py` (control send), `transforms.py` (sample → ROS message conversion), and `packet_parser.py` (binary telemetry parsing).
- Parameters: `host` (default `192.168.4.1`), `port` (80), `path` (`/ws`), `auto_reconnect`, `publish_tf`, `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `enable_control`, `control_rate` (50Hz), `max_throttle`/`max_roll_rate`/`max_pitch_rate`/`max_yaw_rate`.
- Telemetry was drained from the WebSocket client's receive queue by a 500Hz polling timer (2ms period) and published to every topic per sample. Connection status was logged at 1Hz.
- Control was enabled only when `enable_control=true`, sent via a `control_rate` Hz timer to `UDPControlClient`, with automatic DISARM on node shutdown.

### Topics and Services

| Kind | Name | Type | Notes |
|---|---|---|---|
| Pub | `/stampfly/imu/raw` | `stampfly_msgs/ImuRaw` | Per received sample |
| Pub | `/stampfly/imu/corrected` | `stampfly_msgs/ImuCorrected` | Same |
| Pub | `/stampfly/eskf/state` | `stampfly_msgs/ESKFState` | Same |
| Pub | `/stampfly/control/input` | `stampfly_msgs/ControlInput` | Same |
| Pub | `/stampfly/range/sensors` | `stampfly_msgs/RangeSensors` | Same |
| Pub | `/stampfly/flow` | `stampfly_msgs/OpticalFlow` | Same |
| Pub | `/stampfly/pose` | `geometry_msgs/PoseStamped` | Same |
| Pub | `/stampfly/velocity` | `geometry_msgs/TwistStamped` | Same |
| Pub | `/stampfly/imu` | `sensor_msgs/Imu` | Same |
| Pub | `/stampfly/range/bottom` | `sensor_msgs/Range` | Same |
| Pub | `/stampfly/range/front` | `sensor_msgs/Range` | Same |
| TF | `odom` → `base_link` | `tf2_ros.TransformBroadcaster` | When `publish_tf=true` |
| Sub | `/stampfly/cmd_vel` | `geometry_msgs/Twist` | Only when `enable_control=true`; `linear.z`→throttle, `angular.x/y/z`→roll/pitch/yaw rate |
| Srv | `/stampfly/arm` | `std_srvs/SetBool` | ARM/DISARM |

QoS: telemetry topics used `BEST_EFFORT` / `KEEP_LAST` / depth 10 (tolerating drops of high-rate data).

### Message Definitions (`stampfly_msgs`)

| Message | Fields |
|---|---|
| `ImuRaw` | `header`, `angular_velocity` (Vector3, rad/s), `linear_acceleration` (Vector3, m/s²) |
| `ImuCorrected` | `header`, `angular_velocity` (bias-corrected), `linear_acceleration` (bias-corrected) |
| `ESKFState` | `header`, `orientation` (Quaternion), `position` (Point, ENU), `velocity` (Vector3, ENU), `gyro_bias` (Vector3), `accel_bias` (Vector3), `status` (uint8) |
| `ControlInput` | `header`, `throttle` (float32, 0–1), `roll`/`pitch`/`yaw` (float32, -1 to 1) |
| `RangeSensors` | `header`, `tof_bottom`/`tof_front`/`baro_altitude` (float32, m) |
| `OpticalFlow` | `header`, `flow_x`/`flow_y` (int16), `quality` (uint8, 0–255) |

### Control Packet (UDP:8888, `udp_client.py`)

Designed to send the `ControlPacket` (16 bytes) from `firmware/common/protocol/include/udp_protocol.hpp`: `header` (fixed 0xAA) / `packet_type` (0x01) / `sequence` (uint8) / `device_id` (uint8) / `throttle`, `roll`, `pitch`, `yaw` (each uint16, 0–4095, center 2048) / `flags` (bit0=ARM, bit1=FLIP, bit2=MODE, bit3=ALT_MODE) / `reserved` / `checksum` (CRC16-CCITT). **The receiving implementation for this existed only in `firmware/vehicle_old`'s `sf_svc_udp`, and was deleted along with `vehicle_old`. The current vehicle has no corresponding receive mechanism.**

## 4. Open Questions

- Which ROS2 distribution to target (the old plan assumed Humble; reconsider the current recommended LTS at redesign time)
- Which of the §2 options, (a) or (b), to adopt (not decided in this document)
- Whether to cover telemetry subscription only, or control as well (requirements should be reconfirmed before starting)
- How to handle the wire-format stability of the DataStream (UDP 8890 binary), which has changed multiple times historically — using it as a ROS2-side contract would presuppose stabilizing it first

## 5. Reference Links (carried over from the old plan)

| Resource | URL | Description |
|---|---|---|
| micro-ROS official | https://micro.ros.org/ | Documentation and tutorials |
| ESP-IDF Component | https://github.com/micro-ROS/micro_ros_espidf_component | For ESP-IDF |
| ROS2 Humble | https://docs.ros.org/en/humble/ | ROS2 LTS documentation |
