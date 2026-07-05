# UDP テレメトリ設計記録

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 背景と問題

TCP WebSocket テレメトリ (840B × 100Hz = 83.2 KB/s) が ESP-NOW 制御信号を阻害し、
飛行中に操縦不能になった。ESP32-S3 は WiFi ラジオが1つしかなく、
テレメトリ送信中は ESP-NOW パケットを受信できない。

## 2. 失敗したアプローチと教訓

### 方式A: センサ独立パケット（sendto 265回/秒）

各センサ型を個別パケットで送信。帯域計算では問題なかったが、
**sendto() のブロック時間がパケットサイズに依存しない** ことを見逃した。

```
ベンチマーク実測:
  117B: avg=3075μs
  325B: avg=3031μs
  840B: avg=2854μs
  → サイズに関係なく ~2.5ms/回
```

265回/秒 × 2.5ms = 662ms/秒 = **ラジオ占有率 66%**。
ESP-NOW が使える時間は 34% しかなく、制御パケットがドロップした。

**教訓: sendto() のオーバヘッドは per-call であり per-byte ではない。
小さいパケットを多数送るのは最悪の戦略。**

### 制御ループ内での直接 sendto()

400Hz IMU ループ内で sendto() を呼んだところ、IMU レートが 200Hz に低下。
sendto() の 2.5ms ブロックが 2.5ms のIMUセマフォ周期を超過した。

**教訓: ブロッキング I/O をリアルタイムループ内で呼ばない。**
→ Producer-Consumer パターン（FreeRTOS キュー）で送信を分離。

### data_ready フラグ競合

imu_task と telemetry_task が同じ `volatile bool` フラグを消費。
先に読んだタスクがフラグを false にし、後のタスクはデータを見逃す。
ToF 0Hz、Flow 56Hz の原因だった。

**教訓: volatile フラグは複数のコンシューマ間で共有できない。**
→ タイムスタンプ変化検出（非破壊的）に変更。

### ToF タイムスタンプ共有

bottom と front の ToF が同じ `g_tof_last_timestamp_us` を更新。
電池駆動時に front ToF が初期化成功し、1ループで2回更新 → 2倍のレートに見えた。

**教訓: 1データソース = 1変数。共有しない。**
→ `g_tof_bottom_last_timestamp_us` / `g_tof_front_last_timestamp_us` に分離。

### スタックオーバーフロー（複数回）

WiFi CLI タスクのスタック 4096B に対して SendItem (342B) + ws_batch_pkt (840B) をスタックに確保。

**教訓: 組込みタスクのスタックは小さい。大きな構造体は module-static にする。**

## 3. 最終解決: 統合パケット方式

8 IMU サイクル分のデータを 1 パケット (~1000B) にまとめて 50Hz で送信。

```
パケット構造:
  [Header 4B]
  [IMU+ESKF × 8]   640B (固定)
  [PosVel × 8]     224B (固定)
  [RateRef × 8]     48B (固定)
  [entry_count 1B]
  [SensorEntries]   可変 (Control, Flow, ToF, Baro, Mag)
  [Checksum 1B]
  合計: ~920-1000B
```

### 結果

| 指標 | TCP(旧) | UDP方式A | UDP統合(最終) |
|------|---------|---------|-------------|
| sendto() | 100/秒 | 272/秒 | **50/秒** |
| ラジオ占有 | >70% | 68% | **12.5%** |
| パケットロス | 19% | 5% | **0%** |
| ESP-NOW操縦 | 不可 | 困難 | **正常** |

### 核心的知見

**ESP32 の lwIP sendto() は1回あたり ~2.5ms の固定オーバヘッドを持つ。**
ペイロードサイズ（117B〜950B）による差はない。
ESP-NOW との共存品質を決めるのはデータ量ではなく **sendto() の呼び出し回数**。

## 4. 関連ファイル

> **注記:** 本設計は当初の旧 `firmware/vehicle`（現 `firmware/vehicle_old`、凍結・新規開発なし）で確立された。ファームウェア昇格リファクタ（`vehicle_new` → `vehicle`）後は、電文互換の「Data Stream」として `firmware/vehicle/components/sf_telemetry/` 配下に再実装されており、`sf log wifi` / `sf log viz` は無改造で動作する。

| ファイル | 役割 |
|---------|------|
| `firmware/vehicle/components/sf_telemetry/include/data_stream_wire.hpp` | パケット構造体定義（`WireHeader`/`WireImuEskf`/`WirePosVel` 等、統合パケット 0x50） |
| `firmware/vehicle/components/sf_telemetry/data_stream.cpp` | `sf::DataStream` 実装（UDPログサーバ相当） |
| `firmware/vehicle/tasks/telemetry_task.cpp` | 統合パケット構築・送信（`TelemetryTask` が `sf::DataStream` を所有） |
| `tools/log_analyzer/udp_capture.py` | Python UDP 受信・統計・JSONL保存 |
| `tools/log_analyzer/visualize_jsonl.py` | 静的一覧ビューワー |
| `tools/log_analyzer/visualize_interactive.py` | インタラクティブビューワー |

---

<a id="english"></a>

## 1. Background

TCP WebSocket telemetry (840B × 100Hz) blocked ESP-NOW control,
making flight impossible. ESP32-S3 has a single WiFi radio shared
between telemetry and ESP-NOW.

## 2. Failed Approaches and Lessons

### Method A: Individual sensor packets (sendto 265/sec)

Bandwidth calculations showed feasibility, but missed the key fact:
**sendto() block time is independent of packet size** (~2.5ms/call
whether 117B or 840B).

265 calls/sec × 2.5ms = 66% radio occupancy → ESP-NOW starved.

**Lesson: sendto() overhead is per-call, not per-byte.**

### Direct sendto() in control loop

Calling sendto() in the 400Hz IMU loop halved the rate to 200Hz.

**Lesson: Never call blocking I/O in a real-time loop.**
→ Fixed with producer-consumer (FreeRTOS queue).

### Shared data_ready flags

imu_task and telemetry_task consumed the same volatile bool.
First reader wins, second misses data.

**Lesson: Volatile flags cannot be shared between consumers.**
→ Fixed with timestamp-change detection.

### Shared ToF timestamp

Bottom and front ToF updated the same variable → 2× detected rate.

**Lesson: One variable per data source.**

### Stack overflows

Large structures on 4KB task stacks caused reboots.

**Lesson: All large structures must be module-static.**

## 3. Final Solution: Unified Packet

8 IMU cycles packed into one ~1000B packet at 50Hz.

| Metric | TCP | UDP-A | UDP Unified |
|--------|-----|-------|-------------|
| sendto() | 100/s | 272/s | **50/s** |
| Radio use | >70% | 68% | **12.5%** |
| Loss | 19% | 5% | **0%** |
| Control | impossible | difficult | **normal** |

**Key insight:** ESP32 lwIP sendto() has ~2.5ms fixed overhead per call.
Minimizing call count determines ESP-NOW coexistence quality.
