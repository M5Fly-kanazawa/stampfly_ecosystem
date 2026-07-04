# vehicle_new Topic Reference
# vehicle_new トピックリファレンス

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

### このドキュメントについて

vehicle_new の **Pub-Sub Topic システムの Single Source of Truth (SSOT)** 文書である。学習者・利用者・拡張者が Topic を使うために必要な情報を **この 1 文書で完結** させる。

### 対象読者

- L0 学習者（Workshop 受講者）— `ws::*` API の裏で何が起きているかを理解したい時の参考
- **L1 学習者（推定・制御・ガイダンス）** — メインターゲット。自分のロジックを書くために Topic を subscribe / publish する
- L2 / L3 学習者・実装者 — Topic が他のレイヤーとどう接続するかを把握する

### Topic SSOT ルール（R9）

- 新規 Topic を追加するときは、本文書の表 (§3) を **同じ PR で必ず更新** する
- Topic 名・データ型・rate・publisher / subscriber を変更したときも本文書を同期
- Topic を**削除**する場合は、本文書から削除し、変更履歴を §8 に記録する

### 関連文書

| 文書 | 役割 |
|------|------|
| [`architecture.md`](architecture.md) | 階層構造・コンポーネント・横断ルール |
| [`detailed_design.md`](detailed_design.md) §2 | Topic 実装方針（`extern` 宣言 + 実体定義）、API、追加手順 |
| [`hardware_init.md`](hardware_init.md) | BSP 層・HW 初期化（`sensor_health` の publisher 側） |
| **本文書** | **Topic 一覧の SSOT、使用パターン、ハマりポイント** |

---

## 2. 3 つのバッファ方式

vehicle_new の Topic はテンプレートで 3 つのバッファ方式から選ぶ。データの特性に合わせて使い分ける。

```cpp
Topic<DataType, BufferPolicy, BufferSize>  topic_name;
```

### 2.1 Latest — 最新値キャッシュ

| 項目 | 内容 |
|-----|-----|
| 実装 | `xSemaphoreTake` / `xSemaphoreGive` による mutex 保護 |
| 容量 | 常に最新 1 個のみ（古い値は上書き） |
| ISR 安全性 | ❌ NG（mutex は ISR 禁止） |
| 複数 subscriber | ✅ OK（latest() がコピーを返す） |
| 用途 | 状態推定値、制御出力、コマンド設定値、システムモード |
| 損失 | 損失許容（古い値は失われる） |

### 2.2 RingBuffer — ロックフリーリング

| 項目 | 内容 |
|-----|-----|
| 実装 | `std::atomic<uint32_t>` による head / tail、`memory_order_acquire/release` |
| 容量 | テンプレート指定（**2 のべき乗必須**、ビットマスクで高速化） |
| ISR 安全性 | ✅ OK（ロックフリー atomic） |
| 複数 subscriber | ❌ **SPSC（Single Producer / Single Consumer）専用** |
| 用途 | 高レート連続データ（IMU 400Hz） |
| 損失 | 満杯時は最古を上書き（FIFO） |

### 2.3 Queue — FreeRTOS Queue

| 項目 | 内容 |
|-----|-----|
| 実装 | `xQueueSend` / `xQueueReceive` |
| 容量 | テンプレート指定 |
| ISR 安全性 | ✅ OK（`xQueueSendFromISR` 利用可） |
| 複数 subscriber | ❌ NG（read で消費される） |
| タイムアウト | ✅ `read(out, ticks)` で待機可 |
| 用途 | 低レートセンサ（ToF / Flow / Mag / Baro）、システムアラート |
| 損失 | 満杯時は publish ドロップ（**サイレント**） |

### 2.4 選択フローチャート

```
　Topic 設計
　　│
　　├─ 高レート (>100Hz) で全サンプル必要 ──→ RingBuffer (size = 2 のべき乗)
　　│
　　├─ 低レートでイベント駆動 ─────────→ Queue (size = ~2-4)
　　│
　　└─ 最新値だけ必要、複数 reader 想定 ──→ Latest (size = 1)
```

---

## 3. Topic 一覧表

### 3.1 定義済み Topic（実体定義済み）

| # | Topic 名 | データ型 | バッファ | サイズ | Publisher | Subscriber | レート | 用途 |
|---|---------|---------|--------|------|-----------|-----------|------|----|
| 1 | `sensor_imu` | `ImuData` | RingBuffer | 8 | ImuTask | ImuTask 内 estimator | 400Hz | IMU 加速度+ジャイロ（生 SPI 読み取り後の変換済み） |
| 2 | `sensor_tof` | `TofData` | Queue | 2 | TofTask | ImuTask::processAsyncSensors | 30Hz | ToF 距離（底面） |
| 3 | `sensor_flow` | `FlowData` | Queue | 2 | FlowTask | ImuTask::processAsyncSensors | 100Hz | OptFlow 速度 |
| 4 | `sensor_mag` | `MagData` | Queue | 2 | MagTask | ImuTask::processAsyncSensors | 25Hz | 地磁気 |
| 5 | `sensor_baro` | `BaroData` | Queue | 2 | BaroTask | ImuTask::processAsyncSensors | 50Hz | 気圧・高度 |
| 6 | `sensor_power` | `PowerData` | Latest | 1 | PowerTask | TelemetryTask, FailsafeTask | 10Hz | 電源・電流 |
| 7 | `estimate_state` | `StateEstimate` | Latest | 1 | ImuTask | ControlTask, TelemetryTask | 400Hz | 姿勢・位置・速度 |
| 8 | `command_setpoint` | `CommandSetpoint` | Latest | 1 | CommTask (ESP-NOW recv) | ControlTask | 50Hz | パイロット指令 |
| 9 | `control_output` | `ControlOutput` | Latest | 1 | ControlTask | TelemetryTask | 400Hz | 推力・トルク出力 |
| 9b | `controller_status` | `ControllerStatus` | Latest | 1 | ControlTask | ApiTask | 400Hz | guidance_active の事実（誘導解除を API へ同期, M-3） |
| 10 | `actuator_motor` | `MotorOutput` | Latest | 1 | ControlTask | (motor driver) | 400Hz | モータ duty |
| 11 | `system_mode` | `SystemMode` | Latest | 1 | StateTask | ControlTask, NotifyTask | event | ARM 状態・フライトモード |
| 12 | `system_alert` | `SystemAlert` | Queue | 4 | FailsafeTask | TelemetryTask, NotifyTask | event | 警告・エラー |
| 13 | `pilot_request` | `PilotRequest` | Latest | 1 | CommTask | StateTask | 50Hz | ARM + フライトモード選択（sf_comm → StateTask） |
| 14 | `system_status` | `SystemStatus` | Latest | 1 | ImuTask | StateManager(pre-arm), StateTask | 400Hz | 起動準備（calibrated）+ airborne + held |
| 15 | `estimator_command` | `EstimatorCommand` | Queue | 4 | StateManager callbacks | ImuTask | event | 推定器 reset / 位置速度reset / bias freeze / recalibrate 指令（onEnter/onExit 集約） |
| 16 | `controller_command` | `ControllerCommand` | Queue | 4 | StateManager callbacks | ControlTask | event | 制御器 reset 指令（onEnter/onExit 集約） |
| 17 | `notify_command` | `NotifyCommand` | Queue | 8 | StateManager / Failsafe | NotifyTask | event | LED/ブザー通知指令（arm/disarm 音等。配線は Phase 6） |
| 18 | `sensor_health` | `SensorHealth` | Latest | 1 | sf_board | TelemetryTask, FailsafeTask | 1Hz | センサ presence / 鮮度（R15。publish 配線は Phase 6） |
| 19 | `pairing_state` | `PairingStatus` | Latest | 1 | StateTask (StateManager) | CommTask, NotifyTask | event | 現在の PairingState（NotPaired/Pairing/Paired）。comm が送出制御、notify が LED/ブザー表示 |
| 20 | `pairing_complete` | `PairingComplete` | Latest | 1 | CommTask (ESP-NOW recv / NVS load) | StateTask | event | comm の現在のバインド状態（bound + 学習/復元した送信機 MAC）。起動時の NVS 復元と Pairing 成立の両方を運ぶ |
| 21 | `sensor_snapshot` | `SensorSnapshot` | Latest | 1 | ImuTask (processAsyncSensors) | CLI (`sensor`), Telemetry | 400Hz | mag/baro/tof/flow の最新生値ミラー（SPSC キューを奪わず監視できるよう ImuTask が複製）|
| 22 | `ui_command` | `UiCommand` | Queue | 4 | CLI (`sound`/`led`) | NotifyTask | event | UI 設定指令（ブザー mute / LED 輝度）。将来 WiFi/UDP からも注入可 |
| 23 | `motor_test` | `MotorTest` | Latest | 1 | CLI (`motor`) | ControlTask | event | ベンチ用モータ単体テスト（**disarmed 限定**、active/motor_id/duty/expiry_us、既定 inactive）|

### 3.2 予約 Topic（実体定義済み・producer 未配線、または未定義）

vehicle_new v3 設計で予約した将来 Topic。`command_target` / `nav_path` は実体を定義済み（producer は将来配線）、`sensor_imu_raw` は未定義（型 `ImuRawData` も未定義）。

| # | Topic 名 | データ型 | バッファ | サイズ | Publisher | Subscriber | レート | 用途 | 状態 |
|---|---------|---------|--------|------|-----------|-----------|------|------|------|
| 24 | `command_target` | `GuidanceTarget` | Latest | 1 | (Navigator / Guidance) | ControlTask, NotifyTask | 10Hz | 位置 + yaw target、ウェイポイント | 実体定義済 (M4+ 配線) |
| 25 | `nav_path` | `NavigationPath` | Queue | 4 | (Navigator) | (Guidance) | 1Hz | 経路シーケンス | 実体定義済 (Phase 6 配線) |
| 26 | `sensor_imu_raw` | `ImuRawData` | RingBuffer | 8 | ImuTask | (学習者・SIL 検証) | 400Hz | キャリブ前の生 IMU。教育用、L2 学習者向け | 未定義 (M2) |

**新規 Topic の根拠（横断ルール対応）:**
- `sensor_imu_raw` — L2 学習者が「キャリブ前の生 IMU を見たい」「自分でキャリブを学びたい」シナリオに対応
- `sensor_health` — R15: publisher 死活監視を Topic 経由で構造化、旧 vehicle/ の `volatile bool g_*_healthy` を排除
- `command_target` / `nav_path` — R11: Guidance / Navigation 層の置き場所と入出力契約を今のうちに確定

### 3.3 データ型定義の場所

各 Topic のデータ型は `components/sf_core/include/data_types.hpp` で一元定義。
詳細は [`detailed_design.md`](detailed_design.md) §2「データ型定義」を参照。

---

## 4. 使用パターン（学習者が真似する最小コード）

### 4.1 パターン A — 最新値だけ欲しい（典型: 制御学習者）

```cpp
// ControlTask: 400Hz で最新の状態とコマンドを読む
void ControlTask(void* pvParameters) {
    while (true) {
        // ImuTask からの通知で起こされる（IMU 同期）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 最新値のスナップショットを取得（複数回呼んでも安全）
        sf::StateEstimate state = sf::estimate_state.latest();
        sf::CommandSetpoint cmd  = sf::command_setpoint.latest();

        // 制御計算
        sf::ControlOutput ctrl = controller.compute(state, cmd, dt);
        sf::control_output.publish(ctrl);
    }
}
```

### 4.2 パターン B — 全サンプルを順に処理（典型: 推定学習者）

```cpp
// ImuTask: RingBuffer に溜まった全 IMU サンプルを処理
sf::ImuData imu;
while (sf::sensor_imu.read(imu)) {
    estimator.predict(imu, dt);   // 1 サンプルずつ予測ステップ
}

// Queue 系も同じパターンで使える
sf::TofData tof;
while (sf::sensor_tof.read(tof)) {
    estimator.updateTof(tof);    // 観測更新
}
```

### 4.3 パターン C — Publisher 側（典型: HW 学習者）

```cpp
// TofTask: 30Hz で ToF を読んで publish
void TofTask(void* pvParameters) {
    auto cfg = stampfly::VL53L3CXWrapper::Config::defaultBottom(
        sf::internal::board::i2c_bus()
    );
    static stampfly::VL53L3CXWrapper tof;
    tof.init(cfg);
    tof.startRanging();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // 30Hz

    while (true) {
        bool ready = false;
        tof.isDataReady(ready);
        if (ready) {
            stampfly::DistanceData d;
            if (tof.getDistance(d) == ESP_OK && d.range_status == 0) {
                sf::TofData out;
                out.timestamp = esp_timer_get_time();
                out.distance_m = d.distance_mm * 1e-3f;
                sf::sensor_tof.publish(out);  // ← Topic に流す
            }
            tof.clearInterruptAndStartMeasurement();
        }
        vTaskDelayUntil(&last_wake, period);
    }
}
```

---

## 5. 運用上のハマりポイント

### 5.1 Subscribe するタイミング

| 質問 | 回答 |
|-----|------|
| Topic を subscribe する前にタスクを起動しても大丈夫？ | **OK**。`sf::topics_init()` を `app_main()` の Phase 2 で先に呼んでバッファを確保する。タスク起動順は不問 |

### 5.2 Buffer overflow

| 方式 | 満杯時の挙動 | 検知方法 |
|-----|-----------|--------|
| Latest | 古い値を上書き（常に最新 1 つ） | overflow という概念なし |
| RingBuffer | **最古サンプルを上書き** | サイレント（log なし） — 課題、§6 で改善計画 |
| Queue | **publish がドロップ** | サイレント — 課題、§6 で改善計画 |

→ 現状はサイレント喪失の検知手段がない。R14 で `overflow_count` 内蔵を予定。

### 5.3 Publisher が止まったときの検知

Topic は「鮮度（age）」を持たない。Subscriber 側でタイムスタンプ判定が必要。

```cpp
// CommandSetpoint には timestamp [μs] が含まれる
auto cmd = sf::command_setpoint.latest();
uint32_t now = esp_timer_get_time();
if (now - cmd.timestamp > 500'000) {  // 500ms 以上古い
    // Failsafe 発動 (R16)
    sf::system_alert.publish(SystemAlert{AlertType::COMM_TIMEOUT, now});
}
```

### 5.4 1 つの Topic に複数 Subscriber

| 方式 | 複数 subscriber | 注意点 |
|-----|--------------|------|
| Latest | ✅ OK | 各 subscriber が独立コピーを取得（mutex で保護） |
| RingBuffer | ❌ NG | SPSC 専用。複数 reader は **動作未定義**。必要なら Latest 並置か、別 Topic で再 publish |
| Queue | ❌ NG | `read` が値を消費するため、複数 reader 不可 |

**実例**: `estimate_state` は ControlTask と TelemetryTask の両方が `latest()` で読んでいる。`sensor_imu` は ImuTask 内 estimator のみが `read()` する（他から触らない）。

### 5.5 ISR からの publish

| 方式 | ISR 安全 | 備考 |
|-----|--------|----|
| Latest | ❌ | mutex を ISR 内で取れない |
| RingBuffer | ✅ | atomic のみ、追加コードなしで OK |
| Queue | ✅ | ただし `xQueueSendFromISR` を呼ぶ必要あり |

**実装例**: ボタン GPIO 割り込みハンドラから `button_event` Topic（Queue）に publish するなど。

---

## 6. 設計上の課題と改善計画

### 6.1 サイレント overflow（R14）

**実装済み（R14 達成）**: 全 Topic テンプレート（`TopicRing` / `TopicQueue`）に `std::atomic<uint32_t> overflow_count_` を内蔵し、満杯時のドロップ（RingBuffer の最古上書き、Queue のドロップ）を計上する。`overflowCount()` getter で参照可能（`TopicLatest` は上書きが仕様ゆえ常に 0 を返す＝監視側の統一アクセス用）。telemetry / sensor_health 経由の監視出力への接続は Phase 6 で行う。

```cpp
template<typename T, int Size>
class TopicQueue {
public:
    void publish(const T& d) {
        if (xQueueSend(queue_, &d, 0) != pdTRUE) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    uint32_t overflow_count() const { return overflow_count_.load(); }
private:
    std::atomic<uint32_t> overflow_count_{0};
};
```

### 6.2 Publisher / Subscriber 関係の検証手段（R13）

**現状**: 「タスク A が publish、B が subscribe」を宣言する仕組みがなく、コメントと本文書 §3 表だけが頼り。

**改善計画**: 各 task ヘッダ（`tasks/*.cpp`）の冒頭に `@publisher` / `@subscriber` アノテーションを必須化。

```cpp
/**
 * @file imu_task.cpp
 * @publisher  sensor_imu, estimate_state
 * @subscriber sensor_tof, sensor_flow, sensor_mag, sensor_baro
 * @design architecture.md §6 — ImuTask
 */
```

将来 lint で「宣言 ↔ 実コードの一致」を検証する余地を残す。

### 6.3 RingBuffer の SPSC 制限

**現状**: `sensor_imu` を複数 task から `read()` すると未定義動作。

**回避策**: 必要が出たら、対象 Topic を Latest 化するか、派生 Topic（例: `sensor_imu_decimated_50hz`）を別 publisher が再発行する。

### 6.4 Publisher 死活監視（R15）

**現状**: Topic 自体は鮮度を持たない。

**改善計画**: `sensor_health` Topic を `sf_board` が 1Hz publish。各 publisher の presence / last_update_us / quality を集約。Failsafe / Telemetry がここを subscribe して死活を判定。

```cpp
struct SensorHealth {
    uint32_t timestamp;
    struct Item {
        bool present;
        uint32_t last_update_us;
        float quality;  // 0.0 (bad) - 1.0 (good)
    };
    Item imu, tof, flow, mag, baro, power, comm;
};
```

---

## 7. Topic 追加時の手順（PR チェックリスト）

新しい Topic を追加するときは以下を全て満たすこと。

- [ ] `data_types.hpp` にデータ構造を追加（`timestamp` フィールドを必ず含める）
- [ ] `topics.hpp` に `extern Topic<>` 宣言を 1 行追加
- [ ] `topics.cpp`（または `params.cpp` 中の topic 実体定義）に対応する実体定義を 1 行追加
- [ ] **本文書 §3 の表に 1 行追加**（Topic 名 / データ型 / バッファ方式 / サイズ / publisher / subscriber / レート / 用途）
- [ ] Publisher 側の task ヘッダに `@publisher <topic_name>` を追加（R13）
- [ ] Subscriber 側の task ヘッダに `@subscriber <topic_name>` を追加
- [ ] レビュアーは「同じ PR 内で本文書が更新されているか」を確認
- [ ] バッファ方式の選定根拠を PR description に記載（§2.4 のどのケースに該当するか）
- [ ] レート要件と overflow 時の挙動を確認（特に Queue サイズの根拠）

---

## 8. 変更履歴（Topic の追加 / 変更 / 削除）

新規 Topic の追加、データ型の変更、削除があったときに記録する。

| 日付 | 変更 | Topic | 理由 / PR |
|------|------|-------|---------|
| 2026-04-12 | 追加 | sensor_imu, sensor_tof, sensor_flow, sensor_mag, sensor_baro, sensor_power, estimate_state, command_setpoint, control_output, actuator_motor, system_mode, system_alert | 初期実装（12 Topic） |
| 2026-05-09 | 予約 | sensor_imu_raw, sensor_health, command_target, nav_path | v3 設計で予約定義（M1b） |
| 2026-06-07 | 追加 | estimator_command, controller_command, notify_command | 設計準拠リファクタ Phase 0 — reset を onEnter/onExit に集約するための指令チャネル（R5） |
| 2026-06-07 | 同期 | pilot_request, system_status | 既存だが §3.1 表に未記載だったため追記（R9 同期） |
| 2026-06-07 | 実体定義 | sensor_health（予約→実体, publish は Phase 6）, command_target, nav_path | 予約 Topic の実体を定義（producer は将来配線） |
| 2026-06-07 | 全 Topic | overflow_count 内蔵（R14） | TopicRing/TopicQueue に overflow_count_ + getter を追加 |
| 2026-06-14 | 追加 | controller_status | 誘導解除（パイロット介入/モード変更）を API へ同期する guidance_active 事実（ControlTask → ApiTask, code-review M-3） |

---

<a id="english"></a>

## 1. About This Document

> **Status:** Topic system Single Source of Truth (SSOT). The Japanese section above is the authoritative version. Full English translation pending in M1c.

This document is the SSOT reference for the vehicle_new Pub-Sub Topic system. It covers:

- The 3 buffer policies (Latest / RingBuffer / Queue) and selection criteria
- Complete topic catalog (12 implemented + 4 reserved in v3)
- Usage patterns with minimal code examples
- Operational pitfalls (subscribe timing, overflow, publisher liveness, multi-subscriber, ISR usage)
- Design issues and improvement plans (R13–R16)
- Procedure for adding a new topic (PR checklist)

See the Japanese section above for full content.
