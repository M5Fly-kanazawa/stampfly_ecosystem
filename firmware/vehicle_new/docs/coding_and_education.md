# vehicle_new Coding Policy and Education Plan
# vehicle_new コーディング方針・教育計画

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

**本文書はvehicle_newの全実装に適用される必須ルールである。**

vehicle_newのコードは単なるファームウェアではなく、ドローンのファームウェアを作ろうとする人が参考にできる**模範的なソースコード**であること、また**学習教材として機能する**ことを最重要目標とする。

### 基本方針

- ドローンファームを作ろうとする人が参考にできる可読性と簡潔さを兼ね備える
- 独自ファームに改造したり学習するためのExampleを豊富に用意する
- チュートリアル作成・ワークショップ展開を視野に入れる

## 2. コーディング規約

### 可読性ルール

| ルール | 内容 |
|--------|------|
| **1関数1責務** | 1つの関数は1つのことだけやる。50行以内を目安 |
| **バイリンガルコメント** | 英語 → 日本語の順で全関数・全ブロックにコメント |
| **関数冒頭ドキュメント** | 何をするか・なぜ必要かを3〜5行で説明 |
| **マジックナンバー禁止** | 全ての数値にconfig定数名またはパラメータ名をつける |
| **略語禁止** | `s`, `p`, `r` ではなく `state`, `params`, `roll` |
| **ネスト2段まで** | 深いif/forは早期returnか関数分割で解消 |
| **Pub-Subで分離** | コンポーネント間の直接呼び出しを禁止 |
| **@designタグ必須** | クラス・インターフェース・状態遷移の実装に設計文書の参照を記載 |
| **設計矛盾の即時報告** | 実装中に設計文書との矛盾・不都合を発見したら、実装を進めず報告・議論する |

### @designタグ（設計トレーサビリティ）

クラス定義、インターフェース実装、状態遷移コールバックには `@design` タグで設計文書の該当箇所を参照すること。内部ヘルパーや自明な実装には不要。

```cpp
/// Compute control output from state estimate and setpoint
/// 推定値とセットポイントから制御出力を計算
///
/// @design architecture.md §4 — IController interface definition
/// @design detailed_design.md §4 — Control interface: compute/reset/onModeChange
/// @design requirements.md §4 — Component #6: replaceable control
///
ControlOutput PidController::compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt)
{
    // ...
}
```

```cpp
/// State transition: FLYING → IDLE_GROUND (crash or pilot DISARM)
/// 状態遷移: FLYING → IDLE_GROUND（衝突検知 or パイロットDISARM）
///
/// @design architecture.md §4 — FAILSAFE as event, State Mgr transitions
/// @design detailed_design.md §3 — onExit/onEnter callback table
/// @design requirements.md §2 — Transition: FLYING → IDLE_GROUND
///
void StateManager::onEnterIdleGround()
{
    motor.stop();           // @design detailed_design.md §3 — モーター停止
    eskf.reset();           // @design detailed_design.md §3 — ESKFリセット
    buzzer.play(DISARM);    // @design detailed_design.md §3 — ブザー(disarm音)
}
```

### 設計矛盾の発見時の対応

実装中に設計文書（requirements.md / architecture.md / detailed_design.md）との矛盾や不都合が明らかになった場合：

1. **実装を進めずに立ち止まる** — 矛盾を抱えたまま実装しない
2. **矛盾の内容を具体的に報告する** — どの設計文書のどの項目と、実装上の何が矛盾するか
3. **議論して設計を更新する** — 設計変更が必要なら文書を更新してからコードに反映する
4. **変更履歴を残す** — コミットメッセージに設計変更の理由を記載する

### コードスタイル例

```cpp
// ============================================================
// Bad: 避けるべきコード
// ============================================================
void ControlTask(void* p) {
    auto& s = stampfly::StampFlyState::getInstance();
    while(true) {
        if (s.getFlightState() == stampfly::FlightState::ARMED ||
            s.getFlightState() == stampfly::FlightState::FLYING) {
            float r = g_rate_pid_roll.compute(
                g_cmd.roll * config::rate_control::ROLL_RATE_MAX -
                g_state.gyro[0], config::IMU_DT);
            // ... 100行の密結合コード
        }
    }
}

// ============================================================
// Good: 目指すコード
// ============================================================

/// Control task — runs at 400Hz, synchronized with IMU
/// 制御タスク — 400Hz、IMU同期で動作
///
/// Reads the latest state estimate and command setpoint,
/// computes thrust/torque via the active controller,
/// and publishes the result for actuation.
///
/// 最新の推定値とコマンドセットポイントを読み取り、
/// アクティブなコントローラで推力/トルクを計算し、
/// アクチュエーションに向けて発行する。
///
void ControlTask(void* pvParameters)
{
    // Wait for IMU sync notification
    // IMU同期通知を待つ
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Read inputs from topics
    // トピックから入力を読む
    auto state = estimate_state.latest();
    auto setpoint = command_setpoint.latest();

    // Compute control output
    // 制御出力を計算
    auto output = controller->compute(state, setpoint, dt);

    // Publish for actuation and logging
    // アクチュエーションとログに向けて発行
    control_output.publish(output);
}
```

## 3. Examples（サンプル集）計画

### 設計原則

各Exampleは：
- **単独でビルド・実行可能**（vehicle_new全体のビルド不要）
- **README.mdに完全な説明**（目的、必要な知識、接続図、手順、コードの解説）
- **段階的に複雑度が上がる**（前のExampleの知識を前提に）
- **コメントは本体より多くてもいい**
- **「ここを変えてみよう」セクションで改造の余地を示す**

### Example構成

```
examples/01_blink_led/
├── CMakeLists.txt          # 単独ビルド可能
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp            # コメント豊富
└── README.md               # 目的、接続図、実行手順、解説
```

### Level 1: ハードウェア基礎（ESP-IDFとセンサ）

| Example | 学べること | 行数目安 |
|---------|-----------|---------|
| `01_blink_led` | ESP-IDF基礎、GPIO、WS2812制御 | ~30行 |
| `02_buzzer_melody` | LEDC PWM、トーン生成 | ~50行 |
| `03_button_event` | GPIO入力、デバウンス、イベント処理 | ~50行 |
| `04_read_imu` | SPI通信、BMI270ドライバ、センサデータ表示 | ~60行 |
| `05_read_tof` | I2C通信、VL53L3CX、距離計測 | ~50行 |
| `06_read_baro` | I2C通信、BMP280、気圧→高度変換 | ~50行 |
| `07_motor_spin` | LEDC PWM、モーター単体制御、安全停止 | ~40行 |
| `08_battery_monitor` | INA3221、電圧/電流読み取り | ~40行 |

### Level 2: 通信と制御の基礎

| Example | 学べること | 行数目安 |
|---------|-----------|---------|
| `09_espnow_pair` | ESP-NOW通信、ペアリング、パケット送受信 | ~80行 |
| `10_udp_telemetry` | WiFi AP、UDP送信、PCでの受信 | ~80行 |
| `11_pid_single_axis` | PID制御の基本、1軸モーター制御 | ~100行 |
| `12_complementary_filter` | 加速度+ジャイロ、相補フィルタで姿勢推定 | ~80行 |
| `13_parameter_tuning` | パラメータシステム、WiFi経由でPIDゲイン変更 | ~100行 |

### Level 3: フライトシステム

| Example | 学べること | 行数目安 |
|---------|-----------|---------|
| `14_attitude_estimation` | ESKF基礎、IMUから姿勢推定 | ~120行 |
| `15_rate_control` | レート制御、4モーターミキシング | ~120行 |
| `16_stabilize_flight` | 姿勢安定化飛行の最小構成 | ~150行 |
| `17_altitude_hold` | ToF + 高度制御PID | ~150行 |
| `18_position_hold` | OptFlow + 位置制御PID | ~150行 |
| `19_state_machine` | 状態管理の実装パターン | ~100行 |
| `20_pubsub_basics` | Pub-Subの使い方、トピック追加 | ~80行 |

### Level 4: 応用・拡張

| Example | 学べること | 行数目安 |
|---------|-----------|---------|
| `21_custom_controller` | IControllerを実装して独自制御を試す | ~100行 |
| `22_custom_estimator` | IEstimatorを実装して独自推定を試す | ~120行 |
| `23_tello_api` | TelloSDK互換コマンドで制御 | ~100行 |
| `24_ros2_bridge` | ROS2トピックとの連携 | ~100行 |
| `25_blackbox_analysis` | Blackboxログの取得と解析 | ~80行 |

## 4. チュートリアル計画

### チュートリアル構成（docs/tutorial/）

| Chapter | タイトル | 対応Example | 所要時間 |
|---------|---------|------------|---------|
| Ch.1 | StampFlyを光らせよう | 01-03 | 30分 |
| Ch.2 | センサを読んでみよう | 04-06, 08 | 45分 |
| Ch.3 | モーターを回そう | 07 | 20分 |
| Ch.4 | コントローラと通信しよう | 09-10 | 30分 |
| Ch.5 | PID制御を理解しよう | 11 | 45分 |
| Ch.6 | 姿勢を推定しよう | 12, 14 | 60分 |
| Ch.7 | 初めてのフライト | 15-16 | 60分 |
| Ch.8 | 高度を維持しよう | 17 | 45分 |
| Ch.9 | 位置を保持しよう | 18 | 45分 |
| Ch.10 | 自分だけのコントローラを作ろう | 21-22 | 60分 |

### ワークショップ向けの配慮

| 配慮 | 具体策 |
|------|--------|
| 環境構築の簡略化 | `sf doctor` で環境チェック、問題を自動診断 |
| つまずきポイントの先回り | READMEに「よくあるエラーと対処」セクション |
| 段階的な成功体験 | LEDが光る → センサ値が見える → モーターが回る → 飛ぶ |
| コピペで動く | 各Exampleはコピペして即ビルド可能 |
| 改造の余地を示す | 「ここを変えてみよう」セクション |

## 5. 実装の優先順位

| 優先度 | 内容 |
|--------|------|
| 1 | sf_core（Pub-Sub、データ型）— 全ての基盤 |
| 2 | sf_state（状態管理）— 再構築の核心 |
| 3 | HALコピー + Level 1 Examples（01-08）— 即動くものを先に |
| 4 | メインパイプライン（推定→制御→アクチュエーション） |
| 5 | Level 2-3 Examples（09-20）— フライト関連 |
| 6 | 通信・テレメトリ・ログ |
| 7 | Level 4 Examples（21-25）— 応用 |
| 8 | チュートリアルドキュメント |
| 9 | README.md（最後） |

---

<a id="english"></a>

## 1. Purpose of This Document

**This document applies as mandatory rules to all vehicle_new implementation.**

vehicle_new code is not just firmware — it must serve as **exemplary source code** that people building drone firmware can reference, and it must function as **educational material**.

### Core Policy

- Combine readability and simplicity that drone firmware developers can reference
- Provide abundant Examples for customization and learning
- Keep tutorial creation and workshop deployment in scope

## 2. Coding Standards

### Readability Rules

| Rule | Description |
|------|-------------|
| **One function, one responsibility** | Each function does one thing. Target under 50 lines |
| **Bilingual comments** | English first, then Japanese, on all functions and blocks |
| **Function header documentation** | 3-5 lines explaining what and why |
| **No magic numbers** | All values get a config constant or parameter name |
| **No abbreviations** | `state`, `params`, `roll` — not `s`, `p`, `r` |
| **Max 2 levels of nesting** | Use early returns or function extraction |
| **Pub-Sub separation** | No direct cross-component calls |
| **@design tag required** | Reference design docs on class/interface/state transition implementations |
| **Report design conflicts** | If implementation reveals conflicts with design docs, stop and discuss before proceeding |

## 3. Examples Plan

### Design Principles

Each Example must be:
- **Independently buildable** (no need to build full vehicle_new)
- **Fully documented with README.md** (purpose, wiring, steps, explanation)
- **Progressive in complexity** (builds on previous Examples)
- **More comments than code is OK**
- **Show modification opportunities** ("Try changing this" section)

### Level 1: Hardware Basics (8 examples)
01_blink_led through 08_battery_monitor

### Level 2: Communication and Control Basics (5 examples)
09_espnow_pair through 13_parameter_tuning

### Level 3: Flight Systems (7 examples)
14_attitude_estimation through 20_pubsub_basics

### Level 4: Advanced/Extension (5 examples)
21_custom_controller through 25_blackbox_analysis

## 4. Tutorial Plan

10 chapters from "Make StampFly Light Up" to "Build Your Own Controller", corresponding to Examples, with estimated 30-60 minutes per chapter.

## 5. Implementation Priority

1. sf_core → 2. sf_state → 3. HAL + Level 1 Examples → 4. Main pipeline → 5. Level 2-3 Examples → 6. Communication/Telemetry/Log → 7. Level 4 Examples → 8. Tutorials → 9. README.md (last)
