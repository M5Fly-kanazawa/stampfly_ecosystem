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

### @designタグ（設計トレーサビリティ）— 本プロジェクトの特徴

**設計と実装の対応をコード上で可視化する。** クラス定義、インターフェース実装、状態遷移コールバックには `@design` タグで設計文書の該当箇所を参照し、判定ステータスを付記すること。内部ヘルパーや自明な実装には不要。

#### 判定ステータス

| ステータス | 意味 |
|-----------|------|
| `[OK]` | 設計通りに実装済み・確認済み |
| `[NG]` | 未実装または設計と乖離あり（理由を併記） |
| `[--]` | 未チェック（実装直後、レビュー前） |

**リリース時点で全ての@designタグが `[OK]` であること。** `[NG]` や `[--]` が残っている状態はリリース不可。

#### 例: 全項目OK（完成状態）

```cpp
/// Compute control output from state estimate and setpoint
/// 推定値とセットポイントから制御出力を計算
///
/// @design architecture.md §4 — IController interface definition       [OK]
/// @design detailed_design.md §4 — Control interface: compute/reset    [OK]
/// @design requirements.md §4 — Component #6: replaceable control      [OK]
/// @design coding_and_education.md §2 — Bilingual comments             [OK]
/// @design coding_and_education.md §2 — Max 50 lines                   [OK]
///
ControlOutput PidController::compute(
    const StateEstimate& state,
    const CommandSetpoint& setpoint,
    float dt)
{
    // ...
}
```

#### 例: 中間段階（NGあり）

```cpp
/// State transition: FLYING → IDLE_GROUND (crash or pilot DISARM)
/// 状態遷移: FLYING → IDLE_GROUND（衝突検知 or パイロットDISARM）
///
/// @design architecture.md §4 — FAILSAFE as event, State Mgr transitions  [OK]
/// @design detailed_design.md §3 — onEnter: ESKFリセット                   [OK]
/// @design detailed_design.md §3 — onEnter: ブザー(disarm音)               [NG] sf_notify未実装
/// @design detailed_design.md §3 — onEnter: モーター停止                   [OK]
///
void StateManager::onEnterIdleGround()
{
    motor.stop();
    eskf.reset();
    // TODO: buzzer.play(DISARM) — sf_notify未実装
}
```

#### 例: 実装直後（未チェック）

```cpp
/// @design detailed_design.md §2 — Topic<T, BufferPolicy, Size>        [--]
/// @design architecture.md §3 — Lightweight Pub-Sub                    [--]
///
template<typename T, typename Policy, int Size>
class Topic {
    // ...
};
```
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

### Namespace 規約（4 階層アクセス対応）

vehicle_new は学習者・実装者がそれぞれのレベルで作業に集中できるよう、namespace を 4 階層に明確に分離する（[`architecture.md`](architecture.md) §2.5「学習者の入口」、横断ルール R8）。各 namespace は **公開先と所有資源** が異なる。

| 層 | namespace | 公開ヘッダ | 何を提供するか | 学習者からの見え方 |
|----|---------|---------|------------|------------------|
| **L0** | `ws::` | `ws_api.hpp` | Sketch API（`setup()` / `loop_400Hz(dt)` / `ws::motor_set_duty()` 等の関数群） | Workshop 受講者・初心者が `ws_api.hpp` 1 つを include して完結 |
| **L1** | `sf::api::` | `sf_api.hpp` | Topic / params / state / mode への公開 API | 推定・制御・ガイダンス学習者が Topic を subscribe / publish |
| **L2** | `stampfly::` | 各 HAL の `*_wrapper.hpp` | `BMI270Wrapper`, `VL53L3CXWrapper` 等の HAL クラス | HW 学習者が SPI / I2C / RMT / LEDC を直接扱う |
| **L3** | `sf::internal::` | `sf_board.hpp` 等 | bus handle、event_loop、起動シーケンスなど BSP 内部資源 | ファーム実装者・拡張者のみ。学習者は通常触らない |

#### include の規約

**学習者向けコードは「自分の層」のヘッダだけを include する**ことを徹底する。

```cpp
// L0 学習者（Workshop 受講者）
#include "ws_api.hpp"           // ws::motor_set_duty, ws::gyro_x, ws::print 等のみ

// L1 学習者（推定・制御・ガイダンス）
#include "sf_api.hpp"            // Topic, params, state, mode の公開 API

// L2 学習者（HW を学びたい）
#include "sf_api.hpp"
#include "bmi270_wrapper.hpp"   // 個別 HAL を直接

// L3 ファーム実装者
#include "sf_api.hpp"
#include "sf_board.hpp"          // BSP getter
#include "bmi270_wrapper.hpp"
```

#### 禁止事項（R3、R8）

- **`sf::api::*` から `sf::internal::*` を transitive に exposure しない**（forward declaration、PIMPL を活用）
- **`ws::*` を `sf::internal::*` に直接依存させない**（`ws::*` は `sf::api::*` への薄いラッパーとして実装）
- **学習者向けヘッダで extern グローバル変数を公開しない**（Topic / params / Board getter のいずれかで提供）

#### 既存 Workshop 移行への含意（R12）

現在の `firmware/workshop/` は HAL コードを vehicle_new からコピーで持っているが、本規約に基づき **vehicle_new の HAL を直接共有する形** に統合する。`ws::*` API は `sf::api::*` を呼ぶ薄いラッパーとして再実装される。詳細は [`workshop_migration.md`](workshop_migration.md) を参照。

## 3. Examples（サンプル集）計画

### Examples Level と Access Tier の関係（重要）

混同しやすいので明確化する。

- **Examples Level 1〜4**: サンプル集の **学習難度の段階**（本節で定義）
- **Access Tier L0〜L3**: ファームの **API アクセス階層**（§2「Namespace 規約」、[`architecture.md`](architecture.md) §2.5 で定義）

両者は別軸であり、Example は「どの難度（Level）か」と「どの Tier の API を使うか」の 2 軸で分類される。Workshop 受講者向けの **Lesson 形式（L0 Tier 中心）** は本サンプル集とは別系統で `firmware/workshop/` 配下に置かれる（M5 で vehicle_new HAL に統合予定、[`workshop_migration.md`](workshop_migration.md) 参照）。

### 設計原則

各Exampleは：
- **単独でビルド・実行可能**（vehicle_new全体のビルド不要）
- **README.mdに完全な説明**（目的、必要な知識、接続図、手順、コードの解説）
- **段階的に複雑度が上がる**（前のExampleの知識を前提に）
- **コメントは本体より多くてもいい**
- **「ここを変えてみよう」セクションで改造の余地を示す**
- **ファイル冒頭に対応 Tier を明記**（例: `// Tier: L2 (HAL Direct)`）

### Example構成

```
examples/01_blink_led/
├── CMakeLists.txt          # 単独ビルド可能
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp            # コメント豊富
└── README.md               # 目的、接続図、実行手順、解説
```

### Level 1: ハードウェア基礎（ESP-IDFとセンサ）— Tier L2 中心

ハードウェア層を学ぶ Example 群。`stampfly::*Wrapper` を直接使い、SPI / I2C / LEDC / GPIO の理解を促す。

| Example | Tier | 学べること | 行数目安 |
|---------|-----|-----------|---------|
| `01_blink_led` | L2 | ESP-IDF基礎、GPIO、WS2812制御 | ~30行 |
| `02_buzzer_melody` | L2 | LEDC PWM、トーン生成 | ~50行 |
| `03_button_event` | L2 | GPIO入力、デバウンス、イベント処理 | ~50行 |
| `04_read_imu` | L2 | SPI通信、BMI270ドライバ、センサデータ表示 | ~60行 |
| `05_read_tof` | L2 | I2C通信、VL53L3CX、距離計測 | ~50行 |
| `06_read_baro` | L2 | I2C通信、BMP280、気圧→高度変換 | ~50行 |
| `07_motor_spin` | L2 | LEDC PWM、モーター単体制御、安全停止 | ~40行 |
| `08_battery_monitor` | L2 | INA3221、電圧/電流読み取り | ~40行 |

### Level 2: 通信と制御の基礎 — Tier L1/L2 混在

Topic API を初めて使う段階。`sf::api::*` で Pub-Sub に触れつつ、必要に応じて HAL も触る。

| Example | Tier | 学べること | 行数目安 |
|---------|-----|-----------|---------|
| `09_espnow_pair` | L2 | ESP-NOW通信、ペアリング、パケット送受信 | ~80行 |
| `10_udp_telemetry` | L1 | Topic を subscribe して UDP 送信、PC で受信 | ~80行 |
| `11_pid_single_axis` | L1 | Topic 経由で PID 制御の基本、1軸モーター制御 | ~100行 |
| `12_complementary_filter` | L1 | `sensor_imu` を読み、相補フィルタで姿勢推定 | ~80行 |
| `13_parameter_tuning` | L1 | パラメータシステム、WiFi 経由で PID ゲイン変更 | ~100行 |

### Level 3: フライトシステム — Tier L1 中心

Topic API でフライト制御パイプラインを構築する段階。`IEstimator` / `IController` インターフェースの理解が深まる。

| Example | Tier | 学べること | 行数目安 |
|---------|-----|-----------|---------|
| `14_attitude_estimation` | L1 | ESKF基礎、`sensor_imu` から姿勢推定、`estimate_state` に publish | ~120行 |
| `15_rate_control` | L1 | レート制御、4モーターミキシング | ~120行 |
| `16_stabilize_flight` | L1 | 姿勢安定化飛行の最小構成 | ~150行 |
| `17_altitude_hold` | L1 | ToF + 高度制御PID | ~150行 |
| `18_position_hold` | L1 | OptFlow + 位置制御PID | ~150行 |
| `19_state_machine` | L1 | 状態管理の実装パターン | ~100行 |
| `20_pubsub_basics` | L1 | Pub-Sub の使い方、トピック追加（[`topic_reference.md`](topic_reference.md) §7 PR チェックリスト準拠） | ~80行 |

### Level 4: 応用・拡張 — Tier L1（差し替え）+ L4 (応用)

業界互換やインターフェース差し替えを学ぶ。学習者が「自分の何か」を実装するフェーズ。

| Example | Tier | 学べること | 行数目安 |
|---------|-----|-----------|---------|
| `21_custom_controller` | L1 | `IController` を実装して独自制御を試す | ~100行 |
| `22_custom_estimator` | L1 | `IEstimator` を実装して独自推定を試す | ~120行 |
| `23_custom_guidance` | L1 | `command_target` を publish する Guidance を書く（[`detailed_design.md`](detailed_design.md) §9） | ~120行 |
| `24_tello_api` | L1 | TelloSDK 互換コマンドで制御 | ~100行 |
| `25_ros2_bridge` | L1 | ROS2 トピックとの連携 | ~100行 |
| `26_blackbox_analysis` | L1 | Blackbox ログの取得と解析 | ~80行 |

### Workshop Lesson との関係（Tier L0）

Workshop 向けの **Lesson 形式 Example** は別系統で、L0 Tier の `ws::*` API のみで完結する。設計上は本 Examples と独立しているが、最終的に同じ HAL / Topic を共有する（M5 で統合）。

| Workshop Lesson | Tier | 対応する vehicle_new Example |
|---------------|-----|-------------------------|
| Lesson 0 (Setup) | L0 | — |
| Lesson 1 (Motor) | L0 | `07_motor_spin`（L2 で再学習）|
| Lesson 2 (Controller) | L0 | `09_espnow_pair`（L2 で再学習）|
| Lesson 3 (LED) | L0 | `01_blink_led`（L2 で再学習）|
| Lesson 4 (IMU) | L0 | `04_read_imu`（L2 で再学習）|
| Lesson 5-8 (PID) | L0 | `11_pid_single_axis`, `15_rate_control`（L1 で再学習）|
| Lesson 9 (Estimation) | L0 | `12_complementary_filter`, `14_attitude_estimation`（L1 で再学習）|
| Lesson 10 (API) | L0 | `20_pubsub_basics`（L1 で再学習）|

**学習導線の意図**: Workshop で L0 を経験 → 興味のある層で L1〜L2 の Example を読み返して深める → L1 で自分の差し替え（21〜23）を作る、という階段。詳細は [`workshop_migration.md`](workshop_migration.md) を参照。

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
