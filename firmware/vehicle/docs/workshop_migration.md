# Workshop Migration Plan
# Workshop 移行計画

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

### このドキュメントについて

現行の `firmware/workshop/` は vehicle とは別ファームとして独立して動作しているが、HAL コードを vehicle からコピーで持っているため **二重メンテ** が発生している。本文書は、vehicle 完成後に Workshop を **vehicle + L0 (`ws::*`) ラッパー** として再構成するための移行計画を定義する（横断ルール R12）。

### スコープ

- 現行 Workshop の機能棚卸し（30+ API、13 Lesson）
- vehicle で再現するために必要な機能リスト
- L0 `ws::*` ラッパーの実装方針
- 移行時の 5 大論点と解決方針
- 実施フェーズ（M5 で実施予定）

### 関連文書

| 文書 | 役割 |
|------|------|
| [`architecture.md`](architecture.md) §2.5 | 4 階層アクセスにおける L0 の位置づけ |
| [`coding_and_education.md`](coding_and_education.md) §2 | namespace 規約（`ws::` / `sf::api` / `stampfly::` / `sf::internal`） |
| [`coding_and_education.md`](coding_and_education.md) §3 | Examples Level と Workshop Lesson の関係表 |
| [`topic_reference.md`](topic_reference.md) | `ws::*` ラッパーが裏で使う Topic |
| **本文書** | **Workshop 移行の実行計画** |

---

## 2. 現行 Workshop の機能棚卸し

### 2.1 学習者向け API 一覧（30+ 関数）

`firmware/workshop/main/workshop_api.hpp` で公開されている `ws::*` 名前空間の関数群。

#### 通信設定
- `ws::set_channel(channel)` — WiFi ESP-NOW チャンネル設定（1/6/11）、NVS 永続化対応

#### モータ制御
- `ws::motor_set_duty(id, duty)` — 個別モータ制御（id=1-4、duty=0.0-1.0）
- `ws::motor_set_all(duty)` — 全モータ同一 duty
- `ws::motor_stop_all()` — 全モータ即座停止
- `ws::motor_mixer(T, R, P, Y)` — 4 軸モータミキサー（推力 + 3 軸トルク）

#### コントローラ入力
- `ws::rc_throttle()` / `ws::rc_roll()` / `ws::rc_pitch()` / `ws::rc_yaw()`
- `ws::arm()` / `ws::disarm()` / `ws::is_armed()`
- `ws::rc_throttle_yaw_button()` / `ws::rc_roll_pitch_button()`
- `ws::rc_stabilize_acro_mode()` / `ws::rc_alt_mode()` / `ws::rc_pos_mode()`

#### LED 制御
- `ws::disable_led_task()` / `ws::enable_led_task()` / `ws::is_led_task_disabled()`
- `ws::led_color(r, g, b)`

#### IMU センサ
- `ws::gyro_x()` / `ws::gyro_y()` / `ws::gyro_z()` — 角速度 [rad/s]
- `ws::accel_x()` / `ws::accel_y()` / `ws::accel_z()` — 加速度 [m/s²]

#### 環境・距離センサ
- `ws::baro_altitude()` / `ws::baro_pressure()`
- `ws::mag_x()` / `ws::mag_y()` / `ws::mag_z()`
- `ws::tof_bottom()` / `ws::tof_front()`
- `ws::flow_vx()` / `ws::flow_vy()` / `ws::flow_quality()`

#### 姿勢推定
- `ws::estimated_roll()` / `ws::estimated_pitch()` / `ws::estimated_yaw()` — [rad]
- `ws::estimated_altitude()` — [m]

#### ユーティリティ
- `ws::millis()` — 起動後経過時間 [ms]
- `ws::battery_voltage()` — [V]
- `ws::print(fmt, ...)` — シリアル出力（Teleplot 対応）

### 2.2 内部実装の概要

現行 Workshop は **vehicle ファームウェアの IMU/Mag/Baro/ToF/OptFlow/LED/Button/Power タスクをそのまま再利用し、ControlTask のみ置換** する戦略。

- モータ制御: vehicle の `g_motor.setMotor()` / `g_motor.setMixerOutput()` を直結
- IMU: `g_fusion.getState()` で ESKF 状態 + `StampFlyState::getIMUCorrected()` で補正済み値
- 状態管理: `StampFlyState::getInstance()` シングルトン
- LED: `LEDManager::getInstance()` の Channel/Priority/Pattern 機構
- 通信: `ControllerComm` で ESP-NOW + NVS チャンネル管理

### 2.3 Lesson 構成（13 Lesson + 競技会）

| Day | Lesson | 目標 | 主要 API |
|-----|--------|------|---------|
| Day 1 | 0: セットアップ | ビルド確認、シリアル確認 | `ws::print()` |
| | 1: モータ制御 | PWM duty、モータ配置理解 | `ws::motor_set_duty()` |
| | 2: コントローラ | スティック入力、ARM | `ws::rc_*()` |
| | 3: LED 表示 | RGB 制御 | `ws::led_color()` |
| | 4: IMU センサ | ジャイロ・加速度、Teleplot | `ws::gyro_*()` |
| Day 2 | 5: P 制御 | 初飛行、角速度フィードバック | `ws::motor_mixer()` |
| | 6: モデリング | システム同定 | Loop Shaping Tool |
| | 7: SysID | フライトデータ分析 | `sf log wifi` |
| Day 3 | 8: PID 制御 | I/D 項追加、アンチワインドアップ | `ws::gyro_*()` + PID |
| | 9: 姿勢推定 | 相補フィルタ vs ESKF 比較 | `ws::estimated_*()` |
| Day 4 | 10: API 概要 | 拡張、カスタムアプリ | Baro / Mag / ToF / Flow |
| | 13: 競技会 | 精密着陸、ゲイン最適化 | PID チューニング |

---

## 3. 移行後のアーキテクチャ

### 3.1 ファイル構成

移行後、`firmware/workshop/` は以下のような薄いラッパー構成になる（HAL コピーは廃止）。

```
firmware/workshop/
├── CMakeLists.txt           # vehicle component を依存として参照
├── main/
│   ├── workshop_main.cpp    # 起動・ARM ロジック・400Hz 同期（既存維持）
│   ├── user_code.cpp        # 学習者が書く setup() / loop_400Hz() （既存維持）
│   ├── ws_api.hpp           # ws::* API 宣言（vehicle からの依存ラッパー）
│   └── ws_api.cpp           # ws::* 実装（sf::api::* と Topic に委譲）
├── lessons/                 # Lesson 教材（既存維持）
└── components/              # ★ HAL コピーを削除
                             # vehicle/components/sf_hal_* を直接参照する
```

### 3.2 `ws::*` 実装方針（薄いラッパー）

L0 API は内部的に **L1 (Topic) または L2 (HAL) を呼ぶ薄いラッパー** として再実装する。

```cpp
// ws_api.cpp の実装例

namespace ws {

// === IMU: Topic から最新値を取得 ===
// Topic は RingBuffer なので、最新値の peek は別途用意するか
// Latest ミラー Topic を経由する
float gyro_x() {
    return sf::api::imu_latest().gyro[0];
}

// === Estimated state: Topic から最新を取得 ===
float estimated_roll() {
    return sf::api::estimate_state.latest().attitude.roll;
}
float estimated_altitude() {
    return sf::api::estimate_state.latest().altitude;
}

// === Motor: Actuator サービスへ委譲 ===
void motor_set_duty(int id, float duty) {
    sf::api::actuator::set_motor_duty(id, duty);
}

void motor_mixer(float T, float R, float P, float Y) {
    sf::api::actuator::mix_and_publish(T, R, P, Y);
}

// === RC inputs: command_setpoint Topic から取得 ===
float rc_throttle() {
    return sf::api::command_setpoint.latest().throttle;
}

// === ARM logic: StateManager に委譲（二重起動防止 guard 付き）===
void arm() {
    sf::api::state::request_arm();
}
bool is_armed() {
    return sf::api::state::current_mode() == sf::SystemMode::ARMED ||
           sf::api::state::current_mode() == sf::SystemMode::FLYING;
}

}  // namespace ws
```

### 3.3 利点

| 利点 | 内容 |
|-----|-----|
| HAL コピー廃止 | 30+ ファイルの二重メンテ解消、バグ修正・新機能追加が片方だけで済む |
| 同一 estimator | Workshop と vehicle で同じ ESKF を使うため、Lesson 9 の「相補フィルタ vs ESKF 比較」が常に最新の挙動と一致 |
| Examples からの階段 | Workshop で L0 を経験した学習者が、Examples Level 1〜4 で L1/L2 へ降りる導線が滑らか |
| 設計改善の伝播 | vehicle の `@design`、横断ルール R1〜R16、Topic SSOT 等が Workshop にも適用される |

---

## 4. 移行時の 5 大論点と解決方針

### 論点 1: ESKF API 凍結

**問題**: Workshop の `ws::estimated_*()` は vehicle 共有の ESKF に直結している。vehicle で ESKF が quaternion → Euler 変換の仕様を変えると、Lesson 9（相補フィルタ vs ESKF 比較）の比較が不整合になる。

**解決策**:
- vehicle の `IEstimator` インターフェース（`getAttitudeEuler()` / `getPosition()` / `getVelocity()`）の **シグネチャ・単位・座標系を凍結** する
- 凍結内容を [`detailed_design.md`](detailed_design.md) §5 に明記し、変更には ADR（Architecture Decision Record）を要求
- `StateEstimate` データ型のフィールド追加は OK、フィールド変更は破壊的変更扱い

### 論点 2: ARM ロジックの状態遷移 guard

**問題**: 現行 Workshop は `StampFlyState::requestArm()` と `g_motor.arm()` を両方呼ぶ。vehicle の StateManager が stateless だと、ARM 中に `requestArm()` を再呼び出しすると二重起動の危険。

**解決策**:
- `sf_state::StateManager` に **ARM guard** を組み込む（`if (state != ARMED) state = ARMED`）
- `sf::api::state::request_arm()` は冪等（idempotent）にする — 既に ARM 状態なら何もしない
- Lesson 中の `while (true) ws::arm()` のような誤用パターンでもモータが暴走しない

### 論点 3: 400Hz タイミング同期の信頼性

**問題**: 現行 Workshop は IMU timer → semaphore で 400Hz 同期（ESP timer 2500μs 周期）。`loop_400Hz()` が遅延すると IMU semaphore を取り損ねて skip される。Bluetooth/WiFi タスクが Core 1 を奪うと制御ゲイン調整（Lesson 5-8）の再現性が損なわれる。

**解決策**:
- ImuTask と ControlTask（または `loop_400Hz` 駆動タスク）を **Core 1 固定**、優先度 24 / 23 で起動
- `xTaskNotify` を使った IMU → Control の同期を堅牢化（旧の semaphore より高速・安全）
- watchdog timeout を 10 秒に設定し、`loop_400Hz` 内で長時間ブロックしないことを学習者に明示

### 論点 4: ControlPacket の互換性とバージョニング

**問題**: 現行 Workshop は vehicle の `ControlPacket` 構造体 + `CTRL_FLAG_*` enum を直結している。vehicle で新フォーマットに変えると全 Lesson が壊れる。

**解決策**:
- `ControlPacket` 構造体に **`uint8_t version` フィールド**を先頭に追加
- ESP-NOW 受信側（`sf_comm`）でバージョン判定し、複数バージョンをサポート
- 既存 Workshop が使うバージョンを `v1`、vehicle 拡張版を `v2` として共存
- Workshop 統合時に Workshop 側を `v2` に揃える

### 論点 5: NVS namespace 分離

**問題**: 現行 Workshop の `ws::set_channel()` は `nvs_set_u8(handle, "wifi_ch")` で共有 NVS namespace を汚染する。Workshop と vehicle のデュアルビルド環境で値が迷走する。

**解決策**:
- vehicle は **`nvs_open("vehicle", ...)` namespace** を使う
- Workshop は移行後 vehicle と同じ namespace を共有する（同一 firmware なので競合なし）
- 旧 Workshop の `wifi_ch` キーは初回起動時にマイグレーション（既存 NVS 値があれば読み取って vehicle namespace に書き直す）

---

## 5. vehicle 側で必要な追加機能

Workshop 移行を成立させるために、vehicle 側で実装する必要がある追加機能のリスト。M5 までに以下を完備する。

### 5.1 sf::api 公開関数

| 関数 | 用途 | 実装担当 |
|-----|-----|--------|
| `sf::api::imu_latest()` | RingBuffer の最新 IMU をコピーで返す | M2 で sf_core 拡張 |
| `sf::api::actuator::set_motor_duty(id, duty)` | モータ単体制御 | M2 で sf_actuator 拡張 |
| `sf::api::actuator::mix_and_publish(T,R,P,Y)` | ミキサー経由で actuator_motor を publish | M2 |
| `sf::api::state::request_arm()` / `request_disarm()` | ARM 要求（冪等） | M2 で sf_state 拡張 |
| `sf::api::state::current_mode()` | 現在のシステムモード | M2 |
| `sf::api::params::set/get<T>(key)` | 学習者向け簡易パラメータ API | M2 |

### 5.2 Topic からの最新値取得 helper

RingBuffer は SPSC 制限があるため、Workshop の `ws::gyro_x()` のような「複数所からの最新値取得」には対応できない。以下のいずれかで解決：

- **A 案**: `sensor_imu` を読む専用の Latest mirror Topic（`sensor_imu_latest`）を別途用意し、ImuTask が両方に publish
- **B 案**: `sf::api::imu_latest()` を thread-safe な内部 cached value として実装（atomic でスナップショット）

→ 採用は M2 で決定。教育性と性能のトレードオフを評価してから。

### 5.3 ControlPacket バージョニング

`firmware/common/protocol/` に `ControlPacket v2` を定義し、vehicle と Workshop が共通で使う形を整備（M5 で実施）。

---

## 6. 移行チェックリスト（M5 で実施）

### A. ハードウェア層対応

- [ ] vehicle の `sf_actuator::set_motor_duty()` / `mix_and_publish()` が動作確認済み
- [ ] vehicle の IMU / Mag / Baro / ToF / OptFlow / LED / Button / Power が全て Topic に publish できる状態
- [ ] vehicle の ESKF が `getAttitudeEuler()` / `getPosition()` で値を返す

### B. 状態・通信層対応

- [ ] `sf::SystemMode` に IDLE / ARMED / FLYING の遷移が実装済み
- [ ] `sf::api::state::request_arm()` の冪等性確認
- [ ] ControlPacket v2 が定義され、`sf_comm` が v1 / v2 両方を受信可能
- [ ] 400Hz IMU → ControlTask 同期が `xTaskNotify` で動作

### C. ws::\* API 再実装（30+ 関数）

カテゴリごとに対応関係を確認しながら実装：

- [ ] モータ系（4 関数）: `motor_set_duty`, `motor_set_all`, `motor_stop_all`, `motor_mixer`
- [ ] RC 入力系（11 関数）: `rc_throttle/roll/pitch/yaw`, `arm/disarm/is_armed`, `rc_*_button`, `rc_*_mode`
- [ ] LED 系（4 関数）: `disable/enable_led_task`, `led_color`, `is_led_task_disabled`
- [ ] IMU 系（6 関数）: `gyro_*`, `accel_*`
- [ ] 環境センサ系（9 関数）: `baro_*`, `mag_*`, `tof_*`, `flow_*`, `flow_quality`
- [ ] 推定系（4 関数）: `estimated_roll/pitch/yaw/altitude`
- [ ] 通信系（1 関数）: `set_channel`
- [ ] ユーティリティ（3 関数）: `millis`, `battery_voltage`, `print`

### D. ControlTask ロジック（workshop_main.cpp）

- [ ] `setup()` / `loop_400Hz(float dt)` のシグネチャ定義
- [ ] 400Hz タスク通知で `loop_400Hz()` 呼び出し
- [ ] ARM 状態管理: `is_armed()` で FlightState 確認
- [ ] 安全機構: 非 ARM 時に `motor_stop_all()` 強制実行
- [ ] edge detection: RC ARM フラグの立ち上がり / 立ち下がり検出
- [ ] LED タスク無効時も `loop_400Hz()` が IDLE で動作

### E. 13 Lesson の動作確認

- [ ] Lesson 0-4: 基本 API（モータ / 入力 / LED / IMU）が動く
- [ ] Lesson 5-8: PID ループ、ミキサー出力、ジャイロフィードバックが動く
- [ ] Lesson 9: ESKF 推定姿勢角が動く
- [ ] Lesson 10: Baro / Mag / ToF / Flow にアクセスできる
- [ ] Lesson 13: 競技ロジック（ARM → 飛行 → 着陸タイム計測）が動く

### F. テレメトリ・デバッグ機能

- [ ] `ws::print()` → Teleplot 対応（`>name:value` format）
- [ ] `sf log wifi` → ControlPacket 受信パース
- [ ] Serial CLI: `wifi_ch`, `arm/disarm` コマンド

### G. 既知制限の確認

- [ ] ARM ボタン = スロットル / ヨーボタン兼用（アーキテクチャ固定）
- [ ] ペアリング時 LED はシステム優先度（PAIRING パターン）
- [ ] IDLE 中 `loop_400Hz()` は LED タスク無効化で有効化
- [ ] ESKF quaternion state: vehicle と同じ実装を使うため互換維持

---

## 7. 実施フェーズ

| フェーズ | 内容 | 前提 |
|---------|-----|------|
| **M2** | sf_board + Phase 2a 手直し + sf::api 公開関数の整備 | M1 シリーズ完了 |
| **M3** | Phase 2.2 ToF 結合 | M2 |
| **M3a-d** | Phase 2.3 Baro / 2.4 Flow / 2.5 Mag / 2.7 NVS 持続化 | M3 |
| **M4** | sf_logger / sf_telemetry の実装、Phase 3 ACRO 同定の準備 | M3a-d |
| **M5** | **本文書に従い Workshop を vehicle に統合** | M4 |
| **M5a** | 13 Lesson の動作確認、競技会要件の検証 | M5 |
| **Phase 4 以降** | STABILIZE / ALT / POS の実機検証 | M5a |

M5 完了後、`firmware/workshop/` は vehicle と同じファームウェアバイナリ（または同じ HAL を共有するバリアント）として運用される。

---

## 8. 実施記録（2026-07-18 実施済み）

**本計画（M5）は 2026-07-18 に実施した。** `firmware/workshop/` は現行 `firmware/vehicle/`
のコンポーネント基盤上に再構築され、vehicle_old への依存は解消された。DXH 講座
（2026-07-18）で発生した「workshop 課題を書き込むとチャンネルが変わりペアリングが崩れる」
事象（＝本文書 論点5 が予言していた NVS 保存場所の分裂）が実施の直接の契機である。

### 実施時に確定した設計判断（本文からの更新点）

| 論点 | 計画時 | 実施時の確定 |
|------|--------|------------|
| §4 論点1（ESKF API 凍結） | ADR 要求 | 変更なし（同一コンポーネント共有で自然に一致。凍結ルールは引き続き有効） |
| §4 論点2（ARM guard） | StateManager に guard 追加 | **既存実装で充足**。`StateManager::requestArm()` は状態ゲート（IDLE_GROUND のみ受理）で冪等。`ws::arm()` は `api_command` トピックに ApiCmd::Arm を発行するだけ |
| §4 論点3（400Hz 同期） | xTaskNotify 化 | **既存実装で充足**。vehicle の ImuTask→ControlTask 通知（タイムアウト安全網付き）をそのまま流用。WorkshopControlTask が `sf::tasks::control_handle()` を提供 |
| §4 論点4（ControlPacket バージョニング） | v1/v2 共存 | **不要になり廃止**。workshop が vehicle と同じ sf_comm/sf_command を共有するため乖離が構造的に発生しない |
| §4 論点5（NVS namespace） | vehicle 専用 namespace | `sf_params`/`wifi.channel`（param 系）に一本化。旧キー `stampfly`/`wifi_ch` は**初回起動時に一度だけ取り込み**（workshop_main.cpp の importLegacyWifiChannel、param 系未保存の場合のみ） |
| §5.2（最新値取得 A案/B案） | M2 で決定 | **B 案相当が既に実装済み**: `sf::api::imu_latest()`（RingBuffer の latest() peek、任意タスクから安全） |
| §5.1（sf::api::actuator / state 公開関数） | M2 で整備 | **今回は導入せず**。vehicle の ControlTask と同じく「タスク層が sf::Actuator を直接所有」する形を WorkshopControlTask が踏襲（precedent 準拠）。sf::api への正式昇格は将来課題 |

### 実装の骨子

- **ControlTask 置換のみ**: タスク表は vehicle と同一で、`WorkshopControlTask`
  （workshop_control_task.cpp）だけが差し替わる。学習者の `setup()`/`loop_400Hz(dt)` を
  呼び、モータ要求（ws_internal::MotorRequest）を ARM ゲート内で `Actuator::applyTestDuties()`
  に解決する。**INV-1 との関係**: workshop ビルドでは学習者ループが唯一の制御パイプライン
  （置換であって並列ではない）。
- **旧ミキサー式の完全再現**: `ws::motor_mixer()` は vehicle_old `setMixerOutput` の
  電圧スケール式（`T + 0.25·(±R±P±Y)/3.7`）を桁まで再現（ws_internal.hpp）。旧ファームで
  調整した学習者ゲインの互換性を保証する。
- **LED**: `ws::led_color()`/`disable_led_task()` は `ui_command` トピックの
  `LedUserOverride`/`LedUserColor` verb（sf_notify に追加）で実現。色は変化時のみ publish。
- **RC 正規化の互換**: 旧 `getControlInput`（throttle=上半分のみ 0..1、他 ±1）と
  vehicle `CommandSetpoint` は同一意味論であることをソースで確認済み。
- **チャンネル/ペアリングの一本化**: チャンネルは `wifi.channel`、ペアリング MAC は
  `sf_pair`/`ctrl_mac` — vehicle と workshop でどちらをフラッシュしても保持される。

### 既知の制約（実施時点）

| 項目 | 内容 |
|------|------|
| `ws::tof_front()` | 常に -1.0（vehicle パイプラインに前方 ToF が無い）。Lesson 10 から使用を除去済み |
| `ws::flow_vx/vy()` | 速度 [m/s] → 生カウントに単位変更（使用は Lesson 10 の表示のみ） |
| `ws::rc_roll_pitch_button()` | 常に false（現行プロトコルに FLIP フラグ無し。使用レッスン無し） |
| Data Stream の rate_ref/angle_ref | 常に 0（学習者ループはカスケード目標を持たない）。Lesson 7 の sysid 手順は SCI チュートリアル（2026-09）前に要再検証 |
| Lesson 12（Python SDK） | ビルドは PASS。vehicle の Tello API 経由の飛行検証は実機確認待ち |

### 検証記録（2026-07-18）

- `sf build vehicle` 緑（1118.8 KB）／`sf build workshop` 緑（1104.4 KB）
- SIL 全39シナリオ: 31 PASS / 8 FAIL — **8 件は変更前ベースラインと完全一致の既存失敗**
  （変更ファイルのみ stash した前後比較で退行ゼロを確認。既存失敗は本移行と無関係）
- 全レッスン×(student|solution) 27通りのビルド: 全 PASS
- 実機ベンチ・飛行レッスン（L5-8, 13）の実機検証は未実施（翌朝ベンチ確認から）

---

<a id="english"></a>

## 1. About This Document

> **Status:** Migration plan for integrating the existing `firmware/workshop/` (which currently has copied HAL code) into the vehicle firmware as a thin L0 (`ws::*`) wrapper layer. The Japanese section above is the authoritative version. Full English translation pending.

This document defines:
- Inventory of the current Workshop firmware (30+ APIs, 13 Lessons)
- Required features in vehicle to support the migration
- Implementation strategy for `ws::*` as a thin wrapper over `sf::api::*` and `stampfly::*Wrapper`
- Five major points of contention and their resolution strategies (ESKF API freezing, ARM guard, 400Hz sync robustness, ControlPacket versioning, NVS namespace isolation)
- Migration checklist for M5 milestone

See the Japanese section for full content.
