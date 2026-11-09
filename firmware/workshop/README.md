# StampFly Workshop Skeleton

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このファームウェアについて

Workshop Skeleton は StampFly 勉強会のための簡易ファームウェアです。車両ファームウェアの複雑さ（FreeRTOS タスク、HAL 抽象化、センサフュージョン等）を `ws::` 名前空間の簡易 API で隠蔽し、学生は `setup()` と `loop_400Hz()` の2つの関数だけを実装します。

### 構成

```
firmware/workshop/
├── CMakeLists.txt              # ESP-IDF プロジェクト設定
├── sdkconfig.defaults          # ESP32-S3 設定（vehicle から流用）
├── partitions.csv              # パーティションテーブル
├── main/
│   ├── workshop_api.hpp        # ★ 学生向け簡易 API（これだけ読めばOK）
│   ├── user_code.cpp           # ★ 学生が編集するファイル（sf lesson switch で差替え）
│   ├── workshop_main.cpp       # タスク起動（触らない）
│   ├── workshop_task.cpp       # 400Hz ループ実行（触らない）
│   ├── workshop_api.cpp        # ws:: API 実装（触らない）
│   └── workshop_globals.*      # 内部グローバル（触らない）
└── lessons/
    ├── lesson_00_setup/        # Lesson 0: 環境セットアップ
    │   ├── student.cpp         # スケルトン（TODO 付き）
    │   ├── solution.cpp        # 解答
    │   └── README.md           # ハンドアウト
    ├── lesson_01_motor/        # Lesson 1: モータ制御
    ├── ...
    └── lesson_11_competition/  # Lesson 11: 競技会
```

## 2. クイックスタート

### 環境準備

```bash
# 開発環境のセットアップ
source setup_env.sh

# 環境診断
sf doctor
```

### レッスンの切り替え

```bash
# レッスン一覧を表示
sf lesson list

# レッスン 1 に切り替え
sf lesson switch 1

# ビルド & フラッシュ（モニタ付き）
sf lesson build
sf lesson flash
```

### ビルド & フラッシュ（直接）

```bash
sf lesson build
sf lesson flash
```

## 3. 学生向け API

### 基本構造

```cpp
#include "workshop_api.hpp"

void setup()
{
    // 起動時に一度だけ呼ばれる
    ws::print("Hello StampFly!");
}

void loop_400Hz(float dt)
{
    // ARM 時に 400Hz で呼ばれる
    // dt = 0.0025 (秒)
}
```

### API 一覧

| カテゴリ | 関数 | 説明 |
|---------|------|------|
| **Motor** | `ws::motor_set_duty(id, duty)` | モータ個別制御 (id=1-4, duty=0.0-1.0) |
| | `ws::motor_set_all(duty)` | 全モータ同一 duty |
| | `ws::motor_stop_all()` | 全モータ停止 |
| | `ws::motor_mixer(T, R, P, Y)` | ミキサー (thrust/roll/pitch/yaw) |
| **Controller** | `ws::rc_throttle()` | スロットル (0.0-1.0) |
| | `ws::rc_roll()` | ロール (-1.0 to 1.0) |
| | `ws::rc_pitch()` | ピッチ (-1.0 to 1.0) |
| | `ws::rc_yaw()` | ヨー (-1.0 to 1.0) |
| | `ws::is_armed()` | ARM 状態 |
| **LED** | `ws::led_color(r, g, b)` | LED 色設定 (0-255) |
| **IMU** | `ws::gyro_x/y/z()` | 角速度 (rad/s) |
| | `ws::accel_x/y/z()` | 加速度 (m/s^2) |
| **Telemetry** | `ws::telemetry_send(name, value)` | テレメトリ送信 |
| **Estimation** | `ws::estimated_roll/pitch/yaw()` | 推定姿勢角 (rad) |
| | `ws::estimated_altitude()` | 推定高度 (m) |
| **Utility** | `ws::millis()` | 起動後ミリ秒 |
| | `ws::battery_voltage()` | バッテリ電圧 (V) |
| | `ws::print(msg)` | シリアル出力 |

### モータ配置

```
           Front
      FL (M4)   FR (M1)
         ╲   ▲   ╱
          ╲  │  ╱
           ╲ │ ╱
            ╲│╱
             ╳
            ╱│╲
           ╱ │ ╲
          ╱  │  ╲
         ╱   │   ╲
      RL (M3)   RR (M2)
           Rear

M1(FR): CCW  M2(RR): CW  M3(RL): CCW  M4(FL): CW
```

## 4. レッスン一覧

| Day | Lesson | 内容 | API |
|-----|--------|------|-----|
| Day 1 | 00 | 環境セットアップ | `ws::print()` |
| | 01 | モータ制御 | `ws::motor_set_duty()` |
| | 02 | コントローラ入力 | `ws::rc_*()` |
| | 03 | LED 表示 | `ws::led_color()` |
| | 04 | IMU センサ | `ws::gyro_*()`, `ws::telemetry_send()` |
| Day 2 | 05 | 角速度 P 制御 | `ws::motor_mixer()` |
| | 06 | PID 制御 | I 項/D 項追加 |
| | 07 | テレメトリ活用 | `sf log wifi`, `sf log viz` |
| Day 3 | 08 | モデリング/制御系設計 | Loop Shaping Tool |
| | 09 | 姿勢推定 | 相補フィルタ, ESKF 比較 |
| Day 4 | 10 | Python SDK | `StampFly`, Jupyter |
| | 11 | ホバリングタイム競技 | PIDゲイン最適化 |

## 5. sf CLI コマンド

```bash
# レッスン管理
sf lesson list              # 一覧表示
sf lesson switch <N>        # レッスン N に切り替え
sf lesson solution <N>      # 解答を diff 表示
sf lesson build             # ビルド
sf lesson flash             # フラッシュ（モニタ付き）

# 競技会ツール
sf competition hover-time   # ホバリングタイム計測
sf competition score        # スコア・ランキング表示

# テレメトリ
sf log wifi                 # WiFi テレメトリ取得
sf log viz                  # ログ可視化
```

## 6. トラブルシューティング

| 症状 | 対処 |
|------|------|
| ビルドエラー | `sf lesson build -c` でクリーンビルド |
| ポート未検出 | USB ケーブルを確認、`sf doctor` で診断 |
| モータが回らない | ARM 状態か確認 (`ws::is_armed()`) |
| テレメトリが来ない | WiFi AP に接続されているか確認 |

## 7. 下向き ToF が止まったときの挙動

### 何が効くか

Workshop は vehicle の `StateTask`・`ImuTask`・`Failsafe`（ToF の出力が止まったことの判定）をそのまま使うので、下向き ToF（高度センサ）が止まる故障（新サンプルが 0.3 s 来ない、飛行中に無効サンプルだけが続く、サンプルは戻ったが推定器がまだ取り込み直していない）への備えのうち、制御器を持たなくても成り立つものがそのまま効く。LED の黄色の高速点滅と `status` の `tof` 行（CLI）で状態が分かる。

| 場面 | Workshop での挙動 |
|------|------------------|
| 地上（ARM 前） | **ARM を拒否する**（ACRO を含む全モード。`ws::is_armed()` は偽のまま）。ToF が戻っても、推定器が取り込み直す（連続 10 サンプルが採用され、ToF が 1 s 生存）まで拒否が続くので、復帰後も約 1 s 待つ |
| ARM 中・地上（ARMED_GROUND） | 自動で DISARM（モータ停止） |
| 飛行中・ACRO / STABILIZE（既定） | **強制着陸しない**。LED と音で警告するだけで、学習者のコード（`loop_400Hz()`）はそのまま動き続ける |
| 飛行中・ALT_HOLD / POS_HOLD（送信機のモードスイッチ） | LANDING に入り、**モータを止める**（下記） |

### Workshop にないもの

- **ToF なし降下がない。** vehicle は ToF が止まった ALT_HOLD / POS_HOLD で、加速度計だけを頼りに重さ未満の推力で降りる（自動着陸）。Workshop には制御器（`PidController`）がなく、学習者のコードがモータを駆動するので、その降下則は使えない。LANDING に入ると `WorkshopControlTask` が**モータを止める**（自由落下）。ALT_HOLD / POS_HOLD は Workshop では高度を保つ機能を持たない（学習者のコードが推力を決める）ので、ToF が故障する可能性のある場所では ACRO / STABILIZE で飛ばすこと。
- **`controller_status` を出さない。** vehicle の制御器は、上昇量の推定（加速度の積分）などを `controller_status` に出し、`ImuTask` の ToF 健全性の判定が使う。Workshop はこれを出さないので上昇量は常に 0 と読まれ、**離陸中（TAKEOFF）の無効サンプルは TAKEOFF に入ってから 2 s 後にしか数えられない**（vehicle は上昇が 5 cm に達した 0.15 s 後）。つまり、離陸中に ToF が無効のままでも、判定が出るまでに最大で約 2.15 s かかる。FLYING に入った後は vehicle と同じ（0.5 s）。
- 高度センサの値 `ws::tof_bottom()` / `ws::estimated_altitude()` は、ToF が止まっている間は古い値のまま、または無効な値になる。レッスンのコードで高度を使うときは、無効な値（範囲外）を前提にすること。

### 確かめ方

SILS（模擬飛行）で、ToF の出力停止を注入して挙動を再現できる。vehicle 用のシナリオを Workshop の機体モデルで走らせる: `sf sils scenario simulator/sils/scenarios/tof_silent_prearm.scn --target workshop --duration 12000000`（ARM の拒否）と、同じ形で `tof_silent_armed_ground.scn`（ARM 中・地上で ToF が止まると DISARM）。どちらも 2026-11-09 に PASS を確認した。飛行中の挙動（上の表の下 2 行）と、離陸中の無効の判定が 2 s 遅れる点は、SILS のシナリオでは未確認。

---

<a id="english"></a>

## 1. Overview

### About This Firmware

Workshop Skeleton is a simplified firmware for the StampFly workshop. It hides the vehicle firmware's complexity (FreeRTOS tasks, HAL abstraction, sensor fusion, etc.) behind a simple `ws::` namespace API. Students only need to implement two functions: `setup()` and `loop_400Hz()`.

## 2. Quick Start

```bash
# Activate development environment
source setup_env.sh

# Switch to a lesson
sf lesson switch 1

# Build and flash
sf lesson build
sf lesson flash
```

## 3. Student API

```cpp
#include "workshop_api.hpp"

void setup() {
    ws::print("Hello StampFly!");
}

void loop_400Hz(float dt) {
    // Called at 400Hz when armed
    // dt = 0.0025 seconds
}
```

See the Japanese section above for the complete API reference table.

## 4. Lesson List

| Day | Lesson | Topic | Key API |
|-----|--------|-------|---------|
| Day 1 | 00-04 | Setup, Motors, Controller, LED, IMU | `ws::motor_*`, `ws::rc_*`, `ws::gyro_*` |
| Day 2 | 05-07 | P control, PID, Telemetry | `ws::motor_mixer()`, `sf log wifi` |
| Day 3 | 08-09 | Modeling, Estimation | Loop Shaping, ESKF |
| Day 4 | 10-11 | Python SDK, Competition | `StampFly`, Jupyter |

## 5. CLI Commands

```bash
sf lesson list / switch / solution / build / flash
sf competition hover-time / score
```

## 6. Troubleshooting

| Symptom | Fix |
|---------|-----|
| Build error | Clean build with `sf lesson build -c` |
| Port not found | Check the USB cable, diagnose with `sf doctor` |
| Motors do not spin | Check the ARM state (`ws::is_armed()`); also see chapter 7 (a silent ToF refuses ARM) |
| No telemetry | Check that you are connected to the WiFi AP |

## 7. Behaviour when the downward ToF goes silent

### What applies

Workshop reuses vehicle's `StateTask`, `ImuTask` and `Failsafe` (the judgement that the ToF output has stopped) unchanged. So the protections against a failing downward ToF (the altitude sensor: no new sample for 0.3 s, only invalid samples while flying, or samples are back but the estimator has not re-acquired the ToF yet) that do not need a controller apply as they are. The state is shown by the fast yellow LED blink and by the `tof` line of the CLI `status`.

| Situation | Behaviour in Workshop |
|-----------|-----------------------|
| On the ground (before ARM) | **ARM is refused** (every mode, ACRO included; `ws::is_armed()` stays false). The refusal lasts until the estimator has re-acquired the ToF (10 consecutive samples adopted and the ToF alive for 1 s), so wait about 1 s after it returns |
| Armed on the ground (ARMED_GROUND) | Automatic DISARM (motors stop) |
| In flight, ACRO / STABILIZE (default) | **No forced landing.** Only a LED and tone warning; the learner code (`loop_400Hz()`) keeps running |
| In flight, ALT_HOLD / POS_HOLD (transmitter mode switch) | LANDING is entered and the **motors are stopped** (below) |

### What Workshop does not have

- **No descent without a ToF.** In ALT_HOLD / POS_HOLD with a silent ToF, vehicle descends on a thrust below the weight using only the accelerometer (auto-landing). Workshop has no controller (`PidController`); the learner code drives the motors, so that descent law cannot be used. When LANDING is entered, `WorkshopControlTask` **stops the motors** (a free fall). ALT_HOLD / POS_HOLD do not hold altitude in Workshop (the learner code sets the thrust), so fly in ACRO / STABILIZE wherever the ToF may fail.
- **No `controller_status`.** Vehicle's controller publishes the rise estimate (an accelerometer integral) and more on `controller_status`, which the ToF health judgement in `ImuTask` uses. Workshop does not publish it, so the rise always reads 0 and **invalid samples in TAKEOFF are counted only 2 s after TAKEOFF was entered** (vehicle: 0.15 s after the rise reaches 5 cm). A ToF that stays invalid during take-off therefore takes up to about 2.15 s to be judged. After FLYING is entered it is the same as vehicle (0.5 s).
- While the ToF is silent, `ws::tof_bottom()` and `ws::estimated_altitude()` keep an old value or become invalid. When lesson code uses the altitude, expect invalid (out-of-range) values.

### How to check

The ToF silence can be injected in SILS. Run the vehicle scenarios against the Workshop build: `sf sils scenario simulator/sils/scenarios/tof_silent_prearm.scn --target workshop --duration 12000000` (ARM refused) and, the same way, `tof_silent_armed_ground.scn` (a ToF that stops while armed on the ground gives a DISARM). Both PASSED on 2026-11-09. The in-flight behaviour (the last two rows of the table above) and the 2 s delay of the invalid-sample judgement in take-off are NOT confirmed by a SILS scenario.
