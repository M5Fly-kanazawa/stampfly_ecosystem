# 09_topic_api_hello — L1 Topic API 入門

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> **命名についての注記:** `docs/coding_and_education.md` §3「Examples 計画」の表では
> 09/10 番は `09_espnow_pair`（L2）/`10_udp_telemetry`（L1）に予約されている。本サンプル
> （`09_topic_api_hello`）はその計画より前に、L1 Topic API 単体に絞った最小例として
> 別途依頼・作成された。番号の重複は計画表の更新（採番のずらし、または本サンプルの
> 別名への変更）が必要になる可能性がある — 設計文書との不一致として記録しておく。

## 1. 目的

`sf::api::*`（L1 Topic API）だけを使って、動いているファームウェアから状態推定値を
読み出す方法を学ぶ。HAL（センサドライバ）にもタスクにも触れない、L1 学習者の最小の
第一歩。

## 2. Topic API とは（Pub-Sub: 発行/購読）

StampFly vehicle のコンポーネント間通信は、直接関数を呼び合うのではなく **Topic**
（データの入れ物）を介した **Pub-Sub**（Publish=発行 / Subscribe=購読）方式で行う
（`docs/architecture.md` §3）。発行側（Publisher）は「このデータが最新です」と
`publish()` するだけで、誰が読むかを知らない。購読側（Subscriber）は `latest()` で
「今ある最新値」を取得するだけで、誰が発行したかを知らない。両者は疎結合。

本サンプルが読む Topic は `estimate_state`（姿勢・位置・速度の融合推定値、400Hz、
最新値のみ保持）。`sf::api::estimate_latest()` はこの Topic の `latest()` を呼ぶだけの
薄いラッパーである（`components/sf_api/include/sf_api.hpp:84`）。

## 3. 重要な5〜10行

本サンプルで学ぶべきコードは `main/main.cpp` の `printRollPitchYaw()` 関数、これだけ。

```cpp
sf::StateEstimate state = sf::api::estimate_latest();
sf::math::Quat attitude(state.attitude[0], state.attitude[1],
                         state.attitude[2], state.attitude[3]);
sf::math::Vec3 euler_radians = attitude.to_euler();

float roll_degrees  = euler_radians.x * config::kRadToDeg;
float pitch_degrees = euler_radians.y * config::kRadToDeg;
float yaw_degrees   = euler_radians.z * config::kRadToDeg;
```

`estimate_state` の姿勢はクォータニオン `[w,x,y,z]`（`data_types.hpp:127`）で保持される
ため、`sf_math.hpp` の `Quat::to_euler()` で roll/pitch/yaw（ラジアン）に変換し、度に
換算して表示する。他のファイル（`internal_sensor_feed.cpp`、`main.cpp` のこの関数より
前の部分）は、この5〜10行に「読むべき本物のデータ」を用意するための下ごしらえに
過ぎない（次節参照）。

## 4. ビルド・書き込み・実行

```bash
source setup_env.sh
cd firmware/vehicle/examples/09_topic_api_hello
idf.py set-target esp32s3
idf.py build
idf.py -p <port> flash monitor
```

`sf` CLI を使う場合（vehicle 本体ビルドとは別のスタンドアロンプロジェクトのため、
`sf build` の対象外 — 上記の `idf.py` を直接使うこと）。

## 5. 期待される出力

```
I (327) topic_api_hello: === Example 09: L1 Topic API hello ===
I (341) internal_feed: IMU + complementary filter ready
roll=  +0.12 deg  pitch=  -1.84 deg  yaw=  +0.00 deg
roll=  +0.09 deg  pitch=  -1.79 deg  yaw=  +0.00 deg
roll= +14.37 deg  pitch=  -1.66 deg  yaw=  +0.01 deg   ← 機体を傾けると反応する
...
```

- 起動直後は水平姿勢（クォータニオン単位元）から積分が始まるため roll/pitch は
  ほぼ 0。StampFly を手で傾けると数値が追従する。
- yaw はジャイロ積分のみ（地磁気を使わない相補フィルタ、§6 参照）で補正が無いため、
  静置していても時間とともにゆっくりドリフトする。これは想定内の制限であり、
  実ファームの ESKF（`14_attitude_estimation` 相当、地磁気補正あり）で解消される。

## 6. この例がどう組み立てられているか（L1/L2 境界）

`sf::api::estimate_latest()` は「最後に誰かが `estimate_state` に publish した値」を
返すだけで、その「誰か」を用意する責任は呼び出し側にある。実ファームでは
`tasks/imu_task.cpp`（約950行: 起動校正・フェイルセーフ・モード遷移・ESKF/相補フィルタ
切替を含む）がその役目を担うが、これは `main` コンポーネントに直接コンパイルされる
差し替え不可能なコードで、単体サンプルからそのまま再利用することはできない。

そこで本サンプルは `internal_sensor_feed.cpp`（**L2 相当の下ごしらえ**）で、設計が許す
最小の正直な代替を実装した:

| 行っていること | 行っていないこと |
|---|---|
| 実際の BMI270 を SPI で読む（`examples/04_read_imu` と同じ配線・同じドライバ） | 起動校正（バイアス測定） |
| `ComplementaryEstimator`（`IEstimator` 実装、Mahony型相補フィルタ）で predict() を1ステップ実行 | フェイルセーフ・衝撃検知 |
| `sensor_imu` と `estimate_state` の2つの Topic に publish | 離着陸ロジック、モーター制御 |

つまり **publish される数値は缶詰ではなく実際に動くセンサ由来**だが、この構成は
意図的にフライト不可能である。L1 学習者は `internal_sensor_feed.cpp` のようなコードを
自分で書く必要はない — 実ファームでは `ImuTask` が既に用意している。

## 7. よくあるエラーと対処

| 症状 | 対処 |
|------|------|
| `BMI270 init failed: ESP_ERR_...` | SPI配線を確認（MOSI=14, MISO=43, SCK=44, CS=46 — `examples/04_read_imu/README.md` 参照） |
| `IMU read failed` が繰り返し出る | SPIクロック・配線長・ノイズを疑う。StampFly実機なら通常発生しない |
| roll/pitch が起動直後だけ大きくずれる | 起動校正なしの構成のため（§6）。数秒待つか、水平な場所で起動する |

## 8. 次のステップ

- **`IEstimator` を差し替える**: `internal_sensor_feed.cpp` の
  `sf::ComplementaryEstimator attitude_estimator;` を別の `IEstimator` 実装に変えると、
  `main.cpp` の L1 コード（`printRollPitchYaw()`）は一切変更不要で動く —
  これが `architecture.md` §2.5 の「差替可能設計」の実体。詳しくは
  `docs/coding_and_education.md` §3 の `22_custom_estimator` 計画を参照。
- **`IController` を差し替える**: 制御側の同じ考え方は `examples/10_custom_controller`
  を参照。
- **設計文書**: `docs/architecture.md` §2.5「学習者の入口（4階層アクセス）」、
  `docs/topic_reference.md` §4「使用パターン」。

---

<a id="english"></a>

## 1. Overview

Learn how to read a state estimate from the running firmware using only
`sf::api::*` (the L1 Topic API) — no HAL (sensor driver), no task, the
smallest possible first step for a Tier L1 learner.

## 2. What the Topic API Is (Pub-Sub: Publish / Subscribe)

Components in the StampFly vehicle firmware never call each other's
functions directly; they communicate through **Topics** (typed data
containers) using a **Pub-Sub** (Publish / Subscribe) pattern
(`docs/architecture.md` §3). A publisher just calls `publish()` — "here is
the newest value" — without knowing who reads it. A subscriber just calls
`latest()` — "give me whatever is newest right now" — without knowing who
published it. The two sides are decoupled.

This example reads the `estimate_state` Topic (fused attitude / position /
velocity estimate, 400 Hz, latest-value only). `sf::api::estimate_latest()`
is a thin wrapper that just calls that Topic's `latest()`
(`components/sf_api/include/sf_api.hpp:84`).

## 3. The 5–10 Lines That Matter

The only code you need to study in this example lives in the
`printRollPitchYaw()` function in `main/main.cpp`:

```cpp
sf::StateEstimate state = sf::api::estimate_latest();
sf::math::Quat attitude(state.attitude[0], state.attitude[1],
                         state.attitude[2], state.attitude[3]);
sf::math::Vec3 euler_radians = attitude.to_euler();

float roll_degrees  = euler_radians.x * config::kRadToDeg;
float pitch_degrees = euler_radians.y * config::kRadToDeg;
float yaw_degrees   = euler_radians.z * config::kRadToDeg;
```

`estimate_state`'s attitude is stored as a quaternion `[w,x,y,z]`
(`data_types.hpp:127`), so `sf_math.hpp`'s `Quat::to_euler()` converts it to
roll/pitch/yaw in radians, which we scale to degrees for display. Every
other file (`internal_sensor_feed.cpp`, and the part of `main.cpp` above
this function) exists only to give these 5–10 lines real data to read (see
section 6).

## 4. Build / Flash / Run

```bash
source setup_env.sh
cd firmware/vehicle/examples/09_topic_api_hello
idf.py set-target esp32s3
idf.py build
idf.py -p <port> flash monitor
```

This is a standalone project (not part of the vehicle firmware build), so
use `idf.py` directly rather than `sf build`.

## 5. Expected Output

```
I (327) topic_api_hello: === Example 09: L1 Topic API hello ===
I (341) internal_feed: IMU + complementary filter ready
roll=  +0.12 deg  pitch=  -1.84 deg  yaw=  +0.00 deg
roll=  +0.09 deg  pitch=  -1.79 deg  yaw=  +0.00 deg
roll= +14.37 deg  pitch=  -1.66 deg  yaw=  +0.01 deg   <- responds when you tilt the board
...
```

- Right after boot, roll/pitch start near 0 (the filter integrates from the
  identity quaternion — level attitude). Tilting the StampFly by hand makes
  the numbers track it.
- Yaw is gyro-integration only (this complementary filter does not fuse the
  magnetometer — see section 6), so it drifts slowly even at rest. This is
  an expected limitation of this minimal build, resolved in the real
  firmware's ESKF (the `14_attitude_estimation` example equivalent, which
  does fuse the magnetometer).

## 6. How This Example Is Built (the L1/L2 Boundary)

`sf::api::estimate_latest()` only returns whatever the last publisher put on
`estimate_state` — supplying that publisher is the caller's responsibility.
In the real firmware, `tasks/imu_task.cpp` (~950 lines: boot-calibration
gating, failsafe, mode-transition handling, ESKF/complementary-filter
switching) does that job, but it compiles straight into the non-swappable
`main` component and cannot be reused as-is from a standalone example
project.

So this example implements the smallest honest substitute the design allows
in `internal_sensor_feed.cpp` (**L2-level boilerplate**):

| What it does | What it does NOT do |
|---|---|
| Reads a real BMI270 over SPI (same wiring/driver as `examples/04_read_imu`) | Boot calibration (bias measurement) |
| Runs one predict() step of `ComplementaryEstimator` (an `IEstimator` implementation — Mahony-style complementary filter) | Failsafe / impact detection |
| Publishes to the `sensor_imu` and `estimate_state` Topics | Takeoff/landing logic, motor control |

So **the published numbers come from a real, moving sensor**, not a canned
value — but this build is deliberately not flight-capable. A Tier L1
learner never needs to write code like `internal_sensor_feed.cpp`
themselves; the real firmware's `ImuTask` already provides it.

## 7. Common Errors

| Symptom | Fix |
|---|---|
| `BMI270 init failed: ESP_ERR_...` | Check SPI wiring (MOSI=14, MISO=43, SCK=44, CS=46 — see `examples/04_read_imu/README.md`) |
| Repeated `IMU read failed` | Suspect SPI clock/wiring length/noise; should not happen on real StampFly hardware |
| Large roll/pitch error right after boot | Expected — this build has no boot calibration (section 6); wait a few seconds, or boot on a level surface |

## 8. Next Steps

- **Swap the `IEstimator`**: change `internal_sensor_feed.cpp`'s
  `sf::ComplementaryEstimator attitude_estimator;` to a different
  `IEstimator` implementation — `main.cpp`'s L1 code (`printRollPitchYaw()`)
  needs zero changes. This is what `architecture.md` §2.5's "replaceable
  design" means in practice; see the planned `22_custom_estimator` example
  in `docs/coding_and_education.md` §3.
- **Swap the `IController`**: the same idea on the control side is in
  `examples/10_custom_controller`.
- **Design docs**: `docs/architecture.md` §2.5 "学習者の入口（4階層アクセス）",
  `docs/topic_reference.md` §4 "使用パターン".
