# 実機テストプラン — 1月末以降の未検証機能を段階的に検証

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 背景

1月26日の hooks celebration jump テスト以降、約19日間ハードウェアテストが行われていない。
この間に以下の重要な変更が積み上がっている：

| カテゴリ | コミット期間 | リスク | 概要 |
|---------|------------|--------|------|
| SystemStateManager リファクタ | 1/26 | 中-高 | 状態管理の大幅リファクタ、ブート時クラッシュ修正歴あり |
| ALTITUDE_HOLD モード | 2/11-12 | **高** | カスケード PID + Vbat 補正、完全未テスト |
| POSITION_HOLD モード | 2/12 | **高** | カスケード PID + 位置制御、完全未テスト |
| Tello 互換移動コマンド | 2/13-14 | 中 | up/down/forward/back/left/right/cw/ccw ハンドラ |
| RC 制御コマンド | 2/14 | 中 | `rc a b c d` WiFi 経由リアルタイム入力 |
| テレメトリクエリ | 2/13 | 低 | battery?/height?/tof? 等の読み取り専用コマンド |
| Python SDK | 2/14 | 低 | ソフトウェアのみ、ファームウェア変更なし |

### 方針

安全な非飛行テストから始め、段階的にリスクを上げる。各ステージに Go/No-Go 判定を設け、失敗時は先に進まない。

### 前提条件

- StampFly 機体が正常（プロペラ破損なし、バッテリー充電済み）
- ESP-IDF 環境セットアップ済み
- Python SDK インストール済み（`pip install -e .`）
- テスト環境: 広い室内、柔らかい床面（クラッシュ対策）

## 2. Stage 0: ビルド＆書き込み（飛行なし）

### 目的

ファームウェアがコンパイル・起動できることを確認。

### 手順

```bash
source setup_env.sh
sf build vehicle
sf flash vehicle -m   # シリアルモニタでブートログ確認
```

### 確認項目

- [ ] ビルドエラーなし
- [ ] ブートログに `INIT → CALIBRATING → IDLE` 遷移が表示される
- [ ] パニック/リブートループなし（SystemStateManager 初期化順序問題の再発確認）
- [ ] WiFi AP `StampFly_XXXX` が出現する
- [ ] センサー初期化ログ（IMU, ToF, Baro, OptFlow）にエラーなし

### Go/No-Go

全項目クリアで Stage 1 へ。ブートクラッシュ発生時はシリアルログから原因特定。

## 3. Stage 1: WiFi 接続 + テレメトリクエリ（飛行なし）

### 目的

通信レイヤー（TCP CLI + WebSocket）の動作確認。

### 1a. sf CLI でクエリ

```bash
# StampFly WiFi AP に接続後
sf battery       # バッテリー残量
sf height        # 高度（地上: 0付近）
sf tof           # ToF距離（地上: 3-10cm程度）
sf baro          # 気圧高度
sf attitude      # 姿勢（静置: P≈0 R≈0 Y=任意）
sf acceleration  # 加速度（静置: X≈0 Y≈0 Z≈-981）
```

### 確認項目（1a）

- [ ] 各コマンドが応答を返す（タイムアウトしない）
- [ ] 値が物理的に妥当（バッテリー 0-100%, 加速度 Z ≈ -981 cm/s²）

### 1b. Python SDK で接続 + クエリ

```python
from stampfly import StampFly
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")
print(f"Height:  {drone.get_height()} cm")
print(f"ToF:     {drone.get_distance_tof()} cm")
print(f"Baro:    {drone.get_barometer()} cm")
print(f"Att:     {drone.get_attitude()}")
print(f"Accel:   {drone.get_acceleration()}")
print(f"Speed:   {drone.get_speed()} cm/s")

# WebSocket テレメトリ
import time
time.sleep(2)  # テレメトリ受信待ち
telem = drone.get_telemetry()
print(f"Telemetry keys: {list(telem.keys())}")
print(f"Telemetry: {telem}")

drone.end()
```

### 確認項目（1b）

- [ ] `drone.connect()` が例外なく完了する
- [ ] 全7クエリが値を返す
- [ ] `get_telemetry()` が空でない dict を返す
- [ ] `drone.end()` が正常終了する（スレッドハング等なし）

### 1c. RC 制御コマンド（モーターOFF状態で）

```python
drone = StampFly()
drone.connect()

# ゼロ入力（ニュートラル）
drone.send_rc_control(0, 0, 0, 0)
print("RC zero sent")

# 小さい値
drone.send_rc_control(10, 0, 0, 0)
print("RC small value sent")

drone.end()
```

### 確認項目（1c）

- [ ] コマンドが例外なく完了する
- [ ] シリアルログに RC 入力受信が記録される（モーターは回らない: DISARM 状態のため）

### Go/No-Go

全クエリ正常 + SDK 接続/切断正常で Stage 2 へ。

## 4. Stage 2: Jump コマンド（最小リスク飛行）

### 目的

最も単純な飛行コマンド（ToF 直読み P 制御）で基本的な飛行能力を確認。

Jump は ESKF を使わず ToF 生値で制御するため、状態推定のバグに影響されにくい。
既に1月26日に成功実績がある（ただし SystemStateManager リファクタ後は未テスト）。

### 2a. sf CLI で Jump

```bash
sf jump         # デフォルト 15cm
```

### 確認項目（2a）

- [ ] Auto-ARM が機能する（プロペラ回転開始）
- [ ] 10-20cm 程度浮上する
- [ ] 自動着陸する
- [ ] DISARM に戻る
- [ ] シリアルログに `JUMP: INIT → CLIMBING → DESCENDING → DONE` が表示される

### 2b. Python SDK で Jump

```python
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")
drone.jump(0.15)  # 15cm
print("Jump completed")
print(f"Height after: {drone.get_height()} cm")

drone.end()
```

### 確認項目（2b）

- [ ] `drone.jump()` がブロッキングで完了を待つ
- [ ] 飛行後のクエリが正常に動作する（接続が切れていない）

### トラブルシューティング

- モーターが回らない → シリアルログで ARM 状態確認。`flight status` コマンドで状態確認
- 高度オーバーシュート → HOVER_THROTTLE (0.60) と KP (0.8) の調整が必要
- クラッシュ/暴走 → **即座に機体を裏返す**（プロペラ保護）

### Go/No-Go

Jump が安定動作で Stage 3 へ。オーバーシュートが ±5cm 以内であること。

## 5. Stage 3: Takeoff → Hover → Land（ALTITUDE_HOLD テスト）

### 目的

ALTITUDE_HOLD カスケード PID の初回検証。これが最大のリスクポイント。

### 重要な注意事項

ALTITUDE_HOLD は完全未テスト。PID ゲインはデスクチューニング値：

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| 外側ループ KP | 2.0 | 高度→速度 |
| 外側ループ TI | 3.0 | 高度→速度（積分時定数） |
| 外側ループ TD | 0.1 | 高度→速度（微分時定数） |
| 内側ループ KP | 0.3 | 速度→スラスト |
| 内側ループ TI | 1.0 | 速度→スラスト（積分時定数） |
| 内側ループ TD | 0.02 | 速度→スラスト（微分時定数） |
| ホバースラスト | ≈ 0.57 | mass(0.035) × g(9.81) / MAX_THRUST(0.60) × correction(1.0) |

発散や振動の可能性がある。**手を伸ばせば届く高度**（0.3m以下）でテスト。

### 3a. sf CLI で Takeoff → Land（最短）

```bash
sf takeoff 0.3      # 30cm で離陸（デフォルト 50cm より低く）
# 浮上したら数秒観察
sf land
```

### 確認項目（3a）

- [ ] 離陸して概ね 30cm 付近でホバリングする
- [ ] 高度が振動していない（±10cm 以内）
- [ ] Land で安全に着陸する
- [ ] シリアルログの ESKF 高度推定値が ToF と概ね一致する

### 3b. Hover コマンドで安定性評価

```bash
sf hover 30 10      # 30cm で 10秒間ホバリング
```

### 確認項目（3b）

- [ ] 10秒間の高度維持が安定（視覚的に大きな上下動がない）

### 3c. Python SDK で一連のフロー

```python
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")

drone.takeoff()   # デフォルト 50cm
import time
time.sleep(3)

# ホバリング中のテレメトリ確認
telem = drone.get_telemetry()
print(f"Hovering - ToF: {drone.get_distance_tof()} cm")
print(f"Hovering - Height: {drone.get_height()} cm")
print(f"Hovering - Attitude: {drone.get_attitude()}")

drone.land()
print("Landing completed")

drone.end()
```

### トラブルシューティング

| 症状 | 対応 |
|------|------|
| 高度が発振（上下に激しく揺れる） | `sf emergency` で即時停止。ALT_KP を下げる（2.0 → 1.0） |
| 上昇が止まらない | `sf emergency`。Takeoff のスラスト上限 (0.95) が高すぎる可能性 |
| 急降下 | ホバースラスト値が低すぎる。HOVER_THRUST_CORRECTION を 1.0 → 1.2 に上げる |

### Go/No-Go

Hover が ±15cm 以内で安定していれば Stage 4 へ。

## 6. Stage 4: 垂直移動コマンド（Up/Down）

### 目的

ALTITUDE_HOLD ベースの相対高度移動を検証。

### 手順

```python
drone = StampFly()
drone.connect()

drone.takeoff()

# 最小距離でテスト
drone.move_up(20)    # 20cm 上昇
print(f"After up: {drone.get_height()} cm")

drone.move_down(20)  # 20cm 下降
print(f"After down: {drone.get_height()} cm")

drone.land()
drone.end()
```

### 確認項目

- [ ] `move_up(20)` で概ね 20cm 上昇する
- [ ] `move_down(20)` で概ね 20cm 下降する
- [ ] 30秒タイムアウト内にコマンドが完了する
- [ ] コマンド間のトランジションがスムーズ

### Go/No-Go

垂直移動が概ね意図通りなら Stage 5 へ。

## 7. Stage 5: 回転コマンド（CW/CCW）

### 目的

ヨー角制御の検証。位置制御なしで安全。

### 手順

```python
drone = StampFly()
drone.connect()

drone.takeoff()

drone.rotate_clockwise(90)        # 90度右回転
print(f"After CW: {drone.get_attitude()}")

drone.rotate_counter_clockwise(90) # 90度左回転（元に戻る）
print(f"After CCW: {drone.get_attitude()}")

drone.land()
drone.end()
```

### 確認項目

- [ ] 回転方向が正しい（CW = 時計回り = ヨー角増加）
- [ ] 概ね 90度回転する（±20度は許容）
- [ ] 回転中に高度が大きく変化しない

## 8. Stage 6: 水平移動コマンド（POSITION_HOLD テスト）

### 目的

POSITION_HOLD カスケード PID の検証。Stage 3 の次に高リスク。

### 重要な注意事項

POSITION_HOLD も完全未テスト。オプティカルフロー品質に強く依存。

| パラメータ | 値 |
|-----------|-----|
| 位置制御 POS_KP | 1.0 |
| 速度制御 VEL_KP | 0.3 |
| 最大傾斜 | 10度（0.1745 rad） |
| 最大水平速度 | 0.5 m/s |

**テスト環境:** テクスチャのある床面（フローリング、マット等）。白い無地の床は避ける。

### 6a. 最小距離で前進のみ

```python
drone = StampFly()
drone.connect()

drone.takeoff()

import time
time.sleep(3)  # ホバリング安定待ち

drone.move_forward(20)  # 最小距離 20cm
print(f"After forward: {drone.get_height()} cm")

drone.land()
drone.end()
```

### 確認項目（6a）

- [ ] 前方に概ね移動する（方向が正しい）
- [ ] 移動中に高度が安定している
- [ ] 30秒タイムアウト内に完了する

### 6b. 4方向テスト

```python
drone = StampFly()
drone.connect()

drone.takeoff()
import time
time.sleep(3)

for direction in ['forward', 'back', 'left', 'right']:
    cmd = getattr(drone, f'move_{direction}')
    cmd(30)
    print(f"After {direction}: Height={drone.get_height()} cm")
    time.sleep(1)

drone.land()
drone.end()
```

### トラブルシューティング

| 症状 | 対応 |
|------|------|
| 移動方向が逆/斜め | フレーム変換（Body↔NED）のバグ。ヨー角オフセット確認 |
| 移動せずタイムアウト | オプティカルフロー無効 or 位置推定が機能していない |
| ドリフト/暴走 | `sf emergency`。POS_KP を下げる（1.0 → 0.5） |

## 9. Stage 7: 総合テスト（Tello 教材互換確認）

### 目的

djitellopy 教材相当のスクリプトが動作することを確認。

### 手順

```python
from stampfly import StampFly

drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")

drone.takeoff()

# 四角形を描く
# Draw a square
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)

drone.land()

print(f"Battery after: {drone.get_battery()}%")
drone.end()
```

### 確認項目

- [ ] 一連のシーケンスが最後まで完走する
- [ ] 概ね四角形の軌跡を描く
- [ ] バッテリーレベルが妥当な減少を示す

## 10. 緊急時手順

全ステージ共通：

| 優先度 | 状況 | 対応 |
|--------|------|------|
| 1 | 暴走/振動 | `sf emergency` または **機体を手で裏返す** |
| 2 | 高度上昇が止まらない | バッテリーを抜く（最終手段） |
| 3 | WiFi 切断 | 500ms タイムアウト後、制御入力がゼロになる（ホバリング or 降下） |
| 4 | Python SDK がハング | Ctrl+C で中断、別ターミナルから `sf emergency` |

## 11. 既知のリスクと対策

| リスク | 影響 | 対策 |
|--------|------|------|
| ALTITUDE_HOLD PID 発散 | 高度暴走 | Stage 3 で低高度 (30cm) から開始 |
| POSITION_HOLD PID 発散 | 水平暴走 | Stage 6 で最小距離 (20cm) から開始 |
| ホバースラスト不一致 | 浮力不足/過剰 | Jump (Stage 2) の結果でスラスト値推定 |
| SystemStateManager 初期化順序 | ブート時クラッシュ | Stage 0 で確認、既に修正済みだが要検証 |
| オプティカルフロー品質 | 位置推定不良 | テクスチャのある床面でテスト |
| 30秒タイムアウト | 長距離移動が完了しない | まず短距離でテスト |

## 12. テスト後の記録

各ステージ完了後に記録すべき情報：

- シリアルログ（`sf monitor` の出力）
- バッテリー電圧の推移
- 発見したバグと再現手順
- PID ゲイン調整が必要だった場合の変更値

---

<a id="english"></a>

# Hardware Test Plan — Staged Verification of Features Since Late January

## 1. Overview

### Background

No hardware testing has been performed for approximately 19 days since the hooks celebration jump test on January 26.
The following significant changes have accumulated during this period:

| Category | Commit Period | Risk | Summary |
|----------|--------------|------|---------|
| SystemStateManager Refactor | 1/26 | Med-High | Major state management refactor, boot crash fix history |
| ALTITUDE_HOLD Mode | 2/11-12 | **High** | Cascade PID + Vbat compensation, completely untested |
| POSITION_HOLD Mode | 2/12 | **High** | Cascade PID + position control, completely untested |
| Tello-Compatible Move Commands | 2/13-14 | Med | up/down/forward/back/left/right/cw/ccw handlers |
| RC Control Command | 2/14 | Med | `rc a b c d` real-time input via WiFi |
| Telemetry Queries | 2/13 | Low | battery?/height?/tof? read-only commands |
| Python SDK | 2/14 | Low | Software only, no firmware changes |

### Approach

Start with safe non-flight tests and gradually increase risk. Each stage has a Go/No-Go decision point; do not proceed if a stage fails.

### Prerequisites

- StampFly unit in good condition (no propeller damage, battery charged)
- ESP-IDF environment set up
- Python SDK installed (`pip install -e .`)
- Test environment: spacious indoor area, soft floor surface (crash protection)

## 2. Stage 0: Build & Flash (No Flight)

### Purpose

Verify that firmware compiles and boots correctly.

### Procedure

```bash
source setup_env.sh
sf build vehicle
sf flash vehicle -m   # Check boot log via serial monitor
```

### Checklist

- [ ] No build errors
- [ ] Boot log shows `INIT → CALIBRATING → IDLE` transition
- [ ] No panic/reboot loop (verify SystemStateManager init order fix)
- [ ] WiFi AP `StampFly_XXXX` appears
- [ ] Sensor init logs (IMU, ToF, Baro, OptFlow) show no errors

### Go/No-Go

All items clear → proceed to Stage 1. On boot crash, identify cause from serial log.

## 3. Stage 1: WiFi Connection + Telemetry Queries (No Flight)

### Purpose

Verify communication layer (TCP CLI + WebSocket) operation.

### 1a. Queries via sf CLI

```bash
# After connecting to StampFly WiFi AP
sf battery       # Battery level
sf height        # Altitude (ground: near 0)
sf tof           # ToF distance (ground: 3-10cm)
sf baro          # Barometric altitude
sf attitude      # Attitude (stationary: P≈0 R≈0 Y=any)
sf acceleration  # Acceleration (stationary: X≈0 Y≈0 Z≈-981)
```

### Checklist (1a)

- [ ] Each command returns a response (no timeout)
- [ ] Values are physically plausible (battery 0-100%, accel Z ≈ -981 cm/s²)

### 1b. Connection + Queries via Python SDK

```python
from stampfly import StampFly
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")
print(f"Height:  {drone.get_height()} cm")
print(f"ToF:     {drone.get_distance_tof()} cm")
print(f"Baro:    {drone.get_barometer()} cm")
print(f"Att:     {drone.get_attitude()}")
print(f"Accel:   {drone.get_acceleration()}")
print(f"Speed:   {drone.get_speed()} cm/s")

# WebSocket telemetry
import time
time.sleep(2)  # Wait for telemetry reception
telem = drone.get_telemetry()
print(f"Telemetry keys: {list(telem.keys())}")
print(f"Telemetry: {telem}")

drone.end()
```

### Checklist (1b)

- [ ] `drone.connect()` completes without exception
- [ ] All 7 queries return values
- [ ] `get_telemetry()` returns a non-empty dict
- [ ] `drone.end()` terminates cleanly (no thread hang)

### 1c. RC Control Commands (Motors OFF)

```python
drone = StampFly()
drone.connect()

# Zero input (neutral)
drone.send_rc_control(0, 0, 0, 0)
print("RC zero sent")

# Small value
drone.send_rc_control(10, 0, 0, 0)
print("RC small value sent")

drone.end()
```

### Checklist (1c)

- [ ] Commands complete without exception
- [ ] Serial log shows RC input received (motors don't spin: DISARM state)

### Go/No-Go

All queries normal + SDK connect/disconnect normal → proceed to Stage 2.

## 4. Stage 2: Jump Command (Minimum Risk Flight)

### Purpose

Verify basic flight capability with the simplest flight command (ToF direct-read P control).

Jump uses raw ToF values instead of ESKF, so it's less affected by state estimation bugs.
Successfully tested on January 26 (but untested after SystemStateManager refactor).

### 2a. Jump via sf CLI

```bash
sf jump         # Default 15cm
```

### Checklist (2a)

- [ ] Auto-ARM functions (propellers start spinning)
- [ ] Rises approximately 10-20cm
- [ ] Auto-lands
- [ ] Returns to DISARM
- [ ] Serial log shows `JUMP: INIT → CLIMBING → DESCENDING → DONE`

### 2b. Jump via Python SDK

```python
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")
drone.jump(0.15)  # 15cm
print("Jump completed")
print(f"Height after: {drone.get_height()} cm")

drone.end()
```

### Checklist (2b)

- [ ] `drone.jump()` blocks until completion
- [ ] Post-flight queries work normally (connection not lost)

### Troubleshooting

- Motors don't spin → Check ARM state in serial log. Use `flight status` command
- Altitude overshoot → Adjust HOVER_THROTTLE (0.60) and KP (0.8)
- Crash/runaway → **Immediately flip the unit upside down** (propeller protection)

### Go/No-Go

Jump operates stably → proceed to Stage 3. Overshoot must be within ±5cm.

## 5. Stage 3: Takeoff → Hover → Land (ALTITUDE_HOLD Test)

### Purpose

First verification of ALTITUDE_HOLD cascade PID. This is the highest risk point.

### Important Notes

ALTITUDE_HOLD is completely untested. PID gains are desk-tuned values:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Outer Loop KP | 2.0 | Altitude → Velocity |
| Outer Loop TI | 3.0 | Altitude → Velocity (integral time) |
| Outer Loop TD | 0.1 | Altitude → Velocity (derivative time) |
| Inner Loop KP | 0.3 | Velocity → Thrust |
| Inner Loop TI | 1.0 | Velocity → Thrust (integral time) |
| Inner Loop TD | 0.02 | Velocity → Thrust (derivative time) |
| Hover Thrust | ≈ 0.57 | mass(0.035) × g(9.81) / MAX_THRUST(0.60) × correction(1.0) |

Divergence and oscillation are possible. **Test at heights reachable by hand** (0.3m or below).

### 3a. Takeoff → Land via sf CLI (Shortest)

```bash
sf takeoff 0.3      # Takeoff at 30cm (lower than default 50cm)
# Observe for a few seconds after rising
sf land
```

### Checklist (3a)

- [ ] Takes off and hovers near 30cm
- [ ] Altitude is not oscillating (within ±10cm)
- [ ] Lands safely with Land command
- [ ] ESKF altitude estimate in serial log roughly matches ToF

### 3b. Stability Evaluation with Hover Command

```bash
sf hover 30 10      # Hover at 30cm for 10 seconds
```

### Checklist (3b)

- [ ] Altitude maintained stably for 10 seconds (no visually significant vertical oscillation)

### 3c. Full Flow via Python SDK

```python
drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")

drone.takeoff()   # Default 50cm
import time
time.sleep(3)

# Check telemetry during hover
telem = drone.get_telemetry()
print(f"Hovering - ToF: {drone.get_distance_tof()} cm")
print(f"Hovering - Height: {drone.get_height()} cm")
print(f"Hovering - Attitude: {drone.get_attitude()}")

drone.land()
print("Landing completed")

drone.end()
```

### Troubleshooting

| Symptom | Action |
|---------|--------|
| Altitude oscillates (violent up/down) | `sf emergency` for immediate stop. Lower ALT_KP (2.0 → 1.0) |
| Keeps climbing | `sf emergency`. Takeoff thrust cap (0.95) may be too high |
| Sudden descent | Hover thrust too low. Increase HOVER_THRUST_CORRECTION 1.0 → 1.2 |

### Go/No-Go

Hover stable within ±15cm → proceed to Stage 4.

## 6. Stage 4: Vertical Move Commands (Up/Down)

### Purpose

Verify ALTITUDE_HOLD-based relative altitude movement.

### Procedure

```python
drone = StampFly()
drone.connect()

drone.takeoff()

# Test with minimum distance
drone.move_up(20)    # Rise 20cm
print(f"After up: {drone.get_height()} cm")

drone.move_down(20)  # Descend 20cm
print(f"After down: {drone.get_height()} cm")

drone.land()
drone.end()
```

### Checklist

- [ ] `move_up(20)` rises approximately 20cm
- [ ] `move_down(20)` descends approximately 20cm
- [ ] Command completes within 30-second timeout
- [ ] Smooth transition between commands

### Go/No-Go

Vertical movement works as intended → proceed to Stage 5.

## 7. Stage 5: Rotation Commands (CW/CCW)

### Purpose

Verify yaw angle control. Safe without position control.

### Procedure

```python
drone = StampFly()
drone.connect()

drone.takeoff()

drone.rotate_clockwise(90)        # 90° right rotation
print(f"After CW: {drone.get_attitude()}")

drone.rotate_counter_clockwise(90) # 90° left rotation (return to original)
print(f"After CCW: {drone.get_attitude()}")

drone.land()
drone.end()
```

### Checklist

- [ ] Rotation direction is correct (CW = clockwise = yaw angle increase)
- [ ] Rotates approximately 90° (±20° tolerance)
- [ ] Altitude doesn't change significantly during rotation

## 8. Stage 6: Horizontal Move Commands (POSITION_HOLD Test)

### Purpose

Verify POSITION_HOLD cascade PID. Second highest risk after Stage 3.

### Important Notes

POSITION_HOLD is also completely untested. Heavily dependent on optical flow quality.

| Parameter | Value |
|-----------|-------|
| Position Control POS_KP | 1.0 |
| Velocity Control VEL_KP | 0.3 |
| Max Tilt | 10° (0.1745 rad) |
| Max Horizontal Speed | 0.5 m/s |

**Test environment:** Textured floor surface (hardwood, mat, etc.). Avoid plain white floors.

### 6a. Forward Only at Minimum Distance

```python
drone = StampFly()
drone.connect()

drone.takeoff()

import time
time.sleep(3)  # Wait for hover stabilization

drone.move_forward(20)  # Minimum distance 20cm
print(f"After forward: {drone.get_height()} cm")

drone.land()
drone.end()
```

### Checklist (6a)

- [ ] Moves roughly forward (direction is correct)
- [ ] Altitude stable during movement
- [ ] Completes within 30-second timeout

### 6b. Four-Direction Test

```python
drone = StampFly()
drone.connect()

drone.takeoff()
import time
time.sleep(3)

for direction in ['forward', 'back', 'left', 'right']:
    cmd = getattr(drone, f'move_{direction}')
    cmd(30)
    print(f"After {direction}: Height={drone.get_height()} cm")
    time.sleep(1)

drone.land()
drone.end()
```

### Troubleshooting

| Symptom | Action |
|---------|--------|
| Movement direction wrong/diagonal | Frame transform (Body↔NED) bug. Check yaw offset |
| No movement, timeout | Optical flow disabled or position estimation not working |
| Drift/runaway | `sf emergency`. Lower POS_KP (1.0 → 0.5) |

## 9. Stage 7: Integration Test (Tello Curriculum Compatibility)

### Purpose

Verify that scripts equivalent to djitellopy curriculum work correctly.

### Procedure

```python
from stampfly import StampFly

drone = StampFly()
drone.connect()

print(f"Battery: {drone.get_battery()}%")

drone.takeoff()

# Draw a square
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.rotate_clockwise(90)

drone.land()

print(f"Battery after: {drone.get_battery()}%")
drone.end()
```

### Checklist

- [ ] Full sequence completes to the end
- [ ] Roughly traces a square trajectory
- [ ] Battery level shows reasonable decrease

## 10. Emergency Procedures

Common to all stages:

| Priority | Situation | Action |
|----------|-----------|--------|
| 1 | Runaway/oscillation | `sf emergency` or **flip the unit upside down by hand** |
| 2 | Altitude keeps rising | Disconnect battery (last resort) |
| 3 | WiFi disconnection | After 500ms timeout, control input zeroes out (hover or descend) |
| 4 | Python SDK hangs | Ctrl+C to interrupt, `sf emergency` from another terminal |

## 11. Known Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| ALTITUDE_HOLD PID divergence | Altitude runaway | Start at low altitude (30cm) in Stage 3 |
| POSITION_HOLD PID divergence | Horizontal runaway | Start at minimum distance (20cm) in Stage 6 |
| Hover thrust mismatch | Insufficient/excessive lift | Estimate thrust value from Jump (Stage 2) results |
| SystemStateManager init order | Boot crash | Verify in Stage 0, already fixed but needs confirmation |
| Optical flow quality | Poor position estimation | Test on textured floor surface |
| 30-second timeout | Long-distance moves don't complete | Test short distances first |

## 12. Post-Test Records

Information to record after completing each stage:

- Serial log (`sf monitor` output)
- Battery voltage trends
- Discovered bugs and reproduction steps
- PID gain adjustments if needed
