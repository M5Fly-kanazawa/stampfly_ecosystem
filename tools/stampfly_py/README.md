# StampFly × Tello SDK Compatibility

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

StampFly の `vehicle` ファームは **DJI Tello SDK プロトコル**を話します。これにより、
**既存の Tello 用 Python プログラム（特に `djitellopy` ライブラリ）をほとんどそのまま** StampFly で
動かせます。カメラが無いため映像系だけは動きませんが、それ以外（離陸・着陸・移動・回転・連続
マニュアル操作・各種状態取得）は動きます。

| ファイル | 説明 |
|---------|------|
| `example_djitellopy.py` | 純正 `djitellopy` を無改変で使う例（推奨の入口）。50cm×4辺・90°ターンの正方形飛行 |
| `example_djitellopy2.py` | 同上のバリエーション。離陸後20秒ホバー待機＋30cm×3辺・180°ターンのジグザグ飛行 |
| `stampfly.py` | 依存ゼロの軽量 Tello 風クライアント（`djitellopy` を入れたくない場合）|
| `example_square.py` | `stampfly.py` を使う 50cm 四方飛行の例 |

### なぜそのまま動くのか

- ファームは **UDP:8889** で Tello テキストコマンドを受け、**UDP:8890** で状態文字列を ~10Hz 送出する。
  `djitellopy` の `connect()` は 8890 の状態パケットが届くことを必須とし、`get_*()` はこれを読む。
- SoftAP の IP を **192.168.10.1**（実機 Tello と同じサブネット = `djitellopy` の既定ホスト）にした。
  そのため `Tello()` を**引数なし**で繋げる。

## 2. 使い方

### djitellopy（推奨）

```bash
pip install djitellopy
# 機体を起動・静置 → Ready 音（静止校正）→ 送信機を中立で ON のまま（安全パイロット）
# 機体の WiFi AP（SSID: StampFly-XXXX）に PC を接続（PC は 192.168.10.x を取得）
python3 example_djitellopy.py
```

```python
from djitellopy import Tello
tello = Tello()            # 既定ホスト 192.168.10.1 = StampFly。無改変
tello.connect()
print(tello.get_battery())
tello.takeoff()
tello.move_forward(50)
tello.rotate_clockwise(90)
tello.land()
```

### STA モード（共有ルータ経由）

```python
tello = Tello(host="192.168.1.42")   # 機体の LAN IP を渡す
```

## 3. 対応コマンド一覧

| カテゴリ | コマンド | 状態 |
|---------|---------|------|
| 制御 | `command` `takeoff` `land` `emergency` `stop` | ✅ |
| 移動 | `up` `down` `left` `right` `forward` `back`（cm）| ✅ ブロック（到達まで）|
| 回転 | `cw` `ccw`（度）| ✅ ブロック（到達まで）|
| 絶対移動 | `go x y z speed` | ✅ |
| 設定 | `speed x`（巡航速度 cm/s, set_speed）| ✅ verb 移動の既定速度に反映 |
| マニュアル | `rc a b c d`（連続操作 = `send_rc_control`）| ✅ POS_HOLD 速度指令へ橋渡し（送信停止で保持・スティックで即解除）|
| 読み取り | `battery?` `height?` `attitude?` `speed?` `time?` `tof?` `temp?` `baro?` `acceleration?` `sdk?` `sn?` `wifi?` | ✅ |
| 状態ストリーム | UDP:8890（`get_battery`/`get_height`/`get_distance_tof` 等が読む）| ✅ ~10Hz |
| カメラ | `streamon` `streamoff` | ⚠️ ok を返すが映像は出ない（カメラ無し）|
| 宙返り | `flip` | ❌ `error`（小型機で高リスクのため非対応）|
| 円弧 | `curve` | ❌ `error`（未実装）|
| ミッションパッド | `mon` `moff` `mdirection` | ❌ `error`（EDU 専用機能）|

## 4. 安全・注意

- **送信機（プロポ）を安全装置として併用する。** ペアリング済み送信機を中立で ON のまま保持。PC の
  プログラムで飛ばすが、**スティックを動かせばいつでもパイロットが即介入・停止できる**（パイロット優先）。
- `emergency()` はどの状態でも即時モータ停止。
- **`sf log wifi` と同時に実行しない。** どちらも PC 側で UDP:8890 を使う。
- 屋内安全の上限（≈3m）を超える移動はその上限へクランプされ、応答は `ok`（プログラムは
  止まらず移動量が減るだけ）。巡航速度は `speed x` で変えられる。

---

<a id="english"></a>

## 1. Overview

### About This Directory

StampFly's `vehicle` firmware speaks the **DJI Tello SDK protocol**, so existing
Tello Python programs (especially the `djitellopy` library) run on StampFly **nearly
as-is**. There is no camera, so video is the one unsupported feature; everything else
(takeoff, land, moves, rotation, continuous manual control, telemetry getters) works.

| File | Description |
|------|-------------|
| `example_djitellopy.py` | Uses the stock `djitellopy` unchanged (recommended entry point). 50cm x4 square, 90 deg turns |
| `example_djitellopy2.py` | Variant of the above: 20s hover pause after takeoff, then a 30cm x3 zigzag with 180 deg turns |
| `stampfly.py` | Dependency-free lightweight Tello-style client (if you'd rather not install `djitellopy`) |
| `example_square.py` | 50 cm square flight using `stampfly.py` |

### Why it runs unchanged

- The firmware accepts Tello text commands on **UDP:8889** and pushes a state string on
  **UDP:8890** at ~10 Hz. `djitellopy`'s `connect()` requires a state packet on 8890, and
  every `get_*()` reads it.
- The SoftAP is addressed at **192.168.10.1** (the real Tello subnet = `djitellopy`'s
  default host), so `Tello()` connects with **no arguments**.

## 2. Usage

```bash
pip install djitellopy
# Power on, place still, wait for the ready chime. Keep the paired RC ON, sticks
# neutral (it is the safety pilot). Join the vehicle's WiFi AP (StampFly-XXXX).
python3 example_djitellopy.py
```

For STA mode: `Tello(host="<vehicle-LAN-ip>")`.

## 3. Supported Commands

| Category | Commands | Status |
|----------|----------|--------|
| Control | `command` `takeoff` `land` `emergency` `stop` | ✅ |
| Move | `up` `down` `left` `right` `forward` `back` (cm) | ✅ blocks until reached |
| Rotate | `cw` `ccw` (deg) | ✅ blocks until reached |
| Absolute | `go x y z speed` | ✅ |
| Set | `speed x` (cruise speed cm/s, set_speed) | ✅ applies to the verb moves' default speed |
| Manual | `rc a b c d` (continuous = `send_rc_control`) | ✅ bridged to POS_HOLD velocity (holds when the stream stops; pilot stick cancels instantly) |
| Read | `battery?` `height?` `attitude?` `speed?` `time?` `tof?` `temp?` `baro?` `acceleration?` `sdk?` `sn?` `wifi?` | ✅ |
| State stream | UDP:8890 (read by `get_battery`/`get_height`/`get_distance_tof`...) | ✅ ~10 Hz |
| Camera | `streamon` `streamoff` | ⚠️ returns ok, no video (no camera) |
| Flip | `flip` | ❌ `error` (unsafe on this small craft) |
| Curve | `curve` | ❌ `error` (not implemented) |
| Mission pads | `mon` `moff` `mdirection` | ❌ `error` (EDU-only feature) |

## 4. Safety & Notes

- **Keep the RC transmitter ON as a safety device.** Hold the paired transmitter with
  neutral sticks. You fly from the PC program, but **moving any stick lets the pilot
  intervene/stop instantly** (the pilot always wins).
- `emergency()` cuts the motors immediately in any state.
- **Do not run `sf log wifi` at the same time** — both use UDP:8890 on the PC.
- A move longer than the indoor-safe ceiling (≈3 m) is clamped to that ceiling and
  still replies `ok` (the program keeps running; the craft just travels less).
