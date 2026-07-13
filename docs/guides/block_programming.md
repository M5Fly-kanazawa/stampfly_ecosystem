# StampFly ブロックプログラミングガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このガイドについて

`sf blocks` は、ブラウザ上でブロック（部品）を並べて StampFly を飛ばせる Web UI（Blockly ベース）です。
文字でコマンドを打つ代わりに「離陸する」「前に 50cm 進む」のようなブロックをドラッグ＆ドロップで
組み合わせ、「実行（Run）」ボタン一つで機体に送信します。

### 対象読者

プログラミング入門者・初めてドローンに触れる生徒・授業で StampFly を使う教員を想定しています。
テキストでコマンドを打つ `tools/stampfly_py/`（Tello 互換 Python SDK）よりも前段階の、ノーコードに
近い操作方法です。

## 2. 必要なもの

| 項目 | 備考 |
|------|------|
| StampFly 実機 | プロペラガード装着済みであること |
| 送信機（プロポ） | 安全担当者が中立スティックで保持する（必須、詳細は「5. 安全上の注意」）|
| sf CLI 導入済み PC | `source setup_env.sh` 済みの ESP-IDF 開発環境 |

**インターネット接続は不要です。** Blockly 本体（JavaScript ライブラリ）は sf CLI に同梱されており、
教室が StampFly の WiFi アクセスポイントに接続されている（＝一般的なインターネットから切り離されて
いる）オフライン環境でもそのまま動作します。

## 3. 機体側の準備

`sf blocks` は StampFly の Tello 互換 API（UDP:8889/8890）を使うため、機体を SoftAP モード（機体自身
が WiFi アクセスポイントになるモード）にしておく必要があります。

```bash
# 機体に USB 接続した状態で（1回だけ設定すればよい）
sf monitor
# シリアルコンソール上で:
param set wifi.mode 1     # 1 = SoftAP モード
param save
# その後、機体を再起動（電源off/on）
```

設定後、機体の電源を入れると SSID `StampFly-XXYY`（XXYY は機体固有の識別子）の WiFi アクセスポイント
が立ち上がります。既定パスワードは `stampfly` です。PC 側でこの WiFi に接続してください。

| 項目 | 値 |
|------|-----|
| SSID | `StampFly-XXYY` |
| パスワード（既定） | `stampfly` |
| 機体アドレス | `192.168.10.1`（既定） |

**API 経由の飛行は POS_HOLD モード（位置保持モード）で動作します。** 送信機側のフライトモードは
POS_HOLD を選んでおいてください。

## 4. 使い方

### 起動

```bash
source setup_env.sh

# 実機に接続する場合
sf blocks

# 機体なしで練習する場合（デモモード、UDP通信なし）
sf blocks --demo
```

起動するとブラウザが自動的に開きます（`sf telemetry --web` と同様の挙動）。開かない場合は
`http://127.0.0.1:5007` を手動で開いてください。

### 操作の流れ

1. 画面の「接続（Connect）」ボタンで機体に接続する（デモモードでは常に成功する）
2. 左側のブロック一覧からブロックをドラッグしてワークスペースに並べる
3. 「実行（Run）」ボタンで、並べたブロックを上から順に機体へ送信する
4. 途中で止めたい場合は「停止（Stop）」、危険を感じたら「緊急停止（Emergency）」を押す

### ブロック一覧

| ブロック | 内容 | 引数の範囲 |
|---------|------|-----------|
| 離陸する | `takeoff` を送信 | — |
| 着陸する | `land` を送信 | — |
| 前 / 後 / 左 / 右 / 上 / 下に進む | `forward`/`back`/`left`/`right`/`up`/`down` | 距離 10〜300 cm |
| 右 / 左に回転する | `cw`/`ccw` | 角度 1〜360 度 |
| N 秒待つ | 機体には何も送らずプログラムの進行を止める | 範囲 0.1〜30 秒 |
| 速度を N cm/s にする | `speed` を送信 | 10〜100 cm/s |
| くり返す | 内側のブロックを指定回数くり返す | — |

## 5. 安全上の注意

- **送信機を持った安全担当者を必ず配置してください。** `sf blocks` はブラウザから機体に指令を送る
  だけのツールで、PC の WiFi 接続が切れても機体は自動着陸しません。送信機のスティックはいつでも
  操作を上書きできるため、異常時はスティックで即座に介入してください（`tools/stampfly_py/README.md`
  の安全方針と同じ考え方です）。
- **緊急停止ボタンはモーターを即座に停止させます。** 空中で押すと機体はその場で落下しますので、
  本当に危険なとき以外は使わないでください。通常の停止には「停止（Stop）」ブロック／ボタンを
  使ってください。
- **現時点では位置制御の精度に限界があります。** コマンド自体は受け付けられますが、指定した距離
  や角度どおりに正確に動くとは限りません（POS_HOLD の精度改善は継続的に進行中です）。まずは
  広く障害物のない場所で、少ないブロック数（2〜3個程度）から試してください。

## 6. トラブルシューティング

| 症状 | 確認すること |
|------|-------------|
| 「接続」を押しても接続できない | PC が機体の WiFi（`StampFly-XXYY`）に接続されているか。`wifi.mode` が 1（SoftAP）になっているか。機体の電源が入っているか |
| コマンドを送るとエラーになる | 機体が POS_HOLD モードになっているか（送信機側のモード切替を確認）。バッテリー残量が十分か |
| 途中で応答が止まる／タイムアウトする | WiFi の電波状況（機体との距離・障害物）を確認。他のアプリ（`sf log wifi`、djitellopy スクリプト等）が同時に UDP:8890 を使っていないか確認 |

---

<a id="english"></a>

## 1. Overview

### About This Guide

`sf blocks` is a Blockly-based web UI that lets you fly StampFly by arranging blocks
in a browser instead of typing text commands. Drag and drop blocks such as "take off"
or "move forward 50cm" into a workspace, then press "Run" to send them to the vehicle.

### Target Audience

Programming beginners, students trying a drone for the first time, and instructors
using StampFly in class. It sits one step before the text-based
`tools/stampfly_py/` (Tello-compatible Python SDK) — closer to no-code.

## 2. Requirements

| Item | Note |
|------|------|
| StampFly vehicle | Propeller guards must be fitted |
| RC transmitter | Held by a safety pilot with neutral sticks (mandatory, see "5. Safety Notes") |
| PC with sf CLI | ESP-IDF dev environment with `source setup_env.sh` run |

**No internet connection is required.** The Blockly library itself ships with the sf
CLI, so it works fully offline in a classroom whose PCs are joined to the vehicle's
WiFi access point (and therefore disconnected from the general internet).

## 3. Preparing the Vehicle

`sf blocks` talks to StampFly's Tello-compatible API (UDP:8889/8890), so the vehicle
must be in SoftAP mode (the vehicle itself becomes a WiFi access point).

```bash
# With the vehicle connected via USB (one-time setup)
sf monitor
# In the serial console:
param set wifi.mode 1     # 1 = SoftAP mode
param save
# Then power-cycle the vehicle
```

After this, powering on the vehicle brings up a WiFi access point with SSID
`StampFly-XXYY` (XXYY is a per-vehicle identifier). The default password is
`stampfly`. Join this WiFi network from the PC.

| Item | Value |
|------|-------|
| SSID | `StampFly-XXYY` |
| Password (default) | `stampfly` |
| Vehicle address | `192.168.10.1` (default) |

**Flight via the API runs in POS_HOLD mode.** Set the transmitter's flight mode
switch to POS_HOLD before flying.

## 4. Usage

### Starting

```bash
source setup_env.sh

# Connect to a real vehicle
sf blocks

# Practice without a vehicle (demo mode, no UDP traffic)
sf blocks --demo
```

A browser opens automatically on startup (same behavior as `sf telemetry --web`).
If it does not open, browse to `http://127.0.0.1:5007` manually.

### Workflow

1. Click "Connect" to connect to the vehicle (demo mode always succeeds)
2. Drag blocks from the palette on the left into the workspace
3. Click "Run" to send the arranged blocks to the vehicle in order
4. Use "Stop" to halt mid-sequence, or "Emergency" if something looks dangerous

### Block List

| Block | Effect | Argument range |
|-------|--------|-----------------|
| Take off | Sends `takeoff` | — |
| Land | Sends `land` | — |
| Move forward/back/left/right/up/down | `forward`/`back`/`left`/`right`/`up`/`down` | Distance 10-300 cm |
| Rotate CW/CCW | `cw`/`ccw` | Angle 1-360 deg |
| Wait N seconds | Pauses the program without sending anything | Range 0.1-30 s |
| Set speed to N cm/s | Sends `speed` | 10-100 cm/s |
| Repeat | Repeats the enclosed blocks N times | — |

## 5. Safety Notes

- **Always station a safety pilot holding the RC transmitter.** `sf blocks` only
  sends commands from a browser; the vehicle does not auto-land if the PC's WiFi
  connection drops. Transmitter sticks always override the API, so the pilot must
  be ready to intervene instantly (same policy as `tools/stampfly_py/README.md`).
- **The Emergency button cuts the motors immediately.** Pressing it in flight makes
  the vehicle drop where it is — use it only for genuine emergencies. For a normal
  halt, use the "Stop" block/button instead.
- **Position-control accuracy is currently limited.** Commands are accepted, but the
  vehicle may not travel the exact distance or angle requested (POS_HOLD accuracy
  improvements are ongoing work). Start in a large, obstacle-free space with short
  sequences (2-3 blocks) before attempting longer programs.

## 6. Troubleshooting

| Symptom | What to check |
|---------|---------------|
| "Connect" fails | Is the PC joined to the vehicle's WiFi (`StampFly-XXYY`)? Is `wifi.mode` set to 1 (SoftAP)? Is the vehicle powered on? |
| Commands return an error | Is the vehicle in POS_HOLD mode (check the transmitter's mode switch)? Is battery level sufficient? |
| Responses stall or time out | Check WiFi signal (distance/obstacles). Confirm no other tool (e.g. `sf log wifi`, a djitellopy script) is also using UDP:8890 at the same time |
