# StampFly Tello 互換 API リファレンス

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**最終更新: 2026-09-13**（現行 `firmware/vehicle` の実装に合わせて全面書き換え。旧版は TCP 23 + WebSocket を前提にした 2026-02-14 時点の設計検討記録で、現行実装と通信方式のレベルで食い違っていたため置き換えた。旧版の原文は `git show archive/2026-09-13:docs/architecture/tello-api-reference.md` で参照できる）。

## 1. 概要と接続

### このドキュメントについて

StampFly の主力ファーム `firmware/vehicle` は DJI Tello SDK 互換のテキストコマンドを UDP で話す。本書は実装（`firmware/vehicle/tasks/api_task.cpp`、`firmware/vehicle/components/sf_telemetry/`）から抽出した現行仕様であり、`djitellopy` など Tello 用 Python プログラムを StampFly で動かす際の一次参照とする。実機運用の手順は `firmware/vehicle/docs/operation_manual.md` を参照。

### 接続

機体は既定で **SoftAP** を張り、IP は Tello 実機と同じ **`192.168.10.1`**（`djitellopy` の既定ホストと一致するため `Tello()` を引数なしで呼べる）。この振り直しは `firmware/vehicle/components/sf_comm/comm.cpp` が SoftAP 起動時に行う。`wifi.mode` パラメータ（既定値 `0` = STA クライアントモード）を `1` にすると SoftAP になる。STA モードで共有ルータに接続する場合は、機体の LAN IP を明示的に渡す（例: `Tello(host="192.168.1.42")`）。

### ポート一覧

| ポート | 方向 | 内容 | 実装 |
|---|---|---|---|
| UDP 8889 | PC→機体（コマンド）、機体→PC（応答） | Tello 互換テキストコマンド | `tasks/api_task.cpp`（`ApiTask`） |
| UDP 8890（送信専用） | 機体→PC | Tello 互換の状態文字列、10Hz | `tasks/api_task.cpp`（`TelloStateTask`）。書式は `components/sf_telemetry/include/tello_state.hpp` |
| UDP 8890（機体側 bind の別ソケット、`sf log wifi` 用） | PC↔機体 | 400Hz サンプルをまとめたバイナリ統合パケット（50Hz 送出）。上記の状態文字列とは別プロトコルで、**同時使用不可**（どちらも PC 側で同じポートを使う） | `components/sf_telemetry/{data_stream.hpp,data_stream.cpp}`。詳細は `docs/architecture/udp-telemetry-design.md` |
| UDP 5005（ブロードキャスト） | 機体→ブロードキャスト | `sf::TelemetryPacket`（104バイト、姿勢・角速度・加速度・位置・速度・推力/トルク・モータduty）、50Hz | `components/sf_telemetry/{telemetry.hpp,telemetry.cpp}` |
| TCP 23 | PC↔機体（対話式） | telnet 的 CLI（`param`/`status`/`sensor`/`version`/`mac`/`pair`/`unpair`/`sound`/`led`/`motor`/`wifi`/`magcal`/`reboot`）。**飛行コマンドは無い** | `tasks/cli_task.cpp` |

Tello 実機の映像ポート（UDP 11111）は StampFly には無い（カメラ非搭載）。

## 2. コマンド一覧

`command` を送って SDK モードに入るまで、他のコマンドは `"error not in command mode"` を返す。以降のコマンドは成功時 `"ok"`、失敗時 `"error ..."` を返す（クエリ行は数値・文字列を返す）。

### 制御

| コマンド | 構文 | 説明 | 前提条件 |
|---|---|---|---|
| `command` | `command` | SDK モード開始 | なし |
| `takeoff` | `takeoff` | 自動離陸（POS_HOLD、手動 RC 離陸と同じ経路）。FLYING 到達まで最大12秒ブロック | 地上（IDLE_GROUND/ARMED_GROUND）かつ静止校正済み |
| `land` | `land` | 自動着陸。IDLE_GROUND 到達まで最大20秒ブロック | 飛行中 |
| `emergency` | `emergency` | 全モーター緊急停止。他の全ての判定より優先して処理される | 常時 |
| `stop` | `stop` | 現在位置を目標に再設定してホバリング | FLYING かつ API 誘導目標が有効 |

### 移動・回転

| コマンド | 構文 | 範囲 | 説明 |
|---|---|---|---|
| `up`/`down`/`left`/`right`/`forward`/`back` | `<verb> x`（cm） | 合成移動距離 10〜300cm（Tello実機は20〜500cm。下限は屋内向けにきめ細かく設定。300cm超は方向を保ったまま300cmにクランプし `ok` を返す） | 現在の誘導目標（推定値でなく目標値）に合成し、到達until ブロック |
| `cw`/`ccw` | `<verb> x`（度） | 1〜360度 | ヨー目標を加算し、到達until ブロック |
| `go` | `go x y z speed` | xyz: cm（合成距離は up/down 等と同じ10〜300cmクランプ）, speed: 10〜100 cm/s | 3軸同時移動 |
| `rc` | `rc a b c d` | 各 -100〜100 | 連続速度指令（撃ちっぱなし、応答なし）。a=ロール（右+）、b=ピッチ（前+）、c=スロットル（上+）、d=ヨーレート（時計回り+）。送信停止で自動的にその場保持へ戻る。パイロットがスティックを動かすと即解除される |

移動・回転・`go`・`rc` は FLYING かつ API 誘導目標が有効（`takeoff` 直後、またはパイロット介入で解除された場合は `takeoff`/`go` 等で再係合）でないと `"error not flying"` を返す（`rc` は応答なしで無視）。

### 独自拡張（StampFly 固有、Tello SDK にない）

| コマンド | 構文 | 説明 |
|---|---|---|
| `autotune` | `autotune <roll\|pitch\|yaw> [ωc rad/s] [PM deg]`（既定 roll/pitch: ωc=25・PM=60、yaw: ωc=18・PM=60） | オンボードで①ステップドサイン9点掃引（2〜35Hz）②プラント同定 ③位相余裕仕様を満たすPID設計 ④ライブ適用（NVS 未保存）を行う。FLYING時のみ |
| `sysid` | `sysid <roll\|pitch\|yaw> <doublet\|chirp> <amp_dps> <dur_s>` | レートループ同定用の励振をホバー中に1軸へ注入。励振完了までブロックしてから `ok` |

### クエリ（`?` を含む行）

| コマンド | 応答例 | 単位 | 備考 |
|---|---|---|---|
| `battery?` | `"87"` | % | 電圧から 3.3V=0%/4.2V=100% の線形換算 |
| `height?` | `"50"` | cm | 推定高度（−Down成分） |
| `attitude?` | `"pitch:0;roll:0;yaw:0;"` | 度 | 姿勢クォータニオンから算出 |
| `speed?` | `"30"` | cm/s | **設定済みの巡航速度**（`speed x` で設定した値。実速度は状態ストリームの `vgx/vgy/vgz`） |
| `sdk?` | `"20"` | - | 固定値。SDK 2.0 を名乗る（`djitellopy` が最大互換で前提にするバージョン） |
| `sn?` | `"STAMPFLY-XXXXXX"` | - | STA MAC 末尾3バイトの16進。**Tello実機の `"0TQDF6GEBMB5HF"` 形式とは異なる独自形式** |
| `time?` | `"15"` | 秒 | FLYING 開始からの経過秒 |
| `wifi?` | `"90"` | - | 固定値（SNR取得源が機体側に無いため強信号の定数を返す） |
| `tof?` | `"10"` | cm | 下向き ToF 距離 |
| `temp?` | `"25C"` | - | IMU温度＋`"C"`。**Tello実機の `"62~65"` 形式とは異なる** |
| `baro?` | `"170.07"` | m（生値） | 気圧高度。**Tello実機は cm** |
| `acceleration?` | `"agx:-13.00;agy:-5.00;agz:-998.00;"` | 0.001g（ミリg） | IMU加速度 |

### 設定

| コマンド | 構文 | 説明 |
|---|---|---|
| `speed` | `speed x`（10〜100 cm/s にクランプ） | 以後の verb 移動（up/down/left/right/forward/back）の巡航速度を設定 |

### 応答はするがハードとして非対応

| コマンド | 応答 | 理由 |
|---|---|---|
| `streamon`/`streamoff` | `"ok"` | カメラ非搭載だが、映像を読み取らずに切り替えるだけのプログラムを止めないための措置 |
| `flip <l/r/f/b>` | `"error flip not supported on StampFly"` | 小型機での宙返りは高リスクという判断（2026-06-23） |
| `mon`/`moff`/`mdirection` | `"error mission pads not supported"` | ミッションパッドは Tello EDU/RoboMaster TT 専用機能 |
| 上記以外の未知コマンド | `"error unknown command"` | — |

## 3. 状態ストリーム（UDP 8890、10Hz）

UDP:8889 へ何か1回でも送ると、送信元IPへ 8890 から Tello 互換の状態文字列が 10Hz で送られ始める（`djitellopy` の `connect()` はこの受信を必須とする。届かないと例外になる）。

```
mid:-2;x:0;y:0;z:0;mpry:0,0,0;pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy:%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f;time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n
```

| フィールド | 単位 | 内容 |
|---|---|---|
| `mid` | - | 固定 `-2`（ミッションパッド検出無効。EDU専用機能を実装していないことを示す有効値であり、エラーではない） |
| `x`/`y`/`z`/`mpry` | - | 固定 `0`（ミッションパッド座標。未実装のダミー値） |
| `pitch`/`roll`/`yaw` | 度 | 姿勢クォータニオンから変換 |
| `vgx`/`vgy`/`vgz` | cm/s | 対地速度。NED速度を機体系（前後/左右/上下）へ回転して変換 |
| `templ`/`temph` | °C | IMU温度（1センサのため両方同じ値） |
| `tof` | cm | 下向き ToF 距離 |
| `h` | cm | 高度（推定位置の−Down成分） |
| `bat` | % | バッテリー残量（`battery?`と同じ算出、両者は同じ関数を共有しドリフトしない） |
| `baro` | m（生値） | 気圧高度。**`djitellopy` の `get_barometer()` は内部で×100してcm扱いするため、電文はm単位のまま送っている（Tello実機と同じ流儀）** |
| `time` | 秒 | FLYING 開始からの経過秒 |
| `agx`/`agy`/`agz` | 0.001g | IMU加速度 |

姿勢・速度の符号や座標系が実機上で `djitellopy` の想定と一致するかは、実装コード内のコメントで「実機で要照合」と明記されており、本書では**未確認**。

## 4. Python から使う

`tools/stampfly_py/` に2つの入口がある。

- **`djitellopy`（推奨）**: `pip install djitellopy` するだけで、無改変の `Tello()` が SoftAP既定IP（192.168.10.1）にそのまま繋がる。`tools/stampfly_py/example_djitellopy.py`・`example_djitellopy2.py` が使用例
- **`stampfly.py`**: 依存ゼロの軽量クライアント（`djitellopy`を入れたくない場合）。`example_square.py` が使用例

| 分類 | コマンド・機能 | 対応 |
|---|---|---|
| 制御・移動・回転・絶対移動・巡航速度設定・連続手動操作・読み取り全般 | `command`/`takeoff`/`land`/`emergency`/`stop`/`up`等/`go`/`speed`/`rc`/`battery?`等 | 対応 |
| カメラ | `streamon`/`streamoff`、フレーム取得 | 非対応（`ok`は返すが映像は出ない。カメラ非搭載） |
| 宙返り | `flip`系 | 非対応（`error`。小型機での高リスクを理由に拒否） |
| 円弧移動 | `curve` | 非対応（`error`。未実装） |
| ミッションパッド | `mon`/`moff`/`mdirection`、パッド基準の`go`/`curve`/`jump` | 非対応（`error`。EDU専用機能） |

**注意:** `sf log wifi`（400Hz/50Hzの DataStream キャプチャ）と Tello 状態ストリームは、どちらも PC 側で UDP:8890 を使うため**同時実行不可**。

## 5. 経緯

2026-02-14 に「TCP CLI + WebSocket をそのまま活用し、API 名レベルの互換を優先する」という設計検討（当時の `firmware/vehicle`＝現在の `vehicle_old` 世代の CLI/WebSocket を前提）が作成された。その後 2026-06-11 のコミット `caca9cc5`（"Tello-style programmatic flight — guidance + ApiTask + Python SDK"）で、UDP 8889/8890 を使う Tello 実機準拠のプロトコルとして実装された。TCP+WebSocket 前提から UDP ネイティブ実装へと転換した理由を記した設計判断の記録は見当たらなかった（**未確認**）。

## 6. 非対応・未実装

- カメラ映像（`streamon`/`streamoff` は `ok` を返すが実際の映像配信はしない）
- `flip`（宙返り。ハード制約というより安全上の判断）
- `curve`（円弧移動、および座標移動系のミッションパッド指定 `mid` 引数）
- ミッションパッド機能一式（`mon`/`moff`/`mdirection`、パッド基準の `go`/`curve`/`jump`）
- SDK 3.0 系コマンド（`motoron`/`motoroff`/`throwfly`/`setbitrate`/`setfps`/`setresolution` 等）
- `wifi ssid/pass`・`ap ssid/pass`（UDP:8889 経由の設定コマンドとしては未実装。同等の設定は TCP CLI の `wifi` コマンドで可能）
- 「無通信N秒での自動着陸」に相当する処理は `api_task.cpp` 内には見当たらない（**未確認** — 他タスクに実装されている可能性は本書の調査範囲外）

---

<a id="english"></a>

# StampFly Tello-Compatible API Reference

**Last updated: 2026-09-13** (fully rewritten to match the current `firmware/vehicle` implementation. The previous version was a 2026-02-14 design-study record that assumed TCP 23 + WebSocket, which no longer matches the current implementation even at the level of communication method; the original text can be retrieved via `git show archive/2026-09-13:docs/architecture/tello-api-reference.md`).

## 1. Overview and Connection

### About This Document

The main `firmware/vehicle` firmware speaks DJI Tello SDK-compatible text commands over UDP. This document is the current specification extracted from the implementation (`firmware/vehicle/tasks/api_task.cpp`, `firmware/vehicle/components/sf_telemetry/`), and is the primary reference for running Tello Python programs such as `djitellopy` against StampFly. See `firmware/vehicle/docs/operation_manual.md` for real-vehicle operating procedures.

### Connection

By default the vehicle runs a **SoftAP** addressed at **`192.168.10.1`**, the same IP as the real Tello (matching `djitellopy`'s default host, so `Tello()` connects with no arguments). This re-addressing is done by `firmware/vehicle/components/sf_comm/comm.cpp` when the SoftAP starts. Setting the `wifi.mode` parameter (default `0` = STA client mode) to `1` enables the SoftAP. When using STA mode to join a shared router, pass the vehicle's LAN IP explicitly (e.g. `Tello(host="192.168.1.42")`).

### Port List

| Port | Direction | Content | Implementation |
|---|---|---|---|
| UDP 8889 | PC→vehicle (commands), vehicle→PC (replies) | Tello-compatible text commands | `tasks/api_task.cpp` (`ApiTask`) |
| UDP 8890 (send-only) | vehicle→PC | Tello-compatible state string, 10Hz | `tasks/api_task.cpp` (`TelloStateTask`); format in `components/sf_telemetry/include/tello_state.hpp` |
| UDP 8890 (a separate socket bound on the vehicle, for `sf log wifi`) | PC↔vehicle | Binary bundle packets of 400Hz samples (sent at 50Hz). A different protocol from the state string above — **cannot be used at the same time** (both use the same port on the PC side) | `components/sf_telemetry/{data_stream.hpp,data_stream.cpp}`; details in `docs/architecture/udp-telemetry-design.md` |
| UDP 5005 (broadcast) | vehicle→broadcast | `sf::TelemetryPacket` (104 bytes: attitude, angular rate, acceleration, position, velocity, thrust/torque, motor duty), 50Hz | `components/sf_telemetry/{telemetry.hpp,telemetry.cpp}` |
| TCP 23 | PC↔vehicle (interactive) | telnet-style CLI (`param`/`status`/`sensor`/`version`/`mac`/`pair`/`unpair`/`sound`/`led`/`motor`/`wifi`/`magcal`/`reboot`). **No flight commands here** | `tasks/cli_task.cpp` |

There is no video port (UDP 11111, as on the real Tello) on StampFly — there is no camera.

## 2. Command List

Until `command` is sent to enter SDK mode, every other command returns `"error not in command mode"`. After that, commands return `"ok"` on success or `"error ..."` on failure (query lines return a numeric or string value).

### Control

| Command | Syntax | Description | Precondition |
|---|---|---|---|
| `command` | `command` | Enter SDK mode | None |
| `takeoff` | `takeoff` | Auto takeoff (POS_HOLD, the same path as manual RC takeoff). Blocks up to 12 s until FLYING | On the ground (IDLE_GROUND/ARMED_GROUND) and stillness-calibrated |
| `land` | `land` | Auto land. Blocks up to 20 s until IDLE_GROUND | Airborne |
| `emergency` | `emergency` | Kill all motors immediately; processed ahead of every other gate | Always |
| `stop` | `stop` | Re-target the current position and hover | FLYING with an active API guidance target |

### Movement and Rotation

| Command | Syntax | Range | Description |
|---|---|---|---|
| `up`/`down`/`left`/`right`/`forward`/`back` | `<verb> x` (cm) | Combined travel distance 10–300 cm (the real Tello is 20–500 cm; the lower bound is finer for indoor use; distances over 300 cm are clamped to 300 cm, preserving direction, and still reply `ok`) | Composes onto the current guidance TARGET (not the state estimate); blocks until reached |
| `cw`/`ccw` | `<verb> x` (degrees) | 1–360 degrees | Adds to the yaw target; blocks until reached |
| `go` | `go x y z speed` | xyz: cm (same 10–300 cm clamp as above), speed: 10–100 cm/s | Simultaneous 3-axis move |
| `rc` | `rc a b c d` | each -100..100 | Continuous velocity command (fire-and-forget, no reply). a=roll (right+), b=pitch (forward+), c=throttle (up+), d=yaw rate (clockwise+). Automatically reverts to holding position when the stream stops; instantly overridden if the pilot moves the stick |

Movement, rotation, `go`, and `rc` return `"error not flying"` unless FLYING with an active API guidance target (established right after `takeoff`, or re-established with `takeoff`/`go` etc. after a pilot override released it); `rc` is silently ignored instead.

### StampFly-Specific Extensions (not in the Tello SDK)

| Command | Syntax | Description |
|---|---|---|
| `autotune` | `autotune <roll\|pitch\|yaw> [wc rad/s] [PM deg]` (default roll/pitch: wc=25, PM=60; yaw: wc=18, PM=60) | Onboard: (1) a 9-point stepped-sine sweep (2–35 Hz), (2) plant identification, (3) PID design meeting the phase-margin spec, (4) live application (not saved to NVS). FLYING only |
| `sysid` | `sysid <roll\|pitch\|yaw> <doublet\|chirp> <amp_dps> <dur_s>` | Injects a rate-loop identification excitation on one axis while hovering; blocks until the excitation finishes, then replies `ok` |

### Queries (lines containing `?`)

| Command | Example reply | Unit | Notes |
|---|---|---|---|
| `battery?` | `"87"` | % | Linear mapping from voltage: 3.3V=0%, 4.2V=100% |
| `height?` | `"50"` | cm | Estimated altitude (the −Down component) |
| `attitude?` | `"pitch:0;roll:0;yaw:0;"` | degrees | Computed from the attitude quaternion |
| `speed?` | `"30"` | cm/s | The **configured** cruise speed (set via `speed x`), not the live ground speed (that's `vgx`/`vgy`/`vgz` in the state stream) |
| `sdk?` | `"20"` | - | Fixed value; reports SDK 2.0 (the version `djitellopy` targets for broadest compatibility) |
| `sn?` | `"STAMPFLY-XXXXXX"` | - | Hex of the last 3 bytes of the STA MAC. **Differs from the real Tello's `"0TQDF6GEBMB5HF"`-style format** |
| `time?` | `"15"` | seconds | Elapsed seconds since FLYING began |
| `wifi?` | `"90"` | - | Fixed value (no SNR source on the vehicle side, so a strong-signal constant is returned) |
| `tof?` | `"10"` | cm | Downward-facing ToF distance |
| `temp?` | `"25C"` | - | IMU temperature plus `"C"`. **Differs from the real Tello's `"62~65"`-style format** |
| `baro?` | `"170.07"` | m (raw) | Barometric altitude. **The real Tello reports cm** |
| `acceleration?` | `"agx:-13.00;agy:-5.00;agz:-998.00;"` | 0.001g (milli-g) | IMU acceleration |

### Settings

| Command | Syntax | Description |
|---|---|---|
| `speed` | `speed x` (clamped to 10–100 cm/s) | Sets the cruise speed used by subsequent verb moves (up/down/left/right/forward/back) |

### Acknowledged but Not Supported in Hardware

| Command | Reply | Reason |
|---|---|---|
| `streamon`/`streamoff` | `"ok"` | No camera, but replying `ok` keeps programs that merely toggle the stream (without reading frames) from stalling |
| `flip <l/r/f/b>` | `"error flip not supported on StampFly"` | A flip is judged too risky an acro maneuver for this small craft (decided 2026-06-23) |
| `mon`/`moff`/`mdirection` | `"error mission pads not supported"` | Mission pads are a Tello EDU/RoboMaster TT-only feature |
| Any other unknown command | `"error unknown command"` | — |

## 3. State Stream (UDP 8890, 10Hz)

Once anything at all has been sent to UDP:8889, the vehicle starts pushing a Tello-compatible state string to the sender's IP on port 8890 at 10Hz (`djitellopy`'s `connect()` requires at least one such packet, or it raises).

```
mid:-2;x:0;y:0;z:0;mpry:0,0,0;pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy:%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%.2f;time:%d;agx:%.2f;agy:%.2f;agz:%.2f;\r\n
```

| Field | Unit | Content |
|---|---|---|
| `mid` | - | Fixed `-2` (mission-pad detection disabled — a valid value indicating the EDU-only feature is not implemented, not an error) |
| `x`/`y`/`z`/`mpry` | - | Fixed `0` (mission-pad coordinates; unimplemented placeholders) |
| `pitch`/`roll`/`yaw` | degrees | Converted from the attitude quaternion |
| `vgx`/`vgy`/`vgz` | cm/s | Ground speed; NED velocity rotated into the body frame (forward/right/up) |
| `templ`/`temph` | °C | IMU temperature (one sensor, so both fields are identical) |
| `tof` | cm | Downward-facing ToF distance |
| `h` | cm | Height (the −Down component of the estimated position) |
| `bat` | % | Battery percentage (shares the same computation as `battery?`, so the two never drift apart) |
| `baro` | m (raw) | Barometric altitude. **`djitellopy`'s `get_barometer()` multiplies internally by 100 to report cm, so the value on the wire stays in meters — matching the real Tello's convention** |
| `time` | seconds | Elapsed seconds since FLYING began |
| `agx`/`agy`/`agz` | 0.001g | IMU acceleration |

Whether the sign conventions and frame for attitude/velocity match what `djitellopy` expects on real hardware is explicitly flagged as "needs empirical checking on hardware" in a code comment, and is **unconfirmed** in this document.

## 4. Using It From Python

`tools/stampfly_py/` provides two entry points.

- **`djitellopy` (recommended)**: just `pip install djitellopy`; the unmodified `Tello()` connects directly to the SoftAP's default IP (192.168.10.1). `tools/stampfly_py/example_djitellopy.py` and `example_djitellopy2.py` are working examples.
- **`stampfly.py`**: a dependency-free lightweight client (if you'd rather not install `djitellopy`). `example_square.py` is a usage example.

| Category | Commands / features | Support |
|---|---|---|
| Control, movement, rotation, absolute move, cruise-speed setting, continuous manual control, general reads | `command`/`takeoff`/`land`/`emergency`/`stop`/`up` etc./`go`/`speed`/`rc`/`battery?` etc. | Supported |
| Camera | `streamon`/`streamoff`, frame capture | Not supported (`ok` is returned but no video is produced — no camera) |
| Flip | `flip`-family | Not supported (`error` — refused as too risky for this small craft) |
| Curve | `curve` | Not supported (`error` — not implemented) |
| Mission pads | `mon`/`moff`/`mdirection`, pad-relative `go`/`curve`/`jump` | Not supported (`error` — EDU-only feature) |

**Note:** `sf log wifi` (the 400Hz/50Hz DataStream capture) and the Tello state stream both use UDP:8890 on the PC side, so **they cannot run at the same time**.

## 5. History

On 2026-02-14, a design study was written that proposed leveraging the existing TCP CLI + WebSocket infrastructure and prioritizing API-name-level compatibility (assuming the CLI/WebSocket of the `firmware/vehicle` of that era — the codebase now called `vehicle_old`). It was then implemented on 2026-06-11 in commit `caca9cc5` ("Tello-style programmatic flight — guidance + ApiTask + Python SDK") as a real Tello-hardware-compatible protocol over UDP 8889/8890. No record explaining the reasoning for the switch from TCP+WebSocket to a native UDP implementation was found (**unconfirmed**).

## 6. Not Supported / Not Implemented

- Camera video (`streamon`/`streamoff` reply `ok` but no video is actually streamed)
- `flip` (a safety decision, not strictly a hardware limitation)
- `curve` (arc movement, and the pad-relative `mid` argument on movement commands)
- The full mission-pad feature set (`mon`/`moff`/`mdirection`, pad-relative `go`/`curve`/`jump`)
- SDK 3.0-era commands (`motoron`/`motoroff`/`throwfly`/`setbitrate`/`setfps`/`setresolution`, etc.)
- `wifi ssid/pass` and `ap ssid/pass` as UDP:8889 commands (not implemented there; the equivalent setting is available via the TCP CLI's `wifi` command)
- No processing equivalent to "auto-land after N seconds without a command" was found inside `api_task.cpp` (**unconfirmed** — it may exist in another task, which is outside the scope of this document's investigation)
