# StampFly vehicle_new 操作マニュアル / Operation Manual

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle_new ファームウェアの**起動シーケンス・LED/音の意味・コントローラ操作・ペアリング手順・
CLI コマンド**をまとめた実機運用マニュアル。UI（LED/ブザー）と操作系は旧 vehicle ファームを踏襲する。

### 対象読者

実機（M5StampFly）を起動・操作する人、ブリングアップ（development_roadmap Phase 2/3）を行う人。

### 安全上の注意

- **モータ単体テスト（CLI `motor`）は必ずプロペラを外すか機体を保持して行う。**
- ARM/離陸は周囲の安全を確認してから。コントローラの DISARM（または機体ボタン）で即停止できる。

## 2. 起動シーケンス

電源 ON 後、ファームは Phase 0〜4 を順に実行し、LED と音で状態を知らせる。

### 起動フェーズと LED/音

| 段階 | 内容 | StampS3 LED（状態, GPIO21） | 音 |
|------|------|---------------------------|-----|
| Phase 0 | NVS 初期化 | — | — |
| Phase 1 | BSP（sf_board）: I2C/SPI/LEDC バス | — | — |
| Phase 2 | Pub-Sub トピック初期化 | — | — |
| Phase 3 | パラメータ読込（NVS or 既定） | — | — |
| Phase 4 | 14 タスク起動 → NotifyTask 起動 | **白 常灯**（INIT） | **startTone**（ドミソ: C5→E5→G5）|
| — | INIT → IDLE_GROUND（IMU が有効値を出す）| 白 → 緑/マゼンタへ | — |
| 校正 | 起動バイアス校正（静止確認済 2.5 秒分を平均。動かすとやり直し）| **マゼンタ 低速点滅** | （無音）|
| 完了 | 校正完了 → ARM 可能 | **緑 常灯** | **readyTone**（ピッ×3）|
| ペア | 未ペアなら自動ペアリング待機 | **青 高速点滅** | **pairingTone**（C5→G5）|

**ポイント:**
- 起動音（ドミソ）が鳴り**白点灯**＝電源 ON・初期化開始。
- **マゼンタ点滅中は静止**（バイアス校正中）。動きを検出すると蓄積を破棄してやり直すため、機体を置いて LED が緑になる（＋ピッ×3）まで待つ。
- **緑常灯＋ピッ×3**＝校正完了・ARM 可能。
- **青高速点滅**＝送信機を探索中（ペアリング待ち、§5 参照）。
- Critical なハード故障時は同じ StampS3 LED が**赤 高速点滅**して停止する（sf_board の致命停止経路）。

## 3. LED / 音 リファレンス

LED は**2 チャネル**に分かれる:

| チャネル | LED | 役割 |
|---------|-----|------|
| **モード色** | 本体 LED（基板の表裏 ×2、常に同一表示） | フライトモードの色。**常に実モード**を表示: 地上 DISARM＝低速点滅、ARM 後・飛行中＝常灯 |
| **システム状態** | StampS3 内蔵 LED（GPIO21 ×1） | INIT/校正/ペアリング/IDLE/ARMED/離陸/着陸などの状態表示 |

基板の表裏は機体姿勢でどちらかが見えなくなるため同一表示とし、StampS3 LED を独立した状態表示に使う。

### モード色（本体 LED, GPIO39×2）

| モード | 色 | 地上（DISARM） | ARM 後・飛行中 |
|--------|-----|---------------|---------------|
| ACRO | 青 | 低速点滅 | 常灯 |
| STABILIZE | 黄緑 | 低速点滅 | 常灯 |
| ALT_HOLD | オレンジ | 低速点滅 | 常灯 |
| POS_HOLD | マゼンタ | 低速点滅 | 常灯 |
| （INIT 中）| 白 | 常灯 | — |
| **低電圧**（最優先）| シアン | 高速点滅 | 高速点滅 |

- **モード変更は地上（DISARM/ARM 中）でも受理される**（仕様 2026-06-11）。スイッチを切り替えると実モードと LED が即座に変わる。
- ALT_HOLD/POS_HOLD で ARM してもプロペラは回らない（制御器が接地中の推力をゼロにゲート）。これらのモードの飛行開始は**自動離陸**（下記 §4）。

### システム状態（StampS3 内蔵 LED, GPIO21）

優先度は上から（上位が下位を上書き）: 低電圧 > ペアリング > 校正中 > 飛行状態。

| 状態 | 色 | パターン |
|------|-----|---------|
| 低電圧（電池電圧が `safety.battery.low_v` 以下）| シアン | 低速点滅 |
| ペアリング中（探索中）| 青 | 高速点滅 |
| 校正中（地上・校正未完了。**静止するまで完了しない**）| マゼンタ | 低速点滅 |
| INIT（初期化中）| 白 | 常灯 |
| IDLE_GROUND（校正済・ARM 可）| 緑 | 常灯 |
| IDLE_HELD（手持ち検出）| シアン | 高速点滅 |
| ARMED_GROUND（ARM 済・地上）| 緑 | 低速点滅 |
| TAKEOFF（離陸シーケンス）| 白 | 高速点滅 |
| FLYING（飛行中）| 緑 | 常灯 |
| LANDING（着陸シーケンス）| オレンジ | 低速点滅 |
| Critical ハード故障（停止）| 赤 | 高速点滅 |

### イベント → 音（ブザー）

| イベント | 音 | 音列 |
|---------|-----|------|
| 起動 | startTone | C5→E5→G5（ドミソ）|
| 校正完了（ARM 可能）| readyTone | C5 ×3（ピッピッピッ）|
| ARM | armTone | E5→G5（上昇）|
| DISARM | disarmTone | G5→E5（下降）|
| ペアリング開始 | pairingTone | C5→G5 |
| 低電圧警告 | lowBatteryWarning | A4 …（繰返し）|
| エラー（フェイルセーフ）| errorTone | C4（長く低い）|

ブザーは CLI `sound off` で無効化できる（NVS 保存）。LED 輝度は `led <0-255>`。

## 4. コントローラ操作

コントローラ（送信機）は固定ファーム。機体は SSOT プロトコル（`protocol/spec/messages.yaml` の
ControlPacket 14B）に準拠してデコードする。**操作は旧 vehicle と同一。**

### スティック割当

| スティック | 役割 | 値（12bit ADC, 中央2048）|
|-----------|------|------------------------|
| スロットル | 上昇/下降・推力 | 0..4095（下半分は0クリップ）|
| ロール | 左右傾斜 | 中央2048、±でロール |
| ピッチ | 前後傾斜 | 中央2048、±でピッチ |
| ヨー | 旋回 | 中央2048、±でヨーレート |

中央付近はデッドバンド（±0.05）で 0 に丸める。

### ARM / DISARM

| 操作 | 動作 |
|------|------|
| コントローラの ARM スイッチ ON（flags bit0 立ち上がり）| IDLE_GROUND → ARMED_GROUND（ARM）。校正完了・電圧正常が条件 |
| ARM スイッチ OFF（bit0 立ち下がり）| → IDLE_GROUND（DISARM）。飛行中でも即カット |
| 機体ボタン クリック（地上）| ARM/DISARM トグル（飛行中のクリックは無視）|

**ARM できない時**: 校正中（マゼンタ点滅）・低電圧/USB 給電・**ペアリング中**は ARM 拒否（CLI `status`
で確認可）。

### フライトモード切替（flags）

| flags ビット | モード | 制御 |
|-------------|--------|------|
| （なし）| STABILIZE | 姿勢安定（既定）|
| bit2 = MODE | ACRO | 角速度制御 |
| bit3 = ALT_MODE | ALTITUDE_HOLD | 高度保持 |
| bit4 = POS_MODE | POSITION_HOLD | 位置保持 |

**優先順位: POS_HOLD > ALT_HOLD > ACRO > STABILIZE**（複数立っていれば上位を採用）。

### 離陸・着陸

| 操作 | 動作 |
|------|------|
| ARMED_GROUND でスロットルを上げる（>0.5）— STABILIZE/ACRO | **手動離陸**: スロットル＝推力。TAKEOFF → FLYING（ToF が空中検出で確定）|
| ARMED_GROUND でスロットルを上げる（>0.5）— ALT_HOLD/POS_HOLD | **自動離陸**: 固定 0.3 m/s 上昇・水平姿勢（POS は発進点保持）。空中検知で完了し、ALT_HOLD が現在高度を捕捉。スティックを中央（raw 3072）に戻すとその高度を保持 |
| スロットルを下げる / DISARM | 降下・着陸（FLYING → LANDING/IDLE_GROUND）|

ALT/POS の自動離陸中はスティック入力を無視する（着陸シーケンスの鏡像）。中断は DISARM。

### 操作フロー全体

```
電源ON →（自動ペアリング待機: 青点滅）
  → コントローラをペアリングモードに → 成立（緑常灯）
  → 校正完了（ピッ×3, 緑常灯）
  → モードスイッチで飛行モード選択（本体LED が選択モード色で点滅）
  → ARM（armTone, 本体LED モード色常灯）
  → スロットルUP → 離陸（STAB/ACRO=手動、ALT/POS=自動 0.3 m/s 上昇）
  → 飛行中もモードスイッチで切替可（ACRO/STAB/ALT/POS）
  → スロットルDOWN / DISARM → 着陸 → IDLE_GROUND（緑常灯）
```

## 5. ペアリング手順

機体とコントローラを1対1に束ね、複数機・複数送信機の混信を防ぐ。詳細は
[`pairing_plan.md`](pairing_plan.md)。

### 手順

1. 機体の電源を入れる。**未ペアなら自動でペアリング待機**（青 高速点滅 ＋ pairingTone）。
   機体は自分の MAC を含む PairingPacket を 500ms 周期で broadcast する。
2. **コントローラをペアリングモードにする**（CH1-13 をスキャンして機体を探す）。
3. コントローラが機体を発見 → 機体 MAC を学習 → 機体宛に ControlPacket を送信。
4. 機体が最初の ControlPacket の送信元 MAC を相手として確定 → **Paired**（青点滅停止 → 緑常灯）。
   相手 MAC は NVS に保存され、次回起動時に自動復元される。
5. 以降、相手以外の送信機のパケットは破棄される（混信対策）。

### 再ペアリング / 解除

| 操作 | 動作 |
|------|------|
| 機体ボタン 長押し3秒（地上）| 既存バインドを破棄して再ペアリング |
| CLI `unpair` | 同上（バインド破棄＋ペアリング再突入）|
| CLI `pair status` | 現在の PairingState とバインド済み相手 MAC を表示 |

### 注意（重要）

- **ペアリングは1ペアずつ順番に行う。** 複数の未ペア機を**同時に**ペアリングモードにすると、
  取り違え（自コントローラが隣の機体とペア）が起こり得る。既にペア済みで飛行/待機中の他機・他
  コントローラは干渉しない（[`pairing_plan.md`](pairing_plan.md) §P4 の既知の弱点を参照）。

## 6. CLI コマンド一覧

USB シリアル（USB CDC）で接続し、`stampfly>` プロンプトにコマンドを入力する。`help` で一覧、
TAB で補完。

### vehicle_new の現行コマンド

| コマンド | 引数 | 説明 |
|---------|------|------|
| `param` | `[list\|get <name>\|set <name> <val>\|save]` | パラメータ読み書き（範囲検証・NVS 保存）|
| `status` | — | 状態/モード/ARM・ペアリング・姿勢・高度・電池・センサ |
| `sensor` | `[imu\|mag\|baro\|tof\|flow\|power\|all]` | センサ最新値表示 |
| `version` | — | ファーム名・ビルド日時 |
| `pair` | `[start\|status]` | ペアリング再突入 / バインド状態表示 |
| `unpair` | — | バインド破棄＋ペアリング再突入 |
| `sound` | `[on\|off]` | ブザー有効/無効（NVS 保存）|
| `led` | `<0-255>` | 本体 LED 輝度（NVS 保存）|
| `motor` | `[test <1-4> <0-100>\|all <0-100>\|stop]` | **モータ単体テスト（disarmed 限定・要プロペラ除去）**。M1=FR M2=RR M3=RL M4=FL、2秒自動停止 |
| `reboot` | — | 再起動 |

**実装方針:** 各コマンドは状態/HAL に直接触れず、トピックへ「事実」を publish して所有タスクが
適用する（例: `motor` → `motor_test` トピック → ControlTask、`sound`/`led` → `ui_command` → NotifyTask、
`unpair` → `button_event` → StateTask）。これにより**将来 WiFi/UDP から同じトピックへ注入すれば
WiFi 経由の制御を追加できる**（§7）。

### 旧 vehicle のコマンド（参考・将来移植候補）

旧 vehicle には約50コマンドがある（移植は対応する内部機能が要るため順次）。主なもの:

| カテゴリ | コマンド例 | 説明 |
|---------|-----------|------|
| 飛行 | `takeoff [m]` / `land` / `hover [m] [s]` / `jump [m]` | 自律離着陸・ホバー |
| 移動 | `up/down/forward/back/left/right <cm>` / `cw/ccw <deg>` | 相対移動（Tello 風）|
| クエリ | `battery?` / `height?` / `tof?` / `attitude?` | 値問い合わせ（Tello 互換）|
| 制御 | `trim` / `gain <axis> <param> <val>` | トリム・PID ゲイン調整 |
| キャリブ | `magcal start/stop/save` | 地磁気キャリブ（Figure-8）|
| 通信 | `comm [espnow\|udp]` / `wifi ...` | 通信モード・WiFi 設定 |

これらは「自律飛行・WiFi 制御・Tello 互換」用。コントローラ手動飛行には不要で、必要に応じて
vehicle_new に移植する。

## 7. 将来の WiFi 制御（方針）

CLI コマンドはすべて**トピック publish**で実装してあるため、UDP/TCP の受信経路を追加して同じ
トピック（`motor_test` / `ui_command` / `command_setpoint` 等）へ注入すれば、**コマンド実装を変えずに
WiFi 制御を追加できる**。受信層（UDP サーバ）の追加が今後の作業。

---

<a id="english"></a>

## 1. Overview

### About This Document

Operation manual for the vehicle_new firmware: **boot sequence, LED/sound meanings, controller
operation, pairing procedure, and CLI commands.** The UI (LED/buzzer) and controls follow the
legacy vehicle firmware.

### Safety Notes

- **Always remove the propellers or hold the craft when running the motor test (CLI `motor`).**
- Ensure the area is clear before ARM/takeoff; DISARM (controller or on-board button) cuts motors.

## 2. Boot Sequence

After power-on the firmware runs Phase 0–4 in order, signalling state via LED and sound.

| Stage | Action | StampS3 LED (state, GPIO21) | Sound |
|-------|--------|------------------------------|-------|
| Phase 0 | NVS init | — | — |
| Phase 1 | BSP (sf_board): I2C/SPI/LEDC buses | — | — |
| Phase 2 | Pub-Sub topics | — | — |
| Phase 3 | Parameters (NVS or defaults) | — | — |
| Phase 4 | 14 tasks start → NotifyTask | **white solid** (INIT) | **startTone** (C5→E5→G5) |
| — | INIT → IDLE_GROUND (IMU valid) | white → green/magenta | — |
| Calib | Boot bias calibration (averages 2.5 s of VERIFIED-still samples; motion restarts it) | **magenta slow blink** | (silent) |
| Ready | Calibration done → can ARM | **green solid** | **readyTone** (beep ×3) |
| Pair | Auto-pairing if unpaired | **blue fast blink** | **pairingTone** (C5→G5) |

A Critical hardware failure lights the same StampS3 LED **red fast blink** and halts (sf_board).

## 3. LED / Sound Reference

The LEDs form **two channels**:

| Channel | LED | Role |
|---------|-----|------|
| **Mode colour** | Body LEDs (board top+bottom ×2, always identical) | Flight-mode colour. **Always the ACTIVE mode**: disarmed on ground = slow blink; armed/airborne = solid |
| **System state** | StampS3 built-in LED (GPIO21 ×1) | INIT/calibrating/pairing/idle/armed/takeoff/landing status |

The board's top/bottom LEDs show the same content (one of them is hidden depending on the
craft's pose); the StampS3 LED serves as the independent status display.

### Mode colour (body LEDs, GPIO39×2)

| Mode | Colour | On ground (disarmed) | Armed / airborne |
|------|--------|---------------------|------------------|
| ACRO | blue | slow blink | solid |
| STABILIZE | yellow-green | slow blink | solid |
| ALT_HOLD | orange | slow blink | solid |
| POS_HOLD | magenta | slow blink | solid |
| (during INIT) | white | solid | — |
| **Low battery** (top priority) | cyan | fast blink | fast blink |

- **Mode changes are accepted on the ground** (disarmed or armed; spec 2026-06-11). Flipping
  the switch changes the actual mode — and the LED — immediately.
- ARMing in ALT_HOLD/POS_HOLD keeps the props stopped (the controller gates thrust to zero on
  the ground). Flight in these modes starts with an **auto-takeoff** (fixed 0.3 m/s climb,
  level attitude; POS holds the launch point) triggered by raising the throttle.

### System state (StampS3 built-in LED, GPIO21), priority: low-battery > pairing > calibrating > flight state

| State | Colour | Pattern |
|-------|--------|---------|
| Low battery (≤ `safety.battery.low_v`) | cyan | slow blink |
| Pairing (searching) | blue | fast blink |
| Calibrating (ground, not done; **never completes while moving**) | magenta | slow blink |
| INIT | white | solid |
| IDLE_GROUND (ready) | green | solid |
| IDLE_HELD | cyan | fast blink |
| ARMED_GROUND | green | slow blink |
| TAKEOFF | white | fast blink |
| FLYING | green | solid |
| LANDING | orange | slow blink |
| Critical hardware failure (halt) | red | fast blink |

### Event → Sound

| Event | Sound | Notes |
|-------|-------|-------|
| Boot | startTone | C5→E5→G5 |
| Calibration done (ready) | readyTone | beep ×3 |
| ARM | armTone | E5→G5 |
| DISARM | disarmTone | G5→E5 |
| Pairing start | pairingTone | C5→G5 |
| Low battery | lowBatteryWarning | repeated |
| Error (failsafe) | errorTone | low/long |

Mute via `sound off` (NVS-saved); LED brightness via `led <0-255>`.

## 4. Controller Operation

The transmitter is fixed firmware; the vehicle decodes the SSOT ControlPacket (14B). **Operation is
identical to the legacy vehicle.**

- **Sticks** (12-bit ADC, centre 2048): throttle (0..4095, low half clipped to 0), roll/pitch/yaw
  (±about centre, deadband ±0.05).
- **ARM/DISARM**: controller ARM switch (flags bit0) rising = ARM (gated on calibration + voltage),
  falling = DISARM. On-board button click (on ground) toggles ARM/DISARM.
- **Flight modes** (flags): bit2 = ACRO, bit3 = ALT_HOLD, bit4 = POS_HOLD; default STABILIZE.
  Priority **POS_HOLD > ALT_HOLD > ACRO > STABILIZE**.
- **Takeoff**: throttle > 0.5 in ARMED_GROUND → TAKEOFF → FLYING (ToF airborne). **Landing**: throttle
  down / DISARM.

ARM is refused while calibrating, on low/USB power, or while pairing (check `status`).

## 5. Pairing Procedure

Binds one transmitter to one vehicle to prevent crosstalk. See [`pairing_plan.md`](pairing_plan.md).

1. Power on the vehicle. **If unpaired it auto-enters Pairing** (blue fast blink + tone) and broadcasts
   a PairingPacket (its MAC) every 500 ms.
2. **Put the controller into pairing mode** (it scans CH1-13 for the vehicle).
3. The controller finds the vehicle, learns its MAC, and unicasts a ControlPacket to it.
4. The vehicle fixes the first ControlPacket's src MAC as its peer → **Paired** (blue blink stops →
   green solid). The peer MAC is saved to NVS and restored on the next boot.
5. Thereafter ControlPackets from any other transmitter are dropped.

**Re-pair / clear**: on-board button long-press 3 s (on the ground), or CLI `unpair`. `pair status`
shows the current PairingState and bound MAC.

**Important**: pair ONE pair at a time. Putting multiple unpaired vehicles into pairing mode
simultaneously can cross-pair (see the known limitation in [`pairing_plan.md`](pairing_plan.md) §P4).
Already-paired vehicles/controllers nearby do not interfere.

## 6. CLI Commands

Connect over USB serial (USB CDC); type at the `stampfly>` prompt. `help` lists commands, TAB
completes.

| Command | Args | Description |
|---------|------|-------------|
| `param` | `[list\|get <name>\|set <name> <val>\|save]` | Read/write parameters (validated, NVS) |
| `status` | — | State/mode/arm, pairing, attitude, altitude, battery, sensors |
| `sensor` | `[imu\|mag\|baro\|tof\|flow\|power\|all]` | Print sensor readings |
| `version` | — | Firmware name / build date |
| `pair` | `[start\|status]` | Re-enter pairing / show bind |
| `unpair` | — | Clear bind and re-enter pairing |
| `sound` | `[on\|off]` | Enable/disable the buzzer (NVS) |
| `led` | `<0-255>` | Body LED brightness (NVS) |
| `motor` | `[test <1-4> <0-100>\|all <0-100>\|stop]` | **Bench motor test (DISARMED only — props off!)**. M1=FR M2=RR M3=RL M4=FL, 2 s auto-stop |
| `reboot` | — | Restart the flight controller |

Every command publishes a FACT on a topic and the owning task applies it (e.g. `motor` → `motor_test`
→ ControlTask; `sound`/`led` → `ui_command` → NotifyTask; `unpair` → `button_event` → StateTask). So a
future WiFi/UDP receiver can inject the same topics to add **WiFi control without changing the command
implementations** (§7).

The legacy vehicle has ~50 commands (flight: `takeoff`/`hover`/`jump`; Tello-style moves and queries:
`battery?`/`height?`; `trim`/`gain`/`magcal`/`comm`/`wifi`). These are for autonomous flight / WiFi /
Tello compatibility and are ported to vehicle_new on demand.

## 7. Future WiFi Control (Direction)

Because every CLI command is implemented as a topic publish, adding a UDP/TCP receiver that injects
the same topics (`motor_test` / `ui_command` / `command_setpoint`, …) adds WiFi control **without
changing the command logic**. The receiver layer (UDP server) is future work.
