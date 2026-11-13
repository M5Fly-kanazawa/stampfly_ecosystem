# 次のステップ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

README の手順でシミュレータと実機の飛行までできたら、この文書で操縦と開発の詳細に進みます。
シミュレータの使い方、通信モードの選択、飛行前の確認、飛行方法、ビルドせずに書き込む方法、sf CLI の基本操作、開発者向け機能をまとめています。

---

## 1. 必要なもの

### ハードウェア

| 項目 | 型番 | 備考 |
|-----|------|------|
| StampFly 機体 | - | M5Stamp S3 搭載 |
| コントローラ | M5Stack AtomS3 + Atom JoyStick | セットで使用 |
| USB-Cケーブル | - | 書き込み用 × 2本 |
| LiPoバッテリー | 1S 3.7V | 機体用 |

### ソフトウェア

| 項目 | バージョン | 備考 |
|-----|----------|------|
| ESP-IDF | v5.5.2 | install.shで自動インストール可 |
| Git | 最新版 | - |
| Python | 3.10〜3.12（推奨3.12） | - |

## 2. シミュレータの使い方

送信機の書き込みと USB HID（USB のゲームパッド等の標準規格）モードへの切替は README の「まずはシミュレータで飛ばしてみよう！」を、送信機の操作全般は [送信機の使い方](guides/controller.md) を参照してください。

### 起動

```bash
sf sim run vpython
```

ブラウザが自動で開き、3D ビューが表示されます。

### スティック操作

| 操作 | Mode 2（既定） | Mode 3 |
|-----|---------------|--------|
| スロットル（上昇/下降） | 左スティック上下 | 右スティック上下 |
| ロール（左右移動） | 右スティック左右 | 左スティック左右 |
| ピッチ（前後移動） | 右スティック上下 | 左スティック上下 |
| ヨー（旋回） | 左スティック左右 | 右スティック左右 |

### 飛ばし方

1. **スロットルをゆっくり上げる** → ドローンが浮上
2. **スティックで姿勢を調整** → 好きな方向に飛行
3. **スロットルを下げる** → 着陸

### ワールドオプション

| オプション | コマンド | 説明 |
|-----------|---------|------|
| 既定 | `sf sim run vpython` | 標準のワールド |
| リングワールド | `sf sim run vpython --world ringworld` | リング状の地形 |
| シード指定 | `sf sim run vpython --seed 12345` | 地形などの乱数シードを固定 |

### Genesis シミュレータ（任意・高精度物理）

VPython より高精度な物理演算を行うシミュレータです。別途インストールが必要です。

```bash
sf setup genesis
sf sim run genesis
```

### トラブルシューティング

| 症状 | 対処 |
|-----|------|
| コントローラが認識されない | USB HIDモードに切り替えたか確認。PCを再起動 |
| スティックがドリフトする | メニューから「Calibration」を実行 |
| vpythonが見つからない | `sf setup sim` を実行 |

## 3. 通信モードの選択

StampFly は2つの通信モードをサポートしています。

| モード | 特徴 | 用途 |
|-------|------|------|
| ESP-NOW（Espressif 社の無線直接通信方式） | 低遅延、TDMA（時分割多元接続）同期、最大10台同時 | 複数機編隊飛行、レース |
| UDP | シンプル、WiFi AP経由、単機運用 | 開発・デバッグ、単機飛行 |

### ESP-NOWモード（既定）

複数機の編隊飛行や TDMA 同期が必要な場合に使用します。ペアリング手順は [送信機の使い方](guides/controller.md) を参照してください。

### UDPモード（推奨：単機運用時）

単機での飛行や開発・デバッグ時に使用します。設定が簡単です。

**設定手順:**

1. **Vehicle側（CLI）:**
   ```
   comm udp
   ```
   この設定は NVS（電源を切っても消えない設定保存領域）に保存され、次回起動時も維持されます。

2. **Controller側:**
   画面（ボタン）を押してメニューを開き、右スティックで `Comm: ESP-NOW` の行を選び、右ボタン（モードボタン）を1回押して `Comm: UDP` にします（ESP-NOW → UDP → USB HID の順で切り替わります）。UDP への切替では再起動しません。

**通信の仕組み:**
```
┌────────────┐      WiFi AP      ┌────────────┐
│ Controller │ ←───────────────→ │  Vehicle   │
│   (STA)    │   192.168.10.1    │   (AP)     │
└────────────┘                   └────────────┘
```

**注意:** UDPモードではペアリングは不要です。VehicleのWiFi APに自動接続します。

## 4. 飛行前の確認

### チェックリスト

- [ ] バッテリーは十分に充電されているか（3.7V以上推奨）
- [ ] プロペラは正しく取り付けられているか
- [ ] 周囲に障害物がないか（最低2m四方の空間を確保）
- [ ] コントローラのスティックが中立位置にあるか

### スティックモード (Mode 2 / Mode 3)

スティックモードは送信機のメニューで切り替えます（画面を押してメニューを開き、右スティックで `Stick: Mode 2` の行を選び、右ボタンで決定すると Mode 2 と Mode 3 が入れ替わります）。設定は保存され、次回以降も維持されます。既定は Mode 2、本書では Mode 3 を推奨します。詳細は [送信機の使い方](guides/controller.md) を参照してください。

## 5. 飛行方法

機体への書き込みは README の「実際に飛ばしてみよう」を参照してください。

### スティック配置

#### Mode 2

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   スロットル  │          │   ピッチ    │
     │ ←ヨー    ヨー→│          │←ロール ロール→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

#### Mode 3（推奨）

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   ピッチ     │          │   スロットル │
     │ ←ロール ロール→│          │ ←ヨー    ヨー→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

### 基本操作

#### 1. アーム（モーター起動）

1. 機体を平らな場所に置く
2. スロットルを最下げ位置にする
3. **スロットルスティックボタン**を押す
4. モーターが回転を始める

#### 2. 離陸

1. スロットルをゆっくり上げる
2. 機体が浮き始めたら、ホバリング位置で止める

#### 3. 着陸

1. スロットルをゆっくり下げる
2. 機体が着地したらスロットルを最下げ
3. **スロットルスティックボタン**を押してディスアーム

### 制御モード

| モード | 説明 | 必要センサ | 状態 |
|-------|------|-----------|------|
| ACRO | 角速度制御 | IMU | **実装済み** |
| STABILIZE | 角度制御（自動水平） | IMU | **実装済み** |
| ALTITUDE_HOLD | 高度維持 | IMU + ToF or Baro | **実装済み** |
| POSITION_HOLD | 位置保持 | IMU + ToF/Baro + Flow | **実装済み** |

> **Tips:** コントローラの右ボタンで制御モード、左ボタンで高度モードを切り替えられます。
> 必要なセンサが無効（config.hpp で OFF）の場合、自動的に STABILIZE に降格します。
> POSITION_HOLD（POS_HOLD）は実機飛行での位置保持動作まで確認済みです。

## 6. ビルドせずに書き込む

開発環境を構築しなくても、ブラウザから実機にファームウェアを書き込めます。

### Web Flasher

| 項目 | 内容 |
|-----|------|
| URL | https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/ |
| 必要環境 | Chrome または Edge（Web Serial対応）＋ USB-Cケーブル（データ通信対応） |
| できること | 最新の GitHub Release のフルイメージをブラウザから直接書き込み |

### デスクトップ版（StampFly Flasher）

ブラウザ書き込み（Web Serial）は macOS の Chrome でクラッシュする既知の不具合があるほか、
ブラウザなしでオフライン書き込みしたい教室にも便利なため、デスクトップ GUI アプリも用意されています。

| 項目 | 内容 |
|-----|------|
| 入手先 | [リリースページ](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest)（v2026.07.1 以降のリリースに添付。Linux版は v2026.07.2 以降） |
| 対応 | Windows / macOS（Apple Silicon・Intel）/ Linux（x64） |
| 使い方 | ダウンロード → 起動 → 対象を選んで書き込み（macOS初回は右クリック→開く） |
| ソースから | `python3 tools/flasher_gui/stampfly_flasher.py`（要 `pip install esptool`）または `sf flash --gui` |
| `sf` 導入済みなら | `sf flasher install` でダウンロード〜SHA256検証〜ネイティブアプリ導入を自動化（詳細: [sf flasher](commands/sf-flasher.md)） |

## 7. sf CLI クイックスタート

**sf CLI** は、ビルド、書き込み、モニタ、ログ取得などをシンプルに実行できる統合コマンドラインツールです。

環境を有効化します（新しい端末を開くたびに実行。Windows は `setup_env.bat`）。

```bash
source setup_env.sh
```

環境を診断します。

```bash
sf doctor
```

機体ファームウェアをビルドし、書き込み、モニタを開きます。

```bash
sf build vehicle && sf flash vehicle -m
```

### よく使うコマンド

| コマンド | 説明 |
|---------|------|
| `sf build vehicle` | 機体ファームウェアをビルド |
| `sf flash vehicle -m` | 書き込み後にモニタを開く |
| `sf monitor` | シリアルモニタを開く |
| `sf log wifi` | WiFiテレメトリをキャプチャ |
| `sf cal gyro` | ジャイロキャリブレーション |
| `sf sim run vpython` | シミュレータを起動 |
| `sf app new my_ctrl` | 自分の制御則プロジェクトを作成（研究・実験用） |

**→ [コマンドリファレンス](commands/README.md)** | **[セットアップガイド](setup/README.md)**

## 8. 開発者向け機能

### WiFi テレメトリー

機体は WiFi 上で UDP だけを話す（WebSocket・HTTP のサーバは無い）。接続のしかたと全ポートは
[Tello 互換 API リファレンス](architecture/tello-api-reference.md)を参照。

| 用途 | ポート | PC 側の入口 |
|-----|-------|------------|
| 飛行コマンド・問い合わせ（Tello 風テキスト） | UDP 8889 | `sf takeoff`・`sf land`・`sf battery` など（[飛行コマンド](commands/flight-commands.md)） |
| 状態の表示（50Hz） | UDP 5005 | `sf telemetry`（`--web` でブラウザ表示） |
| 400Hz の記録 | UDP 8890 | `sf log wifi` |

### ログキャプチャ

| コマンド | 説明 |
|---------|------|
| `sf log wifi -d 30` | WiFi 経由でテレメトリを 30 秒間キャプチャ |
| `sf log capture -d 60` | USB シリアル経由で 60 秒間キャプチャ |
| `sf log list` | ログ一覧を表示 |
| `sf log analyze` | フライトログを解析 |

### キャリブレーション

| コマンド | 説明 |
|---------|------|
| `sf cal gyro` | ジャイロキャリブレーション |
| `sf cal mag start` | 磁気キャリブレーション開始（この後、機体を8の字に回転させる） |
| `sf cal mag stop` | 磁気キャリブレーション終了 |
| `sf cal mag save` | キャリブレーション結果を保存 |

### CLI（コマンドラインインターフェース）

```bash
sf monitor
```

主要コマンド:

| コマンド | 説明 |
|---------|------|
| `help` | コマンド一覧 |
| `status` | システム状態 |
| `sensor` | センサデータ表示 |
| `motor test 1 30` | モーター1を30%で回転 |
| `gain` | PIDゲイン設定 |

### Tello SDK 互換 API（プログラムからの飛行制御）

コントローラでの通常操作に加えて、Vehicle は **Tello SDK 互換の UDP コマンド**（`UDP:8889`）を受け付け、
`djitellopy` など既存の Tello 用 Python プログラムがほぼ無改変で動作します。例（`tools/stampfly_py` 配下）:

```bash
python3 tools/stampfly_py/example_djitellopy.py
```

対応コマンド一覧・安全上の注意は [`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) を
参照してください。**送信機は安全装置として中立位置でONのまま保持してください**（スティックを動かせば
いつでもパイロットが介入・停止できます）。

### 自分の制御則・推定器を書く

`sf app new my_ctrl` から始めて、自分の制御則や推定器を vehicle 本体に組み込み、SILS で確認して実機で飛ばすまでの流れは [独自プログラム開発入門](guides/custom_program.md) を参照してください。

## 9. さらに学ぶ

| リンク | 説明 |
|-------|------|
| [コマンドリファレンス](commands/README.md) | sf CLI の全コマンド |
| [firmware/vehicle/README.md](../firmware/vehicle/README.md) | 機体ファームウェア詳細 |
| [firmware/controller/README.md](../firmware/controller/README.md) | コントローラ詳細 |
| [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) | Tello SDK 互換 API 詳細 |

---

<a id="english"></a>

# Next Steps

Once you can fly both the simulator and the real vehicle using the steps in the README, use this
document to go deeper on piloting and development: simulator usage, communication mode selection,
pre-flight checks, how to fly, flashing without building, sf CLI basics, and developer features.

---

## 1. What You Need

### Hardware

| Item | Model | Notes |
|------|-------|-------|
| StampFly Vehicle | - | With M5Stamp S3 |
| Controller | M5Stack AtomS3 + Atom JoyStick | Used together |
| USB-C Cable | - | For flashing × 2 |
| LiPo Battery | 1S 3.7V | For vehicle |

### Software

| Item | Version | Notes |
|------|---------|-------|
| ESP-IDF | v5.5.2 | Auto-install via install.sh |
| Git | Latest | - |
| Python | 3.10-3.12 (3.12 recommended) | - |

## 2. Using the Simulator

Flashing the transmitter and switching it to USB HID (a standard USB gamepad protocol) mode are
covered in the README's "Try the Simulator First!" section. For everything else about operating
the transmitter, see [Using the Transmitter](guides/controller.md).

### Launch

```bash
sf sim run vpython
```

A browser opens automatically with the 3D view.

### Stick Controls

| Control | Mode 2 (Default) | Mode 3 |
|---------|------------------|--------|
| Throttle | Left stick up/down | Right stick up/down |
| Roll | Right stick left/right | Left stick left/right |
| Pitch | Right stick up/down | Left stick up/down |
| Yaw | Left stick left/right | Right stick left/right |

### How to Fly

1. **Slowly raise the throttle** → the drone lifts off
2. **Adjust attitude with the sticks** → fly in the direction you want
3. **Lower the throttle** → land

### World Options

| Option | Command | Description |
|--------|---------|-------------|
| Default | `sf sim run vpython` | Standard world |
| Ring world | `sf sim run vpython --world ringworld` | Ring-shaped terrain |
| Fixed seed | `sf sim run vpython --seed 12345` | Fix the random seed for terrain, etc. |

### Genesis Simulator (Optional, High-Precision Physics)

A higher-precision physics simulator than VPython. Requires a separate installation.

```bash
sf setup genesis
sf sim run genesis
```

### Troubleshooting

| Symptom | Solution |
|---------|----------|
| Controller not recognized | Check USB HID mode is enabled. Restart PC |
| Stick drifting | Run "Calibration" from menu |
| vpython not found | Run `sf setup sim` |

## 3. Choosing a Communication Mode

StampFly supports two communication modes.

| Mode | Features | Use Case |
|------|----------|----------|
| ESP-NOW (Espressif's proprietary direct wireless protocol) | Low latency, TDMA (time-division multiple access) sync, up to 10 devices | Multi-vehicle formation, racing |
| UDP | Simple, via WiFi AP, single-vehicle | Development, solo flight |

### ESP-NOW Mode (Default)

For multi-vehicle formation or TDMA synchronization. See [Using the Transmitter](guides/controller.md)
for the pairing procedure.

### UDP Mode (Recommended for single-vehicle)

For solo flights or development/debugging. Simple setup.

**Setup:**

1. **Vehicle (CLI):**
   ```
   comm udp
   ```
   This setting is saved to NVS (non-volatile storage that survives power-off) and persists
   after restart.

2. **Controller:**
   Press the screen (button) to open the menu, use the right stick to select the `Comm: ESP-NOW`
   row, and press the right button (mode button) once to switch to `Comm: UDP` (the order cycles
   ESP-NOW → UDP → USB HID). Switching to UDP does not restart the controller.

**How it works:**
```
┌────────────┐      WiFi AP      ┌────────────┐
│ Controller │ ←───────────────→ │  Vehicle   │
│   (STA)    │   192.168.10.1    │   (AP)     │
└────────────┘                   └────────────┘
```

**Note:** UDP mode does not require pairing. Auto-connects to Vehicle's WiFi AP.

## 4. Pre-flight Checks

### Checklist

- [ ] Battery charged (3.7V+ recommended)
- [ ] Propellers correctly attached
- [ ] Clear surroundings (2m × 2m minimum)
- [ ] Controller sticks in neutral

### Stick Mode (Mode 2 / Mode 3)

The stick mode is switched from the controller menu: press the screen to open the menu, select the `Stick: Mode 2` row with the right stick, and press the right button to toggle between Mode 2 and Mode 3. The setting is saved and kept for later sessions. The default is Mode 2; this guide recommends Mode 3. See the [Controller Guide](guides/controller.md) for details.

## 5. How to Fly

For flashing the vehicle firmware, see the README's "Let's Actually Fly" section.

### Stick Layout

#### Mode 2

```
        Left Stick                Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   Throttle   │          │    Pitch    │
     │ ←Yaw    Yaw→ │          │←Roll   Roll→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

#### Mode 3 (Recommended)

```
        Left Stick                Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │    Pitch     │          │   Throttle  │
     │←Roll   Roll→ │          │ ←Yaw    Yaw→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

### Basic Operation

#### 1. Arm (Start Motors)

1. Place the drone on a flat surface
2. Set throttle to lowest position
3. **Press the throttle stick button**
4. Motors start spinning

#### 2. Takeoff

1. Slowly raise the throttle
2. Once the drone lifts off, hold at hover position

#### 3. Landing

1. Slowly lower the throttle
2. Once landed, set throttle to lowest
3. **Press the throttle stick button** to disarm

### Control Modes

| Mode | Description | Required Sensors | Status |
|------|-------------|-----------------|--------|
| ACRO | Rate control | IMU | **Implemented** |
| STABILIZE | Angle control (auto-level) | IMU | **Implemented** |
| ALTITUDE_HOLD | Altitude hold | IMU + ToF or Baro | **Implemented** |
| POSITION_HOLD | Position hold | IMU + ToF/Baro + Flow | **Implemented** |

> **Tips:** Use the right button on the controller to switch control modes, and the left button for altitude mode.
> If required sensors are disabled (OFF in config.hpp), the mode automatically downgrades to STABILIZE.
> POSITION_HOLD (POS_HOLD) has been validated on real hardware, including in-flight position hold.

## 6. Flashing Without Building

You can flash firmware to the real hardware straight from your browser — no build environment needed.

### Web Flasher

| Item | Details |
|------|---------|
| URL | https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/ |
| Requirements | Chrome or Edge (Web Serial support) + a USB-C data cable |
| What it does | Flashes the latest GitHub Release full firmware image directly from the browser |

### Desktop App (StampFly Flasher)

A desktop GUI app is also available — the browser flasher (Web Serial) has a known bug that
crashes Chrome on macOS, and some classrooms need offline/no-browser flashing anyway.

| Item | Details |
|------|---------|
| Get it | [Releases page](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) (attached to releases from v2026.07.1 onward; Linux build from v2026.07.2 onward) |
| Supported | Windows / macOS (Apple Silicon and Intel) / Linux (x64) |
| Usage | Download → launch → select target → flash (on macOS, right-click → Open the first time) |
| From source | `python3 tools/flasher_gui/stampfly_flasher.py` (requires `pip install esptool`) or `sf flash --gui` |
| If `sf` is set up | `sf flasher install` automates download, SHA256 verification, and native-app install (details: [sf flasher](commands/sf-flasher.md)) |

## 7. sf CLI Quick Start

**sf CLI** is an integrated command-line tool for build, flash, monitor, log capture, and more.

Activate the environment (run in every new terminal; on Windows use `setup_env.bat`).

```bash
source setup_env.sh
```

Run the diagnostics.

```bash
sf doctor
```

Build the vehicle firmware, flash it, and open the monitor.

```bash
sf build vehicle && sf flash vehicle -m
```

### Common Commands

| Command | Description |
|---------|-------------|
| `sf build vehicle` | Build vehicle firmware |
| `sf flash vehicle -m` | Flash and open monitor |
| `sf monitor` | Open serial monitor |
| `sf log wifi` | Capture WiFi telemetry |
| `sf cal gyro` | Gyro calibration |
| `sf sim run vpython` | Run simulator |
| `sf app new my_ctrl` | Create your own control-law project (research/experiments) |

**→ [Command Reference](commands/README.md)** | **[Setup Guide](setup/README.md)**

## 8. Developer Features

### WiFi Telemetry

The vehicle speaks only UDP over WiFi (there is no WebSocket or HTTP server). See the
[Tello-compatible API reference](architecture/tello-api-reference.md) for how to connect and for every port.

| Purpose | Port | Entry on the PC |
|---------|------|-----------------|
| Flight commands and queries (Tello-style text) | UDP 8889 | `sf takeoff`, `sf land`, `sf battery`, ... ([flight commands](commands/flight-commands.md)) |
| Live state display (50 Hz) | UDP 5005 | `sf telemetry` (`--web` for the browser) |
| 400 Hz recording | UDP 8890 | `sf log wifi` |

### Log Capture

| Command | Description |
|---------|-------------|
| `sf log wifi -d 30` | Capture telemetry over WiFi for 30 seconds |
| `sf log capture -d 60` | Capture over USB serial for 60 seconds |
| `sf log list` | List logs |
| `sf log analyze` | Analyze a flight log |

### Calibration

| Command | Description |
|---------|-------------|
| `sf cal gyro` | Gyro calibration |
| `sf cal mag start` | Start magnetometer calibration (then rotate the vehicle in a figure-8) |
| `sf cal mag stop` | Stop magnetometer calibration |
| `sf cal mag save` | Save the calibration result |

### CLI

```bash
sf monitor
```

### Tello SDK-Compatible API (Programmatic Flight Control)

In addition to normal controller operation, the Vehicle accepts **Tello SDK-compatible UDP
commands** (`UDP:8889`), so existing Tello Python programs (e.g. `djitellopy`) run nearly
unmodified. Example (under `tools/stampfly_py`):

```bash
python3 tools/stampfly_py/example_djitellopy.py
```

See [`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) for the full command list
and safety notes. **Keep the paired RC transmitter ON with neutral sticks as a safety device**
— the pilot can intervene and stop the vehicle at any time by moving a stick.


### Writing Your Own Controller or Estimator

Starting from `sf app new my_ctrl`, to build your own control law or estimator into the vehicle firmware, verify it in SILS, and fly it on the real drone, see the [Custom Program Guide](guides/custom_program.md).

## 9. Learn More

| Link | Description |
|------|-------------|
| [Command Reference](commands/README.md) | All sf CLI commands |
| [firmware/vehicle/README.md](../firmware/vehicle/README.md) | Vehicle firmware details |
| [firmware/controller/README.md](../firmware/controller/README.md) | Controller details |
| [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) | Tello SDK-compatible API details |
</content>
