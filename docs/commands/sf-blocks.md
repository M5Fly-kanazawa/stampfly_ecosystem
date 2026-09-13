# sf blocks

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

ブラウザの Blockly UI で、プログラミングを書かずにブロックを並べてドローンを操作する。
`sf blocks` はローカル HTTP ブリッジとして PC（127.0.0.1）で起動し、ブロックで組んだ
プログラムを Tello 互換のテキスト API（UDP:8889）経由で機体へ送信する。

### 対象読者

プログラミング入門者・初めてドローンに触れる生徒・コードを書かずに飛ばしたい教員を想定しています。
`sf app`（カスタムプログラム向け）と並行する、初心者向けの入口です。

## 2. 前提

### 機体側の準備

`sf blocks` は Tello 互換 API（UDP:8889/8890）を使うため、機体を SoftAP モード
（WiFi アクセスポイント）に設定してください。

`sf monitor` でシリアルコンソールを開き、次の 2 行を入力してから機体を再起動する。

```
param set wifi.mode 1
param save
```

| 項目 | 値 |
|------|-----|
| WiFi SSID | `StampFly-XXYY`（XXYY は機体固有） |
| パスワード（既定） | `stampfly` |
| 機体アドレス | `192.168.10.1`（既定） |

**API 経由の飛行は POS_HOLD モード必須です。** 送信機の飛行モードスイッチを POS_HOLD に設定してください。

### インターネット

不要です。Blockly ライブラリは sf CLI に同梱されており、オフライン環境で動作します。

## 3. 使い方

### 起動

実機に接続する場合:

```bash
source setup_env.sh
sf blocks
```

機体なしで練習する場合（デモモード）:

```bash
sf blocks --demo
```

ブラウザが自動で開きます（開かない場合は `http://127.0.0.1:5007` を手動で開く）。

### 操作の流れ

1. 画面の「接続」ボタンで機体に接続（デモモードでは常に成功）
2. 左側のブロック一覧からブロックをドラッグしてワークスペースに配置
3. 「実行」ボタンで、ブロックを順に機体へ送信
4. 途中で止める場合は「停止」、危険時は「緊急停止」を押す

## 4. オプション

| オプション | 既定値 | 説明 |
|-----------|--------|------|
| `--host HOST` | `192.168.10.1` | 機体アドレス |
| `--port PORT` | `5007` | HTTP ブリッジのポート（ブラウザはこのポートで UI を開く） |
| `--demo` | (未指定) | デモモード（機体接続なし、UDP 通信せず） |
| `--no-browser` | (未指定) | ブラウザを自動で開かない |

## 5. 安全上の注意

詳細は `docs/guides/block_programming.md` を参照してください。

- **送信機を持った安全担当者を必ず配置。** PC の WiFi が切れても機体は自動着陸しません
- **緊急停止ボタンはモーターを即座に停止。** 本当に危険なとき以外は「停止」ボタンを使う
- **プロペラガード装着必須**。位置制御精度は改善中のため、広い場所で少数ブロックから試す

## 6. 関連

- `docs/guides/block_programming.md` — 機体の準備・操作手順・安全方針（詳細）
- `docs/architecture/tello-api-reference.md` — API コマンド一覧・仕様

---

<a id="english"></a>

## 1. Overview

Fly a drone by arranging Blockly blocks in a browser instead of writing code.
`sf blocks` is a local HTTP bridge (running on 127.0.0.1) that sends block
programs to the vehicle via a Tello-compatible text API (UDP:8889).

### Target Audience

Programming beginners, students trying a drone for the first time, and instructors
who want to fly without writing code. It complements `sf app` (custom program tool)
as an entry point for newcomers.

## 2. Prerequisites

### Vehicle Preparation

`sf blocks` requires the vehicle in SoftAP mode (WiFi access point).

Open the serial console with `sf monitor`, enter the two lines below, then power-cycle the vehicle.

```
param set wifi.mode 1
param save
```

| Item | Value |
|------|-------|
| WiFi SSID | `StampFly-XXYY` (XXYY is per-vehicle) |
| Password (default) | `stampfly` |
| Vehicle address | `192.168.10.1` (default) |

**Flight via the API requires POS_HOLD mode.** Set the transmitter's flight mode
switch to POS_HOLD before flying.

### Internet

Not required. The Blockly library ships with the sf CLI and works offline.

## 3. Usage

### Starting

With a real vehicle:

```bash
source setup_env.sh
sf blocks
```

Without a vehicle (demo mode):

```bash
sf blocks --demo
```

A browser opens automatically (or browse to `http://127.0.0.1:5007` manually).

### Workflow

1. Click "Connect" to connect to the vehicle (demo mode always succeeds)
2. Drag blocks from the left palette into the workspace
3. Click "Run" to send the block sequence to the vehicle in order
4. Use "Stop" to halt, or "Emergency" if needed

## 4. Options

| Option | Default | Description |
|--------|---------|-------------|
| `--host HOST` | `192.168.10.1` | Vehicle address |
| `--port PORT` | `5007` | HTTP bridge port (browser UI opens on this port) |
| `--demo` | (not set) | Demo mode (no vehicle connection, no UDP) |
| `--no-browser` | (not set) | Do not auto-open the browser |

## 5. Safety Notes

For full details, see `docs/guides/block_programming.md`.

- **A safety pilot with transmitter is mandatory.** WiFi disconnect does not auto-land the vehicle
- **Emergency stop cuts motors immediately.** Use "Stop" button for normal halting
- **Propeller guards must be fitted.** Position control precision is under development; start in open space with few blocks

## 6. Related

- `docs/guides/block_programming.md` — Vehicle setup, workflow, safety policy (detailed)
- `docs/architecture/tello-api-reference.md` — API command list and specification
