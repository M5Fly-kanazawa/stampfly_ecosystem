# 照会コマンド（Query Commands）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`sf battery`・`sf height` などは、機体の UDP 8889 へ Tello 風の問い合わせ（`battery?` など）を 1 つ送り、
機体の答えを表示するトップレベルコマンド群（それぞれ独立したコマンドで、`sf query <command>` という
サブコマンドではない）。単位は機体の応答のもの（`firmware/vehicle/tasks/api_task.cpp` の `cmdQuery`）。
実装は `lib/sfcli/commands/query.py`。

## 2. コマンド

### コマンド一覧

| コマンド | 機体へ送る文字列 | 機体の応答の例 | 単位 |
|---------|----------------|---------------|------|
| `sf battery` | `battery?` | `87` | %（電圧から 3.3 V=0%、4.2 V=100% の線形換算） |
| `sf height` | `height?` | `50` | cm（推定高度） |
| `sf tof` | `tof?` | `10` | cm（下向き ToF 距離） |
| `sf baro` | `baro?` | `170.07` | m（気圧高度） |
| `sf attitude` | `attitude?` | `pitch:0;roll:0;yaw:0;` | 度 |
| `sf acceleration` | `acceleration?` | `agx:-13.00;agy:-5.00;agz:-998.00;` | 0.001 g |
| `sf speed` | `speed?` | `30` | cm/s（設定した巡航速度。実速度ではない） |

### 終了コード

0 = 機体が答えた、1 = 機体が `error ...` を返した（文面のまま表示）、3 = 応答なし（タイムアウト）。

## 3. 使用例

```bash
sf battery
sf height
sf attitude
```

---

<a id="english"></a>

# Query Commands

## 1. Overview

### About This Document

`sf battery`, `sf height` and the others are top-level commands that send one Tello-style query (`battery?`
and so on) to the vehicle's UDP port 8889 and print its answer (each is an independent command, not a
subcommand of `sf query <command>`). The units are those of the vehicle's replies (`cmdQuery` in
`firmware/vehicle/tasks/api_task.cpp`). The implementation is `lib/sfcli/commands/query.py`.

## 2. Commands

### Command List

| Command | Text sent to the vehicle | Example reply | Unit |
|---------|--------------------------|---------------|------|
| `sf battery` | `battery?` | `87` | % (linear from the voltage: 3.3 V = 0%, 4.2 V = 100%) |
| `sf height` | `height?` | `50` | cm (estimated height) |
| `sf tof` | `tof?` | `10` | cm (downward ToF distance) |
| `sf baro` | `baro?` | `170.07` | m (barometric altitude) |
| `sf attitude` | `attitude?` | `pitch:0;roll:0;yaw:0;` | degrees |
| `sf acceleration` | `acceleration?` | `agx:-13.00;agy:-5.00;agz:-998.00;` | 0.001 g |
| `sf speed` | `speed?` | `30` | cm/s (the cruise-speed setting, not the actual speed) |

### Exit Status

0 = the vehicle answered, 1 = the vehicle replied `error ...` (printed verbatim), 3 = no reply (timeout).

## 3. Examples

```bash
sf battery
sf height
sf attitude
```
