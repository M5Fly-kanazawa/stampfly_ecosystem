# sf rc

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`sf rc` は、機体の UDP 8889 へ `rc a b c d`（連続操縦値）を送る。機体は `rc` に応答せず、API で飛行中
（`sf takeoff` の後）でなければ無視する。送信が止まると機体は位置を保持する。パイロットが送信機の
スティックを動かすと、機体は `rc` による操縦を即座に解除する。実装は `lib/sfcli/commands/rc.py`。

## 2. 使い方

### 構文

```bash
sf rc [a b c d]
```

### パラメータ（Tello SDK 互換）

| パラメータ | 範囲 | 説明 |
|-----------|------|------|
| `a` | -100 ~ 100 | 左右（roll、右が正） |
| `b` | -100 ~ 100 | 前後（pitch、前が正） |
| `c` | -100 ~ 100 | 上下（throttle、上が正） |
| `d` | -100 ~ 100 | ヨー（時計回りが正） |

範囲外の値は送らずに拒否する。

### モード

| モード | 内容 |
|-------|------|
| ワンショット `sf rc 0 50 0 0` | `command` を送ったあと `rc` を 1 行送る。応答は無いので、送ったことだけを表示する |
| インタラクティブ `sf rc` | キーボード操作。`--rate` の周期（既定 20 Hz）で `rc` を送り続ける。`l` は 0 を送ってから `land` を送り、機体が地上に降りるまで待つ。`x` は `emergency` を送る。Ctrl-C は `rc 0 0 0 0` を送って終わる |

### 終了コード

0 = 送った、1 = 範囲外・機体が `error ...` を返した、3 = `command` への応答なし（タイムアウト）。

## 3. 使用例

```bash
sf rc 0 50 0 0
sf rc
```

---

<a id="english"></a>

# sf rc

## 1. Overview

### About This Document

`sf rc` sends `rc a b c d` (continuous stick values) to the vehicle's UDP port 8889. The vehicle never
answers `rc` and ignores it unless it is flying under the API (after `sf takeoff`). When the stream stops, the
vehicle holds its position. When the pilot moves a stick on the transmitter, the vehicle drops the `rc`
control at once. The implementation is `lib/sfcli/commands/rc.py`.

## 2. Usage

### Syntax

```bash
sf rc [a b c d]
```

### Parameters (Tello SDK compatible)

| Parameter | Range | Description |
|-----------|-------|-------------|
| `a` | -100 to 100 | left/right (roll, right is positive) |
| `b` | -100 to 100 | forward/back (pitch, forward is positive) |
| `c` | -100 to 100 | up/down (throttle, up is positive) |
| `d` | -100 to 100 | yaw (clockwise is positive) |

A value out of range is refused without sending.

### Modes

| Mode | Content |
|------|---------|
| One-shot `sf rc 0 50 0 0` | Sends `command`, then one `rc` line. There is no reply, so it only reports that it sent |
| Interactive `sf rc` | Keyboard control. Sends `rc` every `--rate` period (default 20 Hz). `l` sends zeros and then `land`, and waits until the vehicle is on the ground. `x` sends `emergency`. Ctrl-C sends `rc 0 0 0 0` and ends |

### Exit Status

0 = sent, 1 = out of range or the vehicle replied `error ...`, 3 = no reply to `command` (timeout).

## 3. Examples

```bash
sf rc 0 50 0 0
sf rc
```
