# 飛行コマンド（Flight Commands）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`sf takeoff`・`sf land` などは、機体の UDP 8889 へ Tello 風のテキストコマンドを 1 つ送り、機体の応答を
待つトップレベルコマンド群（それぞれ独立したコマンドで、`sf flight <command>` というサブコマンドではない）。
文法・単位・応答は機体ファームウェア（`firmware/vehicle/tasks/api_task.cpp`）のもので、
[Tello 互換 API リファレンス](../architecture/tello-api-reference.md)が基準となる文書。実装は
`lib/sfcli/commands/flight.py` と `lib/sfcli/utils/vehicle_api.py`。

### 接続

機体の WiFi（既定のアドレス `192.168.10.1`）に PC をつなぐ。別のアドレスは `--ip` で指定する。
機体は飛行コマンドを UDP 8889 だけで受け付ける（TCP 23 の CLI は台上・デバッグ用で、飛行コマンドは無い）。

## 2. コマンド

### コマンド一覧

| コマンド | 機体へ送る文字列 | 範囲・単位 | 機体が応答するとき |
|---------|----------------|-----------|------------------|
| `sf takeoff` | `takeoff` | 引数なし。高度は機体固定の 0.5 m | 飛行状態になったとき（最大 12 秒） |
| `sf land` | `land` | 引数なし | 地上に降りたとき（最大 32 秒） |
| `sf up <cm>` | `up <cm>` | 10〜300 cm | 到達したとき |
| `sf down <cm>` | `down <cm>` | 10〜300 cm | 到達したとき |
| `sf forward <cm>` | `forward <cm>` | 10〜300 cm | 到達したとき |
| `sf back <cm>` | `back <cm>` | 10〜300 cm | 到達したとき |
| `sf left <cm>` | `left <cm>` | 10〜300 cm | 到達したとき |
| `sf right <cm>` | `right <cm>` | 10〜300 cm | 到達したとき |
| `sf cw <deg>` | `cw <deg>` | 1〜360 度 | 向きが揃ったとき（最大 8 秒） |
| `sf ccw <deg>` | `ccw <deg>` | 1〜360 度 | 向きが揃ったとき（最大 8 秒） |
| `sf stop` | `stop` | 引数なし | すぐ（現在位置を保持） |
| `sf emergency` | `emergency` | 引数なし | すぐ（全モータ停止） |

どのコマンドも、先に `command`（SDK モードへの切り替え）を送ってから本体を送る。`sf stop` と
`sf emergency` だけは、別のコマンドが機体を待たせていても待たされないよう、この手順なしで直接送る。

### 範囲の検査

| 項目 | 内容 |
|-----|------|
| 移動距離 | 10〜300 cm。機体は 10 cm 未満を `error out of range` で拒否し、300 cm を超える分は黙って 300 cm に縮めて `ok` を返すので、PC 側で拒否する |
| 回頭角 | 1〜360 度 |
| 高度の上限・下限 | 高度の上限は 120 cm、移動の目標高度の下限は 20 cm。`sf up` と `sf down` は先に `height?` で現在の推定高度を問い合わせ、移動後が範囲を出るなら送らずに拒否する（機体は範囲外の移動を縮めて `ok` を返すため）。機体は推定でなく誘導目標に移動を足すので、追従誤差（数 cm）の差がありうる。`height?` に応答がなければ送らない |

### 終了コードと表示

| 終了コード | 意味 |
|-----------|------|
| 0 | 機体が `ok` を返した |
| 1 | 機体が `error ...` を返した、または引数が範囲外で送らなかった |
| 3 | 機体から応答がなかった（タイムアウト） |
| 130 | Ctrl-C で中断した |

機体のエラー応答は文面のまま表示する。応答待ち時間は機体自身のタイムアウトから決めてあり、`--timeout <秒>` で
変えられる。自動で再送はしない（`up 50` の再送は 2 回上昇してしまうため）。

### 応答待ちの間の Ctrl-C

| コマンド | Ctrl-C で送るもの | 理由 |
|---------|------------------|------|
| `takeoff`・移動・`cw`・`ccw` | `stop` | `stop` は現在位置を保持し、移動・回頭はその場で終わる。離陸の上昇中は機体が `stop` を拒否して離陸を続ける（拒否の応答を表示し、`sf land` か `sf emergency` を案内する） |
| `land` | 何も送らない | 機体は既に着陸している最中。止める理由がない |

`emergency` は自動では送らない。空中でモータを止めるかどうかは操縦者が決める。

### 削除したコマンド

| コマンド | 理由 |
|---------|------|
| `sf hover`・`sf jump` | API に対応する文字列が無い。離陸高度は 0.5 m 固定で、`hover` の高度指定も `jump` の 0.15 m（移動の下限 0.2 m より低い）も表せない |

## 3. 使用例

```bash
sf battery
sf takeoff
sf forward 50
sf cw 90
sf land
sf emergency
```

---

<a id="english"></a>

# Flight Commands

## 1. Overview

### About This Document

`sf takeoff`, `sf land` and the others are top-level commands that send one Tello-style text command to the
vehicle's UDP port 8889 and wait for its reply (each is an independent command, not a subcommand of
`sf flight <command>`). The grammar, units and replies are those of the vehicle firmware
(`firmware/vehicle/tasks/api_task.cpp`); the [Tello-compatible API reference](../architecture/tello-api-reference.md)
is the reference document. The implementation is `lib/sfcli/commands/flight.py` and
`lib/sfcli/utils/vehicle_api.py`.

### Connection

Connect the PC to the vehicle's WiFi (default address `192.168.10.1`); give another address with `--ip`.
The vehicle accepts flight commands on UDP 8889 only (the TCP 23 CLI is a bench / debug console with no
flight commands).

## 2. Commands

### Command List

| Command | Text sent to the vehicle | Range, unit | The vehicle replies |
|---------|--------------------------|-------------|---------------------|
| `sf takeoff` | `takeoff` | no argument; the altitude is the vehicle's fixed 0.5 m | when it is flying (up to 12 s) |
| `sf land` | `land` | no argument | when it is on the ground (up to 32 s) |
| `sf up <cm>` | `up <cm>` | 10-300 cm | on arrival |
| `sf down <cm>` | `down <cm>` | 10-300 cm | on arrival |
| `sf forward <cm>` | `forward <cm>` | 10-300 cm | on arrival |
| `sf back <cm>` | `back <cm>` | 10-300 cm | on arrival |
| `sf left <cm>` | `left <cm>` | 10-300 cm | on arrival |
| `sf right <cm>` | `right <cm>` | 10-300 cm | on arrival |
| `sf cw <deg>` | `cw <deg>` | 1-360 deg | when the heading is reached (up to 8 s) |
| `sf ccw <deg>` | `ccw <deg>` | 1-360 deg | when the heading is reached (up to 8 s) |
| `sf stop` | `stop` | no argument | at once (holds the current position) |
| `sf emergency` | `emergency` | no argument | at once (all motors off) |

Every command first sends `command` (enter SDK mode) and then the command itself. Only `sf stop` and
`sf emergency` are sent directly, without that step, so that they never wait while another command holds
the vehicle.

### Range Checks

| Item | Content |
|------|---------|
| Move distance | 10-300 cm. The vehicle refuses less than 10 cm with `error out of range` and silently shortens more than 300 cm to 300 cm while replying `ok`, so the PC rejects it |
| Rotation angle | 1-360 deg |
| Altitude ceiling and floor | The ceiling is 120 cm and the floor of a move's target altitude is 20 cm. `sf up` and `sf down` first ask `height?` for the estimated height and refuse, without sending, a move that would leave the range (the vehicle shortens such a move and replies `ok`). The vehicle adds a move to its guidance target, not to the estimate, so they can differ by the tracking error (a few cm). Without a reply to `height?` nothing is sent |

### Exit Status and Output

| Exit status | Meaning |
|-------------|---------|
| 0 | the vehicle replied `ok` |
| 1 | the vehicle replied `error ...`, or an argument was out of range and nothing was sent |
| 3 | no reply from the vehicle (timeout) |
| 130 | interrupted with Ctrl-C |

The vehicle's error reply is printed verbatim. The reply wait is derived from the vehicle's own timeouts and can
be changed with `--timeout <seconds>`. Nothing is resent automatically (resending `up 50` would climb twice).

### Ctrl-C While Waiting for a Reply

| Command | Sent on Ctrl-C | Reason |
|---------|----------------|--------|
| `takeoff`, the moves, `cw`, `ccw` | `stop` | `stop` holds the current position and ends a move or rotation at once. While the takeoff is still climbing, the vehicle refuses `stop` and carries on (the refusal is printed together with a pointer to `sf land` and `sf emergency`) |
| `land` | nothing | the vehicle is already landing; there is nothing to stop |

`emergency` is never sent automatically: whether to cut the motors in the air is the operator's decision.

### Removed Commands

| Command | Reason |
|---------|--------|
| `sf hover`, `sf jump` | The API has no matching text. The takeoff altitude is fixed at 0.5 m, so neither the altitude argument of `hover` nor the 0.15 m of `jump` (below the 0.2 m floor of a move) can be expressed |

## 3. Examples

```bash
sf battery
sf takeoff
sf forward 50
sf cw 90
sf land
sf emergency
```
