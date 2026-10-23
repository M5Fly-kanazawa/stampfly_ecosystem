# USB Hub Power Cycling for StampS3 Flashing

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

PC から USB ポートの電源（VBUS: USB の 5 V 電源線）を切り入れできるハブを使い、StampFly（ESP32-S3 / StampS3）の書き込みと起動を**ケーブルの抜き差しなし**で行う方法をまとめる。2026-10-23 に macOS で検証した結果の記録であり、将来この手順をスクリプト化するときの設計資料を兼ねる。

**本書の手順は、下記の特定のハブを持っている場合にだけ使える。** 汎用の手順ではないため、現時点では `sf` コマンド等へのスクリプト組み込みは行っていない（5章）。

### 対象読者

- StampFly の書き込みで「BOOT ボタンを押しながら挿す」「書き込み後に抜き差しする」作業を減らしたい開発者
- 同じ方式を別のハブや Linux で再現したい開発者

### 解決したい問題

| 場面 | これまでの作業 |
|---|---|
| 時々、書き込みが始まらない | BOOT ボタン（G0）を押しながら USB を挿し直して書き込みモードに入れる |
| 書き込みモードで書き込んだ後 | ファームウェアが起動しないので USB を抜き差しする |

## 2. 検証環境

| 項目 | 内容 |
|---|---|
| PC | Mac Studio（Apple Silicon）、macOS 27.0 |
| ハブ | StarTech.com **HB31C3A1CPD3**（USB-C 接続、USB-A ×3 + USB-C ×1、PD 入力付き） |
| 制御ソフト | `uhubctl` 2.6.0（USB ハブのポート電源を切り入れするコマンド。`brew install uhubctl`）、libusb 1.0.30（uhubctl が使う USB 通信ライブラリ） |
| 書き込み | ESP-IDF 5.5（ESP32 の開発環境）付属の esptool 4.12.0（ESP32 への書き込みツール。`~/.espressif/python_env/idf5.5_py3.12_env/bin/python -m esptool`） |
| 機体 | StampFly（StampS3）、`firmware/vehicle`（2026-10-23 ビルド。ビルドとはソースから書き込み用ファイルを作ること） |

ハブは PC から見ると 2 つのハブに分かれている（USB 3.0 対応ハブの一般的な構成）。

| 場所（location） | 中身 | VID:PID（USB の製造元番号:製品番号） | ハブが申告するポート数 |
|---|---|---|---|
| `4-1` | USB 2.0 側 | `14b0:013a` | 5 |
| `4-2` | USB 3.0 側 | `14b0:0131` | 4 |

場所の名前（`4-1` など）は Mac 側のどの USB ポートに挿すかで変わる。`uhubctl` を引数なしで実行し、`14b0` のハブを探して確認する。

## 3. 結論

### 電源の切り方

```bash
# OFF（USB 3.0 側 → USB 2.0 側の順）
uhubctl -e -l 4-2 -p 2 -a off
uhubctl -e -l 4-1 -p 2 -a off
# ON
uhubctl -e -l 4-1 -p 2 -a on
uhubctl -e -l 4-2 -p 2 -a on
```

- `sudo`（管理者権限での実行）は不要。外部電源（PD 入力）の有無で結果は変わらない。
- **USB-A の 3 口は、両側のポート 2 の組でまとめて切り入れされる。** 機体をどの USB-A 口に挿していても同じコマンドを使う。USB-A に挿した他の機器も一緒に切れる。
- **USB-C の口は常時通電で、切れない**（両側の全ポートを OFF にしても機体の LED は点灯したまま）。機体は必ず USB-A の口に挿す。

### 抜き差しなしでできるようになったこと

| 作業 | 手順 | 結果 |
|---|---|---|
| 書き込み後の起動 | 書き込み（`--after no_reset`）→ 電源を入れ直す | ファームウェアが起動（起動音・LED で確認） |
| 書き込みモードへの切り替え（ボタンあり） | BOOT ボタンを押したまま電源を入れ直し、約 1 秒後に離す | 書き込みモードで起動（`303a:1001` として見える） |
| 書き込み（ボタンなし） | 電源を入れ直す → 約 6 秒待つ → esptool で書き込み → 電源を入れ直す | 書き込みから起動まで手を触れずに完了 |

## 4. 詳細

### オプションなしの uhubctl では切れなかった理由

`uhubctl -l 4-1 -p 2 -a off` だけでは、ハブは `off` と表示するが VBUS は切れなかった（LED 点灯のまま）。原因は uhubctl の判定条件にあり、ハブの故障や macOS の制限ではなかった。

- uhubctl は USB 2.0 側を指定すると、対になる USB 3.0 側（同じ Container ID、すなわち同一の物理ハブであることを示す識別子を持つハブ）を探して一緒に切り替える。その条件の一つに「ポート数が等しいこと」がある（`uhubctl.c` 972 行付近、v2.6.0）。

  ```c
  /* Hubs should have the same number of ports */
  if (hubs[i].nports != hubs[j].nports) {
      if (hubs[i].nports + hubs[j].nports > 3) continue;
  ```

  このハブは USB 2.0 側が 5 ポート、USB 3.0 側が 4 ポートと申告するため対と見なされず、USB 2.0 側しか切り替わらない。
- `-l 4-2` で USB 3.0 側を直接指定すると `No compatible devices detected` になる。uhubctl は USB 3.0 側を USB 2.0 側経由でのみ数える設計のため（同 1040 行付近）。
- `-e`（`--exact`: 対を探さず指定した場所だけを操作する）を付けると、両側を別々に操作できる。

### ポートと電源の対応（LED による実測）

機体の LED を目視して確認した。

| 機体を挿した口 | データ上のポート（USB 2.0 側） | 切った組み合わせ | LED |
|---|---|---|---|
| USB-A | 2 | USB 2.0 側のポート 2 だけ | 点灯 |
| USB-A | 2 | USB 3.0 側のポート 2 だけ | 点灯 |
| USB-A | 2 | 両側のポート 2 | **消灯** |
| USB-A | 4 | 両側のポート 4 → USB 3.0 側の全ポート → USB 2.0 側の 1・3 も追加 | 点灯 |
| USB-A | 4 | 上に加えて USB 2.0 側のポート 2 | **消灯** |
| USB-A | 4 | 両側のポート 2 だけ | **消灯** |
| USB-A | 3 | 両側のポート 2 だけ | **消灯** |
| USB-C | 1 | 両側のポート 2 → 両側のポート 1 → 両側の全ポート | 点灯 |

ハブは「ポートごとに電源を切れる（ppps: per-port power switching）」と申告するが、実際の配線はポート 2 の組による一括切り替えである。

### OFF 中も Mac には機体が見え続ける

uhubctl で電源を切っても、Mac の USB 一覧（macOS の機器情報表示コマンド `ioreg`、`system_profiler`）とシリアルポート（`/dev/cu.usbmodem*`）には 30 秒以上機体が残る。ただし通信はできない（esptool は `No serial data received`）。手で抜いた場合は即座に消える。

uhubctl はハブへの制御要求（USB の制御用通信であるコントロール転送）を OS の USB 管理を通さず直接送るため、OS が切断を知らないことによる（README「USB devices are not removed after port power down on Linux」と同じ現象。Linux では kernel（OS の中核部）6.0 以降で対策済み、macOS には対策なし）。電源が入っているかは `uhubctl -e -l 4-1` の表示（`off` / `power`）で判断し、USB 一覧では判断しない。電源を入れ直せば機体は正しく接続し直される。

### BOOT ボタンなしで書き込める理由と限界

機体のファームウェアは `CONFIG_ESP_CONSOLE_USB_CDC=y`（コンソール、すなわちログ出力先を USB の仮想シリアル通信（CDC）にする設定。`firmware/vehicle/sdkconfig`）で、通常動作中は USB-OTG（ファームウェアが動かす USB。`303a:0009` として見える）を使う。esptool が出す「書き込みモードへ移れ」という指令（シリアル線の制御信号 DTR/RTS の切り替え）は、ファームウェアの USB 処理が受け取ってリセットする仕組みであり、**ファームウェアが正常に動いているときにしか届かない**。

- 「時々書き込めなくなる」のは、ファームウェアや USB 処理が止まって指令が届かない状態と推定する（再現はしていない）。
- 電源を入れ直せばファームウェアは起動し直すので、その直後に esptool を実行すればボタンなしで書き込みモードに入れる（3章、検証済み）。
- ファームウェアが起動直後に止まる場合は、この方法でも入れない。その場合は BOOT ボタンを押したまま電源を入れ直す。

根本的な対策の候補として、コンソールを USB-Serial/JTAG（チップ内蔵の回路で動く USB。`303a:1001`）に切り替える方法がある（`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`）。こちらはファームウェアが止まっていても esptool がチップを直接リセットできる。ただし、`idf.py monitor`（`sf flash vehicle -m`）を開くと機体がリセットされるという欠点がある（6章）。これは 2026-10-15 に一度この切り替えを入れて取り消した理由（`905e39f5` → `f47c8a9b`）と一致する。

## 5. スクリプト化するときの仕様（未実装）

スクリプト化とは、ここまでの手順を 1 つのコマンドで自動実行できるようにすること。

### 基本動作

| 名前（案） | 動作 |
|---|---|
| `hub-cycle` | 両側のポート 2 を OFF → 2 秒待つ → ON |
| `flash-and-boot` | `hub-cycle` → 機体が `303a:0009` で現れるまで待つ（実測約 6 秒）→ `esptool --before default_reset --after no_reset write_flash @flash_args` → `hub-cycle` |
| 失敗時の案内 | esptool が接続できなければ「BOOT ボタンを押したまま Enter」と表示し、`hub-cycle` 後に機体が `303a:1001` で現れたら「ボタンを離す」と案内して書き込みを再試行 |

### 実装上の注意

- ハブの場所（`4-1` / `4-2`）は固定値にしない。`uhubctl` の一覧から `14b0:013a` と `14b0:0131` を探して決める。
- 電源の状態は USB 一覧ではなく uhubctl の表示で判定する（4章）。
- 機体のシリアルポート名（`/dev/cu.usbmodem41301` 等）は挿した口で変わる。書き込み前に毎回探す。
- 書き込み時は `build/flash_args` を使う（ブートローダ 0x0、パーティション表 0x8000、本体 0x10000）。
- このハブを持っていない環境では何もしないこと。ハブが見つからなければ従来手順（手動の抜き差し）を案内する。
- 別のハブで使う場合は、ポートと電源の対応を 4章と同じ方法（LED の目視）で調べ直す必要がある。uhubctl の対応表（README）にある機種でも、OS によって動作が異なる。

## 6. 書き込み後に起動しない問題の原因（追加検証）

### 結論

**BOOT ボタンで書き込みモードに入れて書き込んだ後、esptool の標準の `--after hard_reset` でリセットすると、チップは書き込みモードに残る。`--after watchdog_reset` にすると正常に起動する。** 「書き込み後に抜き差ししないと起動しない」症状の原因はこれである。

- `hard_reset` は USB-Serial/JTAG の制御信号（RTS）でチップをリセットする。BOOT ボタンで入った書き込みモードからは、この方法では抜けられなかった。
- `watchdog_reset` はチップ内のウォッチドッグ（固まったときに自動でリセットする回路）で全体をリセットする。
- `sf flash vehicle` は内部で `idf.py flash` を実行し（`lib/sfcli/commands/flash.py`）、書き込み後のリセット方式は `sdkconfig` の `CONFIG_ESPTOOLPY_AFTER="hard_reset"` で決まる。このため BOOT ボタンで書き込むたびにこの症状が出ていたと考えられる。

### 検証結果

いずれもバッテリーなし。判定は起動音・LED（目視）と、esptool の応答で行った。

| 書き込みモードへの入り方 | 書き込み後のリセット | 結果 | 回数 |
|---|---|---|---|
| BOOT ボタン + 電源投入 | `hard_reset` | 書き込みモードに残る（起動音なし、LED 停止。`--before no_reset` の esptool に ROM が応答） | 1 |
| 上の状態から | `watchdog_reset` | 起動（起動音、LED、`303a:0009`、コマンド入力可） | 1 |
| BOOT ボタン + 電源投入 | `watchdog_reset` | 起動（起動音 1 回、LED、`303a:0009`） | 1 |
| 動作中の USB-CDC 版から esptool で入る | `hard_reset` | 起動（3章の最初の試験） | 1 |

### USB-Serial/JTAG 版で分かったこと

検証用に、コンソールを USB-Serial/JTAG にしたファームウェア（`905e39f5` の変更を戻したもの。ブランチ `test/usj-console-hotplug`）を使った。

- **`idf.py monitor` を開くと機体がリセットされる。** ログに `rst:0x15 (USB_UART_CHIP_RESET)` が出て、バッテリー駆動中でも再起動した（起動音で確認）。起動後に USB をつないで状況を見る用途には使えない。
- 制御信号を動かさずにポートを開けば、バッテリーで起動した後に USB をつないでもコマンド入力（`stampfly>`）は使えた。起動ログは残らない。
- USB-Serial/JTAG 版が動いている状態から USB-CDC 版を書き込み、`hard_reset` で起動すると、**ファームウェアは動く（起動音・LED）が USB が使えない**（esptool も `No serial data received`）。USB の通信の切断・再接続では直らず、電源を入れ直すと直る（2 回再現）。

### 未確認の事項

- 動作中のファームウェアから esptool で書き込みモードに入る通常の書き込みで、`watchdog_reset` に変えても問題がないか。
- USB-CDC 版で、起動後に USB をつないで状況を見られるか。検証に使った確認用プログラムがポートを開くときに USB-CDC 版をリセットしていた可能性があり、結果は無効とした。
- 「抜き差ししても書き込めない」症状は再現できていない。

## 7. 関連

- uhubctl: <https://github.com/mvp/uhubctl>
- [開発ツール使用ガイド](tools.md)
- [トラブルシューティング](troubleshooting.md)

---

<a id="english"></a>

# USB Hub Power Cycling for StampS3 Flashing

## 1. Overview

### About This Document

This document describes how to flash and boot a StampFly (ESP32-S3 / StampS3) **without unplugging the cable**, using a USB hub that lets the PC switch the power of each port (VBUS, the 5 V supply line of USB) on and off. It records the results verified on macOS on 2026-10-23 and also serves as design material for scripting the procedure later.

**The procedure applies only if you own the specific hub described below.** Because it is not a general-purpose procedure, it has not been integrated into the `sf` command or other scripts yet (chapter 5).

### Intended Audience

- Developers who want to reduce the work of "holding the BOOT button while plugging in" and "unplugging and replugging after flashing"
- Developers who want to reproduce the same approach with another hub or on Linux

### Problems to Solve

| Situation | Work done so far |
|---|---|
| Flashing sometimes does not start | Hold the BOOT button (G0) while replugging USB to enter download mode |
| After flashing in download mode | The firmware does not start, so USB must be unplugged and replugged |

## 2. Test Environment

| Item | Details |
|---|---|
| PC | Mac Studio (Apple Silicon), macOS 27.0 |
| Hub | StarTech.com **HB31C3A1CPD3** (USB-C uplink, USB-A x3 + USB-C x1, PD input) |
| Control software | `uhubctl` 2.6.0 (a command that switches USB hub port power; `brew install uhubctl`), libusb 1.0.30 (the USB communication library used by uhubctl) |
| Flashing | esptool 4.12.0 bundled with ESP-IDF 5.5 (the ESP32 development environment; a tool for writing to the ESP32. `~/.espressif/python_env/idf5.5_py3.12_env/bin/python -m esptool`) |
| Vehicle | StampFly (StampS3), `firmware/vehicle` (built on 2026-10-23; building means producing the files to flash from the source code) |

To the PC, the hub appears as two separate hubs (the usual structure of a USB 3.0 hub).

| Location | Contents | VID:PID (USB vendor ID : product ID) | Ports reported by the hub |
|---|---|---|---|
| `4-1` | USB 2.0 side | `14b0:013a` | 5 |
| `4-2` | USB 3.0 side | `14b0:0131` | 4 |

The location name (`4-1`, etc.) changes depending on which USB port of the Mac the hub is plugged into. Run `uhubctl` without arguments and look for the hub with `14b0` to check.

## 3. Conclusion

### How to Switch the Power

```bash
# OFF (USB 3.0 side first, then USB 2.0 side)
uhubctl -e -l 4-2 -p 2 -a off
uhubctl -e -l 4-1 -p 2 -a off
# ON
uhubctl -e -l 4-1 -p 2 -a on
uhubctl -e -l 4-2 -p 2 -a on
```

- `sudo` (running with administrator privileges) is not required. The result does not depend on whether external power (PD input) is connected.
- **The three USB-A ports are switched together, as the pair of port 2 on both sides.** The same commands work whichever USB-A port the vehicle is plugged into. Other devices plugged into USB-A are powered off as well.
- **The USB-C port is always powered and cannot be switched off** (the vehicle's LED stays lit even with all ports on both sides turned OFF). Always plug the vehicle into a USB-A port.

### What Is Now Possible Without Unplugging

| Task | Procedure | Result |
|---|---|---|
| Boot after flashing | Flash (`--after no_reset`) -> power cycle | Firmware starts (confirmed by the startup sound and LED) |
| Switch to download mode (with button) | Power cycle while holding the BOOT button, release after about 1 s | Boots in download mode (seen as `303a:1001`) |
| Flashing (without button) | Power cycle -> wait about 6 s -> flash with esptool -> power cycle | From flashing to booting completes without touching anything |

## 4. Details

### Why Plain uhubctl Could Not Switch the Power Off

With only `uhubctl -l 4-1 -p 2 -a off`, the hub reported `off` but VBUS was not cut (the LED stayed lit). The cause was a condition inside uhubctl, not a hub fault or a macOS restriction.

- When the USB 2.0 side is specified, uhubctl looks for the paired USB 3.0 side (a hub with the same Container ID, an identifier showing that both belong to the same physical hub) and switches both together. One of the conditions is that "the number of ports is equal" (around line 972 of `uhubctl.c`, v2.6.0).

  ```c
  /* Hubs should have the same number of ports */
  if (hubs[i].nports != hubs[j].nports) {
      if (hubs[i].nports + hubs[j].nports > 3) continue;
  ```

  This hub reports 5 ports on the USB 2.0 side and 4 on the USB 3.0 side, so they are not treated as a pair and only the USB 2.0 side is switched.
- Specifying the USB 3.0 side directly with `-l 4-2` gives `No compatible devices detected`, because uhubctl is designed to count the USB 3.0 side only via the USB 2.0 side (around line 1040 of the same file).
- With `-e` (`--exact`: do not look for a pair; operate only on the specified location), both sides can be operated separately.

### Port-to-Power Mapping (Measured by LED)

Confirmed by visually checking the vehicle's LED.

| Port the vehicle was plugged into | Data port (USB 2.0 side) | Combination switched off | LED |
|---|---|---|---|
| USB-A | 2 | Port 2 of the USB 2.0 side only | On |
| USB-A | 2 | Port 2 of the USB 3.0 side only | On |
| USB-A | 2 | Port 2 of both sides | **Off** |
| USB-A | 4 | Port 4 of both sides -> all ports of the USB 3.0 side -> also ports 1 and 3 of the USB 2.0 side | On |
| USB-A | 4 | The above plus port 2 of the USB 2.0 side | **Off** |
| USB-A | 4 | Port 2 of both sides only | **Off** |
| USB-A | 3 | Port 2 of both sides only | **Off** |
| USB-C | 1 | Port 2 of both sides -> port 1 of both sides -> all ports of both sides | On |

The hub reports that it can switch power per port (ppps: per-port power switching), but the actual wiring switches them together as the pair of port 2.

### The Vehicle Stays Visible to the Mac While OFF

Even after uhubctl cuts the power, the vehicle remains in the Mac's USB list (`ioreg` and `system_profiler`, the macOS device-information commands) and serial ports (`/dev/cu.usbmodem*`) for more than 30 seconds. Communication is not possible, however (esptool reports `No serial data received`). When the cable is unplugged by hand, the entries disappear immediately.

This is because uhubctl sends control requests to the hub (control transfers, the USB control communication) directly, bypassing the OS's USB management, so the OS does not learn about the disconnection (the same phenomenon as "USB devices are not removed after port power down on Linux" in the README; on Linux it is addressed since kernel 6.0 (the core of the OS), and macOS has no such measure). Judge whether power is on from the `uhubctl -e -l 4-1` display (`off` / `power`), not from the USB list. Once the power is turned on again, the vehicle is reconnected correctly.

### Why Flashing Works Without the BOOT Button, and Its Limit

The vehicle firmware uses `CONFIG_ESP_CONSOLE_USB_CDC=y` (a setting that sends the console, i.e. the log output, over USB virtual serial communication (CDC); `firmware/vehicle/sdkconfig`), so during normal operation it uses USB-OTG (USB driven by the firmware; seen as `303a:0009`). The "enter download mode" command from esptool (toggling the serial control signals DTR/RTS) is received by the firmware's USB handling, which then resets the chip, so **it arrives only while the firmware is running normally**.

- "Sometimes flashing fails" is presumed to be a state in which the firmware or its USB handling has stopped and the command cannot arrive (not reproduced).
- A power cycle restarts the firmware, so running esptool right after that enters download mode without the button (chapter 3, verified).
- If the firmware stops right after startup, this method cannot enter download mode either. In that case, power cycle while holding the BOOT button.

As a fundamental countermeasure, one candidate is to switch the console to USB-Serial/JTAG (USB driven by a circuit built into the chip; `303a:1001`) with `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`. With this, esptool can reset the chip directly even when the firmware has stopped. However, it has a drawback: opening `idf.py monitor` (`sf flash vehicle -m`) resets the vehicle (chapter 6). This matches the reason the switch was introduced and then reverted on 2026-10-15 (`905e39f5` -> `f47c8a9b`).

## 5. Specification for Scripting (Not Implemented)

Scripting means making the procedure above runnable as a single command.

### Basic Operation

| Name (proposed) | Operation |
|---|---|
| `hub-cycle` | Port 2 of both sides OFF -> wait 2 s -> ON |
| `flash-and-boot` | `hub-cycle` -> wait until the vehicle appears as `303a:0009` (about 6 s measured) -> `esptool --before default_reset --after no_reset write_flash @flash_args` -> `hub-cycle` |
| Guidance on failure | If esptool cannot connect, show "Press Enter while holding the BOOT button"; after `hub-cycle`, when the vehicle appears as `303a:1001`, tell the user to release the button and retry flashing |

### Implementation Notes

- Do not hard-code the hub location (`4-1` / `4-2`). Find it from the `uhubctl` listing by looking for `14b0:013a` and `14b0:0131`.
- Judge the power state from the uhubctl display, not from the USB list (chapter 4).
- The vehicle's serial port name (such as `/dev/cu.usbmodem41301`) changes with the port it is plugged into. Search for it every time before flashing.
- When flashing, use `build/flash_args` (bootloader at 0x0, partition table at 0x8000, application at 0x10000).
- Do nothing in an environment without this hub. If the hub is not found, show the conventional procedure (manual unplugging and replugging).
- To use another hub, the port-to-power mapping must be investigated again by the same method as in chapter 4 (visual LED check). Even for models listed in uhubctl's compatibility table (README), behavior differs by OS.

## 6. Cause of the Firmware Not Starting After Flashing (Additional Tests)

### Conclusion

**After entering download mode with the BOOT button and flashing, resetting with esptool's standard `--after hard_reset` leaves the chip in download mode. With `--after watchdog_reset` it boots normally.** This is the cause of the symptom "the firmware does not start after flashing until the cable is replugged".

- `hard_reset` resets the chip with a USB-Serial/JTAG control signal (RTS). It did not get the chip out of download mode entered with the BOOT button.
- `watchdog_reset` resets the whole chip with its internal watchdog (a circuit that resets the chip automatically when it hangs).
- `sf flash vehicle` runs `idf.py flash` internally (`lib/sfcli/commands/flash.py`), and the reset after flashing is set by `CONFIG_ESPTOOLPY_AFTER="hard_reset"` in `sdkconfig`. This is presumably why the symptom appeared every time the vehicle was flashed via the BOOT button.

### Results

All without the battery. Judged by the startup sound and LED (visual) and by esptool responses.

| How download mode was entered | Reset after flashing | Result | Runs |
|---|---|---|---|
| BOOT button + power on | `hard_reset` | Stays in download mode (no startup sound, LED stopped; the ROM answers esptool with `--before no_reset`) | 1 |
| From the state above | `watchdog_reset` | Boots (startup sound, LED, `303a:0009`, command input works) | 1 |
| BOOT button + power on | `watchdog_reset` | Boots (one startup sound, LED, `303a:0009`) | 1 |
| esptool from running USB-CDC firmware | `hard_reset` | Boots (first test in chapter 3) | 1 |

### Findings with the USB-Serial/JTAG Firmware

For testing, a firmware with the console on USB-Serial/JTAG was used (the change of `905e39f5` re-applied; branch `test/usj-console-hotplug`).

- **Opening `idf.py monitor` resets the vehicle.** The log shows `rst:0x15 (USB_UART_CHIP_RESET)` and the vehicle rebooted even on battery (confirmed by the startup sound). It cannot be used to check the state by connecting USB after boot.
- When the port is opened without toggling the control signals, command input (`stampfly>`) works after connecting USB to a vehicle booted on battery. Boot logs are not kept.
- Flashing the USB-CDC firmware over a running USB-Serial/JTAG firmware and booting it with `hard_reset` leaves **the firmware running (startup sound, LED) but USB unusable** (esptool also reports `No serial data received`). Disconnecting and reconnecting the USB link does not fix it; a power cycle does (reproduced twice).

### Open Items

- Whether switching to `watchdog_reset` causes problems for normal flashing, where esptool enters download mode from running firmware.
- Whether the USB-CDC firmware allows checking the state by connecting USB after boot. The probe program used may have reset the USB-CDC firmware when opening the port, so that result was discarded.
- The symptom "cannot flash even after replugging" has not been reproduced.

## 7. Related

- uhubctl: <https://github.com/mvp/uhubctl>
- [Development Tools Guide](tools.md)
- [Troubleshooting](troubleshooting.md)
