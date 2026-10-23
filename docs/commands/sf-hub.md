# sf hub

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

USB ハブのポート電源を [uhubctl](https://github.com/mvp/uhubctl) で制御し、機体を抜き差しせずにリセット・書き込みモード（BOOT モード）へ入れる開発用コマンドです。対応ハブ: StarTech HB31C3A1CPD3（USB2 側 `14b0:013a`・USB3 側 `14b0:0131`）。ハブの場所（`4-1` 等）は固定せず、VID:PID から毎回検出します。詳しい検証結果は [USB ハブ電源制御ガイド](../guides/usb-hub-power-cycle.md) を参照。

## 2. 構文

```bash
sf hub status
sf hub off
sf hub on
sf hub cycle [--off-time SECONDS] [--boot [--port PORT]]
```

| サブコマンド | 動作 |
|---|---|
| `status` | ハブの場所、ポート 2 の電源状態、接続中の Espressif デバイス（ポート・VID:PID・説明）を表示 |
| `off` / `on` | USB-A 3 ポートの電源を切る／入れる。実行後に状態を読み直して確認し、期待と違えば失敗（終了コード 1） |
| `cycle` | OFF → 待機（既定 2 秒、`--off-time` で変更）→ ON。その後最大 10 秒、Espressif デバイスの出現を待って表示 |
| `cycle --boot` | 対話式。BOOT（G0）ボタンを押したまま Enter → サイクル → ON 直後に「離す」案内 → 約 1.5 秒待ってから ROM 確認。Espressif のシリアルポート（`--port` で指定可、省略時は VID 303a を自動検出）に `esptool --before no_reset --after no_reset chip_id` を実行し、応答すれば書き込みモード確認（ROM ブートローダだけが応答する）。esptool が無い場合は 303a:1001 の出現のみ確認し「未確認」と表示（通常動作中の機体も 303a:1001 になるため） |

## 3. 注意

- 電源が切れるのは USB-A 3 ポートのみ。USB-C ポートは常時給電で切れません。
- 内部では両側（USB2/USB3）のポート 2 を `uhubctl -e`（厳密指定）で操作します。OFF は USB3→USB2、ON は USB2→USB3 の順。
- 電源状態は OS の USB 一覧ではなく uhubctl の表示で判定します。
- `uhubctl` が無い場合は導入方法を表示して終了コード 1。macOS: `brew install uhubctl`。Linux: パッケージ導入と udev ルール設定（sudo 不要化）が必要。
- 対応ハブが見つからなければその旨を表示して終了コード 1。対応機種の追加は `lib/sfcli/commands/hub.py` の `SUPPORTED_HUBS` に追記します。

## 4. 試験

```bash
cd lib && python -m pytest sfcli/commands/test_hub.py
```

subprocess をモックしており、実機・実 uhubctl は不要です。

---

<a id="english"></a>

## 1. Overview

A development command that controls USB hub port power through [uhubctl](https://github.com/mvp/uhubctl), so a board can be reset or put into download (BOOT) mode without unplugging it. Supported hub: StarTech HB31C3A1CPD3 (USB2 half `14b0:013a`, USB3 half `14b0:0131`). Hub locations (e.g. `4-1`) are detected from VID:PID each time, never hard-coded. See the [USB hub power control guide](../guides/usb-hub-power-cycle.md) for the verification details.

## 2. Syntax

```bash
sf hub status
sf hub off
sf hub on
sf hub cycle [--off-time SECONDS] [--boot [--port PORT]]
```

| Subcommand | Action |
|---|---|
| `status` | Show hub locations, port-2 power state and attached Espressif devices (port, VID:PID, description) |
| `off` / `on` | Switch power of the 3 USB-A ports. Re-reads the state afterwards and fails (exit 1) if it is not as expected |
| `cycle` | OFF, wait (default 2 s, `--off-time`), ON; then waits up to 10 s for an Espressif device and prints it |
| `cycle --boot` | Interactive: hold BOOT (G0), press Enter, power cycles, told to release right after ON, then waits ~1.5 s and probes the ROM: runs `esptool --before no_reset --after no_reset chip_id` on the Espressif serial port (`--port` to override; auto-detected by VID 303a otherwise). Only the ROM bootloader answers, so success confirms download mode. If esptool is not importable, only the 303a:1001 appearance is checked and reported as unconfirmed (running firmware may also enumerate as 303a:1001) |

## 3. Notes

- Only the 3 USB-A ports are switched; the USB-C port is always powered.
- Internally port 2 of both halves is switched with `uhubctl -e` (exact). OFF order: USB3 then USB2; ON order: USB2 then USB3.
- Power state is judged from uhubctl output, not from the OS USB device list.
- If `uhubctl` is missing, an install hint is printed and the exit code is 1. macOS: `brew install uhubctl`. Linux: install the package and set udev rules so sudo is not needed.
- If no supported hub is found, this is reported with exit code 1. To add a hub model, append to `SUPPORTED_HUBS` in `lib/sfcli/commands/hub.py`.

## 4. Tests

```bash
cd lib && python -m pytest sfcli/commands/test_hub.py
```

Subprocess is mocked; no hardware or real uhubctl is needed.
