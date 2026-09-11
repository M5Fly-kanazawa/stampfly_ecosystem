# 送信機の使い方

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本書は、StampFly の送信機（コントローラとも呼ぶ）の操作方法をまとめたものです。読者が機体（StampFly）を操縦できるようになることを目的としています。

送信機は ESP-NOW（Espressif社の無線直接通信方式）・UDP・USB HID（USBのゲームパッド等の標準規格）の3つの通信モードを切り替えて使えます。既定（電源を入れた直後の設定）は ESP-NOW で、機体を直接無線操縦するモードです。UDP は機体が開く WiFi アクセスポイント経由で単機運用するモード、USB HID は送信機を PC に接続しシミュレータを操縦するモードです。切り替え方法の詳細は6章、シミュレータでの使い方は11章で扱います。

## 2. 各部の名称

| 部位 | 役割の概要 |
|---|---|
| 画面（LCD、M5ボタンでもある） | 押すとメニューの開閉。表示はタッチ不可、押し込み式のボタンを兼ねる |
| 左スティック | Mode 2/Mode 3でチャンネル割り当てが変わる（8章参照） |
| 右スティック | 同上。メニュー表示中は上下操作でメニュー項目を移動（スティックモードに関わらず常に右スティック） |
| 左スティック押し込みボタン | Mode依存でArmまたはFlip（9章参照） |
| 右スティック押し込みボタン | Mode依存でArmまたはFlip（9章参照） |
| 左ボタン（黄色、左上） | 飛行中は高度モード切替。メニュー内では使わない |
| 右ボタン（黄色、右上） | 飛行中は制御モード切替。メニュー内では「決定」ボタン |

```
     [左ボタン]         [電源スイッチ]         [右ボタン]
   OFF/ALT/POS HOLD                        STABILIZE/ACRO

  (左スティック)                          (右スティック)

              [画面（M5ボタン）]
```

上図は実機の配置を模式化したものです（電源スイッチの操作は本書の対象外）。

## 3. 電源投入と起動

電源スイッチを入れると、通常は既定の ESP-NOW モードで自動的に起動します。起動時には、前回ペアリングした機体の MAC アドレスと通信チャンネルを送信機内部の保存領域（電源を切っても消えない）から読み出し、自動的に再接続を試みます。

画面（M5ボタン）を押しながら電源を入れると、強制ペアリングモードに入ります。画面には候補機体の一覧画面「=== PAIRING ===」が表示されます。詳しい手順は7章で説明します。

電源投入時に左ボタンなどを押していてもスティックモードは変わりません。スティックモードの切り替えはメニュー操作で行います（8章参照）。

## 4. 飛行画面の表示

ESP-NOW モード（既定）でのフライト画面には、上から順に以下の情報が表示されます。

| 表示位置 | 内容 |
|---|---|
| 上段 | ペアリング済み機体の MAC アドレス下位2バイト |
| 2段目 | 送信機（AtomJoyStick）自体のバッテリー電圧（2系統） |
| 3段目 | 現在のスティックモード番号 |
| 4段目 | ESP-NOW の通信チャンネルと Device ID |
| 5段目 | 高度モード（`-Mnual ALT-` / `-Auto ALT-` / `-Pos HOLD-`） |
| 6段目 | 制御モード（`-STABILIZE-` / `-ACRO-`） |
| 7段目 | 送信周波数と同期状態 |

7段目については、（未確認: 複数機体運用時（Device ID が0以外）での同期状態表示は実機で確認できていません）。

UDP モードと USB HID モード中は、画面表示がそれぞれ別の内容に切り替わります。詳細は6章・11章を参照してください。

## 5. メニューの操作

| 操作 | 内容 |
|---|---|
| 開閉 | 画面（M5ボタン）を押すたびに開閉します |
| 移動 | メニュー表示中は、常に物理的な右スティックの上下でカーソルを移動します（スティックモードに関わらず共通） |
| 決定 | 右ボタンを押すと選択中の項目を実行します |
| 戻る | 各設定画面では、画面（M5ボタン）を押すとメニューへ戻ります |

メニュー項目を選び決定ボタンを押したときの動作は以下のとおりです。

| 項目 | 内容 |
|---|---|
| Stick: Mode 2/3 | 決定するたびに Mode 2⇔Mode 3 を切り替える（設定は保存され、切替は即座に反映される） |
| Comm: ESP-NOW/UDP/USB HID | 決定するたびに ESP-NOW→UDP→USB HID→ESP-NOW の順に切り替わる（詳細は6章） |
| Batt: X.XV | 決定すると設定画面に入り、以後決定ボタンを押すたびに警告電圧が3.0V〜4.0Vの範囲で0.1V刻みで変化する。画面（M5ボタン）で確定して戻る |
| Deadband: X% | その場で決定ボタンを押すたびに0%〜5%の範囲で1%刻みに変化する（専用画面には入らない） |
| Stick Test | スティックとボタンの現在値を確認する画面に入る（後述） |
| Calibration | スティックの中立点を再校正する画面に入る（後述） |
| Device ID | 決定すると設定画面に入り、以後決定ボタンを押すたびに ID 0〜9 の範囲でサイクルする |
| Channel | 決定すると設定画面に入る。Device ID が0の送信機のみ、決定ボタンでチャンネルを1/6/11の間でサイクルできる。ID≠0の送信機では、ペアリングにより自動設定されるため変更不可 |
| MAC Address | ペアリング済み機体の MAC アドレスを表示するのみ（変更不可） |
| About | バージョン情報を表示するのみ |
| <- Back | フライト画面へ戻る |

### Calibration（キャリブレーション、スティックの中立点校正）の手順

1. メニューで Calibration を選び決定ボタンを押す
2. スティックには触れず、両スティックを中央付近で静止させたまま数秒待つ（その時点の位置を中立として認識させるため）
3. 位置が安定したら決定（右）ボタンを押して確定する（保存される）
4. 画面（M5ボタン）を押すとキャンセルして戻る
5. 完了すると自動的にメニュー画面へ戻る（専用の完了音はない）

### Stick Test の見方

校正後の左右スティック X/Y 値と、4つのボタンの押下状態（`[AM][FP][AT][AL]` ＝ Arm/Flip/制御モード切替(右)/高度モード切替(左)、押されると表示が反転する）が表示されます。画面（M5ボタン）でメニューへ戻ります。

### Device ID / Channel / MAC Address

Device ID は送信機を識別する番号で、TDMA（時分割による通信衝突回避方式。複数の送信機が同じ通信チャンネルを時間で分け合い、送信の衝突を避ける仕組み）で使われます。Channel は無線の通信チャンネルです。どちらも7章のペアリングと関連し、複数機体・複数送信機を同時に使う場合に重要になります。

## 6. 通信モードの切り替え

| モード | 用途 | 切替時の再起動 |
|---|---|---|
| ESP-NOW | 実機（StampFly）を直接無線操縦する既定モード | — |
| UDP | 機体が開く WiFi アクセスポイントに接続して単機運用する | なし |
| USB HID | PC にゲームパッドとして認識させ、シミュレータを操縦する | あり（自動再起動） |

手順:

1. 画面を押してメニューを開く
2. 右スティック上下で「Comm: ...」の行に合わせる
3. 右ボタン（決定）を押すたびに ESP-NOW→UDP→USB HID→ESP-NOW の順に1段階ずつ切り替わる
4. USB HID へ入る・USB HID から出る操作の時だけ自動的に再起動する

（未確認: ESP-NOW と UDP の間の切替では自動再起動しません。切替後に UDP 通信がうまく始まらない場合は、電源を入れ直してみてください）

## 7. ペアリング（機体との無線接続）

ESP-NOW（Espressif社の無線直接通信方式）でのペアリング手順。以前は「最初に届いた1通」を
無条件に採用していたが、複数組が同時にペアリングすると隣の組と取り違える問題があったため、
現在は**候補一覧から利用者が選んで確定する方式**になっている
（背景は `docs/plans/pairing-methods-plan.md` を参照）:

1. 送信機: 画面（M5ボタン）を押しながら電源を入れる
2. 機体側: 本体のボタンを3秒長押ししてペアリングモードに入る（機体のLEDが青色で速く点滅し、ビープ音が鳴る）
3. 送信機の LCD に `=== PAIRING ===` 画面が表示され、聞こえた機体を受信強度の強い順に
   「MAC下4桁 + チャンネル」（例 `A1B2 CH06`）の一覧として最大6件表示する。まだ何も
   聞こえていなければ「Searching...」と表示される。**一覧の下4桁を機体に貼ったラベルと
   照合すること**（次節参照）
4. 右スティックの上下（または黄ボタン2つ）で選びたい行に合わせ、画面の決定ボタン（M5ボタン）
   を押して確定する。**候補が1件でもこの確定操作は省略できない**（隣の機体しか見えていない
   状況での誤確定を防ぐため）
5. 確定すると画面が「Pairing... waiting for vehicle reply...」に切り替わり、選んだ機体からの
   応答を待つ。応答があればペアリング完了してフライト画面に切り替わる。5秒応答が無ければ
   「No reply」と表示していったん一覧に戻る（機体側がまだペアリングモードか確認すること）
6. ペアリング情報は送信機内部（SPIFFS）に保存され、次回起動時から自動的に同じ機体へ再接続する

やり直す場合は手順1からやり直します（画面を押しながら再度電源を入れる）。

### 取り違え防止（教室・イベント等）

複数組が同じ部屋で同時にペアリングモードに入っても、以下の2つの仕組みで取り違え（隣の組の
機体と誤って組むこと）を防ぐ。

| 仕組み | 内容 |
|---|---|
| 一覧からの選択が必須 | 上記手順4のとおり、「最初に届いた1通」を無条件採用せず、必ず一覧から選んで確定する操作を挟む。機体には MAC 下4桁のラベル（シール）を貼っておき、送信機の候補一覧の表示と照合して自分の機体を選ぶ。ラベルは機体の USB CLI `mac` コマンド（`sf monitor` で接続）で確認できる（機体 ID = ステーション MAC の下4桁。SoftAP の SSID 末尾も同じ値。Wi-Fi スキャンで見える BSSID だけは ESP32 の仕様でこれ + 1）。詳細は[運用マニュアル](../../firmware/vehicle/docs/operation_manual.md)を参照 |
| 機体側の宛先確認 | 機体はペアリング中でも、自分宛（自分の MAC 下3バイトが一致する）操縦電文だけを相手候補にする。別の組の送信機が送る電文は、その機体が選ばれない限り相手候補にならない |

最終確認は**機体の LED が緑色に変わること**（ペア成立の合図）。誤って別のラベルを選んで
しまった場合は、選んだ側の機体の LED が緑になり、自分の機体は青点滅のまま残るので気づける。

### 複数機体・複数送信機を同時に使う場合の注意（教室・イベント等）

各送信機の Device ID を0〜9の中で重複しないように割り当てます（メニューの Device ID 項目）。Device ID が0の送信機のみチャンネル（1/6/11）を変更できるので、グループごとに使用チャンネルを分けたい場合は ID=0 の送信機で設定してください。

## 8. スティックモード（Mode 2 / Mode 3）

| チャンネル | Mode 2 | Mode 3 |
|---|---|---|
| スロットル | 左スティック上下 | 右スティック上下 |
| エルロン（ロール、左右の傾き） | 右スティック左右 | 左スティック左右 |
| エレベータ（ピッチ、前後の傾き） | 右スティック上下 | 左スティック上下 |
| ラダー（ヨー、機首方位） | 左スティック左右 | 右スティック左右 |

切替方法はメニューの「Stick: Mode 2/3」項目のみです（電源投入時の操作による切替はありません。3章の訂正と合わせて確認してください）。

## 9. 飛行中のボタン操作

| 操作 | 動作 |
|---|---|
| スロットル側スティックの押し込み（Mode 2は左、Mode 3は右） | アーム/ディスアーム（モーター始動/停止） |
| もう一方のスティックの押し込み | フリップ動作の指示 |
| 右ボタン | 制御モードを STABILIZE ⇔ ACRO で切り替える（画面表示が変わる） |
| 左ボタン | 高度モードを OFF（手動高度）→ ALT HOLD（高度自動保持）→ POS HOLD（位置自動保持）→ OFF の順で切り替える（画面表示が変わる） |

## 10. 電池警告

メニューの「Batt: X.XV」で、送信機の警告電圧のしきい値（3.0V〜4.0V、0.1V刻み、既定3.3V）を設定・保存できます。

現行ファームウェアでは、この設定値は保存されるだけで、電圧低下時の警告表示や警告音にはまだ使われていません。

## 11. シミュレータで使う（USB HIDモード）

6章の手順で USB HID モードに切り替えると、PC から標準的な USB ゲームパッドとして認識されます（特別なドライバは不要と想定されますが、未確認です）。

スティックの割り当ては8章の Mode 2/Mode 3 設定がそのまま使われます。

シミュレータでの具体的な使い方は、リポジトリ直下の [README.md](../../README.md) にある「まずはシミュレータで飛ばしてみよう！」章を参照してください。

実機操縦に戻すときは、6章の手順で「Comm: USB HID」の行から決定ボタンをもう一度押して ESP-NOW へ戻します（自動再起動）。

## 12. 困ったとき

| 症状 | 確認事項 |
|---|---|
| 起動時に画面が「ESP-NOW: FAIL」で止まる | 無線機能の初期化に失敗しています。起動はそこで完全に停止するので、電源を入れ直してください |
| 起動時に画面が「JOY: FAIL」と表示される | スティック・ボタン基板との内部通信に失敗しています。電源を入れ直してください |
| 起動時に画面が「USB HID: FAIL」と表示される | USB HID 機能の初期化に失敗しています。電源を入れ直してください |
| UDPモードで「AP Not Found」「Check Vehicle」と表示される | 機体の WiFi アクセスポイントが見つかりません。機体の電源と WiFi 機能を確認してください |
| UDPモードで「WiFi: Timeout」と表示される | 機体の WiFi への接続が時間内に完了しませんでした |
| 飛行中に約500ms間隔でビープ音が鳴り続ける | 機体との通信が一定時間途絶えている可能性があります（未確認: 画面表示への反映は実機で確認できていません） |
| 通信モードをESP-NOWからUDPへ切り替えた後、UDPがうまく繋がらない | 切替時は自動再起動しません。繋がらない場合は電源を入れ直してください（未確認: 再起動なしでの動作） |
| Device IDを変更しても複数機体運用時にうまく通信できない | Device IDの変更はその場で保存されますが、通信への反映には再起動が必要です |
| ペアリング画面の一覧に機体が出てこない（「Searching...」のまま） | 機体側もペアリングモードになっているか確認してください（機体のボタン3秒長押し、LEDが青で速く点滅） |
| 確定後「No reply」と表示されて一覧に戻る | 機体側がまだペアリングモード（LEDが青で速く点滅）か確認してください。5秒以内に応答が無いとこの表示になります |
| スティックが中央でもわずかにずれている、飛行が一方向に流れる | メニューのCalibrationを実施してください |
| Batt: の警告電圧を設定しても警告が出ない | 現行ファームウェアでは警告表示・警告音には反映されません（10章参照） |

---

<a id="english"></a>

# Controller Guide

## 1. Overview

This document describes how to operate the StampFly controller (transmitter). Its goal is to let readers fly the vehicle (StampFly).

The controller supports three communication modes that can be switched at any time: ESP-NOW (Espressif's proprietary direct-radio protocol), UDP, and USB HID (the standard USB protocol used by gamepads, etc.). The default mode, active right after power-on, is ESP-NOW, used to fly the real vehicle directly over radio. UDP connects to the WiFi access point the vehicle opens, for single-vehicle operation. USB HID connects the controller to a PC to fly the simulator. Switching between modes is covered in Chapter 6, and simulator use is covered in Chapter 11.

## 2. Parts and Controls

| Part | Role |
|---|---|
| Screen (LCD, also the M5 button) | Press to open/close the menu. The display is not touch-sensitive; it doubles as a push button |
| Left stick | Channel assignment depends on Mode 2/Mode 3 (see Chapter 8) |
| Right stick | Same as above. While the menu is open, up/down on this stick always moves the cursor, regardless of stick mode |
| Left stick push button | Arm or Flip, depending on Mode (see Chapter 9) |
| Right stick push button | Arm or Flip, depending on Mode (see Chapter 9) |
| Left button (yellow, upper left) | In flight: altitude-mode switch. Unused inside the menu |
| Right button (yellow, upper right) | In flight: control-mode switch. Inside the menu: acts as "select" |

```
     [Left Btn]           [Power Switch]          [Right Btn]
   OFF/ALT/POS HOLD                            STABILIZE/ACRO

  (Left Stick)                                (Right Stick)

              [Screen (M5 Button)]
```

The diagram above is a schematic of the physical layout (operating the power switch itself is outside the scope of this document).

## 3. Powering On and Startup

When you flip the power switch, the controller normally boots into the default ESP-NOW mode automatically. At boot, it restores the MAC address and channel of the last paired vehicle from its internal storage (kept across power-off) and automatically attempts to reconnect.

Holding the screen (M5 button) while powering on enters forced pairing mode. The screen shows the candidate list screen, "=== PAIRING ===". The full procedure is in Chapter 7.

Holding the left button or any other button at power-on does not change the stick mode. The stick mode is changed through the menu (see Chapter 8).

## 4. Flight Screen Display

In ESP-NOW mode (default), the flight screen shows the following, top to bottom.

| Position | Content |
|---|---|
| Top row | Lower 2 bytes of the paired vehicle's MAC address |
| 2nd row | The controller's own (AtomJoyStick) battery voltage, 2 rails |
| 3rd row | Current stick mode number |
| 4th row | ESP-NOW communication channel and Device ID |
| 5th row | Altitude mode (`-Mnual ALT-` / `-Auto ALT-` / `-Pos HOLD-`) |
| 6th row | Control mode (`-STABILIZE-` / `-ACRO-`) |
| 7th row | Transmit frequency and sync status |

For the 7th row: (Not verified: the sync status display during multi-vehicle operation — Device ID other than 0 — has not been confirmed on real hardware.)

The UDP and USB HID modes each switch the screen to different content; see Chapters 6 and 11.

## 5. Using the Menu

| Action | Behavior |
|---|---|
| Open/close | Pressing the screen (M5 button) toggles the menu |
| Move | While the menu is open, up/down on the physical right stick always moves the cursor, regardless of stick mode |
| Select | Pressing the right button executes the highlighted item |
| Back | On each settings screen, pressing the screen (M5 button) returns to the menu |

Pressing select on each menu item does the following:

| Item | Behavior |
|---|---|
| Stick: Mode 2/3 | Each select toggles Mode 2⇔Mode 3 (saved, applied immediately) |
| Comm: ESP-NOW/UDP/USB HID | Each select cycles ESP-NOW→UDP→USB HID→ESP-NOW (see Chapter 6) |
| Batt: X.XV | Select enters a settings screen; each further select then steps the warning voltage in 0.1V increments across 3.0V-4.0V. Press the screen (M5 button) to confirm and return |
| Deadband: X% | Each select steps the value in 1% increments across 0%-5%, in place (no dedicated screen) |
| Stick Test | Enters a screen showing live stick/button values (see below) |
| Calibration | Enters the stick neutral-point recalibration screen (see below) |
| Device ID | Select enters a settings screen; each further select cycles the ID across 0-9 |
| Channel | Select enters a settings screen. Only a controller with Device ID 0 can cycle the channel through 1/6/11 by select; controllers with ID != 0 cannot change it, since it is set automatically through pairing |
| MAC Address | Displays the paired vehicle's MAC address only (read-only) |
| About | Displays version information only |
| <- Back | Returns to the flight screen |

### Calibration (recalibrating the stick neutral point)

1. Select Calibration in the menu and press select
2. Do not touch the sticks; hold both sticks near center and wait a few seconds (so the current position can be recognized as neutral)
3. Once the reading is stable, press select (right button) to confirm (it is saved)
4. Press the screen (M5 button) to cancel and return
5. On completion, it returns to the menu automatically (there is no dedicated completion sound)

### Reading Stick Test

Stick Test shows the calibrated left/right stick X/Y values and the state of the four buttons (`[AM][FP][AT][AL]` = Arm/Flip/control-mode switch (right)/altitude-mode switch (left); the display inverts when pressed). Press the screen (M5 button) to return to the menu.

### Device ID / Channel / MAC Address

Device ID identifies a controller for TDMA (Time Division Multiple Access: a scheme where multiple controllers share one radio channel by taking turns in time, avoiding collisions). Channel is the radio channel in use. Both relate to pairing (Chapter 7) and matter when running multiple vehicles/controllers together.

## 6. Switching Communication Modes

| Mode | Use case | Restart on switch |
|---|---|---|
| ESP-NOW | Default mode for flying the real vehicle (StampFly) directly over radio | — |
| UDP | Connects to the WiFi access point the vehicle opens, for single-vehicle operation | None |
| USB HID | Makes the controller appear as a PC gamepad, for flying the simulator | Yes (automatic) |

Steps:

1. Press the screen to open the menu
2. Use the right stick up/down to move to the "Comm: ..." row
3. Each press of the right button (select) advances one step: ESP-NOW→UDP→USB HID→ESP-NOW
4. Only entering or leaving USB HID triggers an automatic restart

(Not verified: switching between ESP-NOW and UDP does not restart automatically. If UDP communication does not start correctly after switching, try power-cycling the controller.)

## 7. Pairing (Connecting to the Vehicle)

Pairing procedure over ESP-NOW (Espressif's direct-radio protocol). It used to adopt the first
packet it heard unconditionally, but that let several pairs pairing at the same time end up
cross-paired with a neighboring set, so the controller now **lists every vehicle it hears and
requires the user to pick one** (see `docs/plans/pairing-methods-plan.md` for the background):

1. Controller: power on while holding the screen (M5 button)
2. Vehicle: hold its button for 3 seconds to enter pairing mode (its LED blinks blue rapidly and it beeps)
3. The controller's LCD shows a `=== PAIRING ===` screen listing every vehicle it hears, strongest
   signal first, as "MAC last 4 hex digits + channel" (e.g. `A1B2 CH06`), up to 6 rows. If nothing
   has been heard yet it shows "Searching...". **Match the last 4 hex digits against the label
   stuck on the vehicle** (see the next section)
4. Move the highlight with the right stick up/down (or the two yellow buttons) and confirm with
   the screen push button (M5 button). **An explicit press is always required, even with a single
   candidate** (this avoids mis-confirming a neighbor's vehicle when only it is visible)
5. After confirming, the screen shows "Pairing... waiting for vehicle reply..." while it waits for
   the chosen vehicle to respond. Success switches to the flight screen; no reply within 5 seconds
   shows "No reply" and returns to the list (check that the vehicle is still in pairing mode)
6. Pairing information is saved inside the controller (SPIFFS) and it automatically reconnects to
   the same vehicle on every subsequent boot

To retry, repeat from step 1 (power on again while holding the screen).

### Avoiding cross-pairing (classrooms, events)

When several pairs enter pairing mode in the same room at the same time, two mechanisms keep them
from cross-pairing (ending up matched with a neighboring pair's vehicle):

| Mechanism | What it does |
|---|---|
| Picking from a list is mandatory | As in step 4 above, the first packet heard is never adopted unconditionally — the user must pick from the list and confirm. Put a sticker with the vehicle's last-4-hex-digit MAC label on each vehicle and match it against the controller's candidate list to find your own vehicle. Read the label with the vehicle's USB CLI `mac` command (connect with `sf monitor`); the vehicle ID is the last 4 hex digits of the station MAC, also the SoftAP SSID tail, while the BSSID seen by a Wi-Fi scanner is that + 1 by ESP32 rule; see the [operation manual](../../firmware/vehicle/docs/operation_manual.md) for details |
| The vehicle checks the destination | Even while pairing, the vehicle only treats a control packet as a bind candidate when it is addressed to itself (the vehicle's own MAC's lower 3 bytes match). A neighboring controller's packets never become a candidate unless that vehicle is the one selected |

The final check is **the vehicle's LED turning green** (the sign that pairing succeeded). If you
pick the wrong label by mistake, that other vehicle's LED turns green while your own vehicle's LED
keeps blinking blue — so the mistake is noticeable.

### Multiple vehicles/controllers at the same location (classrooms, events)

Assign each controller a unique Device ID from 0-9 (the Device ID menu item). Only the controller with Device ID 0 can change the channel (1/6/11); if you need separate channels per group, set it from the ID=0 controller.

## 8. Stick Modes (Mode 2 / Mode 3)

| Channel | Mode 2 | Mode 3 |
|---|---|---|
| Throttle | Left stick up/down | Right stick up/down |
| Aileron (roll, left/right tilt) | Right stick left/right | Left stick left/right |
| Elevator (pitch, forward/back tilt) | Right stick up/down | Left stick up/down |
| Rudder (yaw, heading) | Left stick left/right | Right stick left/right |

The only way to switch is the "Stick: Mode 2/3" menu item (there is no power-on-time switch; see the correction in Chapter 3).

## 9. In-Flight Button Operations

| Operation | Action |
|---|---|
| Pressing the throttle-side stick (left on Mode 2, right on Mode 3) | Arm/disarm (start/stop the motors) |
| Pressing the other stick | Commands a flip |
| Right button | Toggles control mode STABILIZE ⇔ ACRO (screen display changes) |
| Left button | Cycles altitude mode OFF (manual altitude) → ALT HOLD (automatic altitude hold) → POS HOLD (automatic position hold) → OFF (screen display changes) |

## 10. Battery Warning

The "Batt: X.XV" menu item sets and saves the controller's warning-voltage threshold (3.0V-4.0V in 0.1V steps, default 3.3V).

In the current firmware this setting is only saved; it is not yet used for a low-voltage warning display or sound.

## 11. Using It With the Simulator (USB HID Mode)

Switching to USB HID mode (Chapter 6) makes the controller appear as a standard USB gamepad on a PC (a special driver is not expected to be required, but this is not verified).

The stick assignment is whatever Mode 2/Mode 3 setting is active (Chapter 8).

For how to actually fly the simulator, see the "Try the Simulator First!" section of the repository's root [README.md](../../README.md).

To go back to flying the real vehicle, repeat the Chapter 6 steps and press select once more on the "Comm: USB HID" row to return to ESP-NOW (automatic restart).

## 12. Troubleshooting

| Symptom | What to check |
|---|---|
| Boot screen freezes at "ESP-NOW: FAIL" | Radio initialization failed. Boot halts completely there — power-cycle the controller |
| Boot screen shows "JOY: FAIL" | Internal communication with the stick/button board failed. Power-cycle the controller |
| Boot screen shows "USB HID: FAIL" | USB HID initialization failed. Power-cycle the controller |
| UDP mode shows "AP Not Found" / "Check Vehicle" | The vehicle's WiFi access point was not found. Check the vehicle's power and WiFi |
| UDP mode shows "WiFi: Timeout" | Connecting to the vehicle's WiFi did not complete in time |
| A beep repeats roughly every 500ms during flight | Communication with the vehicle may have been lost for a while (not verified: whether this is reflected on screen has not been confirmed on real hardware) |
| After switching from ESP-NOW to UDP, UDP does not connect properly | Switching does not restart automatically. Power-cycle the controller if it does not connect (not verified: behavior without a restart) |
| Changing Device ID does not fix multi-vehicle communication | The Device ID change is saved immediately, but a restart is needed for it to take effect on the radio |
| No vehicle appears in the pairing list ("Searching..." stays) | Check that the vehicle is also in pairing mode (hold its button 3 seconds; its LED should blink blue rapidly) |
| "No reply" appears after confirming, and it returns to the list | Check that the vehicle is still in pairing mode (LED blinking blue rapidly). This shows up whenever there is no reply within 5 seconds |
| Sticks feel slightly off-center, flight drifts one way | Run Calibration from the menu |
| Setting "Batt:" does not produce a warning | The current firmware does not reflect this setting as a display or sound (see Chapter 10) |
