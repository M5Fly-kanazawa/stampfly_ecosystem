# USB HID Gamepad 設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 目標
AtomS3コントローラをUSB HIDゲームパッドとしてPCに認識させ、シミュレータや他のゲーム/アプリで使用可能にする。

### 要件
| 項目 | 内容 |
|------|------|
| モード切替 | メニューから「USB Mode」選択でHIDモードに切り替え |
| 通信排他 | ESP-NOW TDMAを停止してUSB接続に専念 |
| 入力 | 2軸×2スティック + 4ボタンをHIDレポートで送信 |
| 復帰 | ボタン長押しでフライトモードに復帰 |

## 2. アーキテクチャ

```
┌───────────────────────────────────────────────────────────────┐
│  PC / Simulator                                               │
│    └── USB HID Host (Gamepad/Joystick として認識)             │
└───────────────────────────────────────────────────────────────┘
                           │
                      USB (Full Speed)
                           │
┌───────────────────────────────────────────────────────────────┐
│  AtomS3 Controller (ESP32-S3)                                 │
│                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐       │
│  │ InputTask   │───→│ usb_hid     │───→│ TinyUSB     │       │
│  │ (100Hz)     │    │ component   │    │ HID Device  │       │
│  │ atoms3joy   │    │             │    │             │       │
│  └─────────────┘    └─────────────┘    └─────────────┘       │
│        │                                                      │
│        ↓                                                      │
│  ┌─────────────┐                                             │
│  │ Menu System │ ← action_usb_mode() でモード切替            │
│  │             │   tdma_stop() → USB初期化 → HIDレポート開始 │
│  └─────────────┘                                             │
└───────────────────────────────────────────────────────────────┘
```

## 3. HIDレポート仕様

### レポートディスクリプタ
```
Usage Page: Generic Desktop (0x01)
Usage: Gamepad (0x05)
Collection: Application
  Collection: Physical
    - 4 Axes (X, Y, Z, Rz): 8-bit signed (-127 to 127)
    - 4 Buttons: 1-bit each
  End Collection
End Collection
```

### レポート構造（6バイト）

| Offset | Size | 名前 | 説明 |
|--------|------|------|------|
| 0 | 1 | X | 左スティック X軸 (-127〜127) |
| 1 | 1 | Y | 左スティック Y軸 (-127〜127) |
| 2 | 1 | Z | 右スティック X軸 (-127〜127) |
| 3 | 1 | Rz | 右スティック Y軸 (-127〜127) |
| 4 | 1 | Buttons | bit0-3: ボタン1-4 |
| 5 | 1 | Reserved | 0x00 |

### 値マッピング
- atoms3joy入力: 0-4095 → HID出力: -127〜127
- 変換式: `(raw - 2048) * 127 / 2048`

## 4. 実装ファイル

### 新規作成

| ファイル | 内容 |
|----------|------|
| `components/usb_hid/CMakeLists.txt` | コンポーネント定義 |
| `components/usb_hid/include/usb_hid.hpp` | API定義 |
| `components/usb_hid/src/usb_hid.cpp` | TinyUSB HID実装 |

### 修正

| ファイル | 変更内容 |
|----------|----------|
| `idf_component.yml` | esp_tinyusb依存追加 |
| `sdkconfig.defaults` | TinyUSB HID設定 |
| `components/espnow_tdma/include/espnow_tdma.hpp` | tdma_stop/restart追加 |
| `components/espnow_tdma/src/espnow_tdma.cpp` | 停止/再開実装 |
| `components/menu_system/src/menu_system.cpp` | action_usb_mode実装 |
| `main/main.cpp` | USBモード画面・レポート送信追加 |

## 5. 画面設計

### USBモード画面
```
┌─────────────────┐
│  USB HID MODE   │
│                 │
│  L: XXX  R: XXX │
│  X: XXX  Y: XXX │
│                 │
│ [1][2][3][4]    │
│                 │
│ Hold BTN: Exit  │
└─────────────────┘
```

| 要素 | 説明 |
|------|------|
| L/R/X/Y | 各軸の現在値（-127〜127） |
| [1][2][3][4] | ボタン状態（押下時に反転表示） |
| Hold BTN | 長押しでフライトモードに復帰 |

## 6. 注意事項

| 項目 | 内容 |
|------|------|
| USB/ESP-NOW排他 | USB HIDモード中はESP-NOW停止必須 |
| 復帰処理 | フライトモードに戻る際はUSB切断→TDMA再起動 |
| 電源 | USB接続時はPCから給電（バッテリー消費なし） |
| 互換性 | Windows/macOS/Linux標準HIDドライバで動作 |

---

<a id="english"></a>

## 1. Overview

### Goal
Enable AtomS3 controller to be recognized as a USB HID Gamepad by PC for use with simulators and other games/applications.

### Requirements

| Item | Description |
|------|-------------|
| Mode Switch | Switch to HID mode via "USB Mode" in menu |
| Communication Exclusivity | Stop ESP-NOW TDMA and focus on USB connection |
| Input | Send 2×2 stick axes + 4 buttons via HID report |
| Return | Return to flight mode via long button press |

## 2. Architecture

```
┌───────────────────────────────────────────────────────────────┐
│  PC / Simulator                                               │
│    └── USB HID Host (recognized as Gamepad/Joystick)          │
└───────────────────────────────────────────────────────────────┘
                           │
                      USB (Full Speed)
                           │
┌───────────────────────────────────────────────────────────────┐
│  AtomS3 Controller (ESP32-S3)                                 │
│                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐       │
│  │ InputTask   │───→│ usb_hid     │───→│ TinyUSB     │       │
│  │ (100Hz)     │    │ component   │    │ HID Device  │       │
│  │ atoms3joy   │    │             │    │             │       │
│  └─────────────┘    └─────────────┘    └─────────────┘       │
│        │                                                      │
│        ↓                                                      │
│  ┌─────────────┐                                             │
│  │ Menu System │ ← action_usb_mode() switches mode           │
│  │             │   tdma_stop() → USB init → HID report start │
│  └─────────────┘                                             │
└───────────────────────────────────────────────────────────────┘
```

## 3. HID Report Specification

### Report Descriptor
```
Usage Page: Generic Desktop (0x01)
Usage: Gamepad (0x05)
Collection: Application
  Collection: Physical
    - 4 Axes (X, Y, Z, Rz): 8-bit signed (-127 to 127)
    - 4 Buttons: 1-bit each
  End Collection
End Collection
```

### Report Structure (6 bytes)

| Offset | Size | Name | Description |
|--------|------|------|-------------|
| 0 | 1 | X | Left stick X-axis (-127 to 127) |
| 1 | 1 | Y | Left stick Y-axis (-127 to 127) |
| 2 | 1 | Z | Right stick X-axis (-127 to 127) |
| 3 | 1 | Rz | Right stick Y-axis (-127 to 127) |
| 4 | 1 | Buttons | bit0-3: buttons 1-4 |
| 5 | 1 | Reserved | 0x00 |

### Value Mapping
- atoms3joy input: 0-4095 → HID output: -127 to 127
- Formula: `(raw - 2048) * 127 / 2048`

## 4. Implementation Files

### New Files

| File | Description |
|------|-------------|
| `components/usb_hid/CMakeLists.txt` | Component definition |
| `components/usb_hid/include/usb_hid.hpp` | API definition |
| `components/usb_hid/src/usb_hid.cpp` | TinyUSB HID implementation |

### Modified Files

| File | Changes |
|------|---------|
| `idf_component.yml` | Add esp_tinyusb dependency |
| `sdkconfig.defaults` | TinyUSB HID settings |
| `components/espnow_tdma/include/espnow_tdma.hpp` | Add tdma_stop/restart |
| `components/espnow_tdma/src/espnow_tdma.cpp` | Stop/restart implementation |
| `components/menu_system/src/menu_system.cpp` | action_usb_mode implementation |
| `main/main.cpp` | USB mode screen and report transmission |

## 5. Screen Design

### USB Mode Screen
```
┌─────────────────┐
│  USB HID MODE   │
│                 │
│  L: XXX  R: XXX │
│  X: XXX  Y: XXX │
│                 │
│ [1][2][3][4]    │
│                 │
│ Hold BTN: Exit  │
└─────────────────┘
```

| Element | Description |
|---------|-------------|
| L/R/X/Y | Current axis values (-127 to 127) |
| [1][2][3][4] | Button states (inverted when pressed) |
| Hold BTN | Long press to return to flight mode |

## 6. Notes

| Item | Description |
|------|-------------|
| USB/ESP-NOW Exclusivity | ESP-NOW must be stopped during USB HID mode |
| Return Processing | Disconnect USB and restart TDMA when returning to flight mode |
| Power | PC provides power via USB (no battery consumption) |
| Compatibility | Works with standard HID drivers on Windows/macOS/Linux |
