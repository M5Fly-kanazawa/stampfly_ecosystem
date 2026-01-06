# USB HID Joystick 設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**関連ドキュメント**: [docs/todo/CONTROLLER_MENU_USB_PLAN.md](../../../docs/todo/CONTROLLER_MENU_USB_PLAN.md) - 全体計画（Phase 4）

## 1. 概要

### 目標
AtomS3コントローラをUSB HID Joystickとしてデバイスに認識させ、シミュレータや他のゲーム/アプリで使用可能にする。

### 要件

| 項目 | 内容 |
|------|------|
| モード選択 | 起動時: M5.BtnA押下 → USB HIDモード、それ以外 → ESP-NOWモード |
| ランタイム切替 | メニューから「USB Mode」選択（要再起動） |
| 通信排他 | USB HIDモード中はESP-NOW停止 |
| 入力 | 4軸（Throttle/Roll/Pitch/Yaw）+ 8ボタンをHIDレポートで送信 |

## 2. アーキテクチャ

### モード選択フロー

```
                    ┌─────────────────────────────────────────┐
                    │           起動時選択                    │
                    │  M5.BtnA押下 → USB HID モード          │
                    │  それ以外    → ESP-NOW モード          │
                    └─────────────────────────────────────────┘
                              │                    │
              ┌───────────────┘                    └───────────────┐
              ↓                                                    ↓
┌─────────────────────────┐                        ┌─────────────────────────┐
│     USB HID モード      │                        │    ESP-NOW モード       │
│  ┌─────────────────┐   │                        │  ┌─────────────────┐    │
│  │   飛行画面      │   │                        │  │   飛行画面      │    │
│  │  (HID送信中)    │   │                        │  │  (TDMA送信中)   │    │
│  └─────────────────┘   │                        │  └─────────────────┘    │
└─────────────────────────┘                        └─────────────────────────┘
```

### タスク構成

```
┌─────────────────────────────────────────────────────────────────┐
│ app_main                                                        │
│   ├── 起動モード判定                                            │
│   │      M5.BtnA → USB HID モード                              │
│   │      else   → ESP-NOW モード                               │
│   │                                                             │
│   ├── 共通タスク                                                │
│   │      input_task (100Hz) ← ジョイスティック＆ボタン読み取り │
│   │      display_task (10Hz) ← LCD更新                         │
│   │                                                             │
│   ├── ESP-NOWモード専用                                         │
│   │      tdma_send_task                                         │
│   │      beacon_task (Master時)                                 │
│   │                                                             │
│   └── USB HIDモード専用                                         │
│          usb_hid_task (100Hz) ← HIDレポート送信                │
└─────────────────────────────────────────────────────────────────┘
```

## 3. HIDレポート仕様

### レポートディスクリプタ

```
Usage Page: Generic Desktop (0x01)
Usage: Joystick (0x04)
Collection: Application
  - 4 Axes (X, Y, Z, Rx): 8-bit unsigned (0-255)
  - 8 Buttons: 1-bit each
End Collection
```

### レポート構造（6バイト）

| Offset | Size | 名前 | 説明 |
|--------|------|------|------|
| 0 | 1 | Throttle | スロットル (0-255) |
| 1 | 1 | Roll | ロール (0-255) |
| 2 | 1 | Pitch | ピッチ (0-255) |
| 3 | 1 | Yaw | ヨー (0-255) |
| 4 | 1 | Buttons | bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode, bit4-7:予約 |
| 5 | 1 | Reserved | 0x00 |

### 値マッピング

- atoms3joy入力: 0-4095（12bit）→ HID出力: 0-255（8bit）
- 変換式: `raw * 255 / 4095` または `raw >> 4`

### HIDディスクリプタバイト列

```cpp
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xa1, 0x01,        // COLLECTION (Application)

    // 4軸アナログ
    0x09, 0x30,        //   USAGE (X) - Throttle
    0x09, 0x31,        //   USAGE (Y) - Roll
    0x09, 0x32,        //   USAGE (Z) - Pitch
    0x09, 0x33,        //   USAGE (Rx) - Yaw
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x04,        //   REPORT_COUNT (4)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // ボタン (8個)
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,        //   USAGE_MAXIMUM (Button 8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    0xc0               // END_COLLECTION
};
```

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
| `main/main.cpp` | 起動モード分岐、USBタスク追加 |
| `components/menu_system/src/menu_system.cpp` | USB/ESPモード切替メニュー |

## 5. 画面設計

### USB HIDモード画面

```
┌─────────────────┐
│  USB HID MODE   │
│                 │
│  T: XXX  R: XXX │
│  P: XXX  Y: XXX │
│                 │
│ [A][F][M][O]    │
│                 │
│  Connected: Yes │
└─────────────────┘
```

| 要素 | 説明 |
|------|------|
| T/R/P/Y | Throttle/Roll/Pitch/Yaw の現在値（0-255） |
| [A][F][M][O] | Arm/Flip/Mode/Option ボタン状態 |
| Connected | USB接続状態 |

## 6. シミュレータ連携

### VID/PID

| 項目 | 値 |
|------|------|
| Vendor ID | 0x303A (Espressif) |
| Product ID | 0x8001 (StampFly Controller) |

### simulator/interfaces/joystick.py 更新

```python
VENDOR_ID_STAMPFLY = 0x303a
PRODUCT_ID_STAMPFLY = 0x8001

def read_normalized(self):
    """正規化されたスティック値を取得 (-1.0 ~ 1.0)"""
    data = self.read()
    return {
        'throttle': (data[0] - 128) / 128.0,
        'roll': (data[1] - 128) / 128.0,
        'pitch': (data[2] - 128) / 128.0,
        'yaw': (data[3] - 128) / 128.0,
        'arm': bool(data[4] & 0x01),
        'flip': bool(data[4] & 0x02),
        'mode': bool(data[4] & 0x04),
        'alt_mode': bool(data[4] & 0x08),
    }
```

## 7. 注意事項

| 項目 | 内容 |
|------|------|
| USB/ESP-NOW排他 | 同時使用不可、起動時またはメニューで選択 |
| モード変更 | メニューからの変更は要再起動 |
| 電源 | USB接続時はPCから給電 |
| 互換性 | Windows/macOS/Linux標準HIDドライバで動作 |

---

<a id="english"></a>

## 1. Overview

### Goal
Enable AtomS3 controller to be recognized as a USB HID Joystick by devices for use with simulators and other games/applications.

### Requirements

| Item | Description |
|------|-------------|
| Mode Selection | Boot: M5.BtnA pressed → USB HID mode, otherwise → ESP-NOW mode |
| Runtime Switch | Select "USB Mode" from menu (requires reboot) |
| Communication Exclusivity | ESP-NOW stops during USB HID mode |
| Input | Send 4 axes (Throttle/Roll/Pitch/Yaw) + 8 buttons via HID report |

## 2. Architecture

### Mode Selection Flow

```
                    ┌─────────────────────────────────────────┐
                    │           Boot Selection                │
                    │  M5.BtnA pressed → USB HID Mode        │
                    │  Otherwise       → ESP-NOW Mode        │
                    └─────────────────────────────────────────┘
                              │                    │
              ┌───────────────┘                    └───────────────┐
              ↓                                                    ↓
┌─────────────────────────┐                        ┌─────────────────────────┐
│     USB HID Mode        │                        │    ESP-NOW Mode         │
│  ┌─────────────────┐   │                        │  ┌─────────────────┐    │
│  │   Flight Screen │   │                        │  │   Flight Screen │    │
│  │  (HID sending)  │   │                        │  │  (TDMA sending) │    │
│  └─────────────────┘   │                        │  └─────────────────┘    │
└─────────────────────────┘                        └─────────────────────────┘
```

### Task Structure

```
┌─────────────────────────────────────────────────────────────────┐
│ app_main                                                        │
│   ├── Boot mode detection                                       │
│   │      M5.BtnA → USB HID Mode                                │
│   │      else   → ESP-NOW Mode                                 │
│   │                                                             │
│   ├── Common tasks                                              │
│   │      input_task (100Hz) ← Joystick & button reading        │
│   │      display_task (10Hz) ← LCD update                      │
│   │                                                             │
│   ├── ESP-NOW mode only                                         │
│   │      tdma_send_task                                         │
│   │      beacon_task (when Master)                              │
│   │                                                             │
│   └── USB HID mode only                                         │
│          usb_hid_task (100Hz) ← HID report transmission        │
└─────────────────────────────────────────────────────────────────┘
```

## 3. HID Report Specification

### Report Descriptor

```
Usage Page: Generic Desktop (0x01)
Usage: Joystick (0x04)
Collection: Application
  - 4 Axes (X, Y, Z, Rx): 8-bit unsigned (0-255)
  - 8 Buttons: 1-bit each
End Collection
```

### Report Structure (6 bytes)

| Offset | Size | Name | Description |
|--------|------|------|-------------|
| 0 | 1 | Throttle | Throttle (0-255) |
| 1 | 1 | Roll | Roll (0-255) |
| 2 | 1 | Pitch | Pitch (0-255) |
| 3 | 1 | Yaw | Yaw (0-255) |
| 4 | 1 | Buttons | bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode, bit4-7:reserved |
| 5 | 1 | Reserved | 0x00 |

### Value Mapping

- atoms3joy input: 0-4095 (12bit) → HID output: 0-255 (8bit)
- Formula: `raw * 255 / 4095` or `raw >> 4`

### HID Descriptor Bytes

```cpp
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xa1, 0x01,        // COLLECTION (Application)

    // 4 analog axes
    0x09, 0x30,        //   USAGE (X) - Throttle
    0x09, 0x31,        //   USAGE (Y) - Roll
    0x09, 0x32,        //   USAGE (Z) - Pitch
    0x09, 0x33,        //   USAGE (Rx) - Yaw
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x04,        //   REPORT_COUNT (4)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Buttons (8)
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,        //   USAGE_MAXIMUM (Button 8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    0xc0               // END_COLLECTION
};
```

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
| `main/main.cpp` | Boot mode branching, USB task addition |
| `components/menu_system/src/menu_system.cpp` | USB/ESP mode switch menu |

## 5. Screen Design

### USB HID Mode Screen

```
┌─────────────────┐
│  USB HID MODE   │
│                 │
│  T: XXX  R: XXX │
│  P: XXX  Y: XXX │
│                 │
│ [A][F][M][O]    │
│                 │
│  Connected: Yes │
└─────────────────┘
```

| Element | Description |
|---------|-------------|
| T/R/P/Y | Throttle/Roll/Pitch/Yaw current values (0-255) |
| [A][F][M][O] | Arm/Flip/Mode/Option button states |
| Connected | USB connection status |

## 6. Simulator Integration

### VID/PID

| Item | Value |
|------|-------|
| Vendor ID | 0x303A (Espressif) |
| Product ID | 0x8001 (StampFly Controller) |

### simulator/interfaces/joystick.py Update

```python
VENDOR_ID_STAMPFLY = 0x303a
PRODUCT_ID_STAMPFLY = 0x8001

def read_normalized(self):
    """Get normalized stick values (-1.0 ~ 1.0)"""
    data = self.read()
    return {
        'throttle': (data[0] - 128) / 128.0,
        'roll': (data[1] - 128) / 128.0,
        'pitch': (data[2] - 128) / 128.0,
        'yaw': (data[3] - 128) / 128.0,
        'arm': bool(data[4] & 0x01),
        'flip': bool(data[4] & 0x02),
        'mode': bool(data[4] & 0x04),
        'alt_mode': bool(data[4] & 0x08),
    }
```

## 7. Notes

| Item | Description |
|------|-------------|
| USB/ESP-NOW Exclusivity | Cannot use simultaneously, select at boot or via menu |
| Mode Change | Menu changes require reboot |
| Power | PC provides power via USB |
| Compatibility | Works with standard HID drivers on Windows/macOS/Linux |
