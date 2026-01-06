# コントローラ改造計画：メニューシステム＆USB HIDモード

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 目標
1. **メニューシステム**: M5ボタンでメニュー画面を表示、スティックで操作
2. **デュアルモード**: ESP-NOW（実機制御）とUSB HID（シミュレータ）の切り替え
3. **拡張性**: 将来の設定・機能追加に対応できる設計

### 現状分析

| 項目 | 現状 |
|------|------|
| LCD | 7行固定表示、10Hz更新 |
| ボタン | M5.BtnA（タイマー制御用） |
| スティックボタン | Arm, Flip, Mode, Option |
| 通信 | ESP-NOW TDMA固定 |
| USB | 未実装 |

### 進捗状況

| Phase | 内容 | 状態 |
|-------|------|------|
| Phase 1 | メニュー基盤 | **完了** (2025-01-06) |
| Phase 2 | メニュー操作 | 未着手 |
| Phase 3 | メニュー項目実装 | 未着手 |
| Phase 4 | USB HIDモード | 未着手 |
| Phase 5 | シミュレータ連携 | 未着手 |
| Phase 6 | 設定永続化 | 未着手 |

## 2. アーキテクチャ

### 状態遷移図

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
│  └────────┬────────┘   │                        │  └────────┬────────┘    │
│           │ M5.BtnA    │                        │           │ M5.BtnA     │
│           ↓ 短押し     │                        │           ↓ 短押し      │
│  ┌─────────────────┐   │                        │  ┌─────────────────┐    │
│  │   メニュー画面  │   │                        │  │   メニュー画面  │    │
│  │  スティック操作 │   │                        │  │  スティック操作 │    │
│  └────────┬────────┘   │                        │  └────────┬────────┘    │
│           │ 選択       │                        │           │ 選択        │
│           ↓            │                        │           ↓             │
│  ┌─────────────────┐   │                        │  ┌─────────────────┐    │
│  │   設定画面      │   │                        │  │   設定画面      │    │
│  │  値の増減       │   │                        │  │  値の増減       │    │
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

## 3. メニューシステム設計

### 画面レイアウト

```
┌────────────────────┐
│ ▶ 1. Calibration  │  ← カーソル（▶）
│   2. TDMA Setup   │
│   3. Control Mode │
│   4. USB/ESP Mode │
│   5. Battery Warn │
│   6. About        │
│ [↑↓:選択 OK:決定] │
└────────────────────┘
```

### メニュー操作

| 入力 | 動作 |
|------|------|
| M5.BtnA 短押し | メニュー ON/OFF |
| スティック ↑ | カーソル上移動 |
| スティック ↓ | カーソル下移動 |
| スティック → | 決定 / 値+1 |
| スティック ← | 戻る / 値-1 |
| Arm ボタン | 決定（代替） |

### メニュー構造体

```cpp
// menu_system.hpp

typedef enum {
    MENU_TYPE_SUBMENU,    // サブメニューへ遷移
    MENU_TYPE_ACTION,     // アクション実行
    MENU_TYPE_VALUE_INT,  // 整数値変更
    MENU_TYPE_VALUE_BOOL, // ON/OFF切り替え
    MENU_TYPE_BACK,       // 前の画面に戻る
} MenuItemType;

typedef struct MenuItem {
    const char* label;           // 表示ラベル
    MenuItemType type;           // 項目タイプ
    union {
        struct MenuItem* submenu; // サブメニューポインタ
        void (*action)(void);     // アクション関数
        struct {
            int* value;           // 値へのポインタ
            int min, max, step;   // 範囲
        } value_int;
        bool* value_bool;         // ON/OFF値へのポインタ
    };
    uint8_t submenu_count;        // サブメニュー項目数
} MenuItem;

// メニュー定義例
static MenuItem main_menu[] = {
    {"Calibration", MENU_TYPE_SUBMENU, .submenu = calib_menu, 3},
    {"TDMA Setup",  MENU_TYPE_SUBMENU, .submenu = tdma_menu, 4},
    {"Control",     MENU_TYPE_SUBMENU, .submenu = ctrl_menu, 3},
    {"USB/ESP",     MENU_TYPE_ACTION,  .action = switch_comm_mode},
    {"Battery",     MENU_TYPE_VALUE_INT, .value_int = {&batt_warn, 30, 100, 5}},
    {"About",       MENU_TYPE_ACTION,  .action = show_about},
    {"<< Back",     MENU_TYPE_BACK},
};
```

### メニュー状態管理

```cpp
// menu_state.hpp

typedef enum {
    SCREEN_FLIGHT,   // 飛行画面（通常）
    SCREEN_MENU,     // メニュー画面
    SCREEN_SETTING,  // 設定変更画面
    SCREEN_DIALOG,   // 確認ダイアログ
} ScreenState;

typedef struct {
    ScreenState screen;        // 現在の画面
    MenuItem* current_menu;    // 現在のメニュー
    uint8_t menu_count;        // メニュー項目数
    uint8_t cursor;            // カーソル位置
    uint8_t scroll_offset;     // スクロール位置
    MenuItem* menu_stack[4];   // メニュー階層スタック
    uint8_t stack_depth;       // スタック深さ
} MenuState;

extern MenuState g_menu_state;
```

## 4. USB HIDモード詳細設計

### HIDレポート仕様

#### レポートディスクリプタ

```
Usage Page: Generic Desktop (0x01)
Usage: Joystick (0x04)
Collection: Application
  - 4 Axes (X, Y, Z, Rx): 8-bit unsigned (0-255)
  - 8 Buttons: 1-bit each
End Collection
```

#### レポート構造（6バイト）

| Offset | Size | 名前 | 説明 |
|--------|------|------|------|
| 0 | 1 | Throttle | スロットル (0-255) |
| 1 | 1 | Roll | ロール (0-255) |
| 2 | 1 | Pitch | ピッチ (0-255) |
| 3 | 1 | Yaw | ヨー (0-255) |
| 4 | 1 | Buttons | bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode, bit4-7:予約 |
| 5 | 1 | Reserved | 0x00 |

#### 値マッピング

- atoms3joy入力: 0-4095（12bit）→ HID出力: 0-255（8bit）
- 変換式: `raw * 255 / 4095` または `raw >> 4`

### HIDディスクリプタ実装

```cpp
// usb_hid_report.hpp

#pragma pack(push, 1)
typedef struct {
    uint8_t throttle;    // 0-255 (12bit → 8bit)
    uint8_t roll;        // 0-255
    uint8_t pitch;       // 0-255
    uint8_t yaw;         // 0-255
    uint8_t buttons;     // bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
    uint8_t reserved;
} HIDJoystickReport;
#pragma pack(pop)

// HIDディスクリプタ（Joystick）
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xa1, 0x01,        // COLLECTION (Application)

    // 4軸アナログ
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

    // ボタン (8個)
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

### USB初期化・送信

```cpp
// usb_hid.cpp

#include "tinyusb.h"
#include "class/hid/hid_device.h"

static bool usb_hid_initialized = false;

esp_err_t usb_hid_init(void) {
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,  // Use default
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    usb_hid_initialized = true;
    return ESP_OK;
}

void usb_hid_send_report(const HIDJoystickReport* report) {
    if (!usb_hid_initialized) return;
    if (!tud_hid_ready()) return;

    tud_hid_report(0, report, sizeof(HIDJoystickReport));
}
```

### TinyUSB設定

#### idf_component.yml 追加

```yaml
dependencies:
  espressif/esp_tinyusb: "^1.0.0"
```

#### sdkconfig.defaults 追加

```
CONFIG_TINYUSB_HID_ENABLED=y
CONFIG_TINYUSB_DESC_CUSTOM_VID=0x303A
CONFIG_TINYUSB_DESC_CUSTOM_PID=0x8001
CONFIG_TINYUSB_DESC_MANUFACTURER_STRING="StampFly"
CONFIG_TINYUSB_DESC_PRODUCT_STRING="Controller"
```

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

### VID/PID設定

| 項目 | 値 |
|------|------|
| Vendor ID | 0x303A (Espressif) |
| Product ID | 0x8001 (StampFly Controller) |

## 5. コンポーネント構成

### 新規コンポーネント

```
firmware/controller/components/
├── menu_system/              # メニューシステム（Phase 1で作成済み）
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── menu_system.hpp
│   └── src/
│       └── menu_system.cpp
│
└── usb_hid/                  # 新規：USB HID（Phase 4で作成）
    ├── CMakeLists.txt
    ├── include/
    │   └── usb_hid.hpp
    └── src/
        └── usb_hid.cpp
```

### 修正ファイル

| ファイル | 変更内容 |
|----------|----------|
| `main/main.cpp` | 起動モード分岐、USBタスク追加 |
| `main/CMakeLists.txt` | 新コンポーネント依存追加 |
| `idf_component.yml` | esp_tinyusb依存追加 |
| `sdkconfig.defaults` | TinyUSB有効化 |
| `simulator/interfaces/joystick.py` | 自動検出、正規化 |

## 6. 実装フェーズ

### Phase 1: メニュー基盤 ✅ 完了

```
[x] menu_system コンポーネント作成
[x] MenuState 状態管理実装
[x] M5.BtnA をメニュートリガーに変更
[x] 基本画面切り替え（FLIGHT ↔ MENU）
```

**成果物**: メニュー画面の表示・非表示が動作

### Phase 2: メニュー操作

```
[ ] スティックの方向検出実装
    - デッドゾーン設定（中央±500）
    - 方向判定（上下左右）
    - リピート機能（長押し連続入力）
[ ] カーソル移動
[ ] 項目選択・決定
[ ] 値の増減
```

**成果物**: スティックでメニュー操作可能

### Phase 3: メニュー項目実装

```
[ ] Calibration サブメニュー
    - スティック MIN/MAX キャリブレーション
    - バイアス自動設定
[ ] TDMA Setup サブメニュー
    - Channel 選択 (1-14)
    - Device ID 選択 (0-9)
    - ※変更後は再起動必要
[ ] Control サブメニュー
    - Stick Mode (2/3)
    - Rate/Angle デフォルト
[ ] Battery Warning 閾値設定
[ ] About 画面（バージョン情報）
```

**成果物**: 実用的なメニューシステム

### Phase 4: USB HID モード

```
[ ] usb_hid コンポーネント作成
    - CMakeLists.txt
    - usb_hid.hpp / usb_hid.cpp
[ ] TinyUSB 統合
    - idf_component.yml に依存追加
    - sdkconfig.defaults に設定追加
[ ] HID Joystick ディスクリプタ実装
[ ] レポート送信タスク (100Hz)
[ ] 起動時モード選択ロジック
    - M5.BtnA押下判定
[ ] USB HIDモード専用画面
[ ] メニューから USB/ESP 切り替え（要再起動）
```

**成果物**: PCにジョイスティックとして認識

### Phase 5: シミュレータ連携

```
[ ] simulator/interfaces/joystick.py 更新
    - 新VID/PID対応 (0x303A:0x8001)
    - 自動検出機能
    - read_normalized() 実装
[ ] 動作確認・デバッグ
```

**成果物**: シミュレータがコントローラを認識

### Phase 6: 設定永続化

```
[ ] NVS (Non-Volatile Storage) 統合
[ ] 設定保存・読み込み
[ ] 工場出荷時リセット機能
```

**成果物**: 設定が再起動後も保持

## 7. シミュレータ側の変更

### joystick.py 更新

```python
# simulator/interfaces/joystick.py

# StampFly Controller USB HID
VENDOR_ID_STAMPFLY = 0x303a   # Espressif
PRODUCT_ID_STAMPFLY = 0x8001  # StampFly Controller

class Joystick:
    def __init__(self, vendor_id=None, product_id=None):
        # 自動検出
        # Auto-detect
        if vendor_id is None:
            self.vendor_id, self.product_id = self._auto_detect()
        else:
            self.vendor_id = vendor_id
            self.product_id = product_id

    def _auto_detect(self):
        """接続されているStampFlyコントローラを自動検出"""
        """Auto-detect connected StampFly controller"""
        for d in hid.enumerate():
            if d['vendor_id'] == VENDOR_ID_STAMPFLY:
                return d['vendor_id'], d['product_id']
        # フォールバック
        # Fallback
        return VENDOR_ID, PRODUCT_ID

    def read_normalized(self):
        """正規化されたスティック値を取得 (-1.0 ~ 1.0)"""
        """Get normalized stick values (-1.0 ~ 1.0)"""
        data = self.read()
        if data is None:
            return None

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

## 8. テスト計画

### Phase 1-2 テスト

- [x] M5.BtnA でメニュー表示/非表示
- [ ] スティック上下でカーソル移動
- [ ] スティック右で項目選択
- [ ] スティック左で戻る

### Phase 3 テスト

- [ ] キャリブレーション実行・保存
- [ ] TDMA設定変更・再起動確認
- [ ] 各設定項目の動作確認

### Phase 4-5 テスト

- [ ] USB接続でPC認識（デバイスマネージャ/システム情報）
- [ ] Windows ジョイスティックテスト（joy.cpl）
- [ ] macOS Joystick Doctor等
- [ ] シミュレータ連携テスト

## 9. 制約・注意事項

### ハードウェア制約

| 項目 | 内容 |
|------|------|
| LCD | 小型（128x128）→ 1画面6-7行が限界 |
| ボタン | M5.BtnA 1個 → スティック併用必須 |
| USB/ESP-NOW | 同時使用不可、起動時選択 |

### ソフトウェア制約

| 項目 | 内容 |
|------|------|
| TDMA設定変更 | 実行時変更不可、再起動必要 |
| USB切り替え | 実行時変更不可、再起動必要 |
| NVS容量 | 約4KB → 設定項目は最小限に |

### 互換性

| 項目 | 内容 |
|------|------|
| 既存ペアリング | 維持（Drone_mac保存形式変更なし） |
| ESP-NOW プロトコル | 変更なし（14バイトパケット） |
| シミュレータ | joystick.py更新必要 |
| OS互換性 | Windows/macOS/Linux標準HIDドライバで動作 |

## 10. 主要ファイル一覧

### 新規作成

| ファイル | Phase | 内容 |
|----------|-------|------|
| `components/menu_system/` | 1 | メニューシステム一式（完了） |
| `components/usb_hid/CMakeLists.txt` | 4 | コンポーネント定義 |
| `components/usb_hid/include/usb_hid.hpp` | 4 | API定義 |
| `components/usb_hid/src/usb_hid.cpp` | 4 | TinyUSB HID実装 |

### 修正

| ファイル | Phase | 変更内容 |
|----------|-------|----------|
| `main/main.cpp` | 4 | 起動分岐、USBタスク追加 |
| `idf_component.yml` | 4 | esp_tinyusb依存追加 |
| `sdkconfig.defaults` | 4 | TinyUSB有効化 |
| `simulator/interfaces/joystick.py` | 5 | 自動検出、正規化 |

## 11. Phase 1 完了後のフィードバック

### 実装済み（2025-01-06）

- [x] menu_system コンポーネント作成 (`components/menu_system/`)
- [x] ScreenState 状態管理（FLIGHT/MENU/SETTING）
- [x] M5.BtnA 短押しでメニュー切り替え
- [x] 基本画面切り替え（フライト画面 ↔ メニュー画面）
- [x] スティック上下でメニューナビゲーション
- [x] モードボタンでメニュー項目選択
- [x] ビルド成功確認

### 要改善項目

#### UI/UX改善

| 項目 | 現状 | 改善内容 | 優先度 |
|------|------|----------|--------|
| ナビゲーションスティック | 左スティック（スロットル） | **右スティック上下**に変更 | 高 |
| 画面切り替え残像 | 文字・白背景が残る | `fillScreen()`で画面クリア | 高 |
| 文字色 | メニュー用に変更 | **フライト画面はオリジナルの色設定を維持** | 高 |

#### 操作音機能（新規）

| 項目 | 内容 |
|------|------|
| メニュー操作音 | スティック移動・選択時にビープ音 |
| 音量設定 | 設定メニューで ON/OFF 切り替え可能 |
| 保存 | NVSに設定を永続化（Phase 6） |

### 技術メモ

#### 画面切り替え時の残像対策

```cpp
// 状態変更時に画面をクリア
// Clear screen on state change
void menu_set_state(screen_state_t state) {
    if (current_state != state) {
        M5.Display.fillScreen(TFT_BLACK);  // 画面クリア追加
        current_state = state;
        // ...
    }
}
```

---

<a id="english"></a>

## 1. Overview

### Goals
1. **Menu System**: Display menu screen with M5 button, navigate with sticks
2. **Dual Mode**: Switch between ESP-NOW (vehicle control) and USB HID (simulator)
3. **Extensibility**: Design that supports future settings and feature additions

### Progress Status

| Phase | Content | Status |
|-------|---------|--------|
| Phase 1 | Menu foundation | **Complete** (2025-01-06) |
| Phase 2 | Menu operation | Not started |
| Phase 3 | Menu items | Not started |
| Phase 4 | USB HID mode | Not started |
| Phase 5 | Simulator integration | Not started |
| Phase 6 | Settings persistence | Not started |

## 4. USB HID Mode Detailed Design

### HID Report Specification

#### Report Descriptor

```
Usage Page: Generic Desktop (0x01)
Usage: Joystick (0x04)
Collection: Application
  - 4 Axes (X, Y, Z, Rx): 8-bit unsigned (0-255)
  - 8 Buttons: 1-bit each
End Collection
```

#### Report Structure (6 bytes)

| Offset | Size | Name | Description |
|--------|------|------|-------------|
| 0 | 1 | Throttle | Throttle (0-255) |
| 1 | 1 | Roll | Roll (0-255) |
| 2 | 1 | Pitch | Pitch (0-255) |
| 3 | 1 | Yaw | Yaw (0-255) |
| 4 | 1 | Buttons | bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode, bit4-7:reserved |
| 5 | 1 | Reserved | 0x00 |

#### Value Mapping

- atoms3joy input: 0-4095 (12bit) → HID output: 0-255 (8bit)
- Formula: `raw * 255 / 4095` or `raw >> 4`

### VID/PID Settings

| Item | Value |
|------|-------|
| Vendor ID | 0x303A (Espressif) |
| Product ID | 0x8001 (StampFly Controller) |

## 9. Constraints and Notes

### Hardware Constraints

| Item | Content |
|------|---------|
| LCD | Small (128x128) → 6-7 lines per screen max |
| Button | M5.BtnA only → Must use with sticks |
| USB/ESP-NOW | Cannot use simultaneously, select at boot |

### Software Constraints

| Item | Content |
|------|---------|
| TDMA settings | Cannot change at runtime, requires reboot |
| USB switch | Cannot change at runtime, requires reboot |
| NVS capacity | ~4KB → Minimize settings |

### Compatibility

| Item | Content |
|------|---------|
| Existing pairing | Maintained (no change to Drone_mac format) |
| ESP-NOW protocol | No change (14-byte packet) |
| Simulator | joystick.py update required |
| OS compatibility | Works with standard HID drivers on Windows/macOS/Linux |
