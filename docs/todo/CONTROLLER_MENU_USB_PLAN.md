# コントローラ改造計画：メニューシステム＆USB HIDモード

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
│   │      menu_task (50Hz) ← メニュー状態管理（新規）           │
│   │                                                             │
│   ├── ESP-NOWモード専用                                         │
│   │      tdma_send_task                                         │
│   │      beacon_task (Master時)                                 │
│   │                                                             │
│   └── USB HIDモード専用                                         │
│          usb_hid_task (100Hz) ← HIDレポート送信（新規）        │
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

## 4. USB HIDモード設計

### HIDレポート構造

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

### USB初期化

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

## 5. コンポーネント構成

### 新規コンポーネント

```
firmware/controller/components/
├── menu_system/              # 新規：メニューシステム
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── menu_system.hpp   # メニュー構造体
│   │   ├── menu_state.hpp    # 状態管理
│   │   └── menu_items.hpp    # メニュー項目定義
│   └── src/
│       ├── menu_system.cpp   # メニュー処理
│       ├── menu_render.cpp   # 描画処理
│       └── menu_input.cpp    # 入力処理
│
├── usb_hid/                  # 新規：USB HID
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── usb_hid.hpp
│   └── src/
│       └── usb_hid.cpp
│
└── comm_mode/                # 新規：通信モード管理
    ├── CMakeLists.txt
    ├── include/
    │   └── comm_mode.hpp
    └── src/
        └── comm_mode.cpp
```

### 修正ファイル

| ファイル | 変更内容 |
|----------|----------|
| `main/main.cpp` | 起動モード分岐、メニュータスク追加 |
| `main/CMakeLists.txt` | 新コンポーネント依存追加 |
| `components/atoms3joy/` | スティックのデジタル方向検出追加 |
| `sdkconfig.defaults` | TinyUSB有効化 |

## 6. 実装フェーズ

### Phase 1: メニュー基盤（1-2日）

```
□ menu_system コンポーネント作成
□ MenuState 状態管理実装
□ M5.BtnA をメニュートリガーに変更
□ 基本画面切り替え（FLIGHT ↔ MENU）
```

**成果物**: メニュー画面の表示・非表示が動作

### Phase 2: メニュー操作（1-2日）

```
□ スティックの方向検出実装
  - デッドゾーン設定（中央±500）
  - 方向判定（上下左右）
  - リピート機能（長押し連続入力）
□ カーソル移動
□ 項目選択・決定
□ 値の増減
```

**成果物**: スティックでメニュー操作可能

### Phase 3: メニュー項目実装（2-3日）

```
□ Calibration サブメニュー
  - スティック MIN/MAX キャリブレーション
  - バイアス自動設定
□ TDMA Setup サブメニュー
  - Channel 選択 (1-14)
  - Device ID 選択 (0-9)
  - ※変更後は再起動必要
□ Control サブメニュー
  - Stick Mode (2/3)
  - Rate/Angle デフォルト
□ Battery Warning 閾値設定
□ About 画面（バージョン情報）
```

**成果物**: 実用的なメニューシステム

### Phase 4: USB HID モード（2-3日）

```
□ usb_hid コンポーネント作成
□ TinyUSB 統合
□ HID Joystick ディスクリプタ
□ レポート送信タスク
□ 起動時モード選択ロジック
□ メニューから USB/ESP 切り替え（要再起動）
```

**成果物**: PCにジョイスティックとして認識

### Phase 5: シミュレータ連携（1日）

```
□ simulator/interfaces/joystick.py 更新
  - 新VID/PID対応
  - レポート解析
□ 動作確認・デバッグ
```

**成果物**: シミュレータがコントローラを認識

### Phase 6: 設定永続化（1日）

```
□ NVS (Non-Volatile Storage) 統合
□ 設定保存・読み込み
□ 工場出荷時リセット機能
```

**成果物**: 設定が再起動後も保持

## 7. シミュレータ側の変更

### joystick.py 更新

```python
# simulator/interfaces/joystick.py

# StampFly Controller USB HID
VENDOR_ID_STAMPFLY = 0x303a   # Espressif
PRODUCT_ID_STAMPFLY = 0x8001  # StampFly Controller (新規割当)

class Joystick:
    def __init__(self, vendor_id=None, product_id=None):
        # 自動検出
        if vendor_id is None:
            self.vendor_id, self.product_id = self._auto_detect()
        else:
            self.vendor_id = vendor_id
            self.product_id = product_id

    def _auto_detect(self):
        """接続されているStampFlyコントローラを自動検出"""
        for d in hid.enumerate():
            if d['vendor_id'] == VENDOR_ID_STAMPFLY:
                return d['vendor_id'], d['product_id']
        # フォールバック
        return VENDOR_ID, PRODUCT_ID

    def read_normalized(self):
        """正規化されたスティック値を取得 (-1.0 ~ 1.0)"""
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
- [ ] M5.BtnA でメニュー表示/非表示
- [ ] スティック上下でカーソル移動
- [ ] スティック右で項目選択
- [ ] スティック左で戻る

### Phase 3 テスト
- [ ] キャリブレーション実行・保存
- [ ] TDMA設定変更・再起動確認
- [ ] 各設定項目の動作確認

### Phase 4-5 テスト
- [ ] USB接続でPC認識（デバイスマネージャ/システム情報）
- [ ] Windows ジョイスティックテスト
- [ ] macOS Joystick Doctor等
- [ ] シミュレータ連携テスト

## 9. 制約・注意事項

### ハードウェア制約
- **LCD**: 小型（128x128推定）→ 1画面6-7行が限界
- **ボタン**: M5.BtnA 1個 → スティック併用必須
- **USB/ESP-NOW排他**: 同時使用不可、起動時選択

### ソフトウェア制約
- **TDMA設定変更**: 実行時変更不可、再起動必要
- **USB切り替え**: 実行時変更不可、再起動必要
- **NVS容量**: 約4KB → 設定項目は最小限に

### 互換性
- **既存ペアリング**: 維持（Drone_mac保存形式変更なし）
- **ESP-NOW プロトコル**: 変更なし（14バイトパケット）
- **シミュレータ**: joystick.py更新必要

## 10. 主要ファイル一覧

### 新規作成

| ファイル | 内容 |
|----------|------|
| `components/menu_system/` | メニューシステム一式 |
| `components/usb_hid/` | USB HID一式 |
| `components/comm_mode/` | 通信モード管理 |

### 修正

| ファイル | 変更内容 |
|----------|----------|
| `main/main.cpp` | 起動分岐、タスク追加 |
| `components/atoms3joy/` | 方向検出追加 |
| `sdkconfig.defaults` | TinyUSB有効化 |
| `simulator/interfaces/joystick.py` | 自動検出、正規化 |

## 11. 見積もり

| フェーズ | 作業量 | 累計 |
|----------|--------|------|
| Phase 1: メニュー基盤 | 1-2日 | 1-2日 |
| Phase 2: メニュー操作 | 1-2日 | 2-4日 |
| Phase 3: メニュー項目 | 2-3日 | 4-7日 |
| Phase 4: USB HID | 2-3日 | 6-10日 |
| Phase 5: シミュレータ連携 | 1日 | 7-11日 |
| Phase 6: 設定永続化 | 1日 | 8-12日 |

**合計: 約8-12日（実作業日）**

## 12. Phase 1 完了後のフィードバック・改善項目

### 実装済み（2025-01-06）

- [x] menu_system コンポーネント作成 (`components/menu_system/`)
- [x] ScreenState 状態管理（FLIGHT/MENU/SETTING）
- [x] M5.BtnA 短押しでメニュー切り替え
- [x] 基本画面切り替え（フライト画面 ↔ メニュー画面）
- [x] スティック上下でメニューナビゲーション
- [x] モードボタンでメニュー項目選択
- [x] ビルド成功確認

### 要改善（ユーザーフィードバック）

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

#### 設定項目追加（Phase 3）

| 設定項目 | 内容 |
|------|------|
| ナビゲーションスティック選択 | 左/右スティック選択 |
| 操作音 ON/OFF | メニュー操作音の有効/無効 |
| その他の設定 | NVSに保存 |

### 技術メモ

#### 画面切り替え時の残像対策

```cpp
// 状態変更時に画面をクリア
void menu_set_state(screen_state_t state) {
    if (current_state != state) {
        M5.Display.fillScreen(TFT_BLACK);  // 画面クリア追加
        current_state = state;
        // ...
    }
}
```

#### オリジナル文字色の復元

フライト画面では `setTextColor()` を各行で適切に設定（MODE表示の色分けなど）
