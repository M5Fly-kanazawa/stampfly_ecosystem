/*
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 *
 * Menu System Implementation for StampFly Controller (ESP-IDF version)
 * コントローラー用メニューシステム実装（ESP-IDF版）
 */

#include "menu_system.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Menu item definition
// メニュー項目定義
// ============================================================================
typedef void (*menu_callback_t)(void);

typedef struct {
    const char* label;
    menu_callback_t action;
    bool has_submenu;
} menu_item_t;

// ============================================================================
// Private variables
// ============================================================================
static screen_state_t current_state = SCREEN_STATE_FLIGHT;
static uint8_t selected_index = 0;
static uint8_t scroll_offset = 0;

#define MAX_MENU_ITEMS 10
#define VISIBLE_LINES 6

static menu_item_t menu_items[MAX_MENU_ITEMS];
static uint8_t menu_item_count = 0;

// USB mode callback
// USBモードコールバック
static menu_action_callback_t g_usb_mode_callback = NULL;

// Stick mode callback
// Stickモードコールバック
static menu_action_callback_t g_stick_mode_callback = NULL;

// Current stick mode label
// 現在のStickモードラベル
static uint8_t g_stick_mode = 2;  // Default: Mode 2

// Dynamic label buffer for Stick Mode menu item
// Stickモードメニュー項目の動的ラベルバッファ
static char g_stick_mode_label_buf[16] = "Stick: Mode 2";

// Battery warning callback and threshold
// バッテリー警告コールバックと閾値
static menu_action_callback_t g_battery_warn_callback = NULL;
static uint8_t g_battery_warn_threshold = 33;  // Default: 3.3V

// Dynamic label buffer for Battery Warning menu item
// バッテリー警告メニュー項目の動的ラベルバッファ
static char g_battery_warn_label_buf[16] = "Batt: 3.3V";

// ============================================================================
// Menu action callbacks
// メニューアクションコールバック
// ============================================================================
static void action_usb_mode(void) {
    // Call registered callback to switch to USB HID mode
    // 登録されたコールバックを呼び出してUSB HIDモードに切り替え
    if (g_usb_mode_callback != NULL) {
        g_usb_mode_callback();
    }
}

static void action_stick_mode(void) {
    // Toggle stick mode and call registered callback
    // Stickモードを切り替えて登録されたコールバックを呼び出す
    if (g_stick_mode_callback != NULL) {
        g_stick_mode_callback();
    }
}

static void action_stick_calibration(void) {
    // Show stick calibration screen
    // スティックキャリブレーション画面を表示
    menu_set_state(SCREEN_STATE_CALIBRATION);
}

static void action_stick_test(void) {
    // Show stick/button test screen
    // スティック・ボタンテスト画面を表示
    menu_set_state(SCREEN_STATE_STICK_TEST);
}

static void action_channel_setting(void) {
    // Show channel display screen
    // チャンネル表示画面を表示
    menu_set_state(SCREEN_STATE_CHANNEL);
}

static void action_mac_setting(void) {
    // Show MAC address display screen
    // MACアドレス表示画面を表示
    menu_set_state(SCREEN_STATE_MAC);
}

static void action_battery_warn(void) {
    // Show battery warning setting screen
    // バッテリー警告設定画面を表示
    menu_set_state(SCREEN_STATE_BATTERY_WARN);
}

static void action_about(void) {
    // Show about screen
    // バージョン情報画面を表示
    menu_set_state(SCREEN_STATE_ABOUT);
}

static void action_back(void) {
    // Return to flight screen
    // フライト画面に戻る
    menu_set_state(SCREEN_STATE_FLIGHT);
}

// ============================================================================
// Menu System Implementation
// ============================================================================

void menu_init(void) {
    current_state = SCREEN_STATE_FLIGHT;
    selected_index = 0;
    scroll_offset = 0;
    menu_item_count = 0;

    // Define main menu items
    // メインメニュー項目の定義
    menu_items[menu_item_count++] = {g_stick_mode_label_buf, action_stick_mode, false};
    menu_items[menu_item_count++] = {"USB Mode", action_usb_mode, false};
    menu_items[menu_item_count++] = {g_battery_warn_label_buf, action_battery_warn, false};
    menu_items[menu_item_count++] = {"Stick Test", action_stick_test, false};
    menu_items[menu_item_count++] = {"Calibration", action_stick_calibration, false};
    menu_items[menu_item_count++] = {"Channel", action_channel_setting, false};
    menu_items[menu_item_count++] = {"MAC Address", action_mac_setting, false};
    menu_items[menu_item_count++] = {"About", action_about, false};
    menu_items[menu_item_count++] = {"<- Back", action_back, false};
}

screen_state_t menu_get_state(void) {
    return current_state;
}

void menu_set_state(screen_state_t state) {
    if (current_state != state) {
        current_state = state;

        // Reset selection when entering menu
        // メニュー進入時に選択をリセット
        if (state == SCREEN_STATE_MENU) {
            selected_index = 0;
            scroll_offset = 0;
        }
    }
}

bool menu_is_active(void) {
    return current_state != SCREEN_STATE_FLIGHT;
}

void menu_toggle(void) {
    if (current_state == SCREEN_STATE_FLIGHT) {
        menu_set_state(SCREEN_STATE_MENU);
    } else {
        menu_set_state(SCREEN_STATE_FLIGHT);
    }
}

void menu_navigate(const menu_input_t* input) {
    if (current_state != SCREEN_STATE_MENU) return;

    // Up/Down navigation
    // 上下ナビゲーション
    if (input->stick_y > 0) {
        menu_move_up();
    } else if (input->stick_y < 0) {
        menu_move_down();
    }

    // Select/Confirm
    // 選択/決定
    if (input->confirm) {
        menu_select();
    }

    // Back/Cancel
    // 戻る/キャンセル
    if (input->back) {
        menu_back();
    }
}

void menu_move_up(void) {
    if (selected_index > 0) {
        selected_index--;

        // Adjust scroll offset for visibility
        // 表示範囲の調整
        if (selected_index < scroll_offset) {
            scroll_offset = selected_index;
        }
    }
}

void menu_move_down(void) {
    if (selected_index < menu_item_count - 1) {
        selected_index++;

        // Adjust scroll offset for visibility
        // 表示範囲の調整
        if (selected_index >= scroll_offset + VISIBLE_LINES) {
            scroll_offset = selected_index - VISIBLE_LINES + 1;
        }
    }
}

void menu_select(void) {
    if (selected_index < menu_item_count) {
        menu_item_t* item = &menu_items[selected_index];
        if (item->action != NULL) {
            item->action();
        }
    }
}

void menu_back(void) {
    menu_set_state(SCREEN_STATE_FLIGHT);
}

uint8_t menu_get_selected_index(void) {
    return selected_index;
}

uint8_t menu_get_item_count(void) {
    return menu_item_count;
}

uint8_t menu_get_scroll_offset(void) {
    return scroll_offset;
}

// ============================================================================
// Get menu item label (for rendering in main.cpp)
// メニュー項目ラベル取得（main.cppでの描画用）
// ============================================================================
const char* menu_get_item_label(uint8_t index) {
    if (index < menu_item_count) {
        return menu_items[index].label;
    }
    return "";
}

// ============================================================================
// Register USB mode callback
// USBモードコールバック登録
// ============================================================================
void menu_register_usb_mode_callback(menu_action_callback_t callback) {
    g_usb_mode_callback = callback;
}

// ============================================================================
// Stick mode functions
// Stickモード関数
// ============================================================================
void menu_register_stick_mode_callback(menu_action_callback_t callback) {
    g_stick_mode_callback = callback;
}

const char* menu_get_stick_mode_label(void) {
    return (g_stick_mode == 2) ? "Mode 2" : "Mode 3";
}

void menu_set_stick_mode_label(uint8_t mode) {
    g_stick_mode = mode;
    // Update dynamic label buffer
    // 動的ラベルバッファを更新
    if (mode == 2) {
        strcpy(g_stick_mode_label_buf, "Stick: Mode 2");
    } else {
        strcpy(g_stick_mode_label_buf, "Stick: Mode 3");
    }
}

// ============================================================================
// Battery warning functions
// バッテリー警告関数
// ============================================================================
void menu_register_battery_warn_callback(menu_action_callback_t callback) {
    g_battery_warn_callback = callback;
}

uint8_t menu_get_battery_warn_threshold(void) {
    return g_battery_warn_threshold;
}

void menu_set_battery_warn_threshold(uint8_t threshold) {
    g_battery_warn_threshold = threshold;
    // Update dynamic label buffer
    // 動的ラベルバッファを更新
    // Format: "Batt: X.YV" (e.g., 33 -> "Batt: 3.3V")
    int whole = threshold / 10;
    int frac = threshold % 10;
    // Using snprintf for safety
    snprintf(g_battery_warn_label_buf, sizeof(g_battery_warn_label_buf),
             "Batt: %d.%dV", whole, frac);
}
