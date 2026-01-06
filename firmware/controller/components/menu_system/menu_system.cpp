/*
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 *
 * Menu System Implementation for StampFly Controller (ESP-IDF version)
 * コントローラー用メニューシステム実装（ESP-IDF版）
 */

#include "menu_system.h"
#include <string.h>

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

#define MAX_MENU_ITEMS 8
#define VISIBLE_LINES 6

static menu_item_t menu_items[MAX_MENU_ITEMS];
static uint8_t menu_item_count = 0;

// ============================================================================
// Menu action callbacks
// メニューアクションコールバック
// ============================================================================
static void action_usb_mode(void) {
    // TODO: Phase 2 - Switch to USB HID mode
    // USB HIDモードへ切り替え
}

static void action_stick_calibration(void) {
    // TODO: Phase 3 - Stick calibration
    // スティックキャリブレーション
}

static void action_channel_setting(void) {
    // TODO: Phase 3 - Channel setting
    // チャンネル設定
}

static void action_mac_setting(void) {
    // TODO: Phase 3 - MAC address setting
    // MACアドレス設定
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
    menu_items[menu_item_count++] = {"USB Mode", action_usb_mode, false};
    menu_items[menu_item_count++] = {"Calibration", action_stick_calibration, true};
    menu_items[menu_item_count++] = {"Channel", action_channel_setting, true};
    menu_items[menu_item_count++] = {"MAC Address", action_mac_setting, true};
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
