/*
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 *
 * Menu System for StampFly Controller (ESP-IDF version)
 * コントローラー用メニューシステム（ESP-IDF版）
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Screen state enumeration
// 画面状態の列挙
// ============================================================================
typedef enum {
    SCREEN_STATE_FLIGHT = 0,    // Normal flight display / 通常フライト画面
    SCREEN_STATE_MENU,          // Main menu display / メインメニュー画面
    SCREEN_STATE_SETTING        // Setting screen / 設定画面
} screen_state_t;

// ============================================================================
// Menu navigation input
// メニューナビゲーション入力
// ============================================================================
typedef struct {
    int8_t stick_y;         // -1=down, 0=neutral, 1=up
    int8_t stick_x;         // -1=left, 0=neutral, 1=right
    bool confirm;           // Enter/select button
    bool back;              // Back/cancel button
} menu_input_t;

// ============================================================================
// Menu System API
// メニューシステムAPI
// ============================================================================

/**
 * Initialize the menu system
 * メニューシステム初期化
 */
void menu_init(void);

/**
 * Get current screen state
 * 現在の画面状態を取得
 */
screen_state_t menu_get_state(void);

/**
 * Set screen state
 * 画面状態を設定
 */
void menu_set_state(screen_state_t state);

/**
 * Check if menu is active (not in flight mode)
 * メニューがアクティブかチェック（フライトモード以外）
 */
bool menu_is_active(void);

/**
 * Toggle menu on/off
 * メニュー切り替え
 */
void menu_toggle(void);

/**
 * Navigate menu (using stick input)
 * メニューナビゲーション
 */
void menu_navigate(const menu_input_t* input);

/**
 * Move selection up
 * 選択を上に移動
 */
void menu_move_up(void);

/**
 * Move selection down
 * 選択を下に移動
 */
void menu_move_down(void);

/**
 * Select current item
 * 現在の項目を選択
 */
void menu_select(void);

/**
 * Go back / cancel
 * 戻る / キャンセル
 */
void menu_back(void);

/**
 * Get selected menu index
 * 選択されているメニューインデックスを取得
 */
uint8_t menu_get_selected_index(void);

/**
 * Get menu item count
 * メニュー項目数を取得
 */
uint8_t menu_get_item_count(void);

/**
 * Get menu item label
 * メニュー項目ラベルを取得
 */
const char* menu_get_item_label(uint8_t index);

/**
 * USB mode action callback type
 * USBモードアクションコールバック型
 */
typedef void (*usb_mode_callback_t)(void);

/**
 * Register USB mode callback
 * USBモードコールバックを登録
 * @param callback Function to call when USB mode is selected
 */
void menu_register_usb_mode_callback(usb_mode_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // MENU_SYSTEM_H
