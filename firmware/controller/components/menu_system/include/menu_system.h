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
    SCREEN_STATE_SETTING,       // Setting screen / 設定画面
    SCREEN_STATE_ABOUT,         // About screen / バージョン情報画面
    SCREEN_STATE_BATTERY_WARN,  // Battery warning setting / バッテリー警告設定画面
    SCREEN_STATE_CHANNEL,       // Channel display / チャンネル表示画面
    SCREEN_STATE_MAC,           // MAC address display / MACアドレス表示画面
    SCREEN_STATE_CALIBRATION,   // Stick calibration / スティックキャリブレーション画面
    SCREEN_STATE_STICK_TEST     // Stick/button test / スティック・ボタンテスト画面
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
 * Get scroll offset for rendering
 * 描画用スクロールオフセットを取得
 */
uint8_t menu_get_scroll_offset(void);

/**
 * Get menu item label
 * メニュー項目ラベルを取得
 */
const char* menu_get_item_label(uint8_t index);

/**
 * Menu action callback type
 * メニューアクションコールバック型
 */
typedef void (*menu_action_callback_t)(void);

/**
 * Register USB mode callback
 * USBモードコールバックを登録
 * @param callback Function to call when USB mode is selected
 */
void menu_register_usb_mode_callback(menu_action_callback_t callback);

/**
 * Register Stick Mode callback
 * Stick Modeコールバックを登録
 * @param callback Function to call when Stick Mode is selected
 */
void menu_register_stick_mode_callback(menu_action_callback_t callback);

/**
 * Get current stick mode label for display
 * 表示用の現在のStick Modeラベルを取得
 * @return "Mode 2" or "Mode 3"
 */
const char* menu_get_stick_mode_label(void);

/**
 * Set stick mode label (called from main after mode change)
 * Stick Modeラベルを設定（モード変更後にmainから呼ばれる）
 * @param mode 2 or 3
 */
void menu_set_stick_mode_label(uint8_t mode);

/**
 * Register Battery Warning callback
 * バッテリー警告コールバックを登録
 * @param callback Function to call when Battery Warning is selected
 */
void menu_register_battery_warn_callback(menu_action_callback_t callback);

/**
 * Get current battery warning threshold (voltage * 10)
 * 現在のバッテリー警告閾値を取得（電圧×10）
 * @return threshold value (e.g., 33 = 3.3V)
 */
uint8_t menu_get_battery_warn_threshold(void);

/**
 * Set battery warning threshold (called from main)
 * バッテリー警告閾値を設定（mainから呼ばれる）
 * @param threshold voltage * 10 (e.g., 33 = 3.3V)
 */
void menu_set_battery_warn_threshold(uint8_t threshold);

#ifdef __cplusplus
}
#endif

#endif // MENU_SYSTEM_H
