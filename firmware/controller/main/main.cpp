/*
 * StampFly コントローラー - ESP-IDF版
 *
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 *
 * 機能:
 * - ESP-NOW TDMA通信 (マスター/スレーブ対応)
 * - ジョイスティック入力
 * - LCD表示
 * - ブザー音
 */

#include <M5Unified.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_system.h>
#include <string.h>
#include <buzzer.h>
#include <atoms3joy.h>
#include <espnow_tdma.h>
#include <menu_system.h>
#include <usb_hid.hpp>

static const char* TAG = "MAIN";

// Project-specific color macros for AtomS3/GC9107 panel
// AtomS3/GC9107パネル用のプロジェクト独自カラーマクロ
// This panel has R/B swap (TFT_RED→Blue, TFT_BLUE→Red, TFT_GREEN→Green)
// rgb_order=true, R and B channels are swapped
#define SF_BLACK   TFT_BLACK
#define SF_WHITE   TFT_WHITE
#define SF_RED     TFT_BLUE    // Swapped: BLUE displays as RED
#define SF_GREEN   TFT_GREEN   // No swap: GREEN displays as GREEN
#define SF_BLUE    TFT_RED     // Swapped: RED displays as BLUE
#define SF_YELLOW  TFT_CYAN    // R+G→B+G: CYAN displays as YELLOW
#define SF_CYAN    TFT_YELLOW  // G+B→G+R: YELLOW displays as CYAN
#define SF_MAGENTA TFT_MAGENTA // R+B→B+R: same (MAGENTA stays MAGENTA)
#define SF_ORANGE  0x051F      // Custom: swap R/B in original orange

// ログレベル
#define GLOBAL_LOG_LEVEL ESP_LOG_INFO

// 制御モード
#define ANGLECONTROL 0
#define RATECONTROL 1
#define ALT_CONTROL_MODE 1
#define NOT_ALT_CONTROL_MODE 0

// 制御変数
static uint16_t Throttle = 0;
static uint16_t Phi = 0, Theta = 0, Psi = 0;
static uint8_t Mode = ANGLECONTROL;
static uint8_t AltMode = NOT_ALT_CONTROL_MODE;
static uint8_t StickMode = 2;
static float Timer = 0.0f;
static uint8_t Timer_state = 0;
static volatile uint8_t proactive_flag = 0;

// スティックキャリブレーション（NVS保存）
// Stick calibration (stored in NVS)
typedef struct {
    int16_t left_x;    // 左スティックX軸オフセット
    int16_t left_y;    // 左スティックY軸オフセット
    int16_t right_x;   // 右スティックX軸オフセット
    int16_t right_y;   // 右スティックY軸オフセット
} StickCalibration;

static StickCalibration g_stick_cal = {0, 0, 0, 0};

// キャリブレーション中の累積データ
// Calibration accumulation data
static struct {
    int32_t sum_lx, sum_ly, sum_rx, sum_ry;
    uint32_t count;
    bool active;
} g_cal_accum = {0, 0, 0, 0, 0, false};

// 入力タスク用共有データ
typedef struct {
    int16_t throttle_raw;
    int16_t phi_raw;
    int16_t theta_raw;
    int16_t psi_raw;
    bool btn_pressed;
    bool btn_long_pressed;
    uint8_t mode_changed;
    uint8_t alt_mode_changed;
} InputData;

static InputData shared_inputdata;
static SemaphoreHandle_t input_mutex = NULL;
static TaskHandle_t input_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;

// ループタイミング
static uint32_t stime = 0, etime = 0, dtime = 0;
static const float dTime = 0.01f;

// 通信モード (USB HID / ESP-NOW)
// Communication mode (USB HID / ESP-NOW)
typedef enum {
    COMM_MODE_ESPNOW,   // ESP-NOW TDMA mode (default)
    COMM_MODE_USB_HID,  // USB HID Joystick mode
} CommMode;

static CommMode g_comm_mode = COMM_MODE_ESPNOW;

// NVS設定
// NVS settings
#define NVS_NAMESPACE "controller"
#define NVS_KEY_COMM_MODE "comm_mode"
#define NVS_KEY_STICK_MODE "stick_mode"
#define NVS_KEY_BATT_WARN "batt_warn"
#define NVS_KEY_STICK_CAL "stick_cal"

// バッテリー警告閾値 (voltage * 10, e.g., 33 = 3.3V)
// Battery warning threshold
static uint8_t g_battery_warn_threshold = 33;  // Default: 3.3V

// 通信モードをNVSから読み込み
// Load communication mode from NVS
static CommMode load_comm_mode_from_nvs(void) {
    nvs_handle_t handle;
    uint8_t mode = COMM_MODE_ESPNOW;  // デフォルト

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        nvs_get_u8(handle, NVS_KEY_COMM_MODE, &mode);
        nvs_close(handle);
    }

    return (CommMode)mode;
}

// 通信モードをNVSに保存
// Save communication mode to NVS
static esp_err_t save_comm_mode_to_nvs(CommMode mode) {
    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, NVS_KEY_COMM_MODE, (uint8_t)mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// Stick ModeをNVSから読み込み
// Load stick mode from NVS
static uint8_t load_stick_mode_from_nvs(void) {
    nvs_handle_t handle;
    uint8_t mode = STICK_MODE_2;  // デフォルト

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        nvs_get_u8(handle, NVS_KEY_STICK_MODE, &mode);
        nvs_close(handle);
    }

    // 有効な値のみ受け入れ (2 or 3)
    if (mode != STICK_MODE_2 && mode != STICK_MODE_3) {
        mode = STICK_MODE_2;
    }

    return mode;
}

// Stick ModeをNVSに保存
// Save stick mode to NVS
static esp_err_t save_stick_mode_to_nvs(uint8_t mode) {
    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, NVS_KEY_STICK_MODE, mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// Stick Mode切替コールバック（メニューから呼ばれる）
// Stick mode switch callback (called from menu)
static void on_stick_mode_selected(void) {
    // トグル: Mode 2 <-> Mode 3
    uint8_t new_mode = (StickMode == STICK_MODE_2) ? STICK_MODE_3 : STICK_MODE_2;

    if (save_stick_mode_to_nvs(new_mode) == ESP_OK) {
        ESP_LOGI(TAG, "Stick Mode変更: %d -> %d", StickMode, new_mode);
        StickMode = new_mode;
        joy_set_stick_mode(new_mode);
        menu_set_stick_mode_label(new_mode);
    } else {
        ESP_LOGE(TAG, "Stick Mode保存失敗");
    }
}

// バッテリー警告閾値をNVSから読み込み
// Load battery warning threshold from NVS
static uint8_t load_battery_warn_from_nvs(void) {
    nvs_handle_t handle;
    uint8_t threshold = 33;  // デフォルト: 3.3V

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        nvs_get_u8(handle, NVS_KEY_BATT_WARN, &threshold);
        nvs_close(handle);
    }

    // 有効な範囲: 30-40 (3.0V - 4.0V)
    if (threshold < 30 || threshold > 40) {
        threshold = 33;
    }

    return threshold;
}

// バッテリー警告閾値をNVSに保存
// Save battery warning threshold to NVS
static esp_err_t save_battery_warn_to_nvs(uint8_t threshold) {
    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, NVS_KEY_BATT_WARN, threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// バッテリー警告閾値変更コールバック（メニューから呼ばれる）
// Battery warning threshold change callback
static void on_battery_warn_change(void) {
    // サイクル: 3.0V -> 3.1V -> ... -> 4.0V -> 3.0V
    // Cycle: 3.0V -> 3.1V -> ... -> 4.0V -> 3.0V
    uint8_t new_threshold = g_battery_warn_threshold + 1;
    if (new_threshold > 40) {
        new_threshold = 30;
    }

    if (save_battery_warn_to_nvs(new_threshold) == ESP_OK) {
        ESP_LOGI(TAG, "Battery Warning変更: %d.%dV -> %d.%dV",
                 g_battery_warn_threshold / 10, g_battery_warn_threshold % 10,
                 new_threshold / 10, new_threshold % 10);
        g_battery_warn_threshold = new_threshold;
        menu_set_battery_warn_threshold(new_threshold);
    } else {
        ESP_LOGE(TAG, "Battery Warning保存失敗");
    }
}

// スティックキャリブレーションをNVSから読み込み
// Load stick calibration from NVS
static void load_stick_cal_from_nvs(void) {
    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(StickCalibration);
        err = nvs_get_blob(handle, NVS_KEY_STICK_CAL, &g_stick_cal, &size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "キャリブレーション読込: LX=%d LY=%d RX=%d RY=%d",
                     g_stick_cal.left_x, g_stick_cal.left_y,
                     g_stick_cal.right_x, g_stick_cal.right_y);
        } else {
            // 保存データなし、デフォルト値を使用
            memset(&g_stick_cal, 0, sizeof(g_stick_cal));
        }
        nvs_close(handle);
    }
}

// スティックキャリブレーションをNVSに保存
// Save stick calibration to NVS
static esp_err_t save_stick_cal_to_nvs(void) {
    nvs_handle_t handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, NVS_KEY_STICK_CAL, &g_stick_cal, sizeof(StickCalibration));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS set failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "キャリブレーション保存: LX=%d LY=%d RX=%d RY=%d",
             g_stick_cal.left_x, g_stick_cal.left_y,
             g_stick_cal.right_x, g_stick_cal.right_y);
    return err;
}

// キャリブレーション累積開始
// Start calibration accumulation
static void calibration_start(void) {
    g_cal_accum.sum_lx = 0;
    g_cal_accum.sum_ly = 0;
    g_cal_accum.sum_rx = 0;
    g_cal_accum.sum_ry = 0;
    g_cal_accum.count = 0;
    g_cal_accum.active = true;
    ESP_LOGI(TAG, "キャリブレーション開始");
}

// キャリブレーション累積停止
// Stop calibration accumulation
static void calibration_stop(void) {
    g_cal_accum.active = false;
}

// キャリブレーション値を確定してNVS保存
// Confirm calibration values and save to NVS
static void calibration_confirm(void) {
    if (g_cal_accum.count > 0) {
        // 平均値を計算（中央値2048からのオフセット）
        g_stick_cal.left_x = (int16_t)(g_cal_accum.sum_lx / (int32_t)g_cal_accum.count) - 2048;
        g_stick_cal.left_y = (int16_t)(g_cal_accum.sum_ly / (int32_t)g_cal_accum.count) - 2048;
        g_stick_cal.right_x = (int16_t)(g_cal_accum.sum_rx / (int32_t)g_cal_accum.count) - 2048;
        g_stick_cal.right_y = (int16_t)(g_cal_accum.sum_ry / (int32_t)g_cal_accum.count) - 2048;

        save_stick_cal_to_nvs();
        ESP_LOGI(TAG, "キャリブレーション確定 (samples=%lu)", g_cal_accum.count);
    }
    calibration_stop();
}

// キャリブレーション累積更新（入力タスクから呼ばれる）
// Update calibration accumulation
static void calibration_update(uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry) {
    if (g_cal_accum.active) {
        g_cal_accum.sum_lx += lx;
        g_cal_accum.sum_ly += ly;
        g_cal_accum.sum_rx += rx;
        g_cal_accum.sum_ry += ry;
        g_cal_accum.count++;
    }
}

// 補正済みスティック値を取得（0-4095にクランプ）
// Get calibrated stick value (clamped to 0-4095)
static int16_t get_calibrated_value(uint16_t raw, int16_t offset) {
    int32_t result = (int32_t)raw - offset;
    if (result < 0) result = 0;
    if (result > 4095) result = 4095;
    return (int16_t)result;
}

// USBモード切替コールバック（メニューから呼ばれる）
// USB mode switch callback (called from menu)
static void on_usb_mode_selected(void) {
    ESP_LOGI(TAG, "USB HIDモード選択 - NVS保存後に再起動");

    // 現在ESP-NOWモードならUSB HIDに切り替え
    // 現在USB HIDモードならESP-NOWに切り替え（トグル動作）
    CommMode new_mode = (g_comm_mode == COMM_MODE_ESPNOW) ? COMM_MODE_USB_HID : COMM_MODE_ESPNOW;

    if (save_comm_mode_to_nvs(new_mode) == ESP_OK) {
        ESP_LOGI(TAG, "モード保存完了、再起動します");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "モード保存失敗");
    }
}

// millis()相当
static inline uint32_t millis_now(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// 制御モード変更チェック
static uint8_t check_control_mode_change(void)
{
    static uint8_t button_state = 0;
    uint8_t state = 0;

    if (joy_get_mode_button() == 1 && button_state == 0) {
        state = 1;
        button_state = 1;
    } else if (joy_get_mode_button() == 0 && button_state == 1) {
        button_state = 0;
    }

    return state;
}

// 高度モード変更チェック
static uint8_t check_alt_mode_change(void)
{
    static uint8_t button_state = 0;
    uint8_t state = 0;

    if (joy_get_option_button() == 1 && button_state == 0) {
        state = 1;
        button_state = 1;
    } else if (joy_get_option_button() == 0 && button_state == 1) {
        button_state = 0;
    }

    return state;
}

// 入力タスク (100Hz)
static void input_task(void* parameter)
{
    ESP_LOGI(TAG, "入力タスク開始");

    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    InputData local_input;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // M5ボタンとジョイスティック更新
        M5.update();
        joy_update();

        // キャリブレーション中は物理スティック値を累積
        // Accumulate physical stick values during calibration
        if (g_cal_accum.active) {
            calibration_update(
                joy_get_stick_left_x(),
                joy_get_stick_left_y(),
                joy_get_stick_right_x(),
                joy_get_stick_right_y()
            );
        }

        // 生値読み取り
        local_input.throttle_raw = joy_get_throttle();
        local_input.phi_raw = joy_get_aileron();
        local_input.theta_raw = joy_get_elevator();
        local_input.psi_raw = joy_get_rudder();

        // ボタン状態
        local_input.btn_pressed = M5.BtnA.wasPressed();
        local_input.btn_long_pressed = M5.BtnA.pressedFor(400);

        // モード変更チェック (フラグは一度セットしたらメインループがリセットするまで保持)
        if (check_control_mode_change()) {
            local_input.mode_changed = 1;
        }
        if (check_alt_mode_change()) {
            local_input.alt_mode_changed = 1;
        }

        // 共有バッファ更新
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            // mode_changedとalt_mode_changedはフラグとして保持
            uint8_t prev_mode_changed = shared_inputdata.mode_changed;
            uint8_t prev_alt_mode_changed = shared_inputdata.alt_mode_changed;
            memcpy(&shared_inputdata, &local_input, sizeof(InputData));
            if (prev_mode_changed) shared_inputdata.mode_changed = 1;
            if (prev_alt_mode_changed) shared_inputdata.alt_mode_changed = 1;
            xSemaphoreGive(input_mutex);
        }

        // ローカルフラグをリセット (次のエッジ検出のため)
        local_input.mode_changed = 0;
        local_input.alt_mode_changed = 0;
    }
}

// RGB order設定を適用するヘルパー関数
// Helper function to apply RGB order setting
static void apply_rgb_order(void)
{
    auto panel = M5.Display.getPanel();
    if (panel) {
        auto cfg = panel->config();
        cfg.rgb_order = true;  // R/B swap for AtomS3/GC9107
        panel->config(cfg);
    }
}

// LCD初期化
static void init_display(void)
{
    M5.Display.init();

    // Apply rgb_order AFTER init() to prevent reset
    // init()の後にrgb_orderを適用（リセット防止）
    apply_rgb_order();

    M5.Display.setRotation(0);
    M5.Display.setTextSize(1);
    M5.Display.setTextFont(2);
    M5.Display.fillScreen(SF_BLACK);

    ESP_LOGI(TAG, "LCD初期化完了 (rgb_order=true)");
}

// メニュー画面描画
static void render_menu_screen(void)
{
    const int line_height = 17;
    const int visible_lines = 6;

    // タイトルバー
    M5.Display.setCursor(4, 2);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("=== MENU ===    ");

    // メニュー項目（スクロール対応）
    uint8_t item_count = menu_get_item_count();
    uint8_t selected = menu_get_selected_index();
    uint8_t scroll_offset = menu_get_scroll_offset();

    for (int i = 0; i < visible_lines; i++) {
        uint8_t item_index = scroll_offset + i;
        M5.Display.setCursor(4, 2 + (i + 1) * line_height);

        if (item_index < item_count) {
            if (item_index == selected) {
                M5.Display.setTextColor(SF_BLACK, SF_WHITE);
                M5.Display.printf("> %-12s", menu_get_item_label(item_index));
            } else {
                M5.Display.setTextColor(SF_WHITE, SF_BLACK);
                M5.Display.printf("  %-12s", menu_get_item_label(item_index));
            }
        } else {
            // 空行をクリア
            M5.Display.setTextColor(SF_WHITE, SF_BLACK);
            M5.Display.printf("                ");
        }
    }
}

// バッテリー警告設定画面描画
// Battery warning setting screen rendering
static void render_battery_warn_screen(void)
{
    const int line_height = 17;

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("= BATT WARN =   ");

    // 行1: 空行
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.printf("                ");

    // 行2: 現在の閾値
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("Threshold:      ");

    // 行3: 値（大きく表示）
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("   %d.%dV        ",
                      g_battery_warn_threshold / 10,
                      g_battery_warn_threshold % 10);

    // 行4: 範囲
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("(3.0V - 4.0V)   ");

    // 行5: 空行
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.printf("                ");

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_GREEN, SF_BLACK);
    M5.Display.printf("Mode:+  BTN:Back");
}

// Calibration画面描画
// Calibration screen rendering
static void render_calibration_screen(void)
{
    const int line_height = 17;

    // 累積平均のオフセット値を計算（中央値2048からの差）
    // Calculate average offset from center (2048)
    int16_t avg_lx = 0, avg_ly = 0, avg_rx = 0, avg_ry = 0;
    if (g_cal_accum.count > 0) {
        avg_lx = (int16_t)(g_cal_accum.sum_lx / (int32_t)g_cal_accum.count) - 2048;
        avg_ly = (int16_t)(g_cal_accum.sum_ly / (int32_t)g_cal_accum.count) - 2048;
        avg_rx = (int16_t)(g_cal_accum.sum_rx / (int32_t)g_cal_accum.count) - 2048;
        avg_ry = (int16_t)(g_cal_accum.sum_ry / (int32_t)g_cal_accum.count) - 2048;
    }

    // 行0: タイトル（サンプル数表示）
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("CAL N=%5lu     ", g_cal_accum.count);

    // 行1: 左スティック平均オフセット
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("L Avg:          ");

    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf(" X:%+5d Y:%+5d", avg_lx, avg_ly);

    // 行3: 右スティック平均オフセット
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("R Avg:          ");

    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf(" X:%+5d Y:%+5d", avg_rx, avg_ry);

    // 行5: 保存済みキャリブレーション値
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("Saved:%+4d %+4d",
                      g_stick_cal.left_x, g_stick_cal.right_x);

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_GREEN, SF_BLACK);
    M5.Display.printf("Mode:OK BTN:Canc");
}

// Stick Test画面描画（補正済み値を表示）
// Stick Test screen rendering (shows calibrated values)
static void render_stick_test_screen(void)
{
    const int line_height = 17;

    // 補正済みスティック値を計算
    // Calculate calibrated stick values
    uint16_t lx_raw = joy_get_stick_left_x();
    uint16_t ly_raw = joy_get_stick_left_y();
    uint16_t rx_raw = joy_get_stick_right_x();
    uint16_t ry_raw = joy_get_stick_right_y();

    int16_t lx = get_calibrated_value(lx_raw, g_stick_cal.left_x);
    int16_t ly = get_calibrated_value(ly_raw, g_stick_cal.left_y);
    int16_t rx = get_calibrated_value(rx_raw, g_stick_cal.right_x);
    int16_t ry = get_calibrated_value(ry_raw, g_stick_cal.right_y);

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("= STICK TEST =  ");

    // 行1: 左スティック（補正済み）
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("L X:%4d Y:%4d ", lx, ly);

    // 行2: 右スティック（補正済み）
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.printf("R X:%4d Y:%4d ", rx, ry);

    // 行3: ボタン状態タイトル
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("Buttons:        ");

    // 行4: ボタン状態（押下時に反転表示）
    M5.Display.setCursor(4, 2 + 4 * line_height);
    uint8_t arm = joy_get_arm_button();
    uint8_t flip = joy_get_flip_button();
    uint8_t mode = joy_get_mode_button();
    uint8_t opt = joy_get_option_button();

    if (arm) M5.Display.setTextColor(SF_BLACK, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("[AM]");

    if (flip) M5.Display.setTextColor(SF_BLACK, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("[FP]");

    if (mode) M5.Display.setTextColor(SF_BLACK, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("[AT]");

    if (opt) M5.Display.setTextColor(SF_BLACK, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("[AL]");

    // 行5: StickMode表示
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("StickMode: %d    ", StickMode);

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_GREEN, SF_BLACK);
    M5.Display.printf("Press BTN: Back ");
}

// Channel画面描画
// Channel screen rendering
static void render_channel_screen(void)
{
    const int line_height = 17;

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("== CHANNEL ==   ");

    // 行1: 空行
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.printf("                ");

    // 行2: ESP-NOW Channel
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("ESP-NOW:        ");

    // 行3: チャンネル番号（大きく表示）
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("   CH %02d        ", ESPNOW_CHANNEL);

    // 行4: Device ID
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("Device ID: %d    ", TDMA_DEVICE_ID);

    // 行5: 空行
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.printf("                ");

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_GREEN, SF_BLACK);
    M5.Display.printf("Press BTN: Back ");
}

// MAC Address画面描画
// MAC Address screen rendering
static void render_mac_screen(void)
{
    const int line_height = 17;
    const uint8_t* drone_mac = get_drone_peer_addr();

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("= MAC ADDRESS = ");

    // 行1: 空行
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.printf("                ");

    // 行2: Paired Drone
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("Paired Drone:   ");

    // 行3: MAC上位3バイト
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("%02X:%02X:%02X:       ",
                      drone_mac[0], drone_mac[1], drone_mac[2]);

    // 行4: MAC下位3バイト
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("   %02X:%02X:%02X    ",
                      drone_mac[3], drone_mac[4], drone_mac[5]);

    // 行5: 空行
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.printf("                ");

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_GREEN, SF_BLACK);
    M5.Display.printf("Press BTN: Back ");
}

// About画面描画
// About screen rendering
static void render_about_screen(void)
{
    const int line_height = 17;

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.printf("=== ABOUT ===   ");

    // 行1: バージョン
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    M5.Display.printf("StampFly v2.0   ");

    // 行2: ファームウェア
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.printf("Controller      ");

    // 行3: 空行
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.printf("                ");

    // 行4: 著作権
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.printf("(c) 2024-2025   ");

    // 行5: 著者
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.printf("Kouhei Ito     ");

    // 行6: 操作説明
    M5.Display.setCursor(4, 2 + 6 * line_height);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("Press BTN: Back ");
}

// ESP-NOWフライト画面更新
// ESP-NOW flight screen update
static void update_display(void)
{
    const int line_height = 17;

    // フライト画面描画
    // Flight screen rendering with StickMode-based color
    // StickMode 2: 緑 (GREEN), StickMode 3: 黄 (YELLOW)
    const uint8_t* drone_mac = get_drone_peer_addr();
    uint16_t base_color = (StickMode == 2) ? SF_GREEN : SF_YELLOW;  // uint16_t に変更

    // 行0: MACアドレス
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    M5.Display.printf("MAC ADR %02X:%02X    ", drone_mac[4], drone_mac[5]);

    // 行1: バッテリー電圧
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    M5.Display.printf("BAT 1:%4.1f 2:%4.1f",
        joy_get_battery_voltage1(), joy_get_battery_voltage2());

    // 行2: スティックモード
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    M5.Display.printf("MODE: %d        ", StickMode);

    // 行3: チャンネル/ID
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    M5.Display.printf("CH: %02d ID: %d  ", ESPNOW_CHANNEL, TDMA_DEVICE_ID);

    // 行4: 高度モード
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    if (AltMode == ALT_CONTROL_MODE)
        M5.Display.printf("-Auto ALT-  ");
    else
        M5.Display.printf("-Mnual ALT- ");

    // 行5: 制御モード
    M5.Display.setCursor(4, 2 + 5 * line_height);
    M5.Display.setTextColor(base_color, SF_BLACK);
    if (Mode == ANGLECONTROL)
        M5.Display.printf("-STABILIZE-");
    else
        M5.Display.printf("-ACRO-     ");

    // 行6: 周波数/同期状態
    M5.Display.setCursor(4, 2 + 6 * line_height);
    #if TDMA_DEVICE_ID == 0
        M5.Display.setTextColor(base_color, SF_BLACK);
        M5.Display.printf("Freq:%4d M    ", (int)actual_send_freq_hz);
    #else
        if (first_beacon_received) {
            if (is_beacon_lost()) {
                M5.Display.setTextColor(SF_RED, SF_BLACK);
                M5.Display.printf("F:%4d LOST!  ", (int)actual_send_freq_hz);
            } else {
                M5.Display.setTextColor(SF_GREEN, SF_BLACK);
                M5.Display.printf("F:%4d SYNC   ", (int)actual_send_freq_hz);
            }
        } else {
            M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
            M5.Display.printf("F:%4d WAIT   ", (int)actual_send_freq_hz);
        }
    #endif
}

// LCD更新タスク (低優先度, 10Hz)
// LCD update task (low priority, 10Hz)
static volatile bool display_task_enabled = false;

// 前方宣言 / Forward declarations
static void update_usb_hid_display(void);
static void usb_hid_main_loop(void);

static void display_task(void* parameter)
{
    ESP_LOGI(TAG, "LCD更新タスク開始");

    // メインループ開始まで待機
    while (!display_task_enabled) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 画面状態追跡（状態変化時に画面クリア）
    // Track screen state for clearing on transition
    static screen_state_t prev_screen_state = SCREEN_STATE_FLIGHT;
    static bool first_render = true;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        apply_rgb_order();

        screen_state_t current_screen_state = menu_get_state();

        // 画面状態変化時または初回描画時にクリア
        // USB HIDフライト画面は青背景、それ以外は黒背景
        if (first_render || prev_screen_state != current_screen_state) {
            if (current_screen_state == SCREEN_STATE_FLIGHT && g_comm_mode == COMM_MODE_USB_HID) {
                M5.Display.fillScreen(SF_BLUE);
            } else {
                M5.Display.fillScreen(SF_BLACK);
            }
            prev_screen_state = current_screen_state;
            first_render = false;
        }

        // 画面状態に応じて描画（両モード共通）
        // Render based on screen state (common to both modes)
        if (current_screen_state == SCREEN_STATE_BATTERY_WARN) {
            render_battery_warn_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_CALIBRATION) {
            render_calibration_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_STICK_TEST) {
            render_stick_test_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_CHANNEL) {
            render_channel_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_MAC) {
            render_mac_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_ABOUT) {
            render_about_screen();
            continue;
        }
        if (current_screen_state == SCREEN_STATE_MENU) {
            render_menu_screen();
            continue;
        }

        // フライト画面はモードに応じて分岐
        // Flight screen branches by mode
        if (g_comm_mode == COMM_MODE_USB_HID) {
            update_usb_hid_display();
        } else {
            update_display();
        }
    }
}

// USB HIDモード画面更新
// USB HID mode display update
static void update_usb_hid_display(void)
{
    const int line_height = 17;
    InputData local_input;

    // 入力データ取得
    if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        memcpy(&local_input, &shared_inputdata, sizeof(InputData));
        xSemaphoreGive(input_mutex);
    } else {
        memset(&local_input, 0, sizeof(InputData));
    }

    // キャリブレーション適用（StickModeに応じてマッピング）
    // Apply calibration based on StickMode
    int16_t _throttle, _phi, _theta, _psi;
    if (StickMode == STICK_MODE_2) {
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.left_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.right_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.right_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.left_x);
    } else {
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.right_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.left_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.left_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.right_x);
    }

    // 8bit変換値（補正済み）
    uint8_t t_val = convert_12bit_to_8bit(4095 - _throttle);
    uint8_t r_val = convert_12bit_to_8bit(_phi);
    uint8_t p_val = convert_12bit_to_8bit(_theta);
    uint8_t y_val = convert_12bit_to_8bit(_psi);

    // USB HIDモードは青背景
    // USB HID mode uses blue background
    const uint16_t bg_color = SF_BLUE;

    // 行0: タイトル
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.setTextColor(SF_YELLOW, bg_color);
    M5.Display.printf("= USB HID MODE =");

    // 行1: 空行
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("                ");

    // 行2: T/R値
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("T:%3d    R:%3d  ", t_val, r_val);

    // 行3: P/Y値
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.printf("P:%3d    Y:%3d  ", p_val, y_val);

    // 行4: 空行
    M5.Display.setCursor(4, 2 + 4 * line_height);
    M5.Display.printf("                ");

    // 行5: ボタン状態
    M5.Display.setCursor(4, 2 + 5 * line_height);
    uint8_t arm = joy_get_arm_button();
    uint8_t flip = joy_get_flip_button();
    uint8_t mode = joy_get_mode_button();
    uint8_t opt = joy_get_option_button();

    // ボタン表示（押下時は反転）
    if (arm) M5.Display.setTextColor(bg_color, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("[A]");

    if (flip) M5.Display.setTextColor(bg_color, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("[F]");

    if (mode) M5.Display.setTextColor(bg_color, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("[M]");

    if (opt) M5.Display.setTextColor(bg_color, SF_WHITE);
    else M5.Display.setTextColor(SF_WHITE, bg_color);
    M5.Display.printf("[O]  ");

    // 行6: 接続状態
    M5.Display.setCursor(4, 2 + 6 * line_height);
    if (usb_hid_is_mounted()) {
        M5.Display.setTextColor(SF_GREEN, bg_color);
        M5.Display.printf("Connected: Yes ");
    } else {
        M5.Display.setTextColor(SF_YELLOW, bg_color);
        M5.Display.printf("Connected: No  ");
    }
}

// USB HIDモードメインループ
// USB HID mode main loop
static void usb_hid_main_loop(void)
{
    InputData local_input;

    // 入力データ取得
    if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        memcpy(&local_input, &shared_inputdata, sizeof(InputData));
        xSemaphoreGive(input_mutex);
    } else {
        memset(&local_input, 0, sizeof(InputData));
    }

    // 画面状態取得
    screen_state_t current_state = menu_get_state();

    // キャリブレーション画面: 累積開始/停止制御
    // Calibration screen: control accumulation start/stop
    static screen_state_t prev_state_usb = SCREEN_STATE_FLIGHT;
    if (current_state == SCREEN_STATE_CALIBRATION && prev_state_usb != SCREEN_STATE_CALIBRATION) {
        // キャリブレーション画面に入った
        calibration_start();
    } else if (current_state != SCREEN_STATE_CALIBRATION && prev_state_usb == SCREEN_STATE_CALIBRATION) {
        // キャリブレーション画面から出た（キャンセル）
        calibration_stop();
    }
    prev_state_usb = current_state;

    // キャリブレーション画面: Modeボタンで確定
    // Calibration screen: Mode button confirms
    if (current_state == SCREEN_STATE_CALIBRATION && local_input.mode_changed == 1) {
        calibration_confirm();
        menu_set_state(SCREEN_STATE_MENU);
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    // ボタンイベント処理（メニュー）
    // Button event handling (menu)
    if (local_input.btn_pressed) {
        if (current_state == SCREEN_STATE_ABOUT ||
            current_state == SCREEN_STATE_BATTERY_WARN ||
            current_state == SCREEN_STATE_CHANNEL ||
            current_state == SCREEN_STATE_MAC ||
            current_state == SCREEN_STATE_CALIBRATION ||
            current_state == SCREEN_STATE_STICK_TEST) {
            menu_set_state(SCREEN_STATE_MENU);
        } else {
            menu_toggle();
        }
    }

    // メニューナビゲーション（メニュー状態時のみ）
    // Menu navigation (when in menu state only)
    // 常に物理的な右スティックを使用（StickModeに依存しない）
    // Always use physical right stick (independent of StickMode)
    if (current_state == SCREEN_STATE_MENU) {
        static uint32_t last_nav_time = 0;
        const uint32_t NAV_DEBOUNCE_MS = 200;
        uint32_t now = millis_now();

        // 物理右スティックY値を使用
        // Use physical right stick Y value
        int16_t stick_y = (int16_t)joy_get_stick_right_y() - 2048;

        if (now - last_nav_time > NAV_DEBOUNCE_MS) {
            if (stick_y > 800) {
                menu_move_up();
                last_nav_time = now;
            } else if (stick_y < -800) {
                menu_move_down();
                last_nav_time = now;
            }

            if (local_input.mode_changed == 1) {
                menu_select();
                last_nav_time = now;
                if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    shared_inputdata.mode_changed = 0;
                    xSemaphoreGive(input_mutex);
                }
            }
        }
    }

    // Battery warning screen: Mode button changes threshold
    // バッテリー警告画面: Modeボタンで閾値変更
    if (menu_get_state() == SCREEN_STATE_BATTERY_WARN && local_input.mode_changed == 1) {
        on_battery_warn_change();
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    // メニューアクティブ時はHIDレポートを送信しない
    // Don't send HID report when menu is active
    if (menu_is_active()) {
        return;
    }

    // キャリブレーション適用（StickModeに応じてマッピング）
    // Apply calibration based on StickMode
    int16_t _throttle, _phi, _theta, _psi;
    if (StickMode == STICK_MODE_2) {
        // Mode 2: Throttle=左Y, Aileron=右X, Elevator=右Y, Rudder=左X
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.left_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.right_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.right_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.left_x);
    } else {
        // Mode 3: Throttle=右Y, Aileron=左X, Elevator=左Y, Rudder=右X
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.right_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.left_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.left_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.right_x);
    }

    // HIDレポート作成
    HIDJoystickReport report;
    report.throttle = convert_12bit_to_8bit(4095 - _throttle);
    report.roll = convert_12bit_to_8bit(_phi);
    report.pitch = convert_12bit_to_8bit(_theta);
    report.yaw = convert_12bit_to_8bit(_psi);

    // ボタン状態
    report.buttons = 0;
    if (joy_get_arm_button()) report.buttons |= 0x01;
    if (joy_get_flip_button()) report.buttons |= 0x02;
    if (joy_get_mode_button()) report.buttons |= 0x04;
    if (joy_get_option_button()) report.buttons |= 0x08;

    report.reserved = 0;

    // レポート送信
    usb_hid_send_report(&report);
}

// メインループ処理 (ESP-NOW mode)
static void main_loop(void)
{
    int16_t _throttle, _phi, _theta, _psi;
    InputData local_input;

    // タイミング計測
    etime = stime;
    stime = millis_now();
    dtime = stime - etime;

    // 入力データ取得
    if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        memcpy(&local_input, &shared_inputdata, sizeof(InputData));
        xSemaphoreGive(input_mutex);
    } else {
        memset(&local_input, 0, sizeof(InputData));
    }

    // 画面状態取得
    screen_state_t current_state = menu_get_state();

    // キャリブレーション画面: 累積開始/停止制御
    // Calibration screen: control accumulation start/stop
    static screen_state_t prev_state_espnow = SCREEN_STATE_FLIGHT;
    if (current_state == SCREEN_STATE_CALIBRATION && prev_state_espnow != SCREEN_STATE_CALIBRATION) {
        // キャリブレーション画面に入った
        calibration_start();
    } else if (current_state != SCREEN_STATE_CALIBRATION && prev_state_espnow == SCREEN_STATE_CALIBRATION) {
        // キャリブレーション画面から出た（キャンセル）
        calibration_stop();
    }
    prev_state_espnow = current_state;

    // キャリブレーション画面: Modeボタンで確定
    // Calibration screen: Mode button confirms
    if (current_state == SCREEN_STATE_CALIBRATION && local_input.mode_changed == 1) {
        calibration_confirm();
        menu_set_state(SCREEN_STATE_MENU);
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    // ボタンイベント処理
    // Button event handling
    if (local_input.btn_pressed) {
        if (current_state == SCREEN_STATE_ABOUT ||
            current_state == SCREEN_STATE_BATTERY_WARN ||
            current_state == SCREEN_STATE_CHANNEL ||
            current_state == SCREEN_STATE_MAC ||
            current_state == SCREEN_STATE_CALIBRATION ||
            current_state == SCREEN_STATE_STICK_TEST) {
            // サブ画面からメニューに戻る
            // Return from sub screen to Menu
            menu_set_state(SCREEN_STATE_MENU);
        } else {
            // その他は通常のトグル動作
            // Otherwise, normal toggle
            menu_toggle();
        }
    }

    // Long press: Reset timer (only when not in menu)
    // 長押し: タイマーリセット（メニュー外のみ）
    if (local_input.btn_long_pressed && !menu_is_active()) {
        Timer_state = 2;
    }

    // タイマー更新 (only when not in menu)
    if (!menu_is_active()) {
        if (Timer_state == 1) {
            Timer += dTime;
        } else if (Timer_state == 2) {
            Timer = 0.0f;
            Timer_state = 0;
        }
    }

    // Menu navigation using stick (when in menu state only)
    // メニューナビゲーション（メニュー状態時のみ）
    // 常に物理的な右スティックを使用（StickModeに依存しない）
    // Always use physical right stick (independent of StickMode)
    if (menu_get_state() == SCREEN_STATE_MENU) {
        static uint32_t last_nav_time = 0;
        const uint32_t NAV_DEBOUNCE_MS = 200;
        uint32_t now = millis_now();

        // 物理右スティックY値を使用
        // Use physical right stick Y value
        int16_t stick_y = (int16_t)joy_get_stick_right_y() - 2048;

        if (now - last_nav_time > NAV_DEBOUNCE_MS) {
            if (stick_y > 800) {
                menu_move_up();
                last_nav_time = now;
            } else if (stick_y < -800) {
                menu_move_down();
                last_nav_time = now;
            }

            // Use mode button as select
            if (local_input.mode_changed == 1) {
                menu_select();
                last_nav_time = now;
                // Clear the flag
                if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    shared_inputdata.mode_changed = 0;
                    xSemaphoreGive(input_mutex);
                }
            }
        }
    }

    // Battery warning screen: Mode button changes threshold
    // バッテリー警告画面: Modeボタンで閾値変更
    if (menu_get_state() == SCREEN_STATE_BATTERY_WARN && local_input.mode_changed == 1) {
        on_battery_warn_change();
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    // モード変更処理 (only when not in menu)
    // Mode change processing (メニュー外のみ)
    if (!menu_is_active() && local_input.mode_changed == 1) {
        Mode = (Mode == ANGLECONTROL) ? RATECONTROL : ANGLECONTROL;
        ESP_LOGI(TAG, "モード変更: %s", Mode == ANGLECONTROL ? "STABILIZE" : "ACRO");

        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    if (!menu_is_active() && local_input.alt_mode_changed == 1) {
        AltMode = (AltMode == ALT_CONTROL_MODE) ? NOT_ALT_CONTROL_MODE : ALT_CONTROL_MODE;
        ESP_LOGI(TAG, "高度モード変更: %s", AltMode == ALT_CONTROL_MODE ? "Auto ALT" : "Manual ALT");

        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared_inputdata.alt_mode_changed = 0;
            xSemaphoreGive(input_mutex);
        }
    }

    // スティック値取得（キャリブレーション適用）
    // Get stick values (with calibration applied)
    // Note: StickModeに応じてマッピングされた値を補正
    // Throttle/Elevator: 左Y or 右Y, Aileron/Rudder: 右X or 左X
    // キャリブレーションは物理スティック単位で保存されているので適切にマッピング
    if (StickMode == STICK_MODE_2) {
        // Mode 2: Throttle=左Y, Aileron=右X, Elevator=右Y, Rudder=左X
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.left_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.right_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.right_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.left_x);
    } else {
        // Mode 3: Throttle=右Y, Aileron=左X, Elevator=左Y, Rudder=右X
        _throttle = get_calibrated_value(local_input.throttle_raw, g_stick_cal.right_y);
        _phi = get_calibrated_value(local_input.phi_raw, g_stick_cal.left_x);
        _theta = get_calibrated_value(local_input.theta_raw, g_stick_cal.left_y);
        _psi = get_calibrated_value(local_input.psi_raw, g_stick_cal.right_x);
    }

    // 制御値計算
    Throttle = 4095 - _throttle;
    Phi = _phi;
    Theta = _theta;
    Psi = _psi;

    // 送信データ作成
    uint8_t senddata[CONTROL_PACKET_SIZE];
    const uint8_t* drone_mac = get_drone_peer_addr();

    senddata[0] = drone_mac[3];
    senddata[1] = drone_mac[4];
    senddata[2] = drone_mac[5];

    uint8_t* d_int = (uint8_t*)&Throttle;
    senddata[3] = d_int[0];
    senddata[4] = d_int[1];

    d_int = (uint8_t*)&Phi;
    senddata[5] = d_int[0];
    senddata[6] = d_int[1];

    d_int = (uint8_t*)&Theta;
    senddata[7] = d_int[0];
    senddata[8] = d_int[1];

    d_int = (uint8_t*)&Psi;
    senddata[9] = d_int[0];
    senddata[10] = d_int[1];

    senddata[11] = (0x01 & AltMode) << 3 |
                   (0x01 & Mode) << 2 |
                   (0x01 & joy_get_flip_button()) << 1 |
                   (0x01 & joy_get_arm_button());
    senddata[12] = proactive_flag;

    // チェックサム
    senddata[13] = 0;
    for (int i = 0; i < 13; i++) {
        senddata[13] += senddata[i];
    }

    // TDMAタスクに送信データを渡す
    tdma_update_senddata(senddata);

    // ビーコンロストチェック (スレーブのみ)
    static uint32_t last_beep_time = 0;
    if (is_beacon_lost()) {
        uint32_t current_millis = millis_now();
        if (current_millis - last_beep_time >= 500) {
            beep_beacon_loss();
            last_beep_time = current_millis;
        }
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "StampFly Controller 起動中...");

    // NVS初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 共有入力データ初期化
    memset(&shared_inputdata, 0, sizeof(shared_inputdata));

    // ログレベル設定
    esp_log_level_set("*", GLOBAL_LOG_LEVEL);

    // ブザー初期化
    buzzer_init();

    // M5Unified初期化
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_rtc = false;
    cfg.internal_spk = false;
    cfg.internal_mic = false;
    cfg.led_brightness = 0;
    M5.begin(cfg);
    ESP_LOGI(TAG, "M5Unified初期化完了");

    // LCD初期化
    init_display();

    // NVSから通信モード読み込み
    // Load communication mode from NVS
    g_comm_mode = load_comm_mode_from_nvs();
    ESP_LOGI(TAG, "通信モード: %s", g_comm_mode == COMM_MODE_USB_HID ? "USB HID" : "ESP-NOW");

    M5.Display.setCursor(4, 2);
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    if (g_comm_mode == COMM_MODE_USB_HID) {
        M5.Display.println("StampFly USB HID");
    } else {
        M5.Display.println("StampFly ESP-IDF");
    }

    // メニューシステム初期化
    menu_init();
    menu_register_usb_mode_callback(on_usb_mode_selected);
    menu_register_stick_mode_callback(on_stick_mode_selected);
    ESP_LOGI(TAG, "メニューシステム初期化完了");

    // Stick ModeをNVSから読み込み (両モード共通)
    // Load stick mode from NVS (common to both modes)
    StickMode = load_stick_mode_from_nvs();
    joy_set_stick_mode(StickMode);
    menu_set_stick_mode_label(StickMode);
    ESP_LOGI(TAG, "スティックモード: %d (NVSより)", StickMode);

    // バッテリー警告閾値をNVSから読み込み
    // Load battery warning threshold from NVS
    g_battery_warn_threshold = load_battery_warn_from_nvs();
    menu_set_battery_warn_threshold(g_battery_warn_threshold);
    menu_register_battery_warn_callback(on_battery_warn_change);
    ESP_LOGI(TAG, "バッテリー警告閾値: %d.%dV (NVSより)",
             g_battery_warn_threshold / 10, g_battery_warn_threshold % 10);

    // スティックキャリブレーションをNVSから読み込み
    // Load stick calibration from NVS
    load_stick_cal_from_nvs();

    // ジョイスティック初期化 (レガシーI2Cドライバで共有)
    ret = joy_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ジョイスティック初期化完了");
        M5.Display.setTextColor(SF_GREEN, SF_BLACK);
        M5.Display.println("JOY: OK");
    } else {
        ESP_LOGE(TAG, "ジョイスティック初期化失敗: %s", esp_err_to_name(ret));
        M5.Display.setTextColor(SF_RED, SF_BLACK);
        M5.Display.println("JOY: FAIL");
    }

    // モード別初期化
    // Mode-specific initialization
    if (g_comm_mode == COMM_MODE_USB_HID) {
        // USB HIDモード初期化
        // USB HID mode initialization
        ret = usb_hid_init();
        if (ret == ESP_OK) {
            M5.Display.setTextColor(SF_GREEN, SF_BLACK);
            M5.Display.println("USB HID: OK");
            ESP_LOGI(TAG, "USB HID初期化完了");
        } else {
            M5.Display.setTextColor(SF_RED, SF_BLACK);
            M5.Display.println("USB HID: FAIL");
            ESP_LOGE(TAG, "USB HID初期化失敗");
        }
    } else {
        // ESP-NOWモード初期化
        // ESP-NOW mode initialization

        // ピア情報読み込み
        peer_info_load();

        // ESP-NOW初期化
        ret = espnow_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW初期化失敗");
            M5.Display.setTextColor(SF_RED, SF_BLACK);
            M5.Display.println("ESP-NOW: FAIL");
            while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        }
        M5.Display.setTextColor(SF_GREEN, SF_BLACK);
        M5.Display.println("ESP-NOW: OK");

        // ビーコンピア初期化
        beacon_peer_init();

        // ペアリング処理 (ボタン押下時またはMAC未設定時)
        M5.update();
        bool force_pairing = M5.BtnA.isPressed();
        if (force_pairing) {
            M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
            M5.Display.println("Pairing mode...");
            M5.Display.println("Hold StampFly Btn");
            M5.Display.println("until beep!");
        }
        peering_process(force_pairing);

        if (force_pairing) {
            peer_info_save();
        }

        // ドローンピア初期化
        drone_peer_init();

        // Stick Modeは既にNVSから読み込み済み
        // Stick mode already loaded from NVS
        AltMode = NOT_ALT_CONTROL_MODE;
    }

    // 入力Mutex作成
    input_mutex = xSemaphoreCreateMutex();
    if (input_mutex == NULL) {
        ESP_LOGE(TAG, "入力Mutex作成失敗");
    }

    // 入力タスク作成 (高優先度)
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        input_task,
        "InputTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &input_task_handle,
        1
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "入力タスク作成失敗");
    } else {
        ESP_LOGI(TAG, "入力タスク作成完了");
    }

    // LCD更新タスク作成 (低優先度)
    // Create LCD update task (low priority)
    task_ret = xTaskCreatePinnedToCore(
        display_task,
        "DisplayTask",
        4096,
        NULL,
        2,  // 低優先度
        &display_task_handle,
        1
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "LCD更新タスク作成失敗");
    } else {
        ESP_LOGI(TAG, "LCD更新タスク作成完了");
    }

    // TDMA初期化 (ESP-NOWモードのみ)
    // TDMA initialization (ESP-NOW mode only)
    if (g_comm_mode == COMM_MODE_ESPNOW) {
        ret = tdma_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TDMA初期化失敗");
        }

        // 安定化待機
        ESP_LOGI(TAG, "システム安定化待機 (200ms)...");
        vTaskDelay(pdMS_TO_TICKS(200));

        // TDMA開始
        ret = tdma_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TDMA開始失敗");
        }
    }

    // 起動音
    beep_start_tone();
    ESP_LOGI(TAG, "起動完了");

    vTaskDelay(pdMS_TO_TICKS(500));
    M5.Display.fillScreen(SF_BLACK);

    // LCD更新タスク有効化
    // Enable LCD update task
    display_task_enabled = true;

    ESP_LOGI(TAG, "メインループ開始");

    // メインループ (モードに応じて分岐)
    // Main loop (branch by mode)
    while (true) {
        if (g_comm_mode == COMM_MODE_USB_HID) {
            usb_hid_main_loop();
        } else {
            main_loop();
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }
}
