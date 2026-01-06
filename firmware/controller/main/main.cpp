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
#include <string.h>
#include <buzzer.h>
#include <atoms3joy.h>
#include <espnow_tdma.h>
#include <menu_system.h>

static const char* TAG = "MAIN";

// Project-specific color macros for AtomS3/GC9107 panel
// AtomS3/GC9107パネル用のプロジェクト独自カラーマクロ
// This panel has R/G swap issue (GitHub m5stack/M5AtomS3#16)
// rgb_order=true setting swaps R and G channels
#define SF_BLACK   TFT_BLACK
#define SF_WHITE   TFT_WHITE
#define SF_RED     TFT_GREEN   // Swapped: GREEN displays as RED
#define SF_GREEN   TFT_RED     // Swapped: RED displays as GREEN
#define SF_BLUE    TFT_BLUE
#define SF_YELLOW  TFT_YELLOW  // R=G, no swap needed
#define SF_CYAN    TFT_MAGENTA // Swapped: MAGENTA displays as CYAN
#define SF_MAGENTA TFT_CYAN    // Swapped: CYAN displays as MAGENTA
#define SF_ORANGE  0xFD20      // Custom: swap R/G in original 0x07E0+R

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
static int16_t Phi_bias = 0, Theta_bias = 0, Psi_bias = 0, Throttle_bias = 0;
static uint8_t Mode = ANGLECONTROL;
static uint8_t AltMode = NOT_ALT_CONTROL_MODE;
static uint8_t StickMode = 2;
static float Timer = 0.0f;
static uint8_t Timer_state = 0;
static uint8_t average_counter = 0;
static volatile uint8_t proactive_flag = 0;

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

// LCD初期化
static void init_display(void)
{
    // Change color order BEFORE setRotation (which writes MADCTL)
    // setRotationの前にカラーオーダーを変更（MADCTLに書き込まれる）
    auto panel = M5.Display.getPanel();
    if (panel) {
        auto cfg = panel->config();
        ESP_LOGI(TAG, "Current rgb_order: %d", cfg.rgb_order);
        cfg.rgb_order = true;  // Use RGB mode with SF_ color macros
        panel->config(cfg);
        ESP_LOGI(TAG, "Changed rgb_order to: %d", cfg.rgb_order);
    }

    // setRotation() writes MADCTL register including rgb_order setting
    M5.Display.setRotation(0);
    M5.Display.setTextSize(1);
    M5.Display.setTextFont(2);
    M5.Display.fillScreen(SF_BLACK);

    ESP_LOGI(TAG, "LCD初期化完了");
}

// メニュー画面描画
static void render_menu_screen(void)
{
    const int line_height = 17;

    // タイトルバー
    M5.Display.setCursor(4, 2);
    M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
    M5.Display.printf("=== MENU ===    ");

    // メニュー項目
    uint8_t item_count = menu_get_item_count();
    uint8_t selected = menu_get_selected_index();

    for (uint8_t i = 0; i < item_count && i < 6; i++) {
        M5.Display.setCursor(4, 2 + (i + 1) * line_height);

        if (i == selected) {
            M5.Display.setTextColor(SF_BLACK, SF_WHITE);
            M5.Display.printf("> %-12s", menu_get_item_label(i));
        } else {
            M5.Display.setTextColor(SF_WHITE, SF_BLACK);
            M5.Display.printf("  %-12s", menu_get_item_label(i));
        }
    }

    // 残りの行をクリア
    M5.Display.setTextColor(SF_WHITE, SF_BLACK);
    for (uint8_t i = item_count; i < 6; i++) {
        M5.Display.setCursor(4, 2 + (i + 1) * line_height);
        M5.Display.printf("                ");
    }
}

// 画面更新 (全行一括更新)
static void update_display(void)
{
    const int line_height = 17;

    // 画面状態追跡（状態変化時に画面クリア）
    // Track screen state for clearing on transition
    static bool prev_menu_active = false;
    bool current_menu_active = menu_is_active();

    if (prev_menu_active != current_menu_active) {
        M5.Display.fillScreen(SF_BLACK);
        prev_menu_active = current_menu_active;
    }

    // メニューがアクティブな場合はメニュー画面を描画
    if (current_menu_active) {
        render_menu_screen();
        return;
    }

    // フライト画面描画
    // Flight screen rendering with StickMode-based color
    // StickMode 2: 緑 (GREEN), StickMode 3: 黄 (YELLOW)
    const uint8_t* drone_mac = get_drone_peer_addr();
    uint32_t text_color = (StickMode == 2) ? SF_GREEN : SF_YELLOW;
    M5.Display.setTextColor(text_color, SF_BLACK);

    // 行0: MACアドレス
    M5.Display.setCursor(4, 2 + 0 * line_height);
    M5.Display.printf("MAC ADR %02X:%02X    ", drone_mac[4], drone_mac[5]);

    // 行1: バッテリー電圧
    M5.Display.setCursor(4, 2 + 1 * line_height);
    M5.Display.printf("BAT 1:%4.1f 2:%4.1f",
        joy_get_battery_voltage1(), joy_get_battery_voltage2());

    // 行2: スティックモード
    M5.Display.setCursor(4, 2 + 2 * line_height);
    M5.Display.printf("MODE: %d        ", StickMode);

    // 行3: チャンネル/ID
    M5.Display.setCursor(4, 2 + 3 * line_height);
    M5.Display.printf("CH: %02d ID: %d  ", ESPNOW_CHANNEL, TDMA_DEVICE_ID);

    // 行4: 高度モード
    M5.Display.setCursor(4, 2 + 4 * line_height);
    if (AltMode == ALT_CONTROL_MODE)
        M5.Display.printf("-Auto ALT-  ");
    else
        M5.Display.printf("-Mnual ALT- ");

    // 行5: 制御モード
    M5.Display.setCursor(4, 2 + 5 * line_height);
    if (Mode == ANGLECONTROL)
        M5.Display.printf("-STABILIZE-");
    else
        M5.Display.printf("-ACRO-     ");

    // 行6: 周波数/同期状態
    M5.Display.setCursor(4, 2 + 6 * line_height);
    #if TDMA_DEVICE_ID == 0
        M5.Display.printf("Freq:%4d M    ", (int)actual_send_freq_hz);
    #else
        if (first_beacon_received) {
            if (is_beacon_lost()) {
                M5.Display.printf("F:%4d LOST!  ", (int)actual_send_freq_hz);
            } else {
                M5.Display.printf("F:%4d SYNC   ", (int)actual_send_freq_hz);
            }
        } else {
            M5.Display.printf("F:%4d WAIT   ", (int)actual_send_freq_hz);
        }
    #endif
}

// LCD更新タスク (低優先度, 10Hz)
static volatile bool display_task_enabled = false;

static void display_task(void* parameter)
{
    ESP_LOGI(TAG, "LCD更新タスク開始");

    // メインループ開始まで待機
    while (!display_task_enabled) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        update_display();
    }
}

// メインループ処理
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

    // ボタンイベント処理
    // Button event handling - short press toggles menu
    if (local_input.btn_pressed) {
        menu_toggle();
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

    // Menu navigation using stick (when menu is active)
    // メニューナビゲーション（メニューアクティブ時）
    if (menu_is_active()) {
        static uint32_t last_nav_time = 0;
        const uint32_t NAV_DEBOUNCE_MS = 200;
        uint32_t now = millis_now();

        // Use right stick (elevator/theta) for up/down navigation
        // 右スティック（エレベータ/theta）で上下ナビゲーション
        int16_t stick_y = local_input.theta_raw - 2048;

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

    // スティック値取得
    _throttle = local_input.throttle_raw;
    _phi = local_input.phi_raw;
    _theta = local_input.theta_raw;
    _psi = local_input.psi_raw;

    // バイアス計算 (起動時50サンプル平均)
    if (average_counter < 50) {
        average_counter++;
        Throttle_bias += (_throttle - 2048);
        Phi_bias += (_phi - 2048);
        Theta_bias += (_theta - 2048);
        Psi_bias += (_psi - 2048);
    } else if (average_counter == 50) {
        average_counter = 51;
        Throttle_bias /= 50;
        Phi_bias /= 50;
        Theta_bias /= 50;
        Psi_bias /= 50;
    } else {
        if (_throttle < Throttle_bias) _throttle = 0; else _throttle -= Throttle_bias;
        if (_phi < Phi_bias) _phi = 0; else _phi -= Phi_bias;
        if (_theta < Theta_bias) _theta = 0; else _theta -= Theta_bias;
        if (_psi < Psi_bias) _psi = 0; else _psi -= Psi_bias;
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
    M5.Display.setCursor(4, 2);
    M5.Display.setTextColor(SF_CYAN, SF_BLACK);
    M5.Display.println("StampFly ESP-IDF");

    // メニューシステム初期化
    menu_init();
    ESP_LOGI(TAG, "メニューシステム初期化完了");

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

    // スティックモード選択 (左ボタン押しながら起動でMode 3)
    // 起動時はデバウンスなしの生値を使用
    joy_update();
    if (joy_get_button_left_raw()) {
        joy_set_stick_mode(STICK_MODE_3);
        M5.Display.setTextColor(SF_YELLOW, SF_BLACK);
        M5.Display.println("Mode 3 selected");
        // ボタンが離されるまで待機 (生値で判定)
        while (joy_get_button_left_raw()) {
            joy_update();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        joy_set_stick_mode(STICK_MODE_2);
    }
    StickMode = joy_get_stick_mode();
    ESP_LOGI(TAG, "スティックモード: %d", StickMode);
    AltMode = NOT_ALT_CONTROL_MODE;

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

    // TDMA初期化
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

    // 起動音
    beep_start_tone();
    ESP_LOGI(TAG, "起動完了");

    vTaskDelay(pdMS_TO_TICKS(500));
    init_display();  // 画面クリア

    // LCD更新タスク有効化
    display_task_enabled = true;

    ESP_LOGI(TAG, "メインループ開始");

    // メインループ
    while (true) {
        main_loop();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }
}
