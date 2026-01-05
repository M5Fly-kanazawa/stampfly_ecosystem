/*
 * M5Stack Atom JoyStick ドライバ - ESP-IDF I2C API版
 *
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 */
#ifndef ATOMS3JOY_H
#define ATOMS3JOY_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C設定
#define JOY_I2C_ADDRESS     0x59
#define JOY_I2C_PORT        I2C_NUM_1   // Wire1相当
#define JOY_I2C_SDA_PIN     38
#define JOY_I2C_SCL_PIN     39
#define JOY_I2C_FREQ_HZ     400000      // 400kHz

// レジスタアドレス
#define LEFT_STICK_X_ADDRESS        0x00
#define LEFT_STICK_Y_ADDRESS        0x02
#define RIGHT_STICK_X_ADDRESS       0x20
#define RIGHT_STICK_Y_ADDRESS       0x22
#define LEFT_STICK_BUTTON_ADDRESS   0x70
#define RIGHT_STICK_BUTTON_ADDRESS  0x71
#define LEFT_BUTTON_ADDRESS         0x72
#define RIGHT_BUTTON_ADDRESS        0x73
#define BATTERY_VOLTAGE1_ADDRESS    0x60
#define BATTERY_VOLTAGE2_ADDRESS    0x62

// スティックインデックス
#define STICK_LEFTX     0
#define STICK_LEFTY     1
#define STICK_RIGHTX    2
#define STICK_RIGHTY    3

// ボタンインデックス (NEW_ATOM_JOY)
#define BTN_LEFT_STICK  2
#define BTN_RIGHT_STICK 3
#define BTN_LEFT        0
#define BTN_RIGHT       1

// ジョイスティックデータ構造体
typedef struct {
    uint16_t stick[4];          // スティック値 (0-4095)
    uint8_t button[4];          // ボタン生値
    uint8_t button_state[4];    // デバウンス後のボタン状態
    float battery_voltage[2];   // バッテリー電圧 (V)
} joy_data_t;

/**
 * @brief ジョイスティックI2C初期化
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t joy_init(void);

/**
 * @brief ジョイスティックデータ更新
 *        全スティック・ボタン・バッテリー電圧を読み取る
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t joy_update(void);

/**
 * @brief 現在のジョイスティックデータを取得
 * @return ジョイスティックデータへのポインタ
 */
const joy_data_t* joy_get_data(void);

// スティック値取得 (0-4095)
uint16_t joy_get_stick_left_x(void);
uint16_t joy_get_stick_left_y(void);
uint16_t joy_get_stick_right_x(void);
uint16_t joy_get_stick_right_y(void);

// ボタン状態取得 (デバウンス済み, 0または1)
uint8_t joy_get_button_left_stick(void);
uint8_t joy_get_button_right_stick(void);
uint8_t joy_get_button_left(void);
uint8_t joy_get_button_right(void);

// ボタン生値取得 (デバウンスなし, 起動時判定用)
uint8_t joy_get_button_left_raw(void);
uint8_t joy_get_button_right_raw(void);

// バッテリー電圧取得 (V)
float joy_get_battery_voltage1(void);
float joy_get_battery_voltage2(void);

// スティックモード設定
#define STICK_MODE_2 2  // THROTTLE=左Y, AILERON=右X, ELEVATOR=右Y, RUDDER=左X
#define STICK_MODE_3 3  // THROTTLE=右Y, AILERON=左X, ELEVATOR=左Y, RUDDER=右X

/**
 * @brief スティックモード設定
 * @param mode STICK_MODE_2 または STICK_MODE_3
 */
void joy_set_stick_mode(uint8_t mode);

/**
 * @brief 現在のスティックモード取得
 * @return STICK_MODE_2 または STICK_MODE_3
 */
uint8_t joy_get_stick_mode(void);

// ドローン操作用関数 (モードに応じて自動切替)
uint16_t joy_get_throttle(void);
uint16_t joy_get_aileron(void);
uint16_t joy_get_elevator(void);
uint16_t joy_get_rudder(void);
uint8_t joy_get_arm_button(void);
uint8_t joy_get_flip_button(void);

// モード/オプションボタンは固定
#define joy_get_mode_button()   joy_get_button_right()
#define joy_get_option_button() joy_get_button_left()

#ifdef __cplusplus
}
#endif

#endif // ATOMS3JOY_H
