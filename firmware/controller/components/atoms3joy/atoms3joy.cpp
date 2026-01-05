/*
 * M5Stack Atom JoyStick ドライバ - M5Unified I2C API版
 *
 * MIT License
 * Copyright (c) 2024 Kouhei Ito
 */
#include "atoms3joy.h"
#include <M5Unified.h>
#include <esp_log.h>
#include <string.h>

static const char* TAG = "JOY";

// ジョイスティックデータ
static joy_data_t joy_data;
static int16_t button_counter[4] = {0};
static uint8_t button_old_state[4] = {0};
static bool joy_initialized = false;

// I2Cから2バイト読み取り (リトルエンディアン)
static bool read_2byte_data(uint8_t reg_addr, uint16_t *data)
{
    uint8_t buf[2];

    if (!m5::Ex_I2C.readRegister(JOY_I2C_ADDRESS, reg_addr, buf, 2, JOY_I2C_FREQ_HZ)) {
        return false;
    }

    *data = (uint16_t)(buf[1] << 8) | buf[0];
    return true;
}

// I2Cから1バイト読み取り
static bool read_byte_data(uint8_t reg_addr, uint8_t *data)
{
    return m5::Ex_I2C.readRegister(JOY_I2C_ADDRESS, reg_addr, data, 1, JOY_I2C_FREQ_HZ);
}

extern "C" esp_err_t joy_init(void)
{
    if (joy_initialized) {
        return ESP_OK;
    }

    // Ex_I2Cを明示的に初期化 (GPIO 38=SDA, 39=SCL)
    // 現在のピン設定を確認し、異なる場合は再設定
    ESP_LOGI(TAG, "Ex_I2C 現在: enabled=%d, SDA=%d, SCL=%d",
             m5::Ex_I2C.isEnabled(), m5::Ex_I2C.getSDA(), m5::Ex_I2C.getSCL());

    if (m5::Ex_I2C.getSDA() != JOY_I2C_SDA_PIN || m5::Ex_I2C.getSCL() != JOY_I2C_SCL_PIN) {
        ESP_LOGI(TAG, "Ex_I2C を再設定中 (SDA=%d, SCL=%d)", JOY_I2C_SDA_PIN, JOY_I2C_SCL_PIN);
        if (!m5::Ex_I2C.begin(I2C_NUM_1, JOY_I2C_SDA_PIN, JOY_I2C_SCL_PIN)) {
            ESP_LOGE(TAG, "Ex_I2C 初期化失敗");
            return ESP_ERR_INVALID_STATE;
        }
    } else if (!m5::Ex_I2C.isEnabled()) {
        ESP_LOGI(TAG, "Ex_I2C を初期化中");
        if (!m5::Ex_I2C.begin()) {
            ESP_LOGE(TAG, "Ex_I2C 初期化失敗");
            return ESP_ERR_INVALID_STATE;
        }
    }

    // データ初期化
    memset(&joy_data, 0, sizeof(joy_data));
    memset(button_counter, 0, sizeof(button_counter));
    memset(button_old_state, 0, sizeof(button_old_state));

    // 接続確認 (デバイスからの読み取りテスト)
    uint16_t test_data;
    if (!read_2byte_data(LEFT_STICK_X_ADDRESS, &test_data)) {
        ESP_LOGE(TAG, "ジョイスティック応答なし (addr: 0x%02X)", JOY_I2C_ADDRESS);
        return ESP_ERR_NOT_FOUND;
    }

    joy_initialized = true;
    ESP_LOGI(TAG, "ジョイスティック初期化完了 (I2C addr: 0x%02X, Ex_I2C使用)", JOY_I2C_ADDRESS);

    return ESP_OK;
}

extern "C" esp_err_t joy_update(void)
{
    if (!joy_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // スティック値読み取り
    if (!read_2byte_data(LEFT_STICK_X_ADDRESS, &joy_data.stick[STICK_LEFTX])) return ESP_FAIL;
    if (!read_2byte_data(LEFT_STICK_Y_ADDRESS, &joy_data.stick[STICK_LEFTY])) return ESP_FAIL;
    if (!read_2byte_data(RIGHT_STICK_X_ADDRESS, &joy_data.stick[STICK_RIGHTX])) return ESP_FAIL;
    if (!read_2byte_data(RIGHT_STICK_Y_ADDRESS, &joy_data.stick[STICK_RIGHTY])) return ESP_FAIL;

    // ボタン読み取り + デバウンス処理
    for (int i = 0; i < 4; i++) {
        uint8_t raw_button;
        if (!read_byte_data(LEFT_STICK_BUTTON_ADDRESS + i, &raw_button)) return ESP_FAIL;

        // ボタンは負論理 (押すと0)
        joy_data.button[i] = (~raw_button) & 0x01;

        // デバウンス処理
        button_old_state[i] = joy_data.button_state[i];

        if (joy_data.button[i] == 1) {
            if (button_counter[i] < 0) button_counter[i] = 0;
            button_counter[i]++;
            if (button_counter[i] > 1) {
                button_counter[i] = 1;
                joy_data.button_state[i] = 1;  // 押し確定
            }
        } else {
            if (button_counter[i] > 0) button_counter[i] = 0;
            button_counter[i]--;
            if (button_counter[i] < -1) {
                button_counter[i] = -1;
                joy_data.button_state[i] = 0;  // 放し確定
            }
        }
    }

    // バッテリー電圧読み取り
    uint16_t voltage_raw;
    if (read_2byte_data(BATTERY_VOLTAGE1_ADDRESS, &voltage_raw)) {
        joy_data.battery_voltage[0] = (float)voltage_raw / 1000.0f;
    }

    if (read_2byte_data(BATTERY_VOLTAGE2_ADDRESS, &voltage_raw)) {
        joy_data.battery_voltage[1] = (float)voltage_raw / 1000.0f;
    }

    return ESP_OK;
}

extern "C" const joy_data_t* joy_get_data(void)
{
    return &joy_data;
}

// スティック値取得
extern "C" uint16_t joy_get_stick_left_x(void)  { return joy_data.stick[STICK_LEFTX]; }
extern "C" uint16_t joy_get_stick_left_y(void)  { return joy_data.stick[STICK_LEFTY]; }
extern "C" uint16_t joy_get_stick_right_x(void) { return joy_data.stick[STICK_RIGHTX]; }
extern "C" uint16_t joy_get_stick_right_y(void) { return joy_data.stick[STICK_RIGHTY]; }

// ボタン状態取得 (デバウンス済み)
extern "C" uint8_t joy_get_button_left_stick(void)  { return joy_data.button_state[BTN_LEFT_STICK]; }
extern "C" uint8_t joy_get_button_right_stick(void) { return joy_data.button_state[BTN_RIGHT_STICK]; }
extern "C" uint8_t joy_get_button_left(void)        { return joy_data.button_state[BTN_LEFT]; }
extern "C" uint8_t joy_get_button_right(void)       { return joy_data.button_state[BTN_RIGHT]; }

// ボタン生値取得 (デバウンスなし, 起動時判定用)
extern "C" uint8_t joy_get_button_left_raw(void)  { return joy_data.button[BTN_LEFT]; }
extern "C" uint8_t joy_get_button_right_raw(void) { return joy_data.button[BTN_RIGHT]; }

// バッテリー電圧取得
extern "C" float joy_get_battery_voltage1(void) { return joy_data.battery_voltage[0]; }
extern "C" float joy_get_battery_voltage2(void) { return joy_data.battery_voltage[1]; }

// スティックモード (デフォルト: Mode 2)
static uint8_t stick_mode = STICK_MODE_2;

extern "C" void joy_set_stick_mode(uint8_t mode)
{
    if (mode == STICK_MODE_2 || mode == STICK_MODE_3) {
        stick_mode = mode;
        ESP_LOGI(TAG, "スティックモード設定: %d", mode);
    }
}

extern "C" uint8_t joy_get_stick_mode(void)
{
    return stick_mode;
}

// ドローン操作用関数 (モードに応じて自動切替)
extern "C" uint16_t joy_get_throttle(void)
{
    // Mode 2: 左Y, Mode 3: 右Y
    return (stick_mode == STICK_MODE_2) ? joy_data.stick[STICK_LEFTY] : joy_data.stick[STICK_RIGHTY];
}

extern "C" uint16_t joy_get_aileron(void)
{
    // Mode 2: 右X, Mode 3: 左X
    return (stick_mode == STICK_MODE_2) ? joy_data.stick[STICK_RIGHTX] : joy_data.stick[STICK_LEFTX];
}

extern "C" uint16_t joy_get_elevator(void)
{
    // Mode 2: 右Y, Mode 3: 左Y
    return (stick_mode == STICK_MODE_2) ? joy_data.stick[STICK_RIGHTY] : joy_data.stick[STICK_LEFTY];
}

extern "C" uint16_t joy_get_rudder(void)
{
    // Mode 2: 左X, Mode 3: 右X
    return (stick_mode == STICK_MODE_2) ? joy_data.stick[STICK_LEFTX] : joy_data.stick[STICK_RIGHTX];
}

extern "C" uint8_t joy_get_arm_button(void)
{
    // Mode 2: 左スティック押し込み, Mode 3: 右スティック押し込み
    return (stick_mode == STICK_MODE_2) ? joy_data.button_state[BTN_LEFT_STICK] : joy_data.button_state[BTN_RIGHT_STICK];
}

extern "C" uint8_t joy_get_flip_button(void)
{
    // Mode 2: 右スティック押し込み, Mode 3: 左スティック押し込み
    return (stick_mode == STICK_MODE_2) ? joy_data.button_state[BTN_RIGHT_STICK] : joy_data.button_state[BTN_LEFT_STICK];
}
