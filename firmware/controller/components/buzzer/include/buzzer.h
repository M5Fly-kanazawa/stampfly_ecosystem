/*
 * ブザー制御 - ESP-IDF LEDC API版
 *
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 * SPDX-License-Identifier: MIT
 */
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 音階定義 (Hz)
#define NOTE_D1 294
#define NOTE_D2 330
#define NOTE_D3 350
#define NOTE_D4 393
#define NOTE_D5 441
#define NOTE_D6 495
#define NOTE_D7 556

/**
 * @brief ブザーPWM初期化
 */
void buzzer_init(void);

/**
 * @brief 指定周波数・時間でブザーを鳴らす
 * @param frequency 周波数 (Hz)、0で停止
 * @param duration_ms 持続時間 (ミリ秒)
 */
void buzzer_sound(uint32_t frequency, uint32_t duration_ms);

/**
 * @brief ブザーを停止
 */
void buzzer_stop(void);

/**
 * @brief 標準ビープ音 (4000Hz, 100ms)
 */
void beep(void);

/**
 * @brief 起動音
 */
void beep_start_tone(void);

// エラー識別用ビープパターン

/**
 * @brief ビーコンロス警告 (4000Hz 単発、緊急)
 */
void beep_beacon_loss(void);

/**
 * @brief スロットタイミングエラー (3000Hz 2回)
 */
void beep_slot_error(void);

/**
 * @brief ドローンオフライン (1000Hz 長音)
 */
void beep_drone_offline(void);

/**
 * @brief Mutexタイムアウト (2000Hz 3回)
 */
void beep_mutex_timeout(void);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
