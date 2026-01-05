/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
//#include "main.h"
#include "Arduino.h"
#include <driver/ledc.h>
#include "buzzer.h"

const int buzzerPin = 5;
const int channel   = 0;

void setup_pwm_buzzer(void) {
    ledcSetup(channel, 4000, 8);        // 配置PWM通道：通道0，频率3000Hz，分辨率8位
    ledcAttachPin(buzzerPin, channel);  // 将PWM通道绑定到GPIO
}

void buzzer_sound(uint32_t frequency, uint32_t duration_ms) {
    ledcWriteTone(channel, frequency);
    ledcWrite(channel, 64);

    vTaskDelay(duration_ms / portTICK_PERIOD_MS);

    ledcWriteTone(channel, 0);
    digitalWrite(channel, 0);
}

void beep(void) {
    buzzer_sound(4000, 100);
}

void start_tone(void) {
    buzzer_sound(2000, 100);
    buzzer_sound(1000, 100);

    #if 0
    buzzer_sound(NOTE_D1, 200);
    buzzer_sound(NOTE_D5, 200);
    buzzer_sound(NOTE_D3, 200);
    buzzer_sound(NOTE_D4, 200);
    #endif
}

// Different beep patterns for error identification

// Beacon loss: Single high-pitched beep (urgent warning)
void beep_beacon_loss(void) {
    buzzer_sound(4000, 100);
}

// Slot timing error: Double medium-high beep
void beep_slot_error(void) {
    buzzer_sound(3000, 80);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    buzzer_sound(3000, 80);
}

// Drone offline: Single long low-pitched beep
void beep_drone_offline(void) {
    buzzer_sound(1000, 300);
}

// Mutex timeout: Triple medium beep (rapid pattern)
void beep_mutex_timeout(void) {
    buzzer_sound(2000, 60);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    buzzer_sound(2000, 60);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    buzzer_sound(2000, 60);
}