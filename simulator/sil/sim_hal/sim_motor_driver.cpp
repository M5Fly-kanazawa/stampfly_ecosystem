/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sim_motor_driver.cpp
 * @brief SIL host stub for the LEDC MotorDriver (no ESP-IDF driver/ledc.h).
 *        LEDC MotorDriver の SIL ホストスタブ（driver/ledc.h 不要）。
 *
 * The firmware MotorDriver (sf_hal_motor/motor_driver.cpp) drives LEDC PWM and
 * cannot compile on the host (it includes driver/ledc.h). The SIL replaces only
 * the .cpp: the real motor_driver.hpp is used UNCHANGED, so the Actuator links.
 * init() deliberately leaves the driver "not initialized" so Actuator::update()
 * still publishes the actuator_motor topic (which the SIL Plant reads) but skips
 * the nonexistent-on-host hardware write. The firmware is UNMODIFIED.
 *
 * ファームの MotorDriver は LEDC PWM を駆動し host でビルドできない（driver/ledc.h
 * を include）。SIL は .cpp だけ差し替える: 本物の motor_driver.hpp をそのまま使い
 * Actuator がリンクできるようにする。init() は意図的に「未初期化」のままにして、
 * Actuator::update() は actuator_motor トピック（SIL Plant が読む）を発行しつつ、
 * host に存在しない HW 書き込みを飛ばす。ファームは無改変。
 *
 * The methods Actuator references (init, setMotorDuties, arm, disarm) are defined
 * here; isInitialized() is inline in the header (returns the always-false
 * initialized_). arm/disarm track the armed_ flag (no hardware) so the arm gate
 * wired in Actuator works on the host.
 * Actuator が参照するメソッド（init, setMotorDuties, arm, disarm）を定義する。
 * isInitialized() はヘッダのインライン（常に false の initialized_ を返す）。
 * arm/disarm は armed_ フラグを追従（HW なし）し、Actuator の arm ゲートが host でも働く。
 */

#include "motor_driver.hpp"

namespace stampfly {

// init — accept the config but stay "not initialized". On the host there is no
// LEDC, so Actuator::update() must skip the hardware write while still
// publishing duties to the actuator_motor topic for the Plant.
// init — config を受け取るが「未初期化」のままにする。host に LEDC は無いので、
// Actuator::update() は HW 書き込みを飛ばしつつ duty を actuator_motor トピックへ
// 発行し続ける（Plant が読む）。
esp_err_t MotorDriver::init(const Config& config)
{
    config_ = config;
    // initialized_ stays false on purpose (no host PWM peripheral).
    // initialized_ は意図的に false のまま（host に PWM ペリフェラルは無い）。
    return ESP_OK;
}

// setMotorDuties — host no-op. The Plant reads duty from the actuator_motor
// topic, not from the HAL. Defined only because Actuator::update() references it
// (guarded by isInitialized(), so it is never actually called on the host).
// setMotorDuties — host では何もしない。Plant はトピックから duty を読む。
// Actuator::update() が参照するため定義のみ置く（isInitialized() で守られ host では
// 呼ばれない）。
void MotorDriver::setMotorDuties(const float duties[4])
{
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_output_[i] = duties[i];
    }
}

// arm — host stub: track the armed flag (no LEDC). Unlike the real driver it does
// not require initialized_ (the host leaves it false on purpose), so the Actuator
// arm gate still engages on the host.
// arm — host スタブ: armed フラグを追従（LEDC なし）。実ドライバと違い initialized_ を
// 要求しない（host は意図的に false）ので、Actuator の arm ゲートが host でも働く。
esp_err_t MotorDriver::arm()
{
    armed_ = true;
    return ESP_OK;
}

// disarm — host stub: clear the armed flag and zero the duties (no LEDC).
// disarm — host スタブ: armed フラグを下げ duty をゼロにする（LEDC なし）。
esp_err_t MotorDriver::disarm()
{
    armed_ = false;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_output_[i] = 0.0f;
    }
    return ESP_OK;
}

}  // namespace stampfly
