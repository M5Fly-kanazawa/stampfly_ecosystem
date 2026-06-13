/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file motor_hw.hpp
 * @brief Canonical motor LEDC-PWM hardware numerics (single source of truth).
 *        モータ LEDC-PWM ハードウェア数値の単一情報源（SSOT）。
 *
 * The motor PWM frequency and duty resolution are consumed by TWO independent
 * places that MUST agree, or thrust is silently mis-scaled:
 *   - sf_board (board.cpp) configures the shared LEDC timer at this resolution.
 *   - sf_actuator (actuator.cpp) hands the same resolution to the motor HAL,
 *     which computes duty = (1 << bits) - 1.
 * If the timer ran at, say, 10 bits (max 1023) while duty were computed for 8
 * bits (max 255), every motor would output ~1/4 of the intended duty → no
 * take-off / loss of control. Deriving both from this single header makes that
 * divergence structurally impossible (code-review_2026-06-13.md, finding M-5).
 *
 * モータ PWM の周波数と duty 分解能は、一致していないと推力が静かに誤スケール
 * される 2 箇所で独立に消費される:
 *   - sf_board (board.cpp) が共有 LEDC タイマをこの分解能で構成する。
 *   - sf_actuator (actuator.cpp) が同じ分解能をモータ HAL に渡し、HAL が
 *     duty = (1 << bits) - 1 を計算する。
 * 例えばタイマが 10bit(max 1023) で走るのに duty を 8bit(max 255) で計算すると、
 * 全モータが意図の約 1/4 duty しか出さず離陸不能/制御喪失になる。両者をこの
 * 単一ヘッダから導出することで、その乖離を構造的に不可能にする（M-5）。
 *
 * Plain ints (no driver/ledc.h dependency) so sf_core stays buildable in the
 * partial SIL test programs that do not link the ESP-IDF LEDC driver.
 * 生 int（driver/ledc.h 非依存）にして、ESP-IDF LEDC ドライバをリンクしない
 * 部分 SIL 試験でも sf_core がビルドできるようにする。
 */

#pragma once

namespace sf::hw {

/// Motor PWM carrier frequency [Hz] — M5StampFly standard.
/// モータ PWM キャリア周波数 [Hz] — M5StampFly 標準。
inline constexpr int kMotorPwmFreqHz = 150000;

/// Motor PWM duty resolution [bits]. LEDC timer-bit enum values equal the bit
/// count (LEDC_TIMER_8_BIT == 8), so consumers may static_cast this to
/// ledc_timer_bit_t and static_assert the mapping.
/// モータ PWM duty 分解能 [bit]。LEDC の timer-bit enum 値はビット数に等しい
/// (LEDC_TIMER_8_BIT == 8) ため、消費側は ledc_timer_bit_t へ static_cast し
/// マッピングを static_assert できる。
inline constexpr int kMotorPwmResolutionBit = 8;

}  // namespace sf::hw
