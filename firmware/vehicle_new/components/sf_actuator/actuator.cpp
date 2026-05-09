/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file actuator.cpp
 * @brief Mixer and motor output implementation
 *        ミキサー＋モーター出力の実装
 *
 * Subscribes to `control_output` topic, applies an X-quad mixer, clamps
 * the resulting duties, publishes them to `actuator_motor` for telemetry,
 * and finally writes them to the LEDC PWM motor driver.
 *
 * `control_output` トピックを購読し、X-quad ミキサーを適用、duty をクランプ、
 * テレメトリ用に `actuator_motor` トピックへ発行し、最後に LEDC PWM モーター
 * ドライバへ書き込む。
 *
 * @design architecture.md §5 — Actuator subsystem                       [OK]
 * @design detailed_design.md §5 — X-quad mixer                          [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#include "actuator.hpp"
#include "topics.hpp"
#include "motor_driver.hpp"
#include "esp_log.h"

static const char* TAG = "actuator";

namespace sf {

// =============================================================================
// Local constants — these must mirror main/config.hpp.
// ローカル定数 — main/config.hpp と必ず同期させること。
//
// They are duplicated here (instead of `#include "config.hpp"`) because
// the `main` directory is not on the include path of this component, and
// the task specification forbids modifying CMakeLists.txt. If a value in
// config.hpp changes, update it here as well.
//
// `config.hpp` の include パスがこのコンポーネントから通っておらず、本タスク
// では CMakeLists.txt の変更が禁止されているため、値をミラーしている。
// config.hpp 側の値を変更したら、こちらも必ず更新すること。
// =============================================================================

// Motor GPIOs (X-quad: M1=FR, M2=RR, M3=RL, M4=FL)
// モーター GPIO（X-quad: M1=FR, M2=RR, M3=RL, M4=FL）
static constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
static constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
static constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
static constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// LEDC PWM settings
// LEDC PWM 設定
static constexpr int MOTOR_PWM_FREQ_HZ        = 150000;
static constexpr int MOTOR_PWM_RESOLUTION_BIT = 8;

// Duty range — final clamp before writing to PWM
// duty 範囲 — PWM 書き込み前の最終クランプ値
static constexpr float MIN_DUTY = 0.0f;
static constexpr float MAX_DUTY = 1.0f;

// Yaw torque coefficient ratio (Cq / Ct) for the X-quad propellers.
// X-quad プロペラのヨートルク係数比 (Cq / Ct)。
//
// Used as the multiplier on the yaw command in the mixer. Matches the
// value in the legacy `vehicle/` firmware (control_allocation.hpp).
// ミキサー内のヨー指令に掛ける係数。レガシー `vehicle/` ファーム
// (control_allocation.hpp) と同じ値。
static constexpr float MIXER_KAPPA_DEFAULT = 0.00971f;

// =============================================================================
// Motor driver instance (file-local singleton)
// モータードライバインスタンス（ファイル内シングルトン）
// =============================================================================

// One MotorDriver shared by Actuator::init/update/disarm. Kept file-local so
// that the public header does not need to depend on the HAL.
// Actuator::init/update/disarm から共有する単一の MotorDriver。
// 公開ヘッダーが HAL に依存しないようファイルローカルに保持。
static stampfly::MotorDriver g_motor;

// -----------------------------------------------------------------------------
// makeMotorConfig — build the HAL Config from local mirror of config.hpp
// makeMotorConfig — config.hpp のローカルミラーから HAL Config を構築
// -----------------------------------------------------------------------------
static stampfly::MotorDriver::Config makeMotorConfig()
{
    // Map M1..M4 GPIOs into the FR/RR/RL/FL slots defined by MotorDriver.
    // MotorDriver で定義された FR/RR/RL/FL スロットに M1..M4 GPIO を対応付ける。
    stampfly::MotorDriver::Config cfg{};
    cfg.gpio[stampfly::MotorDriver::MOTOR_FR] = GPIO_MOTOR_M1;
    cfg.gpio[stampfly::MotorDriver::MOTOR_RR] = GPIO_MOTOR_M2;
    cfg.gpio[stampfly::MotorDriver::MOTOR_RL] = GPIO_MOTOR_M3;
    cfg.gpio[stampfly::MotorDriver::MOTOR_FL] = GPIO_MOTOR_M4;
    cfg.pwm_freq_hz         = MOTOR_PWM_FREQ_HZ;
    cfg.pwm_resolution_bits = MOTOR_PWM_RESOLUTION_BIT;
    return cfg;
}

// -----------------------------------------------------------------------------
// init — initialize actuator subsystem and motor HAL
// 初期化 — アクチュエータサブシステムとモーター HAL を初期化
// -----------------------------------------------------------------------------
void Actuator::init()
{
    // Build PWM config and hand it to the HAL.
    // PWM 設定を構築して HAL へ渡す。
    const auto cfg = makeMotorConfig();
    const esp_err_t err = g_motor.init(cfg);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Actuator initialized (LEDC %d Hz, %d-bit)",
                 cfg.pwm_freq_hz, cfg.pwm_resolution_bits);
    } else {
        ESP_LOGE(TAG, "Motor HAL init failed: %s", esp_err_to_name(err));
    }
}

// -----------------------------------------------------------------------------
// mixerCompute — pure X-quad mixer (no I/O, unit-testable)
// mixerCompute — 純粋な X-quad ミキサー（I/O 無し、単体テスト可能）
// -----------------------------------------------------------------------------
//
// Maps total thrust + body-frame torques to per-motor duty cycles for an
// X-frame quad with rotor layout M1=FR, M2=RR, M3=RL, M4=FL.
//
// 総推力＋機体座標トルクを X-frame quad の各モーター duty に変換する。
// ロータ配置: M1=FR, M2=RR, M3=RL, M4=FL。
//
// Sign convention (verified against firmware/vehicle/sf_algo_control B^-1 matrix):
// 符号規約（vehicle/sf_algo_control の B^-1 行列で実機検証済み）:
//   - NED frame: +X forward, +Y right, +Z down.
//   - Pitch>0  = nose-down (right-hand rule about +Y) → FRONT motors get +pitch.
//   - Yaw>0    = clockwise from above (right-hand rule about +Z, i.e. nose-right)
//                → CCW propellers (M1=FR, M3=RL) get +yaw, CW propellers (M2=RR,
//                  M4=FL) get -yaw, because CCW props create CW reaction torque.
//   - Roll>0   = right-wing-down (right-hand rule about +X) → LEFT motors get +roll.
//
// Motor positions (from main/config.hpp): M1=FR (+x,+y,CCW), M2=RR (-x,+y,CW),
//                                          M3=RL (-x,-y,CCW), M4=FL (+x,-y,CW).
//
// Formula (kappa = MIXER_KAPPA_DEFAULT; roll/pitch already pre-divided by 4·L
// in the controller, yaw scaled here by kappa):
// 式（kappa = MIXER_KAPPA_DEFAULT、roll/pitch は制御器側で 4·L 既除算、
// ここでは yaw のみ kappa を掛ける）:
//
//     d_FR (CCW) = thrust − roll + pitch + yaw·κ
//     d_RR (CW ) = thrust − roll − pitch − yaw·κ
//     d_RL (CCW) = thrust + roll − pitch + yaw·κ
//     d_FL (CW ) = thrust + roll + pitch − yaw·κ
//
static void mixerCompute(const ControlOutput& u, float duties[4])
{
    const float T = u.thrust;
    const float R = u.torque[0];   // Roll  / ロール
    const float P = u.torque[1];   // Pitch / ピッチ
    const float Y = u.torque[2] * MIXER_KAPPA_DEFAULT;  // Scaled yaw / κ補正済ヨー

    duties[stampfly::MotorDriver::MOTOR_FR] = T - R + P + Y;
    duties[stampfly::MotorDriver::MOTOR_RR] = T - R - P - Y;
    duties[stampfly::MotorDriver::MOTOR_RL] = T + R - P + Y;
    duties[stampfly::MotorDriver::MOTOR_FL] = T + R + P - Y;
}

// -----------------------------------------------------------------------------
// clampDuty — clamp a single duty value to [MIN_DUTY, MAX_DUTY]
// clampDuty — 1 つの duty 値を [MIN_DUTY, MAX_DUTY] にクランプ
// -----------------------------------------------------------------------------
static inline float clampDuty(float v)
{
    if (v < MIN_DUTY) return MIN_DUTY;
    if (v > MAX_DUTY) return MAX_DUTY;
    return v;
}

// -----------------------------------------------------------------------------
// clampDuties — clamp every duty in a 4-element array
// clampDuties — 4 要素配列の全 duty をクランプ
// -----------------------------------------------------------------------------
static void clampDuties(float duties[4])
{
    for (int i = 0; i < 4; ++i) {
        duties[i] = clampDuty(duties[i]);
    }
}

// -----------------------------------------------------------------------------
// publishMotorOutput — publish per-motor duty to telemetry topic
// publishMotorOutput — 各モーター duty をテレメトリトピックへ発行
// -----------------------------------------------------------------------------
static void publishMotorOutput(const float duties[4], uint32_t timestamp)
{
    MotorOutput out{};
    for (int i = 0; i < 4; ++i) {
        out.duty[i] = duties[i];
    }
    out.timestamp = timestamp;
    actuator_motor.publish(out);
}

// -----------------------------------------------------------------------------
// update — read control output, mix, publish, then drive motors
// 更新 — 制御出力を読み取り、ミキシングし、発行してからモーターを駆動
// -----------------------------------------------------------------------------
void Actuator::update()
{
    // Subscribe to the control output topic.
    // 制御出力トピックを購読する。
    const ControlOutput cmd = control_output.latest();

    // Compute mixer → clamp → publish for telemetry.
    // ミキサー計算 → クランプ → テレメトリ発行。
    float duties[4];
    mixerCompute(cmd, duties);
    clampDuties(duties);
    publishMotorOutput(duties, cmd.timestamp);

    // Write duties to the physical motors. Skip if HAL is not ready yet.
    // 物理モーターへ duty を書き込む。HAL 未初期化ならスキップ。
    if (g_motor.isInitialized()) {
        g_motor.setMotorDuties(duties);
    }
}

// -----------------------------------------------------------------------------
// disarm — drive all motors to zero (safety hook for DISARM transitions)
// disarm — 全モーターを 0 にする（DISARM 遷移用の安全フック）
// -----------------------------------------------------------------------------
void Actuator::disarm()
{
    // Build a zeroed duty vector and write it both to telemetry and HAL.
    // 0 の duty ベクタを作りテレメトリと HAL の両方に書き込む。
    float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    publishMotorOutput(zeros, 0);

    if (g_motor.isInitialized()) {
        g_motor.setMotorDuties(zeros);
    }

    ESP_LOGI(TAG, "Actuator disarmed (all duties = 0)");
}

}  // namespace sf
