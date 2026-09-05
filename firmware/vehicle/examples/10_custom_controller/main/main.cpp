/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file main.cpp
 * @brief Example 10 — L1 IController exercise: run `LearnerController`
 *        (learner_controller.hpp) against synthetic inputs and print its
 *        `ControlOutput`. No HAL, no Topics, no vehicle firmware needed —
 *        this bench proves the class compiles and behaves like an
 *        `IController` in complete isolation.
 *        サンプル10 — L1 IController 演習: `LearnerController`
 *        （learner_controller.hpp）を合成入力に対して動かし `ControlOutput` を
 *        表示する。HAL も Topic も実ファームも不要 — このベンチは、このクラスが
 *        単体で `IController` としてビルド・動作することを示す。
 *
 * This is a BENCH, not a flight simulator: there is no plant model (no
 * physics connects torque back to attitude). The "disturbance" is a
 * synthetic pitch-tilt sine wave chosen by config.hpp, not something the
 * controller's own output caused. See README.md "実機で飛ばすレシピ / Recipe
 * to actually fly this" for what is needed to use `LearnerController` in the
 * real firmware.
 * これは「ベンチ」であって飛行シミュレータではない: プラントモデル（トルクが
 * 姿勢へフィードバックする物理）が無い。「外乱」は config.hpp が選ぶ合成ピッチ
 * 傾きの正弦波であり、制御器自身の出力が引き起こしたものではない。実ファームで
 * `LearnerController` を使うために必要なことは README.md「実機で飛ばすレシピ /
 * Recipe to actually fly this」を参照。
 *
 * @design architecture.md §2.5 — L1: IController を実装して差替え         [OK]
 * @design controller.hpp — IController::compute() contract              [OK]
 * @design coding_and_education.md §3 — Examples: 単独ビルド可能           [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "learner_controller.hpp"
#include "flight_state.hpp"
#include "config.hpp"

#include <cmath>

static const char* TAG = "custom_controller";

/// Build a synthetic attitude: level roll/yaw, a slowly oscillating pitch
/// tilt standing in for "the drone got disturbed". Quaternion for a
/// pitch-only rotation by `pitch_radians` about the body Y axis is
/// [cos(p/2), 0, sin(p/2), 0] — see sf_math.hpp Quat::to_euler() for the
/// inverse of this formula.
/// 合成姿勢を作る: roll/yaw は水平、ピッチだけがゆっくり振動する傾き
/// （「機体が外乱を受けた」の代替）。機体Y軸まわりの `pitch_radians` 純ピッチ回転の
/// クォータニオンは [cos(p/2), 0, sin(p/2), 0] — 逆変換は sf_math.hpp の
/// Quat::to_euler() を参照。
static sf::StateEstimate buildSyntheticState(uint32_t step_count)
{
    uint32_t phase_step = step_count % config::kSyntheticPitchPeriodSteps;
    float phase_radians = 2.0f * static_cast<float>(M_PI) *
                           static_cast<float>(phase_step) /
                           static_cast<float>(config::kSyntheticPitchPeriodSteps);
    float pitch_radians = config::kSyntheticPitchAmplitudeRadians * sinf(phase_radians);

    sf::StateEstimate state = {};
    state.attitude[0] = cosf(pitch_radians * 0.5f);  // w
    state.attitude[2] = sinf(pitch_radians * 0.5f);  // y (pitch axis)
    return state;
}

/// Build a synthetic pilot setpoint: sticks centred, mid throttle — "hold
/// level, hover".
/// 合成パイロット指令を作る: スティック中央、スロットル中間 —
/// 「水平を保ってホバー」。
static sf::CommandSetpoint buildSyntheticSetpoint()
{
    sf::CommandSetpoint setpoint = {};
    setpoint.throttle = 0.5f;
    return setpoint;
}

/// Print one ControlOutput line: thrust, the 3 body-frame torques, and the
/// outer-loop tilt setpoints the attitude cascade computed.
/// ControlOutput を1行表示する: 推力、機体系トルク3軸、姿勢カスケードが計算した
/// 外側ループの傾き目標。
static void printControlOutput(const sf::ControlOutput& output)
{
    float roll_ref_degrees = output.angle_ref[0] * config::kRadToDeg;
    float pitch_ref_degrees = output.angle_ref[1] * config::kRadToDeg;

    printf("thrust=%6.2fN  torque(RPY)=[%+7.4f %+7.4f %+7.4f]Nm  "
           "angle_ref(RP)=[%+6.2f %+6.2f]deg\n",
           output.thrust, output.torque[0], output.torque[1], output.torque[2],
           roll_ref_degrees, pitch_ref_degrees);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Example 10: L1 IController exercise ===");

    // The class under test: an IController that forwards everything to a
    // real PidController except one exercise hook — see
    // learner_controller.hpp for why this is the shape of the exercise.
    // 検証対象のクラス: 1つの演習フックを除き全てを実際の PidController へ転送する
    // IController — この形にした理由は learner_controller.hpp を参照。
    sf::LearnerController controller;
    controller.init();
    controller.onModeChange(sf::FlightMode::STABILIZE);

    uint32_t step_count = 0;
    while (true) {
        sf::StateEstimate state = buildSyntheticState(step_count);
        sf::CommandSetpoint setpoint = buildSyntheticSetpoint();
        sf::ControlOutput output =
            controller.compute(state, setpoint, config::kControlComputePeriodSeconds);

        printControlOutput(output);

        ++step_count;
        vTaskDelay(pdMS_TO_TICKS(config::kPrintPeriodMilliseconds));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change config::kYawTorqueScale to 2.0 and rebuild — torque(RPY)'s 3rd
//    (yaw) column doubles, while roll/pitch (computed by the SAME inner
//    PidController) stay exactly the same. That isolation is the point of
//    wrapping instead of copy-pasting the cascade.
//    config::kYawTorqueScale を 2.0 に変えて再ビルドしてみよう — torque(RPY) の
//    3番目（ヨー）列が倍になる一方、roll/pitch（同じ内側の PidController が計算）は
//    全く変わらない。この分離こそが、カスケードをコピペせずラップする狙い。
//
// 2. Replace the body of compute() in learner_controller.cpp entirely — e.g.
//    delete the call to inner_controller_.compute() and hand-write a single
//    P controller on pitch. main.cpp does not need to change at all: it only
//    knows the `IController` interface.
//    learner_controller.cpp の compute() の中身を丸ごと置き換えてみよう — 例えば
//    inner_controller_.compute() の呼び出しを消し、ピッチ用のP制御器を自分で
//    書いてみる。main.cpp は一切変更不要 — `IController` インターフェースしか
//    知らないため。
//
// 3. Change config::kSyntheticPitchAmplitudeRadians to 0.0f — the tilt
//    disappears and torque(RPY)'s pitch column settles near zero (STABILIZE
//    holding level against no disturbance).
//    config::kSyntheticPitchAmplitudeRadians を 0.0f に変えてみよう — 傾きが消え、
//    torque(RPY) のピッチ列がほぼ0に収束する（外乱なしで水平を保つSTABILIZE）。
// ============================================================
