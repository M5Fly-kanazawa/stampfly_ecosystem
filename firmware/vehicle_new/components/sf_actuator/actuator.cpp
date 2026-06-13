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

#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "actuator.hpp"
#include "topics.hpp"
#include "motor_hw.hpp"   // sf::hw motor PWM numerics — SSOT shared with sf_board (M-5)
#include "motor_driver.hpp"
#include "esp_log.h"
#include "esp_timer.h"   // applyTestDuties timestamp

static const char* TAG = "actuator";

namespace sf {

// =============================================================================
// Motor hardware constants (GPIO + PWM)
//
// These currently mirror main/config.hpp because config.hpp lives under main/,
// which DEPENDS ON this component — so sf_actuator cannot include it without a
// circular dependency. Phase 4 (HW ownership, R1/R2) resolves this for real by
// moving the LEDC timer + motor GPIOs into sf_board (the BSP), which both this
// component and the motor HAL then borrow. Until then, keep these in sync with
// config.hpp.
//
// モータ HW 定数（GPIO + PWM）。config.hpp は main/ にあり本コンポーネントに依存するため、
// 循環依存になり include できない（その値をミラーしている理由）。Phase 4（HW 所有・R1/R2）で
// LEDC タイマーとモータ GPIO を sf_board（BSP）へ移し、本コンポーネントとモータ HAL が借りる
// 形で本質的に解消する。それまでは config.hpp と同期させること。
// =============================================================================

// Motor GPIOs (X-quad: M1=FR, M2=RR, M3=RL, M4=FL)
// モーター GPIO（X-quad: M1=FR, M2=RR, M3=RL, M4=FL）
static constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
static constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
static constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
static constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// LEDC PWM settings — sourced from the shared SSOT (sf::hw, motor_hw.hpp) so the
// duty resolution fed to the motor HAL can never diverge from the LEDC timer
// resolution sf_board configures (M-5). Do NOT redefine these literally here.
// LEDC PWM 設定 — 共有 SSOT (sf::hw, motor_hw.hpp) から取得。モータ HAL に渡す
// duty 分解能が sf_board の構成するタイマ分解能と乖離し得ないようにする (M-5)。
// ここで数値を再定義しないこと。
static constexpr int MOTOR_PWM_FREQ_HZ        = sf::hw::kMotorPwmFreqHz;
static constexpr int MOTOR_PWM_RESOLUTION_BIT = sf::hw::kMotorPwmResolutionBit;

// Duty range — final clamp before writing to PWM
// duty 範囲 — PWM 書き込み前の最終クランプ値
static constexpr float MIN_DUTY = 0.0f;
static constexpr float MAX_DUTY = 1.0f;

// X-quad geometry + motor curve for the B^-1 control allocation. Mirrors the
// flight-proven legacy `vehicle/` allocator (control_allocation.hpp) and the
// documented motor model (docs/architecture/stampfly-parameters.md §3). The
// control inputs are PHYSICAL — thrust [N] and torques [Nm] — and the mixer
// inverts the allocation, then converts each motor thrust to a PWM duty through
// the motor curve (the exact inverse of the SIL plant's duty→thrust).
// X-quad 幾何＋モータ曲線（B^-1 制御配分用）。飛行実績のあるレガシー配分器と文書の
// モータモデルに準拠。制御入力は物理量（推力 [N]・トルク [Nm]）で、ミキサーは配分を
// 逆算し各モータ推力をモータ曲線で PWM duty に変換する（SIL プラントの duty→推力 の逆）。
static constexpr float ARM_D    = 0.023f;    // moment arm (x/y offset) [m]
static constexpr float KAPPA    = 0.00971f;  // torque/thrust ratio Cq/Ct [m]
static constexpr float MOTOR_AM = 5.39e-8f;  // V = Am·ω² + Bm·ω + Cm
static constexpr float MOTOR_BM = 6.33e-4f;
static constexpr float MOTOR_CM = 1.53e-2f;
static constexpr float MOTOR_CT = 1.00e-8f;  // thrust T = Ct·ω² [N]

// Supply voltage used to convert motor curve volts → PWM duty (duty = V/Vbat).
// Uses the LIVE battery voltage from sensor_power so the thrust→duty stage tracks
// the 1S LiPo as it sags under load — REQUIRED on real HW (a fixed assumption would
// under/over-drive the motors as the pack deviates from nominal) and matched in SIL
// by the plant's battery sag model (Model Identity). The floor guards the boot
// window (no PowerData yet → voltage 0) and any implausible reading so thrustToDuty
// never divides by ~0 and blows up the duty.
// モータ曲線の電圧 → PWM duty 変換（duty = V/Vbat）に使う電源電圧。sensor_power の実電圧を
// 使い、負荷で垂下する 1S LiPo に thrust→duty 段を追従させる。実機で必須（固定だとパックが
// 公称からずれると過/不足駆動）。SIL ではプラントの電池サグモデルで一致（Model Identity）。
// 下限ガードは起動窓（PowerData 未発行→電圧0）と異常値を弾き、~0 除算で duty を暴発させない。
static constexpr float V_BATT_NOMINAL = 3.7f;   // 1S LiPo nominal fallback [V]
static constexpr float V_BATT_MIN     = 2.5f;   // below this → treat as invalid [V]

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
    // sf_board owns the shared LEDC timer (R1) and configures it in Phase 1, so
    // the motor HAL must not re-configure it — only bind its channels. Set as a
    // plain bool (not via sf_board) because this source is shared with the
    // partial SIL test programs that do not link sf_board.
    // 共有 LEDC タイマは sf_board が所有(R1)し Phase 1 で構成するため、モータ HAL
    // は再構成せずチャンネル紐付けのみ行う。この .cpp は sf_board をリンクしない
    // 部分 SIL 試験と共有のため bool で設定する。
    cfg.skip_timer_init = true;
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
        return;
    }

    // Motor HAL is Critical (R4, hardware_init.md §5): no PWM means no thrust.
    // Do not return and let control_task run a dead motor path — halt loudly
    // (no esp_restart, so the cause stays observable; LED pattern is Phase 6).
    // モータ HAL は Critical (R4, §5): PWM 不能＝推力ゼロ。control_task に死んだ
    // モータ経路を回させず大きく停止する (esp_restart せず原因を保つ。LED は Phase 6)。
    ESP_LOGE(TAG, "CRITICAL: Motor HAL init failed: %s — vehicle cannot fly. Halting.",
             esp_err_to_name(err));
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

// -----------------------------------------------------------------------------
// thrustToDuty — per-motor thrust [N] → PWM duty [0,1] via the motor curve:
//   ω = √(T/Ct) ;  V = Am·ω² + Bm·ω + Cm ;  duty = V / Vbat.
// Exact inverse of the SIL plant's duty→thrust, so a thrust the allocator
// commands is the thrust the (modeled) motor produces. Vbat is the LIVE supply
// voltage (passed in), so the conversion tracks the 1S LiPo as it sags under load.
// thrustToDuty — 各モータ推力 [N] → PWM duty [0,1]（モータ曲線）。SIL プラントの
// duty→推力 の厳密な逆。Vbat は実電源電圧（引数）で、負荷で垂下する 1S LiPo に追従する。
// -----------------------------------------------------------------------------
static float thrustToDuty(float thrust, float vbat)
{
    if (thrust <= 0.0f) return MIN_DUTY;
    const float omega = std::sqrt(thrust / MOTOR_CT);
    const float volts = MOTOR_AM * omega * omega + MOTOR_BM * omega + MOTOR_CM;
    return volts / vbat;   // final clamp to [MIN,MAX]_DUTY by clampDuties()
}

// -----------------------------------------------------------------------------
// mixerCompute — B^-1 control allocation (physical units, no I/O, unit-testable).
// mixerCompute — B^-1 制御配分（物理単位・I/O 無し・単体テスト可能）。
// -----------------------------------------------------------------------------
//
// Inputs are PHYSICAL (matching ControlOutput's declared units): u.thrust = total
// thrust [N], u.torque = body torques [Nm] (roll φ, pitch θ, yaw ψ). The mixing
// matrix B^-1 (from the flight-proven legacy allocator, control_allocation.hpp)
// gives each motor's thrust, then thrustToDuty gives the PWM duty. Rotor layout
// M1=FR, M2=RR, M3=RL, M4=FL; NED body frame (+X fwd, +Y right, +Z down);
// CCW props M1/M3, CW props M2/M4.
//
// 入力は物理量（ControlOutput の宣言単位どおり）: u.thrust = 総推力 [N]、u.torque =
// 機体トルク [Nm]（ロール/ピッチ/ヨー）。飛行実績のあるレガシー配分器の B^-1 で各モータ
// 推力を求め、thrustToDuty で PWM duty にする。
//
//   T_i = ¼( u_t  ±  u_φ/d  ±  u_θ/d  ±  u_ψ/κ )
//
// NOTE: yaw uses 1/κ (NOT ·κ). The old simplified mixer multiplied by κ and so
// under-drove yaw by ~1/κ² (a 1 rad/s yaw command tracked at 0.05 rad/s).
// 注意: ヨーは 1/κ（·κ ではない）。旧簡易ミキサーは ·κ でヨーを約 1/κ² も弱めていた。
//
static void mixerCompute(const ControlOutput& u, float vbat, float duties[4])
{
    const float ut = u.thrust;       // total thrust [N]
    const float up = u.torque[0];    // roll torque  [Nm]
    const float uq = u.torque[1];    // pitch torque [Nm]
    const float ur = u.torque[2];    // yaw torque   [Nm]
    const float id = 1.0f / ARM_D;   // 1/d
    const float ik = 1.0f / KAPPA;   // 1/κ

    // Per-motor thrust [N] from B^-1 (rows = M1FR, M2RR, M3RL, M4FL).
    // 各モータ推力 [N]（B^-1 の行 = M1FR, M2RR, M3RL, M4FL）。
    float T[4];
    T[stampfly::MotorDriver::MOTOR_FR] = 0.25f * (ut - id * up + id * uq + ik * ur);
    T[stampfly::MotorDriver::MOTOR_RR] = 0.25f * (ut - id * up - id * uq - ik * ur);
    T[stampfly::MotorDriver::MOTOR_RL] = 0.25f * (ut + id * up - id * uq + ik * ur);
    T[stampfly::MotorDriver::MOTOR_FL] = 0.25f * (ut + id * up + id * uq - ik * ur);

    for (int i = 0; i < 4; ++i) duties[i] = thrustToDuty(T[i], vbat);
}

// -----------------------------------------------------------------------------
// batteryVoltage — live 1S LiPo voltage from sensor_power, with a safe fallback.
// At boot (no PowerData yet → voltage 0) or on an implausible reading (< V_BATT_MIN)
// fall back to the nominal so thrustToDuty never divides by ~0 and blows up the duty.
// batteryVoltage — sensor_power からの実 1S LiPo 電圧（安全フォールバック付き）。起動時
// (PowerData 未発行→電圧0) や異常値(< V_BATT_MIN)では公称に戻し、thrustToDuty が ~0 で
// 割って duty を暴発させないようにする。
// -----------------------------------------------------------------------------
static float batteryVoltage()
{
    const float v = sensor_power.latest().voltage;
    return (v < V_BATT_MIN) ? V_BATT_NOMINAL : v;
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
// arm — enable motor output (safety gate for ARM transitions).
// arm — モーター出力を有効化（ARM 遷移用の安全ゲート）。
//
// The motor HAL gates every duty write behind its own armed_ flag, so update()
// is a no-op until the HAL is armed here. Idempotent: only the disarmed→armed
// edge touches the HAL, so control_task can call arm() each cycle while armed.
// モーター HAL は全 duty 書き込みを自身の armed_ で gate するため、ここで arm する
// まで update() は無効。冪等: 立上りエッジのみ HAL に触れ、armed 中は毎周期呼べる。
// -----------------------------------------------------------------------------
void Actuator::arm()
{
    if (armed_) {
        return;   // already armed — nothing to do / 既に arm 済み
    }
    if (!g_motor.isInitialized()) {
        return;   // HAL not ready; stay disarmed / HAL 未初期化なら disarm のまま
    }
    g_motor.arm();
    armed_ = true;
    ESP_LOGI(TAG, "Actuator armed (motor output enabled)");
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

    // Compute mixer (with supply voltage) → clamp → publish for telemetry.
    // ミキサー計算（電源電圧で）→ クランプ → テレメトリ発行。
    float duties[4];
    mixerCompute(cmd, batteryVoltage(), duties);
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
    if (!armed_) {
        return;   // already disarmed — motors already at 0 / 既に disarm 済み
    }
    armed_ = false;

    // Telemetry: reflect motors-off. NOTE: zero the HAL via g_motor.disarm(),
    // NOT setMotorDuties(zeros) — the latter is itself gated behind the HAL
    // armed_ flag and would no-op exactly when we need it (during DISARM).
    // テレメトリ: モーター停止を反映。HAL のゼロ化は g_motor.disarm() で行う
    // （setMotorDuties は HAL の armed_ で gate されており DISARM 時に無効になる）。
    float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    publishMotorOutput(zeros, 0);

    if (g_motor.isInitialized()) {
        g_motor.disarm();   // forces LEDC duty=0 and clears the HAL arm gate
    }

    ESP_LOGI(TAG, "Actuator disarmed (all duties = 0)");
}

// -----------------------------------------------------------------------------
// applyTestDuties — bench motor test: drive raw per-motor duties, bypassing the mixer.
// applyTestDuties — ベンチ用モータテスト: ミキサーを介さず生 duty で各モータを駆動。
//
// SAFETY: the caller (ControlTask) gates this to the DISARMED state only; it is never
// reached while armed. We arm the HAL (idempotent) so it accepts the duty write, set the
// duties, and publish for telemetry/SIL. ControlTask calls disarm() to stop the test.
// 安全: 呼び出し側（ControlTask）が disarmed 限定に gate し、armed 中は到達しない。HAL を arm
// （冪等）して duty 書き込みを受理させ、duty を設定し、テレメトリ/SIL 用に発行する。テスト停止は
// ControlTask が disarm() を呼ぶ。
// -----------------------------------------------------------------------------
void Actuator::applyTestDuties(const float duties[4])
{
    arm();   // idempotent: ensure the motor HAL accepts duty writes
    if (g_motor.isInitialized()) {
        g_motor.setMotorDuties(duties);
    }
    publishMotorOutput(duties, static_cast<uint32_t>(esp_timer_get_time()));
}

}  // namespace sf
