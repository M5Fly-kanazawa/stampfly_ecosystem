/**
 * @file motor_model.hpp
 * @brief DC Motor Model for Quadcopter
 *
 * DCモータモデル（クワッドコプター用）
 *
 * Motor dynamics (first-order approximation):
 * モータ動特性（1次近似）:
 *
 *   J·dω/dt = Km·i - Dm·ω - Cq·ω² - Qf·sign(ω)
 *   V = Rm·i + Km·ω
 *
 * Steady-state solution (dω/dt = 0):
 * 定常状態解（dω/dt = 0）:
 *
 *   ω = √(T / Ct)                           ... thrust → angular velocity
 *   V = Rm[(Dm + Km²/Rm)ω + Cqω² + Qf] / Km ... angular velocity → voltage
 *   duty = V / Vbat                         ... voltage → duty cycle
 *
 * Physical parameters:
 * 物理パラメータ:
 *   Ct: Thrust coefficient [N/(rad/s)²]
 *   Cq: Torque coefficient [Nm/(rad/s)²]
 *   Rm: Motor resistance [Ω]
 *   Km: Motor constant (back-EMF constant) [V·s/rad]
 *   Dm: Viscous damping coefficient [Nm·s/rad]
 *   Qf: Static friction torque [Nm]
 *
 * @see docs/architecture/control-allocation-migration.md
 * @see docs/control/dc_motor_model.md
 */

#pragma once

#include <cmath>

namespace stampfly {

/**
 * @brief Motor parameters for thrust-to-duty conversion
 * 推力-Duty変換用モータパラメータ
 *
 * Default values are for StampFly's 0720 coreless motors.
 * デフォルト値はStampFlyの0720コアレスモータ用
 */
struct MotorParams {
    // Aerodynamic coefficients
    // 空力係数
    float Ct = 1.0e-8f;       ///< Thrust coefficient [N/(rad/s)²]
    float Cq = 9.71e-11f;     ///< Torque coefficient [Nm/(rad/s)²]

    // Electrical parameters
    // 電気パラメータ
    float Rm = 0.34f;         ///< Motor resistance [Ω]
    float Km = 6.125e-4f;     ///< Motor constant [V·s/rad]

    // Mechanical parameters
    // 機械パラメータ
    float Dm = 3.69e-8f;      ///< Viscous damping [Nm·s/rad]
    float Qf = 2.76e-5f;      ///< Friction torque [Nm]
    float Jm = 1.0e-9f;       ///< Motor+propeller inertia [kg·m²] (for dynamics)

    // Power supply
    // 電源
    float Vbat = 3.7f;        ///< Battery voltage [V]

    // Computed derived parameters
    // 導出パラメータ
    float kappa() const { return Cq / Ct; }  ///< Torque/thrust ratio [m]
};

/**
 * @brief Convert thrust to angular velocity (steady-state)
 * 推力から角速度への変換（定常状態）
 *
 * ω = √(T / Ct)
 *
 * @param thrust Desired thrust [N]
 * @param Ct Thrust coefficient [N/(rad/s)²]
 * @return Angular velocity [rad/s]
 */
inline float thrustToOmega(float thrust, float Ct)
{
    if (thrust <= 0.0f) {
        return 0.0f;
    }
    return std::sqrt(thrust / Ct);
}

/**
 * @brief Convert angular velocity to required voltage (steady-state)
 * 角速度から必要電圧への変換（定常状態）
 *
 * V = Rm[(Dm + Km²/Rm)ω + Cqω² + Qf] / Km
 *
 * @param omega Angular velocity [rad/s]
 * @param params Motor parameters
 * @return Required voltage [V]
 */
inline float omegaToVoltage(float omega, const MotorParams& params)
{
    if (omega <= 0.0f) {
        return 0.0f;
    }

    // Viscous + back-EMF term
    float viscous_term = (params.Dm + params.Km * params.Km / params.Rm) * omega;
    // Aerodynamic torque term
    float aero_term = params.Cq * omega * omega;
    // Friction term
    float friction_term = params.Qf;

    // Total current × resistance = voltage
    float voltage = params.Rm * (viscous_term + aero_term + friction_term) / params.Km;

    return voltage;
}

/**
 * @brief Convert thrust to duty cycle (steady-state approximation)
 * 推力からDutyサイクルへの変換（定常状態近似）
 *
 * @param thrust Desired thrust [N]
 * @param params Motor parameters
 * @return Duty cycle [0.0 to 1.0]
 */
inline float thrustToDuty(float thrust, const MotorParams& params)
{
    if (thrust <= 0.0f) {
        return 0.0f;
    }

    // Thrust → Angular velocity
    float omega = thrustToOmega(thrust, params.Ct);

    // Angular velocity → Voltage
    float voltage = omegaToVoltage(omega, params);

    // Voltage → Duty
    float duty = voltage / params.Vbat;

    // Clamp to valid range
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    return duty;
}

/**
 * @brief Convert duty cycle to expected thrust (inverse of thrustToDuty)
 * Dutyサイクルから期待推力への変換（thrustToDutyの逆関数）
 *
 * Uses Newton-Raphson iteration for inverse calculation.
 * ニュートン・ラフソン法で逆計算
 *
 * @param duty Duty cycle [0.0 to 1.0]
 * @param params Motor parameters
 * @param max_iter Maximum iterations (default: 10)
 * @return Expected thrust [N]
 */
inline float dutyToThrust(float duty, const MotorParams& params, int max_iter = 10)
{
    if (duty <= 0.0f) {
        return 0.0f;
    }

    // Initial guess: assume linear relationship
    // T_max at duty=1.0 is approximately when omega gives V=Vbat
    // Start with a reasonable initial guess
    float thrust = duty * duty * 0.15f;  // Rough quadratic approximation

    // Newton-Raphson iteration
    for (int i = 0; i < max_iter; i++) {
        float computed_duty = thrustToDuty(thrust, params);
        float error = computed_duty - duty;

        if (std::abs(error) < 1e-6f) {
            break;
        }

        // Numerical derivative
        float delta = thrust * 0.01f + 1e-8f;
        float duty_plus = thrustToDuty(thrust + delta, params);
        float derivative = (duty_plus - computed_duty) / delta;

        if (std::abs(derivative) < 1e-10f) {
            break;
        }

        thrust -= error / derivative;
        if (thrust < 0.0f) thrust = 0.0f;
    }

    return thrust;
}

/**
 * @brief Convert motor thrusts array to duty cycles array
 * モータ推力配列をDutyサイクル配列に変換
 *
 * @param thrusts [T₁, T₂, T₃, T₄] motor thrusts [N]
 * @param duties_out [d₁, d₂, d₃, d₄] duty cycles [0.0 to 1.0]
 * @param params Motor parameters
 */
inline void thrustsToDuties(const float thrusts[4], float duties_out[4],
                            const MotorParams& params)
{
    for (int i = 0; i < 4; i++) {
        duties_out[i] = thrustToDuty(thrusts[i], params);
    }
}

/**
 * @brief Compute motor torque from angular velocity (for simulation)
 * 角速度からモータトルクを計算（シミュレーション用）
 *
 * τ_motor = Km·i = Km·(V - Km·ω) / Rm
 *
 * @param omega Angular velocity [rad/s]
 * @param voltage Applied voltage [V]
 * @param params Motor parameters
 * @return Motor electromagnetic torque [Nm]
 */
inline float computeMotorTorque(float omega, float voltage, const MotorParams& params)
{
    float back_emf = params.Km * omega;
    float current = (voltage - back_emf) / params.Rm;
    return params.Km * current;
}

/**
 * @brief Compute aerodynamic thrust from angular velocity
 * 角速度から空力推力を計算
 *
 * T = Ct·ω²
 *
 * @param omega Angular velocity [rad/s]
 * @param Ct Thrust coefficient [N/(rad/s)²]
 * @return Thrust [N]
 */
inline float computeThrust(float omega, float Ct)
{
    return Ct * omega * omega;
}

/**
 * @brief Compute aerodynamic torque (reaction) from angular velocity
 * 角速度から空力トルク（反作用）を計算
 *
 * Q = Cq·ω²
 *
 * @param omega Angular velocity [rad/s]
 * @param Cq Torque coefficient [Nm/(rad/s)²]
 * @return Reaction torque [Nm]
 */
inline float computeAeroTorque(float omega, float Cq)
{
    return Cq * omega * omega;
}

// Default motor parameters instance
// デフォルトモータパラメータインスタンス
extern const MotorParams DEFAULT_MOTOR_PARAMS;

}  // namespace stampfly
