/**
 * @file pid.hpp
 * @brief Single-axis PID controller with incomplete derivative
 *        不完全微分PID制御器（1軸）
 *
 * Transfer function: C(s) = Kp(1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1))
 * 伝達関数: C(s) = Kp(1 + 1/(Ti*s) + Td*s/(η*Td*s + 1))
 *
 * @design detailed_design.md §4 — IController implementation          [--]
 */

#pragma once

namespace sf {

/// Single-axis PID controller / 1軸PIDコントローラ
struct PID {
    // Gains / ゲイン
    float kp = 0;     // Proportional gain / 比例ゲイン
    float ti = 1.0f;  // Integral time [s] / 積分時間
    float td = 0;     // Derivative time [s] / 微分時間
    float eta = 0.125f;  // Derivative filter coefficient / 微分フィルタ係数

    // Output limit / 出力制限
    float output_limit = 1.0f;

    // State / 状態
    float integral = 0;    // Integral accumulator / 積分蓄積値
    float deriv_filter = 0; // Derivative filter state / 微分フィルタ状態
    float prev_error = 0;  // Previous error / 前回誤差

    /// Compute PID output / PID出力を計算
    /// @param error  Error signal (setpoint - measurement) / 誤差信号
    /// @param dt     Time step [s] / タイムステップ
    /// @return       Control output / 制御出力
    float compute(float error, float dt)
    {
        if (dt <= 0) return 0;

        // Proportional / 比例
        float p_term = kp * error;

        // Integral with anti-windup / アンチワインドアップ付き積分
        if (ti > 0.01f) {
            integral += (kp / ti) * error * dt;
            // Clamp integral / 積分をクランプ
            if (integral > output_limit) integral = output_limit;
            if (integral < -output_limit) integral = -output_limit;
        }

        // Incomplete derivative (filtered) / 不完全微分（フィルタ付き）
        // D(s) = Kp * Td * s / (eta*Td*s + 1)
        // Discrete: d[n] = α*d[n-1] + K*(e[n]-e[n-1])
        //   α = η*Td / (η*Td + dt),  K = Kp*Td / (η*Td + dt)
        float d_term = 0;
        if (td > 0) {
            float tau = td * eta;            // Filter time constant
            float alpha = tau / (tau + dt);  // Filter coefficient (0 < α < 1)
            float K = kp * td / (tau + dt);  // Derivative gain
            deriv_filter = alpha * deriv_filter + K * (error - prev_error);
            d_term = deriv_filter;
        }

        prev_error = error;

        // Total output with limit / 制限付き合計出力
        float output = p_term + integral + d_term;
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;

        return output;
    }

    /// Reset internal state / 内部状態をリセット
    void reset()
    {
        integral = 0;
        deriv_filter = 0;
        prev_error = 0;
    }
};

}  // namespace sf
