/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file pid.hpp
 * @brief Single-axis PID controller with incomplete derivative (Tustin)
 *        不完全微分PID制御器（双一次変換）
 *
 * Transfer function: C(s) = Kp(1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1))
 * 伝達関数: C(s) = Kp(1 + 1/(Ti*s) + Td*s/(η*Td*s + 1))
 *
 * Discretization: bilinear transform (Tustin) for all terms
 * 離散化: 全項に双一次変換（Tustin）を使用
 *
 * Matches vehicle/ firmware PID implementation (sf_algo_pid).
 * 旧ファーム（vehicle/）のPID実装と同じ離散化手法。
 *
 * @design detailed_design.md §4 — IController implementation          [--]
 */

#pragma once

namespace sf {

/// Single-axis PID controller / 1軸PIDコントローラ
struct PID {
    // Gains / ゲイン
    float kp = 0;        // Proportional gain / 比例ゲイン
    float ti = 1.0f;     // Integral time [s] / 積分時間
    float td = 0;        // Derivative time [s] / 微分時間
    float eta = 0.125f;  // Derivative filter coefficient / 微分フィルタ係数

    // Output limit / 出力制限
    float output_limit = 1.0f;

    // State / 状態
    float integral = 0;     // Integral accumulator / 積分蓄積値
    float deriv_filter = 0; // Derivative filter state / 微分フィルタ状態
    float prev_error = 0;   // Previous error / 前回誤差

    /// Compute PID output / PID出力を計算
    /// @param error  Error signal (setpoint - measurement) / 誤差信号
    /// @param dt     Time step [s] / タイムステップ
    /// @return       Control output / 制御出力
    float compute(float error, float dt)
    {
        if (dt <= 0) return 0;

        // Proportional / 比例
        float p_term = kp * error;

        // Integral (trapezoidal / Tustin) with anti-windup
        // 積分（台形積分 / Tustin）+ アンチワインドアップ
        if (ti > 0.01f) {
            integral += (kp / ti) * (error + prev_error) * (dt * 0.5f);
            if (integral > output_limit) integral = output_limit;
            if (integral < -output_limit) integral = -output_limit;
        }

        // Incomplete derivative (bilinear transform / Tustin)
        // 不完全微分（双一次変換 / Tustin）
        //
        // D(s) = Kp * Td * s / (eta*Td*s + 1)
        //
        // Bilinear: s → 2/dt * (z-1)/(z+1)
        // D(z) = ((α-1)/(α+1)) * D(z)*z⁻¹ + (2Td/(dt(α+1))) * (e-e⁻¹)
        // where α = 2*eta*Td/dt
        //
        // 双一次変換: s → 2/dt * (z-1)/(z+1)
        float d_term = 0;
        if (td > 0) {
            float alpha = 2.0f * eta * td / dt;
            float a = (alpha - 1.0f) / (alpha + 1.0f);  // |a| < 1 for alpha > 0
            float b = 2.0f * td / ((alpha + 1.0f) * dt);
            deriv_filter = a * deriv_filter + b * (error - prev_error);
            d_term = kp * deriv_filter;
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
