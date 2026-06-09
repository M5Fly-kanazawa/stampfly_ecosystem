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
 * @design detailed_design.md §4 — IController implementation          [OK]
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

        // Incomplete derivative (bilinear transform / Tustin). Computed before the
        // integral so the anti-windup saturation check below sees the full output.
        // 不完全微分（双一次変換 / Tustin）。アンチワインドアップの飽和判定が全出力を
        // 見られるよう、積分より先に計算する。
        //
        // D(s) = Kp * Td * s / (eta*Td*s + 1)
        // Bilinear: s → 2/dt * (z-1)/(z+1)
        // D(z) = ((α-1)/(α+1)) * D(z)*z⁻¹ + (2Td/(dt(α+1))) * (e-e⁻¹), α = 2*eta*Td/dt
        float d_term = 0;
        if (td > 0) {
            float alpha = 2.0f * eta * td / dt;
            float a = (alpha - 1.0f) / (alpha + 1.0f);  // |a| < 1 for alpha > 0
            float b = 2.0f * td / ((alpha + 1.0f) * dt);
            deriv_filter = a * deriv_filter + b * (error - prev_error);
            d_term = kp * deriv_filter;
        }

        // Integral (trapezoidal / Tustin) with CONDITIONAL-INTEGRATION anti-windup.
        // Merely clamping the integral to ±output_limit is NOT anti-windup: the
        // integral can still wind up to the full output magnitude while the output is
        // saturated, then unwinds slowly when the error reverses → overshoot. So we
        // only accumulate when the resulting output would NOT push an already-saturated
        // output further into saturation. A hard clamp remains as a backstop.
        // 積分（台形 / Tustin）＋条件付き積分アンチワインドアップ。積分を ±output_limit で
        // クランプするだけは不十分: 出力が飽和している間も積分が出力全幅まで巻き上がり、
        // 誤差反転後にゆっくり戻る→オーバーシュート。よって、飽和中の出力をさらに飽和方向へ
        // 押す場合は積分を更新しない（条件付き積分）。ハードクランプは保険として残す。
        // Integrate when Ti is meaningful. Inclusive bound: the parameter system
        // allows ti down to exactly 0.01, which must still integrate (a strict
        // ">" silently disabled the integrator at the minimum allowed value).
        // Ti が有意なら積分する。境界は含む: パラメータは ti=0.01 ちょうどまで許容され、
        // その値でも積分すべき（">" だと許容最小値で積分が黙って無効化されていた）。
        if (ti >= 0.01f) {
            float i_next = integral + (kp / ti) * (error + prev_error) * (dt * 0.5f);
            float out_test = p_term + i_next + d_term;
            bool push_high = (out_test >  output_limit) && (error > 0);
            bool push_low  = (out_test < -output_limit) && (error < 0);
            if (!push_high && !push_low) {
                integral = i_next;
            }
            if (integral >  output_limit) integral =  output_limit;   // backstop
            if (integral < -output_limit) integral = -output_limit;
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
