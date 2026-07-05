/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
 * Derivative acts on the MEASUREMENT (D-on-M), not the error: differentiating
 * the error would inject a spike (derivative kick) every time the setpoint
 * steps — and the rate setpoint comes from a 12-bit stick, so it steps
 * constantly. D-on-M leaves the closed-loop poles unchanged (the derivative
 * path from the measurement is identical) and only removes the setpoint kick.
 * This matches the flight-proven vehicle/ implementation (sf_algo_pid), so its
 * tuned gains transfer 1:1.
 * 微分は誤差ではなく「測定値」に掛ける（D-on-M）: 誤差を微分すると目標値が階段状に
 * 変わるたびにスパイク（微分キック）が乗る — レート目標は 12bit スティック由来で
 * 常に階段状。D-on-M は閉ループ極を変えず（測定値からの微分経路は同一）、目標値
 * キックだけを除去する。飛行実績のある旧ファーム（vehicle/ sf_algo_pid）と同じ
 * 構成であり、その調整済みゲインがそのまま使える。
 *
 * Anti-windup: conditional integration (stop integrating while the output is
 * saturated in the same direction). The legacy firmware used back-calculation
 * (Tt = √(Ti·Td)) instead; both act only DURING saturation, so the linear
 * behaviour — and therefore gain equivalence — is identical.
 * アンチワインドアップ: 条件付き積分（出力が同方向に飽和中は積分停止）。旧ファームは
 * back-calculation（Tt = √(Ti·Td)）だが、どちらも飽和中にのみ働くため線形領域の
 * 挙動 — つまりゲインの等価性 — は同一。
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
    float integral = 0;       // Integral accumulator / 積分蓄積値
    float deriv_filter = 0;   // Derivative filter state / 微分フィルタ状態
    float prev_error = 0;     // Previous error (integral trapezoid) / 前回誤差（積分台形則用）
    float prev_measurement = 0; // Previous measurement (D-on-M) / 前回測定値（測定値微分用）
    bool  first_run = true;   // Primes the D input after reset / リセット後の微分入力初期化

    /// Compute PID output / PID出力を計算
    /// @param setpoint     Target value / 目標値
    /// @param measurement  Measured value (D-on-M input) / 測定値（測定値微分の入力）
    /// @param dt           Time step [s] / タイムステップ
    /// @return             Control output / 制御出力
    float compute(float setpoint, float measurement, float dt)
    {
        if (dt <= 0) return 0;

        const float error = setpoint - measurement;

        // Proportional / 比例
        float p_term = kp * error;

        // Incomplete derivative (bilinear transform / Tustin) on the MEASUREMENT
        // (D-on-M, see file header). Computed before the integral so the
        // anti-windup saturation check below sees the full output. The first call
        // after reset only primes prev_measurement — differentiating from an
        // arbitrary initial value would kick the output.
        // 不完全微分（双一次変換 / Tustin）、入力は「測定値」（D-on-M、ファイル先頭参照）。
        // アンチワインドアップの飽和判定が全出力を見られるよう、積分より先に計算する。
        // リセット後の初回は prev_measurement の初期化のみ — 任意の初期値からの微分は
        // 出力を蹴ってしまう。
        //
        // D(s) = Kp * Td * s / (eta*Td*s + 1), input = -measurement
        // Bilinear: s → 2/dt * (z-1)/(z+1)
        // D(z) = ((α-1)/(α+1))·D(z)·z⁻¹ − (2Td/(dt(α+1)))·(y−y⁻¹), α = 2*eta*Td/dt
        float d_term = 0;
        if (td > 0) {
            if (first_run) {
                prev_measurement = measurement;
            } else {
                float alpha = 2.0f * eta * td / dt;
                float a = (alpha - 1.0f) / (alpha + 1.0f);  // |a| < 1 for alpha > 0
                float b = 2.0f * td / ((alpha + 1.0f) * dt);
                deriv_filter = a * deriv_filter - b * (measurement - prev_measurement);
                d_term = kp * deriv_filter;
            }
        }
        prev_measurement = measurement;
        first_run = false;

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
        prev_measurement = 0;
        first_run = true;
    }
};

}  // namespace sf
