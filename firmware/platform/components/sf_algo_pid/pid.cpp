/**
 * @file pid.cpp
 * @brief PID Controller Implementation
 */

#include "pid.hpp"

namespace stampfly {

void PID::init(const PIDConfig& config)
{
    Kp_ = config.Kp;
    Ti_ = config.Ti;
    Td_ = config.Td;
    eta_ = config.eta;
    output_min_ = config.output_min;
    output_max_ = config.output_max;
    derivative_on_measurement_ = config.derivative_on_measurement;

    // Tracking time constant for anti-windup
    // Default: sqrt(Ti * Td), but at least Ti if Td is 0
    if (config.Tt > 0.0f) {
        Tt_ = config.Tt;
    } else if (Ti_ > 0.0f && Td_ > 0.0f) {
        Tt_ = std::sqrt(Ti_ * Td_);
    } else if (Ti_ > 0.0f) {
        Tt_ = Ti_;
    } else {
        Tt_ = 1.0f;  // Fallback
    }

    reset();
}

void PID::reset()
{
    integral_ = 0.0f;
    deriv_filtered_ = 0.0f;
    prev_error_ = 0.0f;
    prev_deriv_input_ = 0.0f;
    first_run_ = true;
    P_ = 0.0f;
    I_ = 0.0f;
    D_ = 0.0f;
    error_ = 0.0f;
}

float PID::update(float setpoint, float measurement, float dt)
{
    // 1. Compute error
    error_ = setpoint - measurement;

    // 2. Proportional term
    P_ = Kp_ * error_;

    // 3. Integral term (trapezoidal integration)
    I_ = updateIntegral(error_, dt);

    // 4. Derivative term (incomplete derivative with D-on-M option)
    D_ = updateDerivative(error_, measurement, dt);

    // 5. Compute unlimited output
    float output_unlimited = P_ + I_ + D_;

    // 6. Clamp output
    float output = clamp(output_unlimited);

    // 7. Apply back-calculation anti-windup
    applyAntiWindup(output_unlimited, output, dt);

    // 8. Update previous values for next iteration
    prev_error_ = error_;
    first_run_ = false;

    return output;
}

float PID::updateIntegral(float error, float dt)
{
    // Skip integral if Ti <= 0
    if (Ti_ <= 0.0f) {
        return 0.0f;
    }

    // Trapezoidal integration (bilinear transform of 1/(Ti·s))
    // I[k] = I[k-1] + (T / (2·Ti)) * (e[k] + e[k-1])
    // Note: prev_error_ is initialized to 0 at reset, so first step uses (e + 0)/2
    integral_ += (dt / (2.0f * Ti_)) * (error + prev_error_);

    return Kp_ * integral_;
}

float PID::updateDerivative(float error, float measurement, float dt)
{
    // Skip derivative if Td <= 0 or dt <= 0
    if (Td_ <= 0.0f || dt <= 0.0f) {
        return 0.0f;
    }

    // Select derivative input based on D-on-M option
    float deriv_input;
    if (derivative_on_measurement_) {
        // D-on-M: derivative of -measurement (to avoid setpoint spikes)
        deriv_input = -measurement;
    } else {
        // D-on-E: derivative of error
        deriv_input = error;
    }

    // First run: initialize previous value, no derivative output
    if (first_run_) {
        prev_deriv_input_ = deriv_input;
        first_run_ = false;
        return 0.0f;
    }

    // Compute derivative filter coefficients for current dt
    // D(z) from bilinear transform of Td·s / (η·Td·s + 1)
    // D[k] = -a * D[k-1] + b * (input[k] - input[k-1])
    float alpha = 2.0f * eta_ * Td_ / dt;
    float deriv_a = (alpha - 1.0f) / (alpha + 1.0f);
    float deriv_b = 2.0f * Td_ / ((alpha + 1.0f) * dt);

    // Incomplete derivative filter (bilinear transform)
    // y[k] = ((α-1)/(α+1))·y[k-1] + (2·Td/(dt·(α+1)))·(x[k] - x[k-1])
    float deriv_diff = deriv_input - prev_deriv_input_;
    deriv_filtered_ = deriv_a * deriv_filtered_ + deriv_b * deriv_diff;

    // Update previous derivative input
    prev_deriv_input_ = deriv_input;

    return Kp_ * deriv_filtered_;
}

void PID::applyAntiWindup(float output_unlimited, float output_limited, float dt)
{
    // Skip if integral is disabled
    if (Ti_ <= 0.0f) {
        return;
    }

    // Back-calculation anti-windup
    // Feedback the saturation amount to the integral
    float saturation = output_limited - output_unlimited;

    // Only apply if there's saturation
    if (saturation != 0.0f) {
        // integral += saturation * (dt / Tt)
        // Note: integral_ is already divided by Ti in the integration,
        // so we adjust accordingly
        integral_ += saturation * (dt / Tt_) / Kp_;
    }
}

void PID::setTi(float ti)
{
    Ti_ = ti;

    // Update tracking time constant
    if (Ti_ > 0.0f && Td_ > 0.0f) {
        Tt_ = std::sqrt(Ti_ * Td_);
    } else if (Ti_ > 0.0f) {
        Tt_ = Ti_;
    }
}

void PID::setTd(float td)
{
    Td_ = td;

    // Update tracking time constant
    if (Ti_ > 0.0f && Td_ > 0.0f) {
        Tt_ = std::sqrt(Ti_ * Td_);
    }
}

}  // namespace stampfly
