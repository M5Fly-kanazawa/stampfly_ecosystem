/**
 * @file control_allocation.cpp
 * @brief Control Allocation Implementation
 */

#include "control_allocation.hpp"

namespace stampfly {

// Default quad configuration instance
// デフォルトクワッド設定インスタンス
const QuadConfig DEFAULT_QUAD_CONFIG = {};

// Note: DEFAULT_MOTOR_PARAMS is defined in motor_model.cpp
// 注: DEFAULT_MOTOR_PARAMSはmotor_model.cppで定義

void ControlAllocator::init(const QuadConfig& config)
{
    config_ = config;
    max_thrust_ = config.max_thrust_per_motor;
    buildMatrices();
}

void ControlAllocator::setMotorParams(const MotorParams& params)
{
    motor_params_ = params;
}

void ControlAllocator::buildMatrices()
{
    // Build allocation matrix B: u = B × T
    // Row 0: Total thrust (sum of all thrusts)
    // Row 1: Roll torque = -y × T (moment around X-axis from thrust at y position)
    // Row 2: Pitch torque = +x × T (moment around Y-axis from thrust at x position)
    // Row 3: Yaw torque (reaction torques)

    for (int i = 0; i < 4; i++) {
        B_[0][i] = 1.0f;  // Total thrust
        B_[1][i] = -config_.motor_y[i];  // Roll torque
        B_[2][i] = config_.motor_x[i];   // Pitch torque
        B_[3][i] = -static_cast<float>(config_.motor_dir[i]) * config_.kappa;  // Yaw torque
    }

    // Compute inverse matrix B⁻¹ for symmetric X-quad
    // For symmetric X-quad, the inverse has a simple analytical form:
    //       [1   -1/d   +1/d   +1/κ]
    // B⁻¹ = [1   -1/d   -1/d   -1/κ] × (1/4)
    //       [1   +1/d   -1/d   +1/κ]
    //       [1   +1/d   +1/d   -1/κ]

    const float d = config_.d;
    const float kappa = config_.kappa;
    const float inv_d = 1.0f / d;
    const float inv_kappa = 1.0f / kappa;

    // M1 (FR): +x, +y, CCW (σ=-1)
    B_inv_[0][0] = 0.25f;
    B_inv_[0][1] = -0.25f * inv_d;
    B_inv_[0][2] = 0.25f * inv_d;
    B_inv_[0][3] = 0.25f * inv_kappa;

    // M2 (RR): -x, +y, CW (σ=+1)
    B_inv_[1][0] = 0.25f;
    B_inv_[1][1] = -0.25f * inv_d;
    B_inv_[1][2] = -0.25f * inv_d;
    B_inv_[1][3] = -0.25f * inv_kappa;

    // M3 (RL): -x, -y, CCW (σ=-1)
    B_inv_[2][0] = 0.25f;
    B_inv_[2][1] = 0.25f * inv_d;
    B_inv_[2][2] = -0.25f * inv_d;
    B_inv_[2][3] = 0.25f * inv_kappa;

    // M4 (FL): +x, -y, CW (σ=+1)
    B_inv_[3][0] = 0.25f;
    B_inv_[3][1] = 0.25f * inv_d;
    B_inv_[3][2] = 0.25f * inv_d;
    B_inv_[3][3] = -0.25f * inv_kappa;
}

bool ControlAllocator::mix(const float control[4], float thrusts_out[4]) const
{
    // T = B⁻¹ × u
    // Matrix-vector multiplication
    bool saturated = false;

    for (int i = 0; i < 4; i++) {
        float thrust = 0.0f;
        for (int j = 0; j < 4; j++) {
            thrust += B_inv_[i][j] * control[j];
        }

        // Clamp to valid range
        if (thrust < 0.0f) {
            thrust = 0.0f;
            saturated = true;
        } else if (thrust > max_thrust_) {
            thrust = max_thrust_;
            saturated = true;
        }

        thrusts_out[i] = thrust;
    }

    return saturated;
}

void ControlAllocator::allocate(const float thrusts[4], float control_out[4]) const
{
    // u = B × T
    // Matrix-vector multiplication
    for (int i = 0; i < 4; i++) {
        float u = 0.0f;
        for (int j = 0; j < 4; j++) {
            u += B_[i][j] * thrusts[j];
        }
        control_out[i] = u;
    }
}

float ControlAllocator::thrustToDuty(float thrust) const
{
    // Delegate to motor_model function
    // motor_model関数に委譲
    return stampfly::thrustToDuty(thrust, motor_params_);
}

void ControlAllocator::thrustsToDuties(const float thrusts[4], float duties_out[4]) const
{
    // Delegate to motor_model function
    // motor_model関数に委譲
    stampfly::thrustsToDuties(thrusts, duties_out, motor_params_);
}

}  // namespace stampfly
