/**
 * @file motor_model.cpp
 * @brief DC Motor Model Implementation
 */

#include "motor_model.hpp"

namespace stampfly {

// Default motor parameters for StampFly 0720 coreless motors
// StampFly 0720コアレスモータ用デフォルトパラメータ
const MotorParams DEFAULT_MOTOR_PARAMS = {
    .Ct = 1.0e-8f,        // Thrust coefficient [N/(rad/s)²]
    .Cq = 9.71e-11f,      // Torque coefficient [Nm/(rad/s)²]
    .Rm = 0.34f,          // Motor resistance [Ω]
    .Km = 6.125e-4f,      // Motor constant [V·s/rad]
    .Dm = 3.69e-8f,       // Viscous damping [Nm·s/rad]
    .Qf = 2.76e-5f,       // Friction torque [Nm]
    .Jm = 1.0e-9f,        // Motor+propeller inertia [kg·m²]
    .Vbat = 3.7f,         // Battery voltage [V]
};

}  // namespace stampfly
