/*
 * AUTO-GENERATED -- DO NOT EDIT.
 * Source: control/models/stampfly_physical.yaml
 * Regenerate: sf params generate
 *
 * 自動生成 -- 編集しないこと。
 * 元データ: control/models/stampfly_physical.yaml
 * 再生成: sf params generate
 */
#pragma once

namespace sil_params {

// --- constants (calibration-independent) / 較正非依存の定数 ---
constexpr float MASS = 0.037f;  ///< vehicle mass [kg]
constexpr float IXX = 9.16e-06f;  ///< roll moment of inertia [kg*m^2]
constexpr float IYY = 1.33e-05f;  ///< pitch moment of inertia [kg*m^2]
constexpr float IZZ = 2.04e-05f;  ///< yaw moment of inertia [kg*m^2]
constexpr float ARM_OFFSET = 0.023f;  ///< moment arm [m]

// --- calibration_sets.measured_2026_07 (status: adopted) ---
// Consumed directly by simulator/sil/plant/plant.hpp::Config (backlog #2,
// 2026-07-26 motor ODE): Ct/Cq/Jmp/Dm/Qf/Rm/Km all feed the electromechanical
// ODE dw/dt = [-(Dm+Km^2/Rm)*w - Cq*w^2 - Qf + Km*V/Rm] / Jmp directly (no
// longer 'reference only' -- the static Am/Bm/Cm curve they used to sit
// beside is retired).
namespace adopted {
constexpr float CT = 1e-08f;  ///< N/(rad/s)^2, Config::Ct source (T = Ct*omega^2)
constexpr float CQ = 4.1e-11f;  ///< N*m/(rad/s)^2, Config::Cq source (Q = Cq*omega^2, also the ODE's aero-drag term)
constexpr float JMP = 1.375e-08f;  ///< kg*m^2, Config::Jmp source (ODE rotor inertia)
constexpr float DM = 0.0f;  ///< N*m*s/rad, Config::Dm source (viscous damping, ~0 -- 2026-07-26 coast-down refit)
constexpr float QF = 9.507e-06f;  ///< N*m, Config::Qf source (Coulomb friction torque)
constexpr float RM = 0.593f;  ///< ohm, winding resistance
constexpr float KM = 0.0005682f;  ///< V/(rad/s), back-EMF constant
constexpr float KAPPA = 0.0041f;  ///< m, adopted/rounded (matches firmware actuator.cpp KAPPA)
constexpr float KAPPA_EXACT = 0.0041f;  ///< m, full-precision Cq/Ct (reference only, unused)
}  // namespace adopted

// --- calibration_sets.legacy_motor_curve (status: frozen_for_implementation, backlog #3) ---
// Not consumed by any implementation as of 2026-07-26 -- plant.hpp switched
// its motor model to the electromechanical ODE (backlog #2) and no longer
// references Am/Bm/Cm/motor-curve-Ct at all. The only surviving copy of this
// family's Ct is firmware actuator.cpp's MOTOR_CT, a hand literal (NOT
// sourced from here). Kept for historical reference and for backlog #3
// (firmware-side switch) to consult.
namespace legacy {
constexpr float CT = 1e-08f;
constexpr float CQ = 9.71e-11f;
constexpr float AM = 5.39e-08f;
constexpr float BM = 0.000633f;
constexpr float CM = 0.0153f;
constexpr float RM = 0.34f;
constexpr float KM = 0.0006125f;
constexpr float DM = 3.69e-08f;
constexpr float QF = 2.76e-05f;
}  // namespace legacy

}  // namespace sil_params
