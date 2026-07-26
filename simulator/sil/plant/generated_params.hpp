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
namespace adopted {
constexpr float CT = 6.7e-09f;  ///< N/(rad/s)^2 (reference only -- plant.hpp Config::Ct keeps the legacy value, EXEMPT pending backlog #3)
constexpr float CQ = 4.1e-11f;  ///< N*m/(rad/s)^2 (reference only)
constexpr float JMP = 1.375e-08f;  ///< kg*m^2 (reference only)
constexpr float RM = 0.593f;  ///< ohm, winding resistance
constexpr float KM = 0.0005682f;  ///< V/(rad/s), back-EMF constant
constexpr float KAPPA = 0.00612f;  ///< m, adopted/rounded (matches firmware actuator.cpp KAPPA)
constexpr float KAPPA_EXACT = 0.006119402985074627f;  ///< m, full-precision Cq/Ct (reference only, unused)
}  // namespace adopted

// --- calibration_sets.legacy_motor_curve (status: frozen_for_implementation, backlog #3) ---
// Not yet wired into plant.hpp (kept literal there pending backlog #3);
// exposed here for that future recalibration to consume.
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
