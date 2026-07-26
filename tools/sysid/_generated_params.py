"""

AUTO-GENERATED -- DO NOT EDIT.
Source: control/models/stampfly_physical.yaml
Regenerate: sf params generate

自動生成 -- 編集しないこと。
元データ: control/models/stampfly_physical.yaml
再生成: sf params generate
"""

# --- calibration_sets.measured_2026_07 (status: adopted) ---
# tools/sysid/defaults.py's module constants and
# tools/params_audit/params_manifest.py's EXPECTED_* both import from here.
_CT_VALUE = 6.7e-09   # N/(rad/s)^2 -- thrust coeff
_CQ_VALUE = 4.1e-11   # N*m/(rad/s)^2 -- torque coeff
_JMP_VALUE = 1.375e-08   # kg*m^2 -- rotor inertia
_RM_VALUE = 0.593   # ohm -- winding resistance
_KM_VALUE = 0.0005682   # V/(rad/s) -- back-EMF constant
_KAPPA_VALUE = 0.006119402985074627   # m -- kappa = Cq/Ct, full precision (derived)

# Rounded/adopted kappa (3 sig figs), hand-written into firmware's B^-1
# mixer (actuator.cpp KAPPA) and simulator/sil/plant Config::kappa on
# 2026-07-17. Differs from _KAPPA_VALUE (full precision) by ~1e-4 relative --
# both are kept (see control/models/stampfly_physical.yaml kappa_adopted note)
# to avoid changing that existing rounding (behavior neutrality).
KAPPA_ADOPTED = 0.00612

# --- constants (calibration-independent) ---
EXPECTED_MASS = 0.037
EXPECTED_IXX = 9.16e-06
EXPECTED_IYY = 1.33e-05
EXPECTED_IZZ = 2.04e-05
EXPECTED_ARM = 0.023
EXPECTED_URDF_BASE_MASS = 0.033

# --- cross-file comparison values (tools/params_audit/params_manifest.py) ---
EXPECTED_CT = _CT_VALUE
EXPECTED_CQ = _CQ_VALUE
EXPECTED_JMP = _JMP_VALUE
EXPECTED_RM = _RM_VALUE
EXPECTED_KAPPA = KAPPA_ADOPTED
