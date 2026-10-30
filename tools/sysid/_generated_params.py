"""

AUTO-GENERATED -- DO NOT EDIT.
Source: control/models/stampfly_physical.yaml
Regenerate: sf params generate

自動生成 -- 編集しないこと。
元データ: control/models/stampfly_physical.yaml
再生成: sf params generate
"""

# =====================================================================
# Which constant set? / どの較正セットか
#   EXPECTED_FW_*       firmware thrust->duty constants (flight_anchored_motor_curve,
#                       what actuator.cpp holds) -- use when a tool INVERTS firmware duty.
#   EXPECTED_PLANT_*    SILS plant propulsion (bench_2026_10_15) -- use when a tool
#                       MODELS the plant physics the way simulator/sils/plant does.
#   EXPECTED_MEASURED_* superseded measured_2026_07 (own D*V drive model: Genesis,
#                       VPython, loop_shaping_tool, sysid defaults, docs hand tables).
#   EXPECTED_LEGACY_*   legacy_motor_curve (docs history table).
#   EXPECTED_FW_* は actuator.cpp が持つ換算定数、EXPECTED_PLANT_* は SILS プラントの
#   推進系、EXPECTED_MEASURED_* は D×V 駆動のまま移行待ちの置き換え済みセット。
# =====================================================================

# --- calibration_sets.measured_2026_07 (status: superseded 2026-10-30) ---
# tools/sysid/defaults.py's module constants import these underscore names.
# The SILS plant no longer reads this set; Genesis/VPython/loop_shaping_tool/
# sysid defaults still use it (own D*V drive model, migration pending).
_CT_VALUE = 1e-08   # N/(rad/s)^2 -- thrust coeff
_CQ_VALUE = 4.1e-11   # N*m/(rad/s)^2 -- torque coeff
_JMP_VALUE = 1.375e-08   # kg*m^2 -- rotor inertia
_DM_VALUE = 0.0   # N*m*s/rad -- viscous damping (2026-07-26 coast-down 3-term refit, b term ~0)
_QF_VALUE = 9.507e-06   # N*m -- Coulomb friction torque (2026-07-26 coast-down 3-term refit, c term x Jmp)
_RM_VALUE = 0.593   # ohm -- winding resistance
_KM_VALUE = 0.0005682   # V/(rad/s) -- back-EMF constant
_KAPPA_VALUE = 0.0041   # m -- kappa = Cq/Ct, full precision (derived)

# --- firmware thrust->duty constants (flight_anchored_motor_curve, adopted) ---
# What firmware/vehicle/components/sf_actuator/actuator.cpp holds (MOTOR_CT,
# KAPPA, MOTOR_AM/BM/CM). tools/sysid/plant_fit.py inverts exactly this curve.
EXPECTED_FW_CT = 1e-08   # N/(rad/s)^2 -- thrust gauge (MOTOR_CT)
EXPECTED_FW_KAPPA = 0.0041   # m -- torque/thrust ratio (KAPPA)
EXPECTED_FW_AM = 6.0368e-08   # V/(rad/s)^2 (MOTOR_AM)
EXPECTED_FW_BM = 0.0006699042   # V/(rad/s) (MOTOR_BM)
EXPECTED_FW_CM = 0.0153   # V (MOTOR_CM)
# Mixer kappa under its historical name (simulator/tests/test_control_allocation_
# firmware_compat.py imports it): the firmware kappa above.
KAPPA_ADOPTED = EXPECTED_FW_KAPPA

# --- SILS plant propulsion (bench_2026_10_15, adopted) ---
# I_m = (V - Ke*w)/R_out(D); J dw/dt = Kt*I_m - Cq*w^2 - Kt*(I0a + I0b*w^2); T = eta*Ct*w^2
EXPECTED_PLANT_KE = 0.0005905   # V*s/rad
EXPECTED_PLANT_KT = 0.0005905   # N*m/A
EXPECTED_PLANT_CQ = 3.38e-11   # N*m/(rad/s)^2
EXPECTED_PLANT_CT = 7.6e-09   # N/(rad/s)^2
EXPECTED_PLANT_J = 1.54e-08   # kg*m^2
EXPECTED_PLANT_I0A = 0.0443   # A
EXPECTED_PLANT_I0B = 2.915e-09   # A/(rad/s)^2
EXPECTED_PLANT_C_C = 1e-07   # F (informational)
EXPECTED_PLANT_F_PWM = 150000.0   # Hz (informational)
EXPECTED_PLANT_THRUST_EFFICIENCY = 0.976

# R_out(D) [ohm], 101 entries for D = 0.00 .. 1.00 (index = round(D*100)).
# Entry 0 is a placeholder (D < 0.01 means no drive); see generate.py.
PLANT_ROUT_D = (
    0.0, 0.01, 0.02, 0.03, 0.04,
    0.05, 0.06, 0.07, 0.08, 0.09,
    0.1, 0.11, 0.12, 0.13, 0.14,
    0.15, 0.16, 0.17, 0.18, 0.19,
    0.2, 0.21, 0.22, 0.23, 0.24,
    0.25, 0.26, 0.27, 0.28, 0.29,
    0.3, 0.31, 0.32, 0.33, 0.34,
    0.35, 0.36, 0.37, 0.38, 0.39,
    0.4, 0.41, 0.42, 0.43, 0.44,
    0.45, 0.46, 0.47, 0.48, 0.49,
    0.5, 0.51, 0.52, 0.53, 0.54,
    0.55, 0.56, 0.57, 0.58, 0.59,
    0.6, 0.61, 0.62, 0.63, 0.64,
    0.65, 0.66, 0.67, 0.68, 0.69,
    0.7, 0.71, 0.72, 0.73, 0.74,
    0.75, 0.76, 0.77, 0.78, 0.79,
    0.8, 0.81, 0.82, 0.83, 0.84,
    0.85, 0.86, 0.87, 0.88, 0.89,
    0.9, 0.91, 0.92, 0.93, 0.94,
    0.95, 0.96, 0.97, 0.98, 0.99,
    1.0,
)
PLANT_ROUT_OHM = (
    65.91173213056307, 65.91173213056307, 62.069121050341884, 56.7674916909661, 51.05506601978666,
    45.556044185824724, 40.56043249887678, 36.15708797191361, 32.333110770331594, 29.032594216852182,
    26.187, 23.81460830935815, 21.754564003620693, 19.958255812572652, 18.38470737606574,
    16.99976719159469, 15.775101057823576, 14.687222457342648, 13.716628790190413, 12.847080958766051,
    12.065, 11.376345259487092, 10.752071760986238, 10.184311257075857, 9.666357388162664,
    9.19246511579568, 8.757688503866271, 8.357747573773151, 7.988921059612961, 7.647957355521924,
    7.332, 7.006179244014424, 6.703421102640794, 6.4214534888468835, 6.158250986827146,
    5.91200540038133, 5.681099568188779, 5.464083351415048, 5.259652894141671, 5.066641946879757,
    4.884, 4.711244867145366, 4.5470560260956425, 4.390700005900952, 4.241526178162278,
    4.098972358927935, 3.962552816534319, 3.8318561652720775, 3.7065410544642257, 3.5863285482327885,
    3.4710000000000005, 3.359842960937141, 3.2533187385184, 3.15134587170386, 3.053874817959868,
    2.9608791530794973, 2.8723544789583717, 2.7883030067758328, 2.7087291519560477, 2.6336329228864854,
    2.563, 2.4962757756933955, 2.433948127411795, 2.3759184481964986, 2.3220496251631086,
    2.272158353822165, 2.226009660812823, 2.1833092285050832, 2.143701230491983, 2.10676176526127,
    2.072, 2.02043039600521, 1.9704272747079605, 1.9213010230267527, 1.8723435213722366,
    1.822853317817622, 1.7721605113167693, 1.71966054463968, 1.6648500807644697, 1.6073626604130384,
    1.547, 1.4943839541127653, 1.4383612824572924, 1.3792726106555626, 1.3176543995771488,
    1.2542191732785632, 1.1898085598342658, 1.1253451840864626, 1.0617805266136677, 1.000042745464079,
    0.941, 0.8881735837543778, 0.8391821407000162, 0.7946221575624145, 0.7549873119729211,
    0.7206810811312235, 0.6920303203649533, 0.6693007814835199, 0.6527087515434865, 0.6424229926427995,
    0.6385608623323293,
)

# --- legacy_motor_curve (history; docs table only) ---
EXPECTED_LEGACY_AM = 5.39e-08   # V/(rad/s)^2
EXPECTED_LEGACY_BM = 0.000633   # V/(rad/s)
EXPECTED_LEGACY_CM = 0.0153   # V

# --- constants (calibration-independent) ---
EXPECTED_MASS = 0.037
EXPECTED_IXX = 9.16e-06
EXPECTED_IYY = 1.33e-05
EXPECTED_IZZ = 2.04e-05
EXPECTED_ARM = 0.023
EXPECTED_URDF_BASE_MASS = 0.033

# --- superseded measured_2026_07 comparison values (tools/params_audit) ---
# Used for the D*V-drive copies that have not migrated yet (Genesis, VPython,
# loop_shaping_tool, sysid defaults, docs hand tables).
EXPECTED_MEASURED_CT = _CT_VALUE
EXPECTED_MEASURED_CQ = _CQ_VALUE
EXPECTED_MEASURED_JMP = _JMP_VALUE
EXPECTED_MEASURED_DM = _DM_VALUE
EXPECTED_MEASURED_QF = _QF_VALUE
EXPECTED_MEASURED_RM = _RM_VALUE
EXPECTED_MEASURED_KM = _KM_VALUE
EXPECTED_MEASURED_KAPPA = 0.0041   # kappa_adopted (docs table kappa)
