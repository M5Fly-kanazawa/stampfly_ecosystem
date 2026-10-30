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

namespace sils_params {

// --- constants (calibration-independent) / 較正非依存の定数 ---
constexpr float MASS = 0.037f;  ///< vehicle mass [kg]
constexpr float IXX = 9.16e-06f;  ///< roll moment of inertia [kg*m^2]
constexpr float IYY = 1.33e-05f;  ///< pitch moment of inertia [kg*m^2]
constexpr float IZZ = 2.04e-05f;  ///< yaw moment of inertia [kg*m^2]
constexpr float ARM_OFFSET = 0.023f;  ///< moment arm [m]

// --- calibration_sets.bench_2026_10_15 (status: adopted, SILS plant propulsion) ---
// Averaged capacitor-recirculation drive model, consumed by
// simulator/sils/plant/plant.hpp / plant.cpp:
//   I_m = (V - KE*w) / R_out(D)           (I_m = 0 when D < 0.01)
//   J dw/dt = KT*I_m - CQ*w^2 - KT*(I0A + I0B*w^2)
//   T = THRUST_EFFICIENCY * CT * w^2
// 平均化したコンデンサ還流駆動モデル（plant.hpp / plant.cpp が使用）。式は上の英文参照。
// Scalars come first and the R_out arrays last, so the audit regexes that anchor
// on this namespace find each scalar before the first closing brace.
// スカラーを先、R_out 配列を最後に置く（この名前空間に固定した検査用正規表現が
// 最初の閉じ括弧より前でスカラーを見つけられるように）。
namespace plant {
constexpr float KE = 0.0005905f;  ///< back-EMF constant [V*s/rad]
constexpr float KT = 0.0005905f;  ///< torque constant [N*m/A]
constexpr float CQ = 3.38e-11f;  ///< torque coefficient [N*m/(rad/s)^2]
constexpr float CT = 7.6e-09f;  ///< thrust coefficient [N/(rad/s)^2]
constexpr float J = 1.54e-08f;  ///< rotor inertia incl. propeller [kg*m^2]
constexpr float I0A = 0.0443f;  ///< no-load current, constant part [A]
constexpr float I0B = 2.915e-09f;  ///< no-load current, w^2 part [A/(rad/s)^2]
constexpr float C_C = 1e-07f;  ///< recirculation capacitor [F] (informational)
constexpr float F_PWM = 150000.0f;  ///< PWM frequency [Hz] (informational)
constexpr float THRUST_EFFICIENCY = 0.976f;  ///< Config::thrust_efficiency default (eta)

// R_out(D) [ohm], D = 0.00 .. 1.00 in 0.01 steps (ROUT_N entries, index = round(D*100)).
// ROUT_OHM is the ADOPTED table = calculated table x r(D), computed by the generator
// from the measured points (see the YAML). Entry 0 (D = 0.00) is a placeholder equal to
// the D = 0.01 entry and is never used: the plant treats D < 0.01 as 'no drive' (motor
// current 0) explicitly, before looking at the table.
// R_out(D)[Ω]。D=0.00〜1.00、0.01 刻み（ROUT_N 要素、添字=round(D×100)）。ROUT_OHM は
// 採用表（計算表×r(D)、生成器が実測点から計算）。要素 0（D=0.00）は D=0.01 と同値の
// 穴埋めで使われない: プラントは表を見る前に D<0.01 を「駆動なし」（モータ電流 0）と
// 明示的に扱う。
constexpr int ROUT_N = 101;
constexpr float ROUT_D[ROUT_N] = {
    0.0f, 0.01f, 0.02f, 0.03f, 0.04f,
    0.05f, 0.06f, 0.07f, 0.08f, 0.09f,
    0.1f, 0.11f, 0.12f, 0.13f, 0.14f,
    0.15f, 0.16f, 0.17f, 0.18f, 0.19f,
    0.2f, 0.21f, 0.22f, 0.23f, 0.24f,
    0.25f, 0.26f, 0.27f, 0.28f, 0.29f,
    0.3f, 0.31f, 0.32f, 0.33f, 0.34f,
    0.35f, 0.36f, 0.37f, 0.38f, 0.39f,
    0.4f, 0.41f, 0.42f, 0.43f, 0.44f,
    0.45f, 0.46f, 0.47f, 0.48f, 0.49f,
    0.5f, 0.51f, 0.52f, 0.53f, 0.54f,
    0.55f, 0.56f, 0.57f, 0.58f, 0.59f,
    0.6f, 0.61f, 0.62f, 0.63f, 0.64f,
    0.65f, 0.66f, 0.67f, 0.68f, 0.69f,
    0.7f, 0.71f, 0.72f, 0.73f, 0.74f,
    0.75f, 0.76f, 0.77f, 0.78f, 0.79f,
    0.8f, 0.81f, 0.82f, 0.83f, 0.84f,
    0.85f, 0.86f, 0.87f, 0.88f, 0.89f,
    0.9f, 0.91f, 0.92f, 0.93f, 0.94f,
    0.95f, 0.96f, 0.97f, 0.98f, 0.99f,
    1.0f,
};
constexpr float ROUT_OHM[ROUT_N] = {
    65.91173213056307f, 65.91173213056307f, 62.069121050341884f, 56.7674916909661f, 51.05506601978666f,
    45.556044185824724f, 40.56043249887678f, 36.15708797191361f, 32.333110770331594f, 29.032594216852182f,
    26.187f, 23.81460830935815f, 21.754564003620693f, 19.958255812572652f, 18.38470737606574f,
    16.99976719159469f, 15.775101057823576f, 14.687222457342648f, 13.716628790190413f, 12.847080958766051f,
    12.065f, 11.376345259487092f, 10.752071760986238f, 10.184311257075857f, 9.666357388162664f,
    9.19246511579568f, 8.757688503866271f, 8.357747573773151f, 7.988921059612961f, 7.647957355521924f,
    7.332f, 7.006179244014424f, 6.703421102640794f, 6.4214534888468835f, 6.158250986827146f,
    5.91200540038133f, 5.681099568188779f, 5.464083351415048f, 5.259652894141671f, 5.066641946879757f,
    4.884f, 4.711244867145366f, 4.5470560260956425f, 4.390700005900952f, 4.241526178162278f,
    4.098972358927935f, 3.962552816534319f, 3.8318561652720775f, 3.7065410544642257f, 3.5863285482327885f,
    3.4710000000000005f, 3.359842960937141f, 3.2533187385184f, 3.15134587170386f, 3.053874817959868f,
    2.9608791530794973f, 2.8723544789583717f, 2.7883030067758328f, 2.7087291519560477f, 2.6336329228864854f,
    2.563f, 2.4962757756933955f, 2.433948127411795f, 2.3759184481964986f, 2.3220496251631086f,
    2.272158353822165f, 2.226009660812823f, 2.1833092285050832f, 2.143701230491983f, 2.10676176526127f,
    2.072f, 2.02043039600521f, 1.9704272747079605f, 1.9213010230267527f, 1.8723435213722366f,
    1.822853317817622f, 1.7721605113167693f, 1.71966054463968f, 1.6648500807644697f, 1.6073626604130384f,
    1.547f, 1.4943839541127653f, 1.4383612824572924f, 1.3792726106555626f, 1.3176543995771488f,
    1.2542191732785632f, 1.1898085598342658f, 1.1253451840864626f, 1.0617805266136677f, 1.000042745464079f,
    0.941f, 0.8881735837543778f, 0.8391821407000162f, 0.7946221575624145f, 0.7549873119729211f,
    0.7206810811312235f, 0.6920303203649533f, 0.6693007814835199f, 0.6527087515434865f, 0.6424229926427995f,
    0.6385608623323293f,
};
}  // namespace plant

// --- calibration_sets.flight_anchored_motor_curve (status: adopted, role: firmware) ---
// The firmware thrust->duty constants (actuator.cpp MOTOR_CT, KAPPA, MOTOR_AM/BM/CM).
// Not read by the plant; emitted so every copy of them is audited against one set.
// ファームの推力→duty 換算定数。プラントは読まない（コピーを1セットに対して検査するため生成）。
namespace firmware_curve {
constexpr float CT_GAUGE = 1e-08f;  ///< thrust gauge [N/(rad/s)^2]
constexpr float KAPPA = 0.0041f;  ///< torque/thrust ratio [m]
constexpr float AM = 6.0368e-08f;  ///< V = AM*w^2 + BM*w + CM [V/(rad/s)^2]
constexpr float BM = 0.0006699042f;  ///< [V/(rad/s)]
constexpr float CM = 0.0153f;  ///< [V]
}  // namespace firmware_curve

// --- calibration_sets.legacy_motor_curve (status: frozen_for_implementation, history) ---
// Not consumed by any implementation. Kept for historical reference.
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

}  // namespace sils_params
