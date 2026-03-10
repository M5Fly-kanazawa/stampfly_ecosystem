/**
 * @file vehicle_config.hpp
 * @brief Vehicle application-specific configuration
 *        Vehicle アプリケーション固有の設定
 *
 * ESKF tuning, control gains, safety parameters, and other
 * flight-behavior settings specific to the vehicle application.
 * Custom applications may define their own version of this file.
 *
 * ESKF チューニング、制御ゲイン、安全パラメータなど
 * vehicle アプリ固有のフライト動作設定。
 * カスタムアプリは独自バージョンを定義可能。
 */

#pragma once

namespace config {

// =============================================================================
// ESKF (Error-State Kalman Filter) Configuration
// =============================================================================
//
// センサーフュージョン（状態推定）の全設定
// 学習者へ: ここで全てのESKFパラメータを確認・変更できます
//

namespace eskf {

// -----------------------------------------------------------------------------
// センサー有効/無効スイッチ
// デフォルト: 全センサーON。デバッグ時に個別無効化可能
// -----------------------------------------------------------------------------
inline constexpr bool USE_OPTICAL_FLOW = false;    // オプティカルフロー ※姿勢ジャンプ原因切分け中
inline constexpr bool USE_BAROMETER = false;       // 気圧センサー ※姿勢ジャンプ原因切分け中
inline constexpr bool USE_TOF = false;             // ToFセンサー ※姿勢ジャンプ原因切分け中
inline constexpr bool USE_MAGNETOMETER = false;    // 地磁気観測更新（ヨー補正）※テスト用に無効化
inline constexpr bool ENABLE_YAW_ESTIMATION = true; // ヨー推定（ジャイロ積分）※falseで0固定

// -----------------------------------------------------------------------------
// プロセスノイズ (Q行列)
// 値が大きい = センサーを信頼、値が小さい = モデルを信頼
// -----------------------------------------------------------------------------
inline constexpr float GYRO_NOISE = 0.009655f;         // ジャイロノイズ [rad/s/√Hz]
inline constexpr float ACCEL_NOISE = 0.062885f;        // 加速度ノイズ [m/s²/√Hz]
inline constexpr float GYRO_BIAS_NOISE = 0.000013f;    // ジャイロバイアスランダムウォーク
inline constexpr float ACCEL_BIAS_NOISE = 0.001f;      // 加速度バイアスランダムウォーク [m/s²/√s]

// -----------------------------------------------------------------------------
// 観測ノイズ (R行列)
// 値が大きい = 観測を信頼しない、値が小さい = 観測を信頼
// -----------------------------------------------------------------------------
inline constexpr float BARO_NOISE = 0.1f;              // 気圧高度ノイズ [m]
inline constexpr float TOF_NOISE = 0.03f;              // ToFノイズ [m] (VL53L3CX sigma ~20-30mm)
inline constexpr float MAG_NOISE = 1.0f;               // 地磁気ノイズ [uT] 実測std≈0.94
inline constexpr float FLOW_NOISE = 0.01f;             // オプティカルフローノイズ [m/s] 実測std≈0.011
inline constexpr float ACCEL_ATT_NOISE = 0.06f;        // 加速度計姿勢補正ノイズ [m/s²] (初期値: 0.02)

// -----------------------------------------------------------------------------
// 初期共分散 (P行列の初期値)
// 離陸時は位置・速度が既知なので小さめに設定
// -----------------------------------------------------------------------------
inline constexpr float INIT_POS_STD = 0.1f;            // 位置 [m] (10cm)
inline constexpr float INIT_VEL_STD = 0.1f;            // 速度 [m/s] (10cm/s)
inline constexpr float INIT_ATT_STD = 0.1f;            // 姿勢 [rad]
inline constexpr float INIT_GYRO_BIAS_STD = 0.01f;     // ジャイロバイアス [rad/s]
inline constexpr float INIT_ACCEL_BIAS_STD = 0.1f;     // 加速度バイアス [m/s²]

// -----------------------------------------------------------------------------
// 物理定数
// -----------------------------------------------------------------------------
inline constexpr float GRAVITY = 9.81f;                // 重力加速度 [m/s²]

// 地磁気参照ベクトル (NED座標系) - 日本近辺の概算値
inline constexpr float MAG_REF_X = 20.0f;              // 北成分 [uT]
inline constexpr float MAG_REF_Y = 0.0f;               // 東成分 [uT]
inline constexpr float MAG_REF_Z = 40.0f;              // 下成分 [uT]

// -----------------------------------------------------------------------------
// 閾値設定
// -----------------------------------------------------------------------------
inline constexpr float MAHALANOBIS_THRESHOLD = 15.0f;  // アウトライア棄却閾値
inline constexpr float TOF_TILT_THRESHOLD = 0.70f;     // ToF傾き閾値 [rad] (~40度)
inline constexpr float TOF_CHI2_GATE = 3.84f;          // ToFχ²ゲート閾値 [χ²(1,0.95)=3.84, 0=無効]
inline constexpr float ACCEL_MOTION_THRESHOLD = 1.0f;  // 加速度モーション閾値 [m/s²]

// 発散検出閾値
inline constexpr float MAX_POSITION = 100.0f;          // [m]
inline constexpr float MAX_VELOCITY = 50.0f;           // [m/s]

// -----------------------------------------------------------------------------
// オプティカルフロー設定
// -----------------------------------------------------------------------------
inline constexpr float FLOW_MIN_HEIGHT = 0.02f;        // 最小高度 [m]
inline constexpr float FLOW_MAX_HEIGHT = 4.0f;         // 最大高度 [m]
inline constexpr float FLOW_TILT_COS_THRESHOLD = 0.866f; // 傾き閾値 cos(30°)

// PMW3901キャリブレーション
inline constexpr float FLOW_RAD_PER_PIXEL = 0.00222f;  // [rad/pixel]

// カメラ→機体座標変換行列 (2x2)
inline constexpr float FLOW_CAM_TO_BODY_XX = 0.943f;   // X軸スケール
inline constexpr float FLOW_CAM_TO_BODY_XY = 0.0f;
inline constexpr float FLOW_CAM_TO_BODY_YX = 0.0f;
inline constexpr float FLOW_CAM_TO_BODY_YY = 1.015f;   // Y軸スケール

// ジャイロ回転補償スケール
inline constexpr float FLOW_GYRO_SCALE = 1.0f;

// フローオフセット [counts/sample] (キャリブレーション後に設定)
inline constexpr float FLOW_OFFSET_X = 0.0f;
inline constexpr float FLOW_OFFSET_Y = 0.0f;

// -----------------------------------------------------------------------------
// 姿勢補正: 適応的Rスケーリング
// R = R_base × (1 + K_ADAPTIVE × |accel_norm - g|²)
// 動的加速度が大きいほど加速度計の補正を弱める（ゼロにはしない）
// K=0: 常に同じ重みで補正、K=10: 1m/s²偏差で補正が約1/11に低減
// -----------------------------------------------------------------------------
inline constexpr int ATT_UPDATE_MODE = 0;
inline constexpr float K_ADAPTIVE = 10.0f;
inline constexpr float GYRO_ATT_THRESHOLD = 0.5f;      // [rad/s]

// -----------------------------------------------------------------------------
// 着陸検出・位置リセット設定
// -----------------------------------------------------------------------------
inline constexpr bool ENABLE_LANDING_RESET = true;      // 着陸時に位置リセット
inline constexpr float LANDING_ALT_THRESHOLD = 0.05f;   // 着陸判定高度閾値 [m]
inline constexpr float LANDING_VEL_THRESHOLD = 0.1f;    // 着陸判定速度閾値 [m/s]
inline constexpr int LANDING_HOLD_COUNT = 200;          // 着陸判定維持回数 (200回 @ 400Hz = 0.5秒)

} // namespace eskf

// =============================================================================
// Sensor Stability Thresholds (センサー安定判定閾値)
// =============================================================================
//
// ESKF初期化前にセンサーの安定を確認するための閾値
// 実測データ (2025-01-02) に基づいて設定
//

namespace stability {

// 安定判定の標準偏差閾値（std norm）×2.0
inline constexpr float ACCEL_STD_THRESHOLD = 0.05f;      // [m/s²] (初期値×2.0: 0.025×2.0)
inline constexpr float GYRO_STD_THRESHOLD = 0.01f;       // [rad/s] (初期値×2.0: 0.005×2.0)
inline constexpr float MAG_STD_THRESHOLD = 2.6f;         // [µT] (初期値×2.0: 1.3×2.0)
inline constexpr float BARO_STD_THRESHOLD = 0.40f;       // [m] (初期値×2.0: 0.20×2.0)
inline constexpr float TOF_STD_THRESHOLD = 0.006f;       // [m] (初期値×2.0: 0.003×2.0)
inline constexpr float OPTFLOW_STD_THRESHOLD = 6.0f;     // [counts] (初期値×2.0: 3.0×2.0)

// 安定判定のタイミング
inline constexpr int CHECK_INTERVAL_MS = 200;            // チェック間隔 [ms]
inline constexpr int MAX_WAIT_MS = 10000;                // 最大待機時間 [ms]

// バッファ最小サンプル数（統計計算に必要）
inline constexpr int MIN_ACCEL_SAMPLES = 50;
inline constexpr int MIN_GYRO_SAMPLES = 50;
inline constexpr int MIN_MAG_SAMPLES = 50;        // 25Hz × 50 = 2000ms
inline constexpr int MIN_BARO_SAMPLES = 20;      // 50Hz × 20 = 400ms
inline constexpr int MIN_TOF_SAMPLES = 20;       // 30Hz × 20 = 667ms
inline constexpr int MIN_OPTFLOW_SAMPLES = 50;   // 100Hz × 50 = 500ms

} // namespace stability

// =============================================================================
// Simple Estimators (ESKF不要時のバックアップ推定器)
// =============================================================================

namespace attitude_estimator {
inline constexpr float GYRO_WEIGHT = 0.98f;        // 相補フィルタのジャイロ重み
inline constexpr float MAG_DECLINATION = 0.0f;     // 地磁気偏角 [rad]
} // namespace attitude_estimator

namespace altitude_estimator {
inline constexpr float PROCESS_NOISE_ALT = 0.01f;       // 高度プロセスノイズ
inline constexpr float PROCESS_NOISE_VEL = 0.1f;        // 速度プロセスノイズ
inline constexpr float MEASUREMENT_NOISE_BARO = 1.0f;   // 気圧観測ノイズ
inline constexpr float MEASUREMENT_NOISE_TOF = 0.05f;   // ToF観測ノイズ
} // namespace altitude_estimator

// =============================================================================
// Rate Control (角速度制御) Configuration
// =============================================================================
//
// コントローラ入力 → 目標角速度 → PID制御 → モーターミキサー
//

namespace rate_control {

// -----------------------------------------------------------------------------
// 物理単位モード切り替え (Physical Units Mode Switch)
// -----------------------------------------------------------------------------
#define USE_PHYSICAL_UNITS 1  // Physical units mode with corrected k_τ

// C++コードからアクセス可能なフラグ
inline constexpr bool PHYSICAL_UNITS_ENABLED = (USE_PHYSICAL_UNITS == 1);

// -----------------------------------------------------------------------------
// 感度設定 (Sensitivity)
// スティック入力 ±1.0 に対する最大目標角速度 [rad/s]
// -----------------------------------------------------------------------------
inline constexpr float ROLL_RATE_MAX = 1.0f;       // ロール最大角速度 [rad/s] (~57 deg/s)
inline constexpr float PITCH_RATE_MAX = 1.0f;      // ピッチ最大角速度 [rad/s] (~57 deg/s)
inline constexpr float YAW_RATE_MAX = 5.0f;        // ヨー最大角速度 [rad/s] (~286 deg/s)

// -----------------------------------------------------------------------------
// PIDゲイン (Rate Controller)
// -----------------------------------------------------------------------------

#if USE_PHYSICAL_UNITS
// ==========================================================================
// 物理単位ベースゲイン (Physical Units: Torque [Nm])
// ==========================================================================

// Roll rate PID
inline constexpr float ROLL_RATE_KP = 9.1e-4f;     // [Nm/(rad/s)] = 0.65 × 1.4e-3
inline constexpr float ROLL_RATE_TI = 0.7f;        // 積分時間 [s]
inline constexpr float ROLL_RATE_TD = 0.01f;       // 微分時間 [s]

// Pitch rate PID
inline constexpr float PITCH_RATE_KP = 1.33e-3f;   // [Nm/(rad/s)] = 0.95 × 1.4e-3
inline constexpr float PITCH_RATE_TI = 0.7f;       // 積分時間 [s]
inline constexpr float PITCH_RATE_TD = 0.01f;      // 微分時間 [s]

// Yaw rate PID
inline constexpr float YAW_RATE_KP = 1.77e-3f;     // [Nm/(rad/s)] = 3.0 × 5.9e-4
inline constexpr float YAW_RATE_TI = 0.8f;         // 積分時間 [s]
inline constexpr float YAW_RATE_TD = 0.01f;        // 微分時間 [s]

// 出力制限 [Nm]
inline constexpr float ROLL_OUTPUT_LIMIT = 5.2e-3f;   // 3.7 × 1.4e-3
inline constexpr float PITCH_OUTPUT_LIMIT = 5.2e-3f;  // 3.7 × 1.4e-3
inline constexpr float YAW_OUTPUT_LIMIT = 2.2e-3f;    // 3.7 × 5.9e-4

// 共通出力制限（最大値、後方互換性）
inline constexpr float OUTPUT_LIMIT = 5.2e-3f;     // [Nm] (ロール/ピッチ基準)

#else
// ==========================================================================
// 電圧スケールゲイン (Legacy: Voltage [V])
// ==========================================================================

// Roll rate PID
inline constexpr float ROLL_RATE_KP = 0.65f;       // 比例ゲイン
inline constexpr float ROLL_RATE_TI = 0.7f;        // 積分時間 [s]
inline constexpr float ROLL_RATE_TD = 0.01f;       // 微分時間 [s]

// Pitch rate PID
inline constexpr float PITCH_RATE_KP = 0.95f;      // 比例ゲイン
inline constexpr float PITCH_RATE_TI = 0.7f;       // 積分時間 [s]
inline constexpr float PITCH_RATE_TD = 0.025f;     // 微分時間 [s]

// Yaw rate PID
inline constexpr float YAW_RATE_KP = 3.0f;         // 比例ゲイン
inline constexpr float YAW_RATE_TI = 0.8f;         // 積分時間 [s]
inline constexpr float YAW_RATE_TD = 0.01f;        // 微分時間 [s]

// 出力制限 [V]
inline constexpr float ROLL_OUTPUT_LIMIT = 3.7f;
inline constexpr float PITCH_OUTPUT_LIMIT = 3.7f;
inline constexpr float YAW_OUTPUT_LIMIT = 3.7f;
inline constexpr float OUTPUT_LIMIT = 3.7f;        // [V] (電圧スケール)

#endif

// 共通パラメータ（モード非依存）
inline constexpr float PID_ETA = 0.125f;           // 不完全微分フィルタ係数 (0.1~0.2)

} // namespace rate_control

// =============================================================================
// Attitude Control Parameters - 姿勢制御パラメータ（STABILIZE Mode）
// =============================================================================

namespace attitude_control {

// 最大傾斜角
inline constexpr float MAX_ROLL_ANGLE = 0.5236f;   // 30 deg = π/6 rad
inline constexpr float MAX_PITCH_ANGLE = 0.5236f;  // 30 deg = π/6 rad

// 姿勢PIDゲイン（外側ループ）
inline constexpr float ROLL_ANGLE_KP = 5.0f;    // 比例ゲイン [(rad/s) / rad]
inline constexpr float ROLL_ANGLE_TI = 4.0f;    // 積分時間 [s]
inline constexpr float ROLL_ANGLE_TD = 0.04f;   // 微分時間 [s]

inline constexpr float PITCH_ANGLE_KP = 5.0f;   // 比例ゲイン [(rad/s) / rad]
inline constexpr float PITCH_ANGLE_TI = 4.0f;   // 積分時間 [s]
inline constexpr float PITCH_ANGLE_TD = 0.04f;  // 微分時間 [s]

// 出力制限
inline constexpr float MAX_RATE_SETPOINT = 3.0f;  // [rad/s] (~172 deg/s)

// 共通パラメータ
inline constexpr float PID_ETA = 0.125f;  // 不完全微分フィルタ係数

} // namespace attitude_control

// =============================================================================
// Altitude Control Parameters - 高度制御パラメータ（ALTITUDE_HOLD Mode）
// =============================================================================

namespace altitude_control {

// 機体パラメータ
inline constexpr float MASS = 0.035f;                    // [kg]
inline constexpr float GRAVITY = 9.81f;                  // [m/s²]
inline constexpr float MAX_TOTAL_THRUST = 4.0f * 0.15f;  // 0.60N (4 motors)

// ホバー推力推定のデフォルトパラメータ
inline constexpr float HOVER_THRUST_CORRECTION = 1.0f;   // Empirical correction factor

// 高度PID（外ループ）
inline constexpr float ALT_KP = 2.0f;
inline constexpr float ALT_TI = 3.0f;
inline constexpr float ALT_TD = 0.1f;
inline constexpr float ALT_OUTPUT_MAX = 1.0f;  // Max climb/descent rate [m/s]

// 速度PID（内ループ）
inline constexpr float VEL_KP = 0.3f;
inline constexpr float VEL_TI = 1.0f;
inline constexpr float VEL_TD = 0.02f;
inline constexpr float VEL_OUTPUT_MAX = 0.2f;  // Max thrust correction [N]

// スティック設定
inline constexpr float STICK_DEADZONE = 0.1f;
inline constexpr float MAX_CLIMB_RATE = 0.5f;         // [m/s]
inline constexpr float MAX_DESCENT_RATE = 0.3f;       // [m/s]

// 高度制限
inline constexpr float MIN_ALTITUDE = 0.10f;   // [m]
inline constexpr float MAX_ALTITUDE = 3.0f;    // [m]

// 共通
inline constexpr float PID_ETA = 0.125f;

} // namespace altitude_control

// =============================================================================
// Position Control Parameters - 位置制御パラメータ（POSITION_HOLD Mode）
// =============================================================================

namespace position_control {

// 位置PID（外ループ）
inline constexpr float POS_KP = 1.0f;
inline constexpr float POS_TI = 5.0f;
inline constexpr float POS_TD = 0.1f;
inline constexpr float POS_OUTPUT_MAX = 0.5f;  // Max horizontal velocity target [m/s]

// 速度PID（内ループ）
inline constexpr float VEL_KP = 0.3f;
inline constexpr float VEL_TI = 2.0f;
inline constexpr float VEL_TD = 0.02f;
inline constexpr float VEL_OUTPUT_MAX = 0.20f; // Max angle correction ~11.5°

// スティック設定
inline constexpr float STICK_DEADZONE = 0.15f;
inline constexpr float MAX_HORIZONTAL_SPEED = 0.5f; // [m/s]

// 位置制限
inline constexpr float MAX_POSITION_OFFSET = 5.0f;  // [m]

// 安全制限
inline constexpr float MAX_TILT_ANGLE = 0.1745f;    // ~10°

// 共通
inline constexpr float PID_ETA = 0.125f;

} // namespace position_control

// =============================================================================
// Safety Parameters - 安全機能パラメータ
// =============================================================================
namespace safety {

// 衝撃検出（自動Disarm）- 加速度ベース
inline constexpr float IMPACT_ACCEL_THRESHOLD_G = 3.0f;    // 加速度閾値 [G]
inline constexpr float IMPACT_ACCEL_THRESHOLD_MS2 = IMPACT_ACCEL_THRESHOLD_G * 9.81f;  // [m/s^2]

// 異常角速度検出（自動Disarm）- ジャイロベース
inline constexpr float IMPACT_GYRO_THRESHOLD_DPS = 800.0f;  // 角速度閾値 [deg/s]
inline constexpr float IMPACT_GYRO_THRESHOLD_RPS = IMPACT_GYRO_THRESHOLD_DPS * 3.14159f / 180.0f;  // [rad/s]

// 共通パラメータ
inline constexpr int IMPACT_COUNT_THRESHOLD = 2;       // 連続検出回数（誤検出防止）

} // namespace safety

// =============================================================================
// Trim Settings - トリム調整
// =============================================================================

namespace trim {

// デフォルト値（初期値 = 0、トリムなし）
inline constexpr float DEFAULT_ROLL = 0.0f;        // ロールトリム [-1.0 ~ +1.0]
inline constexpr float DEFAULT_PITCH = 0.0f;       // ピッチトリム [-1.0 ~ +1.0]
inline constexpr float DEFAULT_YAW = 0.0f;         // ヨートリム [-1.0 ~ +1.0]

// トリム値の最大範囲
inline constexpr float MAX_TRIM = 0.2f;            // 最大±20% (±0.2)

} // namespace trim

} // namespace config
