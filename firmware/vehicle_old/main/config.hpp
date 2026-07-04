/**
 * @file config.hpp
 * @brief ハードウェア設定とタスク設定
 *
 * GPIO定義、タスク優先度、スタックサイズなどの定数を集約
 */

#pragma once

#include "freertos/FreeRTOS.h"

namespace config {

// =============================================================================
// GPIO Definitions
// =============================================================================

// SPI Bus
inline constexpr int GPIO_SPI_MOSI = 14;
inline constexpr int GPIO_SPI_MISO = 43;
inline constexpr int GPIO_SPI_SCK = 44;
inline constexpr int GPIO_IMU_CS = 46;
inline constexpr int GPIO_FLOW_CS = 12;

// I2C Bus
inline constexpr int GPIO_I2C_SDA = 3;
inline constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT
inline constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
inline constexpr int GPIO_TOF_XSHUT_FRONT = 9;

// Motors (LEDC PWM)
inline constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
inline constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
inline constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
inline constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
inline constexpr int GPIO_LED_MCU = 21;    // M5Stamp S3 内蔵LED
inline constexpr int GPIO_LED_BODY = 39;   // StampFly ボード上LED（2個直列）
inline constexpr int GPIO_LED = 39;        // 後方互換性のため残す（deprecated）
inline constexpr int GPIO_BUZZER = 40;
inline constexpr int GPIO_BUTTON = 0;

// =============================================================================
// Task Priorities
// =============================================================================

inline constexpr UBaseType_t PRIORITY_IMU_TASK = 24;
inline constexpr UBaseType_t PRIORITY_CONTROL_TASK = 23;
inline constexpr UBaseType_t PRIORITY_OPTFLOW_TASK = 20;
inline constexpr UBaseType_t PRIORITY_MAG_TASK = 18;
inline constexpr UBaseType_t PRIORITY_BARO_TASK = 16;
inline constexpr UBaseType_t PRIORITY_COMM_TASK = 15;
inline constexpr UBaseType_t PRIORITY_TOF_TASK = 14;
inline constexpr UBaseType_t PRIORITY_TELEMETRY_TASK = 13;
inline constexpr UBaseType_t PRIORITY_POWER_TASK = 12;
inline constexpr UBaseType_t PRIORITY_BUTTON_TASK = 10;
inline constexpr UBaseType_t PRIORITY_LED_TASK = 8;
inline constexpr UBaseType_t PRIORITY_CLI_TASK = 5;

// =============================================================================
// Task Stack Sizes
// =============================================================================

inline constexpr uint32_t STACK_SIZE_IMU = 16384;      // ESKF行列演算用
inline constexpr uint32_t STACK_SIZE_CONTROL = 8192;
inline constexpr uint32_t STACK_SIZE_OPTFLOW = 8192;
inline constexpr uint32_t STACK_SIZE_MAG = 8192;
inline constexpr uint32_t STACK_SIZE_BARO = 8192;
inline constexpr uint32_t STACK_SIZE_TOF = 8192;
inline constexpr uint32_t STACK_SIZE_POWER = 4096;
inline constexpr uint32_t STACK_SIZE_LED = 4096;
inline constexpr uint32_t STACK_SIZE_BUTTON = 4096;
inline constexpr uint32_t STACK_SIZE_COMM = 4096;
inline constexpr uint32_t STACK_SIZE_CLI = 8192;  // Increased for WiFi command output
inline constexpr uint32_t STACK_SIZE_TELEMETRY = 4096;

// =============================================================================
// Timing Constants
// =============================================================================

inline constexpr float IMU_DT = 0.0025f;          // 400Hz
inline constexpr float OPTFLOW_DT = 0.01f;        // 100Hz
inline constexpr float MAG_DT = 0.04f;            // 25Hz
inline constexpr float BARO_DT = 0.02f;           // 50Hz
inline constexpr float TOF_DT = 0.033f;           // 30Hz

// =============================================================================
// Sensor Thresholds
// =============================================================================

// Optical Flow
inline constexpr uint8_t FLOW_SQUAL_MIN = 0x19;   // 最小品質閾値
inline constexpr float FLOW_DISTANCE_MIN = 0.02f; // [m]
inline constexpr float FLOW_DISTANCE_MAX = 4.0f;  // [m]

// ToF
inline constexpr float TOF_DISTANCE_MIN = 0.01f;  // [m]
inline constexpr float TOF_DISTANCE_MAX = 4.0f;   // [m]

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
inline constexpr bool USE_OPTICAL_FLOW = true;     // POS_HOLD調整のため有効化
inline constexpr bool USE_BAROMETER = false;       // 初期テスト: OFF
inline constexpr bool USE_TOF = true;              // STABILIZE影響検証
inline constexpr bool USE_MAGNETOMETER = false;    // 地磁気無効（モータ磁気干渉が大きいため）
inline constexpr bool ENABLE_YAW_ESTIMATION = true; // ヨー推定（ジャイロ積分）※falseで0固定

// -----------------------------------------------------------------------------
// プロセスノイズ (Q行列)
// 値が大きい = センサーを信頼、値が小さい = モデルを信頼
// -----------------------------------------------------------------------------
inline constexpr float GYRO_NOISE = 0.009655f;         // ジャイロノイズ [rad/s/√Hz]
inline constexpr float ACCEL_NOISE = 0.3f;              // 加速度ノイズ [m/s²/√Hz] (速度推定BW=1.5Hz狙い)
inline constexpr float GYRO_BIAS_NOISE = 0.000013f;    // ジャイロバイアスランダムウォーク
inline constexpr float ACCEL_BIAS_NOISE = 0.0001f;       // 加速度バイアスランダムウォーク [m/s²/√s] (シミュレーション最適化: ドリフト87%削減)

// -----------------------------------------------------------------------------
// 観測ノイズ (R行列)
// 値が大きい = 観測を信頼しない、値が小さい = 観測を信頼
// -----------------------------------------------------------------------------
inline constexpr float BARO_NOISE = 0.1f;              // 気圧高度ノイズ [m]
inline constexpr float TOF_NOISE = 0.03f;              // ToFノイズ [m] (VL53L3CX sigma ~20-30mm)
inline constexpr float MAG_NOISE = 1.0f;               // 地磁気ノイズ [uT] 実測std≈0.94
inline constexpr float FLOW_NOISE = 0.30f;             // オプティカルフローノイズ [m/s] (BW=1.5Hz: AN=0.3との組合せ)
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
inline constexpr float ACCEL_MOTION_THRESHOLD = 1.0f;  // 加速度モーション閾値 [m/s²]

// イノベーションゲート（位置センサ用、絶対値 [m]、0=無効）
// Innovation gates for position sensors (absolute [m], 0=disabled)
// P-collapse makes chi2 gates ineffective for position sensors
inline constexpr float BARO_INNOV_GATE = 0.5f;         // 気圧計 [m]
inline constexpr float TOF_INNOV_GATE = 0.5f;          // ToF [m] (normal P99.5 ≈ 10cm)

// χ²ゲート閾値（姿勢センサ用、0=無効）
// Chi-squared gate thresholds for attitude sensors (0=disabled)
inline constexpr float MAG_CHI2_GATE = 7.81f;          // 地磁気 [χ²(3,0.95)]
inline constexpr float FLOW_CHI2_GATE = 0.0f;          // フロー 無効（発散防止のため）
inline constexpr float FLOW_INNOV_CLAMP = 0.3f;        // フローイノベーションクランプ [m/s]（スパイク制限）
inline constexpr float ACCEL_ATT_CHI2_GATE = 7.81f;    // 加速度姿勢 [χ²(3,0.95)]

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
// 姿勢補正
// -----------------------------------------------------------------------------

// 適応的Rスケーリング係数
// R = R_base × (1 + K_ADAPTIVE × |accel_norm - g|²)
// 動的加速度が大きいほど加速度計の補正を弱める（ゼロにはしない）
// K=0: 常に同じ重みで補正、K=10: 1m/s²偏差で補正が約1/11に低減
inline constexpr float K_ADAPTIVE = 10.0f;

// 姿勢補正クランプ [rad]
// クロス共分散経由の姿勢ジャンプを防止する上限値
// ±0.05 rad ≈ ±2.9°
inline constexpr float ATT_CORRECTION_CLAMP = 0.05f;

// 地磁気ノルム有効範囲 [uT]
// この範囲外の読み取りはアウトライアとして棄却
inline constexpr float MAG_NORM_MIN = 10.0f;
inline constexpr float MAG_NORM_MAX = 100.0f;

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
// | センサー | 観測std norm | 設定閾値 | 余裕 |
// |---------|-------------|---------|------|
// | Accel   | 0.017-0.020 | 0.025   | ~25% |
// | Gyro    | 0.0025-0.003| 0.005   | ~70% |
// | Mag     | 0.85-1.17   | 1.3     | ~10% |
// | Baro    | ~0          | 0.01    | -    |
// | ToF     | 0.0007-0.001| 0.003   | ~200%|
// | OptFlow | dx/dy<1     | 3       | ~200%|
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
// LPF (Low Pass Filter) Settings
// =============================================================================

namespace lpf {
inline constexpr float ACCEL_CUTOFF_HZ = 50.0f;    // 加速度LPFカットオフ [Hz]
inline constexpr float GYRO_CUTOFF_HZ = 100.0f;    // ジャイロLPFカットオフ [Hz]
} // namespace lpf

// Gyro notch filter for rate control loop
// レート制御ループ用ジャイロノッチフィルタ
namespace gyro_notch {
inline constexpr bool ENABLED = false;             // ノッチフィルタ無効化
inline constexpr float CENTER_FREQ_HZ = 12.0f;    // 中心周波数 [Hz]
inline constexpr float Q = 5.0f;                   // Q値（大きいほど狭帯域）
} // namespace gyro_notch

// =============================================================================
// Sensor Driver Settings
// =============================================================================

namespace sensor {

// BMM150 (地磁気センサー)
// data_rate: 0=10Hz, 1=2Hz, 2=6Hz, 3=8Hz, 4=15Hz, 5=20Hz, 6=25Hz, 7=30Hz
inline constexpr int BMM150_DATA_RATE = 6;         // ODR_25HZ
// preset: 0=LOW_POWER, 1=REGULAR, 2=ENHANCED, 3=HIGH_ACCURACY
inline constexpr int BMM150_PRESET = 1;            // REGULAR

// BMP280 (気圧センサー)
// mode: 0=SLEEP, 1=FORCED, 3=NORMAL (2は無効値)
inline constexpr int BMP280_MODE = 3;              // NORMAL
// oversampling: 0=SKIP, 1=X1, 2=X2, 3=X4, 4=X8, 5=X16
inline constexpr int BMP280_PRESS_OVERSAMPLING = 3; // X4
inline constexpr int BMP280_TEMP_OVERSAMPLING = 2;  // X2
// standby: 0=0.5ms, 1=62.5ms, 2=125ms, 3=250ms, 4=500ms, 5=1000ms, 6=2000ms, 7=4000ms
inline constexpr int BMP280_STANDBY = 1;           // MS_62_5
// filter: 0=OFF, 1=COEF_2, 2=COEF_4, 3=COEF_8, 4=COEF_16
inline constexpr int BMP280_FILTER = 2;            // COEF_4

// INA3221 (電源モニター)
inline constexpr int POWER_BATTERY_CHANNEL = 1;    // バッテリー接続チャンネル
inline constexpr float POWER_SHUNT_RESISTOR = 0.1f; // シャント抵抗 [Ω]

} // namespace sensor

// =============================================================================
// Communication Settings
// =============================================================================

namespace comm {
inline constexpr int WIFI_CHANNEL = 1;             // ESP-NOW WiFiチャンネル
inline constexpr int TIMEOUT_MS = 500;             // 通信タイムアウト [ms]
} // namespace comm

namespace telemetry {
inline constexpr int PORT = 80;                    // WebSocketポート
inline constexpr int RATE_HZ = 50;                 // 送信レート [Hz]
} // namespace telemetry

namespace logger {
inline constexpr int RATE_HZ = 400;                // ログレート [Hz]
} // namespace logger

// =============================================================================
// Actuator Settings
// =============================================================================

namespace motor {
inline constexpr int PWM_FREQ_HZ = 150000;         // PWM周波数 [Hz]
inline constexpr int PWM_RESOLUTION_BITS = 8;      // PWM分解能 [bits]
} // namespace motor

namespace buzzer {
inline constexpr int LEDC_CHANNEL = 4;             // LEDCチャンネル
inline constexpr int LEDC_TIMER = 1;               // LEDCタイマー
} // namespace buzzer

// =============================================================================
// Rate Control (角速度制御) Configuration
// =============================================================================
//
// コントローラ入力 → 目標角速度 → PID制御 → モーターミキサー
//
// 感度パラメータ: スティック最大倒し量時の目標角速度 [rad/s]
// PIDゲイン: 角速度追従のためのPIDパラメータ
//
// 物理単位モード有効時:
//   PID出力は物理トルク [Nm] → ControlAllocator → モータ推力 [N] → Duty
// 電圧スケールモード（レガシー）:
//   PID出力は電圧 [V] → 従来ミキサー → Duty
//

namespace rate_control {

// -----------------------------------------------------------------------------
// 物理単位モード切り替え (Physical Units Mode Switch)
// 1: 物理単位ベース（トルク [Nm] 出力）
// 0: 電圧スケール（レガシー [V] 出力）
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
// 不完全微分PID: C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s + 1))
//
// ゲイン変換理論:
//   新Kp = k_τ × 旧Kp  (出力単位変換)
//   新Ti = 旧Ti       (時定数は不変)
//   新Td = 旧Td       (時定数は不変)
//
// k_τ計算（ホバー動作点での線形化）:
//   ホバー時: duty ≈ 0.63, thrust ≈ 0.086 N/motor
//   dT/dDuty ≈ 0.23 N at hover (非線形のためT_maxより小さい)
//   4モータの寄与を考慮:
//     k_τ_roll/pitch = 4 × (0.25/3.7) × 0.23 × 0.023 ≈ 1.4×10⁻³ Nm/V
//     k_τ_yaw        = 4 × (0.25/3.7) × 0.23 × 0.00971 ≈ 5.9×10⁻⁴ Nm/V
// -----------------------------------------------------------------------------

#if USE_PHYSICAL_UNITS
// ==========================================================================
// 物理単位ベースゲイン (Physical Units: Torque [Nm])
// ==========================================================================

// Roll rate PID
inline constexpr float ROLL_RATE_KP = 1.365e-3f;   // [Nm/(rad/s)] = 1.5x of 9.1e-4
inline constexpr float ROLL_RATE_TI = 0.7f;        // 積分時間 [s]
inline constexpr float ROLL_RATE_TD = 0.01f;       // 微分時間 [s]

// Pitch rate PID
inline constexpr float PITCH_RATE_KP = 1.995e-3f;  // [Nm/(rad/s)] = 1.5x of 1.33e-3
inline constexpr float PITCH_RATE_TI = 0.7f;       // 積分時間 [s]
inline constexpr float PITCH_RATE_TD = 0.01f;      // 微分時間 [s]

// Yaw rate PID
inline constexpr float YAW_RATE_KP = 5.31e-3f;     // [Nm/(rad/s)] = 3x of 1.77e-3 (yaw authority)
inline constexpr float YAW_RATE_TI = 1.6f;         // 積分時間 [s] (2x of 0.8, reduce integral buildup)
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
//
// カスケード制御の外側ループ。スティック入力から角度セットポイントを生成し、
// 内側のレートコントローラへのレートセットポイントを出力する。
//
// Outer loop of cascade control. Converts stick input to angle setpoints
// and outputs rate setpoints for the inner rate controller.
//
namespace attitude_control {

// -----------------------------------------------------------------------------
// Maximum Tilt Angles - 最大傾斜角
// スティック最大倒し時の目標角度 [rad]
// -----------------------------------------------------------------------------
inline constexpr float MAX_ROLL_ANGLE = 0.5236f;   // 30 deg = π/6 rad
inline constexpr float MAX_PITCH_ANGLE = 0.5236f;  // 30 deg = π/6 rad

// -----------------------------------------------------------------------------
// Attitude PID Gains (Outer Loop) - 姿勢PIDゲイン（外側ループ）
// 出力: レートセットポイント [rad/s]
// -----------------------------------------------------------------------------
inline constexpr float ROLL_ANGLE_KP = 5.0f;    // 比例ゲイン [(rad/s) / rad]
inline constexpr float ROLL_ANGLE_TI = 4.0f;    // 積分時間 [s]
inline constexpr float ROLL_ANGLE_TD = 0.04f;   // 微分時間 [s]

inline constexpr float PITCH_ANGLE_KP = 5.0f;   // 比例ゲイン [(rad/s) / rad]
inline constexpr float PITCH_ANGLE_TI = 4.0f;   // 積分時間 [s]
inline constexpr float PITCH_ANGLE_TD = 0.04f;  // 微分時間 [s]

// -----------------------------------------------------------------------------
// Output Limits - 出力制限
// 姿勢PIDの出力（レートセットポイント）の最大値
// -----------------------------------------------------------------------------
inline constexpr float MAX_RATE_SETPOINT = 3.0f;  // [rad/s] (~172 deg/s)

// 共通パラメータ
inline constexpr float PID_ETA = 0.125f;  // 不完全微分フィルタ係数

} // namespace attitude_control

// =============================================================================
// Altitude Control Parameters - 高度制御パラメータ（ALTITUDE_HOLD Mode）
// =============================================================================
//
// カスケード制御: 高度PID（外ループ） → 速度PID（内ループ） → 推力補正
// ホバー推力をフィードフォワードとして加算し、補正量のみをPIDで計算
//
// Cascade control: Altitude PID (outer) -> Velocity PID (inner) -> thrust correction
// Hover thrust is added as feedforward; only correction is computed by PID.
//
namespace altitude_control {

// 機体パラメータ
// Vehicle parameters
inline constexpr float MASS = 0.037f;                    // [kg]
inline constexpr float GRAVITY = 9.81f;                  // [m/s²]
inline constexpr float MAX_TOTAL_THRUST = 4.0f * 0.168f;  // 0.672N (4 motors, duty≤0.95)

// ホバー推力推定のデフォルトパラメータ
// Default hover thrust estimation parameters
inline constexpr float HOVER_THRUST_CORRECTION = 1.12f;  // From flight log throttle measurement (stable 1.11-1.13)

// 高度PID（外ループ）: 高度誤差 [m] → 垂直速度目標 [m/s]
// Altitude PID (outer loop): altitude error [m] -> vertical velocity target [m/s]
inline constexpr float ALT_KP = 0.6f;           // 0.8で悪化→0.6に戻す (限界の60%)
inline constexpr float ALT_TI = 7.0f;           // 1.5×Tu
inline constexpr float ALT_TD = 0.0f;
inline constexpr float ALT_OUTPUT_MAX = 0.5f;   // Max climb/descent rate [m/s]

// 速度PID（内ループ）: 速度誤差 [m/s] → 推力補正 [N]
// Velocity PID (inner loop): velocity error [m/s] -> thrust correction [N]
inline constexpr float VEL_KP = 0.1f;
inline constexpr float VEL_TI = 2.5f;           // 0.5×Tu — 定常偏差除去
inline constexpr float VEL_TD = 0.0f;
inline constexpr float VEL_OUTPUT_MAX = 0.15f;   // Max thrust correction [N]

// スティック設定
// Stick configuration
inline constexpr float STICK_DEADZONE = 0.1f;        // Normalized ADC ±0.1 = hold
inline constexpr float MAX_CLIMB_RATE = 0.5f;         // [m/s]
inline constexpr float MAX_DESCENT_RATE = 0.3f;       // [m/s]

// 高度制限
// Altitude limits
inline constexpr float MIN_ALTITUDE = 0.10f;   // [m] Ground protection
inline constexpr float MAX_ALTITUDE = 3.0f;    // [m] ToF effective range

// 共通
inline constexpr float PID_ETA = 0.125f;

} // namespace altitude_control

// =============================================================================
// Position Control Parameters - 位置制御パラメータ（POSITION_HOLD Mode）
// =============================================================================
//
// カスケード制御: 位置PID（外ループ） → 速度PID（内ループ） → 角度補正
// NED座標系で演算し、yaw角でbody frameに変換してAttitudeControllerに渡す
//
// Cascade control: Position PID (outer) -> Velocity PID (inner) -> angle correction
// Computed in NED frame, converted to body frame via yaw rotation.
//
namespace position_control {

// 位置PID（外ループ）: 位置誤差 [m] → 速度目標 [m/s]
// Position PID (outer loop): position error [m] -> velocity target [m/s]
inline constexpr float POS_KP = 1.0f;
inline constexpr float POS_TI = 5.0f;         // Slow integral for drift correction
inline constexpr float POS_TD = 0.1f;
inline constexpr float POS_OUTPUT_MAX = 0.5f;  // Max horizontal velocity target [m/s]

// 速度PID（内ループ）: 速度誤差 [m/s] → 角度補正 [rad]
// Velocity PID (inner loop): velocity error [m/s] -> angle correction [rad]
inline constexpr float VEL_KP = 0.3f;
inline constexpr float VEL_TI = 2.0f;
inline constexpr float VEL_TD = 0.02f;
inline constexpr float VEL_OUTPUT_MAX = 0.20f; // Max angle correction ~11.5° safety limit

// スティック設定
// Stick configuration
inline constexpr float STICK_DEADZONE = 0.15f;
inline constexpr float MAX_HORIZONTAL_SPEED = 0.5f; // [m/s]

// 位置制限
// Position limits
inline constexpr float MAX_POSITION_OFFSET = 5.0f;  // [m] Max distance from capture point

// 安全制限
// Safety limits
inline constexpr float MAX_TILT_ANGLE = 0.1745f;    // ~10° max tilt from position control

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

namespace button {
inline constexpr int DEBOUNCE_MS = 50;             // デバウンス時間 [ms]
} // namespace button

// =============================================================================
// Trim Settings - トリム調整
// =============================================================================
//
// コントローラ入力のオフセット補正。機体の微妙な傾きや
// スティックのセンターずれを補正するために使用。
//
// Offset correction for controller input. Used to compensate for
// subtle aircraft tilt or stick center drift.
//
namespace trim {

// デフォルト値（初期値 = 0、トリムなし）
// Default values (initial = 0, no trim)
inline constexpr float DEFAULT_ROLL = 0.0f;        // ロールトリム [-1.0 ~ +1.0]
inline constexpr float DEFAULT_PITCH = 0.0f;       // ピッチトリム [-1.0 ~ +1.0]
inline constexpr float DEFAULT_YAW = 0.0f;         // ヨートリム [-1.0 ~ +1.0]

// トリム値の最大範囲
// Maximum trim range
inline constexpr float MAX_TRIM = 0.2f;            // 最大±20% (±0.2)

} // namespace trim

namespace led {
// M5Stamp S3 内蔵LED
inline constexpr int NUM_LEDS_MCU = 1;

// StampFly ボード上LED（デイジーチェーン）
inline constexpr int NUM_LEDS_BODY = 2;

// LED インデックス（GPIO_LED_BODYのデイジーチェーン内）
inline constexpr int IDX_BODY_TOP = 0;     // 上面/表
inline constexpr int IDX_BODY_BOTTOM = 1;  // 下面/裏

// 後方互換性のため残す（deprecated）
inline constexpr int NUM_LEDS = 1;
} // namespace led

} // namespace config
