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
inline constexpr uint32_t STACK_SIZE_CLI = 4096;
inline constexpr uint32_t STACK_SIZE_TELEMETRY = 4096;

// =============================================================================
// Timing Constants
// =============================================================================

inline constexpr float IMU_DT = 0.0025f;          // 400Hz
inline constexpr float OPTFLOW_DT = 0.01f;        // 100Hz
inline constexpr float MAG_DT = 0.01f;            // 100Hz
inline constexpr float BARO_DT = 0.02f;           // 50Hz
inline constexpr float TOF_DT = 0.033f;           // ~30Hz

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
inline constexpr bool USE_OPTICAL_FLOW = true;     // オプティカルフロー（水平速度推定）
inline constexpr bool USE_BAROMETER = true;        // 気圧センサー（高度推定）
inline constexpr bool USE_TOF = true;              // ToFセンサー（高度推定）
inline constexpr bool USE_MAGNETOMETER = true;     // 地磁気センサー（ヨー推定）

// -----------------------------------------------------------------------------
// プロセスノイズ (Q行列)
// 値が大きい = センサーを信頼、値が小さい = モデルを信頼
// -----------------------------------------------------------------------------
inline constexpr float GYRO_NOISE = 0.009655f;         // ジャイロノイズ [rad/s/√Hz]
inline constexpr float ACCEL_NOISE = 0.062885f;        // 加速度ノイズ [m/s²/√Hz]
inline constexpr float GYRO_BIAS_NOISE = 0.000013f;    // ジャイロバイアスランダムウォーク
inline constexpr float ACCEL_BIAS_NOISE = 0.0001f;     // 加速度バイアスランダムウォーク（安定重視）

// -----------------------------------------------------------------------------
// 観測ノイズ (R行列)
// 値が大きい = 観測を信頼しない、値が小さい = 観測を信頼
// -----------------------------------------------------------------------------
inline constexpr float BARO_NOISE = 0.1f;              // 気圧高度ノイズ [m]
inline constexpr float TOF_NOISE = 0.002540f;          // ToFノイズ [m]
inline constexpr float MAG_NOISE = 1.0f;               // 地磁気ノイズ [uT] 実測std≈0.94
inline constexpr float FLOW_NOISE = 0.01f;             // オプティカルフローノイズ [m/s] 実測std≈0.011
inline constexpr float ACCEL_ATT_NOISE = 0.02f;        // 加速度計姿勢補正ノイズ [m/s²]

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
// 姿勢補正モード
// 0: 加速度絶対値フィルタのみ
// 1: 適応的R (水平加速度でRをスケーリング)
// 2: 角速度フィルタ (高回転時にRを増加)
// 3: 高回転時バイアス保護
// -----------------------------------------------------------------------------
inline constexpr int ATT_UPDATE_MODE = 0;
inline constexpr float K_ADAPTIVE = 0.0f;
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

// 安定判定の標準偏差閾値（std norm）
inline constexpr float ACCEL_STD_THRESHOLD = 0.025f;     // [m/s²]
inline constexpr float GYRO_STD_THRESHOLD = 0.005f;      // [rad/s]
inline constexpr float MAG_STD_THRESHOLD = 1.3f;         // [µT]
inline constexpr float BARO_STD_THRESHOLD = 0.20f;       // [m] BMP280静置時std≈0.08-0.15m
inline constexpr float TOF_STD_THRESHOLD = 0.003f;       // [m]
inline constexpr float OPTFLOW_STD_THRESHOLD = 3.0f;     // [counts] dx+dy

// 安定判定のタイミング
inline constexpr int CHECK_INTERVAL_MS = 200;            // チェック間隔 [ms]
inline constexpr int MIN_WAIT_MS = 2000;                 // 最小待機時間 [ms]
inline constexpr int MAX_WAIT_MS = 10000;                // 最大待機時間 [ms]
inline constexpr int STABLE_COUNT_REQUIRED = 5;          // 連続安定回数

// バッファ最小サンプル数（統計計算に必要）
inline constexpr int MIN_ACCEL_SAMPLES = 50;
inline constexpr int MIN_GYRO_SAMPLES = 50;
inline constexpr int MIN_MAG_SAMPLES = 50;
inline constexpr int MIN_BARO_SAMPLES = 10;
inline constexpr int MIN_TOF_SAMPLES = 5;
inline constexpr int MIN_OPTFLOW_SAMPLES = 10;

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
// LPF (Low Pass Filter) Settings
// =============================================================================

namespace lpf {
inline constexpr float ACCEL_CUTOFF_HZ = 50.0f;    // 加速度LPFカットオフ [Hz]
inline constexpr float GYRO_CUTOFF_HZ = 100.0f;    // ジャイロLPFカットオフ [Hz]
} // namespace lpf

// =============================================================================
// Sensor Driver Settings
// =============================================================================

namespace sensor {

// BMM150 (地磁気センサー)
// data_rate: 0=10Hz, 1=2Hz, 2=6Hz, 3=8Hz, 4=15Hz, 5=20Hz, 6=25Hz, 7=30Hz
inline constexpr int BMM150_DATA_RATE = 0;         // ODR_10HZ
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

namespace rate_control {

// -----------------------------------------------------------------------------
// 感度設定 (Sensitivity)
// スティック入力 ±1.0 に対する最大目標角速度 [rad/s]
// -----------------------------------------------------------------------------
inline constexpr float ROLL_RATE_MAX = 1.0f;       // ロール最大角速度 [rad/s] (~286 deg/s)
inline constexpr float PITCH_RATE_MAX = 1.0f;      // ピッチ最大角速度 [rad/s] (~286 deg/s)
inline constexpr float YAW_RATE_MAX = 5.0f;        // ヨー最大角速度 [rad/s] (~172 deg/s)

// -----------------------------------------------------------------------------
// PIDゲイン (Rate Controller)
// 不完全微分PID: C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s + 1))
// -----------------------------------------------------------------------------

// Roll rate PID
inline constexpr float ROLL_RATE_KP = 0.65f;       // 比例ゲイン
inline constexpr float ROLL_RATE_TI = 0.7f;        // 積分時間 [s] (0以下で無効) ← P制御のみ
inline constexpr float ROLL_RATE_TD = 0.01f;        // 微分時間 [s] (0以下で無効) ← P制御のみ

// Pitch rate PID
inline constexpr float PITCH_RATE_KP = 0.95f;      // 比例ゲイン
inline constexpr float PITCH_RATE_TI = 0.7f;       // 積分時間 [s] ← P制御のみ
inline constexpr float PITCH_RATE_TD = 0.025f;       // 微分時間 [s] ← P制御のみ

// Yaw rate PID
inline constexpr float YAW_RATE_KP = 3.0f;         // 比例ゲイン
inline constexpr float YAW_RATE_TI = 0.8f;         // 積分時間 [s] ← P制御のみ
inline constexpr float YAW_RATE_TD = 0.01f;         // 微分時間 [s] ← P制御のみ

// 共通パラメータ
inline constexpr float PID_ETA = 0.125f;             // 不完全微分フィルタ係数 (0.1~0.2)
inline constexpr float OUTPUT_LIMIT = 3.7f;        // PID出力制限 [V] (電圧スケール)

} // namespace rate_control

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
