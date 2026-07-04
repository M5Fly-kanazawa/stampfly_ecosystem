/**
 * @file tasks.hpp
 * @brief FreeRTOSタスク関数のプロトタイプ宣言
 */

#pragma once

/**
 * @brief IMUタスク (400Hz)
 * BMI270読み取り、フィルタ適用、センサーフュージョン更新
 */
void IMUTask(void* pvParameters);

/**
 * @brief 制御タスク (400Hz)
 * 姿勢制御（ユーザー実装用スタブ）
 */
void ControlTask(void* pvParameters);

/**
 * @brief オプティカルフロータスク (100Hz)
 * PMW3901読み取り
 */
void OptFlowTask(void* pvParameters);

/**
 * @brief 地磁気タスク (100Hz)
 * BMM150読み取り、キャリブレーション適用
 */
void MagTask(void* pvParameters);

/**
 * @brief 気圧タスク (50Hz)
 * BMP280読み取り
 */
void BaroTask(void* pvParameters);

/**
 * @brief ToFタスク (30Hz)
 * VL53L3CX読み取り（底面・前方）
 */
void ToFTask(void* pvParameters);

/**
 * @brief 電源監視タスク (10Hz)
 * INA3221読み取り、バッテリー状態監視
 */
void PowerTask(void* pvParameters);

/**
 * @brief LEDタスク (50Hz)
 * LED状態更新
 */
void LEDTask(void* pvParameters);

/**
 * @brief ボタンタスク (50Hz)
 * ボタン状態監視
 */
void ButtonTask(void* pvParameters);

/**
 * @brief 通信タスク (50Hz)
 * ESP-NOWコントローラー通信処理
 */
void CommTask(void* pvParameters);

/**
 * @brief CLIタスク
 * USBシリアルコマンドライン処理
 */
void CLITask(void* pvParameters);

/**
 * @brief テレメトリタスク (50Hz)
 * WebSocketブロードキャスト
 */
void TelemetryTask(void* pvParameters);
