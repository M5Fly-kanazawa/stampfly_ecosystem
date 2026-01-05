/**
 * @file init.hpp
 * @brief 初期化関数のプロトタイプ宣言
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

namespace init {

/**
 * @brief I2Cバスの初期化
 */
esp_err_t i2c();

/**
 * @brief センサーの初期化
 * IMU, Mag, Baro, ToF, OptFlow, PowerMonitor
 */
esp_err_t sensors();

/**
 * @brief アクチュエータの初期化
 * Motor, LED, Buzzer, Button
 */
esp_err_t actuators();

/**
 * @brief 推定器の初期化
 * LPF, MagCalibrator, SensorFusion, AttitudeEstimator, AltitudeEstimator
 */
esp_err_t estimators();

/**
 * @brief 通信の初期化
 * ESP-NOW ControllerComm
 */
esp_err_t communication();

/**
 * @brief CLIの初期化
 */
esp_err_t cli();

/**
 * @brief Loggerの初期化
 */
esp_err_t logger();

/**
 * @brief Telemetryの初期化
 */
esp_err_t telemetry();

/**
 * @brief I2Cバスハンドルを取得
 */
i2c_master_bus_handle_t getI2CBus();

} // namespace init
