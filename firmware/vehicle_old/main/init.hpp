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
 * LPF, MagCalibrator, SensorFusion (ESKF), LandingHandler
 */
esp_err_t estimators();

/**
 * @brief 通信の初期化
 * ESP-NOW ControllerComm
 */
esp_err_t communication();

/**
 * @brief Consoleの初期化 (esp_console ベース)
 * Serial/WiFi共通のコマンド基盤
 */
esp_err_t console();

/**
 * @brief Loggerの初期化
 */
esp_err_t logger();

/**
 * @brief Telemetryの初期化
 */
esp_err_t telemetry();

/**
 * @brief WiFi CLIの初期化
 * Telnet-like CLI over WiFi (port 23)
 */
esp_err_t wifi_cli();

/**
 * @brief I2Cバスハンドルを取得
 */
i2c_master_bus_handle_t getI2CBus();

} // namespace init
