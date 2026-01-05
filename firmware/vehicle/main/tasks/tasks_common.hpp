/**
 * @file tasks_common.hpp
 * @brief FreeRTOSタスク用の共通インクルードと定義
 */

#pragma once

#include "tasks.hpp"
#include "../config.hpp"
#include "../globals.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>
#include <cstddef>  // for offsetof

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "bmm150.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "power_monitor.hpp"
#include "mag_calibration.hpp"

// Actuators
#include "motor_driver.hpp"
#include "led.hpp"
#include "button.hpp"

// State and estimation
#include "stampfly_state.hpp"
#include "system_manager.hpp"
#include "sensor_fusion.hpp"
#include "sensor_health.hpp"
#include "filter.hpp"

// Communication
#include "controller_comm.hpp"
#include "cli.hpp"
#include "logger.hpp"
#include "telemetry.hpp"

// Helper functions defined in main.cpp
extern void initializeAttitudeFromBuffers();
extern void setMagReferenceFromBuffer();  // deprecated, use initializeAttitudeFromBuffers

// Debug: Yaw alert counter (control_task sets, led_task checks)
extern volatile int g_yaw_alert_counter;
