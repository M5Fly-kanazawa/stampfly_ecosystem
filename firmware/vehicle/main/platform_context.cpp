/**
 * @file platform_context.cpp
 * @brief Platform context initialization from globals
 *        グローバルからのプラットフォームコンテキスト初期化
 */

#include "platform_context.hpp"
#include "globals.hpp"
#include "esp_log.h"

static const char* TAG = "PlatformCtx";

namespace platform {

static PlatformContext s_context;

PlatformContext& getPlatformContext()
{
    return s_context;
}

void initPlatformContext()
{
    // Sensors
    s_context.imu = &globals::g_imu;
    s_context.mag = &globals::g_mag;
    s_context.mag_cal = &globals::g_mag_cal;
    s_context.baro = &globals::g_baro;
    s_context.tof_bottom = &globals::g_tof_bottom;
    s_context.tof_front = &globals::g_tof_front;
    s_context.optflow = globals::g_optflow;  // Already a pointer
    s_context.power = &globals::g_power;

    // Actuators
    s_context.motor = &globals::g_motor;
    s_context.buzzer = &globals::g_buzzer;
    s_context.button = &globals::g_button;

    // Estimation
    s_context.fusion = &globals::g_fusion;
    s_context.landing_handler = &globals::g_landing_handler;

    // Communication
    s_context.comm = &globals::g_comm;
    s_context.logger = &globals::g_logger;

    // Health
    s_context.health = &globals::g_health;

    ESP_LOGI(TAG, "Platform context initialized");
}

}  // namespace platform
