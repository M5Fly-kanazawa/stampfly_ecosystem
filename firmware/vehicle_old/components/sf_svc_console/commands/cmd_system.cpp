/**
 * @file cmd_system.cpp
 * @brief System commands (status, reboot, version)
 *
 * システムコマンド（status, reboot, version）
 */

#include "console.hpp"
#include "stampfly_state.hpp"
#include "controller_comm.hpp"
#include "esp_console.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// External references
// 外部参照
extern stampfly::ControllerComm* g_comm_ptr;

namespace stampfly {

// =============================================================================
// status command
// =============================================================================

static int cmd_status(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    console.print("=== System Status ===\r\n");

    // Uptime
    // 起動時間
    uint32_t uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t uptime_sec = uptime_ms / 1000;
    uint32_t hours = uptime_sec / 3600;
    uint32_t mins = (uptime_sec % 3600) / 60;
    uint32_t secs = uptime_sec % 60;
    console.print("Uptime: %02lu:%02lu:%02lu (%lu ms)\r\n",
                  (unsigned long)hours, (unsigned long)mins,
                  (unsigned long)secs, (unsigned long)uptime_ms);

    // Flight state
    // 飛行状態
    const char* flight_state_str = "UNKNOWN";
    switch (state.getFlightState()) {
        case FlightState::INIT: flight_state_str = "INIT"; break;
        case FlightState::CALIBRATING: flight_state_str = "CALIBRATING"; break;
        case FlightState::IDLE: flight_state_str = "IDLE"; break;
        case FlightState::ARMED: flight_state_str = "ARMED"; break;
        case FlightState::FLYING: flight_state_str = "FLYING"; break;
        case FlightState::LANDING: flight_state_str = "LANDING"; break;
        case FlightState::ERROR: flight_state_str = "ERROR"; break;
    }
    console.print("Flight State: %s\r\n", flight_state_str);

    // Error code
    // エラーコード
    const char* error_str = "NONE";
    switch (state.getErrorCode()) {
        case ErrorCode::NONE: error_str = "NONE"; break;
        case ErrorCode::SENSOR_IMU: error_str = "SENSOR_IMU"; break;
        case ErrorCode::SENSOR_MAG: error_str = "SENSOR_MAG"; break;
        case ErrorCode::SENSOR_BARO: error_str = "SENSOR_BARO"; break;
        case ErrorCode::SENSOR_TOF: error_str = "SENSOR_TOF"; break;
        case ErrorCode::SENSOR_FLOW: error_str = "SENSOR_FLOW"; break;
        case ErrorCode::SENSOR_POWER: error_str = "SENSOR_POWER"; break;
        case ErrorCode::LOW_BATTERY: error_str = "LOW_BATTERY"; break;
        case ErrorCode::COMM_LOST: error_str = "COMM_LOST"; break;
        case ErrorCode::CALIBRATION_FAILED: error_str = "CALIBRATION_FAILED"; break;
    }
    console.print("Error: %s\r\n", error_str);

    // Battery
    // バッテリー電圧
    console.print("Battery: %.2fV\r\n", state.getVoltage());

    // ESP-NOW status
    // ESP-NOW ステータス
    if (g_comm_ptr != nullptr) {
        console.print("ESP-NOW: %s, %s\r\n",
                      g_comm_ptr->isPaired() ? "paired" : "not paired",
                      g_comm_ptr->isConnected() ? "connected" : "disconnected");
    } else {
        console.print("ESP-NOW: not available\r\n");
    }

    // ESKF status
    // ESKF ステータス
    console.print("ESKF: %s\r\n", state.isESKFInitialized() ? "initialized" : "not initialized");

    // Attitude
    // 姿勢
    StateVector3 att = state.getAttitude();
    console.print("Attitude: R=%.1f P=%.1f Y=%.1f deg\r\n",
                  att.x * 180.0f / M_PI, att.y * 180.0f / M_PI, att.z * 180.0f / M_PI);

    return 0;
}

// =============================================================================
// reboot command
// =============================================================================

static int cmd_reboot(int argc, char** argv)
{
    auto& console = Console::getInstance();
    console.print("Rebooting system...\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return 0;  // Never reached
}

// =============================================================================
// version command
// =============================================================================

static int cmd_version(int argc, char** argv)
{
    auto& console = Console::getInstance();
    console.print("StampFly RTOS\r\n");
    console.print("  ESP-IDF: %s\r\n", esp_get_idf_version());
    console.print("  Chip: ESP32-S3\r\n");
    console.print("  Console: esp_console\r\n");
    return 0;
}

// =============================================================================
// help command (custom implementation for WiFi CLI compatibility)
// =============================================================================

static int cmd_help(int argc, char** argv)
{
    auto& console = Console::getInstance();

    console.print("\r\nAvailable commands:\r\n");
    console.print("  status    - Show system status\r\n");
    console.print("  version   - Show version info\r\n");
    console.print("  reboot    - Reboot the system\r\n");
    console.print("  sensor    - Show sensor data [imu|mag|baro|tof|flow|power|all]\r\n");
    console.print("  loglevel  - Set ESP_LOG level [none|error|warn|info|debug|verbose]\r\n");
    console.print("  binlog    - Binary log [on|off|freq]\r\n");
    console.print("  motor     - Motor control [test|spin|stop]\r\n");
    console.print("  trim      - Trim adjust [roll|pitch|yaw <val>|save|reset]\r\n");
    console.print("  gain      - Rate control gains [axis param value]\r\n");
    console.print("  comm      - Comm mode [espnow|udp|status]\r\n");
    console.print("  wifi      - WiFi settings [sta|ap|off|scan|status]\r\n");
    console.print("  pair      - Pairing control [start|stop|channel]\r\n");
    console.print("  unpair    - Clear pairing\r\n");
    console.print("  calib     - Sensor calibration [gyro|accel|status]\r\n");
    console.print("  magcal    - Magnetometer calibration [start|stop|status|clear]\r\n");
    console.print("  led       - LED [brightness <0-255>]\r\n");
    console.print("  sound     - Sound [on|off]\r\n");
    console.print("  pos       - Position [reset|status]\r\n");
    console.print("  debug     - Debug mode [on|off]\r\n");
    console.print("  ctrl      - Show controller input [watch [sec]]\r\n");
    console.print("  attitude  - Show attitude\r\n");
    console.print("\r\n");

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_system_commands()
{
    // help (custom implementation)
    const esp_console_cmd_t help_cmd = {
        .command = "help",
        .help = "Show available commands",
        .hint = NULL,
        .func = &cmd_help,
        .argtable = NULL,
    };
    esp_console_cmd_register(&help_cmd);

    // status
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "Show system status",
        .hint = NULL,
        .func = &cmd_status,
        .argtable = NULL,
    };
    esp_console_cmd_register(&status_cmd);

    // reboot
    const esp_console_cmd_t reboot_cmd = {
        .command = "reboot",
        .help = "Reboot the system",
        .hint = NULL,
        .func = &cmd_reboot,
        .argtable = NULL,
    };
    esp_console_cmd_register(&reboot_cmd);

    // version
    const esp_console_cmd_t version_cmd = {
        .command = "version",
        .help = "Show version info",
        .hint = NULL,
        .func = &cmd_version,
        .argtable = NULL,
    };
    esp_console_cmd_register(&version_cmd);
}

}  // namespace stampfly
