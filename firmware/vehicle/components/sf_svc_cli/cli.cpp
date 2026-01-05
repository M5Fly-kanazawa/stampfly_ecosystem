/**
 * @file cli.cpp
 * @brief CLI Console Implementation (USB CDC)
 *
 * USB Serial経由のコマンドラインインターフェース
 * - センサ表示
 * - キャリブレーション
 * - モーターテスト
 * - ペアリング制御
 *
 * Note: Uses static arrays and function pointers instead of
 * std::map/std::function to avoid dynamic memory allocation.
 */

#include "cli.hpp"
#include "stampfly_state.hpp"
#include "mag_calibration.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_cdcacm.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <fcntl.h>
#include <unistd.h>

static const char* TAG = "CLI";

// NVS namespace and key for log level
static const char* NVS_NAMESPACE_CLI = "stampfly_cli";
static const char* NVS_KEY_LOGLEVEL = "loglevel";

// Helper: Load log level from NVS
static esp_log_level_t loadLogLevelFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CLI, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ESP_LOG_NONE;  // Default: silent (original behavior)
    }

    uint8_t level_val = ESP_LOG_NONE;
    ret = nvs_get_u8(handle, NVS_KEY_LOGLEVEL, &level_val);
    nvs_close(handle);

    if (ret != ESP_OK) {
        return ESP_LOG_NONE;  // Default: silent if key not found
    }

    // Validate range
    if (level_val > ESP_LOG_VERBOSE) {
        return ESP_LOG_NONE;
    }

    return static_cast<esp_log_level_t>(level_val);
}

// Helper: Save log level to NVS
static esp_err_t saveLogLevelToNVS(esp_log_level_t level)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CLI, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for loglevel: %d", ret);
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_LOGLEVEL, static_cast<uint8_t>(level));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}

// External reference to magnetometer calibrator (defined in main.cpp)
extern stampfly::MagCalibrator* g_mag_calibrator;

// External reference to logger (defined in main.cpp)
#include "logger.hpp"
extern stampfly::Logger* g_logger_ptr;

// External reference to controller comm (defined in main.cpp)
#include "controller_comm.hpp"
extern stampfly::ControllerComm* g_comm_ptr;

// LED Manager
#include "led_manager.hpp"

// External reference to motor driver (defined in main.cpp)
#include "motor_driver.hpp"
extern stampfly::MotorDriver* g_motor_ptr;

// External reference to buzzer (defined in main.cpp)
#include "buzzer.hpp"
extern stampfly::Buzzer* g_buzzer_ptr;

// External reference to sensor fusion (defined in globals.cpp)
#include "sensor_fusion.hpp"
extern sf::SensorFusion* g_fusion_ptr;

// External reference to rate controller (defined in control_task.cpp)
#include "rate_controller.hpp"
extern RateController* g_rate_controller_ptr;

namespace stampfly {

// Forward declarations for command handlers
static void cmd_help(int argc, char** argv, void* context);
static void cmd_status(int argc, char** argv, void* context);
static void cmd_sensor(int argc, char** argv, void* context);
static void cmd_teleplot(int argc, char** argv, void* context);
static void cmd_log(int argc, char** argv, void* context);
static void cmd_binlog(int argc, char** argv, void* context);
static void cmd_loglevel(int argc, char** argv, void* context);
static void cmd_calib(int argc, char** argv, void* context);
static void cmd_magcal(int argc, char** argv, void* context);
static void cmd_motor(int argc, char** argv, void* context);
static void cmd_pair(int argc, char** argv, void* context);
static void cmd_unpair(int argc, char** argv, void* context);
static void cmd_reset(int argc, char** argv, void* context);
static void cmd_gain(int argc, char** argv, void* context);
static void cmd_attitude(int argc, char** argv, void* context);
static void cmd_version(int argc, char** argv, void* context);
static void cmd_ctrl(int argc, char** argv, void* context);
static void cmd_debug(int argc, char** argv, void* context);
static void cmd_led(int argc, char** argv, void* context);
static void cmd_sound(int argc, char** argv, void* context);
static void cmd_pos(int argc, char** argv, void* context);

esp_err_t CLI::init()
{
    ESP_LOGI(TAG, "Initializing CLI");

    // Disable line ending conversion for binary output
    // This prevents 0x0a from being converted to 0x0d 0x0a
    // Using CDC ACM VFS functions since CONFIG_ESP_CONSOLE_USB_CDC is enabled
    // See: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/stdio.html
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_LF);

    // Disable buffering on stdin and stdout for immediate I/O
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // Set stdin to non-blocking mode
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    input_pos_ = 0;
    memset(input_buffer_, 0, sizeof(input_buffer_));
    command_count_ = 0;

    // Load and apply saved log level from NVS
    esp_log_level_t saved_level = loadLogLevelFromNVS();
    esp_log_level_set("*", saved_level);

    static const char* level_names[] = {
        "none", "error", "warn", "info", "debug", "verbose"
    };
    ESP_LOGI(TAG, "Log level restored from NVS: %s", level_names[saved_level]);

    initialized_ = true;
    ESP_LOGI(TAG, "CLI initialized");

    return ESP_OK;
}

void CLI::registerCommand(const char* name, CommandHandlerFn handler,
                          const char* help, void* context)
{
    if (command_count_ >= MAX_COMMANDS) {
        ESP_LOGW(TAG, "Max commands reached, cannot register: %s", name);
        return;
    }

    CommandEntry& entry = commands_[command_count_];
    strncpy(entry.name, name, MAX_CMD_NAME_LEN - 1);
    entry.name[MAX_CMD_NAME_LEN - 1] = '\0';
    strncpy(entry.help, help ? help : "", MAX_HELP_LEN - 1);
    entry.help[MAX_HELP_LEN - 1] = '\0';
    entry.handler = handler;
    entry.context = context;
    command_count_++;
}

void CLI::processInput()
{
    if (!initialized_) return;

    // Non-blocking read from stdin
    int c = getchar();
    if (c == EOF) return;

    // エコーバック - use write() to bypass stdio buffering
    char ch = static_cast<char>(c);
    write(fileno(stdout), &ch, 1);
    fsync(fileno(stdout));

    if (c == '\r' || c == '\n') {
        if (input_pos_ > 0) {
            // 改行を出力
            const char* newline = "\r\n";
            write(fileno(stdout), newline, 2);

            input_buffer_[input_pos_] = '\0';
            parseAndExecute(input_buffer_);
            input_pos_ = 0;
            memset(input_buffer_, 0, sizeof(input_buffer_));

            // プロンプト表示
            print("> ");
        }
    }
    else if (c == '\b' || c == 0x7F) {  // Backspace or Delete
        if (input_pos_ > 0) {
            input_pos_--;
            // 画面上のバックスペース処理
            const char* bs = "\b \b";
            write(fileno(stdout), bs, 3);
        }
    }
    else if (c >= 0x20 && c < 0x7F) {  // Printable ASCII
        if (input_pos_ < MAX_CMD_LEN - 1) {
            input_buffer_[input_pos_++] = ch;
        }
    }
}

void CLI::print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

void CLI::parseAndExecute(const char* line)
{
    char buffer[MAX_CMD_LEN];
    strncpy(buffer, line, MAX_CMD_LEN - 1);
    buffer[MAX_CMD_LEN - 1] = '\0';

    char* argv[MAX_ARGS];
    int argc = 0;

    char* token = strtok(buffer, " \t\n");
    while (token != nullptr && argc < static_cast<int>(MAX_ARGS)) {
        argv[argc++] = token;
        token = strtok(nullptr, " \t\n");
    }

    if (argc == 0) return;

    // Search for command in array
    for (size_t i = 0; i < command_count_; i++) {
        if (strcmp(commands_[i].name, argv[0]) == 0) {
            commands_[i].handler(argc, argv, commands_[i].context);
            return;
        }
    }

    print("Unknown command: %s\r\n", argv[0]);
    print("Type 'help' for available commands\r\n");
}

void CLI::printHelp()
{
    print("\r\n=== StampFly CLI ===\r\n");
    print("Available commands:\r\n");
    for (size_t i = 0; i < command_count_; i++) {
        print("  %-12s %s\r\n", commands_[i].name, commands_[i].help);
    }
    print("\r\n");
}

void CLI::registerDefaultCommands()
{
    registerCommand("help", cmd_help, "Show available commands", this);
    registerCommand("status", cmd_status, "Show system status", this);
    registerCommand("sensor", cmd_sensor, "Show sensor data", this);
    registerCommand("teleplot", cmd_teleplot, "Teleplot stream [on|off]", this);
    registerCommand("log", cmd_log, "CSV log [on|off|header]", this);
    registerCommand("binlog", cmd_binlog, "Binary log [on|off|freq] @400Hz", this);
    registerCommand("loglevel", cmd_loglevel, "Set ESP_LOG level [none|error|warn|info|debug|verbose]", this);
    registerCommand("calib", cmd_calib, "Run calibration", this);
    registerCommand("magcal", cmd_magcal, "Mag calibration [start|stop|status|save|clear]", this);
    registerCommand("motor", cmd_motor, "Motor control", this);
    registerCommand("pair", cmd_pair, "Enter pairing mode", this);
    registerCommand("unpair", cmd_unpair, "Clear pairing", this);
    registerCommand("reset", cmd_reset, "Reset system", this);
    registerCommand("gain", cmd_gain, "Rate control gains [axis param value]", this);
    registerCommand("attitude", cmd_attitude, "Show attitude", this);
    registerCommand("version", cmd_version, "Show version info", this);
    registerCommand("ctrl", cmd_ctrl, "Show controller input [watch]", this);
    registerCommand("debug", cmd_debug, "Debug mode [on|off] (ignore errors)", this);
    registerCommand("led", cmd_led, "LED [brightness <0-255>]", this);
    registerCommand("sound", cmd_sound, "Sound [on|off]", this);
    registerCommand("pos", cmd_pos, "Position [reset|status]", this);
}

// ========== Command Handlers ==========

static void cmd_help(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->printHelp();
}

static void cmd_status(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& state = StampFlyState::getInstance();

    cli->print("=== System Status ===\r\n");

    // Flight state
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
    cli->print("Flight State: %s\r\n", flight_state_str);

    // Error code
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
    cli->print("Error: %s\r\n", error_str);

    // Battery
    cli->print("Battery: %.2fV\r\n", state.getVoltage());

    // ESP-NOW status
    if (g_comm_ptr != nullptr) {
        cli->print("ESP-NOW: %s, %s\r\n",
                   g_comm_ptr->isPaired() ? "paired" : "not paired",
                   g_comm_ptr->isConnected() ? "connected" : "disconnected");
    } else {
        cli->print("ESP-NOW: not available\r\n");
    }

    // ESKF status
    cli->print("ESKF: %s\r\n", state.isESKFInitialized() ? "initialized" : "not initialized");

    // Attitude
    StateVector3 att = state.getAttitude();
    cli->print("Attitude: R=%.1f P=%.1f Y=%.1f deg\r\n",
               att.x * 180.0f / M_PI, att.y * 180.0f / M_PI, att.z * 180.0f / M_PI);
}

static void cmd_sensor(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& state = StampFlyState::getInstance();

    if (argc < 2) {
        cli->print("Usage: sensor [imu|mag|baro|tof|flow|power|all]\r\n");
        return;
    }

    const char* sensor = argv[1];
    if (strcmp(sensor, "imu") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 accel, gyro;
        state.getIMUData(accel, gyro);
        cli->print("IMU:\r\n");
        cli->print("  Accel: X=%.2f, Y=%.2f, Z=%.2f [m/s^2]\r\n", accel.x, accel.y, accel.z);
        cli->print("  Gyro:  X=%.3f, Y=%.3f, Z=%.3f [rad/s]\r\n", gyro.x, gyro.y, gyro.z);
    }
    if (strcmp(sensor, "mag") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 mag;
        state.getMagData(mag);
        cli->print("Mag: X=%.1f, Y=%.1f, Z=%.1f [uT]\r\n", mag.x, mag.y, mag.z);
    }
    if (strcmp(sensor, "baro") == 0 || strcmp(sensor, "all") == 0) {
        float altitude, pressure;
        state.getBaroData(altitude, pressure);
        cli->print("Baro: Pressure=%.0f [Pa], Alt=%.2f [m]\r\n", pressure, altitude);
    }
    if (strcmp(sensor, "tof") == 0 || strcmp(sensor, "all") == 0) {
        float bottom, front;
        state.getToFData(bottom, front);
        cli->print("ToF: Bottom=%.3f [m], Front=%.3f [m]\r\n", bottom, front);
    }
    if (strcmp(sensor, "flow") == 0 || strcmp(sensor, "all") == 0) {
        float vx, vy;
        state.getFlowData(vx, vy);
        cli->print("OptFlow: Vx=%.3f, Vy=%.3f [m/s]\r\n", vx, vy);
    }
    if (strcmp(sensor, "power") == 0 || strcmp(sensor, "all") == 0) {
        float voltage, current;
        state.getPowerData(voltage, current);
        cli->print("Power: %.2f [V], %.1f [mA]\r\n", voltage, current * 1000.0f);
    }
    if (strcmp(sensor, "imu") != 0 && strcmp(sensor, "mag") != 0 &&
        strcmp(sensor, "baro") != 0 && strcmp(sensor, "tof") != 0 &&
        strcmp(sensor, "flow") != 0 && strcmp(sensor, "power") != 0 &&
        strcmp(sensor, "all") != 0) {
        cli->print("Unknown sensor: %s\r\n", sensor);
        cli->print("Available: imu, mag, baro, tof, flow, power, all\r\n");
    }
}

static void cmd_teleplot(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: teleplot [on|off]\r\n");
        cli->print("Current: %s\r\n", cli->isTeleplotEnabled() ? "on" : "off");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        cli->setTeleplotEnabled(true);
        cli->print("Teleplot streaming ON\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        cli->setTeleplotEnabled(false);
        cli->print("Teleplot streaming OFF\r\n");
    } else {
        cli->print("Usage: teleplot [on|off]\r\n");
    }
}

static void cmd_log(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: log [on|off|header]\r\n");
        cli->print("  on     - Start CSV logging (20Hz)\r\n");
        cli->print("  off    - Stop CSV logging\r\n");
        cli->print("  header - Print CSV header\r\n");
        cli->print("Current: %s, samples: %lu\r\n",
                   cli->isLogEnabled() ? "on" : "off",
                   (unsigned long)cli->getLogCounter());
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        cli->resetLogCounter();
        cli->setLogEnabled(true);
        cli->print("CSV logging ON\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        cli->setLogEnabled(false);
        cli->print("CSV logging OFF, total samples: %lu\r\n",
                   (unsigned long)cli->getLogCounter());
    } else if (strcmp(argv[1], "header") == 0) {
        // Print CSV header for ESKF debugging
        cli->print("# StampFly Sensor Log for ESKF Debug\r\n");
        cli->print("# timestamp_ms,");
        cli->print("accel_x,accel_y,accel_z,");
        cli->print("gyro_x,gyro_y,gyro_z,");
        cli->print("mag_x,mag_y,mag_z,");
        cli->print("pressure_pa,baro_alt_m,");
        cli->print("tof_bottom_m,tof_front_m,");
        cli->print("flow_dx,flow_dy,flow_squal\r\n");
    } else {
        cli->print("Usage: log [on|off|header]\r\n");
    }
}

static void cmd_binlog(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_logger_ptr == nullptr) {
        cli->print("Logger not available\r\n");
        return;
    }

    if (argc < 2) {
        cli->print("Usage: binlog [on|off|freq <hz>]\r\n");
        cli->print("  on       - Start binary logging (400Hz, 128B/pkt, sensor + ESKF)\r\n");
        cli->print("  off      - Stop binary logging\r\n");
        cli->print("  freq <hz>- Set logging frequency (10-1000Hz)\r\n");
        cli->print("Current: %s, freq=%luHz, count=%lu, dropped=%lu\r\n",
                   g_logger_ptr->isRunning() ? "on" : "off",
                   (unsigned long)g_logger_ptr->getFrequency(),
                   (unsigned long)g_logger_ptr->getLogCount(),
                   (unsigned long)g_logger_ptr->getDropCount());
        cli->print("\r\nPacket format (128 bytes): Header 0xAA 0x56\r\n");
        cli->print("Contains: IMU, Mag, Baro, ToF, Flow, ESKF estimates\r\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        g_logger_ptr->resetCounters();
        esp_log_level_set("*", ESP_LOG_NONE);
        cli->print("Binary logging ON (%luHz, 128B) - ESP_LOG suppressed\r\n",
                   (unsigned long)g_logger_ptr->getFrequency());
        vTaskDelay(pdMS_TO_TICKS(100));
        g_logger_ptr->start();
    } else if (strcmp(argv[1], "off") == 0) {
        g_logger_ptr->stop();
        esp_log_level_set("*", ESP_LOG_INFO);
        cli->print("Binary logging OFF, total=%lu, dropped=%lu\r\n",
                   (unsigned long)g_logger_ptr->getLogCount(),
                   (unsigned long)g_logger_ptr->getDropCount());
    } else if (strcmp(argv[1], "freq") == 0 && argc >= 3) {
        uint32_t freq = atoi(argv[2]);
        if (g_logger_ptr->setFrequency(freq) == ESP_OK) {
            cli->print("Logging frequency set to %luHz\r\n", (unsigned long)freq);
        } else {
            cli->print("Invalid frequency (10-1000Hz)\r\n");
        }
    } else {
        cli->print("Usage: binlog [on|off|freq <hz>]\r\n");
    }
}

static void cmd_loglevel(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    // Current log level names
    static const char* level_names[] = {
        "none", "error", "warn", "info", "debug", "verbose"
    };

    if (argc < 2) {
        cli->print("Usage: loglevel [none|error|warn|info|debug|verbose] [tag]\r\n");
        cli->print("  none    - No log output\r\n");
        cli->print("  error   - Errors only\r\n");
        cli->print("  warn    - Warnings and errors\r\n");
        cli->print("  info    - Info, warnings, errors (default)\r\n");
        cli->print("  debug   - Debug and above\r\n");
        cli->print("  verbose - All messages\r\n");
        cli->print("  [tag]   - Optional: specific component (e.g., 'main', 'ESKF')\r\n");
        cli->print("            If omitted, applies to all components ('*')\r\n");
        return;
    }

    const char* level_str = argv[1];
    const char* tag = (argc >= 3) ? argv[2] : "*";  // Default to all tags

    esp_log_level_t level;
    if (strcmp(level_str, "none") == 0) {
        level = ESP_LOG_NONE;
    } else if (strcmp(level_str, "error") == 0) {
        level = ESP_LOG_ERROR;
    } else if (strcmp(level_str, "warn") == 0) {
        level = ESP_LOG_WARN;
    } else if (strcmp(level_str, "info") == 0) {
        level = ESP_LOG_INFO;
    } else if (strcmp(level_str, "debug") == 0) {
        level = ESP_LOG_DEBUG;
    } else if (strcmp(level_str, "verbose") == 0) {
        level = ESP_LOG_VERBOSE;
    } else {
        cli->print("Unknown log level: %s\r\n", level_str);
        cli->print("Available: none, error, warn, info, debug, verbose\r\n");
        return;
    }

    esp_log_level_set(tag, level);

    // Save to NVS if setting global log level
    if (strcmp(tag, "*") == 0) {
        if (saveLogLevelToNVS(level) == ESP_OK) {
            cli->print("Log level set to '%s' (saved to NVS)\r\n", level_names[level]);
        } else {
            cli->print("Log level set to '%s' (NVS save failed)\r\n", level_names[level]);
        }
    } else {
        cli->print("Log level set to '%s' for tag '%s'\r\n", level_names[level], tag);
    }
}

static void cmd_calib(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: calib [gyro|accel|mag]\r\n");
        return;
    }

    const char* type = argv[1];
    if (strcmp(type, "gyro") == 0) {
        cli->print("Starting gyro calibration...\r\n");
        cli->print("Keep device still for 3 seconds.\r\n");
        // TODO: SystemManager::runGyroCalibration()
        cli->print("Gyro calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "accel") == 0) {
        cli->print("Starting accelerometer calibration...\r\n");
        cli->print("Place device on flat surface.\r\n");
        // TODO: SystemManager::runAccelCalibration()
        cli->print("Accel calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "mag") == 0) {
        cli->print("Starting magnetometer calibration...\r\n");
        cli->print("Rotate device in all directions.\r\n");
        // TODO: SystemManager::runMagCalibration()
        cli->print("Mag calibration complete (stub)\r\n");
    }
    else {
        cli->print("Unknown calibration type: %s\r\n", type);
    }
}

// Global CLI pointer for magcal log callback
static CLI* g_magcal_cli = nullptr;

// Callback function for magnetometer calibration log messages
static void magcal_log_callback(const char* message)
{
    if (g_magcal_cli) {
        g_magcal_cli->print("%s\r\n", message);
    }
}

static void cmd_magcal(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    g_magcal_cli = cli;  // Set global for callback

    if (g_mag_calibrator == nullptr) {
        cli->print("Magnetometer calibrator not available\r\n");
        return;
    }

    // Set log callback so calibration messages appear on CLI regardless of ESP_LOG level
    g_mag_calibrator->setLogCallback(magcal_log_callback);

    if (argc < 2) {
        cli->print("Usage: magcal [start|stop|status|save|clear]\r\n");
        cli->print("  start  - Start calibration (rotate device in figure-8)\r\n");
        cli->print("  stop   - Stop calibration and compute result\r\n");
        cli->print("  status - Show calibration status\r\n");
        cli->print("  save   - Save calibration to NVS\r\n");
        cli->print("  clear  - Clear saved calibration\r\n");
        return;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "start") == 0) {
        if (g_mag_calibrator->getState() == MagCalibrator::State::COLLECTING) {
            cli->print("Calibration already in progress\r\n");
            return;
        }
        if (g_mag_calibrator->startCalibration() == ESP_OK) {
            cli->print("Magnetometer calibration started\r\n");
            cli->print("Slowly rotate device in all directions (figure-8 pattern)\r\n");
            cli->print("Need at least %d samples. Use 'magcal status' to check progress.\r\n",
                       MagCalibrator::MIN_SAMPLES);
            cli->print("Use 'magcal stop' when done.\r\n");
        } else {
            cli->print("Failed to start calibration\r\n");
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        auto state = g_mag_calibrator->getState();
        if (state != MagCalibrator::State::COLLECTING) {
            cli->print("No calibration in progress\r\n");
            return;
        }

        cli->print("Computing calibration from %d samples...\r\n",
                   g_mag_calibrator->getSampleCount());

        if (g_mag_calibrator->computeCalibration() == ESP_OK) {
            auto& cal = g_mag_calibrator->getCalibration();
            cli->print("\r\n=== Calibration Result ===\r\n");
            cli->print("Hard Iron Offset:\r\n");
            cli->print("  X: %.2f uT\r\n", cal.offset_x);
            cli->print("  Y: %.2f uT\r\n", cal.offset_y);
            cli->print("  Z: %.2f uT\r\n", cal.offset_z);
            cli->print("Soft Iron Scale:\r\n");
            cli->print("  X: %.3f\r\n", cal.scale_x);
            cli->print("  Y: %.3f\r\n", cal.scale_y);
            cli->print("  Z: %.3f\r\n", cal.scale_z);
            cli->print("Sphere Radius: %.2f uT\r\n", cal.sphere_radius);
            cli->print("Fitness: %.2f (higher is better)\r\n", cal.fitness);
            cli->print("\r\nUse 'magcal save' to persist to NVS\r\n");
        } else {
            cli->print("Calibration failed. Try again with more samples and better coverage.\r\n");
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        auto state = g_mag_calibrator->getState();
        const char* state_str = "Unknown";
        switch (state) {
            case MagCalibrator::State::IDLE: state_str = "Idle"; break;
            case MagCalibrator::State::COLLECTING: state_str = "Collecting"; break;
            case MagCalibrator::State::COMPUTING: state_str = "Computing"; break;
            case MagCalibrator::State::DONE: state_str = "Done"; break;
            case MagCalibrator::State::ERROR: state_str = "Error"; break;
        }
        cli->print("State: %s\r\n", state_str);
        cli->print("Samples: %d / %d\r\n",
                   g_mag_calibrator->getSampleCount(),
                   MagCalibrator::MIN_SAMPLES);
        cli->print("Progress: %.0f%%\r\n", g_mag_calibrator->getProgress() * 100.0f);

        if (g_mag_calibrator->isCalibrated()) {
            auto& cal = g_mag_calibrator->getCalibration();
            cli->print("Calibrated: Yes\r\n");
            cli->print("  Offset: [%.2f, %.2f, %.2f] uT\r\n",
                       cal.offset_x, cal.offset_y, cal.offset_z);
            cli->print("  Scale:  [%.3f, %.3f, %.3f]\r\n",
                       cal.scale_x, cal.scale_y, cal.scale_z);
        } else {
            cli->print("Calibrated: No\r\n");
        }

        // Show current mag reading
        // Note: StampFlyState stores calibrated data (if calibration is active)
        auto& mstate = StampFlyState::getInstance();
        Vec3 mag;
        mstate.getMagData(mag);

        if (g_mag_calibrator->isCalibrated()) {
            // Data in state is already calibrated
            cli->print("Calibrated Mag: [%.1f, %.1f, %.1f] uT, norm=%.1f\r\n",
                       mag.x, mag.y, mag.z,
                       sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z));
        } else {
            cli->print("Raw Mag: [%.1f, %.1f, %.1f] uT, norm=%.1f\r\n",
                       mag.x, mag.y, mag.z,
                       sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z));
        }
    }
    else if (strcmp(cmd, "save") == 0) {
        if (!g_mag_calibrator->isCalibrated()) {
            cli->print("No valid calibration to save\r\n");
            return;
        }
        if (g_mag_calibrator->saveToNVS() == ESP_OK) {
            cli->print("Calibration saved to NVS\r\n");
        } else {
            cli->print("Failed to save calibration\r\n");
        }
    }
    else if (strcmp(cmd, "clear") == 0) {
        if (g_mag_calibrator->clearNVS() == ESP_OK) {
            cli->print("Calibration cleared from NVS\r\n");
        } else {
            cli->print("Failed to clear calibration\r\n");
        }
    }
    else {
        cli->print("Unknown subcommand: %s\r\n", cmd);
        cli->print("Usage: magcal [start|stop|status|save|clear]\r\n");
    }
}

static void cmd_motor(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_motor_ptr == nullptr) {
        cli->print("Motor driver not available\r\n");
        return;
    }

    if (argc < 2) {
        cli->print("Usage: motor <command>\r\n");
        cli->print("  test <id> <throttle> - Test single motor (id:1-4, throttle:0-100)\r\n");
        cli->print("  all <throttle>       - Test all motors (throttle:0-100)\r\n");
        cli->print("  stop                 - Stop all motors\r\n");
        cli->print("  stats                - Show duty cycle statistics\r\n");
        cli->print("  stats_reset          - Reset statistics\r\n");
        cli->print("\r\n");
        cli->print("Motor layout (top view):\r\n");
        cli->print("       Front\r\n");
        cli->print("   M4(FL)  M1(FR)\r\n");
        cli->print("       X\r\n");
        cli->print("   M3(RL)  M2(RR)\r\n");
        cli->print("        Rear\r\n");
        return;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "stop") == 0) {
        // Use testMotor to bypass arm check
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FR, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RR, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RL, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FL, 0);
        cli->print("All motors stopped\r\n");
    }
    else if (strcmp(cmd, "all") == 0) {
        if (argc < 3) {
            cli->print("Usage: motor all <throttle>\r\n");
            return;
        }
        int throttle = atoi(argv[2]);
        if (throttle < 0 || throttle > 100) {
            cli->print("Invalid throttle. Use 0-100.\r\n");
            return;
        }
        // Use testMotor to bypass arm check
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FR, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RR, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RL, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FL, throttle);
        cli->print("All motors at %d%%\r\n", throttle);
    }
    else if (strcmp(cmd, "test") == 0) {
        if (argc < 4) {
            cli->print("Usage: motor test <id> <throttle>\r\n");
            cli->print("  id: 1=FR, 2=RR, 3=RL, 4=FL\r\n");
            return;
        }
        int id = atoi(argv[2]);
        int throttle = atoi(argv[3]);
        if (id < 1 || id > 4) {
            cli->print("Invalid motor ID. Use 1-4.\r\n");
            return;
        }
        if (throttle < 0 || throttle > 100) {
            cli->print("Invalid throttle. Use 0-100.\r\n");
            return;
        }

        const char* motor_names[] = {"FR (M1)", "RR (M2)", "RL (M3)", "FL (M4)"};
        g_motor_ptr->testMotor(id - 1, throttle);
        cli->print("Motor %s at %d%%\r\n", motor_names[id - 1], throttle);
    }
    else if (strcmp(cmd, "stats") == 0) {
        // Load last flight stats from NVS and display
        g_motor_ptr->loadStatsFromNVS();

        const char* names[] = {"M1(FR)", "M2(RR)", "M3(RL)", "M4(FL)"};
        const char* types[] = {"CCW", "CW ", "CCW", "CW "};
        cli->print("Last Flight Motor Statistics:\r\n");
        cli->print("Motor Type   Avg     Min     Max     Count\r\n");
        cli->print("------------------------------------------\r\n");
        for (int i = 0; i < 4; i++) {
            auto stats = g_motor_ptr->getLastFlightStats(i);
            float avg = (stats.count > 0) ? (stats.sum / stats.count) : 0.0f;
            cli->print("%s  %s  %.3f   %.3f   %.3f   %lu\r\n",
                       names[i], types[i], avg, stats.min, stats.max, stats.count);
        }
        // CCW vs CW comparison
        auto s0 = g_motor_ptr->getLastFlightStats(0);
        auto s1 = g_motor_ptr->getLastFlightStats(1);
        auto s2 = g_motor_ptr->getLastFlightStats(2);
        auto s3 = g_motor_ptr->getLastFlightStats(3);
        float ccw_avg = 0, cw_avg = 0;
        uint32_t ccw_count = s0.count + s2.count;
        uint32_t cw_count = s1.count + s3.count;
        if (ccw_count > 0) ccw_avg = (s0.sum + s2.sum) / ccw_count;
        if (cw_count > 0) cw_avg = (s1.sum + s3.sum) / cw_count;
        cli->print("------------------------------------------\r\n");
        cli->print("CCW avg: %.3f, CW avg: %.3f, diff: %.3f\r\n",
                   ccw_avg, cw_avg, ccw_avg - cw_avg);
    }
    else if (strcmp(cmd, "stats_reset") == 0) {
        g_motor_ptr->resetStats();
        cli->print("Motor statistics reset\r\n");
    }
    else {
        cli->print("Unknown motor command: %s\r\n", cmd);
    }
}

static void cmd_pair(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_comm_ptr == nullptr) {
        cli->print("ControllerComm not available\r\n");
        return;
    }

    // Show current status
    if (argc < 2) {
        cli->print("ESP-NOW Pairing Status:\r\n");
        cli->print("  Initialized: %s\r\n", g_comm_ptr->isInitialized() ? "yes" : "no");
        cli->print("  Channel: %d\r\n", g_comm_ptr->getChannel());
        cli->print("  Paired: %s\r\n", g_comm_ptr->isPaired() ? "yes" : "no");
        cli->print("  Connected: %s\r\n", g_comm_ptr->isConnected() ? "yes" : "no");
        cli->print("  Pairing mode: %s\r\n", g_comm_ptr->isPairingMode() ? "active" : "inactive");
        cli->print("\r\nUsage:\r\n");
        cli->print("  pair start      - Enter pairing mode\r\n");
        cli->print("  pair stop       - Exit pairing mode\r\n");
        cli->print("  pair channel <n>- Set WiFi channel (1-13)\r\n");
        return;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "start") == 0) {
        if (g_comm_ptr->isPairingMode()) {
            cli->print("Already in pairing mode\r\n");
            return;
        }
        cli->print("Entering pairing mode on channel %d...\r\n", g_comm_ptr->getChannel());
        cli->print("Send control packet from controller to complete pairing.\r\n");
        g_comm_ptr->enterPairingMode();
    } else if (strcmp(cmd, "stop") == 0) {
        if (!g_comm_ptr->isPairingMode()) {
            cli->print("Not in pairing mode\r\n");
            return;
        }
        cli->print("Exiting pairing mode\r\n");
        g_comm_ptr->exitPairingMode();
    } else if (strcmp(cmd, "channel") == 0) {
        if (argc < 3) {
            cli->print("Current channel: %d\r\n", g_comm_ptr->getChannel());
            cli->print("Usage: pair channel <1-13>\r\n");
            return;
        }
        int channel = atoi(argv[2]);
        if (channel < 1 || channel > 13) {
            cli->print("Invalid channel. Use 1-13.\r\n");
            return;
        }
        esp_err_t ret = g_comm_ptr->setChannel(channel);
        if (ret == ESP_OK) {
            cli->print("WiFi channel set to %d\r\n", channel);
        } else {
            cli->print("Failed to set channel: %s\r\n", esp_err_to_name(ret));
        }
    } else {
        cli->print("Unknown subcommand: %s\r\n", cmd);
        cli->print("Usage: pair [start|stop|channel]\r\n");
    }
}

static void cmd_unpair(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_comm_ptr == nullptr) {
        cli->print("ControllerComm not available\r\n");
        return;
    }

    if (!g_comm_ptr->isPaired()) {
        cli->print("Not paired to any controller\r\n");
        return;
    }

    cli->print("Clearing pairing information...\r\n");
    esp_err_t ret = g_comm_ptr->clearPairingFromNVS();
    if (ret == ESP_OK) {
        cli->print("Pairing cleared successfully\r\n");
    } else {
        cli->print("Failed to clear pairing: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_reset(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Resetting system...\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

static void cmd_gain(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_rate_controller_ptr == nullptr) {
        cli->print("Rate controller not available\r\n");
        return;
    }

    auto& rc = *g_rate_controller_ptr;

    // Show current gains
    if (argc < 2) {
        cli->print("=== Rate Control Gains ===\r\n");
        cli->print("Sensitivity (max rate [rad/s]):\r\n");
        cli->print("  roll_max:  %.2f\r\n", rc.roll_rate_max);
        cli->print("  pitch_max: %.2f\r\n", rc.pitch_rate_max);
        cli->print("  yaw_max:   %.2f\r\n", rc.yaw_rate_max);
        cli->print("\r\nRoll PID:\r\n");
        cli->print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                   rc.roll_pid.getKp(), rc.roll_pid.getTi(), rc.roll_pid.getTd());
        cli->print("Pitch PID:\r\n");
        cli->print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                   rc.pitch_pid.getKp(), rc.pitch_pid.getTi(), rc.pitch_pid.getTd());
        cli->print("Yaw PID:\r\n");
        cli->print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                   rc.yaw_pid.getKp(), rc.yaw_pid.getTi(), rc.yaw_pid.getTd());
        cli->print("\r\nUsage: gain <axis> <param> <value>\r\n");
        cli->print("  axis:  roll, pitch, yaw\r\n");
        cli->print("  param: kp, ti, td, max\r\n");
        cli->print("Example: gain roll kp 0.2\r\n");
        return;
    }

    if (argc < 4) {
        cli->print("Usage: gain <axis> <param> <value>\r\n");
        cli->print("  axis:  roll, pitch, yaw\r\n");
        cli->print("  param: kp, ti, td, max\r\n");
        return;
    }

    const char* axis = argv[1];
    const char* param = argv[2];
    float value = atof(argv[3]);

    // Select PID by axis
    PID* pid = nullptr;
    float* rate_max = nullptr;

    if (strcmp(axis, "roll") == 0) {
        pid = &rc.roll_pid;
        rate_max = &rc.roll_rate_max;
    } else if (strcmp(axis, "pitch") == 0) {
        pid = &rc.pitch_pid;
        rate_max = &rc.pitch_rate_max;
    } else if (strcmp(axis, "yaw") == 0) {
        pid = &rc.yaw_pid;
        rate_max = &rc.yaw_rate_max;
    } else {
        cli->print("Unknown axis: %s (use roll, pitch, yaw)\r\n", axis);
        return;
    }

    // Set parameter
    if (strcmp(param, "kp") == 0) {
        pid->setKp(value);
        cli->print("%s Kp = %.4f\r\n", axis, value);
    } else if (strcmp(param, "ti") == 0) {
        pid->setTi(value);
        cli->print("%s Ti = %.4f\r\n", axis, value);
    } else if (strcmp(param, "td") == 0) {
        pid->setTd(value);
        cli->print("%s Td = %.5f\r\n", axis, value);
    } else if (strcmp(param, "max") == 0) {
        if (rate_max) {
            *rate_max = value;
            cli->print("%s rate_max = %.2f [rad/s]\r\n", axis, value);
        }
    } else {
        cli->print("Unknown param: %s (use kp, ti, td, max)\r\n", param);
    }
}

static void cmd_attitude(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Attitude (stub):\r\n");
    cli->print("  Roll:  0.00 [deg]\r\n");
    cli->print("  Pitch: 0.00 [deg]\r\n");
    cli->print("  Yaw:   0.00 [deg]\r\n");
    // TODO: Get from StampFlyState
}

static void cmd_version(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("StampFly RTOS Skeleton\r\n");
    cli->print("  ESP-IDF: %s\r\n", esp_get_idf_version());
    cli->print("  Chip: ESP32-S3\r\n");
}

static void cmd_ctrl(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& state = StampFlyState::getInstance();

    // Parse duration for watch mode (default 10 seconds)
    int watch_seconds = 0;
    if (argc >= 2) {
        if (strcmp(argv[1], "watch") == 0) {
            watch_seconds = (argc >= 3) ? atoi(argv[2]) : 10;
            if (watch_seconds < 1) watch_seconds = 10;
            if (watch_seconds > 60) watch_seconds = 60;
        }
    }

    if (watch_seconds > 0) {
        cli->print("Controller input (%d sec):\r\n", watch_seconds);
    }

    int iterations = watch_seconds * 10;  // 10Hz
    do {
        uint16_t throttle, roll, pitch, yaw;
        state.getRawControlInput(throttle, roll, pitch, yaw);
        uint8_t flags = state.getControlFlags();

        // Check connection status
        const char* conn_status = "disconnected";
        if (g_comm_ptr != nullptr && g_comm_ptr->isConnected()) {
            conn_status = "connected";
        }

        if (watch_seconds > 0) {
            // Overwrite line with \r
            // Flags: A=Arm, F=Flip, M=Mode, H=AltMode(Hold)
            cli->print("\rT:%4u R:%4u P:%4u Y:%4u [%c%c%c%c] [%s]   ",
                       throttle, roll, pitch, yaw,
                       (flags & CTRL_FLAG_ARM) ? 'A' : '-',
                       (flags & CTRL_FLAG_FLIP) ? 'F' : '-',
                       (flags & CTRL_FLAG_MODE) ? 'M' : '-',
                       (flags & CTRL_FLAG_ALT_MODE) ? 'H' : '-',
                       conn_status);
            vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update
            iterations--;
        } else {
            cli->print("Controller Input (raw ADC values):\r\n");
            cli->print("  Throttle: %u (0-4095)\r\n", throttle);
            cli->print("  Roll:     %u (2048=center)\r\n", roll);
            cli->print("  Pitch:    %u (2048=center)\r\n", pitch);
            cli->print("  Yaw:      %u (2048=center)\r\n", yaw);
            cli->print("  Flags:    0x%02X\r\n", flags);
            cli->print("    ARM:      %s\r\n", (flags & CTRL_FLAG_ARM) ? "ON" : "OFF");
            cli->print("    FLIP:     %s\r\n", (flags & CTRL_FLAG_FLIP) ? "ON" : "OFF");
            cli->print("    MODE:     %s\r\n", (flags & CTRL_FLAG_MODE) ? "ON" : "OFF");
            cli->print("    ALT_MODE: %s\r\n", (flags & CTRL_FLAG_ALT_MODE) ? "ON" : "OFF");
            cli->print("  Status:   %s\r\n", conn_status);
        }
    } while (iterations > 0);

    if (watch_seconds > 0) {
        cli->print("\r\n");
    }
}

static void cmd_debug(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& state = StampFlyState::getInstance();

    if (argc < 2) {
        cli->print("Debug mode: %s\r\n", state.isDebugMode() ? "ON" : "OFF");
        cli->print("Usage: debug [on|off]\r\n");
        cli->print("  When ON, ARM ignores errors (LOW_BATTERY, sensors, etc.)\r\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        state.setDebugMode(true);
        cli->print("Debug mode ON - errors will be ignored for ARM\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        state.setDebugMode(false);
        cli->print("Debug mode OFF\r\n");
    } else {
        cli->print("Usage: debug [on|off]\r\n");
    }
}

static void cmd_led(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& led_mgr = stampfly::LEDManager::getInstance();

    if (argc < 2) {
        cli->print("LED brightness: %d (0-255)\r\n", led_mgr.getBrightness());
        cli->print("Usage: led brightness <0-255>\r\n");
        return;
    }

    if (strcmp(argv[1], "brightness") == 0) {
        if (argc < 3) {
            cli->print("Current brightness: %d\r\n", led_mgr.getBrightness());
            cli->print("Usage: led brightness <0-255>\r\n");
            return;
        }
        int brightness = atoi(argv[2]);
        if (brightness < 0 || brightness > 255) {
            cli->print("Invalid brightness. Use 0-255.\r\n");
            return;
        }
        led_mgr.setBrightness(static_cast<uint8_t>(brightness), true);  // save to NVS
        cli->print("LED brightness set to %d (saved)\r\n", brightness);
    } else {
        cli->print("Usage: led brightness <0-255>\r\n");
    }
}

static void cmd_sound(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_buzzer_ptr == nullptr) {
        cli->print("Buzzer not available\r\n");
        return;
    }

    if (argc < 2) {
        cli->print("Sound: %s\r\n", g_buzzer_ptr->isMuted() ? "OFF" : "ON");
        cli->print("Usage: sound [on|off]\r\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        g_buzzer_ptr->setMuted(false, true);  // save to NVS
        cli->print("Sound ON (saved)\r\n");
        g_buzzer_ptr->beep();  // Play confirmation beep
    } else if (strcmp(argv[1], "off") == 0) {
        g_buzzer_ptr->setMuted(true, true);  // save to NVS
        cli->print("Sound OFF (saved)\r\n");
    } else {
        cli->print("Usage: sound [on|off]\r\n");
    }
}

static void cmd_pos(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (g_fusion_ptr == nullptr) {
        cli->print("Sensor fusion not available\r\n");
        return;
    }

    if (argc < 2) {
        // Show current position
        auto state = g_fusion_ptr->getState();
        cli->print("Position [m]: X=%.3f Y=%.3f Z=%.3f\r\n",
                   state.position.x, state.position.y, state.position.z);
        cli->print("Velocity [m/s]: X=%.3f Y=%.3f Z=%.3f\r\n",
                   state.velocity.x, state.velocity.y, state.velocity.z);
        cli->print("Usage: pos [reset|status]\r\n");
        return;
    }

    if (strcmp(argv[1], "reset") == 0) {
        g_fusion_ptr->resetPositionVelocity();
        cli->print("Position/Velocity reset to origin\r\n");
    } else if (strcmp(argv[1], "status") == 0) {
        auto state = g_fusion_ptr->getState();
        cli->print("=== Position Status ===\r\n");
        cli->print("Position [m]: X=%.3f Y=%.3f Z=%.3f\r\n",
                   state.position.x, state.position.y, state.position.z);
        cli->print("Velocity [m/s]: X=%.3f Y=%.3f Z=%.3f\r\n",
                   state.velocity.x, state.velocity.y, state.velocity.z);
        cli->print("Diverged: %s\r\n", g_fusion_ptr->isDiverged() ? "YES" : "NO");
    } else {
        cli->print("Usage: pos [reset|status]\r\n");
    }
}

void CLI::outputTeleplot()
{
    if (!teleplot_enabled_) return;

    auto& state = StampFlyState::getInstance();

    // バッファにまとめて出力（シリアル出力のブロッキングを最小化）
    static char buf[1024];
    int len = 0;

    // すべてのデータを一度に取得してからフォーマット
    Vec3 accel, gyro, mag;
    float altitude, pressure;
    float tof_bottom, tof_front;
    float flow_vx, flow_vy;
    float voltage, current;
    float roll, pitch, yaw;
    StateVector3 pos, vel;

    // データ取得（各関数内でmutexを使用）
    state.getIMUData(accel, gyro);
    state.getMagData(mag);
    state.getBaroData(altitude, pressure);
    state.getToFData(tof_bottom, tof_front);
    state.getFlowData(flow_vx, flow_vy);
    state.getPowerData(voltage, current);
    state.getAttitudeEuler(roll, pitch, yaw);
    pos = state.getPosition();
    vel = state.getVelocity();

    // バッファにフォーマット（IMU + 姿勢のみ、主要データに絞る）
    len += snprintf(buf + len, sizeof(buf) - len,
        ">accel_x:%.3f\r\n>accel_y:%.3f\r\n>accel_z:%.3f\r\n",
        accel.x, accel.y, accel.z);
    len += snprintf(buf + len, sizeof(buf) - len,
        ">gyro_x:%.4f\r\n>gyro_y:%.4f\r\n>gyro_z:%.4f\r\n",
        gyro.x, gyro.y, gyro.z);
    len += snprintf(buf + len, sizeof(buf) - len,
        ">roll:%.4f\r\n>pitch:%.4f\r\n>yaw:%.4f\r\n",
        roll, pitch, yaw);
    len += snprintf(buf + len, sizeof(buf) - len,
        ">tof_bottom:%.3f\r\n>altitude:%.3f\r\n",
        tof_bottom, altitude);
    len += snprintf(buf + len, sizeof(buf) - len,
        ">pos_z:%.4f\r\n>vel_z:%.4f\r\n",
        pos.z, vel.z);

    // 一度に出力
    if (len > 0) {
        write(STDOUT_FILENO, buf, len);
    }
}

void CLI::outputCSVLog()
{
    if (!log_enabled_) return;

    auto& state = StampFlyState::getInstance();

    // Get timestamp (milliseconds since boot)
    uint32_t timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // IMU data
    Vec3 accel, gyro;
    state.getIMUData(accel, gyro);

    // Mag data
    Vec3 mag;
    state.getMagData(mag);

    // Baro data
    float baro_alt, pressure;
    state.getBaroData(baro_alt, pressure);

    // ToF data
    float tof_bottom, tof_front;
    state.getToFData(tof_bottom, tof_front);

    // OptFlow raw data (dx, dy, squal)
    int16_t flow_dx, flow_dy;
    uint8_t flow_squal;
    state.getFlowRawData(flow_dx, flow_dy, flow_squal);

    // Output CSV line (one line per sample)
    print("%lu,", (unsigned long)timestamp_ms);
    print("%.4f,%.4f,%.4f,", accel.x, accel.y, accel.z);
    print("%.5f,%.5f,%.5f,", gyro.x, gyro.y, gyro.z);
    print("%.2f,%.2f,%.2f,", mag.x, mag.y, mag.z);
    print("%.1f,%.4f,", pressure, baro_alt);
    print("%.4f,%.4f,", tof_bottom, tof_front);
    print("%d,%d,%u\r\n", flow_dx, flow_dy, flow_squal);

    log_counter_++;
}

// ==========================================================================
// V1 Binary Log Output - DEPRECATED (commented out for future removal)
// ==========================================================================
#if 0
void CLI::outputBinaryLog()
{
    if (!binlog_enabled_) return;

    auto& state = StampFlyState::getInstance();

    BinaryLogPacket pkt;

    // Header
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;

    // Timestamp
    pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // IMU data
    Vec3 accel, gyro;
    state.getIMUData(accel, gyro);
    pkt.accel_x = accel.x;
    pkt.accel_y = accel.y;
    pkt.accel_z = accel.z;
    pkt.gyro_x = gyro.x;
    pkt.gyro_y = gyro.y;
    pkt.gyro_z = gyro.z;

    // Mag data
    Vec3 mag;
    state.getMagData(mag);
    pkt.mag_x = mag.x;
    pkt.mag_y = mag.y;
    pkt.mag_z = mag.z;

    // Baro data
    float baro_alt, pressure;
    state.getBaroData(baro_alt, pressure);
    pkt.pressure = pressure;
    pkt.baro_alt = baro_alt;

    // ToF data
    float tof_bottom, tof_front;
    state.getToFData(tof_bottom, tof_front);
    pkt.tof_bottom = tof_bottom;
    pkt.tof_front = tof_front;

    // OptFlow raw data
    int16_t flow_dx, flow_dy;
    uint8_t flow_squal;
    state.getFlowRawData(flow_dx, flow_dy, flow_squal);
    pkt.flow_dx = flow_dx;
    pkt.flow_dy = flow_dy;
    pkt.flow_squal = flow_squal;

    // Calculate checksum (XOR of bytes 2-62)
    uint8_t* data = reinterpret_cast<uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 63; i++) {
        checksum ^= data[i];
    }
    pkt.checksum = checksum;

    // Write binary packet using raw write()
    int fd = fileno(stdout);
    write(fd, data, sizeof(pkt));

    binlog_counter_++;
}
#endif

void CLI::outputBinaryLogV2()
{
    if (!binlog_v2_enabled_) return;

    auto& state = StampFlyState::getInstance();

    BinaryLogPacketV2 pkt;

    // Header (V2: 0xAA, 0x56)
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x56;

    // Timestamp
    pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // IMU data
    Vec3 accel, gyro;
    state.getIMUData(accel, gyro);
    pkt.accel_x = accel.x;
    pkt.accel_y = accel.y;
    pkt.accel_z = accel.z;
    pkt.gyro_x = gyro.x;
    pkt.gyro_y = gyro.y;
    pkt.gyro_z = gyro.z;

    // Mag data
    Vec3 mag;
    state.getMagData(mag);
    pkt.mag_x = mag.x;
    pkt.mag_y = mag.y;
    pkt.mag_z = mag.z;

    // Baro data
    float baro_alt, pressure;
    state.getBaroData(baro_alt, pressure);
    pkt.pressure = pressure;
    pkt.baro_alt = baro_alt;

    // ToF data
    float tof_bottom, tof_front;
    state.getToFData(tof_bottom, tof_front);
    pkt.tof_bottom = tof_bottom;
    pkt.tof_front = tof_front;

    // OptFlow raw data
    int16_t flow_dx, flow_dy;
    uint8_t flow_squal;
    state.getFlowRawData(flow_dx, flow_dy, flow_squal);
    pkt.flow_dx = flow_dx;
    pkt.flow_dy = flow_dy;
    pkt.flow_squal = flow_squal;

    // === ESKF Estimates ===
    // Position
    StateVector3 pos = state.getPosition();
    pkt.pos_x = pos.x;
    pkt.pos_y = pos.y;
    pkt.pos_z = pos.z;

    // Velocity
    StateVector3 vel = state.getVelocity();
    pkt.vel_x = vel.x;
    pkt.vel_y = vel.y;
    pkt.vel_z = vel.z;

    // Attitude
    float roll, pitch, yaw;
    state.getAttitudeEuler(roll, pitch, yaw);
    pkt.roll = roll;
    pkt.pitch = pitch;
    pkt.yaw = yaw;

    // Biases (only most important ones to save space)
    StateVector3 gyro_bias = state.getGyroBias();
    StateVector3 accel_bias = state.getAccelBias();
    pkt.gyro_bias_z = gyro_bias.z;      // Yaw bias (most important)
    pkt.accel_bias_x = accel_bias.x;
    pkt.accel_bias_y = accel_bias.y;

    // Status and metadata
    pkt.eskf_status = state.isESKFInitialized() ? 1 : 0;
    pkt.baro_ref_alt = state.getBaroReferenceAltitude();
    memset(pkt.reserved, 0, sizeof(pkt.reserved));

    // Calculate checksum (XOR of bytes 2-126)
    uint8_t* data = reinterpret_cast<uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 127; i++) {
        checksum ^= data[i];
    }
    pkt.checksum = checksum;

    // Write binary packet using raw write()
    int fd = fileno(stdout);
    write(fd, data, sizeof(pkt));

    binlog_counter_++;
}

}  // namespace stampfly
