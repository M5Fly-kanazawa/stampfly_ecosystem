/**
 * @file cmd_sensor.cpp
 * @brief Sensor commands (sensor, teleplot, log, binlog, loglevel)
 *
 * センサーコマンド
 */

#include "console.hpp"
#include "stampfly_state.hpp"
#include "logger.hpp"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cmath>

// External references
// 外部参照
extern stampfly::Logger* g_logger_ptr;

// NVS namespace and keys
static const char* NVS_NAMESPACE_CLI = "stampfly_cli";
static const char* NVS_KEY_LOGLEVEL = "loglevel";

namespace stampfly {

// =============================================================================
// sensor command
// =============================================================================

static int cmd_sensor(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    if (argc < 2) {
        console.print("Usage: sensor [imu|mag|baro|tof|flow|power|all]\r\n");
        return 1;
    }

    const char* sensor = argv[1];
    bool found = false;

    if (strcmp(sensor, "imu") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 accel, gyro;
        state.getIMUData(accel, gyro);
        console.print("IMU:\r\n");
        console.print("  Accel: X=%.2f, Y=%.2f, Z=%.2f [m/s^2]\r\n", accel.x, accel.y, accel.z);
        console.print("  Gyro:  X=%.3f, Y=%.3f, Z=%.3f [rad/s]\r\n", gyro.x, gyro.y, gyro.z);
        found = true;
    }
    if (strcmp(sensor, "mag") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 mag;
        state.getMagData(mag);
        console.print("Mag: X=%.1f, Y=%.1f, Z=%.1f [uT]\r\n", mag.x, mag.y, mag.z);
        found = true;
    }
    if (strcmp(sensor, "baro") == 0 || strcmp(sensor, "all") == 0) {
        float altitude, pressure;
        state.getBaroData(altitude, pressure);
        console.print("Baro: Pressure=%.0f [Pa], Alt=%.2f [m]\r\n", pressure, altitude);
        found = true;
    }
    if (strcmp(sensor, "tof") == 0 || strcmp(sensor, "all") == 0) {
        float bottom, front;
        state.getToFData(bottom, front);
        console.print("ToF: Bottom=%.3f [m], Front=%.3f [m]\r\n", bottom, front);
        found = true;
    }
    if (strcmp(sensor, "flow") == 0 || strcmp(sensor, "all") == 0) {
        float vx, vy;
        state.getFlowData(vx, vy);
        console.print("OptFlow: Vx=%.3f, Vy=%.3f [m/s]\r\n", vx, vy);
        found = true;
    }
    if (strcmp(sensor, "power") == 0 || strcmp(sensor, "all") == 0) {
        float voltage, current;
        state.getPowerData(voltage, current);
        console.print("Power: %.2f [V], %.1f [mA]\r\n", voltage, current * 1000.0f);
        found = true;
    }

    if (strcmp(sensor, "diag") == 0) {
        console.print("=== Sensor Diagnostics ===\r\n");
        console.print("  %-5s %s %7s %7s %7s %7s %7s %7s\r\n",
                       "Name", "OK", "AvgHz", "MinHz", "MaxHz", "StdMs", "Loops", "MinMs");
        console.print("  %-5s %s %7s %7s %7s %7s %7s %7s\r\n",
                       "-----", "--", "------", "------", "------", "------", "------", "------");

        const char* names[] = {"imu", "flow", "tof_b", "tof_f", "baro", "mag"};
        for (const char* name : names) {
            auto d = state.getSensorDiag(name);
            if (d.period_count == 0) {
                console.print("  %-5s  %d     ---     ---     ---     ---  %6lu     ---\r\n",
                               name, d.healthy, (unsigned long)d.loop_count);
                continue;
            }
            float avg_us = (float)d.period_sum_us / d.period_count;
            float avg_hz = avg_us > 0 ? 1e6f / avg_us : 0;
            float min_hz = d.period_max_us > 0 ? 1e6f / d.period_max_us : 0;
            float max_hz = d.period_min_us > 0 ? 1e6f / d.period_min_us : 0;
            float min_ms = d.period_min_us / 1000.0f;
            // Variance = E[x²] - E[x]²
            float mean_sq = (float)d.period_sq_sum / d.period_count;
            float variance = mean_sq - avg_us * avg_us;
            float std_ms = variance > 0 ? sqrtf(variance) / 1000.0f : 0;
            console.print("  %-5s  %d  %5.1f  %5.1f  %5.1f  %5.2f  %6lu  %5.1f\r\n",
                           name, d.healthy, avg_hz, min_hz, max_hz, std_ms,
                           (unsigned long)d.loop_count, min_ms);
        }

        console.print("\r\nESKF: init=%d\r\n", state.isESKFInitialized());
        found = true;
    }

    if (!found) {
        console.print("Unknown sensor: %s\r\n", sensor);
        console.print("Available: imu, mag, baro, tof, flow, power, all, diag\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// loglevel command
// =============================================================================

// Helper: Load log level from NVS
static esp_log_level_t loadLogLevelFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CLI, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ESP_LOG_NONE;
    }

    uint8_t level_val = ESP_LOG_NONE;
    ret = nvs_get_u8(handle, NVS_KEY_LOGLEVEL, &level_val);
    nvs_close(handle);

    if (ret != ESP_OK || level_val > ESP_LOG_VERBOSE) {
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
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_LOGLEVEL, static_cast<uint8_t>(level));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}

static int cmd_loglevel(int argc, char** argv)
{
    auto& console = Console::getInstance();

    static const char* level_names[] = {
        "none", "error", "warn", "info", "debug", "verbose"
    };

    if (argc < 2) {
        console.print("Usage: loglevel [none|error|warn|info|debug|verbose] [tag]\r\n");
        console.print("  none    - No log output\r\n");
        console.print("  error   - Errors only\r\n");
        console.print("  warn    - Warnings and errors\r\n");
        console.print("  info    - Info, warnings, errors (default)\r\n");
        console.print("  debug   - Debug and above\r\n");
        console.print("  verbose - All messages\r\n");
        console.print("  [tag]   - Optional: specific component\r\n");
        return 0;
    }

    const char* level_str = argv[1];
    const char* tag = (argc >= 3) ? argv[2] : "*";

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
        console.print("Unknown log level: %s\r\n", level_str);
        return 1;
    }

    esp_log_level_set(tag, level);

    if (strcmp(tag, "*") == 0) {
        if (saveLogLevelToNVS(level) == ESP_OK) {
            console.print("Log level set to '%s' (saved to NVS)\r\n", level_names[level]);
        } else {
            console.print("Log level set to '%s' (NVS save failed)\r\n", level_names[level]);
        }
    } else {
        console.print("Log level set to '%s' for tag '%s'\r\n", level_names[level], tag);
    }

    return 0;
}

// =============================================================================
// binlog command
// =============================================================================

static int cmd_binlog(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (g_logger_ptr == nullptr) {
        console.print("Logger not available\r\n");
        return 1;
    }

    if (argc < 2) {
        console.print("Usage: binlog [on|off|freq <hz>]\r\n");
        console.print("  on       - Start binary logging (400Hz)\r\n");
        console.print("  off      - Stop binary logging\r\n");
        console.print("  freq <hz>- Set logging frequency (10-1000Hz)\r\n");
        console.print("Current: %s, freq=%luHz, count=%lu, dropped=%lu\r\n",
                      g_logger_ptr->isRunning() ? "on" : "off",
                      (unsigned long)g_logger_ptr->getFrequency(),
                      (unsigned long)g_logger_ptr->getLogCount(),
                      (unsigned long)g_logger_ptr->getDropCount());
        return 0;
    }

    if (strcmp(argv[1], "on") == 0) {
        g_logger_ptr->resetCounters();
        esp_log_level_set("*", ESP_LOG_NONE);
        console.print("Binary logging ON (%luHz) - ESP_LOG suppressed\r\n",
                      (unsigned long)g_logger_ptr->getFrequency());
        vTaskDelay(pdMS_TO_TICKS(100));
        g_logger_ptr->start();
    } else if (strcmp(argv[1], "off") == 0) {
        g_logger_ptr->stop();
        esp_log_level_set("*", ESP_LOG_INFO);
        console.print("Binary logging OFF, total=%lu, dropped=%lu\r\n",
                      (unsigned long)g_logger_ptr->getLogCount(),
                      (unsigned long)g_logger_ptr->getDropCount());
    } else if (strcmp(argv[1], "freq") == 0 && argc >= 3) {
        uint32_t freq = atoi(argv[2]);
        if (g_logger_ptr->setFrequency(freq) == ESP_OK) {
            console.print("Logging frequency set to %luHz\r\n", (unsigned long)freq);
        } else {
            console.print("Invalid frequency (10-1000Hz)\r\n");
        }
    } else {
        console.print("Usage: binlog [on|off|freq <hz>]\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_sensor_commands()
{
    // Load log level from NVS at startup
    // 起動時に NVS からログレベルを読み込む
    esp_log_level_t saved_level = loadLogLevelFromNVS();
    esp_log_level_set("*", saved_level);
    ESP_LOGI("SensorCmds", "Log level loaded from NVS: %d", saved_level);

    // sensor
    const esp_console_cmd_t sensor_cmd = {
        .command = "sensor",
        .help = "Show sensor data [imu|mag|baro|tof|flow|power|all]",
        .hint = NULL,
        .func = &cmd_sensor,
        .argtable = NULL,
    };
    esp_console_cmd_register(&sensor_cmd);

    // loglevel
    const esp_console_cmd_t loglevel_cmd = {
        .command = "loglevel",
        .help = "Set ESP_LOG level [none|error|warn|info|debug|verbose]",
        .hint = NULL,
        .func = &cmd_loglevel,
        .argtable = NULL,
    };
    esp_console_cmd_register(&loglevel_cmd);

    // binlog
    const esp_console_cmd_t binlog_cmd = {
        .command = "binlog",
        .help = "Binary log [on|off|freq]",
        .hint = NULL,
        .func = &cmd_binlog,
        .argtable = NULL,
    };
    esp_console_cmd_register(&binlog_cmd);
}

}  // namespace stampfly
