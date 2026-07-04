/**
 * @file cmd_control.cpp
 * @brief Control commands (trim, gain)
 *
 * 制御コマンド（トリム、ゲイン調整）
 */

#include "console.hpp"
#include "rate_controller.hpp"
#include "esp_console.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cstdlib>

// External references
// 外部参照
extern RateController* g_rate_controller_ptr;

// Trim values (accessed from control_task.cpp)
// トリム値（control_task.cppからアクセスされる）
float g_trim_roll = 0.0f;
float g_trim_pitch = 0.0f;
float g_trim_yaw = 0.0f;

// NVS keys
static const char* NVS_NAMESPACE_CLI = "stampfly_cli";
static const char* NVS_KEY_TRIM_ROLL = "trim_roll";
static const char* NVS_KEY_TRIM_PITCH = "trim_pitch";
static const char* NVS_KEY_TRIM_YAW = "trim_yaw";

namespace stampfly {

// =============================================================================
// NVS Helper Functions
// =============================================================================

static esp_err_t loadTrimFromNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CLI, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;  // NVS not initialized or namespace not found
    }

    int32_t roll_val = 0, pitch_val = 0, yaw_val = 0;
    nvs_get_i32(handle, NVS_KEY_TRIM_ROLL, &roll_val);
    nvs_get_i32(handle, NVS_KEY_TRIM_PITCH, &pitch_val);
    nvs_get_i32(handle, NVS_KEY_TRIM_YAW, &yaw_val);
    nvs_close(handle);

    g_trim_roll = static_cast<float>(roll_val) / 10000.0f;
    g_trim_pitch = static_cast<float>(pitch_val) / 10000.0f;
    g_trim_yaw = static_cast<float>(yaw_val) / 10000.0f;

    return ESP_OK;
}

static esp_err_t saveTrimToNVS()
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_CLI, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Store as int32 × 10000 for precision
    nvs_set_i32(handle, NVS_KEY_TRIM_ROLL, static_cast<int32_t>(g_trim_roll * 10000));
    nvs_set_i32(handle, NVS_KEY_TRIM_PITCH, static_cast<int32_t>(g_trim_pitch * 10000));
    nvs_set_i32(handle, NVS_KEY_TRIM_YAW, static_cast<int32_t>(g_trim_yaw * 10000));

    ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

// =============================================================================
// trim command
// =============================================================================

static int cmd_trim(int argc, char** argv)
{
    auto& console = Console::getInstance();

    constexpr float MAX_TRIM = 0.2f;

    if (argc < 2) {
        console.print("=== Trim Settings ===\r\n");
        console.print("  Roll:  %+.4f\r\n", g_trim_roll);
        console.print("  Pitch: %+.4f\r\n", g_trim_pitch);
        console.print("  Yaw:   %+.4f\r\n", g_trim_yaw);
        console.print("\r\nUsage:\r\n");
        console.print("  trim roll <value>   - Set roll trim (%.2f to +%.2f)\r\n", -MAX_TRIM, MAX_TRIM);
        console.print("  trim pitch <value>  - Set pitch trim\r\n");
        console.print("  trim yaw <value>    - Set yaw trim\r\n");
        console.print("  trim save           - Save to NVS\r\n");
        console.print("  trim reset          - Reset to zero\r\n");
        return 0;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "roll") == 0) {
        if (argc < 3) {
            console.print("Roll trim: %+.4f\r\n", g_trim_roll);
            return 0;
        }
        float val = atof(argv[2]);
        if (val < -MAX_TRIM || val > MAX_TRIM) {
            console.print("Value out of range. Use %.2f to +%.2f\r\n", -MAX_TRIM, MAX_TRIM);
            return 1;
        }
        g_trim_roll = val;
        console.print("Roll trim set to %+.4f\r\n", g_trim_roll);
    }
    else if (strcmp(cmd, "pitch") == 0) {
        if (argc < 3) {
            console.print("Pitch trim: %+.4f\r\n", g_trim_pitch);
            return 0;
        }
        float val = atof(argv[2]);
        if (val < -MAX_TRIM || val > MAX_TRIM) {
            console.print("Value out of range. Use %.2f to +%.2f\r\n", -MAX_TRIM, MAX_TRIM);
            return 1;
        }
        g_trim_pitch = val;
        console.print("Pitch trim set to %+.4f\r\n", g_trim_pitch);
    }
    else if (strcmp(cmd, "yaw") == 0) {
        if (argc < 3) {
            console.print("Yaw trim: %+.4f\r\n", g_trim_yaw);
            return 0;
        }
        float val = atof(argv[2]);
        if (val < -MAX_TRIM || val > MAX_TRIM) {
            console.print("Value out of range. Use %.2f to +%.2f\r\n", -MAX_TRIM, MAX_TRIM);
            return 1;
        }
        g_trim_yaw = val;
        console.print("Yaw trim set to %+.4f\r\n", g_trim_yaw);
    }
    else if (strcmp(cmd, "save") == 0) {
        if (saveTrimToNVS() == ESP_OK) {
            console.print("Trim saved to NVS\r\n");
            console.print("  Roll:  %+.4f\r\n", g_trim_roll);
            console.print("  Pitch: %+.4f\r\n", g_trim_pitch);
            console.print("  Yaw:   %+.4f\r\n", g_trim_yaw);
        } else {
            console.print("Failed to save trim to NVS\r\n");
            return 1;
        }
    }
    else if (strcmp(cmd, "reset") == 0) {
        g_trim_roll = 0.0f;
        g_trim_pitch = 0.0f;
        g_trim_yaw = 0.0f;
        console.print("Trim reset to zero\r\n");
        console.print("Use 'trim save' to persist\r\n");
    }
    else {
        console.print("Unknown subcommand: %s\r\n", cmd);
        return 1;
    }

    return 0;
}

// =============================================================================
// gain command
// =============================================================================

static int cmd_gain(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (g_rate_controller_ptr == nullptr) {
        console.print("Rate controller not available\r\n");
        return 1;
    }

    auto& rc = *g_rate_controller_ptr;

    if (argc < 2) {
        console.print("=== Rate Control Gains ===\r\n");
        console.print("Sensitivity (max rate [rad/s]):\r\n");
        console.print("  roll_max:  %.2f\r\n", rc.roll_rate_max);
        console.print("  pitch_max: %.2f\r\n", rc.pitch_rate_max);
        console.print("  yaw_max:   %.2f\r\n", rc.yaw_rate_max);
        console.print("\r\nRoll PID:\r\n");
        console.print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                      rc.roll_pid.getKp(), rc.roll_pid.getTi(), rc.roll_pid.getTd());
        console.print("Pitch PID:\r\n");
        console.print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                      rc.pitch_pid.getKp(), rc.pitch_pid.getTi(), rc.pitch_pid.getTd());
        console.print("Yaw PID:\r\n");
        console.print("  Kp: %.4f  Ti: %.4f  Td: %.5f\r\n",
                      rc.yaw_pid.getKp(), rc.yaw_pid.getTi(), rc.yaw_pid.getTd());
        console.print("\r\nUsage: gain <axis> <param> <value>\r\n");
        console.print("  axis:  roll, pitch, yaw\r\n");
        console.print("  param: kp, ti, td, max\r\n");
        return 0;
    }

    if (argc < 4) {
        console.print("Usage: gain <axis> <param> <value>\r\n");
        return 1;
    }

    const char* axis = argv[1];
    const char* param = argv[2];
    float value = atof(argv[3]);

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
        console.print("Unknown axis: %s (use roll, pitch, yaw)\r\n", axis);
        return 1;
    }

    if (strcmp(param, "kp") == 0) {
        pid->setKp(value);
        console.print("%s Kp = %.4f\r\n", axis, value);
    } else if (strcmp(param, "ti") == 0) {
        pid->setTi(value);
        console.print("%s Ti = %.4f\r\n", axis, value);
    } else if (strcmp(param, "td") == 0) {
        pid->setTd(value);
        console.print("%s Td = %.5f\r\n", axis, value);
    } else if (strcmp(param, "max") == 0) {
        if (rate_max) {
            *rate_max = value;
            console.print("%s rate_max = %.2f [rad/s]\r\n", axis, value);
        }
    } else {
        console.print("Unknown param: %s (use kp, ti, td, max)\r\n", param);
        return 1;
    }

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_control_commands()
{
    // Load trim values from NVS at startup
    // 起動時に NVS からトリム値を読み込む
    if (loadTrimFromNVS() == ESP_OK) {
        ESP_LOGI("ControlCmds", "Trim loaded: roll=%.4f, pitch=%.4f, yaw=%.4f",
                 g_trim_roll, g_trim_pitch, g_trim_yaw);
    }

    // trim
    const esp_console_cmd_t trim_cmd = {
        .command = "trim",
        .help = "Trim adjust [roll|pitch|yaw <val>|save|reset]",
        .hint = NULL,
        .func = &cmd_trim,
        .argtable = NULL,
    };
    esp_console_cmd_register(&trim_cmd);

    // gain
    const esp_console_cmd_t gain_cmd = {
        .command = "gain",
        .help = "Rate control gains [axis param value]",
        .hint = NULL,
        .func = &cmd_gain,
        .argtable = NULL,
    };
    esp_console_cmd_register(&gain_cmd);
}

}  // namespace stampfly
