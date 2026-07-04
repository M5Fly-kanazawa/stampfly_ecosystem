/**
 * @file cmd_misc.cpp
 * @brief Miscellaneous commands (led, sound, pos, debug, ctrl, attitude)
 *
 * その他のコマンド
 */

#include "console.hpp"
#include "stampfly_state.hpp"
#include "led_manager.hpp"
#include "buzzer.hpp"
#include "sensor_fusion.hpp"
#include "controller_comm.hpp"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdlib>

// External references
// 外部参照
extern stampfly::Buzzer* g_buzzer_ptr;
extern sf::SensorFusion* g_fusion_ptr;
extern stampfly::ControllerComm* g_comm_ptr;

namespace stampfly {

// =============================================================================
// led command
// =============================================================================

static int cmd_led(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& led_mgr = LEDManager::getInstance();

    if (argc < 2) {
        console.print("LED brightness: %d (0-255)\r\n", led_mgr.getBrightness());
        console.print("Usage: led brightness <0-255>\r\n");
        return 0;
    }

    if (strcmp(argv[1], "brightness") == 0) {
        if (argc < 3) {
            console.print("Current brightness: %d\r\n", led_mgr.getBrightness());
            console.print("Usage: led brightness <0-255>\r\n");
            return 0;
        }
        int brightness = atoi(argv[2]);
        if (brightness < 0 || brightness > 255) {
            console.print("Invalid brightness. Use 0-255.\r\n");
            return 1;
        }
        led_mgr.setBrightness(static_cast<uint8_t>(brightness), true);  // save to NVS
        console.print("LED brightness set to %d (saved)\r\n", brightness);
    } else {
        console.print("Usage: led brightness <0-255>\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// sound command
// =============================================================================

static int cmd_sound(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (g_buzzer_ptr == nullptr) {
        console.print("Buzzer not available\r\n");
        return 1;
    }

    if (argc < 2) {
        console.print("Sound: %s\r\n", g_buzzer_ptr->isMuted() ? "OFF" : "ON");
        console.print("Usage: sound [on|off]\r\n");
        return 0;
    }

    if (strcmp(argv[1], "on") == 0) {
        g_buzzer_ptr->setMuted(false, true);  // save to NVS
        console.print("Sound ON (saved)\r\n");
        g_buzzer_ptr->beep();  // Play confirmation beep
    } else if (strcmp(argv[1], "off") == 0) {
        g_buzzer_ptr->setMuted(true, true);  // save to NVS
        console.print("Sound OFF (saved)\r\n");
    } else {
        console.print("Usage: sound [on|off]\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// pos command
// =============================================================================

static int cmd_pos(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (g_fusion_ptr == nullptr) {
        console.print("Sensor fusion not available\r\n");
        return 1;
    }

    if (argc < 2) {
        auto state = g_fusion_ptr->getState();
        console.print("Position [m]: X=%.3f Y=%.3f Z=%.3f\r\n",
                      state.position.x, state.position.y, state.position.z);
        console.print("Velocity [m/s]: X=%.3f Y=%.3f Z=%.3f\r\n",
                      state.velocity.x, state.velocity.y, state.velocity.z);
        console.print("Usage: pos [reset|status]\r\n");
        return 0;
    }

    if (strcmp(argv[1], "reset") == 0) {
        g_fusion_ptr->resetPositionVelocity();
        console.print("Position/Velocity reset to origin\r\n");
    } else if (strcmp(argv[1], "status") == 0) {
        auto state = g_fusion_ptr->getState();
        console.print("=== Position Status ===\r\n");
        console.print("Position [m]: X=%.3f Y=%.3f Z=%.3f\r\n",
                      state.position.x, state.position.y, state.position.z);
        console.print("Velocity [m/s]: X=%.3f Y=%.3f Z=%.3f\r\n",
                      state.velocity.x, state.velocity.y, state.velocity.z);
        console.print("Diverged: %s\r\n", g_fusion_ptr->isDiverged() ? "YES" : "NO");
    } else {
        console.print("Usage: pos [reset|status]\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// debug command
// =============================================================================

static int cmd_debug(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    if (argc < 2) {
        console.print("Debug mode: %s\r\n", state.isDebugMode() ? "ON" : "OFF");
        console.print("Usage: debug [on|off]\r\n");
        console.print("  When ON, ARM ignores errors (LOW_BATTERY, sensors, etc.)\r\n");
        return 0;
    }

    if (strcmp(argv[1], "on") == 0) {
        state.setDebugMode(true);
        console.print("Debug mode ON - errors will be ignored for ARM\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        state.setDebugMode(false);
        console.print("Debug mode OFF\r\n");
    } else {
        console.print("Usage: debug [on|off]\r\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// ctrl command
// =============================================================================

static int cmd_ctrl(int argc, char** argv)
{
    auto& console = Console::getInstance();
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
        console.print("Controller input (%d sec):\r\n", watch_seconds);
    }

    int iterations = watch_seconds * 10;  // 10Hz
    do {
        uint16_t throttle, roll, pitch, yaw;
        state.getRawControlInput(throttle, roll, pitch, yaw);
        uint8_t flags = state.getControlFlags();

        const char* conn_status = "disconnected";
        if (g_comm_ptr != nullptr && g_comm_ptr->isConnected()) {
            conn_status = "connected";
        }

        if (watch_seconds > 0) {
            // Overwrite line with \r
            console.print("\rT:%4u R:%4u P:%4u Y:%4u [%c%c%c%c%c] [%s]   ",
                          throttle, roll, pitch, yaw,
                          (flags & CTRL_FLAG_ARM) ? 'A' : '-',
                          (flags & CTRL_FLAG_FLIP) ? 'F' : '-',
                          (flags & CTRL_FLAG_MODE) ? 'M' : '-',
                          (flags & CTRL_FLAG_ALT_MODE) ? 'H' : '-',
                          (flags & CTRL_FLAG_POS_MODE) ? 'P' : '-',
                          conn_status);
            vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update
            iterations--;
        } else {
            console.print("Controller Input (raw ADC values):\r\n");
            console.print("  Throttle: %u (0-4095)\r\n", throttle);
            console.print("  Roll:     %u (2048=center)\r\n", roll);
            console.print("  Pitch:    %u (2048=center)\r\n", pitch);
            console.print("  Yaw:      %u (2048=center)\r\n", yaw);
            console.print("  Flags:    0x%02X\r\n", flags);
            console.print("    ARM:      %s\r\n", (flags & CTRL_FLAG_ARM) ? "ON" : "OFF");
            console.print("    FLIP:     %s\r\n", (flags & CTRL_FLAG_FLIP) ? "ON" : "OFF");
            console.print("    MODE:     %s\r\n", (flags & CTRL_FLAG_MODE) ? "ON" : "OFF");
            console.print("    ALT_MODE: %s\r\n", (flags & CTRL_FLAG_ALT_MODE) ? "ON" : "OFF");
            console.print("    POS_MODE: %s\r\n", (flags & CTRL_FLAG_POS_MODE) ? "ON" : "OFF");
            console.print("  Status:   %s\r\n", conn_status);
        }
    } while (iterations > 0);

    if (watch_seconds > 0) {
        console.print("\r\n");
    }

    return 0;
}

// =============================================================================
// attitude command
// =============================================================================

static int cmd_attitude(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    float roll, pitch, yaw;
    state.getAttitudeEuler(roll, pitch, yaw);

    console.print("Attitude:\r\n");
    console.print("  Roll:  %.2f [deg]\r\n", roll * 180.0f / M_PI);
    console.print("  Pitch: %.2f [deg]\r\n", pitch * 180.0f / M_PI);
    console.print("  Yaw:   %.2f [deg]\r\n", yaw * 180.0f / M_PI);

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_misc_commands()
{
    // led
    const esp_console_cmd_t led_cmd = {
        .command = "led",
        .help = "LED [brightness <0-255>]",
        .hint = NULL,
        .func = &cmd_led,
        .argtable = NULL,
    };
    esp_console_cmd_register(&led_cmd);

    // sound
    const esp_console_cmd_t sound_cmd = {
        .command = "sound",
        .help = "Sound [on|off]",
        .hint = NULL,
        .func = &cmd_sound,
        .argtable = NULL,
    };
    esp_console_cmd_register(&sound_cmd);

    // pos
    const esp_console_cmd_t pos_cmd = {
        .command = "pos",
        .help = "Position [reset|status]",
        .hint = NULL,
        .func = &cmd_pos,
        .argtable = NULL,
    };
    esp_console_cmd_register(&pos_cmd);

    // debug
    const esp_console_cmd_t debug_cmd = {
        .command = "debug",
        .help = "Debug mode [on|off]",
        .hint = NULL,
        .func = &cmd_debug,
        .argtable = NULL,
    };
    esp_console_cmd_register(&debug_cmd);

    // ctrl
    const esp_console_cmd_t ctrl_cmd = {
        .command = "ctrl",
        .help = "Show controller input [watch [sec]]",
        .hint = NULL,
        .func = &cmd_ctrl,
        .argtable = NULL,
    };
    esp_console_cmd_register(&ctrl_cmd);

    // attitude
    const esp_console_cmd_t attitude_cmd = {
        .command = "attitude",
        .help = "Show attitude",
        .hint = NULL,
        .func = &cmd_attitude,
        .argtable = NULL,
    };
    esp_console_cmd_register(&attitude_cmd);
}

}  // namespace stampfly
