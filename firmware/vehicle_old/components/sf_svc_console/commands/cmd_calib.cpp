/**
 * @file cmd_calib.cpp
 * @brief Calibration commands (calib, magcal)
 *
 * キャリブレーションコマンド
 */

#include "console.hpp"
#include "mag_calibration.hpp"
#include "stampfly_state.hpp"
#include "esp_console.h"
#include <cstring>
#include <cmath>

// External references
// 外部参照
extern stampfly::MagCalibrator* g_mag_calibrator;

namespace stampfly {

// Global Console pointer for magcal log callback
static Console* g_magcal_console = nullptr;

// Callback function for magnetometer calibration log messages
static void magcal_log_callback(const char* message)
{
    if (g_magcal_console) {
        g_magcal_console->print("%s\r\n", message);
    }
}

// =============================================================================
// calib command
// =============================================================================

static int cmd_calib(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (argc < 2) {
        console.print("Usage: calib [gyro|accel|mag]\r\n");
        return 1;
    }

    const char* type = argv[1];
    if (strcmp(type, "gyro") == 0) {
        console.print("Starting gyro calibration...\r\n");
        console.print("Keep device still for 3 seconds.\r\n");
        // TODO: SystemManager::runGyroCalibration()
        console.print("Gyro calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "accel") == 0) {
        console.print("Starting accelerometer calibration...\r\n");
        console.print("Place device on flat surface.\r\n");
        // TODO: SystemManager::runAccelCalibration()
        console.print("Accel calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "mag") == 0) {
        console.print("Starting magnetometer calibration...\r\n");
        console.print("Rotate device in all directions.\r\n");
        // TODO: SystemManager::runMagCalibration()
        console.print("Mag calibration complete (stub)\r\n");
    }
    else {
        console.print("Unknown calibration type: %s\r\n", type);
        return 1;
    }

    return 0;
}

// =============================================================================
// magcal command
// =============================================================================

static int cmd_magcal(int argc, char** argv)
{
    auto& console = Console::getInstance();
    g_magcal_console = &console;  // Set for callback

    if (g_mag_calibrator == nullptr) {
        console.print("Magnetometer calibrator not available\r\n");
        return 1;
    }

    // Set log callback
    g_mag_calibrator->setLogCallback(magcal_log_callback);

    if (argc < 2) {
        console.print("Usage: magcal [start|stop|status|save|clear]\r\n");
        console.print("  start  - Start calibration (rotate device in figure-8)\r\n");
        console.print("  stop   - Stop calibration and compute result\r\n");
        console.print("  status - Show calibration status\r\n");
        console.print("  save   - Save calibration to NVS\r\n");
        console.print("  clear  - Clear saved calibration\r\n");
        return 0;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "start") == 0) {
        if (g_mag_calibrator->getState() == MagCalibrator::State::COLLECTING) {
            console.print("Calibration already in progress\r\n");
            return 0;
        }
        if (g_mag_calibrator->startCalibration() == ESP_OK) {
            console.print("Magnetometer calibration started\r\n");
            console.print("Slowly rotate device in all directions (figure-8 pattern)\r\n");
            console.print("Need at least %d samples. Use 'magcal status' to check progress.\r\n",
                          MagCalibrator::MIN_SAMPLES);
            console.print("Use 'magcal stop' when done.\r\n");
        } else {
            console.print("Failed to start calibration\r\n");
            return 1;
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        auto state = g_mag_calibrator->getState();
        if (state != MagCalibrator::State::COLLECTING) {
            console.print("No calibration in progress\r\n");
            return 0;
        }

        console.print("Computing calibration from %d samples...\r\n",
                      g_mag_calibrator->getSampleCount());

        if (g_mag_calibrator->computeCalibration() == ESP_OK) {
            auto& cal = g_mag_calibrator->getCalibration();
            console.print("\r\n=== Calibration Result ===\r\n");
            console.print("Hard Iron Offset:\r\n");
            console.print("  X: %.2f uT\r\n", cal.offset_x);
            console.print("  Y: %.2f uT\r\n", cal.offset_y);
            console.print("  Z: %.2f uT\r\n", cal.offset_z);
            console.print("Soft Iron Scale:\r\n");
            console.print("  X: %.3f\r\n", cal.scale_x);
            console.print("  Y: %.3f\r\n", cal.scale_y);
            console.print("  Z: %.3f\r\n", cal.scale_z);
            console.print("Sphere Radius: %.2f uT\r\n", cal.sphere_radius);
            console.print("Fitness: %.2f (higher is better)\r\n", cal.fitness);
            console.print("\r\nUse 'magcal save' to persist to NVS\r\n");
        } else {
            console.print("Calibration failed. Try again with more samples and better coverage.\r\n");
            return 1;
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
        console.print("State: %s\r\n", state_str);
        console.print("Samples: %d / %d\r\n",
                      g_mag_calibrator->getSampleCount(),
                      MagCalibrator::MIN_SAMPLES);
        console.print("Progress: %.0f%%\r\n", g_mag_calibrator->getProgress() * 100.0f);

        if (g_mag_calibrator->isCalibrated()) {
            auto& cal = g_mag_calibrator->getCalibration();
            console.print("Calibrated: Yes\r\n");
            console.print("  Offset: [%.2f, %.2f, %.2f] uT\r\n",
                          cal.offset_x, cal.offset_y, cal.offset_z);
            console.print("  Scale:  [%.3f, %.3f, %.3f]\r\n",
                          cal.scale_x, cal.scale_y, cal.scale_z);
        } else {
            console.print("Calibrated: No\r\n");
        }

        // Show current mag reading
        auto& mstate = StampFlyState::getInstance();
        Vec3 mag;
        mstate.getMagData(mag);

        if (g_mag_calibrator->isCalibrated()) {
            console.print("Calibrated Mag: [%.1f, %.1f, %.1f] uT, norm=%.1f\r\n",
                          mag.x, mag.y, mag.z,
                          sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z));
        } else {
            console.print("Raw Mag: [%.1f, %.1f, %.1f] uT, norm=%.1f\r\n",
                          mag.x, mag.y, mag.z,
                          sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z));
        }
    }
    else if (strcmp(cmd, "save") == 0) {
        if (!g_mag_calibrator->isCalibrated()) {
            console.print("No valid calibration to save\r\n");
            return 1;
        }
        if (g_mag_calibrator->saveToNVS() == ESP_OK) {
            console.print("Calibration saved to NVS\r\n");
        } else {
            console.print("Failed to save calibration\r\n");
            return 1;
        }
    }
    else if (strcmp(cmd, "clear") == 0) {
        if (g_mag_calibrator->clearNVS() == ESP_OK) {
            console.print("Calibration cleared from NVS\r\n");
        } else {
            console.print("Failed to clear calibration\r\n");
            return 1;
        }
    }
    else {
        console.print("Unknown subcommand: %s\r\n", cmd);
        return 1;
    }

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_calib_commands()
{
    // calib
    const esp_console_cmd_t calib_cmd = {
        .command = "calib",
        .help = "Run calibration [gyro|accel|mag]",
        .hint = NULL,
        .func = &cmd_calib,
        .argtable = NULL,
    };
    esp_console_cmd_register(&calib_cmd);

    // magcal
    const esp_console_cmd_t magcal_cmd = {
        .command = "magcal",
        .help = "Mag calibration [start|stop|status|save|clear]",
        .hint = NULL,
        .func = &cmd_magcal,
        .argtable = NULL,
    };
    esp_console_cmd_register(&magcal_cmd);
}

}  // namespace stampfly
