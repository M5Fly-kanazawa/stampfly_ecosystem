/**
 * @file cmd_motor.cpp
 * @brief Motor commands (motor)
 *
 * モーターコマンド
 */

#include "console.hpp"
#include "motor_driver.hpp"
#include "esp_console.h"
#include <cstring>
#include <cstdlib>

// External references
// 外部参照
extern stampfly::MotorDriver* g_motor_ptr;

namespace stampfly {

// =============================================================================
// motor command
// =============================================================================

static int cmd_motor(int argc, char** argv)
{
    auto& console = Console::getInstance();

    if (g_motor_ptr == nullptr) {
        console.print("Motor driver not available\r\n");
        return 1;
    }

    if (argc < 2) {
        console.print("Usage: motor <command>\r\n");
        console.print("  test <id> <throttle> - Test single motor (id:1-4, throttle:0-100)\r\n");
        console.print("  all <throttle>       - Test all motors (throttle:0-100)\r\n");
        console.print("  stop                 - Stop all motors\r\n");
        console.print("  stats                - Show duty cycle statistics\r\n");
        console.print("  stats_reset          - Reset statistics\r\n");
        console.print("\r\n");
        console.print("Motor layout (top view):\r\n");
        console.print("       Front\r\n");
        console.print("   M4(FL)  M1(FR)\r\n");
        console.print("       X\r\n");
        console.print("   M3(RL)  M2(RR)\r\n");
        console.print("        Rear\r\n");
        return 0;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "stop") == 0) {
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FR, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RR, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RL, 0);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FL, 0);
        console.print("All motors stopped\r\n");
    }
    else if (strcmp(cmd, "all") == 0) {
        if (argc < 3) {
            console.print("Usage: motor all <throttle>\r\n");
            return 1;
        }
        int throttle = atoi(argv[2]);
        if (throttle < 0 || throttle > 100) {
            console.print("Invalid throttle. Use 0-100.\r\n");
            return 1;
        }
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FR, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RR, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_RL, throttle);
        g_motor_ptr->testMotor(MotorDriver::MOTOR_FL, throttle);
        console.print("All motors at %d%%\r\n", throttle);
    }
    else if (strcmp(cmd, "test") == 0) {
        if (argc < 4) {
            console.print("Usage: motor test <id> <throttle>\r\n");
            console.print("  id: 1=FR, 2=RR, 3=RL, 4=FL\r\n");
            return 1;
        }
        int id = atoi(argv[2]);
        int throttle = atoi(argv[3]);
        if (id < 1 || id > 4) {
            console.print("Invalid motor ID. Use 1-4.\r\n");
            return 1;
        }
        if (throttle < 0 || throttle > 100) {
            console.print("Invalid throttle. Use 0-100.\r\n");
            return 1;
        }

        const char* motor_names[] = {"FR (M1)", "RR (M2)", "RL (M3)", "FL (M4)"};
        g_motor_ptr->testMotor(id - 1, throttle);
        console.print("Motor %s at %d%%\r\n", motor_names[id - 1], throttle);
    }
    else if (strcmp(cmd, "stats") == 0) {
        g_motor_ptr->loadStatsFromNVS();

        const char* names[] = {"M1(FR)", "M2(RR)", "M3(RL)", "M4(FL)"};
        const char* types[] = {"CCW", "CW ", "CCW", "CW "};
        console.print("Last Flight Motor Statistics:\r\n");
        console.print("Motor Type   Avg     Min     Max     Count\r\n");
        console.print("------------------------------------------\r\n");
        for (int i = 0; i < 4; i++) {
            auto stats = g_motor_ptr->getLastFlightStats(i);
            float avg = (stats.count > 0) ? (stats.sum / stats.count) : 0.0f;
            console.print("%s  %s  %.3f   %.3f   %.3f   %lu\r\n",
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
        console.print("------------------------------------------\r\n");
        console.print("CCW avg: %.3f, CW avg: %.3f, diff: %.3f\r\n",
                      ccw_avg, cw_avg, ccw_avg - cw_avg);
    }
    else if (strcmp(cmd, "stats_reset") == 0) {
        g_motor_ptr->resetStats();
        console.print("Motor statistics reset\r\n");
    }
    else {
        console.print("Unknown motor command: %s\r\n", cmd);
        return 1;
    }

    return 0;
}

// =============================================================================
// Command Registration
// =============================================================================

void register_motor_commands()
{
    const esp_console_cmd_t motor_cmd = {
        .command = "motor",
        .help = "Motor control [test|all|stop|stats]",
        .hint = NULL,
        .func = &cmd_motor,
        .argtable = NULL,
    };
    esp_console_cmd_register(&motor_cmd);
}

}  // namespace stampfly
