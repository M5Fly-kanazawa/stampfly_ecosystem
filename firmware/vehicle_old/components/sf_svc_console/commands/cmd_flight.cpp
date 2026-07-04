/**
 * @file cmd_flight.cpp
 * @brief Flight command CLI handlers
 *        飛行コマンドCLIハンドラ
 */

#include "console.hpp"
#include "flight_command.hpp"
#include "command_queue.hpp"
#include "system_state.hpp"
#include "motor_driver.hpp"
#include "stampfly_state.hpp"
#include "sensor_fusion.hpp"
#include "control_arbiter.hpp"
#include "udp_protocol.hpp"  // for CTRL_FLAG_ALT_MODE
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

using namespace stampfly;

// External references for emergency stop
// 緊急停止用の外部参照
extern MotorDriver* g_motor_ptr;

namespace globals {
    extern sf::SensorFusion g_fusion;
}

// Jump command: jump [altitude_m] [hover_sec]
// ジャンプコマンド: jump [高度_m] [ホバリング_秒]
static int cmd_jump(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    float altitude = 0.15f;  // Default 0.15m (15cm)
    if (argc >= 2) {
        altitude = atof(argv[1]);
        if (altitude < 0.1f || altitude > 2.0f) {
            console.print("Error: Altitude must be 0.1-2.0 [m]\r\n");
            return 1;
        }
    }

    FlightCommandParams params;
    params.target_altitude = altitude;
    params.duration_s = 0.0f;      // Not used (hovering phase skipped in JUMP)
    params.climb_rate = 0.4f;      // 0.4 m/s climb
    params.descent_rate = 0.3f;    // 0.3 m/s descent

    if (flight.executeCommand(FlightCommandType::JUMP, params)) {
        // Command successfully enqueued
        // コマンドがキューに登録された
        auto& sys_state = SystemStateManager::getInstance();

        CalibrationState calib_state = sys_state.getCalibrationState();

        if (calib_state == CalibrationState::COMPLETED) {
            console.print("Jump command enqueued: climb to %.2f m then descend\r\n", altitude);
        } else {
            console.print("Jump command queued (waiting for calibration to complete)\r\n");
            console.print("  Target altitude: %.2f m\r\n", altitude);
            console.print("  Watch for GREEN LED before execution starts\r\n");
        }
        return 0;
    } else {
        console.print("Failed to enqueue jump command (queue full)\r\n");
        return 1;
    }
}

// Takeoff command: takeoff [altitude_m]
// 離陸コマンド: takeoff [高度_m]
static int cmd_takeoff(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    float altitude = 0.5f;  // Default 0.5m
    if (argc >= 2) {
        altitude = atof(argv[1]);
        if (altitude < 0.1f || altitude > 2.0f) {
            console.print("Error: Altitude must be 0.1-2.0 [m]\r\n");
            return 1;
        }
    }

    FlightCommandParams params;
    params.target_altitude = altitude;
    params.climb_rate = 0.3f;  // 0.3 m/s

    if (flight.executeCommand(FlightCommandType::TAKEOFF, params)) {
        auto& sys_state = SystemStateManager::getInstance();
        CalibrationState calib_state = sys_state.getCalibrationState();

        if (calib_state == CalibrationState::COMPLETED) {
            console.print("Takeoff command enqueued (target: %.2f m)\r\n", altitude);
        } else {
            console.print("Takeoff command queued (waiting for calibration)\r\n");
            console.print("  Target altitude: %.2f m\r\n", altitude);
        }
        return 0;
    } else {
        console.print("Failed to enqueue takeoff command\r\n");
        return 1;
    }
}

// Land command: land
// 着陸コマンド: land
static int cmd_land(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    FlightCommandParams params;
    params.descent_rate = 0.3f;  // 0.3 m/s

    if (flight.executeCommand(FlightCommandType::LAND, params)) {
        console.print("Land command enqueued\r\n");
        return 0;
    } else {
        console.print("Failed to enqueue land command\r\n");
        return 1;
    }
}

// Hover command: hover [altitude_m] [duration_s]
// ホバリングコマンド: hover [高度_m] [時間_秒]
static int cmd_hover(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    float altitude = 0.5f;  // Default 0.5m
    if (argc >= 2) {
        altitude = atof(argv[1]);
        if (altitude < 0.1f || altitude > 2.0f) {
            console.print("Error: Altitude must be 0.1-2.0 [m]\r\n");
            return 1;
        }
    }

    float duration = 5.0f;  // Default 5s
    if (argc >= 3) {
        duration = atof(argv[2]);
        if (duration < 0.1f || duration > 60.0f) {
            console.print("Error: Duration must be 0.1-60.0 [s]\r\n");
            return 1;
        }
    }

    FlightCommandParams params;
    params.target_altitude = altitude;
    params.duration_s = duration;

    if (flight.executeCommand(FlightCommandType::HOVER, params)) {
        auto& sys_state = SystemStateManager::getInstance();
        CalibrationState calib_state = sys_state.getCalibrationState();

        if (calib_state == CalibrationState::COMPLETED) {
            console.print("Hover command enqueued (alt: %.2f m, duration: %.1f s)\r\n",
                         altitude, duration);
        } else {
            console.print("Hover command queued (waiting for calibration)\r\n");
            console.print("  Target altitude: %.2f m, duration: %.1f s\r\n", altitude, duration);
            console.print("  Watch for GREEN LED before execution starts\r\n");
        }
        return 0;
    } else {
        console.print("Failed to enqueue hover command (queue full)\r\n");
        return 1;
    }
}

// Flight status/cancel: flight [status|cancel]
// 飛行状態/キャンセル: flight [status|cancel]
static int cmd_flight(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: flight [status|cancel]\r\n");
        return 1;
    }

    if (strcmp(argv[1], "status") == 0) {
        // Show status
        // 状態表示
        if (flight.isRunning()) {
            const char* cmd_name = "UNKNOWN";
            switch (flight.getCurrentCommand()) {
                case FlightCommandType::JUMP:          cmd_name = "JUMP"; break;
                case FlightCommandType::TAKEOFF:       cmd_name = "TAKEOFF"; break;
                case FlightCommandType::LAND:          cmd_name = "LAND"; break;
                case FlightCommandType::HOVER:         cmd_name = "HOVER"; break;
                case FlightCommandType::MOVE_VERTICAL: cmd_name = "MOVE_VERTICAL"; break;
                case FlightCommandType::ROTATE_YAW:    cmd_name = "ROTATE_YAW"; break;
                case FlightCommandType::MOVE_HORIZONTAL: cmd_name = "MOVE_HORIZONTAL"; break;
                default: break;
            }
            console.print("Flight command: %s\r\n", cmd_name);
            console.print("State: RUNNING\r\n");
            console.print("Elapsed: %.1f s\r\n", flight.getElapsedTime());
        } else {
            console.print("Flight command: NONE\r\n");
            console.print("State: IDLE\r\n");
        }
        return 0;

    } else if (strcmp(argv[1], "cancel") == 0) {
        // Cancel command
        // コマンドキャンセル
        if (flight.isRunning()) {
            flight.cancel();
            console.print("Flight command cancelled\r\n");
        } else {
            console.print("No command running\r\n");
        }
        return 0;

    } else {
        console.print("Unknown subcommand: %s\r\n", argv[1]);
        console.print("Usage: flight [status|cancel]\r\n");
        return 1;
    }
}

// Up command: up <cm>
// 上昇コマンド: up <cm>
static int cmd_up(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: up <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Calculate absolute target altitude = current + distance
    // 絶対目標高度 = 現在高度 + 移動距離
    float current_alt = -globals::g_fusion.getState().position.z;
    float target_alt = current_alt + (float)cm / 100.0f;

    FlightCommandParams params{};
    params.target_altitude = target_alt;

    if (flight.executeCommand(FlightCommandType::MOVE_VERTICAL, params)) {
        console.print("Up %d cm enqueued (target: %.2f m)\r\n", cm, target_alt);
        return 0;
    } else {
        console.print("Failed to enqueue up command\r\n");
        return 1;
    }
}

// Down command: down <cm>
// 降下コマンド: down <cm>
static int cmd_down(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: down <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Calculate absolute target altitude = current - distance
    // 絶対目標高度 = 現在高度 - 移動距離
    float current_alt = -globals::g_fusion.getState().position.z;
    float target_alt = current_alt - (float)cm / 100.0f;
    if (target_alt < 0.05f) target_alt = 0.05f;  // Minimum safe altitude / 最低安全高度

    FlightCommandParams params{};
    params.target_altitude = target_alt;

    if (flight.executeCommand(FlightCommandType::MOVE_VERTICAL, params)) {
        console.print("Down %d cm enqueued (target: %.2f m)\r\n", cm, target_alt);
        return 0;
    } else {
        console.print("Failed to enqueue down command\r\n");
        return 1;
    }
}

// CW command: cw <deg>
// 時計回りコマンド: cw <deg>
static int cmd_cw(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: cw <deg>  (1-360)\r\n");
        return 1;
    }

    int deg = atoi(argv[1]);
    if (deg < 1 || deg > 360) {
        console.print("Error: Angle must be 1-360 [deg]\r\n");
        return 1;
    }

    // Calculate absolute target yaw = current + angle
    // 絶対目標ヨー角 = 現在ヨー角 + 回転角
    float current_yaw_deg = globals::g_fusion.getState().yaw * 180.0f / 3.14159265f;
    float target_yaw_deg = current_yaw_deg + (float)deg;

    // Maintain current altitude during rotation
    // 回転中は現在高度を維持
    float current_alt = -globals::g_fusion.getState().position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_yaw_deg = target_yaw_deg;

    if (flight.executeCommand(FlightCommandType::ROTATE_YAW, params)) {
        console.print("CW %d deg enqueued (target yaw: %.1f deg)\r\n", deg, target_yaw_deg);
        return 0;
    } else {
        console.print("Failed to enqueue cw command\r\n");
        return 1;
    }
}

// CCW command: ccw <deg>
// 反時計回りコマンド: ccw <deg>
static int cmd_ccw(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: ccw <deg>  (1-360)\r\n");
        return 1;
    }

    int deg = atoi(argv[1]);
    if (deg < 1 || deg > 360) {
        console.print("Error: Angle must be 1-360 [deg]\r\n");
        return 1;
    }

    // Calculate absolute target yaw = current - angle
    // 絶対目標ヨー角 = 現在ヨー角 - 回転角
    float current_yaw_deg = globals::g_fusion.getState().yaw * 180.0f / 3.14159265f;
    float target_yaw_deg = current_yaw_deg - (float)deg;

    // Maintain current altitude during rotation
    // 回転中は現在高度を維持
    float current_alt = -globals::g_fusion.getState().position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_yaw_deg = target_yaw_deg;

    if (flight.executeCommand(FlightCommandType::ROTATE_YAW, params)) {
        console.print("CCW %d deg enqueued (target yaw: %.1f deg)\r\n", deg, target_yaw_deg);
        return 0;
    } else {
        console.print("Failed to enqueue ccw command\r\n");
        return 1;
    }
}

// Forward command: forward <cm>
// 前進コマンド: forward <cm>
static int cmd_forward(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: forward <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Calculate target position in NED frame (body forward → NED)
    // NED座標系で目標位置を計算（機体前方 → NED）
    auto state = globals::g_fusion.getState();
    float current_x = state.position.x;
    float current_y = state.position.y;
    float yaw = state.yaw;
    float dist = (float)cm / 100.0f;

    // Body forward → NED: X += dist*cos(yaw), Y += dist*sin(yaw)
    float target_x = current_x + dist * cosf(yaw);
    float target_y = current_y + dist * sinf(yaw);
    float current_alt = -state.position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_pos_x = target_x;
    params.target_pos_y = target_y;

    if (flight.executeCommand(FlightCommandType::MOVE_HORIZONTAL, params)) {
        console.print("Forward %d cm enqueued (target: %.2f, %.2f)\r\n", cm, target_x, target_y);
        return 0;
    } else {
        console.print("Failed to enqueue forward command\r\n");
        return 1;
    }
}

// Back command: back <cm>
// 後退コマンド: back <cm>
static int cmd_back(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: back <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Body backward → NED: X -= dist*cos(yaw), Y -= dist*sin(yaw)
    auto state = globals::g_fusion.getState();
    float current_x = state.position.x;
    float current_y = state.position.y;
    float yaw = state.yaw;
    float dist = (float)cm / 100.0f;

    float target_x = current_x - dist * cosf(yaw);
    float target_y = current_y - dist * sinf(yaw);
    float current_alt = -state.position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_pos_x = target_x;
    params.target_pos_y = target_y;

    if (flight.executeCommand(FlightCommandType::MOVE_HORIZONTAL, params)) {
        console.print("Back %d cm enqueued (target: %.2f, %.2f)\r\n", cm, target_x, target_y);
        return 0;
    } else {
        console.print("Failed to enqueue back command\r\n");
        return 1;
    }
}

// Left command: left <cm>
// 左移動コマンド: left <cm>
static int cmd_left(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: left <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Body left → NED: perpendicular to forward, left = -90° from heading
    // NED: X += dist*sin(yaw) * (-1), Y -= dist*cos(yaw) ... wait
    // Body left in NED: X += dist*(-sin(yaw)), Y += dist*cos(yaw)
    // Actually: body left = yaw - 90° direction
    // forward direction = (cos(yaw), sin(yaw))
    // left direction = (-sin(yaw), cos(yaw))
    auto state = globals::g_fusion.getState();
    float current_x = state.position.x;
    float current_y = state.position.y;
    float yaw = state.yaw;
    float dist = (float)cm / 100.0f;

    float target_x = current_x - dist * sinf(yaw);
    float target_y = current_y + dist * cosf(yaw);
    float current_alt = -state.position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_pos_x = target_x;
    params.target_pos_y = target_y;

    if (flight.executeCommand(FlightCommandType::MOVE_HORIZONTAL, params)) {
        console.print("Left %d cm enqueued (target: %.2f, %.2f)\r\n", cm, target_x, target_y);
        return 0;
    } else {
        console.print("Failed to enqueue left command\r\n");
        return 1;
    }
}

// Right command: right <cm>
// 右移動コマンド: right <cm>
static int cmd_right(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    if (argc < 2) {
        console.print("Usage: right <cm>  (20-200)\r\n");
        return 1;
    }

    int cm = atoi(argv[1]);
    if (cm < 20 || cm > 200) {
        console.print("Error: Distance must be 20-200 [cm]\r\n");
        return 1;
    }

    // Body right → NED: right = +90° from heading
    // right direction = (sin(yaw), -cos(yaw))
    auto state = globals::g_fusion.getState();
    float current_x = state.position.x;
    float current_y = state.position.y;
    float yaw = state.yaw;
    float dist = (float)cm / 100.0f;

    float target_x = current_x + dist * sinf(yaw);
    float target_y = current_y - dist * cosf(yaw);
    float current_alt = -state.position.z;

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.target_pos_x = target_x;
    params.target_pos_y = target_y;

    if (flight.executeCommand(FlightCommandType::MOVE_HORIZONTAL, params)) {
        console.print("Right %d cm enqueued (target: %.2f, %.2f)\r\n", cm, target_x, target_y);
        return 0;
    } else {
        console.print("Failed to enqueue right command\r\n");
        return 1;
    }
}

// Emergency command: emergency
// 緊急停止コマンド: emergency
static int cmd_emergency(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    // Cancel any running command
    // 実行中のコマンドをキャンセル
    flight.cancel();

    // Cancel all queued commands
    // キュー内の全コマンドをキャンセル
    CommandQueue::getInstance().cancelAll();

    // Force motor stop via disarm
    // disarm で強制モーター停止
    if (g_motor_ptr) {
        g_motor_ptr->disarm();
    }

    // Reset flight state to IDLE
    // フライト状態を IDLE にリセット
    StampFlyState::getInstance().setFlightState(FlightState::IDLE);
    SystemStateManager::getInstance().setFlightState(FlightState::IDLE);

    console.print("EMERGENCY: All motors stopped, state reset to IDLE\r\n");
    return 0;
}

// Stop command: stop (hover at current position)
// 停止コマンド: stop（現在位置でホバリング）
static int cmd_stop(int argc, char** argv) {
    auto& console = Console::getInstance();
    auto& flight = FlightCommandService::getInstance();

    // Cancel current command first
    // まず現在のコマンドをキャンセル
    if (flight.isRunning()) {
        flight.cancel();
    }

    // Hover at current altitude indefinitely (9999s)
    // 現在高度で無期限ホバリング（9999秒）
    float current_alt = -globals::g_fusion.getState().position.z;
    if (current_alt < 0.1f) current_alt = 0.1f;  // Minimum hover altitude / 最低ホバリング高度

    FlightCommandParams params{};
    params.target_altitude = current_alt;
    params.duration_s = 9999.0f;

    if (flight.executeCommand(FlightCommandType::HOVER, params)) {
        console.print("Stop: hovering at %.2f m\r\n", current_alt);
        return 0;
    } else {
        console.print("Failed to enqueue stop (hover) command\r\n");
        return 1;
    }
}

// RC control command: rc <a> <b> <c> <d>
// RC制御コマンド: rc <左右> <前後> <上下> <ヨー> (-100~100)
static int cmd_rc(int argc, char** argv) {
    auto& console = Console::getInstance();

    if (argc < 5) {
        console.print("Usage: rc <a> <b> <c> <d>  (-100~100)\r\n");
        console.print("  a: left/right (roll)\r\n");
        console.print("  b: forward/backward (pitch)\r\n");
        console.print("  c: up/down (throttle)\r\n");
        console.print("  d: yaw\r\n");
        return 1;
    }

    // Parse and clamp to [-100, 100]
    // 解析して [-100, 100] にクランプ
    int a = std::clamp(atoi(argv[1]), -100, 100);  // left/right → roll
    int b = std::clamp(atoi(argv[2]), -100, 100);  // fwd/back → pitch
    int c = std::clamp(atoi(argv[3]), -100, 100);  // up/down → throttle
    int d = std::clamp(atoi(argv[4]), -100, 100);  // yaw

    // Normalize: roll/pitch/yaw [-1, 1], throttle [0, 1]
    // 正規化: roll/pitch/yaw [-1, 1], throttle [0, 1]
    float roll     = a / 100.0f;
    float pitch    = b / 100.0f;
    float throttle = (c + 100.0f) / 200.0f;  // [-100,100] → [0,1]
    float yaw      = d / 100.0f;

    auto& arbiter = ControlArbiter::getInstance();
    arbiter.updateFromWebSocket(throttle, roll, pitch, yaw, stampfly::udp::CTRL_FLAG_ALT_MODE);

    console.print("ok\r\n");
    return 0;
}

// Register flight commands
// 飛行コマンドを登録
extern "C" void register_flight_commands() {
    const esp_console_cmd_t jump_cmd = {
        .command = "jump",
        .help = "Jump (takeoff, hover, land) [jump <altitude_m> <hover_sec>]",
        .hint = NULL,
        .func = &cmd_jump,
    };
    esp_console_cmd_register(&jump_cmd);

    const esp_console_cmd_t takeoff_cmd = {
        .command = "takeoff",
        .help = "Takeoff to specified altitude [takeoff <altitude_m>]",
        .hint = NULL,
        .func = &cmd_takeoff,
    };
    esp_console_cmd_register(&takeoff_cmd);

    const esp_console_cmd_t land_cmd = {
        .command = "land",
        .help = "Land the vehicle",
        .hint = NULL,
        .func = &cmd_land,
    };
    esp_console_cmd_register(&land_cmd);

    const esp_console_cmd_t hover_cmd = {
        .command = "hover",
        .help = "Hover at altitude [hover <altitude_m> <duration_s>]",
        .hint = NULL,
        .func = &cmd_hover,
    };
    esp_console_cmd_register(&hover_cmd);

    const esp_console_cmd_t flight_status_cmd = {
        .command = "flight",
        .help = "Flight command status [flight status|cancel]",
        .hint = NULL,
        .func = &cmd_flight,
    };
    esp_console_cmd_register(&flight_status_cmd);

    const esp_console_cmd_t up_cmd = {
        .command = "up",
        .help = "Move up [up <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_up,
    };
    esp_console_cmd_register(&up_cmd);

    const esp_console_cmd_t down_cmd = {
        .command = "down",
        .help = "Move down [down <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_down,
    };
    esp_console_cmd_register(&down_cmd);

    const esp_console_cmd_t cw_cmd = {
        .command = "cw",
        .help = "Rotate clockwise [cw <deg>] (1-360)",
        .hint = NULL,
        .func = &cmd_cw,
    };
    esp_console_cmd_register(&cw_cmd);

    const esp_console_cmd_t ccw_cmd = {
        .command = "ccw",
        .help = "Rotate counter-clockwise [ccw <deg>] (1-360)",
        .hint = NULL,
        .func = &cmd_ccw,
    };
    esp_console_cmd_register(&ccw_cmd);

    const esp_console_cmd_t forward_cmd = {
        .command = "forward",
        .help = "Move forward [forward <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_forward,
    };
    esp_console_cmd_register(&forward_cmd);

    const esp_console_cmd_t back_cmd = {
        .command = "back",
        .help = "Move backward [back <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_back,
    };
    esp_console_cmd_register(&back_cmd);

    const esp_console_cmd_t left_cmd = {
        .command = "left",
        .help = "Move left [left <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_left,
    };
    esp_console_cmd_register(&left_cmd);

    const esp_console_cmd_t right_cmd = {
        .command = "right",
        .help = "Move right [right <cm>] (20-200)",
        .hint = NULL,
        .func = &cmd_right,
    };
    esp_console_cmd_register(&right_cmd);

    const esp_console_cmd_t emergency_cmd = {
        .command = "emergency",
        .help = "Emergency stop (kill all motors immediately)",
        .hint = NULL,
        .func = &cmd_emergency,
    };
    esp_console_cmd_register(&emergency_cmd);

    const esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help = "Stop and hover at current position",
        .hint = NULL,
        .func = &cmd_stop,
    };
    esp_console_cmd_register(&stop_cmd);

    const esp_console_cmd_t rc_cmd = {
        .command = "rc",
        .help = "RC control [rc <a> <b> <c> <d>] (-100~100: roll pitch throttle yaw)",
        .hint = NULL,
        .func = &cmd_rc,
    };
    esp_console_cmd_register(&rc_cmd);
}
