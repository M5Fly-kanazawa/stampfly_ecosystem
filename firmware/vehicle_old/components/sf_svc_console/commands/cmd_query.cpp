/**
 * @file cmd_query.cpp
 * @brief Tello-compatible query commands (battery?, height?, tof?, etc.)
 *
 * Tello互換クエリコマンド
 * Python SDK / sf CLI からパースしやすい簡潔な応答を返す。
 * 応答は値のみ + \r\n（例: "85\r\n"）
 */

#include "console.hpp"
#include "stampfly_state.hpp"
#include "sensor_fusion.hpp"
#include "esp_console.h"
#include <cstring>
#include <cmath>

using namespace stampfly;

// External references
// 外部参照
namespace globals {
    extern sf::SensorFusion g_fusion;
}

// =============================================================================
// battery? — Battery level in percent
// バッテリー残量（%）
// =============================================================================

static int cmd_battery_query(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    float voltage, current;
    state.getPowerData(voltage, current);

    // Calculate battery percent from voltage (1S LiPo: 3.3V=0%, 4.2V=100%)
    // 電圧からバッテリー%を計算（1S LiPo: 3.3V=0%, 4.2V=100%）
    int percent = (int)((voltage - 3.3f) / (4.2f - 3.3f) * 100.0f);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    console.print("%d\r\n", percent);
    return 0;
}

// =============================================================================
// height? — ESKF estimated height in cm
// ESKF推定高度（cm）
// =============================================================================

static int cmd_height_query(int argc, char** argv)
{
    auto& console = Console::getInstance();

    // ESKF position.z is in NED (negative = up), convert to positive altitude in cm
    // ESKF position.z は NED（負=上）、正の高度cmに変換
    float height_m = -globals::g_fusion.getState().position.z;
    int height_cm = (int)(height_m * 100.0f);
    if (height_cm < 0) height_cm = 0;

    console.print("%d\r\n", height_cm);
    return 0;
}

// =============================================================================
// tof? — ToF bottom distance in cm
// ToF下方距離（cm）
// =============================================================================

static int cmd_tof_query(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    float bottom, front;
    state.getToFData(bottom, front);
    int tof_cm = (int)(bottom * 100.0f);

    console.print("%d\r\n", tof_cm);
    return 0;
}

// =============================================================================
// baro? — Barometric altitude in cm
// 気圧高度（cm）
// =============================================================================

static int cmd_baro_query(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    float altitude, pressure;
    state.getBaroData(altitude, pressure);
    int alt_cm = (int)(altitude * 100.0f);

    console.print("%d\r\n", alt_cm);
    return 0;
}

// =============================================================================
// attitude? — Attitude angles in degrees (pitch roll yaw)
// 姿勢角（度）: ピッチ ロール ヨー
// =============================================================================

static int cmd_attitude_query(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    StateVector3 att = state.getAttitude();
    // att.x=roll, att.y=pitch, att.z=yaw (radians)
    // Convert to degrees, output as pitch roll yaw (Tello order)
    // ラジアンから度に変換、ピッチ ロール ヨーの順で出力
    int pitch = (int)(att.y * 180.0f / M_PI);
    int roll = (int)(att.x * 180.0f / M_PI);
    int yaw = (int)(att.z * 180.0f / M_PI);

    console.print("%d %d %d\r\n", pitch, roll, yaw);
    return 0;
}

// =============================================================================
// acceleration? — Acceleration in cm/s² (x y z)
// 加速度（cm/s²）: x y z
// =============================================================================

static int cmd_acceleration_query(int argc, char** argv)
{
    auto& console = Console::getInstance();
    auto& state = StampFlyState::getInstance();

    Vec3 accel, gyro;
    state.getIMUData(accel, gyro);
    // Convert m/s² to cm/s² (integer)
    // m/s² を cm/s² に変換（整数）
    int ax = (int)(accel.x * 100.0f);
    int ay = (int)(accel.y * 100.0f);
    int az = (int)(accel.z * 100.0f);

    console.print("%d %d %d\r\n", ax, ay, az);
    return 0;
}

// =============================================================================
// speed? — Current configured speed (cm/s)
// 設定済み移動速度（cm/s）
// =============================================================================

static int cmd_speed_query(int argc, char** argv)
{
    auto& console = Console::getInstance();

    // Default movement speed (fixed value for now)
    // デフォルト移動速度（現時点では固定値）
    console.print("50\r\n");
    return 0;
}

// =============================================================================
// Command Registration
// コマンド登録
// =============================================================================

extern "C" void register_query_commands()
{
    const esp_console_cmd_t battery_cmd = {
        .command = "battery?",
        .help = "Query battery level (%)",
        .hint = NULL,
        .func = &cmd_battery_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&battery_cmd);

    const esp_console_cmd_t height_cmd = {
        .command = "height?",
        .help = "Query ESKF estimated height (cm)",
        .hint = NULL,
        .func = &cmd_height_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&height_cmd);

    const esp_console_cmd_t tof_cmd = {
        .command = "tof?",
        .help = "Query ToF bottom distance (cm)",
        .hint = NULL,
        .func = &cmd_tof_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&tof_cmd);

    const esp_console_cmd_t baro_cmd = {
        .command = "baro?",
        .help = "Query barometric altitude (cm)",
        .hint = NULL,
        .func = &cmd_baro_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&baro_cmd);

    const esp_console_cmd_t attitude_cmd = {
        .command = "attitude?",
        .help = "Query attitude (pitch roll yaw in deg)",
        .hint = NULL,
        .func = &cmd_attitude_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&attitude_cmd);

    const esp_console_cmd_t acceleration_cmd = {
        .command = "acceleration?",
        .help = "Query acceleration (x y z in cm/s^2)",
        .hint = NULL,
        .func = &cmd_acceleration_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&acceleration_cmd);

    const esp_console_cmd_t speed_cmd = {
        .command = "speed?",
        .help = "Query configured speed (cm/s)",
        .hint = NULL,
        .func = &cmd_speed_query,
        .argtable = NULL,
    };
    esp_console_cmd_register(&speed_cmd);
}
