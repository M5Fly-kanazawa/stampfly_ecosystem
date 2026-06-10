/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file cli_task.cpp
 * @brief CLI + parameter access task (USB-CDC serial console)
 *        CLI + パラメータアクセスタスク（USB-CDC シリアルコンソール）
 *
 * Owns the interactive serial console. Registers a small command set via the
 * ESP-IDF console registry — `param` (list/get/set/save), `status`, `reboot` — then
 * starts the USB-CDC REPL, which reads the serial line in its own task. Commands are
 * held in a static {name, help, func} table and registered in a loop (R6: registry
 * pattern, no extern global console pointer). The CLI is a ground/bring-up tool and
 * is not on the flight-critical path.
 *
 * 対話式シリアルコンソールを所有する。ESP-IDF コンソールレジストリに小さなコマンド集
 * （param(list/get/set/save)・status・reboot）を登録し、USB-CDC REPL を起動する。REPL は
 * 自前のタスクでシリアル行を読む。コマンドは静的な {name, help, func} テーブルに持ち、
 * ループで登録する（R6: レジストリパターン、extern グローバルのコンソールポインタを作らない）。
 * CLI は地上/ブリングアップ用ツールで、飛行クリティカル経路には載らない。
 *
 * @subscriber system_mode, sensor_power, sensor_health, pairing_state, pairing_complete
 * @publisher button_event (CLI `pair` injects a LongPress3s gesture fact)
 * @design architecture.md §6 — CLITask: CLI + Parameters              [OK]
 * @design architecture.md §3 — R6 CLI command registry pattern         [OK]
 * @design detailed_design.md §8 — CLITask                            [OK]
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>     // quaternion → euler for `status`/`sensor`

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_timer.h"
#include "nvs.h"       // wifi credentials (CLI writes, sf_comm reads at boot)
#include "lwip/sockets.h"   // TCP CLI server (requirements §7: CLI over TCP)
#include <cerrno>

#include "topics.hpp"
#include "params.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "CLITask";

namespace {

// =============================================================================
// Command handlers — signature int(int argc, char** argv) per esp_console.
// コマンドハンドラ — esp_console の int(int argc, char** argv) シグネチャ。
// =============================================================================

/// Find a registered parameter by name (linear scan over the params table).
/// 名前で登録パラメータを探す（params テーブルの線形走査）。
const sf::params::ParamEntry* findParam(const char* name)
{
    for (int i = 0, n = sf::params::count(); i < n; ++i) {
        const sf::params::ParamEntry* entry = sf::params::entry(i);
        if (entry != nullptr && std::strcmp(entry->name, name) == 0) {
            return entry;
        }
    }
    return nullptr;
}

/// Print one parameter's current value, formatted by its type.
/// パラメータ1件の現在値を型に応じて整形表示する。
void printParam(const sf::params::ParamEntry* entry)
{
    using sf::params::ParamType;
    switch (entry->type) {
    case ParamType::FLOAT: {
        float value = 0.0f;
        sf::params::get_float(entry->name, value);
        std::printf("%s = %.6g (float)\n", entry->name, value);
        break;
    }
    case ParamType::BOOL: {
        bool value = false;
        sf::params::get_bool(entry->name, value);
        std::printf("%s = %s (bool)\n", entry->name, value ? "true" : "false");
        break;
    }
    case ParamType::INT: {
        int32_t value = 0;
        sf::params::get_int(entry->name, value);
        std::printf("%s = %ld (int)\n", entry->name, static_cast<long>(value));
        break;
    }
    }
}

/// `param [list | get <name> | set <name> <value> | save]` — read/write parameters.
/// set_*() validates the range, runs the change callback and persists to NVS.
/// `param [list | get <name> | set <name> <value> | save]` — パラメータ読み書き。
/// set_*() は範囲検証・変更コールバック・NVS 保存を行う。
int cmd_param(int argc, char** argv)
{
    if (argc < 2 || std::strcmp(argv[1], "list") == 0) {
        sf::params::list();
        return 0;
    }
    if (std::strcmp(argv[1], "save") == 0) {
        // Flash sector erase during the NVS commit disables the cache and
        // stalls the 400Hz IMU loop >10ms (verified on hardware: the ControlTask
        // watchdog fired during a bench save). Harmless disarmed; in flight it
        // would zero the motors for the stall — so refuse while armed. Live
        // tuning (param set → ReloadParams) works in flight; persist on the
        // ground afterwards.
        // NVS commit のフラッシュセクタ消去中はキャッシュが止まり 400Hz IMU ループが
        // 10ms 超ストールする（実機検証: ベンチ save 中に ControlTask ウォッチドッグ
        // 発火）。disarmed では無害だが飛行中はストール分モータがゼロになる — armed 中は
        // 拒否する。ライブチューニング（param set → ReloadParams）は飛行中も使え、
        // 永続化は着陸後に行えばよい。
        const sf::SystemMode mode = sf::system_mode.latest();
        if (sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
            std::printf("refused: param save stalls the 400Hz loop (flash erase) — "
                        "land/disarm first\n");
            return 1;
        }
        sf::params::save();
        std::printf("params saved to NVS\n");
        return 0;
    }
    if (std::strcmp(argv[1], "get") == 0 && argc >= 3) {
        const sf::params::ParamEntry* entry = findParam(argv[2]);
        if (entry == nullptr) {
            std::printf("unknown param: %s\n", argv[2]);
            return 1;
        }
        printParam(entry);
        return 0;
    }
    if (std::strcmp(argv[1], "set") == 0 && argc >= 4) {
        const sf::params::ParamEntry* entry = findParam(argv[2]);
        if (entry == nullptr) {
            std::printf("unknown param: %s\n", argv[2]);
            return 1;
        }
        using sf::params::ParamType;
        bool ok = false;
        switch (entry->type) {
        case ParamType::FLOAT:
            ok = sf::params::set_float(entry->name, std::strtof(argv[3], nullptr));
            break;
        case ParamType::BOOL: {
            const bool value = (std::strcmp(argv[3], "1") == 0 ||
                                std::strcmp(argv[3], "true") == 0);
            ok = sf::params::set_bool(entry->name, value);
            break;
        }
        case ParamType::INT:
            ok = sf::params::set_int(
                entry->name, static_cast<int32_t>(std::strtol(argv[3], nullptr, 10)));
            break;
        }
        if (ok) {
            printParam(entry);
        } else {
            std::printf("set failed (out of range?): %s\n", entry->name);
        }
        return ok ? 0 : 1;
    }
    std::printf("usage: param [list | get <name> | set <name> <value> | save]\n");
    return 0;
}

/// Human-readable PairingState name (NotPaired/Pairing/Paired).
/// PairingState の名前（NotPaired/Pairing/Paired）。
const char* pairingStateName(uint8_t state)
{
    switch (static_cast<sf::PairingState>(state)) {
        case sf::PairingState::Pairing: return "Pairing";
        case sf::PairingState::Paired:  return "Paired";
        default:                        return "NotPaired";
    }
}

/// Convert an attitude quaternion [w,x,y,z] to roll/pitch/yaw in DEGREES (ZYX).
/// 姿勢クォータニオン [w,x,y,z] を roll/pitch/yaw（度, ZYX）へ変換する。
void quatToEulerDeg(const float q[4], float& roll, float& pitch, float& yaw)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float kRad2Deg = 57.29578f;
    roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * kRad2Deg;
    float sinp = 2.0f * (w * y - z * x);
    sinp = (sinp > 1.0f) ? 1.0f : (sinp < -1.0f ? -1.0f : sinp);  // clamp for asin
    pitch = std::asin(sinp) * kRad2Deg;
    yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * kRad2Deg;
}

/// `status` — flight state / mode / arm, pairing, attitude, altitude, battery, sensors.
/// `status` — フライト状態/モード/ARM、ペアリング、姿勢、高度、電池、センサ。
int cmd_status(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    const sf::SystemMode mode = sf::system_mode.latest();
    const sf::PowerData power = sf::sensor_power.latest();
    const sf::SensorHealth health = sf::sensor_health.latest();
    const sf::PairingStatus pairing = sf::pairing_state.latest();
    const sf::StateEstimate est = sf::estimate_state.latest();
    float roll, pitch, yaw;
    quatToEulerDeg(est.attitude, roll, pitch, yaw);
    std::printf("state   : %s\n",
                sf::flightStateName(static_cast<sf::FlightState>(mode.state)));
    std::printf("mode    : %s\n",
                sf::flightModeName(static_cast<sf::FlightMode>(mode.sub_mode)));
    std::printf("armed   : %s\n", mode.armed ? "yes" : "no");
    std::printf("pairing : %s\n", pairingStateName(pairing.state));
    std::printf("attitude: roll %.1f  pitch %.1f  yaw %.1f  deg\n", roll, pitch, yaw);
    std::printf("altitude: %.2f m (ESKF)\n", -est.position[2]);  // NED: up = -z
    std::printf("battery : %.2f V, %.0f mA\n", power.voltage, power.current);
    std::printf("sensors : present=0x%02X healthy=0x%02X\n",
                health.present_mask, health.healthy_mask);
    return 0;
}

/// `sensor [imu|mag|baro|tof|flow|power|all]` — print the latest reading of one (or all)
/// sensors. Reads the sensor_* topics directly (read-only fact display, like status).
/// `sensor [imu|mag|baro|tof|flow|power|all]` — 1つ（または全部）のセンサの最新値を表示。
/// sensor_* トピックを直接読む（status と同じ読み取り専用の事実表示）。
int cmd_sensor(int argc, char** argv)
{
    const char* which = (argc >= 2) ? argv[1] : "all";
    const bool all = (std::strcmp(which, "all") == 0);

    // IMU (RingBuffer) and power (Latest) expose a non-consuming latest(). The async
    // sensors (mag/baro/tof/flow) are SPSC queues consumed by ImuTask, so we read their
    // mirrored values from sensor_snapshot (Latest) instead of stealing from the queues.
    // IMU(RingBuffer)と power(Latest)は非消費の latest() を持つ。非同期センサ(mag/baro/tof/
    // flow)は ImuTask が消費する SPSC キューなので、奪わずに sensor_snapshot(Latest)のミラーを読む。
    const sf::SensorSnapshot snap = sf::sensor_snapshot.latest();

    if (all || std::strcmp(which, "imu") == 0) {
        const sf::ImuData d = sf::sensor_imu.latest();
        std::printf("imu  : accel[%.2f %.2f %.2f] m/s2  gyro[%.3f %.3f %.3f] rad/s  %.1fC\n",
                    d.accel[0], d.accel[1], d.accel[2], d.gyro[0], d.gyro[1], d.gyro[2],
                    d.temperature);
    }
    if (all || std::strcmp(which, "mag") == 0) {
        std::printf("mag  : [%.1f %.1f %.1f] uT\n", snap.mag[0], snap.mag[1], snap.mag[2]);
    }
    if (all || std::strcmp(which, "baro") == 0) {
        std::printf("baro : %.1f Pa  alt %.2f m\n", snap.baro_pressure, snap.baro_altitude);
    }
    if (all || std::strcmp(which, "tof") == 0) {
        std::printf("tof  : %.3f m  status=%u valid=%s\n", snap.tof_distance, snap.tof_status,
                    snap.tof_valid ? "yes" : "no");
    }
    if (all || std::strcmp(which, "flow") == 0) {
        std::printf("flow : dx=%d dy=%d squal=%u\n", snap.flow_dx, snap.flow_dy, snap.flow_squal);
    }
    if (all || std::strcmp(which, "power") == 0) {
        const sf::PowerData d = sf::sensor_power.latest();
        std::printf("power: %.2f V  %.0f mA  %.0f mW\n", d.voltage, d.current, d.power);
    }
    return 0;
}

/// `version` — firmware name + build date/time.
/// `version` — ファーム名＋ビルド日時。
int cmd_version(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("StampFly vehicle_new firmware\n");
    std::printf("build : %s %s\n", __DATE__, __TIME__);
    return 0;
}

/// `unpair` — clear the stored controller pairing and re-enter Pairing so a (new)
/// transmitter can bind. Publishes a LongPress3s button gesture FACT — the same path
/// as a 3 s button hold — so the StateManager decides (no cross-task coupling).
/// `unpair` — 保存済みペアリングを破棄し Pairing に再突入して（新しい）送信機がバインドできる
/// ようにする。LongPress3s ジェスチャの事実を発行（ボタン長押し3秒と同一経路）し、StateManager
/// が判断する（タスク間結合なし）。
int cmd_unpair(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    sf::ButtonEvent ev{};
    ev.gesture   = static_cast<uint8_t>(sf::ButtonGesture::LongPress3s);
    ev.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::button_event.publish(ev);
    std::printf("unpair requested — clearing bind, re-entering pairing (on the ground)\n");
    return 0;
}

/// `pair` — pairing control. `pair` / `pair start` re-enters Pairing (discards the
/// current bind and searches for a transmitter, like a 3 s button long-press);
/// `pair status` prints the PairingState and the bound transmitter MAC. The start
/// path publishes a button_event FACT (LongPress3s) so the StateManager — the sole
/// authority — decides, exactly as for the physical button (no cross-task coupling).
/// `pair` — ペアリング操作。`pair`/`pair start` は Pairing に再突入（現在のバインドを破棄し
/// 送信機を探索、ボタン長押し3秒と同じ）、`pair status` は PairingState とバインド済み送信機
/// MAC を表示。start は button_event の事実（LongPress3s）を発行し、唯一の権限者である
/// StateManager が判断する（物理ボタンと同一経路、タスク間結合なし）。
int cmd_pair(int argc, char** argv)
{
    if (argc >= 2 && std::strcmp(argv[1], "status") == 0) {
        const sf::PairingStatus ps = sf::pairing_state.latest();
        const sf::PairingComplete bind = sf::pairing_complete.latest();
        const char* name = "NotPaired";
        switch (static_cast<sf::PairingState>(ps.state)) {
            case sf::PairingState::Pairing: name = "Pairing"; break;
            case sf::PairingState::Paired:  name = "Paired";  break;
            default:                        break;
        }
        std::printf("pairing : %s\n", name);
        if (bind.bound) {
            std::printf("bound   : %02X:%02X:%02X:%02X:%02X:%02X%s\n",
                        bind.controller_mac[0], bind.controller_mac[1],
                        bind.controller_mac[2], bind.controller_mac[3],
                        bind.controller_mac[4], bind.controller_mac[5],
                        bind.restored ? " (restored)" : "");
        } else {
            std::printf("bound   : none\n");
        }
        return 0;
    }
    if (argc < 2 || std::strcmp(argv[1], "start") == 0) {
        // Inject a LongPress3s gesture fact; the StateManager re-enters Pairing
        // (it gates this to the ground / disarmed and clears the existing bind).
        // LongPress3s ジェスチャの事実を注入。StateManager が Pairing に再突入する
        // （地上/disarmed に限定し既存バインドを破棄する）。
        sf::ButtonEvent ev{};
        ev.gesture   = static_cast<uint8_t>(sf::ButtonGesture::LongPress3s);
        ev.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        sf::button_event.publish(ev);
        std::printf("pairing requested (takes effect on the ground / disarmed)\n");
        return 0;
    }
    std::printf("usage: pair [start | status]\n");
    return 0;
}

/// `sound [on|off]` — enable/disable the buzzer (mute), persisted to NVS. Publishes a
/// UiCommand FACT; NotifyTask (which owns the buzzer) applies it.
/// `sound [on|off]` — ブザー有効/無効（ミュート, NVS 保存）。UiCommand を発行し、ブザーを所有
/// する NotifyTask が適用する。
int cmd_sound(int argc, char** argv)
{
    const bool on  = (argc >= 2 && std::strcmp(argv[1], "on") == 0);
    const bool off = (argc >= 2 && std::strcmp(argv[1], "off") == 0);
    if (!on && !off) {
        std::printf("usage: sound [on|off]\n");
        return 0;
    }
    sf::UiCommand c{};
    c.command   = static_cast<uint8_t>(sf::UiCmd::SoundMute);
    c.value     = off ? 1 : 0;   // off → mute / off=ミュート
    c.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::ui_command.publish(c);
    std::printf("sound %s\n", on ? "on" : "off");
    return 0;
}

/// `led <0-255>` — set body-LED brightness, persisted to NVS (UiCommand → NotifyTask).
/// `led <0-255>` — 本体 LED 輝度を設定（NVS 保存, UiCommand → NotifyTask）。
int cmd_led(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: led <0-255>\n");
        return 0;
    }
    int b = std::atoi(argv[1]);
    if (b < 0)   b = 0;
    if (b > 255) b = 255;
    sf::UiCommand c{};
    c.command   = static_cast<uint8_t>(sf::UiCmd::LedBrightness);
    c.value     = static_cast<uint8_t>(b);
    c.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::ui_command.publish(c);
    std::printf("led brightness = %d\n", b);
    return 0;
}

/// `motor [test <1-4> <0-100> | all <0-100> | stop]` — bench motor wiring/direction check.
/// Spins the motor(s) at a fixed duty for ~2 s, DISARMED ONLY (ControlTask ignores it when
/// armed). Publishes a MotorTest FACT. IDs: M1=FR M2=RR M3=RL M4=FL. KEEP PROPS OFF / hold
/// the craft. `motor stop` ends it early.
/// `motor [test <1-4> <0-100> | all <0-100> | stop]` — ベンチのモータ配線/回転方向確認。
/// 約2秒、**disarmed 限定**で固定 duty 回転（armed 時 ControlTask は無視）。MotorTest を発行。
/// ID: M1=FR M2=RR M3=RL M4=FL。**プロペラを外すか機体を保持**。`motor stop` で即停止。
int cmd_motor(int argc, char** argv)
{
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t kTestDurationUs = 2000000;  // 2 s auto-stop / 2秒自動停止

    if (argc >= 2 && std::strcmp(argv[1], "stop") == 0) {
        sf::MotorTest t{};
        t.active = false;
        t.timestamp = now;
        sf::motor_test.publish(t);
        std::printf("motor test stopped\n");
        return 0;
    }

    bool all = (argc >= 2 && std::strcmp(argv[1], "all") == 0);
    bool test = (argc >= 2 && std::strcmp(argv[1], "test") == 0);
    if ((test && argc >= 4) || (all && argc >= 3)) {
        int pct = std::atoi(all ? argv[2] : argv[3]);
        if (pct < 0)   pct = 0;
        if (pct > 100) pct = 100;
        sf::MotorTest t{};
        t.active    = true;
        t.duty      = pct / 100.0f;
        t.expiry_us = now + kTestDurationUs;
        t.timestamp = now;
        if (all) {
            t.motor_id = 0xFF;
            std::printf("motor ALL test @ %d%% for 2s (DISARMED only — props off!)\n", pct);
        } else {
            int id = std::atoi(argv[2]);   // M1..M4
            if (id < 1 || id > 4) {
                std::printf("usage: motor test <1-4> <0-100>\n");
                return 0;
            }
            t.motor_id = static_cast<uint8_t>(id - 1);   // M1→FR(0) … M4→FL(3)
            std::printf("motor M%d test @ %d%% for 2s (DISARMED only — props off!)\n", id, pct);
        }
        sf::motor_test.publish(t);
        return 0;
    }
    std::printf("usage: motor [test <1-4> <0-100> | all <0-100> | stop]\n");
    return 0;
}

/// `wifi` — telemetry WiFi configuration. Mode lives in the wifi.mode parameter
/// (boot-time; `param save` + reboot to apply); the STA credentials live in the
/// NVS namespace "sf_wifi" that sf_comm reads at boot (keys "ssid"/"pass" — the
/// CLI is the writer, sf_comm the reader; plain config storage, not state).
/// `wifi` — テレメトリ WiFi 設定。モードは wifi.mode パラメータ（起動時反映;
/// `param save`＋再起動で適用）、STA 資格情報は sf_comm が起動時に読む NVS
/// namespace "sf_wifi"（キー "ssid"/"pass" — CLI が書き手・sf_comm が読み手。
/// 状態でなく単なる設定ストレージ）。
int cmd_wifi(int argc, char** argv)
{
    constexpr const char* kNvsNamespace = "sf_wifi";   // matches sf_comm / sf_comm と一致

    if (argc >= 2 && std::strcmp(argv[1], "mode") == 0 && argc >= 3) {
        int32_t mode = -1;
        if (std::strcmp(argv[2], "sta") == 0) mode = 0;
        if (std::strcmp(argv[2], "ap")  == 0) mode = 1;
        if (mode < 0) {
            std::printf("usage: wifi mode <sta|ap>\n");
            return 0;
        }
        sf::params::set_int("wifi.mode", mode);
        std::printf("wifi.mode = %s — run `param save` and reboot to apply\n",
                    mode == 0 ? "sta" : "ap");
        return 0;
    }

    if (argc >= 2 && (std::strcmp(argv[1], "ssid") == 0 ||
                      std::strcmp(argv[1], "pass") == 0) && argc >= 3) {
        nvs_handle_t handle;
        if (nvs_open(kNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
            std::printf("NVS open failed\n");
            return 0;
        }
        const esp_err_t set_err    = nvs_set_str(handle, argv[1], argv[2]);
        const esp_err_t commit_err = nvs_commit(handle);
        nvs_close(handle);
        if (set_err != ESP_OK || commit_err != ESP_OK) {
            std::printf("NVS write failed (%s/%s)\n",
                        esp_err_to_name(set_err), esp_err_to_name(commit_err));
        } else {
            std::printf("wifi %s saved — reboot to apply\n", argv[1]);
        }
        return 0;
    }

    if (argc == 1 || std::strcmp(argv[1], "show") == 0) {
        int32_t mode = 0;
        sf::params::get_int("wifi.mode", mode);
        char ssid[33] = {};
        size_t len = sizeof(ssid);
        nvs_handle_t handle;
        if (nvs_open(kNvsNamespace, NVS_READONLY, &handle) == ESP_OK) {
            nvs_get_str(handle, "ssid", ssid, &len);
            nvs_close(handle);
        }
        // The password is deliberately never printed.
        // パスワードは意図的に表示しない。
        std::printf("mode : %s\n", mode == 0 ? "sta" : "ap");
        std::printf("ssid : %s\n", ssid[0] ? ssid : "(unset)");
        return 0;
    }

    std::printf("usage: wifi [show | mode <sta|ap> | ssid <name> | pass <secret>]\n");
    return 0;
}

/// `reboot` — restart the flight controller.
/// `reboot` — フライトコントローラを再起動する。
int cmd_reboot(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("rebooting...\n");
    std::fflush(stdout);
    esp_restart();
    return 0;  // not reached / 到達しない
}

// =============================================================================
// TCP CLI server (requirements §7: CLI = USB Serial / TCP)
// TCP CLI サーバ（要件§7: CLI = USB Serial / TCP）
//
// Bench operations (motor test, tuning) run on battery with the USB cable off,
// so the same registered commands must be reachable over WiFi. One client at a
// time; commands are dispatched with esp_console_run() in THIS task, and since
// stdout is task-local in ESP-IDF, swapping it to the socket FILE routes every
// command's printf output to the TCP client without touching any handler.
// ベンチ作業（モータテスト・チューニング）は USB を外した電池駆動で行うため、同じ
// 登録済みコマンドが WiFi 経由で届く必要がある。クライアントは同時1接続。コマンドは
// 「本タスク内」で esp_console_run() により実行され、ESP-IDF の stdout はタスク
// ローカルなので、ソケットの FILE に差し替えるだけで全コマンドの printf 出力が
// ハンドラ無改変のまま TCP クライアントへ流れる。
// =============================================================================

/// TCP CLI port — telnet's, so `nc <ip> 23` and telnet clients both work.
/// TCP CLI ポート — telnet と同じにして `nc <ip> 23` でも telnet でも繋がる。
constexpr uint16_t kTcpCliPort = 23;

/// Serve one connected client until it disconnects (or the link dies).
/// 接続中のクライアント 1 つを切断（またはリンク死亡）まで担当する。
void serveTcpClient(const int client_fd)
{
    // Route printf to the socket via the task-local stdout (see block comment).
    // タスクローカル stdout 経由で printf をソケットへ（上のブロックコメント参照）。
    FILE* client = fdopen(client_fd, "w");
    if (client == nullptr) {
        ::close(client_fd);
        return;
    }
    FILE* saved_stdout = stdout;
    stdout = client;

    std::printf("StampFly vehicle_new TCP CLI — type 'help' for commands\n");
    std::printf("stampfly> ");
    std::fflush(client);

    char line[256];
    size_t length = 0;
    while (true) {
        uint8_t byte = 0;
        const ssize_t received = ::recv(client_fd, &byte, 1, 0);
        if (received == 0) {
            break;   // orderly close / 正常切断
        }
        if (received < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                continue;
            }
            break;   // link error (keepalive timeout etc.) / リンク異常
        }
        if (byte == '\r') {
            continue;   // tolerate CRLF clients (telnet) / CRLF クライアント許容
        }
        if (byte != '\n') {
            if (length < sizeof(line) - 1) {
                line[length++] = static_cast<char>(byte);
            }
            continue;
        }
        line[length] = '\0';
        length = 0;
        if (line[0] != '\0') {
            int command_ret = 0;
            const esp_err_t err = esp_console_run(line, &command_ret);
            if (err == ESP_ERR_NOT_FOUND) {
                std::printf("unknown command (try 'help')\n");
            } else if (err != ESP_OK) {
                std::printf("error: %s\n", esp_err_to_name(err));
            }
        }
        std::printf("stampfly> ");
        std::fflush(client);
    }

    stdout = saved_stdout;
    std::fclose(client);   // also closes client_fd / client_fd も閉じる
}

/// Accept loop — runs forever in CLITask after the USB REPL has started.
/// accept ループ — USB REPL 起動後、CLITask 内で永続的に走る。
void runTcpCliServer()
{
    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGW(TAG, "TCP CLI disabled: socket() errno=%d", errno);
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(kTcpCliPort);
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        ::listen(listen_fd, 1) < 0) {
        ESP_LOGW(TAG, "TCP CLI disabled: bind/listen errno=%d", errno);
        ::close(listen_fd);
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    ESP_LOGI(TAG, "TCP CLI listening on port %u (nc <vehicle-ip> %u)",
             static_cast<unsigned>(kTcpCliPort),
             static_cast<unsigned>(kTcpCliPort));

    while (true) {
        sockaddr_in peer = {};
        socklen_t peer_len = sizeof(peer);
        const int client_fd =
            ::accept(listen_fd, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (client_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));   // no client yet / クライアント未着
            continue;
        }

        // TCP keepalive: a client that vanishes without FIN (WiFi drop, sleep)
        // would otherwise hold the single CLI slot forever. ~60s detection.
        // TCP keepalive: FIN なしで消えたクライアント（WiFi 断・スリープ）が唯一の
        // CLI 枠を永久に塞ぐのを防ぐ。検出は約60秒。
        ::setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
#ifdef TCP_KEEPIDLE
        int idle = 30, interval = 10, count = 3;
        ::setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        ::setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
        ::setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
#endif

        ESP_LOGI(TAG, "TCP CLI client connected");
        serveTcpClient(client_fd);
        ESP_LOGI(TAG, "TCP CLI client disconnected");
    }
}

// =============================================================================
// Command registry (R6) — static {name, help, func} table registered in a loop.
// コマンドレジストリ（R6）— 静的な {name, help, func} テーブルをループ登録。
// =============================================================================

struct CliCommand {
    const char* name;
    const char* help;
    esp_console_cmd_func_t func;
};

const CliCommand kCommands[] = {
    {"param",   "param [list|get <name>|set <name> <value>|save]", &cmd_param},
    {"status",  "Show flight state, pairing, attitude, battery, sensors", &cmd_status},
    {"sensor",  "sensor [imu|mag|baro|tof|flow|power|all] — print readings", &cmd_sensor},
    {"version", "Show firmware version / build date",            &cmd_version},
    {"pair",    "pair [start|status] — (re-)enter pairing / show bind", &cmd_pair},
    {"unpair",  "Clear pairing and re-enter pairing mode",       &cmd_unpair},
    {"sound",   "sound [on|off] — enable/disable the buzzer",    &cmd_sound},
    {"led",     "led <0-255> — body LED brightness",             &cmd_led},
    {"motor",   "motor [test <1-4> <0-100>|all <0-100>|stop] — bench test (disarmed)", &cmd_motor},
    {"wifi",    "wifi [show|mode <sta|ap>|ssid <name>|pass <secret>] — telemetry WiFi", &cmd_wifi},
    {"reboot",  "Reboot the flight controller",                  &cmd_reboot},
};

void registerCommands()
{
    for (const CliCommand& command : kCommands) {
        esp_console_cmd_t cmd = {};
        cmd.command  = command.name;
        cmd.help     = command.help;
        cmd.hint     = nullptr;
        cmd.func     = command.func;
        cmd.argtable = nullptr;
        esp_console_cmd_register(&cmd);
    }
}

}  // namespace

void CLITask(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "CLITask started");

    // Create the USB-CDC console REPL. esp_console_new_repl_usb_cdc() runs
    // esp_console_init() internally, so commands must be registered AFTER it.
    // USB-CDC コンソール REPL を生成する。esp_console_new_repl_usb_cdc() は内部で
    // esp_console_init() を呼ぶため、コマンド登録はこの後で行う。
    esp_console_repl_t* repl = nullptr;
    esp_console_repl_config_t repl_config = {};
    repl_config.max_history_len    = 16;
    repl_config.history_save_path  = nullptr;
    repl_config.task_stack_size    = config::STACK_CLI;
    repl_config.task_priority      = config::PRIORITY_CLI;
    repl_config.prompt             = "stampfly> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_usb_cdc_config_t cdc_config = {};

    esp_err_t err = esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "console REPL init failed: %s — CLI disabled",
                 esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    esp_console_register_help_command();  // built-in 'help' / 組み込みの 'help'
    registerCommands();                   // R6 registry / R6 レジストリ

    // Start the interactive REPL — esp_console reads the USB-CDC serial line in its
    // own task. On the SIL host this is inert (commands stay registered and remain
    // dispatchable via a scenario console feeder).
    // 対話 REPL を起動 — esp_console が自前タスクで USB-CDC シリアル行を読む。SIL ホスト
    // では inert（コマンドは登録済みのままで、シナリオのコンソールフィーダから dispatch 可）。
    esp_console_start_repl(repl);

    // The REPL task now owns USB interactive input. This task serves the TCP
    // CLI (requirements §7: CLI = USB Serial / TCP): bench operations like the
    // motor test run on battery with the USB cable OFF, so commands must be
    // reachable over WiFi. Connect with `nc <vehicle-ip> 23` (SoftAP:
    // 192.168.4.1). On the SIL host the inert socket shim never accepts, so
    // this loop just idles.
    // 以降は REPL タスクが USB の対話入力を所有する。本タスクは TCP CLI を提供する
    // （要件§7: CLI = USB Serial / TCP）: モータテスト等のベンチ作業は USB ケーブルを
    // 外した電池駆動で行うため、コマンドは WiFi 経由で届く必要がある。接続は
    // `nc <機体IP> 23`（SoftAP なら 192.168.4.1）。SIL ホストでは inert ソケットシムが
    // accept しないため、このループは待機するだけ。
    runTcpCliServer();
}
