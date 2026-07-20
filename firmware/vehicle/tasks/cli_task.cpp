/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
/// set_*() validates the range and runs the change callback (RAM only —
/// persistence is `param save`).
/// `param [list | get <name> | set <name> <value> | save]` — パラメータ読み書き。
/// set_*() は範囲検証・変更コールバックのみ（RAM 反映のみ — 永続化は
/// `param save`）。
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
    if (std::strcmp(argv[1], "reset") == 0) {
        // Restore compiled-in defaults (RAM + live reload via the change
        // callbacks). NVS is untouched until `param save`, so a reboot before
        // saving undoes the reset. Needed after a firmware update changes the
        // defaults: saved NVS values would otherwise keep overriding them.
        // コンパイル時既定値へ復元（RAM＋変更コールバックでライブ反映）。`param save`
        // までは NVS は変わらず、保存前に再起動すれば元に戻る。ファーム更新で既定値が
        // 変わった後に必要 — NVS の保存値が新既定値を覆い隠し続けるため。
        sf::params::reset_all();
        std::printf("all params reset to compiled defaults (RAM only). "
                    "run 'param save' (disarmed) to persist\n");
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
    std::printf("usage: param [list | get <name> | set <name> <value> | save | reset]\n");
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
    std::printf("StampFly vehicle firmware\n");
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
    // `sound test [start|ok|fail]` — play an autotune cue NOW (disarmed, on the
    // ground) to verify the buzzer is working/audible without flying. Respects mute,
    // so `sound on` first if you hear nothing. Goes via notify_command (NotifyTask
    // owns the buzzer); default = the loud start warble.
    // `sound test [start|ok|fail]` — autotune 合図音を今すぐ鳴らして地上で可聴性を確認。
    // mute を尊重するので無音なら先に `sound on`。
    if (argc >= 2 && std::strcmp(argv[1], "test") == 0) {
        sf::NotifyEvent ev = sf::NotifyEvent::AutotuneStart;
        if (argc >= 3 && std::strcmp(argv[2], "ok")   == 0) ev = sf::NotifyEvent::AutotuneOk;
        if (argc >= 3 && std::strcmp(argv[2], "fail") == 0) ev = sf::NotifyEvent::AutotuneFail;
        sf::notify_command.publish({static_cast<uint8_t>(ev),
                                    static_cast<uint32_t>(esp_timer_get_time())});
        std::printf("sound test: playing %s tone (mute on = silent; `sound on` first)\n",
                    argc >= 3 ? argv[2] : "start");
        return 0;
    }
    const bool on  = (argc >= 2 && std::strcmp(argv[1], "on") == 0);
    const bool off = (argc >= 2 && std::strcmp(argv[1], "off") == 0);
    if (!on && !off) {
        std::printf("usage: sound [on|off|test [start|ok|fail]]\n");
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

// =============================================================================
// `motor sweep` abort support — stop the motors and unblock the CLI as soon as
// the operator asks, instead of running the up-to-~45s worst case to the end.
// `motor sweep` 中断サポート — 操縦者が求めた瞬間にモータを止め CLI を解放する
// （最悪 ~45秒の実行を最後まで待たせない）。
// =============================================================================

/// Publish a MotorTest FACT that stops every motor immediately (duty 0, active
/// defaults to false). One definition of "off" shared by `motor stop`, the
/// inter-motor rest in a sweep, the pre-baseline clear, and an aborted sweep
/// (input received / low battery, both below in collectPower()).
/// 全モータを即座に停止する MotorTest FACT を発行する（duty 0, active は既定
/// false）。「停止」の定義1つを `motor stop`・スイープのモータ間休止・ベースライン
/// 前のクリア・下記 collectPower() のスイープ中断（入力受信/低電圧）で共有する。
void publishMotorStop()
{
    sf::MotorTest off{};
    off.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::motor_test.publish(off);
}

/// TCP client socket currently dispatching a CLI command, or -1 when the CLI is
/// idle or the running command came from the USB-CDC REPL. Set by
/// serveTcpClient() around its esp_console_run() calls (§ TCP CLI server,
/// below); read by tcpAbortRequested() so a blocking `motor sweep` can be
/// aborted from the SAME TCP session that started it.
/// 現在 CLI コマンドを実行中の TCP クライアントソケット。CLI が idle か、実行中の
/// コマンドが USB-CDC REPL から来た場合は -1。serveTcpClient() が
/// esp_console_run() 呼び出しの前後で設定する（下記「TCP CLI サーバ」節）。
/// tcpAbortRequested() が読み、ブロッキングする `motor sweep` を起動したのと
/// 同じ TCP セッションから中断できるようにする。
int s_active_tcp_client_fd = -1;

/// Non-blocking abort check for `motor sweep` (used by collectPower(), below):
/// true if the TCP session that issued the command has sent anything (or hung
/// up) since the last poll. Uses MSG_DONTWAIT — the same per-call-non-blocking
/// idiom api_task.cpp uses for its UDP command socket — so it never stalls the
/// 20ms sample loop.
///
/// Always false for a USB-REPL-issued sweep (s_active_tcp_client_fd == -1): the
/// REPL task IS the one blocked running this command, so nothing else is free
/// to read stdin concurrently, and ESP-IDF exposes no PUBLIC non-blocking
/// "bytes waiting" query for the USB-CDC console (only a private one explicitly
/// marked "not for applications" — esp_private/usb_console.h). A USB user
/// currently has to wait out the (now bounded-by-abort-elsewhere) sweep or
/// power-cycle; the WiFi/TCP path — the reported failure, `motor stop` sent
/// over WiFi was never read — now aborts immediately.
/// `motor sweep` の非ブロッキング中断チェック（下記 collectPower() が使用）:
/// コマンドを発行した TCP セッションが前回ポーリング以降に何か送信（または切断）
/// していれば true。MSG_DONTWAIT（api_task.cpp の UDP コマンドソケットと同じ
/// 呼び出し単位の非ブロッキング流儀）を使うので 20ms サンプリングループを止めない。
///
/// USB REPL から起動したスイープでは常に false（s_active_tcp_client_fd == -1）—
/// REPL タスク自身が本コマンドの実行でブロックしており並行して stdin を読める者が
/// おらず、ESP-IDF は USB-CDC コンソールの「入力待ち」を問い合わせる公開の非ブロッキング
/// API を持たない（非公開・「アプリから呼ぶな」明記の esp_private/usb_console.h の
/// みが存在）。USB ユーザーは現状スイープを待つか電源断で対応するしかないが、
/// WiFi/TCP 経路（報告された不具合 — WiFi 越しの `motor stop` が読まれない）は
/// 今すぐ中断できるようになった。
bool tcpAbortRequested()
{
    if (s_active_tcp_client_fd < 0) {
        return false;   // USB REPL command — see block comment above / USB REPL コマンド
    }
    char buf[32];
    const ssize_t received =
        ::recv(s_active_tcp_client_fd, buf, sizeof(buf), MSG_DONTWAIT);
    if (received < 0) {
        return false;   // EAGAIN/EWOULDBLOCK — nothing queued yet / 未着（通常経路）
    }
    if (received > 0) {
        // Drain any further immediately-available bytes (best effort, bounded)
        // so the rest of the line / a queued follow-up command is not replayed
        // once the TCP recv loop in serveTcpClient() resumes.
        // 直後にすぐ読める分もあれば読み捨てる（ベストエフォート、回数上限あり）—
        // serveTcpClient() の TCP recv ループ再開後にはぐれコマンドが再生されない
        // ように。
        for (int extra = 0; extra < 4; ++extra) {
            if (::recv(s_active_tcp_client_fd, buf, sizeof(buf), MSG_DONTWAIT) <= 0) {
                break;
            }
        }
    }
    return true;   // received > 0 (input) or == 0 (peer hung up) / 入力あり、または切断
}

/// Aggregated current/voltage statistics over a sweep sampling window (see
/// `collectPower`): sample count, mean/std current [mA], mean voltage [V], and
/// whether the poll loop was cut short by an abort (input on the issuing TCP
/// session, or pack voltage at/under safety.battery.low_v).
/// スイープの1サンプリング区間の集計結果（`collectPower` 参照）: サンプル数、
/// 平均/標準偏差電流[mA]、平均電圧[V]、および中断（発行元 TCP セッションへの入力、
/// または電圧が safety.battery.low_v 以下）でポーリングを打ち切ったか。
struct SweepStat {
    int   n       = 0;
    float i_mean  = 0.0f;
    float i_std   = 0.0f;
    float v_mean  = 0.0f;
    bool  aborted = false;
};

/// Poll `sensor_power` for `duration_us`, deduping repeated Latest-topic reads
/// by timestamp (the 10Hz PowerTask publishes slower than our poll), and
/// average current/voltage over the BACK part only — samples inside the first
/// `skip_us` (start-up transient / not-yet-settled window) are polled but not
/// counted. If `keepalive` is non-null, its MotorTest FACT is republished
/// every MOTOR_SWEEP_KEEPALIVE_US in ONE continuous timer spanning both the
/// skip and sample windows, so a call longer than MOTOR_TEST_DURATION_US never
/// drops to zero duty mid-sample (splitting this into two separate polling
/// calls would each reset the keepalive timer and could stack close to the
/// 2s auto-expiry at the boundary — kept as one loop to avoid that).
///
/// Every 20ms tick also checks two abort conditions BEFORE sampling: input on
/// the issuing TCP session (tcpAbortRequested()) and pack voltage at/under
/// `low_v_threshold` (safety.battery.low_v, cached once by the caller — see
/// cmd_motor_sweep). Either one stops the motor (publishMotorStop()) and
/// breaks out of the poll loop immediately, with SweepStat::aborted set and the
/// reason already printed — the caller just needs to check `.aborted` and
/// return without printing its own summary line.
/// `duration_us` の間 `sensor_power` をポーリングし、Latest トピックの重複読み
/// （PowerTask の 10Hz publish はポーリングより遅い）をタイムスタンプで除きながら
/// 後半のみ平均する — 先頭 `skip_us`（起動過渡・未整定区間）はポーリングするが
/// 集計しない。`keepalive` が非 null なら MOTOR_SWEEP_KEEPALIVE_US 毎に MotorTest
/// FACT を再発行する。このタイマーは skip・sample 両区間にまたがる単一ループで
/// 動かす（2回のポーリング呼び出しに分けると各々でタイマーがリセットされ、境界で
/// 2秒の自動失効に接近しうるため、1ループに保つ）。
///
/// 20ms ティック毎に、サンプリングの前に2つの中断条件もチェックする: 発行元 TCP
/// セッションへの入力（tcpAbortRequested()）と、`low_v_threshold`
/// （safety.battery.low_v、呼び出し元が1回キャッシュ — cmd_motor_sweep 参照）以下の
/// パック電圧。どちらもモータを止め（publishMotorStop()）ポーリングループを即座に
/// 抜ける — SweepStat::aborted を立て、理由は既に印字済みなので、呼び出し元は
/// `.aborted` を見て自前のサマリ行を印字せず戻ればよい。
SweepStat collectPower(uint32_t duration_us, uint32_t skip_us, const sf::MotorTest* keepalive,
                        float low_v_threshold)
{
    const uint32_t start = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t last_ts = sf::sensor_power.latest().timestamp;  // seed: don't recount the stale sample
    uint32_t last_keepalive = start;
    double sum_i = 0.0, sum_i2 = 0.0, sum_v = 0.0;
    int n = 0;
    SweepStat stat{};

    while (static_cast<uint32_t>(esp_timer_get_time()) - start < duration_us) {
        vTaskDelay(pdMS_TO_TICKS(config::MOTOR_SWEEP_POLL_MS));
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

        // Abort request: the channel that issued `motor sweep` has sent
        // something (or hung up) — stop NOW rather than finishing the up to
        // ~45s worst case run (MOTOR_SWEEP_MAX_SEC, config.hpp).
        // 中断要求: `motor sweep` を発行したチャネルから入力があった（または
        // 切断された）— 最悪 ~45秒（MOTOR_SWEEP_MAX_SEC, config.hpp）かかる実行を
        // 最後まで待たず今すぐ止める。
        if (tcpAbortRequested()) {
            publishMotorStop();
            stat.aborted = true;
            std::printf("SWEEP ABORTED (input received)\n");
            break;
        }

        // Low-battery auto-abort. MOTOR_SWEEP_VOLTAGE_VALID_MIN guards against a
        // stale/zero reading before PowerTask's first publish (same idiom as
        // sf_notify's low-battery LED guard, notify.cpp: kVoltageValidMin).
        // 低電圧自動中止。MOTOR_SWEEP_VOLTAGE_VALID_MIN は PowerTask 初回 publish
        // 前の未初期化/ゼロ読みに対するガード（sf_notify の低電圧 LED ガードと
        // 同じ流儀, notify.cpp の kVoltageValidMin）。
        const sf::PowerData d = sf::sensor_power.latest();
        if (d.voltage > config::MOTOR_SWEEP_VOLTAGE_VALID_MIN && d.voltage <= low_v_threshold) {
            publishMotorStop();
            stat.aborted = true;
            std::printf("SWEEP ABORTED (low battery %.2fV)\n", d.voltage);
            break;
        }

        if (keepalive != nullptr && now - last_keepalive >= config::MOTOR_SWEEP_KEEPALIVE_US) {
            sf::MotorTest t = *keepalive;
            t.timestamp = now;
            t.expiry_us = now + config::MOTOR_TEST_DURATION_US;
            sf::motor_test.publish(t);
            last_keepalive = now;
        }
        if (now - start < skip_us) {
            continue;   // still settling — keepalive only, don't sample / 整定待ち
        }
        if (d.timestamp == last_ts) {
            continue;   // no fresh sample yet / まだ新規サンプルなし
        }
        last_ts = d.timestamp;
        sum_i  += d.current;
        sum_i2 += static_cast<double>(d.current) * d.current;
        sum_v  += d.voltage;
        ++n;
    }

    stat.n = n;
    if (n > 0) {
        stat.i_mean = static_cast<float>(sum_i / n);
        stat.v_mean = static_cast<float>(sum_v / n);
        const double variance = (sum_i2 / n) - static_cast<double>(stat.i_mean) * stat.i_mean;
        stat.i_std = static_cast<float>(variance > 0.0 ? std::sqrt(variance) : 0.0);
    }
    return stat;
}

/// Spin one motor at `duty` for `duration_us`, sampling only the back half
/// (MOTOR_SWEEP_SETTLE_FRACTION skipped as start-up transient) via
/// `collectPower`. Stops the motor and rests MOTOR_SWEEP_REST_US before
/// returning so the next motor's baseline is not contaminated by this one's
/// spin-down — unless collectPower() aborted (input / low battery), in which
/// case the motor is already stopped and the rest is skipped so control
/// returns to the operator immediately instead of eating one more second.
/// 1モータを `duty` で `duration_us` の間回し、`collectPower` で後半のみ
/// サンプルする（先頭 MOTOR_SWEEP_SETTLE_FRACTION は起動過渡としてスキップ）。
/// モータ停止後 MOTOR_SWEEP_REST_US 休止してから戻り、次のモータの計測がこの
/// モータの回転停止過渡で汚染されないようにする — ただし collectPower() が
/// 中断（入力/低電圧）した場合はモータは既に停止済みなので休止をスキップし、
/// 追加で1秒待たせず即座に操縦者へ制御を戻す。
SweepStat runSingleMotor(uint8_t motor_id, float duty, uint32_t duration_us, float low_v_threshold)
{
    const uint32_t settle_us = static_cast<uint32_t>(duration_us * config::MOTOR_SWEEP_SETTLE_FRACTION);
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

    sf::MotorTest t{};
    t.active    = true;
    t.motor_id  = motor_id;
    t.duty      = duty;
    t.expiry_us = now + config::MOTOR_TEST_DURATION_US;
    t.timestamp = now;
    sf::motor_test.publish(t);

    const SweepStat stat = collectPower(duration_us, settle_us, &t, low_v_threshold);

    publishMotorStop();
    if (!stat.aborted) {
        vTaskDelay(pdMS_TO_TICKS(config::MOTOR_SWEEP_REST_US / 1000));
    }
    return stat;
}

/// `motor sweep [duty% sec_per_motor]` — bench CW/CCW current-asymmetry sweep.
/// Spins M1..M4 in turn at a fixed duty, measures an all-off baseline first,
/// samples the back half of each motor's run (skipping the spin-up transient),
/// and prints a per-motor current/voltage table plus one machine-readable
/// summary line. DISARMED ONLY (explicit refusal — unlike `motor test`, this
/// blocks the CLI task for many seconds, so silently doing nothing while armed
/// would just waste that wait). Blocking is vTaskDelay-based, the same
/// ground-tool-blocking precedent as `magcal start`. Abortable mid-run from the
/// issuing TCP session (any input) or automatically on low battery — see
/// collectPower()/tcpAbortRequested() above.
/// `motor sweep [duty% sec_per_motor]` — ベンチの CW/CCW 電流非対称スイープ。
/// M1..M4 を順に固定 duty で回し、まず全停止ベースラインを測定、各モータの回転
/// 区間の後半（起動過渡を除く）をサンプルし、モータ毎の電流/電圧表と機械可読な
/// サマリ行を1行印字する。**disarmed 限定**（明示拒否 — `motor test` と異なり
/// 本コマンドは数秒間 CLI タスクをブロックするため、armed 中に無言で何もせず
/// 待つだけになるのを避ける）。ブロックは vTaskDelay ベースで `magcal start` と
/// 同じ地上ツール・ブロッキングの前例に倣う。発行元 TCP セッションへの入力で実行中
/// 中断可能、低電圧では自動中断（上記 collectPower()/tcpAbortRequested() 参照）。
int cmd_motor_sweep(int argc, char** argv)
{
    if (sf::isArmed(static_cast<sf::FlightState>(sf::system_mode.latest().state))) {
        std::printf("refused: motor sweep needs the craft disarmed\n");
        return 1;
    }

    int duty_pct = config::MOTOR_SWEEP_DEFAULT_DUTY_PCT;
    float sec    = config::MOTOR_SWEEP_DEFAULT_SEC;
    if (argc >= 3) duty_pct = std::atoi(argv[2]);
    if (argc >= 4) sec      = std::strtof(argv[3], nullptr);
    if (duty_pct < config::MOTOR_SWEEP_MIN_DUTY_PCT) duty_pct = config::MOTOR_SWEEP_MIN_DUTY_PCT;
    if (duty_pct > config::MOTOR_SWEEP_MAX_DUTY_PCT) duty_pct = config::MOTOR_SWEEP_MAX_DUTY_PCT;
    if (sec < config::MOTOR_SWEEP_MIN_SEC) sec = config::MOTOR_SWEEP_MIN_SEC;
    if (sec > config::MOTOR_SWEEP_MAX_SEC) sec = config::MOTOR_SWEEP_MAX_SEC;

    // Cache the low-battery abort threshold once (safety.battery.low_v) — same
    // one-shot-read rationale as sf_notify's low_v_threshold_ (notify.cpp): the
    // value rarely changes mid-run, and collectPower() polls it every 20ms.
    // 低電圧中断閾値（safety.battery.low_v）を1回だけ読みキャッシュ — sf_notify の
    // low_v_threshold_（notify.cpp）と同じ一括読みの理由: 実行中はほぼ変わらず、
    // collectPower() は 20ms 毎にポーリングする。
    float low_v = 3.4f;   // matches params.cpp's compiled default / params.cpp のコンパイル時既定値
    sf::params::get_float("safety.battery.low_v", low_v);

    const float total_s = (config::MOTOR_SWEEP_BASELINE_US / 1.0e6f) +
                           4.0f * (sec + config::MOTOR_SWEEP_REST_US / 1.0e6f);
    std::printf("motor sweep: duty=%d%% sec/motor=%.1f (~%.0fs total) — "
                "PROPS OFF or hold the craft, DISARMED only. Send anything "
                "(or Enter) on this session to abort.\n", duty_pct, sec, total_s);

    publishMotorStop();   // clear any leftover test before the baseline / ベースライン前のクリア
    const SweepStat base = collectPower(config::MOTOR_SWEEP_BASELINE_US, 0, nullptr, low_v);
    if (base.aborted) {
        return 1;   // ABORTED message already printed by collectPower() / メッセージは collectPower() が印字済み
    }
    std::printf("baseline    : V=%.2fV I=%.0fmA (n=%d)\n", base.v_mean, base.i_mean, base.n);

    static const char* kNames[4] = {"M1(FR,CCW)", "M2(RR,CW) ", "M3(RL,CCW)", "M4(FL,CW) "};
    SweepStat results[4];
    const uint32_t duration_us = static_cast<uint32_t>(sec * 1.0e6f);
    for (int i = 0; i < 4; ++i) {
        results[i] = runSingleMotor(static_cast<uint8_t>(i), duty_pct / 100.0f, duration_us, low_v);
        if (results[i].aborted) {
            return 1;   // ABORTED message already printed by collectPower() / 同上
        }
        std::printf("%s : V=%.2fV I=%.0fmA std=%.0f dI=%+.0fmA (n=%d)\n", kNames[i],
                    results[i].v_mean, results[i].i_mean, results[i].i_std,
                    results[i].i_mean - base.i_mean, results[i].n);
    }

    std::printf("SWEEP m1=%.1f m2=%.1f m3=%.1f m4=%.1f base=%.1f duty=%d vbat=%.2f\n",
                results[0].i_mean, results[1].i_mean, results[2].i_mean, results[3].i_mean,
                base.i_mean, duty_pct, base.v_mean);
    return 0;
}

/// `motor [test <1-4> <0-100> | all <0-100> | sweep [duty% sec] | stop]` — bench motor
/// wiring/direction check + CW/CCW current-asymmetry sweep. `test`/`all` spin at a fixed
/// duty for MOTOR_TEST_DURATION_US, DISARMED ONLY — refused with a message, same as
/// `sweep` (cmd_motor_sweep): ControlTask already ignores this FACT while armed, but a
/// silent no-op that still prints a success-looking line is a UX trap, so we refuse
/// up front instead. `sweep` is a longer, self-contained measurement — see
/// `cmd_motor_sweep`. Publishes a MotorTest FACT. IDs: M1=FR M2=RR M3=RL M4=FL. KEEP
/// PROPS OFF / hold the craft. `motor stop` ends it early (always allowed — it is the
/// safety net, not a test).
/// `motor [test <1-4> <0-100> | all <0-100> | sweep [duty% sec] | stop]` — ベンチのモータ
/// 配線/回転方向確認＋CW/CCW 電流非対称スイープ。`test`/`all` は MOTOR_TEST_DURATION_US の
/// 間固定 duty で回転、**disarmed 限定** — `sweep`（cmd_motor_sweep）と同様メッセージ付きで
/// 拒否する: ControlTask は armed 中このFACTを元々無視するが、それでも成功風の行を印字する
/// 無言の空振りは UX の罠なので、事前に拒否する。`sweep` はより長い自己完結型の計測
/// （`cmd_motor_sweep` 参照）。MotorTest を発行。ID: M1=FR M2=RR M3=RL M4=FL。
/// **プロペラを外すか機体を保持**。`motor stop` で即停止（常に許可 — テストではなく安全弁）。
int cmd_motor(int argc, char** argv)
{
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

    if (argc >= 2 && std::strcmp(argv[1], "stop") == 0) {
        publishMotorStop();
        std::printf("motor test stopped\n");
        return 0;
    }

    if (argc >= 2 && std::strcmp(argv[1], "sweep") == 0) {
        return cmd_motor_sweep(argc, argv);
    }

    bool all = (argc >= 2 && std::strcmp(argv[1], "all") == 0);
    bool test = (argc >= 2 && std::strcmp(argv[1], "test") == 0);
    if ((test && argc >= 4) || (all && argc >= 3)) {
        // Same disarmed-only guard as `motor sweep` (cmd_motor_sweep) — see the
        // function doc comment above for why this refuses rather than staying a
        // silent, ControlTask-ignored no-op.
        // `motor sweep`（cmd_motor_sweep）と同じ disarmed 限定ガード — 無言で
        // ControlTask に無視される空振りのままにせず拒否する理由は上の関数
        // ドキュメントコメント参照。
        if (sf::isArmed(static_cast<sf::FlightState>(sf::system_mode.latest().state))) {
            std::printf("refused: motor test needs the craft disarmed\n");
            return 1;
        }
        int pct = std::atoi(all ? argv[2] : argv[3]);
        if (pct < 0)   pct = 0;
        if (pct > 100) pct = 100;
        sf::MotorTest t{};
        t.active    = true;
        t.duty      = pct / 100.0f;
        t.expiry_us = now + config::MOTOR_TEST_DURATION_US;
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
    std::printf("usage: motor [test <1-4> <0-100> | all <0-100> | sweep [duty%% sec] | stop]\n");
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
        // ssid/pass write commits to NVS; the flash sector erase stalls the 400Hz
        // loop >10ms exactly like `param save` / `magcal save`, which both refuse
        // while armed. Mirror that guard here (in flight the stall would zero the
        // motors). `wifi mode` above only touches RAM, so it needs no guard.
        // ssid/pass の書込みは NVS commit を伴い、フラッシュセクタ消去が `param save` /
        // `magcal save`（いずれも armed 中拒否）と同様に 400Hz ループを 10ms 超ストール
        // させる。同じガードをここにも置く（飛行中はストール分モータがゼロになる）。
        // 上の `wifi mode` は RAM のみ変更ゆえガード不要。
        const sf::SystemMode mode = sf::system_mode.latest();
        if (sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
            std::printf("refused: wifi config writes NVS (flash erase stalls the "
                        "400Hz loop) — land/disarm first\n");
            return 1;
        }
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

/// `magcal [start|stop|status|save|clear]` — magnetometer hard/soft-iron
/// calibration. The calibrator is OWNED by MagTask; this command only publishes
/// verbs (mag_command) and reads the status topic (mag_cal_status) — R5.
/// Flow: `magcal start` → rotate the craft in all orientations (figure-8) →
/// `magcal stop` (computes the sphere fit) → `magcal save` (NVS). The new
/// calibration takes effect immediately; the ESKF yaw aiding engages at the
/// next boot/recalibration (mag reference capture).
/// `magcal [start|stop|status|save|clear]` — 地磁気のハード/ソフトアイアン校正。
/// 校正器は MagTask が所有し、本コマンドは verb の発行（mag_command）と状態トピック
/// （mag_cal_status）の読み出しのみ行う（R5）。
/// 手順: `magcal start` → 機体を全方位に回す（8の字）→ `magcal stop`（球面フィット
/// 計算）→ `magcal save`（NVS）。補正は即時有効。ESKF のヨー補助は次回起動/再校正
/// 時（磁気参照捕捉）に係合する。
const char* magCalStateName(uint8_t state)
{
    // Mirrors stampfly::MagCalibrator::State order (sf_hal_bmm150).
    // stampfly::MagCalibrator::State の順序を写す（sf_hal_bmm150）。
    static const char* kNames[] = {"IDLE", "COLLECTING", "COMPUTING", "DONE", "ERROR"};
    return (state < 5) ? kNames[state] : "?";
}

void printMagCalStatus(const sf::MagCalStatus& st)
{
    std::printf("state: %s  samples: %u  calibrated: %s\n",
                magCalStateName(st.state), st.sample_count, st.valid ? "yes" : "no");
    if (st.valid) {
        std::printf("offset: [%.1f %.1f %.1f] uT  scale: [%.2f %.2f %.2f]  fitness: %.2f\n",
                    st.offset[0], st.offset[1], st.offset[2],
                    st.scale[0], st.scale[1], st.scale[2], st.fitness);
    }
}

int cmd_magcal(int argc, char** argv)
{
    if (argc < 2 || std::strcmp(argv[1], "status") == 0) {
        printMagCalStatus(sf::mag_cal_status.latest());
        if (argc < 2) {
            std::printf("usage: magcal [start|stop|status|save|clear]\n"
                        "  start  - begin collection, then rotate the craft (figure-8)\n"
                        "  stop   - finish collection and compute the fit\n"
                        "  save   - persist to NVS (disarmed only)\n"
                        "  clear  - erase the saved calibration\n");
        }
        return 0;
    }

    sf::MagCalCmd verb = sf::MagCalCmd::None;
    if      (std::strcmp(argv[1], "start") == 0) verb = sf::MagCalCmd::Start;
    else if (std::strcmp(argv[1], "stop")  == 0) verb = sf::MagCalCmd::Stop;
    else if (std::strcmp(argv[1], "save")  == 0) verb = sf::MagCalCmd::Save;
    else if (std::strcmp(argv[1], "clear") == 0) verb = sf::MagCalCmd::Clear;
    else {
        std::printf("unknown subcommand: %s\n", argv[1]);
        return 1;
    }

    if (verb == sf::MagCalCmd::Start &&
        sf::isArmed(static_cast<sf::FlightState>(sf::system_mode.latest().state))) {
        std::printf("refused: magcal needs the craft disarmed in your hands\n");
        return 1;
    }

    sf::mag_command.publish(
        {static_cast<uint8_t>(verb), static_cast<uint32_t>(esp_timer_get_time())});

    // Give MagTask one cycle (25Hz = 40ms) to execute, then report the outcome.
    // MagTask の 1 周期（25Hz = 40ms）待って結果を報告する。
    vTaskDelay(pdMS_TO_TICKS(100));
    printMagCalStatus(sf::mag_cal_status.latest());

    if (verb == sf::MagCalCmd::Start) {
        std::printf("rotate the craft in ALL orientations (figure-8), then run "
                    "'magcal stop'\n('magcal status' shows the sample count)\n");
    } else if (verb == sf::MagCalCmd::Save) {
        std::printf("saved. yaw aiding engages at the next boot "
                    "(set 'param set eskf.use_mag 1' to enable fusion)\n");
    }
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

    // This session now owns the CLI — route `motor sweep` abort polling to it
    // (s_active_tcp_client_fd / tcpAbortRequested(), above collectPower()).
    // Restored to -1 (no active TCP session) when this client disconnects,
    // below.
    // このセッションが CLI を保持する間、`motor sweep` の中断ポーリングをここへ
    // 向ける（collectPower() 手前の s_active_tcp_client_fd / tcpAbortRequested()
    // 参照）。このクライアントが切断したら下で -1（TCP セッション無し）へ戻す。
    s_active_tcp_client_fd = client_fd;

    std::printf("StampFly vehicle TCP CLI — type 'help' for commands\n");
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

    s_active_tcp_client_fd = -1;
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
    {"sound",   "sound [on|off|test [start|ok|fail]] — buzzer mute / play a cue", &cmd_sound},
    {"led",     "led <0-255> — body LED brightness",             &cmd_led},
    {"motor",   "motor [test <1-4> <0-100>|all <0-100>|sweep [duty% sec]|stop] — bench test (disarmed)", &cmd_motor},
    {"wifi",    "wifi [show|mode <sta|ap>|ssid <name>|pass <secret>] — telemetry WiFi", &cmd_wifi},
    {"magcal",  "magcal [start|stop|status|save|clear] — magnetometer calibration", &cmd_magcal},
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
    // 192.168.10.1). On the SIL host the inert socket shim never accepts, so
    // this loop just idles.
    // 以降は REPL タスクが USB の対話入力を所有する。本タスクは TCP CLI を提供する
    // （要件§7: CLI = USB Serial / TCP）: モータテスト等のベンチ作業は USB ケーブルを
    // 外した電池駆動で行うため、コマンドは WiFi 経由で届く必要がある。接続は
    // `nc <機体IP> 23`（SoftAP なら 192.168.10.1）。SIL ホストでは inert ソケットシムが
    // accept しないため、このループは待機するだけ。
    runTcpCliServer();
}
