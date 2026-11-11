/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario.cpp
 * @brief Input-scenario parser + deterministic driver task implementation.
 *        入力シナリオのパーサ＋決定論的ドライバタスクの実装。
 */

#include "scenario.hpp"
#include "scenario_inject.hpp"
#include "console_feeder.hpp"
#include "emu_record.hpp"
#include "virtual_board.hpp"     // P7: sils_board_set_wind / sils_board_set_motor_health
#include "params.hpp"            // scenario `param`: set a firmware parameter mid-run / 実行中のパラメータ設定

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// API-injection hook, registered by the TARGET-specific glue (only vehicle
// has an ApiTask; referencing its symbol directly would break the link for
// any emu target that has no ApiTask). Unregistered targets record the line
// as unsupported.
// API 注入フック。ターゲット別グルーが登録する（ApiTask を持つのは vehicle のみ。
// シンボル直参照は ApiTask を持たない emu ターゲットのリンクを壊す）。
// 未登録ターゲットでは未対応として記録。
static void (*g_api_inject_fn)(const char*) = nullptr;
extern "C" void sils_scenario_register_api_inject(void (*fn)(const char*))
{
    g_api_inject_fn = fn;
}

enum class Channel { Rc, RcRamp, Key, Btn, Wind, Fault, Bias, Handle, Api, TofSilent, TofInvalid, TofDeadZone, Param, Battery };

// Which injector an "rc"-family event uses: the paired transmitter (rc), a
// different non-paired one exercised AFTER bind (rc_foreign — crosstalk test), or
// one of the two roles in the own-address-filter test (rc_ctrl_a: addresses a
// DIFFERENT vehicle; rc_ctrl_b: addresses THIS vehicle) — pairing-methods-plan.md
// §4.4.
// "rc" 系事象がどの注入元を使うか: ペア送信機(rc)、バインド後に試す別の未ペア送信機
// (rc_foreign — 混信試験)、または自機宛フィルタ試験の2役(rc_ctrl_a: 別の機体宛、
// rc_ctrl_b: この機体宛) — pairing-methods-plan.md §4.4。
enum class RcSource { Paired, Foreign, ControllerA, ControllerB };

struct Event {
    int64_t at_us = 0;        // absolute virtual time (frozen) / 絶対仮想時刻
    Channel ch = Channel::Rc;
    int     line = 0;         // source line (diagnostics) / ソース行

    // rc
    uint16_t thr = 0, roll = 0, pitch = 0, yaw = 0;
    uint8_t  arm = 0;
    uint8_t  alt = 0;         // 1 => CTRL_FLAG_ALT_MODE (ALTITUDE_HOLD) / 1 で高度保持
    uint8_t  acro = 0;        // 1 => CTRL_FLAG_MODE (ACRO/rate) / 1 で ACRO（角速度）
    uint8_t  pos = 0;         // 1 => CTRL_FLAG_POS_MODE (POSITION_HOLD) / 1 で位置保持
    uint8_t  flip = 0;        // 1 => CTRL_FLAG_FLIP (FLIP button, Phase 3) / 1 で FLIP ボタン
    RcSource source = RcSource::Paired;  // which injector (see RcSource above) / 注入元
    int      hold_ms = 0;     // 0 = single frame / 0=単発
    int      rate_hz = 20;

    // rc_ramp
    int      ramp_field = 0;  // 0=throttle 1=roll 2=pitch 3=yaw
    uint16_t ramp_from = 0, ramp_to = 0;
    int      ramp_step = 1;

    // key
    std::string text;

    // wind (P7): external force NED [N]; dur_ms>0 = gust pulse (revert to 0 after).
    // wind (P7): NED 外乱力 [N]。dur_ms>0 で突風パルス（後で 0 に戻す）。
    float wind_fx = 0.0f, wind_fy = 0.0f, wind_fz = 0.0f;
    int   wind_dur_ms = 0;

    // fault (P7): degrade one motor's thrust health gain (motor 0..3, gain 0..1).
    // fault (P7): 1モータの推力健全度ゲインを劣化（motor 0..3, gain 0..1）。
    int   fault_motor = 0;
    float fault_gain = 1.0f;

    // bias (P2-3): deterministic raw IMU bias (body FRD), accel [m/s²] + gyro [rad/s].
    // bias (P2-3): 決定論的な生 IMU バイアス（機体 FRD）、accel[m/s²] + gyro[rad/s]。
    float bias_ax = 0.0f, bias_ay = 0.0f, bias_az = 0.0f;
    float bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;

    // tof_silent: stop (1) / resume (0) the downward ToF's sample output (fault injection).
    // tof_silent: 下向き ToF のサンプル出力を停止（1）/再開（0）（故障注入）。
    int   tof_silent = 0;

    // tof_invalid: make every downward-ToF sample invalid (1) / restore (0) (fault injection).
    // tof_invalid: 下向き ToF の全サンプルを無効に（1）/元に戻す（0）（故障注入）。
    int   tof_invalid = 0;

    // tof_deadzone: model (1, the default of a run) / ignore (0) the downward ToF's close-range dead zone.
    // tof_deadzone: 下向き ToF の近距離の不感帯を模擬する（1、どの実行でも既定）/ しない（0）。
    int   tof_deadzone = 1;

    // param: firmware parameter name (text) and value, applied mid-run (fault / error injection).
    // param: ファームのパラメータ名（text）と値。実行中に適用する（故障・誤差の注入）。
    double param_value = 0.0;

    // battery: the held battery terminal voltage [V] (> 0), or 0 = release the hold (the battery model decides again).
    // battery: 保持する電池端子電圧 [V]（0 より大きい）。0 は保持の解除（電池モデルが再び決める）。
    float battery_v = 0.0f;

    // handle (P8): physical handling maneuver — carry altitude [m up], placement
    // x/y [NED m], and lift/carry/place phase durations [ms]. Instantaneous on the
    // timeline (triggers the Plant trajectory; surrounding disarmed rc holds keep the
    // link alive and occupy the maneuver's wall-clock).
    // handle (P8): 物理ハンドリング動作 — carry 高度[上方m]、設置 x/y[NED m]、lift/carry/
    // place 各相の所要時間[ms]。タイムライン上は瞬時（Plant 軌道を起動。周囲の disarmed rc
    // ホールドがリンクを維持し動作の実時間を占有する）。
    float handle_carry_alt = 0.40f;
    float handle_place_x = 0.0f, handle_place_y = 0.0f;
    int   handle_lift_ms = 1500, handle_carry_ms = 1500, handle_place_ms = 1500;

    // btn (still deferred): keep raw args for the warn note.
    // btn（未実装）: warn 用に生引数を保持。
    std::string raw;
};

std::vector<Event> g_events;
bool g_active = false;

// period (ms) for a given send rate, integer-floored (1 tick = 1 ms). The parser
// validates rate_hz in [1,1000] so period is always >= 1 (no vTaskDelay(0)).
// 送信周期(ms)。パーサが rate_hz を [1,1000] に制限するので period>=1（vTaskDelay(0)なし）。
int period_ms(int rate_hz) { return rate_hz > 0 ? (1000 / rate_hz) : 50; }

// Frames the driver emits for an rc hold: at least one for any positive hold,
// rounded to the nearest send period (hold_ms is quantized to the period). The
// 64-bit product guards against overflow. ONE helper shared by the parser
// (timeline duration) and the driver (emit loop) so '+'-relative timing matches
// the bytes actually sent — and a sub-period hold can never emit zero frames.
// rc ホールドの発射フレーム数: 正のホールドは最低1、周期最近接に丸め（64bitで桁あふれ回避）。
// パーサ（時間幅）とドライバ（発射ループ）が同一ヘルパを共有 → '+' 相対時刻が実送出と一致。
long long rc_hold_frames(int hold_ms, int rate_hz)
{
    if (hold_ms <= 0) return 0;
    long long f = ((long long)hold_ms * rate_hz + 500) / 1000;  // round to nearest
    return f < 1 ? 1 : f;
}

int ramp_frames(uint16_t from, uint16_t to, int step)
{
    int s = step < 0 ? -step : step;
    if (s == 0) return 1;
    int span = (to >= from) ? (to - from) : (from - to);
    return span / s + 1;
}

// Duration the event occupies on the timeline (for '+'-relative resolution).
// For rc this is EXACTLY what the driver spends (frames * period), so the parser
// and driver agree even when rate_hz does not divide 1000.
// 事象がタイムライン上で占める時間。rc はドライバの実消費（frames*period）と厳密一致。
int64_t event_duration_us(const Event& e)
{
    switch (e.ch) {
        case Channel::Rc:
            return rc_hold_frames(e.hold_ms, e.rate_hz)
                 * (int64_t)period_ms(e.rate_hz) * 1000;
        case Channel::RcRamp:
            return (int64_t)ramp_frames(e.ramp_from, e.ramp_to, e.ramp_step)
                 * period_ms(e.rate_hz) * 1000;
        case Channel::Wind:
            // A gust pulse (dur_ms>0) holds the wind then reverts, so it occupies
            // dur_ms on the timeline; a sustained step (dur_ms=0) is instantaneous.
            // 突風パルス(dur_ms>0)は風を保持後に戻すので dur_ms 占有。定常(0)は瞬時。
            return (int64_t)e.wind_dur_ms * 1000;
        default:
            return 0;  // key/btn/fault are instantaneous on the timeline
    }
}

void err(const char* path, int line, const std::string& msg)
{
    std::fprintf(stderr, "[scenario] %s:%d: %s\n", path, line, msg.c_str());
}

// Parse a quoted "..." string (with \n \t \r \\ \" escapes) from `rest`.
bool parse_quoted(const std::string& rest, std::string& out)
{
    size_t i = rest.find('"');
    if (i == std::string::npos) return false;
    ++i;
    out.clear();
    for (; i < rest.size(); ++i) {
        char c = rest[i];
        if (c == '"') return true;               // closing quote
        if (c == '\\' && i + 1 < rest.size()) {
            char n = rest[++i];
            switch (n) {
                case 'n': out.push_back('\n'); break;
                case 't': out.push_back('\t'); break;
                case 'r': out.push_back('\r'); break;
                case '\\': out.push_back('\\'); break;
                case '"': out.push_back('"'); break;
                default: out.push_back(n); break;
            }
        } else {
            out.push_back(c);
        }
    }
    return false;  // unterminated
}

bool parse_rc_field(const std::string& s, int& field)
{
    if (s == "throttle") { field = 0; return true; }
    if (s == "roll")     { field = 1; return true; }
    if (s == "pitch")    { field = 2; return true; }
    if (s == "yaw")      { field = 3; return true; }
    return false;
}

bool in_adc(long v) { return v >= 0 && v <= 4095; }

// Parse the downward-ToF fault-injection channels (tof_silent, tof_invalid, tof_deadzone). Returns 1 when `ch` is
// one of them and the line is valid, -1 on an error (already reported), 0 when `ch` is some other channel.
// 下向き ToF の故障注入のチャネル（tof_silent、tof_invalid、tof_deadzone）を解釈する。`ch` がそのどれかで行が
// 正しければ 1、エラー（報告済み）なら −1、別のチャネルなら 0 を返す。
int parseTofChannel(const std::string& ch, std::istringstream& iss, Event& e, const char* path, int lineno)
{
    if (ch == "tof_silent") {
        // tof_silent <0|1> — 1 stops the downward ToF's sample output (the chip model
        // never reports "data ready"; no I2C error), 0 resumes it. Models the 2026-11
        // accident: the sensor delivers a last sample and then nothing.
        // tof_silent <0|1> — 1 で下向き ToF のサンプル出力を停止（チップモデルが「データ
        // 準備完了」を返さない。I2C エラー無し）、0 で再開。2026-11 の事故（最後のサンプルの
        // 後は何も来ない）を模擬する。
        e.ch = Channel::TofSilent;
        if (!(iss >> e.tof_silent) || (e.tof_silent != 0 && e.tof_silent != 1)) {
            err(path, lineno, "tof_silent needs <0|1>"); return -1;
        }

    } else if (ch == "tof_invalid") {
        // tof_invalid <0|1> — 1 makes every sample of the downward ToF invalid ("no target"),
        // 0 restores it. The samples keep arriving on time (no silence): a floor the sensor
        // cannot range.
        // tof_invalid <0|1> — 1 で下向き ToF の全サンプルを無効（「目標なし」）にし、0 で元に戻す。
        // サンプルは時間どおり届き続ける（出力停止ではない）: センサが測れない床。
        e.ch = Channel::TofInvalid;
        if (!(iss >> e.tof_invalid) || (e.tof_invalid != 0 && e.tof_invalid != 1)) {
            err(path, lineno, "tof_invalid needs <0|1>"); return -1;
        }

    } else if (ch == "tof_deadzone") {
        // tof_deadzone <0|1> — 1 models the close-range dead zone of the downward ToF (below 31 mm
        // there is no return pulse: the floor reads "no target"; the default of every run), 0 turns the
        // model off (the sensor then reads the true distance down to the floor, as before 2026-11-09).
        // tof_deadzone <0|1> — 1 で下向き ToF の近距離の不感帯を模擬する（約 4 cm より近いと反射パルスが
        // 無く、床は「目標なし」を返す。どの実行でも既定）、0 でモデルを切る（2026-11-09 より前と同じく、
        // 床まで真の距離を読む）。
        e.ch = Channel::TofDeadZone;
        if (!(iss >> e.tof_deadzone) || (e.tof_deadzone != 0 && e.tof_deadzone != 1)) {
            err(path, lineno, "tof_deadzone needs <0|1>"); return -1;
        }

    } else {
        return 0;
    }
    return 1;
}

// Parse the `param` channel (same return convention as parseTofChannel()).
// `param` チャネルを解釈する（戻り値の約束は parseTofChannel() と同じ）。
int parseParamChannel(const std::string& ch, std::istringstream& iss, Event& e, const char* path, int lineno)
{
    if (ch == "param") {
        // param <name> <value> — set one firmware parameter at this point of the run through the
        // type-correct setter (range-validated; the parameter's change callback fires, so a
        // controller parameter is reloaded exactly as `param set` on the console would). Used to
        // inject an ERROR into a value the firmware relies on without changing the flight before
        // it (e.g. hover.thrust_corr: the hover-thrust estimate is wrong from here on).
        // param <name> <value> — 実行のこの時点でファームのパラメータを 1 つ、型に正しいセッタで
        // 設定する（範囲検証あり。変更コールバックが呼ばれるので、制御器のパラメータはコンソールの
        // `param set` と同じく再読込される）。ファームが頼る値に「誤差」を、それ以前の飛行を
        // 変えずに注入するのに使う（例: hover.thrust_corr: 以後ホバー推力の見積もりが違う）。
        e.ch = Channel::Param;
        if (!(iss >> e.text >> e.param_value)) {
            err(path, lineno, "param needs <name> <value>"); return -1;
        }
    } else {
        return 0;
    }
    return 1;
}

// Parse the `battery` channel (same return convention as parseTofChannel()).
// `battery` チャネルを解釈する（戻り値の約束は parseTofChannel() と同じ）。
int parseBatteryChannel(const std::string& ch, std::istringstream& iss, Event& e, const char* path, int lineno)
{
    if (ch != "battery") {
        return 0;
    }
    // battery <volts|model> — hold the battery terminal voltage [V] at this value from this point of the run
    // (the motors' supply and the power monitor's reading; the Coulomb count stops), or `model` to release it
    // so that the battery model decides again. Models a weak pack whose voltage falls below the critical
    // threshold (battery_alert_policy.hpp) at a chosen time. A scenario without this event is unchanged.
    // battery <volts|model> — 実行のこの時点から、電池の端子電圧 [V] をこの値に保つ（モータの電源と電源
    // モニタの読み。クーロンカウントは止まる）。`model` で保持を解き、電池モデルが再び決める。弱った電池の
    // 電圧が選んだ時刻に危険閾値（battery_alert_policy.hpp）を下回る状況を模擬する。この事象を使わない
    // シナリオは変わらない。
    e.ch = Channel::Battery;
    std::string word;
    if (!(iss >> word)) {
        err(path, lineno, "battery needs <volts|model>"); return -1;
    }
    if (word == "model") {
        e.battery_v = 0.0f;
        return 1;
    }
    char* end = nullptr;
    e.battery_v = std::strtof(word.c_str(), &end);
    if (end == word.c_str() || *end != '\0' || !(e.battery_v > 0.0f) || e.battery_v > 5.0f) {
        err(path, lineno, "battery <volts> must be a number in (0, 5] V (or `model`)"); return -1;
    }
    return 1;
}

// Parse the `handle` channel (same return convention as parseTofChannel()).
// `handle` チャネルを解釈する（戻り値の約束は parseTofChannel() と同じ）。
int parseHandleChannel(const std::string& ch, std::istringstream& iss, Event& e, const char* path, int lineno)
{
    if (ch == "handle") {
        // handle <carry_alt_m> <place_x_ned> <place_y_ned> <lift_ms> <carry_ms> <place_ms>
        // Trigger a physical handling maneuver (lift→right→carry→place). Instantaneous
        // on the timeline; the maneuver runs in Plant time while the next disarmed rc
        // hold provides the link + wall-clock.
        // handle … — 物理ハンドリング動作を起動。タイムライン上は瞬時、動作は Plant 時間で
        // 進み、次の disarmed rc ホールドがリンクと実時間を与える。
        e.ch = Channel::Handle;
        if (!(iss >> e.handle_carry_alt >> e.handle_place_x >> e.handle_place_y
                  >> e.handle_lift_ms >> e.handle_carry_ms >> e.handle_place_ms)) {
            err(path, lineno,
                "handle needs <carry_alt_m> <place_x> <place_y> <lift_ms> <carry_ms> <place_ms>");
            return -1;
        }
        if (e.handle_carry_alt <= 0.0f) {
            err(path, lineno, "handle <carry_alt_m> must be > 0"); return -1;
        }
        if (e.handle_lift_ms <= 0 || e.handle_carry_ms <= 0 || e.handle_place_ms <= 0) {
            err(path, lineno, "handle lift/carry/place durations must be > 0 ms"); return -1;
        }
    } else {
        return 0;
    }
    return 1;
}

// Parse the channels beyond rc / key / api / wind / fault / bias / btn: the downward-ToF fault injection,
// `param` and `handle`. Return value as parseTofChannel().
// rc / key / api / wind / fault / bias / btn の他のチャネルを解釈する: 下向き ToF の故障注入、`param`、`handle`。
// 戻り値は parseTofChannel() と同じ。
int parseExtraChannel(const std::string& ch, std::istringstream& iss, Event& e, const char* path, int lineno)
{
    int result = parseTofChannel(ch, iss, e, path, lineno);
    if (result != 0) return result;
    result = parseParamChannel(ch, iss, e, path, lineno);
    if (result != 0) return result;
    result = parseBatteryChannel(ch, iss, e, path, lineno);
    if (result != 0) return result;
    return parseHandleChannel(ch, iss, e, path, lineno);
}

// Apply a downward-ToF fault-injection event (tof_silent, tof_invalid, tof_deadzone).
// 下向き ToF の故障注入の事象（tof_silent、tof_invalid、tof_deadzone）を適用する。
void applyTofEvent(const Event& e)
{
    switch (e.ch) {
        case Channel::TofSilent: {
            // Stop/resume the downward ToF sample output. The firmware must notice
            // the silence by itself (no error is reported on any bus).
            // 下向き ToF のサンプル出力を停止/再開。ファームは出力停止を自力で検知しなければ
            // ならない（どのバスにもエラーは報告されない）。
            sils_board_set_tof_silent(e.tof_silent);
            sils_emu_record_note("tof_silent", e.tof_silent ? "1 (ToF output stopped)"
                                                            : "0 (ToF output resumed)");
            break;
        }
        case Channel::TofInvalid: {
            // Make every downward-ToF sample invalid / restore it (samples keep arriving).
            // 下向き ToF の全サンプルを無効にする / 元に戻す（サンプルは届き続ける）。
            sils_board_set_tof_invalid(e.tof_invalid);
            sils_emu_record_note("tof_invalid", e.tof_invalid ? "1 (ToF samples all invalid)"
                                                              : "0 (ToF samples restored)");
            break;
        }
        case Channel::TofDeadZone: {
            // Model / ignore the close-range dead zone of the downward ToF.
            // 下向き ToF の近距離の不感帯を模擬する / しない。
            sils_board_set_tof_deadzone(e.tof_deadzone);
            sils_emu_record_note("tof_deadzone", e.tof_deadzone ? "1 (ToF close-range dead zone modelled)"
                                                                : "0 (ToF dead zone off)");
            break;
        }
        default:
            break;
    }
}

// Apply a `param` event: one firmware parameter, type looked up in the SSOT table (like the SILS_EMU_PARAMS_FILE
// path in emu_main.cpp) and set through the type-correct setter.
// `param` の事象を適用する: ファームのパラメータを 1 つ。型は SSOT テーブルで引き（emu_main.cpp の
// SILS_EMU_PARAMS_FILE 経路と同じ）、型に正しいセッタで設定する。
void applyParamEvent(const Event& e)
{
    bool ok = false;
    for (int i = 0; i < sf::params::count(); ++i) {
        const sf::params::ParamEntry* entry = sf::params::entry(i);
        if (!entry || e.text != entry->name) continue;
        switch (entry->type) {
            case sf::params::ParamType::FLOAT:
                ok = sf::params::set_float(e.text.c_str(), static_cast<float>(e.param_value));
                break;
            case sf::params::ParamType::BOOL:
                ok = sf::params::set_bool(e.text.c_str(), e.param_value != 0.0);
                break;
            default:
                ok = sf::params::set_int(e.text.c_str(), static_cast<int32_t>(e.param_value));
                break;
        }
        break;
    }
    char note[128];
    std::snprintf(note, sizeof(note), "%s = %g (%s)", e.text.c_str(), e.param_value,
                  ok ? "applied" : "REJECTED");
    sils_emu_record_note("param", note);
}

// Apply a `handle` event: trigger the physical handling maneuver on the Plant (lift, right, carry, place).
// `handle` の事象を適用する: 物理ハンドリング動作（持ち上げ・水平に戻す・運ぶ・置く）を Plant に起動する。
void applyHandleEvent(const Event& e)
{
    // P8: trigger the physical handling maneuver on the Plant (lift→right→
    // carry→place). Instantaneous here; the maneuver advances in Plant time
    // while the following disarmed rc holds keep the link alive and provide the
    // wall-clock. Durations ms → s for the Plant API.
    // P8: 物理ハンドリング動作を Plant に起動。ここでは瞬時、動作は Plant 時間で
    // 進み、続く disarmed rc ホールドがリンクを維持し実時間を与える。
    sils_board_handle_place(e.handle_carry_alt, e.handle_place_x, e.handle_place_y,
                           e.handle_lift_ms  * 1e-3f,
                           e.handle_carry_ms * 1e-3f,
                           e.handle_place_ms * 1e-3f);
    char note[112];
    std::snprintf(note, sizeof(note),
                  "handle carry=%.2fm place=(%.2f,%.2f) lift=%dms carry=%dms place=%dms",
                  e.handle_carry_alt, e.handle_place_x, e.handle_place_y,
                  e.handle_lift_ms, e.handle_carry_ms, e.handle_place_ms);
    sils_emu_record_note("handle", note);
}

// Apply a `battery` event: hold the Plant's battery terminal voltage, or release it.
// `battery` の事象を適用する: Plant の電池端子電圧を保つ、または解く。
void applyBatteryEvent(const Event& e)
{
    sils_board_hold_battery_voltage(e.battery_v);
    char note[64];
    if (e.battery_v > 0.0f) {
        std::snprintf(note, sizeof(note), "%.3f V (held)", e.battery_v);
    } else {
        std::snprintf(note, sizeof(note), "model (hold released)");
    }
    sils_emu_record_note("battery", note);
}

// Apply the events beyond rc / key / api / wind / fault / bias / btn (see parseExtraChannel()).
// rc / key / api / wind / fault / bias / btn の他の事象を適用する（parseExtraChannel() 参照）。
void applyExtraEvent(const Event& e)
{
    switch (e.ch) {
        case Channel::TofSilent:
        case Channel::TofInvalid:
        case Channel::TofDeadZone: applyTofEvent(e);    break;
        case Channel::Param:       applyParamEvent(e);  break;
        case Channel::Handle:      applyHandleEvent(e); break;
        case Channel::Battery:     applyBatteryEvent(e); break;
        default:                   break;
    }
}

}  // namespace

extern "C" {

int sils_scenario_load(const char* path)
{
    g_events.clear();
    g_active = false;
    if (path == nullptr || path[0] == '\0') return 0;

    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "[scenario] cannot open '%s'\n", path);
        return -1;
    }

    int64_t prev_end_us = 0;
    std::string line;
    int lineno = 0;

    while (std::getline(in, line)) {
        ++lineno;
        // strip comment + skip blank
        size_t hash = line.find('#');
        std::string body = (hash == std::string::npos) ? line : line.substr(0, hash);
        std::istringstream iss(body);
        std::string ttok;
        if (!(iss >> ttok)) continue;  // blank/comment-only

        // --- time token: absolute ms, or '+<ms>'/'+' relative to prev end ----
        int64_t at_us;
        if (ttok[0] == '+') {
            long ms = (ttok.size() > 1) ? std::atol(ttok.c_str() + 1) : 0;
            if (ms < 0) { err(path, lineno, "negative relative time"); return -1; }
            at_us = prev_end_us + (int64_t)ms * 1000;
        } else {
            char* endp = nullptr;
            long ms = std::strtol(ttok.c_str(), &endp, 10);
            if (endp == ttok.c_str() || *endp != '\0' || ms < 0) {
                err(path, lineno, "bad time token '" + ttok + "'"); return -1;
            }
            at_us = (int64_t)ms * 1000;
        }
        // Reject an event that starts BEFORE the previous event finishes: the
        // single-threaded driver runs events sequentially, so an absolute time
        // landing inside a prior rc hold/ramp would fire late and silently. '+'
        // resolves to exactly prev_end_us, so relative events always pass.
        // 直前事象の終了より前に始まる事象は拒否（逐次ドライバで遅延作動＝見えない不具合に）。
        if (!g_events.empty() && at_us < prev_end_us) {
            char m[96];
            std::snprintf(m, sizeof(m),
                "event starts before the previous one ends (prev ends at %lld ms)",
                (long long)(prev_end_us / 1000));
            err(path, lineno, m);
            return -1;
        }

        std::string ch;
        if (!(iss >> ch)) { err(path, lineno, "missing channel"); return -1; }

        Event e;
        e.at_us = at_us;
        e.line = lineno;

        if (ch == "rc" || ch == "rc_foreign" || ch == "rc_ctrl_a" || ch == "rc_ctrl_b") {
            // rc_foreign injects an identical ControlPacket but from a DIFFERENT
            // transmitter MAC — used to verify the pairing crosstalk filter drops
            // packets from a non-paired transmitter. rc_ctrl_a/rc_ctrl_b are the
            // two-controller own-address-filter test (pairing-methods-plan.md
            // §4.4): rc_ctrl_a addresses a DIFFERENT vehicle (must be rejected as
            // a bind candidate), rc_ctrl_b addresses THIS vehicle (must bind).
            // Same grammar as rc.
            // rc_foreign は同一の ControlPacket を別の送信機 MAC から注入する — ペアリングの
            // 混信フィルタが未ペア送信機のパケットを破棄することの検証に使う。rc_ctrl_a/
            // rc_ctrl_b は2台コントローラの自機宛フィルタ試験（pairing-methods-plan.md
            // §4.4）: rc_ctrl_a は別の機体宛（バインド候補として棄却されるべき）、rc_ctrl_b
            // はこの機体宛（バインドされるべき）。文法は rc と同じ。
            long v[5];
            for (int k = 0; k < 5; ++k) {
                if (!(iss >> v[k])) { err(path, lineno, "rc needs <thr> <roll> <pitch> <yaw> <arm>"); return -1; }
            }
            for (int k = 0; k < 4; ++k) {
                if (!in_adc(v[k])) { err(path, lineno, "rc stick out of ADC range 0..4095"); return -1; }
            }
            if (v[4] != 0 && v[4] != 1) { err(path, lineno, "rc <arm> must be 0 or 1"); return -1; }
            e.ch = Channel::Rc;
            if (ch == "rc_foreign")      e.source = RcSource::Foreign;
            else if (ch == "rc_ctrl_a")  e.source = RcSource::ControllerA;
            else if (ch == "rc_ctrl_b")  e.source = RcSource::ControllerB;
            else                          e.source = RcSource::Paired;
            e.thr = (uint16_t)v[0]; e.roll = (uint16_t)v[1];
            e.pitch = (uint16_t)v[2]; e.yaw = (uint16_t)v[3];
            e.arm = (uint8_t)v[4];
            long hold = 0, rate = 20;
            if (iss >> hold) {
                if (hold < 0) { err(path, lineno, "rc hold_ms must be >= 0"); return -1; }
                if (iss >> rate) {
                    if (rate < 1 || rate > 1000) { err(path, lineno, "rc rate_hz must be 1..1000 (1 tick = 1 ms)"); return -1; }
                }
            }
            e.hold_ms = (int)hold; e.rate_hz = (int)rate;
            // OPTIONAL 4th token: alt (1 => ALTITUDE_HOLD). Only reachable after
            // hold_ms and rate_hz are present, so always write the full form when
            // requesting alt: `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt>`.
            // 任意の4番目トークン alt（1 で高度保持）。hold_ms・rate_hz の後にのみ到達するため、
            // alt を出す行は完全形で書くこと。
            long alt = 0;
            if (iss >> alt) {
                if (alt != 0 && alt != 1) { err(path, lineno, "rc <alt> must be 0 or 1"); return -1; }
            }
            e.alt = (uint8_t)alt;
            // OPTIONAL 5th token: acro (1 => CTRL_FLAG_MODE → ACRO/rate mode).
            // After alt, so the full form is:
            // `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt> <acro>`.
            // 任意の5番目トークン acro（1 で CTRL_FLAG_MODE → ACRO/角速度モード）。alt の後。
            long acro = 0;
            if (iss >> acro) {
                if (acro != 0 && acro != 1) { err(path, lineno, "rc <acro> must be 0 or 1"); return -1; }
            }
            e.acro = (uint8_t)acro;
            // OPTIONAL 6th token: pos (1 => CTRL_FLAG_POS_MODE → POSITION_HOLD). After
            // acro, so the full form is:
            // `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt> <acro> <pos>`.
            // 任意の6番目トークン pos（1 で CTRL_FLAG_POS_MODE → POSITION_HOLD）。acro の後。
            long pos = 0;
            if (iss >> pos) {
                if (pos != 0 && pos != 1) { err(path, lineno, "rc <pos> must be 0 or 1"); return -1; }
            }
            e.pos = (uint8_t)pos;
            // OPTIONAL 7th token: flip (1 => CTRL_FLAG_FLIP → FLIP button,
            // Phase 3, flip-maneuver-plan.md §4.1/§9-6). After pos, so the full
            // form is:
            // `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt> <acro> <pos> <flip>`.
            // Existing lines that omit it are unaffected (defaults to 0).
            // 任意の7番目トークン flip（1 で CTRL_FLAG_FLIP → FLIP ボタン、Phase 3,
            // plan §4.1/§9-6）。pos の後。省略した既存行の挙動は変わらない（既定0）。
            long flip = 0;
            if (iss >> flip) {
                if (flip != 0 && flip != 1) { err(path, lineno, "rc <flip> must be 0 or 1"); return -1; }
            }
            e.flip = (uint8_t)flip;

        } else if (ch == "rc_ramp") {
            std::string field; long from, to, step, rate, arm;
            if (!(iss >> field >> from >> to >> step >> rate >> arm)) {
                err(path, lineno, "rc_ramp needs <field> <from> <to> <step> <rate_hz> <arm>"); return -1;
            }
            if (!parse_rc_field(field, e.ramp_field)) { err(path, lineno, "rc_ramp field must be throttle|roll|pitch|yaw"); return -1; }
            if (!in_adc(from) || !in_adc(to)) { err(path, lineno, "rc_ramp from/to out of ADC range 0..4095"); return -1; }
            if (step == 0) { err(path, lineno, "rc_ramp step must be != 0"); return -1; }
            if (rate < 1 || rate > 1000) { err(path, lineno, "rc_ramp rate_hz must be 1..1000 (1 tick = 1 ms)"); return -1; }
            if (arm != 0 && arm != 1) { err(path, lineno, "rc_ramp <arm> must be 0 or 1"); return -1; }
            e.ch = Channel::RcRamp;
            e.ramp_from = (uint16_t)from; e.ramp_to = (uint16_t)to;
            e.ramp_step = (int)(step < 0 ? -step : step);  // magnitude; direction from from/to
            e.rate_hz = (int)rate; e.arm = (uint8_t)arm;
            // OPTIONAL 7th token: alt (1 => ALTITUDE_HOLD), after the 6 required.
            // 任意の7番目トークン alt（1 で高度保持）。必須6個の後。
            long ralt = 0;
            if (iss >> ralt) {
                if (ralt != 0 && ralt != 1) { err(path, lineno, "rc_ramp <alt> must be 0 or 1"); return -1; }
            }
            e.alt = (uint8_t)ralt;
            // OPTIONAL 8th token: acro (1 => CTRL_FLAG_MODE → ACRO/rate mode), after alt.
            // 任意の8番目トークン acro（1 で ACRO/角速度モード）。alt の後。
            long racro = 0;
            if (iss >> racro) {
                if (racro != 0 && racro != 1) { err(path, lineno, "rc_ramp <acro> must be 0 or 1"); return -1; }
            }
            e.acro = (uint8_t)racro;

        } else if (ch == "key") {
            std::string rest;
            std::getline(iss, rest);
            if (!parse_quoted(rest, e.text)) { err(path, lineno, "key needs a quoted \"...\" string"); return -1; }
            e.ch = Channel::Key;

        } else if (ch == "api") {
            // api "<command line>" — inject one Tello-style API command into the
            // firmware's ApiTask parser (sf_api_inject_line). The UDP transport is
            // inert on the SILS host; the parser/executor is the REAL firmware code.
            // api "<コマンド行>" — Tello 風 API コマンドをファームの ApiTask パーサへ
            // 注入（sf_api_inject_line）。SILS ホストでは UDP 輸送が inert なため、
            // パーサ/実行系の「実ファームコード」だけを駆動する。
            std::string rest;
            std::getline(iss, rest);
            if (!parse_quoted(rest, e.text)) { err(path, lineno, "api needs a quoted \"...\" string"); return -1; }
            e.ch = Channel::Api;

        } else if (ch == "wind") {
            // wind <fx> <fy> <fz> [dur_ms] — external force NED [N]; dur_ms>0 = gust.
            // wind <fx> <fy> <fz> [dur_ms] — NED 外乱力 [N]、dur_ms>0 で突風。
            e.ch = Channel::Wind;
            if (!(iss >> e.wind_fx >> e.wind_fy >> e.wind_fz)) {
                err(path, lineno, "wind needs <fx> <fy> <fz> [dur_ms]"); return -1;
            }
            if (!(iss >> e.wind_dur_ms)) e.wind_dur_ms = 0;     // optional → sustained step
            if (e.wind_dur_ms < 0) { err(path, lineno, "wind dur_ms must be >= 0"); return -1; }

        } else if (ch == "fault") {
            // fault <motor 0-3> <gain 0-1> — degrade one motor's thrust health.
            // fault <motor 0-3> <gain 0-1> — 1モータの推力健全度を劣化。
            e.ch = Channel::Fault;
            if (!(iss >> e.fault_motor >> e.fault_gain)) {
                err(path, lineno, "fault needs <motor 0-3> <gain 0-1>"); return -1;
            }
            if (e.fault_motor < 0 || e.fault_motor > 3) {
                err(path, lineno, "fault <motor> must be 0..3"); return -1;
            }
            if (e.fault_gain < 0.0f || e.fault_gain > 1.0f) {
                err(path, lineno, "fault <gain> must be 0.0..1.0"); return -1;
            }

        } else if (ch == "bias") {
            // bias <ax> <ay> <az> <gx> <gy> <gz> — deterministic raw IMU bias (body
            // FRD): accel [m/s²] + gyro [rad/s]. Models a pre-calibration MEMS offset.
            // bias <ax> <ay> <az> <gx> <gy> <gz> — 決定論的な生 IMU バイアス（機体 FRD）:
            // accel[m/s²] + gyro[rad/s]。校正前 MEMS オフセットを模擬。
            e.ch = Channel::Bias;
            if (!(iss >> e.bias_ax >> e.bias_ay >> e.bias_az
                      >> e.bias_gx >> e.bias_gy >> e.bias_gz)) {
                err(path, lineno, "bias needs <ax> <ay> <az> <gx> <gy> <gz>"); return -1;
            }

        } else if (const int extra = parseExtraChannel(ch, iss, e, path, lineno); extra != 0) {
            if (extra < 0) return -1;

        } else if (ch == "btn") {
            std::string rest; std::getline(iss, rest);
            e.ch = Channel::Btn;
            e.raw = ch + rest;

        } else {
            err(path, lineno, "unknown channel '" + ch + "'");
            return -1;
        }

        prev_end_us = at_us + event_duration_us(e);
        g_events.push_back(std::move(e));
    }

    g_active = !g_events.empty();
    if (g_active) {
        std::printf("[scenario] loaded %zu events from %s\n", g_events.size(), path);
    } else {
        std::fprintf(stderr, "[scenario] %s has no events\n", path);
    }
    return g_active ? 1 : -1;
}

bool sils_scenario_active(void) { return g_active; }

void sils_scenario_driver_task(void* /*arg*/)
{
    std::printf("[scenario] driver online (%zu events)\n", g_events.size());

    bool warned_deferred = false;

    for (const Event& e : g_events) {
        // Wait until the virtual clock reaches this event's time (absolute tick).
        // 仮想時計がこの事象の時刻（絶対 tick）に達するまで待つ。
        TickType_t target = (TickType_t)(e.at_us / 1000);
        while (xTaskGetTickCount() < target) {
            vTaskDelay(1);
        }

        switch (e.ch) {
            case Channel::Rc: {
                const int period = period_ms(e.rate_hz);
                const uint8_t flags = (e.arm  ? sils::kFlagArm     : 0) |
                                      (e.alt  ? sils::kFlagAltMode : 0) |
                                      (e.acro ? sils::kFlagMode    : 0) |
                                      (e.pos  ? sils::kFlagPosMode : 0) |
                                      (e.flip ? sils::kFlagFlip    : 0);
                // Pick the injector: the paired transmitter (rc), a different
                // non-paired one (rc_foreign — exercises the crosstalk filter), or
                // one of the two own-address-filter test roles (rc_ctrl_a/rc_ctrl_b
                // — pairing-methods-plan.md §4.4).
                // 注入元を選ぶ: ペア済み送信機（rc）、別の未ペア送信機（rc_foreign — 混信
                // フィルタを試す）、または自機宛フィルタ試験の2役（rc_ctrl_a/rc_ctrl_b —
                // pairing-methods-plan.md §4.4）。
                void (*inject)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t) =
                    &sils::inject_rc;
                switch (e.source) {
                    case RcSource::Foreign:     inject = &sils::inject_rc_foreign;     break;
                    case RcSource::ControllerA: inject = &sils::inject_rc_controller_a; break;
                    case RcSource::ControllerB: inject = &sils::inject_rc_controller_b; break;
                    default:                    break;
                }
                if (e.hold_ms <= 0) {
                    inject(e.thr, e.roll, e.pitch, e.yaw, flags);
                } else {
                    // Same frame count the parser used for the timeline duration
                    // (>=1 for any positive hold) so emit and timing agree.
                    // パーサの時間幅と同じフレーム数（正のホールドは最低1）で発射・時刻を一致。
                    const long long frames = rc_hold_frames(e.hold_ms, e.rate_hz);
                    for (long long i = 0; i < frames; ++i) {
                        inject(e.thr, e.roll, e.pitch, e.yaw, flags);
                        vTaskDelay((TickType_t)period);
                    }
                }
                break;
            }
            case Channel::RcRamp: {
                const int period = period_ms(e.rate_hz);
                const uint8_t flags = (e.arm  ? sils::kFlagArm     : 0) |
                                      (e.alt  ? sils::kFlagAltMode : 0) |
                                      (e.acro ? sils::kFlagMode    : 0) |
                                      (e.pos  ? sils::kFlagPosMode : 0);
                const int astep = (e.ramp_to >= e.ramp_from) ? e.ramp_step : -e.ramp_step;
                for (long v = e.ramp_from;
                     (astep > 0) ? (v <= e.ramp_to) : (v >= e.ramp_to);
                     v += astep) {
                    uint16_t s[4] = {sils::kAdcCentre, sils::kAdcCentre, sils::kAdcCentre, sils::kAdcCentre};
                    s[e.ramp_field] = (uint16_t)v;
                    sils::inject_rc(s[0], s[1], s[2], s[3], flags);
                    vTaskDelay((TickType_t)period);
                }
                break;
            }
            case Channel::Key:
                sils_console_write(e.text.data(), (int)e.text.size());
                break;
            case Channel::Api: {
                if (g_api_inject_fn) {
                    g_api_inject_fn(e.text.c_str());
                    sils_emu_record_note("api", e.text.c_str());
                } else {
                    sils_emu_record_note("api", "(unsupported on this target)");
                }
                break;
            }
            case Channel::Wind: {
                // P7: apply the external wind force to the Plant. A gust pulse
                // (dur_ms>0) holds the force, then reverts to zero so the controller
                // must recover; a sustained step (dur_ms=0) leaves it applied.
                // P7: 外乱風力を Plant に適用。突風パルス(dur_ms>0)は保持後ゼロに戻し制御の
                // 回復を要求。定常(dur_ms=0)は適用したまま。
                sils_board_set_wind(e.wind_fx, e.wind_fy, e.wind_fz);
                char note[96];
                std::snprintf(note, sizeof(note), "wind %.3f %.3f %.3f dur=%dms",
                              e.wind_fx, e.wind_fy, e.wind_fz, e.wind_dur_ms);
                sils_emu_record_note("wind", note);
                if (e.wind_dur_ms > 0) {
                    vTaskDelay((TickType_t)e.wind_dur_ms);
                    sils_board_set_wind(0.0f, 0.0f, 0.0f);
                    sils_emu_record_note("wind", "0 0 0 (gust end)");
                }
                break;
            }
            case Channel::Fault: {
                // P7: degrade one motor's thrust health (sustained). The mixer must
                // compensate to keep attitude/altitude bounded (G3).
                // P7: 1モータの推力健全度を劣化（持続）。ミキサーが補償し姿勢/高度を有界に保つ（G3）。
                sils_board_set_motor_health(e.fault_motor, e.fault_gain);
                char note[64];
                std::snprintf(note, sizeof(note), "fault motor=%d gain=%.2f",
                              e.fault_motor, e.fault_gain);
                sils_emu_record_note("fault", note);
                break;
            }
            case Channel::Bias: {
                // P2-3: inject a deterministic raw IMU bias (body FRD). The firmware
                // boot calibration should measure and remove it; without calibration
                // the ESKF attitude loop degrades (the contrast test).
                // P2-3: 決定論的な生 IMU バイアス（機体 FRD）を注入。ファーム起動校正が
                // 測定・除去すべき。校正なしでは ESKF 姿勢ループが劣化（対照試験）。
                sils_board_set_imu_bias(e.bias_ax, e.bias_ay, e.bias_az,
                                       e.bias_gx, e.bias_gy, e.bias_gz);
                char note[96];
                std::snprintf(note, sizeof(note),
                              "bias accel=[%.3f %.3f %.3f] gyro=[%.4f %.4f %.4f]",
                              e.bias_ax, e.bias_ay, e.bias_az,
                              e.bias_gx, e.bias_gy, e.bias_gz);
                sils_emu_record_note("bias", note);
                break;
            }
            case Channel::TofSilent:
            case Channel::TofInvalid:
            case Channel::TofDeadZone:
            case Channel::Param:
            case Channel::Battery:
            case Channel::Handle:
                applyExtraEvent(e);   // ToF fault injection, param, handle / ToF の故障注入、param、handle
                break;
            case Channel::Btn:
                if (!warned_deferred) {
                    std::printf("[scenario] note: btn is deferred (GPIO future) — skipping\n");
                    warned_deferred = true;
                }
                sils_emu_record_note("skip", e.raw.c_str());
                break;
        }
    }

    std::printf("[scenario] timeline exhausted — driver done\n");
    vTaskDelete(nullptr);
}

}  // extern "C"
